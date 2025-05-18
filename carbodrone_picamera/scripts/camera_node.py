#!/usr/bin/env python3

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge
from picamera2 import Picamera2
import threading
import cv2
import math


def intrinsics_from_hfov(
    hfov_deg: float,
    resolution: tuple[int, int]
) -> tuple[float, float, float, float]:
    width, height = resolution

    # convert FOV to radians
    hfov_rad = math.radians(hfov_deg)

    # fx from horizontal FOV
    fx = width / (2.0 * math.tan(hfov_rad / 2.0))

    # derive vertical FOV from aspect ratio, then fy
    vfov_rad = 2.0 * math.atan((height / width) * math.tan(hfov_rad / 2.0))
    fy = height / (2.0 * math.tan(vfov_rad / 2.0))

    # principal point (centre of the image)
    cx = width / 2.0
    cy = height / 2.0

    return fx, fy, cx, cy


CAMERA_FOV_DEG = 1.204 * 180 / math.pi


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher = self.create_publisher(Image, 'camera/image', qos_profile_sensor_data)
        self.compressed_publisher = self.create_publisher(CompressedImage, 'camera/image/compressed', qos_profile_sensor_data)
        self.camera_info_publisher = self.create_publisher(CameraInfo, 'camera/camera_info', qos_profile_sensor_data)
        self.cv_bridge = CvBridge()

        self.picam2 = Picamera2()
        video_config = self.picam2.create_video_configuration(
            main={"size": (3280, 2464), "format": "RGB888"},
            raw={"size": (3280, 2464)},
            controls={"FrameRate": 10, "ExposureTime": 500},
        )
        self.picam2.configure(video_config)
        self.picam2.start()

        self.camera_info = self.create_camera_info()

        self.compression_enabled = False
        self.is_running = True
        self.thread = threading.Thread(target=self.capture_and_publish)
        self.thread.start()

    def create_camera_info(self):
        # Get image dimensions from camera configuration
        width = self.picam2.camera_config["main"]["size"][0]
        height = self.picam2.camera_config["main"]["size"][1]

        fx, fy, cx, cy = intrinsics_from_hfov(CAMERA_FOV_DEG, (width, height))

        camera_info = CameraInfo()
        camera_info.header.stamp = self.get_clock().now().to_msg()
        camera_info.header.frame_id = "camera_optical_frame"
        camera_info.height = height
        camera_info.width = width
        camera_info.distortion_model = "plumb_bob"

        # Distortion coefficients (no distortion)
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Camera matrix (K)
        camera_info.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        ]

        # Rectification matrix (R) - identity for non-stereo cameras
        camera_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        # Projection matrix (P)
        camera_info.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        # Binning
        camera_info.binning_x = 0
        camera_info.binning_y = 0

        # ROI
        camera_info.roi.x_offset = 0
        camera_info.roi.y_offset = 0
        camera_info.roi.height = 0
        camera_info.roi.width = 0
        camera_info.roi.do_rectify = False

        return camera_info

    def capture_and_publish(self):
        while self.is_running and rclpy.ok():
            frame = self.picam2.capture_array('main')
            stamp = self.get_clock().now().to_msg()

            msg = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
            msg.header.stamp = stamp
            msg.header.frame_id = self.camera_info.header.frame_id
            self.publisher.publish(msg)

            if self.compressed_publisher.get_subscription_count() > 0:
                if not self.compression_enabled:
                    self.get_logger().info("Enabling compression")
                    self.compression_enabled = True
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = stamp
                compressed_msg.header.frame_id = self.camera_info.header.frame_id
                compressed_msg.format = "jpeg"
                compressed_msg.data = cv2.imencode('.jpg', frame)[1].tobytes()
                self.compressed_publisher.publish(compressed_msg)
            elif self.compression_enabled:
                self.get_logger().info("Disabling compression")
                self.compression_enabled = False

            self.camera_info.header.stamp = stamp
            self.camera_info_publisher.publish(self.camera_info)


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
