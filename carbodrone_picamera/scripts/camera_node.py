#!/usr/bin/env python3

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from picamera2 import Picamera2
import threading


CAMERA_FOCAL_LENGTH_PX = 1260


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher = self.create_publisher(Image, 'camera/image', qos_profile_sensor_data)
        self.camera_info_publisher = self.create_publisher(CameraInfo, 'camera/camera_info', qos_profile_sensor_data)
        self.cv_bridge = CvBridge()

        self.picam2 = Picamera2()
        video_config = self.picam2.create_video_configuration(
            main={"size": (1920, 1080), "format": "YUV420"},
        )
        self.picam2.configure(video_config)
        self.picam2.start()

        # Initialize camera_info after picam2 is configured
        self.camera_info = self.create_camera_info()

        self.is_running = True
        self.thread = threading.Thread(target=self.capture_and_publish)
        self.thread.start()

    def create_camera_info(self):
        # Get image dimensions from camera configuration
        width = self.picam2.camera_config["main"]["size"][0]
        height = self.picam2.camera_config["main"]["size"][1]
        img_cx = width * 0.5
        img_cy = height * 0.5

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
            CAMERA_FOCAL_LENGTH_PX, 0.0, img_cx,
            0.0, CAMERA_FOCAL_LENGTH_PX, img_cy,
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
            CAMERA_FOCAL_LENGTH_PX, 0.0, img_cx, 0.0,
            0.0, CAMERA_FOCAL_LENGTH_PX, img_cy, 0.0,
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

            msg = self.cv_bridge.cv2_to_imgmsg(frame, "yuv420")
            msg.header.stamp = stamp
            msg.header.frame_id = self.camera_info.header.frame_id
            self.publisher.publish(msg)

            self.camera_info.header.stamp = stamp
            self.camera_info_publisher.publish(self.camera_info)

    def stop(self):
        self.is_running = False
        self.thread.join()
        self.picam2.stop()


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        camera_node.stop()
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
