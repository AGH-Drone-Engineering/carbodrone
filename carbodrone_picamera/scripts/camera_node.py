#!/usr/bin/env python3

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from picamera2 import Picamera2
import threading
import socket
import numpy as np
import struct
import os


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher = self.create_publisher(Image, 'camera/image', qos_profile_sensor_data)
        self.camera_info_publisher = self.create_publisher(CameraInfo, 'camera/camera_info', qos_profile_sensor_data)
        self.cv_bridge = CvBridge()
        self.camera_info = self.create_camera_info()

        self.picam2 = Picamera2()
        video_config = self.picam2.create_video_configuration(
            main={"size": (1280, 720), "format": "YUV420"},
            buffer_count=2,
            queue=False,
        )
        self.picam2.configure(video_config)
        self.picam2.start()

        self.is_running = True
        self.thread = threading.Thread(target=self.capture_and_publish)
        self.thread.start()

    def create_camera_info(self):
        # camera_matrix = np.array([[320, 0, 160], [0, 320, 160], [0, 0, 1]])
        # distortion_coefficients = np.array([0, 0, 0, 0, 0])

        camera_info = CameraInfo()
        camera_info.header.frame_id = "camera_optical_frame"

        # camera_info.d = distortion_coefficients.tolist()
        # camera_info.k = camera_matrix.flatten().tolist()

        return camera_info

    def capture_and_publish(self):
        while self.is_running and rclpy.ok():
            frame = self.picam2.capture_array('main')
            msg = self.cv_bridge.cv2_to_imgmsg(frame, "yuv420")
            self.publisher.publish(msg)

            self.camera_info.header.stamp = self.get_clock().now().to_msg()
            self.camera_info.height = frame.shape[0]
            self.camera_info.width = frame.shape[1]
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
    