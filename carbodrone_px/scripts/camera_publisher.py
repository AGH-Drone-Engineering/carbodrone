#!/usr/bin/env python3

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import threading
import socket
import numpy as np
import struct
import os


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', qos_profile_sensor_data)
        self.camera_info_publisher_ = self.create_publisher(CameraInfo, 'camera/camera_info', qos_profile_sensor_data)
        self.cv_bridge = CvBridge()
        self.camera_info = self.create_camera_info()

        self.create_socket('/tmp/carbodrone/camera')

        self.is_running = True
        self.thread = threading.Thread(target=self.capture_and_publish)
        self.thread.start()

    def create_socket(self, socket_path):
        try:
            os.unlink(socket_path)
        except OSError:
            if os.path.exists(socket_path):
                raise

        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.bind(socket_path)
        self.sock.listen(1)
        self.conn, _ = self.sock.accept()

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
            data = self.conn.recv(12)
            if not data:
                break

            height, width, channels = struct.unpack('<III', data)
            length = height * width * channels

            frame_data = b''
            while len(frame_data) < length:
                packet = self.conn.recv(length - len(frame_data))
                if not packet:
                    break
                frame_data += packet

            if len(frame_data) != length:
                break

            frame = np.frombuffer(frame_data, np.uint8).reshape(height, width, channels)

            msg = self.cv_bridge.cv2_to_imgmsg(frame, "rgb8")
            self.publisher_.publish(msg)

            self.camera_info.header.stamp = self.get_clock().now().to_msg()
            self.camera_info.height = frame.shape[0]
            self.camera_info.width = frame.shape[1]
            self.camera_info_publisher_.publish(self.camera_info)

    def stop(self):
        self.is_running = False
        self.thread.join()
        self.sock.close()


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.stop()
        camera_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
