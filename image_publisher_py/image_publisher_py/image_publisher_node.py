#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from picamera2 import Picamera2


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher_node')

        # Create a publisher on the topic '/camera/image'
        self.publisher_ = self.create_publisher(Image, '/camera/image', 10)

        # Use CvBridge to convert OpenCV images to ROS2 Image messages
        self.bridge = CvBridge()
        
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_video_configuration(main={"size": (640, 640)}))
        self.picam2.start()

        # Create a timer that will call the publish_image method periodically (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_image)

    def publish_image(self):
        # Convert the OpenCV image (BGR format) to a ROS Image message
        
        image = cv2.cvtColor(self.picam2.capture_array(),cv2.COLOR_BGRA2RGB)
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')

        # Publish the image
        self.publisher_.publish(ros_image)

        self.get_logger().info('Publishing image')


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
