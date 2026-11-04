#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry

# image from cv_1/image_raw
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageProcessing(Node):
    def __init__(self):
        super().__init__("image_processing")

        self.image_subscribers = self.create_subscription(
            Image, "XXXXX", self.image_callback, 10
        )
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, "XXXX", 10)

    def image_callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
