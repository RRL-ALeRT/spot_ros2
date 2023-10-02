#!/usr/bin/env python3

import time
import bosdyn.client
import bosdyn.client.util
from bosdyn.client.image import ImageClient, build_image_request, ImageDataError, depth_image_to_pointcloud
from bosdyn.client.exceptions import UnableToConnectToRobotError, TimedOutError, RpcError
from bosdyn.api import image_pb2
from spot_driver.ros_helpers import get_from_env_and_fall_back_to_param

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np

from geometry_msgs.msg import TwistStamped

KINOVA_TWIST_TOPIC = "/kinova_twist"
KINOVA_SERVO_TOPIC = "/servo_node/delta_twist_cmds"


class GetPCL(Node):
    def __init__(self) -> None:
        # ROS2 init
        super().__init__('kinova_servo_controller')

        self.create_timer(0.01, self.timerCb)

        self.create_subscription(TwistStamped, KINOVA_TWIST_TOPIC, self.twistCB, 1)

        self.pub = self.create_publisher(TwistStamped, KINOVA_SERVO_TOPIC, 1)

        self.msg = TwistStamped()

        self.msg_validity_time = 0

    def twistCB(self, msg):
        self.msg = msg
        self.msg_validity_time = (self.get_clock().now().nanoseconds / 1e9) + 0.2

    def timerCb(self):
        if self.get_clock().now().nanoseconds / 1e9 > self.msg_validity_time:
            self.msg = TwistStamped()

        self.msg.header.frame_id = "base_link"
        self.pub.publish(self.msg)


def main(args = None):
    rclpy.init(args=args)
    pcl_node = GetPCL()

    try:
        rclpy.spin(pcl_node)
    except KeyboardInterrupt:
        pass
    finally:
        pcl_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
