import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient

from webots_spot_msgs.srv import SpotMotion
from webots_spot_msgs.action import Stack

from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import PlanningOptions
from moveit_msgs.action import MoveGroup

import time
import numpy as np
from copy import deepcopy


class GripperActionClient(Node):
    def __init__(self):
        super().__init__('gripper_execute_python')
        self._action_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')

    def send_goal(self, position):
        if position == "open":
            t = 0.0
        elif position == "close":
            t = 0.8

        self.goal_done = False

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = t
        goal_msg.command.max_effort = 10.0

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.goal_done = True
        time.sleep(1)
