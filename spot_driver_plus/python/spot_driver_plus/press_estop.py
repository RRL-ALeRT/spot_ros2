#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import math

from moveit_ik import MoveitIKClientAsync as IK
from moveit_action_client import MoveGroupActionClient as Moveit
from gripper_action_client import GripperActionClient as Gripper


class Quaternion:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def __mul__(self, other):
        result = Quaternion(0, 0, 0, 0)
        result.w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
        result.x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y
        result.y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x
        result.z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
        return result


def moveit_motion(x,y,z,qx,qy,qz,qw):
    ik = IK()
    moveit = Moveit()
    target_angles = ik.send_request(x,y,z,qx,qy,qz,qw)
    if target_angles != None:
        ik.destroy_node()
        moveit.send_goal(target_angles)
        while not moveit.goal_done:
            rclpy.spin_once(moveit)


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q
    

class GetTF(Node):
    def __init__(self):
        super().__init__('move_manipulator')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def get_tf(self, header, child):
        try:
            transform = self.tf_buffer.lookup_transform(
                header,
                child,
                rclpy.time.Time()
            )
            return transform.transform
            self.get_logger().warn(f"Got it")

        except Exception as e:
            self.get_logger().warn(f"TF2 lookup failed: {str(e)}")
        return None


def main(args=None):
    rclpy.init(args=args)

    node = GetTF()
    position_2 = None
    while position_2 is None and rclpy.ok():
        position_2 = node.get_tf("base_link", "tool_estop_target_0_05")
        rclpy.spin_once(node)

    position_1 = None
    while position_1 is None and rclpy.ok():
        position_1 = node.get_tf("base_link", "tool_estop_target_0_1")
        rclpy.spin_once(node)

    gripper = Gripper()

    x_1 = position_1.translation.x
    y_1 = position_1.translation.y
    z_1 = position_1.translation.z
    xa_1 = position_1.rotation.x
    ya_1 = position_1.rotation.y
    za_1 = position_1.rotation.z
    wa_1 = position_1.rotation.w

    x_2 = position_2.translation.x
    y_2 = position_2.translation.y
    z_2 = position_2.translation.z
    xa_2 = position_2.rotation.x
    ya_2 = position_2.rotation.y
    za_2 = position_2.rotation.z
    wa_2 = position_2.rotation.w

    moveit_motion(x_1, y_1, z_1, xa_1, ya_1, za_1, wa_1)

    gripper.send_goal("close")
    while (not gripper.goal_done) and rclpy.ok():
        rclpy.spin_once(gripper)

    moveit_motion(x_2, y_2, z_2, xa_2, ya_2, za_2, wa_2)
    moveit_motion(x_1, y_1, z_1, xa_1, ya_1, za_1, wa_1)

    gripper.send_goal("open")
    while (not gripper.goal_done) and rclpy.ok():
        rclpy.spin_once(gripper)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
