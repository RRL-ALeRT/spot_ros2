#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import math

from moveit_ik import MoveitIKClientAsync as IK
from moveit_action_client import MoveGroupActionClient as Moveit


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
            if header == "base_link":
                self.position = transform.transform
            else:
                self.orientation = transform.transform

        except Exception as e:
            # self.get_logger().warn(f"TF2 lookup failed: {str(e)}")
            pass


def main(args=None):
    rclpy.init(args=args)

    node = GetTF()
    node.position = None
    while node.position is None:
        node.get_tf("base_link", "tool_target")
        rclpy.spin_once(node)

    x = node.position.translation.x
    y = node.position.translation.y
    z = node.position.translation.z
    xa = node.position.rotation.x
    ya = node.position.rotation.y
    za = node.position.rotation.z
    wa = node.position.rotation.w

    moveit_motion(x - 0.15, y, z, xa, ya, za, wa)
    moveit_motion(x - 0.1, y, z, xa, ya, za, wa)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()

"""

ros2 run tf2_ros static_transform_publisher 0 0.1 0.85 0 1.57 -1.57 estop_set spot_target
ros2 run tf2_ros static_transform_publisher 0 0 0.2 -0.785 3.14 0 estop_set tool_target
ros2 run tf2_ros static_transform_publisher 0 0 0 0 3.14 0 estop tool_estop_target

"""