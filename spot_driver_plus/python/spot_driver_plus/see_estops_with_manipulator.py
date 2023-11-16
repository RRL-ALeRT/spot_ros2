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
        node.get_tf("base_link", "tool_frame")
        rclpy.spin_once(node)

    x = node.position.translation.x
    y = node.position.translation.y
    z = node.position.translation.z
    xa = node.position.rotation.x
    ya = node.position.rotation.y
    za = node.position.rotation.z
    wa = node.position.rotation.w

    moveit_motion(x, y, z + 0.1, xa, ya, za, wa)
    print(xa, ya, za, wa)

    node = GetTF()
    node.position = None
    node.orientation = None
    while node.position is None or node.orientation is None:
        node.get_tf("base_link", "tool_frame")
        node.get_tf("camera_link", "estop_set")
        rclpy.spin_once(node)

    x = node.position.translation.x
    y = node.position.translation.y
    z = node.position.translation.z
    xa = node.position.rotation.x
    ya = node.position.rotation.y
    za = node.position.rotation.z
    wa = node.position.rotation.w

    dx = node.orientation.translation.x
    dy = node.orientation.translation.y
    dz = node.orientation.translation.z

    # Calculate the direction vector from frame1 to frame2
    direction_vector = np.array([dx, dy, dz])
    # direction_vector /= np.linalg.norm(direction_vector)  # Normalize the vector

    # Calculate the quaternion to point in that direction
    roll = math.atan2(dy, dz)
    pitch = -math.atan2(dx, dz)
    # pitch = math.asin(-direction_vector[2])
    yaw = 0  # Assuming no roll is needed

    quaternion = quaternion_from_euler(roll, pitch, yaw)

    # print(quaternion)
    q1 = Quaternion(xa, ya, za, wa)
    q2 = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
    result = q1 * q2 # Adding them

    print(result.x, result.y, result.z, result.w)

    moveit_motion(x, y, z, result.x, result.y, result.z, result.w)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
