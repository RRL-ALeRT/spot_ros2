#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros
import numpy as np

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.target_frame = 'spot_target'

        # PID control constants
        self.kp_linear = 0.8  # Proportional gain for linear velocity
        self.ki_linear = 0.0  # Integral gain for linear velocity (initially set to 0)
        self.kd_linear = 0.0003  # Derivative gain for linear velocity (initially set to 0)

        self.kp_angular = 1.5  # Proportional gain for angular velocity
        self.ki_angular = 0.0001  # Integral gain for angular velocity (initially set to 0)
        self.kd_angular = 0.0003  # Derivative gain for angular velocity (initially set to 0)

        # Initialize PID controller variables
        self.prev_error_linear = 0.0
        self.integral_linear = 0.0

        self.prev_error_angular = 0.0
        self.integral_angular = 0.0

        # Move the robot
        self.create_timer(0.1, self.move_robot)

        self.reach_time = self.get_clock().now().to_msg().sec

    def move_robot(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'body',
                self.target_frame,
                rclpy.time.Time()
            )

            # Calculate position errors in X and Y
            error_x = transform.transform.translation.x
            error_y = transform.transform.translation.y

            # Calculate linear velocity for position control using PID controller
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = self.calculate_pid_linear(error_x)
            cmd_vel_msg.linear.y = self.calculate_pid_linear(error_y)

            transform = self.tf_buffer.lookup_transform(
                'body',
                'estop_set',
                rclpy.time.Time()
            )
            error_angle = np.arctan2(transform.transform.translation.y, transform.transform.translation.x)

            # Calculate angular velocity for orientation control using PID controller
            cmd_vel_msg.angular.z = self.calculate_pid_angular(error_angle)

            # Set upper and lower limits
            cmd_vel_msg.linear.x = max(-0.25, min(cmd_vel_msg.linear.x, 0.25))
            cmd_vel_msg.linear.y = max(-0.25, min(cmd_vel_msg.linear.y, 0.25))
            cmd_vel_msg.angular.z = max(-0.4, min(cmd_vel_msg.angular.z, 0.4))

            if abs(error_x) < 0.2 and abs(error_y) < 0.2 and abs(error_angle) < 0.1:
                if self.get_clock().now().to_msg().sec > self.reach_time:
                    exit()
            else:
                self.reach_time = self.get_clock().now().to_msg().sec + 3

            # Publish the velocity commands to cmd_vel topic
            self.cmd_vel_publisher.publish(cmd_vel_msg)

        except Exception as e:
            self.get_logger().warn(f"TF2 lookup failed: {str(e)}")

    def calculate_pid_linear(self, error):
        # PID control for linear velocity
        # Calculate proportional term
        proportional = self.kp_linear * error

        # Calculate integral term (approximate integral using the cumulative sum)
        self.integral_linear += error
        integral = self.ki_linear * self.integral_linear

        # Calculate derivative term
        derivative = self.kd_linear * (error - self.prev_error_linear)

        # Calculate the control signal
        control_signal = proportional + integral + derivative

        # Update previous error
        self.prev_error_linear = error

        return control_signal

    def calculate_pid_angular(self, error):
        # PID control for angular velocity
        # Calculate proportional term
        proportional = self.kp_angular * error

        # Calculate integral term (approximate integral using the cumulative sum)
        self.integral_angular += error
        integral = self.ki_angular * self.integral_angular

        # Calculate derivative term
        derivative = self.kd_angular * (error - self.prev_error_angular)

        # Calculate the control signal
        control_signal = proportional + integral + derivative

        # Update previous error
        self.prev_error_angular = error

        return control_signal

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
