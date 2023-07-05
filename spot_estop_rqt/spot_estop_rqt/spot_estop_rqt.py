#!/usr/bin/env python3

import sys
import os
import subprocess

import rclpy
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Empty

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel

from rqt_gui.main import Main


class EstopRqtPlugin(Plugin):
    def __init__(self, context):
        """Initialize the plugin."""
        super().__init__(context)
        self._context = context

        # Create the publisher for estop commands
        self.publisher = self._context.node.create_publisher(String, 'estop_command', QoSProfile(depth=10))

        # Create the subscriber for status updates
        self.subscriber = self._context.node.create_subscription(String, 'estop_status', self.status_callback, QoSProfile(depth=10))

        # Create service clients
        self.world_reset_client = self._context.node.create_client(Empty, '/reset_map_frame_and_travelled_path')
        self.world_info_reset_client = self._context.node.create_client(Empty, '/reset_world_info')

        # Create the main widget and set up the layout
        self.widget = QWidget()
        layout = QVBoxLayout()
        self.widget.setLayout(layout)

        # Set minimum size for the widget
        self.widget.setMinimumSize(200, 200)

        # Create the buttons for sending commands
        stop_button = QPushButton('STOP')
        stop_button.clicked.connect(self.send_stop_command)
        stop_button.setStyleSheet("background-color: red; font: bold 20px; border-width: 5px; border-radius:20px; padding: 20px")

        release_button = QPushButton('Release')
        release_button.clicked.connect(self.send_release_command)
        release_button.setStyleSheet("background-color: green; font: bold 20px; border-width: 5px; border-radius:20px; padding: 20px")

        estop_button = QPushButton('Spot')
        estop_button.clicked.connect(self.run_ssh_command)
        estop_button.setStyleSheet("background-color: blue; font: bold 20px; border-width: 5px; border-radius:20px; padding: 20px")

        hazmat_button = QPushButton('Hazmat')
        hazmat_button.clicked.connect(self.hazmat_command)
        hazmat_button.setStyleSheet("background-color: pink; font: bold 10px; border-width: 5px; border-radius:10px; padding: 10px")
        
        world_reset_button = QPushButton('World Reset')
        world_reset_button.clicked.connect(self.world_reset)
        world_reset_button.setStyleSheet("background-color: pink; font: bold 10px; border-width: 5px; border-radius:10px; padding: 10px")

        world_info_reset_button = QPushButton('World Info Reset')
        world_info_reset_button.clicked.connect(self.world_info_reset)
        world_info_reset_button.setStyleSheet("background-color: pink; font: bold 10px; border-width: 5px; border-radius:10px; padding: 10px")

        # Create the label for displaying status
        self.status_label = QLabel('Status: Unknown')
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("font: bold 16px")

        second_row_buttons = QHBoxLayout()
        second_row_buttons.addWidget(release_button)
        second_row_buttons.addWidget(estop_button)

        third_row_buttons = QHBoxLayout()
        third_row_buttons.addWidget(hazmat_button)
        third_row_buttons.addWidget(world_reset_button)
        third_row_buttons.addWidget(world_info_reset_button)

        layout.addWidget(stop_button)
        layout.addLayout(second_row_buttons)
        layout.addLayout(third_row_buttons)
        layout.addWidget(self.status_label)

        # Add the widget to the plugin
        self._widget = self.widget
        context.add_widget(self._widget)

    def send_stop_command(self):
        command_msg = String()
        command_msg.data = 'stop'
        self.publisher.publish(command_msg)

    def send_release_command(self):
        command_msg = String()
        command_msg.data = 'release'
        self.publisher.publish(command_msg)

    def status_callback(self, msg):
        status = msg.data
        self.status_label.setText(f'Status: {status}')

    def run_ssh_command(self):
        MAX_IP = os.getenv('MAX_IP')
        if MAX_IP is None:
            print("MAX_IP environment variable is not set.")
            return
        ssh_command = f'ssh -t max1@{MAX_IP} "screen -S spot_session -p EStop -X stuff \\"python /home/max1/estop.py\\\\n\\""'
        subprocess.Popen(ssh_command, shell=True)
        ssh_command = f'ssh -t max1@{MAX_IP} "screen -S spot_session -p Spot -X stuff \\"ros2 launch spot_driver_plus spot_launch.py\\\\n\\""'
        subprocess.Popen(ssh_command, shell=True)

    def hazmat_command(self):
        MAX_IP = os.getenv('MAX_IP')
        if MAX_IP is None:
            print("MAX_IP environment variable is not set.")
            return
        ssh_command = f'ssh -t max1@{MAX_IP} "screen -S spot_session -p Hazmat -X stuff \\"ros2 run world_info object_detection_yolov5 hazmat /kinova_color\\\\n\\""'
        subprocess.Popen(ssh_command, shell=True)

    def world_reset(self):
        if not self.world_reset_client.wait_for_service(timeout_sec=1.0):
            self._context.node.get_logger().info('service world_reset not available, skipping command')
            return
        self.world_reset_client.call_async(Empty.Request())

    def world_info_reset(self):
        if not self.world_reset_client.wait_for_service(timeout_sec=1.0):
            self._context.node.get_logger().info('service world_reset not available, skipping command')
            return
        self.world_info_reset_client.call_async(Empty.Request())

    def shutdown_plugin(self):
        self._context.node.destroy_node()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass


def main():
    """Run the plugin."""
    # Initialize the ROS node
    rclpy.init()

    # Create the plugin and run it
    sys.exit(Main().main(sys.argv, standalone="my_rqt_plugin.my_rqt_plugin"))

if __name__ == "__main__":
    main()
