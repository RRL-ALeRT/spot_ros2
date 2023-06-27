#!/usr/bin/env python3

import sys
import os
import subprocess

import rclpy
from rclpy.qos import QoSProfile
from std_msgs.msg import String
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

        # Create the label for displaying status
        self.status_label = QLabel('Status: Unknown')
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("font: bold 16px")

        button_layout = QHBoxLayout()
        button_layout.addWidget(release_button)
        button_layout.addWidget(estop_button)

        layout.addWidget(stop_button)
        layout.addLayout(button_layout)
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
