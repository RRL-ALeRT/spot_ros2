#!/usr/bin/env python3

import sys
import os
import subprocess
from functools import partial

import rclpy
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Empty
from composition_interfaces.srv import LoadNode, UnloadNode, ListNodes

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QScrollBar

from rqt_gui.main import Main


class EstopRqtPlugin(Plugin):
    def __init__(self, context):
        """Initialize the plugin."""
        super().__init__(context)
        self._context = context

        # Create the publisher for estop commands
        self.publisher = self._context.node.create_publisher(String, 'estop_command', QoSProfile(depth=10))

        self.lights_pub = self._context.node.create_publisher(String, 'lights_camera', QoSProfile(depth=1))

        # Create the subscriber for status updates
        self.subscriber = self._context.node.create_subscription(String, 'estop_status', self.status_callback, QoSProfile(depth=10))

        # Create service clients
        self.map_reset_client = self._context.node.create_client(Empty, '/reset_map_frame_and_travelled_path')
        self.world_info_reset_client = self._context.node.create_client(Empty, '/reset_world_info')
        self.octomap_reset_client = self._context.node.create_client(Empty, '/octomap_server/reset')

        # Run "ros2 run rclcpp_components component_container --ros-args -r __node:=OperatorAudio" in operator's terminal
        self.load_audio_operator_node = self._context.node.create_client(LoadNode, '/OperatorAudio/_container/load_node')
        self.unload_audio_operator_node = self._context.node.create_client(UnloadNode, '/OperatorAudio/_container/unload_node')
        self.list_audio_operator_nodes = self._context.node.create_client(ListNodes, '/OperatorAudio/_container/list_nodes')

        # Run "ros2 run rclcpp_components component_container --ros-args -r __node:=NucAudio" in nuc's terminal
        # Run "pulseaudio --daemonize" if NUC is connected in ssh
        self.load_audio_nuc_node = self._context.node.create_client(LoadNode, '/NucAudio/_container/load_node')
        self.unload_audio_nuc_node = self._context.node.create_client(UnloadNode, '/NucAudio/_container/unload_node')
        self.list_audio_nuc_nodes = self._context.node.create_client(ListNodes, '/NucAudio/_container/list_nodes')

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
        world_reset_button.clicked.connect(self.world_reset_command)
        world_reset_button.setStyleSheet("background-color: pink; font: bold 10px; border-width: 5px; border-radius:10px; padding: 10px")

        # Create the label for displaying status
        self.status_label = QLabel('Status: Unknown')
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("font: bold 16px")

        # Create the toggle button
        two_way_audio = QPushButton('2WayAudio')
        two_way_audio.setCheckable(True)
        two_way_audio.setStyleSheet("QPushButton { font: bold 10px; border-width: 5px; border-radius:10px; padding: 10px }"
                                          "QPushButton:checked { background-color: green }"
                                          "QPushButton:unchecked { background-color: red }")
        two_way_audio.clicked.connect(partial(self.toggle_component, two_way_audio))

        second_row_buttons = QHBoxLayout()
        second_row_buttons.addWidget(release_button)
        second_row_buttons.addWidget(estop_button)

        third_row_buttons = QHBoxLayout()
        third_row_buttons.addWidget(hazmat_button)
        third_row_buttons.addWidget(world_reset_button)
        third_row_buttons.addWidget(two_way_audio)

        layout.addWidget(stop_button)
        layout.addLayout(second_row_buttons)
        layout.addLayout(third_row_buttons)
        layout.addWidget(self.status_label)

        # Create the scroll bars
        front_light = QScrollBar(Qt.Horizontal)
        left_light = QScrollBar(Qt.Horizontal)
        right_light = QScrollBar(Qt.Horizontal)

        # Connect callbacks to scroll bars
        front_light.valueChanged.connect(self.front_light_value_change_callback)
        left_light.valueChanged.connect(self.left_light_value_change_callback)
        right_light.valueChanged.connect(self.right_light_value_change_callback)

        front_light.sliderReleased.connect(self.front_light_callback)
        left_light.sliderReleased.connect(self.left_light_callback)
        right_light.sliderReleased.connect(self.right_light_callback)

        # Add the scroll areas to the layout
        layout.addWidget(front_light)
        layout.addWidget(left_light)
        layout.addWidget(right_light)

        # Add the widget to the plugin
        self._widget = self.widget
        context.add_widget(self._widget)

        self.front_value = 0
        self.left_value = 0
        self.right_value = 0

    def toggle_component(self, button):
        if button.isChecked():
            if not self.load_audio_operator_node.wait_for_service(timeout_sec=1.0):
                self._context.node.get_logger().info('service load_audio_operator_node not available, skipping command')
                return
            
            if not self.load_audio_nuc_node.wait_for_service(timeout_sec=1.0):
                self._context.node.get_logger().info('service load_audio_nuc_node not available, skipping command')
                return
            
            # One capture and other plays with topics from each other's capture nodes
            req = LoadNode.Request()
            req.package_name = "audio_capture"
            req.plugin_name = "audio_capture::AudioCaptureNode"
            req.node_namespace = "/operator_to_nuc"
            self.load_audio_operator_node.call_async(req)

            req.node_namespace = "/nuc_to_operator"
            self.load_audio_nuc_node.call_async(req)

            req = LoadNode.Request()
            req.package_name = "audio_play"
            req.plugin_name = "audio_play::AudioPlayNode"
            req.node_namespace = "/nuc_to_operator"
            self.load_audio_operator_node.call_async(req)

            req.node_namespace = "/operator_to_nuc"
            self.load_audio_nuc_node.call_async(req)
        else:
            # OPERATOR
            if not self.list_audio_operator_nodes.wait_for_service(timeout_sec=1.0):
                self._context.node.get_logger().info('service list_audio_operator_nodes not available, skipping command')
                return
            
            list_of_nodes = self.list_audio_operator_nodes.call_async(ListNodes.Request())

            while not list_of_nodes.done() and rclpy.ok():
                pass

            if not self.unload_audio_operator_node.wait_for_service(timeout_sec=1.0):
                self._context.node.get_logger().info('service unload_audio_operator_node not available, skipping command')
                return
            
            for id in list(list_of_nodes.result().unique_ids):
                req = UnloadNode.Request()
                req.unique_id = id
                self.unload_audio_operator_node.call_async(req)
            
            # NUC
            if not self.list_audio_nuc_nodes.wait_for_service(timeout_sec=1.0):
                self._context.node.get_logger().info('service list_audio_nuc_nodes not available, skipping command')
                return
            
            list_of_nodes = self.list_audio_nuc_nodes.call_async(ListNodes.Request())

            while not list_of_nodes.done() and rclpy.ok():
                pass

            if not self.unload_audio_nuc_node.wait_for_service(timeout_sec=1.0):
                self._context.node.get_logger().info('service unload_audio_nuc_node not available, skipping command')
                return
            
            for id in list(list_of_nodes.result().unique_ids):
                req = UnloadNode.Request()
                req.unique_id = id
                self.unload_audio_nuc_node.call_async(req)

    def front_light_value_change_callback(self, value):
        self.front_value = int((value + 1) * 255 / 100)
        if self.front_value < 4:
            self.front_value = 0

    def left_light_value_change_callback(self, value):
        self.left_value = int((value + 1) * 255 / 100)
        if self.left_value < 4:
            self.left_value = 0

    def right_light_value_change_callback(self, value):
        self.right_value = int((value + 1) * 255 / 100)
        if self.right_value < 4:
            self.right_value = 0

    def front_light_callback(self):
        msg = String()
        msg.data = f"camera_light:{self.front_value}"
        self.lights_pub.publish(msg)

    def left_light_callback(self):
        msg = String()
        msg.data = f"left_light:{self.left_value}"
        self.lights_pub.publish(msg)

    def right_light_callback(self):
        msg = String()
        msg.data = f"right_light:{self.right_value}"
        self.lights_pub.publish(msg)

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

    def world_reset_command(self):
        if not self.map_reset_client.wait_for_service(timeout_sec=1.0):
            self._context.node.get_logger().info('service map_reset not available, skipping command')
            return
        self.map_reset_client.call_async(Empty.Request())

        if not self.world_info_reset_client.wait_for_service(timeout_sec=1.0):
            self._context.node.get_logger().info('service world_info_reset not available, skipping command')
            return
        self.world_info_reset_client.call_async(Empty.Request())

        if not self.octomap_reset_client.wait_for_service(timeout_sec=1.0):
            self._context.node.get_logger().info('service octomap_reset not available, skipping command')
            return
        self.octomap_reset_client.call_async(Empty.Request())
          

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
