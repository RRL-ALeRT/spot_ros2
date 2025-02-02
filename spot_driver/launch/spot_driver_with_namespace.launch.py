import os

import launch
import launch_ros
import xacro
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> launch.LaunchDescription:
    spot_name = LaunchConfiguration("spot_name")
    spot_name_arg = DeclareLaunchArgument("spot_name", description="Name of spot")

    config_file = LaunchConfiguration("config_file")
    config_file_arg = DeclareLaunchArgument("config_file", description="Path to configuration file for the driver.")

    pkg_share = FindPackageShare("spot_description").find("spot_description")
    urdf_dir = os.path.join(pkg_share, "urdf")
    xacro_file = os.path.join(urdf_dir, "spot.urdf.xacro")
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml(indent="  ")

    spot_driver_node = launch_ros.actions.Node(
        package="spot_driver",
        executable="spot_ros2",
        name="spot_ros2",
        output="screen",
        namespace=spot_name,
        parameters=[config_file, {"spot_name": spot_name}],
    )

    params = {"robot_description": robot_desc, "frame_prefix": PathJoinSubstitution([spot_name, ""])}
    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        namespace=spot_name,
        parameters=[params],
    )

    return launch.LaunchDescription(
        [
            spot_name_arg,
            config_file_arg,
            spot_driver_node,
            robot_state_publisher,
        ]
    )
