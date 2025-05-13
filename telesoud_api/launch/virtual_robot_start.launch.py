import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from moveit_configs_utils import MoveItConfigsBuilder
import xacro

def generate_launch_description():
    api_node = Node(
        package="telesoud_api",
        executable="telesoud_api",
        output="screen",
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return LaunchDescription(
        [
            api_node
        ]
    )

