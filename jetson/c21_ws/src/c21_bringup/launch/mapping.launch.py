"""Mapping mode: slam_toolbox online async SLAM on /scan. The user drives with the APP."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('c21_bringup'), 'config', 'slam_mapping.yaml')
    return LaunchDescription([
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             parameters=[params], output='screen'),
    ])
