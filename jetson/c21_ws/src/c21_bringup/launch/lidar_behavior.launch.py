"""Lidar behaviour mode: ros2 launch c21_bringup lidar_behavior.launch.py behavior:=avoid|follow|guard"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('c21_bringup'), 'config', 'c21.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('behavior'),
        Node(package='c21_behaviors', executable='lidar_behavior', name='c21_behavior', output='screen',
             parameters=[params, {'behavior': LaunchConfiguration('behavior')}]),
    ])
