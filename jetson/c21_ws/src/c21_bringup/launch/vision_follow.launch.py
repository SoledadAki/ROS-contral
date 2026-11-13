"""Vision follow mode: ros2 launch c21_bringup vision_follow.launch.py (needs the host c21-yolo service)"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('c21_bringup'), 'config', 'c21.yaml')
    return LaunchDescription([
        Node(package='c21_behaviors', executable='vision_follow', name='c21_behavior', output='screen',
             parameters=[params]),
    ])
