"""Navigation mode: map_server + AMCL on a saved map, Nav2 planner/controller/behaviors/BT.

    ros2 launch c21_bringup navigation.launch.py map:=<name>     (<maps_dir>/<name>.yaml)

Velocity goes to cmd_vel_nav, the lowest-priority arbiter source.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

NODES = ['map_server', 'amcl', 'controller_server', 'planner_server', 'behavior_server', 'bt_navigator']


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('c21_bringup'), 'config')
    params = os.path.join(cfg, 'nav2.yaml')
    # own tree: replan at 1 Hz, on blockage clear costmaps + wait and retry; no Spin / BackUp
    bt = {'default_nav_to_pose_bt_xml': os.path.join(cfg, 'c21_nav_to_pose.xml')}
    map_yaml = PathJoinSubstitution([LaunchConfiguration('maps_dir'),
                                     [LaunchConfiguration('map'), '.yaml']])
    to_nav = [('cmd_vel', 'cmd_vel_nav')]
    return LaunchDescription([
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('maps_dir', default_value='/root/c21_maps'),

        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[params, {'yaml_filename': map_yaml}], output='screen'),
        Node(package='nav2_amcl', executable='amcl', name='amcl', parameters=[params], output='screen'),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=[params], remappings=to_nav, output='screen'),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[params], output='screen'),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             parameters=[params], remappings=to_nav, output='screen'),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=[params, bt], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_nav',
             parameters=[{'autostart': True, 'node_names': NODES, 'bond_timeout': 4.0}], output='screen'),
    ])
