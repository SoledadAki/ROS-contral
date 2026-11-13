"""Always-on stack: chassis driver, safety arbiter, robot model, lidar, camera, APP gateway."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

LIDAR_PORT = ('/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
              'd063449b9a31f111ae12895f30d20014-if00-port0')


def generate_launch_description():
    bringup = get_package_share_directory('c21_bringup')
    params = os.path.join(bringup, 'config', 'c21.yaml')
    modes = os.path.join(bringup, 'config', 'modes.yaml')
    urdf = os.path.join(get_package_share_directory('c21_description'), 'urdf', 'c21.urdf.xacro')
    lidar = LaunchConfiguration('lidar')
    camera = LaunchConfiguration('camera')

    robot_description = ParameterValue(
        Command(['xacro ', urdf, ' lidar_yaw:=', LaunchConfiguration('lidar_yaw')]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('lidar_yaw', default_value='3.14159265'),

        Node(package='c21_base', executable='base_node', name='c21_base',
             parameters=[params], output='screen', respawn=True, respawn_delay=2.0),
        Node(package='c21_safety', executable='arbiter', name='c21_safety',
             parameters=[params], output='screen', respawn=True, respawn_delay=1.0),
        Node(package='c21_gateway', executable='gateway', name='c21_gateway',
             parameters=[params, {'modes_file': modes}], output='screen', respawn=True, respawn_delay=2.0),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='log'),

        Node(package='sllidar_ros2', executable='sllidar_node', name='sllidar_node', condition=IfCondition(lidar),
             parameters=[{'channel_type': 'serial', 'serial_port': LIDAR_PORT, 'serial_baudrate': 460800,
                          'frame_id': 'lidar_link', 'inverted': False, 'angle_compensate': True,
                          'scan_mode': 'Standard'}],
             output='log', respawn=True, respawn_delay=3.0),

        # Run the Orbbec node directly: the vendor gemini_330_series launch passes every
        # parameter explicitly, and the 336L (fw 1.2.81) rejects depth AE priority (prop 2052).
        Node(package='orbbec_camera', executable='orbbec_camera_node', name='camera', namespace='camera',
             condition=IfCondition(camera),
             parameters=[{'camera_name': 'camera', 'enable_color': True, 'enable_depth': True,
                          'color_width': 640, 'color_height': 480, 'color_fps': 15, 'color_format': 'MJPG',
                          'depth_width': 640, 'depth_height': 480, 'depth_fps': 15,
                          'enable_point_cloud': False, 'depth_registration': False, 'publish_tf': True}],
             output='log', respawn=True, respawn_delay=3.0),
        Node(package='c21_safety', executable='depth_obstacles', name='c21_depth_obstacles',
             condition=IfCondition(camera), parameters=[params], output='screen',
             respawn=True, respawn_delay=2.0),
    ])
