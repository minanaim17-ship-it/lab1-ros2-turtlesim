"""Starts turtlesim + goToPose server + reset service + square server.

  ros2 launch turtle_bringup bringup.launch.py
  ros2 launch turtle_bringup bringup.launch.py max_linear_speed:=1.0 max_angular_speed:=1.5
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params_file = os.path.join(get_package_share_directory('turtle_bringup'),
                               'config', 'turtle_params.yaml')

    max_linear = DeclareLaunchArgument(
        'max_linear_speed', default_value='2.0', description='Max linear speed (units/s)')
    max_angular = DeclareLaunchArgument(
        'max_angular_speed', default_value='2.0', description='Max angular speed (rad/s)')

    return LaunchDescription([
        max_linear,
        max_angular,
        Node(package='turtlesim', executable='turtlesim_node', name='turtlesim',
             output='screen'),
        Node(package='turtle_control', executable='go_to_pose_server',
             name='go_to_pose_server', output='screen',
             parameters=[params_file, {
                 'max_linear_speed': ParameterValue(
                     LaunchConfiguration('max_linear_speed'), value_type=float),
                 'max_angular_speed': ParameterValue(
                     LaunchConfiguration('max_angular_speed'), value_type=float),
             }]),
        Node(package='turtle_control', executable='reset_service',
             name='reset_service', output='screen'),
        Node(package='turtle_control', executable='draw_square_server',
             name='draw_square_server', output='screen', parameters=[params_file]),
    ])
