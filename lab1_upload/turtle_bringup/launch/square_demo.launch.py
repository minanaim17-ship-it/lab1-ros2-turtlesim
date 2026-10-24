"""Starts the whole system, waits for it to come up, then triggers the square.

  ros2 launch turtle_bringup square_demo.launch.py side_length:=3.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    bringup = os.path.join(get_package_share_directory('turtle_bringup'),
                           'launch', 'bringup.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('max_linear_speed', default_value='2.0'),
        DeclareLaunchArgument('max_angular_speed', default_value='2.0'),
        DeclareLaunchArgument('side_length', default_value='3.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={
                'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            }.items()),
        TimerAction(period=4.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'action', 'send_goal', '/draw_square',
                     'turtle_interfaces/action/DrawSquare',
                     [TextSubstitution(text='{side_length: '),
                      LaunchConfiguration('side_length'),
                      TextSubstitution(text='}')],
                     '--feedback'],
                output='screen'),
        ]),
    ])
