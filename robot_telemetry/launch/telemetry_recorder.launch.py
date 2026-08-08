#!/usr/bin/env python3
"""Launch the telemetry recorder.

    ros2 launch robot_telemetry telemetry_recorder.launch.py log_dir:=/tmp/tel
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'log_dir', default_value='',
            description='Parent directory for telemetry run logs'),
        LogInfo(msg='Starting telemetry recorder'),
        Node(
            package='robot_telemetry',
            executable='telemetry_recorder',
            name='telemetry_recorder',
            output='screen',
            parameters=[{'log_dir': LaunchConfiguration('log_dir')}]),
    ])
