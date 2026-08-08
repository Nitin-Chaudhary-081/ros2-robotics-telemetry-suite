#!/usr/bin/env python3
"""
Launch the obstacle-avoidance demo: custom kinematics/LiDAR sim + avoider.

ros2 launch basic_robot_sim obstacle_avoidance.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    sim_path = os.path.join(
        get_package_share_directory('basic_robot_sim'),
        'scripts', 'pybullet_sim.py')

    return LaunchDescription([
        LogInfo(msg='Starting obstacle-avoidance demo '
                    '(pybullet_sim + obstacle_avoider)'),
        ExecuteProcess(
            cmd=['python3', sim_path],
            name='pybullet_sim',
            output='screen'),
        Node(
            package='basic_robot_sim',
            executable='obstacle_avoider',
            name='obstacle_avoider',
            output='screen'),
    ])
