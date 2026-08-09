#!/usr/bin/env python3
"""
Launch the obstacle-avoidance demo: sim + avoider + robot_state_publisher.

ros2 launch basic_robot_sim obstacle_avoidance.launch.py

robot_state_publisher owns the URDF (robot.urdf) and publishes the
base_link -> {left_wheel, right_wheel, lidar_link, caster_link} link
transforms from /joint_states. pybullet_sim publishes odom -> base_link.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('basic_robot_sim')
    sim_path = os.path.join(pkg_share, 'scripts', 'pybullet_sim.py')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')

    with open(urdf_path) as fh:
        robot_description = fh.read()

    return LaunchDescription([
        LogInfo(msg='Starting obstacle-avoidance demo '
                    '(pybullet_sim + obstacle_avoider + robot_state_publisher)'),
        ExecuteProcess(
            cmd=['python3', sim_path],
            name='pybullet_sim',
            output='screen'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]),
        Node(
            package='basic_robot_sim',
            executable='obstacle_avoider',
            name='obstacle_avoider',
            output='screen'),
    ])
