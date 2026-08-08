#!/usr/bin/env python3
"""Launch the 3-DOF arm pick-and-place demo controller.

    ros2 launch arm_pick_place pick_place_demo.launch.py
"""

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='Starting pick-and-place controller '
                    '(/joint_states, /arm_pose, /pick_place_status)'),
        Node(
            package='arm_pick_place',
            executable='pick_place_controller',
            name='pick_place_controller',
            output='screen'),
    ])
