#!/usr/bin/env python3
"""
Publish the simulator's obstacle definitions as 3D cylinders for Foxglove.

Reads the shared obstacle world (scripts/obstacle_world.py -- the exact
definitions pybullet_sim uses for its LiDAR/collision logic) and publishes
a MarkerArray of CYLINDER markers on /visualization/obstacles in the odom
frame at 10 Hz.

Obstacles are static, so the publisher uses transient-local durability:
Foxglove receives the markers immediately even if it connects after this
node started (continuous publishing also keeps them fresh).
"""

from obstacle_world import get_obstacles
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

# Cylinder height, chosen to the robot/environment scale (wheels are
# 0.05 m radius, lidar sits at 0.07 m): clearly visible, upright cylinders.
OBSTACLE_HEIGHT = 0.5

# Static obstacles: 10 Hz is plenty (c.md says 5-10 Hz, no 100 Hz).
PUBLISH_PERIOD = 0.1

MARKER_NAMESPACE = 'simulated_obstacles'


def build_marker_array(obstacles, stamp):
    """
    Convert (x, y, radius) obstacle tuples into a MarkerArray.

    Pure function (no ROS node state) so it can be unit-tested directly.
    Marker positions/radii come from the simulator's obstacle definitions.
    """
    markers = MarkerArray()
    for index, (x, y, radius) in enumerate(obstacles):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = 'odom'
        marker.ns = MARKER_NAMESPACE
        marker.id = index
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = OBSTACLE_HEIGHT / 2.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        # scale is diameter (2*r), not radius
        marker.scale.x = 2.0 * radius
        marker.scale.y = 2.0 * radius
        marker.scale.z = OBSTACLE_HEIGHT
        marker.color.r = 0.95
        marker.color.g = 0.45
        marker.color.b = 0.05
        marker.color.a = 1.0
        # lifetime left at its default (zero duration) = persistent marker.
        # NOTE: assigning rclpy.duration.Duration here aborts in Jazzy
        # (rosidl C-extension assert), and the default is already 0.
        markers.markers.append(marker)
    return markers


class ObstacleVisualizer(Node):
    """Publishes the simulator's obstacles as CYLINDER markers."""

    def __init__(self):
        super().__init__('obstacle_visualizer')

        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(
            MarkerArray, '/visualization/obstacles', qos)

        self.obstacles = get_obstacles()
        self.timer = self.create_timer(PUBLISH_PERIOD, self.publish_obstacles)

        self.get_logger().info(
            f'obstacle_visualizer: publishing {len(self.obstacles)} obstacle '
            f'cylinders on /visualization/obstacles at 10 Hz (frame odom)')

    def publish_obstacles(self):
        stamp = self.get_clock().now().to_msg()
        self.pub.publish(build_marker_array(self.obstacles, stamp))


def main():
    rclpy.init()
    node = ObstacleVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
