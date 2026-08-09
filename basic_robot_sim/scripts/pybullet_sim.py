#!/usr/bin/env python3
"""
Pure Python differential-drive robot simulation with ROS 2 interfaces.

Provides:
    /cmd_vel      (subscriber) -- velocity commands
    /odom         (publisher)  -- odometry with covariance
    /tf           (publisher)  -- odom -> base_link (robot motion only)
    /scan         (publisher)  -- 360-beam LiDAR, frame 'lidar_link'
    /joint_states (publisher)  -- wheel joint angles (consumed by
                                  robot_state_publisher for link TFs)

Designed as a drop-in replacement for gz-sim (whose physics engines are
unusable on this platform) and works with the obstacle_avoider node.
Link transforms for wheels/lidar/caster come from robot_state_publisher
(launched by obstacle_avoidance.launch.py); this node owns odom -> base_link.
"""

import math

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import TransformBroadcaster


class RobotSim(Node):
    """Differential-drive robot with kinematic simulation and LiDAR."""

    def __init__(self):
        super().__init__('pybullet_robot_sim')

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_linear = 0.0
        self.v_angular = 0.0

        # Wheel joint angles (for /joint_states visualization)
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0

        # Robot params
        self.wheel_base = 0.35
        self.wheel_radius = 0.05
        self.max_linear_vel = 1.0
        self.max_angular_vel = 2.0

        # Obstacles (x, y, radius)
        self.obstacles = [
            (2.0, 0.0, 0.3),   # directly ahead at x=2
            (4.0, 0.5, 0.4),
            (6.0, -0.3, 0.3),
            (8.0, 0.0, 0.4),   # directly ahead at x=8
            (10.0, -0.5, 0.3),
        ]

        # Lidar params
        self.num_beams = 360
        self.max_range = 10.0
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = (self.angle_max - self.angle_min) / self.num_beams

        # ROS 2 interfaces
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos)

        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for simulation step (100 Hz)
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.step)
        self.last_time = self.get_clock().now()

        self.get_logger().info('PyBullet-style robot sim started')

    def cmd_vel_callback(self, msg):
        self.v_linear = max(-self.max_linear_vel,
                            min(self.max_linear_vel, msg.linear.x))
        self.v_angular = max(-self.max_angular_vel,
                             min(self.max_angular_vel, msg.angular.z))

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt > 0.1:
            dt = self.dt

        # Differential drive kinematics
        self.x += self.v_linear * math.cos(self.theta) * dt
        self.y += self.v_linear * math.sin(self.theta) * dt
        self.theta += self.v_angular * dt

        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Integrate wheel angles for /joint_states visualization
        v_left = self.v_linear - self.v_angular * self.wheel_base / 2.0
        v_right = self.v_linear + self.v_angular * self.wheel_base / 2.0
        self.left_wheel_angle += v_left / self.wheel_radius * dt
        self.right_wheel_angle += v_right / self.wheel_radius * dt

        # Publish odom
        self.publish_odom(now, dt)
        self.publish_tf(now)
        self.publish_scan(now)
        self.publish_joint_states(now)

    def publish_odom(self, now, dt):
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = self.euler_to_quaternion(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = self.v_linear
        odom.twist.twist.angular.z = self.v_angular

        # Covariance (small values)
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.01
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[35] = 0.01

        self.odom_pub.publish(odom)

    def publish_tf(self, now):
        # Simulated motion only: odom -> base_link. The URDF link transforms
        # (left/right_wheel, lidar_link, caster_link) are published by
        # robot_state_publisher from the URDF joints + /joint_states.
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        q = self.euler_to_quaternion(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def publish_scan(self, now):
        scan = LaserScan()
        scan.header.stamp = now.to_msg()
        scan.header.frame_id = 'lidar_link'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self.dt
        scan.range_min = 0.1
        scan.range_max = self.max_range

        ranges = []
        for i in range(self.num_beams):
            angle = self.angle_min + i * self.angle_increment
            beam_angle = self.theta + angle
            ray_x = math.cos(beam_angle)
            ray_y = math.sin(beam_angle)

            min_dist = self.max_range

            # Ray-circle collision: check obstacles from robot position
            for ox, oy, orad in self.obstacles:
                dx = ox - self.x
                dy = oy - self.y
                proj = dx * ray_x + dy * ray_y
                if proj > 0.1:
                    perp_dist_sq = dx * dx + dy * dy - proj * proj
                    if perp_dist_sq <= orad * orad:
                        dist = proj - math.sqrt(
                            max(0, orad * orad - perp_dist_sq))
                        if dist < min_dist:
                            min_dist = max(0.1, dist)

            ranges.append(min_dist)

        scan.ranges = ranges
        self.scan_pub.publish(scan)

    def publish_joint_states(self, now):
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self.left_wheel_angle, self.right_wheel_angle]
        self.joint_pub.publish(js)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        q = [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ]
        return q


def main():
    rclpy.init()
    node = RobotSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
