#!/usr/bin/env python3
"""
Pure Python differential-drive robot simulation with ROS 2 interfaces.

Provides:
    /cmd_vel              (subscriber) -- velocity commands
    /odom                 (publisher)  -- odometry with covariance
    /tf                   (publisher)  -- odom -> base_link (robot motion only)
    /scan                 (publisher)  -- 360-beam LiDAR, frame 'lidar_link'
    /joint_states         (publisher)  -- wheel joint angles (consumed by
                                          robot_state_publisher for link TFs)
    /simulation_status    (publisher)  -- IDLE / RUNNING / PAUSED / COMPLETED
    /reset_simulation     (service)    -- return to the default pose (0, 0, 0)
    /pause_simulation     (service)    -- freeze physics (odom stays stable)
    /resume_simulation    (service)    -- continue from PAUSED

The simulation is intentionally bounded: once the robot has travelled
max_travel_distance meters from the origin (or left the configurable safety
boundary), it enters COMPLETED and stops responding to /cmd_vel until reset.

Designed as a drop-in replacement for gz-sim (whose physics engines are
unusable on this platform) and works with the obstacle_avoider node.
Link transforms for wheels/lidar/caster come from robot_state_publisher
(launched by obstacle_avoidance.launch.py); this node owns odom -> base_link.
"""

import math

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from obstacle_world import get_obstacles
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster


class RobotSim(Node):
    """Differential-drive robot with kinematic simulation and LiDAR."""

    STATE_IDLE = 'IDLE'
    STATE_RUNNING = 'RUNNING'
    STATE_PAUSED = 'PAUSED'
    STATE_COMPLETED = 'COMPLETED'
    STATE_RESETTING = 'RESETTING'
    STATE_ERROR = 'ERROR'

    def __init__(self):
        super().__init__('pybullet_robot_sim')

        # Simulation control parameters
        self.declare_parameter('max_travel_distance', 12.0)
        self.declare_parameter('auto_drive', True)
        self.declare_parameter('max_x', 15.0)
        self.declare_parameter('min_x', -5.0)
        self.declare_parameter('max_y', 8.0)
        self.declare_parameter('min_y', -8.0)
        self.max_travel_distance = self.get_parameter(
            'max_travel_distance').value
        self.auto_drive = self.get_parameter('auto_drive').value
        self.max_x = self.get_parameter('max_x').value
        self.min_x = self.get_parameter('min_x').value
        self.max_y = self.get_parameter('max_y').value
        self.min_y = self.get_parameter('min_y').value

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

        # Obstacles (x, y, radius) -- shared with obstacle_visualizer so the
        # Foxglove markers match the LiDAR/collision world exactly
        self.obstacles = get_obstacles()

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

        # Control services + status topic (transient-local so late
        # subscribers still see the current state)
        status_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(
            String, '/simulation_status', status_qos)
        self.create_service(
            Empty, '/reset_simulation', self.reset_callback)
        self.create_service(
            Empty, '/pause_simulation', self.pause_callback)
        self.create_service(
            Empty, '/resume_simulation', self.resume_callback)

        # Timer for simulation step (100 Hz)
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.step)
        self.last_time = self.get_clock().now()
        # Keep the status topic fresh for very late subscribers
        self.create_timer(1.0, self.publish_status)

        # Simulation state machine (after all interfaces exist)
        if self.auto_drive:
            self.set_state(self.STATE_RUNNING)
        else:
            self.set_state(self.STATE_IDLE)

        self.get_logger().info(
            f'PyBullet-style robot sim started '
            f'(state={self.state}, max_travel_distance='
            f'{self.max_travel_distance} m)')

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------
    def set_state(self, new_state):
        """Update the simulation state and publish /simulation_status."""
        self.state = new_state
        self.publish_status()
        self.get_logger().info(f'simulation state -> {self.state}')

    def publish_status(self):
        msg = String()
        msg.data = self.state
        self.status_pub.publish(msg)

    def reset_callback(self, request, response):
        """Reset robot pose/velocities/odometry to the default (0, 0, 0)."""
        was_paused = self.state == self.STATE_PAUSED
        self.set_state(self.STATE_RESETTING)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_linear = 0.0
        self.v_angular = 0.0
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        now = self.get_clock().now()
        self.publish_odom(now, 0.0)
        self.publish_tf(now)
        self.publish_joint_states(now)
        if was_paused:
            self.set_state(self.STATE_PAUSED)
        elif self.auto_drive:
            self.set_state(self.STATE_RUNNING)
        else:
            self.set_state(self.STATE_IDLE)
        self.get_logger().info('simulation reset: pose restored to (0, 0, 0)')
        return response

    def pause_callback(self, request, response):
        """Freeze the simulation (odom/scan/TF keep publishing, pose stable)."""
        if self.state in (self.STATE_RUNNING, self.STATE_IDLE):
            self.set_state(self.STATE_PAUSED)
        return response

    def resume_callback(self, request, response):
        """Resume a paused simulation."""
        if self.state == self.STATE_PAUSED:
            self.set_state(self.STATE_RUNNING)
        return response

    # ------------------------------------------------------------------
    # Core loop
    # ------------------------------------------------------------------
    def cmd_vel_callback(self, msg):
        if self.state in (self.STATE_PAUSED, self.STATE_COMPLETED):
            return
        self.v_linear = max(-self.max_linear_vel,
                            min(self.max_linear_vel, msg.linear.x))
        self.v_angular = max(-self.max_angular_vel,
                             min(self.max_angular_vel, msg.angular.z))
        if self.state == self.STATE_IDLE and (msg.linear.x or msg.angular.z):
            self.set_state(self.STATE_RUNNING)

    def check_bounds(self):
        """Return True once the course is complete (max distance/boundary)."""
        traveled = math.hypot(self.x, self.y)
        if traveled >= self.max_travel_distance:
            return True
        if self.x > self.max_x or self.x < self.min_x:
            return True
        if self.y > self.max_y or self.y < self.min_y:
            return True
        return False

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt > 0.1:
            dt = self.dt

        if self.state == self.STATE_RUNNING and self.check_bounds():
            self.set_state(self.STATE_COMPLETED)
            self.v_linear = 0.0
            self.v_angular = 0.0
            self.get_logger().info(
                'simulation COMPLETED: max travel distance / boundary '
                'reached - robot stopped (call /reset_simulation to rerun)')

        if self.state != self.STATE_RUNNING:
            # Paused/completed/idle: no integration, but keep the world
            # stream fresh (Foxglove stays connected and pose is stable)
            self.publish_odom(now, dt)
            self.publish_tf(now)
            self.publish_scan(now)
            self.publish_joint_states(now)
            return

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
