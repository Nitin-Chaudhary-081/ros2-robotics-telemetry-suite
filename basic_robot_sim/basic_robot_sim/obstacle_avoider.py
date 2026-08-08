#!/usr/bin/env python3
import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ObstacleAvoider(Node):
    """Subscribes to LiDAR scans and steers the robot away from obstacles."""

    SAFETY_DISTANCE = 1.2       # m - trigger avoidance when closer than this
    FORWARD_SPEED = 0.25        # m/s
    TURN_SPEED = 0.6            # rad/s
    CENTER_ANGLE_DEG = 30       # half-width of the "ahead" wedge

    def __init__(self):
        super().__init__('obstacle_avoider')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_cmd)
        self.center_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.get_logger().info('obstacle_avoider started: listening on /scan')

    def scan_callback(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0 or msg.angle_min is None:
            return

        def distance_at(angle_deg):
            idx = int((math.radians(angle_deg) - msg.angle_min) / msg.angle_increment)
            idx = max(0, min(n - 1, idx))
            r = msg.ranges[idx]
            return r if math.isfinite(r) and r > msg.range_min else float('inf')

        self.center_distance = min(
            (distance_at(a) for a in range(-self.CENTER_ANGLE_DEG, self.CENTER_ANGLE_DEG + 1, 1)),
            default=float('inf'))
        self.left_distance = min(
            (distance_at(a) for a in range(20, 71, 1)), default=float('inf'))
        self.right_distance = min(
            (distance_at(a) for a in range(-70, -19, 1)), default=float('inf'))

        self.get_logger().info(
            f'scan: center={self.center_distance:.2f} m, '
            f'left={self.left_distance:.2f} m, right={self.right_distance:.2f} m')

    def control_cmd(self):
        msg = Twist()
        if getattr(self, 'center_distance', float('inf')) < self.SAFETY_DISTANCE:
            # Obstacle ahead: turn toward the more open side
            if self.left_distance >= self.right_distance:
                msg.angular.z = self.TURN_SPEED
                self.get_logger().info('OBSTACLE ahead -> turning LEFT')
            else:
                msg.angular.z = -self.TURN_SPEED
                self.get_logger().info('OBSTACLE ahead -> turning RIGHT')
        else:
            msg.linear.x = self.FORWARD_SPEED
            self.get_logger().info('PATH CLEAR -> moving forward')
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
