#!/usr/bin/env python3
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class RobotMover(Node):
    """Simulated differential-drive robot: moves forward, then turns, in a loop."""

    FORWARD_TIME_STEPS = 10
    TURN_TIME_STEPS = 10
    LINEAR_SPEED = 0.5   # m/s
    ANGULAR_SPEED = 0.8  # rad/s

    def __init__(self):
        super().__init__('robot_mover')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.step_ = 0
        self.phase_ = 'forward'
        self.timer_ = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('robot_mover started: publishing to /cmd_vel')

    def timer_callback(self):
        msg = Twist()
        if self.phase_ == 'forward':
            msg.linear.x = self.LINEAR_SPEED
            msg.angular.z = 0.0
        else:
            msg.linear.x = 0.0
            msg.angular.z = self.ANGULAR_SPEED

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Phase: {self.phase_}, linear.x={msg.linear.x:.2f}, '
            f'angular.z={msg.angular.z:.2f}')

        self.step_ += 1
        if (self.phase_ == 'forward' and self.step_ >= self.FORWARD_TIME_STEPS):
            self.phase_ = 'turn'
            self.step_ = 0
        elif (self.phase_ == 'turn' and self.step_ >= self.TURN_TIME_STEPS):
            self.phase_ = 'forward'
            self.step_ = 0


def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
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
