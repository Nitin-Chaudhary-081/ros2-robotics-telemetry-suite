#!/usr/bin/env python3
"""Pick-and-place state machine for the 3-DOF arm.

Simulates the full sequence: move to target, close gripper, lift,
move to place location, release, and return home.

Publishes:
    /joint_states   sensor_msgs/msg/JointState   joint angles + gripper
    /arm_pose       geometry_msgs/msg/PoseStamped  end-effector pose (frame "arm_tool")
    /pick_place_status std_msgs/msg/String       human-readable status text

Joint names: waist_joint, shoulder_joint, elbow_joint,
             gripper_left_joint, gripper_right_joint
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
from geometry_msgs.msg import PoseStamped

from arm_pick_place.arm_kinematics import ThreeDOFArm


class PickPlaceController(Node):
    """State-machine node that drives the arm through a pick-and-place cycle."""

    RATE_HZ = 20.0

    # Gripper opening in metres (0.0 == fully closed)
    GRIPPER_OPEN = 0.04
    GRIPPER_CLOSED = 0.005

    # Waypoints
    PICK_XY = (0.40, 0.30)
    PLACE_XY = (0.40, -0.30)
    TABLE_Z = 0.12          # object height on the table
    CLEAR_Z = 0.30          # safe transit height
    HOME = (0.0, 0.0, 0.0)

    # Motion timing (seconds per state)
    DURATION = {
        'APPROACH_PICK': 2.0,
        'LOWER_TO_PICK': 1.5,
        'GRASP': 1.0,
        'LIFT': 1.5,
        'APPROACH_PLACE': 2.0,
        'LOWER_TO_PLACE': 1.5,
        'RELEASE': 1.0,
        'RETRACT': 1.5,
        'RETURN_HOME': 2.0,
    }

    JOINT_NAMES = ['waist_joint', 'shoulder_joint', 'elbow_joint',
                   'gripper_left_joint', 'gripper_right_joint']

    def __init__(self):
        super().__init__('pick_place_controller')
        self.arm = ThreeDOFArm()

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/arm_pose', 10)
        self.status_pub = self.create_publisher(String, '/pick_place_status', 10)

        self.state = 'HOME'
        self.timer_t0 = time.monotonic()

        # Interpolated state
        self.current_theta = list(self.HOME)
        self.gripper = self.GRIPPER_OPEN

        # Target joint config for the active state
        self.target_theta = list(self.HOME)
        self.target_gripper = self.GRIPPER_OPEN

        # Bookkeeping for logging
        self.object_held = False
        self.cycle = 0

        self.timer = self.create_timer(1.0 / self.RATE_HZ, self.control_loop)
        self.get_logger().info(
            'pick_place_controller started: publishing /joint_states, '
            '/arm_pose, /pick_place_status')
        self._log('Initialising pick-and-place cycle')

    # ------------------------------------------------------------------
    def control_loop(self):
        """Timer callback: advance the state machine and publish state."""
        self._state_machine()
        self._publish()

    # ------------------------------------------------------------------
    def _state_machine(self):
        if self.state == 'DONE':
            if time.monotonic() - self.timer_t0 > 5.0:
                self.state = 'HOME'
                self.timer_t0 = time.monotonic()
                self._log('Restarting cycle %d' % (self.cycle + 1))
            return

        if self.state == 'HOME':
            self.target_theta = list(self.HOME)
            self.target_gripper = self.GRIPPER_OPEN
            self.state = 'APPROACH_PICK'
            self.timer_t0 = time.monotonic()
            self._log('STATE: APPROACH_PICK  (moving above pick location)')
            return

        elapsed = time.monotonic() - self.timer_t0
        if elapsed < self.DURATION[self.state]:
            return  # keep moving toward current target

        if self.state == 'APPROACH_PICK':
            self._set_target_cartesian(self.PICK_XY[0], self.PICK_XY[1],
                                       self.CLEAR_Z)
            self.state = 'LOWER_TO_PICK'
            self.timer_t0 = time.monotonic()
            self._log('STATE: LOWER_TO_PICK  (descending to object)')

        elif self.state == 'LOWER_TO_PICK':
            self._set_target_cartesian(self.PICK_XY[0], self.PICK_XY[1],
                                       self.TABLE_Z)
            self.state = 'GRASP'
            self.timer_t0 = time.monotonic()
            self._log('STATE: GRASP  (closing gripper)')

        elif self.state == 'GRASP':
            self.target_gripper = self.GRIPPER_CLOSED
            self.object_held = True
            self.state = 'LIFT'
            self.timer_t0 = time.monotonic()
            self._log('STATE: LIFT  (object grasped, lifting)')

        elif self.state == 'LIFT':
            self._set_target_cartesian(self.PICK_XY[0], self.PICK_XY[1],
                                       self.CLEAR_Z)
            self.state = 'APPROACH_PLACE'
            self.timer_t0 = time.monotonic()
            self._log('STATE: APPROACH_PLACE  (carrying object to place)')

        elif self.state == 'APPROACH_PLACE':
            self._set_target_cartesian(self.PLACE_XY[0], self.PLACE_XY[1],
                                       self.CLEAR_Z)
            self.state = 'LOWER_TO_PLACE'
            self.timer_t0 = time.monotonic()
            self._log('STATE: LOWER_TO_PLACE  (descending to place location)')

        elif self.state == 'LOWER_TO_PLACE':
            self._set_target_cartesian(self.PLACE_XY[0], self.PLACE_XY[1],
                                       self.TABLE_Z)
            self.state = 'RELEASE'
            self.timer_t0 = time.monotonic()
            self._log('STATE: RELEASE  (opening gripper)')

        elif self.state == 'RELEASE':
            self.target_gripper = self.GRIPPER_OPEN
            self.object_held = False
            self.state = 'RETRACT'
            self.timer_t0 = time.monotonic()
            self._log('STATE: RETRACT  (object placed, lifting away)')

        elif self.state == 'RETRACT':
            self._set_target_cartesian(self.PLACE_XY[0], self.PLACE_XY[1],
                                       self.CLEAR_Z)
            self.state = 'RETURN_HOME'
            self.timer_t0 = time.monotonic()
            self._log('STATE: RETURN_HOME  (returning to rest pose)')

        elif self.state == 'RETURN_HOME':
            self.target_theta = list(self.HOME)
            self.target_gripper = self.GRIPPER_OPEN
            self.cycle += 1
            self._log('CYCLE %d COMPLETE: pick-and-place finished' % self.cycle)
            # Stay at home; restart after a pause.
            self.state = 'DONE'
            self.timer_t0 = time.monotonic()

    # ------------------------------------------------------------------
    def _set_target_cartesian(self, x, y, z):
        """Solve IK for a Cartesian target and store the joint target."""
        joints = self.arm.inverse_kinematics(x, y, z, elbow_up=True)
        if joints is None:
            self._log('WARNING: target (%.2f, %.2f, %.2f) unreachable' % (x, y, z))
            joints = self.arm.forward_kinematics(*self.current_theta)
        self.target_theta = list(joints)

    # ------------------------------------------------------------------
    def _publish(self):
        """Move the current joint state toward the target and publish."""
        step = 1.0 / (self.DURATION[self.state] * self.RATE_HZ) if \
            self.DURATION.get(self.state) else 0.05
        for i in range(3):
            self.current_theta[i] += (self.target_theta[i] - self.current_theta[i]) * step
        self.gripper += (self.target_gripper - self.gripper) * step

        now = self.get_clock().now().to_msg()
        js = JointState()
        js.header = Header(stamp=now)
        js.header.frame_id = 'arm_base'
        js.name = list(self.JOINT_NAMES)
        js.position = [self.current_theta[0], self.current_theta[1],
                       self.current_theta[2],
                       self.gripper, -self.gripper]
        js.velocity = [0.0] * 5
        js.effort = [0.0] * 5
        self.joint_pub.publish(js)

        xyz = self.arm.forward_kinematics(*self.current_theta)
        pose = PoseStamped()
        pose.header = Header(stamp=now)
        pose.header.frame_id = 'arm_base'
        pose.pose.position.x = xyz[0]
        pose.pose.position.y = xyz[1]
        pose.pose.position.z = xyz[2]
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

    # ------------------------------------------------------------------
    def _log(self, text):
        self.get_logger().info(text)
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceController()
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
