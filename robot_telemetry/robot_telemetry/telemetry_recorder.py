#!/usr/bin/env python3
"""Telemetry recorder node.

Subscribes to the robot's operational topics and logs every message to
structured CSV files (one per topic) plus a JSON-lines log:

    <log_dir>/<run_id>/
        cmd_vel.csv           linear/angular velocities
        odom.csv              pose + velocities
        scan.csv              lidar summary (min/mean range)
        joint_states.csv      drive wheel joint angles (sim)
        arm_joint_states.csv  arm joint angles + gripper
        arm_pose.csv          end-effector position
        pick_place_status.csv state machine status text
        telemetry.jsonl       every event as one JSON object per line

Run:
    ros2 run robot_telemetry telemetry_recorder
    ros2 run robot_telemetry telemetry_recorder --ros-args -p log_dir:=/tmp/tel
"""

import math
import os
import queue
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String

from robot_telemetry.telemetry_csv import CsvLog, JsonlLog

DEFAULT_BASE = os.path.join(os.path.expanduser('~'), 'telemetry_logs')


class TelemetryRecorder(Node):
    """Logs subscribed topics to CSV + JSONL under <log_dir>/<run_id>/.

    ROS callbacks only enqueue rows; a background writer thread drains the
    queue and flushes files in batches, so file I/O never blocks the spin
    thread (or the 100 Hz simulation publishers that feed it).
    """

    STATUS_PERIOD_S = 5.0
    BATCH_PERIOD_S = 0.25

    def __init__(self):
        super().__init__('telemetry_recorder')
        self.declare_parameter('log_dir', '')

        base = self.get_parameter('log_dir').value or DEFAULT_BASE
        run_id = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_dir = os.path.join(base, run_id)
        os.makedirs(self.log_dir, exist_ok=True)

        self._queue = queue.Queue()
        self._stop = threading.Event()
        self._csv = {}
        self._jsonl = JsonlLog(os.path.join(self.log_dir, 'telemetry.jsonl'))
        self._counts = {}

        self._open_topic_logs()
        self._create_subscribers()
        self._writer = threading.Thread(target=self._write_worker,
                                        name='telemetry-writer',
                                        daemon=True)
        self._writer.start()
        self.create_timer(self.STATUS_PERIOD_S, self._status_timer)
        self.get_logger().info(
            'telemetry_recorder started: logging to %s' % self.log_dir)

    # ------------------------------------------------------------------
    def _open_topic_logs(self):
        self._csv['/cmd_vel'] = CsvLog(
            os.path.join(self.log_dir, 'cmd_vel.csv'),
            ['sec', 'nanosec', 'linear_x', 'linear_y', 'angular_z'])
        self._csv['/odom'] = CsvLog(
            os.path.join(self.log_dir, 'odom.csv'),
            ['sec', 'nanosec', 'pos_x', 'pos_y', 'pos_z',
             'orient_w', 'lin_vel_x', 'ang_vel_z'])
        self._csv['/scan'] = CsvLog(
            os.path.join(self.log_dir, 'scan.csv'),
            ['sec', 'nanosec', 'num_beams', 'range_min', 'range_max',
             'min_range', 'mean_range'])
        self._csv['/joint_states'] = CsvLog(
            os.path.join(self.log_dir, 'joint_states.csv'),
            ['sec', 'nanosec', 'left_wheel_joint', 'right_wheel_joint'])
        self._csv['/arm_joint_states'] = CsvLog(
            os.path.join(self.log_dir, 'arm_joint_states.csv'),
            ['sec', 'nanosec', 'waist', 'shoulder', 'elbow',
             'gripper_left', 'gripper_right'])
        self._csv['/arm_pose'] = CsvLog(
            os.path.join(self.log_dir, 'arm_pose.csv'),
            ['sec', 'nanosec', 'x', 'y', 'z'])
        self._csv['/pick_place_status'] = CsvLog(
            os.path.join(self.log_dir, 'pick_place_status.csv'),
            ['sec', 'nanosec', 'status'])

    # ------------------------------------------------------------------
    def _create_subscribers(self):
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        sensor_qos = QoSProfile(depth=10,
                                reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Twist, '/cmd_vel',
                                 self._on_cmd_vel, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile=sensor_qos)
        self.create_subscription(JointState, '/joint_states',
                                 self._on_joint_states, 10)
        self.create_subscription(JointState, '/arm_joint_states',
                                 self._on_arm_joint_states, 10)
        self.create_subscription(PoseStamped, '/arm_pose',
                                 self._on_arm_pose, 10)
        self.create_subscription(String, '/pick_place_status',
                                 self._on_status, 10)

    # ------------------------------------------------------------------
    def _stamp(self, msg):
        """Return (sec, nanosec) from the message header or current time."""
        header = getattr(msg, 'header', None)
        if header is not None and header.stamp.sec != 0:
            return header.stamp.sec, header.stamp.nanosec
        now = self.get_clock().now().to_msg()
        return now.sec, now.nanosec

    def _record(self, topic, row, data):
        self._queue.put((topic, row, data))

    def _write_worker(self):
        """Background thread: drain the queue, write rows in batches."""
        while not self._stop.is_set():
            try:
                item = self._queue.get(timeout=self.BATCH_PERIOD_S)
            except queue.Empty:
                self._flush_all()
                continue
            batch = [item]
            while True:
                try:
                    batch.append(self._queue.get_nowait())
                except queue.Empty:
                    break
            for topic, row, data in batch:
                self._csv[topic].write(row)
                self._jsonl.write({'topic': topic,
                                   'sec': row[0], 'nanosec': row[1],
                                   'data': data})
                self._counts[topic] = self._counts.get(topic, 0) + 1
            self._flush_all()

    def _flush_all(self):
        for csv_log in self._csv.values():
            csv_log.flush()
        self._jsonl.flush()

    # ------------------------------------------------------------------
    def _on_cmd_vel(self, msg):
        s, ns = self._stamp(msg)
        self._record('/cmd_vel', [s, ns, msg.linear.x, msg.linear.y,
                                  msg.angular.z],
                     {'linear_x': msg.linear.x, 'linear_y': msg.linear.y,
                      'angular_z': msg.angular.z})

    def _on_odom(self, msg):
        s, ns = self._stamp(msg)
        self._record('/odom',
                     [s, ns, msg.pose.pose.position.x,
                      msg.pose.pose.position.y,
                      msg.pose.pose.position.z,
                      msg.pose.pose.orientation.w,
                      msg.twist.twist.linear.x,
                      msg.twist.twist.angular.z],
                     {'pos_x': msg.pose.pose.position.x,
                      'pos_y': msg.pose.pose.position.y,
                      'lin_vel_x': msg.twist.twist.linear.x,
                      'ang_vel_z': msg.twist.twist.angular.z})

    def _on_scan(self, msg):
        s, ns = self._stamp(msg)
        finite = [r for r in msg.ranges
                  if math.isfinite(r) and r > msg.range_min]
        mn = min(finite) if finite else msg.range_max
        mean = sum(finite) / len(finite) if finite else msg.range_max
        self._record('/scan',
                     [s, ns, len(msg.ranges), msg.range_min, msg.range_max,
                      mn, round(mean, 4)],
                     {'num_beams': len(msg.ranges), 'min_range': mn,
                      'mean_range': round(mean, 4)})

    def _on_joint_states(self, msg):
        s, ns = self._stamp(msg)
        pos = list(msg.position) + [0.0] * 2
        self._record('/joint_states',
                     [s, ns, pos[0], pos[1]],
                     {'left_wheel_joint': pos[0],
                      'right_wheel_joint': pos[1]})

    def _on_arm_joint_states(self, msg):
        s, ns = self._stamp(msg)
        pos = list(msg.position) + [0.0] * 5
        self._record('/arm_joint_states',
                     [s, ns, pos[0], pos[1], pos[2], pos[3], pos[4]],
                     {'waist': pos[0], 'shoulder': pos[1], 'elbow': pos[2],
                      'gripper_left': pos[3], 'gripper_right': pos[4]})

    def _on_arm_pose(self, msg):
        s, ns = self._stamp(msg)
        p = msg.pose.position
        self._record('/arm_pose', [s, ns, p.x, p.y, p.z],
                     {'x': p.x, 'y': p.y, 'z': p.z})

    def _on_status(self, msg):
        s, ns = self._stamp(msg)
        self._record('/pick_place_status', [s, ns, msg.data],
                     {'status': msg.data})

    # ------------------------------------------------------------------
    def _status_timer(self):
        total = sum(self._counts.values())
        self.get_logger().info(
            'status: %d messages (%s), files in %s' %
            (total, ', '.join('%s=%d' % (t, self._counts.get(t, 0))
                              for t in sorted(self._counts)), self.log_dir))

    def shutdown(self):
        # Drain the queue, then close the files
        self._stop.set()
        if self._writer is not None and self._writer.is_alive():
            self._writer.join(timeout=5.0)
        if not self._queue.empty():
            batch = []
            while not self._queue.empty():
                batch.append(self._queue.get_nowait())
            for topic, row, data in batch:
                self._csv[topic].write(row)
                self._jsonl.write({'topic': topic,
                                   'sec': row[0], 'nanosec': row[1],
                                   'data': data})
        for csv_log in self._csv.values():
            csv_log.close()
        self._jsonl.close()
        self.get_logger().info('telemetry_recorder stopped: files in %s'
                               % self.log_dir)


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
