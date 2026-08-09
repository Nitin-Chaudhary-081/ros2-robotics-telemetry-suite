"""
Unit tests for the bounded/controllable simulation (d.md).

Tests instantiate the sim and avoider nodes headless and drive their
handlers directly (no spin), verifying: reset -> pose zero, pause freezes
movement, resume continues, COMPLETED stops cmd_vel, max travel distance
and safety boundary bound the robot, auto_drive=false leaves the robot to
external /cmd_vel, and odom/TF stay valid after a reset.
"""

import os
import sys

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))
sys.path.insert(0, os.path.join(HERE, '..'))

# Isolate from any live stack: tests must not see a running sim/avoider
# publishing on the default ROS domain.
os.environ.setdefault('ROS_DOMAIN_ID', '99')

from basic_robot_sim.obstacle_avoider import ObstacleAvoider  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from obstacle_world import OBSTACLES  # noqa: E402
from pybullet_sim import RobotSim  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from rclpy.qos import QoSProfile  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

rclpy.init()
SIM = None
AVOIDER = None
ODOM_MSGS = []
TF_MSGS = []
CMD_MSGS = []
STATUS_MSGS = []


def pump(node, timeout=0.2):
    """Spin briefly so intra-process pub/sub messages get delivered."""
    end = node.get_clock().now().nanoseconds + int(timeout * 1e9)
    while node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.01)


def make_scan(ranges=(float('inf'),) * 360):
    msg = LaserScan()
    msg.angle_min = -3.141592653589793
    msg.angle_max = 3.141592653589793
    msg.angle_increment = 2 * 3.141592653589793 / 360
    msg.range_min = 0.1
    msg.range_max = 10.0
    msg.ranges = list(ranges)
    return msg


@pytest.fixture(autouse=True)
def fresh_state():
    SIM.timer.cancel()
    AVOIDER.timer.cancel()
    SIM.set_state(RobotSim.STATE_RUNNING)
    SIM.x = 0.0
    SIM.y = 0.0
    SIM.theta = 0.0
    SIM.v_linear = 0.0
    SIM.v_angular = 0.0
    SIM.left_wheel_angle = 0.0
    SIM.right_wheel_angle = 0.0
    AVOIDER.sim_status = 'RUNNING'
    AVOIDER.auto_drive = True
    for bucket in (ODOM_MSGS, TF_MSGS, CMD_MSGS, STATUS_MSGS):
        del bucket[:]
    yield


def test_reset_returns_pose_to_zero():
    SIM.x = 5.0
    SIM.y = 3.0
    SIM.theta = 1.0
    SIM.v_linear = 0.5
    SIM.v_angular = 0.2
    SIM.left_wheel_angle = 12.0
    SIM.right_wheel_angle = -8.0
    SIM.reset_callback(None, None)
    assert SIM.x == 0.0
    assert SIM.y == 0.0
    assert SIM.theta == 0.0
    assert SIM.v_linear == 0.0
    assert SIM.v_angular == 0.0
    assert SIM.left_wheel_angle == 0.0
    assert SIM.right_wheel_angle == 0.0
    assert SIM.state == RobotSim.STATE_RUNNING
    assert SIM.check_bounds() is False
    pump(SIM)
    assert any(m.pose.pose.position.x == 0.0 for m in ODOM_MSGS)


def test_reset_while_paused_stays_paused_at_origin():
    SIM.pause_callback(None, None)
    assert SIM.state == RobotSim.STATE_PAUSED
    SIM.reset_callback(None, None)
    assert SIM.state == RobotSim.STATE_PAUSED
    assert SIM.x == 0.0


def test_odom_valid_after_reset():
    SIM.x = 4.0
    SIM.y = 2.0
    SIM.reset_callback(None, None)
    pump(SIM)
    odom = next(m for m in ODOM_MSGS if m.pose.pose.position.x == 0.0)
    assert odom.header.frame_id == 'odom'
    assert odom.child_frame_id == 'base_link'


def test_tf_valid_after_reset():
    SIM.x = 4.0
    SIM.y = 2.0
    SIM.reset_callback(None, None)
    pump(SIM)
    tf = next(t for t in TF_MSGS
              if t.transforms and t.transforms[0].child_frame_id == 'base_link')
    assert tf.transforms[0].header.frame_id == 'odom'
    assert tf.transforms[0].transform.translation.x == 0.0
    assert tf.transforms[0].transform.translation.y == 0.0


def test_pause_stops_movement():
    SIM.v_linear = 0.5
    SIM.step()
    x_before = SIM.x
    SIM.pause_callback(None, None)
    assert SIM.state == RobotSim.STATE_PAUSED
    SIM.step()
    SIM.step()
    assert SIM.x == x_before
    pump(SIM)
    assert any(m.header.stamp.sec > 0 for m in ODOM_MSGS)


def test_resume_continues_movement():
    SIM.v_linear = 0.5
    SIM.pause_callback(None, None)
    SIM.resume_callback(None, None)
    assert SIM.state == RobotSim.STATE_RUNNING
    SIM.step()
    assert SIM.x > 0.0


def test_completed_state_stops_cmd_vel():
    AVOIDER.scan_callback(make_scan())
    AVOIDER.sim_status = 'COMPLETED'
    AVOIDER.control_cmd()
    pump(AVOIDER)
    assert CMD_MSGS
    for msg in CMD_MSGS:
        assert msg.linear.x == 0.0
        assert msg.angular.z == 0.0


def test_paused_state_stops_cmd_vel():
    AVOIDER.scan_callback(make_scan())
    AVOIDER.sim_status = 'PAUSED'
    AVOIDER.control_cmd()
    pump(AVOIDER)
    assert all(m.linear.x == 0.0 and m.angular.z == 0.0 for m in CMD_MSGS)


def test_max_distance_prevents_unbounded_movement():
    SIM.x = 13.0  # > max_travel_distance (12.0)
    SIM.step()
    assert SIM.state == RobotSim.STATE_COMPLETED
    x_before = SIM.x
    SIM.step()
    assert SIM.x == x_before


def test_safety_boundary_prevents_escape():
    SIM.y = 20.0  # > max_y (8.0)
    SIM.step()
    assert SIM.state == RobotSim.STATE_COMPLETED


def test_auto_drive_false_allows_external_cmd_vel():
    AVOIDER.auto_drive = False
    AVOIDER.scan_callback(make_scan())
    AVOIDER.control_cmd()
    pump(AVOIDER)
    assert not CMD_MSGS
    twist = Twist()
    twist.linear.x = 0.5
    SIM.cmd_vel_callback(twist)
    assert SIM.v_linear == 0.5
    SIM.step()
    assert SIM.x > 0.0


def test_auto_drive_true_publishes_forward_when_clear():
    AVOIDER.auto_drive = True
    AVOIDER.sim_status = 'RUNNING'
    AVOIDER.scan_callback(make_scan())
    AVOIDER.control_cmd()
    pump(AVOIDER)
    assert CMD_MSGS
    assert any(m.linear.x > 0.0 for m in CMD_MSGS)


def test_obstacle_world_unchanged_for_visualizer():
    assert len(OBSTACLES) == 5
    assert OBSTACLES[0] == (2.0, 0.0, 0.3)
    assert OBSTACLES[3] == (8.0, 0.0, 0.4)


def test_cmd_vel_ignored_when_completed():
    SIM.x = 13.0
    SIM.step()
    assert SIM.state == RobotSim.STATE_COMPLETED
    twist = Twist()
    twist.linear.x = 0.9
    SIM.cmd_vel_callback(twist)
    SIM.step()
    assert SIM.v_linear == 0.0
    assert SIM.x == 13.0


@pytest.fixture(scope='session', autouse=True)
def nodes():
    global SIM, AVOIDER
    SIM = RobotSim()
    AVOIDER = ObstacleAvoider()
    SIM.timer.cancel()
    AVOIDER.timer.cancel()
    q = QoSProfile(depth=10)
    SIM.create_subscription(Odometry, '/odom',
                            lambda m: ODOM_MSGS.append(m), q)
    SIM.create_subscription(TFMessage, '/tf',
                            lambda m: TF_MSGS.append(m), q)
    SIM.create_subscription(String, '/simulation_status',
                            lambda m: STATUS_MSGS.append(m), q)
    AVOIDER.create_subscription(Twist, '/cmd_vel',
                                lambda m: CMD_MSGS.append(m), q)
    yield
    SIM.destroy_node()
    AVOIDER.destroy_node()
    rclpy.shutdown()
