import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from builtin_interfaces.msg import Time  # noqa: E402
from obstacle_visualizer import (  # noqa: E402
    build_marker_array, MARKER_NAMESPACE, OBSTACLE_HEIGHT,
)
from obstacle_world import get_obstacles, OBSTACLES  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402


def make_stamp(sec, nanosec):
    stamp = Time()
    stamp.sec = sec
    stamp.nanosec = nanosec
    return stamp


def test_shared_obstacles_match_simulator_world():
    assert len(OBSTACLES) == 5
    assert OBSTACLES[0] == (2.0, 0.0, 0.3)
    assert OBSTACLES[4] == (10.0, -0.5, 0.3)
    assert get_obstacles() == OBSTACLES


def test_get_obstacles_returns_fresh_copy():
    first = get_obstacles()
    first.append((99.0, 99.0, 9.9))
    assert get_obstacles() == OBSTACLES


def test_marker_count_matches_obstacles():
    markers = build_marker_array(OBSTACLES, make_stamp(1, 2))
    assert len(markers.markers) == len(OBSTACLES) == 5


def test_marker_ids_are_stable_and_sequential():
    markers = build_marker_array(OBSTACLES, make_stamp(1, 2))
    assert [m.id for m in markers.markers] == [0, 1, 2, 3, 4]


def test_marker_namespace():
    markers = build_marker_array(OBSTACLES, make_stamp(1, 2))
    assert all(m.ns == MARKER_NAMESPACE == 'simulated_obstacles'
               for m in markers.markers)


def test_marker_frame_id_is_odom():
    markers = build_marker_array(OBSTACLES, make_stamp(1, 2))
    assert all(m.header.frame_id == 'odom' for m in markers.markers)


def test_marker_positions_match_obstacle_definitions():
    markers = build_marker_array(OBSTACLES, make_stamp(1, 2))
    for marker, (x, y, radius) in zip(markers.markers, OBSTACLES):
        assert marker.pose.position.x == x
        assert marker.pose.position.y == y


def test_marker_diameter_is_twice_radius():
    markers = build_marker_array(OBSTACLES, make_stamp(1, 2))
    for marker, (x, y, radius) in zip(markers.markers, OBSTACLES):
        assert marker.scale.x == 2.0 * radius
        assert marker.scale.y == 2.0 * radius
        assert marker.scale.x == marker.scale.y


def test_marker_height():
    markers = build_marker_array(OBSTACLES, make_stamp(1, 2))
    assert all(m.scale.z == OBSTACLE_HEIGHT == 0.5
               for m in markers.markers)
    assert all(m.pose.position.z == OBSTACLE_HEIGHT / 2.0
               for m in markers.markers)


def test_marker_type_is_cylinder():
    markers = build_marker_array(OBSTACLES, make_stamp(1, 2))
    assert all(m.type == Marker.CYLINDER for m in markers.markers)


def test_marker_action_is_add():
    markers = build_marker_array(OBSTACLES, make_stamp(1, 2))
    assert all(m.action == Marker.ADD for m in markers.markers)


def test_marker_orientation_is_identity_upright():
    markers = build_marker_array(OBSTACLES, make_stamp(1, 2))
    for m in markers.markers:
        assert m.pose.orientation.x == 0.0
        assert m.pose.orientation.y == 0.0
        assert m.pose.orientation.z == 0.0
        assert m.pose.orientation.w == 1.0


def test_marker_lifetime_is_persistent():
    markers = build_marker_array(OBSTACLES, make_stamp(1, 2))
    for m in markers.markers:
        assert m.lifetime.sec == 0
        assert m.lifetime.nanosec == 0


def test_timestamp_is_forwarded():
    stamp = make_stamp(123, 456)
    markers = build_marker_array(OBSTACLES, stamp)
    for m in markers.markers:
        assert m.header.stamp.sec == 123
        assert m.header.stamp.nanosec == 456


def test_different_timestamps_flow_through():
    first = build_marker_array(OBSTACLES, make_stamp(1, 1))
    second = build_marker_array(OBSTACLES, make_stamp(2, 2))
    assert first.markers[0].header.stamp.sec == 1
    assert second.markers[0].header.stamp.sec == 2
