import csv
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from robot_telemetry.analyze_telemetry import (  # noqa: E402
    compute_metrics, format_report, load_telemetry,
)


def write_csv(path, header, rows):
    with open(path, 'w', newline='') as fh:
        writer = csv.writer(fh)
        writer.writerow(header)
        for row in rows:
            writer.writerow(row)


def make_log_dir(tmp_path):
    """Create a telemetry log dir with synthetic CSV data."""
    log_dir = str(tmp_path / 'run1')
    os.makedirs(log_dir, exist_ok=True)

    write_csv(os.path.join(log_dir, 'odom.csv'),
              ['sec', 'nanosec', 'pos_x', 'pos_y', 'pos_z',
               'orient_w', 'lin_vel_x', 'ang_vel_z'],
              [[0, 0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0],
               [1, 0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0],
               [2, 0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.5]])

    write_csv(os.path.join(log_dir, 'scan.csv'),
              ['sec', 'nanosec', 'num_beams', 'range_min', 'range_max',
               'min_range', 'mean_range'],
              [[0, 0, 360, 0.1, 10.0, 3.00, 8.0],
               [1, 0, 360, 0.1, 10.0, 1.50, 7.0],
               [2, 0, 360, 0.1, 10.0, 0.80, 6.0]])

    write_csv(os.path.join(log_dir, 'arm_pose.csv'),
              ['sec', 'nanosec', 'x', 'y', 'z'],
              [[0, 0, 0.0, 0.0, 0.0],
               [1, 0, 0.5, 0.0, 0.0],
               [2, 0, 0.5, 0.5, 0.0]])

    write_csv(os.path.join(log_dir, 'pick_place_status.csv'),
              ['sec', 'nanosec', 'status'],
              [[0, 0, 'STATE: APPROACH_PICK  (moving above pick location)'],
               [2, 0, 'STATE: GRASP  (closing gripper)'],
               [5, 0, 'STATE: RELEASE  (opening gripper)'],
               [6, 0, 'CYCLE 1 COMPLETE: pick-and-place finished']])

    return log_dir


def test_load_telemetry_finds_csvs(tmp_path):
    log_dir = make_log_dir(tmp_path)
    data = load_telemetry(log_dir)
    assert set(data.keys()) == {'/odom', '/scan', '/arm_pose',
                                '/pick_place_status'}
    assert len(data['/odom']) == 3
    assert data['/odom'][1]['pos_x'] == 1.0


def test_motion_metrics(tmp_path):
    data = load_telemetry(make_log_dir(tmp_path))
    m = compute_metrics(data)['motion']
    assert m['distance_m'] == 2.0          # (0,0)->(1,0)->(1,1)
    assert m['avg_speed_mps'] == 1.0
    assert m['max_speed_mps'] == 1.0
    assert m['samples'] == 3


def test_scan_metrics(tmp_path):
    data = load_telemetry(make_log_dir(tmp_path))
    m = compute_metrics(data)['scan']
    assert m['min_range_m'] == 0.8
    assert m['mean_min_range_m'] == round((3.0 + 1.5 + 0.8) / 3, 3)
    assert m['near_samples'] == 1          # only 0.8 < 1.2
    assert m['samples'] == 3


def test_arm_metrics(tmp_path):
    data = load_telemetry(make_log_dir(tmp_path))
    m = compute_metrics(data)['arm']
    assert m['max_reach_m'] == round(math.sqrt(0.5), 3)  # (0.5,0.5,0)
    assert m['eef_path_m'] == 1.0          # 0.5 + 0.5
    assert m['pose_samples'] == 3


def test_cycle_metrics(tmp_path):
    data = load_telemetry(make_log_dir(tmp_path))
    m = compute_metrics(data)['cycles']
    assert m['cycles'] == 1
    assert m['cycle_times_s'] == [6.0]     # first state -> CYCLE 1 COMPLETE
    assert m['mean_cycle_s'] == 6.0
    states = m['state_durations']
    assert states['APPROACH_PICK']['count'] == 1
    assert states['APPROACH_PICK']['mean_s'] == 2.0
    assert states['GRASP']['mean_s'] == 3.0
    assert states['RELEASE']['mean_s'] == 1.0


def test_format_report_contains_sections(tmp_path):
    data = load_telemetry(make_log_dir(tmp_path))
    metrics = compute_metrics(data)
    report = format_report(metrics, str(tmp_path / 'run1'))
    assert 'TELEMETRY ANALYSIS REPORT' in report
    assert 'Total distance traveled : 2.000 m' in report
    assert 'Minimum obstacle distance : 0.800 m' in report
    assert 'Completed cycles          : 1' in report
    assert 'APPROACH_PICK' in report
