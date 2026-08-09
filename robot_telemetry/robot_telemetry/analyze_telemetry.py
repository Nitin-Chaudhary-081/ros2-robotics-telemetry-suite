#!/usr/bin/env python3
"""Offline telemetry analysis.

Reads the structured CSV logs written by telemetry_recorder (one run
directory), computes operational performance metrics, and prints a
formatted analytical summary report.

Metrics computed:
  - Coverage: message counts + time span per topic
  - Motion:   total distance traveled, average / max speed (from /odom)
  - Obstacles: minimum obstacle distance, proximity alerts (from /scan)
  - Arm:      max end-effector reach, end-effector path length,
              per-joint motion range (from /arm_pose, /joint_states)
  - Pick-and-place: completed cycles, mean cycle time, per-state
              durations (from /pick_place_status)

Usage:
    analyze_telemetry <telemetry_log_dir> [--format text|json]

For ros2 bag outputs: play the bag while a recorder runs, then analyze
the recorder's log dir, e.g.:
    ros2 bag play ~/ros2_ws/logs/bags/my_bag &
    ros2 run robot_telemetry telemetry_recorder --ros-args -p log_dir:=/tmp/live
    # ... stop both, then:
    analyze_telemetry /tmp/live
"""

import argparse
import csv
import json
import math
import os
import re
import sys
from datetime import datetime

CSV_FIELDS = {
    'cmd_vel': ['sec', 'nanosec', 'linear_x', 'linear_y', 'angular_z'],
    'odom': ['sec', 'nanosec', 'pos_x', 'pos_y', 'pos_z',
             'orient_w', 'lin_vel_x', 'ang_vel_z'],
    'scan': ['sec', 'nanosec', 'num_beams', 'range_min', 'range_max',
             'min_range', 'mean_range'],
    'joint_states': ['sec', 'nanosec', 'left_wheel_joint',
                     'right_wheel_joint'],
    'arm_joint_states': ['sec', 'nanosec', 'waist', 'shoulder', 'elbow',
                         'gripper_left', 'gripper_right'],
    'arm_pose': ['sec', 'nanosec', 'x', 'y', 'z'],
    'pick_place_status': ['sec', 'nanosec', 'status'],
}

STATE_RE = re.compile(r'^STATE:\s*([A-Z_]+)')
CYCLE_RE = re.compile(r'^CYCLE\s+(\d+)\s+COMPLETE')

NEAR_OBSTACLE_M = 1.2  # matches obstacle_avoider safety distance


# ----------------------------------------------------------------------
# Loading
# ----------------------------------------------------------------------
def _to_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0


def load_csv(path):
    """Return list of row dicts; numeric columns converted to float."""
    rows = []
    with open(path, newline='') as fh:
        reader = csv.DictReader(fh)
        fieldnames = reader.fieldnames or []
        for raw in reader:
            row = {}
            for name in fieldnames:
                row[name] = _to_float(raw[name]) if name != 'status' \
                    else raw[name]
            rows.append(row)
    return rows


def load_telemetry(log_dir):
    """Return {topic: rows} for every CSV present in log_dir."""
    data = {}
    for name, fields in CSV_FIELDS.items():
        path = os.path.join(log_dir, name + '.csv')
        if os.path.isfile(path):
            data['/' + name] = load_csv(path)
    return data


def _time(row):
    return row.get('sec', 0) + row.get('nanosec', 0) * 1e-9


# ----------------------------------------------------------------------
# Metrics
# ----------------------------------------------------------------------
def _motion_metrics(rows):
    """Total distance, average/max speed from odometry samples."""
    m = {'distance_m': 0.0, 'avg_speed_mps': 0.0, 'max_speed_mps': 0.0,
         'avg_ang_vel_radps': 0.0, 'samples': len(rows)}
    if len(rows) < 2:
        return m
    dist = 0.0
    speeds = []
    angs = []
    for prev, cur in zip(rows, rows[1:]):
        dist += math.hypot(cur['pos_x'] - prev['pos_x'],
                           cur['pos_y'] - prev['pos_y'])
        speeds.append(cur['lin_vel_x'])
        angs.append(abs(cur['ang_vel_z']))
    m['distance_m'] = round(dist, 3)
    m['avg_speed_mps'] = round(sum(speeds) / len(speeds), 3)
    m['max_speed_mps'] = round(max(speeds), 3)
    m['avg_ang_vel_radps'] = round(sum(angs) / len(angs), 3)
    return m


def _scan_metrics(rows):
    """Minimum obstacle distance and proximity alerts."""
    m = {'min_range_m': None, 'mean_min_range_m': 0.0,
         'near_samples': 0, 'samples': len(rows)}
    if not rows:
        return m
    mins = [r['min_range'] for r in rows]
    m['min_range_m'] = min(mins)
    m['mean_min_range_m'] = round(sum(mins) / len(mins), 3)
    m['near_samples'] = sum(1 for r in rows
                            if r['min_range'] < NEAR_OBSTACLE_M)
    return m


def _arm_metrics(pose_rows, joint_rows):
    """End-effector reach / path length and per-joint motion ranges."""
    m = {'max_reach_m': 0.0, 'eef_path_m': 0.0,
         'pose_samples': len(pose_rows), 'joint_samples': len(joint_rows)}
    if pose_rows:
        m['max_reach_m'] = round(
            max(math.hypot(math.hypot(r['x'], r['y']), r['z'])
                for r in pose_rows), 3)
        path = 0.0
        for prev, cur in zip(pose_rows, pose_rows[1:]):
            path += math.hypot(cur['x'] - prev['x'], cur['y'] - prev['y'])
        m['eef_path_m'] = round(path, 3)
    if joint_rows:
        for joint in ('waist', 'shoulder', 'elbow'):
            vals = [r[joint] for r in joint_rows]
            m[joint + '_range_rad'] = round(max(vals) - min(vals), 3)
    return m


def _wheel_metrics(rows):
    """Drive wheel joint travel (from sim /joint_states)."""
    m = {'samples': len(rows),
         'left_range_rad': 0.0, 'right_range_rad': 0.0}
    if rows:
        left = [r['left_wheel_joint'] for r in rows]
        right = [r['right_wheel_joint'] for r in rows]
        m['left_range_rad'] = round(max(left) - min(left), 3)
        m['right_range_rad'] = round(max(right) - min(right), 3)
    return m


def _cycle_metrics(status_rows):
    """Pick-and-place state durations and cycle times."""
    m = {'cycles': 0, 'state_durations': {}, 'cycle_times_s': []}
    events = []
    for row in status_rows:
        text = str(row['status']).strip()
        st = STATE_RE.match(text)
        cy = CYCLE_RE.match(text)
        if st:
            events.append((_time(row), 'state', st.group(1)))
        elif cy:
            events.append((_time(row), 'cycle', int(cy.group(1))))
    if not events:
        return m

    events.sort(key=lambda e: e[0])
    durations = {}
    for i, (t, kind, name) in enumerate(events):
        end = events[i + 1][0] if i + 1 < len(events) else t
        if kind == 'state':
            durations.setdefault(name, []).append(end - t)

    m['state_durations'] = {
        name: {'count': len(vals),
               'mean_s': round(sum(vals) / len(vals), 2),
               'min_s': round(min(vals), 2),
               'max_s': round(max(vals), 2)}
        for name, vals in durations.items()
    }

    cycle_times = [e[0] for e in events if e[1] == 'cycle']
    m['cycles'] = len(cycle_times)
    if cycle_times:
        start = events[0][0]
        first = cycle_times[0] - start
        rest = [b - a for a, b in zip(cycle_times, cycle_times[1:])]
        m['cycle_times_s'] = [round(first, 2)] + [round(c, 2) for c in rest]
        total = first + sum(rest)
        m['mean_cycle_s'] = round(total / len(cycle_times), 2)
    return m


def compute_metrics(data):
    """Full metric set for a {topic: rows} dict."""
    metrics = {'coverage': {}, 'span_s': 0.0}
    for topic, rows in data.items():
        metrics['coverage'][topic] = len(rows)
        if rows:
            metrics['span_s'] = max(metrics['span_s'],
                                    _time(rows[-1]) - _time(rows[0]))
    metrics['span_s'] = round(metrics['span_s'], 2)

    if '/odom' in data:
        metrics['motion'] = _motion_metrics(data['/odom'])
    if '/scan' in data:
        metrics['scan'] = _scan_metrics(data['/scan'])
    if '/arm_pose' in data or '/arm_joint_states' in data:
        metrics['arm'] = _arm_metrics(
            data.get('/arm_pose', []),
            data.get('/arm_joint_states', data.get('/joint_states', [])))
    if '/joint_states' in data:
        metrics['wheel'] = _wheel_metrics(data['/joint_states'])
    if '/pick_place_status' in data:
        metrics['cycles'] = _cycle_metrics(data['/pick_place_status'])
    return metrics


# ----------------------------------------------------------------------
# Reporting
# ----------------------------------------------------------------------
def format_report(metrics, source_dir):
    lines = []
    add = lines.append
    add('=' * 62)
    add('           TELEMETRY ANALYSIS REPORT')
    add('=' * 62)
    add('Source directory : %s' % source_dir)
    add('Generated at     : %s' % datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
    add('')

    add('[ 1. Data coverage ]')
    add('  %-22s %10s' % ('topic', 'messages'))
    for topic, count in sorted(metrics['coverage'].items()):
        add('  %-22s %10d' % (topic, count))
    add('  Recorded span   : %.1f s' % metrics['span_s'])
    add('')

    add('[ 2. Robot motion (from /odom) ]')
    if 'motion' in metrics:
        mo = metrics['motion']
        add('  Total distance traveled : %.3f m' % mo['distance_m'])
        add('  Average speed           : %.3f m/s' % mo['avg_speed_mps'])
        add('  Max speed               : %.3f m/s' % mo['max_speed_mps'])
        add('  Avg |angular velocity|  : %.3f rad/s'
            % mo['avg_ang_vel_radps'])
        add('  Odometry samples        : %d' % mo['samples'])
    else:
        add('  (no /odom data)')
    add('')

    add('[ 3. Obstacle proximity (from /scan) ]')
    if 'scan' in metrics:
        sc = metrics['scan']
        add('  Minimum obstacle distance : %.3f m' % sc['min_range_m'])
        add('  Mean min-range per scan   : %.3f m' % sc['mean_min_range_m'])
        add('  Scans with obstacle < %.1f m: %d'
            % (NEAR_OBSTACLE_M, sc['near_samples']))
        add('  Scan samples              : %d' % sc['samples'])
    else:
        add('  (no /scan data)')
    add('')

    add('[ 4. Arm motion (from /arm_pose, /arm_joint_states) ]')
    if 'arm' in metrics:
        ar = metrics['arm']
        add('  Max end-effector reach    : %.3f m' % ar['max_reach_m'])
        add('  End-effector path length  : %.3f m' % ar['eef_path_m'])
        if 'waist_range_rad' in ar:
            add('  Waist joint range         : %.3f rad'
                % ar['waist_range_rad'])
            add('  Shoulder joint range      : %.3f rad'
                % ar['shoulder_range_rad'])
            add('  Elbow joint range         : %.3f rad'
                % ar['elbow_range_rad'])
    else:
        add('  (no arm data)')
    if 'wheel' in metrics:
        wh = metrics['wheel']
        add('  Wheel joint travel (L/R)  : %.3f / %.3f rad'
            % (wh['left_range_rad'], wh['right_range_rad']))
        add('  Wheel samples             : %d' % wh['samples'])
    add('')

    add('[ 5. Pick-and-place cycles (from /pick_place_status) ]')
    if 'cycles' in metrics:
        cy = metrics['cycles']
        add('  Completed cycles          : %d' % cy['cycles'])
        if cy['cycle_times_s']:
            add('  Cycle times (s)           : %s'
                % ', '.join('%.2f' % c for c in cy['cycle_times_s']))
            add('  Mean cycle duration       : %.2f s' % cy['mean_cycle_s'])
        if cy['state_durations']:
            add('  State durations (s):')
            add('    %-18s %6s %8s %8s %8s'
                % ('state', 'count', 'mean', 'min', 'max'))
            for name, st in sorted(cy['state_durations'].items()):
                add('    %-18s %6d %8.2f %8.2f %8.2f'
                    % (name, st['count'], st['mean_s'],
                       st['min_s'], st['max_s']))
    else:
        add('  (no pick-and-place status data)')
    add('')
    add('=' * 62)
    add('Analysis complete.')
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Analyze telemetry CSV logs recorded by '
                    'telemetry_recorder')
    parser.add_argument('log_dir', help='directory with telemetry CSVs')
    parser.add_argument('--format', choices=['text', 'json'], default='text',
                        help='output format (default: text)')
    args = parser.parse_args(argv)

    if not os.path.isdir(args.log_dir):
        print('ERROR: not a directory: %s' % args.log_dir, file=sys.stderr)
        return 1

    data = load_telemetry(args.log_dir)
    if not data:
        print('ERROR: no telemetry CSV files found in %s' % args.log_dir,
              file=sys.stderr)
        return 1

    metrics = compute_metrics(data)
    if args.format == 'json':
        print(json.dumps(metrics, indent=2))
    else:
        print(format_report(metrics, args.log_dir))
    return 0


if __name__ == '__main__':
    sys.exit(main())
