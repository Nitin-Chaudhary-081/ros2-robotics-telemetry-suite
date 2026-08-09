#!/usr/bin/env python3
"""
Single source of truth for the simulated obstacle world.

Both the physics sim (pybullet_sim.py) and the visualization node
(obstacle_visualizer.py) read obstacle definitions from this module, so the
3D markers always match exactly what the LiDAR/collision logic uses.

Each obstacle is a tuple (x, y, radius) in the odom/world frame.
"""

OBSTACLES = [
    (2.0, 0.0, 0.3),   # directly ahead at x=2
    (4.0, 0.5, 0.4),
    (6.0, -0.3, 0.3),
    (8.0, 0.0, 0.4),   # directly ahead at x=8
    (10.0, -0.5, 0.3),
]


def get_obstacles():
    """Return a fresh copy of the obstacle list [(x, y, radius), ...]."""
    return list(OBSTACLES)
