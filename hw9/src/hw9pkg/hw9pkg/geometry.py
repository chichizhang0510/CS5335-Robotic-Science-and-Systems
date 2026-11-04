"""Angle helpers and lidar clustering for HW9 (NumPy only, no ROS)."""

import math
from dataclasses import dataclass

import numpy as np


def wrap(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def wrap_array(a):
    return np.arctan2(np.sin(a), np.cos(a))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def circular_mean(angles):
    return math.atan2(sum(math.sin(a) for a in angles), sum(math.cos(a) for a in angles))


def scan_to_polar(ranges, angle_min, angle_increment, range_min, range_max,
                  yaw_offset, min_valid=0.0):
    """
    LaserScan -> (bearings, ranges) of the valid returns.

    Bearings are in the robot's orientation (0 = straight ahead, + = left),
    measured from the lidar: `yaw_offset` is the lidar's yaw on the robot
    (+pi/2 on the TurtleBot 4, see hw6). Returns closer than `min_valid`
    (the robot's own frame) and inf/NaN are dropped.
    """
    r = np.asarray(ranges, dtype=float)
    ang = angle_min + angle_increment * np.arange(r.size) + yaw_offset
    ok = np.isfinite(r) & (r >= max(range_min, min_valid)) & (r <= range_max)
    return wrap_array(ang[ok]), r[ok]


@dataclass
class Cluster:
    bearing: float     # mean bearing of the cluster's points (rad)
    range: float       # range of its nearest points (m)
    count: int         # number of points
    compact: bool      # small isolated object (cone), not part of a wall


def find_cluster(bearings, ranges, target, window, expected=None, tol=None,
                 max_range=None, radius=0.12):
    """
    The nearest object within `window` rad of bearing `target`.

    Optionally only returns within `tol` of an `expected` range, or closer
    than `max_range`, are considered. The cluster is every return within
    `radius` m of the nearest one. It is `compact` if there are (almost) no
    further returns between `radius` and 2*radius from it, which is true for a
    cone and false for a wall.
    """
    if bearings.size == 0:
        return None
    d = np.abs(wrap_array(bearings - target))
    sel = d <= window
    if max_range is not None:
        sel &= ranges <= max_range
    if expected is not None and tol is not None:
        sel &= np.abs(ranges - expected) <= tol
    idx = np.flatnonzero(sel)
    if idx.size == 0:
        return None
    k = idx[np.argmin(ranges[idx])]
    px, py = ranges * np.cos(bearings), ranges * np.sin(bearings)
    dist = np.hypot(px - px[k], py - py[k])
    local = d <= 2 * window + 0.2
    near = local & (dist <= radius)
    ring = int((local & (dist > radius) & (dist <= 2 * radius)).sum())
    n = int(near.sum())
    b = math.atan2(np.sin(bearings[near]).mean(), np.cos(bearings[near]).mean())
    # Range = median of the returns from the middle of the cluster (facing the
    # lidar). Picking the smallest ranges instead would be biased short by noise.
    off = np.abs(wrap_array(bearings[near] - b))
    mid = off <= max(0.35 * off.max(), math.radians(0.6))
    rng = float(np.median(ranges[near][mid]))
    return Cluster(b, rng, n, ring <= max(1, n // 4))


def group_points(points, radius):
    """Greedy grouping of (x, y) points; returns lists of indices."""
    groups = []          # [sum_x, sum_y, [indices]]
    for i, (x, y) in enumerate(points):
        for g in groups:
            n = len(g[2])
            if math.hypot(x - g[0] / n, y - g[1] / n) <= radius:
                g[0] += x
                g[1] += y
                g[2].append(i)
                break
        else:
            groups.append([x, y, [i]])
    return [g[2] for g in groups]


def group_angles(angles, tol):
    """Greedy grouping of angles on the circle; returns lists of indices."""
    groups = []          # [sum_sin, sum_cos, [indices]]
    for i, a in enumerate(angles):
        for g in groups:
            if abs(wrap(a - math.atan2(g[0], g[1]))) <= tol:
                g[0] += math.sin(a)
                g[1] += math.cos(a)
                g[2].append(i)
                break
        else:
            groups.append([math.sin(a), math.cos(a), [i]])
    return [g[2] for g in groups]
