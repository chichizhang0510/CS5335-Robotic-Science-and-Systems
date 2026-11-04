"""
HW9: pick a cone out of a LaserScan.

The camera says roughly which way a cone is; the lidar says exactly how far
away it is and refines the direction. Angles here are in the robot frame:
0 = straight ahead, + = to the left (counter-clockwise), like /odom yaw.

The TurtleBot 4 lidar is mounted rotated +90 deg about z (see hw6 / hw7),
so robot angle = scan angle + laser_yaw_offset.
"""

import math
import statistics
from dataclasses import dataclass

RANGE_JUMP = 0.10          # m: a bigger jump between neighbouring rays starts a new object
EXTRA_WINDOW = math.radians(20)   # look this far past the window so edge objects are not cut


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class LidarObject:
    bearing: float    # rad, robot frame, centre of the object's visible face
    range: float      # m from the lidar to the nearest point of the object
    width: float      # m, approximate visible width
    n: int            # number of rays that hit it


def scan_points(scan, yaw_offset, min_valid):
    """[(robot-frame angle, range)] for the valid rays of a LaserScan.

    Rays shorter than min_valid are dropped: they hit the robot's own
    standoffs / shell, not the world.
    """
    lo = max(scan.range_min, min_valid)
    pts = []
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < lo or r > scan.range_max:
            continue
        pts.append((wrap_angle(scan.angle_min + i * scan.angle_increment + yaw_offset), r))
    return pts


def angle_step(scan):
    return abs(scan.angle_increment) or math.radians(1.0)


def find_object(points, bearing, half_window, max_range, max_width, step, min_points=3):
    """Nearest small object whose centre is within half_window of `bearing`.

    Rays are grouped into objects wherever neighbouring ranges jump by more
    than RANGE_JUMP or rays are missing. Objects wider than max_width (walls)
    are ignored. Inside the cone triangle there is no clutter, so the
    nearest small object in the direction the camera saw a colour is the cone.
    """
    reach = half_window + EXTRA_WINDOW
    near = sorted((wrap_angle(a - bearing), r) for a, r in points
                  if r <= max_range and abs(wrap_angle(a - bearing)) <= reach)
    clusters, current = [], []
    for rel, r in near:
        if current and (abs(r - current[-1][1]) > RANGE_JUMP or rel - current[-1][0] > 3 * step):
            clusters.append(current)
            current = []
        current.append((rel, r))
    if current:
        clusters.append(current)

    best = None
    for cl in clusters:
        if len(cl) < min_points:
            continue
        rels = [p[0] for p in cl]
        r_near = min(p[1] for p in cl)
        width = r_near * (rels[-1] - rels[0] + step)
        centre = statistics.mean(rels)
        if width > max_width or abs(centre) > half_window:
            continue
        obj = LidarObject(wrap_angle(bearing + centre), r_near, width, len(cl))
        if best is None or obj.range < best.range:
            best = obj
    return best


def nearest_in_sector(points, half_angle, centre=0.0):
    """Smallest range among rays within half_angle of `centre`, or None."""
    rs = [r for a, r in points if abs(wrap_angle(a - centre)) <= half_angle]
    return min(rs) if rs else None
