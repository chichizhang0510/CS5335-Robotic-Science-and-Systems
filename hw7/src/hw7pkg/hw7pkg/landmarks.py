"""
HW7 Task 2: find landmarks in two occupancy grids and compare them.

  python3 landmarks.py hw7_output/run1.npz hw7_output/run2.npz
  ros2 run hw7pkg landmarks hw7_output/run1.npz hw7_output/run2.npz --out hw7_output

Landmarks
---------
Occupied cells (log-odds > --occ-thresh) are turned into points, then:

- corner : sequential RANSAC fits wall lines, each line is split into
           contiguous segments, and two segments that meet at 45-135 deg
           give a corner at their intersection. Corners of walls and boxes
           are stationary and can be located to about one cell.
- post   : a small connected blob of occupied cells (chair / table leg,
           bin, narrow box) that is not part of a wall line, at its centroid.

Matching
--------
Run 2 has its own frame (the robot turned 180 deg first), so its landmarks
are transformed into run 1's frame with a rigid transform (rotation + shift):

1. RANSAC over landmark pairs: every pair (A_i, A_j) in run 1 and (B_k, B_l)
   in run 2 with the same separation defines a transform; keep the one under
   which the most run 2 landmarks land on a run 1 landmark of the same type.
   This is the hint's "align one landmark, then find the rotation", done for
   every candidate pairing.
2. Refine with least squares (Kabsch) on all matched pairs.

By default, candidate transforms more than --gate-dist / --gate-deg away from
what odometry predicts are discarded, which rejects matches that only work
because the room is repetitive. Use --no-odom-gate to rely on landmarks alone.

Outputs (in --out): landmarks_run1.png, landmarks_run2.png, overlay.png,
landmarks.csv and report.txt (also printed).
"""

import argparse
import itertools
import math
import os

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

try:                                   # installed as ROS package
    from hw7pkg.grid_core import OccupancyGrid, draw_grid, new_figure
except ImportError:                    # run directly from the source folder
    from grid_core import OccupancyGrid, draw_grid, new_figure


# ---------------------------------------------------------------------- #
# Geometry helpers
# ---------------------------------------------------------------------- #
def rot(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def apply_tf(tf, pts):
    """tf = (theta, tx, ty): p1 = R(theta) p2 + t."""
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    return pts @ rot(tf[0]).T + np.array(tf[1:])


def kabsch(src, dst):
    """Rigid transform (theta, tx, ty) minimising |R src + t - dst|^2."""
    ms, md = src.mean(0), dst.mean(0)
    h = (src - ms).T @ (dst - md)
    theta = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    t = md - rot(theta) @ ms
    return (theta, t[0], t[1])


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def odom_transform(origin1, origin2):
    """Transform from run 2's frame to run 1's frame predicted by odometry."""
    x1, y1, th1 = origin1
    x2, y2, th2 = origin2
    t = rot(-th1) @ np.array([x2 - x1, y2 - y1])
    return (wrap(th2 - th1), t[0], t[1])


# ---------------------------------------------------------------------- #
# Landmark detection
# ---------------------------------------------------------------------- #
def fit_segment(pts):
    """Total-least-squares line through pts -> (centre, direction, t_min, t_max)."""
    m = pts.mean(0)
    _, _, vt = np.linalg.svd(pts - m, full_matrices=False)
    d = vt[0]
    t = (pts - m) @ d
    return m, d, t.min(), t.max()


def longest_run(pts, idx, origin, direction, normal, thresh, gap):
    """Indices (into pts) of the longest contiguous run of points within
    `thresh` of the line, where neighbours along the line are <= gap apart."""
    q = pts[idx] - origin
    inl = idx[np.abs(q @ normal) < thresh]
    if len(inl) == 0:
        return inl
    t = (pts[inl] - origin) @ direction
    order = np.argsort(t)
    breaks = np.nonzero(np.diff(t[order]) > gap)[0] + 1
    return inl[max(np.split(order, breaks), key=len)]


def ransac_segments(pts, rng, res, thresh=0.075, min_pts=6, min_len=0.3, gap=0.2,
                    coverage=0.6, iters=200, max_lines=60, max_fails=3):
    """Sequential RANSAC: repeatedly take the line with the longest contiguous
    run of inliers, keep it as a segment if it is long and dense enough, and
    remove its points."""
    remaining = np.ones(len(pts), dtype=bool)
    segments = []
    fails = 0
    while remaining.sum() >= min_pts and len(segments) < max_lines and fails < max_fails:
        idx = np.nonzero(remaining)[0]
        q = pts[idx]
        # seed pairs from nearby points: lines through two far-apart random
        # points rarely follow a real wall
        i = rng.integers(0, len(q), iters)
        d_all = np.linalg.norm(q[None, :, :] - q[i, None, :], axis=2)
        near = (d_all > 0.1) & (d_all < 1.0)
        ok = near.any(1)
        if not ok.any():
            break
        i = i[ok]
        j = np.array([rng.choice(np.nonzero(row)[0]) for row in near[ok]])
        dvec = (q[j] - q[i]) / np.linalg.norm(q[j] - q[i], axis=1)[:, None]
        # score = size of the longest contiguous run, not all inliers on the
        # infinite line, so collinear clutter far away does not count
        best = max(range(len(i)), key=lambda k: len(longest_run(
            pts, idx, q[i[k]], dvec[k], np.array([-dvec[k, 1], dvec[k, 0]]), thresh, gap)))
        run = longest_run(pts, idx, q[i[best]], dvec[best],
                          np.array([-dvec[best, 1], dvec[best, 0]]), thresh, gap)
        if len(run) < 2:
            fails += 1
            continue
        # refit with total least squares and collect the run again
        c, dd, _, _ = fit_segment(pts[run])
        nn = np.array([-dd[1], dd[0]])
        run = longest_run(pts, idx, c, dd, nn, thresh, gap)
        c, dd, t0, t1 = fit_segment(pts[run])
        nn = np.array([-dd[1], dd[0]])
        t = (pts[run] - c) @ dd
        filled = len(np.unique(np.floor((t - t0) / res))) / max(1.0, (t1 - t0) / res)
        if len(run) < min_pts or t1 - t0 < min_len or filled < coverage:
            fails += 1
            continue
        fails = 0
        # remove a band twice as wide as the inlier band, so a thick
        # (slightly smeared) wall gives one segment, not several parallel ones
        along = (pts - c) @ dd
        band = (np.abs((pts - c) @ nn) < 2 * thresh) & \
               (along > t0 - gap) & (along < t1 + gap) & remaining
        members = np.nonzero(band)[0]
        remaining[members] = False
        segments.append({'c': c, 'd': dd, 't0': t0, 't1': t1,
                         'p0': c + t0 * dd, 'p1': c + t1 * dd, 'members': members})
    return segments


def seg_distance(seg, p):
    t = np.clip((p - seg['c']) @ seg['d'], seg['t0'], seg['t1'])
    return np.linalg.norm(seg['c'] + t * seg['d'] - p)


def find_corners(segments, tol=0.3, min_angle=45.0, merge=0.15):
    s_min = math.sin(math.radians(min_angle))
    corners = []
    for a, b in itertools.combinations(segments, 2):
        cross = a['d'][0] * b['d'][1] - a['d'][1] * b['d'][0]
        if abs(cross) < s_min:
            continue
        # solve c_a + s d_a = c_b + u d_b
        m = np.array([a['d'], -b['d']]).T
        s, _ = np.linalg.solve(m, b['c'] - a['c'])
        p = a['c'] + s * a['d']
        if seg_distance(a, p) <= tol and seg_distance(b, p) <= tol:
            corners.append(p)
    # merge corners found more than once (e.g. three segments meeting)
    merged = []
    for p in corners:
        for g in merged:
            if np.linalg.norm(np.mean(g, 0) - p) < merge:
                g.append(p)
                break
        else:
            merged.append([p])
    return [np.mean(g, 0) for g in merged]


def find_posts(grid, occ, max_size=0.45, min_cells=2):
    """Small isolated blobs of occupied cells -> (centroids, mask of their cells)."""
    # label after a one-cell dilation, so the visible faces of one object that
    # are separated by a missed cell still form one blob
    grown = ndimage.binary_dilation(occ, structure=np.ones((3, 3)))
    labels, _ = ndimage.label(grown, structure=np.ones((3, 3)))
    labels[~occ] = 0
    posts, used = [], np.zeros_like(occ)
    for k, sl in enumerate(ndimage.find_objects(labels), start=1):
        if sl is None:
            continue
        comp = labels[sl] == k
        rr, cc = np.nonzero(comp)
        h = (rr.max() - rr.min() + 1) * grid.res
        w = (cc.max() - cc.min() + 1) * grid.res
        if len(rr) < min_cells or max(h, w) > max_size:
            continue
        x, y = grid.cell_to_world(rr + sl[0].start, cc + sl[1].start)
        posts.append(np.array([x.mean(), y.mean()]))
        used[rr + sl[0].start, cc + sl[1].start] = True
    return posts, used


def detect(grid, args, rng):
    """Posts first (small blobs), then RANSAC wall segments and their corners
    on the remaining occupied cells."""
    occ = grid.logodds > args.occ_thresh
    posts, post_cells = find_posts(grid, occ, max_size=args.post_size)
    rows, cols = np.nonzero(occ & ~post_cells)
    x, y = grid.cell_to_world(rows, cols)
    pts = np.stack([x, y], 1)
    segments = ransac_segments(pts, rng, grid.res, thresh=args.line_thresh,
                               min_pts=args.min_pts, min_len=args.min_len)
    lms = [{'type': 'corner', 'xy': p} for p in find_corners(segments, tol=args.corner_tol)]
    lms += [{'type': 'post', 'xy': p} for p in posts]
    return lms, segments


# ---------------------------------------------------------------------- #
# Matching
# ---------------------------------------------------------------------- #
def pair_up(p1, t1, p2t, t2, tol):
    """Mutual nearest neighbours of the same type closer than tol -> [(i, j, d)]."""
    if len(p1) == 0 or len(p2t) == 0:
        return []
    d = np.linalg.norm(p1[:, None, :] - p2t[None, :, :], axis=2)
    d[t1[:, None] != t2[None, :]] = np.inf
    pairs = []
    for j in range(len(p2t)):
        i = int(np.argmin(d[:, j]))
        if d[i, j] < tol and int(np.argmin(d[i, :])) == j:
            pairs.append((i, j, d[i, j]))
    return pairs


def match(l1, l2, tol, prior=None, gate=None, max_hyp=40000, rng=None):
    """Return (transform, pairs, n_hypotheses) mapping run 2 coords into run 1."""
    p1 = np.array([l['xy'] for l in l1]).reshape(-1, 2)
    p2 = np.array([l['xy'] for l in l2]).reshape(-1, 2)
    t1 = np.array([l['type'] for l in l1])
    t2 = np.array([l['type'] for l in l2])

    def allowed(tf):
        if gate is None or prior is None:
            return True
        return (abs(wrap(tf[0] - prior[0])) <= math.radians(gate[1]) and
                math.hypot(tf[1] - prior[1], tf[2] - prior[2]) <= gate[0])

    pairs1 = [(i, j) for i, j in itertools.combinations(range(len(p1)), 2)
              if np.linalg.norm(p1[i] - p1[j]) > 0.4]
    pairs2 = [(k, l) for k, l in itertools.permutations(range(len(p2)), 2)]
    d1 = np.array([np.linalg.norm(p1[i] - p1[j]) for i, j in pairs1])
    d2 = np.array([np.linalg.norm(p2[k] - p2[l]) for k, l in pairs2])
    hyps = []
    for a, (i, j) in enumerate(pairs1):
        for b in np.nonzero(np.abs(d2 - d1[a]) < tol)[0]:
            k, l = pairs2[b]
            if t1[i] == t2[k] and t1[j] == t2[l]:
                hyps.append((i, j, k, l))
    if len(hyps) > max_hyp:
        hyps = [hyps[n] for n in rng.choice(len(hyps), max_hyp, replace=False)]

    best, best_key = None, (0, 0.0)
    tree = cKDTree(p1) if len(p1) else None
    t1_pad = np.append(t1, '')            # index len(p1) = "no neighbour"
    for i, j, k, l in hyps:
        tf = kabsch(p2[[k, l]], p1[[i, j]])
        if not allowed(tf):
            continue
        d, nn = tree.query(apply_tf(tf, p2), distance_upper_bound=tol)
        good = (d < tol) & (t1_pad[nn] == t2)
        key = (int(good.sum()), -float(d[good].sum()))
        if key > best_key:
            best, best_key = tf, key

    if best is None or best_key[0] < 3:
        if prior is None:
            return None, [], len(hyps)
        best = prior                      # not enough landmark support: use odometry
    tf = best
    for _ in range(10):                   # least-squares refinement on all pairs
        pr = pair_up(p1, t1, apply_tf(tf, p2), t2, tol)
        if len(pr) < 2:
            break
        new = kabsch(p2[[j for _, j, _ in pr]], p1[[i for i, _, _ in pr]])
        if np.allclose(new, tf, atol=1e-6):
            break
        tf = new
    pr = pair_up(p1, t1, apply_tf(tf, p2), t2, tol)
    return tf, pr, len(hyps)


# ---------------------------------------------------------------------- #
# Plots and report
# ---------------------------------------------------------------------- #
MARK = {'corner': 's', 'post': 'o'}


def draw_landmarks(ax, lms, prefix, matched, pts=None):
    for n, l in enumerate(lms):
        x, y = l['xy'] if pts is None else pts[n]
        color = 'tab:green' if n in matched else 'tab:red'
        ax.plot(x, y, MARK[l['type']], mfc='none', mec=color, mew=2, ms=9)
        ax.annotate(f'{prefix}{n + 1}', (x, y), xytext=(5, 5), textcoords='offset points',
                    fontsize=8, color=color, fontweight='bold')


def plot_run(path, grid, data, lms, segments, prefix, matched, title):
    fig = new_figure((9, 8))
    ax = fig.add_subplot(111)
    traj = data.get('trajectory')
    draw_grid(ax, grid, None, title, traj)
    for s in segments:
        ax.plot([s['p0'][0], s['p1'][0]], [s['p0'][1], s['p1'][1]], '-',
                color='tab:cyan', lw=1.5, alpha=0.8)
    draw_landmarks(ax, lms, prefix, matched)
    ax.plot([], [], 's', mfc='none', mec='k', label='corner')
    ax.plot([], [], 'o', mfc='none', mec='k', label='post')
    ax.plot([], [], '-', color='tab:cyan', label='RANSAC wall segment')
    ax.plot([], [], 's', color='tab:green', label='in both grids')
    ax.plot([], [], 's', color='tab:red', label='only in this grid')
    ax.legend(loc='upper right', fontsize=8, framealpha=0.85)
    fig.savefig(path, dpi=120, bbox_inches='tight')


def plot_overlay(path, g1, g2, d1, d2, l1, l2, tf, pairs, thresh):
    fig = new_figure((9, 8))
    ax = fig.add_subplot(111)
    draw_grid(ax, g1, None, 'Run 2 landmarks transformed into run 1 frame', None)
    rows, cols = np.nonzero(g2.logodds > thresh)
    occ2 = apply_tf(tf, np.stack(g2.cell_to_world(rows, cols), 1))
    ax.scatter(occ2[:, 0], occ2[:, 1], s=1.5, color='tab:red', alpha=0.35,
               label='run 2 occupied cells (transformed)')
    for traj, tfm, color, name in ((d1.get('trajectory'), None, 'tab:blue', 'run 1 path'),
                                   (d2.get('trajectory'), tf, 'tab:red', 'run 2 path')):
        if traj is not None and len(traj):
            xy = traj[:, :2] if tfm is None else apply_tf(tfm, traj[:, :2])
            ax.plot(xy[:, 0], xy[:, 1], '-', color=color, lw=1.2, label=name)
    p1 = np.array([l['xy'] for l in l1]).reshape(-1, 2)
    p2 = apply_tf(tf, [l['xy'] for l in l2]) if l2 else np.zeros((0, 2))
    for n, l in enumerate(l1):
        ax.plot(*p1[n], MARK[l['type']], mfc='none', mec='tab:blue', mew=2, ms=9)
        ax.annotate(f'A{n + 1}', p1[n], xytext=(-14, 6), textcoords='offset points',
                    fontsize=8, color='tab:blue', fontweight='bold')
    for n, l in enumerate(l2):
        ax.plot(*p2[n], 'x', color='tab:red', mew=2, ms=8)
        ax.annotate(f'B{n + 1}', p2[n], xytext=(5, -10), textcoords='offset points',
                    fontsize=8, color='tab:red', fontweight='bold')
    for i, j, _ in pairs:
        ax.plot([p1[i][0], p2[j][0]], [p1[i][1], p2[j][1]], '-', color='tab:green', lw=1.5)
    ax.plot([], [], 's', mfc='none', mec='tab:blue', label='run 1 landmark (A)')
    ax.plot([], [], 'x', color='tab:red', label='run 2 landmark (B), transformed')
    ax.plot([], [], '-', color='tab:green', label='matched pair')
    ax.legend(loc='upper right', fontsize=8, framealpha=0.85)
    fig.savefig(path, dpi=120, bbox_inches='tight')


def fmt_tf(tf):
    return f'rotation {math.degrees(tf[0]):7.2f} deg, translation ({tf[1]:6.3f}, {tf[2]:6.3f}) m'


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('run1')
    p.add_argument('run2')
    p.add_argument('--out', default=None, help='output folder (default: next to run1)')
    p.add_argument('--occ-thresh', type=float, default=1.0,
                   help='log-odds above which a cell counts as occupied (1.0 ~ p 0.73)')
    p.add_argument('--line-thresh', type=float, default=0.075, help='RANSAC inlier distance (m)')
    p.add_argument('--min-pts', type=int, default=6, help='min cells in a wall segment')
    p.add_argument('--min-len', type=float, default=0.3, help='min wall segment length (m)')
    p.add_argument('--corner-tol', type=float, default=0.3,
                   help='max gap between a segment end and the corner (m)')
    p.add_argument('--post-size', type=float, default=0.45, help='max post blob size (m)')
    p.add_argument('--match-tol', type=float, default=0.3,
                   help='max distance for two landmarks to count as the same (m)')
    p.add_argument('--gate-dist', type=float, default=1.0)
    p.add_argument('--gate-deg', type=float, default=30.0)
    p.add_argument('--no-odom-gate', action='store_true')
    p.add_argument('--seed', type=int, default=0)
    args = p.parse_args()
    out = args.out or os.path.dirname(os.path.abspath(args.run1))
    os.makedirs(out, exist_ok=True)
    rng = np.random.default_rng(args.seed)

    g1, d1 = OccupancyGrid.load(args.run1)
    g2, d2 = OccupancyGrid.load(args.run2)
    l1, s1 = detect(g1, args, rng)
    l2, s2 = detect(g2, args, rng)

    prior = None
    if 'odom_origin' in d1 and 'odom_origin' in d2:
        prior = odom_transform(d1['odom_origin'], d2['odom_origin'])
    gate = None if args.no_odom_gate else (args.gate_dist, args.gate_deg)
    tf, pairs, n_hyp = match(l1, l2, args.match_tol, prior, gate, rng=rng)
    if tf is None:
        raise SystemExit('Could not align the grids: fewer than 3 landmarks match and '
                         'the files have no odometry origin. Try lowering --occ-thresh.')

    m1 = {i for i, _, _ in pairs}
    m2 = {j for _, j, _ in pairs}
    plot_run(os.path.join(out, 'landmarks_run1.png'), g1, d1, l1, s1, 'A', m1,
             f'Run 1 landmarks ({len(l1)}: A1-A{len(l1)})')
    plot_run(os.path.join(out, 'landmarks_run2.png'), g2, d2, l2, s2, 'B', m2,
             f'Run 2 landmarks in run 2 frame ({len(l2)}: B1-B{len(l2)})')
    plot_overlay(os.path.join(out, 'overlay.png'), g1, g2, d1, d2, l1, l2, tf, pairs,
                 args.occ_thresh)

    p2t = apply_tf(tf, [l['xy'] for l in l2]) if l2 else np.zeros((0, 2))
    lines = [f'Run 1: {args.run1}  ->  {len(l1)} landmarks '
             f'({sum(l["type"] == "corner" for l in l1)} corners, '
             f'{sum(l["type"] == "post" for l in l1)} posts), {len(s1)} wall segments',
             f'Run 2: {args.run2}  ->  {len(l2)} landmarks '
             f'({sum(l["type"] == "corner" for l in l2)} corners, '
             f'{sum(l["type"] == "post" for l in l2)} posts), {len(s2)} wall segments',
             '',
             'Transform from run 2 frame to run 1 frame (p1 = R p2 + t)',
             f'  from landmarks : {fmt_tf(tf)}   ({len(pairs)} pairs, {n_hyp} hypotheses)']
    if prior is not None:
        lines.append(f'  from odometry  : {fmt_tf(prior)}')
        lines.append(f'  difference     : {math.degrees(wrap(tf[0] - prior[0])):.2f} deg, '
                     f'{math.hypot(tf[1] - prior[1], tf[2] - prior[2]):.3f} m')
    lines += ['', 'Landmarks found in both grids (run 2 positions transformed into run 1 frame)',
              f'  {"A":>4} {"B":>4} {"type":>6}   {"A x":>7} {"A y":>7}   '
              f'{"B x":>7} {"B y":>7}   {"diff (m)":>8}']
    for i, j, d in sorted(pairs):
        lines.append(f'  A{i + 1:<3} B{j + 1:<3} {l1[i]["type"]:>6}   '
                     f'{l1[i]["xy"][0]:7.3f} {l1[i]["xy"][1]:7.3f}   '
                     f'{p2t[j][0]:7.3f} {p2t[j][1]:7.3f}   {d:8.3f}')
    if pairs:
        diffs = np.array([d for _, _, d in pairs])
        lines.append(f'  Mean position difference: {diffs.mean():.3f} m '
                     f'(RMS {np.sqrt((diffs ** 2).mean()):.3f} m, max {diffs.max():.3f} m, '
                     f'n = {len(diffs)})')
        if prior is not None:
            p2o = apply_tf(prior, [l2[j]['xy'] for _, j, _ in pairs])
            od = np.linalg.norm(np.array([l1[i]['xy'] for i, _, _ in pairs]) - p2o, axis=1)
            lines.append(f'  Same pairs, aligned with odometry only: mean {od.mean():.3f} m, '
                         f'max {od.max():.3f} m')
    lines += ['', 'Found in run 1 only:']
    lines += [f'  A{i + 1:<3} {l1[i]["type"]:>6}   ({l1[i]["xy"][0]:7.3f}, {l1[i]["xy"][1]:7.3f})'
              for i in range(len(l1)) if i not in m1] or ['  (none)']
    lines += ['', 'Found in run 2 only (run 2 frame -> run 1 frame):']
    lines += [f'  B{j + 1:<3} {l2[j]["type"]:>6}   ({l2[j]["xy"][0]:7.3f}, {l2[j]["xy"][1]:7.3f})'
              f' -> ({p2t[j][0]:7.3f}, {p2t[j][1]:7.3f})'
              for j in range(len(l2)) if j not in m2] or ['  (none)']
    report = '\n'.join(lines)
    print(report)
    with open(os.path.join(out, 'report.txt'), 'w') as f:
        f.write(report + '\n')

    with open(os.path.join(out, 'landmarks.csv'), 'w') as f:
        f.write('id,grid,type,x_own,y_own,x_run1,y_run1,match\n')
        partner1 = {i: j for i, j, _ in pairs}
        partner2 = {j: i for i, j, _ in pairs}
        for i, l in enumerate(l1):
            m = f'B{partner1[i] + 1}' if i in partner1 else ''
            f.write(f'A{i + 1},run1,{l["type"]},{l["xy"][0]:.3f},{l["xy"][1]:.3f},'
                    f'{l["xy"][0]:.3f},{l["xy"][1]:.3f},{m}\n')
        for j, l in enumerate(l2):
            m = f'A{partner2[j] + 1}' if j in partner2 else ''
            f.write(f'B{j + 1},run2,{l["type"]},{l["xy"][0]:.3f},{l["xy"][1]:.3f},'
                    f'{p2t[j][0]:.3f},{p2t[j][1]:.3f},{m}\n')
    print(f'\nSaved landmarks_run1.png, landmarks_run2.png, overlay.png, '
          f'report.txt, landmarks.csv in {out}')


if __name__ == '__main__':
    main()
