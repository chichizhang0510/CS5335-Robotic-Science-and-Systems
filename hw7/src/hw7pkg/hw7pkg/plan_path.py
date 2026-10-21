"""
HW7 Task 3: A* path planning on an occupancy grid, accounting for robot width.

  python3 plan_path.py hw7_output/task3.npz --goal 2.5 0.0
  ros2 run hw7pkg plan_path hw7_output/task3.npz --goal 2.5 0.0 --out hw7_output

Coordinates are metres in the grid's frame: the robot started at (0, 0)
facing +x. Any .npz written by mapping_node (or simulate) works.

How the robot's width is handled
--------------------------------
The robot is treated as a disc of radius --robot-radius (TurtleBot 4:
0.171 m) plus --margin. Every occupied cell is grown by that radius
(configuration space), so planning a path for the robot's centre point
through the remaining free cells keeps the whole body clear. Unknown cells
are not entered unless --unknown free is given.

Search
------
8-connected A* with an octile-distance heuristic. Diagonal moves may not cut
corners of blocked cells. Each step also costs a little extra near obstacles
(--clearance-weight), so the path keeps some distance instead of grazing the
inflated boundary. The cell path is then shortened to a few waypoints by
line-of-sight checks, and printed as `turn` / `move` commands for the hw4
executive node (day4.launch.py).

Output: <out>/astar_path.png, astar_path.txt (report) and astar_commands.txt
(the commands alone, to replay with `ros2 run day4pkg executive_node < astar_commands.txt`).
"""

import argparse
import heapq
import math
import os

import numpy as np
from scipy import ndimage

try:                                   # installed as ROS package
    from hw7pkg.grid_core import OccupancyGrid, draw_grid, new_figure
except ImportError:                    # run directly from the source folder
    from grid_core import OccupancyGrid, draw_grid, new_figure


SQRT2 = math.sqrt(2.0)
NEIGHBOURS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
              (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]


def build_cspace(grid, args, start):
    """Return (blocked, clearance_m, occ, unknown) boolean / float arrays."""
    lo = grid.logodds
    occ = lo > args.occ_thresh
    free = lo < args.free_thresh
    # small unknown holes between diverging lidar beams are not real unknowns
    free = ndimage.binary_closing(free, structure=np.ones((3, 3))) & ~occ
    unknown = ~occ & ~free

    clearance = ndimage.distance_transform_edt(~occ) * grid.res   # m to nearest obstacle
    radius = args.robot_radius + args.margin
    blocked = clearance < radius
    if args.unknown == 'blocked':
        blocked |= unknown
    # the lidar cannot see the floor under the robot: unknown cells under the
    # robot at the start are free as long as no obstacle is there
    r, c = grid.world_to_cell(*start)
    rr, cc = np.ogrid[:grid.size, :grid.size]
    disc = (rr - r) ** 2 + (cc - c) ** 2 <= (args.robot_radius / grid.res) ** 2
    blocked[disc & unknown & (clearance >= radius)] = False
    return blocked, clearance, occ, unknown


def astar(blocked, clearance, start, goal, res, inflate, weight, falloff):
    """8-connected A* on cells. Returns the list of (row, col) or None."""
    rows, cols = blocked.shape
    # extra cost per metre near obstacles: weight at the inflated boundary,
    # fading to 0 at `falloff` metres beyond it
    near = np.clip(1.0 - (clearance - inflate) / falloff, 0.0, 1.0) * weight

    def h(cell):
        dr, dc = abs(cell[0] - goal[0]), abs(cell[1] - goal[1])
        return res * ((dr + dc) + (SQRT2 - 2.0) * min(dr, dc))

    g = {start: 0.0}
    parent = {start: None}
    heap = [(h(start), 0.0, start)]
    closed = set()
    while heap:
        _, gc, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        if cur == goal:
            path = []
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            return path[::-1]
        closed.add(cur)
        r, c = cur
        for dr, dc, step in NEIGHBOURS:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols) or blocked[nr, nc]:
                continue
            if dr and dc and (blocked[r + dr, c] or blocked[r, c + dc]):
                continue                  # no cutting past a blocked corner
            ng = gc + step * res * (1.0 + near[nr, nc])
            if ng < g.get((nr, nc), math.inf):
                g[(nr, nc)] = ng
                parent[(nr, nc)] = cur
                heapq.heappush(heap, (ng + h((nr, nc)), ng, (nr, nc)))
    return None


def line_free(blocked, a, b):
    """True if every cell on the straight line from cell a to cell b is free."""
    return not any(blocked[r, c] for r, c in OccupancyGrid.bresenham(*a, *b))


def shortcut(blocked, path):
    """Greedy line-of-sight smoothing: keep only the cells where the path must turn."""
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_free(blocked, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def commands(waypoints, yaw0):
    """turn/move commands for the hw4 executive (turn = CCW degrees in [0, 360))."""
    cmds, yaw = [], yaw0
    for (x0, y0), (x1, y1) in zip(waypoints[:-1], waypoints[1:]):
        heading = math.atan2(y1 - y0, x1 - x0)
        turn = math.degrees(math.atan2(math.sin(heading - yaw), math.cos(heading - yaw)))
        if abs(turn) >= 0.5:
            cmds.append(f'turn {turn % 360.0:.1f}')
        cmds.append(f'move {math.hypot(x1 - x0, y1 - y0):.2f}')
        yaw = heading
    return cmds


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('grid', help='.npz from mapping_node or simulate')
    p.add_argument('--goal', type=float, nargs=2, required=True, metavar=('X', 'Y'))
    p.add_argument('--start', type=float, nargs=2, default=(0.0, 0.0), metavar=('X', 'Y'))
    p.add_argument('--start-yaw', type=float, default=0.0,
                   help='robot heading at the start (deg, 0 = +x)')
    p.add_argument('--robot-radius', type=float, default=0.171,
                   help='TurtleBot 4 radius (m); its diameter is 0.342 m')
    p.add_argument('--margin', type=float, default=0.05, help='extra safety distance (m)')
    p.add_argument('--occ-thresh', type=float, default=0.4,
                   help='log-odds above which a cell is an obstacle (0.4 ~ p 0.6)')
    p.add_argument('--free-thresh', type=float, default=-0.2,
                   help='log-odds below which a cell is known free')
    p.add_argument('--unknown', choices=('blocked', 'free'), default='blocked')
    p.add_argument('--clearance-weight', type=float, default=1.0,
                   help='extra cost near obstacles (0 = pure shortest path)')
    p.add_argument('--clearance-falloff', type=float, default=0.3,
                   help='distance beyond the inflated boundary where the extra cost ends (m)')
    p.add_argument('--out', default=None, help='output folder (default: next to the grid)')
    args = p.parse_args()
    out = args.out or os.path.dirname(os.path.abspath(args.grid))
    os.makedirs(out, exist_ok=True)

    grid, data = OccupancyGrid.load(args.grid)
    start, goal = tuple(args.start), tuple(args.goal)
    sep = math.hypot(goal[0] - start[0], goal[1] - start[1])
    if sep < 2.0:
        print(f'WARNING: goal is only {sep:.2f} m from the start; the task asks for >= 2 m.')

    blocked, clearance, occ, unknown = build_cspace(grid, args, start)
    s_cell, g_cell = grid.world_to_cell(*start), grid.world_to_cell(*goal)
    for name, cell in (('start', s_cell), ('goal', g_cell)):
        if not grid.in_bounds(*cell):
            raise SystemExit(f'{name} is outside the grid.')
        if blocked[cell]:
            why = 'in unknown space (behind an obstacle? try --unknown free)' \
                  if unknown[cell] else \
                  f'{clearance[cell]:.2f} m from an obstacle (needs ' \
                  f'{args.robot_radius + args.margin:.2f} m)'
            raise SystemExit(f'{name} {start if name == "start" else goal} is not reachable: '
                             f'it is {why}.')

    inflate = args.robot_radius + args.margin
    path = astar(blocked, clearance, s_cell, g_cell, grid.res, inflate,
                 args.clearance_weight, args.clearance_falloff)
    if path is None:
        raise SystemExit('A* found no path. If the goal is behind an obstacle seen from '
                         'one spot only, its shadow is unknown space: try --unknown free, '
                         'a smaller --margin, or another goal.')

    rows = np.array([c[0] for c in path])
    cols = np.array([c[1] for c in path])
    px, py = grid.cell_to_world(rows, cols)
    length = float(np.sum(np.hypot(np.diff(px), np.diff(py))))
    way_cells = shortcut(blocked, path)
    wx, wy = grid.cell_to_world(np.array([c[0] for c in way_cells]),
                                np.array([c[1] for c in way_cells]))
    way = [start] + list(zip(wx[1:-1], wy[1:-1])) + [goal]
    way_len = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(way[:-1], way[1:]))

    straight_free = line_free(blocked, s_cell, g_cell)
    d = np.array([goal[0] - start[0], goal[1] - start[1]]) / sep
    deviation = np.abs((px - start[0]) * d[1] - (py - start[1]) * d[0])
    min_clear = float(clearance[rows, cols].min())

    # ---------------------------------------------------------------- plot
    fig = new_figure((9, 8))
    ax = fig.add_subplot(111)
    draw_grid(ax, grid, None, None, None)
    ext = grid.extent()
    overlay = np.zeros((*blocked.shape, 4))
    infl = blocked & ~occ & ~unknown
    overlay[infl] = (1.0, 0.3, 0.3, 0.35)              # inflated obstacles
    overlay[occ] = (0.6, 0.0, 0.0, 0.9)                # obstacles
    ax.imshow(overlay, origin='lower', extent=ext, interpolation='nearest')
    ax.plot([start[0], goal[0]], [start[1], goal[1]], '--', color='0.4', lw=1,
            label='straight line (' + ('clear' if straight_free else 'blocked') + ')')
    ax.plot(px, py, '-', color='tab:blue', lw=1, alpha=0.7, label=f'A* cells ({length:.2f} m)')
    wa = np.array(way)
    ax.plot(wa[:, 0], wa[:, 1], 'o-', color='tab:orange', lw=2, ms=4,
            label=f'waypoints ({way_len:.2f} m)')
    from matplotlib.patches import Circle
    for k in range(0, len(px), max(1, len(px) // 12)):     # robot footprint along the path
        ax.add_patch(Circle((px[k], py[k]), args.robot_radius, fill=False,
                            ec='tab:blue', lw=0.6, alpha=0.6))
    ax.plot(*start, 'o', color='tab:green', ms=10, label='start')
    ax.plot(*goal, '*', color='tab:red', ms=15, label='goal')
    ax.plot([], [], 's', color=(1.0, 0.3, 0.3, 0.35),
            label=f'obstacle grown by {inflate:.2f} m')
    ax.plot([], [], 'o', mfc='none', mec='tab:blue', label='robot footprint')
    pad = 1.0
    xs = np.r_[px, start[0], goal[0]]
    ys = np.r_[py, start[1], goal[1]]
    ax.set_xlim(xs.min() - pad, xs.max() + pad)
    ax.set_ylim(ys.min() - pad, ys.max() + pad)
    ax.set_title(f'A* path: start ({start[0]:.1f}, {start[1]:.1f}) -> '
                 f'goal ({goal[0]:.1f}, {goal[1]:.1f}), {sep:.2f} m apart')
    ax.legend(loc='upper right', fontsize=8, framealpha=0.85)
    png = os.path.join(out, 'astar_path.png')
    fig.savefig(png, dpi=120, bbox_inches='tight')

    # ---------------------------------------------------------------- report
    lines = [f'Grid: {args.grid}',
             f'Start ({start[0]:.2f}, {start[1]:.2f}) -> goal ({goal[0]:.2f}, {goal[1]:.2f}), '
             f'{sep:.2f} m apart',
             f'Robot radius {args.robot_radius:.3f} m + margin {args.margin:.3f} m '
             f'-> obstacles grown by {inflate:.3f} m',
             f'A* path: {len(path)} cells, {length:.2f} m; after line-of-sight '
             f'shortening: {len(way)} waypoints, {way_len:.2f} m',
             f'Closest approach of the robot centre to an obstacle: {min_clear:.3f} m',
             f'Max deviation from the straight start-goal line: {deviation.max():.2f} m',
             'Straight line start -> goal: ' + ('CLEAR' if straight_free else
                                               'BLOCKED, so the path has to bend'),
             '', 'Waypoints (m):']
    lines += [f'  {k}: ({x:6.2f}, {y:6.2f})' for k, (x, y) in enumerate(way)]
    lines += ['', 'Commands for the hw4 executive (starting at the start pose, '
              f'heading {args.start_yaw:.0f} deg):']
    cmds = commands(way, math.radians(args.start_yaw))
    lines += [f'  {c}' for c in cmds]
    if straight_free:
        lines += ['', 'NOTE: the straight line is already clear. Task 3 needs the path to '
                  'bend around an obstacle: put an obstacle on the line or pick another goal.']
    report = '\n'.join(lines)
    print(report)
    with open(os.path.join(out, 'astar_path.txt'), 'w') as f:
        f.write(report + '\n')
    # one command per line, ending with quit, so the hw4 executive can replay it:
    #   ros2 run day4pkg executive_node < astar_commands.txt
    cmd_path = os.path.join(out, 'astar_commands.txt')
    with open(cmd_path, 'w') as f:
        f.write('\n'.join(cmds + ['quit']) + '\n')
    print(f'\nSaved {png}')
    print(f'Saved {cmd_path}')


if __name__ == '__main__':
    main()
