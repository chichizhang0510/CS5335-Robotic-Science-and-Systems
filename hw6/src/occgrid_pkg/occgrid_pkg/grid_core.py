"""
Occupancy grid core logic (pure Python + NumPy, no ROS dependency).

Grid convention
---------------
- size x size cells, each `resolution` meters (default 200 x 200, 5 cm -> 10 m x 10 m)
- World origin (0, 0) = robot start position = cell (row=size//2, col=size//2)
- row index follows +y, col index follows +x
  (display with origin='lower' so +y points up in the image)
- Each cell stores log-odds:  l = log(p / (1 - p)),  l = 0  <=>  p = 0.5 (unknown)
"""

import math
import numpy as np


class OccupancyGrid:
    def __init__(self, size=200, resolution=0.05,
                 l_occ=0.85, l_free=-0.4, l_min=-4.0, l_max=4.0):
        self.size = size
        self.res = resolution
        self.origin = size // 2           # cell index of world (0, 0)
        self.l_occ = l_occ                # log(0.7/0.3)  -> hit cell
        self.l_free = l_free              # ~log(0.4/0.6) -> cells the beam passed through
        self.l_min = l_min                # clamp so cells can still change later
        self.l_max = l_max
        self.logodds = np.zeros((size, size), dtype=np.float64)

    # ------------------------------------------------------------------ #
    # Coordinate helpers
    # ------------------------------------------------------------------ #
    def world_to_cell(self, x, y):
        """World (m) -> (row, col). May be outside the grid."""
        col = int(math.floor(x / self.res)) + self.origin
        row = int(math.floor(y / self.res)) + self.origin
        return row, col

    def in_bounds(self, row, col):
        return 0 <= row < self.size and 0 <= col < self.size

    @staticmethod
    def bresenham(r0, c0, r1, c1):
        """All integer cells on the line from (r0, c0) to (r1, c1), inclusive."""
        cells = []
        dc = abs(c1 - c0)
        dr = -abs(r1 - r0)
        sc = 1 if c0 < c1 else -1
        sr = 1 if r0 < r1 else -1
        err = dc + dr
        while True:
            cells.append((r0, c0))
            if r0 == r1 and c0 == c1:
                break
            e2 = 2 * err
            if e2 >= dr:
                err += dr
                c0 += sc
            if e2 <= dc:
                err += dc
                r0 += sr
        return cells

    # ------------------------------------------------------------------ #
    # Main update: ray casting with the inverse sensor model
    # ------------------------------------------------------------------ #
    def update(self, ranges, angle_min, angle_increment, range_min, range_max,
               pose=(0.0, 0.0, 0.0), laser_yaw_offset=0.0,
               max_range_as_free=False):
        """
        Integrate one LaserScan into the grid.

        ranges            : list of distances (m); may contain inf / nan
        pose              : robot pose (x, y, theta) in the grid's world frame
        laser_yaw_offset  : yaw of the lidar frame relative to the robot base (rad)
        max_range_as_free : if True, 'no return' beams (inf / > range_max) clear cells
                            out to range_max without marking a hit. Default False
                            (safer: some drivers also report inf for too-close returns).
        """
        xr, yr, th = pose
        r0, c0 = self.world_to_cell(xr, yr)

        free_cells = set()
        occ_cells = set()

        for i, r in enumerate(ranges):
            if r is None or math.isnan(r):
                continue
            hit = True
            if math.isinf(r) or r > range_max:
                if not max_range_as_free:
                    continue
                r = range_max
                hit = False
            elif r < range_min:
                continue

            ang = th + laser_yaw_offset + angle_min + i * angle_increment
            xe = xr + r * math.cos(ang)
            ye = yr + r * math.sin(ang)
            r1, c1 = self.world_to_cell(xe, ye)

            ray = self.bresenham(r0, c0, r1, c1)
            # every cell before the endpoint: the beam passed through -> free
            for cell in ray[:-1]:
                free_cells.add(cell)
            # endpoint: obstacle (if it was a real return)
            if hit:
                occ_cells.add(ray[-1])
            else:
                free_cells.add(ray[-1])

        # A cell hit by one beam should not be cleared by a neighbouring beam
        # in the same scan; each cell is updated at most once per scan.
        free_cells -= occ_cells

        for (row, col) in free_cells:
            if self.in_bounds(row, col):
                self.logodds[row, col] += self.l_free
        for (row, col) in occ_cells:
            if self.in_bounds(row, col):
                self.logodds[row, col] += self.l_occ

        np.clip(self.logodds, self.l_min, self.l_max, out=self.logodds)

    # ------------------------------------------------------------------ #
    # Output
    # ------------------------------------------------------------------ #
    def probability(self):
        """Occupancy probability p = 1 - 1 / (1 + exp(l))."""
        return 1.0 - 1.0 / (1.0 + np.exp(self.logodds))

    def to_image(self):
        """Grayscale image in [0,1]: free = white, occupied = black, unknown = gray."""
        return 1.0 - self.probability()

    def extent(self):
        """[xmin, xmax, ymin, ymax] in meters, for matplotlib imshow."""
        half = self.origin * self.res
        far = (self.size - self.origin) * self.res
        return [-half, far, -half, far]


def draw_grid(ax, grid, pose=None, title=None):
    """Draw the grid (and optionally the robot pose) on a matplotlib Axes."""
    ax.imshow(grid.to_image(), cmap='gray', vmin=0.0, vmax=1.0,
              origin='lower', extent=grid.extent(), interpolation='nearest')
    if pose is not None:
        x, y, th = pose
        ax.plot(x, y, 'ro', markersize=4)
        ax.arrow(x, y, 0.3 * math.cos(th), 0.3 * math.sin(th),
                 head_width=0.1, color='r')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    if title:
        ax.set_title(title)


def save_grid_image(grid, path, pose=None, title=None):
    """Render the grid to a PNG file (works without a display)."""
    # Figure + Agg canvas directly: no GUI backend needed, safe inside a ROS node
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    fig = Figure(figsize=(7, 7))
    FigureCanvasAgg(fig)
    ax = fig.add_subplot(111)
    draw_grid(ax, grid, pose, title)
    fig.savefig(path, dpi=120, bbox_inches='tight')
