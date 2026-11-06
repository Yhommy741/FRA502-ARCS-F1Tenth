"""Frenet frame of a closed path: Cartesian (x, y) <-> Frenet (s, d).

    s = arc length along the path (0 .. length, wraps around: closed loop)
    d = signed lateral offset, + = left of the path direction

    d > 0 ──────────────── left wall
            ↑ d
    path  ─────●──────────►  s
    d < 0 ──────────────── right wall

Pure numpy, no ROS: used by frenet_node, obstacle_detector and the planners.
"""

import math

import numpy as np


def wrap_angle(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def load_path_csv(path):
    """generate_raceline.py CSV 's; x; y; psi; kappa; vx; ax' -> dict of arrays (x, y, v; psi/kappa recomputed)."""
    data = np.loadtxt(path, delimiter=';', comments='#', ndmin=2)
    if data.shape[1] < 6:
        raise ValueError(f'{path}: expected columns s; x; y; psi; kappa; vx; ...')
    return {'x': data[:, 1], 'y': data[:, 2], 'v': data[:, 5]}


class FrenetPath:
    """Closed path through points (x[i], y[i]); the last point connects back to the first."""

    def __init__(self, x, y, v=None):
        self.x = np.asarray(x, dtype=float)
        self.y = np.asarray(y, dtype=float)
        n = len(self.x)
        if n < 3:
            raise ValueError('path needs at least 3 points')
        nxt = np.roll(np.arange(n), -1)
        self.seg_dx = self.x[nxt] - self.x
        self.seg_dy = self.y[nxt] - self.y
        self.seg_len = np.hypot(self.seg_dx, self.seg_dy)
        self.seg_len[self.seg_len < 1e-9] = 1e-9
        self.s = np.concatenate([[0.0], np.cumsum(self.seg_len)[:-1]])
        self.length = float(self.seg_len.sum())
        self.psi = np.arctan2(self.seg_dy, self.seg_dx)            # heading of segment i -> i+1
        # curvature: heading change per arc length, centered on each point
        dpsi = wrap_angle(self.psi - np.roll(self.psi, 1))
        self.kappa = dpsi / (0.5 * (self.seg_len + np.roll(self.seg_len, 1)))
        self.v = np.asarray(v, dtype=float) if v is not None else np.zeros(n)
        self.spacing = self.length / n
        # lateral free space to the walls (set by set_widths, e.g. from the occupancy map)
        self.d_left = np.full(n, np.inf)
        self.d_right = np.full(n, np.inf)

    # ------------------------------------------------------------ s helpers
    def wrap_s(self, s):
        return np.mod(s, self.length)

    def ds(self, s_to, s_from):
        """Signed distance along the path from s_from to s_to, in (-length/2, length/2]."""
        return (np.asarray(s_to) - s_from + 0.5 * self.length) % self.length - 0.5 * self.length

    def index_at(self, s):
        """Segment index containing s (s wrapped)."""
        s = self.wrap_s(s)
        return np.clip(np.searchsorted(self.s, s, side='right') - 1, 0, len(self.s) - 1)

    def interp(self, values, s):
        """Periodic linear interpolation of a per-point array at arc length(s) s."""
        s = self.wrap_s(np.asarray(s, dtype=float))
        i = self.index_at(s)
        j = (i + 1) % len(self.s)
        t = (s - self.s[i]) / self.seg_len[i]
        return values[i] + t * (values[j] - values[i])

    # ------------------------------------------------------- conversions
    def project(self, x, y, hint=None, window=None):
        """Cartesian -> Frenet. Returns (s, d, index).

        hint/window: only search segments within `window` metres of the segment `hint`
        (keeps the projection on the right part of the track where it passes close to itself).
        """
        n = len(self.s)
        if hint is None or window is None:
            idx = np.arange(n)
        else:
            k = max(int(window / self.spacing), 2)
            idx = np.arange(hint - k, hint + k + 1) % n
        px, py = x - self.x[idx], y - self.y[idx]
        t = np.clip((px * self.seg_dx[idx] + py * self.seg_dy[idx]) / self.seg_len[idx] ** 2, 0.0, 1.0)
        cx, cy = px - t * self.seg_dx[idx], py - t * self.seg_dy[idx]
        k = int(np.argmin(cx * cx + cy * cy))
        i = int(idx[k])
        s = (self.s[i] + t[k] * self.seg_len[i]) % self.length
        # sign: left of the segment direction = positive (cross product)
        cross = self.seg_dx[i] * py[k] - self.seg_dy[i] * px[k]
        d = math.copysign(math.hypot(cx[k], cy[k]), cross)
        return s, d, i

    def to_cartesian(self, s, d):
        """Frenet -> Cartesian (arrays allowed). Returns (x, y)."""
        s = np.asarray(s, dtype=float)
        d = np.asarray(d, dtype=float)
        i = self.index_at(s)
        t = (self.wrap_s(s) - self.s[i]) / self.seg_len[i]
        bx = self.x[i] + t * self.seg_dx[i]
        by = self.y[i] + t * self.seg_dy[i]
        psi = self.heading_at(s)
        return bx - d * np.sin(psi), by + d * np.cos(psi)

    def heading_at(self, s):
        """Smooth path heading at s (interpolated between neighbouring segment headings)."""
        s = self.wrap_s(np.asarray(s, dtype=float))
        i = self.index_at(s)
        t = (s - self.s[i]) / self.seg_len[i]
        # heading at a point = mean of the segments before and after it
        h_point = self.psi_point()
        j = (i + 1) % len(self.s)
        return h_point[i] + t * wrap_angle(h_point[j] - h_point[i])

    def psi_point(self):
        if not hasattr(self, '_psi_point'):
            prev = np.roll(self.psi, 1)
            self._psi_point = prev + 0.5 * wrap_angle(self.psi - prev)
        return self._psi_point

    # ---------------------------------------------------------- track width
    def set_widths_from_grid(self, grid, resolution, origin_x, origin_y, max_width=3.0,
                             occupied=65):
        """Free space left/right of every path point from an occupancy grid (nav_msgs/OccupancyGrid data).

        grid: 2D int array [row = y, col = x] (row 0 = origin_y), -1 unknown, 0..100 occupancy.
        A cell counts as a wall when occupied >= `occupied` or unknown.
        """
        h, w = grid.shape
        blocked = (grid >= occupied) | (grid < 0)
        steps = np.arange(resolution, max_width + 1e-9, resolution * 0.5)
        normal_x = -np.sin(self.psi_point())
        normal_y = np.cos(self.psi_point())
        for side, out in ((1.0, 'd_left'), (-1.0, 'd_right')):
            px = self.x[:, None] + side * normal_x[:, None] * steps[None, :]
            py = self.y[:, None] + side * normal_y[:, None] * steps[None, :]
            col = np.floor((px - origin_x) / resolution).astype(int)
            row = np.floor((py - origin_y) / resolution).astype(int)
            outside = (col < 0) | (col >= w) | (row < 0) | (row >= h)
            hit = outside.copy()
            hit[~outside] = blocked[row[~outside], col[~outside]]
            first = np.where(hit.any(axis=1), hit.argmax(axis=1), len(steps) - 1)
            setattr(self, out, steps[first] - resolution * 0.5)
        return self.d_left, self.d_right
