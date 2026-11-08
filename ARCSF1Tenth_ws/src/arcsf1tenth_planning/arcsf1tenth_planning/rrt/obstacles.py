import math

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree


# Return the lidar points in the map frame: x, y, range and a mask of the usable beams.
def scan_to_map(ranges, angle_min, angle_increment, range_min, range_max, max_range, pose):
    r = np.asarray(ranges, dtype=float)
    a = angle_min + np.arange(len(r)) * angle_increment
    valid = np.isfinite(r) & (r >= range_min) & (r <= min(range_max, max_range))
    r = np.where(valid, r, 0.0)
    x0, y0, yaw = pose
    lx, ly = r * np.cos(a), r * np.sin(a)
    x = x0 + math.cos(yaw) * lx - math.sin(yaw) * ly
    y = y0 + math.sin(yaw) * lx + math.cos(yaw) * ly
    return x, y, r, valid


# Group consecutive beams (scan order) whose points are closer than gap, wrapping around 360 deg.
def clusters(mask, x, y, gap):
    idx = np.flatnonzero(mask)
    if len(idx) == 0:
        return []
    groups = [[idx[0]]]
    for a, b in zip(idx[:-1], idx[1:]):
        if math.hypot(x[b] - x[a], y[b] - y[a]) < gap:
            groups[-1].append(b)
        else:
            groups.append([b])
    if len(groups) > 1 and math.hypot(x[idx[0]] - x[idx[-1]], y[idx[0]] - y[idx[-1]]) < gap:
        groups[0] = groups.pop() + groups[0]   # first and last beam are neighbours (360 deg scan)
    return [np.array(g) for g in groups]


class ObstacleDetector:
    # Store the detection rules (same rules as obstacle_detector.py, plus a range-dependent wall margin).
    def __init__(self, wall_margin=0.3, range_gain=0.035, cluster_gap=0.3, min_points=3, max_size=2.0,
                 track_filter=True, track_margin=0.1, max_width=3.0):
        self.wall_margin = wall_margin      # [m] points closer to a map wall = wall ...
        self.range_gain = range_gain        # [m/m] ... plus this * range (0 = fixed margin)
        self.cluster_gap = cluster_gap      # [m] max distance between neighbouring points of one obstacle
        self.min_points = min_points        # points per obstacle (fewer = noise)
        self.max_size = max_size            # [m] wider clusters = mislocalized wall, ignored
        self.track_filter = track_filter    # drop clusters outside the track
        self.track_margin = track_margin    # [m] keep clusters reaching this far into the track
        self.max_width = max_width          # [m] track width search limit per side
        self.dist = None                    # distance to the nearest wall per map cell [m]
        self.resolution = None
        self.origin = None
        self.path = None                    # (x, y, psi) of the global path
        self.path_tree = None
        self.d_left = None                  # [m] free space left / right of each path point
        self.d_right = None

    # Compute the wall distance field and the track widths left and right of the global path.
    def set_map(self, grid, resolution, origin_x, origin_y, path_x, path_y, path_psi, occupied=65):
        grid = np.asarray(grid)
        wall = (grid >= occupied) | (grid < 0)
        self.dist = ndimage.distance_transform_edt(~wall) * resolution
        self.resolution = resolution
        self.origin = (origin_x, origin_y)
        self.path = tuple(np.asarray(a, dtype=float) for a in (path_x, path_y, path_psi))
        self.path_tree = cKDTree(np.column_stack(self.path[:2]))
        self.d_left, self.d_right = self.widths(wall)

    # Return the free space [m] along the left and right normal of every path point (as frenet_path.py).
    def widths(self, wall):
        px0, py0, psi = self.path
        steps = np.arange(self.resolution, self.max_width + 1e-9, self.resolution * 0.5)
        out = []
        for side in (1.0, -1.0):
            px = px0[:, None] - side * np.sin(psi)[:, None] * steps
            py = py0[:, None] + side * np.cos(psi)[:, None] * steps
            row, col, inside = self.cells(px, py)
            hit = ~inside
            hit[inside] = wall[row[inside], col[inside]]
            first = np.where(hit.any(axis=1), hit.argmax(axis=1), len(steps) - 1)
            out.append(steps[first] - self.resolution * 0.5)
        return out

    # Return True if a cluster lies outside the track (entirely beyond the left or the right wall).
    def outside_track(self, cx, cy):
        px, py, psi = self.path
        j = self.path_tree.query([cx.mean(), cy.mean()])[1]
        d = -np.sin(psi[j]) * (cx - px[j]) + np.cos(psi[j]) * (cy - py[j])
        return d.min() > self.d_left[j] - self.track_margin or d.max() < -self.d_right[j] + self.track_margin

    # Return True once a map has been set.
    @property
    def ready(self):
        return self.dist is not None

    # Return the grid row, column and an inside-the-map mask of each point.
    def cells(self, x, y):
        col = np.floor((np.asarray(x, dtype=float) - self.origin[0]) / self.resolution).astype(int)
        row = np.floor((np.asarray(y, dtype=float) - self.origin[1]) / self.resolution).astype(int)
        h, w = self.dist.shape
        inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
        return row, col, inside

    # Return the obstacle points (N, 2) of one scan already in the map frame.
    def detect(self, x, y, r, valid):
        row, col, inside = self.cells(x, y)
        ok = valid & inside
        wall_dist = np.zeros(len(x))
        wall_dist[ok] = self.dist[row[ok], col[ok]]
        # not a wall of the map; a heading error moves far points more, so the margin grows with range
        unexplained = ok & (wall_dist > self.wall_margin + self.range_gain * r)
        keep = []
        for cluster in clusters(unexplained, x, y, self.cluster_gap):
            if len(cluster) < self.min_points:
                continue
            cx, cy = x[cluster], y[cluster]
            if 2.0 * np.max(np.hypot(cx - cx.mean(), cy - cy.mean())) > self.max_size:
                continue
            if self.track_filter and self.outside_track(cx, cy):
                continue      # outside the track (seen through a gap in a wall)
            keep.append(cluster)
        if not keep:
            return np.zeros((0, 2))
        idx = np.concatenate(keep)
        return np.column_stack([x[idx], y[idx]])
