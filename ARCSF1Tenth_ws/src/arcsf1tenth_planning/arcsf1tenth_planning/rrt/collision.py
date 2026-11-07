"""Collision checking for the car footprint against the saved map and lidar obstacles.

Footprint: the car rectangle (rear axle -> front bumper, full width) covered by n_circles
circles along its center line:

        ┌───────────────────────┐
        │ (  o  )(  o  )(  o  ) │   circle radius = hypot(segment / 2, half width)
        └───────────────────────┘
        rear    rear axle ─► x   front

A pose (x, y, yaw of the rear axle) is free when every circle is at least
radius + wall_margin from a map wall and radius + obstacle_margin from every obstacle point.

    static walls : distance transform of the occupancy grid (occupied or unknown = wall),
                   computed once per map
    obstacles    : lidar points not explained by the map (obstacle_points), KD-tree, every scan
"""

import math

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree


class CollisionChecker:
    def __init__(self, half_width, car_front, car_rear, wall_margin, obstacle_margin, n_circles=3):
        length = car_front + car_rear
        seg = length / n_circles
        self.offsets = -car_rear + seg * (np.arange(n_circles) + 0.5)   # circle centers along x [m]
        self.radius = math.hypot(seg / 2, half_width)
        self.wall_margin = wall_margin
        self.obstacle_margin = obstacle_margin
        self.dist = None            # distance to the nearest wall per map cell [m]
        self.resolution = None
        self.origin = None
        self.tree = None            # KD-tree of obstacle points
        self.points = np.zeros((0, 2))

    # ---------------------------------------------------------------- inputs
    def set_map(self, grid, resolution, origin_x, origin_y, occupied=65):
        """grid: (H, W) int occupancy (-1 unknown, 0..100); row = y, col = x (nav_msgs/OccupancyGrid)."""
        grid = np.asarray(grid)
        wall = (grid >= occupied) | (grid < 0)
        self.dist = ndimage.distance_transform_edt(~wall) * resolution
        self.resolution = resolution
        self.origin = (origin_x, origin_y)

    def set_obstacles(self, points):
        self.points = np.asarray(points, dtype=float).reshape(-1, 2)
        self.tree = cKDTree(self.points) if len(self.points) else None

    @property
    def ready(self):
        return self.dist is not None

    # --------------------------------------------------------------- queries
    def wall_distance(self, x, y):
        """Distance [m] to the nearest map wall; 0 outside the map."""
        x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
        col = np.floor((x - self.origin[0]) / self.resolution).astype(int)
        row = np.floor((y - self.origin[1]) / self.resolution).astype(int)
        h, w = self.dist.shape
        inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
        out = np.zeros(x.shape)
        out[inside] = self.dist[row[inside], col[inside]]
        return out

    def obstacle_distance(self, x, y, limit=5.0):
        """Distance [m] to the nearest obstacle point (inf if none within limit)."""
        x = np.asarray(x, dtype=float)
        if self.tree is None:
            return np.full(x.shape, np.inf)
        d, _ = self.tree.query(np.column_stack([x.ravel(), np.asarray(y, dtype=float).ravel()]),
                               distance_upper_bound=limit)
        return d.reshape(x.shape)

    def circles(self, x, y, yaw):
        x, y, yaw = (np.asarray(a, dtype=float) for a in (x, y, yaw))
        return (x[..., None] + np.cos(yaw)[..., None] * self.offsets,
                y[..., None] + np.sin(yaw)[..., None] * self.offsets)

    def clearance(self, x, y, yaw):
        """Signed clearance [m] of the footprint at each pose (< 0 = collision). Arrays of any shape."""
        cx, cy = self.circles(x, y, yaw)
        wall = self.wall_distance(cx, cy) - self.radius - self.wall_margin
        obstacle = self.obstacle_distance(cx, cy) - self.radius - self.obstacle_margin
        return np.minimum(wall, obstacle).min(axis=-1)

    def free(self, x, y, yaw):
        """Footprint clear of map walls and obstacles (the RRT's check)."""
        return self.clearance(x, y, yaw) >= 0.0

    def obstacle_free(self, x, y, yaw, margin=None):
        """Footprint clear of obstacles only (map walls ignored): is a path that was planned on
        the map (global path, committed RRT path) blocked by something new?"""
        margin = self.obstacle_margin if margin is None else margin
        cx, cy = self.circles(x, y, yaw)
        return (self.obstacle_distance(cx, cy) - self.radius - margin).min(axis=-1) >= 0.0

    def unexplained_points(self, x, y, wall_margin):
        """Mask of points further than wall_margin from every map wall (= not part of the map)."""
        return self.wall_distance(x, y) > wall_margin
