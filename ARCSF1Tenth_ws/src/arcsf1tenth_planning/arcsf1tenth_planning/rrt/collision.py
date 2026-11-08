import math

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree


class CollisionChecker:
    # Store the car footprint (as circles) and the safety margins.
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

    # Compute the wall distance field from an occupancy grid.
    def set_map(self, grid, resolution, origin_x, origin_y, occupied=65):
        grid = np.asarray(grid)
        wall = (grid >= occupied) | (grid < 0)
        self.dist = ndimage.distance_transform_edt(~wall) * resolution
        self.resolution = resolution
        self.origin = (origin_x, origin_y)

    # Store the obstacle points and build their KD-tree.
    def set_obstacles(self, points):
        self.points = np.asarray(points, dtype=float).reshape(-1, 2)
        self.tree = cKDTree(self.points) if len(self.points) else None

    # Return True once a map has been set.
    @property
    def ready(self):
        return self.dist is not None

    # Return the distance [m] to the nearest map wall (0 outside the map).
    def wall_distance(self, x, y):
        x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
        col = np.floor((x - self.origin[0]) / self.resolution).astype(int)
        row = np.floor((y - self.origin[1]) / self.resolution).astype(int)
        h, w = self.dist.shape
        inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
        out = np.zeros(x.shape)
        out[inside] = self.dist[row[inside], col[inside]]
        return out

    # Return the distance [m] to the nearest obstacle point (inf if none within limit).
    def obstacle_distance(self, x, y, limit=5.0):
        x = np.asarray(x, dtype=float)
        if self.tree is None:
            return np.full(x.shape, np.inf)
        d, _ = self.tree.query(np.column_stack([x.ravel(), np.asarray(y, dtype=float).ravel()]),
                               distance_upper_bound=limit)
        return d.reshape(x.shape)

    # Return the footprint circle centres at each pose.
    def circles(self, x, y, yaw):
        x, y, yaw = (np.asarray(a, dtype=float) for a in (x, y, yaw))
        return (x[..., None] + np.cos(yaw)[..., None] * self.offsets,
                y[..., None] + np.sin(yaw)[..., None] * self.offsets)

    # Return the signed footprint clearance [m] at each pose (< 0 = collision).
    def clearance(self, x, y, yaw):
        cx, cy = self.circles(x, y, yaw)
        wall = self.wall_distance(cx, cy) - self.radius - self.wall_margin
        obstacle = self.obstacle_distance(cx, cy) - self.radius - self.obstacle_margin
        return np.minimum(wall, obstacle).min(axis=-1)

    # Return True where the footprint is clear of map walls and obstacles.
    def free(self, x, y, yaw):
        return self.clearance(x, y, yaw) >= 0.0

    # Return True where the footprint is clear of obstacles only (map walls ignored).
    def obstacle_free(self, x, y, yaw, margin=None):
        margin = self.obstacle_margin if margin is None else margin
        cx, cy = self.circles(x, y, yaw)
        return (self.obstacle_distance(cx, cy) - self.radius - margin).min(axis=-1) >= 0.0

    # Return a mask of the points further than wall_margin from every map wall.
    def unexplained_points(self, x, y, wall_margin):
        return self.wall_distance(x, y) > wall_margin
