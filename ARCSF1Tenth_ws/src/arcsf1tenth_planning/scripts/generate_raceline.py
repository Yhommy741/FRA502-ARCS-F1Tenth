#!/usr/bin/python3
"""Generate a centerline and a minimum-curvature raceline from an occupancy map.

Pipeline:
  1. track area   free cells connected to the start pose (small specks filled)
  2. centerline   skeleton of the track area, main loop through the start pose in the
                  driving direction (dead ends ignored), smoothed and resampled
  3. widths       distance to the walls left and right of each centerline point
  4. raceline     minimum curvature: each point moves sideways within the track
                  (keeping wall_margin) so the path bends as little as possible
  5. speed        v = min(max_speed, sqrt(a_lat / |curvature|)) with acceleration
                  and braking limits

Outputs (next to the map unless --out-dir), in the map frame:
  <map>_centerline.csv   x_m, y_m, w_tr_right_m, w_tr_left_m        (F1TENTH / TUM format)
  <map>_raceline.csv     s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2
  <map>_raceline.png     check picture (--show also opens it in a window;
                         show_raceline.py shows existing files without regenerating)

Usage:
  generate_raceline.py <map.yaml | track name> [--start X Y YAW] [--wall-margin 0.3] [--a-lat 4.0] ...
"""

import argparse
import math
import os

import cv2
import numpy as np
import yaml
from scipy import interpolate, ndimage, sparse
from scipy.sparse import csgraph
from scipy.sparse.linalg import splu

from arcsf1tenth_planning.tracks import resolve_map


# ------------------------------------------------------------------------- map

class GridMap:
    def __init__(self, map_yaml):
        with open(map_yaml) as f:
            meta = yaml.safe_load(f)
        image_path = os.path.join(os.path.dirname(os.path.abspath(map_yaml)), meta['image'])
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise FileNotFoundError(f'Cannot read map image {image_path}')
        occupancy = image.astype(float) / 255.0
        if not meta.get('negate', 0):
            occupancy = 1.0 - occupancy
        self.free = occupancy < meta['free_thresh']  # same rule as nav2 map_server
        self.resolution = float(meta['resolution'])
        self.origin = meta['origin'][:2]
        self.height, self.width = image.shape
        self.image = image

    def to_pixel(self, x, y):
        col = (np.asarray(x) - self.origin[0]) / self.resolution - 0.5
        row = self.height - 0.5 - (np.asarray(y) - self.origin[1]) / self.resolution
        return col, row

    def to_world(self, col, row):
        x = self.origin[0] + (np.asarray(col) + 0.5) * self.resolution
        y = self.origin[1] + (self.height - np.asarray(row) - 0.5) * self.resolution
        return x, y

    def extent(self):
        x0, y0 = self.origin
        return [x0, x0 + self.width * self.resolution, y0, y0 + self.height * self.resolution]


def track_area(grid, start, min_hole_area):
    """Free cells connected to the start, with small holes (noise specks) filled."""
    free = cv2.morphologyEx(grid.free.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    col, row = (int(round(v)) for v in grid.to_pixel(*start[:2]))
    if not (0 <= row < grid.height and 0 <= col < grid.width) or not free[row, col]:
        raise RuntimeError(f'Start pose {start[:2]} is not in free space on the map (use --start X Y YAW)')
    labels, _ = ndimage.label(free)
    area = labels == labels[row, col]

    # Fill holes smaller than min_hole_area (keeps the real inner walls of the loop)
    holes, n = ndimage.label(~area)
    border = set(np.unique(np.concatenate([holes[0], holes[-1], holes[:, 0], holes[:, -1]])))
    sizes = ndimage.sum(np.ones_like(holes), holes, index=np.arange(1, n + 1))
    min_px = min_hole_area / grid.resolution ** 2
    for label, size in zip(range(1, n + 1), sizes):
        if label not in border and size < min_px:
            area[holes == label] = True
    return area


# ------------------------------------------------------------------ centerline

def main_loop_pixels(area, grid, start, cut_half_width=1.5):
    """Main loop of the track as ordered world points, starting at the start pose.

    The skeleton (medial axis) of the track area is cut by a short line across the
    track at the start pose; the shortest path along the skeleton from just ahead of
    the cut to just behind it then has to go all the way around the loop in the
    driving direction. Dead-end corridors and small skeleton loops (map noise) are
    never on that shortest path, so they are ignored automatically.
    """
    skeleton = cv2.ximgproc.thinning(area.astype(np.uint8) * 255, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN) > 0
    rows, cols = np.nonzero(skeleton)
    if len(rows) < 20:
        raise RuntimeError('Track area has no usable skeleton')

    # Cut across the track at the start: skeleton pixels within +-1.5 px along the heading
    scol, srow = grid.to_pixel(*start[:2])
    heading = np.array([math.cos(start[2]), -math.sin(start[2])])  # (col, row): image rows point down
    along = (cols - scol) * heading[0] + (rows - srow) * heading[1]
    across = -(cols - scol) * heading[1] + (rows - srow) * heading[0]
    near = np.abs(across) < cut_half_width / grid.resolution
    keep = ~(near & (np.abs(along) <= 1.5))
    ahead = np.flatnonzero(keep & near & (along > 1.5) & (along < 10))
    behind = np.flatnonzero(keep & near & (along < -1.5) & (along > -10))
    if len(ahead) == 0 or len(behind) == 0:
        raise RuntimeError('Start pose is not on the track centerline area (use --start X Y YAW)')
    source = ahead[np.argmin(along[ahead])]
    target = behind[np.argmax(along[behind])]

    # 8-connected pixel graph of the cut skeleton
    index = -np.ones(skeleton.shape, int)
    index[rows[keep], cols[keep]] = np.flatnonzero(keep)
    edges_from, edges_to, weights = [], [], []
    for dr, dc in [(0, 1), (1, -1), (1, 0), (1, 1)]:
        r2, c2 = rows + dr, cols + dc
        ok = keep & (r2 >= 0) & (r2 < skeleton.shape[0]) & (c2 >= 0) & (c2 < skeleton.shape[1])
        other = np.full(len(rows), -1)
        other[ok] = index[r2[ok], c2[ok]]
        ok &= other >= 0
        edges_from.append(np.flatnonzero(ok))
        edges_to.append(other[ok])
        weights.append(np.full(ok.sum(), math.hypot(dr, dc)))
    graph = sparse.coo_matrix((np.concatenate(weights), (np.concatenate(edges_from), np.concatenate(edges_to))),
                              shape=(len(rows), len(rows))).tocsr()

    distance, predecessor = csgraph.dijkstra(graph, directed=False, indices=source, return_predecessors=True)
    if not np.isfinite(distance[target]):
        raise RuntimeError('No closed loop through the start pose (is the track closed?)')
    path = [target]
    while path[-1] != source:
        path.append(predecessor[path[-1]])
    path = path[::-1]
    return np.column_stack(grid.to_world(cols[path], rows[path]))


def resample_closed(points, spacing, smoothing=0.0):
    """Periodic spline through points, resampled every `spacing` metres.

    Returns (points, tangent unit vectors, curvature)."""
    pts = points[np.r_[np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-6, True]]
    tck, _ = interpolate.splprep([pts[:, 0], pts[:, 1]], s=smoothing * len(pts), per=1, k=3)
    dense = np.column_stack(interpolate.splev(np.linspace(0, 1, 20 * len(pts), endpoint=False), tck))
    seg = np.linalg.norm(np.diff(np.vstack([dense, dense[:1]]), axis=0), axis=1)
    length = seg.sum()
    n = max(int(round(length / spacing)), 10)
    u_dense = np.linspace(0, 1, len(dense), endpoint=False)
    s_dense = np.concatenate([[0.0], np.cumsum(seg)[:-1]])
    u = np.interp(np.arange(n) * length / n, s_dense, u_dense)
    x, y = interpolate.splev(u, tck)
    dx, dy = interpolate.splev(u, tck, der=1)
    ddx, ddy = interpolate.splev(u, tck, der=2)
    speed = np.hypot(dx, dy)
    tangent = np.column_stack([dx, dy]) / speed[:, None]
    curvature = (dx * ddy - dy * ddx) / speed ** 3
    return np.column_stack([x, y]), tangent, curvature


def track_widths(points, tangent, area, grid, max_width=10.0):
    """Distance from each point to the track edge on the right and on the left."""
    normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])  # left
    step = grid.resolution / 2.0
    distances = np.arange(step, max_width, step)
    widths = []
    for sign in (-1.0, 1.0):  # right, left
        ray = points[:, None, :] + sign * normal[:, None, :] * distances[None, :, None]
        col, row = grid.to_pixel(ray[..., 0], ray[..., 1])
        col, row = np.round(col).astype(int), np.round(row).astype(int)
        inside = (row >= 0) & (row < grid.height) & (col >= 0) & (col < grid.width)
        free = np.zeros_like(inside)
        free[inside] = area[row[inside], col[inside]]
        first_blocked = np.argmax(~free, axis=1)
        first_blocked[free.all(axis=1)] = len(distances) - 1
        widths.append(distances[first_blocked] - step)
    return widths[0], widths[1], normal


# -------------------------------------------------------------------- raceline

def minimum_curvature(points, normal, w_right, w_left, margin, max_iter=20000):
    """Lateral offsets a (left positive) minimising sum |second difference of position|^2.

    Box-constrained QP  min 1/2 a'Ha - g'a,  lower <= a <= upper,  H = A'A (sparse, banded),
    solved with a primal active-set method: start at a = 0 (the centerline, feasible),
    step toward the optimum of the free offsets and stop at the first bound hit; when the
    full step fits, release bounds whose gradient pulls back inside. Each iteration is one
    sparse direct solve, so it is exact and fast even for long tracks (iterative least
    squares solvers converge very slowly here: the second difference is ill-conditioned).
    """
    n = len(points)
    D = sparse.diags([np.ones(n), -2 * np.ones(n), np.ones(n)], [-1, 0, 1], shape=(n, n), format='lil')
    D[0, n - 1] = 1.0
    D[n - 1, 0] = 1.0
    D = D.tocsr()
    A = sparse.vstack([D @ sparse.diags(normal[:, 0]), D @ sparse.diags(normal[:, 1])]).tocsr()
    b = -np.concatenate([D @ points[:, 0], D @ points[:, 1]])
    H = (A.T @ A + 1e-9 * sparse.eye(n)).tocsc()
    g = A.T @ b
    upper = np.maximum(w_left - margin, 1e-3)
    lower = -np.maximum(w_right - margin, 1e-3)

    a = np.zeros(n)
    bound = np.zeros(n, dtype=np.int8)  # -1 at lower, +1 at upper, 0 free
    for _ in range(max_iter):
        free = bound == 0
        target = a.copy()
        if free.any():
            fixed = ~free
            rhs = g[free] - H[free][:, fixed] @ a[fixed]
            target[free] = splu(H[free][:, free].tocsc()).solve(rhs)
        step = target - a

        # Largest fraction of the step that stays inside the bounds
        with np.errstate(divide='ignore', invalid='ignore'):
            frac = np.where(step > 1e-12, (upper - a) / step,
                            np.where(step < -1e-12, (lower - a) / step, np.inf))
        frac[~free] = np.inf
        alpha = min(1.0, frac.min())
        a = a + alpha * step
        if alpha < 1.0:
            hit = free & (frac <= alpha + 1e-12)
            bound[hit & (step > 0)] = 1
            bound[hit & (step < 0)] = -1
            a[bound == 1] = upper[bound == 1]
            a[bound == -1] = lower[bound == -1]
            continue

        grad = H @ a - g
        release = ((bound == -1) & (grad < -1e-10)) | ((bound == 1) & (grad > 1e-10))
        if not release.any():
            return points + a[:, None] * normal
        bound[release] = 0
    raise RuntimeError(f'minimum curvature did not converge in {max_iter} iterations')


def speed_profile(curvature, ds, max_speed, a_lat, a_accel, a_brake):
    """Corner limit, then forward (acceleration) and backward (braking) passes on a closed loop."""
    n = len(curvature)
    v_limit = np.minimum(max_speed, np.sqrt(a_lat / np.maximum(np.abs(curvature), 1e-6)))
    v = np.tile(v_limit, 3)  # three laps so the passes wrap around the loop
    for i in range(1, len(v)):
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * a_accel * ds))
    for i in range(len(v) - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * a_brake * ds))
    v = v[n:2 * n]
    ax = (np.roll(v, -1) ** 2 - v ** 2) / (2 * ds)
    return v, ax


# ---------------------------------------------------------------------- output

def save_outputs(prefix, center, w_right, w_left, race, tangent, kappa, v, ax, grid, start, show):
    centerline = np.column_stack([center, w_right, w_left])
    np.savetxt(f'{prefix}_centerline.csv', centerline,
               delimiter=', ', fmt='%.6f', header='x_m, y_m, w_tr_right_m, w_tr_left_m')
    ds = np.linalg.norm(np.diff(np.vstack([race, race[:1]]), axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(ds)[:-1]])
    psi = np.arctan2(tangent[:, 1], tangent[:, 0])
    raceline = np.column_stack([s, race, psi, kappa, v, ax])
    np.savetxt(f'{prefix}_raceline.csv', raceline,
               delimiter='; ', fmt='%.7f', header='s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2')

    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from arcsf1tenth_planning.plotting import plot_track
    fig = plot_track(grid.image, grid.extent(), centerline, raceline, start, title=os.path.basename(prefix))
    fig.savefig(f'{prefix}_raceline.png', dpi=120, bbox_inches='tight')
    print(f'wrote {prefix}_centerline.csv, {prefix}_raceline.csv, {prefix}_raceline.png')
    if show:
        plt.show()


# ------------------------------------------------------------------------ main

def load_vehicle(path):
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('arcsf1tenth_description'), 'config', 'vehicle.yaml')
    with open(path) as f:
        return yaml.safe_load(f)['/**']['ros__parameters']


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('map_yaml', help='map yaml (e.g. slam_map.yaml) or a track name (e.g. brands_hatch: '
                                         'its slam_map.yaml, else map.yaml)')
    parser.add_argument('--start', type=float, nargs=3, default=None, metavar=('X', 'Y', 'YAW'),
                        help='start pose in the map frame; YAW sets the driving direction '
                             '(default 0 0 0 = where SLAM mapping started; track name + map.yaml: track.yaml start_pose)')
    parser.add_argument('--out-dir', default=None, help='output folder (default: next to the map)')
    parser.add_argument('--spacing', type=float, default=0.1, help='[m] distance between points')
    parser.add_argument('--wall-margin', type=float, default=0.3, help='[m] keep the raceline this far from walls')
    parser.add_argument('--a-lat', type=float, default=4.0, help='[m/s^2] max lateral acceleration in corners')
    parser.add_argument('--min-hole-area', type=float, default=0.5,
                        help='[m^2] fill obstacles smaller than this inside the track (map noise)')
    parser.add_argument('--smoothing', type=float, default=0.002, help='centerline spline smoothing')
    parser.add_argument('--iterations', type=int, default=3, help='minimum-curvature iterations')
    parser.add_argument('--vehicle-config', default=None, help='vehicle.yaml (default: arcsf1tenth_description)')
    parser.add_argument('--show', action='store_true', help='also open the picture in a window')
    args = parser.parse_args()

    map_yaml, track_start = resolve_map(args.map_yaml)
    vehicle = load_vehicle(args.vehicle_config)
    grid = GridMap(map_yaml)
    start = args.start or track_start or [0.0, 0.0, 0.0]

    area = track_area(grid, start, args.min_hole_area)
    loop = main_loop_pixels(area, grid, start)
    center, tangent, _ = resample_closed(loop, args.spacing, args.smoothing)
    w_right, w_left, normal = track_widths(center, tangent, area, grid)
    print(f'centerline: {len(center)} points, {len(center) * args.spacing:.1f} m, '
          f'track width {np.median(w_right + w_left):.2f} m (median), {np.min(w_right + w_left):.2f} m (min)')

    race = center
    for _ in range(args.iterations):
        race_tangent = resample_closed(race, args.spacing)[1]
        wr, wl, n = track_widths(race, race_tangent, area, grid)
        race = minimum_curvature(race, n, wr, wl, args.wall_margin)
        race = resample_closed(race, args.spacing)[0]
    race, race_tangent, kappa = resample_closed(race, args.spacing)
    kappa = ndimage.uniform_filter1d(kappa, size=5, mode='wrap')

    ds = np.mean(np.linalg.norm(np.diff(np.vstack([race, race[:1]]), axis=0), axis=1))
    v, ax = speed_profile(kappa, ds, vehicle['max_speed'], args.a_lat,
                          vehicle['max_acceleration'], vehicle['max_acceleration'])

    out_dir = args.out_dir or os.path.dirname(os.path.abspath(map_yaml))
    os.makedirs(out_dir, exist_ok=True)
    prefix = os.path.join(out_dir, os.path.splitext(os.path.basename(map_yaml))[0])
    print(f'raceline: {len(race) * ds:.1f} m, speed {v.min():.2f}-{v.max():.2f} m/s, '
          f'est. lap time {np.sum(ds / np.maximum(v, 0.05)):.1f} s')
    save_outputs(prefix, center, w_right, w_left, race, race_tangent, kappa, v, ax, grid, start, args.show)


if __name__ == '__main__':
    main()
