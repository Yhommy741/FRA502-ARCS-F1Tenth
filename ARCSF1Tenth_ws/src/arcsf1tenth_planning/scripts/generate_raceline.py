#!/usr/bin/python3

import argparse
import math
import os

import cv2
import numpy as np
import yaml
from scipy import interpolate, ndimage, sparse
from scipy.sparse import csgraph
from scipy.sparse.linalg import splu
from scipy.spatial import cKDTree

from arcsf1tenth_planning.utils.tracks import resolve_map


class GridMap:
    # Load a map_server map as a grid with its resolution and origin.
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

    # Convert world (x, y) to pixel (col, row).
    def to_pixel(self, x, y):
        col = (np.asarray(x) - self.origin[0]) / self.resolution - 0.5
        row = self.height - 0.5 - (np.asarray(y) - self.origin[1]) / self.resolution
        return col, row

    # Convert pixel (col, row) to world (x, y).
    def to_world(self, col, row):
        x = self.origin[0] + (np.asarray(col) + 0.5) * self.resolution
        y = self.origin[1] + (self.height - np.asarray(row) - 0.5) * self.resolution
        return x, y

    # Return the map extent [x_min, x_max, y_min, y_max] for plotting.
    def extent(self):
        x0, y0 = self.origin
        return [x0, x0 + self.width * self.resolution, y0, y0 + self.height * self.resolution]


# Return the drivable cells: connected to the start, with small isolated specks filled.
def track_area(grid, start, min_hole_area):
    blocked, n = ndimage.label(~grid.free, structure=np.ones((3, 3)))  # 8-connected: diagonal walls stay one piece
    sizes = ndimage.sum(np.ones_like(blocked), blocked, index=np.arange(1, n + 1))
    border = set(np.unique(np.concatenate([blocked[0], blocked[-1], blocked[:, 0], blocked[:, -1]])))
    min_px = min_hole_area / grid.resolution ** 2
    is_wall = np.zeros(n + 1, bool)
    is_wall[1:] = sizes >= min_px
    is_wall[[label for label in border if label]] = True
    drivable = ~is_wall[blocked]

    col, row = (int(round(v)) for v in grid.to_pixel(*start[:2]))
    if not (0 <= row < grid.height and 0 <= col < grid.width) or not drivable[row, col]:
        raise RuntimeError(f'Start pose {start[:2]} is not in free space on the map (use --start X Y YAW)')
    labels, _ = ndimage.label(drivable)  # 4-connected: cannot leak diagonally through a wall
    return labels == labels[row, col]


# Return the main track loop as ordered world points, from the skeleton cut at the start pose.
def main_loop_pixels(area, grid, start, cut_half_width=1.5):
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


# Fit a periodic spline and resample it every `spacing` metres: (points, tangents, curvature).
def resample_closed(points, spacing, smoothing=0.0):
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


# Return the distance from each point to the right and left track edges.
def track_widths(points, tangent, area, grid, max_width=10.0):
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


# Return the lateral offsets that minimise curvature, solved as a box-constrained QP.
def minimum_curvature(points, normal, w_right, w_left, margin, max_iter=20000):
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


# Return the distance [m] from every track cell to the nearest wall.
def clearance_map(area, grid):
    return ndimage.distance_transform_edt(area) * grid.resolution


# Return points along the car outline in the car frame (rear axle origin, x forward).
def outline_points(front, rear, width, step=0.05):
    xs = np.linspace(-rear, front, max(2, int(math.ceil((front + rear) / step)) + 1))
    ys = np.linspace(-width / 2.0, width / 2.0, max(2, int(math.ceil(width / step)) + 1))
    return np.vstack([np.column_stack([xs, np.full_like(xs, -width / 2.0)]),
                      np.column_stack([xs, np.full_like(xs, width / 2.0)]),
                      np.column_stack([np.full_like(ys, -rear), ys]),
                      np.column_stack([np.full_like(ys, front), ys])])


# Return the smallest wall distance [m] of the car outline at every pose of the path.
def body_clearance(path, heading, outline, clearance, grid):
    c, s = np.cos(heading)[:, None], np.sin(heading)[:, None]
    x = path[:, 0:1] + c * outline[None, :, 0] - s * outline[None, :, 1]
    y = path[:, 1:2] + s * outline[None, :, 0] + c * outline[None, :, 1]
    col, row = grid.to_pixel(x, y)
    col, row = np.round(col).astype(int), np.round(row).astype(int)
    inside = (row >= 0) & (row < grid.height) & (col >= 0) & (col < grid.width)
    values = np.zeros(x.shape)
    values[inside] = clearance[row[inside], col[inside]]
    return values.min(axis=1)


# Return a minimum-curvature raceline whose car outline keeps `safety` from the walls.
def optimize_raceline(center, area, grid, clearance, outline, half_width, safety,
                      spacing, iterations, max_rounds=12):
    tree = cKDTree(center)
    margin = np.full(len(center), half_width + safety)
    window = max(1, int(round(0.6 / spacing)))   # widen the margin over +-0.6 m around a hit
    for round_ in range(max_rounds):
        race = center
        for _ in range(iterations):
            race_tangent = resample_closed(race, spacing)[1]
            wr, wl, normal = track_widths(race, race_tangent, area, grid)
            race = minimum_curvature(race, normal, wr, wl, margin[tree.query(race)[1]])
            race = resample_closed(race, spacing)[0]
        race, race_tangent, kappa = resample_closed(race, spacing)
        body = body_clearance(race, np.arctan2(race_tangent[:, 1], race_tangent[:, 0]), outline, clearance, grid)
        hits = body < safety - 1e-3
        if not hits.any():
            return race, race_tangent, kappa, body, round_
        for i, gap in zip(tree.query(race[hits])[1], safety - body[hits]):
            idx = np.arange(i - window, i + window + 1) % len(center)
            margin[idx] = np.maximum(margin[idx], margin[i] + gap + 0.02)
    print(f'warning: raceline outline still within {safety:.2f} m of a wall after {max_rounds} rounds '
          f'(closest {body.min():.3f} m): the track is too narrow there for this --safety')
    return race, race_tangent, kappa, body, max_rounds


# Return the speed profile: corner limit, then acceleration and braking passes on a closed loop.
def speed_profile(curvature, ds, max_speed, a_lat, a_accel, a_brake):
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


# Return rows s, x, y, psi, kappa, vx, ax (F1TENTH / TUM raceline format).
def line_table(points, tangent, kappa, v, ax):
    ds = np.linalg.norm(np.diff(np.vstack([points, points[:1]]), axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(ds)[:-1]])
    psi = np.arctan2(tangent[:, 1], tangent[:, 0])
    return np.column_stack([s, points, psi, kappa, v, ax])


# Write the centerline and raceline CSVs and the preview image.
def save_outputs(prefix, centerline, raceline, grid, start, show):
    header = 's_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2'
    for name, table in (('centerline', centerline), ('raceline', raceline)):
        np.savetxt(f'{prefix}_{name}.csv', table, delimiter='; ', fmt='%.7f', header=header)

    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from arcsf1tenth_planning.utils.plotting import plot_track
    fig = plot_track(grid.image, grid.extent(), centerline[:, 1:3], raceline, start, title=os.path.basename(prefix))
    fig.savefig(f'{prefix}_raceline.png', dpi=120, bbox_inches='tight')
    print(f'wrote {prefix}_centerline.csv, {prefix}_raceline.csv, {prefix}_raceline.png')
    if show:
        plt.show()


# Load the vehicle parameters from vehicle.yaml.
def load_vehicle(path):
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('arcsf1tenth_description'), 'config', 'vehicle.yaml')
    with open(path) as f:
        return yaml.safe_load(f)['/**']['ros__parameters']


# Generate the centerline and the minimum-curvature raceline of a track.
def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('map_yaml', help='map yaml (e.g. cartographer_map.yaml) or a track name (e.g. brands_hatch: '
                                         'its cartographer_map.yaml, else map.yaml)')
    parser.add_argument('--start', type=float, nargs=3, default=None, metavar=('X', 'Y', 'YAW'),
                        help='start pose in the map frame; YAW sets the driving direction '
                             '(default 0 0 0 = where SLAM mapping started; track name + map.yaml: track.yaml start_pose)')
    parser.add_argument('--out-dir', default=None, help='output folder (default: next to the map)')
    parser.add_argument('--spacing', type=float, default=0.1, help='[m] distance between points')
    parser.add_argument('--safety', type=float, default=0.10,
                        help='[m] minimum distance between the car outline and the walls')
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

    clearance = clearance_map(area, grid)
    outline = outline_points(vehicle['footprint_front'], vehicle['footprint_rear'], vehicle['footprint_width'])
    half_width = vehicle['footprint_width'] / 2.0

    # Compute the speed profile and return the rows of the line file.
    def finish(points, tangent, kappa):
        kappa = ndimage.uniform_filter1d(kappa, size=5, mode='wrap')
        ds = np.mean(np.linalg.norm(np.diff(np.vstack([points, points[:1]]), axis=0), axis=1))
        v, ax = speed_profile(kappa, ds, vehicle['max_speed'], args.a_lat,
                              vehicle['max_acceleration'], vehicle['max_acceleration'])
        return line_table(points, tangent, kappa, v, ax), ds

    # Centerline (fixed): only checked
    center_kappa = resample_closed(loop, args.spacing, args.smoothing)[2]
    center_body = body_clearance(center, np.arctan2(tangent[:, 1], tangent[:, 0]), outline, clearance, grid)
    centerline, center_ds = finish(center, tangent, center_kappa)

    # Raceline: minimum curvature, pushed away from walls where the car outline would hit
    race, race_tangent, race_kappa, race_body, rounds = optimize_raceline(
        center, area, grid, clearance, outline, half_width, args.safety, args.spacing, args.iterations)
    raceline, race_ds = finish(race, race_tangent, race_kappa)

    for name, table, ds, body in (('centerline', centerline, center_ds, center_body),
                                  ('raceline', raceline, race_ds, race_body)):
        v = table[:, 5]
        print(f'{name:<10}: {len(table) * ds:6.1f} m, speed {v.min():.2f}-{v.max():.2f} m/s, '
              f'est. lap {np.sum(ds / np.maximum(v, 0.05)):5.1f} s, closest car-to-wall {body.min():.3f} m'
              + ('' if body.min() >= args.safety - 1e-3 else f'  WARNING: below --safety {args.safety:.2f} m'))
    print(f'(raceline pushed off the walls in {rounds} extra round(s) for the car outline)' if rounds else
          '(raceline clear of the walls without adjustment)')

    out_dir = args.out_dir or os.path.dirname(os.path.abspath(map_yaml))
    os.makedirs(out_dir, exist_ok=True)
    prefix = os.path.join(out_dir, os.path.splitext(os.path.basename(map_yaml))[0])
    save_outputs(prefix, centerline, raceline, grid, start, args.show)


if __name__ == '__main__':
    main()
