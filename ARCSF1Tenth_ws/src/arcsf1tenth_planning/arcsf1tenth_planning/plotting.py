"""Plot an occupancy map with a centerline and a raceline (shared by the planning scripts)."""

import math
import os

import cv2
import numpy as np
import yaml


def load_map_image(map_yaml):
    """Return (grayscale image, [x_min, x_max, y_min, y_max]) of a map_server map."""
    with open(map_yaml) as f:
        meta = yaml.safe_load(f)
    image_path = os.path.join(os.path.dirname(os.path.abspath(map_yaml)), meta['image'])
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f'Cannot read map image {image_path}')
    x0, y0 = meta['origin'][:2]
    height, width = image.shape
    res = float(meta['resolution'])
    return image, [x0, x0 + width * res, y0, y0 + height * res]


def load_centerline(path):
    """x, y (+ widths) from a centerline CSV (x_m, y_m, w_tr_right_m, w_tr_left_m)."""
    return np.loadtxt(path, delimiter=',', comments='#', ndmin=2)


def load_raceline(path):
    """Columns s, x, y, psi, kappa, vx, ax from a raceline CSV (semicolon separated)."""
    return np.loadtxt(path, delimiter=';', comments='#', ndmin=2)


def plot_track(image, extent, centerline=None, raceline=None, start=None, title=''):
    """Figure with the map, the centerline (dashed) and the raceline coloured by speed.

    centerline: Nx2+ array (x, y, ...); raceline: Nx7 array (s, x, y, psi, kappa, vx, ax).
    """
    import matplotlib.pyplot as plt

    fig, axis = plt.subplots(figsize=(12, 9))
    axis.imshow(image, cmap='gray', extent=extent, origin='upper', alpha=0.8)
    shown = []

    if centerline is not None:
        loop = np.vstack([centerline[:, :2], centerline[:1, :2]])
        axis.plot(loop[:, 0], loop[:, 1], '--', color='tab:blue', lw=1, label='centerline')
        shown.append(centerline[:, :2])

    if raceline is not None:
        xy, speed = raceline[:, 1:3], raceline[:, 5]
        points = axis.scatter(xy[:, 0], xy[:, 1], c=speed, cmap='plasma', s=6, label='raceline')
        fig.colorbar(points, ax=axis, label='speed [m/s]')
        shown.append(xy)
        ds = np.linalg.norm(np.diff(np.vstack([xy, xy[:1]]), axis=0), axis=1)
        title += f'{": " if title else ""}lap {ds.sum():.1f} m, est. lap time {np.sum(ds / np.maximum(speed, 0.05)):.1f} s'

    if start is not None:
        x, y, yaw = start
        axis.annotate('', xy=(x + math.cos(yaw), y + math.sin(yaw)), xytext=(x, y),
                      arrowprops=dict(color='lime', width=3, headwidth=10))

    if shown:  # zoom to the track
        xy = np.vstack(shown)
        axis.set_xlim(xy[:, 0].min() - 2.0, xy[:, 0].max() + 2.0)
        axis.set_ylim(xy[:, 1].min() - 2.0, xy[:, 1].max() + 2.0)
        axis.legend(loc='upper right')
    axis.set_aspect('equal')
    axis.set_xlabel('x [m]')
    axis.set_ylabel('y [m]')
    axis.set_title(title.strip())
    return fig
