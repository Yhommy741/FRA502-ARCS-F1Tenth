"""Global reference path (closed loop) loaded from a TUM-format CSV.

CSV columns: s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2
(generate_raceline.py output: cartographer_map_centerline.csv / cartographer_map_raceline.csv).
Points are assumed roughly evenly spaced (generate_raceline.py writes 0.1 m spacing).
"""

import math

import numpy as np


def wrap_angle(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


class GlobalPath:
    def __init__(self, x, y, psi, v, kappa=None):
        self.x = np.asarray(x, dtype=float)
        self.y = np.asarray(y, dtype=float)
        self.psi = np.asarray(psi, dtype=float)
        self.v = np.asarray(v, dtype=float)
        self.kappa = np.zeros_like(self.x) if kappa is None else np.asarray(kappa, dtype=float)
        seg = np.hypot(np.diff(self.x, append=self.x[0]), np.diff(self.y, append=self.y[0]))
        self.s = np.concatenate([[0.0], np.cumsum(seg)[:-1]])
        self.length = float(seg.sum())
        self.n = len(self.x)
        self.step = self.length / self.n          # mean spacing [m]

    @classmethod
    def from_csv(cls, path):
        data = np.loadtxt(path, delimiter=';', comments='#', ndmin=2)
        if data.shape[1] < 6:
            raise RuntimeError(f'{path}: expected columns s; x; y; psi; kappa; vx; ...')
        return cls(data[:, 1], data[:, 2], data[:, 3], data[:, 5], data[:, 4])

    def count(self, distance):
        """Number of points covering distance [m]."""
        return max(int(math.ceil(distance / self.step)), 1)

    def nearest(self, x, y, hint=None, window=5.0):
        """Index of the nearest point; only within window [m] of hint (if given) so close sections don't jump."""
        if hint is None:
            idx = np.arange(self.n)
        else:
            k = self.count(window)
            idx = (hint + np.arange(-k, k + 1)) % self.n
        d2 = (self.x[idx] - x) ** 2 + (self.y[idx] - y) ** 2
        return int(idx[np.argmin(d2)])

    def window(self, i, behind, ahead):
        """Indices from behind [m] before point i to ahead [m] after it (wrapping)."""
        return (i + np.arange(-self.count(behind) if behind > 0 else 0, self.count(ahead) + 1)) % self.n

    def project(self, x, y, hint, window=8.0):
        """Global (s, d, index) of each point (x, y arrays), searching near index hint. d > 0 = left."""
        k = self.count(window)
        idx = (hint + np.arange(-k, k + 1)) % self.n
        dx = np.asarray(x)[:, None] - self.x[idx][None, :]
        dy = np.asarray(y)[:, None] - self.y[idx][None, :]
        j = idx[np.argmin(dx ** 2 + dy ** 2, axis=1)]
        ex, ey = np.asarray(x) - self.x[j], np.asarray(y) - self.y[j]
        d = -np.sin(self.psi[j]) * ex + np.cos(self.psi[j]) * ey
        return self.s[j], d, j

    def offset_point(self, i, d):
        """Point d [m] to the left of path point i."""
        return self.x[i] - np.sin(self.psi[i]) * d, self.y[i] + np.cos(self.psi[i]) * d
