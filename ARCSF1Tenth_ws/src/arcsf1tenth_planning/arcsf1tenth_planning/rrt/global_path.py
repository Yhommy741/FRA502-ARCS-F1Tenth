import math

import numpy as np


# Wrap an angle (or array) to [-pi, pi].
def wrap_angle(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


class GlobalPath:
    # Store the path points and precompute their arc length.
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

    # Load a global path from a generate_raceline.py CSV.
    @classmethod
    def from_csv(cls, path):
        data = np.loadtxt(path, delimiter=';', comments='#', ndmin=2)
        if data.shape[1] < 6:
            raise RuntimeError(f'{path}: expected columns s; x; y; psi; kappa; vx; ...')
        return cls(data[:, 1], data[:, 2], data[:, 3], data[:, 5], data[:, 4])

    # Return the number of points covering a distance [m].
    def count(self, distance):
        return max(int(math.ceil(distance / self.step)), 1)

    # Return the nearest point index, searched only within window [m] of hint if given.
    def nearest(self, x, y, hint=None, window=5.0):
        if hint is None:
            idx = np.arange(self.n)
        else:
            k = self.count(window)
            idx = (hint + np.arange(-k, k + 1)) % self.n
        d2 = (self.x[idx] - x) ** 2 + (self.y[idx] - y) ** 2
        return int(idx[np.argmin(d2)])

    # Return the indices from behind [m] before point i to ahead [m] after it (wrapping).
    def window(self, i, behind, ahead):
        return (i + np.arange(-self.count(behind) if behind > 0 else 0, self.count(ahead) + 1)) % self.n

    # Return the global (s, d, index) of each point, searching near hint (d > 0 = left).
    def project(self, x, y, hint, window=8.0):
        k = self.count(window)
        idx = (hint + np.arange(-k, k + 1)) % self.n
        dx = np.asarray(x)[:, None] - self.x[idx][None, :]
        dy = np.asarray(y)[:, None] - self.y[idx][None, :]
        j = idx[np.argmin(dx ** 2 + dy ** 2, axis=1)]
        ex, ey = np.asarray(x) - self.x[j], np.asarray(y) - self.y[j]
        d = -np.sin(self.psi[j]) * ex + np.cos(self.psi[j]) * ey
        return self.s[j], d, j

    # Return the point d [m] to the left of path point i.
    def offset_point(self, i, d):
        return self.x[i] - np.sin(self.psi[i]) * d, self.y[i] + np.cos(self.psi[i]) * d
