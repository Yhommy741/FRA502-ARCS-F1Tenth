import math
import time
from dataclasses import dataclass, field

import numpy as np


# Wrap an angle (or array) to [-pi, pi].
def wrap_angle(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


@dataclass
class RRTParams:
    wheelbase: float = 0.30            # [m]
    max_steer: float = 0.47            # [rad] usable steering (a bit below the joint limit)
    steer_rate: float = 1.0            # [rad/s] servo slew rate
    speed: float = 1.5                 # [m/s] planning speed: turns steer_rate into rad per metre
    edge_length: float = 0.5           # [m] travel per edge
    integration_step: float = 0.05     # [m] collision-check / path sample spacing
    n_controls: int = 7                # steering commands tried per extension
    goal_bias: float = 0.3             # probability of sampling the goal
    goal_tolerance: float = 0.4        # [m] goal gate half width (sideways)
    goal_depth: float = 1.0            # [m] goal gate depth past the goal point
    goal_yaw_tolerance: float = 0.4    # [rad]
    w_heading: float = 1.0             # [m/rad] nearest-node metric: heading term
    w_steer: float = 0.2               # [m/rad] cost of steering changes
    max_fails: int = 3                 # node with this many fully blocked extensions is not extended again
    max_nodes: int = 2000
    max_iterations: int = 4000
    refine_iterations: int = 100       # iterations after the first solution (0 = stop at the first)
    time_budget: float = 0.08          # [s] hard limit per plan() call


@dataclass
class RRTResult:
    ok: bool
    reason: str = ''
    x: np.ndarray = field(default_factory=lambda: np.zeros(0))       # path samples, start first
    y: np.ndarray = field(default_factory=lambda: np.zeros(0))
    yaw: np.ndarray = field(default_factory=lambda: np.zeros(0))
    delta: np.ndarray = field(default_factory=lambda: np.zeros(0))
    cost: float = math.inf
    iterations: int = 0
    nodes: int = 0
    plan_time: float = 0.0
    edges: list = field(default_factory=list)   # (xs, ys) of every tree edge, for visualization


# Integrate the bicycle model from one state for K steering commands and return X, Y, YAW, DELTA.
def propagate(x, y, yaw, delta, delta_cmd, p: RRTParams, length=None):
    length = p.edge_length if length is None else length
    n = max(int(math.ceil(length / p.integration_step)), 1)
    ds = length / n
    max_change = p.steer_rate / max(p.speed, 0.1) * ds
    cmd = np.asarray(delta_cmd, dtype=float)
    k = len(cmd)
    out = np.empty((4, k, n))
    cx, cy = np.full(k, float(x)), np.full(k, float(y))
    cyaw, cdelta = np.full(k, float(yaw)), np.full(k, float(delta))
    for i in range(n):
        cdelta = cdelta + np.clip(cmd - cdelta, -max_change, max_change)
        new_yaw = cyaw + np.tan(cdelta) / p.wheelbase * ds
        mid = 0.5 * (cyaw + new_yaw)
        cx = cx + np.cos(mid) * ds
        cy = cy + np.sin(mid) * ds
        cyaw = new_yaw
        out[:, :, i] = cx, cy, cyaw, cdelta
    return out[0], out[1], out[2], out[3]


class MotionPrimitives:
    # Precompute the steering command grid and its servo-reachable range.
    def __init__(self, p: RRTParams):
        self.p = p
        self.grid = np.linspace(-p.max_steer, p.max_steer, p.n_controls)
        self.max_change = p.steer_rate / max(p.speed, 0.1) * p.edge_length
        self.table = [self._local(d0) for d0 in self.grid]

    # Return the command indices the servo can reach from steering angle delta within one edge.
    def reachable(self, delta):
        return np.flatnonzero(np.abs(self.grid - delta) <= self.max_change + 1e-9)

    # Return the reachable command indices around a steering angle that is not on the grid.
    def _local(self, delta):
        idx = self.reachable(delta)
        return (idx,) + propagate(0.0, 0.0, 0.0, delta, self.grid[idx], self.p)

    # Return the edges from a node: (command indices, X, Y, YAW, DELTA).
    def expand(self, x, y, yaw, delta, index=None):
        idx, lx, ly, lyaw, ld = self.table[index] if index is not None else self._local(delta)
        c, s = math.cos(yaw), math.sin(yaw)
        return idx, x + c * lx - s * ly, y + s * lx + c * ly, yaw + lyaw, ld


# Grow a kinodynamic RRT from start toward goal and return the best collision-free path.
def plan(start, goal, sample_fn, is_free, p: RRTParams, rng=None):
    rng = np.random.default_rng() if rng is None else rng
    t0 = time.perf_counter()
    gx, gy, gyaw = goal
    cos_g, sin_g = math.cos(gyaw), math.sin(gyaw)
    if not is_free(np.array([start[0]]), np.array([start[1]]), np.array([start[2]]))[0]:
        return RRTResult(ok=False, reason='start in collision', plan_time=time.perf_counter() - t0)

    cap = p.max_nodes
    X, Y, YAW, D = (np.empty(cap) for _ in range(4))
    cost = np.empty(cap)
    parent = np.full(cap, -1, dtype=int)
    fails = np.zeros(cap, dtype=int)             # failed extensions per node (dead ends get retired)
    steer_index = np.full(cap, -1, dtype=int)    # grid index of the steering (-1 = root, off grid)
    edges = [None] * cap                         # samples of the edge parent -> node
    X[0], Y[0], YAW[0], D[0] = start
    cost[0] = 0.0
    n = 1
    primitives = MotionPrimitives(p)

    best = -1
    iterations = 0
    stop_at = p.max_iterations
    while iterations < stop_at and n < cap and time.perf_counter() - t0 < p.time_budget:
        iterations += 1
        sx, sy = (gx, gy) if rng.random() < p.goal_bias else sample_fn(rng)

        # nearest node, heading-aware (the car only drives forward)
        dx, dy = sx - X[:n], sy - Y[:n]
        bearing = np.abs(wrap_angle(np.arctan2(dy, dx) - YAW[:n]))
        metric = np.hypot(dx, dy) + p.w_heading * bearing
        metric[(bearing > 0.5 * math.pi) | (fails[:n] >= p.max_fails)] = np.inf
        k = int(np.argmin(metric))
        if not np.isfinite(metric[k]):
            continue

        # extend: every control, keep the collision-free one ending closest to the sample
        cmd, ex, ey, eyaw, ed = primitives.expand(X[k], Y[k], YAW[k], D[k],
                                                  steer_index[k] if steer_index[k] >= 0 else None)
        ok = is_free(ex, ey, eyaw).all(axis=1)
        if not ok.any():
            fails[k] += 1
            if n == 1 and fails[0] >= p.max_fails:
                break                            # every edge from the start collides
            continue
        end = np.hypot(ex[:, -1] - sx, ey[:, -1] - sy)
        end[~ok] = np.inf
        c = int(np.argmin(end))
        xs, ys, yaws, ds = ex[c], ey[c], eyaw[c], ed[c]

        # goal reached somewhere along the edge? cut the edge there
        along = (xs - gx) * cos_g + (ys - gy) * sin_g
        across = -(xs - gx) * sin_g + (ys - gy) * cos_g
        at_goal = (along >= 0.0) & (along <= p.goal_depth) & (np.abs(across) < p.goal_tolerance) & \
                  (np.abs(wrap_angle(yaws - gyaw)) < p.goal_yaw_tolerance)
        if at_goal.any():
            m = int(np.argmax(at_goal)) + 1
            xs, ys, yaws, ds = xs[:m], ys[:m], yaws[:m], ds[:m]

        length = len(xs) * (p.edge_length / ex.shape[1])
        X[n], Y[n], YAW[n], D[n] = xs[-1], ys[-1], yaws[-1], ds[-1]
        parent[n] = k
        steer_index[n] = cmd[c]
        steer = np.abs(np.diff(np.concatenate([[D[k]], ds]))).sum()
        cost[n] = cost[k] + length + p.w_steer * steer
        edges[n] = (xs, ys, yaws, ds)
        if at_goal.any():
            fails[n] = p.max_fails               # solutions end here: do not grow past the goal
            if best < 0 or cost[n] < cost[best]:
                if best < 0:
                    stop_at = min(p.max_iterations, iterations + p.refine_iterations)
                best = n
        n += 1

    plan_time = time.perf_counter() - t0
    tree = [(edges[i][0], edges[i][1], X[parent[i]], Y[parent[i]]) for i in range(1, n)]
    if best < 0:
        reason = 'start boxed in' if n == 1 else f'goal not reached ({n} nodes, {iterations} iterations)'
        return RRTResult(ok=False, reason=reason,
                         iterations=iterations, nodes=n, plan_time=plan_time, edges=tree)

    chain = []
    i = best
    while i > 0:
        chain.append(edges[i])
        i = parent[i]
    chain.reverse()
    px = np.concatenate([[start[0]]] + [e[0] for e in chain])
    py = np.concatenate([[start[1]]] + [e[1] for e in chain])
    pyaw = np.concatenate([[start[2]]] + [e[2] for e in chain])
    pd = np.concatenate([[start[3]]] + [e[3] for e in chain])
    return RRTResult(ok=True, x=px, y=py, yaw=pyaw, delta=pd, cost=float(cost[best]),
                     iterations=iterations, nodes=n, plan_time=plan_time, edges=tree)


# Return the speed along a path, capped by lateral acceleration and limited by acceleration/braking.
def speed_profile(kappa, ds, v_max, a_lat, a_long, v_start, v_min=0.3):
    kappa = np.abs(np.asarray(kappa, dtype=float))
    v = np.minimum(np.broadcast_to(v_max, kappa.shape).astype(float),
                   np.sqrt(a_lat / np.maximum(kappa, 1e-6)))
    ds = np.broadcast_to(ds, (max(len(v) - 1, 0),))
    v[0] = min(v[0], max(v_start, v_min))
    for i in range(len(v) - 1):
        v[i + 1] = min(v[i + 1], math.sqrt(v[i] ** 2 + 2 * a_long * ds[i]))
    for i in range(len(v) - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * a_long * ds[i]))
    return np.maximum(v, v_min)
