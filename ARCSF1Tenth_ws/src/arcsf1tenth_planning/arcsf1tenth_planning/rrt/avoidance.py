import math
from dataclasses import dataclass, field

import numpy as np

from arcsf1tenth_planning.rrt.kinodynamic_rrt import RRTResult, plan, propagate, speed_profile


@dataclass
class AvoidanceParams:
    goal_after_obstacle: float = 2.0   # [m] goal on the global path this far past the obstacle
    min_goal_distance: float = 4.0     # [m] and at least this far ahead of the car
    max_goal_distance: float = 10.0    # [m] goal pushed forward while blocked, up to this
    sample_width: float = 1.2          # [m] samples within +- this of the global path
    merge_length: float = 3.0          # [m] global path appended after the RRT goal
    avoid_speed_max: float = 2.0       # [m/s] on the RRT path
    min_plan_speed: float = 1.0        # [m/s] steering-rate limit is computed for at least this speed
    max_lateral_accel: float = 4.0     # [m/s^2]
    max_speed: float = 3.0             # [m/s] vehicle limit
    max_acceleration: float = 3.0      # [m/s^2] vehicle limit
    obstacle_margin: float = 0.3       # [m] car edge to obstacle point
    escape_margin: float = 0.05        # [m] used when the car is already inside obstacle_margin
    lead_time: float = 0.06            # [s] data age + request latency, added to the RRT time budget


@dataclass
class Avoidance:
    ok: bool
    reason: str = ''
    rrt: RRTResult = None
    goal: tuple = None
    margin: float = 0.0
    path: dict = field(default_factory=dict)   # x, y, yaw, kappa, v, s, d (arrays) and end (RRT part ends)


class AvoidancePlanner:
    # Store the global path, collision checker (map + obstacles) and planner parameters.
    def __init__(self, path, checker, rrt, params: AvoidanceParams, rng=None):
        self.path = path
        self.checker = checker
        self.rrt = rrt
        self.p = params
        self.rng = np.random.default_rng() if rng is None else rng

    # Plan a detour: from where the car will be when the plan is ready to a clear goal on the global path.
    # i = nearest global path index of the car, blocked = path points from i to the first blocked pose.
    def plan(self, x, y, yaw, v, yaw_rate, i, blocked):
        p, path, rrt = self.p, self.path, self.rrt
        rrt.speed = min(max(v, p.min_plan_speed), p.avoid_speed_max)

        # start where the car will be when the plan is ready (it keeps driving meanwhile)
        delta0 = math.atan(rrt.wheelbase * yaw_rate / v) if v > 0.3 else 0.0
        delta0 = max(-rrt.max_steer, min(rrt.max_steer, delta0))
        lead = v * (rrt.time_budget + p.lead_time)
        if lead > 2 * rrt.integration_step:
            px, py, pyaw, pd = (a[0] for a in propagate(x, y, yaw, delta0, [delta0], rrt, length=lead))
            px, py, pyaw, pd = np.r_[x, px], np.r_[y, py], np.r_[yaw, pyaw], np.r_[delta0, pd]
        else:
            px, py, pyaw, pd = np.array([x]), np.array([y]), np.array([yaw]), np.array([delta0])
        start = (px[-1], py[-1], pyaw[-1], pd[-1])

        # goal: on the global path past the obstacle, with a clear runway: no obstacle within
        # goal_after_obstacle before it or merge_length after it (obstacles close together, e.g. a
        # slalom, get one plan around all of them instead of a merge into the next one)
        before, after = path.count(p.goal_after_obstacle), path.count(p.merge_length)
        k_max = path.count(p.max_goal_distance)
        idx = path.window(i, 0.0, p.max_goal_distance + p.merge_length)
        hit = ~self.checker.obstacle_free(path.x[idx], path.y[idx], path.psi[idx])
        k = max(blocked + before, path.count(p.min_goal_distance))
        while k < k_max:
            g = idx[k]
            if not hit[max(k - before, 0):k + after + 1].any() and \
                    self.checker.free(path.x[[g]], path.y[[g]], path.psi[[g]])[0]:
                break
            k += path.count(0.25)
        k = min(k, k_max)
        g = idx[k]
        goal = (path.x[g], path.y[g], path.psi[g])

        k_end = k + path.count(1.0)
        width = p.sample_width

        # Sample a random point near the global path between the car and the goal.
        def sample(rng):
            j = (i + int(rng.integers(0, k_end))) % path.n
            return path.offset_point(j, rng.uniform(-width, width))

        margin = p.obstacle_margin
        result = plan(start, goal, sample, self.checker.free, rrt, self.rng)
        if not result.ok and result.reason in ('start in collision', 'start boxed in'):
            # the car is inside the safety margin (stopped close to an obstacle, or an obstacle
            # appeared next to it): plan with the smaller escape margin, or it never moves again
            margin = p.escape_margin
            self.checker.obstacle_margin = margin
            try:
                result = plan(start, goal, sample, self.checker.free, rrt, self.rng)
            finally:
                self.checker.obstacle_margin = p.obstacle_margin
        if not result.ok:
            return Avoidance(ok=False, reason=result.reason, rrt=result, goal=goal, margin=margin)

        # committed path = lead-in + RRT path + merge back onto the global path: the goal gate lets
        # the RRT end up to goal_tolerance beside it, so the merge blends that offset to 0
        # (smoothstep over merge_length) instead of jumping
        gi = path.nearest(result.x[-1], result.y[-1], g)
        merge = path.window(gi, 0.0, p.merge_length)[1:]
        nx, ny = -np.sin(path.psi[gi]), np.cos(path.psi[gi])
        d_end = (result.x[-1] - path.x[gi]) * nx + (result.y[-1] - path.y[gi]) * ny
        u = np.arange(1, len(merge) + 1) / len(merge)
        blend = d_end * (1.0 - u * u * (3.0 - 2.0 * u))
        mx, my = path.offset_point(merge, blend)
        xs = np.r_[px[:-1], result.x, mx]
        ys = np.r_[py[:-1], result.y, my]
        yaws = np.r_[pyaw[:-1], result.yaw, path.psi[merge]]
        kappa = np.r_[np.tan(pd[:-1]) / rrt.wheelbase, np.tan(result.delta) / rrt.wheelbase, path.kappa[merge]]
        if len(merge) > 2:     # merge heading / curvature from its geometry
            hx, hy = np.r_[result.x[-1], mx], np.r_[result.y[-1], my]
            heading = np.unwrap(np.arctan2(np.diff(hy), np.diff(hx)))
            yaws[-len(merge):] = heading
            kappa[-len(merge):] = np.gradient(heading) / np.maximum(np.hypot(np.diff(hx), np.diff(hy)), 1e-6)
        s, d, nearest = path.project(xs, ys, i, window=p.max_goal_distance + p.merge_length + 2.0)
        ds = np.hypot(np.diff(xs), np.diff(ys))
        v_ref = np.minimum(np.minimum(path.v[nearest], p.avoid_speed_max), p.max_speed)
        speed = speed_profile(kappa, ds, v_ref, p.max_lateral_accel, p.max_acceleration, v)
        out = dict(x=xs, y=ys, yaw=yaws, kappa=kappa, v=speed, s=s, d=d, end=len(px) - 1 + len(result.x) - 1)
        return Avoidance(ok=True, rrt=result, goal=goal, margin=margin, path=out)
