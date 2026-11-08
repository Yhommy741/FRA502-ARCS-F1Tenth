from dataclasses import dataclass, field

import numpy as np

from arcsf1tenth_planning.frenet.frenet_path import FrenetPath, wrap_angle


@dataclass
class PlannerParams:
    horizon: float = 12.0                  # [m] planned distance ahead
    step: float = 0.1                      # [m] sample spacing along s
    maneuver_lengths: tuple = (3.0, 4.0, 5.0, 6.0, 8.0)  # [m] candidate L
    min_maneuver_time: float = 0.8         # [s] L >= speed * this (no violent swerves at speed)
    d_step: float = 0.1                    # [m] spacing of candidate d_target
    half_width: float = 0.146              # [m] car half width (vehicle.yaml footprint_width / 2)
    car_front: float = 0.381               # [m] rear axle -> front bumper
    car_rear: float = 0.075                # [m] rear axle -> rear end
    wall_margin: float = 0.15              # [m] keep this from the walls
    obstacle_margin: float = 0.25          # [m] keep this from obstacles (sideways and along s)
    clearance_preferred: float = 0.5       # [m] clearance below this costs extra
    max_curvature: float = 1.7             # [1/m] tan(max_steering) / wheelbase, with margin
    max_lateral_accel: float = 4.0         # [m/s^2] speed cap in curves
    speed_max: float = 3.0                 # [m/s]
    avoid_speed_max: float = 2.0           # [m/s] speed cap while off the global path
    w_jerk: float = 1.0
    w_offset: float = 2.0
    w_length: float = 1.0
    w_consistency: float = 1.0             # < w_offset, or the car stays offset after passing
    w_clearance: float = 20.0


@dataclass
class PlanResult:
    ok: bool
    d_target: float = 0.0
    s: np.ndarray = field(default_factory=lambda: np.zeros(0))   # global s (wrapped)
    d: np.ndarray = field(default_factory=lambda: np.zeros(0))
    dd: np.ndarray = field(default_factory=lambda: np.zeros(0))  # d' = dd/ds
    ddd: np.ndarray = field(default_factory=lambda: np.zeros(0))  # d''
    x: np.ndarray = field(default_factory=lambda: np.zeros(0))
    y: np.ndarray = field(default_factory=lambda: np.zeros(0))
    psi: np.ndarray = field(default_factory=lambda: np.zeros(0))
    kappa: np.ndarray = field(default_factory=lambda: np.zeros(0))
    v: np.ndarray = field(default_factory=lambda: np.zeros(0))
    candidates: int = 0
    feasible: int = 0
    reason: str = ''


# Return the quintic coefficients a0..a5 from (d, d', d'') at the start to (d1, 0, 0) at length L.
def quintic(d0, dd0, ddd0, d1, length):
    L = length
    a0, a1, a2 = d0, dd0, 0.5 * ddd0
    A = np.array([[L ** 3, L ** 4, L ** 5],
                  [3 * L ** 2, 4 * L ** 3, 5 * L ** 4],
                  [6 * L, 12 * L ** 2, 20 * L ** 3]])
    b = np.array([d1 - (a0 + a1 * L + a2 * L ** 2),
                  -(a1 + 2 * a2 * L),
                  -2 * a2])
    a3, a4, a5 = np.linalg.solve(A, b)
    return np.array([a0, a1, a2, a3, a4, a5])


# Evaluate a quintic and its first three derivatives at sigma.
def eval_quintic(c, sigma):
    d = c[0] + c[1] * sigma + c[2] * sigma ** 2 + c[3] * sigma ** 3 + c[4] * sigma ** 4 + c[5] * sigma ** 5
    d1 = c[1] + 2 * c[2] * sigma + 3 * c[3] * sigma ** 2 + 4 * c[4] * sigma ** 3 + 5 * c[5] * sigma ** 4
    d2 = 2 * c[2] + 6 * c[3] * sigma + 12 * c[4] * sigma ** 2 + 20 * c[5] * sigma ** 3
    d3 = 6 * c[3] + 24 * c[4] * sigma + 60 * c[5] * sigma ** 2
    return d, d1, d2, d3


# Sample lateral candidate paths, reject unsafe ones, and return the cheapest as a PlanResult.
def plan(path: FrenetPath, s0, d0, dd0, ddd0, speed, obstacles, params: PlannerParams,
         previous_d_target=None):
    p = params
    sigma = np.arange(0.0, p.horizon + 1e-9, p.step)          # distance ahead of the car
    s_abs = s0 + sigma                                        # unwrapped global s
    d_left = path.interp(path.d_left, s_abs)
    d_right = path.interp(path.d_right, s_abs)
    upper = d_left - p.half_width - p.wall_margin             # allowed d for the car center
    lower = -(d_right - p.half_width - p.wall_margin)

    # candidate targets: across the narrowest part of the track ahead (+ always the global path)
    hi, lo = float(np.min(upper[sigma <= p.horizon])), float(np.max(lower))
    if hi < lo:
        return PlanResult(False, reason='track narrower than the car + margins')
    targets = np.arange(np.ceil(lo / p.d_step) * p.d_step, hi + 1e-9, p.d_step)
    if lo <= 0.0 <= hi:
        targets = np.unique(np.concatenate([targets, [0.0]]))
    min_length = max(speed * p.min_maneuver_time, p.maneuver_lengths[0])
    lengths = [L for L in p.maneuver_lengths if L >= min_length - 1e-9] or [max(p.maneuver_lengths)]

    # obstacles relative to the car: (start, end) ahead in sigma, lateral interval
    obs = []
    for s_start, s_end, d_min, d_max in obstacles:
        a = path.ds(s_start, s0)
        b = a + (s_end - s_start) % path.length
        if b < -p.car_front - p.obstacle_margin or a > p.horizon + p.car_rear:
            continue
        obs.append((a - p.car_front - p.obstacle_margin, b + p.car_rear + p.obstacle_margin, d_min, d_max))

    best = None
    count = feasible = 0
    for L in lengths:
        in_maneuver = sigma <= L
        for target in targets:
            count += 1
            c = quintic(d0, dd0, ddd0, target, L)
            d, d1, d2, d3 = eval_quintic(c, np.minimum(sigma, L))
            d1 = np.where(in_maneuver, d1, 0.0)
            d2 = np.where(in_maneuver, d2, 0.0)
            d3 = np.where(in_maneuver, d3, 0.0)
            # 1. track bounds (skip the first metre: the car may already be outside, it must get back)
            check = sigma > 1.0
            if np.any(d[check] > upper[check]) or np.any(d[check] < lower[check]):
                continue
            # 2. obstacles: lateral gap between the car (d ± half width) and the obstacle interval
            clearance_cost = 0.0
            hit = False
            for a, b, d_min, d_max in obs:
                along = (sigma >= a) & (sigma <= b)
                if not along.any():
                    continue
                gap = np.maximum(d_min - (d[along] + p.half_width), (d[along] - p.half_width) - d_max)
                # the first metre starts where the car IS (it cannot move sideways instantly):
                # there only real contact is rejected, the full margin applies further ahead
                need = np.where(sigma[along] < 1.0, 0.0, p.obstacle_margin)
                if np.any(gap < need):
                    hit = True
                    break
                clearance_cost += float(np.sum(np.maximum(0.0, p.clearance_preferred - gap) ** 2)) * p.step
            if hit:
                continue
            # 3. curvature of the resulting path (Frenet -> Cartesian, small-angle form)
            k_ref = path.interp(path.kappa, s_abs)
            one = 1.0 - k_ref * d
            if np.any(one <= 0.1):
                continue
            theta = np.arctan2(d1, one)                         # heading relative to the path
            kappa = _curvature(k_ref, d, d1, d2)
            if np.any(np.abs(kappa[sigma > 0.5]) > p.max_curvature):
                continue
            feasible += 1
            jerk = float(np.sum(d3[in_maneuver] ** 2)) * p.step
            cost = (p.w_jerk * jerk + p.w_offset * target ** 2 + p.w_length / L
                    + p.w_clearance * clearance_cost)
            if previous_d_target is not None:
                cost += p.w_consistency * (target - previous_d_target) ** 2
            if best is None or cost < best[0]:
                best = (cost, target, d, d1, d2, kappa, theta)

    if best is None:
        return PlanResult(False, candidates=count, feasible=0,
                          reason='no collision-free path inside the track')
    _, target, d, d1, d2, kappa, theta = best
    x, y = path.to_cartesian(s_abs, d)
    psi = wrap_angle(path.heading_at(s_abs) + theta)
    v = np.minimum(path.interp(path.v, s_abs), p.speed_max)
    v = np.minimum(v, np.sqrt(p.max_lateral_accel / np.maximum(np.abs(kappa), 1e-3)))
    if np.max(np.abs(d)) > 0.05:
        v = np.minimum(v, p.avoid_speed_max)
    return PlanResult(True, float(target), path.wrap_s(s_abs), d, d1, d2, x, y, psi, kappa, v,
                      candidates=count, feasible=feasible)


# Return the curvature of the path offset by d from a reference with curvature k_ref.
def _curvature(k_ref, d, d1, d2):
    a = 1.0 - k_ref * d
    return (a * (a * k_ref + d2) + 2.0 * k_ref * d1 ** 2) / (a ** 2 + d1 ** 2) ** 1.5
