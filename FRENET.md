# Frenet Frame and Frenet Optimal Obstacle Avoidance

This document explains how the car describes its position relative to the global path (the **Frenet frame**), and how the local planner uses that frame to find the best path around obstacles. It follows the implementation in:

| File | Role |
|---|---|
| `arcsf1tenth_planning/arcsf1tenth_planning/frenet/frenet_path.py` | Frenet frame of a closed path: projection, inverse, heading, curvature, track widths |
| `arcsf1tenth_planning/arcsf1tenth_planning/frenet/frenet_optimal.py` | Sampling planner: candidate paths, feasibility checks, cost, speed |
| `arcsf1tenth_planning/scripts/frenet_node.py` | Publishes the car's Frenet state and the global trajectory |
| `arcsf1tenth_planning/scripts/obstacle_detector.py` | Lidar obstacles as Frenet intervals |
| `arcsf1tenth_planning/scripts/yhom_local_planner.py` | Runs the planner at 10 Hz |
| `arcsf1tenth_applications/scripts/yhom_state_machine.py` | Decides when the car follows the local path |

## 1. Pipeline

```
global path CSV ─► frenet_node ──► global_trajectory (path + track widths)
map ─────────────┘            └──► frenet/state      (s, d, yaw error, speed)
scan ─► obstacle_detector ───────► obstacles         (s_start, s_end, d_min, d_max)
                    │
                    ▼
        yhom_local_planner (10 Hz) ──► yhom/local_trajectory
                    │
                    ▼
        yhom_state_machine  GLOBAL / AVOID / BLOCKED ──► yhom/trajectory ──► pure_pursuit ──► drive
```

The planner runs every cycle. With no obstacles its cheapest candidate is the global path itself ($d = 0$). The state machine switches to that local path only when an obstacle blocks the corridor ahead.

## 2. The Frenet frame

### 2.1 Reference path

The global path (centerline or raceline) is a closed polyline of $N$ points $\mathbf{p}_i = (x_i, y_i)$, where the last point connects back to the first. For each segment $i \to i+1$:

$$
\Delta x_i = x_{i+1} - x_i, \qquad \Delta y_i = y_{i+1} - y_i, \qquad \ell_i = \sqrt{\Delta x_i^2 + \Delta y_i^2}
$$

$$
\psi_i = \operatorname{atan2}(\Delta y_i, \Delta x_i)
$$

The arc length at point $i$ and the total length are

$$
s_i = \sum_{k=0}^{i-1} \ell_k, \qquad L_{\text{path}} = \sum_{k=0}^{N-1} \ell_k
$$

Arc length is periodic: every $s$ is wrapped into $[0, L_{\text{path}})$.

**Curvature** at point $i$ is the heading change divided by the mean length of the two neighbouring segments:

$$
\kappa_i = \frac{\operatorname{wrap}(\psi_i - \psi_{i-1})}{\tfrac{1}{2}(\ell_i + \ell_{i-1})}
$$

where $\operatorname{wrap}(\cdot)$ maps an angle into $[-\pi, \pi)$.

**Point heading** (used for smooth interpolation) is the mean of the segment headings before and after the point:

$$
\bar\psi_i = \psi_{i-1} + \tfrac{1}{2}\operatorname{wrap}(\psi_i - \psi_{i-1})
$$

### 2.2 Frenet coordinates

At arc length $s$, the reference point is $\mathbf{r}(s)$, the unit tangent is $\mathbf{t}(s) = (\cos\psi, \sin\psi)$ and the unit normal (pointing **left**) is $\mathbf{n}(s) = (-\sin\psi, \cos\psi)$.

Any point near the path is described by $(s, d)$:

$$
\mathbf{P}(s, d) = \mathbf{r}(s) + d\,\mathbf{n}(s)
$$

- $s$: distance travelled along the path.
- $d$: signed sideways offset, with $d > 0$ to the left and $d < 0$ to the right.

```
          d > 0 (left)
            ▲   ● car (s, d)
            │   ┆
 ───────────┼───●──────────────►  path, direction of s
            │   r(s)
          d < 0 (right)
```

### 2.3 Cartesian → Frenet (projection)

To find $(s, d)$ for a point $\mathbf{q} = (x, y)$, the point is projected onto every candidate segment $i$:

$$
\tau_i = \operatorname{clip}\!\left(\frac{(\mathbf{q} - \mathbf{p}_i)\cdot(\Delta x_i, \Delta y_i)}{\ell_i^2},\ 0,\ 1\right)
$$

$$
\mathbf{c}_i = (\mathbf{q} - \mathbf{p}_i) - \tau_i\,(\Delta x_i, \Delta y_i)
$$

The segment with the smallest $\lVert \mathbf{c}_i \rVert$ wins, and then

$$
s = s_i + \tau_i\,\ell_i, \qquad
d = \operatorname{sign}\!\big(\Delta x_i\,(y - y_i) - \Delta y_i\,(x - x_i)\big)\,\lVert \mathbf{c}_i \rVert
$$

The sign comes from the 2D cross product: positive when the point is to the left of the segment.

**Search window.** A race track can pass close to itself (for example a hairpin), so the nearest segment overall may belong to the wrong part of the track. After the first projection, only segments within `search_window` (5 m) of the previous segment are searched. If the car is ever more than `max_width + 1` m off the path (lost or teleported), the next projection searches the whole path again.

### 2.4 Frenet → Cartesian

The inverse uses the interpolated base point and the interpolated point heading:

$$
x = x_b - d\sin\psi(s), \qquad y = y_b + d\cos\psi(s)
$$

with $(x_b, y_b) = \mathbf{p}_i + \tau\,(\Delta x_i, \Delta y_i)$, and $\psi(s)$ interpolated linearly between $\bar\psi_i$ and $\bar\psi_{i+1}$.

### 2.5 Signed distance along the path

Because the path is closed, the distance from $s_{\text{from}}$ to $s_{\text{to}}$ is taken the short way round:

$$
\Delta s = \big((s_{\text{to}} - s_{\text{from}} + \tfrac{1}{2}L_{\text{path}}) \bmod L_{\text{path}}\big) - \tfrac{1}{2}L_{\text{path}}
\ \in \left(-\tfrac{L_{\text{path}}}{2}, \tfrac{L_{\text{path}}}{2}\right]
$$

### 2.6 Track widths

For each path point, the free space to the left ($d_{\text{left}}$) and to the right ($d_{\text{right}}$) is measured by stepping along $\pm\mathbf{n}$ through the occupancy grid until the first wall or unknown cell (up to `max_width` = 3 m). These widths turn the track walls into simple bounds on $d$.

### 2.7 Frenet state of the car

`frenet_node` projects the car pose (rear axle, `base_footprint`) at 50 Hz and publishes `frenet/state`. With speed $v$, heading error $\Delta\psi = \operatorname{wrap}(\psi_{\text{car}} - \psi(s))$ and path curvature $\kappa$:

$$
\dot s = \frac{v\cos\Delta\psi}{\max(1 - \kappa d,\ 0.1)}, \qquad \dot d = v\sin\Delta\psi
$$

The factor $1 - \kappa d$ says that a car on the inside of a bend ($\kappa d > 0$) covers more reference arc length per metre driven.

The planner uses the lateral offset as a function of **arc length** instead of time. Its derivatives are written with primes:

$$
d' = \frac{\mathrm{d}d}{\mathrm{d}s}, \qquad d'' = \frac{\mathrm{d}^2 d}{\mathrm{d}s^2}, \qquad d''' = \frac{\mathrm{d}^3 d}{\mathrm{d}s^3}
$$

Measured from the car, the slope is

$$
d' = (1 - \kappa d)\tan\Delta\psi
$$

## 3. Finding the best path around obstacles

The planner follows the sampling approach of Werling et al. (2010), *Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenét Frame*, in the arc-length domain. Every cycle it:

1. builds many candidate lateral paths $d(\sigma)$,
2. rejects the ones that leave the track, hit an obstacle or curve too tightly,
3. scores the rest with a cost function and keeps the cheapest,
4. converts the winner to a Cartesian trajectory with a speed profile.

Here $\sigma \in [0, H]$ is the distance ahead of the car ($s = s_0 + \sigma$), sampled every `step` = 0.1 m up to the horizon $H$ = 12 m.

### 3.1 Initial state

The new plan must start exactly where the car is, with the same offset, slope and curvature, or the steering would jump. The start state $(d_0, d'_0, d''_0)$ is taken:

- **from the previous plan** at the car's current $s$, when the car is within `replan_distance` (0.3 m) of it. This keeps successive plans continuous and stops them from flickering with localization noise.
- **from the measured state** otherwise: $d_0 = d$, $d'_0 = (1 - \kappa d)\tan\Delta\psi$, $d''_0 = 0$.

### 3.2 Candidate paths: quintic polynomials

Each candidate is defined by a **target offset** $d_T$ and a **maneuver length** $L$. Over the maneuver it is a fifth-order polynomial:

$$
d(\sigma) = a_0 + a_1\sigma + a_2\sigma^2 + a_3\sigma^3 + a_4\sigma^4 + a_5\sigma^5, \qquad 0 \le \sigma \le L
$$

It must match the car at the start and arrive at $d_T$ parallel to the path with zero curvature:

$$
d(0) = d_0, \quad d'(0) = d'_0, \quad d''(0) = d''_0, \qquad
d(L) = d_T, \quad d'(L) = 0, \quad d''(L) = 0
$$

The first three conditions give the low-order coefficients directly:

$$
a_0 = d_0, \qquad a_1 = d'_0, \qquad a_2 = \tfrac{1}{2}d''_0
$$

The last three give a $3 \times 3$ linear system for the rest:

$$
\begin{bmatrix}
L^3 & L^4 & L^5 \\
3L^2 & 4L^3 & 5L^4 \\
6L & 12L^2 & 20L^3
\end{bmatrix}
\begin{bmatrix} a_3 \\ a_4 \\ a_5 \end{bmatrix}
=
\begin{bmatrix}
d_T - (a_0 + a_1 L + a_2 L^2) \\
-(a_1 + 2a_2 L) \\
-2a_2
\end{bmatrix}
$$

After the maneuver ($\sigma > L$) the path holds the target: $d = d_T$ and $d' = d'' = d''' = 0$ until the end of the horizon.

```
 d ▲
   │            ______________ d_T
   │          /
   │ d_0 ____/   quintic over L
   └───────────────────────────────► σ
   0          L               H
```

A quintic is used because it is the lowest order that can satisfy six boundary conditions, and it is the curve that minimizes $\int (d''')^2\,\mathrm{d}\sigma$ (jerk) between them.

### 3.3 Sampling the candidates

**Allowed offsets.** The car's centre must keep half the car width $w/2$ plus `wall_margin` $m_w$ from each wall:

$$
d_{\text{up}}(\sigma) = d_{\text{left}}(\sigma) - \tfrac{w}{2} - m_w, \qquad
d_{\text{low}}(\sigma) = -\big(d_{\text{right}}(\sigma) - \tfrac{w}{2} - m_w\big)
$$

The target offsets are sampled every `d_step` = 0.1 m across the **narrowest** part of the track within the horizon:

$$
d_T \in \left[\ \max_\sigma d_{\text{low}}(\sigma),\ \min_\sigma d_{\text{up}}(\sigma)\ \right]
$$

$d_T = 0$ (the global path) is always added when it lies in that range. If the range is empty, the track is narrower than the car plus margins and planning fails.

**Maneuver lengths.** $L$ comes from `maneuver_lengths` = {3, 4, 5, 6, 8} m. Lengths that would make the swerve too fast for the current speed are dropped:

$$
L \ge \max\big(v \cdot T_{\min},\ L_{\min}\big)
$$

where $T_{\min}$ = `min_maneuver_time` = 0.8 s and $L_{\min}$ is the shortest length in the list. If no length is left, the longest one is used.

On a 2 m wide track this gives about 15 targets × 5 lengths ≈ 75 candidates per cycle.

### 3.4 Feasibility checks

A candidate is rejected as soon as any check fails.

**1. Track bounds.** For every sample with $\sigma > 1$ m:

$$
d_{\text{low}}(\sigma) \le d(\sigma) \le d_{\text{up}}(\sigma)
$$

The first metre is not checked, so a car that is already too close to a wall still has a way back.

**2. Obstacles.** Each obstacle from `obstacle_detector` is an interval $[s_{\text{start}}, s_{\text{end}}] \times [d_{\min}, d_{\max}]$ in the Frenet frame. Relative to the car, it starts at $\sigma_a = \Delta s(s_{\text{start}}, s_0)$ and ends at $\sigma_b$. Along $\sigma$, it is grown by the car's overhang (front bumper $l_f$ and rear end $l_r$, both measured from the rear axle) and by `obstacle_margin` $m_o$:

$$
\sigma \in \left[\ \sigma_a - l_f - m_o,\ \ \sigma_b + l_r + m_o\ \right]
$$

Inside that range, the sideways gap between the car's edges and the obstacle is

$$
g(\sigma) = \max\Big(d_{\min} - \big(d(\sigma) + \tfrac{w}{2}\big),\ \ \big(d(\sigma) - \tfrac{w}{2}\big) - d_{\max}\Big)
$$

The candidate is rejected if

$$
g(\sigma) < \begin{cases} 0 & \sigma < 1\ \text{m (only real contact)} \\ m_o & \text{otherwise} \end{cases}
$$

**3. Curvature.** An offset path curves differently from its reference: on the inside of a bend it curves tighter. With $a = 1 - \kappa_r d$, where $\kappa_r$ is the reference curvature, the path $\mathbf{P}(s) = \mathbf{r}(s) + d(s)\,\mathbf{n}(s)$ has (neglecting $\mathrm{d}\kappa_r/\mathrm{d}s$):

$$
\mathbf{P}' = a\,\mathbf{t} + d'\,\mathbf{n}, \qquad
\mathbf{P}'' = -2\kappa_r d'\,\mathbf{t} + (a\kappa_r + d'')\,\mathbf{n}
$$

$$
\kappa = \frac{\mathbf{P}' \times \mathbf{P}''}{\lVert \mathbf{P}' \rVert^3}
= \frac{a\,(a\kappa_r + d'') + 2\kappa_r d'^2}{\left(a^2 + d'^2\right)^{3/2}}
$$

The candidate is rejected if $a \le 0.1$ anywhere (the offset reaches the bend's centre of curvature), or if, for $\sigma > 0.5$ m,

$$
|\kappa(\sigma)| > \kappa_{\max} = 0.9\,\frac{\tan\delta_{\max}}{L_{wb}}
$$

where $\delta_{\max}$ is the maximum steering angle, $L_{wb}$ the wheelbase, and 0.9 a safety factor. For this car ($\delta_{\max}$ = 0.5236 rad, $L_{wb}$ = 0.30 m), $\kappa_{\max} \approx 1.73\ \text{m}^{-1}$, which is a minimum turning radius of about 0.58 m.

### 3.5 Cost function

Every feasible candidate gets a cost, and the cheapest one wins:

$$
J = w_{\text{jerk}} \int_0^{L} \big(d'''(\sigma)\big)^2\,\mathrm{d}\sigma
\;+\; w_{\text{offset}}\, d_T^2
\;+\; \frac{w_{\text{length}}}{L}
\;+\; w_{\text{cons}}\,\big(d_T - d_T^{\text{prev}}\big)^2
\;+\; w_{\text{clear}}\, C_{\text{clear}}
$$

with the clearance penalty

$$
C_{\text{clear}} = \sum_{\text{obstacles}} \int \max\big(0,\ c_{\text{pref}} - g(\sigma)\big)^2\,\mathrm{d}\sigma
$$

The integrals are evaluated as sums over the samples, multiplied by the step $\Delta\sigma$ = 0.1 m.

| Term | Weight | Effect |
|---|---|---|
| $\int (d''')^2$ | $w_{\text{jerk}}$ = 1 | smooth swerve, no sudden steering |
| $d_T^2$ | $w_{\text{offset}}$ = 2 | stay close to the global path |
| $1/L$ | $w_{\text{length}}$ = 1 | finish the maneuver sooner (against a lazy, very long swerve) |
| $(d_T - d_T^{\text{prev}})^2$ | $w_{\text{cons}}$ = 1 | keep the previous choice, no zig-zag between cycles |
| $C_{\text{clear}}$ | $w_{\text{clear}}$ = 20 | soft push away from obstacles, up to $c_{\text{pref}}$ = 0.5 m |

The terms pull against each other:

- **Jerk vs. length.** A short $L$ means a sharp swerve (high jerk), a long $L$ is penalized by $w_{\text{length}}/L$. Their balance sets how sharply the car swerves.
- **Offset vs. consistency.** $w_{\text{cons}}$ must stay **below** $w_{\text{offset}}$. Otherwise, after passing the obstacle, "keep the old target" costs less than "return to $d = 0$" and the car stays offset.
- **Hard vs. soft clearance.** `obstacle_margin` (0.25 m) is a hard limit that rejects candidates. `clearance_preferred` (0.5 m) is a soft preference, so the car leaves more room when the track allows it.

### 3.6 Speed profile and output

The winning path is converted back to Cartesian coordinates (section 2.4). Its heading is the path heading plus the angle of the offset path:

$$
\theta = \operatorname{atan2}\big(d',\ 1 - \kappa_r d\big), \qquad \psi = \psi_r + \theta
$$

The speed at each point is the global path's speed, capped by the vehicle limit and by the lateral acceleration in the **new** curvature:

$$
v(\sigma) = \min\!\left(v_{\text{ref}}(\sigma),\ v_{\max},\ \sqrt{\frac{a_{\text{lat,max}}}{|\kappa(\sigma)|}}\right)
$$

with $a_{\text{lat,max}}$ = 4 m/s². If the path leaves the global path at all ($\max|d| > 0.05$ m), the speed is also capped at `avoid_speed_max` = 2 m/s.

The result is published as `yhom/local_trajectory`. If **no** candidate passed the checks, an empty trajectory is published, and the state machine treats it as "no path": it brakes along the global path to stop 1 m before the obstacle.

## 4. When the local path is used

The planner always runs, but `yhom_state_machine` (20 Hz) decides which trajectory pure pursuit follows:

| State | Follows | Switch when |
|---|---|---|
| `GLOBAL` | global path | an obstacle blocks the corridor → `AVOID` if a local path exists, else `BLOCKED` |
| `AVOID` | `yhom/local_trajectory` | corridor clear **and** car back on the path ($\lvert d \rvert$ < 0.15 m) **and** at least 1 s in AVOID → `GLOBAL`; local path lost → `BLOCKED` |
| `BLOCKED` | global path with a braking profile | a local path exists for 0.5 s → `AVOID`; obstacle gone and car on the path → `GLOBAL` |

An obstacle **blocks the corridor** when it overlaps $\lvert d \rvert \le w/2 + 0.25$ m and is closer than

$$
\text{reach} = d_{\text{trigger}} + v \cdot t_{\text{trigger}} \quad (= 3\ \text{m} + v \cdot 1.5\ \text{s})
$$

The braking profile in `BLOCKED` is $v = \sqrt{2\,a_{\text{brake}}\,\Delta s_{\text{stop}}}$, with $a_{\text{brake}}$ = 2 m/s² and $\Delta s_{\text{stop}}$ the distance left to the stop point.

## 5. Parameters

Planner parameters are in `arcsf1tenth_planning/config/yhom_planner.yaml`, and state machine parameters in `arcsf1tenth_applications/config/yhom_state_machine.yaml`. The car size, steering limit and maximum speed come from `arcsf1tenth_description/config/vehicle.yaml`.

| Symbol | Parameter | Default |
|---|---|---|
| $H$ | `horizon` | 12.0 m |
| $\Delta\sigma$ | `step` | 0.1 m |
| $L$ | `maneuver_lengths` | 3, 4, 5, 6, 8 m |
| $T_{\min}$ | `min_maneuver_time` | 0.8 s |
| — | `d_step` | 0.1 m |
| $m_w$ | `wall_margin` | 0.15 m |
| $m_o$ | `obstacle_margin` | 0.25 m |
| $c_{\text{pref}}$ | `clearance_preferred` | 0.5 m |
| — | `replan_distance` | 0.3 m |
| $a_{\text{lat,max}}$ | `max_lateral_accel` | 4.0 m/s² |
| — | `avoid_speed_max` | 2.0 m/s |
| $w$ | `footprint_width` | 0.292 m |
| $l_f$, $l_r$ | `footprint_front`, `footprint_rear` | 0.381 m, 0.075 m |
| $L_{wb}$ | `wheelbase` | 0.300 m |
| $\delta_{\max}$ | `max_steering_angle` | 0.5236 rad |

## 6. Limitations

- **Only the lateral motion is sampled.** Speed is computed afterwards from the curvature. The full Werling method also samples longitudinal profiles over time.
- **Obstacles are static.** Each one is a fixed box in $(s, d)$, refreshed every scan (0.1 s). A moving obstacle is handled by replanning, not by predicting where it will be.
- **Conservative bounds.** The target range is set by the narrowest point within 12 m, so a narrow spot far ahead limits the choices near the car.
- **Small-angle conversion.** The curvature formula neglects $\mathrm{d}\kappa_r/\mathrm{d}s$, which is a good approximation on a smooth raceline or centerline.
