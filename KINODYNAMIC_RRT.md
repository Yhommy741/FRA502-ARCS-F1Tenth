# Kinodynamic RRT Obstacle Avoidance

This document explains how `non_controller` finds a drivable detour around obstacles that block the global path: how obstacles are detected, how the car is modelled, how the **kinodynamic RRT** grows a tree of drivable motions, and how the state machine decides when to use it. It follows the implementation in:

| File | Role |
|---|---|
| `arcsf1tenth_planning/arcsf1tenth_planning/rrt/obstacles.py` | Lidar points not in the saved map: wall test, clusters, size and track filters |
| `arcsf1tenth_planning/arcsf1tenth_planning/rrt/collision.py` | Car footprint (3 circles) against map walls and obstacle points |
| `arcsf1tenth_planning/arcsf1tenth_planning/rrt/kinodynamic_rrt.py` | Bicycle model, motion primitives, the RRT itself, speed profile |
| `arcsf1tenth_planning/arcsf1tenth_planning/rrt/avoidance.py` | One detour: lead-in, goal on the global path, RRT, merge back, speeds |
| `arcsf1tenth_planning/arcsf1tenth_planning/rrt/global_path.py` | Global path (centerline or raceline CSV): nearest point, windows, projection |
| `arcsf1tenth_planning/scripts/non_obstacle_detector.py` | Publishes the obstacle points of every scan |
| `arcsf1tenth_planning/scripts/non_rrt_node.py` | `rrt/plan` service: plans a detour on request |
| `arcsf1tenth_applications/scripts/non_state_machine.py` | Decides when the car follows the detour |

The library files (`rrt/`) use only numpy and scipy, so they can be tested without ROS (`test/test_kinodynamic_rrt.py`).

## 1. Pipeline

```
scan + map ──► non_obstacle_detector ──► rrt/obstacle_points (PointCloud2)
                                                 │
global path CSV ──┐                              ▼
TF, odometry ─────┴─────────────────► non_state_machine (10 Hz)  GLOBAL / AVOID / BLOCKED
                                          │            ▲
                    request: car state +  │            │  reply: detour with speeds
                    obstacle points       ▼            │
                                     non_rrt_node (rrt/plan service)
                                          │
                                          └──► rrt/tree, rrt/goal (RViz)

non_state_machine ──► rrt/trajectory ──► pure_pursuit ──► drive
```

Unlike a planner that runs every cycle, the RRT only runs **when the path ahead is blocked**. Once it finds a detour, the car **commits** to it until it is back on the global path. A new random tree every cycle would give a different, equally good path each time, and the car would zig-zag between them.

The controller (`pure_pursuit`) is shared with `yhom_controller`, so the two pipelines differ only in how they avoid obstacles.

## 2. Obstacle detection

An obstacle is anything the lidar sees that is **not a wall of the saved map**.

### 2.1 Scan to map frame

Beam $i$ with range $r_i$ and angle $\alpha_i$ is placed with the lidar pose $(x_l, y_l, \psi_l)$ **at the time of the scan**:

$$
\begin{aligned}
x_i &= x_l + r_i\cos(\psi_l + \alpha_i) \\
y_i &= y_l + r_i\sin(\psi_l + \alpha_i)
\end{aligned}
$$

The pose is looked up at the scan's timestamp, not the latest one. If the car turns at $\omega$ and the latest pose is $\Delta t$ newer than the scan, every point would be rotated by $\omega\,\Delta t$, which moves a point at range $r$ by about

$$
\Delta p \approx r\,\omega\,\Delta t
$$

In a corner ($\omega$ = 2 rad/s, $\Delta t$ = 50 ms) a wall 3 m away moves 0.3 m and would look like an obstacle. If the pose at the scan time is not available yet, the latest pose is used with a warning.

### 2.2 Wall test

The saved map is turned once into a distance field $D(x, y)$: the distance from every cell to the nearest occupied or unknown cell. A point is **not a wall** when

$$
D(x_i, y_i) > m_d + g\,r_i
$$

with $m_d$ = `detect_wall_margin` (0.3 m) and $g$ = `detect_range_gain` (0.035 m/m). The second term grows with range because a heading error $\delta\psi$ of the localization moves a point by $r\,\delta\psi$; 0.035 corresponds to about 2°. With $g = 0$ the rule is the fixed margin of `obstacle_detector.py`.

### 2.3 Clusters and filters

The remaining points are grouped in **scan order**: consecutive beams belong to the same cluster when their points are closer than `cluster_gap` (0.3 m). The first and last beam are neighbours on a 360° lidar. A cluster is dropped when:

- it has fewer than `min_points` (3) points: noise;
- it is wider than `max_size` (2 m), i.e. $2\max_j \lVert \mathbf{p}_j - \bar{\mathbf{p}} \rVert > 2$ m: usually a mislocalized wall;
- it lies **outside the track**. For every global path point, the free width to the left $w_L$ and right $w_R$ is measured along the normal in the map (as in `frenet_path.py`). With $d_j$ the signed lateral offset of the cluster points from the path point nearest to the cluster center, the cluster is dropped when

$$
\min_j d_j > w_L - m_t \quad\text{or}\quad \max_j d_j < -w_R + m_t
$$

with $m_t$ = `track_margin` (0.1 m). This removes objects seen through a gap in a wall.

The kept points are published as they are. The planner needs the real outline, not a circle around it, to find gaps.

## 3. Vehicle model

### 3.1 Kinematic bicycle in arc length

The RRT plans **geometry**: time is replaced by the distance travelled $s$ ($\mathrm{d}s = v\,\mathrm{d}t$). The state is

$$
q = (x,\ y,\ \psi,\ \delta)
$$

with $\delta$ the front steering angle, and the model is

$$
\begin{aligned}
\frac{\mathrm{d}x}{\mathrm{d}s} &= \cos\psi \\
\frac{\mathrm{d}y}{\mathrm{d}s} &= \sin\psi \\
\frac{\mathrm{d}\psi}{\mathrm{d}s} &= \frac{\tan\delta}{L} = \kappa \\
\frac{\mathrm{d}\delta}{\mathrm{d}s} &= \operatorname{clip}\!\left(\delta_{\text{cmd}} - \delta,\ \pm\frac{\dot\delta_{\max}}{v}\right)
\end{aligned}
$$

$L$ = 0.30 m is the wheelbase, and $(x, y)$ is the rear axle. The steering is part of the state because the servo turns at a limited rate: whether a hard right turn is possible depends on where the steering is now. At a higher planning speed $v$ the steering changes less per metre, so the paths are smoother.

Two limits follow. The usable steering is $\delta_{\max} = 0.9 \times 0.5236 = 0.471$ rad (`steer_margin`), so

$$
\kappa_{\max} = \frac{\tan\delta_{\max}}{L} \approx 1.70\ \text{m}^{-1} \quad\Rightarrow\quad R_{\min} \approx 0.59\ \text{m}
$$

and the usable steering rate is $\dot\delta_{\max} = 0.7 \times 1.0$ rad/s (`steer_rate_margin`), leaving the controller room for corrections.

### 3.2 Integration

Each edge is integrated in steps of $\Delta s$ = `integration_step` (0.05 m) with the **midpoint heading**:

$$
\begin{aligned}
\delta_{k+1} &= \delta_k + \operatorname{clip}\!\left(\delta_{\text{cmd}} - \delta_k,\ \pm\frac{\dot\delta_{\max}}{v}\Delta s\right) \\
\psi_{k+1} &= \psi_k + \frac{\tan\delta_{k+1}}{L}\Delta s \\
\bar\psi &= \tfrac{1}{2}(\psi_k + \psi_{k+1}) \\
x_{k+1} &= x_k + \cos\bar\psi\,\Delta s, \qquad y_{k+1} = y_k + \sin\bar\psi\,\Delta s
\end{aligned}
$$

On an arc of constant curvature the chord points exactly along the mean heading, so this is much more accurate than using $\psi_k$: over one 0.5 m edge at full steering the end point is 0.15 mm from the exact circle, against 20.6 mm for the plain Euler step.

## 4. The kinodynamic RRT

A basic RRT connects the nearest node to a random sample with a straight line, which a car cannot drive. A **kinodynamic** RRT instead grows the tree by **simulating the vehicle model** with a few steering commands, so every edge, and therefore every path, is drivable. It never solves for the controls that reach a given point exactly (a hard two-point boundary value problem): it simulates forward and keeps what lands closest.

One iteration:

```
1. sample     the goal (probability goal_bias) or a random point near the global path
2. node       the best node to grow from (distance + heading metric)
3. simulate   every reachable steering command for one edge (0.5 m)
4. collision  check the car footprint at every 0.05 m pose of every edge
5. keep       the collision-free edge whose end is closest to the sample
6. goal       if a pose of that edge is inside the goal gate, cut the edge there: a solution
7. add        the end of the edge as a new node
```

### 4.1 Sampling

With probability `goal_bias` the sample is the goal itself. Otherwise it is a random global path point between the car and 1 m past the goal, moved sideways by a uniform offset in $\pm$`sample_width` (1.2 m):

$$
\mathbf{p}_s = \mathbf{p}_j + u\,\mathbf{n}_j, \qquad j \sim \mathcal{U}\{i_{\text{car}},\dots,i_{\text{goal}} + 1\,\text{m}\},\quad u \sim \mathcal{U}(-1.2, 1.2)
$$

with $\mathbf{n}_j$ the left normal of the path. The sample is only a **direction**: the new node ends wherever the chosen edge ends. Because random samples already lie in a band along the path towards the goal, the band does most of the goal-directed work, and `goal_bias` changes little on a narrow track.

### 4.2 Node to grow from

Euclidean distance is a poor metric for a car: a node right next to the sample but facing the other way is useless, because the car only drives forward. The metric adds a heading term. For a sample $\mathbf{p}_s = (x_s, y_s)$ and node $k$, the bearing is how far the node would have to turn to face the sample:

$$
\beta_k = \left|\operatorname{wrap}\!\left(\operatorname{atan2}(y_s - y_k,\ x_s - x_k) - \psi_k\right)\right|
$$

$$
m_k =
\begin{cases}
\infty & \text{if } \beta_k > \dfrac{\pi}{2} \ \text{(sample behind the node: forward only)} \\[4pt]
\infty & \text{if } \text{fails}_k \ge \text{max\_fails} \ \text{(dead end, e.g. facing a wall)} \\[4pt]
\left\lVert \mathbf{p}_s - (x_k, y_k) \right\rVert + w_h\,\beta_k & \text{otherwise}
\end{cases}
$$

with $w_h$ = `w_heading` (1.0 m/rad). The node to grow from is $k^* = \arg\min_k m_k$; if every $m_k$ is infinite, the iteration adds nothing.

A node is **retired** after `max_fails` (3) extensions in which every edge collided. This stops the tree from spending samples on a node boxed in by an obstacle.

### 4.3 Motion primitives

The steering commands lie on a **grid** of `n_controls` (7) values:

$$
\delta_{\text{cmd}} \in \left\{ -\delta_{\max},\ \dots,\ +\delta_{\max} \right\},
\qquad \text{spacing} = \frac{2\delta_{\max}}{6} \approx 0.157\ \text{rad}
$$

From a node with steering $\delta$, only the commands the servo can **reach within one edge** are allowed:

$$
\left|\delta_{\text{cmd}} - \delta\right| \le \Delta\delta_{\text{edge}} = \frac{\dot\delta_{\max}}{v}\,\ell_{\text{edge}},
\qquad \ell_{\text{edge}} = 0.5\ \text{m}
$$

At $v$ = 1.5 m/s this is 0.233 rad, so usually **3 of the 7** commands are reachable (2 at full lock). Each edge is $\ell_{\text{edge}}$ = `edge_length` (0.5 m) long: a node and its child are always 0.5 m apart along the curve, except when the edge is cut at the goal. Long paths are chains of many edges.

The edges for every grid steering are computed once from the origin and only rotated and translated to each node, so extending a node costs one small matrix operation.

### 4.4 Collision check

The car outline (`footprint_front` 0.381 m, `footprint_rear` 0.075 m, `footprint_width` 0.292 m, measured from the rear axle) is covered by **3 circles** along its axis, each of radius

$$
\rho = \sqrt{\left(\tfrac{l}{6}\right)^2 + \left(\tfrac{w}{2}\right)^2} \approx 0.165\ \text{m}, \qquad l = l_f + l_r
$$

A pose is free when every circle center $\mathbf{c}_k$ satisfies

$$
D(\mathbf{c}_k) - \rho \ge m_w \qquad\text{and}\qquad \min_j \lVert \mathbf{c}_k - \mathbf{o}_j \rVert - \rho \ge m_o
$$

with $D$ the wall distance field, $\mathbf{o}_j$ the obstacle points (KD-tree), $m_w$ = `wall_margin` (0.05 m) and $m_o$ = `obstacle_margin` (0.3 m). An edge that collides at **any** of its 10 poses is discarded whole.

### 4.5 Goal gate

The goal $(x_g, y_g, \psi_g)$ is a point on the global path, and a node does not have to land on it exactly. A pose $(x, y, \psi)$ reaches the goal when

$$
\begin{aligned}
a &= (x - x_g)\cos\psi_g + (y - y_g)\sin\psi_g, & 0 &\le a \le \text{goal\_depth}\ (1.0\ \text{m}) \\
c &= -(x - x_g)\sin\psi_g + (y - y_g)\cos\psi_g, & \lvert c \rvert &< \text{goal\_tolerance}\ (0.4\ \text{m}) \\
& & \lvert \operatorname{wrap}(\psi - \psi_g) \rvert &< \text{goal\_yaw\_tolerance}\ (0.4\ \text{rad})
\end{aligned}
$$

i.e. inside a 1 m deep, 0.8 m wide box past the goal point, heading roughly along the path. Every 0.05 m pose of a new edge is tested, so an edge cannot jump over the 1 m deep gate.

### 4.6 Cost and refinement

$$
c_n = c_{\text{parent}} + \ell_n + w_s \sum_k \lvert \delta_k - \delta_{k-1} \rvert
$$

with $\ell_n$ the edge length and $w_s$ = `w_steer` (0.2 m/rad): shorter paths with less steering are cheaper. After the first solution the tree grows for `refine_iterations` (100) more iterations and keeps the cheapest goal node. Planning stops at that point, after `time_budget` (80 ms), or at `max_nodes` / `max_iterations`. The path is traced back through the parents.

## 5. One detour (`avoidance.py`)

### 5.1 Lead-in: plan from where the car will be

The car keeps driving while the RRT plans. The current steering is estimated from the yaw rate ($\dot\psi = v\tan\delta/L$):

$$
\delta_0 = \arctan\!\left(\frac{L\,\dot\psi}{v}\right) \qquad (v > 0.3\ \text{m/s, else } 0)
$$

and the model is propagated with $\delta_0$ for

$$
\ell_{\text{lead}} = v\left(t_{\text{budget}} + \frac{0.5}{f} + t_{\text{request}}\right) \approx v\,(0.08 + 0.05 + 0.005)\ \text{s}
$$

where $0.5/f$ is the average age of the data at the 10 Hz state machine rate and $t_{\text{request}}$ the service round trip (about 3 ms measured). The RRT starts at the end of this lead-in.

### 5.2 Goal on the global path

With $k_b$ the number of path points from the car to the first blocked pose, the goal index starts at

$$
k = \max\big(k_b + n(\text{goal\_after\_obstacle}),\ n(\text{min\_goal\_distance})\big)
$$

($n(\cdot)$ = number of path points covering a distance; 2 m and 4 m) and moves forward in 0.25 m steps until no obstacle touches the path within `goal_after_obstacle` before it or `merge_length` (3 m) after it, and the car fits there. It is capped at `max_goal_distance` (10 m). Obstacles close together, like a slalom, therefore get one detour around all of them instead of a merge into the next one.

### 5.3 Escape margin

If the RRT fails because the start itself is inside `obstacle_margin` (the car stopped close to an obstacle, or one appeared next to it), it plans again with `escape_margin` (0.05 m). Without this the car could never move again. The reply reports the margin the path keeps.

### 5.4 Merge back

The goal gate lets the RRT end up to 0.4 m beside the path. The last `merge_length` of the detour blends that offset $d_{\text{end}}$ back to zero with a smoothstep:

$$
d(u) = d_{\text{end}}\left(1 - u^2(3 - 2u)\right), \qquad u \in (0, 1]
$$

so the controller never sees a jump. The trajectory sent to the controller is **lead-in + RRT path + merge**.

### 5.5 Speed profile

The curvature of every pose is $\kappa = \tan\delta / L$. The speed is the global path speed, capped at `avoid_speed_max` (2 m/s), at the lateral acceleration limit, and at the acceleration and braking limits:

$$
v_i \le \sqrt{\frac{a_{\text{lat,max}}}{\lvert\kappa_i\rvert}}, \qquad v_{i+1}^2 \le v_i^2 + 2a\,\Delta s_i \ \ \text{(forward)}, \qquad v_i^2 \le v_{i+1}^2 + 2a\,\Delta s_i \ \ \text{(backward)}
$$

with $a_{\text{lat,max}}$ = 4 m/s² and $a$ = `max_acceleration` (3 m/s²).

## 6. When the detour is used (`non_state_machine`)

The state machine runs at 10 Hz. A pose of the global path is **blocked** when the car footprint placed there comes within `obstacle_margin` of an obstacle point. It checks the path from the car to

$$
\text{reach} = d_{\text{check}} + v\,t_{\text{check}} \quad (= 3\ \text{m} + v \cdot 1.5\ \text{s})
$$

| State | Follows | Switch when |
|---|---|---|
| `GLOBAL` | global path (1 m behind to 12 m ahead) | the path ahead is blocked → request a detour |
| `AVOID` | the committed detour | the car reaches its end → `GLOBAL`; car > `replan_distance` (0.4 m) off it → dropped; a new obstacle blocks it → request a new detour |
| `BLOCKED` | global path (or the old detour, mid-maneuver) with a braking profile | a detour is found → `AVOID`; obstacle gone → `GLOBAL`. A new request is sent every cycle |

The request is **asynchronous**: the state machine keeps running and publishes the braking trajectory while the RRT plans, then publishes the detour as soon as the reply arrives. Far from the obstacle the braking profile does not slow the car yet.

The braking profile stops the rear axle `stop_distance` (0.5 m) before the blocked pose:

$$
v = \min\!\left(v_{\text{path}},\ \sqrt{2\,a_{\text{brake}}\,\Delta s_{\text{stop}}}\right)
$$

with $a_{\text{brake}}$ = 2 m/s² and $\Delta s_{\text{stop}}$ the distance left to the stop point. In the middle of a detour, the car brakes along the detour, because the global path would steer back towards the obstacle.

Obstacle points older than `obstacle_timeout` (0.5 s) are ignored.

## 7. Parameters

All three nodes read `arcsf1tenth_planning/config/non_rrt.yaml`. The car size and limits come from `arcsf1tenth_description/config/vehicle.yaml`, and the controller from `arcsf1tenth_control/config/pure_pursuit.yaml` (`non_controller.launch.py` sets `speed_min` to 0.1 m/s so the braking profile is followed down to the stop).

| Symbol | Parameter | Default | Node |
|---|---|---|---|
| $m_d$ | `detect_wall_margin` | 0.3 m | detector |
| $g$ | `detect_range_gain` | 0.035 m/m | detector |
| — | `max_range` | 6.0 m | detector |
| — | `cluster_gap`, `min_points` | 0.3 m, 3 | detector |
| — | `max_size` | 2.0 m | detector |
| $m_t$ | `track_margin` | 0.1 m | detector |
| $m_o$ | `obstacle_margin` | 0.3 m | state machine, RRT |
| $m_w$ | `wall_margin` | 0.05 m | RRT |
| — | `escape_margin` | 0.05 m | RRT |
| — | `goal_after_obstacle`, `min_goal_distance`, `max_goal_distance` | 2, 4, 10 m | RRT |
| — | `sample_width` | 1.2 m | RRT |
| — | `goal_bias` | 0.1 | RRT |
| — | `steer_margin`, `steer_rate_margin` | 0.9, 0.7 | RRT |
| $\ell_{\text{edge}}$ | `edge_length` | 0.5 m | RRT |
| $\Delta s$ | `integration_step` | 0.05 m | RRT |
| — | `n_controls` | 7 | RRT |
| — | `goal_tolerance`, `goal_depth`, `goal_yaw_tolerance` | 0.4 m, 1.0 m, 0.4 rad | RRT |
| $w_h$, $w_s$ | `w_heading`, `w_steer` | 1.0, 0.2 m/rad | RRT |
| — | `max_fails` | 3 | RRT |
| — | `refine_iterations` | 100 | RRT |
| $t_{\text{budget}}$ | `time_budget` | 0.08 s | RRT |
| — | `merge_length` | 3.0 m | RRT |
| — | `avoid_speed_max` | 2.0 m/s | RRT |
| $a_{\text{lat,max}}$ | `max_lateral_accel` | 4.0 m/s² | RRT |
| $d_{\text{check}}$, $t_{\text{check}}$ | `check_distance`, `check_time` | 3.0 m, 1.5 s | state machine |
| — | `replan_distance` | 0.4 m | state machine |
| — | `stop_distance`, `brake_decel` | 0.5 m, 2.0 m/s² | state machine |
| $L$ | `wheelbase` | 0.300 m | vehicle |
| $\delta_{\max}$ | `max_steering_angle` | 0.5236 rad | vehicle |
| $\dot\delta_{\max}$ | `max_steering_speed` | 1.0 rad/s | vehicle |

## 8. Limitations

- **Forward only.** The model cannot reverse. If the car stops so close to an obstacle that no forward edge is free even with `escape_margin`, it stays `BLOCKED` until the obstacle moves. `stop_distance` (0.5 m) is short for a car with a 0.59 m turning radius; 1.2 to 1.5 m leaves room to steer around a wide obstacle.
- **Random, not optimal.** RRT finds *a* path, not the best one. With 100 refinement iterations the paths still switch between left and right steering about 12 times over 7 m. RRT\* (rewiring) would smooth them at a higher cost.
- **Fixed planning speed.** Speed is not part of the tree. The steering rate limit is computed for one speed, and the speed profile is added afterwards.
- **Static obstacles.** Obstacles are the points of the latest scan. A moving obstacle is handled by replanning when it blocks the detour, not by predicting where it will be.
- **Time budget is wall-clock time.** On a slow or hot computer fewer iterations fit into 80 ms, so plans fail more often, and the lead-in no longer matches where the car really is. Check the Gazebo real-time factor if planning looks wrong in simulation.
- **Grazing-angle clusters.** A long object seen at a shallow angle splits into several small clusters, so `max_size` does not catch every mislocalized wall.
- **Shared controller.** Pure pursuit follows the path through a lookahead point and can cut corners by up to about 0.2 m on a tight detour. `obstacle_margin` (0.3 m) is chosen with that in mind; a smaller margin plans more paths in narrow spots but leaves less room.
