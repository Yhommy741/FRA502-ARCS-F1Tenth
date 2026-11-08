# FRA502-ARCS-F1Tenth

ROS 2 Jazzy + Gazebo Harmonic simulation of the ARCS F1Tenth car: keyboard teleop,
wheel/IMU odometry fused by a robot_localization EKF, lidar mapping (Cartographer), localization (AMCL),
raceline generation, wall following, pure pursuit, click-to-place obstacles and Frenet optimal
obstacle avoidance. A real-car bringup template uses the same interfaces.

## Packages (`ARCSF1Tenth_ws/src`)

| Package | Contents |
|---|---|
| `arcsf1tenth_description` | URDF/meshes, `config/vehicle.yaml` (geometry, limits, footprint), `config/sensors.yaml` (IMU BNO055, lidar RPLidar C1) |
| `arcsf1tenth_gazebo` | Gazebo model plugins, bridge, simulated drivers: `gazebo_drive`, `gazebo_imu`, `gazebo_lidar`, `gazebo_wheel_odometry`, `map_to_world.py` |
| `arcsf1tenth_localization` | `imu_filter.py` (gyro bias calibration + ZUPT), `save_map.py`, `scan_for_slam.py`, `localization_error.py`, `config/ekf.yaml` (robot_localization), `config/imu_filter.yaml`, `config/cartographer_2d.lua` (mapping), `config/amcl.yaml` (localization) |
| `arcsf1tenth_planning` | Offline: `generate_raceline.py`, `show_raceline.py`. Online (shared): `frenet_node.py` (Frenet frame, track widths), `obstacle_detector.py`. yhom: `yhom_local_planner.py` (Frenet optimal), `yhom_state_machine.py`. non: `non_rrt_planner.py` (kinodynamic RRT), `non_rrt_tracker.py`. Libraries `frenet/` (`frenet_path.py`, `frenet_optimal.py`), `rrt/` (`kinodynamic_rrt.py`, `collision.py`, `global_path.py`), `utils/` (`conversions.py`, `plotting.py`, `tracks.py`); `config/frenet.yaml`, `config/yhom_planner.yaml`, `config/non_rrt.yaml` |
| `arcsf1tenth_interfaces` | Messages: `FrenetState`, `Waypoint`, `Trajectory`, `Obstacle`, `ObstacleArray` |
| `arcsf1tenth_teleop` | `keyboard.py` |
| `arcsf1tenth_tracks` | Tracks: `f1tenth_tracks/` (F1TENTH tracks), `lab_tracks/` (our own) |
| `arcsf1tenth_bringup` | Launch files and RViz configs |
| `arcsf1tenth_applications` | `obstacle_spawner.py` (click in RViz to add/remove obstacles in Gazebo) |
| `arcsf1tenth_control` | `wall_follow.py` (PID wall follower), `pure_pursuit.py` (raceline follower), `config/` |

## Install

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-ackermann-msgs \
    ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-localization ros-jazzy-cartographer-ros \
    ros-jazzy-nav2-amcl ros-jazzy-nav2-map-server ros-jazzy-nav2-lifecycle-manager \
    python3-opencv python3-scipy python3-matplotlib
```
(or the whole Nav2 stack: `ros-jazzy-navigation2 ros-jazzy-nav2-bringup`)

## Build

```bash
cd ARCSF1Tenth_ws
colcon build
source install/setup.bash
```

Run `source install/setup.bash` in every new terminal, and again after adding a new package
(otherwise e.g. `ModuleNotFoundError: No module named 'arcsf1tenth_interfaces'`).

## Run

`<map_name>` in the commands below is a placeholder: replace it with a track folder name from
`arcsf1tenth_tracks` (e.g. `levine`, `brands_hatch`, `austin`, `monza`, or `empty` for the simulation).

### View the robot model (RViz only)
```bash
ros2 launch arcsf1tenth_bringup display.launch.py
```

### Simulation
```bash
ros2 launch arcsf1tenth_bringup simulation.launch.py track:=<map_name>
```

| Argument | Default | Meaning |
|---|---|---|
| `track` | `empty` | track folder in `arcsf1tenth_tracks` (e.g. `levine`, `spielberg`, `monza`) |
| `namespace` | `arcsf1tenth` | topics `/<namespace>/...`, TF frames `<namespace>/...` |
| `z` | `0.1` | spawn height [m] |
| `slam` | `off` | `off`, `mapping` (Cartographer builds a map) or `localization` (AMCL on the saved map) |
| `slam_map` | `<map_name>/maps/sim/cartographer_map.yaml` | saved map for `slam:=localization` |
| `use_ekf` | `true` | robot_localization EKF publishes `odometry/estimated` and the odom TF (`false`: wheel odometry owns the TF) |
| `use_imu` | `true` | fuse the IMU yaw rate in the EKF |
| `ekf_config` | `arcsf1tenth_localization/config/ekf.yaml` | EKF tuning file (all noise parameters, commented) |
| `follow_camera` | `true` | Gazebo camera follows the car |

A wrong track name prints the list of available tracks.

### Real car (template)
```bash
ros2 launch arcsf1tenth_bringup real.launch.py                                        # drivers + EKF only
ros2 launch arcsf1tenth_bringup real.launch.py track:=<map_name> slam:=mapping
ros2 launch arcsf1tenth_bringup real.launch.py track:=<map_name> slam:=localization
```
Same topics as the simulation, no Gazebo and no `world` frame. The hardware drivers (RPLidar, BNO055,
VESC, steering MCU over micro-ROS) are still `TODO` blocks in the launch file. Real maps live in
`<map_name>/maps/real/`. Extra argument: `rviz:=false` for headless.

### Drive with the keyboard (second terminal)
```bash
ros2 run arcsf1tenth_teleop keyboard.py
```
`w`/`s` speed up/down (stays set), hold `a`/`d` to steer (release = straight), `space` stop, `q` quit.
For another namespace: `--ros-args -p drive_topic:=/<namespace>/drive`.

## Wall following (second terminal, simulation running)
```bash
ros2 launch arcsf1tenth_bringup wall_follow.launch.py
```
PID on the distance to the wall (lidar beams at 90 deg and 45 deg on that side, predicted
`lookahead` m ahead) -> steering; speed drops in turns and when the way ahead is blocked.
Arguments: `side:=left|right`, `desired_distance:=<m>`, `speed_max:=<m/s>`, `namespace:=arcsf1tenth`,
`use_sim_time:=false` on the real car. Defaults, gains and the rest are in
`arcsf1tenth_control/config/wall_follow.yaml`. Ctrl-C stops the car.
Don't run the keyboard teleop at the same time (both publish `drive`).

## Obstacles (simulation)
Click in RViz to place static obstacles (started by `simulation.launch.py`, node `obstacle_spawner.py`
in `arcsf1tenth_applications`):
1. Pick **Publish Point** in the RViz toolbar (shortcut `P`).
2. Click on the track: an orange 0.4 m box appears in Gazebo there.
3. Click on an obstacle (within 0.5 m) to remove it. Remove all:
   ```bash
   ros2 service call /arcsf1tenth/clear_obstacles std_srvs/srv/Trigger
   ```
The tool is single-click: pick it again for the next obstacle. Obstacles are static, solid and seen
by the lidar, but they are not in the saved map: `yhom_controller` (Frenet optimal) and
`non_controller` (kinodynamic RRT) detect and avoid them.
Shape and size: node parameters `shape` (`box` | `cylinder`), `size`, `height`, `remove_radius`.

## Controllers (one launch file per group member)
Each member has an own controller launch, so controllers and planning can be changed and compared
without touching the other one:

| Launch | Owner | Now |
|---|---|---|
| `yhom_controller.launch.py` | yhom | state machine: centerline, or Frenet optimal path around obstacles; pure pursuit |
| `non_controller.launch.py` | non | centerline, or kinodynamic RRT path around obstacles ; own tracker (rear wheel feedback) |

Needs a centerline for the track (see [Raceline](#raceline)) and `slam:=localization`. In a second
terminal, with the simulation running:
```bash
ros2 launch arcsf1tenth_bringup simulation.launch.py track:=<map_name> slam:=localization
ros2 launch arcsf1tenth_bringup yhom_controller.launch.py track:=<map_name>   # or non_controller.launch.py
```
Run only one controller (or the keyboard teleop) at a time: they all publish `drive`.

### yhom: obstacle avoidance
```
centerline CSV + map ─► frenet_node ─────── global_trajectory, frenet/state      (shared)
scan + map ──────────► obstacle_detector ── obstacles (Frenet s/d extent)         (shared)
                       yhom_local_planner ─ yhom/local_trajectory  (Frenet optimal path)
                       yhom_state_machine ─ yhom/trajectory ─► pure_pursuit ─► drive
```
- **Frenet frame** (`frenet_node`): `s` = distance along the centerline, `d` = sideways offset
  (left positive). Also measures the free track width left/right of the centerline from the map.
- **Obstacle detection** (`obstacle_detector`): lidar points not on a wall of the saved map
  (further than `wall_margin` = 0.3 m from it), clustered and converted to Frenet. An obstacle
  closer than that to a wall is not seen.
- **Local planner** (`yhom_local_planner`, 10 Hz): samples lateral paths d(s) (quintic from the car
  to a target offset over 3-8 m, then hold), rejects those leaving the track, hitting an obstacle
  (car outline + `obstacle_margin`) or too curvy to steer, picks the cheapest
  (smoothness + offset + clearance + consistency).
- **State machine** (`yhom_state_machine`, 20 Hz):

  | State | Car follows | Leaves when |
  |---|---|---|
  | `GLOBAL` | centerline | obstacle on the centerline ahead → `AVOID` (path found) or `BLOCKED` |
  | `AVOID` | local planner path | obstacle passed and car back on the centerline → `GLOBAL`; no path → `BLOCKED` |
  | `BLOCKED` | centerline, braking to a stop 1 m before the obstacle | a path appears → `AVOID`; obstacle gone → `GLOBAL` |

RViz shows the state above the car, the local path (orange), the detected obstacles (red) and the
track bounds (blue). Topics: `/arcsf1tenth/yhom/state`, `yhom/trajectory`, `yhom/local_trajectory`.
Tuning: `arcsf1tenth_planning/config/yhom_planner.yaml` (planner and state machine, commented),
`arcsf1tenth_planning/config/frenet.yaml` (Frenet frame, obstacle detection),
`arcsf1tenth_control/config/pure_pursuit.yaml` (path following). Argument `line:=raceline` uses the
raceline as the global path.

### non: kinodynamic RRT obstacle avoidance
```bash
ros2 launch arcsf1tenth_bringup simulation.launch.py track:=<map_name> slam:=localization
ros2 launch arcsf1tenth_bringup non_controller.launch.py track:=<map_name>
```
Self-contained: uses none of the `yhom`/shared nodes (own detection, planner, tracker).
```
map + scan + TF + odometry + centerline CSV ─► non_rrt_planner ─ rrt/trajectory ─► non_rrt_tracker ─► drive
```
- **Obstacles**: lidar points further than `detect_wall_margin` (0.3 m) from every wall of the saved map,
  isolated points dropped. Collision check: car outline as 3 circles, `wall_margin` to walls,
  `obstacle_margin` to obstacle points.
- **Kinodynamic RRT** (`kinodynamic_rrt.py`): state (x, y, yaw, steering), kinematic bicycle model,
  forward only. Edges are 0.5 m motion primitives: steering commands on a grid, reachable within the
  servo slew rate (`max_steering_speed` x `steer_rate_margin`) at the planning speed, so every path
  is drivable. Samples near the centerline, heading-aware nearest node, dead-end nodes retired,
  goal = a gate on the centerline past the obstacle(s) with a clear runway. ~20-40 ms per plan.
- **Modes** (`rrt/state`): `GLOBAL` follow the centerline; `AVOID` follow the committed RRT path
  until it merges back (replan if a new obstacle blocks it or the car is > 0.4 m off it);
  `BLOCKED` no path: brake to a stop 0.5 m before the obstacle and keep trying.
- **Tracker** (`non_rrt_tracker`): rear wheel feedback (path curvature feedforward + lateral/heading
  feedback, speed-scheduled gains), or `controller:=pure_pursuit` for comparison.

RViz (add as Marker / Path): `rrt/tree` (search tree), `rrt/path`, `rrt/goal`, `rrt/obstacle_points`,
`rrt/state_marker`. Arguments: `line:=centerline|raceline`, `speed_scale:=<fraction>`.
Code: `arcsf1tenth_planning/scripts/non_rrt_planner.py`, `non_rrt_tracker.py`, library
`arcsf1tenth_planning/arcsf1tenth_planning/rrt/`. Tuning: `arcsf1tenth_planning/config/non_rrt.yaml` (commented).

### Pure pursuit
By default it follows the centerline, `<map_name>/maps/sim/cartographer_map_centerline.csv`
(Cartographer map frame: run the simulation with `slam:=localization` so the car is localized on that map).
With `line:=raceline` it uses `cartographer_map_raceline.csv`, otherwise the original F1TENTH
`raceline.csv` (Gazebo `world` frame, simulation only, works with `slam:=off`).

Arguments: `line:=centerline|raceline` (default `centerline`),
`speed_scale:=<fraction>` (of the path speed, default from `pure_pursuit.yaml`), `raceline:=<file.csv>` +
`raceline_frame:=<frame>` for another file, `use_sim_time:=false` on the real car. Tuning (lookahead,
speed limits) in `arcsf1tenth_control/config/pure_pursuit.yaml`. It stops when the car is more than 1 m
off the line or the pose is lost; Ctrl-C stops the car. RViz shows the raceline and the lookahead point.

With the parameter `trajectory_topic` (used by `yhom_controller`), pure pursuit follows an
`arcsf1tenth_interfaces/Trajectory` topic instead of a CSV: replaced on every message, speed 0 = stop,
empty or stale (> 0.5 s) = stop.

## Mapping (Cartographer) and localization (AMCL)

Cartographer builds the map; AMCL localizes on it while driving (the usual F1TENTH split).
In both, the EKF (`odom -> base_footprint`) gives the motion and the localizer publishes `map -> odom`.
Cartographer reads `scan/slam` (`scan_for_slam.py`: no-return beams kept as free space).

### 1. Build a map
```bash
ros2 launch arcsf1tenth_bringup simulation.launch.py track:=<map_name> slam:=mapping
ros2 run arcsf1tenth_teleop keyboard.py
```
Drive slowly (0.4-0.6 m/s) around the whole loop and finish where you started (loop closure).

### 2. Save the map (while mapping is still running)
```bash
ros2 run arcsf1tenth_localization save_map.py <map_name>
```
It finishes the trajectory (final optimization), then writes
`<map_name>/maps/<sim|real>/cartographer_map.{pgm,yaml}` in the **source** tree
(`sim` if Gazebo's `/clock` is running, else `real`) and copies them into `install/`, so no rebuild
is needed. It always overwrites the existing map of that track; maps are not in git until you commit them.
Other namespace: `--ros-args -r __ns:=/<namespace>`.

### 3. Localize on the saved map
```bash
ros2 launch arcsf1tenth_bringup simulation.launch.py track:=<map_name> slam:=localization
```
AMCL (particle filter) loads `cartographer_map.yaml` with `map_server` and corrects `map -> odom` on every
scan; the map is fixed and never redrawn. The car must start where mapping started (the track's start
pose = map origin): AMCL starts its particles there. Tuning: `arcsf1tenth_localization/config/amcl.yaml`
(commented, with a tuning table).

Measure the localization error against Gazebo's true pose (simulation only):
```bash
ros2 run arcsf1tenth_localization localization_error.py
```

## Raceline

Give a track name (or a map yaml path). A track name uses its `maps/sim/cartographer_map.yaml` if you saved
one, otherwise the original `map.yaml`, and picks the start pose for you
(`0 0 0` for the Cartographer map, `track.yaml` start_pose for the original map).

### Generate from a map
```bash
ros2 run arcsf1tenth_planning generate_raceline.py <map_name> --show
```
Writes `<map>_centerline.csv`, `<map>_raceline.csv` and `<map>_raceline.png` next to the map
(e.g. `cartographer_map_raceline.csv`, in the map frame, in the source tree); run
`colcon build --packages-select arcsf1tenth_tracks` afterwards so the launch files see them.
The raceline is minimum-curvature and checked against the car outline from `vehicle.yaml`.
Options: `--start X Y YAW` (start pose and driving direction), `--safety 0.1` (outline-to-wall margin),
`--a-lat 4.0`, `--spacing 0.1`, `--iterations 3`, `--out-dir DIR`.

### Show existing files
```bash
ros2 run arcsf1tenth_planning show_raceline.py <map_name>
```
Shows `<map>_centerline.csv` / `<map>_raceline.csv`; for an original F1TENTH map without them it shows
the track's own `centerline.csv` / `raceline.csv`. Other files: `--centerline FILE --raceline FILE`,
`--save FILE` to write the picture instead of opening it.

## Tracks

Add a track from a map image (walls are extruded from occupied pixels):
```bash
ros2 run arcsf1tenth_gazebo map_to_world.py <map_name>/maps/sim/map.yaml <map_name>/world/world.sdf --name <map_name>
```
Each track folder needs a `track.yaml` with `start_pose: {x, y, yaw}`; see
`arcsf1tenth_tracks/f1tenth_tracks/README.md` and `arcsf1tenth_tracks/lab_tracks/README.md`.

## Topics and frames (namespace `arcsf1tenth`)

| Topic | Type | From |
|---|---|---|
| `/arcsf1tenth/drive` | `ackermann_msgs/AckermannDriveStamped` | teleop / controller |
| `/arcsf1tenth/odometry/wheel` | `nav_msgs/Odometry` | wheel odometry (speed + steering) |
| `/arcsf1tenth/imu` | `sensor_msgs/Imu` | IMU |
| `/arcsf1tenth/imu/filtered` | `sensor_msgs/Imu` | `imu_filter.py` (gyro bias removed) |
| `/arcsf1tenth/odometry/estimated` | `nav_msgs/Odometry` | robot_localization EKF (wheel odometry + IMU) |
| `/arcsf1tenth/scan` | `sensor_msgs/LaserScan` | lidar |
| `/arcsf1tenth/scan/slam` | `sensor_msgs/LaserScan` | `scan_for_slam.py`: scan with no-return (`inf`) beams as `range_max`, Cartographer input |
| `/arcsf1tenth/map` | `nav_msgs/OccupancyGrid` | Cartographer occupancy grid node (mapping), `map_server` (AMCL localization) |
| `/arcsf1tenth/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL pose estimate |
| `/arcsf1tenth/ground_truth/odometry` | `nav_msgs/Odometry` | Gazebo true pose (sim only) |
| `/clicked_point` | `geometry_msgs/PointStamped` | RViz Publish Point → `obstacle_spawner` |
| `/arcsf1tenth/global_trajectory` | `arcsf1tenth_interfaces/Trajectory` | `frenet_node`: global path with speeds and track widths (latched) |
| `/arcsf1tenth/frenet/state` | `arcsf1tenth_interfaces/FrenetState` | `frenet_node`: car `s`, `d`, `s_dot`, `d_dot`, heading error |
| `/arcsf1tenth/obstacles` | `arcsf1tenth_interfaces/ObstacleArray` | `obstacle_detector` |
| `/arcsf1tenth/yhom/local_trajectory` | `arcsf1tenth_interfaces/Trajectory` | `yhom_local_planner` (empty = no path) |
| `/arcsf1tenth/yhom/trajectory` | `arcsf1tenth_interfaces/Trajectory` | `yhom_state_machine` → pure pursuit |
| `/arcsf1tenth/yhom/state` | `std_msgs/String` | `yhom_state_machine`: `GLOBAL` / `AVOID` / `BLOCKED` |

TF: `world` → `arcsf1tenth/map` → `arcsf1tenth/odom` → `arcsf1tenth/base_footprint` → `arcsf1tenth/base_link` → sensors and wheels.
`world` exists only in simulation (`world -> map` = the track's start pose); `map -> odom` comes from
Cartographer while mapping, AMCL while localizing (a fixed zero transform when `slam:=off`);
`odom -> base_footprint` from the EKF.
