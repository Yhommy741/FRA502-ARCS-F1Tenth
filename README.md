# FRA502-ARCS-F1Tenth

ROS 2 Jazzy + Gazebo Harmonic simulation of the ARCS F1Tenth car: keyboard teleop,
wheel/IMU odometry fused by a robot_localization EKF, lidar mapping (Cartographer), localization (AMCL), raceline
generation, wall following and pure pursuit. A real-car bringup template uses the same interfaces.

## Packages (`ARCSF1Tenth_ws/src`)

| Package | Contents |
|---|---|
| `arcsf1tenth_description` | URDF/meshes, `config/vehicle.yaml` (geometry, limits, footprint), `config/sensors.yaml` (IMU BNO055, lidar RPLidar C1) |
| `arcsf1tenth_gazebo` | Gazebo model plugins, bridge, simulated drivers: `gazebo_drive`, `gazebo_imu`, `gazebo_lidar`, `gazebo_wheel_odometry`, `map_to_world.py` |
| `arcsf1tenth_localization` | `imu_filter.py` (gyro bias calibration + ZUPT), `save_map.py`, `scan_for_slam.py`, `localization_error.py`, `config/ekf.yaml` (robot_localization), `config/imu_filter.yaml`, `config/cartographer_2d.lua` (mapping), `config/amcl.yaml` (localization) |
| `arcsf1tenth_planning` | `generate_raceline.py`, `show_raceline.py` |
| `arcsf1tenth_teleop` | `keyboard.py` |
| `arcsf1tenth_tracks` | Tracks: `f1tenth_tracks/` (F1TENTH tracks), `lab_tracks/` (our own) |
| `arcsf1tenth_bringup` | Launch files and RViz configs |
| `arcsf1tenth_control` | `wall_follow.py` (PID wall follower), `pure_pursuit.py` (raceline follower), `config/` |

## Install

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-ackermann-msgs \
    ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-localization ros-jazzy-cartographer-ros \
    ros-jazzy-nav2-amcl ros-jazzy-nav2-map-server ros-jazzy-nav2-lifecycle-manager \
    python3-opencv python3-scipy python3-matplotlib
```

## Build

```bash
cd ARCSF1Tenth_ws
colcon build
source install/setup.bash
```

Run `source install/setup.bash` in every new terminal, and again after adding a new package.

## Run

`<map_name>` in the commands below is a placeholder: replace it with a track folder name from
`arcsf1tenth_tracks` (e.g. `levine`, `brands_hatch`, `austin`, `monza`, or `empty` for the simulation).

### View the robot model (RViz only)
```bash
ros2 launch arcsf1tenth_bringup display.launch.py
```

### Simulation
```bash
ros2 launch arcsf1tenth_bringup yhom_simulation.launch.py track:=<map_name>
```
`non_simulation.launch.py` is currently an identical copy.

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
ros2 launch arcsf1tenth_bringup yhom_real.launch.py                                   # drivers + EKF only
ros2 launch arcsf1tenth_bringup yhom_real.launch.py track:=<map_name> slam:=mapping
ros2 launch arcsf1tenth_bringup yhom_real.launch.py track:=<map_name> slam:=localization
```
Same topics as the simulation, no Gazebo and no `world` frame. The hardware drivers (RPLidar, BNO055,
VESC, steering MCU over micro-ROS) are still `TODO` blocks in the launch file. Real maps live in
`<map_name>/maps/real/`. Extra argument: `rviz:=false` for headless. `non_real.launch.py` is a copy.

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

## Pure pursuit (follow the raceline)
Needs a raceline for the track (see [Raceline](#raceline)). In a second terminal, with the simulation running:
```bash
ros2 launch arcsf1tenth_bringup pure_pursuit.launch.py track:=<map_name>
```
By default it follows the centerline, `<map_name>/maps/sim/cartographer_map_centerline.csv`
(Cartographer map frame: run the simulation with `slam:=localization` so the car is localized on that map).
With `line:=raceline` it uses `cartographer_map_raceline.csv`, otherwise the original F1TENTH
`raceline.csv` (Gazebo `world` frame, simulation only, works with `slam:=off`).

Arguments: `line:=centerline|raceline` (default `centerline`),
`speed_scale:=0.6` (fraction of the raceline speed), `raceline:=<file.csv>` + `raceline_frame:=<frame>`
for another file, `use_sim_time:=false` on the real car. Tuning (lookahead, speed limits) in
`arcsf1tenth_control/config/pure_pursuit.yaml`. It stops when the car is more than 1 m off the
line or the pose is lost; Ctrl-C stops the car. RViz shows the raceline and the lookahead point.

## Mapping (Cartographer) and localization (AMCL)

Cartographer builds the map; AMCL localizes on it while driving (the usual F1TENTH split).
In both, the EKF (`odom -> base_footprint`) gives the motion and the localizer publishes `map -> odom`.
Cartographer reads `scan/slam` (`scan_for_slam.py`: no-return beams kept as free space).

### 1. Build a map
```bash
ros2 launch arcsf1tenth_bringup yhom_simulation.launch.py track:=<map_name> slam:=mapping
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
ros2 launch arcsf1tenth_bringup yhom_simulation.launch.py track:=<map_name> slam:=localization
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

TF: `world` → `arcsf1tenth/map` → `arcsf1tenth/odom` → `arcsf1tenth/base_footprint` → `arcsf1tenth/base_link` → sensors and wheels.
`world` exists only in simulation (`world -> map` = the track's start pose); `map -> odom` comes from
Cartographer while mapping, AMCL while localizing (a fixed zero transform when `slam:=off`);
`odom -> base_footprint` from the EKF.
