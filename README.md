# FRA502-ARCS-F1Tenth

ROS 2 Jazzy + Gazebo Harmonic simulation of the ARCS F1Tenth car: keyboard teleop,
wheel/IMU odometry with an EKF, lidar SLAM (slam_toolbox), and raceline generation.

## Packages (`ARCSF1Tenth_ws/src`)

| Package | Contents |
|---|---|
| `arcsf1tenth_description` | URDF/meshes, `config/vehicle.yaml` (geometry, limits), `config/sensors.yaml` (IMU BNO055, lidar RPLidar C1) |
| `arcsf1tenth_gazebo` | Gazebo model plugins, bridge, simulated drivers: `gazebo_drive`, `gazebo_imu`, `gazebo_lidar`, `gazebo_measure_odometry`, `map_to_world.py` |
| `arcsf1tenth_localization` | `ekf.py` (wheel odometry + IMU), `odom_eval.py` (score odometry vs Gazebo ground truth), `config/ekf.yaml`, `config/slam_toolbox.yaml` |
| `arcsf1tenth_planning` | `generate_raceline.py`, `show_raceline.py` |
| `arcsf1tenth_teleop` | `keyboard.py` |
| `arcsf1tenth_tracks` | Tracks: `f1tenth_tracks/` (28 F1TENTH tracks), `lab_tracks/` (our own) |
| `arcsf1tenth_bringup` | Launch files and RViz configs |
| `arcsf1tenth_control` | `wall_follow.py` (PID wall follower), `pure_pursuit.py` (raceline follower), `config/` |

## Install

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-ackermann-msgs \
    ros-jazzy-joint-state-publisher-gui ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server \
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
`arcsf1tenth_tracks` (e.g. `levine`, `brands_hatch`, `spielberg`, `monza`, or `empty` for the simulation).

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
| `slam` | `off` | `off`, `mapping` or `localization` |
| `slam_map` | `<map_name>/maps/sim/slam_map` | saved map for `slam:=localization` (path without extension) |
| `use_ekf` | `true` | EKF publishes `odometry/estimated` and the odom TF |
| `use_imu` | `true` | fuse the IMU in the EKF |
| `ekf_config` | `arcsf1tenth_localization/config/ekf.yaml` | EKF tuning file (all noise parameters, commented) |
| `follow_camera` | `true` | Gazebo camera follows the car |

A wrong track name prints the list of available tracks.

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
Arguments: `side:=left|right`, `desired_distance:=0.8`, `speed_max:=1.5`, `namespace:=arcsf1tenth`.
Gains and the rest are in `arcsf1tenth_control/config/wall_follow.yaml`. Ctrl-C stops the car.
Don't run the keyboard teleop at the same time (both publish `drive`).

## Pure pursuit (follow the raceline)
Needs a raceline for the track (see [Raceline](#raceline)). In a second terminal, with the simulation running:
```bash
ros2 launch arcsf1tenth_bringup pure_pursuit.launch.py track:=<map_name>
```
It uses `<map_name>/maps/sim/slam_map_raceline.csv` (SLAM map frame: run the simulation with
`slam:=localization` so the car is localized on that map), otherwise the original F1TENTH
`raceline.csv` (Gazebo `world` frame, simulation only, works with `slam:=off`).
Arguments: `speed_scale:=0.6` (fraction of the raceline speed), `raceline:=<file.csv>` +
`raceline_frame:=<frame>` for another file. Tuning (lookahead, speed limits) in
`arcsf1tenth_control/config/pure_pursuit.yaml`. It stops when the car is more than 1 m off the
raceline or the pose is lost; Ctrl-C stops the car. RViz shows the raceline (magenta) and the
lookahead point (orange).

## Odometry evaluation (simulation)
```bash
ros2 run arcsf1tenth_localization odom_eval.py --ros-args -r __ns:=/arcsf1tenth -p use_sim_time:=true
```
Drive a lap, then Ctrl-C: prints pose error / drift of `odometry/measured` and `odometry/estimated`
against `ground_truth/odometry`, the yaw rate error of each input (gyro, bicycle model, EKF) and the
EKF gyro bias estimate vs the actual bias. Use it to tune `arcsf1tenth_localization/config/ekf.yaml`.

## SLAM

Run every command from `ARCSF1Tenth_ws`. Set the track once per terminal; all paths below follow it
(works for both `f1tenth_tracks/` and `lab_tracks/`):
```bash
TRACK=<map_name>
MAP=$(ls -d $PWD/src/arcsf1tenth_tracks/*/$TRACK)/maps/sim/slam_map
echo $MAP        # check before saving: saving overwrites an existing slam_map of that track
```

### 1. Build a map
```bash
ros2 launch arcsf1tenth_bringup simulation.launch.py track:=$TRACK slam:=mapping
ros2 run arcsf1tenth_teleop keyboard.py
```
Drive slowly (0.4-0.6 m/s) around the whole loop and finish where you started (loop closure).

### 2. Save the map (while mapping is still running)
```bash
# pose graph (needed for localization): slam_map.posegraph + slam_map.data
ros2 service call /arcsf1tenth/slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$MAP'}"
# image: slam_map.pgm + slam_map.yaml
ros2 run nav2_map_server map_saver_cli -f $MAP -t /arcsf1tenth/map

colcon build --packages-select arcsf1tenth_tracks
```
Maps are not in git until you commit them: saving to the wrong track overwrites that track's map for good.

### 3. Localize on the saved map
```bash
ros2 launch arcsf1tenth_bringup simulation.launch.py track:=$TRACK slam:=localization
```
The car must start where mapping started (the track's start pose). If it gets lost, use
RViz "2D Pose Estimate" with its topic set to `/arcsf1tenth/initialpose`.

## Raceline

Give a track name (or a map yaml path). A track name uses its `slam_map.yaml` if you saved one,
otherwise the original `map.yaml`, and picks the start pose for you
(`0 0 0` for a SLAM map, `track.yaml` start_pose for the original map).

### Generate from a map
```bash
ros2 run arcsf1tenth_planning generate_raceline.py <map_name> --show
colcon build --packages-select arcsf1tenth_tracks
```
Writes `slam_map_centerline.csv`, `slam_map_raceline.csv` and `slam_map_raceline.png` next to the map
(in the map frame, in the source tree). Options: `--start X Y YAW` (start pose and driving direction),
`--wall-margin 0.3`, `--a-lat 4.0`, `--spacing 0.1`, `--out-dir DIR`.

### Show existing files
```bash
ros2 run arcsf1tenth_planning show_raceline.py <map_name>
```
Shows `<map>_centerline.csv` / `<map>_raceline.csv`; for an original F1TENTH map without them it shows
the track's own `centerline.csv` / `raceline.csv`. Other files: `--centerline FILE --raceline FILE`.

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
| `/arcsf1tenth/odometry/measured` | `nav_msgs/Odometry` | wheel odometry (speed + steering) |
| `/arcsf1tenth/odometry/estimated` | `nav_msgs/Odometry` | EKF (wheel odometry + IMU) |
| `/arcsf1tenth/imu` | `sensor_msgs/Imu` | IMU |
| `/arcsf1tenth/scan` | `sensor_msgs/LaserScan` | lidar |
| `/arcsf1tenth/map` | `nav_msgs/OccupancyGrid` | slam_toolbox |
| `/arcsf1tenth/ground_truth/odometry` | `nav_msgs/Odometry` | Gazebo true pose (sim only) |

TF: `world` → `arcsf1tenth/map` → `arcsf1tenth/odom` → `arcsf1tenth/base_footprint` → `arcsf1tenth/base_link` → sensors and wheels.
`world` exists only in simulation; `map -> odom` comes from slam_toolbox (a fixed zero transform when `slam:=off`).
