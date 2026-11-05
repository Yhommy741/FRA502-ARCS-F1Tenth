"""Real car bringup (TEMPLATE): drivers + EKF + Cartographer, no Gazebo.

Same interfaces as the simulation, so EKF, SLAM, planning and control run unchanged:
  scan                  sensor_msgs/LaserScan     lidar driver          (sim: gazebo_lidar)
  imu                   sensor_msgs/Imu           IMU driver            (sim: gazebo_imu)
  odometry/wheel        nav_msgs/Odometry         VESC wheel odometry   (sim: gazebo_wheel_odometry)
  drive                 AckermannDriveStamped     -> VESC + steering MCU (sim: gazebo_drive)
TF: <ns>/map -> <ns>/odom (Cartographer) -> <ns>/base_footprint (EKF) -> <ns>/base_link -> sensors.
No 'world' frame on the real car.

TODO blocks below mark the hardware drivers to fill in (see the 'TODO' comments).

Usage:
  ros2 launch arcsf1tenth_bringup yhom_real.launch.py                                  # drivers + EKF only
  ros2 launch arcsf1tenth_bringup yhom_real.launch.py track:=lab_track slam:=mapping
  ros2 launch arcsf1tenth_bringup yhom_real.launch.py track:=lab_track slam:=localization
Real maps are saved in <track>/maps/real/ (separate from the simulation maps in maps/sim/).
"""

import glob
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


USE_SIM_TIME = False  # real car: wall clock


def find_track_dir(track):
    """Track folder in arcsf1tenth_tracks, or None if no track is given."""
    if not track:
        return None
    tracks_pkg = get_package_share_directory('arcsf1tenth_tracks')
    matches = glob.glob(os.path.join(tracks_pkg, '*', track, 'track.yaml'))
    if not matches:
        available = sorted(
            os.path.basename(os.path.dirname(p))
            for p in glob.glob(os.path.join(tracks_pkg, '*', '*', 'track.yaml'))
        )
        raise RuntimeError(f"Track '{track}' not found. Available: {', '.join(available)}")
    return os.path.dirname(matches[0])


def namespaced_rviz_config(template, namespace):
    """RViz cannot read launch arguments: copy the config with 'arcsf1tenth' replaced by the namespace."""
    with open(template) as f:
        text = f.read()
    prefix = f'{namespace}/' if namespace else ''
    text = (text.replace('/arcsf1tenth/', f'/{prefix}')
                .replace('arcsf1tenth/', prefix)
                .replace('arcsf1tenth', namespace))
    path = os.path.join(tempfile.gettempdir(), f'{namespace or "root"}_real_{os.path.basename(template)}')
    with open(path, 'w') as f:
        f.write(text)
    return path


def namespaced_cartographer_configs(config_dir, namespace):
    """Lua cannot read launch arguments: copy the Cartographer configs with the namespace filled in."""
    prefix = f'{namespace}/' if namespace else ''
    out_dir = os.path.join(tempfile.gettempdir(), f'{namespace or "root"}_cartographer')
    os.makedirs(out_dir, exist_ok=True)
    for name in glob.glob(os.path.join(config_dir, 'cartographer_*.lua')):
        with open(name) as f:
            text = f.read().replace('"arcsf1tenth/', f'"{prefix}')
        with open(os.path.join(out_dir, os.path.basename(name)), 'w') as f:
            f.write(text)
    return out_dir


def launch_setup(context):
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    prefix = f'{namespace}/' if namespace else ''  # TF frames: <namespace>/<link>
    track = LaunchConfiguration('track').perform(context)
    use_ekf = LaunchConfiguration('use_ekf').perform(context).lower() == 'true'
    use_imu = LaunchConfiguration('use_imu').perform(context).lower() == 'true'
    slam = LaunchConfiguration('slam').perform(context).lower()
    slam_map = LaunchConfiguration('slam_map').perform(context)
    if slam not in ('off', 'mapping', 'localization'):
        raise RuntimeError(f"slam must be off, mapping or localization, got '{slam}'")
    track_dir = find_track_dir(track)

    description_pkg = get_package_share_directory('arcsf1tenth_description')
    bringup_pkg = get_package_share_directory('arcsf1tenth_bringup')
    localization_pkg = get_package_share_directory('arcsf1tenth_localization')

    xacro_file = os.path.join(description_pkg, 'urdf', 'f1tenth.urdf.xacro')  # robot model only, no Gazebo plugins
    vehicle_config = os.path.join(description_pkg, 'config', 'vehicle.yaml')  # noqa: F841  (for the drivers below)
    sensors_config = os.path.join(description_pkg, 'config', 'sensors.yaml')  # noqa: F841  (for the drivers below)
    imu_filter_config = os.path.join(localization_pkg, 'config', 'imu_filter.yaml')
    ekf_config = LaunchConfiguration('ekf_config').perform(context) or os.path.join(localization_pkg, 'config', 'ekf.yaml')
    if not os.path.isfile(ekf_config):
        raise RuntimeError(f"ekf_config file not found: '{ekf_config}'")
    rviz_config = namespaced_rviz_config(os.path.join(bringup_pkg, 'config', 'simulation.rviz'), namespace)

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    actions = [
        # Robot model TF: base_footprint -> base_link -> lidar, imu, wheels
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'frame_prefix': prefix,
                'use_sim_time': USE_SIM_TIME,
            }],
        ),
        # The real car has no wheel/steering joint encoders: publish them at 0 so the
        # wheel TFs exist. TODO: replace with real joint states if the MCU reports steering.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=namespace,
            parameters=[{'use_sim_time': USE_SIM_TIME}],
        ),

        # ------------------------------------------------------------------ drivers
        # TODO lidar: RPLidar C1 -> scan (sensor_msgs/LaserScan), frame_id <ns>/lidar.
        #   sudo apt install ros-jazzy-sllidar-ros2   (or build Slamtec's sllidar_ros2)
        #   Check the serial port and baud rate for your unit (C1 datasheet / Slamtec docs).
        # Node(
        #     package='sllidar_ros2',
        #     executable='sllidar_node',
        #     namespace=namespace,
        #     output='screen',
        #     parameters=[{
        #         'serial_port': '/dev/ttyUSB0',
        #         'serial_baudrate': 460800,
        #         'frame_id': f'{prefix}lidar',
        #         'angle_compensate': True,
        #     }],
        # ),

        # TODO IMU: BNO055 -> imu (sensor_msgs/Imu), frame_id <ns>/imu, with the gyro
        #   covariance from sensors.yaml (imu.gyro_noise_std). Gyro yaw rate is the only
        #   field the EKF uses (after imu_filter removes the bias).
        # Node(
        #     package='<bno055 driver package>',
        #     executable='<bno055 driver node>',
        #     namespace=namespace,
        #     output='screen',
        #     parameters=[sensors_config, {'frame_id': f'{prefix}imu'}],
        #     remappings=[('<driver imu topic>', 'imu')],
        # ),

        # TODO VESC: motor speed -> odometry/wheel (nav_msgs/Odometry, like
        #   gazebo_wheel_odometry: twist.linear.x = speed, linear.y = 0, angular.z =
        #   v*tan(steering)/wheelbase, with covariances), and drive -> motor command.
        #   Wheel radius, gear ratio, wheelbase: vehicle.yaml.
        # Node(package='<vesc driver package>', executable='<vesc driver>', namespace=namespace, ...),
        # Node(package='<arcsf1tenth_hardware>', executable='<vesc_odometry>', namespace=namespace,
        #      parameters=[vehicle_config]),

        # TODO steering MCU (STM32, micro-ROS): drive -> steering servo.
        # Node(
        #     package='micro_ros_agent',
        #     executable='micro_ros_agent',
        #     arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '115200'],
        #     output='screen',
        # ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config, '-f', f'{prefix}map'],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ]

    # -------------------------------------------------------------- SLAM (Cartographer)
    if slam == 'off':
        # No SLAM: map = odom (no drift correction)
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['--frame-id', f'{prefix}map', '--child-frame-id', f'{prefix}odom'],
        ))
    else:
        config_dir = namespaced_cartographer_configs(os.path.join(localization_pkg, 'config'), namespace)
        arguments = ['-configuration_directory', config_dir]
        if slam == 'mapping':
            arguments += ['-configuration_basename', 'cartographer_2d.lua']
        else:
            if not slam_map:
                if track_dir is None:
                    raise RuntimeError('slam:=localization needs track:=<name> or slam_map:=<file.pbstream>')
                slam_map = os.path.join(track_dir, 'maps', 'real', 'cartographer_map')
            if not slam_map.endswith('.pbstream'):
                slam_map += '.pbstream'
            if not os.path.isfile(slam_map):
                raise RuntimeError(
                    f"No saved map '{slam_map}'. Build one with slam:=mapping, save it "
                    '(finish_trajectory + write_state), then colcon build, or pass slam_map:=<file.pbstream>')
            # Put the car where mapping started: the new trajectory begins at the map origin
            # The trajectory is started below with the known start pose instead of automatically
            arguments += ['-configuration_basename', 'cartographer_2d_localization.lua',
                          '-load_state_filename', slam_map,
                          '-start_trajectory_with_default_topics=false']
        actions += [
            Node(
                package='cartographer_ros',
                executable='cartographer_node',
                namespace=namespace,
                output='screen',
                arguments=arguments,
                remappings=[('odom', 'odometry/estimated')],
                parameters=[{'use_sim_time': USE_SIM_TIME}],
            ),
            Node(
                package='cartographer_ros',
                executable='cartographer_occupancy_grid_node',
                namespace=namespace,
                output='screen',
                arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
                parameters=[{'use_sim_time': USE_SIM_TIME}],
            ),
        ]
        if slam == 'localization':
            # Start localization AT the known start pose: map origin (= where mapping started =
            # the track's start pose), relative to the saved map's trajectory 0. Without it,
            # Cartographer searches the whole map and can snap to a look-alike spot.
            # 'ros2 service call' waits until cartographer_node is ready.
            start_request = (
                f"{{configuration_directory: '{config_dir}', "
                "configuration_basename: 'cartographer_2d_localization.lua', "
                'use_initial_pose: true, '
                'initial_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, '
                'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, '
                'relative_to_trajectory_id: 0}'
            )
            actions.append(ExecuteProcess(
                cmd=['ros2', 'service', 'call', f'/{prefix}start_trajectory',
                     'cartographer_ros_msgs/srv/StartTrajectory', start_request],
                output='screen',
            ))

    # ------------------------------------------------------------------------ EKF
    if use_ekf:
        ekf_overrides = {
            'use_sim_time': USE_SIM_TIME,
            'map_frame': f'{prefix}map',
            'odom_frame': f'{prefix}odom',
            'base_link_frame': f'{prefix}base_footprint',
            'world_frame': f'{prefix}odom',
        }
        if not use_imu:
            ekf_overrides['imu0_config'] = [False] * 15
        actions += [
            # IMU preprocessing (startup calibration + ZUPT gyro bias): imu -> imu/filtered.
            # Real car: also remove suspension tilt and filter BNO055 spikes/noise.
            Node(
                package='arcsf1tenth_localization',
                executable='imu_filter.py',
                namespace=namespace,
                output='screen',
                parameters=[imu_filter_config, {
                    'use_sim_time': USE_SIM_TIME,
                    'project_to_horizontal': True,
                    'gyro_median_window': 3,
                    'gyro_ma_window': 5,
                }],
            ),
            # robot_localization EKF: odometry/wheel + imu -> odometry/estimated + odom TF
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf',  # so the launch overrides beat the node params in ekf.yaml
                namespace=namespace,
                output='screen',
                parameters=[ekf_config, ekf_overrides],
                remappings=[('odometry/filtered', 'odometry/estimated')],
            ),
        ]

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='arcsf1tenth',
            description='Robot namespace: topics /<namespace>/..., TF frames <namespace>/...',
        ),
        DeclareLaunchArgument(
            'track',
            default_value='',
            description='Track folder in arcsf1tenth_tracks; real maps are in <track>/maps/real/',
        ),
        DeclareLaunchArgument(
            'use_ekf',
            default_value='true',
            description='Run the robot_localization EKF (odometry/estimated + odom TF)',
        ),
        DeclareLaunchArgument(
            'use_imu',
            default_value='true',
            description='Fuse IMU yaw rate in the EKF (false = wheel odometry only)',
        ),
        DeclareLaunchArgument(
            'ekf_config',
            default_value='',
            description='EKF parameter file (default: arcsf1tenth_localization/config/ekf.yaml)',
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='off',
            description='Cartographer SLAM: off | mapping (build a new map) | localization (on a saved map)',
        ),
        DeclareLaunchArgument(
            'slam_map',
            default_value='',
            description='Saved Cartographer map for slam:=localization '
                        '(default: <track>/maps/real/cartographer_map.pbstream)',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz (false when running headless on the car computer)',
        ),

        OpaqueFunction(function=launch_setup),
    ])
