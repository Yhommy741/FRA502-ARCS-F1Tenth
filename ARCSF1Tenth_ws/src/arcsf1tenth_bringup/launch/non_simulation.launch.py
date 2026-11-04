import glob
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def find_track(track):
    """Return (world_file, start_pose, track_dir) for a track name in arcsf1tenth_tracks."""
    if track == 'empty':
        gazebo_pkg = get_package_share_directory('arcsf1tenth_gazebo')
        return os.path.join(gazebo_pkg, 'worlds', 'empty.sdf'), {'x': 0.0, 'y': 0.0, 'yaw': 0.0}, None

    tracks_pkg = get_package_share_directory('arcsf1tenth_tracks')
    matches = glob.glob(os.path.join(tracks_pkg, '*', track, 'track.yaml'))
    if not matches:
        available = sorted(
            os.path.basename(os.path.dirname(p))
            for p in glob.glob(os.path.join(tracks_pkg, '*', '*', 'track.yaml'))
        )
        raise RuntimeError(f"Track '{track}' not found. Available: empty, {', '.join(available)}")

    track_dir = os.path.dirname(matches[0])
    with open(matches[0]) as f:
        info = yaml.safe_load(f)
    return os.path.join(track_dir, 'world', 'world.sdf'), info['start_pose'], track_dir


def namespaced_rviz_config(template, namespace):
    """RViz cannot read launch arguments: copy the config with 'arcsf1tenth' replaced by the namespace."""
    with open(template) as f:
        text = f.read()
    prefix = f'{namespace}/' if namespace else ''
    text = (text.replace('/arcsf1tenth/', f'/{prefix}')
                .replace('arcsf1tenth/', prefix)
                .replace('arcsf1tenth', namespace))
    path = os.path.join(tempfile.gettempdir(), f'{namespace or "root"}_{os.path.basename(template)}')
    with open(path, 'w') as f:
        f.write(text)
    return path


def namespaced_cartographer_configs(config_dir, namespace):
    """Lua cannot read launch arguments: copy the Cartographer configs with the namespace filled in.

    Returns the directory to pass as -configuration_directory (Cartographer's own
    map_builder.lua / trajectory_builder.lua are found automatically)."""
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
    spawn_z = LaunchConfiguration('z').perform(context)
    use_ekf = LaunchConfiguration('use_ekf').perform(context).lower() == 'true'
    use_imu = LaunchConfiguration('use_imu').perform(context).lower() == 'true'
    slam = LaunchConfiguration('slam').perform(context).lower()
    slam_map = LaunchConfiguration('slam_map').perform(context)
    if slam not in ('off', 'mapping', 'localization'):
        raise RuntimeError(f"slam must be off, mapping or localization, got '{slam}'")
    world_file, start, track_dir = find_track(track)

    description_pkg = get_package_share_directory('arcsf1tenth_description')
    gazebo_pkg = get_package_share_directory('arcsf1tenth_gazebo')
    bringup_pkg = get_package_share_directory('arcsf1tenth_bringup')
    localization_pkg = get_package_share_directory('arcsf1tenth_localization')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(gazebo_pkg, 'urdf', 'f1tenth_gazebo.urdf.xacro')
    bridge_config = os.path.join(gazebo_pkg, 'config', 'bridge.yaml')
    vehicle_config = os.path.join(description_pkg, 'config', 'vehicle.yaml')
    sensors_config = os.path.join(description_pkg, 'config', 'sensors.yaml')
    ekf_config = LaunchConfiguration('ekf_config').perform(context) or os.path.join(localization_pkg, 'config', 'ekf.yaml')
    if not os.path.isfile(ekf_config):
        raise RuntimeError(f"ekf_config file not found: '{ekf_config}'")
    rviz_config = namespaced_rviz_config(os.path.join(bringup_pkg, 'config', 'simulation.rviz'), namespace)

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, f' namespace:={namespace}' if namespace else '']),
        value_type=str,
    )

    # Gazebo GUI chase camera. follow_offset is in the car's frame
    # (base_footprint: x forward, so -x is behind the car). Waits until the GUI
    # is listening on /gui/track, otherwise the one-shot message would be lost.
    camera_track_msg = (
        'track_mode: FOLLOW_LOOK_AT, '
        'follow_target: {name: "f1tenth"}, track_target: {name: "f1tenth"}, '
        'follow_offset: {x: -1.0, y: 0.0, z: 2.5}, '
        'follow_pgain: 0.05, track_pgain: 0.05'
    )
    follow_camera_cmd = (
        'until gz topic -i -t /gui/track | grep -A1 Subscribers | grep -q CameraTrack; '
        'do sleep 1; done; sleep 2; '
        f"gz topic -t /gui/track -m gz.msgs.CameraTrack -p '{camera_track_msg}'"
    )

    actions = [
        # Let Gazebo resolve package://arcsf1tenth_description/meshes/...
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(description_pkg)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f'-r {world_file}'}.items(),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'frame_prefix': prefix,
                'use_sim_time': True,
            }],
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            namespace=namespace,  # reads /<namespace>/robot_description
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'f1tenth',
                '-x', str(start['x']),
                '-y', str(start['y']),
                '-z', spawn_z,
                '-Y', str(start['yaw']),
            ],
        ),

        # world -> <namespace>/map: the map starts where the car spawns (sim only;
        # the real car has no world frame). Lets RViz show the car's estimate and the
        # Gazebo ground truth (world frame) together.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=[
                '--x', str(start['x']), '--y', str(start['y']), '--yaw', str(start['yaw']),
                '--frame-id', 'world', '--child-frame-id', f'{prefix}map',
            ],
            parameters=[{'use_sim_time': True}],
        ),

        ExecuteProcess(
            cmd=['bash', '-c', follow_camera_cmd],
            condition=IfCondition(LaunchConfiguration('follow_camera')),
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=namespace,
            output='screen',
            parameters=[{'config_file': bridge_config}],
        ),

        # drive (AckermannDriveStamped) -> steering + rear wheel joint commands
        Node(
            package='arcsf1tenth_gazebo',
            executable='gazebo_drive.py',
            namespace=namespace,
            output='screen',
            parameters=[vehicle_config, {'use_sim_time': True}],
        ),

        # Simulated IMU driver (sim stand-in for the MCU): gazebo/imu -> imu
        Node(
            package='arcsf1tenth_gazebo',
            executable='gazebo_imu.py',
            namespace=namespace,
            output='screen',
            parameters=[sensors_config, {'use_sim_time': True}],
        ),

        # Simulated lidar driver (sim stand-in for the RPLidar driver): gazebo/scan -> scan
        Node(
            package='arcsf1tenth_gazebo',
            executable='gazebo_lidar.py',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # Wheel odometry (sim stand-in for the VESC's vesc_to_odom): odometry/measured.
        # Owns the odom -> base_footprint TF unless the EKF runs.
        Node(
            package='arcsf1tenth_gazebo',
            executable='gazebo_measure_odometry.py',
            namespace=namespace,
            output='screen',
            parameters=[vehicle_config, {'use_sim_time': True, 'publish_tf': not use_ekf}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config, '-f', 'world'],
            parameters=[{'use_sim_time': True}],
        ),
    ]

    if slam == 'off':
        # No SLAM: map = odom (no drift correction)
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['--frame-id', f'{prefix}map', '--child-frame-id', f'{prefix}odom'],
            parameters=[{'use_sim_time': True}],
        ))
    else:
        # Cartographer publishes map -> odom (drift correction); the occupancy grid node
        # turns its submaps into the map topic. Odometry input = the EKF output.
        config_dir = namespaced_cartographer_configs(os.path.join(localization_pkg, 'config'), namespace)
        arguments = ['-configuration_directory', config_dir]
        if slam == 'mapping':
            arguments += ['-configuration_basename', 'cartographer_2d.lua']
        else:
            if not slam_map:
                if track_dir is None:
                    raise RuntimeError('slam:=localization needs slam_map:=<path to the saved .pbstream>')
                slam_map = os.path.join(track_dir, 'maps', 'sim', 'cartographer_map')
            if not slam_map.endswith('.pbstream'):
                slam_map += '.pbstream'
            if not os.path.isfile(slam_map):
                raise RuntimeError(
                    f"No saved map '{slam_map}'. Build one with slam:=mapping, save it "
                    '(finish_trajectory + write_state), then colcon build, or pass slam_map:=<file.pbstream>')
            # Car starts where mapping started: the new trajectory begins at the map origin
            arguments += ['-configuration_basename', 'cartographer_2d_localization.lua',
                          '-load_state_filename', slam_map]
        actions += [
            Node(
                package='cartographer_ros',
                executable='cartographer_node',
                namespace=namespace,
                output='screen',
                arguments=arguments,
                remappings=[('odom', 'odometry/estimated')],
                parameters=[{'use_sim_time': True}],
            ),
            Node(
                package='cartographer_ros',
                executable='cartographer_occupancy_grid_node',
                namespace=namespace,
                output='screen',
                arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
                parameters=[{'use_sim_time': True}],
            ),
        ]

    if use_ekf:
        ekf_overrides = {
            'use_sim_time': True,
            'map_frame': f'{prefix}map',
            'odom_frame': f'{prefix}odom',
            'base_link_frame': f'{prefix}base_footprint',
            'world_frame': f'{prefix}odom',
        }
        if not use_imu:
            ekf_overrides['imu0_config'] = [False] * 15
        actions += [
            # Gyro bias removal while standing still (ZUPT): imu -> imu/corrected
            Node(
                package='arcsf1tenth_localization',
                executable='imu_bias_correction.py',
                namespace=namespace,
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
            # robot_localization EKF (15-state general model):
            # odometry/measured + imu/corrected -> odometry/estimated + TF odom -> base_footprint
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
            default_value='empty',
            description='Track folder name in arcsf1tenth_tracks (e.g. levine, spielberg), or "empty"',
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.1',
            description='Spawn height of the robot [m]',
        ),
        DeclareLaunchArgument(
            'use_ekf',
            default_value='true',
            description='Run the robot_localization EKF (odometry/estimated + odom TF); '
                        'false = measured odometry owns the TF',
        ),
        DeclareLaunchArgument(
            'use_imu',
            default_value='true',
            description='Fuse IMU yaw rate in the odometry EKF (false = wheel odometry only)',
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
                        '(default: <track>/maps/sim/cartographer_map.pbstream)',
        ),
        DeclareLaunchArgument(
            'follow_camera',
            default_value='true',
            description='Make the Gazebo GUI camera follow the car',
        ),

        OpaqueFunction(function=launch_setup),
    ])
