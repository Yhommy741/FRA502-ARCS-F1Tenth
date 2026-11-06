"""yhom's controller: global path (centerline) + Frenet optimal obstacle avoidance + pure pursuit.

Run on top of the simulation (or real car) launch, in a second terminal:
  ros2 launch arcsf1tenth_bringup simulation.launch.py track:=<map_name> slam:=localization
  ros2 launch arcsf1tenth_bringup yhom_controller.launch.py track:=<map_name>

    frenet_node        global path CSV + map -> global_trajectory, frenet/state      (shared)
    obstacle_detector  scan + map -> obstacles (Frenet)                               (shared)
    yhom_local_planner Frenet optimal path around obstacles -> yhom/local_trajectory
    yhom_state_machine GLOBAL | AVOID | BLOCKED -> yhom/trajectory
    pure_pursuit       follows yhom/trajectory -> drive

Global path: <track>/maps/sim/cartographer_map_centerline.csv (generate_raceline.py), map frame;
line:=raceline uses the minimum-curvature raceline. Needs slam:=localization (AMCL on the
saved map): obstacles are found by comparing the lidar with that map.
Tuning: arcsf1tenth_planning/config/yhom_planner.yaml (planner, state machine),
        arcsf1tenth_planning/config/frenet.yaml (Frenet frame, obstacle detection),
        arcsf1tenth_control/config/pure_pursuit.yaml (path following).
Each group member has an own controller launch (yhom_controller / non_controller): change
controllers and planning here without touching the other one. Run only one controller
at a time: all of them publish <namespace>/drive.
"""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def find_line(track, line):
    """<track>/maps/sim/cartographer_map_<line>.csv (map frame, from generate_raceline.py)."""
    tracks_pkg = get_package_share_directory('arcsf1tenth_tracks')
    matches = glob.glob(os.path.join(tracks_pkg, '*', track, 'track.yaml'))
    if not matches:
        available = sorted(
            os.path.basename(os.path.dirname(p))
            for p in glob.glob(os.path.join(tracks_pkg, '*', '*', 'track.yaml'))
        )
        raise RuntimeError(f"Track '{track}' not found. Available: {', '.join(available)}")
    if line not in ('raceline', 'centerline'):
        raise RuntimeError(f"line must be raceline or centerline, got '{line}'")
    path = os.path.join(os.path.dirname(matches[0]), 'maps', 'sim', f'cartographer_map_{line}.csv')
    if not os.path.isfile(path):
        raise RuntimeError(
            f"Track '{track}' has no {line} ({path}). Map it (slam:=mapping + save_map.py), then: "
            f'ros2 run arcsf1tenth_planning generate_raceline.py {track}, then colcon build')
    return path


def launch_setup(context):
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    prefix = f'{namespace}/' if namespace else ''
    track = LaunchConfiguration('track').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    if not track:
        raise RuntimeError('Give track:=<map_name>')
    global_path = find_line(track, LaunchConfiguration('line').perform(context))

    vehicle = os.path.join(get_package_share_directory('arcsf1tenth_description'), 'config', 'vehicle.yaml')
    planning_config = os.path.join(get_package_share_directory('arcsf1tenth_planning'), 'config')
    frenet_config = os.path.join(planning_config, 'frenet.yaml')
    yhom_config = os.path.join(planning_config, 'yhom_planner.yaml')
    pp_config = os.path.join(get_package_share_directory('arcsf1tenth_control'), 'config', 'pure_pursuit.yaml')
    common = {'use_sim_time': use_sim_time}

    pp_overrides = {**common, 'trajectory_topic': 'yhom/trajectory', 'raceline_frame': f'{prefix}map'}
    speed_scale = LaunchConfiguration('speed_scale').perform(context)
    if speed_scale:
        pp_overrides['speed_scale'] = float(speed_scale)

    def node(package, executable, name, params):
        return Node(package=package, executable=executable, name=name, namespace=namespace,
                    output='screen', parameters=params)

    return [
        LogInfo(msg=f'yhom_controller: global path {global_path}'),
        node('arcsf1tenth_planning', 'frenet_node.py', 'frenet',
             [frenet_config, {**common, 'global_path': global_path, 'path_frame': f'{prefix}map',
                              'base_frame': f'{prefix}base_footprint'}]),
        node('arcsf1tenth_planning', 'obstacle_detector.py', 'obstacle_detector',
             [frenet_config, {**common, 'map_frame': f'{prefix}map'}]),
        node('arcsf1tenth_planning', 'yhom_local_planner.py', 'yhom_local_planner',
             [vehicle, yhom_config, common]),
        node('arcsf1tenth_planning', 'yhom_state_machine.py', 'yhom_state_machine',
             [vehicle, yhom_config, common]),
        node('arcsf1tenth_control', 'pure_pursuit.py', 'pure_pursuit',
             [vehicle, pp_config, pp_overrides]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='arcsf1tenth',
                              description='Robot namespace: topics /<namespace>/..., TF frames <namespace>/...'),
        DeclareLaunchArgument('track', default_value='',
                              description='Track folder in arcsf1tenth_tracks (needs its cartographer_map_<line>.csv)'),
        DeclareLaunchArgument('line', default_value='centerline',
                              description='Global path: centerline | raceline'),
        DeclareLaunchArgument('speed_scale', default_value='',
                              description='Fraction of the path speed (default: pure_pursuit.yaml)'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the Gazebo clock (false on the real car)'),
        OpaqueFunction(function=launch_setup),
    ])
