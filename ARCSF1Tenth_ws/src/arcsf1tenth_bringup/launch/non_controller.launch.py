"""non's controller: global path + kinodynamic RRT obstacle avoidance + the shared pure_pursuit.

Run on top of the simulation launch, in a second terminal:
  ros2 launch arcsf1tenth_bringup simulation.launch.py track:=<map_name> slam:=localization
  ros2 launch arcsf1tenth_bringup non_controller.launch.py track:=<map_name>

    non_obstacle_detector  map + scan + TF -> rrt/obstacle_points (lidar points not in the saved map)
    non_state_machine      obstacle points + TF + odometry + global path -> rrt/trajectory
                           (GLOBAL | AVOID | BLOCKED; asks non_rrt_node for a detour when blocked)
    non_rrt_node           rrt/plan service: kinodynamic RRT detour around the obstacles
    pure_pursuit           rrt/trajectory -> drive (same controller as yhom_controller)

Each group member has an own avoidance (non_controller / yhom_controller), the controller is shared.
Run only one controller launch at a time: both publish <namespace>/drive.
Global path: <track>/maps/sim/cartographer_map_<line>.csv (generate_raceline.py), map frame.
Needs slam:=localization: obstacles are lidar points that are not in the saved map.
Tuning: arcsf1tenth_planning/config/non_rrt.yaml (pure_pursuit: arcsf1tenth_control/config/pure_pursuit.yaml).
Code: arcsf1tenth_planning/scripts/non_*.py, arcsf1tenth_applications/scripts/non_state_machine.py,
arcsf1tenth_planning/arcsf1tenth_planning/rrt/ (detection + kinodynamic RRT library, no ROS).
RViz: add rrt/obstacle_points (PointCloud2), rrt/tree, rrt/goal, rrt/state_marker (Marker), rrt/path (Path).
"""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def find_line(track, line):
    tracks_pkg = get_package_share_directory('arcsf1tenth_tracks')
    matches = glob.glob(os.path.join(tracks_pkg, '*', track, 'track.yaml'))
    if not matches:
        available = sorted(os.path.basename(os.path.dirname(p))
                           for p in glob.glob(os.path.join(tracks_pkg, '*', '*', 'track.yaml')))
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
    if not track:
        raise RuntimeError('Give track:=<map_name>')
    global_path = find_line(track, LaunchConfiguration('line').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    vehicle = os.path.join(get_package_share_directory('arcsf1tenth_description'), 'config', 'vehicle.yaml')
    config = os.path.join(get_package_share_directory('arcsf1tenth_planning'), 'config', 'non_rrt.yaml')
    pp_config = os.path.join(get_package_share_directory('arcsf1tenth_control'), 'config', 'pure_pursuit.yaml')
    frames = {'map_frame': f'{prefix}map', 'base_frame': f'{prefix}base_footprint'}
    common = {'use_sim_time': use_sim_time}
    own = {**common, 'global_path': global_path}

    # shared controller, following this pipeline's trajectory; speed_min low so the BLOCKED braking
    # profile is followed down to the stop (pure_pursuit raises any speed above 0.05 m/s to speed_min)
    pp_overrides = {**common, 'trajectory_topic': 'rrt/trajectory', 'raceline_frame': frames['map_frame'],
                    'base_frame': frames['base_frame'], 'speed_min': 0.1}
    speed_scale = LaunchConfiguration('speed_scale').perform(context)
    if speed_scale:
        pp_overrides['speed_scale'] = float(speed_scale)

    def node(package, executable, name, params):
        return Node(package=package, executable=executable, name=name, namespace=namespace,
                    output='screen', parameters=params)

    return [
        LogInfo(msg=f'non_controller (kinodynamic RRT): global path {global_path}'),
        node('arcsf1tenth_planning', 'non_obstacle_detector.py', 'non_obstacle_detector',
             [config, {**own, 'map_frame': frames['map_frame']}]),
        node('arcsf1tenth_planning', 'non_rrt_node.py', 'non_rrt_node',
             [vehicle, config, {**own, 'map_frame': frames['map_frame']}]),
        node('arcsf1tenth_applications', 'non_state_machine.py', 'non_state_machine',
             [vehicle, config, {**own, **frames}]),
        node('arcsf1tenth_control', 'pure_pursuit.py', 'pure_pursuit',
             [vehicle, pp_config, pp_overrides]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='arcsf1tenth',
                              description='Robot namespace: topics /<namespace>/..., TF frames <namespace>/...'),
        DeclareLaunchArgument('track', default_value='',
                              description='Track folder in arcsf1tenth_tracks (needs its cartographer_map_<line>.csv)'),
        DeclareLaunchArgument('line', default_value='centerline', description='Global path: centerline | raceline'),
        DeclareLaunchArgument('speed_scale', default_value='', description='Fraction of the path speed (default: pure_pursuit.yaml)'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use the Gazebo clock'),
        OpaqueFunction(function=launch_setup),
    ])
