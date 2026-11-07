"""non's controller: global path + kinodynamic RRT obstacle avoidance + own tracker.

Run on top of the simulation launch, in a second terminal:
  ros2 launch arcsf1tenth_bringup simulation.launch.py track:=<map_name> slam:=localization
  ros2 launch arcsf1tenth_bringup non_controller.launch.py track:=<map_name>

    non_rrt_planner  map + scan + TF + odometry + global path CSV -> rrt/trajectory
                     (obstacle detection, GLOBAL | AVOID | BLOCKED, kinodynamic RRT)
    non_rrt_tracker  rrt/trajectory -> drive (rear wheel feedback, or controller:=pure_pursuit)

Self-contained: uses none of the other controllers' nodes (yhom_controller has its own).
Each group member has an own controller launch (non_controller / yhom_controller). Run only one
controller at a time: all of them publish <namespace>/drive.
Global path: <track>/maps/sim/cartographer_map_<line>.csv (generate_raceline.py), map frame.
Needs slam:=localization: obstacles are lidar points that are not in the saved map.
Tuning: arcsf1tenth_planning/config/non_rrt.yaml. Code: arcsf1tenth_planning/scripts/non_rrt_*.py,
arcsf1tenth_planning/arcsf1tenth_planning/rrt/ (kinodynamic RRT library, no ROS).
RViz: add rrt/tree, rrt/path, rrt/goal, rrt/obstacle_points, rrt/state_marker (Marker / Path).
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
    frames = {'map_frame': f'{prefix}map', 'base_frame': f'{prefix}base_footprint'}
    common = {'use_sim_time': use_sim_time}
    tracker = {**common, 'base_frame': frames['base_frame']}
    speed_scale = LaunchConfiguration('speed_scale').perform(context)
    if speed_scale:
        tracker['speed_scale'] = float(speed_scale)

    return [
        LogInfo(msg=f'non_controller (kinodynamic RRT): global path {global_path}'),
        Node(package='arcsf1tenth_planning', executable='non_rrt_planner.py', name='non_rrt_planner',
             namespace=namespace, output='screen',
             parameters=[vehicle, config, {**common, **frames, 'global_path': global_path}]),
        Node(package='arcsf1tenth_planning', executable='non_rrt_tracker.py', name='non_rrt_tracker',
             namespace=namespace, output='screen', parameters=[vehicle, config, tracker]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='arcsf1tenth',
                              description='Robot namespace: topics /<namespace>/..., TF frames <namespace>/...'),
        DeclareLaunchArgument('track', default_value='',
                              description='Track folder in arcsf1tenth_tracks (needs its cartographer_map_<line>.csv)'),
        DeclareLaunchArgument('line', default_value='centerline', description='Global path: centerline | raceline'),
        DeclareLaunchArgument('speed_scale', default_value='', description='Fraction of the path speed (default: non_rrt.yaml)'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use the Gazebo clock'),
        OpaqueFunction(function=launch_setup),
    ])
