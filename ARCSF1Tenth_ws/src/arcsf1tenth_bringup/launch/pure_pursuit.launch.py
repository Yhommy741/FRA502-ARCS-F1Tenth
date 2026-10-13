import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def find_raceline(track, prefix):
    """Return (raceline CSV, frame) for a track: our SLAM-map raceline, else the original F1TENTH one."""
    tracks_pkg = get_package_share_directory('arcsf1tenth_tracks')
    matches = glob.glob(os.path.join(tracks_pkg, '*', track, 'track.yaml'))
    if not matches:
        available = sorted(
            os.path.basename(os.path.dirname(p))
            for p in glob.glob(os.path.join(tracks_pkg, '*', '*', 'track.yaml'))
        )
        raise RuntimeError(f"Track '{track}' not found. Available: {', '.join(available)}")
    sim_dir = os.path.join(os.path.dirname(matches[0]), 'maps', 'sim')

    # generate_raceline.py on the SLAM map: in the SLAM map frame (needs slam:=localization)
    slam_raceline = os.path.join(sim_dir, 'slam_map_raceline.csv')
    if os.path.isfile(slam_raceline):
        return slam_raceline, f'{prefix}map'
    # Original F1TENTH raceline: in the map.yaml frame = Gazebo world frame (simulation only)
    original = os.path.join(sim_dir, 'raceline.csv')
    if os.path.isfile(original):
        return original, 'world'
    raise RuntimeError(
        f"Track '{track}' has no raceline. Make one with: "
        f'ros2 run arcsf1tenth_planning generate_raceline.py {track}, then colcon build')


def launch_setup(context):
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    prefix = f'{namespace}/' if namespace else ''
    track = LaunchConfiguration('track').perform(context)
    raceline = LaunchConfiguration('raceline').perform(context)
    frame = LaunchConfiguration('raceline_frame').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    if not raceline:
        if not track:
            raise RuntimeError('Give track:=<map_name> (or raceline:=<file.csv> raceline_frame:=<frame>)')
        raceline, auto_frame = find_raceline(track, prefix)
        frame = frame or auto_frame
    frame = frame or f'{prefix}map'

    description_pkg = get_package_share_directory('arcsf1tenth_description')
    control_pkg = get_package_share_directory('arcsf1tenth_control')
    overrides = {'use_sim_time': use_sim_time, 'raceline': raceline, 'raceline_frame': frame}
    speed_scale = LaunchConfiguration('speed_scale').perform(context)
    if speed_scale:
        overrides['speed_scale'] = float(speed_scale)

    return [
        LogInfo(msg=f'pure_pursuit: {raceline} in frame {frame}'),
        # TF pose + raceline -> drive
        Node(
            package='arcsf1tenth_control',
            executable='pure_pursuit.py',
            name='pure_pursuit',
            namespace=namespace,
            output='screen',
            parameters=[
                os.path.join(description_pkg, 'config', 'vehicle.yaml'),
                os.path.join(control_pkg, 'config', 'pure_pursuit.yaml'),
                overrides,
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='arcsf1tenth',
            description='Robot namespace: publishes /<namespace>/drive, TF frames <namespace>/...',
        ),
        DeclareLaunchArgument(
            'track',
            default_value='',
            description='Track folder in arcsf1tenth_tracks: uses maps/sim/slam_map_raceline.csv (frame map), '
                        'else the original maps/sim/raceline.csv (frame world, simulation only)',
        ),
        DeclareLaunchArgument('raceline', default_value='', description='Raceline CSV path (overrides track)'),
        DeclareLaunchArgument('raceline_frame', default_value='', description='TF frame of the raceline x/y'),
        DeclareLaunchArgument('speed_scale', default_value='', description='Fraction of the raceline speed '
                                                                          '(default: pure_pursuit.yaml)'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo clock (false on the real car)',
        ),

        OpaqueFunction(function=launch_setup),
    ])
