import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    description_pkg = get_package_share_directory('arcsf1tenth_description')
    control_pkg = get_package_share_directory('arcsf1tenth_control')
    vehicle_config = os.path.join(description_pkg, 'config', 'vehicle.yaml')
    wall_follow_config = os.path.join(control_pkg, 'config', 'wall_follow.yaml')

    overrides = {'use_sim_time': use_sim_time}
    for name in ('side', 'desired_distance', 'speed_max'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = value if name == 'side' else float(value)

    return [
        # scan -> drive
        Node(
            package='arcsf1tenth_control',
            executable='wall_follow.py',
            name='wall_follow',
            namespace=namespace,
            output='screen',
            parameters=[vehicle_config, wall_follow_config, overrides],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='arcsf1tenth',
            description='Robot namespace: subscribes /<namespace>/scan, publishes /<namespace>/drive',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo clock (false on the real car)',
        ),
        DeclareLaunchArgument('side', default_value='', description='left | right (default: wall_follow.yaml)'),
        DeclareLaunchArgument('desired_distance', default_value='', description='[m] (default: wall_follow.yaml)'),
        DeclareLaunchArgument('speed_max', default_value='', description='[m/s] (default: wall_follow.yaml)'),

        OpaqueFunction(function=launch_setup),
    ])
