import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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


def launch_setup(context):
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    prefix = f'{namespace}/' if namespace else ''  # TF frames: <namespace>/<link>
    use_gui = LaunchConfiguration('gui')

    description_pkg = get_package_share_directory('arcsf1tenth_description')
    bringup_pkg = get_package_share_directory('arcsf1tenth_bringup')

    xacro_file = os.path.join(description_pkg, 'urdf', 'f1tenth.urdf.xacro')
    rviz_config = namespaced_rviz_config(os.path.join(bringup_pkg, 'config', 'display.rviz'), namespace)

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{'robot_description': robot_description, 'frame_prefix': prefix}],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            namespace=namespace,
            condition=IfCondition(use_gui),
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=namespace,
            condition=UnlessCondition(use_gui),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='arcsf1tenth',
            description='Robot namespace: topics /<namespace>/..., TF frames <namespace>/...',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Use joint_state_publisher_gui to move the joints with sliders',
        ),

        OpaqueFunction(function=launch_setup),
    ])
