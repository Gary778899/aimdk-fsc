"""Launch the read-only observer using an explicit robot profile."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('locomotion_wrapper'), 'config', 'wbc_observer.yaml']),
            description='Verified joint and timing configuration for the robot'),
        Node(package='locomotion_wrapper', executable='hardware_wrapper_node',
             name='hardware_wrapper_node', output='screen',
             parameters=[LaunchConfiguration('params_file')]),
    ])
