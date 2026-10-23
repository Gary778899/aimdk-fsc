"""Read-only configured observer plus recorder; expose both launch options."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    recorder = Path(get_package_share_directory('data_collection_edge_agent')) / 'launch' / 'data_collection_edge_agent.launch.py'
    observer = Path(get_package_share_directory('locomotion_wrapper')) / 'launch' / 'observer.launch.py'
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(observer))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(recorder))),
    ])
