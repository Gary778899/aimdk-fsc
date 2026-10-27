"""保留旧命令名称作为独立采集入口别名，全部参数使用新入口定义。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """返回只包含独立采集启动文件的兼容描述，data_root 仍为必填参数。"""
    recorder = (
        Path(get_package_share_directory('data_collection_edge_agent'))
        / 'launch' / 'data_collection_edge_agent.launch.py'
    )
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(recorder))),
    ])
