"""按固定序列号启动左右 D405，分别发布到 /wrist/left 和 /wrist/right。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from aimdk_functions.wrist_camera_config import load_camera_config


def generate_launch_description():
    """返回双相机启动描述；每侧独立调用官方启动文件并隔离启动参数作用域。"""
    config = load_camera_config()
    driver_launch = (
        Path(get_package_share_directory('realsense2_camera'))
        / 'launch' / 'rs_launch.py'
    )
    cameras = []
    for side, serial_no in config['serial_numbers'].items():
        arguments = dict(config['stream_arguments'])
        arguments.update({
            'camera_namespace': config['camera_namespace'],
            'camera_name': side,
            'serial_no': '_' + serial_no,
        })
        cameras.append(GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(driver_launch)),
                launch_arguments=arguments.items(),
            ),
        ]))
    return LaunchDescription(cameras)
