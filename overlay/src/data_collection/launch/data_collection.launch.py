"""按预设统一启动采集器及选中的腕部相机，START／STOP 控制会话落盘。"""

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_value(context, name):
    """返回启动参数 name 在当前上下文中的字符串值。"""
    return LaunchConfiguration(name).perform(context)


def create_components(context):
    """解析来源与关联内参，创建采集器和所选驱动；未知来源或预设抛出 ValueError。"""
    mapping_path = Path(launch_value(context, 'mapping_file'))
    mapping = json.loads(mapping_path.read_text(encoding='utf-8'))
    catalog = {entry['id']: entry for entry in mapping['streams']}
    explicit = launch_value(context, 'streams').strip()
    profile = launch_value(context, 'profile')
    if explicit:
        selected = [item.strip() for item in explicit.split(',') if item.strip()]
    else:
        profiles = json.loads(Path(launch_value(context, 'profiles_file')).read_text(encoding='utf-8'))
        if profile not in profiles:
            raise ValueError(f'未知采集预设：{profile}')
        selected = list(profiles[profile])
    for stream_id in selected:
        if stream_id not in catalog:
            raise ValueError(f'未知采集来源：{stream_id}')
        for dependency in catalog[stream_id].get('includes', []):
            if dependency not in selected:
                selected.append(dependency)
    selected = list(dict.fromkeys(selected))
    if not selected:
        raise ValueError('采集来源不能为空')
    driver_mode = launch_value(context, 'start_wrist_cameras').lower()
    if driver_mode not in ('true', 'false'):
        raise ValueError('start_wrist_cameras 必须为 true 或 false')
    components = []
    acquisition = dict(profile=profile if not explicit else None, start_wrist_cameras=driver_mode == 'true')
    sides = [side for side in ('left', 'right')
             if any(item.startswith(f'wrist_{side}_') for item in selected)]
    if sides and driver_mode == 'true':
        config_path = (Path(get_package_share_directory('aimdk_functions'))
                       / 'config' / 'wrist_cameras.json')
        camera_config = json.loads(config_path.read_text(encoding='utf-8'))
        acquisition['wrist_camera_config'] = camera_config
        acquisition['wrist_driver_arguments'] = {}
        driver_launch = (Path(get_package_share_directory('realsense2_camera'))
                         / 'launch' / 'rs_launch.py')
        for side in sides:
            parameters = dict(camera_config['stream_arguments'])
            parameters.update(
                camera_namespace=camera_config['camera_namespace'], camera_name=side,
                serial_no='_' + camera_config['serial_numbers'][side],
                enable_color=str(any(item in selected for item in
                                     (f'wrist_{side}_rgb', f'wrist_{side}_rgb_info'))).lower(),
                enable_depth=str(any(item in selected for item in
                                     (f'wrist_{side}_depth', f'wrist_{side}_depth_info'))).lower())
            acquisition['wrist_driver_arguments'][side] = parameters
            components.append(GroupAction([
                IncludeLaunchDescription(PythonLaunchDescriptionSource(str(driver_launch)),
                                         launch_arguments=parameters.items()),
            ]))
    components.append(Node(
        package='data_collection', executable='data_collection_node',
        name='data_collection_node', output='screen', parameters=[{
            'mapping_file': ParameterValue(str(mapping_path), value_type=str), 'selected_streams': selected,
            'data_root': ParameterValue(launch_value(context, 'data_root'), value_type=str),
            'mcap_storage_config': ParameterValue(launch_value(context, 'mcap_storage_config'), value_type=str),
            'acquisition_metadata_json': ParameterValue(json.dumps(acquisition), value_type=str),
            'queue_capacity': int(launch_value(context, 'queue_capacity')),
            'zmq_bind_address': ParameterValue(launch_value(context, 'zmq_bind_address'), value_type=str),
            'zmq_port': int(launch_value(context, 'zmq_port')),
            'robot_model': ParameterValue(launch_value(context, 'robot_model'), value_type=str),
            'firmware': ParameterValue(launch_value(context, 'firmware'), value_type=str),
        }],
    ))
    return components


def generate_launch_description():
    """声明来源预设、驱动复用和存储参数，返回统一启动描述。"""
    config = Path(get_package_share_directory('data_collection')) / 'config'
    arguments = [
        DeclareLaunchArgument('data_root', description='持久化数据根目录的绝对路径。'),
        DeclareLaunchArgument('mapping_file', default_value=str(config / 'x2_sources.json')),
        DeclareLaunchArgument('profiles_file', default_value=str(config / 'collection_profiles.json')),
        DeclareLaunchArgument('profile', default_value='upper_body', description='采集来源预设。'),
        DeclareLaunchArgument('streams', default_value='', description='逗号分隔来源 ID，填写后覆盖预设。'),
        DeclareLaunchArgument('start_wrist_cameras', default_value='true',
                              description='启动所选腕部驱动；false 复用已运行驱动。'),
        DeclareLaunchArgument('mcap_storage_config', default_value=str(config / 'mcap_storage.yaml')),
        DeclareLaunchArgument('queue_capacity', default_value='512'),
        DeclareLaunchArgument('zmq_bind_address', default_value='127.0.0.1'),
        DeclareLaunchArgument('zmq_port', default_value='5555'),
        DeclareLaunchArgument('robot_model', default_value='x2-ultra'),
        DeclareLaunchArgument('firmware', default_value='unknown'),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=create_components)])
