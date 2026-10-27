"""独立启动双臂和夹爪状态采集器；数据目录必须由操作者明确指定。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """声明映射、持久化目录与控制参数，返回独立采集节点的启动描述。"""
    default_mapping = (
        Path(get_package_share_directory('data_collection_edge_agent'))
        / 'config' / 'x2_upper_body.json'
    )
    arguments = [
        DeclareLaunchArgument(
            'data_root', description='已确认持久化的数据根目录，必须使用绝对路径。'),
        DeclareLaunchArgument(
            'mapping_file', default_value=str(default_mapping),
            description='SDK 来源到双臂与夹爪统一状态的 JSON 映射。'),
        DeclareLaunchArgument(
            'record_raw_mcap', default_value='false',
            description='是否额外按来源频率保存完整 SDK 原始消息到 MCAP。'),
        DeclareLaunchArgument(
            'queue_capacity', default_value='512',
            description='异步写盘队列的记录条目容量。'),
        DeclareLaunchArgument(
            'zmq_bind_address', default_value='127.0.0.1',
            description='START、STOP、STATUS 控制端口绑定地址。'),
        DeclareLaunchArgument('zmq_port', default_value='5555', description='ZMQ 控制端口。'),
        DeclareLaunchArgument('robot_model', default_value='unknown', description='机器人具体机型。'),
        DeclareLaunchArgument('firmware', default_value='unknown', description='机器人固件版本。'),
    ]
    node = Node(
        package='data_collection_edge_agent',
        executable='data_collection_edge_agent_node',
        name='data_collection_edge_agent_node',
        output='screen',
        parameters=[{
            'data_root': ParameterValue(LaunchConfiguration('data_root'), value_type=str),
            'mapping_file': ParameterValue(LaunchConfiguration('mapping_file'), value_type=str),
            'record_raw_mcap': ParameterValue(
                LaunchConfiguration('record_raw_mcap'), value_type=bool),
            'queue_capacity': ParameterValue(LaunchConfiguration('queue_capacity'), value_type=int),
            'zmq_bind_address': ParameterValue(
                LaunchConfiguration('zmq_bind_address'), value_type=str),
            'zmq_port': ParameterValue(LaunchConfiguration('zmq_port'), value_type=int),
            'robot_model': ParameterValue(LaunchConfiguration('robot_model'), value_type=str),
            'firmware': ParameterValue(LaunchConfiguration('firmware'), value_type=str),
        }],
    )
    return LaunchDescription(arguments + [node])
