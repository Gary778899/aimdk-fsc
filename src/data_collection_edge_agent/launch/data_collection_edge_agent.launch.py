from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bag_root_dir_arg = DeclareLaunchArgument(
        "bag_root_dir",
        default_value="./rosbags",
        description="Root directory where timestamped MCAP bag folders are created.",
    )

    joint_state_topic_arg = DeclareLaunchArgument(
        "joint_state_topic",
        default_value="/humanoid/joint_states",
        description="JointState topic to record.",
    )

    imu_topic_arg = DeclareLaunchArgument(
        "imu_topic",
        default_value="/humanoid/imu",
        description="IMU topic to record.",
    )

    zmq_bind_address_arg = DeclareLaunchArgument(
        "zmq_bind_address",
        default_value="127.0.0.1",
        description="IP address where the ZMQ REP socket binds.",
    )

    zmq_port_arg = DeclareLaunchArgument(
        "zmq_port",
        default_value="5555",
        description="TCP port where the ZMQ REP socket listens for START/STOP.",
    )

    node = Node(
        package="data_collection_edge_agent",
        executable="data_collection_edge_agent_node",
        name="data_collection_edge_agent_node",
        output="screen",
        parameters=[
            {
                "bag_root_dir": LaunchConfiguration("bag_root_dir"),
                "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "zmq_bind_address": LaunchConfiguration("zmq_bind_address"),
                "zmq_port": ParameterValue(LaunchConfiguration("zmq_port"), value_type=int),
            }
        ],
    )

    return LaunchDescription(
        [
            bag_root_dir_arg,
            joint_state_topic_arg,
            imu_topic_arg,
            zmq_bind_address_arg,
            zmq_port_arg,
            node,
        ]
    )
