#!/usr/bin/python3
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

class JointWatcher(Node):
    def __init__(self):
        super().__init__('joint_watcher')
        self.joint_name = self.declare_parameter('joint_name', 'left_knee_joint').value
        self.topic = self.declare_parameter('topic', '/humanoid/joint_states').value
        self.last_msg_ns = None
        self.timeout_sec = self.declare_parameter('timeout_sec', 1.0).value
        self.sub = self.create_subscription(
            JointState, self.topic, self.cb, qos_profile_sensor_data
        )
        self.timer = self.create_timer(2.0, self._heartbeat)
        self.get_logger().info(
            f'Subscribing to {self.topic} for joint {self.joint_name} '
            '(SensorDataQoS/BEST_EFFORT compatible).'
        )

    def _heartbeat(self):
        if self.last_msg_ns is None or (
            time.monotonic_ns() - self.last_msg_ns
        ) / 1e9 > self.timeout_sec:
            self.get_logger().warn(
                f'No recent JointState received on {self.topic}. '
                'Ensure publisher is running and topic name matches.'
            )

    def cb(self, msg: JointState):
        self.last_msg_ns = time.monotonic_ns()
        try:
            i = msg.name.index(self.joint_name)
        except ValueError:
            self.get_logger().warn(f'{self.joint_name} not found in current JointState')
            return
        pos = msg.position[i] if i < len(msg.position) else float('nan')
        vel = msg.velocity[i] if i < len(msg.velocity) else float('nan')
        eff = msg.effort[i] if i < len(msg.effort) else float('nan')
        self.get_logger().info(
            f'{self.joint_name}: pos={pos:.6f}, vel={vel:.6f}, eff={eff:.6f}'
        )

def main():
    rclpy.init()
    node = JointWatcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()