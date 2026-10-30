#!/usr/bin/env python3
"""
Navigation example: publishes navigation task requests (start/pause/resume/stop)
via an interactive terminal menu.
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node

from aimdk_msgs.msg import MessageHeader, PncTaskRequest
from geometry_msgs.msg import PoseStamped

STARTUP_DELAY = 5.0

# task_request enum
TASK_REQUEST_START = 1
TASK_REQUEST_STOP = 2
TASK_REQUEST_PAUSE = 3
TASK_REQUEST_RESUME = 4


class NavigationExample(Node):

    def __init__(self):
        super().__init__('navigation_example')
        self.publisher = self.create_publisher(
            PncTaskRequest,
            '/aima/te/pnc_task_request',
            10,
        )
        self.task_id = int(time.time() * 1000)
        self.map_id = 0
        self.get_logger().info(
            f'Navigation example node started, task_id={self.task_id}')

    def _make_header(self) -> MessageHeader:
        header = MessageHeader()
        now = self.get_clock().now().to_msg()
        header.stamp = now
        header.meas_stamp = now
        header.frame_id = 'map'
        header.sequence = 0
        return header

    def send_start(self, map_id: int, x: float, y: float, z: float,
                   ox: float, oy: float, oz: float, ow: float,
                   radius: float, pnc_mode: int, max_speed: float):
        """Start navigation."""
        self.map_id = map_id
        msg = PncTaskRequest()
        msg.header = self._make_header()
        msg.task_type = 2
        msg.task_request = TASK_REQUEST_START
        msg.pnc_mode = pnc_mode
        msg.max_forward_speed = max_speed
        msg.task_id = self.task_id
        msg.map_id = map_id
        msg.target_pose_radius = radius
        msg.reserve_info = np.zeros(64, dtype=np.uint8)

        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'map'
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        target.pose.orientation.x = ox
        target.pose.orientation.y = oy
        target.pose.orientation.z = oz
        target.pose.orientation.w = ow
        msg.target_pose = target

        self.publisher.publish(msg)
        self.get_logger().info(
            f'[Start] task_id={self.task_id}, map_id={map_id}, '
            f'target=({x}, {y}, {z}), radius={radius}'
        )

    def send_pause(self):
        """Pause navigation."""
        msg = PncTaskRequest()
        msg.header = self._make_header()
        msg.task_type = 2
        msg.task_request = TASK_REQUEST_PAUSE
        msg.pnc_mode = 0
        msg.max_forward_speed = 0.5
        msg.task_id = self.task_id
        msg.map_id = 0
        msg.target_pose_radius = 0.0
        msg.reserve_info = np.zeros(64, dtype=np.uint8)

        target = PoseStamped()
        target.pose.orientation.w = 1.0
        msg.target_pose = target

        self.publisher.publish(msg)
        self.get_logger().info(f'[Pause] task_id={self.task_id}')

    def send_resume(self):
        """Resume navigation."""
        msg = PncTaskRequest()
        msg.header = self._make_header()
        msg.task_type = 2
        msg.task_request = TASK_REQUEST_RESUME
        msg.pnc_mode = 0
        msg.max_forward_speed = 0.5
        msg.task_id = self.task_id
        msg.map_id = self.map_id
        msg.target_pose_radius = 0.0
        msg.reserve_info = np.zeros(64, dtype=np.uint8)

        target = PoseStamped()
        target.pose.orientation.w = 1.0
        msg.target_pose = target

        self.publisher.publish(msg)
        self.get_logger().info(
            f'[Resume] task_id={self.task_id}, map_id={self.map_id}')

    def send_stop(self):
        """Stop navigation."""
        msg = PncTaskRequest()
        msg.header = self._make_header()
        msg.task_type = 2
        msg.task_request = TASK_REQUEST_STOP
        msg.pnc_mode = 0
        msg.max_forward_speed = 0.5
        msg.task_id = self.task_id
        msg.map_id = self.map_id
        msg.target_pose_radius = 0.0
        msg.reserve_info = np.zeros(64, dtype=np.uint8)

        target = PoseStamped()
        target.pose.orientation.w = 1.0
        msg.target_pose = target

        self.publisher.publish(msg)
        self.get_logger().info(
            f'[Stop] task_id={self.task_id}, map_id={self.map_id}')


def _input_float(prompt: str, default: float) -> float:
    raw = input(f'{prompt} [{default}]: ').strip()
    if not raw:
        return default
    return float(raw)


def _input_int(prompt: str, default: int) -> int:
    raw = input(f'{prompt} [{default}]: ').strip()
    if not raw:
        return default
    return int(raw)


def print_menu(task_id: int):
    print('\n' + '=' * 40)
    print(f'  Navigation Control  (task_id={task_id})')
    print('=' * 40)
    print('  1. Start navigation')
    print('  2. Pause navigation')
    print('  3. Resume navigation')
    print('  4. Stop navigation')
    print('  q. Quit')
    print('=' * 40)


def main():
    rclpy.init()
    nav = NavigationExample()

    nav.get_logger().info(
        f'Waiting for node communication ready ({STARTUP_DELAY}s)...')
    time.sleep(STARTUP_DELAY)

    try:
        while True:
            print_menu(nav.task_id)
            choice = input('Select an option: ').strip()

            if choice == '1':
                print('\n--- Start Navigation Parameters ---')
                map_id = _input_int('Map ID (map_id)', 1773113429735)
                x = _input_float('Target x', 1.0)
                y = _input_float('Target y', 2.0)
                z = _input_float('Target z', 0.0)
                ox = _input_float('Orientation x', 0.0)
                oy = _input_float('Orientation y', 0.0)
                oz = _input_float('Orientation z', 0.0)
                ow = _input_float('Orientation w', 1.0)
                radius = _input_float('Target pose radius', 0.5)
                pnc_mode = _input_int('pnc_mode', 0)
                max_speed = _input_float('Max forward speed', 0.5)
                nav.send_start(map_id, x, y, z, ox, oy, oz,
                               ow, radius, pnc_mode, max_speed)

            elif choice == '2':
                nav.send_pause()

            elif choice == '3':
                nav.send_resume()

            elif choice == '4':
                nav.send_stop()

            elif choice.lower() == 'q':
                print('Exiting navigation control.')
                break

            else:
                print('Invalid input, please try again.')

    except KeyboardInterrupt:
        nav.get_logger().info('Interrupted by user (Ctrl+C)')
    finally:
        nav.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
