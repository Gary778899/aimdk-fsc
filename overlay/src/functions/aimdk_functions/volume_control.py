#!/usr/bin/env python3
"""Query and set speaker volume and mute state using SDK audio services."""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from aimdk_msgs.msg import CommonState
from aimdk_msgs.srv import GetMute, GetVolume, SetMute, SetVolume


def volume_value(value):
    """Return an integer volume in 0..100; raise ArgumentTypeError otherwise."""
    try:
        volume = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('音量必须为 0–100 的整数') from exc
    if not 0 <= volume <= 100:
        raise argparse.ArgumentTypeError('音量必须为 0–100 的整数')
    return volume


def timeout_value(value):
    """Return finite positive seconds; raise ArgumentTypeError if invalid."""
    try:
        timeout = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('超时必须为有限正数（秒）') from exc
    if not math.isfinite(timeout) or timeout <= 0:
        raise argparse.ArgumentTypeError('超时必须为有限正数（秒）')
    return timeout


def parse_arguments(args):
    """Parse application arguments without ROS options into a Namespace."""
    parser = argparse.ArgumentParser(
        description='查询、设置机器人音量和静音状态',
        epilog='示例：ros2 run aimdk_functions volume_control set-volume 30',
    )
    parser.add_argument(
        '--timeout', type=timeout_value, default=5.0,
        help='服务发现和响应各自的超时秒数（默认：5）',
    )
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('get-volume', help='查询音量')
    set_volume = commands.add_parser('set-volume', help='设置音量（0–100）')
    set_volume.add_argument('volume', type=volume_value, help='音量：0–100')
    commands.add_parser('get-mute', help='查询静音状态')
    set_mute = commands.add_parser('set-mute', help='设置或取消静音')
    set_mute.add_argument(
        'mute', choices=('on', 'off'), help='on：静音；off：取消静音',
    )
    return parser.parse_args(args)


class VolumeControlClient(Node):
    """Call the four SDK audio services and report their returned state."""

    def __init__(self):
        """Create the ROS node; service clients are created when requested."""
        super().__init__('volume_control_client')

    def call_service(self, service_type, request, timeout):
        """Return a successful response, or raise RuntimeError on failure.

        Args:
            service_type: SDK service class defining the request and response.
            request: Request instance with any operation parameters populated.
            timeout: Maximum seconds for discovery and for the response each.
        """
        service_name = '/aimdk_5Fmsgs/srv/' + service_type.__name__
        client = self.create_client(service_type, service_name)
        future = None
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f'等待服务超时：{service_name}')
            request.request.header.stamp = self.get_clock().now().to_msg()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            if not future.done():
                raise RuntimeError(
                    f'等待响应超时：{service_name}；设置请求可能已生效，请查询确认'
                )
            if future.cancelled():
                raise RuntimeError(f'服务请求已取消：{service_name}')
            response = future.result()
            if response is None:
                raise RuntimeError(f'服务未返回响应：{service_name}')
            # The SDK interface intentionally spells this field "reponse".
            if response.reponse.status.value != CommonState.SUCCESS:
                raise RuntimeError(
                    f'{service_name} 失败 '
                    f'(status={response.reponse.status.value})：'
                    f'{response.reponse.message}'
                )
            return response
        finally:
            if future is not None and not future.done():
                future.cancel()
            self.destroy_client(client)

    def execute(self, options):
        """Execute one parsed command and log the state returned by the SDK."""
        service_types = {
            'get-volume': GetVolume,
            'set-volume': SetVolume,
            'get-mute': GetMute,
            'set-mute': SetMute,
        }
        service_type = service_types[options.command]
        request = service_type.Request()
        if options.command == 'set-volume':
            request.audio_volume = options.volume
        elif options.command == 'set-mute':
            request.is_mute = options.mute == 'on'

        response = self.call_service(service_type, request, options.timeout)
        if options.command in ('get-volume', 'set-volume'):
            self.get_logger().info(f'当前音量：{response.audio_volume}/100')
        else:
            state = '已静音' if response.is_mute else '未静音'
            self.get_logger().info(f'当前静音状态：{state}')


def main(args=None):
    """Run the CLI, clean up ROS resources and exit nonzero on failure.

    Args:
        args: Optional argument list excluding the executable; defaults to argv.
    """
    argv = sys.argv if args is None else ['volume_control'] + list(args)
    options = parse_arguments(remove_ros_args(args=argv)[1:])
    node = None
    exit_code = 0
    try:
        rclpy.init(args=argv[1:])
        node = VolumeControlClient()
        node.execute(options)
    except KeyboardInterrupt:
        exit_code = 130
    except Exception as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f'初始化失败：{exc}', file=sys.stderr)
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
