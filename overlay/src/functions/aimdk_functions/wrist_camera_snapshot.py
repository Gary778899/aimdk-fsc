"""订阅左右 D405 的首张彩色图，保存 PNG、读回校验并记录采样信息。"""

import argparse
from datetime import datetime, timezone
from functools import partial
import json
import math
from pathlib import Path
import sys
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image

from aimdk_functions.wrist_camera_config import load_camera_config


def parse_arguments(args):
    """解析命令参数并返回配置；args 为去除 ROS 参数后的命令行列表。"""
    parser = argparse.ArgumentParser(description='保存左右腕部相机各一张彩色 PNG')
    parser.add_argument(
        '--output-dir', type=Path, default=Path('data/images'),
        help='保存目录，每次创建独立 UTC 时间子目录（默认：当前目录下 data/images）',
    )
    parser.add_argument(
        '--timeout', type=float, default=10.0,
        help='等待两侧图像的总超时秒数（默认：10）',
    )
    options = parser.parse_args(args)
    if not math.isfinite(options.timeout) or options.timeout <= 0:
        parser.error('--timeout 必须为有限正秒数')
    return options


class WristCameraSnapshot(Node):
    """只读订阅左右相机，为每侧保留启动订阅后收到的第一张彩色图。"""

    def __init__(self, config):
        """按配置创建图像订阅；config 包含命名空间及左右设备绑定。"""
        super().__init__('wrist_camera_snapshot')
        namespace = config['camera_namespace'].strip('/')
        self.topics = {
            side: f'/{namespace}/{side}/color/image_rect_raw'
            for side in config['serial_numbers']
        }
        self.frames = {}
        self._image_subscriptions = [
            self.create_subscription(
                Image, topic, partial(self.receive_image, side),
                qos_profile_sensor_data,
            )
            for side, topic in self.topics.items()
        ]
        for side, topic in self.topics.items():
            self.get_logger().info(f'等待 {side} 彩色图：{topic}')

    def receive_image(self, side, message):
        """保留指定侧的首帧及接收墙钟纳秒；side 为左右侧，message 为原始 Image。"""
        if side not in self.frames:
            self.frames[side] = (message, time.time_ns())
            self.get_logger().info(f'收到 {side} 彩色图：{self.topics[side]}')

    def capture(self, timeout):
        """等待全部图像，返回每侧首帧及接收时间；超过 timeout 秒则报告缺失侧。"""
        deadline = time.monotonic() + timeout
        while len(self.frames) < len(self.topics):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                missing = [side for side in self.topics if side not in self.frames]
                raise RuntimeError(f'等待彩色图超时，缺少：{", ".join(missing)}')
            rclpy.spin_once(self, timeout_sec=remaining)
        return self.frames


def save_images(config, topics, frames, output_root):
    """保存并读回校验两张 PNG，最后写入 manifest.json，返回本次目录。

    config 是相机配置，topics 是左右话题映射，frames 是 Image 和接收时间映射，
    output_root 是保存根目录。转换、写盘或读回校验失败时抛出异常；失败目录保留。
    """
    captured_at = datetime.now(timezone.utc)
    output_dir = output_root.expanduser().resolve() / captured_at.strftime('%Y%m%dT%H%M%S%fZ')
    output_dir.mkdir(parents=True)
    bridge = CvBridge()
    records = {}
    for side, (message, received_ns) in frames.items():
        image = bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        image_path = output_dir / f'{side}.png'
        if not cv2.imwrite(str(image_path), image):
            raise RuntimeError(f'PNG 写入失败：{image_path}')
        decoded = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if decoded is None or not np.array_equal(image, decoded):
            raise RuntimeError(f'PNG 读回校验失败：{image_path}')
        stamp = message.header.stamp
        records[side] = {
            'file': image_path.name,
            'configured_serial_no': config['serial_numbers'][side],
            'topic': topics[side],
            'frame_id': message.header.frame_id,
            'source_stamp': {'sec': stamp.sec, 'nanosec': stamp.nanosec},
            'received_unix_ns': received_ns,
            'width': message.width,
            'height': message.height,
            'source_encoding': message.encoding,
            'conversion_encoding': 'bgr8',
            'png_readback_equal': True,
        }
    manifest = {
        'schema_version': 1,
        'saved_at_utc': captured_at.isoformat(),
        'capture_policy': 'first_received_per_camera',
        'frames': records,
    }
    (output_dir / 'manifest.json').write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8',
    )
    return output_dir


def main(args=None):
    """执行单帧保存并释放 ROS 资源；args 为可选参数列表，失败时以非零码退出。"""
    argv = sys.argv if args is None else ['wrist_camera_snapshot'] + list(args)
    options = parse_arguments(remove_ros_args(args=argv)[1:])
    node = None
    exit_code = 0
    try:
        config = load_camera_config()
        rclpy.init(args=argv[1:])
        node = WristCameraSnapshot(config)
        frames = node.capture(options.timeout)
        output_dir = save_images(config, node.topics, frames, options.output_dir)
        node.get_logger().info(f'左右 PNG 保存及读回校验通过：{output_dir}')
    except KeyboardInterrupt:
        exit_code = 130
    except Exception as exc:
        print(f'单帧保存失败：{exc}', file=sys.stderr)
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
