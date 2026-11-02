"""只读观察原生上肢和HAL手臂命令话题的发布者、QoS及原始消息。"""

import argparse
from functools import partial
import json
import math
import time

from aimdk_functions.upper_body_diagnostics import UpperBodyLog, log_directory


TOPICS = ('/mc/upper_body_command', '/aima/hal/joint/arm/command')


def parse_options(args):
    """解析观察秒数和日志目录；duration为0时持续到Ctrl-C，非法时间由argparse报错。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=30.,
                        help='观察秒数，默认30；0表示持续到Ctrl-C')
    parser.add_argument('--log-dir', help='默认使用检测到的工作区log目录')
    options = parser.parse_args(args)
    if not math.isfinite(options.duration) or options.duration < 0:
        parser.error('--duration须为有限非负秒数')
    return options


def publisher_description(endpoint):
    """将Humble发布端点信息转换为可记录字典，保留节点、类型、GID及主要QoS。"""
    qos = endpoint.qos_profile
    return dict(node_name=endpoint.node_name, node_namespace=endpoint.node_namespace,
                topic_type=endpoint.topic_type, endpoint_gid=list(endpoint.endpoint_gid),
                qos=dict(reliability=qos.reliability.name, durability=qos.durability.name,
                         history=qos.history.name, depth=qos.depth))


class CommandWatch:
    """订阅两个命令话题并轮询发布端点；只保存观测，不创建命令发布器或模式客户端。"""

    def __init__(self, node, diagnostics):
        """使用BEST_EFFORT/VOLATILE订阅以兼容常见发布QoS；diagnostics负责JSONL写入。"""
        from aimdk_msgs.msg import JointCommandArray, UpperBodyCommandArray
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

        self.node = node
        self.diagnostics = diagnostics
        self.counts = dict.fromkeys(TOPICS, 0)
        self.previous_counts = self.counts.copy()
        self.previous_endpoints = {}
        self.last_summary = time.monotonic()
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.subscriptions = []
        for topic, kind in zip(TOPICS, (UpperBodyCommandArray, JointCommandArray)):
            self.subscriptions.append(node.create_subscription(
                kind, topic, partial(self.receive, topic), qos))
        diagnostics.record('watch_subscription', topics=list(TOPICS),
                           qos=dict(reliability='BEST_EFFORT', durability='VOLATILE', depth=100))

    def receive(self, topic, msg):
        """记录每条已收到消息的完整字段和接收时间；首条消息同时在终端完整显示。"""
        from rosidl_runtime_py.convert import message_to_ordereddict

        received = time.monotonic()
        raw = message_to_ordereddict(msg)
        self.counts[topic] += 1
        self.diagnostics.record('command_observed', topic=topic,
                                received_monotonic_s=received,
                                received_ros_time_ns=self.node.get_clock().now().nanoseconds,
                                count=self.counts[topic], raw=raw)
        if self.counts[topic] == 1:
            print(f'首条消息 {topic}：\n{json.dumps(raw, ensure_ascii=False)}', flush=True)

    def inspect_publishers(self):
        """查询并记录发布端点变化；零发布者也记录，DDS发现延迟由后续轮询体现。"""
        for topic in TOPICS:
            endpoints = [publisher_description(item)
                         for item in self.node.get_publishers_info_by_topic(topic)]
            endpoints.sort(key=lambda item: (item['node_namespace'], item['node_name'],
                                             item['endpoint_gid']))
            if endpoints == self.previous_endpoints.get(topic):
                continue
            self.previous_endpoints[topic] = endpoints
            self.diagnostics.record('publishers', topic=topic, endpoints=endpoints)
            print(f'发布端点 {topic}：{len(endpoints)} 个\n'
                  f'{json.dumps(endpoints, ensure_ascii=False)}', flush=True)

    def summarize(self):
        """输出各话题累计接收数及本观察窗口接收频率，并保存当前发布端点快照。"""
        now = time.monotonic()
        elapsed = now - self.last_summary
        rates = {topic: (count-self.previous_counts[topic])/elapsed
                 for topic, count in self.counts.items()}
        self.diagnostics.record('watch_summary', counts=self.counts, received_hz=rates,
                                window_seconds=elapsed, publishers=self.previous_endpoints)
        for topic in TOPICS:
            print(f'{topic}：已收 {self.counts[topic]} 条，最近 {rates[topic]:.1f} Hz', flush=True)
        self.previous_counts = self.counts.copy()
        self.last_summary = now


def main(args=None):
    """在机器人Humble容器只读观察；返回0正常、130中断、1异常，并始终关闭日志和ROS资源。"""
    options = parse_options(args)
    diagnostics = node = None
    initialized = False
    reason, result = 'failed', 1
    try:
        import rclpy
        from rclpy.signals import SignalHandlerOptions

        diagnostics = UpperBodyLog(log_directory(options.log_dir),
                                   command='upper_body_command_watch', options=vars(options))
        print(f'只读观察；日志：{diagnostics.path}', flush=True)
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = rclpy.create_node('upper_body_command_watch')
        watch = CommandWatch(node, diagnostics)
        began = time.monotonic()
        next_graph, next_summary = began, began + 5.
        try:
            while rclpy.ok():
                now = time.monotonic()
                if options.duration and now - began >= options.duration:
                    break
                if now >= next_graph:
                    watch.inspect_publishers()
                    next_graph = now + 1.
                if now >= next_summary:
                    watch.summarize()
                    next_summary = now + 5.
                rclpy.spin_once(node, timeout_sec=.1)
            result, reason = 0, 'completed'
        finally:
            watch.summarize()
    except KeyboardInterrupt:
        result, reason = 130, 'interrupted'
    except Exception as error:
        if diagnostics is not None:
            diagnostics.record('error', error_type=type(error).__name__, message=str(error))
        print(f'观察失败：{error}', flush=True)
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if initialized and rclpy.ok():
                rclpy.shutdown()
        finally:
            if diagnostics is not None:
                diagnostics.close(reason=reason)
    return result
