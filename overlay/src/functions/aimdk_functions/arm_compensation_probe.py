#!/usr/bin/env python3
"""按 SD/URS 标签只读记录运控拓扑、模式、上肢命令及反馈，供离线分析。"""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import platform
import sys
import time
import uuid


MODES = {'SD': 'STAND_DEFAULT', 'URS': 'UPPERBODY_REMOTE_SPLIT'}
TOPICS = {
    '/mc/upper_body_command': 'aimdk_msgs/msg/UpperBodyCommandArray',
    '/aima/mc/common/state': 'aimdk_msgs/msg/McCommonState',
    '/aima/hal/joint/arm/command': 'aimdk_msgs/msg/JointCommandArray',
    '/aima/hal/joint/arm/state': 'aimdk_msgs/msg/JointStateArray',
    '/aima/hal/joint/head/command': 'aimdk_msgs/msg/JointCommandArray',
    '/aima/hal/joint/head/state': 'aimdk_msgs/msg/JointStateArray',
    '/aima/hal/joint/hand/command': 'aimdk_msgs/msg/HandCommandArray',
    '/aima/hal/joint/hand/state': 'aimdk_msgs/msg/HandStateArray',
    '/aima/hal/joint/waist/command': 'aimdk_msgs/msg/JointCommandArray',
    '/aima/hal/joint/waist/state': 'aimdk_msgs/msg/JointStateArray',
    '/aima/hal/imu/chest/state': 'sensor_msgs/msg/Imu',
    '/aima/hal/imu/torso/state': 'sensor_msgs/msg/Imu',
    '/aima/mc_debug_f64': 'std_msgs/msg/Float64MultiArray',
    '/aima/mc/rl/debug': 'aimdk_msgs/msg/McRLDebug',
}
SERVICE = '/aimdk_5Fmsgs/srv/GetMcAction'


def finite_json(value):
    """递归转换消息字典；非有限浮点保留为字符串，保证生成标准 JSON。"""
    if isinstance(value, dict):
        return {str(k): finite_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value


def write_json(path, value):
    """独占创建 UTF-8 JSON 文件；路径冲突或写盘失败时抛出异常。"""
    with path.open('x', encoding='utf-8') as stream:
        json.dump(finite_json(value), stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def positive_number(text):
    """解析正有限秒数或频率；非法值向 argparse 报错。"""
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError('必须为正有限数')
    return value


def parse_args():
    """解析预期模式、采集时长、保存频率和日志根目录。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tag', type=str.upper, choices=MODES, required=True)
    parser.add_argument('--seconds', type=positive_number, default=15.0,
                        help='正式采集秒数，默认 15；不含发现等待')
    parser.add_argument('--sample-hz', type=positive_number, default=50.0,
                        help='每话题最多保存频率，默认 50 Hz；不会提高源频率')
    parser.add_argument('--discovery-seconds', type=positive_number, default=3.0,
                        help='ROS 图发现等待秒数，默认 3')
    parser.add_argument('--log-dir', type=Path, help='默认自动定位工作区 log/')
    return parser.parse_args()


def session_directory(args):
    """创建唯一会话目录；未找到工作区且未指定路径时抛出 ValueError。"""
    root = args.log_dir
    if root is None:
        candidates = (Path.cwd(), *Path.cwd().parents, *Path(__file__).resolve().parents)
        workspace = next((p for marker in ('overlay/src/functions/package.xml', 'src/functions/package.xml')
                          for p in candidates
                          if (p / marker).is_file()), None)
        if workspace is None:
            raise ValueError('无法定位工作区，请传入 --log-dir /实际工作区/log')
        root = workspace / 'log'
    label = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    directory = root.expanduser().resolve() / f'arm_probe_{args.tag}_{label}_{uuid.uuid4().hex[:8]}'
    directory.mkdir(parents=True, exist_ok=False)
    return directory


def graph_snapshot(node):
    """读取当前 ROS 图及目标话题端点 QoS；不调用控制服务。"""
    topics = {}
    for topic in TOPICS:
        endpoints = {}
        for role, getter in (('publishers', node.get_publishers_info_by_topic),
                             ('subscriptions', node.get_subscriptions_info_by_topic)):
            endpoints[role] = [dict(node_name=e.node_name, namespace=e.node_namespace,
                                    topic_type=e.topic_type, gid=list(e.endpoint_gid),
                                    qos=str(e.qos_profile)) for e in getter(topic)]
        topics[topic] = endpoints
    return dict(wall_time_ns=time.time_ns(), nodes=node.get_node_names_and_namespaces(),
                topics_and_types=node.get_topic_names_and_types(),
                services_and_types=node.get_service_names_and_types(), endpoints=topics)


def write_report(directory, summary):
    """生成便于终端查看及回传的中文报告；不推断是否启用了重力补偿。"""
    lines = [f"标签：{summary['tag']}，预期模式：{summary['expected_mode']} / RUNNING(100)",
             f"退出码：{summary['exit_code']}，中断：{summary['interrupted']}",
             f"实际采集秒数：{summary.get('actual_capture_seconds', '未开始')}",
             f"服务确认目标模式：{summary['mode_confirmed']}",
             f"观察到模式不匹配：{summary['mode_mismatch_observed']}",
             '模式观测（来源、模式、状态）：']
    modes = dict.fromkeys((o['source'], o['mode'], o['status'])
                          for o in summary['mode_observations'])
    lines.extend(f'  {source}: {mode} / {status}' for source, mode, status in modes)
    lines.append('话题统计（收到 / 保存 / 回调观测Hz；不代表硬件频率）：')
    for topic in TOPICS:
        stats = summary['topics'].get(topic, {})
        lines.append(f"  {topic}: {stats.get('received', 0)} / {stats.get('saved', 0)} / "
                     f"{stats.get('observed_callback_hz', '不可估算')}")
    lines.append('缺失的必需流：' + ', '.join(summary['missing_required_topics']))
    lines.append('错误：')
    lines.extend('  ' + error for error in summary['errors'])
    lines.append('本报告仅验证观测过程；重力补偿是否存在需结合消息与控制器实现分析。')
    with (directory / 'report.log').open('x', encoding='utf-8') as stream:
        stream.write('\n'.join(lines) + '\n')


def collect(args, directory):
    """在 Humble 环境只读采集；返回状态码，失败和中断也尽量保留原始日志。"""
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message
    from aimdk_msgs.srv import GetMcAction

    rclpy.init(args=[])
    node = None
    stream = None
    summary = dict(tag=args.tag, expected_mode=MODES[args.tag],
                   requested_seconds=args.seconds, sample_hz=args.sample_hz,
                   topics={}, mode_observations=[], errors=[], interrupted=False)
    started = time.monotonic()
    active = False
    mode_failed = False
    future = None
    request_started = 0.0
    capture_started = None

    def emit(event, **fields):
        """逐行保存事件，附带接收墙钟与单调时钟；写盘错误向外传播。"""
        record = dict(event=event, wall_time_ns=time.time_ns(),
                      monotonic_ns=time.monotonic_ns(), **fields)
        stream.write(json.dumps(finite_json(record), ensure_ascii=False, allow_nan=False) + '\n')

    def observe_mode(info, source):
        """记录实际模式及 RUNNING 匹配结果；不切模式、不提前停止采集。"""
        nonlocal mode_failed
        match = info.action_desc == MODES[args.tag] and info.status.value == 100
        entry = dict(source=source, elapsed_s=time.monotonic()-started,
                     mode=info.action_desc, status=info.status.value, matches_expected=match)
        summary['mode_observations'].append(entry)
        mode_failed |= not match
        emit('mode', **entry)

    def make_callback(topic):
        """为指定话题创建计数、源时间检查及限频保存回调。"""
        stats = dict(received=0, saved=0, source_stamp_repeat=0,
                     source_stamp_regression=0, source_stamp_zero=0)
        summary['topics'][topic] = stats
        last_saved = None
        last_stamp = None

        def callback(msg):
            """接收完整消息；按话题限频写盘，所有收到的样本均参与计数。"""
            nonlocal last_saved, last_stamp
            if not active:
                return
            now = time.monotonic()
            stats['received'] += 1
            stats.setdefault('first_receive_elapsed_s', now-started)
            stats['last_receive_elapsed_s'] = now-started
            header = getattr(msg, 'header', None)
            if header is not None and hasattr(header, 'stamp'):
                stamp = header.stamp.sec * 1000000000 + header.stamp.nanosec
                stats['source_stamp_zero'] += int(stamp == 0)
                if last_stamp is not None:
                    stats['source_stamp_repeat'] += int(stamp == last_stamp)
                    stats['source_stamp_regression'] += int(stamp < last_stamp)
                last_stamp = stamp
            if topic == '/aima/mc/common/state':
                previous = summary.get('last_topic_mode')
                current = [msg.action_info.action_desc, msg.action_info.status.value]
                if current != previous:
                    observe_mode(msg.action_info, topic)
                    summary['last_topic_mode'] = current
            if last_saved is None or now-last_saved >= 1.0/args.sample_hz:
                emit('message', topic=topic, message=message_to_ordereddict(msg))
                stats['saved'] += 1
                last_saved = now

        return callback

    try:
        stream = (directory / 'events.jsonl').open('x', encoding='utf-8', buffering=1)
        node = rclpy.create_node(f'arm_compensation_probe_{os.getpid()}')
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        subscriptions = []
        for topic, type_name in TOPICS.items():
            try:
                msg_type = get_message(type_name)
            except (ImportError, AttributeError, ModuleNotFoundError) as exc:
                summary['errors'].append(f'{topic}: 消息类型加载失败: {exc}')
                continue
            subscriptions.append(node.create_subscription(msg_type, topic, make_callback(topic), qos))
        client = node.create_client(GetMcAction, SERVICE)
        deadline = time.monotonic() + args.discovery_seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        write_json(directory / 'graph_start.json', graph_snapshot(node))
        started = time.monotonic()
        capture_started = started
        active = True
        emit('capture_start', tag=args.tag, expected_mode=MODES[args.tag])
        next_request = started
        while rclpy.ok() and time.monotonic()-started < args.seconds:
            now = time.monotonic()
            if future is not None and future.done():
                response = future.result()
                emit('get_mc_action_response', response=message_to_ordereddict(response))
                if response.header.code != 0:
                    summary['errors'].append('GetMcAction 返回非零 code，详见原始响应')
                else:
                    observe_mode(response.info, SERVICE)
                future = None
            if future is not None and now-request_started >= 2.0:
                future.cancel()
                future = None
                summary['errors'].append('GetMcAction 请求超时（2 秒）')
                emit('get_mc_action_timeout')
            if future is None and now >= next_request:
                next_request = now + 1.0
                if client.service_is_ready():
                    request = GetMcAction.Request()
                    request.request.header.stamp = node.get_clock().now().to_msg()
                    future = client.call_async(request)
                    request_started = now
                else:
                    emit('get_mc_action_unavailable')
            rclpy.spin_once(node, timeout_sec=0.05)
        active = False
        summary['actual_capture_seconds'] = time.monotonic()-started
        write_json(directory / 'graph_end.json', graph_snapshot(node))
    except (KeyboardInterrupt, ExternalShutdownException):
        summary['interrupted'] = True
    except Exception as exc:
        summary['errors'].append(f'{type(exc).__name__}: {exc}')
    finally:
        active = False
        if capture_started is not None:
            summary.setdefault('actual_capture_seconds', time.monotonic()-capture_started)
        if future is not None and not future.done():
            future.cancel()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if stream is not None:
            stream.close()
        required = ('/aima/hal/joint/arm/command', '/aima/hal/joint/arm/state')
        summary['missing_required_topics'] = [t for t in required
                                             if not summary['topics'].get(t, {}).get('saved')]
        summary['topics_without_samples'] = [t for t in TOPICS
                                            if not summary['topics'].get(t, {}).get('saved')]
        for stats in summary['topics'].values():
            if stats['received'] > 1:
                duration = stats['last_receive_elapsed_s'] - stats['first_receive_elapsed_s']
                stats['observed_callback_hz'] = (stats['received']-1)/duration if duration else None
        summary['mode_confirmed'] = any(o['source'] == SERVICE and o['matches_expected']
                                        for o in summary['mode_observations'])
        summary['mode_mismatch_observed'] = mode_failed
        summary['exit_code'] = (130 if summary['interrupted'] else
                                2 if summary['errors'] or mode_failed or
                                not summary['mode_confirmed'] or
                                summary['missing_required_topics'] else 0)
        write_json(directory / 'summary.json', summary)
        write_report(directory, summary)
    return summary['exit_code']


def main():
    """创建会话并执行采集；返回 0/2/130，环境或文件错误返回 1。"""
    args = parse_args()
    directory = None
    try:
        directory = session_directory(args)
        write_json(directory / 'metadata.json', dict(
            schema_version=1, tag=args.tag, expected_mode=MODES[args.tag],
            command=sys.argv, hostname=platform.node(), platform=platform.platform(),
            python=sys.version, cwd=str(Path.cwd()), script=str(Path(__file__).resolve()),
            environment={k: os.environ.get(k) for k in
                         ('ROS_DISTRO', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION',
                          'ROS_LOCALHOST_ONLY', 'AMENT_PREFIX_PATH')},
            topics=TOPICS, subscription_qos='best_effort / volatile / keep_last depth=100',
            sample_hz=args.sample_hz, seconds=args.seconds,
            notes=['只读，不切换模式、不发布运动命令。',
                   '消息按话题限频保存，不保证全量、严格同步或硬件同步。',
                   '没有自动推断重力补偿是否存在；退出码 0 仅表示观测完整且模式匹配。']))
        print(f'只读观测：{args.tag} / {MODES[args.tag]}\n日志目录：{directory}', flush=True)
        result = collect(args, directory)
        print(f'采集结束，退出码 {result}；请先查看 report.log，再查看 events.jsonl', flush=True)
        return result
    except Exception as exc:
        if directory is not None:
            try:
                write_json(directory / 'failure.json',
                           dict(error_type=type(exc).__name__, message=str(exc)))
            except OSError:
                pass
        print(f'采集失败：{type(exc).__name__}: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
