"""在机器人 Humble 容器内导出示教命令与 MCAP 观测的索引及时间差。"""

import argparse
from array import array
from bisect import bisect_right
from decimal import Decimal, InvalidOperation
import json
from pathlib import Path
import sys


XR_TOPIC = '/remote_control/xr/raw'
XR_SCHEMA = 'remote_control.xr_raw.v1'


def read_jsonl(path):
    """逐行读入日志 path，返回行号、事件名和字段；格式错误报告源位置。"""
    with path.open(encoding='utf-8') as stream:
        for number, line in enumerate(stream, 1):
            try:
                row = json.loads(line)
                yield number, row['event'], row['fields']
            except (ValueError, KeyError, TypeError) as error:
                raise ValueError(f'{path}:{number}：{error}') from error


def read_timeline(directory, topics, trial_id):
    """从已关闭 MCAP 读取指定话题的 ROS 接收时间及 XR 事件标识，位置按读回话题从零计数。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(directory / 'raw'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    topics_present = {item.name for item in reader.get_all_topics_and_types()}
    missing = (set(topics) | {XR_TOPIC}) - topics_present
    if missing:
        raise ValueError(f'MCAP 未登记所需话题：{sorted(missing)}')
    timeline = {topic: array('q') for topic in topics}
    xr_events, xr_count = {}, 0
    while reader.has_next():
        topic, payload, received_ns = reader.read_next()
        if topic == XR_TOPIC:
            index = xr_count
            xr_count += 1
            event = json.loads(deserialize_message(payload, String).data)
            if event['schema'] != XR_SCHEMA:
                raise ValueError(f'不支持的 XR 原始事件格式：{event["schema"]}')
            if event.get('session_id') != trial_id:
                continue
            key = (event['sdk_connection_id'], event['callback_seq'])
            if key in xr_events:
                raise ValueError(f'MCAP 有重复 XR 事件：{key}')
            xr_events[key] = dict(topic_message_index=index, receive_ros_ns=received_ns)
        elif topic in timeline:
            stamps = timeline[topic]
            if stamps and received_ns < stamps[-1]:
                raise ValueError(f'{topic} 的 MCAP 接收时间回退，无法按单一时间轴配对')
            stamps.append(received_ns)
    return timeline, xr_events


def read_xr_frames(path, trial_id):
    """核对日志 path 的试次归属，建立 input_seq 到原始回调和时间的索引。"""
    frames, events = {}, {}
    for line, kind, fields in read_jsonl(path):
        if line == 1 and (kind != 'metadata' or fields.get('session_id') != trial_id):
            raise ValueError(f'{path} 的会话编号与 experiment.json 不一致')
        if kind == 'sdk_event':
            raw = fields['raw']
            key = raw['sdk_connection_id'], raw['callback_seq']
            if key in events:
                raise ValueError(f'{path}:{line}：重复 SDK 回调标识 {key}')
            events[key] = dict(sdk_connection_id=key[0], callback_seq=key[1],
                               sdk_event_log_line=line, received_system_time_ns=raw['received_system_time_ns'],
                               received_monotonic_ns=raw['received_monotonic_ns'])
        elif kind == 'xr_frame':
            frame = fields['frame']
            key = fields['sdk_connection_id'], fields['callback_seq']
            if frame['seq'] in frames or key not in events:
                raise ValueError(f'{path}:{line}：输入编号重复或没有对应的 SDK 回调')
            frames[frame['seq']] = dict(events[key], frame_log_line=line,
                                        source_time_ns=frame['source_time_ns'])
    return frames


def clock_report(session):
    """读取采集接收门的时钟锚点，报告本轮 ROS 与系统时钟偏差的变化。"""
    if session['metadata']['use_sim_time']:
        raise ValueError('示教配对要求实机 ROS 时钟')
    anchors = session['clock_anchors']
    opened, closed = anchors['gate_opened'], anchors['gate_closed']
    if (closed['ros_time_ns'] <= opened['ros_time_ns'] or
            closed['monotonic_after_ns'] <= opened['monotonic_before_ns']):
        raise ValueError('采集结束的时钟锚点没有推进')
    return dict(
        anchors=anchors,
        ros_minus_system_change_ns=(closed['ros_time_ns'] - closed['system_time_ns'] -
                                    opened['ros_time_ns'] + opened['system_time_ns']),
        ros_elapsed_minus_monotonic_ns=(closed['ros_time_ns'] - opened['ros_time_ns'] -
                                        closed['monotonic_before_ns'] + opened['monotonic_before_ns']))


def align_command(fields, line, timeline, frames, xr_events, max_age_ns):
    """按输入标识和发布时刻构建一条配对记录；超龄或无样本时保留缺样状态。"""
    command_ns = fields['command_stamp_ns']
    input_seq = fields['input_seq']
    frame = frames.get(input_seq)
    if input_seq is not None and frame is None:
        raise ValueError(f'控制日志第 {line} 行找不到 XR 输入 {input_seq}')
    xr = None
    if frame is not None:
        key = frame['sdk_connection_id'], frame['callback_seq']
        xr = dict(frame, mcap=xr_events.get(key))
    observations = {}
    for topic, stamps in timeline.items():
        index = bisect_right(stamps, command_ns) - 1
        if index < 0:
            observations[topic] = dict(status='no_prior_sample')
            continue
        delta = stamps[index] - command_ns
        if max_age_ns is not None and -delta > max_age_ns:
            observations[topic] = dict(status='too_old', nearest_delta_ns=delta)
            continue
        observations[topic] = dict(status='matched', topic_message_index=index,
                                   receive_ros_ns=stamps[index], delta_ns=delta)
    return dict(
        control_log_line=line, command_sequence=fields['sequence'], input_seq=input_seq,
        command_stamp_ns=command_ns, publish_begin_monotonic_ns=fields['publish_begin_monotonic_ns'],
        sent_arm_rad=fields['arm_rad'], sent_hand_openings=fields['hand_openings'], xr=xr,
        used_arm_feedback=dict(serial=fields['arm_feedback_serial'], stamp_s=fields['arm_feedback_stamp_s'],
                               received_monotonic_s=fields['arm_feedback_received_monotonic_s'],
                               positions_rad=fields['actual_arm_rad']),
        observations=observations)


def export_alignment(directory, stream_ids, max_age_ns):
    """读回单次会话并返回元数据和配对行，使用当前录制格式及有效采集窗口。"""
    session = json.loads((directory / 'session.json').read_text(encoding='utf-8'))
    experiment = json.loads((directory / 'experiment.json').read_text(encoding='utf-8'))
    if not session['closed']:
        raise ValueError('请先关闭采集会话')
    clocks = clock_report(session)
    streams = {entry['id']: entry['topic'] for entry in session['mapping']['streams']}
    observations = stream_ids or [stream_id for stream_id in streams if stream_id != 'xr_raw']
    unsupported = (set(observations) - streams.keys()) | (set(observations) & {'xr_raw'})
    if unsupported:
        raise ValueError(f'无效观测来源：{sorted(unsupported)}')
    artifacts = experiment['artifacts']
    if not artifacts['xr'] or not artifacts['control']:
        raise ValueError('会话缺少 XR 或控制日志')
    trial_id = experiment['trial_id']
    frames = read_xr_frames(directory / artifacts['xr'], trial_id)
    timeline, xr_events = read_timeline(directory, [streams[stream_id] for stream_id in observations],
                                       trial_id)
    first_ros_ns = clocks['anchors']['gate_opened']['ros_time_ns']
    last_ros_ns = clocks['anchors']['gate_closed']['ros_time_ns']
    first_monotonic_ns = clocks['anchors']['gate_opened']['monotonic_before_ns']
    last_monotonic_ns = clocks['anchors']['gate_closed']['monotonic_after_ns']
    rows, previous = [], first_ros_ns
    arm_names = None
    for line, event, fields in read_jsonl(directory / artifacts['control']):
        if line == 1 and (event != 'metadata' or fields.get('session_id') != trial_id):
            raise ValueError('控制日志的会话编号与 experiment.json 不一致')
        if event == 'snapshot':
            arm_names = fields['arm_names']
        if event != 'command_published':
            continue
        stamp = fields['command_stamp_ns']
        if not previous <= stamp <= last_ros_ns:
            raise ValueError(f'控制日志第 {line} 行的 ROS 时间回退或超出采集窗口')
        if not first_monotonic_ns <= fields['publish_begin_monotonic_ns'] <= last_monotonic_ns:
            raise ValueError(f'控制日志第 {line} 行的单调时间超出采集窗口，请核对机器人时钟与文件归属')
        previous = stamp
        rows.append(align_command(fields, line, timeline, frames, xr_events, max_age_ns))
    if not rows:
        raise ValueError('会话没有实际发送命令可配对')
    metadata = dict(schema='aimdk_fsc.trial_alignment.v1', type='metadata',
                    trial_id=trial_id, artifacts=artifacts,
                    arm_names=arm_names,
                    clock=clocks, matching='latest_mcap_receive_at_or_before_command_ros_stamp',
                    max_age_ns=max_age_ns, streams={key: streams[key] for key in observations},
                    command_count=len(rows))
    return metadata, rows


def main():
    """解析会话、观测来源与允许年龄，生成新 JSONL；成功返回零，读取或配对失败返回一。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path, help='含 experiment.json 的已关闭示教目录')
    parser.add_argument('--output', type=Path, help='输出文件，默认写入会话内 alignment.jsonl')
    parser.add_argument('--stream', action='append', help='观测来源 ID，可重复，默认选中除 XR 外的来源')
    parser.add_argument('--max-age-ms', help='近似配对允许的样本最大年龄，默认只报告差值')
    options = parser.parse_args()
    try:
        age = None if options.max_age_ms is None else Decimal(options.max_age_ms)
        if age is not None and (not age.is_finite() or age <= 0):
            raise ValueError('--max-age-ms 须为有限正数')
        metadata, rows = export_alignment(options.session, options.stream,
                                          None if age is None else int(age * 1_000_000))
        output = options.output or options.session / 'alignment.jsonl'
        with output.open('x', encoding='utf-8') as stream:
            stream.write(json.dumps(metadata, ensure_ascii=False, allow_nan=False) + '\n')
            for row in rows:
                stream.write(json.dumps(dict(type='command', **row), ensure_ascii=False, allow_nan=False) + '\n')
        print(f'已导出 {len(rows)} 条命令配对到 {output}')
        return 0
    except (ImportError, OSError, ValueError, KeyError, TypeError, RuntimeError, InvalidOperation) as error:
        print(f'示教配对失败：{error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
