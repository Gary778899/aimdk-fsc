"""只读XR回调及完整输入帧的JSONL读写与接收节奏统计。"""

import json
import math
from pathlib import Path
import time

from .frames import XRFrame


SCHEMA = 'remote_control.xr.v1'


class XRLog:
    """独占创建JSONL文件，逐事件flush保留原始回调和归一化帧。"""

    def __init__(self, path, *, unique_path=False, **metadata):
        """创建path并记录元数据；unique_path启用重名编号，其他写入异常直接传播。

        默认拒绝已有文件；启用后依次尝试文件名_1、文件名_2，保留原扩展名。
        实际创建路径保存在self.path，所有候选均以独占模式创建。
        """
        self.path = Path(path).expanduser().resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        requested = self.path
        index = 0
        while True:
            try:
                self.stream = self.path.open('x', encoding='utf-8')
                break
            except FileExistsError:
                if not unique_path:
                    raise
                index += 1
                self.path = requested.with_name(f'{requested.stem}_{index}{requested.suffix}')
        try:
            self.record('metadata', **metadata)
        except Exception:
            self.stream.close()
            raise

    def record(self, event, **fields):
        """写入JSON兼容字段；非有限值报错，不替换原始信息。"""
        value = dict(schema=SCHEMA, event=event, wall_time_ns=time.time_ns(),
                     monotonic_s=time.monotonic(), fields=fields)
        self.stream.write(json.dumps(value, ensure_ascii=False, allow_nan=False) + '\n')
        self.stream.flush()

    def close(self, reason='closed'):
        """记录结束原因并关闭文件，可重复调用。"""
        if not self.stream.closed:
            try:
                self.record('exit', reason=reason)
            finally:
                self.stream.close()


def read_records(path):
    """按文件顺序返回严格schema的JSONL记录；报告包含行号的格式错误。"""
    with Path(path).expanduser().open(encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, start=1):
            try:
                value = json.loads(line)
                if (not isinstance(value, dict) or value.get('schema') != SCHEMA
                        or not isinstance(value.get('event'), str)
                        or not isinstance(value.get('fields'), dict)):
                    raise ValueError('缺少schema/event/fields或schema不匹配')
                yield value
            except (ValueError, TypeError) as error:
                raise ValueError(f'{path}:{line_number}: {error}') from error


def read_frames(path):
    """读回已归一化XR帧；异常或失联事件明确终止，不跨断链回放旧参考。

    要继续失联后的实验，重新录制或由会话调用方显式处理原始记录。
    输入序号和接收时间必须严格推进，源时间重复保留供统计、回退时报错。
    """
    previous = None
    last_known_source = None
    for line_number, record in enumerate(read_records(path), 1):
        raw_frame = record['fields'].get('frame')
        sequence = raw_frame.get('seq') if isinstance(raw_frame, dict) else None
        try:
            if record['event'] in ('error', 'session_invalidated'):
                raise ValueError(f"XR记录包含{record['event']}：{record['fields']}")
            if record['event'] != 'xr_frame':
                continue
            frame = XRFrame.from_dict(raw_frame)
            if previous is not None:
                if frame.seq <= previous.seq or frame.received_monotonic_s <= previous.received_monotonic_s:
                    raise ValueError('XR输入序号或接收时间未推进')
            if frame.source_time_ns is not None:
                if last_known_source is not None and frame.source_time_ns < last_known_source:
                    raise ValueError('XR源时间回退')
                last_known_source = frame.source_time_ns
        except (ValueError, TypeError, KeyError) as error:
            raise ValueError(f'{path}:{line_number}: XR帧seq={sequence}: {error}') from error
        previous = frame
        yield frame


def distribution(values):
    """返回有限样本的P50/P95/P99/最大值；无样本时各项为None。"""
    ordered = sorted(values)
    if not all(math.isfinite(item) for item in ordered):
        raise ValueError('统计样本须有限')
    result = {}
    for label, fraction in (('p50', .5), ('p95', .95), ('p99', .99)):
        if not ordered:
            result[label] = None
            continue
        position = (len(ordered) - 1) * fraction
        lower, upper = math.floor(position), math.ceil(position)
        result[label] = ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)
    result['max'] = max(ordered) if ordered else None
    return result


def summarize(path):
    """统计回调／输入帧数、接收间隔、源时间及积压；不推算跨设备传输延迟。"""
    callbacks, frames, event_counts, backlog = [], [], {}, []
    errors, invalidations, exits = [], [], []
    for record in read_records(path):
        event, fields = record['event'], record['fields']
        event_counts[event] = event_counts.get(event, 0) + 1
        if event == 'error':
            errors.append(fields)
        elif event == 'session_invalidated':
            invalidations.append(fields)
        elif event == 'exit':
            exits.append(fields)
        if event == 'sdk_event':
            callbacks.append(fields['raw'])
            backlog.append(fields['backlog'])
        elif event == 'xr_frame':
            frames.append(XRFrame.from_dict(fields['frame']))
    intervals = [b.received_monotonic_s - a.received_monotonic_s for a, b in zip(frames, frames[1:])]
    duration = frames[-1].received_monotonic_s - frames[0].received_monotonic_s if len(frames) > 1 else 0.
    # 跨过缺失值比较已知源时间，缺失计数仍以原始帧为准。
    known_sources = [frame.source_time_ns for frame in frames if frame.source_time_ns is not None]
    pairs = list(zip(known_sources, known_sources[1:]))
    seq_pairs = [(a['callback_seq'], b['callback_seq']) for a, b in zip(callbacks, callbacks[1:])]
    callback_errors = [event for event in callbacks if event.get('error')]
    reversed_source = sum(b < a for a, b in pairs)
    input_sequence_errors = sum(b.seq <= a.seq for a, b in zip(frames, frames[1:]))
    callback_sequence_errors = sum(b <= a for a, b in seq_pairs)
    missing_callbacks = sum(max(0, b - a - 1) for a, b in seq_pairs)
    return dict(event_counts=event_counts, errors=errors, callback_errors=callback_errors,
                session_invalidations=invalidations,
                exit_reasons=exits,
                recording_complete=bool(frames and exits and exits[-1].get('reason') == 'completed'
                                        and not errors and not invalidations and not callback_errors
                                        and reversed_source == 0 and input_sequence_errors == 0
                                        and callback_sequence_errors == 0 and missing_callbacks == 0
                                        and all(value > 0 for value in intervals)),
                callback_count=len(callbacks), input_frame_count=len(frames),
                effective_input_hz=(len(frames) - 1) / duration if duration > 0 else None,
                receive_interval_s=distribution(intervals), backlog=distribution(backlog),
                repeated_source_time=sum(a == b for a, b in pairs),
                reversed_source_time=reversed_source,
                missing_source_time=sum(frame.source_time_ns is None for frame in frames),
                nonadvancing_input_sequence=input_sequence_errors,
                nonadvancing_callback_sequence=callback_sequence_errors,
                missing_callback_sequences=missing_callbacks)
