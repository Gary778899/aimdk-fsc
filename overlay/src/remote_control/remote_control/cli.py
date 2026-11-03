"""XR只读录制与JSONL统计入口，不导入ROS或发布机器人命令。"""

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
import time

from .protocol import InputConfig, parse_tracking
from .recording import XRLog, summarize
from .sdk import SDKSource


class Recorder:
    """将固定设备的原始回调和完整帧写入同一日志，其他设备只保留原始事件。"""

    def __init__(self, log, device_id, config):
        """保存日志、显式设备标识及输入约定；不自动选择首台头显。"""
        if not device_id:
            raise ValueError('device_id须为非空字符串')
        self.log, self.device_id, self.config = log, device_id, config
        self.frame_count = 0

    def consume(self, event, backlog, drained=False):
        """先记录原始事件，再归一化所选设备帧；错误保留日志并向调用方传播。

        返回已归一化XRFrame，无输入时返回None；drained表示SDK终止后的退出排空，
        只保存原始事件，避免把主动关闭当作输入失联。
        """
        self.log.record('sdk_event', raw=event.to_dict(), backlog=backlog, drained=drained)
        if drained:
            return
        if event.error:
            raise ValueError(f'SDK回调失败：{event.error}')
        if event.kind == 'server_disconnected' or (
                event.kind == 'device_missing' and event.device_id == self.device_id):
            self.log.record('session_invalidated', callback_seq=event.callback_seq,
                            received_monotonic_s=event.received_monotonic_s,
                            reason=event.kind, device_id=self.device_id)
            return
        if event.device_id != self.device_id:
            return
        frame = parse_tracking(event, self.frame_count + 1, self.config)
        if frame is not None:
            self.log.record('xr_frame', frame=frame.to_dict(), callback_seq=event.callback_seq)
            self.frame_count += 1
        return frame


def record_parser():
    """定义显式输入协议、固定设备与输出路径，不提供任何运动执行选项。"""
    parser = argparse.ArgumentParser(description='只读录制PICO完整XR回调及归一化帧')
    parser.add_argument('--input-config', required=True, help='输入协议JSON配置')
    parser.add_argument('--device-id', required=True, help='所选头显设备标识，须与SDK回调一致')
    parser.add_argument('--output', required=True, help='新建JSONL文件，已有文件拒绝覆盖')
    parser.add_argument('--sdk-library', help='显式libPXREARobotSDK.so；省略时定位xrobotoolkit-sdk随包库')
    parser.add_argument('--duration', type=float, help='录制秒数；省略时持续至Ctrl-C')
    return parser


def record_main(args=None):
    """录制固定设备并退出前关闭SDK、排空回调；返回0完成、1失败、130中断。"""
    options = record_parser().parse_args(args)
    return run_recording(options)


def run_recording(options, observer=None):
    """执行共享录制生命周期；observer可显示启动、事件及周期状态。

    options包含record_parser定义的字段；observer为空时保持静默录制行为。
    返回0完成、1失败、130中断；关闭及退出排空沿用同一实现。
    """
    source = log = recorder = None
    reason, result = 'failed', 1
    try:
        if options.duration is not None and (not math.isfinite(options.duration) or options.duration <= 0):
            raise ValueError('--duration须为有限正秒数')
        config = InputConfig.load(options.input_config)
        source = SDKSource(options.sdk_library)
        log = XRLog(options.output, device_id=options.device_id, input_config=asdict(config),
                    sdk_library=str(source.library_path),
                    sdk_sha256=hashlib.sha256(source.library_path.read_bytes()).hexdigest(),
                    input_config_sha256=hashlib.sha256(Path(options.input_config).read_bytes()).hexdigest(),
                    snapshot_api='PXREAInit/ctypes', python=sys.version)
        recorder = Recorder(log, options.device_id, config)
        if observer is not None:
            observer.started(log.path)
        source.start()
        began = time.monotonic()
        while options.duration is None or time.monotonic() - began < options.duration:
            wait = .1 if options.duration is None else min(.1, max(0., options.duration - (time.monotonic() - began)))
            event = source.read(timeout=wait)
            if event is not None:
                frame = recorder.consume(event, source.backlog)
                if observer is not None:
                    observer.consume(event, frame)
            if observer is not None:
                observer.report(source.backlog)
        if not recorder.frame_count:
            raise ValueError('录制窗口内未收到所选设备的完整Controller帧')
        reason, result = 'completed', 0
    except KeyboardInterrupt:
        reason, result = 'interrupted', 130
    except Exception as error:
        if log is not None:
            log.record('error', error_type=type(error).__name__, message=str(error))
        print(f'XR录制失败：{error}', file=sys.stderr)
    finally:
        try:
            if source is not None:
                try:
                    source.close()
                except Exception as error:
                    reason, result = 'close_failed', 1
                    if log is not None:
                        log.record('error', error_type=type(error).__name__, message=str(error))
                    print(f'XR关闭失败：{error}', file=sys.stderr)
                if recorder is not None and source.is_closed:
                    while True:
                        event = source.read(timeout=0.)
                        if event is None:
                            break
                        recorder.consume(event, source.backlog, drained=True)
                    if source.error is not None:
                        reason, result = 'callback_failed', 1
                        log.record('error', error_type='SDKCallbackError', message=source.error)
        finally:
            if log is not None:
                log.close(reason)
    if log is not None:
        print(f'日志：{log.path}；完整输入帧：{recorder.frame_count if recorder else 0}')
    return result


def stats_main(args=None):
    """读回XR日志并输出接收节奏及异常计数，格式错误返回1。"""
    parser = argparse.ArgumentParser(description='统计XR JSONL的输入节奏、序号和积压')
    parser.add_argument('input', help='xr_record生成的JSONL')
    options = parser.parse_args(args)
    try:
        print(json.dumps(summarize(options.input), ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    except Exception as error:
        print(f'XR统计失败：{error}', file=sys.stderr)
        return 1
