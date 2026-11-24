"""XR只读录制、可选原始回调发布和JSONL统计入口。"""

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
import time

from .protocol import InputConfig
from .recording import Recorder, XRLog, summarize
from .sdk import SDKSource


def record_parser():
    """返回只读XR录制的参数解析器，包含可选原始话题发布和服务启动说明。"""
    parser = argparse.ArgumentParser(
        description='只读录制PICO完整XR回调及归一化帧',
        epilog="录制前启动XR Service。在/opt/apps/roboticsservice目录运行 "
               "bash ./runService.sh，再用 ss -ltn 'sport = :60061' 检查监听。")
    parser.add_argument('--input-config', required=True, help='输入协议JSON配置')
    parser.add_argument('--device-id', required=True, help='所选头显设备标识，须与SDK回调一致')
    parser.add_argument('--output', required=True, help='新建JSONL文件，已有文件拒绝覆盖')
    parser.add_argument('--sdk-library', help='显式libPXREARobotSDK.so；省略时定位xrobotoolkit-sdk随包库')
    parser.add_argument('--duration', type=float, help='录制秒数；省略时持续至Ctrl-C')
    parser.add_argument('--publish-xr-raw', action='store_true',
                        help='同时向/remote_control/xr/raw发布原始SDK回调')
    return parser


def record_main(args=None):
    """解析args并录制固定设备；返回0完成、1失败或130中断，参数错误时由解析器退出。"""
    options = record_parser().parse_args(args)
    return run_recording(options)


def run_recording(options, observer=None):
    """执行共享录制生命周期；observer可显示启动、事件及周期状态。

    options包含录制参数，可选publish_xr_raw开启原始话题发布。
    退出时先关闭SDK并排空回调，再释放ROS资源并收尾日志。
    返回0完成、1录制或发布失败、130中断。
    """
    source = log = recorder = raw_publisher = node = None
    ros_initialized = False
    publish_xr_raw = getattr(options, 'publish_xr_raw', False)
    reason, result = 'failed', 1
    try:
        if options.duration is not None and (not math.isfinite(options.duration) or options.duration <= 0):
            raise ValueError('--duration须为有限正秒数')
        config = InputConfig.load(options.input_config)
        source = SDKSource(options.sdk_library)
        log = XRLog(options.output, unique_path=getattr(options, 'unique_path', False),
                    device_id=options.device_id, input_config=asdict(config), publish_xr_raw=publish_xr_raw,
                    sdk_library=str(source.library_path),
                    sdk_sha256=hashlib.sha256(source.library_path.read_bytes()).hexdigest(),
                    input_config_sha256=hashlib.sha256(Path(options.input_config).read_bytes()).hexdigest(),
                    snapshot_api='PXREAInit/ctypes', python=sys.version)
        if publish_xr_raw:
            import rclpy
            from rclpy.signals import SignalHandlerOptions

            from .raw_publisher import XRRawPublisher

            # 由Python处理Ctrl-C，保留ROS上下文直到SDK关闭并完成排空。
            rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
            ros_initialized = True
            node = rclpy.create_node('xr_record')
            raw_publisher = XRRawPublisher(node)
        recorder = Recorder(log, options.device_id, config, on_raw=raw_publisher)
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
            raise ValueError('录制窗口内未收到所选设备的完整Controller帧。'
                             '请检查XR Service、设备标识和PICO的Controller及Send设置。')
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
                    recorder.drain(source)
                    if source.error is not None:
                        reason, result = 'callback_failed', 1
                        log.record('error', error_type='SDKCallbackError', message=source.error)
            if raw_publisher is not None and raw_publisher.first_error is not None:
                if result != 1:
                    reason, result = 'xr_raw_publish_failed', 1
                message = raw_publisher.first_error['message']
                print(f'XR原始话题发布失败：{message}', file=sys.stderr)
        finally:
            try:
                if ros_initialized:
                    try:
                        if node is not None:
                            node.destroy_node()
                    finally:
                        if rclpy.ok():
                            rclpy.shutdown()
            finally:
                if log is not None:
                    try:
                        if raw_publisher is not None:
                            raw_publisher.record_summary(log)
                    finally:
                        log.close(reason)
    if log is not None:
        print(f'日志：{log.path}；完整输入帧：{recorder.frame_count if recorder else 0}')
    return result


def stats_main(args=None):
    """解析args指定的XR日志并输出接收节奏及异常计数；成功返回0，读回失败返回1。"""
    parser = argparse.ArgumentParser(description='统计XR JSONL的输入节奏、序号和积压')
    parser.add_argument('input', help='xr_record生成的JSONL')
    options = parser.parse_args(args)
    try:
        print(json.dumps(summarize(options.input), ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    except Exception as error:
        print(f'XR统计失败：{error}', file=sys.stderr)
        return 1
