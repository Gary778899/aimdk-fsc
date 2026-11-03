"""实际反馈与PICO输入的持续只读双臂IK观测；不创建运动发布器。"""

import argparse
from dataclasses import asdict
import hashlib
import math
import sys
import time

from remote_control.cli import Recorder
from remote_control.mapping import InputUnavailable, MappingConfig, Session
from remote_control.protocol import InputConfig
from remote_control.recording import XRLog
from remote_control.sdk import SDKSource

from .dual_arm_control_core import ARM_NAMES
from .upper_body_cartesian_core import CapturedTarget
from .upper_body_diagnostics import UpperBodyLog, code_manifest, log_directory
from .upper_body_execution import UpperBodyExecutor
from .upper_body_kinematics.adapter import ArmKinematics
from .upper_body_online_core import DualArmOnlineIK
from .upper_body_teleop_replay import replay_frames


def parser():
    """定义实际输入及只读观测参数；输入时限必须由操作者显式指定。"""
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--input-config', required=True, help='安装版本的输入协议JSON')
    result.add_argument('--mapping-config', required=True, help='坐标、Grip与夹爪映射JSON')
    result.add_argument('--device-id', required=True, help='所选头显设备标识')
    result.add_argument('--sdk-library', help='显式libPXREARobotSDK.so路径')
    result.add_argument('--xr-output', required=True, help='新建原始XR及完整帧JSONL路径')
    result.add_argument('--log-dir', help='IK与反馈日志目录，默认工作区log')
    result.add_argument('--load-description', required=True, help='实际姿态、工具和负载说明')
    result.add_argument('--input-timeout', type=float, required=True,
                        help='显式接收／源时间停滞时限，单位秒；同样用于积压帧年龄检查')
    result.add_argument('--feedback-timeout', type=float, default=.5, help='沿用反馈时限0.2..1秒')
    result.add_argument('--timeout', type=float, default=5., help='初始反馈等待1..10秒')
    result.add_argument('--duration', type=float, help='观测秒数，省略时持续至Ctrl-C')
    result.add_argument('--execute', action='store_true', help='首轮只读，提供此参数立即拒绝')
    result.set_defaults(compensation=False)
    return result


def live_frames(source, recorder, port, options):
    """处理ROS反馈并逐个消费SDK事件，返回新XR帧；失效或显式时限超出时抛出异常。

    输入源时间未知时仅检查接收时效；已知源时间保持原值并单独检查推进。
    generator恢复时继续处理队列，IK期间的反馈时效由reference_provider复核。
    """
    began = last_received = source_advanced = time.monotonic()
    last_source = None
    have_frame = False
    while options.duration is None or time.monotonic() - began < options.duration:
        port.rclpy.spin_once(port.node, timeout_sec=0.)
        if not port.rclpy.ok():
            raise InputUnavailable('ROS上下文停止')
        port.feedback.checked()
        event = source.read(timeout=.001)
        frame = None
        if event is not None:
            frame = recorder.consume(event, source.backlog)
            if event.kind == 'server_disconnected' or (
                    event.kind == 'device_missing' and event.device_id == options.device_id):
                raise InputUnavailable(f'XR输入失效：{event.kind}')
        # 同步日志写盘之后重新取时，区分回调间隔与排队／处理等待。
        now = time.monotonic()
        if frame is not None:
            gap = frame.received_monotonic_s - last_received if have_frame else None
            age = now - frame.received_monotonic_s
            recorder.log.record('input_timing', input_seq=frame.seq,
                                receive_gap_s=gap, processing_age_s=age)
            if gap is not None and gap > options.input_timeout:
                raise InputUnavailable('XR回调接收间隔超过显式input-timeout')
            if age > options.input_timeout:
                raise InputUnavailable('XR积压帧年龄超过显式input-timeout')
            last_received = frame.received_monotonic_s
            have_frame = True
            if frame.source_time_ns is not None:
                if last_source is not None and frame.source_time_ns < last_source:
                    raise InputUnavailable('XR源时间回退')
                if last_source is None or frame.source_time_ns > last_source:
                    source_advanced = frame.received_monotonic_s
                last_source = frame.source_time_ns
        if now - last_received > options.input_timeout:
            raise InputUnavailable('XR接收间隔超过显式input-timeout')
        if last_source is not None and now - source_advanced > options.input_timeout:
            raise InputUnavailable('XR源时间停滞超过显式input-timeout')
        if frame is not None:
            yield frame


def main(args=None):
    """持续记录实际反馈、完整XR帧和双侧IK；返回0完成、1失败或130中断。"""
    options = parser().parse_args(args)
    if options.execute:
        print('首轮遥操入口只读，尚未开放--execute。', file=sys.stderr)
        return 1
    source = recorder = xr_log = diagnostics = port = node = None
    initialized = False
    reason, result = 'failed', 1
    try:
        for name, lower, upper in (('feedback_timeout', .2, 1.), ('timeout', 1., 10.)):
            value = getattr(options, name)
            if not math.isfinite(value) or not lower <= value <= upper:
                raise ValueError(f'{name}须为{lower}..{upper}秒')
        for name in ('input_timeout', 'duration'):
            value = getattr(options, name)
            if value is not None and (not math.isfinite(value) or value <= 0):
                raise ValueError(f'{name}须为有限正秒数')
        input_config = InputConfig.load(options.input_config)
        mapping_config = MappingConfig.load(options.mapping_config)
        source = SDKSource(options.sdk_library)
        diagnostics = UpperBodyLog(
            log_directory(options.log_dir), command='upper_body_teleop_observe',
            options=vars(options), code_sha256=code_manifest(),
            mapping_config=asdict(mapping_config), input_config=asdict(input_config),
            execute=False, compensation_enabled=False, reference_policy='actual_feedback_on_grip')
        xr_log = XRLog(
            options.xr_output, device_id=options.device_id, input_config=asdict(input_config),
            sdk_library=str(source.library_path),
            sdk_sha256=hashlib.sha256(source.library_path.read_bytes()).hexdigest(),
            snapshot_api='PXREAInit/ctypes', python=sys.version,
            ik_log=str(diagnostics.path))
        recorder = Recorder(xr_log, options.device_id, input_config)
        import rclpy
        from rclpy.signals import SignalHandlerOptions

        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = rclpy.create_node('upper_body_teleop_observe')
        if node.get_parameter('use_sim_time').value:
            raise ValueError('实际反馈观测不支持use_sim_time')
        port = UpperBodyExecutor(rclpy, node, options, diagnostics)
        arms = {side: ArmKinematics(side, tcp_mode='gripper') for side in ('left', 'right')}
        values = port.capture(refreshed_after=time.monotonic())
        captured = CapturedTarget(values['arm'].positions, values['hand'].positions)
        solver = DualArmOnlineIK(captured, arms)
        diagnostics.record('snapshot', arm_names=list(ARM_NAMES),
                            streams={name: asdict(value) for name, value in values.items()})
        diagnostics.record('online_ik_config', **solver.describe())
        session = Session(mapping_config)

        def actual_reference(frame):
            """用新鲜实际反馈计算本帧的左右TCP参考，记录其反馈序号关联。"""
            values = port.feedback.checked()
            actual = CapturedTarget(values['arm'].positions, values['hand'].positions)
            diagnostics.record('mapping_feedback', input_seq=frame.seq,
                                streams={name: value.serial for name, value in values.items()})
            if time.monotonic() - frame.received_monotonic_s > options.input_timeout:
                raise InputUnavailable('映射前XR帧年龄超过显式input-timeout')
            return solver.tcp_poses(actual), actual.hand_openings

        print(f'只读IK日志：{diagnostics.path}；XR日志：{xr_log.path}', flush=True)
        source.start()
        replay_frames(live_frames(source, recorder, port, options), session, solver,
                      diagnostics, reference_provider=actual_reference)
        reason, result = 'completed', 0
    except KeyboardInterrupt:
        reason, result = 'interrupted', 130
    except Exception as error:
        if diagnostics is not None:
            diagnostics.record('error', error_type=type(error).__name__, message=str(error))
        if xr_log is not None:
            xr_log.record('error', error_type=type(error).__name__, message=str(error))
        print(f'只读观测失败：{error}', file=sys.stderr)
    finally:
        # 先停止SDK并保存剩余原始事件，再释放ROS资源，最后关闭两份日志。
        try:
            if source is not None:
                try:
                    source.close()
                except Exception as error:
                    reason, result = 'close_failed', 1
                    if diagnostics is not None:
                        diagnostics.record('error', error_type=type(error).__name__, message=str(error))
                    if xr_log is not None:
                        xr_log.record('error', error_type=type(error).__name__, message=str(error))
                if recorder is not None and source.is_closed:
                    while True:
                        event = source.read(timeout=0.)
                        if event is None:
                            break
                        recorder.consume(event, source.backlog, drained=True)
                if source.error is not None:
                    reason, result = 'callback_failed', 1
                    if xr_log is not None:
                        xr_log.record('error', error_type='SDKCallbackError', message=source.error)
                    if diagnostics is not None:
                        diagnostics.record('error', error_type='SDKCallbackError', message=source.error)
        finally:
            try:
                if port is not None:
                    port.close()
                if node is not None:
                    node.destroy_node()
                if initialized and rclpy.ok():
                    rclpy.shutdown()
            finally:
                try:
                    if xr_log is not None:
                        xr_log.close(reason)
                finally:
                    if diagnostics is not None:
                        diagnostics.close(reason)
    return result


if __name__ == '__main__':
    sys.exit(main())
