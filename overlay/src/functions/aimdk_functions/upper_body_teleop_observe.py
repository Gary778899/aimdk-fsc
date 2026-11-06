"""PICO持续接收、20Hz最新位姿IK与Grip边沿；显式execute开启50Hz发布。"""

import argparse
from dataclasses import asdict
import hashlib
from itertools import chain
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
from .upper_body_control_core import ArmCompensationGroup
from .upper_body_diagnostics import UpperBodyLog, code_manifest, log_directory
from .upper_body_execution import UpperBodyExecutor
from .upper_body_teleop_execution import UpperBodyTeleopExecutor
from .upper_body_teleop_input import LatestInput
from .upper_body_mode import confirm_initial_mode
from .upper_body_kinematics.adapter import ArmKinematics
from .upper_body_online_core import DualArmOnlineIK
from .upper_body_teleop_replay import replay_frames
from .upper_body_teleop_tools import MappingDisplay, live_result_timing


def parser():
    """定义实际输入及只读观测参数；默认等待新输入，时限仅供显式观测实验。"""
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--input-config', required=True, help='安装版本的输入协议JSON')
    result.add_argument('--mapping-config', required=True, help='坐标、Grip与夹爪映射JSON')
    result.add_argument('--device-id', required=True, help='所选头显设备标识')
    result.add_argument('--sdk-library', help='显式libPXREARobotSDK.so路径')
    result.add_argument('--xr-output', required=True, help='原始XR及完整帧JSONL路径；重名时自动添加编号')
    result.add_argument('--log-dir', help='IK与反馈日志目录，默认工作区log')
    result.add_argument('--load-description', required=True, help='实际姿态、工具和负载说明')
    result.add_argument('--input-timeout', type=float,
                        help='输入时限秒数；只读默认不设，execute默认0.5，首次Grip前持续等待')
    result.add_argument('--feedback-timeout', type=float, default=.5, help='沿用反馈时限0.2..1秒')
    result.add_argument('--timeout', type=float, default=5., help='初始机器人反馈等待1..10秒')
    result.add_argument('--duration', type=float, help='输入会话秒数，含等待Grip，省略时持续至Ctrl-C')
    result.add_argument('--display-interval', type=float, default=1.,
                        help='映射显示间隔秒数，默认1；Grip事件立即显示，0关闭显示')
    result.add_argument('--ik-recovery-duration', type=float, default=.5,
                        help='局部IK无解后恢复过渡秒数，默认0.5')
    result.add_argument('--joint-speed-deg-s', type=float, default=30.,
                        help='执行侧理想关节目标渐进速度/度每秒，默认30，须大于0')
    result.add_argument('--execute', action='store_true', help='显式开启原生上肢持续执行，要求URS/RUNNING')
    result.add_argument('--compensation', action=argparse.BooleanOptionalAction, default=True,
                        help='执行默认开启位置偏置补偿，--no-compensation关闭')
    result.add_argument('--compensation-duration', type=float, default=5.,
                        help='每侧首次接管后补偿从零引入的秒数，默认5，Grip重按不重置')
    result.add_argument('--record-body', action='store_true',
                        help='独立记录骨盆IMU与腰原始反馈；关闭补偿时不作为运动门槛')
    result.set_defaults(arm_start_source='mc-command')
    return result


def command_reference(frame, values, solver, config, diagnostics):
    """返回TCP参考、夹爪参考及首次接管候选；values须为已校验时效的完整反馈。

    每侧首次Grip取HAL位置命令，同会话重按沿用理想目标；命令参考随IK候选提交。
    实际关节只记录偏差，夹爪沿用已接受开度，启动值取实际反馈。
    """
    command = CapturedTarget(values['arm_command'].positions, values['hand'].positions)
    rebase_sides = tuple(side for side in ('left', 'right')
                         if side not in solver.command_reference_sides
                         and getattr(frame, side).grip >= config.grip_threshold)
    reference = solver.accepted
    for side in rebase_sides:
        index = ('left', 'right').index(side)
        reference = reference.for_arm(side, command.arm_rad[index * 7:index * 7 + 7])
    diagnostics.record(
        'mapping_reference', input_seq=frame.seq, rebase_sides=list(rebase_sides),
        sources={side: 'mc-command' if side in rebase_sides else 'accepted_ideal'
                 for side in ('left', 'right')},
        reference_arm_rad=list(reference.arm_rad),
        streams={name: value.serial for name, value in values.items()},
        command_minus_actual_rad=[a-b for a, b in zip(command.arm_rad, values['arm'].positions)],
        command_stamp_s=values['arm_command'].stamp, state_stamp_s=values['arm'].stamp,
        source_skew_s=values['arm_command'].stamp-values['arm'].stamp)
    return solver.tcp_poses(reference), reference.hand_openings, command, rebase_sides


def live_frames(receiver, port, options):
    """持续处理ROS状态，立即交付Grip边沿，普通位姿按20Hz上限取最新完整帧。

    接收及录制由独立线程推进；主线程等待时不重复消费缓存输入，不补算错过的周期。
    """
    while True:
        if options.execute:
            port.check_background()
        else:
            port.rclpy.spin_once(port.node, timeout_sec=0.)
        if not port.rclpy.ok():
            raise InputUnavailable('ROS上下文停止')
        port.feedback.checked()
        selected = receiver.take(time.monotonic())
        if selected is not None:
            frame = selected.frame
            if options.execute:
                port.observe_input(frame)
            port.diagnostics.record('input_selected', input_seq=frame.seq,
                                    grip_edge=selected.edge, generation=selected.generation,
                                    coalesced_pose_frames=receiver.coalesced,
                                    selection_age_s=time.monotonic()-frame.received_monotonic_s)
            yield frame
        elif receiver.finished:
            return
        else:
            time.sleep(.001)


def main(args=None):
    """连续记录XR并选择最新位姿求解；显式执行，返回0完成、1失败或130中断。"""
    options = parser().parse_args(args)
    source = recorder = xr_log = diagnostics = port = node = client = receiver = None
    initialized = False
    reason, result = 'failed', 1
    try:
        options.compensation_requested = options.compensation
        options.compensation = options.execute and options.compensation
        if options.execute and options.input_timeout is None:
            options.input_timeout = .5
        display = MappingDisplay(options.display_interval)
        for name, lower, upper in (('feedback_timeout', .2, 1.), ('timeout', 1., 10.)):
            value = getattr(options, name)
            if not math.isfinite(value) or not lower <= value <= upper:
                raise ValueError(f'{name}须为{lower}..{upper}秒')
        for name in ('input_timeout', 'duration', 'ik_recovery_duration', 'joint_speed_deg_s'):
            value = getattr(options, name)
            if value is not None and (not math.isfinite(value) or value <= 0):
                raise ValueError(f'{name}须为有限正秒数')
        if options.compensation and (not math.isfinite(options.compensation_duration)
                                     or options.compensation_duration <= 0):
            raise ValueError('compensation-duration须为有限正秒数')
        input_config = InputConfig.load(options.input_config)
        mapping_config = MappingConfig.load(options.mapping_config)
        source = SDKSource(options.sdk_library)
        diagnostics = UpperBodyLog(
            log_directory(options.log_dir), command='upper_body_teleop_observe',
            options=vars(options), code_sha256=code_manifest(),
            mapping_config=asdict(mapping_config), input_config=asdict(input_config),
            execute=options.execute, compensation_enabled=options.compensation,
            reference_policy=('mc_command_first_grip_then_published_ideal'
                              if options.execute else 'mc_command_first_grip_then_accepted_ideal'),
            ik_failure_policy='hold_complete_target_retry_new_input',
            joint_approach_policy=('common_joint_progress_before_compensation'
                                   if options.execute else None),
            ik_recovery_duration_s=options.ik_recovery_duration)
        xr_log = XRLog(
            options.xr_output, unique_path=True,
            device_id=options.device_id, input_config=asdict(input_config),
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
        port_class = UpperBodyTeleopExecutor if options.execute else UpperBodyExecutor
        port = port_class(rclpy, node, options, diagnostics)
        if options.execute:
            from aimdk_msgs.srv import GetMcAction

            client = node.create_client(GetMcAction, '/aimdk_5Fmsgs/srv/GetMcAction')
            confirm_initial_mode(rclpy, node, options, diagnostics, client)
        arms = {side: ArmKinematics(side, tcp_mode='gripper') for side in ('left', 'right')}
        if options.compensation:
            from .upper_body_compensation import PositionCompensation

            port.compensation = ArmCompensationGroup({
                side: PositionCompensation(arm, 0., (0., 0., 0.)) for side, arm in arms.items()})
            diagnostics.record(
                'compensation_config', duration_s=options.compensation_duration,
                engagement='quintic_per_side_first_grip_no_reset_on_regrip',
                arms={side: dict(calculator.describe(), engagement='quintic_per_side_first_grip')
                      for side, calculator in port.compensation.calculators.items()})
            print(f'双臂补偿默认开启，空载；每侧首次Grip后{options.compensation_duration:g}秒从零引入。', flush=True)
        values = port.capture(refreshed_after=time.monotonic())
        captured = port.capture_target(values)
        if options.execute:
            port.start_background()
            # 先完成DDS匹配，首次Grip前定时器没有在线目标，不会发布。
            port.enable_commands(captured)

        print(f'{"执行" if options.execute else "只读IK"}日志：{diagnostics.path}；'
              f'XR日志：{xr_log.path}', flush=True)
        source.start()
        receiver = LatestInput(source, recorder, mapping_config, diagnostics, options,
                               lock=port.command_lock if options.execute else None,
                               on_error=port.stop if options.execute else None,
                               on_edge=port.hold_current if options.execute else None)
        if options.execute:
            port.input_receiver = receiver
        diagnostics.record('live_input_policy', pose_hz=20.,
                           pose_policy='latest_complete_frame', grip_policy='ordered_edges_hold_motion')
        receiver.start()
        print('等待XR完整帧；位姿IK上限20Hz，Grip边沿保序处理；Ctrl-C结束。', flush=True)
        print(f'局部IK无解时保持完整目标；新目标可解后用{options.ik_recovery_duration:g}秒恢复跟踪。',
              flush=True)
        if options.execute:
            print(f'发布侧理想关节目标按{options.joint_speed_deg_s:g}°/s渐进靠近；补偿随后计算。',
                  flush=True)
        frames = iter(live_frames(receiver, port, options))
        first_frame = initial_values = None
        if options.execute:
            print(f'URS/RUNNING已确认；等待首次Grip，接管后输入时限{options.input_timeout:g}秒。', flush=True)
            diagnostics.record('teleop_waiting_for_grip', grip_threshold=mapping_config.grip_threshold)
            for frame in frames:
                if any(getattr(frame, side).grip >= mapping_config.grip_threshold for side in ('left', 'right')):
                    first_frame = frame
                    break
            if first_frame is None:
                diagnostics.record('teleop_no_takeover', published_count=port.sequence)
                print('会话结束，未按Grip接管，未下发指令。', flush=True)
            else:
                # 首次控制完整14轴均从此刻新鲜HAL命令建立，包含尚未按Grip的一侧。
                initial_values = port.feedback.checked()
                captured = port.capture_target(initial_values)
                port.captured = captured
                values = initial_values
                frames = chain((first_frame,), frames)
        if not options.execute or first_frame is not None:
            solver = DualArmOnlineIK(captured, arms)
            diagnostics.record('snapshot', arm_names=list(ARM_NAMES),
                                streams={name: asdict(value) for name, value in values.items()})
            diagnostics.record('online_ik_config', **solver.describe())
            session = Session(mapping_config)

            def hold_execution(frame, reason):
                """在共同锁内用当前发布理想目标同步IK种子及映射保持；记录取消的远端终点。"""
                previous = solver.accepted
                target = port.hold_current()
                if target is None:
                    return
                solver.hold_published(target)
                session.hold(solver.tcp_poses(), target.hand_openings)
                diagnostics.record('execution_target_held', input_seq=frame.seq, reason=reason,
                                   previous_ik_goal_arm_rad=list(previous.arm_rad),
                                   held_ideal_arm_rad=list(target.arm_rad),
                                   hand_openings=list(target.hand_openings),
                                   last_accepted_input_seq=solver.last_input_seq)

            def hold_failed_target(frame):
                """可恢复空解已拒绝后取消渐进队列，以当前发布理想目标开始后续恢复。"""
                hold_execution(frame, 'local_ik_unavailable')

            def live_reference(frame):
                """校验输入与反馈时效，生成HAL首次接管或理想目标重接管参考。"""
                if options.execute:
                    port.check_background()
                    with receiver.lock:
                        if receiver.selected.edge and solver.command_reference_sides:
                            hold_execution(frame, 'grip_edge')
                values = (initial_values if first_frame is not None and frame.seq == first_frame.seq
                          else port.feedback.checked())
                if (options.input_timeout is not None
                        and time.monotonic() - frame.received_monotonic_s > options.input_timeout):
                    raise InputUnavailable('映射前XR帧年龄超过显式input-timeout')
                return command_reference(frame, values, solver, mapping_config, diagnostics)

            def before_live_commit():
                """在共同提交锁内复核输入及反馈，拒绝跨Grip边沿的旧普通候选。"""
                if options.execute:
                    port.check_background()
                    port.check_input_timeout(time.monotonic())
                receiver.check()
                port.feedback.checked()
                if (options.input_timeout is not None and time.monotonic()
                        - receiver.selected.frame.received_monotonic_s > options.input_timeout):
                    raise InputUnavailable('求解完成时XR帧年龄超过input-timeout')
                return receiver.can_commit()

            def hold_edge_motion(frame):
                """边沿只建立参考或保持，普通帧才推进双侧运动目标。"""
                return receiver.selected.edge

            if options.execute:
                with receiver.lock:
                    receiver.activate()
                    receiver.check()
                    port.begin_online(solver.store, first_frame)
            replay_frames(frames, session, solver,
                          diagnostics, reference_provider=live_reference, display=display,
                          timing_provider=live_result_timing,
                          before_commit=before_live_commit,
                          on_error=port.stop if options.execute else None,
                          commit_lock=receiver.lock, hold_motion=hold_edge_motion,
                          ik_recovery_duration=options.ik_recovery_duration,
                          on_commit=port.accept_online if options.execute else None,
                          on_local_failure=hold_failed_target if options.execute else None)
        if options.execute:
            port.stop()
            port.check_background()
        receiver.check()
        reason, result = 'completed', 0
    except KeyboardInterrupt:
        if port is not None:
            port.stop()
        reason, result = 'interrupted', 130
    except Exception as error:
        if port is not None:
            port.stop()
        if receiver is not None:
            receiver.close()
        if diagnostics is not None:
            diagnostics.record('error', error_type=type(error).__name__, message=str(error))
        if xr_log is not None:
            xr_log.record('error', error_type=type(error).__name__, message=str(error))
        print(f'遥操{"执行" if options.execute else "观测"}失败：{error}', file=sys.stderr)
    finally:
        # 先停发，再停止SDK并保存剩余原始事件，最后释放ROS和日志。
        if port is not None:
            port.stop()
        if receiver is not None:
            receiver.close()
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
                if client is not None:
                    node.destroy_client(client)
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
