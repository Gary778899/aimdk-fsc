"""从XR记录和机器人命令／反馈快照回放双臂逐帧IK；只读、无ROS导入。"""

import argparse
from contextlib import nullcontext
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

from remote_control.frames import Pose, TCPGoal
from remote_control.mapping import MappingConfig, Session
from remote_control.recording import read_frames

from .dual_arm_control_core import ARM_NAMES, interpolate
from .upper_body_cartesian_core import CapturedTarget
from .upper_body_diagnostics import UpperBodyLog, code_manifest, log_directory
from .upper_body_online_core import DualArmOnlineIK, LocalIKUnavailable, OnlineIKFailure
from .upper_body_kinematics.vendor.x2_arm_model import axis_angle_to_matrix
from .upper_body_teleop_tools import MappingDisplay, ReplayPacer, rotation_vector_deg, tracking_state


def parser():
    """定义只读回放参数；真机执行参数保留为显式拒绝入口。"""
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--xr-log', required=True, help='xr_record生成的JSONL路径')
    result.add_argument('--mapping-config', required=True, help='显式坐标、Grip及夹爪映射JSON')
    result.add_argument('--snapshot-log', required=True, help='upper_body_kinematics_check的snapshot日志')
    result.add_argument('--snapshot-index', type=int, default=0, help='从零开始的snapshot事件索引')
    result.add_argument('--arm-start-source', choices=('mc-command', 'state'), default='mc-command',
                        help='默认取快照HAL位置命令；state仅用于历史实际反馈对照')
    result.add_argument('--log-dir', help='输出日志父目录，默认工作区log')
    result.add_argument('--load-description', required=True, help='采样时工具安装及实际负载说明')
    result.add_argument('--replay-timing', choices=('immediate', 'recorded'), default='immediate',
                        help='immediate尽快逐帧计算；recorded按原接收间隔FIFO消费')
    result.add_argument('--display-interval', type=float, default=1.,
                        help='映射显示间隔秒数，默认1；Grip事件立即显示，0关闭显示')
    result.add_argument('--ik-recovery-duration', type=float, default=.5,
                        help='局部IK无解后恢复过渡秒数，默认0.5；按原始帧接收时间推进')
    result.add_argument('--execute', action='store_true', help='首轮只读，提供此参数立即拒绝')
    return result


def read_snapshot(path, index, arm_start_source='mc-command'):
    """从已有日志选择第index个snapshot，按名称重排14关节；非法数据抛出ValueError。

    arm_start_source选择HAL位置命令或实际反馈，不自动回退；夹爪取实际开度。
    完整原始snapshot随输出元数据保存；HAL命令沿用按名称解码后的14轴槽位。
    """
    if index < 0:
        raise ValueError('snapshot-index须为非负整数')
    count = 0
    with Path(path).expanduser().open(encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except (TypeError, ValueError) as error:
                raise ValueError(f'快照日志第{line_number}行JSON无效') from error
            if row.get('event') != 'snapshot':
                continue
            if count != index:
                count += 1
                continue
            fields = row['fields']
            names = fields['arm_names']
            if arm_start_source not in ('mc-command', 'state'):
                raise ValueError('无效的手臂起点来源')
            stream_name = 'arm_command' if arm_start_source == 'mc-command' else 'arm'
            if stream_name not in fields['streams']:
                raise ValueError(f'快照缺少{stream_name}；请采集含HAL命令的新快照，或显式选择state对照')
            positions = fields['streams'][stream_name]['positions']
            if (len(names) != 14 or len(set(names)) != 14 or set(names) != set(ARM_NAMES)
                    or len(positions) != 14):
                raise ValueError('快照须包含完整且唯一的14个关节名称与位置')
            by_name = dict(zip(names, positions))
            target = CapturedTarget(tuple(by_name[name] for name in ARM_NAMES),
                                    fields['streams']['hand']['positions'])
            return target, fields
    raise ValueError(f'日志不存在索引为{index}的snapshot事件')


def recovery_target(target, poses, openings, blend):
    """从保持位姿向最新目标插值，返回带原始输入时间的过渡TargetFrame。

    blend为五次曲线进度0..1；平移及夹爪线性混合，旋转沿最短轴角混合。
    未使能侧保持原目标。返回位姿仍须由IK求解后才能提交。
    """
    goals = {}
    hands = list(openings)
    for index, side in enumerate(('left', 'right')):
        goal = getattr(target, side)
        origin = poses[side]
        if not goal.enabled:
            goals[side] = TCPGoal(origin, False)
            continue
        position = ((1. - blend) * np.asarray(origin.position_m)
                    + blend * np.asarray(goal.pose.position_m))
        rotation = np.asarray(origin.rotation_matrix)
        delta = np.radians(rotation_vector_deg(np.asarray(goal.pose.rotation_matrix) @ rotation.T))
        angle = float(np.linalg.norm(delta))
        if angle > 0.:
            rotation = axis_angle_to_matrix(delta / angle, blend * angle) @ rotation
        goals[side] = TCPGoal(Pose(position, rotation), True)
        hands[index] += blend * (target.hand_openings[index] - hands[index])
    return replace(target, left=goals['left'], right=goals['right'], hand_openings=tuple(hands))


def replay_frames(frames, session, solver, diagnostics, reference_provider=None,
                  display=None, timing_provider=None, before_commit=None, on_error=None,
                  commit_lock=None, hold_motion=None, ik_recovery_duration=None):
    """串行消费XR帧并联合提交映射与IK，返回接受数量。

    离线首帧使用所选快照建立TCP参考，后续Grip重按使用最后已接受理想目标；
    静态快照不能模拟现场首次Grip时重新采样HAL命令。reference_provider(frame)
    返回(robot_tcp_poses, hand_openings, command_reference, rebase_sides)，
    供现场入口在首次Grip接管时事务式重建HAL命令参考。display接收映射候选与结果；
    timing_provider(frame)返回与运行环境对应的输入至结果耗时；before_commit检查执行侧异常，
    before_commit显式返回False时丢弃被新Grip边沿取代的普通候选并继续；
    hold_motion(frame)为True时只处理边沿参考和保持。on_error在失败诊断写盘前停止发布；
    commit_lock串行化联合提交、接收边沿与执行侧停发。
    ik_recovery_duration为正秒数时，局部空解保持完整目标并继续；新目标可解后按
    原始接收时间做五次过渡。None沿用首错终止，其他异常始终终止。
    """
    if ik_recovery_duration is not None and (not np.isfinite(ik_recovery_duration)
                                             or ik_recovery_duration <= 0):
        raise ValueError('ik-recovery-duration须为有限正秒数')
    accepted = 0
    rejected = 0
    holding = False
    recovery_started = recovery_poses = recovery_openings = None
    previous_accepted = None
    for frame in frames:
        began = time.monotonic()
        diagnostics.record('xr_input', frame=frame.to_dict())
        mapping_proposal = None
        mapping_committed = False
        committed = False
        before_ik, before_mapping = tracking_state(solver), session.describe()
        try:
            poses, openings, reference, rebase_sides = (
                (solver.tcp_poses(), solver.accepted.hand_openings, None, ())
                if reference_provider is None else reference_provider(frame))
            edge = hold_motion(frame) if hold_motion else False
            mapping_proposal = session.propose(frame, poses, openings, hold_motion=edge)
            diagnostics.record('mapping_candidate', input_seq=frame.seq,
                                arms=mapping_proposal.report)
            diagnostics.record('tcp_target', target=mapping_proposal.target.to_dict())
            candidate = solver.propose(mapping_proposal.target, reference=reference,
                                       rebase_sides=rebase_sides)
            next_recovery = None
            accepted_target = None
            if holding and not edge:
                started = (frame.received_monotonic_s if recovery_started is None
                           else recovery_started)
                anchors = solver.tcp_poses() if recovery_started is None else recovery_poses
                hands = solver.accepted.hand_openings if recovery_started is None else recovery_openings
                blend = interpolate((0.,), (1.,), frame.received_monotonic_s-started,
                                    ik_recovery_duration)[0]
                target = recovery_target(mapping_proposal.target, anchors, hands, blend)
                # 先确认原始目标可解，再求过渡位姿；任一空解均保持上一完整目标。
                endpoint_seconds = candidate.report['calculation_seconds']
                if blend < 1.:
                    candidate = solver.propose(target, reference=reference, rebase_sides=rebase_sides)
                    candidate.report['calculation_seconds'] += endpoint_seconds
                actual_poses = solver.tcp_poses(candidate.target)
                accepted_target = replace(target, left=TCPGoal(actual_poses['left'], target.left.enabled),
                                          right=TCPGoal(actual_poses['right'], target.right.enabled))
                next_recovery = (started, anchors, hands, blend)
                candidate.report['recovery'] = dict(
                    phase='recovering' if blend < 1. else 'tracking', blend=blend,
                    started_received_monotonic_s=started, duration_s=ik_recovery_duration,
                    endpoint_check_seconds=endpoint_seconds)
            with commit_lock if commit_lock is not None else nullcontext():
                if before_commit is not None and before_commit() is False:
                    session.reject(mapping_proposal)
                    diagnostics.record('ik_superseded', input_seq=frame.seq,
                                       reason='求解期间收到新Grip边沿，保持已接受目标')
                    if timing_provider is not None:
                        diagnostics.record('frame_result_timing', input_seq=frame.seq,
                                           accepted=False, **timing_provider(frame))
                    continue
                session.validate_commit(mapping_proposal)
                solver.validate_commit(candidate)
                # 两方预检后联合提交；执行时与发布／停发共用同一锁。
                session.commit(mapping_proposal, accepted_target=accepted_target)
                mapping_committed = True
                value = solver.commit(candidate)
                committed = True
                if holding:
                    if edge:
                        recovery_started = None
                    else:
                        recovery_started, recovery_poses, recovery_openings, blend = next_recovery
                        holding = blend < 1.
            accepted += 1
            diagnostics.record('ik_accepted', input_seq=frame.seq,
                                accepted_monotonic_s=value.accepted_monotonic_s,
                                result=candidate.report, calculation_seconds=time.monotonic() - began)
            previous_accepted = dict(input_frame=frame.to_dict(),
                                     tcp_target=(accepted_target or mapping_proposal.target).to_dict(),
                                     state=tracking_state(solver))
            if next_recovery is not None:
                diagnostics.record('ik_recovery', input_seq=frame.seq, **candidate.report['recovery'])
        except Exception as error:
            recoverable = (not committed and isinstance(error, LocalIKUnavailable)
                           and ik_recovery_duration is not None)
            if recoverable and before_commit is not None:
                try:
                    with commit_lock if commit_lock is not None else nullcontext():
                        current = before_commit()
                except Exception:
                    if on_error is not None:
                        on_error()
                    raise
                if current is False:
                    session.reject(mapping_proposal)
                    diagnostics.record('ik_superseded', input_seq=frame.seq,
                                       reason='求解期间收到新Grip边沿，保持已接受目标')
                    if timing_provider is not None:
                        diagnostics.record('frame_result_timing', input_seq=frame.seq,
                                           accepted=False, **timing_provider(frame))
                    continue
            if not recoverable and on_error is not None:
                on_error()
            if committed:
                raise
            if mapping_proposal is not None and not mapping_committed:
                session.reject(mapping_proposal)
            diagnostics.record('ik_rejected', input_seq=frame.seq,
                                recoverable=recoverable,
                                reason=str(error), error_type=type(error).__name__,
                                result=error.report if isinstance(error, OnlineIKFailure) else None,
                                last_accepted_rad=list(solver.accepted.arm_rad),
                                input_frame=frame.to_dict(), previous_accepted=previous_accepted,
                                mapping_candidate=(mapping_proposal.report
                                                   if mapping_proposal is not None else None),
                                ik_before=before_ik, ik_after=tracking_state(solver),
                                mapping_before=before_mapping, mapping_after=session.describe(),
                                calculation_seconds=time.monotonic() - began)
            if timing_provider is not None:
                diagnostics.record('frame_result_timing', input_seq=frame.seq,
                                    accepted=False, **timing_provider(frame))
            if display is not None:
                display.show(frame, mapping_proposal.report if mapping_proposal else None,
                             False, (f'{error}；保持双臂及夹爪，等待可解的新目标'
                                     if recoverable else str(error)),
                             transient=recoverable and holding and recovery_started is None)
            if recoverable:
                rejected += 1
                if not holding or recovery_started is not None:
                    diagnostics.record('ik_hold', input_seq=frame.seq, reason=str(error),
                                       last_accepted_input_seq=solver.last_input_seq)
                holding, recovery_started = True, None
                continue
            raise
        if timing_provider is not None:
            diagnostics.record('frame_result_timing', input_seq=frame.seq,
                                accepted=True, **timing_provider(frame))
        if display is not None:
            recovery = candidate.report.get('recovery')
            display.show(frame, mapping_proposal.report, True,
                         (f'IK恢复过渡 {recovery["blend"]:.0%}' if recovery else None))
    if accepted == 0:
        raise ValueError('XR日志没有可回放的完整输入帧')
    diagnostics.record('replay_summary', accepted_frames=accepted, rejected_frames=rejected,
                        ik_recovery_pending=holding,
                        last_input_seq=solver.last_input_seq,
                        arm_rad=list(solver.accepted.arm_rad),
                        hand_openings=list(solver.accepted.hand_openings))
    return accepted


def main(args=None):
    """回放完整XR帧并保存日志，返回0成功、1失败或130中断；不创建ROS节点。"""
    options = parser().parse_args(args)
    if options.execute:
        print('首轮遥操入口只读，尚未开放--execute。', file=sys.stderr)
        return 1
    diagnostics = None
    reason = 'failed'
    try:
        display = MappingDisplay(options.display_interval)
        captured, snapshot = read_snapshot(options.snapshot_log, options.snapshot_index,
                                           options.arm_start_source)
        config_path = Path(options.mapping_config).expanduser().resolve()
        config = MappingConfig.load(config_path)
        diagnostics = UpperBodyLog(
            log_directory(options.log_dir), command='upper_body_teleop_replay', options=vars(options),
            snapshot=snapshot, mapping_config=json.loads(config_path.read_text(encoding='utf-8')),
            mapping_config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
            code_sha256=code_manifest(), compensation_enabled=False, execute=False,
            reference_policy=f'{options.arm_start_source}_snapshot_then_last_accepted_ideal_target',
            ik_failure_policy='hold_complete_target_retry_new_input',
            ik_recovery_duration_s=options.ik_recovery_duration)
        print(f'只读回放日志：{diagnostics.path}', flush=True)
        solver = DualArmOnlineIK(captured)
        diagnostics.record('online_ik_config', **solver.describe())
        session = Session(config)
        # 预载完整帧以计算原节奏下已到期的等待帧数；校验失败不进入IK。
        try:
            frames = list(read_frames(options.xr_log))
        except Exception as error:
            diagnostics.record('replay_input_error', path=options.xr_log,
                                error_type=type(error).__name__, reason=str(error))
            raise
        diagnostics.record('replay_schedule', mode=options.replay_timing, frame_count=len(frames),
                            frame_policy='fifo_no_drop', input_preloaded=True)
        pacer = ReplayPacer(frames, diagnostics, options.replay_timing)
        count = replay_frames(pacer, session, solver, diagnostics, display=display,
                              timing_provider=pacer.result_timing,
                              ik_recovery_duration=options.ik_recovery_duration)
        print(f'回放完成：接受{count}帧；未发布运动命令。', flush=True)
        reason = 'completed'
        return 0
    except KeyboardInterrupt:
        reason = 'interrupted'
        return 130
    except Exception as error:
        if diagnostics is not None:
            diagnostics.record('error', error_type=type(error).__name__, message=str(error))
        print(f'只读回放失败：{error}', file=sys.stderr)
        return 1
    finally:
        if diagnostics is not None:
            diagnostics.close(reason)


if __name__ == '__main__':
    sys.exit(main())
