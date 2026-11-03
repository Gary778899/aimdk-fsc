"""从XR记录和机器人命令／反馈快照回放双臂逐帧IK；只读、无ROS导入。"""

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

from remote_control.mapping import MappingConfig, Session
from remote_control.recording import read_frames

from .dual_arm_control_core import ARM_NAMES
from .upper_body_cartesian_core import CapturedTarget
from .upper_body_diagnostics import UpperBodyLog, code_manifest, log_directory
from .upper_body_online_core import DualArmOnlineIK, OnlineIKFailure


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


def replay_frames(frames, session, solver, diagnostics, reference_provider=None):
    """串行消费XR帧并联合提交映射与IK，返回接受数量；任一错误记录后终止。

    离线首帧使用所选快照建立TCP参考，后续Grip重按使用最后已接受理想目标；
    静态快照不能模拟现场首次Grip时重新采样HAL命令。reference_provider(frame)
    返回(robot_tcp_poses, hand_openings, command_reference, rebase_sides)，
    供现场入口在首次Grip接管时事务式重建HAL命令参考。
    """
    accepted = 0
    for frame in frames:
        diagnostics.record('xr_input', frame=frame.to_dict())
        mapping_proposal = None
        mapping_committed = False
        committed = False
        began = time.monotonic()
        try:
            poses, openings, reference, rebase_sides = (
                (solver.tcp_poses(), solver.accepted.hand_openings, None, ())
                if reference_provider is None else reference_provider(frame))
            mapping_proposal = session.propose(frame, poses, openings)
            diagnostics.record('tcp_target', target=mapping_proposal.target.to_dict())
            candidate = solver.propose(mapping_proposal.target, reference=reference,
                                       rebase_sides=rebase_sides)
            session.validate_commit(mapping_proposal)
            solver.validate_commit(candidate)
            # 两方预检后串行提交；入口独占会话和IK实例，期间不运行回调。
            session.commit(mapping_proposal)
            mapping_committed = True
            value = solver.commit(candidate)
            committed = True
            accepted += 1
            diagnostics.record('ik_accepted', input_seq=frame.seq,
                                accepted_monotonic_s=value.accepted_monotonic_s,
                                result=candidate.report, calculation_seconds=time.monotonic() - began)
        except Exception as error:
            if committed:
                raise
            if mapping_proposal is not None and not mapping_committed:
                session.reject(mapping_proposal)
            diagnostics.record('ik_rejected', input_seq=frame.seq,
                                reason=str(error), error_type=type(error).__name__,
                                result=error.report if isinstance(error, OnlineIKFailure) else None,
                                last_accepted_rad=list(solver.accepted.arm_rad),
                                calculation_seconds=time.monotonic() - began)
            raise
    if accepted == 0:
        raise ValueError('XR日志没有可回放的完整输入帧')
    diagnostics.record('replay_summary', accepted_frames=accepted, rejected_frames=0,
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
        captured, snapshot = read_snapshot(options.snapshot_log, options.snapshot_index,
                                           options.arm_start_source)
        config_path = Path(options.mapping_config).expanduser().resolve()
        config = MappingConfig.load(config_path)
        diagnostics = UpperBodyLog(
            log_directory(options.log_dir), command='upper_body_teleop_replay', options=vars(options),
            snapshot=snapshot, mapping_config=json.loads(config_path.read_text(encoding='utf-8')),
            mapping_config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
            code_sha256=code_manifest(), compensation_enabled=False, execute=False,
            reference_policy=f'{options.arm_start_source}_snapshot_then_last_accepted_ideal_target')
        print(f'只读回放日志：{diagnostics.path}', flush=True)
        solver = DualArmOnlineIK(captured)
        diagnostics.record('online_ik_config', **solver.describe())
        session = Session(config)
        count = replay_frames(read_frames(options.xr_log), session, solver, diagnostics)
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
