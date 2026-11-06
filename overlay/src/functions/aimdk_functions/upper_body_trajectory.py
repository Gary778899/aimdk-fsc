"""将遥操已发送目标导出为独立轨迹文件，或预览/执行双臂与夹爪轨迹回放。"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

from .upper_body_control_core import validate_target
from .upper_body_diagnostics import UpperBodyLog, code_manifest, log_directory
from .upper_body_execution import UpperBodyExecutor
from .upper_body_mode import confirm_initial_mode
from .upper_body_trajectory_core import ReplayTrajectory, export_trajectory, load_trajectory


def parse_options(args):
    """解析export/run操作；执行时要求显式过渡秒数，时限沿用现有执行器范围。"""
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='operation', required=True)
    export = commands.add_parser('export', help='本地离线提取已发布目标，独占创建轨迹JSONL')
    export.add_argument('--log', required=True, help='upper_body_teleop_observe执行日志')
    export.add_argument('--output', help='默认data/trajectories/<源日志名>.jsonl')
    export.add_argument('--start', type=float, default=0., help='相对首条发布的窗口起点/秒')
    export.add_argument('--end', type=float, help='相对首条发布的窗口终点/秒，默认末条')
    run = commands.add_parser('run', help='离线预览轨迹，--execute在机器人执行')
    run.add_argument('--trajectory', required=True, help='轨迹JSONL路径')
    run.add_argument('--execute', action='store_true', help='显式允许起点过渡及轨迹发布')
    run.add_argument('--start-duration', type=float, help='从当前姿态过渡到轨迹首点的秒数；执行必填')
    run.add_argument('--speed', type=float, default=1., help='原轨迹时间倍速，默认1')
    run.add_argument('--timeout', type=float, default=5., help='启动/服务/匹配等待1..10秒')
    run.add_argument('--feedback-timeout', type=float, default=.5, help='反馈时限0.2..1秒')
    run.add_argument('--log-dir', help='回放日志父目录，默认工作区log')
    run.set_defaults(action='trajectory', side='both', compensation=False,
                     arm_start_source='mc-command')
    options = parser.parse_args(args)
    if options.operation == 'run':
        for name in ('speed', 'start_duration'):
            value = getattr(options, name)
            if value is not None and (not math.isfinite(value) or value <= 0):
                parser.error(f'--{name.replace("_", "-")}须为有限正数')
        if options.execute and options.start_duration is None:
            parser.error('--execute要求显式指定--start-duration秒数')
        for name, lower, upper in (('timeout', 1., 10.), ('feedback_timeout', .2, 1.)):
            value = getattr(options, name)
            if not math.isfinite(value) or not lower <= value <= upper:
                parser.error(f'--{name.replace("_", "-")}须为{lower:g}..{upper:g}秒')
    return options


class TrajectoryExecutor(UpperBodyExecutor):
    """复用单次原生执行器，并为每条回放发布记录原轨迹点及源日志位置。"""

    def publish(self):
        """执行既有校验与发布成功后记录源点；过渡阶段记录为空的源点索引。"""
        previous = self.sequence
        super().publish()
        if self.sequence == previous:
            return
        sample = self.trajectory.sample(self.last_elapsed)
        index = sample['point_index']
        point = None if index is None else self.trajectory.recording.points[index]
        self.diagnostics.record(
            'trajectory_sample_published', sequence=(self.sequence-1) % (2**32),
            elapsed_motion_s=self.last_elapsed, publish_monotonic_s=self.last_publish,
            source_line=None if point is None else point.source_line,
            source_sequence=None if point is None else point.source_sequence,
            source_point_time_s=None if point is None else point.time_s, **sample)


def execute_recording(options, recording, diagnostics):
    """在机器人Humble容器采样、查询URS、过渡并回放；异常向上传播，始终关闭通信资源。"""
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from aimdk_msgs.srv import GetMcAction

    node = port = client = None
    initialized = False
    try:
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = rclpy.create_node('upper_body_trajectory')
        if node.get_parameter('use_sim_time').value:
            raise ValueError('本命令仅用于真机，不支持use_sim_time')
        port = TrajectoryExecutor(rclpy, node, options, diagnostics)
        client = node.create_client(GetMcAction, '/aimdk_5Fmsgs/srv/GetMcAction')
        confirm_initial_mode(rclpy, node, options, diagnostics, client)
        captured = port.capture_target(port.capture(refreshed_after=time.monotonic()))
        validate_target(captured, recording.points[0].target, options)
        trajectory = ReplayTrajectory(captured, recording, options.start_duration, options.speed)
        options.duration = trajectory.duration
        target = recording.points[-1].target
        diagnostics.record('trajectory_plan', **recording.summary(), speed=options.speed,
                           start_duration_s=options.start_duration, total_duration_s=trajectory.duration,
                           captured_arm_rad=list(captured.arm_rad),
                           captured_hand_openings=list(captured.hand_openings),
                           interpolation='quintic_approach_then_zero_order_hold',
                           tracking_policy=port.tracking_policy, compensation_enabled=False)
        print(f'当前双臂/rad={list(captured.arm_rad)}\n'
              f'当前夹爪={captured.hand_openings}；起点过渡={options.start_duration:g}s；'
              f'轨迹={recording.times[-1]/options.speed:.3f}s。', flush=True)
        port.check_start(port.capture(refreshed_after=time.monotonic()))
        final = port.execute(captured, target, trajectory)
        diagnostics.record('trajectory_result', actual_arm_rad=list(final['arm'].positions),
                           actual_hand_openings=list(final['hand'].positions),
                           target_arm_rad=list(target.arm_rad), target_hand_openings=list(target.hand_openings),
                           hand_opening_check=False)
        print('回放完成，双臂终点通过既有±5°持续0.3秒确认，已停发；夹爪到位未判定。', flush=True)
    finally:
        if port is not None:
            port.close()
        if client is not None:
            node.destroy_client(client)
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    """导出、离线预览或显式执行轨迹；返回0完成、1失败、130中断。"""
    options = parse_options(args)
    diagnostics, result, reason = None, 1, 'failed'
    try:
        if options.operation == 'export':
            source = Path(options.log).expanduser().resolve()
            output = (Path(options.output).expanduser() if options.output else
                      Path('data/trajectories') / source.name).resolve()
            recording = export_trajectory(source, output, options.start, options.end)
            print(f'轨迹已导出并读回：{output}', flush=True)
        else:
            path = Path(options.trajectory).expanduser().resolve()
            recording = load_trajectory(path)
            print(f'轨迹：{path}\n倍速={options.speed:g}；'
                  f'回放时长={recording.times[-1]/options.speed:.3f}s；额外补偿关闭。', flush=True)
        print(json.dumps(recording.summary(), ensure_ascii=False, indent=2), flush=True)
        for entry in recording.metadata.get('source_exits', []):
            print(f'源日志结束记录：{json.dumps(entry, ensure_ascii=False)}', flush=True)
        if options.operation == 'run' and options.execute:
            diagnostics = UpperBodyLog(
                log_directory(options.log_dir), command='upper_body_trajectory', options=vars(options),
                trajectory_path=str(path), trajectory_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                trajectory_metadata=recording.metadata, code_sha256=code_manifest(), compensation_enabled=False)
            print(f'回放日志：{diagnostics.path}', flush=True)
            execute_recording(options, recording, diagnostics)
        elif options.operation == 'run':
            print('离线预览完成；执行需指定--start-duration秒数及--execute。', flush=True)
        result, reason = 0, 'completed'
    except KeyboardInterrupt:
        result, reason = 130, 'interrupted'
        print('用户中断，结束回放。', file=sys.stderr)
    except Exception as error:
        if diagnostics is not None:
            diagnostics.record('error', error_type=type(error).__name__, message=str(error))
        print(f'轨迹操作失败：{error}', file=sys.stderr)
    finally:
        if diagnostics is not None:
            diagnostics.close(reason=reason)
    return result


if __name__ == '__main__':
    sys.exit(main())
