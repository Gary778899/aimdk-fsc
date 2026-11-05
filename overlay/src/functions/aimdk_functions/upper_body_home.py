"""记录URS下的双臂home命令及夹爪开度，或通过原生上肢接口平滑回位。"""

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

from .dual_arm_control_core import ARM_NAMES, ControlFailure, decode_urs_mc, interpolate
from .upper_body_cartesian_core import CapturedTarget
from .upper_body_control_core import validate_target
from .upper_body_diagnostics import UpperBodyLog, code_manifest, log_directory
from .upper_body_execution import UpperBodyExecutor
from .upper_body_mode import confirm_initial_mode


def parse_options(args):
    """解析record/run子命令；返回执行器参数，非法时长由argparse报错。"""
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='operation', required=True)
    for operation, help_text in (('record', '记录当前位置为home，覆盖配置'),
                                 ('run', '读取home配置，只读预览或显式执行')):
        command = commands.add_parser(operation, help=help_text)
        command.add_argument('--config', default='data/home.json',
                             help='配置路径，默认当前工作目录下data/home.json')
        command.add_argument('--timeout', type=float, default=5., help='启动/服务/匹配等待1..10秒')
        command.add_argument('--feedback-timeout', type=float, default=.5, help='反馈时限0.2..1秒')
        command.add_argument('--log-dir', help='默认工作区log目录')
        if operation == 'run':
            command.add_argument('--execute', action='store_true', help='在URS/RUNNING下发布回位目标')
            command.add_argument('--duration', type=float, default=5., help='回位时长/秒，默认5，须大于0')
        command.set_defaults(action='home', side='both', compensation=False,
                             arm_start_source='mc-command')
    options = parser.parse_args(args)
    if options.operation == 'record':
        options.execute, options.duration = False, 5.
    for name, lower, upper in (('timeout', 1., 10.), ('feedback_timeout', .2, 1.)):
        value = getattr(options, name)
        if not math.isfinite(value) or not lower <= value <= upper:
            parser.error(f'--{name.replace("_", "-")}须为{lower:g}..{upper:g}秒')
    if not math.isfinite(options.duration) or options.duration <= 0:
        parser.error('--duration须为有限正秒数')
    return options


def load_home(path):
    """读取v1配置，按完整关节名称重排并返回目标；格式不符抛出ControlFailure。"""
    document = json.loads(path.read_text(encoding='utf-8'))
    if (document['schema_version'] != 1 or document['robot_model'] != 'x2-ultra'
            or document['arm_unit'] != 'rad'):
        raise ControlFailure('home配置要求schema_version=1、robot_model=x2-ultra、arm_unit=rad')
    positions = document['arm_positions']
    if set(positions) != set(ARM_NAMES):
        raise ControlFailure('home配置须包含完整且仅有的14个双臂关节名称')
    openings = document['hand_openings']
    return CapturedTarget(tuple(positions[name] for name in ARM_NAMES),
                          (openings['left'], openings['right']))


def save_home(path, captured, values, diagnostics):
    """覆盖写入目标和采样来源；values保留源时间及实际反馈，写盘失败向上传播。"""
    document = dict(
        schema_version=1, robot_model='x2-ultra', urdf_version='v1.3.0',
        recorded_at_utc=datetime.now(timezone.utc).isoformat(),
        arm_unit='rad', arm_positions=dict(zip(ARM_NAMES, captured.arm_rad)),
        hand_openings=dict(zip(('left', 'right'), captured.hand_openings)),
        recording=dict(
            mode=values['mc'].data,
            arm_source='/aima/hal/joint/arm/command',
            hand_source='/aima/hal/joint/hand/state',
            source_stamp_s={name: value.stamp for name, value in values.items()},
            received_monotonic_s={name: value.received for name, value in values.items()},
            actual_arm_rad=list(values['arm'].positions),
            log_path=str(diagnostics.path)))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + '\n',
                    encoding='utf-8')
    diagnostics.record('home_recorded', config=str(path), configuration=document)


@dataclass(frozen=True)
class HomeTrajectory:
    """双臂rad及左右夹爪0..1开度共用五次进度的绝对目标轨迹。"""

    captured: CapturedTarget
    target: CapturedTarget
    duration: float
    kind: str = 'home'

    def at(self, elapsed):
        """返回经过elapsed秒的完整目标；起点和终点原样返回以避免插值尾数误差。"""
        if elapsed <= 0:
            return self.captured
        if elapsed >= self.duration:
            return self.target
        return CapturedTarget(
            interpolate(self.captured.arm_rad, self.target.arm_rad, elapsed, self.duration),
            interpolate(self.captured.hand_openings, self.target.hand_openings, elapsed, self.duration))


def main(args=None):
    """记录home或预览/执行回位；返回0完成、1失败、130中断，始终关闭ROS及日志资源。"""
    options = parse_options(args)
    path = Path(options.config).expanduser().resolve()
    diagnostics = node = port = client = None
    initialized = False
    result, reason = 1, 'failed'
    try:
        target = load_home(path) if options.operation == 'run' else None
        diagnostics = UpperBodyLog(log_directory(options.log_dir), command='upper_body_home',
                                   options=vars(options), config=str(path), code_sha256=code_manifest())
        print(f'配置：{path}\n日志：{diagnostics.path}', flush=True)
        import rclpy
        from rclpy.signals import SignalHandlerOptions

        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = rclpy.create_node('upper_body_home')
        if node.get_parameter('use_sim_time').value:
            raise ControlFailure('本命令仅用于真机，不支持use_sim_time')
        port = UpperBodyExecutor(rclpy, node, options, diagnostics)
        if options.operation == 'record':
            # 录制也要求URS/RUNNING，仍不创建运动发布器。
            port.feedback.streams['mc'].decoder = decode_urs_mc
        if options.operation == 'record' or options.execute:
            from aimdk_msgs.srv import GetMcAction

            client = node.create_client(GetMcAction, '/aimdk_5Fmsgs/srv/GetMcAction')
            confirm_initial_mode(rclpy, node, options, diagnostics, client)
        values = port.capture(refreshed_after=time.monotonic())
        captured = port.capture_target(values)
        if options.operation == 'record':
            validate_target(captured, captured, options)
            save_home(path, captured, values, diagnostics)
            print(f'已记录home：双臂/rad={list(captured.arm_rad)}；'
                  f'夹爪={list(captured.hand_openings)}', flush=True)
        else:
            validate_target(captured, target, options)
            trajectory = HomeTrajectory(captured, target, options.duration)
            diagnostics.record('home_plan', start_arm_rad=list(captured.arm_rad),
                               target_arm_rad=list(target.arm_rad),
                               start_hand_openings=list(captured.hand_openings),
                               target_hand_openings=list(target.hand_openings),
                               duration_s=options.duration, interpolation='quintic',
                               tracking_policy=port.tracking_policy,
                               compensation_enabled=False)
            print(f'双臂起点/rad={list(captured.arm_rad)}\n目标/rad={list(target.arm_rad)}\n'
                  f'夹爪：{captured.hand_openings} → {target.hand_openings}；'
                  f'时长={options.duration:g}s，补偿关闭；跟踪误差仅记录，终点按±5°确认。', flush=True)
            if options.execute:
                port.check_start(port.capture(refreshed_after=time.monotonic()))
                final = port.execute(captured, target, trajectory)
                diagnostics.record('home_result', actual_arm_rad=list(final['arm'].positions),
                                   actual_hand_openings=list(final['hand'].positions),
                                   target_arm_rad=list(target.arm_rad),
                                   target_hand_openings=list(target.hand_openings),
                                   hand_opening_check=False)
                print('双臂通过既有±5°终点确认，已停止发布。\n'
                      f'夹爪目标={target.hand_openings}；实测={final["hand"].positions}；'
                      '夹爪到位未判定。', flush=True)
            else:
                print('只读预览完成；增加--execute执行回位。', flush=True)
        result, reason = 0, 'completed'
    except KeyboardInterrupt:
        result, reason = 130, 'interrupted'
        if port is not None:
            port.stop()
        print('用户中断，停止发布。', file=sys.stderr)
    except Exception as error:
        if port is not None:
            port.stop()
        if diagnostics is not None:
            diagnostics.record('error', error_type=type(error).__name__, message=str(error))
        print(f'home失败：{error}', file=sys.stderr)
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
            if diagnostics is not None:
                diagnostics.close(reason=reason)
    return result


if __name__ == '__main__':
    sys.exit(main())
