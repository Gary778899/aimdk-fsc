"""只读采样实际双臂反馈并执行 FK→IK→FK；不创建运动发布器或模式客户端。"""

import argparse
from dataclasses import asdict
import hashlib
import math
import os
from pathlib import Path
import platform
import sys
import time
import traceback

import numpy as np

from .dual_arm_control_core import ARM_NAMES, ControlFailure
from .upper_body_diagnostics import UpperBodyLog
from .upper_body_kinematics.adapter import ArmKinematics
from .upper_body_kinematics.parameters import parameter_report
from .upper_body_kinematics_check_core import diagnostic_cases, round_trip
from .upper_body_ros import UpperBodyFeedbackPort


def parser():
    """创建步骤三只读入口参数；不提供 --execute，不允许通过本命令执行运动。"""
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--side', choices=('left', 'right', 'both'), default='both')
    result.add_argument('--samples', type=int, default=3, help='实际快照数，默认3')
    result.add_argument('--interval', type=float, default=1., help='快照最小间隔秒数，默认1')
    result.add_argument('--timeout', type=float, default=5., help='启动/等待下一有效快照的超时秒数')
    result.add_argument('--feedback-timeout', type=float, default=.5,
                        help='复用既有接收间隔/源停滞/源年龄上限，0.2..1秒')
    result.add_argument('--check-cases', action='store_true', help='采样后增加纯数学边界及失败用例')
    result.add_argument('--log-dir', help='默认自动探测工作区log目录')
    result.add_argument('--label', default='', help='姿态或本次实验说明')
    result.add_argument('--sdk-version', default='unknown', help='操作者核对的SDK版本')
    result.add_argument('--firmware-version', default='unknown', help='操作者核对的固件版本')
    result.add_argument('--load-description', default='unknown', help='工具、负载及现场条件')
    return result


def validate_options(options):
    """校验采样参数；仅复用既有反馈时限范围，不新增精度或运动授权门槛。"""
    if options.samples < 1:
        raise ValueError('--samples 必须大于0')
    for name in ('interval', 'timeout'):
        value = getattr(options, name)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'--{name} 必须为有限正数')
    if not math.isfinite(options.feedback_timeout) or not .2 <= options.feedback_timeout <= 1.:
        raise ValueError('--feedback-timeout 必须为0.2..1秒')


def log_directory(value):
    """返回显式目录或按功能包标记探测的工作区log；探测失败要求显式指定目录。"""
    if value:
        return Path(value).expanduser().resolve()
    candidates = (Path.cwd(), *Path.cwd().parents, *Path(__file__).resolve().parents)
    for candidate in candidates:
        if (candidate / 'src/functions/package.xml').is_file():
            return candidate / 'log'
    raise ValueError('无法探测工作区，请指定 --log-dir /实际工作区/log')


def code_manifest():
    """记录实际加载安装位置及相关Python文件摘要，不依赖git或开发者源码绝对路径。"""
    root = Path(__file__).resolve().parent
    paths = (list(root.glob('upper_body_*.py'))
             + list((root / 'upper_body_kinematics').rglob('*.py'))
             + [root / 'dual_arm_control_core.py'])
    return {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(paths)}


def collect_snapshots(options, diagnostics):
    """在ROS回调持续处理期间采样完整反馈；返回快照列表，之后销毁订阅再做耗时求解。

    等待源时间推进，有界等待启动和下一快照；源时间、时效或硬件错误立即中止。
    各流独立采样，不声称硬件同步；不要求URS，不调用模式服务。
    """
    import rclpy

    node = None
    port = None
    captures = []
    rclpy.init(args=[])
    try:
        node = rclpy.create_node('upper_body_kinematics_check')
        port = UpperBodyFeedbackPort(node, diagnostics, require_urs=False,
                                     feedback_timeout=options.feedback_timeout)
        due = time.monotonic()
        deadline = due + options.timeout
        last_arm_stamp = None
        started = False
        while len(captures) < options.samples:
            if not rclpy.ok():
                raise ControlFailure('ROS上下文停止')
            rclpy.spin_once(node, timeout_sec=.01)
            now = time.monotonic()
            ros_now = node.get_clock().now().nanoseconds / 1e9
            for stream in port.streams.values():
                stream.monitor.observe_clock(now, ros_now)
                if stream.error:
                    raise ControlFailure(f'{stream.name}: {stream.error}')
            ready = all(stream.monitor.latest is not None
                        and stream.monitor.latest.stamp > stream.first_stamp
                        for stream in port.streams.values())
            if started or ready:
                values = port.checked()
                started = True
                if now >= due and (last_arm_stamp is None or values['arm'].stamp > last_arm_stamp):
                    capture = dict(index=len(captures), arm_names=list(ARM_NAMES),
                                   sampled_monotonic_s=now, sampled_ros_time_s=ros_now,
                                   streams={name: asdict(value) for name, value in values.items()})
                    diagnostics.record('snapshot', **capture)
                    captures.append(capture)
                    last_arm_stamp = values['arm'].stamp
                    due = now + options.interval
                    deadline = due + options.timeout
                    print(f'已采样 {len(captures)}/{options.samples}，'
                          f'实际模式={values["mc"].data["action"]} / '
                          f'{values["mc"].data["action_status"]}', flush=True)
            if len(captures) < options.samples and now >= deadline:
                missing = [name for name, stream in port.streams.items()
                           if stream.monitor.latest is None
                           or stream.monitor.latest.stamp <= stream.first_stamp]
                raise ControlFailure(f'等待有效反馈快照超时，未就绪流={missing}')
        diagnostics.record('capture_finished', count=len(captures),
                           note='订阅随后关闭，数学计算使用已保存快照，不代表实时反馈')
        return captures
    finally:
        try:
            if port is not None:
                port.close()
        finally:
            try:
                if node is not None:
                    node.destroy_node()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()


def analyze(captures, arms, options, diagnostics):
    """对每个快照逐侧计算并可选运行纯数学失败用例；返回不完整/契约异常计数。

    边界用例不设置残差通过阈值；正常快照无解/拒绝记为不完整，预期拒绝异常记为契约失败。
    """
    incomplete = 0
    observed = 0
    for capture in captures:
        for side, arm in arms.items():
            offset = 0 if side == 'left' else 7
            joints = capture['streams']['arm']['positions'][offset:offset + 7]
            result = round_trip(arm, joints)
            observed += 1
            incomplete += result['status'] != 'candidate_returned'
            diagnostics.record('round_trip', snapshot_index=capture['index'], side=side, **result)
            print(f'快照{capture["index"]} {side}: {result["status"]}，'
                  f'位置残差/m={result.get("position_residual_m")}，'
                  f'旋转残差/rad={result.get("rotation_residual_rad")}', flush=True)
            print(f'  FK位置/m={result["requested_position_m"]}；'
                  f'R_torso_tcp={result["requested_rotation_matrix"]}', flush=True)
    case_count = 0
    if options.check_cases:
        for side, arm in arms.items():
            for case, expectation, result in diagnostic_cases(arm):
                if expectation == 'observe':
                    matched = None
                elif expectation == 'no_solution_or_rejected':
                    matched = result['status'] in ('no_solution', 'rejected')
                else:
                    matched = result['status'] == expectation
                if matched is False or result['status'] == 'invalid_candidate':
                    incomplete += 1
                case_count += 1
                diagnostics.record('diagnostic_case', side=side, case=case,
                                   expectation=expectation, expectation_matched=matched, **result)
                print(f'数学用例 {side} {case}: {result["status"]}，预期符合={matched}', flush=True)
    diagnostics.record('summary', snapshot_count=len(captures), round_trip_count=observed,
                       diagnostic_case_count=case_count, incomplete_or_unexpected=incomplete,
                       residual_acceptance_threshold=None, motion_published=False)
    print(f'完成：{observed}次实际快照往返计算，{case_count}项数学用例，'
          f'不完整或契约异常={incomplete}。残差仅记录，不代表实机定位验收。', flush=True)
    return incomplete


def main(args=None):
    """运行只读命令；退出0表示采样及候选返回完整，2不完整，130中断，1环境/运行异常。"""
    options = parser().parse_args(args)
    diagnostics = None
    exit_code = 1
    reason = 'error'
    try:
        validate_options(options)
        diagnostics = UpperBodyLog(
            log_directory(options.log_dir), command='upper_body_kinematics_check',
            options=vars(options), python=sys.version, numpy=np.__version__,
            platform=platform.platform(), ros_distro=os.environ.get('ROS_DISTRO', 'unknown'),
            package_path=str(Path(__file__).resolve().parent), code_sha256=code_manifest(),
            parameters=parameter_report(), reference_commit='444598038cf0e5b27e32e3e5295ab24503ecd43f',
            tcp_mode='none', motion_enabled=False, mode_query_performed=False,
        )
        print(f'只读验证日志：{diagnostics.path}', flush=True)
        sides = ('left', 'right') if options.side == 'both' else (options.side,)
        arms = {side: ArmKinematics(side, tcp_mode='none') for side in sides}
        diagnostics.record('tools', tools={side: arm.tool for side, arm in arms.items()})
        captures = collect_snapshots(options, diagnostics)
        incomplete = analyze(captures, arms, options, diagnostics)
        exit_code = 2 if incomplete else 0
        reason = 'incomplete' if incomplete else 'completed'
    except KeyboardInterrupt:
        exit_code, reason = 130, 'interrupted'
        print('用户中断；已写入的反馈和快照保留。', file=sys.stderr)
    except Exception as error:
        print(f'只读验证失败：{type(error).__name__}: {error}', file=sys.stderr)
        if diagnostics is not None:
            try:
                diagnostics.record('error', error_type=type(error).__name__, message=str(error),
                                   traceback=traceback.format_exc())
            except Exception as log_error:
                print(f'错误日志写入失败：{log_error}', file=sys.stderr)
    finally:
        if diagnostics is not None:
            try:
                diagnostics.close(reason=reason)
            except Exception as error:
                exit_code = 1
                print(f'日志关闭失败：{error}', file=sys.stderr)
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
