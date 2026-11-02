"""URS持续发布下的两段TCP平移实验；默认只读，不切换模式。"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import math
from pathlib import Path
import sys
import time

from aimdk_functions.dual_arm_control_core import ControlFailure, check_arm
from aimdk_functions.upper_body_cartesian_core import CapturedTarget
from aimdk_functions.upper_body_control_core import plan_motion, selected_sides
from aimdk_functions.upper_body_diagnostics import UpperBodyLog, code_manifest, log_directory
from aimdk_functions.upper_body_execution import (
    ARM_ENDPOINT_TOLERANCE, ARM_TRACKING_TOLERANCE, UpperBodyExecutor,
)
from aimdk_functions.upper_body_kinematics.adapter import ArmKinematics
from aimdk_functions.upper_body_mode import confirm_initial_mode


def parse_options(args):
    """解析实验参数；时间单位秒、位移单位毫米，非法范围由argparse报错。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('left', 'right', 'both'), default='both')
    parser.add_argument('--axis', choices=('x', 'y', 'z'), default='x')
    parser.add_argument('--delta-mm', type=float, default=20., help='每段相对位移，0用于重设参考实验')
    parser.add_argument('--duration', type=float, default=5., help='每段运动时间，5..10秒')
    parser.add_argument('--hold', type=float, default=1., help='第一段确认后持续发送终点的时间，至少0秒')
    parser.add_argument('--second-origin', choices=('command', 'feedback'), default='command',
                        help='第二段从上一目标或保持结束时的实际位置开始')
    parser.add_argument('--timeout', type=float, default=5., help='启动等待及第二段规划时限，1..10秒')
    parser.add_argument('--feedback-timeout', type=float, default=.5, help='反馈时限，0.2..1秒')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--log-dir')
    parser.add_argument('--load-description', default='unknown')
    options = parser.parse_args(args)
    for name, lower, upper in (('duration', 5., 10.), ('timeout', 1., 10.),
                               ('feedback_timeout', .2, 1.)):
        value = getattr(options, name)
        if not math.isfinite(value) or not lower <= value <= upper:
            parser.error(f'{name}须为{lower:g}..{upper:g}秒')
    if not math.isfinite(options.hold) or options.hold < 0:
        parser.error('--hold须为有限非负秒数')
    if not math.isfinite(options.delta_mm):
        parser.error('--delta-mm须为有限毫米数')
    # 与既有执行器共享TCP目标契约；本实验固定关闭自研补偿。
    options.action, options.path, options.compensation = 'tcp', 'endpoint', False
    displacement = [0., 0., 0.]
    displacement[('x', 'y', 'z').index(options.axis)] = options.delta_mm
    options.requests = {side: tuple(displacement) for side in selected_sides(options.side)}
    return options


def target_from_feedback(values, previous=None, side='both'):
    """构造完整目标；重采样时仅更新所选臂，其他关节和夹爪保留上一命令。"""
    if previous is None:
        return CapturedTarget(values['arm'].positions, values['hand'].positions)
    positions = list(previous.arm_rad)
    for selected in selected_sides(side):
        offset = 0 if selected == 'left' else 7
        positions[offset:offset + 7] = values['arm'].positions[offset:offset + 7]
    return CapturedTarget(tuple(positions), previous.hand_openings)


def observe(port):
    """处理一次回调并复用既有5度跟踪检查；返回最新完整反馈，异常向上传播。"""
    values = port.spin()
    if port.last_sent is not None:
        check_arm(values['arm'].positions, port.last_sent.arm_rad,
                  tolerance=ARM_TRACKING_TOLERANCE)
    return values


def run_segment(port, trajectory, index):
    """启动一段轨迹并持续发布；沿用5度、0.3秒及额外2秒终点确认，结束时保持发布。"""
    initial = port.feedback.checked()
    port.trajectory = trajectory
    port.began = time.monotonic()
    port.command_target = trajectory.at(0.)
    stable_since = None
    arrival = port.began + port.options.duration
    port.diagnostics.record('segment_begin', index=index, began_monotonic_s=port.began,
                            origin_rad=list(trajectory.captured.arm_rad),
                            target_rad=list(trajectory.target.arm_rad),
                            previous_sent_rad=(list(port.last_sent.arm_rad)
                                               if port.last_sent is not None else None))
    while True:
        values = observe(port)
        now = time.monotonic()
        if now < arrival:
            continue
        fresh = (port.last_publish is not None and port.last_publish >= arrival
                 and port.last_elapsed >= port.options.duration
                 and all(values[name].received > arrival
                         and values[name].serial > initial[name].serial
                         and values[name].stamp > initial[name].stamp for name in ('arm', 'hand')))
        reached = all(abs(actual - desired) <= ARM_ENDPOINT_TOLERANCE
                      for actual, desired in zip(values['arm'].positions, trajectory.target.arm_rad))
        stable_since = (now if stable_since is None else stable_since) if fresh and reached else None
        if stable_since is not None and now - stable_since >= .3:
            port.diagnostics.record('segment_confirmed', index=index,
                                    actual_arm_rad=list(values['arm'].positions),
                                    target_rad=list(trajectory.target.arm_rad))
            return values
        if now > arrival + 2.:
            raise ControlFailure(f'第{index}段终点未在额外2秒内连续确认0.3秒')


def plan_second(port, origin, arms, pool):
    """后台规划第二段并保持前段发布；超时或规划失败抛出异常，不自动执行旧规划。"""
    began = time.monotonic()
    future = pool.submit(plan_motion, origin, port.options, arms)
    while not future.done():
        observe(port)
        if time.monotonic() - began > port.options.timeout:
            raise ControlFailure('第二段规划超时')
    trajectory, reports = future.result()
    port.diagnostics.record('second_plan', origin_rad=list(origin.arm_rad),
                            target_rad=list(trajectory.target.arm_rad), reports=reports,
                            planning_seconds=time.monotonic() - began)
    return trajectory


def main(args=None):
    """只读预览或执行两段实验；返回0完成、1失败、130中断，退出时停止发布。"""
    options = parse_options(args)
    diagnostics = node = port = client = pool = None
    initialized = False
    reason, result = 'failed', 1
    try:
        diagnostics = UpperBodyLog(
            log_directory(options.log_dir), command='upper_body_sequence_test', options=vars(options),
            code_sha256=code_manifest(),
            experiment_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            compensation_enabled=False, tcp='wrist_origin_assumed_flange_center',
            sdk_version='unknown', firmware_version='unknown')
        print(f'日志：{diagnostics.path}', flush=True)
        import rclpy
        from rclpy.signals import SignalHandlerOptions

        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = rclpy.create_node('upper_body_sequence_test')
        if node.get_parameter('use_sim_time').value:
            raise ControlFailure('本实验仅用于真机，不支持use_sim_time')
        port = UpperBodyExecutor(rclpy, node, options, diagnostics)
        if options.execute:
            from aimdk_msgs.srv import GetMcAction

            client = node.create_client(GetMcAction, '/aimdk_5Fmsgs/srv/GetMcAction')
            confirm_initial_mode(rclpy, node, options, diagnostics, client)
        port.capture()
        arms = {side: ArmKinematics(side, tcp_mode='none') for side in selected_sides(options.side)}
        values = port.capture(refreshed_after=time.monotonic())
        captured = target_from_feedback(values)
        first, reports = plan_motion(captured, options, arms)
        diagnostics.record('first_plan', origin_rad=list(captured.arm_rad),
                            target_rad=list(first.target.arm_rad), reports=reports)
        # command分支的第二段可提前规划，保持阶段不引入额外求解等待。
        second = None
        if options.second_origin == 'command' or not options.execute:
            second, reports = plan_motion(first.target, options, arms)
            diagnostics.record('second_preview', target_rad=list(second.target.arm_rad),
                                reports=reports, provisional=options.second_origin == 'feedback')
        if not options.execute:
            print('只读预览完成；feedback分支第二段为基于第一目标的暂定预览，执行时重新采样。')
        else:
            refreshed = port.capture(refreshed_after=time.monotonic())
            check_arm(refreshed['arm'].positions, captured.arm_rad)
            port.enable_commands(captured)
            run_segment(port, first, 1)
            hold_began = time.monotonic()
            diagnostics.record('hold_begin', target_rad=list(first.target.arm_rad))
            values = port.feedback.checked()
            while time.monotonic() - hold_began < options.hold:
                values = observe(port)
            if options.second_origin == 'feedback':
                origin = target_from_feedback(values, first.target, options.side)
                diagnostics.record('second_origin_capture', origin_rad=list(origin.arm_rad),
                                    source_stamp_s=values['arm'].stamp,
                                    received_monotonic_s=values['arm'].received)
                pool = ThreadPoolExecutor(max_workers=1)
                second = plan_second(port, origin, arms, pool)
            diagnostics.record('hold_end', actual_hold_seconds=time.monotonic() - hold_began)
            run_segment(port, second, 2)
            port.stop()
            print('两段执行完成；已停止发布，不切换模式。完成条件为关节±5°，TCP位移需结合日志评价。')
        result, reason = 0, 'completed'
    except KeyboardInterrupt:
        result, reason = 130, 'interrupted'
    except Exception as error:
        if diagnostics is not None:
            diagnostics.record('error', error_type=type(error).__name__, message=str(error))
        print(f'失败：{error}', file=sys.stderr)
    finally:
        # 先停止运动发布，再处理求解线程及ROS资源。
        try:
            if port is not None:
                port.close()
            if pool is not None:
                pool.shutdown(wait=True)
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
