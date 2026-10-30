"""单臂端点运动及直线命令共用执行层；默认端点MoveJ，只读且不自动返程或切换模式。"""

import argparse
from dataclasses import asdict
import math
import os
import sys
import time

from .dual_arm_control import confirm_initial_mode
from .dual_arm_control_core import (
    ARM_NAMES, ARM_POSITION_TOLERANCE, ControlFailure, check_arm, interpolate,
)
from .upper_body_cartesian_core import CapturedTarget
from .upper_body_diagnostics import UpperBodyLog
from .upper_body_kinematics.adapter import ArmKinematics
from .upper_body_kinematics.parameters import parameter_report
from .upper_body_kinematics_check import code_manifest, log_directory
from .upper_body_move_core import plan_translation
from .upper_body_ros import UpperBodyFeedbackPort, build_upper_body_command


def parser(*, line=False):
    """创建平移命令参数；line选择直线路径说明，delta-mm为torso单轴位移。"""
    result = argparse.ArgumentParser(description=('单臂直线离散IK跟踪；默认只读，显式执行。' if line else __doc__))
    result.add_argument('--side', choices=('left', 'right'), required=True)
    result.add_argument('--axis', choices=('x', 'y', 'z'), default='x')
    result.add_argument('--delta-mm', type=float, required=True)
    result.add_argument('--duration', type=float, default=5., help='关节插值5..10秒，默认5')
    result.add_argument('--execute', action='store_true', help='显式执行；操作者预先进入URS/RUNNING')
    result.add_argument('--timeout', type=float, default=5., help='启动/服务/匹配超时1..10秒')
    result.add_argument('--feedback-timeout', type=float, default=.5, help='反馈时限0.2..1秒')
    result.add_argument('--compensation', action='store_true', help='显式启用固定参数位置偏置适配版')
    result.add_argument('--payload-mass-kg', type=float, default=0., help='腕末端附加载荷质量，默认0kg')
    result.add_argument('--payload-com-m', type=float, nargs=3, default=(0., 0., 0.),
                        metavar=('X', 'Y', 'Z'), help='负载质心相对腕原点，在腕坐标系中表达/m')
    result.add_argument('--load-description', default='unknown', help='实际姿态、工具、负载及实验条件')
    result.add_argument('--log-dir', help='默认自动探测工作区log目录')
    return result


def validate_options(options):
    """校验有限位移及沿用既有实验时长、等待和反馈时限；无效时抛出ControlFailure。"""
    if (not math.isfinite(options.payload_mass_kg) or options.payload_mass_kg < 0
            or not all(math.isfinite(v) for v in options.payload_com_m)):
        raise ControlFailure('负载质量须为有限非负kg，质心须为三个有限m数值')
    if not options.compensation and (options.payload_mass_kg != 0 or any(options.payload_com_m)):
        raise ControlFailure('负载参数需要同时指定--compensation')
    if not math.isfinite(options.delta_mm):
        raise ControlFailure('--delta-mm必须为有限数值')
    for name, lower, upper in (('duration', 5., 10.), ('timeout', 1., 10.),
                               ('feedback_timeout', .2, 1.)):
        value = getattr(options, name)
        if not math.isfinite(value) or not lower <= value <= upper:
            raise ControlFailure(f'--{name.replace("_", "-")}须为{lower:g}..{upper:g}秒')


class MovePort:
    """复用原始反馈与既有执行约定；只负责一次关节插值，不求解周期IK。"""

    def __init__(self, rclpy, node, options, diagnostics):
        """创建只读反馈适配，execute时持续监视URS；发布器延迟到enable_commands创建。"""
        self.rclpy, self.node, self.options, self.diagnostics = rclpy, node, options, diagnostics
        self.feedback = UpperBodyFeedbackPort(
            node, diagnostics, require_urs=options.execute,
            feedback_timeout=options.feedback_timeout, compensation=options.compensation)
        self.publisher = self.timer = self.command_target = None
        self.compensation = None
        self.compensation_details = None
        self.last_sent = None
        self.last_sent_compensation = None
        self.captured = None
        self.sequence = 0
        self.last_publish = None
        self.stopped = False

    def spin(self, ready=True):
        """处理一次反馈/定时回调并观测时钟；ready时返回时效及错误校验后的完整反馈。"""
        self.rclpy.spin_once(self.node, timeout_sec=.01)
        if not self.rclpy.ok():
            raise ControlFailure('ROS上下文停止')
        now, ros_now = time.monotonic(), self.node.get_clock().now().nanoseconds / 1e9
        for stream in self.feedback.streams.values():
            stream.monitor.observe_clock(now, ros_now)
            if stream.error:
                raise ControlFailure(f'{stream.name}: {stream.error}')
        return self.feedback.checked() if ready else None

    def capture(self, refreshed_after=None):
        """有界等待各流推进并返回一次快照；可要求接收时间晚于求解完成时刻以刷新回调。

        求解期间未创建发布器；刷新时要求源时间也晚于本次刷新开始的ROS时间，
        避免规划期间积压的旧消息因回调刚执行而被误认为新样本。
        有界等待期间持续解码并保留错误锁存，收齐后仍按原时限检查真实源时间，
        不改写时间、不扩大时限；冻结反馈最终超时。
        """
        deadline = time.monotonic() + self.options.timeout
        source_after = (self.node.get_clock().now().nanoseconds / 1e9
                        if refreshed_after is not None else None)
        if source_after is not None:
            self.diagnostics.record('capture_refresh_begin',
                                    received_after_monotonic_s=refreshed_after,
                                    source_after_ros_s=source_after)
        while True:
            self.spin(ready=False)
            ready = all(stream.monitor.latest is not None
                        and stream.monitor.latest.stamp > stream.first_stamp
                        and (refreshed_after is None
                             or (stream.monitor.latest.received >= refreshed_after
                                 and stream.monitor.latest.stamp > source_after))
                        for stream in self.feedback.streams.values())
            if ready:
                values = self.feedback.checked()
                self.diagnostics.record('capture', refreshed=refreshed_after is not None,
                                        streams={name: asdict(value) for name, value in values.items()})
                return values
            if time.monotonic() >= deadline:
                self.diagnostics.record(
                    'capture_timeout', source_after_ros_s=source_after,
                    streams={name: (asdict(stream.monitor.latest)
                                    if stream.monitor.latest is not None else None)
                             for name, stream in self.feedback.streams.items()})
                raise ControlFailure('等待完整且源时间推进的反馈超时'
                                     '（刷新要求源时间晚于刷新开始）'
                                     if source_after is not None else '等待完整且源时间推进的反馈超时')

    def enable_commands(self, captured):
        """仅显式执行时创建50Hz发布器/定时器，等待DDS匹配并复核计划起点未漂移超过既有3度。"""
        from aimdk_msgs.msg import UpperBodyCommandArray

        if not self.options.execute:
            raise ControlFailure('未显式指定--execute')
        self.captured = captured
        self.publisher = self.node.create_publisher(UpperBodyCommandArray, '/mc/upper_body_command', 10)
        self.timer = self.node.create_timer(.02, self.publish)
        deadline = time.monotonic() + self.options.timeout
        while self.publisher.get_subscription_count() == 0:
            self.spin()
            if time.monotonic() >= deadline:
                raise ControlFailure('上肢命令DDS订阅匹配超时')
        values = self.feedback.checked()
        check_arm(values['arm'].positions, captured.arm_rad)
        self.diagnostics.record('command_transport', period_s=.02, publish_gap_limit_s=.08,
                                subscription_count=self.publisher.get_subscription_count(),
                                source='native_upper_body', mode='URS/RUNNING')

    def publish(self):
        """按50Hz发布完整目标；发布前检查反馈、模式、关节跟踪、所选臂限位及既有80ms间隔。"""
        if self.command_target is None:
            return
        values = self.feedback.checked()
        now = time.monotonic()
        gap = None if self.last_publish is None else now - self.last_publish
        if gap is not None and gap > .08:
            raise ControlFailure(f'命令发布间隔{gap:.3f}秒超过0.08秒')
        target = self.command_target
        offset = 0 if self.options.side == 'left' else 7
        target = self.captured.for_arm(self.options.side, target.arm_rad[offset:offset + 7])
        check_arm(values['arm'].positions, target.arm_rad)
        msg = build_upper_body_command(self.node, target, self.sequence % (2 ** 32))
        self.publisher.publish(msg)
        self.last_sent = target
        self.last_sent_compensation = self.compensation_details
        self.last_publish = now
        self.sequence += 1
        self.diagnostics.record('command_published', sequence=msg.header.sequence,
                                arm_rad=list(target.arm_rad), hand_openings=list(target.hand_openings),
                                command_stamp_s=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                                publish_gap_s=gap, compensation=self.compensation_details)

    def execute(self, captured, target, trajectory=None):
        """执行一次五次关节插值，终点新反馈在既有3度容差内持续0.3秒；额外最多2秒，不以夹爪状态判定到位。

        trajectory可提供预先接受的直线路径；省略时沿用端点关节五次插值。
        """
        self.enable_commands(captured)
        first = self.feedback.checked()
        began = time.monotonic()
        arrival = began + self.options.duration
        stable_since = None
        self.command_target = captured
        self.diagnostics.record('motion_begin', origin_rad=list(captured.arm_rad),
                                target_rad=list(target.arm_rad), duration_s=self.options.duration,
                                tracking_tolerance_deg=math.degrees(ARM_POSITION_TOLERANCE),
                                settling_timeout_s=2., endpoint_stable_s=.3,
                                interpolation=('line_ik_joint_linear_between_samples'
                                               if trajectory is not None else 'joint_quintic'),
                                ideal_cartesian_straight_line=trajectory is not None,
                                cartesian_straight_line=(trajectory is not None
                                                         and self.compensation is None),
                                compensation_enabled=self.compensation is not None,
                                began_monotonic_s=began)
        while True:
            values = self.spin()
            now = time.monotonic()
            positions = (trajectory.at(now - began) if trajectory is not None else
                         interpolate(captured.arm_rad, target.arm_rad, now - began, self.options.duration))
            ideal = CapturedTarget(positions, captured.hand_openings)
            expected = target
            if self.compensation is not None:
                blend = interpolate((0.,), (1.,), now - began, self.options.duration)[0]
                self.command_target, self.compensation_details = self.compensation.apply(
                    captured, ideal, values, blend)
                expected = self.command_target
            else:
                self.command_target = ideal
            check_arm(values['arm'].positions, self.command_target.arm_rad)
            if now >= arrival:
                if self.compensation is not None and self.last_sent is not None:
                    expected = self.last_sent
                newer = self.sequence > 0 and all(
                    values[name].serial > first[name].serial
                    and values[name].stamp > first[name].stamp
                    and values[name].received > arrival for name in ('arm', 'hand'))
                if self.compensation is not None:
                    newer = (newer and self.last_sent_compensation is not None
                             and self.last_sent_compensation['blend'] == 1.)
                reached = all(abs(actual - desired) <= ARM_POSITION_TOLERANCE
                              for actual, desired in zip(values['arm'].positions, expected.arm_rad))
                stable_since = (stable_since if stable_since is not None else now) if newer and reached else None
                if stable_since is not None and now - stable_since >= .3:
                    self.stop()
                    self.diagnostics.record('endpoint_confirmed', arm_rad=list(values['arm'].positions),
                                            target_rad=list(expected.arm_rad),
                                            ideal_target_rad=list(target.arm_rad),
                                            hand_openings=list(values['hand'].positions),
                                            hand_target=list(captured.hand_openings),
                                            hand_opening_check=False, hand_state_check=False,
                                            published_count=self.sequence)
                    return values
                if now > arrival + 2.:
                    self.diagnostics.record('endpoint_timeout', arm_rad=list(values['arm'].positions),
                                            target_rad=list(expected.arm_rad), ideal_target_rad=list(target.arm_rad), fresh_feedback=newer,
                                            hand_states=values['hand'].data['hands'])
                    raise ControlFailure('关节终点未在额外2秒内连续确认0.3秒')

    def stop(self):
        """先清除目标并取消定时器，停止本脚本发布；不发送返程、模式切换或急停命令。"""
        self.command_target = None
        if self.timer is not None:
            self.timer.cancel()
        if not self.stopped:
            self.stopped = True
            self.diagnostics.record('commands_stopped', published_count=self.sequence)

    def close(self):
        """停止发布后销毁本实例的通信资源；不改变机器人模式或退出后的物理保持行为。"""
        self.stop()
        if self.timer is not None:
            self.node.destroy_timer(self.timer)
            self.timer = None
        if self.publisher is not None:
            self.node.destroy_publisher(self.publisher)
            self.publisher = None
        self.feedback.close()


def main(args=None, *, line=False):
    """执行只读计划或显式单程运动；line选择逐点直线IK，否则使用端点MoveJ。

    args为命令参数，返回0完成、1失败、130人工中断；退出不自动返程。
    """
    options = parser(line=line).parse_args(args)
    diagnostics = node = port = client = None
    initialized = False
    exit_code, reason = 1, 'failed'
    try:
        validate_options(options)
        diagnostics = UpperBodyLog(log_directory(options.log_dir), command='upper_body_line' if line else 'upper_body_move',
                                   options=vars(options), parameters=parameter_report(),
                                   code_sha256=code_manifest(), python=sys.version,
                                   ros_distro=os.environ.get('ROS_DISTRO', 'unknown'),
                                   sdk_version='unknown', firmware_version='unknown',
                                   tcp='none', compensation_enabled=options.compensation,
                                   hand_state_check=False, hand_fault_check=True)
        print(f'运动验证日志：{diagnostics.path}', flush=True)
        # 本轮屏蔽夹爪状态/静止门槛，原始反馈及解码器的故障、时效检查仍保留。
        print('本轮已屏蔽夹爪状态及静止检查；保持采样开度，继续记录状态并检查故障码。', flush=True)
        arm = ArmKinematics(options.side, tcp_mode='none')
        diagnostics.record('tool', side=options.side, tool=arm.tool)
        compensation = None
        if options.compensation:
            from .upper_body_compensation import PositionCompensation

            compensation = PositionCompensation(arm, options.payload_mass_kg, options.payload_com_m)
            diagnostics.record('compensation_config', **compensation.describe())
            print('启用位置偏置适配版：刚度40 N·m/rad，偏置限幅±12°；'
                  '随本次五次进度从零引入，不裁剪最终目标。', flush=True)
        import rclpy
        from rclpy.signals import SignalHandlerOptions

        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = rclpy.create_node('upper_body_line' if line else 'upper_body_move')
        if node.get_parameter('use_sim_time').value:
            raise ControlFailure('本命令仅用于真机，不支持use_sim_time')
        if options.execute:
            from aimdk_msgs.srv import GetMcAction

            client = node.create_client(GetMcAction, '/aimdk_5Fmsgs/srv/GetMcAction')
            confirm_initial_mode(rclpy, node, options, diagnostics, client)
        port = MovePort(rclpy, node, options, diagnostics)
        port.compensation = compensation
        values = port.capture()
        captured = CapturedTarget(values['arm'].positions, values['hand'].positions)
        trajectory = None
        if line:
            from .upper_body_line_core import plan_line

            target, report, trajectory = plan_line(
                captured, options.side, options.axis, options.delta_mm, options.duration, arm=arm)
            for point in trajectory.points:
                diagnostics.record('line_plan_point', **point)
            print(f'直线规划{report["point_count"]}点；最大位置残差/m='
                  f'{report["max_path_position_residual_m"]}，最大姿态残差/rad='
                  f'{report["max_path_rotation_residual_rad"]}，最大相邻关节变化/度='
                  f'{report["max_adjacent_joint_delta_deg"]}', flush=True)
        else:
            target, report = plan_translation(captured, options.side, options.axis, options.delta_mm, arm=arm)
        diagnostics.record('plan', **report)
        if compensation is not None:
            # 先用采样重力逐点预览；执行阶段重新刷新并逐周期计算，不沿用此快照。
            count = int(math.ceil(options.duration / .02))
            for index in range(count + 1):
                seconds = options.duration * index / count
                positions = (trajectory.at(seconds) if trajectory is not None else
                             interpolate(captured.arm_rad, target.arm_rad, seconds, options.duration))
                ideal = CapturedTarget(positions, captured.hand_openings)
                preview, details = compensation.apply(
                    captured, ideal, values, interpolate((0.,), (1.,), seconds, options.duration)[0])
                diagnostics.record('compensation_plan_point', index=index, time_s=seconds, **details)
            print(f'补偿后终点/rad={list(preview.arm_rad)}；'
                  f'终点偏置/度={[math.degrees(v) for v in details["bias_rad"]]}', flush=True)
        path_description = ('直线离散IK跟踪，点间关节插值近似直线' if line
                            else '关节插值，路径不保证笛卡尔直线')
        print(f'计划：{options.side}腕原点在torso的{options.axis}轴平移{options.delta_mm:+g}mm，'
              f'{options.duration:g}秒{path_description}。', flush=True)
        print(f'起点FK位置/m={report["start_position_m"]}；请求位置/m={report["requested_position_m"]}；'
              f'候选FK位置/m={report["target_position_m"]}', flush=True)
        print(f'IK位置残差/m={report["position_residual_m"]}，旋转残差/rad={report["rotation_residual_rad"]}', flush=True)
        for name, actual, expected in zip(ARM_NAMES, captured.arm_rad, target.arm_rad):
            print(f'{name}: {math.degrees(actual):+.3f}° → {math.degrees(expected):+.3f}°，'
                  f'变化{math.degrees(expected-actual):+.3f}°', flush=True)
        if options.execute:
            refreshed = port.capture(refreshed_after=time.monotonic())
            check_arm(refreshed['arm'].positions, captured.arm_rad)
            final = port.execute(captured, target, trajectory=trajectory)
            offset = 0 if options.side == 'left' else 7
            actual_position, actual_rotation = arm.forward(final['arm'].positions[offset:offset + 7])
            diagnostics.record('execution_result', actual_arm_rad=list(final['arm'].positions),
                               target_arm_rad=list(target.arm_rad),
                               sent_arm_rad=(port.last_sent_compensation['sent_arm_rad']
                                             if port.last_sent_compensation else list(target.arm_rad)),
                               actual_fk_position_m=actual_position.tolist(),
                               actual_fk_rotation_matrix=actual_rotation.tolist(),
                               hand_target=list(captured.hand_openings),
                               actual_hand_openings=list(final['hand'].positions))
            for name, origin, actual, expected in zip(
                    ARM_NAMES, captured.arm_rad, final['arm'].positions, target.arm_rad):
                print(f'{name}: 实际增量{math.degrees(actual-origin):+.3f}°，'
                      f'原IK目标{math.degrees(expected):+.3f}°，实测{math.degrees(actual):+.3f}°，'
                      f'相对原IK误差{math.degrees(actual-expected):+.3f}°', flush=True)
            print(f'反馈FK位置/m={actual_position.tolist()}；已停止发布，不自动返程或切换模式。', flush=True)
            print('到位相对最终发送目标采用既有±3°容差；不代表完成了精确的末端位移。停止发布不等于急停。', flush=True)
        else:
            print('只读计划完成，未创建运动发布器；显式--execute才执行一次单程。', flush=True)
        exit_code, reason = 0, 'completed'
    except KeyboardInterrupt:
        exit_code, reason = 130, 'interrupted'
        if port is not None:
            port.stop()
        print('用户中断；停止发布，不返程、不切换模式。', file=sys.stderr)
    except Exception as error:
        if port is not None:
            port.stop()
        if diagnostics is not None:
            diagnostics.record('error', error_type=type(error).__name__, message=str(error))
        print(f'失败：{error}', file=sys.stderr)
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
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
