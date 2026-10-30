"""单臂和双臂共用的原生上肢执行器；IK在发布前完成。"""

from dataclasses import asdict
import time
import math

from .dual_arm_control_core import ARM_POSITION_TOLERANCE, ControlFailure, check_arm, interpolate
from .upper_body_ros import UpperBodyFeedbackPort, build_upper_body_command
from .upper_body_control_core import validate_target


class UpperBodyExecutor:
    """复用原始反馈与既有执行约定；按统一时间轴执行完整目标，不求解周期IK。"""

    def __init__(self, rclpy, node, options, diagnostics):
        """创建只读反馈适配，execute时持续监视URS；发布器延迟到enable_commands创建。"""
        self.rclpy, self.node, self.options, self.diagnostics = rclpy, node, options, diagnostics
        self.feedback = UpperBodyFeedbackPort(
            node, diagnostics, require_urs=options.execute,
            feedback_timeout=options.feedback_timeout, compensation=options.compensation)
        self.publisher = self.timer = self.command_target = None
        self.trajectory = self.began = None
        self.compensation = None
        self.compensation_details = None
        self.last_sent = None
        self.last_sent_compensation = None
        self.captured = None
        self.sequence = 0
        self.last_publish = self.last_elapsed = None
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
        """按50Hz统一取两侧轨迹并用最新反馈计算补偿，校验后发布完整目标；间隔上限80ms。"""
        if self.command_target is None:
            return
        values = self.feedback.checked()
        now = time.monotonic()
        gap = None if self.last_publish is None else now - self.last_publish
        if gap is not None and gap > .08:
            raise ControlFailure(f'命令发布间隔{gap:.3f}秒超过0.08秒')
        ideal = self.trajectory.at(now - self.began)
        target = ideal
        self.compensation_details = None
        if self.compensation is not None:
            blend = interpolate((0.,), (1.,), now - self.began, self.options.duration)[0]
            target, self.compensation_details = self.compensation.apply(
                self.captured, ideal, values, blend)
        self.command_target = target
        validate_target(self.captured, target, self.options)
        check_arm(values['arm'].positions, target.arm_rad)
        # 双侧计算完成后再次校验时效，并以实际发送时刻检查既有间隔。
        self.feedback.checked()
        msg = build_upper_body_command(self.node, target, self.sequence % (2 ** 32))
        sent_at = time.monotonic()
        gap = None if self.last_publish is None else sent_at - self.last_publish
        if gap is not None and gap > .08:
            raise ControlFailure(f'命令发布间隔{gap:.3f}秒超过0.08秒')
        self.publisher.publish(msg)
        self.last_sent = target
        self.last_sent_compensation = self.compensation_details
        self.last_publish = sent_at
        self.last_elapsed = now - self.began
        self.sequence += 1
        self.diagnostics.record('command_published', sequence=msg.header.sequence,
                                arm_rad=list(target.arm_rad), hand_openings=list(target.hand_openings),
                                command_stamp_s=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                                publish_gap_s=gap, compensation=self.compensation_details,
                                ideal_arm_rad=list(ideal.arm_rad),
                                elapsed_motion_s=now-self.began,
                                calculation_seconds=sent_at-now)

    def execute(self, captured, target, trajectory):
        """执行一次完整轨迹，终点新反馈在既有3度容差内持续0.3秒；额外最多2秒，不以夹爪状态判定到位。

        trajectory提供同一时间网格的完整目标；夹爪仅更新开度并持续发送。
        """
        self.enable_commands(captured)
        first = self.feedback.checked()
        began = time.monotonic()
        self.began, self.trajectory = began, trajectory
        arrival = began + self.options.duration
        stable_since = None
        self.command_target = trajectory.at(0.)
        self.diagnostics.record('motion_begin', origin_rad=list(captured.arm_rad),
                                target_rad=list(target.arm_rad), duration_s=self.options.duration,
                                tracking_tolerance_deg=math.degrees(ARM_POSITION_TOLERANCE),
                                settling_timeout_s=2., endpoint_stable_s=.3,
                                interpolation=trajectory.kind,
                                ideal_cartesian_straight_line=trajectory.kind == 'line',
                                compensation_enabled=self.compensation is not None,
                                began_monotonic_s=began)
        while True:
            values = self.spin()
            now = time.monotonic()
            expected = self.last_sent if self.last_sent is not None else captured
            check_arm(values['arm'].positions, expected.arm_rad)
            if now >= arrival:
                newer = (self.last_sent is not None and self.last_elapsed >= self.options.duration
                         and self.last_sent.hand_openings == target.hand_openings) and all(
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
                                            hand_target=list(target.hand_openings),
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
