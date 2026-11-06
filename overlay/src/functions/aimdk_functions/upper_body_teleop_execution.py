"""遥操原生上肢发布适配：独立ROS线程处理反馈与50Hz定时器。"""

from dataclasses import replace
import math
import threading
import time

from .dual_arm_control_core import ControlFailure, interpolate
from .upper_body_cartesian_core import CapturedTarget
from .upper_body_execution import ARM_TRACKING_TOLERANCE, UpperBodyExecutor, check_publish_gap
from .upper_body_ros import build_upper_body_command


def approach_joints(previous, goal, elapsed, speed_rad_s):
    """按共同进度向完整goal靠近，返回目标及进度；每轴步长不超过速度乘elapsed。

    previous为上一成功发布的理想目标；夹爪使用goal开度，不施加关节速度限制。
    """
    delta = tuple(b-a for a, b in zip(previous.arm_rad, goal.arm_rad))
    largest = max(abs(value) for value in delta)
    scale = 1. if largest == 0. else min(1., speed_rad_s * max(0., elapsed) / largest)
    joints = (goal.arm_rad if scale == 1. else tuple(
        a + scale * change for a, change in zip(previous.arm_rad, delta)))
    return CapturedTarget(joints, goal.hand_openings), scale


class UpperBodyTeleopExecutor(UpperBodyExecutor):
    """复用原生执行接口，首次Grip前不发布，接管后读取完整已接受目标快照。"""

    def __init__(self, rclpy, node, options, diagnostics):
        """初始化发布状态和输入时间；ROS线程在模式查询及初始采样完成后启动。"""
        super().__init__(rclpy, node, options, diagnostics)
        self.command_lock = threading.RLock()
        self.online_store = None
        self.background_error = None
        self.ros_executor = self.ros_thread = None
        self.input_received = self.source_advanced = self.last_source = None
        self.input_receiver = None
        self.compensation_started = {}
        self.last_ideal = self.last_ideal_time = self.held_target = None
        self.motion_paused = False

    def hold_current(self):
        """在共同锁内取消未完成推进，保持上一成功发布的完整理想目标；接管前返回None。"""
        with self.command_lock:
            if self.last_ideal is None:
                return None
            self.motion_paused = True
            self.held_target = self.last_ideal
            return self.held_target

    def accept_online(self, value, edge):
        """IK联合提交后更新推进状态；Grip边沿保持候选，普通可解帧恢复渐进。

        边沿候选已从发布目标重建；首次接管的新侧按既有契约使用新鲜HAL命令。
        """
        with self.command_lock:
            self.motion_paused = edge
            if edge:
                self.held_target = value.target

    def approach_target(self, value, now):
        """从上一成功发布的理想目标推进，返回快照及诊断；补偿在此后计算。

        elapsed从上一publish调用结束计至本次计算开始，保守使用实际发布间隔。
        Grip边沿和局部无解暂停时，返回冻结目标；首帧保留HAL零增量。
        """
        elapsed = None if self.last_ideal_time is None else max(0., now-self.last_ideal_time)
        speed = math.radians(self.options.joint_speed_deg_s)
        if self.last_ideal is None:
            ideal, scale = value.target, 1.
        elif self.motion_paused:
            ideal, scale = self.held_target, 0.
        else:
            ideal, scale = approach_joints(self.last_ideal, value.target, elapsed, speed)
        report = dict(speed_rad_s=speed, elapsed_s=elapsed, scale=scale,
                      paused=self.motion_paused,
                      remaining_rad=[b-a for a, b in zip(ideal.arm_rad, value.target.arm_rad)])
        return replace(value, target=ideal), report

    def start_background(self):
        """让独立单线程ROS执行器处理反馈及发布，IK主线程不再调用spin_once。"""
        from rclpy.executors import SingleThreadedExecutor

        self.ros_executor = SingleThreadedExecutor(context=self.node.context)
        self.ros_executor.add_node(self.node)
        self.ros_thread = threading.Thread(target=self._spin_background, name='teleop_ros')
        self.ros_thread.start()

    def _spin_background(self):
        """运行ROS回调，锁存首次异常并立即停止发布，由主线程记录和收尾。"""
        try:
            self.ros_executor.spin()
        except Exception as error:
            if not self.stopped:
                self.background_error = error
                self.stop()

    def check_background(self):
        """将ROS线程异常传回主线程，供输入循环及提交前检查。"""
        if self.background_error is not None:
            raise self.background_error

    def spin(self, ready=True):
        """后台启动前复用采样spin；启动后仅等待回调推进，避免并发spin同一节点。"""
        if self.ros_thread is None:
            return super().spin(ready)
        self.check_background()
        time.sleep(.01)
        return self.feedback.checked() if ready else None

    def observe_input(self, frame):
        """保存已选中处理帧的原始接收时刻，不以接收线程的新帧掩盖主线程阻塞。"""
        with self.command_lock:
            self.input_received = frame.received_monotonic_s
            if frame.source_time_ns is not None:
                if self.last_source is not None and frame.source_time_ns < self.last_source:
                    raise ControlFailure('XR源时间回退')
                if self.last_source is None or frame.source_time_ns > self.last_source:
                    self.source_advanced = frame.received_monotonic_s
                self.last_source = frame.source_time_ns

    def begin_online(self, store, frame):
        """首次Grip安装已从新鲜HAL命令初始化的目标缓存，并立即发送零增量首帧。"""
        with self.command_lock:
            self.observe_input(frame)
            self.online_store = store
            self.diagnostics.record('teleop_execution_begin', input_seq=frame.seq,
                                    arm_rad=list(store.get().target.arm_rad),
                                    input_timeout_s=self.options.input_timeout,
                                    tracking_policy='observation_only',
                                    tracking_observation_limit_rad=ARM_TRACKING_TOLERANCE,
                                    joint_speed_deg_s=self.options.joint_speed_deg_s,
                                    approach_policy='common_joint_progress_before_compensation',
                                    compensation_enabled=self.compensation is not None)
            self.publish()

    def check_input_timeout(self, now):
        """分别检查持续接收与主线程处理年龄；沿用input-timeout，失效时停发。"""
        if self.input_receiver is not None:
            self.input_receiver.check(now)
        timeout = self.options.input_timeout
        if now - self.input_received > timeout:
            raise ControlFailure(f'XR输入接收／处理年龄超过{timeout:g}秒，停止发布')
        if self.last_source is not None and now - self.source_advanced > timeout:
            raise ControlFailure(f'XR源时间停滞超过{timeout:g}秒，停止发布')

    def compensate_target(self, value, values, now):
        """对已接管侧计算实时补偿；各侧从零引入，Grip重按保持进度，返回发送目标及日志。

        value为不可变理想目标快照；补偿结果不写回IK缓存，尚未接管侧保持原目标。
        """
        ideal = value.target
        if self.compensation is None:
            return ideal, None
        positions, reports = list(ideal.arm_rad), {}
        for side in value.compensation_sides:
            start = self.compensation_started.setdefault(side, now)
            blend = interpolate((0.,), (1.,), now-start, self.options.compensation_duration)[0]
            target, report = self.compensation.calculators[side].apply(self.captured, ideal, values, blend)
            offset = 0 if side == 'left' else 7
            positions[offset:offset+7] = target.arm_rad[offset:offset+7]
            reports[side] = report
        target = CapturedTarget(tuple(positions), ideal.hand_openings)
        return target, dict(arms=reports, ideal_arm_rad=list(ideal.arm_rad), sent_arm_rad=list(target.arm_rad),
                            engagement='quintic_per_side_first_grip_no_reset_on_regrip')

    def publish(self):
        """按50Hz发送完整目标；跟踪误差只观测，保留反馈、输入时限及80ms间隔检查。"""
        with self.command_lock:
            if self.online_store is None or self.stopped:
                return
            try:
                now = time.monotonic()
                self.check_input_timeout(now)
                value = self.online_store.get()
                goal = value.target
                value, approach = self.approach_target(value, now)
                ideal = value.target
                for index, side in enumerate(('left', 'right')):
                    self.captured.for_arm(side, goal.arm_rad[index*7:index*7+7])
                    self.captured.for_arm(side, ideal.arm_rad[index*7:index*7+7])
                values = self.feedback.checked()
                target, details = self.compensate_target(value, values, now)
                actual = values['arm'].positions
                previous_target, previous_sequence = self.last_sent, (self.sequence - 1) % (2 ** 32)
                tracking_error = [a-b for a, b in zip(actual, target.arm_rad)]
                previous_error = (None if previous_target is None else
                                  [a-b for a, b in zip(actual, previous_target.arm_rad)])
                msg = build_upper_body_command(self.node, target, self.sequence % (2 ** 32))
                sent_at = time.monotonic()
                self.check_input_timeout(sent_at)
                gap = check_publish_gap(self.last_publish, sent_at)
                publish_begin = time.monotonic()
                self.publisher.publish(msg)
                publish_end = time.monotonic()
                self.last_sent, self.last_publish = target, sent_at
                self.last_ideal, self.last_ideal_time = ideal, publish_end
                self.last_sent_compensation = details
                self.sequence += 1
                self.diagnostics.record(
                    'command_published', sequence=msg.header.sequence, input_seq=value.input_seq,
                    accepted_monotonic_s=value.accepted_monotonic_s,
                    arm_rad=list(target.arm_rad), ideal_arm_rad=list(ideal.arm_rad),
                    ik_goal_arm_rad=list(goal.arm_rad), joint_approach=approach,
                    hand_openings=list(target.hand_openings), compensation=details,
                    command_stamp_s=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    publish_begin_monotonic_s=publish_begin,
                    publish_end_monotonic_s=publish_end,
                    arm_feedback_serial=values['arm'].serial,
                    arm_feedback_stamp_s=values['arm'].stamp,
                    arm_feedback_received_monotonic_s=values['arm'].received,
                    publish_gap_s=gap, input_age_s=sent_at-self.input_received,
                    latest_receive_age_s=(sent_at-self.input_receiver.last_received
                                          if self.input_receiver is not None else None),
                    calculation_seconds=sent_at-now,
                    actual_arm_rad=list(actual),
                    tracking_error_rad=tracking_error,
                    tracking_exceeds_5deg_observation=any(
                        abs(error) > ARM_TRACKING_TOLERANCE for error in tracking_error),
                    previous_sent_arm_rad=(list(previous_target.arm_rad)
                                           if previous_target is not None else None),
                    previous_command_sequence=(previous_sequence if previous_target is not None else None),
                    previous_target_tracking_error_rad=previous_error,
                    previous_target_tracking_exceeds_5deg_observation=(
                        any(abs(error) > ARM_TRACKING_TOLERANCE for error in previous_error)
                        if previous_error is not None else None))
            except Exception as error:
                self.background_error = error
                self.stop()
                raise

    def stop(self):
        """在发布锁内撤销在线目标并取消定时器，后续缓存提交不会恢复发布。"""
        with self.command_lock:
            self.online_store = None
            super().stop()

    def close(self):
        """先停发并结束ROS线程，再销毁发布器及反馈订阅；不自动恢复控制。"""
        self.stop()
        if self.ros_executor is not None:
            self.ros_executor.shutdown()
            self.ros_thread.join()
            self.ros_executor.remove_node(self.node)
        super().close()
