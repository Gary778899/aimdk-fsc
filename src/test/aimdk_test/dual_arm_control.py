"""已处于 URS 时执行双肩 pitch 抬升 5 度、开闭夹爪及返回；默认只读。"""

import argparse
import math
import sys
import time

from aimdk_test.arm_test import BaselinePort
from aimdk_test.arm_test_core import ARM_NAMES, URS, require_static_hands
from aimdk_test.dual_arm_control_core import (
    ARM_POSITION_TOLERANCE, SHOULDER_UP_SIGNS, check_arm, decode_urs_mc, interpolate, shoulder_target,
)
from aimdk_test.head_test_core import FeedbackMonitor, TestFailure
from aimdk_test.head_test_diagnostics import DiagnosticLog


def parser(probe=False, close_gripper=False):
    """创建选项；probe 配置单肩验证，close_gripper 配置独立夹爪开度命令。"""
    description = ('保持双臂实测位置并控制指定夹爪开度；默认只读，要求预先进入 URS。'
                   if close_gripper else '单侧肩 pitch 小幅往返，观察正负方向；默认只读，不开闭夹爪。'
                   if probe else __doc__)
    result = argparse.ArgumentParser(description=description)
    result.add_argument('--execute', action='store_true', help='显式执行动作；现场确认空夹爪及运动空间')
    if not close_gripper:
        result.add_argument('--leg-seconds', type=float, default=5., help='单程运动时间，5..10 秒，默认 5')
    result.add_argument('--timeout', type=float, default=5., help='启动/服务/夹爪到位超时，1..10 秒')
    result.add_argument('--mc-feedback-timeout', type=float, default=.5,
                        help='MC 状态接收间隔/源时间停滞/源年龄上限，0.2..1 秒，默认 0.5')
    result.add_argument('--arm-feedback-timeout', type=float, default=.5,
                        help='观察和执行阶段的手臂反馈时限，0.2..1 秒，默认 0.5')
    result.add_argument('--hand-feedback-timeout', type=float, default=.5,
                        help='观察和执行阶段的夹爪反馈时限，0.2..1 秒，默认 0.5')
    result.add_argument('--log-dir', help='默认写入工作区 log/')
    if close_gripper:
        result.add_argument('--side', choices=('left', 'right', 'both'), default='both',
                            help='控制左、右或两侧夹爪，默认 both；未选侧保持实测开度')
        result.add_argument('--opening', type=float, default=0.,
                            help='选定夹爪的目标开度，0..1；0 闭合，1 完全张开，默认 0')
    elif probe:
        result.add_argument('--side', choices=('left', 'right'), required=True, help='只验证指定侧肩 pitch')
        result.add_argument('--delta-deg', type=float, default=1., help='带符号偏移，绝对值 0.5..5 度，默认 1')
    else:
        result.add_argument('--left-up-sign', type=int, choices=(-1, 1), default=SHOULDER_UP_SIGNS[0],
                            help='左肩上抬方向，当前机器人实测默认 -1')
        result.add_argument('--right-up-sign', type=int, choices=(-1, 1), default=SHOULDER_UP_SIGNS[1],
                            help='右肩上抬方向，当前机器人实测默认 -1')
    return result


def validate_options(options, probe, close_gripper=False):
    """检查参数并返回左右肩 rad 偏移；夹爪控制返回零偏移，无效参数抛出 TestFailure。"""
    if not math.isfinite(options.timeout) or not 1 <= options.timeout <= 10:
        raise TestFailure('--timeout 必须为 1..10 秒')
    if not close_gripper and (not math.isfinite(options.leg_seconds) or not 5 <= options.leg_seconds <= 10):
        raise TestFailure('--leg-seconds 必须为 5..10 秒')
    for name in ('mc', 'arm', 'hand'):
        limit = getattr(options, f'{name}_feedback_timeout')
        if not math.isfinite(limit) or not .2 <= limit <= 1.:
            raise TestFailure(f'--{name}-feedback-timeout 必须为 0.2..1 秒')
    if close_gripper:
        if not math.isfinite(options.opening) or not 0. <= options.opening <= 1.:
            raise TestFailure('--opening 必须为 0..1 的有限数值（0 闭合，1 完全张开）')
        return (0., 0.)
    if probe:
        if not math.isfinite(options.delta_deg) or not .5 <= abs(options.delta_deg) <= 5:
            raise TestFailure('--delta-deg 的绝对值必须为 0.5..5 度')
        delta = math.radians(options.delta_deg)
        return (delta, 0.) if options.side == 'left' else (0., delta)
    return tuple(math.radians(5) * sign
                 for sign in (options.left_up_sign, options.right_up_sign))


def accept_mode(response, diagnostics):
    """校验启动时的 GetMcAction 响应；非 URS/RUNNING 时提示用户并失败。"""
    if response is None:
        raise TestFailure('get_mc_action 未返回响应')
    action, status = response.info.action_desc, response.info.status.value
    diagnostics.record('mode_response', code=response.header.code, action=action, status=status)
    if response.header.code != 0:
        raise TestFailure(f'get_mc_action 查询失败，code={response.header.code}')
    if action != URS or status != 100:
        raise TestFailure(f'当前模式为 {action} / status={status}；'
                          '请先手动切换至 URS（UPPERBODY_REMOTE_SPLIT），等待 RUNNING 后重新运行')


def confirm_initial_mode(rclpy, node, options, diagnostics, client):
    """用给定客户端在启动时确认 URS；请求每 0.25 秒有界重试。

    服务发现和响应各有 options.timeout 秒预算。非 URS、错误响应或 ROS
    停止时直接抛出 TestFailure，不将其视为可重试的响应超时。
    """
    from aimdk_msgs.srv import GetMcAction
    future = None
    began = time.monotonic()
    discovery_deadline = began + options.timeout
    try:
        while not client.service_is_ready():
            if not rclpy.ok():
                raise TestFailure('ROS 已停止，无法查询 get_mc_action')
            if time.monotonic() >= discovery_deadline:
                raise TestFailure('get_mc_action 服务发现超时')
            rclpy.spin_once(node, timeout_sec=.01)
        diagnostics.record('mode_service_ready', discovery_seconds=time.monotonic() - began)
        deadline = time.monotonic() + options.timeout
        attempt = 0
        while time.monotonic() < deadline:
            if not rclpy.ok():
                raise TestFailure('ROS 已停止，无法查询 get_mc_action')
            attempt += 1
            sent = time.monotonic()
            request = GetMcAction.Request()
            request.request.header.stamp = node.get_clock().now().to_msg()
            future = client.call_async(request)
            diagnostics.record('mode_query_sent', phase='startup', attempt=attempt)
            attempt_deadline = min(deadline, sent + .25)
            while not future.done() and time.monotonic() < attempt_deadline:
                if not rclpy.ok():
                    raise TestFailure('ROS 已停止，无法等待 get_mc_action 响应')
                rclpy.spin_once(node, timeout_sec=.01)
            if not rclpy.ok():
                raise TestFailure('ROS 已停止，无法确认 get_mc_action 响应')
            if future.done() and time.monotonic() < attempt_deadline:
                diagnostics.record('mode_query_received', phase='startup', attempt=attempt,
                                    response_seconds=time.monotonic() - sent)
                return accept_mode(future.result(), diagnostics)
            # 仅重试只读查询；移除旧请求，迟到响应不能确认后续的模式归属。
            client.remove_pending_request(future)
            future = None
            diagnostics.record('mode_query_retry', phase='startup', attempt=attempt,
                                response_wait_seconds=time.monotonic() - sent)
        raise TestFailure(f'get_mc_action 响应超时：{options.timeout:g} 秒内已发送 {attempt} 次查询')
    finally:
        if future is not None and not future.done():
            client.remove_pending_request(future)


class DualArmPort(BaselinePort):
    """复用原测试反馈与故障锁存，单独实现允许夹爪开闭的 URS 执行流程。"""

    def __init__(self, rclpy, node, options, diagnostics, get_client):
        """初始化反馈订阅；启动查询客户端保留但不轮询，发布器延迟到显式执行时创建。"""
        super().__init__(rclpy, node, options, diagnostics, get_client=get_client)
        self.streams['mc'].decoder = decode_urs_mc
        # 按流配置观察和执行共用的时限；此时尚未处理任何订阅回调。
        for name, stream in self.streams.items():
            stream.monitor = FeedbackMonitor(feedback_timeout=getattr(options, f'{name}_feedback_timeout'))
        self.publisher = None
        self.command_timer = None
        self.command_target = None
        self.sequence = 0
        self.last_publish = None

    def tick(self, ready=True):
        """处理反馈回调；ready=True 时校验时效，模式变化由 MC 解码器锁存并中止。"""
        self.spin()
        return self.checked() if ready else None

    def capture(self):
        """等齐推进中的反馈，连续观察 3 秒，记录全部双臂起点与夹爪开度。"""
        deadline = time.monotonic() + self.options.timeout
        while not all(s.monitor.latest is not None and s.monitor.latest.stamp > s.first_stamp
                      for s in self.streams.values()):
            if time.monotonic() >= deadline:
                raise TestFailure('等待双臂、夹爪、MC 的源时间推进超时')
            self.tick(ready=False)
        print('URS / RUNNING 已确认；等待 3 秒后记录双臂起点。', flush=True)
        deadline = time.monotonic() + 3.
        while time.monotonic() < deadline:
            self.tick()
        values = self.tick()
        require_static_hands(values['hand'])
        self.diagnostics.record('start_captured', arm_names=list(ARM_NAMES),
                                arm_rad=list(values['arm'].positions),
                                hand_openings=list(values['hand'].positions))
        for name, position in zip(ARM_NAMES, values['arm'].positions):
            print(f'{name}: {position:.6f} rad ({math.degrees(position):+.3f} 度)')
        return values['arm'].positions, values['hand'].positions

    def enable_commands(self):
        """创建 publisher 和定时器，有界等待 DDS 订阅匹配后才允许设置目标。"""
        from aimdk_msgs.msg import UpperBodyCommandArray
        self.message_type = UpperBodyCommandArray
        self.publisher = self.node.create_publisher(UpperBodyCommandArray, '/mc/upper_body_command', 10)
        self.command_timer = self.node.create_timer(.02, self.publish)
        self.diagnostics.record('command_transport', timer_period_s=.02, qos='rclpy_default_depth_10',
                                stamp='current_ros_time', reference='py_examples/upper_body_control.py')
        deadline = time.monotonic() + self.options.timeout
        while self.publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise TestFailure('上肢命令话题未匹配到 DDS 订阅者；请核查原生 MC 订阅及 QoS')
            self.tick()
        self.diagnostics.record('command_subscriber_matched',
                                subscription_count=self.publisher.get_subscription_count())
        print('上肢命令话题已匹配 DDS 订阅者；这只确认连接，不代表固件已接受目标。', flush=True)

    def set_target(self, positions, hands):
        """保存经校验的完整 rad 目标及左右开度，供定时器发布，不直接发送消息。"""
        check_arm(positions, positions)
        if len(hands) != 2 or not all(math.isfinite(v) and 0 <= v <= 1 for v in hands):
            raise TestFailure('夹爪目标必须是左右两个 0..1 开度')
        self.command_target = (tuple(positions), tuple(hands))

    def publish(self):
        """20 ms 定时回调：检查新鲜反馈及 URS 后，以当前 ROS 时间发送最新目标。

        没有目标时不发布；MC 模式变化、反馈失效或发布间隔超过 80 ms 时抛出 TestFailure。
        """
        if self.command_target is None:
            return
        values = self.checked()
        now = time.monotonic()
        if self.publisher is None:
            raise TestFailure('缺少上肢命令发布器')
        gap = now - self.last_publish if self.last_publish is not None else None
        if gap is not None and gap > .08:
            raise TestFailure(f'命令发布间隔 {gap:.3f} 秒超过 0.08 秒')
        positions, hands = self.command_target
        check_arm(values['arm'].positions, positions)
        msg = self.message_type()
        # 与官方上肢示例一致，仅为命令填写当前 ROS 时间；反馈源时间不变。
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'mc_upper_body'
        msg.header.sequence = self.sequence
        msg.source = 'dual_arm_control'
        msg.head_pos = [0., 0.]  # SDK 上肢专章说明 URS 下头部目标无效。
        msg.arm_pos = list(positions)
        msg.hand_sub_mode = self.message_type.HAND_CLAW_OPEN_CLOSE
        msg.hand_pos = list(hands)
        self.publisher.publish(msg)
        self.last_publish = now
        self.sequence += 1
        self.diagnostics.record('command_published', sequence=msg.header.sequence,
                                arm_rad=list(positions), hand_openings=list(hands),
                                command_stamp_s=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                                publish_gap_s=gap, subscription_count=self.publisher.get_subscription_count())

    def segment(self, origin, target, hands, duration, arm_tolerance, hand_motion=False, phase='motion'):
        """持续发送五次插值目标并检查跟踪；终点需新源反馈连续稳定 0.3 秒。

        hand_motion 允许夹爪移动，仍拒绝异常状态及硬件故障；到位超时则失败。
        phase 为日志和错误中的阶段名称。位置参数单位为 rad，duration 单位为秒。
        arm_tolerance 仅约束本阶段改变目标的关节，其余关节使用 3 度保持容差。
        """
        began = time.monotonic()
        arrival = began + duration
        since = None
        first = self.checked()
        serials = tuple(first[name].serial for name in ('arm', 'hand'))
        stamps = tuple(first[name].stamp for name in ('arm', 'hand'))
        phase_published_at = None
        previous_sequence = self.sequence
        settling_timeout = self.options.timeout if hand_motion else 2.
        # 仅移动关节使用本阶段终点容差，不给其余保持关节施加精密定位门槛。
        tolerances = tuple(arm_tolerance if abs(a-b) > 1e-9 else ARM_POSITION_TOLERANCE
                           for a, b in zip(origin, target))
        tolerances_deg = [math.degrees(value) for value in tolerances]
        self.diagnostics.record('segment_begin', phase=phase, duration_s=duration,
                                settling_timeout_s=settling_timeout, origin_rad=list(origin),
                                target_rad=list(target), arm_tolerance_deg=math.degrees(arm_tolerance),
                                arm_tolerances_deg=tolerances_deg,
                                hand_target=list(hands), first_sequence=previous_sequence)
        print(f'阶段 {phase}：轨迹 {duration:g} 秒，移动关节容差 ±{math.degrees(arm_tolerance):g}°，'
              f'保持关节容差 ±{math.degrees(ARM_POSITION_TOLERANCE):g}°，'
              f'终点最多等待 {settling_timeout:g} 秒。', flush=True)
        # 进入本阶段后先更新目标，避免下一次定时回调发送上个阶段的手爪目标。
        self.set_target(origin if duration else target, hands)
        while True:
            values = self.tick()
            now = time.monotonic()
            commanded = interpolate(origin, target, now - began, duration) if duration else target
            check_arm(values['arm'].positions, commanded)
            if any(h['state'] not in (0, 1) for h in values['hand'].data['hands']):
                raise TestFailure('夹爪出现非空闲/移动状态，停止动作')
            if not hand_motion:
                require_static_hands(values['hand'])
                if any(abs(a-b) > .02 for a, b in zip(values['hand'].positions, hands)):
                    raise TestFailure('夹爪保持开度误差超过 0.02')
            self.set_target(commanded, hands)
            if phase_published_at is None and self.sequence > previous_sequence:
                phase_published_at = self.last_publish
            if now >= arrival:
                reached = (all(abs(a-b) <= tolerance
                               for a, b, tolerance in zip(values['arm'].positions, target, tolerances))
                           and all(abs(a-b) <= .02 for a, b in zip(values['hand'].positions, hands))
                           and all(h['stationary_candidate'] for h in values['hand'].data['hands']))
                newer = phase_published_at is not None and all(
                    values[name].serial > serial and values[name].stamp > stamp
                    and values[name].received > max(arrival, phase_published_at)
                    for name, serial, stamp in zip(('arm', 'hand'), serials, stamps))
                since = (since if since is not None else now) if reached and newer else None
                if since is not None and now - since >= .3:
                    self.diagnostics.record('endpoint_confirmed', phase=phase,
                                            arm_rad=list(values['arm'].positions),
                                            target_rad=list(target), hand_openings=list(values['hand'].positions),
                                            arm_tolerance_deg=math.degrees(arm_tolerance),
                                            arm_tolerances_deg=tolerances_deg,
                                            published_count=self.sequence-previous_sequence)
                    return
                if now > arrival + settling_timeout:
                    errors = [math.degrees(a-b) for a, b in zip(values['arm'].positions, target)]
                    joints = [f'{name}={error:+.3f}°（上限 ±{limit:g}°）'
                              for name, error, limit in zip(ARM_NAMES, errors, tolerances_deg)
                              if abs(error) > limit]
                    hand_states = [dict(side=h['side'], opening=h['position'], target=expected,
                                        state=h['state'], velocity=h['velocity'],
                                        stationary=h['stationary_candidate'])
                                   for h, expected in zip(values['hand'].data['hands'], hands)]
                    self.diagnostics.record('endpoint_timeout', phase=phase, arm_error_deg=errors,
                                            arm_tolerance_deg=math.degrees(arm_tolerance),
                                            arm_tolerances_deg=tolerances_deg,
                                            hand_states=hand_states, fresh_after_phase_publish=newer,
                                            stable_seconds=now-since if since is not None else 0.,
                                            published_count=self.sequence-previous_sequence,
                                            first_publish_monotonic_s=phase_published_at,
                                            subscription_count=self.publisher.get_subscription_count())
                    hand_text = '; '.join(f"{h['side']} 开度={h['opening']:.4f}/{h['target']:.4f}, "
                                          f"state={h['state']}, velocity={h['velocity']:.4f}"
                                          for h in hand_states)
                    raise TestFailure(f'阶段 {phase} 未稳定到位：关节超差 [{", ".join(joints) or "无"}]；'
                                      f'夹爪 [{hand_text}]；新源反馈={newer}；'
                                      f'本阶段已发布 {self.sequence-previous_sequence} 帧；'
                                      f'DDS 订阅匹配数={self.publisher.get_subscription_count()}')

    def control_grippers(self, start, hands, side, opening):
        """保持完整双臂起点，控制 side 指定夹爪开度；未选侧重发实测开度。

        side 为 left、right 或 both，opening 为已校验的 0..1 目标开度，0 闭合、1 全开。
        复用阶段执行器，不到位、堵转、故障或反馈失效时中止；不切换模式。
        """
        targets = tuple(opening if side in (name, 'both') else current
                        for name, current in zip(('left', 'right'), hands))
        self.diagnostics.record('gripper_control_plan', side=side, opening=opening, arm_hold_rad=list(start),
                                initial_openings=list(hands), target_openings=list(targets))
        print(f'控制夹爪：{side}；当前开度={list(hands)}，目标开度={list(targets)}；'
              f'双臂保持实测起点，最多 {self.options.timeout:g} 秒持续发送并等待开度到位。', flush=True)
        self.enable_commands()
        self.segment(start, start, targets, 0., ARM_POSITION_TOLERANCE,
                     hand_motion=True, phase='gripper_control')
        print('完成：指定夹爪开度到位，双臂位置误差符合 ±3°；模式保持 URS。', flush=True)

    def execute(self, start, hands, offsets, probe):
        """执行一次往返；正式流程在抬升到位后开爪，保持 3 秒，再闭爪返回。"""
        peak = shoulder_target(start, offsets)
        # 执行沿用观察阶段的反馈时限，记录监视器的实际配置。
        self.diagnostics.record('execution_feedback_limits',
                                arm_timeout_s=self.streams['arm'].monitor.feedback_timeout,
                                hand_timeout_s=self.streams['hand'].monitor.feedback_timeout,
                                mc_timeout_s=self.streams['mc'].monitor.feedback_timeout)
        self.enable_commands()
        # 直接从实测起点开始平滑轨迹，不增加起点精密到位确认阶段。
        print(f'肩 pitch 偏移（左、右）：{[math.degrees(v) for v in offsets]} 度', flush=True)
        # 双臂各阶段共用 3 度容差；方向验证另限制为偏移的一半，保证能观察到运动。
        outbound_tolerance = (min(ARM_POSITION_TOLERANCE, max(abs(value) for value in offsets) / 2)
                              if probe else ARM_POSITION_TOLERANCE)
        self.diagnostics.record('execution_position_limits',
                                tracking_holding_return_tolerance_deg=math.degrees(ARM_POSITION_TOLERANCE),
                                outbound_tolerance_deg=math.degrees(outbound_tolerance),
                                endpoint_stable_seconds=.3)
        self.segment(start, peak, hands, self.options.leg_seconds, outbound_tolerance, phase='raise')
        if probe:
            self.segment(peak, peak, hands, 3., ARM_POSITION_TOLERANCE, phase='raised_hold')
        else:
            print('双肩到位，打开夹爪；全开到位后保持 3 秒。', flush=True)
            self.segment(peak, peak, (1., 1.), 0., ARM_POSITION_TOLERANCE, hand_motion=True, phase='open')
            self.segment(peak, peak, (1., 1.), 3., ARM_POSITION_TOLERANCE, phase='open_hold')
            print('关闭夹爪，闭合到位后返回双臂初始位置。', flush=True)
            self.segment(peak, peak, (0., 0.), 0., ARM_POSITION_TOLERANCE, hand_motion=True, phase='close')
            hands = (0., 0.)
        self.segment(peak, start, hands, self.options.leg_seconds, ARM_POSITION_TOLERANCE, phase='return')
        print('完成：双臂返回实测初始位置，误差符合要求；模式保持 URS。', flush=True)

    def close(self):
        """取消发布定时器并清除目标；不执行自动回程或模式切换。"""
        self.command_target = None
        if self.command_timer is not None:
            self.command_timer.cancel()
        self.diagnostics.record('commands_stopped', published_count=self.sequence)


def main(args=None, probe=False, close_gripper=False):
    """执行双臂、单肩或独立夹爪流程；默认只读，返回成功 0、失败 1、中断 130。"""
    options = parser(probe, close_gripper).parse_args(args)
    diagnostics = node = port = None
    initialized = False
    try:
        offsets = validate_options(options, probe, close_gripper)
        prefix = 'gripper_close' if close_gripper else 'shoulder_direction' if probe else 'dual_arm'
        diagnostics = DiagnosticLog(options.log_dir, prefix=prefix)
        diagnostics.record('run_start', execute=options.execute, probe=probe, close_gripper=close_gripper,
                           side=getattr(options, 'side', 'both'), offsets_rad=list(offsets),
                           opening=getattr(options, 'opening', None),
                           leg_seconds=getattr(options, 'leg_seconds', None),
                           mc_feedback_timeout_s=options.mc_feedback_timeout,
                           arm_feedback_timeout_s=options.arm_feedback_timeout,
                           hand_feedback_timeout_s=options.hand_feedback_timeout,
                           runtime_mode_source='/aima/mc/common/state')
        print(f'诊断日志：{diagnostics.path}', flush=True)
        import rclpy
        from aimdk_msgs.srv import GetMcAction
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = rclpy.create_node('gripper_close' if close_gripper else
                                 'shoulder_direction_test' if probe else 'dual_arm_control')
        if node.get_parameter('use_sim_time').value:
            raise TestFailure('本脚本仅用于真机，不支持 use_sim_time')
        client = node.create_client(GetMcAction, '/aimdk_5Fmsgs/srv/GetMcAction')
        confirm_initial_mode(rclpy, node, options, diagnostics, client)
        diagnostics.record('runtime_mode_monitor', source='/aima/mc/common/state',
                           required_action=URS, required_status=100)
        port = DualArmPort(rclpy, node, options, diagnostics, client)
        start, hands = port.capture()
        if close_gripper:
            print(f'当前夹爪开度（左、右）：{list(hands)}；'
                  f'选择侧别：{options.side}，目标开度：{options.opening:g}。', flush=True)
        if options.execute:
            if close_gripper:
                port.control_grippers(start, hands, options.side, options.opening)
            else:
                port.execute(start, hands, offsets, probe)
        else:
            print('只读检查完成，未发送运动命令；使用 --execute 显式执行。')
        diagnostics.record('run_pass', read_only=not options.execute)
        return 0
    except KeyboardInterrupt:
        if diagnostics:
            diagnostics.record('run_interrupted')
        print('用户中断；停止发布。请检查机器人状态，停止发布不等于急停。', file=sys.stderr)
        return 130
    except Exception as error:
        if diagnostics:
            diagnostics.record('run_failed', error=str(error), error_type=type(error).__name__)
        print(f'失败：{error}。已停止本脚本命令；请检查机器人状态。', file=sys.stderr)
        return 1
    finally:
        if port is not None:
            port.close()
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        if diagnostics:
            diagnostics.close()


if __name__ == '__main__':
    sys.exit(main())
