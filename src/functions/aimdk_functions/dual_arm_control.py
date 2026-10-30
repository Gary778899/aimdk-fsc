"""已处于 URS 时执行一次关节增量或夹爪开度动作；默认只读，不自动往返。"""
import argparse
import math
import sys
import time

from aimdk_functions.dual_arm_control_core import (
    ARM_NAMES, ARM_POSITION_TOLERANCE, JOINT_TAGS, URS, ControlFailure,
    Stream, FeedbackMonitor, decode_arm, decode_hand, decode_urs_mc,
    check_arm, interpolate, joint_target, gripper_target,
)
from aimdk_functions.dual_arm_control_diagnostics import DiagnosticLog


def diagnostic_value(value):
    """将原始反馈中的非有限数转为可记录字符串；递归保留字典和数组字段。"""
    if isinstance(value, float) and not math.isfinite(value):
        return repr(value)
    if isinstance(value, dict):
        return {key: diagnostic_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [diagnostic_value(item) for item in value]
    return value


def parser():
    """创建单次 joint/gripper 命令参数；所有动作默认只读，--execute 才允许发布。"""
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('action', choices=('joint', 'gripper'), help='关节增量或夹爪开度动作')
    result.add_argument('--side', choices=('left', 'right', 'both'), required=True, help='选择左、右或双侧')
    result.add_argument('--tag', choices=JOINT_TAGS, help='joint 动作必须指定的七关节 tag')
    result.add_argument('--delta-deg', type=float, help='选中侧共用增量，绝对值不超过 10 度')
    result.add_argument('--left-delta-deg', type=float, help='both 时的左侧增量，与共用增量互斥')
    result.add_argument('--right-delta-deg', type=float, help='both 时的右侧增量，与共用增量互斥')
    result.add_argument('--opening', type=float, help='gripper 动作目标开度，0 闭合、1 全开')
    result.add_argument('--duration', type=float, help='joint 单程时间，5..10 秒，默认 5')
    result.add_argument('--execute', action='store_true', help='显式执行；现场确认空爪及动作空间')
    result.add_argument('--timeout', type=float, default=5., help='服务/匹配超时及夹爪目标持续发送时间，1..10 秒，默认 5')
    for name in ('mc', 'arm', 'hand'):
        result.add_argument(f'--{name}-feedback-timeout', type=float, default=.5,
                            help='接收间隔/源停滞/源年龄上限，0.2..1 秒，默认 0.5')
    result.add_argument('--log-dir', help='默认使用功能包所在工作区 log/')
    return result


def validate_options(options):
    """校验动作参数并返回左右 rad 增量（未选侧为 None）；无效或混用参数时抛出 ControlFailure。"""
    if not math.isfinite(options.timeout) or not 1 <= options.timeout <= 10:
        raise ControlFailure('--timeout 必须为 1..10 秒')
    for name in ('mc', 'arm', 'hand'):
        value = getattr(options, f'{name}_feedback_timeout')
        if not math.isfinite(value) or not .2 <= value <= 1.:
            raise ControlFailure(f'--{name}-feedback-timeout 必须为 0.2..1 秒')
    deltas = (options.delta_deg, options.left_delta_deg, options.right_delta_deg)
    if options.action == 'gripper':
        if options.tag is not None or any(v is not None for v in deltas) or options.duration is not None:
            raise ControlFailure('gripper 不接受 --tag、关节增量或 --duration')
        if options.opening is None or not math.isfinite(options.opening) or not 0 <= options.opening <= 1:
            raise ControlFailure('gripper 必须指定 --opening 0..1')
        return (None, None)
    if options.tag is None or options.opening is not None:
        raise ControlFailure('joint 必须指定 --tag，且不能包含 --opening')
    options.duration = 5. if options.duration is None else options.duration
    if not math.isfinite(options.duration) or not 5 <= options.duration <= 10:
        raise ControlFailure('--duration 必须为 5..10 秒')
    common, left, right = deltas
    if common is not None:
        if left is not None or right is not None:
            raise ControlFailure('--delta-deg 与左右独立增量不能同时使用')
        selected = (common if options.side in ('left', 'both') else None,
                    common if options.side in ('right', 'both') else None)
    else:
        if options.side != 'both' or left is None or right is None:
            raise ControlFailure('单侧须指定 --delta-deg；both 须指定共用增量或完整左右独立增量')
        selected = (left, right)
    if any(v is not None and (not math.isfinite(v) or abs(v) > 10) for v in selected):
        raise ControlFailure('单次关节增量必须为有限数值且绝对值不超过 10 度')
    return tuple(None if v is None else math.radians(v) for v in selected)


def accept_mode(response, diagnostics):
    """校验启动时的 GetMcAction 响应；非 URS/RUNNING 时提示用户并失败。"""
    if response is None:
        raise ControlFailure('get_mc_action 未返回响应')
    action, status = response.info.action_desc, response.info.status.value
    diagnostics.record('mode_response', code=response.header.code, action=action, status=status)
    if response.header.code != 0:
        raise ControlFailure(f'get_mc_action 查询失败，code={response.header.code}')
    if action != URS or status != 100:
        raise ControlFailure(f'当前模式为 {action} / status={status}；'
                          '请先手动切换至 URS（UPPERBODY_REMOTE_SPLIT），等待 RUNNING 后重新运行')


def confirm_initial_mode(rclpy, node, options, diagnostics, client):
    """用给定客户端在启动时确认 URS；请求每 0.25 秒有界重试。

    服务发现和响应各有 options.timeout 秒预算。非 URS、错误响应或 ROS
    停止时直接抛出 ControlFailure，不将其视为可重试的响应超时。
    """
    from aimdk_msgs.srv import GetMcAction
    future = None
    began = time.monotonic()
    discovery_deadline = began + options.timeout
    try:
        while not client.service_is_ready():
            if not rclpy.ok():
                raise ControlFailure('ROS 已停止，无法查询 get_mc_action')
            if time.monotonic() >= discovery_deadline:
                raise ControlFailure('get_mc_action 服务发现超时')
            rclpy.spin_once(node, timeout_sec=.01)
        diagnostics.record('mode_service_ready', discovery_seconds=time.monotonic() - began)
        deadline = time.monotonic() + options.timeout
        attempt = 0
        while time.monotonic() < deadline:
            if not rclpy.ok():
                raise ControlFailure('ROS 已停止，无法查询 get_mc_action')
            attempt += 1
            sent = time.monotonic()
            request = GetMcAction.Request()
            request.request.header.stamp = node.get_clock().now().to_msg()
            future = client.call_async(request)
            diagnostics.record('mode_query_sent', phase='startup', attempt=attempt)
            attempt_deadline = min(deadline, sent + .25)
            while not future.done() and time.monotonic() < attempt_deadline:
                if not rclpy.ok():
                    raise ControlFailure('ROS 已停止，无法等待 get_mc_action 响应')
                rclpy.spin_once(node, timeout_sec=.01)
            if not rclpy.ok():
                raise ControlFailure('ROS 已停止，无法确认 get_mc_action 响应')
            if future.done() and time.monotonic() < attempt_deadline:
                diagnostics.record('mode_query_received', phase='startup', attempt=attempt,
                                    response_seconds=time.monotonic() - sent)
                return accept_mode(future.result(), diagnostics)
            # 仅重试只读查询；移除旧请求，迟到响应不能确认后续的模式归属。
            client.remove_pending_request(future)
            future = None
            diagnostics.record('mode_query_retry', phase='startup', attempt=attempt,
                                response_wait_seconds=time.monotonic() - sent)
        raise ControlFailure(f'get_mc_action 响应超时：{options.timeout:g} 秒内已发送 {attempt} 次查询')
    finally:
        if future is not None and not future.done():
            client.remove_pending_request(future)


class DualArmPort:
    """单次动作的 ROS 适配与执行器；不切换模式，不自动返回。"""

    def __init__(self, rclpy, node, options, diagnostics):
        """按 SDK QoS 创建只读订阅与各流监视器；发布器延迟至显式执行时创建。"""
        from aimdk_msgs.msg import JointStateArray, HandStateArray, McCommonState
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        self.rclpy, self.node, self.options = rclpy, node, options
        self.diagnostics = diagnostics
        self.streams = dict(arm=Stream('arm', decode_arm), hand=Stream('hand', decode_hand),
                            mc=Stream('mc', decode_urs_mc))
        for name, stream in self.streams.items():
            stream.monitor = FeedbackMonitor(getattr(options, f'{name}_feedback_timeout'))
        self.publisher = None
        self.command_timer = None
        self.command_target = None
        self.closed = False
        self.sequence = 0
        self.last_publish = None
        self.last_log = dict.fromkeys(self.streams, float('-inf'))
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions = [
            node.create_subscription(JointStateArray, '/aima/hal/joint/arm/state', self.on_arm, qos),
            node.create_subscription(HandStateArray, '/aima/hal/joint/hand/state', self.on_hand, qos),
            node.create_subscription(McCommonState, '/aima/mc/common/state', self.on_mc, qos),
        ]

    def receive(self, name, msg):
        """解码反馈并记录原始字段及时间；错误永久锁存且立即抛出 ControlFailure。"""
        from rosidl_runtime_py.convert import message_to_ordereddict
        now = time.monotonic()
        stream = self.streams[name]
        stream.accept(msg, now)
        if stream.error or now - self.last_log[name] >= .1:
            self.last_log[name] = now
            self.diagnostics.record('feedback', stream=name, received_monotonic_s=now,
                                    ros_time_s=self.node.get_clock().now().nanoseconds / 1e9,
                                    raw=diagnostic_value(message_to_ordereddict(msg)),
                                    error=str(stream.error) if stream.error else None)
        if stream.error:
            raise ControlFailure(f'{name}: {stream.error}')

    def on_arm(self, msg):
        """接收一次完整双臂反馈消息并交给具名流。"""
        self.receive('arm', msg)

    def on_hand(self, msg):
        """接收一次两侧夹爪反馈消息并交给具名流。"""
        self.receive('hand', msg)

    def on_mc(self, msg):
        """接收站立/URS 模式反馈，不解释 FSM 数值。"""
        self.receive('mc', msg)

    def spin(self):
        """处理最多 10 ms 回调并观测各流 ROS 时钟；ROS 停止或已锁存错误时抛出 ControlFailure。"""
        self.rclpy.spin_once(self.node, timeout_sec=.01)
        if not self.rclpy.ok():
            raise ControlFailure('ROS context stopped')
        now, ros_now = time.monotonic(), self.node.get_clock().now().nanoseconds / 1e9
        for stream in self.streams.values():
            stream.monitor.observe_clock(now, ros_now)
            if stream.error:
                raise ControlFailure(f'{stream.name}: {stream.error}')

    def checked(self):
        """返回同时满足各自接收与源时间约束的手臂、夹爪和 MC 样本。"""
        now, ros_now = time.monotonic(), self.node.get_clock().now().nanoseconds / 1e9
        return {name: stream.checked(now, ros_now) for name, stream in self.streams.items()}

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
                raise ControlFailure('等待双臂、夹爪、MC 的源时间推进超时')
            self.tick(ready=False)
        print('URS / RUNNING 已确认；等待 3 秒后记录双臂起点。', flush=True)
        deadline = time.monotonic() + 3.
        while time.monotonic() < deadline:
            self.tick()
        values = self.tick()
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
                raise ControlFailure('上肢命令话题未匹配到 DDS 订阅者；请核查原生 MC 订阅及 QoS')
            self.tick()
        self.diagnostics.record('command_subscriber_matched',
                                subscription_count=self.publisher.get_subscription_count())
        print('上肢命令话题已匹配 DDS 订阅者；这只确认连接，不代表固件已接受目标。', flush=True)

    def set_target(self, positions, hands):
        """保存经校验的完整 rad 目标及左右开度，供定时器发布，不直接发送消息。"""
        check_arm(positions, positions)
        if len(hands) != 2 or not all(math.isfinite(v) and 0 <= v <= 1 for v in hands):
            raise ControlFailure('夹爪目标必须是左右两个 0..1 开度')
        self.command_target = (tuple(positions), tuple(hands))

    def publish(self):
        """20 ms 定时回调：检查新鲜反馈及 URS 后，以当前 ROS 时间发送最新目标。

        没有目标时不发布；MC 模式变化、反馈失效或发布间隔超过 80 ms 时抛出 ControlFailure。
        """
        if self.command_target is None:
            return
        values = self.checked()
        now = time.monotonic()
        if self.publisher is None:
            raise ControlFailure('缺少上肢命令发布器')
        gap = now - self.last_publish if self.last_publish is not None else None
        if gap is not None and gap > .08:
            raise ControlFailure(f'命令发布间隔 {gap:.3f} 秒超过 0.08 秒')
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

        hand_motion 标记夹爪动作；duration 为目标持续发送时间，不生成手臂轨迹。
        夹爪状态、速度和开度误差仅记录，不作为中止或终点条件；硬件故障仍中止。
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
        settling_timeout = 2.
        # 仅移动关节使用本阶段终点容差，不给其余保持关节施加精密定位门槛。
        tolerances = tuple(arm_tolerance if abs(a-b) > 1e-9 else ARM_POSITION_TOLERANCE
                           for a, b in zip(origin, target))
        tolerances_deg = [math.degrees(value) for value in tolerances]
        self.diagnostics.record('segment_begin', phase=phase, duration_s=duration,
                                settling_timeout_s=settling_timeout, origin_rad=list(origin),
                                target_rad=list(target), arm_tolerance_deg=math.degrees(arm_tolerance),
                                arm_tolerances_deg=tolerances_deg,
                                hand_target=list(hands), hand_opening_check=False, hand_state_check=False,
                                first_sequence=previous_sequence)
        timing = '夹爪目标持续发送' if hand_motion else '轨迹'
        print(f'阶段 {phase}：{timing} {duration:g} 秒，移动关节容差 ±{math.degrees(arm_tolerance):g}°，'
              f'保持关节容差 ±{math.degrees(ARM_POSITION_TOLERANCE):g}°，'
              f'终点最多等待 {settling_timeout:g} 秒。', flush=True)
        # 进入本阶段后先更新目标，避免下一次定时回调发送上个阶段的手爪目标。
        self.set_target(origin if duration else target, hands)
        while True:
            values = self.tick()
            now = time.monotonic()
            commanded = (target if hand_motion or not duration
                         else interpolate(origin, target, now - began, duration))
            check_arm(values['arm'].positions, commanded)
            self.set_target(commanded, hands)
            if phase_published_at is None and self.sequence > previous_sequence:
                phase_published_at = self.last_publish
            if now >= arrival:
                reached = all(abs(a-b) <= tolerance
                              for a, b, tolerance in zip(values['arm'].positions, target, tolerances))
                newer = phase_published_at is not None and all(
                    values[name].serial > serial and values[name].stamp > stamp
                    and values[name].received > max(arrival, phase_published_at)
                    for name, serial, stamp in zip(('arm', 'hand'), serials, stamps))
                since = (since if since is not None else now) if reached and newer else None
                if since is not None and now - since >= .3:
                    self.diagnostics.record('endpoint_confirmed', phase=phase,
                                            arm_rad=list(values['arm'].positions),
                                            target_rad=list(target), hand_openings=list(values['hand'].positions),
                                            hand_target=list(hands), hand_opening_check=False, hand_state_check=False,
                                            arm_tolerance_deg=math.degrees(arm_tolerance),
                                            arm_tolerances_deg=tolerances_deg,
                                            published_count=self.sequence-previous_sequence)
                    return values
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
                    raise ControlFailure(f'阶段 {phase} 未稳定到位：关节超差 [{", ".join(joints) or "无"}]；'
                                      f'夹爪 [{hand_text}]；新源反馈={newer}；'
                                      f'本阶段已发布 {self.sequence-previous_sequence} 帧；'
                                      f'DDS 订阅匹配数={self.publisher.get_subscription_count()}')

    def execute(self, start, target, hand_targets):
        """发送一次完整目标；夹爪按 timeout 持续发送，确认关节后停止并报告实测误差。"""
        self.diagnostics.record('execution_feedback_limits',
                                **{f'{name}_timeout_s': stream.monitor.feedback_timeout
                                   for name, stream in self.streams.items()})
        self.enable_commands()
        hand_motion = self.options.action == 'gripper'
        values = self.segment(start, target, hand_targets,
                              self.options.timeout if hand_motion else self.options.duration,
                              ARM_POSITION_TOLERANCE, hand_motion=hand_motion,
                              phase=self.options.action)
        self.close()
        # 沿用已验证的 ±3°判定，报告实际运动，避免把容差内通过误称为精确完成小增量。
        for name, origin, actual, expected in zip(ARM_NAMES, start, values['arm'].positions, target):
            print(f'{name}: 目标={math.degrees(expected):+.3f}°，'
                  f'实测={math.degrees(actual):+.3f}°，'
                  f'实际增量={math.degrees(actual-origin):+.3f}°，'
                  f'目标误差={math.degrees(actual-expected):+.3f}°', flush=True)
        print(f'夹爪目标={list(hand_targets)}，实测={list(values["hand"].positions)}', flush=True)
        print('本次关节目标在 ±3° 容差内确认；夹爪状态及静止检查已屏蔽，不判定夹爪到位。'
              '小增量的通过不证明发生了同等幅度的实际运动。', flush=True)

    def close(self):
        """取消发布定时器并清除目标；不执行自动回程或模式切换。"""
        if self.closed:
            return
        self.closed = True
        self.command_target = None
        if self.command_timer is not None:
            self.command_timer.cancel()
        self.diagnostics.record('commands_stopped', published_count=self.sequence)



def main(args=None):
    """执行一次命令；默认只读。返回成功 0、失败 1、人工中断 130；不返回起点或切换模式。"""
    options = parser().parse_args(args)
    diagnostics = node = port = None
    initialized = False
    try:
        offsets = validate_options(options)
        diagnostics = DiagnosticLog(options.log_dir)
        diagnostics.record('run_start', options=vars(options), offsets_rad=list(offsets),
                           hand_state_check=False, hand_opening_check=False, hand_fault_check=True)
        print(f'诊断日志：{diagnostics.path}', flush=True)
        print('hand_state_check=False：已屏蔽夹爪状态及静止门槛；继续记录原始反馈并检查故障码和时效。',
              flush=True)
        import rclpy
        from aimdk_msgs.srv import GetMcAction
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = rclpy.create_node('dual_arm_control')
        if node.get_parameter('use_sim_time').value:
            raise ControlFailure('本脚本仅用于真机，不支持 use_sim_time')
        client = node.create_client(GetMcAction, '/aimdk_5Fmsgs/srv/GetMcAction')
        confirm_initial_mode(rclpy, node, options, diagnostics, client)
        diagnostics.record('runtime_mode_monitor', source='/aima/mc/common/state',
                           required_action=URS, required_status=100)
        port = DualArmPort(rclpy, node, options, diagnostics)
        start, hands = port.capture()
        target = joint_target(start, options.tag, offsets) if options.action == 'joint' else start
        hand_targets = (hands if options.action == 'joint'
                        else gripper_target(hands, options.side, options.opening))
        diagnostics.record('command_plan', arm_origin_rad=list(start), arm_target_rad=list(target),
                           hand_origin=list(hands), hand_target=list(hand_targets))
        if options.action == 'joint':
            for index, (origin, expected) in enumerate(zip(start, target)):
                if offsets[index // 7] is not None and index % 7 == JOINT_TAGS.index(options.tag):
                    print(f'计划 {ARM_NAMES[index]}：{math.degrees(origin):+.3f}° → '
                          f'{math.degrees(expected):+.3f}°，用时 {options.duration:g} 秒', flush=True)
        else:
            print(f'夹爪目标（左、右）：{list(hand_targets)}；双臂保持实测起点。', flush=True)
        if options.execute:
            port.execute(start, target, hand_targets)
            port.close()
            print('本次动作结束，已停止发布；无自动回程或模式切换。'
                  '停止发布不保证持续保持目标。', flush=True)
        else:
            print('只读检查完成，未创建命令发布器；使用 --execute 显式执行。', flush=True)
        diagnostics.record('run_pass', read_only=not options.execute)
        return 0
    except KeyboardInterrupt:
        if port is not None:
            port.close()
        if diagnostics:
            diagnostics.record('run_interrupted')
        print('用户中断；停止发布，不自动返回或切换模式。停止发布不等于急停。', file=sys.stderr)
        return 130
    except Exception as error:
        if port is not None:
            port.close()
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
