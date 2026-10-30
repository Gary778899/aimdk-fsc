"""包内只读反馈适配与完整命令组装；运动发布状态机留给单臂执行验证阶段。"""

from functools import partial
import time

from .dual_arm_control_core import (
    ControlFailure, FeedbackMonitor, Stream, decode_arm, decode_hand,
    decode_urs_mc, sample,
)
from .upper_body_cartesian_core import CapturedTarget


def decode_observed_mode(msg, received, serial):
    """保留实际 MC 模式及源时间；只读观测不要求 URS，不解释 FSM 原始数值。"""
    return sample(msg, received, serial, (), dict(
        body=msg.body_status.value, action=msg.action_info.action_desc,
        action_status=msg.action_info.status.value,
        fsm_raw=msg.fsm_state.current_state))


class UpperBodyFeedbackPort:
    """复用现有解码、源时间监视及错误锁存，不创建发布器、不请求模式切换。"""

    def __init__(self, node, diagnostics, *, require_urs=False, feedback_timeout=.5, compensation=False):
        """创建 SDK 反馈订阅；require_urs 仅供后续执行监视，超时单位为秒。

        diagnostics 需提供 record(event, **fields)。compensation启用骨盆IMU及腰反馈订阅。
        模式确认服务及运动发布不在本类中。
        """
        from aimdk_msgs.msg import JointStateArray, HandStateArray, McCommonState
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

        self.node, self.diagnostics = node, diagnostics
        self.streams = dict(arm=Stream('arm', decode_arm), hand=Stream('hand', decode_hand),
                            mc=Stream('mc', decode_urs_mc if require_urs else decode_observed_mode))
        if compensation:
            from .upper_body_compensation import decode_pelvis_imu, decode_waist

            self.streams.update(pelvis_imu=Stream('pelvis_imu', decode_pelvis_imu),
                                waist=Stream('waist', decode_waist))
        for stream in self.streams.values():
            stream.monitor = FeedbackMonitor(feedback_timeout)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions = []
        try:
            topics = [
                    ('arm', JointStateArray, '/aima/hal/joint/arm/state'),
                    ('hand', HandStateArray, '/aima/hal/joint/hand/state'),
                    ('mc', McCommonState, '/aima/mc/common/state')]
            if compensation:
                from sensor_msgs.msg import Imu

                topics.extend([('pelvis_imu', Imu, '/aima/hal/imu/torso/state'),
                               ('waist', JointStateArray, '/aima/hal/joint/waist/state')])
            sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                                    durability=DurabilityPolicy.VOLATILE)
            for name, kind, topic in topics:
                self.subscriptions.append(node.create_subscription(
                    kind, topic, partial(self.receive, name),
                    sensor_qos if name in ('pelvis_imu', 'waist') else qos))
        except Exception:
            self.close()
            raise

    def receive(self, name, msg):
        """记录完整原始字段及接收时间，复用首次错误锁存；解码失败仍先记录原始消息。"""
        from rosidl_runtime_py.convert import message_to_ordereddict

        now = time.monotonic()
        stream = self.streams[name]
        stream.accept(msg, now)
        self.diagnostics.record('feedback', stream=name, received_monotonic_s=now,
                                ros_time_s=self.node.get_clock().now().nanoseconds / 1e9,
                                raw=message_to_ordereddict(msg), serial=stream.serial,
                                error=str(stream.error) if stream.error else None)
        if stream.error:
            raise ControlFailure(f'{name}: {stream.error}')

    def checked(self):
        """返回各路新鲜且源时间推进的样本；任何锁存错误或时钟异常向上传播。"""
        now = time.monotonic()
        ros_now = self.node.get_clock().now().nanoseconds / 1e9
        return {name: stream.checked(now, ros_now) for name, stream in self.streams.items()}

    def close(self):
        """销毁本适配器创建的订阅；不改变机器人模式或发送任何运动消息。"""
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
        self.subscriptions.clear()


def build_upper_body_command(node, target, sequence):
    """由 CapturedTarget 组装原生完整左7右7 rad 及模式1夹爪消息，不发布。

    调用方仍须执行模式有界查询、反馈监视和发布前候选检查；本函数不授予运动许可。
    target 应由固定采样的 for_arm() 生成；sequence 为 uint32 范围内整数序号。
    """
    from aimdk_msgs.msg import UpperBodyCommandArray

    if not isinstance(target, CapturedTarget):
        raise ControlFailure('命令目标必须为 CapturedTarget')
    if (isinstance(sequence, bool) or not isinstance(sequence, int)
            or not 0 <= sequence <= 0xffffffff):
        raise ControlFailure('命令序号须为 uint32 范围内整数')
    msg = UpperBodyCommandArray()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'mc_upper_body'
    msg.header.sequence = sequence
    msg.source = 'upper_body_cartesian'
    msg.head_pos = [0., 0.]  # SDK 规定 URS 下头部目标无效。
    msg.arm_pos = list(target.arm_rad)
    msg.hand_sub_mode = UpperBodyCommandArray.HAND_CLAW_OPEN_CLOSE
    msg.hand_pos = list(target.hand_openings)
    return msg
