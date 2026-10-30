"""单次双臂/夹爪控制的纯计算、反馈校验与源时间监视；不导入 ROS。"""
from dataclasses import dataclass
import math


JOINT_TAGS = ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
              'wrist_yaw', 'wrist_pitch', 'wrist_roll')
ARM_NAMES = tuple(f'{side}_{tag}_joint' for side in ('left', 'right') for tag in JOINT_TAGS)
URS = 'UPPERBODY_REMOTE_SPLIT'
ARM_POSITION_TOLERANCE = math.radians(3)
# SDK 1.7.1 按 0.5 度舍入的活动范围；右侧 roll 的符号映射依据官方 motocontrol.py。
DOCUMENT_LIMITS_DEG = ((-176.5, 116.5), (-3.5, 174.5), (-146.5, 146.5),
                       (-135., 0.), (-146.5, 146.5), (-30., 30.), (-86.5, 41.5),
                       (-176.5, 116.5), (-174.5, 3.5), (-146.5, 146.5),
                       (-135., 0.), (-146.5, 146.5), (-30., 30.), (-41.5, 86.5))
# 示例中更窄的边界不自动放宽；取文档范围与示例范围的交集，不附加任意余量。
EXAMPLE_LIMITS_RAD = ((-3.08, 2.04), (-.061, 2.993), (-2.556, 2.556),
                      (-2.3556, 0.), (-2.556, 2.556), (-.558, .558), (-1.571, .724),
                      (-3.08, 2.04), (-2.993, .061), (-2.556, 2.556),
                      (-2.3556, 0.), (-2.556, 2.556), (-.558, .558), (-.724, 1.571))
JOINT_LIMITS_RAD = tuple((max(math.radians(a), c), min(math.radians(b), d))
                         for (a, b), (c, d) in zip(DOCUMENT_LIMITS_DEG, EXAMPLE_LIMITS_RAD))


class ControlFailure(RuntimeError):
    """控制前置条件或反馈检查失败；不代表物理急停。"""


class FeedbackMonitor:
    def __init__(self, feedback_timeout=.2):
        """初始化反馈监视器。

        feedback_timeout 是接收间隔、源时间停滞及源时间年龄的秒数上限，
        默认 0.2 秒；无效数值抛出 ControlFailure。ROS 时钟停滞上限始终为 0.2 秒。
        """
        if not math.isfinite(feedback_timeout) or feedback_timeout <= 0:
            raise ControlFailure('feedback_timeout must be positive and finite')
        self.feedback_timeout = feedback_timeout
        self.latest = None
        self.source_advanced_at = None
        self.last_ros = None
        self.clock_advanced_at = None

    def accept(self, feedback):
        """接收含源时间与本地接收时间的样本；源时间回退时抛出 ControlFailure。"""
        previous = self.latest
        if previous and feedback.stamp < previous.stamp:
            raise ControlFailure('feedback source clock moved backwards')
        if previous is None or feedback.stamp > previous.stamp:
            self.source_advanced_at = feedback.received
        self.latest = feedback

    def observe_clock(self, now, ros_now):
        """以单调时钟观测 ROS 时钟推进；无效时间或回退时抛出 ControlFailure。"""
        if not math.isfinite(ros_now) or ros_now <= 0:
            raise ControlFailure('invalid ROS clock')
        if self.last_ros is not None and ros_now < self.last_ros:
            raise ControlFailure('ROS clock moved backwards')
        if self.last_ros is None or ros_now > self.last_ros:
            self.clock_advanced_at = now
        self.last_ros = ros_now

    def checked(self, now, ros_now):
        """返回满足各自时限的反馈；失败时抛出含实际年龄及上限的 ControlFailure。"""
        self.observe_clock(now, ros_now)
        fb = self.latest
        if fb is None:
            raise ControlFailure('feedback unavailable')
        ages = {
            'receive gap': (now - fb.received, self.feedback_timeout),
            'source timestamp frozen': (now - self.source_advanced_at, self.feedback_timeout),
            'ROS clock frozen': (now - self.clock_advanced_at, .2),
        }
        failures = [f'{name}={age:.6f}s > {limit:.6f}s'
                    for name, (age, limit) in ages.items() if age > limit]
        if failures:
            raise ControlFailure('; '.join(failures))
        age = ros_now - fb.stamp
        if not -.05 <= age <= self.feedback_timeout:
            raise ControlFailure(f'feedback source/local clock mismatch or stale stamp: {age:.3f}s; '
                              f'allowed [-0.050000, {self.feedback_timeout:.6f}]s')
        return fb



@dataclass(frozen=True)
class Sample:
    """保留解码值、原始源时间、本地单调接收时间和序号的反馈样本。"""
    positions: tuple
    stamp: float
    received: float
    serial: int
    data: dict


def header_data(header):
    """返回消息头原始源时间、测量时间与序号，用于日志，不改写时间。"""
    return dict(stamp_s=header.stamp.sec + header.stamp.nanosec * 1e-9,
                meas_stamp_s=header.meas_stamp.sec + header.meas_stamp.nanosec * 1e-9,
                sequence=header.sequence)


def sample(msg, received, serial, positions, data):
    """由原始消息、单调接收时间及位置构造 Sample；源时间无效时抛出 ControlFailure。"""
    header = header_data(msg.header)
    if not math.isfinite(header['stamp_s']) or header['stamp_s'] <= 0:
        raise ControlFailure('source timestamp must be nonzero and finite')
    return Sample(tuple(positions), header['stamp_s'], received, serial,
                  dict(header=header, **data))


def decode_arm(msg, received, serial):
    """按名称重排完整 14 关节反馈；缺失、重复、故障或非有限值时抛出 ControlFailure。"""
    names = [joint.name for joint in msg.joints]
    if len(names) != 14 or len(set(names)) != 14 or set(names) != set(ARM_NAMES):
        raise ControlFailure(f'expected exactly 14 unique arm joints; observed {names}')
    if msg.state.value != 0:
        raise ControlFailure(f'arm domain error={msg.state.value}')
    found = {joint.name: joint for joint in msg.joints}
    joints = []
    for name in ARM_NAMES:
        joint = found[name]
        if joint.error_code or not all(math.isfinite(v) for v in (
                joint.position, joint.velocity, joint.effort)):
            raise ControlFailure(f'invalid arm joint {name}: error={joint.error_code}')
        joints.append(dict(name=name, position=joint.position, velocity=joint.velocity,
                           effort=joint.effort, error_code=joint.error_code))
    return sample(msg, received, serial, [j['position'] for j in joints],
                  dict(observed_names=names, domain_state=msg.state.value, joints=joints))


def decode_hand(msg, received, serial):
    """解码两侧 CLAW 夹爪反馈并记录运动状态；类型、数量、开度或故障无效时抛出 ControlFailure。"""
    hands = []
    for side in ('left', 'right'):
        kind = getattr(msg, f'{side}_hand_type').value
        values = getattr(msg, f'{side}_hands')
        if kind != 2 or len(values) != 1:
            raise ControlFailure(f'{side}: expected CLAW=2 and one hand; type={kind}, count={len(values)}')
        hand = values[0]
        if (hand.faultcode or not all(math.isfinite(v) for v in (
                hand.position, hand.velocity, hand.effort)) or not 0 <= hand.position <= 1):
            raise ControlFailure(f'{side}: invalid claw feedback, faultcode={hand.faultcode}')
        hands.append(dict(side=side, type=kind, name=hand.name, position=hand.position,
                          velocity=hand.velocity, effort=hand.effort,
                          state=hand.state, faultcode=hand.faultcode,
                          stationary_candidate=hand.state == 0 and abs(hand.velocity) <= .01))
    return sample(msg, received, serial, [h['position'] for h in hands], dict(hands=hands))


class Stream:
    """为各流保留时钟与时效检查及永久错误锁存。"""

    def __init__(self, name, decoder):
        """初始化具名反馈流及错误锁存；decoder 接受消息、单调接收时间和样本序号。"""
        self.name, self.decoder = name, decoder
        self.monitor = FeedbackMonitor()
        self.error = None
        self.serial = 0
        self.first_stamp = None

    def accept(self, msg, received):
        """解码并接收样本；首次错误永久锁存，后续正常反馈不会清除错误。"""
        self.serial += 1
        try:
            value = self.decoder(msg, received, self.serial)
            self.monitor.accept(value)
            if self.first_stamp is None:
                self.first_stamp = value.stamp
        except ControlFailure as error:
            self.error = self.error or error

    def checked(self, now, ros_now):
        """返回接收时效、源时间年龄及推进均有效的样本；无效时抛出 ControlFailure。"""
        if self.error:
            raise ControlFailure(f'{self.name}: {self.error}')
        try:
            value = self.monitor.checked(now, ros_now)
        except ControlFailure as error:
            raise ControlFailure(f'{self.name}: {error}') from error
        if value.stamp <= self.first_stamp:
            raise ControlFailure(f'{self.name}: waiting for source timestamp advancement')
        return value


def require_static_hands(value):
    """要求两侧夹爪均为静止候选，否则抛出 ControlFailure；硬件故障由解码器检查。"""
    if not all(h['stationary_candidate'] for h in value.data['hands']):
        raise ControlFailure('both claws must remain stationary, state=0, abs(velocity)<=0.01')


def decode_urs_mc(msg, received, serial):
    """校验站立、URS/RUNNING 并保留原始源时间；状态不符时抛出 ControlFailure。"""
    action, status = msg.action_info.action_desc, msg.action_info.status.value
    if msg.body_status.value != 1 or action != URS or status != 100:
        raise ControlFailure(f'MC 必须为 body=STAND、URS/RUNNING：'
                             f'body={msg.body_status.value}, action={action}, status={status}')
    return sample(msg, received, serial, (), dict(body=msg.body_status.value,
                  action=action, action_status=status, fsm_raw=msg.fsm_state.current_state))


def joint_target(start, tag, offsets):
    """从完整 rad 起点和左右可选 rad 增量生成目标；None 表示未选侧。

    仅对被选侧的 tag 检查起点和目标限位；其余关节原样保持、不裁剪。
    无效数据、增量超过 10 度或越界时抛出 ControlFailure。
    """
    if (len(start) != 14 or not all(math.isfinite(v) for v in start)
            or tag not in JOINT_TAGS or len(offsets) != 2):
        raise ControlFailure('需要完整 14 关节有限起点、有效 tag 和左右增量')
    result = list(start)
    for side_index, offset in enumerate(offsets):
        if offset is None:
            continue
        if not math.isfinite(offset) or abs(offset) > math.radians(10):
            raise ControlFailure('单次关节增量绝对值不得超过 10 度，且必须为有限数值')
        index = side_index * 7 + JOINT_TAGS.index(tag)
        lower, upper = JOINT_LIMITS_RAD[index]
        target = start[index] + offset
        if not lower <= start[index] <= upper or not lower <= target <= upper:
            raise ControlFailure(f'{ARM_NAMES[index]} 起点 {math.degrees(start[index]):+.6f}° '
                                 f'或目标 {math.degrees(target):+.6f}° 超出范围 '
                                 f'[{math.degrees(lower):+.6f}, {math.degrees(upper):+.6f}]°；'
                                 '不裁剪、不自动调整起点，请核实姿态与机型限位')
        result[index] = target
    return tuple(result)


def gripper_target(hands, side, opening):
    """返回左右夹爪目标开度；未选侧保持采样值，无效参数时抛出 ControlFailure。"""
    if (side not in ('left', 'right', 'both') or len(hands) != 2
            or not all(math.isfinite(v) and 0 <= v <= 1 for v in (*hands, opening))):
        raise ControlFailure('需要有效侧别及两个当前开度和目标开度，范围均为 0..1')
    return tuple(opening if side in (name, 'both') else current
                 for name, current in zip(('left', 'right'), hands))


def interpolate(origin, target, elapsed, duration):
    """以秒为时间单位，返回起终点速度和加速度为零的完整五次插值 rad 目标。"""
    phase = max(0., min(1., elapsed / duration))
    blend = phase ** 3 * (10 - 15 * phase + 6 * phase * phase)
    return tuple(a + (b - a) * blend for a, b in zip(origin, target))


def check_arm(actual, target, tolerance=ARM_POSITION_TOLERANCE):
    """检查全部 14 个有限 rad 位置与目标的误差；超过 tolerance 时抛出 ControlFailure。"""
    if (len(actual) != 14 or len(target) != 14
            or not all(math.isfinite(v) for v in (*actual, *target))):
        raise ControlFailure('双臂反馈和目标必须是 14 个有限位置')
    for name, value, expected in zip(ARM_NAMES, actual, target):
        if abs(value - expected) > tolerance:
            raise ControlFailure(f'{name} 位置误差 {math.degrees(value-expected):+.3f}°，'
                                 f'超过 ±{math.degrees(tolerance):g}°')
