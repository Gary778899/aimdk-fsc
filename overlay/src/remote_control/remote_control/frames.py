"""XR回调快照及通用双侧TCP目标契约，位置单位m、姿态为旋转矩阵。"""

from dataclasses import asdict, dataclass
import math

from .geometry import immutable_matrix, vector


SIDES = ('left', 'right')


def integer(value, label, minimum=0):
    """校验非布尔整数及下界，失败抛出ValueError并保留字段名称。"""
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f'{label}须为不小于{minimum}的整数')
    return value


def finite(value, label, minimum=None, maximum=None):
    """校验有限标量及可选闭区间，返回float；非法输入抛出ValueError。"""
    if isinstance(value, bool):
        raise ValueError(f'{label}须为有限数值')
    result = float(value)
    if (not math.isfinite(result) or (minimum is not None and result < minimum)
            or (maximum is not None and result > maximum)):
        raise ValueError(f'{label}超出有限数值范围')
    return result


def boolean(value, label):
    """校验显式布尔值；其他类型抛出ValueError。"""
    if not isinstance(value, bool):
        raise ValueError(f'{label}须为布尔值')
    return value


@dataclass(frozen=True)
class Pose:
    """不可变位置m及将工具坐标映射至父坐标系的旋转矩阵。"""

    position_m: tuple
    rotation_matrix: tuple

    def __post_init__(self):
        """检查有限位置和正交旋转，并复制为元组。"""
        object.__setattr__(self, 'position_m', tuple(vector(self.position_m, 3, '位置m')))
        object.__setattr__(self, 'rotation_matrix', immutable_matrix(self.rotation_matrix))

    def to_dict(self):
        """返回可序列化的完整位姿。"""
        return asdict(self)

    @classmethod
    def from_dict(cls, value):
        """从position_m和rotation_matrix构造位姿；缺字段或非法值直接报错。"""
        return cls(value['position_m'], value['rotation_matrix'])


@dataclass(frozen=True)
class Controller:
    """单手柄同帧状态；tracking_valid=None表示源端未提供判定字段。"""

    pose: Pose
    grip: float
    trigger: float
    buttons: tuple = ()
    tracking_valid: object = None
    axis: object = None

    def __post_init__(self):
        """校验位姿、归一化模拟量、按钮及可选跟踪状态。"""
        if not isinstance(self.pose, Pose):
            raise ValueError('手柄pose须为Pose')
        object.__setattr__(self, 'grip', finite(self.grip, 'grip', 0., 1.))
        object.__setattr__(self, 'trigger', finite(self.trigger, 'trigger', 0., 1.))
        buttons = self.buttons.items() if isinstance(self.buttons, dict) else self.buttons
        items = tuple((str(key), boolean(value, str(key))) for key, value in buttons)
        if len(dict(items)) != len(items):
            raise ValueError('按钮名重复')
        object.__setattr__(self, 'buttons', items)
        if self.tracking_valid is not None:
            boolean(self.tracking_valid, 'tracking_valid')
        if self.axis is not None:
            values = tuple(vector(self.axis, 2, '摇杆'))
            if not all(-1 <= value <= 1 for value in values):
                raise ValueError('摇杆轴须处于−1..1')
            object.__setattr__(self, 'axis', values)

    def to_dict(self):
        """返回位姿与按钮字典，保留未知跟踪状态。"""
        return dict(pose=self.pose.to_dict(), grip=self.grip, trigger=self.trigger,
                    buttons=dict(self.buttons), tracking_valid=self.tracking_valid, axis=self.axis)

    @classmethod
    def from_dict(cls, value):
        """从完整手柄字典构造同帧状态。"""
        return cls(Pose.from_dict(value['pose']), value['grip'], value['trigger'],
                   value.get('buttons', {}), value.get('tracking_valid'), value.get('axis'))


@dataclass(frozen=True)
class XRFrame:
    """同一SDK回调的双手输入；接收序号不等同于源端硬件帧号。"""

    seq: int
    source_time_ns: object
    received_monotonic_s: float
    left: Controller
    right: Controller
    connected: bool = True
    reference_space_id: object = None
    device_id: str = ''

    def __post_init__(self):
        """固定元数据并校验完整双手快照；源时间允许显式未知。"""
        integer(self.seq, 'seq', 1)
        if self.source_time_ns is not None:
            integer(self.source_time_ns, 'source_time_ns')
        object.__setattr__(self, 'received_monotonic_s',
                           finite(self.received_monotonic_s, '接收时间', 0.))
        for side in SIDES:
            if not isinstance(getattr(self, side), Controller):
                raise ValueError(f'{side}须为完整Controller')
        boolean(self.connected, 'connected')
        if self.reference_space_id is not None and not isinstance(self.reference_space_id, str):
            raise ValueError('reference_space_id须为字符串或None')
        if not isinstance(self.device_id, str):
            raise ValueError('device_id须为字符串')

    def to_dict(self):
        """返回标准XR日志记录，完整保留源端与接收时间。"""
        return dict(seq=self.seq, source_time_ns=self.source_time_ns,
                    received_monotonic_s=self.received_monotonic_s,
                    left=self.left.to_dict(), right=self.right.to_dict(),
                    connected=self.connected, reference_space_id=self.reference_space_id,
                    device_id=self.device_id)

    @classmethod
    def from_dict(cls, value):
        """读回标准XR帧；不推断缺失的连接状态或时间字段。"""
        return cls(value['seq'], value['source_time_ns'], value['received_monotonic_s'],
                   Controller.from_dict(value['left']), Controller.from_dict(value['right']),
                   value['connected'], value.get('reference_space_id'), value.get('device_id', ''))


@dataclass(frozen=True)
class TCPGoal:
    """单侧完整TCP目标及会话使能状态。"""

    pose: Pose
    enabled: bool

    def __post_init__(self):
        """校验目标位姿及显式使能标记。"""
        if not isinstance(self.pose, Pose):
            raise ValueError('TCP目标pose须为Pose')
        boolean(self.enabled, 'enabled')

    def to_dict(self):
        """返回完整TCP目标字典。"""
        return dict(pose=self.pose.to_dict(), enabled=self.enabled)

    @classmethod
    def from_dict(cls, value):
        """从目标位姿及使能状态恢复TCPGoal。"""
        return cls(Pose.from_dict(value['pose']), value['enabled'])


@dataclass(frozen=True)
class TargetFrame:
    """映射后的完整双侧目标，含左右夹爪开度及输入序号关联。"""

    input_seq: int
    source_time_ns: object
    received_monotonic_s: float
    left: TCPGoal
    right: TCPGoal
    hand_openings: tuple
    base_frame: str = 'torso_link'

    def __post_init__(self):
        """校验完整目标与时间；夹爪开度均处于0..1。"""
        integer(self.input_seq, 'input_seq', 1)
        if self.source_time_ns is not None:
            integer(self.source_time_ns, 'source_time_ns')
        object.__setattr__(self, 'received_monotonic_s',
                           finite(self.received_monotonic_s, '接收时间', 0.))
        for side in SIDES:
            if not isinstance(getattr(self, side), TCPGoal):
                raise ValueError(f'{side}须为TCPGoal')
        openings = tuple(vector(self.hand_openings, 2, '夹爪开度'))
        if not all(0 <= opening <= 1 for opening in openings):
            raise ValueError('夹爪开度须处于0..1')
        object.__setattr__(self, 'hand_openings', openings)
        if not isinstance(self.base_frame, str) or not self.base_frame:
            raise ValueError('base_frame须为非空字符串')

    def to_dict(self):
        """返回含输入关联的完整目标字典。"""
        return dict(input_seq=self.input_seq, source_time_ns=self.source_time_ns,
                    received_monotonic_s=self.received_monotonic_s,
                    left=self.left.to_dict(), right=self.right.to_dict(),
                    hand_openings=list(self.hand_openings), base_frame=self.base_frame)

    @classmethod
    def from_dict(cls, value):
        """从完整字典恢复双侧TCP目标。"""
        return cls(value['input_seq'], value['source_time_ns'], value['received_monotonic_s'],
                   TCPGoal.from_dict(value['left']), TCPGoal.from_dict(value['right']),
                   value['hand_openings'], value['base_frame'])
