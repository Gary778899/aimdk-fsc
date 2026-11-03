"""将一条完整SDK Tracking JSON解析为显式坐标约定下的XR帧。"""

from dataclasses import dataclass
import json
import math
from pathlib import Path

from .frames import Controller, Pose, XRFrame
from .geometry import quaternion_matrix


@dataclass(frozen=True)
class InputConfig:
    """安装版本的原始数据约定；状态字段及其取值由现场确认后显式填写。"""

    quaternion_order: str
    position_scale_m: float
    reference_space_id: str
    validity_field: str | None
    valid_values: tuple
    reference_space_field: str | None

    def __post_init__(self):
        """校验坐标、单位和可选字段约定；配置不完整时抛出ValueError。"""
        if self.quaternion_order not in ('xyzw', 'wxyz'):
            raise ValueError('quaternion_order须显式为xyzw或wxyz')
        if (isinstance(self.position_scale_m, bool)
                or not math.isfinite(self.position_scale_m) or self.position_scale_m <= 0):
            raise ValueError('position_scale_m须为有限正数')
        if not isinstance(self.reference_space_id, str) or not self.reference_space_id:
            raise ValueError('reference_space_id须为非空字符串')
        for field in (self.validity_field, self.reference_space_field):
            if field is not None and (not isinstance(field, str) or not field):
                raise ValueError('可选状态字段须为非空字符串或null')
        if not isinstance(self.valid_values, (list, tuple)):
            raise ValueError('valid_values须为显式数组')
        if self.validity_field is not None and not self.valid_values:
            raise ValueError('配置validity_field时须显式提供valid_values')
        object.__setattr__(self, 'valid_values', tuple(self.valid_values))

    @classmethod
    def from_dict(cls, value):
        """读取全部显式配置字段；未知或缺失字段按构造异常报告。"""
        return cls(**value)

    @classmethod
    def load(cls, path):
        """从JSON文件加载输入协议约定；路径或格式错误直接传播。"""
        return cls.from_dict(json.loads(Path(path).read_text(encoding='utf-8')))


def _number(value, label):
    """返回有限数值，拒绝布尔量与非数值；非法输入抛出ValueError。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{label}须为有限数值')
    return float(value)


def _controller(value, config):
    """从单侧原始对象构造控制器；位置单位和四元数顺序使用显式配置。"""
    if not isinstance(value, dict):
        raise ValueError('Controller.left/right须为对象')
    raw_pose = value['pose']
    if isinstance(raw_pose, str):
        raw_pose = [float(item) for item in raw_pose.split(',')]
    if not isinstance(raw_pose, (list, tuple)) or len(raw_pose) != 7:
        raise ValueError('手柄pose须包含3位置和4四元数分量')
    pose_values = tuple(_number(item, '手柄pose') for item in raw_pose)
    pose = Pose(tuple(item * config.position_scale_m for item in pose_values[:3]),
                quaternion_matrix(pose_values[3:], config.quaternion_order))
    buttons = []
    for name in ('menuButton', 'primaryButton', 'secondaryButton', 'axisClick'):
        if name in value:
            if not isinstance(value[name], bool):
                raise ValueError(f'{name}须为布尔值')
            buttons.append((name, value[name]))
    valid = None
    if config.validity_field is not None and config.validity_field in value:
        raw_validity = value[config.validity_field]
        valid = any(type(raw_validity) is type(item) and raw_validity == item
                    for item in config.valid_values)
    axis = None
    if 'axisX' in value or 'axisY' in value:
        if 'axisX' not in value or 'axisY' not in value:
            raise ValueError('摇杆axisX/axisY须同时出现')
        axis = (_number(value['axisX'], 'axisX'), _number(value['axisY'], 'axisY'))
    return Controller(pose=pose, grip=_number(value['grip'], 'grip'),
                      trigger=_number(value['trigger'], 'trigger'),
                      buttons=tuple(buttons), tracking_valid=valid, axis=axis)


def parse_tracking(event, seq, config):
    """将完整设备回调转为XRFrame；非Tracking或未发送Controller时返回None。

    event为SDKEvent，seq为消费端分配的输入帧序号；源时间缺失保留None，
    跟踪字段缺失保留未知。JSON或左右帧不完整抛出ValueError/KeyError。
    """
    if event.kind != 'device_state':
        return None
    if event.error:
        raise ValueError(event.error)
    outer = json.loads(event.payload().decode('utf-8'))
    if not isinstance(outer, dict):
        raise ValueError('SDK设备状态须为JSON对象')
    function = outer.get('functionName', outer.get('function'))
    if function != 'Tracking':
        return None
    value = outer['value']
    if isinstance(value, str):
        value = json.loads(value)
    if not isinstance(value, dict):
        raise ValueError('Tracking.value须为JSON对象或对象字符串')
    if 'Controller' not in value:
        return None
    controllers = value['Controller']
    if not isinstance(controllers, dict):
        raise ValueError('Tracking.Controller须为对象')
    source_time = value.get('timeStampNs')
    if source_time is not None and (type(source_time) is not int or source_time < 0):
        raise ValueError('timeStampNs须为非负整数或缺失')
    reference = config.reference_space_id
    if config.reference_space_field is not None:
        reference = value[config.reference_space_field]
        if not isinstance(reference, str) or not reference:
            raise ValueError('参考空间标识须为非空字符串')
    return XRFrame(seq=seq, source_time_ns=source_time,
                   received_monotonic_s=event.received_monotonic_s,
                   left=_controller(controllers['left'], config),
                   right=_controller(controllers['right'], config),
                   connected=True, reference_space_id=reference, device_id=event.device_id)
