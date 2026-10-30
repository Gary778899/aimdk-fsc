"""X2 Ultra v1.3.0模型资源、槽位和统一执行限位；不导入ROS。

执行限位采用参考项目x2ik 2.1.0范围，左肩roll下限减1°、右肩roll上限加1°。
关节、TCP和补偿后的发送目标共用EXECUTION_LIMITS_RAD，不与旧SDK范围取交集。
"""

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ..dual_arm_control_core import ARM_NAMES


BASE_FRAME = 'torso_link'
POSITION_UNIT = 'm'
JOINT_UNIT = 'rad'
QUATERNION_ORDER = 'xyzw'
MODEL_SHA256 = 'f628a969aaf3d5785b8c8901611a85f358d7eb62a8b06b1216dc42291a72944e'
REFERENCE_MODEL_SHA256 = '1163b3c76b31c4ea0afd284b67b28948003ce46f7ea4b2d9826f1309e9af7f11'
REFERENCE_LIMITS_PATH = Path(__file__).resolve().parent / 'assets' / 'reference_joint_limits.json'


@dataclass(frozen=True)
class JointMapping:
    """一个关节的具名映射；model_index 为单臂索引，sdk_slot 为 arm_pos 槽位，均从零开始。"""

    name: str
    side: str
    model_index: int
    sdk_slot: int


ARM_JOINT_MAP = tuple(
    JointMapping(name, 'left' if slot < 7 else 'right', slot % 7, slot)
    for slot, name in enumerate(ARM_NAMES)
)


def model_path():
    """返回安装包内完整官方 URDF 的路径；字节哈希不匹配时抛出 ValueError。"""
    path = Path(__file__).resolve().parent / 'assets' / 'x2_ultra_v1_3_0.urdf'
    if hashlib.sha256(path.read_bytes()).hexdigest() != MODEL_SHA256:
        raise ValueError('包内 X2 Ultra v1.3.0 URDF 哈希不匹配')
    return path


def model_limits():
    """按 ARM_NAMES 返回官方模型的 14 组 rad 范围；缺失、重复或无效限位时抛出 ValueError。"""
    joints = ET.parse(model_path()).getroot().findall('joint')
    result = []
    for name in ARM_NAMES:
        matches = [joint for joint in joints if joint.get('name') == name]
        if len(matches) != 1 or matches[0].find('limit') is None:
            raise ValueError(f'官方模型关节或限位不唯一：{name}')
        limit = matches[0].find('limit')
        lower, upper = float(limit.get('lower')), float(limit.get('upper'))
        if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
            raise ValueError(f'官方模型关节限位无效：{name}')
        result.append((lower, upper))
    return tuple(result)


MODEL_LIMITS_RAD = model_limits()


def reference_limits():
    """按ARM_NAMES读取固定参考URDF的14组rad限位；来源、名称或数值无效时抛出ValueError。

    返回调整前的参考基准，不与官方模型或旧命令范围求交；不改变原始资源。
    """
    data = json.loads(REFERENCE_LIMITS_PATH.read_text(encoding='utf-8'))
    if (data['schema_version'] != 1 or data['units'] != 'rad'
            or data['source']['urdf_sha256'] != REFERENCE_MODEL_SHA256):
        raise ValueError('参考关节限位配置来源或单位不匹配')
    entries = data['joints']
    names = [entry['name'] for entry in entries]
    if len(names) != 14 or len(set(names)) != 14 or set(names) != set(ARM_NAMES):
        raise ValueError('参考限位须包含完整且唯一的14个关节名称')
    found = {entry['name']: entry for entry in entries}
    result = []
    for name in ARM_NAMES:
        lower, upper = float(found[name]['lower']), float(found[name]['upper'])
        if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
            raise ValueError(f'参考关节限位无效：{name}')
        result.append((lower, upper))
    return tuple(result)


REFERENCE_LIMITS_RAD = reference_limits()
# 按用户要求沿前述超限方向扩展肩roll：左下限减1°，右上限加1°；原始参考数据不改写。
LIMIT_ADJUSTMENTS_DEG = {
    'left_shoulder_roll_joint': (-1.0, 0.0),
    'right_shoulder_roll_joint': (0.0, 1.0),
}
# 求解器、候选检查和完整目标组装共用调整后的范围，不另外放宽校验容差。
EXECUTION_LIMITS_RAD = tuple(
    (lower + math.radians(LIMIT_ADJUSTMENTS_DEG.get(name, (0.0, 0.0))[0]),
     upper + math.radians(LIMIT_ADJUSTMENTS_DEG.get(name, (0.0, 0.0))[1]))
    for name, (lower, upper) in zip(ARM_NAMES, REFERENCE_LIMITS_RAD)
)


def side_limits(side):
    """返回 left/right 单臂的七组有效 rad 范围；侧别错误时抛出 ValueError。"""
    if side not in ('left', 'right'):
        raise ValueError('side 必须为 left 或 right')
    start = 0 if side == 'left' else 7
    return EXECUTION_LIMITS_RAD[start:start + 7]


def apply_execution_limits(model):
    """给新建单臂模型注入一致范围，返回该模型；侧别、顺序或数组不符时抛出 ValueError。

    model 需提供 side、joint_names、可原位写入的七维 q_min/q_max。
    调用方必须在构造 IK 求解器及其批量限位缓存之前调用，不用于修改运行中的求解器。
    不裁剪任何关节输入或目标，也不改变模型 geoms 中保留的官方原始范围。
    """
    limits = side_limits(model.side)
    expected = tuple(item.name for item in ARM_JOINT_MAP if item.side == model.side)
    if tuple(model.joint_names) != expected:
        raise ValueError('单臂模型关节顺序与 SDK 映射不一致')
    if len(model.q_min) != 7 or len(model.q_max) != 7:
        raise ValueError('单臂模型限位必须各含七项')
    model.q_min[:] = [lower for lower, upper in limits]
    model.q_max[:] = [upper for lower, upper in limits]
    return model


def parameter_report():
    """返回可序列化的坐标约定、模型哈希和完整 14 关节模型/执行范围及槽位对应表。"""
    return dict(
        base_frame=BASE_FRAME, position_unit=POSITION_UNIT, joint_unit=JOINT_UNIT,
        quaternion_order=QUATERNION_ORDER, model_sha256=MODEL_SHA256,
        execution_limits_policy='reference_urdf_with_shoulder_roll_extension',
        execution_limits_source=json.loads(REFERENCE_LIMITS_PATH.read_text(encoding='utf-8'))['source'],
        execution_limit_adjustments_deg=LIMIT_ADJUSTMENTS_DEG,
        rotation_convention='R_torso_tcp 将 TCP 坐标向量变换到 torso_link；列向量、主动旋转',
        joints=[dict(name=item.name, side=item.side, model_index=item.model_index,
                     sdk_slot=item.sdk_slot, model_limits_rad=MODEL_LIMITS_RAD[item.sdk_slot],
                     reference_limits_rad=REFERENCE_LIMITS_RAD[item.sdk_slot],
                     execution_limits_rad=EXECUTION_LIMITS_RAD[item.sdk_slot])
                for item in ARM_JOINT_MAP],
    )
