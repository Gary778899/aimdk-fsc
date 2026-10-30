"""笛卡尔适配的完整关节目标契约；不导入 ROS，不执行运动或生成轨迹。"""

from dataclasses import dataclass
import math

from .dual_arm_control_core import ARM_NAMES, ControlFailure
from .upper_body_kinematics.parameters import EXECUTION_LIMITS_RAD


def finite_positions(values, count, label):
    """将 values 转为 count 个有限浮点数；数量、类型或数值异常抛出 ControlFailure。"""
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError) as error:
        raise ControlFailure(f'{label}: 必须为有限数值序列') from error
    if len(result) != count or not all(math.isfinite(value) for value in result):
        raise ControlFailure(f'{label}: 需要 {count} 个有限数值')
    return result


@dataclass(frozen=True)
class CapturedTarget:
    """本次采样的左7右7关节 rad 及左右夹爪开度，不随反馈自动更新。"""

    arm_rad: tuple
    hand_openings: tuple

    def __post_init__(self):
        """校验完整采样并固定元组；不裁剪任何关节，夹爪须处于 0..1。"""
        object.__setattr__(self, 'arm_rad', finite_positions(self.arm_rad, 14, '采样关节'))
        hands = finite_positions(self.hand_openings, 2, '采样夹爪')
        if not all(0 <= value <= 1 for value in hands):
            raise ControlFailure('采样夹爪开度须处于 0..1')
        object.__setattr__(self, 'hand_openings', hands)

    def for_arm(self, side, candidate_rad):
        """以单臂7关节候选构造完整目标；检查所选臂起点及目标范围，其余原样保持。

        side 为 left/right。超限或非有限输入抛出 ControlFailure，不裁剪、不重求解。
        返回新的 CapturedTarget，原采样保持不变；这里不检查 FK 残差或连续性。
        """
        if side not in ('left', 'right'):
            raise ControlFailure('side 必须为 left 或 right')
        offset = 0 if side == 'left' else 7
        candidate = finite_positions(candidate_rad, 7, '单臂候选')
        for index, value in enumerate(candidate, start=offset):
            lower, upper = EXECUTION_LIMITS_RAD[index]
            start = self.arm_rad[index]
            if not lower <= start <= upper or not lower <= value <= upper:
                raise ControlFailure(f'{ARM_NAMES[index]}: start={start}, target={value}, '
                                     f'执行范围=[{lower}, {upper}] rad')
        complete = list(self.arm_rad)
        complete[offset:offset + 7] = candidate
        return CapturedTarget(tuple(complete), self.hand_openings)
