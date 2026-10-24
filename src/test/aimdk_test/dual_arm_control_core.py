"""双臂肩 pitch 小幅动作的纯计算与反馈检查，不导入 ROS。"""

import math

from aimdk_test.arm_test_core import ARM_NAMES, URS, decode_mc
from aimdk_test.head_test_core import TestFailure


SHOULDER_INDICES = (0, 7)
# 2026-10-24 用户实机确认：左右肩 pitch 正增量均向下，上抬使用负增量；见工作区 test.md。
SHOULDER_UP_SIGNS = (-1, -1)
# 双臂跟踪、保持及到位共用 3 度容差，避免各阶段遗留更严格的独立门槛。
ARM_POSITION_TOLERANCE = math.radians(3)


def decode_urs_mc(msg, received, serial):
    """保留 MC 原始源时间并要求站立、URS/RUNNING；状态变化时抛出 TestFailure。"""
    value = decode_mc(msg, received, serial)
    action, status = value.data['action'], value.data['action_status']
    if action != URS or status != 100:
        raise TestFailure(f'MC 反馈不再是 URS / RUNNING：{action} / status={status}')
    return value


def shoulder_target(start, offsets):
    """返回完整 14 关节目标；offsets 是左右肩 pitch 的 rad 增量。

    数据无效、增量超过 5 度或肩 pitch 路径超出保守范围时抛出 TestFailure。
    """
    if (len(start) != 14 or len(offsets) != 2
            or not all(math.isfinite(v) for v in (*start, *offsets))
            or any(abs(v) > math.radians(5) for v in offsets)):
        raise TestFailure('需要 14 个有限关节位置及两个不超过 5 度的肩 pitch 增量')
    target = list(start)
    for index, offset in zip(SHOULDER_INDICES, offsets):
        target[index] += offset
        # 来自 SDK motocontrol.py 的 pitch 范围，向内保留 1 度；非碰撞检查。
        if not all(-3.08 + math.radians(1) <= value <= 2.04 - math.radians(1)
                   for value in (start[index], target[index])):
            raise TestFailure(f'{ARM_NAMES[index]} 起点或目标超出保守 pitch 范围')
    return tuple(target)


def interpolate(origin, target, elapsed, duration):
    """以秒为时间单位，返回起终点速度、加速度为零的五次插值 rad 目标。"""
    phase = max(0., min(1., elapsed / duration))
    blend = phase ** 3 * (10 - 15 * phase + 6 * phase * phase)
    return tuple(a + (b - a) * blend for a, b in zip(origin, target))


def check_arm(actual, target, tolerance=ARM_POSITION_TOLERANCE):
    """检查全部 14 个关节与 rad 目标的误差，超过 tolerance 时抛出 TestFailure。"""
    if (len(actual) != 14 or len(target) != 14
            or not all(math.isfinite(v) for v in (*actual, *target))):
        raise TestFailure('双臂反馈和目标必须是 14 个有限位置')
    for name, value, expected in zip(ARM_NAMES, actual, target):
        if abs(value - expected) > tolerance:
            raise TestFailure(f'{name} 位置误差 {math.degrees(value - expected):+.3f} 度，'
                              f'超过 ±{math.degrees(tolerance):g} 度')
