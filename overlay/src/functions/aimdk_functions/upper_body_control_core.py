"""统一上肢目标规划与双臂调度；无ROS导入，统一采用参考项目及肩roll调整限位。"""

from dataclasses import dataclass
import math

from .dual_arm_control_core import (
    ARM_NAMES, JOINT_TAGS, ControlFailure, gripper_target, interpolate, joint_target,
)
from .upper_body_cartesian_core import CapturedTarget
from .upper_body_kinematics.parameters import EXECUTION_LIMITS_RAD
from .upper_body_move_core import plan_translation
from .upper_body_line_core import plan_line


def selected_sides(side):
    """将left/right/both展开为按SDK槽位排序的侧别元组；无效侧别抛出ControlFailure。"""
    if side not in ('left', 'right', 'both'):
        raise ControlFailure('side须为left/right/both')
    return ('left', 'right') if side == 'both' else (side,)


def validate_target(captured, target, options):
    """检查实际参与控制的关节起点和目标范围，其余保持采样；越界抛出ControlFailure。

    joint关闭补偿时只检查所选关节，tcp及开启补偿时检查所选臂全部关节。
    gripper不修改手臂；完整目标由CapturedTarget校验有限值和夹爪开度。
    """
    active = set()
    if options.action != 'gripper':
        for side in selected_sides(options.side):
            offset = 0 if side == 'left' else 7
            indices = (range(7) if options.action == 'tcp' or options.compensation
                       else (JOINT_TAGS.index(options.tag),))
            active.update(offset + index for index in indices)
    for index, (start, value) in enumerate(zip(captured.arm_rad, target.arm_rad)):
        if index not in active:
            if value != start:
                raise ControlFailure(f'{ARM_NAMES[index]}未选中，须保持采样目标')
            continue
        lower, upper = EXECUTION_LIMITS_RAD[index]
        if not lower <= start <= upper or not lower <= value <= upper:
            raise ControlFailure(f'{ARM_NAMES[index]} start={start}, target={value}, '
                                 f'执行范围=[{lower}, {upper}] rad')


@dataclass(frozen=True)
class MotionTrajectory:
    """同一时长内的完整双臂和夹爪目标；line按预先接受的双侧路径插值。"""

    captured: CapturedTarget
    target: CapturedTarget
    duration: float
    kind: str
    lines: dict

    def at(self, elapsed):
        """按经过秒数返回完整目标；夹爪首帧更新，关节/端点五次插值，直线查表。"""
        if self.kind == 'gripper':
            return self.target
        if self.kind != 'line':
            return CapturedTarget(interpolate(self.captured.arm_rad, self.target.arm_rad,
                                              elapsed, self.duration), self.target.hand_openings)
        positions = list(self.captured.arm_rad)
        for side, line in self.lines.items():
            offset = 0 if side == 'left' else 7
            positions[offset:offset + 7] = line.at(elapsed)[offset:offset + 7]
        return CapturedTarget(tuple(positions), self.target.hand_openings)


def plan_motion(captured, options, arms):
    """用同一次采样规划全部所选侧，返回完整轨迹及分侧报告；任一失败整体拒绝。

    options由入口校验，arms为侧别到ArmKinematics的映射；IK在发布前全部完成。
    """
    lines, reports = {}, {}
    if options.action == 'gripper':
        openings = list(captured.hand_openings)
        for side, opening in options.requests.items():
            openings = list(gripper_target(openings, side, opening))
        target = CapturedTarget(captured.arm_rad, tuple(openings))
        kind = 'gripper'
    elif options.action == 'joint':
        offsets = tuple(math.radians(options.requests[side]) if side in options.requests else None
                        for side in ('left', 'right'))
        target = CapturedTarget(joint_target(captured.arm_rad, options.tag, offsets),
                                captured.hand_openings)
        kind = 'joint'
    else:
        positions = list(captured.arm_rad)
        kind = options.path
        for side, displacement in options.requests.items():
            if kind == 'line':
                candidate, report, line = plan_line(captured, side, 'x', 0., options.duration,
                                                    arm=arms[side], translation_mm=displacement)
                lines[side] = line
            else:
                candidate, report = plan_translation(captured, side, 'x', 0., arm=arms[side],
                                                      translation_mm=displacement)
            offset = 0 if side == 'left' else 7
            positions[offset:offset + 7] = candidate.arm_rad[offset:offset + 7]
            reports[side] = report
        target = CapturedTarget(tuple(positions), captured.hand_openings)
    validate_target(captured, target, options)
    return MotionTrajectory(captured, target, options.duration, kind, lines), reports


class ArmCompensationGroup:
    """所选臂独立计算补偿，从同一理想目标取各侧切片后合并，保持未选目标。"""

    def __init__(self, calculators):
        """保存侧别到PositionCompensation的映射；至少包含一侧，由入口构造。"""
        self.calculators = calculators

    def apply(self, captured, ideal, values, blend=1.):
        """使用同一组有效IMU/腰反馈返回完整补偿目标和分侧诊断；任一越界整体拒绝。"""
        positions, details = list(ideal.arm_rad), {}
        for side, calculator in self.calculators.items():
            candidate, report = calculator.apply(captured, ideal, values, blend)
            offset = 0 if side == 'left' else 7
            positions[offset:offset + 7] = candidate.arm_rad[offset:offset + 7]
            details[side] = report
        target = CapturedTarget(tuple(positions), ideal.hand_openings)
        return target, dict(blend=blend, arms=details, ideal_arm_rad=list(ideal.arm_rad),
                            sent_arm_rad=list(target.arm_rad))
