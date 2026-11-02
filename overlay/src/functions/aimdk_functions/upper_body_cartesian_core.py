"""完整目标契约与TCP端点、直线纯IK规划；不导入ROS或发布命令。"""

from dataclasses import dataclass
import math
import time

import numpy as np

from .dual_arm_control_core import ARM_NAMES, ControlFailure, interpolate
from .upper_body_kinematics.adapter import ArmKinematics
from .upper_body_kinematics.parameters import EXECUTION_LIMITS_RAD
from .upper_body_kinematics.vendor.x2_arm_model import log3


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


# 沿用既有单次关节增量实验的10°幅度上限，逐关节检查，不解释为厂商额定范围。
MAX_JOINT_DELTA_RAD = math.radians(10.0)


def plan_translation(captured, side, axis, delta_mm, *, arm=None, translation_mm=None):
    """从本次完整采样规划单臂在torso坐标轴上的平移端点，返回完整目标及诊断字典。

    captured为CapturedTarget，side为left/right，axis为x/y/z，delta_mm为有符号毫米增量。
    可传入同侧腕原点ArmKinematics复用模型；否则在本函数内构造。
    translation_mm可指定torso系XYZ毫米向量，优先于单轴输入。零增量保留当前关节；其他目标以当前SEW作局部track，不进行全局搜索或投影。
    无解、输入无效、超限或任一关节总变化超过既有10°约定时抛出ControlFailure。
    返回目标供后续关节插值MoveJ使用，不保证实际TCP沿笛卡尔直线移动。
    """
    # translation_mm为torso系XYZ位移；未提供时沿用axis/delta_mm。
    began = time.perf_counter()
    if not isinstance(captured, CapturedTarget):
        raise ControlFailure('规划起点必须为本次采样的CapturedTarget')
    if side not in ('left', 'right') or axis not in ('x', 'y', 'z'):
        raise ControlFailure('side须为left/right，axis须为x/y/z')
    try:
        delta_mm = float(delta_mm)
    except (TypeError, ValueError, OverflowError) as error:
        raise ControlFailure('delta_mm必须为有限毫米数值') from error
    if not math.isfinite(delta_mm):
        raise ControlFailure('delta_mm必须为有限毫米数值')
    if arm is None:
        arm = ArmKinematics(side, tcp_mode='none')
    if arm.side != side or arm.tool['mode'] != 'none':
        raise ControlFailure('最小平移规划要求同侧腕原点none模型')
    offset = 0 if side == 'left' else 7
    seed = np.asarray(captured.arm_rad[offset:offset + 7], dtype=float)
    # 先校验所选臂完整起点；未选臂按既有契约原样保持，不因其范围而裁剪。
    captured.for_arm(side, seed)
    start_position, start_rotation = arm.forward(seed)
    requested_position = start_position.copy()
    displacement = np.zeros(3)
    if translation_mm is None:
        displacement[('x', 'y', 'z').index(axis)] = delta_mm
    else:
        displacement = np.asarray(finite_positions(translation_mm, 3, 'TCP位移mm'))
    requested_position += displacement / 1000.0
    requested_rotation = start_rotation.copy()
    seed_psi = float(arm.solver.sew_angle(seed))
    solve_began = time.perf_counter()
    if not np.any(displacement):
        # FK目标没有改变，当前关节本身就是一致解，不调用候选评分重排肩腕。
        candidate = seed.copy()
        psi = seed_psi
        branch = None
    else:
        try:
            solution = arm.track(requested_position, requested_rotation, seed,
                                 previous_psi=seed_psi, previous_branch=None)
        except ValueError as error:
            raise ControlFailure(f'{side}局部IK候选被拒绝：{error}') from error
        if solution is None:
            raise ControlFailure(f'{side}局部IK无解；不投影、不转全局搜索，请减小位移或核对起点')
        candidate = np.asarray(finite_positions(solution.q, 7, '局部IK候选'), dtype=float)
        psi = float(solution.psi)
        branch = [int(solution.elbow_branch), int(solution.shoulder_branch),
                  int(solution.wrist_branch)]
    solve_seconds = time.perf_counter() - solve_began
    target = captured.for_arm(side, candidate)
    delta = candidate - seed
    largest = float(np.max(np.abs(delta)))
    if largest > MAX_JOINT_DELTA_RAD:
        index = int(np.argmax(np.abs(delta)))
        raise ControlFailure(f'{arm.model.joint_names[index]}总变化'
                             f'{math.degrees(delta[index]):+.3f}°超过既有单次10°约定；'
                             '候选不裁剪，请减小请求位移')
    target_position, target_rotation = arm.forward(candidate)
    position_residual = float(np.linalg.norm(target_position - requested_position))
    rotation_residual = float(np.linalg.norm(log3(requested_rotation @ target_rotation.T)))
    if (not np.isfinite(target_position).all() or not np.isfinite(target_rotation).all()
            or not math.isfinite(position_residual) or not math.isfinite(rotation_residual)):
        raise ControlFailure('局部IK回算产生非有限位姿或残差')
    report = dict(
        side=side, axis=axis, delta_mm=delta_mm, translation_mm=displacement.tolist(), base_frame='torso_link', tcp_mode='none',
        start_arm_rad=list(captured.arm_rad), target_arm_rad=list(target.arm_rad),
        arm_names=list(ARM_NAMES), joint_names=list(arm.model.joint_names),
        full_joint_delta_rad=[end - start for start, end in zip(captured.arm_rad, target.arm_rad)],
        hand_openings=list(target.hand_openings), seed_rad=seed.tolist(),
        candidate_rad=candidate.tolist(), joint_delta_rad=delta.tolist(),
        joint_delta_deg=np.degrees(delta).tolist(), max_joint_delta_deg=math.degrees(largest),
        start_position_m=start_position.tolist(), start_rotation_matrix=start_rotation.tolist(),
        requested_position_m=requested_position.tolist(),
        requested_rotation_matrix=requested_rotation.tolist(),
        target_position_m=target_position.tolist(), target_rotation_matrix=target_rotation.tolist(),
        position_residual_m=position_residual, rotation_residual_rad=rotation_residual,
        seed_sew_rad=seed_psi, sew_rad=psi, branch=branch, branch_prev=None, branch_locked=False,
        solve_seconds=solve_seconds, planning_seconds=time.perf_counter() - began,
        seed_preserved=not np.any(displacement), clamp_reach=False, fallback=False,
        compensation_enabled=False, path_type='joint_quintic_movej',
        residual_acceptance_threshold=None,
    )
    return target, report


@dataclass(frozen=True)
class LineTrajectory:
    """保存从实际采样出发的20ms时间网格、完整双臂目标及逐点数学诊断。"""

    times: tuple
    targets: tuple
    points: tuple

    def at(self, elapsed):
        """按实际经过秒数返回相邻规划点间线性关节目标，超出时域时保持端点。

        规划点位于笛卡尔直线上，点间关节插值仅近似直线；不在线调用IK。
        """
        if elapsed <= 0:
            return self.targets[0].arm_rad
        if elapsed >= self.times[-1]:
            return self.targets[-1].arm_rad
        index = min(int(elapsed / (self.times[1] - self.times[0])), len(self.times) - 2)
        fraction = (elapsed - self.times[index]) / (self.times[index + 1] - self.times[index])
        return tuple(a + fraction * (b - a) for a, b in zip(
            self.targets[index].arm_rad, self.targets[index + 1].arm_rad))


def plan_line(captured, side, axis, delta_mm, duration, *, arm=None, translation_mm=None):
    """返回完整目标、汇总报告和直线轨迹；距离为mm、时长为秒。

    translation_mm可指定torso系XYZ毫米向量，优先于单轴输入。

    首点保留采样角；后续点以前一点关节/SEW求局部IK，首个非零点确定分支后锁定。
    无解、非有限、越界、分支改变或任一关节相对起点超过既有10度时抛出ControlFailure。
    不以未经实测的残差阈值阻断；记录所有点残差及相邻关节变化供审阅。
    """
    began = time.perf_counter()
    if not math.isfinite(duration) or not 5 <= duration <= 10:
        raise ControlFailure('直线轨迹时长须为5..10秒')
    arm = arm if arm is not None else ArmKinematics(side, tcp_mode='none')
    _, report = plan_translation(captured, side, axis, 0., arm=arm)
    # 复用XYZ有限值检查；直线路径仍从实际采样关节开始。
    if translation_mm is None:
        displacement = np.zeros(3)
        displacement[('x', 'y', 'z').index(axis)] = delta_mm
    else:
        displacement = np.asarray(finite_positions(translation_mm, 3, 'TCP位移mm'))
    if not math.isfinite(delta_mm):
        raise ControlFailure('直线位移须为有限毫米数值')
    offset = 0 if side == 'left' else 7
    seed = np.asarray(captured.arm_rad[offset:offset + 7])
    position = np.asarray(report['start_position_m'])
    rotation = np.asarray(report['start_rotation_matrix'])
    previous, psi, branch = seed.copy(), report['seed_sew_rad'], None
    count = int(math.ceil(duration / .02))
    times = tuple(duration * i / count for i in range(count + 1))
    targets, points = [], []
    solve_began = time.perf_counter()
    for index, seconds in enumerate(times):
        progress = interpolate((0.,), (1.,), seconds, duration)[0]
        requested = position.copy()
        requested += displacement * progress / 1000.
        current_branch = branch
        if index == 0 or not np.any(displacement):
            candidate = seed.copy()
        else:
            try:
                solution = arm.track(requested, rotation, previous,
                                     previous_psi=psi, previous_branch=branch)
                if solution is None:
                    raise ValueError('局部IK无解')
                candidate = np.asarray(solution.q, dtype=float)
                current_branch = (int(solution.elbow_branch), int(solution.shoulder_branch),
                                  int(solution.wrist_branch))
                if branch is not None and current_branch != branch:
                    raise ValueError('解析分支改变')
                psi = float(solution.psi)
                if not math.isfinite(psi):
                    raise ValueError('SEW非有限')
            except ValueError as error:
                raise ControlFailure(f'直线路径第{index}点 t={seconds:.3f}s 被拒绝：{error}') from error
        target = captured.for_arm(side, candidate)
        excursion = float(np.max(np.abs(candidate - seed)))
        step = float(np.max(np.abs(candidate - previous)))
        if excursion > MAX_JOINT_DELTA_RAD or step > MAX_JOINT_DELTA_RAD:
            raise ControlFailure(f'直线路径第{index}点总变化或相邻变化超过既有10度约定')
        actual, actual_rotation = arm.forward(candidate)
        residual = float(np.linalg.norm(actual - requested))
        angular = float(np.linalg.norm(log3(rotation @ actual_rotation.T)))
        if not math.isfinite(residual) or not math.isfinite(angular):
            raise ControlFailure(f'直线路径第{index}点FK残差非有限')
        points.append(dict(index=index, time_s=seconds, progress=progress,
                           requested_position_m=requested.tolist(), target_position_m=actual.tolist(),
                           target_arm_rad=list(target.arm_rad), position_residual_m=residual,
                           rotation_residual_rad=angular, sew_rad=psi, branch=current_branch,
                           max_joint_delta_deg=math.degrees(excursion),
                           max_adjacent_joint_delta_deg=math.degrees(step)))
        targets.append(target)
        previous, branch = candidate.copy(), current_branch
    final = points[-1]
    report.update(delta_mm=delta_mm, translation_mm=displacement.tolist(), target_arm_rad=list(targets[-1].arm_rad),
                  requested_position_m=final['requested_position_m'],
                  target_position_m=final['target_position_m'],
                  position_residual_m=final['position_residual_m'],
                  rotation_residual_rad=final['rotation_residual_rad'],
                  max_joint_delta_deg=max(p['max_joint_delta_deg'] for p in points),
                  max_adjacent_joint_delta_deg=max(p['max_adjacent_joint_delta_deg'] for p in points),
                  max_path_position_residual_m=max(p['position_residual_m'] for p in points),
                  max_path_rotation_residual_rad=max(p['rotation_residual_rad'] for p in points),
                  candidate_rad=previous.tolist(), sew_rad=psi, branch=branch,
                  branch_locked=branch is not None, seed_preserved=not np.any(displacement),
                  path_type='cartesian_line_discrete_ik', point_count=len(points),
                  duration_s=duration, solve_seconds=time.perf_counter() - solve_began,
                  planning_seconds=time.perf_counter() - began)
    # 覆盖零位移模板中的端点角度与旋转，避免日志沿用模板的零增量。
    report['target_rotation_matrix'] = actual_rotation.tolist()
    report['full_joint_delta_rad'] = [b - a for a, b in zip(captured.arm_rad, targets[-1].arm_rad)]
    report['joint_delta_rad'] = (previous - seed).tolist()
    report['joint_delta_deg'] = np.degrees(previous - seed).tolist()
    return targets[-1], report, LineTrajectory(times, tuple(targets), tuple(points))
