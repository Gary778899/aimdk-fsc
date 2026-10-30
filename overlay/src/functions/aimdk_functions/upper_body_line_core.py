"""无ROS的单臂直线离散规划；完整路径接受后才能交给执行层。"""

from dataclasses import dataclass
import math
import time

import numpy as np

from .dual_arm_control_core import ControlFailure, interpolate
from .upper_body_kinematics.adapter import ArmKinematics
from .upper_body_kinematics.vendor.x2_arm_model import log3
from .upper_body_move_core import MAX_JOINT_DELTA_RAD, plan_translation
from .upper_body_cartesian_core import finite_positions


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
