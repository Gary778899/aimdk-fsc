"""步骤三的无 ROS 数学诊断：实际快照往返检查与合成边界/失败用例。"""

import time

import numpy as np

from .dual_arm_control_core import ControlFailure
from .upper_body_cartesian_core import CapturedTarget
from .upper_body_kinematics.vendor.x2_arm_model import log3


def evaluate_pose(arm, seed, position, rotation):
    """以原始位姿和种子求解并独立回算，返回可序列化诊断；不设置实测精度门槛。

    arm 为 ArmKinematics；输入为7维rad、3维m及3×3旋转矩阵。
    非有限值、越界、无解均记录明确状态；候选只表示数学返回值，不表示定位验收。
    """
    seed = np.asarray(seed, dtype=float)
    position = np.asarray(position, dtype=float)
    rotation = np.asarray(rotation, dtype=float)
    result = dict(seed_rad=seed.tolist(), requested_position_m=position.tolist(),
                  requested_rotation_matrix=rotation.tolist(), projected=False,
                  orientation_relaxed=False, compensation_enabled=False)
    began = time.perf_counter()
    try:
        solution = arm.solve_initial(position, rotation, seed)
    except ValueError as error:
        result.update(status='rejected', reason=str(error),
                      solve_seconds=time.perf_counter() - began)
    else:
        result['solve_seconds'] = time.perf_counter() - began
        if solution is None:
            result.update(status='no_solution')
        else:
            recovered_position, recovered_rotation = arm.forward(solution.q)
            delta = solution.q - seed
            position_error = float(np.linalg.norm(recovered_position - position))
            rotation_error = float(np.linalg.norm(log3(rotation @ recovered_rotation.T)))
            candidate_finite = bool(np.isfinite(solution.q).all()
                                    and np.isfinite(recovered_position).all()
                                    and np.isfinite(recovered_rotation).all()
                                    and np.isfinite(position_error)
                                    and np.isfinite(rotation_error))
            result.update(
                status='candidate_returned' if candidate_finite else 'invalid_candidate',
                candidate_rad=solution.q.tolist(),
                recovered_position_m=recovered_position.tolist(),
                recovered_rotation_matrix=recovered_rotation.tolist(),
                position_residual_m=position_error, rotation_residual_rad=rotation_error,
                seed_distance_rad=float(np.linalg.norm(delta)),
                seed_max_abs_delta_rad=float(np.max(np.abs(delta))),
                seed_wrapped_distance_rad=float(np.linalg.norm(np.arctan2(np.sin(delta), np.cos(delta)))),
                seed_sew_rad=float(arm.solver.sew_angle(seed)), sew_rad=float(solution.psi),
                branch=[int(solution.elbow_branch), int(solution.shoulder_branch),
                        int(solution.wrist_branch)], solver_in_limits=bool(solution.in_limits),
                solver_position_residual_m=float(solution.pos_error),
                solver_rotation_residual_rad=float(solution.rot_error),
            )
    result['evaluation_seconds'] = time.perf_counter() - began
    return result


def round_trip(arm, joints):
    """对实际7关节rad快照先做FK，再以该位姿和原快照做IK；越界反馈仍保留FK。

    返回完整输入、执行范围外关节及求解诊断。种子越界明确拒绝，不裁剪后重新求解。
    """
    joints = np.asarray(joints, dtype=float)
    position, rotation = arm.forward(joints)
    outside = np.flatnonzero((joints < arm.limits[:, 0]) | (joints > arm.limits[:, 1]))
    result = evaluate_pose(arm, joints, position, rotation)
    result['outside_execution_limits'] = [
        dict(name=arm.model.joint_names[index], position_rad=float(joints[index]),
             limits_rad=arm.limits[index].tolist()) for index in outside
    ]
    return result


def diagnostic_cases(arm):
    """生成纯数学用例，逐个返回(case, expectation, result)，不发送任何目标。

    边界采用精确限位及相邻的内侧浮点数；越界采用相邻外侧浮点数。
    远目标超过肩腕几何最大距离、工具偏置和额外1m；1m仅用于构造不可达输入。
    边界残差/无解只记录，越界种子必须拒绝，远目标必须返回无解或拒绝。
    """
    middle = arm.limits.mean(axis=1)
    position, rotation = arm.forward(middle)
    for index, name in enumerate(arm.model.joint_names):
        for endpoint, direction in ((0, -np.inf), (1, np.inf)):
            bound = arm.limits[index, endpoint]
            for kind, value in (('boundary', bound),
                                ('inside_adjacent', np.nextafter(bound, middle[index]))):
                seed = middle.copy()
                seed[index] = value
                yield f'{name}:{endpoint}:{kind}', 'observe', round_trip(arm, seed)
            seed = middle.copy()
            seed[index] = np.nextafter(bound, direction)
            yield (f'{name}:{endpoint}:outside_seed', 'rejected',
                   evaluate_pose(arm, seed, position, rotation))
            full = np.zeros(14)
            offset = 0 if arm.side == 'left' else 7
            full[offset:offset + 7] = middle
            sampled = CapturedTarget(tuple(full), (0.5, 0.5))
            began = time.perf_counter()
            target_result = dict(requested_candidate_rad=seed.tolist(), clipped=False)
            try:
                accepted = sampled.for_arm(arm.side, seed)
            except ControlFailure as error:
                target_result.update(status='rejected', reason=str(error))
            else:
                target_result.update(status='candidate_returned', arm_rad=list(accepted.arm_rad))
            target_result['evaluation_seconds'] = time.perf_counter() - began
            yield f'{name}:{endpoint}:outside_joint_target', 'rejected', target_result
    invalid = middle.copy()
    invalid[0] = np.nan
    yield 'nonfinite_seed', 'rejected', evaluate_pose(arm, invalid, position, rotation)
    distance = (arm.solver.reach_max + np.linalg.norm(arm.model.tcp_offset)
                + np.linalg.norm(arm.model.wrist_center_local) + 1.0)
    unreachable = arm.solver.shoulder + np.array([distance, 0., 0.])
    yield 'unreachable_position', 'no_solution_or_rejected', evaluate_pose(
        arm, middle, unreachable, rotation)
