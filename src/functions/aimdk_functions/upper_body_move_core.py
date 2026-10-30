"""单臂平移端点的最小局部IK规划；无ROS导入，不发送命令、不生成笛卡尔路径。"""

import math
import time

import numpy as np

from .dual_arm_control_core import ARM_NAMES, ControlFailure
from .upper_body_cartesian_core import CapturedTarget, finite_positions
from .upper_body_kinematics.adapter import ArmKinematics
from .upper_body_kinematics.vendor.x2_arm_model import log3


# 沿用既有单次关节增量实验的10°幅度上限，逐关节检查，不解释为厂商额定范围。
MAX_JOINT_DELTA_RAD = math.radians(10.0)


def plan_translation(captured, side, axis, delta_mm, *, arm=None):
    """从本次完整采样规划单臂在torso坐标轴上的平移端点，返回完整目标及诊断字典。

    captured为CapturedTarget，side为left/right，axis为x/y/z，delta_mm为有符号毫米增量。
    可传入同侧腕原点ArmKinematics复用模型；否则在本函数内构造。
    零增量保留当前关节；其他目标以当前SEW作局部track，不进行全局搜索或投影。
    无解、输入无效、超限或任一关节总变化超过既有10°约定时抛出ControlFailure。
    返回目标供后续关节插值MoveJ使用，不保证实际TCP沿笛卡尔直线移动。
    """
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
    requested_position[('x', 'y', 'z').index(axis)] += delta_mm / 1000.0
    requested_rotation = start_rotation.copy()
    seed_psi = float(arm.solver.sew_angle(seed))
    solve_began = time.perf_counter()
    if delta_mm == 0.0:
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
        side=side, axis=axis, delta_mm=delta_mm, base_frame='torso_link', tcp_mode='none',
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
        seed_preserved=delta_mm == 0.0, clamp_reach=False, fallback=False,
        compensation_enabled=False, path_type='joint_quintic_movej',
        residual_acceptance_threshold=None,
    )
    return target, report
