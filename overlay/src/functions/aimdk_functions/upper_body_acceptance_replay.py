"""从保存的实际快照回放平移、旋转和组合轨迹；无 ROS 导入，不发送运动。"""

import math
import time

import numpy as np

from .dual_arm_control_core import interpolate
from .upper_body_kinematics.adapter import ArmKinematics
from .upper_body_kinematics.vendor.x2_arm_model import log3
from .upper_body_move_core import MAX_JOINT_DELTA_RAD


def distribution(values):
    """返回有限观测值的数量、最小值、中位数、95分位和最大值；空输入返回空字典。"""
    data = np.asarray(values, dtype=float)
    if not len(data):
        return {}
    return dict(count=len(data), min=float(data.min()), median=float(np.median(data)),
                p95=float(np.percentile(data, 95)), max=float(data.max()))


def replay(snapshot, side, kind, translation_mm, rotation_deg, path):
    """从快照生成5秒、251点轨迹并逐帧保存；返回完整性及指标。

    平移沿torso X，旋转绕初始TCP局部Z；组合同时应用二者。旋转使用R0·Exp(a·Log(R0ᵀR1))。
    初解仅作诊断，track从实际种子开始；候选接受前检查限位、分支和既有10度变化上限。
    失败保留请求、失败帧及最后已接受状态；不设新的精度或耗时门槛。
    """
    arm = ArmKinematics(side, tcp_mode='none')
    offset = 0 if side == 'left' else 7
    seed = np.asarray(snapshot['streams']['arm']['positions'][offset:offset + 7])
    position, rotation = arm.forward(seed)
    angle = math.radians(rotation_deg) if kind != 'translation' else 0.
    displacement = np.array([translation_mm / 1000. if kind != 'rotation' else 0., 0., 0.])
    previous, psi, branch = seed.copy(), float(arm.solver.sew_angle(seed)), None
    samples, failure = [], None
    with path.open('x', encoding='utf-8') as stream:
        import json

        stream.write(json.dumps(dict(event='input', snapshot=snapshot, side=side, kind=kind,
                                     translation_mm=translation_mm, rotation_deg=rotation_deg,
                                     tcp='none', clamp_reach=False, fallback=False)) + '\n')
        began = time.perf_counter()
        try:
            initial = arm.solve_initial(position, rotation, seed)
            if initial is None:
                raise ValueError('初始位姿IK无解')
            stream.write(json.dumps(dict(event='initial_solution', q=initial.q.tolist(),
                                         solve_seconds=time.perf_counter()-began)) + '\n')
            for index in range(251):
                progress = interpolate((0.,), (1.,), index * .02, 5.)[0]
                theta = progress * angle
                c, s = math.cos(theta), math.sin(theta)
                requested_rotation = rotation @ np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
                requested_position = position + progress * displacement
                row = dict(event='frame', index=index, time_s=index * .02,
                           requested_position_m=requested_position.tolist(),
                           requested_rotation_matrix=requested_rotation.tolist(),
                           last_accepted_rad=previous.tolist(), previous_sew_rad=psi,
                           previous_branch=branch)
                began = time.perf_counter()
                try:
                    candidate, next_psi, next_branch = seed.copy(), psi, branch
                    if index:
                        solution = arm.track(requested_position, requested_rotation, previous,
                                             previous_psi=psi, previous_branch=branch)
                        if solution is None:
                            raise ValueError('局部IK无解')
                        candidate = np.asarray(solution.q)
                        next_psi = float(solution.psi)
                        next_branch = [int(solution.elbow_branch), int(solution.shoulder_branch),
                                       int(solution.wrist_branch)]
                        if branch is not None and next_branch != branch:
                            raise ValueError('解析分支改变')
                    step = float(np.max(np.abs(candidate - previous)))
                    excursion = float(np.max(np.abs(candidate - seed)))
                    if (not np.isfinite(candidate).all() or not math.isfinite(next_psi)
                            or np.any(candidate < arm.limits[:, 0]) or np.any(candidate > arm.limits[:, 1])):
                        raise ValueError('候选非有限或越限')
                    if max(step, excursion) > MAX_JOINT_DELTA_RAD:
                        raise ValueError('相邻步长或相对起点变化超过既有10度约定')
                    actual_position, actual_rotation = arm.forward(candidate)
                    residual = float(np.linalg.norm(actual_position - requested_position))
                    angular = float(np.linalg.norm(log3(requested_rotation @ actual_rotation.T)))
                    if not math.isfinite(residual) or not math.isfinite(angular):
                        raise ValueError('FK残差非有限')
                    row.update(status='accepted', candidate_rad=candidate.tolist(), sew_rad=next_psi,
                               branch=next_branch, step_deg=math.degrees(step),
                               excursion_deg=math.degrees(excursion), position_residual_m=residual,
                               rotation_residual_rad=angular)
                    previous, psi, branch = candidate.copy(), next_psi, next_branch
                except ValueError as error:
                    row.update(status='failed', reason=str(error))
                    failure = row
                row['calculation_seconds'] = time.perf_counter() - began
                stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
                stream.flush()
                if failure:
                    break
                samples.append(row)
        except ValueError as error:
            failure = dict(event='initial_failure', reason=str(error), last_accepted_rad=previous.tolist())
            stream.write(json.dumps(failure, ensure_ascii=False) + '\n')
    return dict(complete=len(samples) == 251, accepted_frames=len(samples), failure=failure,
                metrics={key: distribution([row[key] for row in samples]) for key in (
                    'calculation_seconds', 'step_deg', 'position_residual_m', 'rotation_residual_rad')},
                log=str(path))
