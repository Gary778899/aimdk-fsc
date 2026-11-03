"""双侧逐帧IK与已接受目标存储；不导入ROS，不发送运动命令。"""

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import threading
import time

import numpy as np

from remote_control.frames import Pose, TargetFrame

from .dual_arm_control_core import ControlFailure
from .upper_body_cartesian_core import CapturedTarget, MAX_JOINT_DELTA_RAD, finite_positions
from .upper_body_kinematics.adapter import ArmKinematics
from .upper_body_kinematics.parameters import MODEL_SHA256, model_path
from .upper_body_kinematics.vendor.x2_arm_model import log3


SIDES = ('left', 'right')


class OnlineIKFailure(ControlFailure):
    """携带失败帧与已完成候选诊断；失败不推进任何双侧状态。"""

    def __init__(self, message, report):
        """保存拒绝原因message和可序列化report供入口记录。"""
        super().__init__(message)
        self.report = report


@dataclass(frozen=True)
class ArmTrackingState:
    """单侧上一已接受的理想关节rad、SEW及解析分支。"""

    joints: tuple
    sew: float
    branch: object


@dataclass(frozen=True)
class IKProposal:
    """未提交的完整双侧候选；owner和revision用于拒绝过期提交。"""

    owner: object
    revision: int
    input_seq: int
    target: CapturedTarget
    states: tuple
    report: dict
    origin: CapturedTarget


@dataclass(frozen=True)
class AcceptedTarget:
    """保存目标对应的输入序号及本机接受时间，用于后续周期发布关联。"""

    target: CapturedTarget
    input_seq: object
    accepted_monotonic_s: float


class OnlineTargetStore:
    """以锁保护完整已接受目标快照；无新帧时读取同一目标与原始时间。"""

    def __init__(self, initial):
        """以CapturedTarget初始化；初始快照input_seq为空，不代表已收到XR输入。"""
        self._lock = threading.Lock()
        self._value = AcceptedTarget(initial, None, time.monotonic())

    def put(self, target, input_seq):
        """原子替换完整目标，返回带接受时间的快照；不重写输入源时间。"""
        value = AcceptedTarget(target, input_seq, time.monotonic())
        with self._lock:
            self._value = value
        return value

    def get(self):
        """返回不可变快照；读取操作不刷新目标的接受时间。"""
        with self._lock:
            return self._value


class DualArmOnlineIK:
    """先求解并校验全部候选，再显式提交双侧状态；调用方串行使用。"""

    def __init__(self, captured, arms=None):
        """使用完整关节起点captured和gripper模型初始化理想状态，非法输入抛出异常。

        arms可注入同侧ArmKinematics用于离线验证；默认加载包内左右gripper模型。
        首次Grip可事务式更新所选侧命令参考；之后10°实验范围不随Grip重按扩大。
        """
        if not isinstance(captured, CapturedTarget):
            raise ControlFailure('在线IK起点须为CapturedTarget')
        self.arms = arms if arms is not None else {
            side: ArmKinematics(side, tcp_mode='gripper') for side in SIDES}
        self.origin = captured
        self.accepted = captured
        self.states = {}
        self.revision = 0
        self.last_input_seq = None
        self.command_reference_sides = set()
        self.store = OnlineTargetStore(captured)
        for index, side in enumerate(SIDES):
            arm = self.arms[side]
            if arm.side != side or arm.tool['mode'] != 'gripper':
                raise ControlFailure('在线IK要求同侧gripper TCP模型')
            joints = captured.arm_rad[index * 7:index * 7 + 7]
            captured.for_arm(side, joints)
            sew = float(arm.solver.sew_angle(np.asarray(joints)))
            if not math.isfinite(sew):
                raise ControlFailure(f'{side}起点SEW非有限')
            self.states[side] = ArmTrackingState(joints, sew, None)

    def tcp_poses(self, target=None):
        """对完整target计算左右torso_link下TCP位姿；省略时使用已接受理想目标。"""
        target = self.accepted if target is None else target
        result = {}
        for index, side in enumerate(SIDES):
            position, rotation = self.arms[side].forward(target.arm_rad[index * 7:index * 7 + 7])
            result[side] = Pose(position, rotation)
        return result

    def describe(self):
        """返回实际模型、完整工具配置及源目标，供入口记录启动元数据。"""
        preset = Path(__file__).resolve().parent / 'upper_body_kinematics/vendor/assets/tcp_presets.json'
        return dict(tcp_mode='gripper', tools={side: arm.tool for side, arm in self.arms.items()},
                    tcp_presets_path=str(preset), tcp_presets_sha256=hashlib.sha256(preset.read_bytes()).hexdigest(),
                    model_path=str(model_path()), model_sha256=MODEL_SHA256,
                    origin_arm_rad=list(self.origin.arm_rad),
                    origin_hand_openings=list(self.origin.hand_openings),
                    joint_delta_limit_rad=MAX_JOINT_DELTA_RAD,
                    clamp_reach=False, fallback=False, compensation_enabled=False)

    def propose(self, frame, *, reference=None, rebase_sides=()):
        """为TargetFrame求完整候选；无解、越限或分支改变抛出OnlineIKFailure。

        每侧残差只记录；沿用10°相邻及相对实例起点范围。调用不改变已接受目标、
        种子、SEW、分支、revision或目标存储；未使能侧保持上一已接受目标。
        reference为首次接管的新鲜HAL命令；rebase_sides指定首次接管的侧，同时重建
        这些侧的种子、SEW及10°范围起点，候选提交前不生效。
        """
        began = time.monotonic()
        report = dict(status='rejected', arms={}, started_monotonic_s=began,
                      previous_input_seq=self.last_input_seq)
        try:
            if not isinstance(frame, TargetFrame):
                raise ControlFailure('在线IK输入须为TargetFrame')
            if frame.base_frame != 'torso_link':
                raise ControlFailure('在线IK目标坐标系须为torso_link')
            report.update(input_seq=frame.input_seq, source_time_ns=frame.source_time_ns,
                          received_monotonic_s=frame.received_monotonic_s)
            if self.last_input_seq is not None and frame.input_seq <= self.last_input_seq:
                raise ControlFailure('目标序号未推进')
            baseline, origin = self.accepted, self.origin
            reference_states = dict(self.states)
            if any(side not in SIDES for side in rebase_sides):
                raise ControlFailure('重新建立参考的侧须为left或right')
            if self.command_reference_sides.intersection(rebase_sides):
                raise ControlFailure('同会话已接管侧须延续理想目标；重新采样命令需新建会话')
            for side in rebase_sides:
                if not isinstance(reference, CapturedTarget) or not getattr(frame, side).enabled:
                    raise ControlFailure('首次接管要求完整命令参考及使能侧')
                index = SIDES.index(side)
                joints = reference.arm_rad[index * 7:index * 7 + 7]
                baseline = baseline.for_arm(side, joints)
                origin = origin.for_arm(side, joints)
                sew = float(self.arms[side].solver.sew_angle(np.asarray(joints)))
                if not math.isfinite(sew):
                    raise ControlFailure(f'{side}命令参考SEW非有限')
                reference_states[side] = ArmTrackingState(joints, sew, None)
            report['rebase_sides'] = list(rebase_sides)
            candidate_target = CapturedTarget(baseline.arm_rad, frame.hand_openings)
            states = []
            for index, side in enumerate(SIDES):
                arm, previous = self.arms[side], reference_states[side]
                goal = getattr(frame, side)
                row = dict(enabled=goal.enabled, seed_rad=list(previous.joints),
                           previous_sew_rad=previous.sew, previous_branch=previous.branch,
                           requested_position_m=list(goal.pose.position_m),
                           requested_rotation_matrix=[list(v) for v in goal.pose.rotation_matrix])
                report['arms'][side] = row
                start = time.monotonic()
                joints, sew, branch = previous.joints, previous.sew, previous.branch
                position, rotation = arm.forward(previous.joints)
                desired_position = np.asarray(goal.pose.position_m)
                desired_rotation = np.asarray(goal.pose.rotation_matrix)
                # 格式转换的浮点舍入视作相同目标，避免零增量重排解析解。
                unchanged = (np.allclose(position, desired_position, rtol=0., atol=1e-12)
                             and np.allclose(rotation, desired_rotation, rtol=0., atol=1e-12))
                if goal.enabled and not unchanged:
                    solution = arm.track(desired_position, desired_rotation, previous.joints,
                                         previous_psi=previous.sew, previous_branch=previous.branch)
                    if solution is None:
                        raise ControlFailure(f'{side}局部IK无解')
                    joints = finite_positions(solution.q, 7, f'{side}候选')
                    sew = float(solution.psi)
                    branch = (int(solution.elbow_branch), int(solution.shoulder_branch),
                              int(solution.wrist_branch))
                    if previous.branch is not None and branch != previous.branch:
                        raise ControlFailure(f'{side}解析分支改变')
                if not math.isfinite(sew):
                    raise ControlFailure(f'{side}候选SEW非有限')
                candidate_target = candidate_target.for_arm(side, joints)
                origin_joints = origin.arm_rad[index * 7:index * 7 + 7]
                step = max(abs(a - b) for a, b in zip(joints, previous.joints))
                excursion = max(abs(a - b) for a, b in zip(joints, origin_joints))
                if max(step, excursion) > MAX_JOINT_DELTA_RAD:
                    raise ControlFailure(f'{side}相邻或会话总变化超过既有10°范围')
                actual, actual_rotation = arm.forward(joints)
                residual = float(np.linalg.norm(actual - desired_position))
                angular = float(np.linalg.norm(log3(desired_rotation @ actual_rotation.T)))
                if not math.isfinite(residual) or not math.isfinite(angular):
                    raise ControlFailure(f'{side}FK残差非有限')
                row.update(candidate_rad=list(joints), sew_rad=sew, branch=branch,
                           step_rad=step, excursion_rad=excursion,
                           seed_preserved=not goal.enabled or unchanged,
                           position_residual_m=residual, rotation_residual_rad=angular,
                           solve_seconds=time.monotonic() - start, status='accepted')
                states.append(ArmTrackingState(tuple(joints), sew, branch))
            report.update(status='candidate', arm_rad=list(candidate_target.arm_rad),
                          hand_openings=list(candidate_target.hand_openings),
                          calculation_seconds=time.monotonic() - began)
            return IKProposal(self, self.revision, frame.input_seq, candidate_target, tuple(states), report, origin)
        except Exception as error:
            report.update(reason=str(error), calculation_seconds=time.monotonic() - began,
                          last_accepted_rad=list(self.accepted.arm_rad))
            raise OnlineIKFailure(str(error), report) from error

    def validate_commit(self, proposal):
        """检查候选所属实例与revision，供会话和IK共同提交前校验。"""
        if not isinstance(proposal, IKProposal) or proposal.owner is not self or proposal.revision != self.revision:
            raise ControlFailure('IK候选属于其他实例或已过期')

    def commit(self, proposal):
        """提交本实例当前revision候选并更新完整目标存储；过期或异源候选抛出ControlFailure。"""
        self.validate_commit(proposal)
        self.accepted = proposal.target
        self.origin = proposal.origin
        self.command_reference_sides.update(proposal.report['rebase_sides'])
        self.states = dict(zip(SIDES, proposal.states))
        self.last_input_seq = proposal.input_seq
        self.revision += 1
        return self.store.put(proposal.target, proposal.input_seq)
