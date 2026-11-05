"""显式坐标映射与事务式Grip会话；与机器人消息及IK实现解耦。"""

from dataclasses import dataclass, replace
import json
import math
from pathlib import Path

import numpy as np

from .frames import SIDES, Pose, TCPGoal, TargetFrame, XRFrame, finite
from .geometry import immutable_matrix


class InputUnavailable(ValueError):
    """输入失效或参考空间变化，调用方应停止消费旧目标并记录原因。"""


@dataclass(frozen=True)
class MappingConfig:
    """输入基至目标基的正交变换及显式操作策略，不含设备默认轴向。"""

    basis: tuple
    translation_scale: float
    rotation_mode: str
    grip_threshold: float
    gripper_mode: str
    trigger_openings: tuple
    unknown_tracking: str
    base_frame: str
    reference_mode: str = 'world'

    def __post_init__(self):
        """校验反射基、位移比例、Grip与夹爪策略，配置错误抛出ValueError。"""
        object.__setattr__(self, 'basis', immutable_matrix(self.basis, reflection=True))
        scale = finite(self.translation_scale, 'translation_scale', 0.)
        if scale == 0:
            raise ValueError('translation_scale须大于0')
        object.__setattr__(self, 'translation_scale', scale)
        threshold = finite(self.grip_threshold, 'grip_threshold', 0., 1.)
        if threshold == 0:
            raise ValueError('grip_threshold须大于0')
        object.__setattr__(self, 'grip_threshold', threshold)
        if self.rotation_mode not in ('relative', 'fixed'):
            raise ValueError('rotation_mode须为relative或fixed')
        if self.gripper_mode not in ('hold', 'trigger'):
            raise ValueError('gripper_mode须为hold或trigger')
        if self.gripper_mode == 'trigger':
            if self.trigger_openings is None or len(self.trigger_openings) != 2:
                raise ValueError('trigger_openings须显式提供扳机0和1对应的开度')
            object.__setattr__(self, 'trigger_openings', tuple(
                finite(value, 'trigger_openings', 0., 1.) for value in self.trigger_openings))
        elif self.trigger_openings is not None:
            raise ValueError('hold模式的trigger_openings须为null')
        if self.unknown_tracking not in ('reject', 'allow'):
            raise ValueError('unknown_tracking须显式指定reject或allow')
        if not isinstance(self.base_frame, str) or not self.base_frame:
            raise ValueError('base_frame须为非空字符串')
        if self.reference_mode not in ('world', 'grip_head_yaw'):
            raise ValueError('reference_mode须为world或grip_head_yaw')

    @classmethod
    def from_dict(cls, value):
        """读取全部显式配置字段；缺字段抛出KeyError，非法值抛出ValueError。"""
        return cls(*(value[key] for key in (
            'basis', 'translation_scale', 'rotation_mode', 'grip_threshold',
            'gripper_mode', 'trigger_openings', 'unknown_tracking', 'base_frame')),
                   reference_mode=value.get('reference_mode', 'world'))

    @classmethod
    def load(cls, path):
        """从UTF-8 JSON文件加载显式映射配置；文件或解析错误直接传播。"""
        return cls.from_dict(json.loads(Path(path).read_text(encoding='utf-8')))


@dataclass(frozen=True)
class _SideState:
    """单侧已提交手柄参考、机器人参考、最后目标与Grip状态。"""

    controller_reference: object = None
    robot_reference: object = None
    held_pose: object = None
    opening: object = None
    pressed: bool = False
    grip_generation: int = -1
    mapping_basis: object = None
    head_yaw_rad: object = None


@dataclass(frozen=True)
class Proposal:
    """一次映射候选；仅对应会话commit成功后写入参考与保持目标。"""

    target: TargetFrame
    revision: int
    states: tuple
    report: dict


class Session:
    """双侧Grip参考及相对运动事务；IK失败时reject保持已提交状态。"""

    def __init__(self, config):
        """使用MappingConfig创建空会话；首次输入允许按下Grip建立参考。"""
        if not isinstance(config, MappingConfig):
            raise ValueError('config须为MappingConfig')
        self.config = config
        self._states = (_SideState(), _SideState())
        self._blocked = [False, False]
        self._observed_pressed = [False, False]
        self._grip_generation = [0, 0]
        self._revision = 0
        self._pending = None
        self._last_seq = 0
        self._source_time = None
        self._received_time = None
        self._identity = None
        self.reset_reason = None

    def reset(self, reason):
        """撤销两侧参考及待提交候选，保留最后目标，要求松开Grip后重建参考。"""
        self._states = tuple(_SideState(held_pose=state.held_pose, opening=state.opening)
                             for state in self._states)
        self._blocked = [True, True]
        self._pending = None
        self._revision += 1
        self.reset_reason = str(reason)

    def describe(self):
        """返回已提交参考和输入消费状态供诊断；不暴露可修改的会话对象。"""
        sides = {}
        for index, side in enumerate(SIDES):
            state = self._states[index]
            sides[side] = dict(
                pressed=state.pressed,
                controller_reference=(state.controller_reference.to_dict()
                                      if state.controller_reference is not None else None),
                robot_reference=(state.robot_reference.to_dict()
                                 if state.robot_reference is not None else None),
                held_pose=state.held_pose.to_dict() if state.held_pose is not None else None,
                opening=state.opening, grip_generation=state.grip_generation,
                mapping_basis=state.mapping_basis, head_yaw_rad=state.head_yaw_rad)
        return dict(revision=self._revision, last_consumed_seq=self._last_seq,
                    pending=self._pending is not None, reset_reason=self.reset_reason,
                    observed_blocked=dict(zip(SIDES, self._blocked)), sides=sides)

    def observe_input(self, frame):
        """消费输入序号并校验连接、时间与参考空间；异常重置并抛出InputUnavailable。

        连续接收器可使用独立Session逐帧校验，不创建映射候选或修改Grip参考。
        """
        if not isinstance(frame, XRFrame):
            raise ValueError('输入须为XRFrame')
        if self._pending is not None:
            raise ValueError('上一映射候选尚未commit或reject')
        if frame.seq <= self._last_seq:
            raise ValueError('输入序号须严格推进，重复缓存不得重复求解')
        self._last_seq = frame.seq
        identity = (frame.device_id, frame.reference_space_id)
        reason = None
        if not frame.connected:
            reason = 'XR连接中断'
        elif self.config.reference_mode == 'grip_head_yaw' and frame.head is None:
            reason = 'grip_head_yaw需要同帧Head位姿，请开启Head tracking'
        elif self._identity is not None and identity != self._identity:
            reason = 'XR设备或参考空间改变'
        elif (self._source_time is not None and frame.source_time_ns is not None
              and frame.source_time_ns < self._source_time):
            reason = 'XR源时间回退'
        elif (self._received_time is not None
              and frame.received_monotonic_s < self._received_time):
            reason = 'XR接收单调时间回退'
        else:
            for side in SIDES:
                valid = getattr(frame, side).tracking_valid
                if valid is False or (valid is None and self.config.unknown_tracking == 'reject'):
                    reason = f'{side}跟踪无效或未提供有效性'
                    break
        self._identity = identity
        # 缺失值按原样记录，推进检查继续使用最后已知源时间。
        if frame.source_time_ns is not None:
            self._source_time = frame.source_time_ns
        self._received_time = frame.received_monotonic_s
        if reason is not None:
            self.reset(reason)
            raise InputUnavailable(reason)

    def propose(self, frame, robot_tcp_poses, hand_openings, *, hold_motion=False):
        """映射一帧并返回候选，robot_tcp_poses为两侧参考Pose，hand_openings为保持开度。

        初次或Grip上升沿使用传入机器人参考；调用方可按侧选择实际FK或最后已接受目标FK。
        hold_motion为True时仅更新Grip参考和保持状态，双侧位姿与夹爪保持已接受值。
        任一输入异常抛出ValueError，提交前不修改参考和保持目标。
        """
        self.observe_input(frame)
        if len(hand_openings) != 2:
            raise ValueError('hand_openings须包含左右两侧开度')
        basis = np.asarray(self.config.basis)
        goals, states, openings, report = {}, [], [], {}
        for index, side in enumerate(SIDES):
            robot_pose = robot_tcp_poses[side]
            if not isinstance(robot_pose, Pose):
                raise ValueError(f'{side}机器人参考须为Pose')
            fallback_opening = finite(hand_openings[index], f'{side}夹爪', 0., 1.)
            controller = getattr(frame, side)
            state = self._states[index]
            pressed = controller.grip >= self.config.grip_threshold
            if not pressed:
                self._blocked[index] = False
                if self._observed_pressed[index]:
                    self._grip_generation[index] += 1
            self._observed_pressed[index] = pressed
            active = pressed and not self._blocked[index]
            opening = state.opening if state.opening is not None else fallback_opening
            pose = robot_pose if hold_motion else (state.held_pose or robot_pose)
            reference, robot_reference = state.controller_reference, state.robot_reference
            mapping_basis, head_yaw = state.mapping_basis, state.head_yaw_rad
            established = False
            input_translation = input_rotation = tcp_translation = tcp_rotation = None
            if active:
                if not state.pressed or state.grip_generation != self._grip_generation[index]:
                    reference, robot_reference = controller.pose, robot_pose
                    mapping_basis = basis
                    head_yaw = None
                    if self.config.reference_mode == 'grip_head_yaw':
                        head_rotation = basis @ np.asarray(frame.head.rotation_matrix) @ basis.T
                        forward = head_rotation[:2, 0]
                        if np.linalg.norm(forward) <= np.finfo(float).eps:
                            raise InputUnavailable('头显前向水平投影为零，无法建立Grip朝向')
                        head_yaw = math.atan2(forward[1], forward[0])
                        c, s = math.cos(head_yaw), math.sin(head_yaw)
                        yaw_inverse = np.array(((c, s, 0.), (-s, c, 0.), (0., 0., 1.)))
                        mapping_basis = yaw_inverse @ basis
                    mapping_basis = immutable_matrix(mapping_basis, reflection=True)
                    established = True
                active_basis = np.asarray(mapping_basis)
                input_translation = (np.asarray(controller.pose.position_m)
                                     - np.asarray(reference.position_m))
                input_rotation = (np.asarray(controller.pose.rotation_matrix)
                                  @ np.asarray(reference.rotation_matrix).T)
                tcp_translation = self.config.translation_scale * active_basis @ input_translation
                position = np.asarray(robot_reference.position_m) + tcp_translation
                rotation = np.asarray(robot_reference.rotation_matrix)
                tcp_rotation = np.eye(3)
                if self.config.rotation_mode == 'relative':
                    tcp_rotation = active_basis @ input_rotation @ active_basis.T
                    rotation = tcp_rotation @ rotation
                if not hold_motion:
                    pose = Pose(position, rotation)
                elif established:
                    pose = robot_pose
                if hold_motion:
                    tcp_translation = (np.asarray(pose.position_m)
                                       - np.asarray(robot_reference.position_m))
                    tcp_rotation = (np.asarray(pose.rotation_matrix)
                                    @ np.asarray(robot_reference.rotation_matrix).T)
                if self.config.gripper_mode == 'trigger' and not hold_motion:
                    zero, one = self.config.trigger_openings
                    opening = zero + controller.trigger * (one - zero)
            report[side] = dict(
                event=('reference_established' if established else 'tracking' if active
                       else 'released' if state.pressed and not pressed
                       else 'blocked' if pressed else 'holding'),
                enabled=active, hold_motion=hold_motion, grip=controller.grip, trigger=controller.trigger,
                controller_reference=reference.to_dict() if active else None,
                robot_reference=robot_reference.to_dict() if active else None,
                reference_mode=self.config.reference_mode,
                mapping_basis=mapping_basis if active else None,
                head_yaw_rad=head_yaw if active else None,
                input_translation_m=input_translation.tolist() if active else None,
                input_rotation_matrix=input_rotation.tolist() if active else None,
                tcp_translation_m=tcp_translation.tolist() if active else None,
                tcp_rotation_matrix=tcp_rotation.tolist() if active else None,
                target_pose=pose.to_dict(), hand_opening=opening)
            goals[side] = TCPGoal(pose, active)
            openings.append(opening)
            states.append(_SideState(reference if active else None,
                                     robot_reference if active else None,
                                     pose, opening, active, self._grip_generation[index],
                                     mapping_basis if active else None, head_yaw if active else None))
        target = TargetFrame(frame.seq, frame.source_time_ns, frame.received_monotonic_s,
                             goals['left'], goals['right'], tuple(openings), self.config.base_frame)
        proposal = Proposal(target, self._revision, tuple(states), report)
        self._pending = proposal
        return proposal

    def validate_commit(self, proposal):
        """只读检查候选归属与版本，供调用方在双对象提交前预检。"""
        if proposal is not self._pending or proposal.revision != self._revision:
            raise ValueError('映射候选已失效或不属于当前会话')

    def commit(self, proposal, *, accepted_target=None):
        """提交映射参考，返回目标；accepted_target可指定执行侧实际接受的过渡保持目标。

        过渡目标只替换held_pose与opening，Grip参考、使能状态及世代仍来自proposal。
        """
        self.validate_commit(proposal)
        self._states = (proposal.states if accepted_target is None else tuple(
            replace(state, held_pose=getattr(accepted_target, side).pose,
                    opening=accepted_target.hand_openings[index])
            for index, (side, state) in enumerate(zip(SIDES, proposal.states))))
        self._pending = None
        self._revision += 1
        return proposal.target if accepted_target is None else accepted_target

    def reject(self, proposal):
        """拒绝当前候选，保留先前参考及保持目标；该输入序号已消费。"""
        self.validate_commit(proposal)
        self._pending = None
