"""显式坐标映射与事务式Grip会话；与机器人消息及IK实现解耦。"""

from dataclasses import dataclass
import json
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

    @classmethod
    def from_dict(cls, value):
        """读取全部显式配置字段；缺字段抛出KeyError，非法值抛出ValueError。"""
        return cls(*(value[key] for key in (
            'basis', 'translation_scale', 'rotation_mode', 'grip_threshold',
            'gripper_mode', 'trigger_openings', 'unknown_tracking', 'base_frame')))

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


@dataclass(frozen=True)
class Proposal:
    """一次映射候选；仅对应会话commit成功后写入参考与保持目标。"""

    target: TargetFrame
    revision: int
    states: tuple


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

    def _observe(self, frame):
        """消费输入序号并检查连接、时间及参考空间，异常重置后抛出InputUnavailable。"""
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

    def propose(self, frame, robot_tcp_poses, hand_openings):
        """映射一帧并返回候选，robot_tcp_poses为两侧参考Pose，hand_openings为保持开度。

        初次或Grip上升沿使用传入机器人参考；调用方可按侧选择实际FK或最后已接受目标FK。
        任一输入异常抛出ValueError，提交前不修改参考和保持目标。
        """
        self._observe(frame)
        if len(hand_openings) != 2:
            raise ValueError('hand_openings须包含左右两侧开度')
        basis = np.asarray(self.config.basis)
        goals, states, openings = {}, [], []
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
            pose = state.held_pose or robot_pose
            reference, robot_reference = state.controller_reference, state.robot_reference
            if active:
                if not state.pressed or state.grip_generation != self._grip_generation[index]:
                    reference, robot_reference = controller.pose, robot_pose
                position = (np.asarray(robot_reference.position_m) + self.config.translation_scale
                            * basis @ (np.asarray(controller.pose.position_m)
                                       - np.asarray(reference.position_m)))
                rotation = np.asarray(robot_reference.rotation_matrix)
                if self.config.rotation_mode == 'relative':
                    delta = (np.asarray(controller.pose.rotation_matrix)
                             @ np.asarray(reference.rotation_matrix).T)
                    rotation = basis @ delta @ basis.T @ rotation
                pose = Pose(position, rotation)
                if self.config.gripper_mode == 'trigger':
                    zero, one = self.config.trigger_openings
                    opening = zero + controller.trigger * (one - zero)
            goals[side] = TCPGoal(pose, active)
            openings.append(opening)
            states.append(_SideState(reference if active else None,
                                     robot_reference if active else None,
                                     pose, opening, active, self._grip_generation[index]))
        target = TargetFrame(frame.seq, frame.source_time_ns, frame.received_monotonic_s,
                             goals['left'], goals['right'], tuple(openings), self.config.base_frame)
        proposal = Proposal(target, self._revision, tuple(states))
        self._pending = proposal
        return proposal

    def validate_commit(self, proposal):
        """只读检查候选归属与版本，供调用方在双对象提交前预检。"""
        if proposal is not self._pending or proposal.revision != self._revision:
            raise ValueError('映射候选已失效或不属于当前会话')

    def commit(self, proposal):
        """提交已接受的双侧映射参考及保持目标，返回完整目标帧。"""
        self.validate_commit(proposal)
        self._states = proposal.states
        self._pending = None
        self._revision += 1
        return proposal.target

    def reject(self, proposal):
        """拒绝当前候选，保留先前参考及保持目标；该输入序号已消费。"""
        self.validate_commit(proposal)
        self._pending = None
