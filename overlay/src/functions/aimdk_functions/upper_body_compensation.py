"""原生位置接口的显式补偿适配；保留固定参数，不使用effort推算负载。"""

import math

import numpy as np

from .dual_arm_control_core import ControlFailure, Sample, sample
from .upper_body_kinematics.parameters import MODEL_SHA256, model_path
from .upper_body_kinematics.reference_compensation.x2_arm_dynamics import ArmDynamics
from .upper_body_kinematics.reference_compensation.x2_compensation import fixed_compensation
from .upper_body_kinematics.reference_compensation.x2_frames import (
    G_WORLD, WAIST_JOINTS, WaistChain, quat_to_matrix,
)


def decode_waist(msg, received, serial):
    """按名称读取三腰关节rad及错误；无效、缺失或重复时抛出ControlFailure。"""
    names = [joint.name for joint in msg.joints]
    if len(names) != 3 or set(names) != set(WAIST_JOINTS) or msg.state.value != 0:
        raise ControlFailure('补偿需要完整无故障的三腰关节反馈')
    found = {joint.name: joint for joint in msg.joints}
    for joint in found.values():
        if joint.error_code or not all(math.isfinite(v) for v in (
                joint.position, joint.velocity, joint.effort)):
            raise ControlFailure(f'腰反馈无效：{joint.name}')
    return sample(msg, received, serial, [found[name].position for name in WAIST_JOINTS],
                  dict(observed_names=names))


def decode_pelvis_imu(msg, received, serial):
    """读取骨盆IMU四元数及真实源时间；拒绝无姿态、零四元数和未知坐标标签。"""
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    # 实机观测的frame_id为base_link；模型base_link与pelvis固定重合，不能据此选chest话题。
    if msg.header.frame_id not in ('base_link', 'pelvis', 'imu_in_pelvis_link'):
        raise ControlFailure(f'未核对的骨盆IMU frame_id：{msg.header.frame_id}')
    quat = tuple(getattr(msg.orientation, axis) for axis in ('x', 'y', 'z', 'w'))
    norm = math.hypot(*quat)
    if (not math.isfinite(stamp) or stamp <= 0 or msg.orientation_covariance[0] == -1
            or not all(math.isfinite(v) for v in quat)
            or not math.isfinite(norm) or norm < 1e-12):
        raise ControlFailure('骨盆IMU源时间或姿态无效；不使用静态重力或旧缓存兜底')
    return Sample(quat, stamp, received, serial, dict(frame_id=msg.header.frame_id))


class PositionCompensation:
    """官方v1.3.0惯性模型下的固定参数适配版；只修改所选臂目标。"""

    def __init__(self, arm, payload_mass, payload_com):
        """以单臂模型及腕系负载质心m、质量kg构造计算器；参数格式由入口校验。"""
        self.side = arm.side
        self.config = fixed_compensation()
        self.chain = WaistChain(model_path())
        self.dynamics = ArmDynamics(arm.model, payload_mass=payload_mass, payload_com=payload_com)

    def describe(self):
        """返回日志中的固定配置、模型及适配差异；不宣称实机标定完成。"""
        return dict(**self.config, implementation='adapted_official_v1_3_0',
                    model_sha256=MODEL_SHA256, payload_mass_kg=self.dynamics.payload_mass,
                    payload_com_wrist_m=self.dynamics.payload_com.tolist(),
                    gravity_input='pelvis_imu_orientation_plus_named_waist',
                    gravity_filter='none', position_limit_policy='reject_without_clipping',
                    engagement='quintic_0_to_1_over_motion_duration', selected_side=self.side)

    def apply(self, captured, ideal, values, blend=1.):
        """由已校验的新鲜IMU/腰样本计算目标与诊断；blend为启用进度，越限明确拒绝。"""
        imu, waist = values['pelvis_imu'], values['waist']
        _, torso_in_pelvis = self.chain.torso_pose_in_pelvis(waist.positions)
        gravity = (quat_to_matrix(*imu.positions) @ torso_in_pelvis).T @ G_WORLD
        self.dynamics.gravity = gravity
        offset = 0 if self.side == 'left' else 7
        joints = np.asarray(ideal.arm_rad[offset:offset + 7])
        tau = self.dynamics.gravity_torque(joints)
        raw = tau / self.config['stiffness']
        limit = math.radians(self.config['bias_limit_deg'])
        bounded = np.clip(raw, -limit, limit)
        bias = bounded * blend
        if not np.isfinite(tau).all() or not math.isfinite(blend) or not 0 <= blend <= 1:
            raise ControlFailure('补偿力矩或启用进度无效')
        details = dict(ideal_arm_rad=list(ideal.arm_rad), gravity_torso_m_s2=gravity.tolist(),
                       gravity_torque_nm=tau.tolist(), raw_bias_rad=raw.tolist(),
                       bounded_bias_rad=bounded.tolist(), bias_rad=bias.tolist(), blend=blend,
                       bias_saturated=(np.abs(raw) > limit).tolist(),
                       imu_stamp_s=imu.stamp, imu_received_monotonic_s=imu.received,
                       waist_stamp_s=waist.stamp, waist_received_monotonic_s=waist.received,
                       input_source_skew_s=imu.stamp-waist.stamp)
        try:
            target = captured.for_arm(self.side, joints + bias)
        except ControlFailure as error:
            raise ControlFailure(f'补偿后目标越限或无效：{error}；补偿诊断={details}') from error
        details['sent_arm_rad'] = list(target.arm_rad)
        return target, details
