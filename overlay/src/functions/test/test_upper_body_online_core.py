"""验证双侧原子提交、保持与分支拒绝；由用户在机器人容器内运行。"""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np

from remote_control.frames import Controller, Pose, TCPGoal, TargetFrame, XRFrame
from remote_control.mapping import MappingConfig, Session
from aimdk_functions.upper_body_cartesian_core import CapturedTarget
from aimdk_functions.upper_body_kinematics.parameters import EXECUTION_LIMITS_RAD
from aimdk_functions.upper_body_online_core import DualArmOnlineIK, OnlineIKFailure
from aimdk_functions.upper_body_teleop_observe import command_reference
from aimdk_functions.upper_body_teleop_replay import replay_frames


class FakeArm:
    """用单轴位置模拟局部IK，保留真实执行限位并允许注入失败及分支变化。"""

    def __init__(self, side):
        """以side设置模型身份和gripper配置，默认使用固定解析分支。"""
        self.side = side
        self.tool = dict(mode='gripper')
        self.solver = self
        self.fail = False
        self.branch = (1, 1, 1)
        self.calls = 0

    def sew_angle(self, joints):
        """返回用于确认回滚的可观测SEW值。"""
        return float(joints[0])

    def forward(self, joints):
        """以首关节作为X位置并返回单位姿态。"""
        return np.asarray([joints[0], 0., 0.]), np.eye(3)

    def track(self, position_m, rotation_matrix, previous_joints,
              previous_psi=None, previous_branch=None):
        """记录调用并按指定位置生成局部候选；fail为真时模拟无解。"""
        self.calls += 1
        if self.fail:
            return None
        result = np.asarray(previous_joints).copy()
        result[0] = position_m[0]
        return SimpleNamespace(q=result, psi=float(result[0]),
                               elbow_branch=self.branch[0], shoulder_branch=self.branch[1],
                               wrist_branch=self.branch[2])


class OnlineCoreTest(unittest.TestCase):
    """仅注入数学后端检查状态契约，不导入ROS或发布消息。"""

    def setUp(self):
        """使用有效范围中点创建完整采样和两个可控后端。"""
        self.captured = CapturedTarget(tuple((lower + upper) / 2 for lower, upper in EXECUTION_LIMITS_RAD),
                                       (.3, .7))
        self.arms = {side: FakeArm(side) for side in ('left', 'right')}
        self.solver = DualArmOnlineIK(self.captured, self.arms)

    def frame(self, seq, left_delta=.01, right_delta=.01, enabled=(True, True), base_frame='torso_link'):
        """构造相对实例起点的小幅目标，两侧夹爪保持原开度。"""
        goals = []
        for index, (delta, active) in enumerate(zip((left_delta, right_delta), enabled)):
            x = self.captured.arm_rad[index * 7] + delta
            goals.append(TCPGoal(Pose((x, 0., 0.), np.eye(3)), active))
        return TargetFrame(seq, seq * 1000, float(seq), *goals, self.captured.hand_openings, base_frame)

    def test_right_failure_does_not_advance_left(self):
        """右侧无解时左侧候选不得改变双侧种子、SEW、分支或完整目标。"""
        before_states = dict(self.solver.states)
        before_store = self.solver.store.get()
        self.arms['right'].fail = True
        with self.assertRaises(OnlineIKFailure):
            self.solver.propose(self.frame(1))
        self.assertEqual(self.arms['left'].calls, 1)
        self.assertEqual(self.solver.states, before_states)
        self.assertEqual(self.solver.accepted, self.captured)
        self.assertEqual(self.solver.revision, 0)
        self.assertIs(self.solver.store.get(), before_store)

    def test_commit_updates_both_once_and_rejects_stale(self):
        """成功候选仅显式提交后生效，重复提交及旧候选均被拒绝。"""
        first = self.solver.propose(self.frame(1))
        stale = self.solver.propose(self.frame(2))
        self.assertEqual(self.solver.accepted, self.captured)
        value = self.solver.commit(first)
        self.assertEqual(self.solver.last_input_seq, 1)
        self.assertEqual(value.input_seq, 1)
        self.assertEqual(self.solver.states['left'].branch, (1, 1, 1))
        self.assertEqual(self.solver.states['right'].branch, (1, 1, 1))
        with self.assertRaises(RuntimeError):
            self.solver.commit(stale)
        with self.assertRaises(RuntimeError):
            self.solver.commit(first)

    def test_branch_change_rolls_back_both(self):
        """已锁分支改变时双方维持上一完整已接受状态。"""
        self.solver.commit(self.solver.propose(self.frame(1)))
        before = dict(self.solver.states)
        self.arms['right'].branch = (-1, 1, 1)
        with self.assertRaises(OnlineIKFailure):
            self.solver.propose(self.frame(2, .02, .02))
        self.assertEqual(self.solver.states, before)
        self.assertEqual(self.solver.last_input_seq, 1)

    def test_disabled_side_preserves_sample(self):
        """未使能侧保持采样目标且不调用该侧IK。"""
        candidate = self.solver.propose(self.frame(1, enabled=(True, False)))
        self.solver.commit(candidate)
        self.assertEqual(self.solver.accepted.arm_rad[7:], self.captured.arm_rad[7:])
        self.assertEqual(self.arms['right'].calls, 0)

    def test_range_and_coordinate_frame_rejected(self):
        """拒绝会话10度以外目标和未转换到torso_link的目标。"""
        for frame in (self.frame(1, .2, .01), self.frame(1, base_frame='world')):
            with self.assertRaises(OnlineIKFailure):
                self.solver.propose(frame)
        self.assertEqual(self.solver.accepted, self.captured)

    def test_zero_increment_preserves_seed(self):
        """零增量保持双侧原始关节，目标缓存读取不刷新接受时间。"""
        candidate = self.solver.propose(self.frame(1, 0., 0.))
        value = self.solver.commit(candidate)
        self.assertEqual(value.target, self.captured)
        self.assertEqual(sum(arm.calls for arm in self.arms.values()), 0)
        self.assertIs(self.solver.store.get(), value)

    def teleop_step(self, seq, command_delta, grips=(1., 0.), x=0.):
        """使用不同于实际反馈的HAL命令回放一帧，返回该帧采样的完整命令。"""
        if not hasattr(self, 'session'):
            self.config = MappingConfig.from_dict(dict(
                basis=np.eye(3), translation_scale=1., rotation_mode='relative',
                grip_threshold=.5, gripper_mode='hold', trigger_openings=None,
                unknown_tracking='reject', base_frame='torso_link'))
            self.session = Session(self.config)
        joints = list(self.captured.arm_rad)
        for index in (0, 7):
            joints[index] += command_delta
        values = dict(
            arm=SimpleNamespace(positions=self.captured.arm_rad, stamp=1., serial=seq),
            arm_command=SimpleNamespace(positions=tuple(joints), stamp=1.01, serial=seq),
            hand=SimpleNamespace(positions=self.captured.hand_openings, serial=seq))
        hands = [Controller(Pose((x, 0., 0.), np.eye(3)), grip, 0., tracking_valid=True)
                 for grip in grips]
        frame = XRFrame(seq, seq * 1000, float(seq), *hands)
        diagnostics = Mock()

        def reference_provider(value):
            """以人工反馈提供现场参考，保持与真实入口相同的提交路径。"""
            return command_reference(value, values, self.solver, self.config, diagnostics)

        replay_frames([frame], self.session, self.solver, diagnostics, reference_provider)
        return tuple(joints)

    def test_first_grip_samples_command_and_zero_increment_preserves_it(self):
        """启动后等待期间HAL变化，首次Grip使用按下时命令，SEW同步且不调用IK。"""
        self.teleop_step(1, .01, grips=(0., 0.))
        command = self.teleop_step(2, .03)
        self.assertEqual(self.solver.accepted.arm_rad[:7], command[:7])
        self.assertEqual(self.solver.origin.arm_rad[:7], command[:7])
        self.assertEqual(self.solver.states['left'].sew, command[0])
        self.assertEqual(self.solver.command_reference_sides, {'left'})
        self.assertEqual(self.arms['left'].calls, 0)

    def test_regrip_uses_ideal_and_later_first_grip_is_independent(self):
        """左臂重按忽略新的HAL偏置，右臂首次按下独立取新命令且不重置左臂范围。"""
        command = self.teleop_step(1, .02)
        self.teleop_step(2, .04, x=.01)
        ideal = self.solver.accepted.arm_rad[:7]
        self.teleop_step(3, .06, grips=(0., 0.), x=.2)
        new_command = self.teleop_step(4, .08, grips=(1., 1.), x=.3)
        self.assertEqual(self.solver.accepted.arm_rad[:7], ideal)
        self.assertEqual(self.solver.origin.arm_rad[:7], command[:7])
        self.assertEqual(self.solver.accepted.arm_rad[7:], new_command[7:])
        self.assertEqual(self.solver.command_reference_sides, {'left', 'right'})

    def test_failed_first_grip_does_not_commit_reference(self):
        """右臂失败时，左臂首次命令参考、范围起点及已接管标记均不生效。"""
        self.teleop_step(1, .02, grips=(0., 1.))
        before, origin = self.solver.accepted, self.solver.origin
        store, states = self.solver.store.get(), dict(self.solver.states)
        self.arms['right'].fail = True
        with self.assertRaises(OnlineIKFailure):
            self.teleop_step(2, .04, grips=(1., 1.), x=.01)
        self.assertEqual(self.solver.accepted, before)
        self.assertEqual(self.solver.origin, origin)
        self.assertEqual(self.solver.states, states)
        self.assertIs(self.solver.store.get(), store)
        self.assertEqual(self.solver.command_reference_sides, {'right'})

    def test_restart_discards_old_ideal_and_samples_new_command(self):
        """异常后重启会话，首次接管重新取HAL命令，不复用中断前理想目标。"""
        self.teleop_step(1, .02, grips=(1., 1.))
        self.teleop_step(2, .02, grips=(1., 1.), x=.01)
        self.solver = DualArmOnlineIK(self.captured, self.arms)
        del self.session
        command = self.teleop_step(3, .06, grips=(1., 1.))
        self.assertEqual(self.solver.accepted.arm_rad, command)
        self.assertEqual(self.solver.origin.arm_rad, command)


if __name__ == '__main__':
    unittest.main()
