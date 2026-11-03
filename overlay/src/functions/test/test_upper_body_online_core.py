"""验证双侧原子提交、保持与分支拒绝；由用户在机器人容器内运行。"""

from types import SimpleNamespace
import unittest

import numpy as np

from remote_control.frames import Pose, TCPGoal, TargetFrame
from aimdk_functions.upper_body_cartesian_core import CapturedTarget
from aimdk_functions.upper_body_kinematics.parameters import EXECUTION_LIMITS_RAD
from aimdk_functions.upper_body_online_core import DualArmOnlineIK, OnlineIKFailure


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


if __name__ == '__main__':
    unittest.main()
