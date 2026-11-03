"""通用映射与事务边界回归用例；由用户在开发容器内运行。"""

from dataclasses import replace
import unittest

import numpy as np

from remote_control.frames import Controller, Pose, XRFrame
from remote_control.mapping import InputUnavailable, MappingConfig, Session


def pose(x=0., rotation=None):
    """创建人工输入用的X轴位姿，可显式指定旋转矩阵。"""
    return Pose((x, 0., 0.), np.eye(3) if rotation is None else rotation)


def config(**changes):
    """创建人工坐标系配置并覆盖指定字段，不代表PICO现场参数。"""
    values = dict(basis=np.eye(3), translation_scale=1., rotation_mode='relative',
                  grip_threshold=.5, gripper_mode='hold', trigger_openings=None,
                  unknown_tracking='reject', base_frame='torso_link')
    values.update(changes)
    return MappingConfig.from_dict(values)


def frame(seq, x=0., grip=1., valid=True, connected=True, space='room', rotation=None,
          trigger=0., source=None):
    """创建同帧双手输入；源时间默认随序号推进。"""
    controller = Controller(pose(x, rotation), grip, trigger, tracking_valid=valid)
    return XRFrame(seq, seq*1000 if source is None else source, seq*.1,
                   controller, controller, connected, space, 'synthetic')


class MappingTests(unittest.TestCase):
    """覆盖坐标反射、Grip重接管、失败回滚及失效输入恢复。"""

    def setUp(self):
        """为每个用例构造独立会话与机器人参考。"""
        self.session = Session(config())
        self.robot = {'left': pose(1.), 'right': pose(2.)}
        self.hands = (.3, .7)

    def accept(self, value, robot=None):
        """映射并提交人工输入，返回已提交目标。"""
        proposal = self.session.propose(value, self.robot if robot is None else robot, self.hands)
        self.session.commit(proposal)
        return proposal.target

    def test_rejection_keeps_reference_and_goal(self):
        """移动候选被拒绝时保持先前参考，下一帧按原参考重新计算。"""
        self.accept(frame(1))
        rejected = self.session.propose(frame(2, x=.1), self.robot, self.hands)
        self.session.reject(rejected)
        accepted = self.accept(frame(3, x=.2))
        self.assertAlmostEqual(accepted.left.pose.position_m[0], 1.2)
        self.assertEqual(accepted.hand_openings, self.hands)

    def test_rejected_release_still_requires_new_reference(self):
        """Grip释放帧即使被拒绝，重按也必须使用新机器人反馈建立参考。"""
        self.accept(frame(1))
        release = self.session.propose(frame(2, x=.5, grip=0.), self.robot, self.hands)
        self.session.reject(release)
        refreshed = {'left': pose(1.1), 'right': pose(2.1)}
        target = self.accept(frame(3, x=.8), refreshed)
        self.assertAlmostEqual(target.left.pose.position_m[0], 1.1)
        target = self.accept(frame(4, x=.9), refreshed)
        self.assertAlmostEqual(target.left.pose.position_m[0], 1.2)

    def test_disconnect_requires_release_before_reacquire(self):
        """断链清除旧参考，持续握住Grip不能自动恢复旧会话。"""
        self.accept(frame(1))
        with self.assertRaises(InputUnavailable):
            self.accept(frame(2, connected=False))
        blocked = self.accept(frame(3, x=.5))
        self.assertFalse(blocked.left.enabled)
        self.accept(frame(4, x=.5, grip=0.))
        reacquired = self.accept(frame(5, x=.7))
        self.assertTrue(reacquired.left.enabled)
        self.assertAlmostEqual(reacquired.left.pose.position_m[0], 1.)

    def test_tracking_or_space_changes_invalidate_reference(self):
        """无效跟踪与参考空间改变均使映射候选失效。"""
        self.accept(frame(1))
        pending = self.session.propose(frame(2), self.robot, self.hands)
        self.session.reset('external_tracking_loss')
        with self.assertRaises(ValueError):
            self.session.commit(pending)
        with self.assertRaises(InputUnavailable):
            self.accept(frame(3, valid=False))
        with self.assertRaises(InputUnavailable):
            self.accept(frame(4, space='new_room'))

    def test_source_regression_and_duplicate_not_silently_consumed(self):
        """源时间回退产生异常，重复接收序号不再生成目标。"""
        self.accept(frame(1, source=100))
        with self.assertRaises(InputUnavailable):
            self.accept(frame(2, source=99))
        with self.assertRaises(ValueError):
            self.accept(frame(2, source=101))

    def test_missing_source_time_preserves_regression_check(self):
        """源时间中间缺失不清除已知时间，后续回退仍使会话失效。"""
        self.accept(frame(1, source=100))
        unknown = replace(frame(2), source_time_ns=None)
        accepted = self.accept(unknown)
        self.assertIsNone(accepted.source_time_ns)
        with self.assertRaises(InputUnavailable):
            self.accept(frame(3, source=99))

    def test_missing_source_time_allows_later_advancing_time(self):
        """未知源时间保持None，后续已知时间正常推进可继续只读映射。"""
        self.accept(replace(frame(1), source_time_ns=None))
        self.accept(frame(2, source=100))
        self.accept(replace(frame(3), source_time_ns=None))
        self.assertEqual(self.accept(frame(4, source=101)).source_time_ns, 101)

    def test_unknown_tracking_is_explicit(self):
        """未知跟踪仅在显式allow配置下可用于只读映射。"""
        with self.assertRaises(InputUnavailable):
            self.accept(frame(1, valid=None))
        self.session = Session(config(unknown_tracking='allow'))
        self.assertTrue(self.accept(frame(1, valid=None)).left.enabled)

    def test_reflection_conjugates_rotation(self):
        """左右手坐标反射同时变换位置和旋转增量。"""
        basis = np.diag((-1., 1., 1.))
        self.session = Session(config(basis=basis))
        self.accept(frame(1))
        rotation = np.array(((0., -1., 0.), (1., 0., 0.), (0., 0., 1.)))
        target = self.accept(frame(2, x=.1, rotation=rotation))
        self.assertAlmostEqual(target.left.pose.position_m[0], .9)
        np.testing.assert_allclose(target.left.pose.rotation_matrix, basis @ rotation @ basis.T)

    def test_trigger_mapping_only_when_enabled(self):
        """扳机端点映射仅在使能时更新，释放Grip后保持最后开度。"""
        self.session = Session(config(gripper_mode='trigger', trigger_openings=(1., 0.)))
        target = self.accept(frame(1, trigger=.2))
        self.assertAlmostEqual(target.hand_openings[0], .8)
        held = self.accept(frame(2, grip=0., trigger=1.))
        self.assertAlmostEqual(held.hand_openings[0], .8)
        self.assertFalse(held.left.enabled)

    def test_serialization_preserves_immutable_values(self):
        """回读保留摇杆、按钮、未知跟踪和两类时间字段。"""
        controller = Controller(pose(), .3, .7, {'primary': True}, None, (.2, -.4))
        value = XRFrame(1, None, .1, controller, controller, True, None, 'test')
        self.assertEqual(XRFrame.from_dict(value.to_dict()), value)


if __name__ == '__main__':
    unittest.main()
