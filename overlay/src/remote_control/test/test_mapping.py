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

    def test_grip_head_yaw_maps_forward_and_freezes_head_motion(self):
        """任意水平朝向的前推映射为机器人前推，Grip内转头和平移不改变目标。"""
        basis = np.array(((0., 0., -1.), (-1., 0., 0.), (0., 1., 0.)))
        yaw = np.array(((0., -1., 0.), (1., 0., 0.), (0., 0., 1.)))
        session = Session(config(basis=basis, reference_mode='grip_head_yaw'))
        head = Pose((1., 2., 3.), basis.T @ yaw @ basis)
        first = replace(frame(1), head=head)
        proposal = session.propose(first, self.robot, self.hands)
        session.commit(proposal)
        angle = .2
        world_rotation = np.array(((np.cos(angle), 0., np.sin(angle)), (0., 1., 0.),
                                   (-np.sin(angle), 0., np.cos(angle))))
        controller = replace(first.left, pose=Pose(basis.T @ (0., .1, 0.),
                                                   basis.T @ world_rotation @ basis))
        moved = replace(frame(2), left=controller, right=controller,
                        head=Pose((5., 6., 7.), np.eye(3)))
        proposal = session.propose(moved, self.robot, self.hands)
        np.testing.assert_allclose(proposal.target.left.pose.position_m, (1.1, 0., 0.), atol=1e-12)
        expected_rotation = ((1., 0., 0.), (0., np.cos(angle), -np.sin(angle)),
                             (0., np.sin(angle), np.cos(angle)))
        np.testing.assert_allclose(proposal.target.left.pose.rotation_matrix, expected_rotation, atol=1e-12)
        self.assertAlmostEqual(proposal.report['left']['head_yaw_rad'], np.pi/2)
        previous_target = proposal.target
        session.commit(proposal)
        proposal = session.propose(replace(moved, seq=3, received_monotonic_s=.3,
                                           source_time_ns=3000, head=head), self.robot, self.hands)
        self.assertEqual(proposal.target.left.pose, previous_target.left.pose)

    def test_grip_head_yaw_vertical_forward_is_undefined(self):
        """头显前向垂直时无法确定水平朝向，拒绝参考而不使用身份旋转替代。"""
        session = Session(config(reference_mode='grip_head_yaw'))
        vertical = ((0., 0., 1.), (0., 1., 0.), (-1., 0., 0.))
        with self.assertRaisesRegex(InputUnavailable, '水平投影为零'):
            session.propose(replace(frame(1), head=pose(rotation=vertical)), self.robot, self.hands)
        self.assertIsNone(session.describe()['sides']['left']['controller_reference'])

    def test_regrip_yaw_is_per_side_and_rejection_preserves_heading(self):
        """重按只更新对应侧朝向，被拒绝的新朝向不推进已提交参考。"""
        session = Session(config(reference_mode='grip_head_yaw'))
        first = replace(frame(1), head=pose())
        proposal = session.propose(first, self.robot, self.hands)
        session.commit(proposal)
        released = replace(frame(2), head=pose(), right=replace(first.right, grip=0.))
        proposal = session.propose(released, self.robot, self.hands)
        session.commit(proposal)
        yaw = ((0., -1., 0.), (1., 0., 0.), (0., 0., 1.))
        regrip = replace(frame(3), head=pose(rotation=yaw))
        before = session.describe()['sides']
        proposal = session.propose(regrip, self.robot, self.hands)
        self.assertAlmostEqual(proposal.report['left']['head_yaw_rad'], 0.)
        self.assertAlmostEqual(proposal.report['right']['head_yaw_rad'], np.pi/2)
        session.reject(proposal)
        self.assertEqual(session.describe()['sides'], before)
        proposal = session.propose(replace(regrip, seq=4, received_monotonic_s=.4),
                                   self.robot, self.hands)
        session.commit(proposal)
        self.assertAlmostEqual(session.describe()['sides']['right']['head_yaw_rad'], np.pi/2)

    def test_head_mode_requires_head_and_ignores_pitch_roll(self):
        """新模式缺少Head时明确报错；头部俯仰横滚不改变水平前推方向。"""
        session = Session(config(reference_mode='grip_head_yaw'))
        with self.assertRaisesRegex(InputUnavailable, '同帧Head'):
            session.observe_input(frame(1))
        session = Session(config(reference_mode='grip_head_yaw'))
        pitch = np.array(((np.cos(.4), 0., np.sin(.4)), (0., 1., 0.),
                          (-np.sin(.4), 0., np.cos(.4))))
        roll = np.array(((1., 0., 0.), (0., np.cos(.3), -np.sin(.3)),
                         (0., np.sin(.3), np.cos(.3))))
        proposal = session.propose(replace(frame(1), head=pose(rotation=pitch @ roll)),
                                   self.robot, self.hands)
        session.commit(proposal)
        proposal = session.propose(replace(frame(2, x=.1), head=pose()), self.robot, self.hands)
        np.testing.assert_allclose(proposal.target.left.pose.position_m, (1.1, 0., 0.), atol=1e-12)

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

    def test_edge_holds_other_arm_motion_and_trigger(self):
        """单侧松开边沿不推进另一侧位姿或夹爪，下一普通帧继续原参考。"""
        self.accept(frame(1))
        previous = self.accept(frame(2, x=.1))
        accepted_poses = dict(left=previous.left.pose, right=previous.right.pose)
        value = frame(3, x=.4)
        value = replace(value, right=replace(value.right, grip=0.))
        edge = self.session.propose(value, accepted_poses, self.hands, hold_motion=True)
        self.assertEqual(edge.target.left.pose, previous.left.pose)
        self.assertEqual(edge.target.right.pose, previous.right.pose)
        self.session.commit(edge)
        following = self.session.propose(frame(4, x=.5), accepted_poses, self.hands)
        self.assertAlmostEqual(following.target.left.pose.position_m[0], 1.5)
        self.assertAlmostEqual(following.target.right.pose.position_m[0], 2.1)

        trigger_session = Session(config(gripper_mode='trigger', trigger_openings=(0., 1.)))
        first = trigger_session.propose(frame(1, trigger=.2), self.robot, self.hands)
        trigger_session.commit(first)
        edge = trigger_session.propose(frame(2, trigger=.8), self.robot, self.hands, hold_motion=True)
        self.assertEqual(edge.target.hand_openings, (.2, .2))

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

    def test_diagnostics_capture_reflection_and_rejected_reference(self):
        """诊断同时保留输入和映射增量，拒绝候选后已提交参考保持不变。"""
        basis = np.diag((-1., 1., 1.))
        self.session = Session(config(basis=basis, translation_scale=2.))
        initial = self.session.propose(frame(1), self.robot, self.hands)
        self.assertEqual(initial.report['left']['event'], 'reference_established')
        self.session.commit(initial)
        before = self.session.describe()
        rotation = np.array(((0., -1., 0.), (1., 0., 0.), (0., 0., 1.)))
        proposal = self.session.propose(frame(2, x=.1, rotation=rotation), self.robot, self.hands)
        row = proposal.report['left']
        np.testing.assert_allclose(row['input_translation_m'], [.1, 0., 0.])
        np.testing.assert_allclose(row['tcp_translation_m'], [-.2, 0., 0.])
        np.testing.assert_allclose(row['input_rotation_matrix'], rotation)
        np.testing.assert_allclose(row['tcp_rotation_matrix'], basis @ rotation @ basis.T)
        self.session.reject(proposal)
        self.assertEqual(self.session.describe()['sides'], before['sides'])

    def test_fixed_rotation_diagnostics_and_grip_events(self):
        """固定姿态模式记录真实手柄旋转，TCP旋转增量为零；松开重按有独立事件。"""
        self.session = Session(config(rotation_mode='fixed'))
        self.accept(frame(1))
        rotation = np.array(((0., -1., 0.), (1., 0., 0.), (0., 0., 1.)))
        proposal = self.session.propose(frame(2, rotation=rotation), self.robot, self.hands)
        np.testing.assert_allclose(proposal.report['left']['input_rotation_matrix'], rotation)
        np.testing.assert_allclose(proposal.report['left']['tcp_rotation_matrix'], np.eye(3))
        self.session.commit(proposal)
        release = self.session.propose(frame(3, grip=0.), self.robot, self.hands)
        self.assertEqual(release.report['left']['event'], 'released')
        self.session.commit(release)
        regrip = self.session.propose(frame(4, x=.4), self.robot, self.hands)
        self.assertEqual(regrip.report['left']['event'], 'reference_established')
        np.testing.assert_allclose(regrip.report['left']['input_translation_m'], np.zeros(3))

    def test_serialization_preserves_immutable_values(self):
        """回读保留摇杆、按钮、未知跟踪和两类时间字段。"""
        controller = Controller(pose(), .3, .7, {'primary': True}, None, (.2, -.4))
        value = XRFrame(1, None, .1, controller, controller, True, None, 'test')
        self.assertEqual(XRFrame.from_dict(value.to_dict()), value)


if __name__ == '__main__':
    unittest.main()
