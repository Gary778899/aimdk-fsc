"""验证双侧原子提交、保持与分支拒绝；由用户在机器人容器内运行。"""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np

from remote_control.frames import Controller, Pose, TCPGoal, TargetFrame, XRFrame
from remote_control.mapping import MappingConfig, Session
from aimdk_functions.dual_arm_control_core import ControlFailure
from aimdk_functions.upper_body_cartesian_core import CapturedTarget
from aimdk_functions.upper_body_kinematics.parameters import EXECUTION_LIMITS_RAD
from aimdk_functions.upper_body_online_core import DualArmOnlineIK, OnlineIKFailure
from aimdk_functions.upper_body_teleop_observe import command_reference
from aimdk_functions.upper_body_teleop_replay import recovery_target, replay_frames


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
        """拒绝模型关节限位以外目标和未转换到torso_link的目标。"""
        outside = EXECUTION_LIMITS_RAD[0][1] - self.captured.arm_rad[0] + .01
        for frame in (self.frame(1, outside, .01), self.frame(1, base_frame='world')):
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

    def test_changes_above_ten_degrees_are_observed_without_rejection(self):
        """模型限位内大于10°的变化可提交，仍记录相邻及累计变化。"""
        candidate = self.solver.propose(self.frame(1, .2, .2))
        for row in candidate.report['arms'].values():
            self.assertTrue(row['exceeds_10deg_observation'])
            self.assertAlmostEqual(row['step_rad'], .2)
            self.assertAlmostEqual(row['excursion_rad'], .2)
        self.solver.commit(candidate)
        self.assertEqual(self.solver.last_input_seq, 1)

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
        self.assertEqual(self.solver.store.get().compensation_sides, ('left',))
        self.assertEqual(self.arms['left'].calls, 0)

    def test_superseded_candidate_keeps_target_until_release_and_regrip(self):
        """普通求解被Grip边沿取代时不提交，松开和重按仍建立正确参考。"""
        self.teleop_step(1, .02)
        original = self.solver.accepted
        pose = Pose((.1, 0., 0.), np.eye(3))
        pressed = Controller(pose, 1., 0., tracking_valid=True)
        released = Controller(pose, 0., 0., tracking_valid=True)
        frames = [XRFrame(2, 2000, 2., pressed, released),
                  XRFrame(3, 3000, 3., released, released),
                  XRFrame(4, 4000, 4., pressed, released)]
        decisions = iter((False, True, True))

        def before_commit():
            """模拟第一普通候选求解期间收到松开，后续边沿允许提交。"""
            return next(decisions)

        def hold_motion(value):
            """松开及重按仅提交保持／参考状态。"""
            return value.seq != 2

        diagnostics = Mock()
        count = replay_frames(frames, self.session, self.solver, diagnostics,
                              before_commit=before_commit, hold_motion=hold_motion)
        self.assertEqual(count, 2)
        self.assertEqual(self.solver.accepted, original)
        self.assertEqual(self.solver.last_input_seq, 4)
        self.assertEqual(self.session.describe()['sides']['left']['controller_reference'], pose.to_dict())
        superseded = [call for call in diagnostics.record.call_args_list if call.args[0] == 'ik_superseded']
        self.assertEqual(superseded[0].kwargs['input_seq'], 2)

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
        self.assertEqual(self.solver.store.get().compensation_sides, ('left', 'right'))

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

    def test_failure_log_preserves_previous_frame_and_candidate(self):
        """回放首个越限失败保存候选及前后参考，并停止消费后续输入。"""
        config = MappingConfig.from_dict(dict(
            basis=np.eye(3), translation_scale=1., rotation_mode='relative',
            grip_threshold=.5, gripper_mode='hold', trigger_openings=None,
            unknown_tracking='reject', base_frame='torso_link'))
        frames = []
        outside = EXECUTION_LIMITS_RAD[0][1] - self.captured.arm_rad[0] + .01
        for seq, x in enumerate((0., outside, outside+.01), 1):
            hand = Controller(Pose((x, 0., 0.), np.eye(3)), 1., 0., tracking_valid=True)
            frames.append(XRFrame(seq, seq*1000, float(seq), hand, hand))
        diagnostics = Mock()
        with self.assertRaises(OnlineIKFailure):
            replay_frames(frames, Session(config), self.solver, diagnostics)
        failures = [call.kwargs for call in diagnostics.record.call_args_list
                    if call.args[0] == 'ik_rejected']
        self.assertEqual(len(failures), 1)
        failure = failures[0]
        self.assertEqual(failure['input_seq'], 2)
        self.assertEqual(failure['previous_accepted']['input_frame']['seq'], 1)
        self.assertEqual(failure['ik_before'], failure['ik_after'])
        self.assertEqual(failure['mapping_before']['sides'], failure['mapping_after']['sides'])
        self.assertGreater(failure['result']['arms']['left']['step_rad'], .17)
        self.assertIn('candidate_rad', failure['result']['arms']['left'])
        inputs = [call.kwargs['frame']['seq'] for call in diagnostics.record.call_args_list
                  if call.args[0] == 'xr_input']
        self.assertEqual(inputs, [1, 2])

    def test_hold_published_reanchors_seed_preserves_origin_and_invalidates_proposal(self):
        """发布目标落后IK终点时，用中间目标重建种子，旧候选失效且累计参考保持。"""
        accepted = self.solver.propose(self.frame(1, .2, .2))
        self.solver.commit(accepted)
        pending = self.solver.propose(self.frame(2, .25, .25))
        origin, branches = self.solver.origin, {side: state.branch for side, state in self.solver.states.items()}
        joints = list(self.captured.arm_rad)
        joints[0] += .05
        joints[7] += .05
        held = CapturedTarget(tuple(joints), (.3, .7))
        self.solver.hold_published(held)
        self.assertEqual(self.solver.origin, origin)
        self.assertEqual(self.solver.last_input_seq, 1)
        self.assertEqual(self.solver.accepted, held)
        self.assertEqual(self.solver.store.get().target, held)
        for index, side in enumerate(('left', 'right')):
            self.assertEqual(self.solver.states[side].joints, held.arm_rad[index*7:index*7+7])
            self.assertEqual(self.solver.states[side].branch, branches[side])
        with self.assertRaises(ControlFailure):
            self.solver.commit(pending)

    def recovery_frame(self, seq, seconds, x=0., grip=1., trigger=.3):
        """构造双侧同步移动的XR帧；seconds用于确定恢复过渡进度。"""
        hand = Controller(Pose((x, 0., 0.), np.eye(3)), grip, trigger, tracking_valid=True)
        return XRFrame(seq, int(seconds * 1e9), seconds, hand, hand)

    def recovery_session(self):
        """构造带Trigger夹爪映射的会话，用于验证完整目标保持及过渡。"""
        return Session(MappingConfig.from_dict(dict(
            basis=np.eye(3), translation_scale=1., rotation_mode='relative',
            grip_threshold=.5, gripper_mode='trigger', trigger_openings=(0., 1.),
            unknown_tracking='reject', base_frame='torso_link')))

    def test_local_failure_holds_both_then_recovers_with_original_frame_time(self):
        """右侧无解保留双侧及夹爪；新目标可解后在0、0.25、0.5秒提交五次进度。"""
        session, diagnostics, stop = self.recovery_session(), Mock(), Mock()

        def frames():
            """逐帧注入空解，检查失败不更新、恢复首帧保持及中点正确。"""
            yield self.recovery_frame(1, 1.)
            before, states = self.solver.accepted, dict(self.solver.states)
            self.arms['right'].fail = True
            yield self.recovery_frame(2, 2., .2, trigger=.9)
            self.assertEqual(self.solver.accepted, before)
            self.assertEqual(self.solver.states, states)
            yield self.recovery_frame(3, 2.05, .2, trigger=.9)
            self.assertEqual(self.solver.accepted, before)
            self.arms['right'].fail = False
            yield self.recovery_frame(4, 3., .2, trigger=.9)
            self.assertEqual(self.solver.accepted, before)
            yield self.recovery_frame(5, 3.25, .2, trigger=.9)
            for index in (0, 7):
                self.assertAlmostEqual(self.solver.accepted.arm_rad[index], before.arm_rad[index] + .1)
            self.assertAlmostEqual(self.solver.accepted.hand_openings[0], .6)
            yield self.recovery_frame(6, 3.5, .2, trigger=.9)

        count = replay_frames(frames(), session, self.solver, diagnostics, on_error=stop,
                              ik_recovery_duration=.5)
        self.assertEqual(count, 4)
        stop.assert_not_called()
        self.assertEqual(self.solver.last_input_seq, 6)
        self.assertAlmostEqual(self.solver.accepted.arm_rad[0], self.captured.arm_rad[0] + .2)
        failures = [call.kwargs for call in diagnostics.record.call_args_list
                    if call.args[0] == 'ik_rejected']
        self.assertEqual([row['input_seq'] for row in failures], [2, 3])
        for row in failures:
            self.assertTrue(row['recoverable'])
            self.assertEqual(row['ik_before'], row['ik_after'])
            self.assertEqual(row['mapping_before']['sides'], row['mapping_after']['sides'])
        recoveries = [call.kwargs for call in diagnostics.record.call_args_list
                      if call.args[0] == 'ik_recovery']
        self.assertEqual([row['blend'] for row in recoveries], [0., .5, 1.])

    def test_release_during_recovery_holds_intermediate_and_regrip_uses_it(self):
        """恢复中松开两侧Grip保持中间目标，重按建立新参考并丢弃旧恢复终点。"""
        session, diagnostics = self.recovery_session(), Mock()

        def frames():
            """在恢复中点松开，再把手柄移远并重按。"""
            yield self.recovery_frame(1, 1.)
            self.arms['right'].fail = True
            yield self.recovery_frame(2, 2., .2)
            self.arms['right'].fail = False
            yield self.recovery_frame(3, 3., .2)
            yield self.recovery_frame(4, 3.25, .2)
            middle = self.solver.accepted
            yield self.recovery_frame(5, 3.3, .4, grip=0.)
            self.assertEqual(self.solver.accepted, middle)
            yield self.recovery_frame(6, 3.4, .6, grip=0.)
            self.assertEqual(self.solver.accepted, middle)
            yield self.recovery_frame(7, 3.5, .8)
            self.assertEqual(self.solver.accepted, middle)
            yield self.recovery_frame(8, 3.6, .8)
            yield self.recovery_frame(9, 4.1, .8)
            self.assertEqual(self.solver.accepted, middle)

        def hold_motion(frame):
            """模拟现场首次Grip、松开和重按边沿的保持语义。"""
            return frame.seq in (1, 5, 7)

        replay_frames(frames(), session, self.solver, diagnostics,
                      hold_motion=hold_motion, ik_recovery_duration=.5)
        self.assertAlmostEqual(self.solver.accepted.arm_rad[0], self.captured.arm_rad[0] + .1)
        self.assertAlmostEqual(session.describe()['sides']['left']['robot_reference']['position_m'][0],
                               self.captured.arm_rad[0] + .1)

    def test_execution_hold_callback_reanchors_recovery_to_published_intermediate(self):
        """拒绝候选后执行回调取消远端终点，后续恢复从已发布中间目标开始。"""
        session, diagnostics = self.recovery_session(), Mock()
        commits = Mock()
        held_joints = list(self.captured.arm_rad)
        held_joints[0] += .05
        held_joints[7] += .05
        held = CapturedTarget(tuple(held_joints), (.4, .4))

        def freeze(frame):
            """模拟主线程在无解诊断后同步执行侧已发布目标。"""
            self.assertEqual(frame.seq, 3)
            self.solver.hold_published(held)
            session.hold(self.solver.tcp_poses(), held.hand_openings)

        def frames():
            """先接受远端目标，再注入失败，最后恢复首帧应仍为发布中间目标。"""
            yield self.recovery_frame(1, 1.)
            yield self.recovery_frame(2, 1.2, .2)
            self.arms['right'].fail = True
            yield self.recovery_frame(3, 1.3, .3)
            self.assertEqual(self.solver.accepted, held)
            self.assertEqual(session.describe()['sides']['left']['opening'], .4)
            self.arms['right'].fail = False
            yield self.recovery_frame(4, 1.4, .3)
            self.assertEqual(self.solver.accepted, held)

        replay_frames(frames(), session, self.solver, diagnostics, ik_recovery_duration=.5,
                      on_commit=commits, on_local_failure=freeze)
        self.assertEqual(commits.call_count, 3)
        failure = next(c.kwargs for c in diagnostics.record.call_args_list if c.args[0]=='ik_rejected')
        self.assertEqual(failure['ik_before'], failure['ik_after'])

    def test_recovery_keeps_joint_limit_failure_fatal(self):
        """打开空解恢复后，已计算候选越限仍停发并终止消费后续帧。"""
        outside = EXECUTION_LIMITS_RAD[0][1] - self.captured.arm_rad[0] + .01
        frames = [self.recovery_frame(1, 1.), self.recovery_frame(2, 2., outside),
                  self.recovery_frame(3, 3.)]
        diagnostics, stop = Mock(), Mock()
        with self.assertRaises(OnlineIKFailure):
            replay_frames(frames, self.recovery_session(), self.solver, diagnostics,
                          on_error=stop, ik_recovery_duration=.5)
        stop.assert_called_once()
        failures = [call.kwargs for call in diagnostics.record.call_args_list
                    if call.args[0] == 'ik_rejected']
        self.assertFalse(failures[0]['recoverable'])
        self.assertEqual(self.solver.last_input_seq, 1)

    def test_stale_failed_candidate_is_superseded_before_recovery(self):
        """局部空解期间出现新Grip边沿时撤销旧结果，继续处理松开并保持目标。"""
        session, diagnostics, stop = self.recovery_session(), Mock(), Mock()
        decisions = iter((True, False, True))

        def before_commit():
            """第二帧求解期间模拟收到Grip边沿。"""
            return next(decisions)

        def frames():
            """首帧建参考、普通帧空解、最后松开。"""
            yield self.recovery_frame(1, 1.)
            self.arms['right'].fail = True
            yield self.recovery_frame(2, 2., .2)
            yield self.recovery_frame(3, 3., grip=0.)

        count = replay_frames(frames(), session, self.solver, diagnostics,
                              before_commit=before_commit, on_error=stop, ik_recovery_duration=.5)
        self.assertEqual(count, 2)
        stop.assert_not_called()
        self.assertEqual(self.solver.last_input_seq, 3)
        self.assertEqual([call.kwargs['input_seq'] for call in diagnostics.record.call_args_list
                          if call.args[0] == 'ik_superseded'], [2])

    def test_recovery_pose_interpolates_rotation_and_preserves_disabled_side(self):
        """180度混合符号旋转轴的中间姿态有效，未使能侧及原始时间保持。"""
        poses = self.solver.tcp_poses()
        rotation = np.array(((0., -1., 0.), (-1., 0., 0.), (0., 0., -1.)))
        target = TargetFrame(1, 1234, 10.,
                             TCPGoal(Pose(np.asarray(poses['left'].position_m) + (.2, 0., 0.),
                                          rotation), True),
                             TCPGoal(Pose((100., 0., 0.), rotation), False), (.9, .1))
        middle = recovery_target(target, poses, self.captured.hand_openings, .5)
        np.testing.assert_allclose(np.asarray(middle.left.pose.rotation_matrix) @ middle.left.pose.rotation_matrix,
                                   rotation, atol=1e-12)
        self.assertAlmostEqual(middle.left.pose.position_m[0], poses['left'].position_m[0] + .1)
        self.assertEqual(middle.right.pose, poses['right'])
        self.assertEqual(middle.hand_openings[1], self.captured.hand_openings[1])
        self.assertEqual((middle.input_seq, middle.source_time_ns, middle.received_monotonic_s),
                         (1, 1234, 10.))


if __name__ == '__main__':
    unittest.main()
