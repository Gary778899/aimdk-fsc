"""遥操持续发布的首帧、保持、输入超时及停发契约；由用户在机器人容器运行。"""

from types import SimpleNamespace
import math
import unittest
from unittest.mock import Mock, patch

from aimdk_functions.dual_arm_control_core import ControlFailure
from aimdk_functions.upper_body_cartesian_core import CapturedTarget
from aimdk_functions.upper_body_kinematics.parameters import EXECUTION_LIMITS_RAD
from aimdk_functions.upper_body_online_core import OnlineTargetStore
from aimdk_functions.upper_body_teleop_execution import UpperBodyTeleopExecutor, approach_joints


def message(node, target, sequence):
    """构造无需ROS消息库的发送记录替身，保留递增序号。"""
    return SimpleNamespace(header=SimpleNamespace(
        sequence=sequence, stamp=SimpleNamespace(sec=100, nanosec=0)))


class OnlineExecutionTests(unittest.TestCase):
    """注入反馈和发布器替身，检查真实发布适配的行为边界。"""

    def setUp(self):
        """使用限位中点的完整HAL目标，不加载ROS节点或硬件。"""
        self.options = SimpleNamespace(input_timeout=.5, compensation_duration=5., joint_speed_deg_s=30.)
        self.log = Mock()
        with patch('aimdk_functions.upper_body_teleop_execution.UpperBodyExecutor.__init__',
                   return_value=None):
            self.port = UpperBodyTeleopExecutor(Mock(), Mock(), self.options, self.log)
        self.target = CapturedTarget(tuple((lo+hi)/2 for lo, hi in EXECUTION_LIMITS_RAD), (.3, .7))
        self.store = OnlineTargetStore(self.target)
        self.port.options, self.port.diagnostics = self.options, self.log
        self.port.captured, self.port.node = self.target, Mock()
        self.port.feedback = Mock()
        self.port.feedback.checked.return_value = dict(arm=SimpleNamespace(
            positions=self.target.arm_rad, serial=7, stamp=99.97, received=99.98))
        self.port.publisher, self.port.timer = Mock(), Mock()
        self.port.sequence, self.port.last_publish = 0, None
        self.port.last_sent = None
        self.port.stopped = False
        self.port.command_target = None
        self.port.compensation = None
        self.frame = SimpleNamespace(seq=1, received_monotonic_s=100., source_time_ns=1000)

    def test_no_grip_has_no_command_and_first_frame_matches_hal(self):
        """首次接管前定时器不发送；零增量首帧严格保留完整HAL关节命令。"""
        self.port.publish()
        self.port.publisher.publish.assert_not_called()
        with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic', return_value=100.), \
                patch('aimdk_functions.upper_body_teleop_execution.build_upper_body_command',
                      side_effect=message) as build:
            self.port.begin_online(self.store, self.frame)
        self.assertEqual(build.call_args.args[1].arm_rad, self.target.arm_rad)
        self.assertEqual(self.port.last_sent, self.target)
        self.assertEqual(self.port.publisher.publish.call_count, 1)

    def test_tracking_over_five_degrees_is_observed_and_publishing_continues(self):
        """实际位置偏差超过5度仍发送，日志区分新目标和此前成功发送目标，首帧无前目标。"""
        actual = list(self.target.arm_rad)
        actual[10] += .12
        self.port.feedback.checked.return_value = dict(arm=SimpleNamespace(
            positions=tuple(actual), serial=8, stamp=99.97, received=99.98))
        with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic', return_value=100.), \
                patch('aimdk_functions.upper_body_teleop_execution.build_upper_body_command', side_effect=message):
            self.port.begin_online(self.store, self.frame)
        first = [call.kwargs for call in self.log.record.call_args_list
                 if call.args[0] == 'command_published'][-1]
        self.assertTrue(first['tracking_exceeds_5deg_observation'])
        self.assertIsNone(first['previous_target_tracking_error_rad'])
        self.assertIsNone(first['previous_command_sequence'])
        joints = list(self.target.arm_rad)
        joints[10] -= .02
        changed = CapturedTarget(tuple(joints), self.target.hand_openings)
        self.store.put(changed, 2)
        with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic', return_value=100.02), \
                patch('aimdk_functions.upper_body_teleop_execution.build_upper_body_command', side_effect=message):
            self.port.publish()
        last = [call.kwargs for call in self.log.record.call_args_list
                if call.args[0] == 'command_published'][-1]
        self.assertAlmostEqual(last['tracking_error_rad'][10], .12 + math.radians(.6))
        self.assertAlmostEqual(last['previous_target_tracking_error_rad'][10], .12)
        self.assertEqual(last['previous_sent_arm_rad'], list(self.target.arm_rad))
        self.assertEqual(last['previous_command_sequence'], 0)
        self.assertEqual(last['actual_arm_rad'], actual)
        self.assertTrue(last['previous_target_tracking_exceeds_5deg_observation'])
        self.assertEqual(self.port.publisher.publish.call_count, 2)
        self.assertFalse(self.port.stopped)
        self.assertIsNone(self.port.background_error)

    def test_publish_records_selected_feedback_and_call_interval(self):
        """发布日志关联本次使用的反馈快照，并保留发布调用的开始和结束时刻。"""
        with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic',
                   side_effect=[100., 100., 100.001, 100.003]), \
                patch('aimdk_functions.upper_body_teleop_execution.build_upper_body_command',
                      side_effect=message):
            self.port.begin_online(self.store, self.frame)
        row = [call.kwargs for call in self.log.record.call_args_list
               if call.args[0] == 'command_published'][-1]
        self.assertEqual(row['arm_feedback_serial'], 7)
        self.assertEqual(row['arm_feedback_stamp_s'], 99.97)
        self.assertEqual(row['arm_feedback_received_monotonic_s'], 99.98)
        self.assertEqual(row['publish_begin_monotonic_s'], 100.001)
        self.assertEqual(row['publish_end_monotonic_s'], 100.003)

    def test_feedback_failure_still_stops_publishing(self):
        """跟踪仅观测后，反馈故障仍阻止发送并锁存异常。"""
        with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic', return_value=100.), \
                patch('aimdk_functions.upper_body_teleop_execution.build_upper_body_command', side_effect=message):
            self.port.begin_online(self.store, self.frame)
        self.port.feedback.checked.side_effect = ControlFailure('arm反馈故障')
        with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic', return_value=100.02):
            with self.assertRaisesRegex(ControlFailure, '反馈故障'):
                self.port.publish()
        self.assertTrue(self.port.stopped)
        self.assertEqual(self.port.publisher.publish.call_count, 1)

    def test_hold_reuses_target_then_consumes_complete_new_snapshot(self):
        """没有新IK时保持完整目标，新快照统一替换双侧目标并关联输入序号。"""
        with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic', return_value=100.), \
                patch('aimdk_functions.upper_body_teleop_execution.build_upper_body_command', side_effect=message):
            self.port.begin_online(self.store, self.frame)
        with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic', return_value=100.02), \
                patch('aimdk_functions.upper_body_teleop_execution.build_upper_body_command', side_effect=message):
            self.port.publish()
            self.assertEqual(self.port.last_sent, self.target)
            joints = list(self.target.arm_rad)
            joints[0] += .01
            joints[7] += .02
            changed = CapturedTarget(tuple(joints), self.target.hand_openings)
            self.store.put(changed, 2)
        with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic', return_value=100.06), \
                patch('aimdk_functions.upper_body_teleop_execution.build_upper_body_command', side_effect=message):
            self.port.publish()
        self.assertEqual(self.port.last_sent, changed)
        rows = [call.kwargs for call in self.log.record.call_args_list if call.args[0] == 'command_published']
        self.assertEqual(rows[-1]['input_seq'], 2)
        self.assertEqual(rows[-1]['arm_rad'], list(changed.arm_rad))

    def test_large_goal_advances_without_new_ik_and_grip_hold_cancels_remainder(self):
        """大步目标分次推进，无新IK也继续；Grip冻结后取消剩余运动并从冻结点重启。"""
        with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic', return_value=100.), \
                patch('aimdk_functions.upper_body_teleop_execution.build_upper_body_command', side_effect=message):
            self.port.begin_online(self.store, self.frame)
        goal = self.target.for_arm('left', tuple(q + (.2 if i == 0 else 0.)
                                               for i, q in enumerate(self.target.arm_rad[:7])))
        value = self.store.put(goal, 2)
        self.port.accept_online(value, False)
        for stamp in (100.02, 100.04):
            with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic', return_value=stamp), \
                    patch('aimdk_functions.upper_body_teleop_execution.build_upper_body_command', side_effect=message):
                self.port.publish()
        held = self.port.hold_current()
        self.assertAlmostEqual(held.arm_rad[0]-self.target.arm_rad[0], math.radians(1.2))
        self.assertEqual(self.store.get().target, goal)
        with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic', return_value=100.06), \
                patch('aimdk_functions.upper_body_teleop_execution.build_upper_body_command', side_effect=message):
            self.port.publish()
        self.assertEqual(self.port.last_ideal, held)
        self.port.accept_online(self.store.put(held, 3), True)
        self.port.accept_online(self.store.put(goal, 4), False)
        with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic', return_value=100.08), \
                patch('aimdk_functions.upper_body_teleop_execution.build_upper_body_command', side_effect=message):
            self.port.publish()
        self.assertAlmostEqual(self.port.last_ideal.arm_rad[0]-held.arm_rad[0], math.radians(.6))
        row = [c.kwargs for c in self.log.record.call_args_list if c.args[0]=='command_published'][-1]
        self.assertEqual(row['ik_goal_arm_rad'], list(goal.arm_rad))
        self.assertEqual(row['ideal_arm_rad'], list(self.port.last_ideal.arm_rad))

    def test_common_progress_preserves_dual_arm_ratio_without_overshoot(self):
        """左右不同幅度使用共同进度，20ms最大推进0.6度，接近终点时精确到达。"""
        joints = list(self.target.arm_rad)
        joints[0] += math.radians(10.)
        joints[7] -= math.radians(5.)
        goal = CapturedTarget(tuple(joints), (.8, .2))
        next_target, _ = approach_joints(self.target, goal, .02, math.radians(30.))
        self.assertAlmostEqual(math.degrees(next_target.arm_rad[0]-self.target.arm_rad[0]), .6)
        self.assertAlmostEqual(math.degrees(next_target.arm_rad[7]-self.target.arm_rad[7]), -.3)
        self.assertEqual(next_target.hand_openings, goal.hand_openings)
        jittered, _ = approach_joints(self.target, goal, .035, math.radians(30.))
        self.assertAlmostEqual(math.degrees(jittered.arm_rad[0]-self.target.arm_rad[0]), 1.05)
        arrived, _ = approach_joints(next_target, goal, 1., math.radians(30.))
        self.assertEqual(arrived, goal)

    def test_illegal_goal_is_rejected_even_when_intermediate_would_be_in_range(self):
        """渐进不能掩盖非法远端目标，越限仍在发送前停发。"""
        with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic', return_value=100.), \
                patch('aimdk_functions.upper_body_teleop_execution.build_upper_body_command', side_effect=message):
            self.port.begin_online(self.store, self.frame)
        joints = list(self.target.arm_rad)
        joints[0] = EXECUTION_LIMITS_RAD[0][1] + .01
        self.store.put(CapturedTarget(tuple(joints), self.target.hand_openings), 2)
        with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic', return_value=100.02):
            with self.assertRaises(ControlFailure):
                self.port.publish()
        self.assertEqual(self.port.publisher.publish.call_count, 1)
        self.assertEqual(self.port.last_ideal, self.target)
        self.assertTrue(self.port.stopped)

    def test_timeout_stops_publishing_and_new_target_cannot_restart(self):
        """输入超过0.5秒后停发，之后更新缓存也不能自动恢复。"""
        with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic', return_value=100.), \
                patch('aimdk_functions.upper_body_teleop_execution.build_upper_body_command', side_effect=message):
            self.port.begin_online(self.store, self.frame)
        with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic', return_value=100.51):
            with self.assertRaisesRegex(ControlFailure, '接收／处理年龄'):
                self.port.publish()
        self.assertTrue(self.port.stopped)
        self.port.timer.cancel.assert_called()
        self.store.put(self.target, 2)
        self.port.publish()
        self.assertEqual(self.port.publisher.publish.call_count, 1)
        with self.assertRaises(ControlFailure):
            self.port.check_background()

    def test_advancing_receive_time_does_not_hide_frozen_source(self):
        """持续收到新帧但源时间不推进时，发布线程仍按源时间停滞中止。"""
        with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic', return_value=100.), \
                patch('aimdk_functions.upper_body_teleop_execution.build_upper_body_command', side_effect=message):
            self.port.begin_online(self.store, self.frame)
        self.port.observe_input(SimpleNamespace(seq=2, received_monotonic_s=100.5, source_time_ns=1000))
        with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic', return_value=100.51):
            with self.assertRaisesRegex(ControlFailure, '源时间停滞'):
                self.port.publish()
        self.assertEqual(self.port.publisher.publish.call_count, 1)

    def test_continuous_receive_does_not_hide_blocked_ik(self):
        """接收线程持续更新时，主线程选中帧超时仍停止发布。"""
        with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic', return_value=100.), \
                patch('aimdk_functions.upper_body_teleop_execution.build_upper_body_command', side_effect=message):
            self.port.begin_online(self.store, self.frame)
        self.port.input_receiver = Mock(last_received=100.5)
        with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic', return_value=100.51):
            with self.assertRaisesRegex(ControlFailure, '接收／处理年龄'):
                self.port.publish()
        self.port.input_receiver.check.assert_called_with(100.51)
        self.assertTrue(self.port.stopped)
        self.assertEqual(self.port.publisher.publish.call_count, 1)

    def test_compensation_ramps_per_side_without_changing_ideal_store(self):
        """首次补偿为零，重按不重置，未接管侧保持，IK缓存始终保存理想关节。"""
        def apply_left(captured, ideal, values, blend):
            """以已知小偏置模拟左臂补偿，供检查发送与理想目标分离。"""
            joints = list(ideal.arm_rad[:7])
            joints[0] += .02 * blend
            return captured.for_arm('left', joints), dict(blend=blend)

        left, right = Mock(), Mock()
        left.apply.side_effect = apply_left
        self.port.compensation = SimpleNamespace(calculators=dict(left=left, right=right))
        self.store.put(self.target, 1, ('left',))
        with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic', return_value=100.), \
                patch('aimdk_functions.upper_body_teleop_execution.build_upper_body_command', side_effect=message):
            self.port.begin_online(self.store, self.frame)
        self.assertEqual(self.port.last_sent, self.target)
        self.assertEqual(left.apply.call_args.args[-1], 0.)
        self.port.observe_input(SimpleNamespace(seq=2, received_monotonic_s=102.5, source_time_ns=2000))
        self.port.last_publish = 102.48
        with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic', return_value=102.5), \
                patch('aimdk_functions.upper_body_teleop_execution.build_upper_body_command', side_effect=message):
            self.port.publish()
        self.assertAlmostEqual(self.port.last_sent.arm_rad[0] - self.target.arm_rad[0], .01)
        self.assertEqual(self.port.last_sent.arm_rad[7:], self.target.arm_rad[7:])
        self.store.put(self.target, 3, ('left',))
        self.port.observe_input(SimpleNamespace(seq=3, received_monotonic_s=103., source_time_ns=3000))
        self.port.last_publish = 102.98
        with patch('aimdk_functions.upper_body_teleop_execution.time.monotonic', return_value=103.), \
                patch('aimdk_functions.upper_body_teleop_execution.build_upper_body_command', side_effect=message):
            self.port.publish()
        self.assertEqual(self.port.compensation_started['left'], 100.)
        self.assertGreater(left.apply.call_args.args[-1], .5)
        self.assertEqual(self.store.get().target, self.target)
        right.apply.assert_not_called()


if __name__ == '__main__':
    unittest.main()
