"""验证同步日志阻塞后不会向只读IK交付过期XR帧。"""

from types import SimpleNamespace
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from remote_control.mapping import InputUnavailable
from aimdk_functions import upper_body_teleop_observe as observe
from aimdk_functions.dual_arm_control_core import ARM_NAMES
from aimdk_functions.upper_body_teleop_replay import read_snapshot


class ObserveTimingTests(unittest.TestCase):
    """验证主线程只消费选中帧，SDK读取已迁移到独立接收器。"""

    def test_selected_frame_is_delivered_without_reading_sdk(self):
        """主线程等待时不复用旧帧，边沿元数据随选中帧记录。"""
        frame = SimpleNamespace(seq=1, received_monotonic_s=100.)
        receiver = Mock(finished=False, coalesced=4)
        receiver.take.return_value = SimpleNamespace(frame=frame, edge=True, generation=1)
        port = Mock()
        port.rclpy.ok.return_value = True
        options = SimpleNamespace(execute=True)
        with patch.object(observe.time, 'monotonic', return_value=100.):
            self.assertIs(next(observe.live_frames(receiver, port, options)), frame)
        port.observe_input.assert_called_once_with(frame)
        receiver.take.assert_called_once_with(100.)

    def test_finished_input_ends_stream(self):
        """接收会话结束且无待交付帧时退出。"""
        receiver = Mock(finished=True)
        receiver.take.return_value = None
        port = Mock()
        port.rclpy.ok.return_value = True
        self.assertEqual(list(observe.live_frames(receiver, port, SimpleNamespace(execute=False))), [])

    def test_input_failure_propagates(self):
        """主线程选择帧时传播接收线程的锁存错误。"""
        receiver = Mock()
        receiver.take.side_effect = InputUnavailable('XR积压帧年龄超过input-timeout')
        port = Mock()
        port.rclpy.ok.return_value = True
        with self.assertRaisesRegex(InputUnavailable, '积压帧年龄'):
            next(observe.live_frames(receiver, port, SimpleNamespace(execute=True)))


class SnapshotReferenceTests(unittest.TestCase):
    """验证命令起点回放及历史反馈对照的显式选择。"""

    def test_snapshot_command_source_and_explicit_state_comparison(self):
        """两种来源不混用，按名称重排，夹爪始终保留实际开度。"""
        command, actual = tuple(index * .01 for index in range(14)), (0.,) * 14
        fields = dict(arm_names=list(reversed(ARM_NAMES)), streams=dict(
            arm=dict(positions=list(reversed(actual))),
            arm_command=dict(positions=list(reversed(command))), hand=dict(positions=[.3, .7])))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'snapshot.jsonl'
            path.write_text(json.dumps(dict(event='snapshot', fields=fields)) + '\n', encoding='utf-8')
            target, _ = read_snapshot(path, 0)
            self.assertEqual(target.arm_rad, command)
            self.assertEqual(target.hand_openings, (.3, .7))
            state, _ = read_snapshot(path, 0, 'state')
            self.assertEqual(state.arm_rad, actual)
            del fields['streams']['arm_command']
            path.write_text(json.dumps(dict(event='snapshot', fields=fields)) + '\n', encoding='utf-8')
            with self.assertRaisesRegex(ValueError, '缺少arm_command'):
                read_snapshot(path, 0)
            state, _ = read_snapshot(path, 0, 'state')
            self.assertEqual(state.arm_rad, actual)


if __name__ == '__main__':
    unittest.main()
