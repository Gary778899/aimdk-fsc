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
    """使用SDK和ROS替身验证输入年龄的检查位置，不加载原生SDK。"""

    def test_age_is_checked_after_recording(self):
        """模拟consume同步写盘后时间推进，过期帧必须在yield前被拒绝。"""
        frame = SimpleNamespace(seq=1, received_monotonic_s=0., source_time_ns=100)
        event = SimpleNamespace(kind='device_state', device_id='pico')
        source = Mock(backlog=0)
        source.read.return_value = event
        recorder = Mock()
        recorder.consume.return_value = frame
        port = Mock()
        port.rclpy.ok.return_value = True
        options = SimpleNamespace(duration=None, input_timeout=.5, device_id='pico')
        with patch.object(observe.time, 'monotonic', side_effect=[0., 1.]):
            with self.assertRaisesRegex(InputUnavailable, '积压帧年龄'):
                next(observe.live_frames(source, recorder, port, options))
        recorder.consume.assert_called_once_with(event, 0)


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
