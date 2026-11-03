"""验证同步日志阻塞后不会向只读IK交付过期XR帧。"""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from remote_control.mapping import InputUnavailable
from aimdk_functions import upper_body_teleop_observe as observe


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


if __name__ == '__main__':
    unittest.main()
