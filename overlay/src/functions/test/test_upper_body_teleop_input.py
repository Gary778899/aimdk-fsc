"""连续接收、最新位姿及Grip边沿并发契约；由用户在机器人容器运行。"""

from dataclasses import replace
import queue
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from remote_control.frames import Controller, Pose, XRFrame
from remote_control.mapping import InputUnavailable, MappingConfig
from aimdk_functions.upper_body_teleop_input import LatestInput


def frame(seq, received, left=0., right=0., source=None):
    """创建完整人工双手帧；seq默认也作为单调源时间。"""
    pose = Pose((0., 0., 0.), ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.)))
    return XRFrame(seq, seq if source is None else source, received,
                   Controller(pose, left, 0., tracking_valid=True),
                   Controller(pose, right, 0., tracking_valid=True), device_id='pico')


class LatestInputTests(unittest.TestCase):
    """检查完整帧选择、边沿保序、旧候选撤销及持续接收。"""

    def setUp(self):
        """以固定阈值创建接收器，避免加载SDK或ROS。"""
        config = MappingConfig.from_dict(dict(
            basis=((1., 0., 0.), (0., 1., 0.), (0., 0., 1.)), translation_scale=1.,
            rotation_mode='relative', grip_threshold=.9, gripper_mode='hold',
            trigger_openings=None, unknown_tracking='reject', base_frame='torso_link'))
        self.source, self.recorder, self.log = Mock(backlog=0), Mock(), Mock()
        self.options = SimpleNamespace(execute=True, input_timeout=.5, duration=None, device_id='pico')
        self.receiver = LatestInput(self.source, self.recorder, config, self.log, self.options)

    def test_pose_selection_uses_latest_without_catching_up(self):
        """普通位姿50ms内合并，晚于计划时只取最新一帧且不补算。"""
        self.receiver.accept(frame(1, 100.), 100.)
        self.assertEqual(self.receiver.take(100.).frame.seq, 1)
        self.receiver.accept(frame(2, 100.01), 100.01)
        self.receiver.accept(frame(3, 100.02), 100.02)
        self.assertIsNone(self.receiver.take(100.04))
        self.assertEqual(self.receiver.take(100.051).frame.seq, 3)
        self.receiver.accept(frame(4, 100.06), 100.06)
        self.assertEqual(self.receiver.take(100.3).frame.seq, 4)
        self.receiver.accept(frame(5, 100.31), 100.31)
        self.assertIsNone(self.receiver.take(100.32))
        self.assertEqual(self.receiver.coalesced, 1)

    def test_release_and_repress_survive_pose_coalescing(self):
        """两侧边沿完整保序，即使最终Grip仍为按下也不丢失中间松开。"""
        for value in (frame(1, 100., right=1.), frame(2, 100.01, right=1.),
                      frame(3, 100.02), frame(4, 100.03, right=1.),
                      frame(5, 100.04, left=1., right=1.),
                      frame(6, 100.05, left=1., right=1.)):
            self.receiver.accept(value, value.received_monotonic_s)
        selections = [self.receiver.take(100.06) for _ in range(5)]
        self.assertEqual([value.frame.seq for value in selections], [1, 3, 4, 5, 6])
        self.assertEqual([value.edge for value in selections], [True]*4 + [False])

    def test_new_edge_invalidates_inflight_pose_but_not_edge(self):
        """计算期间松开使普通候选失效，松开及重按的参考事务仍按顺序提交。"""
        self.receiver.accept(frame(1, 100., right=1.), 100.)
        self.receiver.take(100.)
        self.receiver.accept(frame(2, 100.01, right=1.), 100.01)
        self.receiver.take(100.01)
        self.receiver.accept(frame(3, 100.02), 100.02)
        self.receiver.accept(frame(4, 100.03, right=1.), 100.03)
        with patch('aimdk_functions.upper_body_teleop_input.time.monotonic', return_value=100.03):
            self.assertFalse(self.receiver.can_commit())
            self.assertEqual(self.receiver.take(100.03).frame.seq, 3)
            self.assertTrue(self.receiver.can_commit())
            self.assertEqual(self.receiver.take(100.03).frame.seq, 4)

    def test_edge_freezes_execution_at_receive_before_main_selects_it(self):
        """接收Grip松开即通知执行侧冻结，主线程尚未消费边沿也不继续追赶。"""
        callback = Mock()
        self.receiver.on_edge = callback
        self.receiver.accept(frame(1, 100., right=1.), 100.)
        self.receiver.accept(frame(2, 100.01, right=1.), 100.01)
        self.receiver.accept(frame(3, 100.02), 100.02)
        self.assertIsNone(self.receiver.selected)
        self.assertEqual(callback.call_count, 2)

    def test_unselected_invalid_tracking_is_not_hidden(self):
        """即使普通帧不进入IK，接收侧仍检查跟踪有效性。"""
        value = frame(1, 100.)
        value = replace(value, right=replace(value.right, tracking_valid=False))
        with self.assertRaisesRegex(InputUnavailable, '跟踪无效'):
            self.receiver.accept(value, 100.)

    def test_explicit_timeout_checks_recording_age_and_source_freeze(self):
        """执行接管前允许等待；接管后记录耗时及源时间冻结仍按0.5秒中止。"""
        self.receiver.accept(frame(1, 100., source=1000), 100.)
        self.receiver.check(105.)
        self.receiver.activate()
        with self.assertRaisesRegex(InputUnavailable, '积压帧年龄'):
            self.receiver.accept(frame(2, 100.1, source=1001), 100.7)
        self.receiver = LatestInput(self.source, self.recorder, self.receiver.config,
                                    self.log, self.options)
        self.receiver.accept(frame(1, 100., source=1000), 100.)
        self.receiver.activate()
        self.receiver.accept(frame(2, 100.4, source=1000), 100.4)
        with self.assertRaisesRegex(InputUnavailable, '源时间停滞'):
            self.receiver.accept(frame(3, 100.51, source=1000), 100.51)

    def test_receiver_records_while_main_thread_does_not_take_frames(self):
        """主线程不消费时接收器仍处理全部原始事件，普通位姿仅保留最新值。"""
        events = queue.Queue()
        complete = threading.Event()
        began = time.monotonic()
        values = [frame(seq, began+seq*.001) for seq in range(1, 4)]
        for value in values:
            events.put(SimpleNamespace(kind='device_state', device_id='pico', frame=value))

        def read(timeout):
            """模拟SDK阻塞读取；队列暂空时返回None。"""
            try:
                return events.get(timeout=timeout)
            except queue.Empty:
                return None

        def record(event, **fields):
            """第三帧接收日志完成后通知主线程，不依赖sleep。"""
            if event == 'input_timing' and fields['input_seq'] == 3:
                complete.set()

        self.source.read.side_effect = read
        self.recorder.consume.side_effect = [*values]
        self.log.record.side_effect = record
        self.receiver.start()
        try:
            self.assertTrue(complete.wait(2.))
            self.assertEqual(self.receiver.latest.frame.seq, 3)
            self.assertEqual(self.recorder.consume.call_count, 3)
            self.assertEqual(self.receiver.coalesced, 2)
        finally:
            self.receiver.close()


if __name__ == '__main__':
    unittest.main()
