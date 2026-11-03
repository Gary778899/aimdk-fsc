"""常驻监听的等待、失联、恢复及日志生命周期验证，供机器人容器执行。"""

from argparse import Namespace
import base64
from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from remote_control.cli import run_recording
from remote_control.protocol import InputConfig, parse_tracking
from remote_control.recording import read_records, summarize
from remote_control.sdk import DEVICE_STATE_JSON, SDKEvent
from remote_control.watch import WatchStatus


def tracking_event(seq=1, received=1.0):
    """构造同一回调内的完整左右手柄输入，供显示及重连日志验证。"""
    controller = dict(pose='0,0,0,0,0,0,1', grip=0.5, trigger=0.2)
    payload = json.dumps(dict(functionName='Tracking', value=json.dumps(
        dict(Controller=dict(left=controller, right=controller), timeStampNs=seq))))
    return SDKEvent(seq, received, 'device_state', DEVICE_STATE_JSON, 0, 'TestDevice',
                    base64.b64encode(payload.encode()).decode())


class WatchTests(unittest.TestCase):
    """验证显示不会把等待当失败或把历史帧当新输入，原始日志保留断链。"""

    def setUp(self):
        """创建可控时钟、输出接收器与完整帧，避免依赖SDK及实际等待。"""
        self.clock = Mock(return_value=1.0)
        self.output = Mock()
        self.status = WatchStatus('TestDevice', output=self.output, clock=self.clock)
        self.config = InputConfig('xyzw', 1.0, 'test-space', None, (), None)
        self.event = tracking_event()
        self.frame = parse_tracking(self.event, 1, self.config)

    def test_idle_waits_without_replaying_latest(self):
        """尚无头显时持续报告零帧；来帧后下一空窗口不重复打印手柄快照。"""
        self.clock.return_value = 2.0
        self.status.report(0)
        self.assertIn('没有新Controller帧', self.output.call_args.args[0])
        self.status.consume(self.event, self.frame)
        self.clock.return_value = 3.0
        self.status.report(0)
        self.assertTrue(any(call.args[0].startswith('left:') for call in self.output.call_args_list))
        self.output.reset_mock()
        self.clock.return_value = 4.0
        self.status.report(0)
        self.assertFalse(any(call.args[0].startswith('left:') for call in self.output.call_args_list))

    def test_disconnect_clears_snapshot_and_accepts_new_frames(self):
        """设备离线后清空显示缓存，后续实际输入可在同一进程恢复。"""
        self.status.consume(self.event, self.frame)
        missing = SDKEvent(2, 2.0, 'device_missing', 1 << 5, 0, 'TestDevice', '')
        self.status.consume(missing, None)
        self.assertIsNone(self.status.latest)
        self.assertFalse(self.status.device_seen)
        self.assertEqual(self.status.window_times, [])
        event = tracking_event(3, 3.0)
        self.status.consume(event, parse_tracking(event, 2, self.config))
        self.assertTrue(self.status.device_seen)
        self.assertEqual(self.status.total_frames, 2)

    def test_other_device_does_not_invalidate_selected_snapshot(self):
        """另一设备的离线事件仅显示，不清除固定设备当前状态。"""
        self.status.consume(self.event, self.frame)
        missing = SDKEvent(2, 2.0, 'device_missing', 1 << 5, 0, 'other', '')
        self.status.consume(missing, None)
        self.assertIs(self.status.latest, self.frame)
        self.status.consume(replace(missing, kind='server_disconnected', device_id=''), None)
        self.assertIsNone(self.status.latest)

    def test_shared_recording_keeps_disconnect_and_drains_on_interrupt(self):
        """用模拟SDK验证断开重连日志、Ctrl-C收尾及退出排空，禁止跨失联认定完整。"""
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            library = directory / 'fake.so'
            library.write_bytes(b'not loaded')
            config_path = directory / 'input.json'
            config_path.write_text(json.dumps(dict(
                quaternion_order='xyzw', position_scale_m=1.0, reference_space_id='test-space',
                validity_field=None, valid_values=[], reference_space_field=None)))
            source = Mock(library_path=library, backlog=0, is_closed=True, error=None)
            missing = SDKEvent(2, 2.0, 'device_missing', 1 << 5, 0, 'TestDevice', '')
            source.read.side_effect = [self.event, missing, tracking_event(3, 3.0),
                                       KeyboardInterrupt(), tracking_event(4, 4.0), None]
            options = Namespace(input_config=str(config_path), device_id='TestDevice',
                                sdk_library=str(library), output=str(directory / 'watch.jsonl'),
                                duration=None)
            with patch('remote_control.cli.SDKSource', return_value=source):
                self.assertEqual(run_recording(options, observer=self.status), 130)
            source.close.assert_called_once()
            records = list(read_records(options.output))
            self.assertEqual(sum(row['event'] == 'xr_frame' for row in records), 2)
            self.assertEqual(sum(row['event'] == 'session_invalidated' for row in records), 1)
            drained = [row for row in records if row['event'] == 'sdk_event'
                       and row['fields']['drained']]
            self.assertEqual(len(drained), 1)
            self.assertFalse(summarize(options.output)['recording_complete'])
