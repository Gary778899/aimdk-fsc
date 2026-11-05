"""完整回调复制、协议解析及断链日志读回的离线验证；不加载SDK或ROS。"""

import base64
import ctypes
from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from remote_control.cli import Recorder
from remote_control.frames import XRFrame
from remote_control.protocol import InputConfig, parse_tracking
from remote_control.recording import XRLog, read_frames, summarize
from remote_control.sdk import DEVICE_STATE_JSON, DeviceStateJson, SDKEvent, SDKSource


def input_config(**overrides):
    """创建测试用显式原始输入约定，可覆盖跟踪状态字段等参数。"""
    values = dict(quaternion_order='xyzw', position_scale_m=1., reference_space_id='test-space',
                  validity_field=None, valid_values=(), reference_space_field=None)
    values.update(overrides)
    return InputConfig(**values)


def tracking_event(seq=1, device_id='selected', received=1., **controller_overrides):
    """生成具有完整双侧控制器的单回调原始事件，左右保持独立数据对象。"""
    left = dict(pose='1,2,3,0,0,0,1', grip=.8, trigger=.4, axisX=.2, axisY=-.2)
    left.update(controller_overrides)
    right = dict(pose='4,5,6,0,0,0,1', grip=.1, trigger=.2)
    value = json.dumps(dict(functionName='Tracking', value=json.dumps(
        dict(Controller=dict(left=left, right=right), timeStampNs=1234))))
    return SDKEvent(seq, received, 'device_state', DEVICE_STATE_JSON, 0, device_id,
                    base64.b64encode(value.encode()).decode())


class SDKRecordingTests(unittest.TestCase):
    """核对跨线程边界复制及故障记录，不调用外部动态库。"""

    def test_head_same_callback_and_roundtrip_preserve_raw_status(self):
        """Head与双手从同一回调解析，位置按配置缩放，status原样写入并读回。"""
        event = tracking_event()
        outer = json.loads(event.payload())
        inner = json.loads(outer['value'])
        inner['Head'] = dict(pose='7,8,9,0,0,0,1', status=3)
        outer['value'] = json.dumps(inner)
        event = replace(event, payload_base64=base64.b64encode(json.dumps(outer).encode()).decode())
        frame = parse_tracking(event, 1, input_config(position_scale_m=.01))
        self.assertEqual(frame.head.position_m, (.07, .08, .09))
        self.assertEqual(frame.head_status, 3)
        self.assertEqual(frame.left.pose.position_m, (.01, .02, .03))
        self.assertEqual(XRFrame.from_dict(frame.to_dict()), frame)
        legacy = frame.to_dict()
        del legacy['head']
        del legacy['head_status']
        self.assertIsNone(XRFrame.from_dict(legacy).head)

    def test_failed_shutdown_retains_callback_owner(self):
        """SDK未确认终止时保持强引用，禁止第二实例覆盖仍可能使用的回调。"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'not_loaded.so'
            path.touch()
            library = Mock()
            library.PXREAInit.return_value = 0
            library.PXREADeinit.return_value = 1
            source = SDKSource(path)
            with patch('remote_control.sdk.ctypes.CDLL', return_value=library):
                source.start()
                try:
                    with self.assertRaisesRegex(RuntimeError, 'Deinit失败'):
                        source.close()
                    self.assertIs(SDKSource._active, source)
                    self.assertFalse(source.is_closed)
                    with self.assertRaisesRegex(RuntimeError, '已有SDKSource'):
                        SDKSource(path).start()
                finally:
                    library.PXREADeinit.return_value = 0
                    source.close()
            self.assertTrue(source.is_closed)
            self.assertIsNone(SDKSource._active)

    def test_callback_copies_buffer_and_preserves_fifo(self):
        """原始C缓冲被改写后已排队内容保持不变，回调序号严格推进。"""
        with tempfile.TemporaryDirectory() as directory:
            library = Path(directory) / 'not_loaded.so'
            library.touch()
            source = SDKSource(library)
            state = DeviceStateJson()
            state.devID = b'selected'
            state.stateJson = b'{"first":1}'
            source._receive(None, DEVICE_STATE_JSON, 17, ctypes.addressof(state))
            state.stateJson = b'{"second":2}'
            source._receive(None, DEVICE_STATE_JSON, 18, ctypes.addressof(state))
            first, second = source.read(0), source.read(0)
            self.assertEqual(first.payload(), b'{"first":1}')
            self.assertEqual(second.payload(), b'{"second":2}')
            self.assertEqual((first.callback_seq, second.callback_seq), (1, 2))
            self.assertEqual((first.status, second.status), (17, 18))

    def test_missing_tracking_remains_unknown_and_axis_retained(self):
        """缺失跟踪字段保持None，同一回调左右位姿及摇杆按原值保留。"""
        frame = parse_tracking(tracking_event(), 1, input_config())
        self.assertIsNone(frame.left.tracking_valid)
        self.assertEqual(frame.left.axis, (.2, -.2))
        self.assertIsNone(frame.right.axis)
        self.assertEqual(frame.left.pose.position_m, (1., 2., 3.))
        self.assertEqual(frame.right.pose.position_m, (4., 5., 6.))

    def test_status_values_are_type_sensitive(self):
        """整数状态1不得因Python相等规则被当作布尔True的有效状态。"""
        frame = parse_tracking(tracking_event(status=1), 1,
                               input_config(validity_field='status', valid_values=(True,)))
        self.assertIs(frame.left.tracking_valid, False)

    def test_partial_axis_and_zero_quaternion_rejected(self):
        """单轴缺失和零四元数必须报错，不能补零或生成身份姿态。"""
        event = tracking_event()
        outer = json.loads(event.payload())
        inner = json.loads(outer['value'])
        del inner['Controller']['left']['axisY']
        outer['value'] = json.dumps(inner)
        broken = SDKEvent(1, 1., 'device_state', DEVICE_STATE_JSON, 0, 'selected',
                          base64.b64encode(json.dumps(outer).encode()).decode())
        with self.assertRaises(ValueError):
            parse_tracking(broken, 1, input_config())
        with self.assertRaises(ValueError):
            parse_tracking(tracking_event(pose='0,0,0,0,0,0,0'), 1, input_config())

    def test_fixed_device_and_disconnect_stop_replay(self):
        """多设备数据不混入选定输入，失联与退出原因保留且回放明确中止。"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'xr.jsonl'
            log = XRLog(path)
            recorder = Recorder(log, 'selected', input_config())
            recorder.consume(tracking_event(device_id='other'), 0)
            recorder.consume(tracking_event(seq=2), 0)
            recorder.consume(SDKEvent(3, 2., 'server_disconnected', 1 << 3, 0, '', ''), 0)
            log.close('completed')
            reader = read_frames(path)
            self.assertEqual(next(reader).device_id, 'selected')
            with self.assertRaises(ValueError):
                next(reader)
            report = summarize(path)
            self.assertEqual(report['input_frame_count'], 1)
            self.assertFalse(report['recording_complete'])
            self.assertEqual(report['session_invalidations'][0]['reason'], 'server_disconnected')
            self.assertEqual(report['exit_reasons'][0]['reason'], 'completed')

    def test_source_regression_across_missing_value_is_reported(self):
        """源时间100、未知、99在读回时拒绝，统计保留缺失并报告回退。"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'xr.jsonl'
            log = XRLog(path)
            original = parse_tracking(tracking_event(), 1, input_config())
            for index, stamp in enumerate((100, None, 99), 1):
                frame = replace(original, seq=index, source_time_ns=stamp,
                                received_monotonic_s=float(index))
                log.record('xr_frame', frame=frame.to_dict())
            log.close('completed')
            reader = read_frames(path)
            self.assertEqual(next(reader).source_time_ns, 100)
            self.assertIsNone(next(reader).source_time_ns)
            with self.assertRaisesRegex(ValueError, r'xr.jsonl:4: XR帧seq=3: XR源时间回退'):
                next(reader)
            report = summarize(path)
            self.assertEqual(report['reversed_source_time'], 1)
            self.assertEqual(report['missing_source_time'], 1)
            self.assertFalse(report['recording_complete'])

    def test_known_source_progression_across_missing_value_is_retained(self):
        """跨缺失值的正常推进与重复分别统计，读回不补造源时间。"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'xr.jsonl'
            log = XRLog(path)
            original = parse_tracking(tracking_event(), 1, input_config())
            for index, stamp in enumerate((None, 100, None, 100, 101), 1):
                frame = replace(original, seq=index, source_time_ns=stamp,
                                received_monotonic_s=float(index))
                log.record('xr_frame', frame=frame.to_dict())
            log.close('completed')
            self.assertEqual([frame.source_time_ns for frame in read_frames(path)],
                             [None, 100, None, 100, 101])
            report = summarize(path)
            self.assertEqual(report['reversed_source_time'], 0)
            self.assertEqual(report['repeated_source_time'], 1)
            self.assertEqual(report['missing_source_time'], 2)
