"""原节奏回放、诊断显示与统计契约；由用户在机器人容器执行。"""

import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np

from aimdk_functions.upper_body_teleop_stats import summarize
from aimdk_functions.upper_body_teleop_tools import MappingDisplay, ReplayPacer, rotation_vector_deg


class FakeClock:
    """人工单调时钟，等待仅推进数值，不实际休眠。"""

    def __init__(self):
        """从与原日志时间不同的100秒开始，记录调度器等待。"""
        self.now = 100.
        self.sleeps = []

    def read(self):
        """返回当前人工时间。"""
        return self.now

    def sleep(self, seconds):
        """记录等待并推进人工时间。"""
        self.sleeps.append(seconds)
        self.now += seconds


class PacerTests(unittest.TestCase):
    """检查绝对时间锚点、FIFO及处理落后时的真实等待帧计数。"""

    def setUp(self):
        """构造原日志处于5000秒附近的10ms输入，避免误用原日志时钟。"""
        self.frames = [SimpleNamespace(seq=index+1, received_monotonic_s=5000.+index*.01)
                       for index in range(3)]
        self.clock, self.log = FakeClock(), Mock()

    def test_recorded_waits_and_retains_original_timestamps(self):
        """处理足够快时按原间隔等待，原始帧时间不被改写。"""
        pacer = ReplayPacer(self.frames, self.log, 'recorded', self.clock.read, self.clock.sleep)
        stream = iter(pacer)
        self.assertIs(next(stream), self.frames[0])
        self.assertIs(next(stream), self.frames[1])
        self.assertAlmostEqual(self.clock.now, 100.01)
        self.assertIs(next(stream), self.frames[2])
        self.assertAlmostEqual(self.clock.now, 100.02)
        self.assertAlmostEqual(self.frames[2].received_monotonic_s, 5000.02)
        self.assertTrue(self.clock.sleeps)

    def test_slow_processing_accumulates_fifo_backlog_without_reanchoring(self):
        """首帧耗时25ms时不跳帧，次帧迟到15ms并有1帧已到期等待。"""
        pacer = ReplayPacer(self.frames, self.log, 'recorded', self.clock.read, self.clock.sleep)
        stream = iter(pacer)
        self.assertEqual(next(stream).seq, 1)
        self.clock.now += .025
        second = next(stream)
        fields = self.log.record.call_args.kwargs
        self.assertEqual(fields['backlog_frames'], 1)
        self.assertAlmostEqual(fields['processing_age_s'], .015)
        result = pacer.result_timing(second)
        self.assertAlmostEqual(result['input_to_result_s'], .015)
        self.assertEqual(next(stream).seq, 3)
        self.assertEqual(self.clock.sleeps, [])

    def test_immediate_does_not_invent_input_age(self):
        """尽快回放没有实时输入年龄及积压测量，不从原日志时间计算延迟。"""
        pacer = ReplayPacer(self.frames, self.log, 'immediate', self.clock.read, self.clock.sleep)
        first = next(iter(pacer))
        self.assertIsNone(self.log.record.call_args.kwargs['processing_age_s'])
        self.assertIsNone(self.log.record.call_args.kwargs['backlog_frames'])
        self.assertIsNone(pacer.result_timing(first)['input_to_result_s'])


class DiagnosticsTests(unittest.TestCase):
    """检查180°旋转诊断、关闭显示及不同时间语义的统计隔离。"""

    def test_rotation_vector_at_mixed_axis_half_turn(self):
        """180°混合符号旋转轴能还原矩阵，允许旋转轴整体反号。"""
        axis = np.array([1., -1., 0.]) / np.sqrt(2.)
        rotation = 2. * np.outer(axis, axis) - np.eye(3)
        vector = np.asarray(rotation_vector_deg(rotation))
        self.assertAlmostEqual(float(np.linalg.norm(vector)), 180.)
        np.testing.assert_allclose(np.outer(vector/180., vector/180.), np.outer(axis, axis), atol=1e-8)

    def test_disabled_display_and_failure_output(self):
        """关闭显示不打印；启用后即使没有映射候选也显示失败输入序号。"""
        output = Mock()
        MappingDisplay(0., output=output).show(SimpleNamespace(seq=3), None, False, '失效')
        output.assert_not_called()
        MappingDisplay(1., output=output).show(SimpleNamespace(seq=3), None, False, '失效')
        self.assertIn('seq=3', output.call_args.args[0])
        self.assertIn('失效', output.call_args.args[0])

    def test_stats_separate_live_events_and_replay_frames(self):
        """实时时间与回放时间分别统计，首个失败给出参考和IK保持判定。"""
        rows = [
            dict(event='xr_input', fields=dict(frame=dict(seq=2))),
            dict(event='input_timing', fields=dict(input_seq=2, timing_context='live',
                                                  processing_age_s=.01, backlog_events=3)),
            dict(event='input_timing', fields=dict(input_seq=2, timing_context='replay_recorded',
                                                  processing_age_s=.02, backlog_frames=1)),
            dict(event='frame_result_timing', fields=dict(input_seq=2, timing_context='replay_recorded',
                                                         completed_monotonic_s=100.,
                                                         input_to_result_s=.03, service_seconds=.01)),
            dict(event='ik_rejected', fields=dict(input_seq=2, reason='右臂无解',
                                                 ik_before=dict(last_input_seq=1),
                                                 ik_after=dict(last_input_seq=1),
                                                 mapping_before=dict(sides={}), mapping_after=dict(sides={}))),
            dict(event='exit', fields=dict(reason='failed')),
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'teleop.jsonl'
            path.write_text(''.join(json.dumps(row) + '\n' for row in rows), encoding='utf-8')
            result = summarize(path)
        self.assertEqual(result['timing']['live']['backlog']['unit'], 'events')
        self.assertEqual(result['timing']['replay_recorded']['backlog']['unit'], 'frames')
        self.assertEqual(result['timing']['live']['distributions']['processing_age_s']['max'], .01)
        self.assertEqual(result['first_failure']['previous_input_seq'], 1)
        self.assertTrue(result['first_failure']['ik_state_preserved'])
        self.assertTrue(result['first_failure']['mapping_reference_preserved'])
        self.assertFalse(result['replay_complete'])

    def test_superseded_input_is_resolved_without_failure(self):
        """跨Grip边沿丢弃的候选属于已处理输入，并统计位姿合并数量。"""
        rows = [dict(event='xr_input', fields=dict(frame=dict(seq=2))),
                dict(event='input_selected', fields=dict(input_seq=2, grip_edge=False,
                                                        coalesced_pose_frames=5)),
                dict(event='ik_superseded', fields=dict(input_seq=2)),
                dict(event='exit', fields=dict(reason='completed'))]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'teleop.jsonl'
            path.write_text(''.join(json.dumps(row) + '\n' for row in rows), encoding='utf-8')
            result = summarize(path)
        self.assertEqual(result['superseded_inputs'], [2])
        self.assertEqual(result['input_selection']['coalesced_pose_frames'], 5)
        self.assertEqual(result['unresolved_inputs'], [])
        self.assertIsNone(result['first_failure'])
        self.assertTrue(result['replay_complete'])

    def test_recoverable_rejection_is_resolved_and_recovery_is_counted(self):
        """可恢复空解允许正常结束，统计过渡进度及完成次数；历史拒绝仍按失败处理。"""
        rows = [dict(event='xr_input', fields=dict(frame=dict(seq=1))),
                dict(event='ik_rejected', fields=dict(input_seq=1, reason='left局部IK无解',
                                                     recoverable=True)),
                dict(event='ik_recovery', fields=dict(input_seq=2, phase='recovering', blend=0.)),
                dict(event='ik_recovery', fields=dict(input_seq=3, phase='tracking', blend=1.)),
                dict(event='exit', fields=dict(reason='completed'))]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'teleop.jsonl'
            path.write_text(''.join(json.dumps(row) + '\n' for row in rows), encoding='utf-8')
            result = summarize(path)
        self.assertTrue(result['replay_complete'])
        self.assertEqual(result['ik_recovery']['recoverable_rejections'], 1)
        self.assertEqual(result['ik_recovery']['completed_transitions'], 1)
        self.assertTrue(result['first_failure']['recoverable'])

    def test_stats_separate_new_and_previous_target_tracking_observations(self):
        """新旧目标误差分别统计；首帧和旧日志缺少前目标时不伪造观测样本。"""
        rows = [dict(event='metadata', fields=dict(execute=True)),
                dict(event='command_published', fields=dict(input_seq=None, tracking_error_rad=[.11])),
                dict(event='command_published', fields=dict(
                    input_seq=1, tracking_error_rad=[.12], tracking_exceeds_5deg_observation=True,
                    previous_target_tracking_error_rad=None,
                    previous_target_tracking_exceeds_5deg_observation=None)),
                dict(event='command_published', fields=dict(
                    input_seq=2, tracking_error_rad=[.14], tracking_exceeds_5deg_observation=True,
                    previous_target_tracking_error_rad=[.12],
                    previous_target_tracking_exceeds_5deg_observation=True))]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'teleop.jsonl'
            path.write_text(''.join(json.dumps(row) + '\n' for row in rows), encoding='utf-8')
            result = summarize(path)
        execution = result['execution']
        self.assertEqual(execution['tracking_observation_frames'], 2)
        self.assertEqual(execution['previous_target_tracking_observation_frames'], 1)
        self.assertEqual(execution['tracking_over_5deg_observation_frames'], 2)
        self.assertEqual(execution['previous_target_tracking_over_5deg_observation_frames'], 1)
        self.assertEqual(result['distributions']['command.max_tracking_error_rad']['max'], .14)
        self.assertEqual(result['distributions']['command.max_previous_target_tracking_error_rad']['max'], .12)


if __name__ == '__main__':
    unittest.main()
