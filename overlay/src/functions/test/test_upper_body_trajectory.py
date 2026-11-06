"""轨迹时间语义、具名目标和源命令选择的回归检查；由用户在机器人容器执行。"""

import json
from pathlib import Path
import tempfile
from unittest import TestCase

from aimdk_functions.dual_arm_control_core import ARM_NAMES, ControlFailure
from aimdk_functions.upper_body_cartesian_core import CapturedTarget
from aimdk_functions.upper_body_trajectory_core import (
    RecordedTrajectory, ReplayTrajectory, SCHEMA, TrajectoryPoint, export_trajectory, load_trajectory,
)


class TrajectoryTests(TestCase):
    """检查回放源、时间选择、首帧衔接和无效轨迹拒绝行为。"""

    def test_export_uses_sent_targets_and_rebases_selected_window(self):
        """补偿后的发送目标原样保留，窗口首点归零，源行及失败退出仍可追溯。"""
        with tempfile.TemporaryDirectory() as directory:
            source, output = Path(directory)/'source.jsonl', Path(directory)/'trajectory.jsonl'
            rows = [dict(event='metadata', fields=dict(command='upper_body_teleop_observe'))]
            for index in range(3):
                rows.append(dict(event='command_published', fields=dict(
                    publish_begin_monotonic_s=10.+index*.25,
                    arm_rad=[-.1 if axis in (3, 10) else .01*index for axis in range(14)],
                    ideal_arm_rad=[0.]*14, hand_openings=[index*.1, index*.2], sequence=index)))
            rows.append(dict(event='exit', fields=dict(reason='failed')))
            source.write_text(''.join(json.dumps(row)+'\n' for row in rows), encoding='utf-8')
            result = export_trajectory(source, output, start=.2, end=.5)
            self.assertEqual(result.times, (0., .25))
            self.assertEqual(result.points[0].target.arm_rad,
                             tuple(-.1 if axis in (3, 10) else .01 for axis in range(14)))
            self.assertEqual(result.points[0].source_line, 3)
            self.assertEqual(result.points[-1].target.hand_openings, (.2, .4))
            self.assertEqual(result.metadata['source_exits'][0]['fields']['reason'], 'failed')
            with self.assertRaises(FileExistsError):
                export_trajectory(source, output)

    def test_load_maps_names_and_rejects_time_rollback_and_joint_limit(self):
        """文件槽位可逆序；读回必须按名称恢复，时间回退或超限必须拒绝。"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'trajectory.jsonl'
            header = dict(schema=SCHEMA, type='metadata', robot_model='x2-ultra',
                          urdf_version='v1.3.0', arm_unit='rad', target_field='arm_rad',
                          arm_names=list(reversed(ARM_NAMES)))
            joints = [.001*index for index in range(14)]
            joints[3] = joints[10] = -.1
            point = dict(type='point', time_s=0., arm_rad=list(reversed(joints)),
                         hand_openings=dict(left=.2, right=.4), source_line=2, source_sequence=0)
            rows = [header, point, dict(point, time_s=.2)]
            path.write_text(''.join(json.dumps(row)+'\n' for row in rows), encoding='utf-8')
            self.assertEqual(load_trajectory(path).points[0].target.arm_rad, tuple(joints))
            rows[2]['time_s'] = 0.
            path.write_text(''.join(json.dumps(row)+'\n' for row in rows), encoding='utf-8')
            with self.assertRaises(ControlFailure):
                load_trajectory(path)
            rows[2] = dict(point, time_s=.2, arm_rad=[100.]*14)
            path.write_text(''.join(json.dumps(row)+'\n' for row in rows), encoding='utf-8')
            with self.assertRaises(ControlFailure):
                load_trajectory(path)

    def test_transition_speed_and_late_callback_follow_absolute_time(self):
        """首帧等于实时采样；过渡同时覆盖夹爪；迟到回调直接选择当前时间对应目标。"""
        captured = CapturedTarget([0.]*14, [0., 0.])
        points = tuple(TrajectoryPoint(stamp, CapturedTarget(
            [-value if axis in (3, 10) else value for axis in range(14)], [value, value]), index+2, index)
                       for index, (stamp, value) in enumerate(((0., .1), (.2, .2), (.5, .3))))
        recording = RecordedTrajectory({}, points, tuple(point.time_s for point in points))
        replay = ReplayTrajectory(captured, recording, start_duration=2., speed=.5)
        self.assertEqual(replay.at(0.), captured)
        self.assertAlmostEqual(replay.at(1.).arm_rad[0], .05)
        self.assertAlmostEqual(replay.at(1.).hand_openings[0], .05)
        self.assertEqual(replay.at(2.), points[0].target)
        self.assertEqual(replay.at(2.8), points[1].target)
        self.assertEqual(replay.at(20.), points[-1].target)
        self.assertEqual(replay.duration, 3.)
        self.assertEqual(replay.sample(20.)['phase'], 'endpoint')
