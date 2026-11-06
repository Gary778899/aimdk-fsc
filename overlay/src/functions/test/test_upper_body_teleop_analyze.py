"""离线误差分解、样本去重与命令历史延迟拟合回归；由用户在容器执行。"""

from unittest import TestCase
from unittest.mock import patch

import numpy as np

from aimdk_functions.upper_body_teleop_analyze import (
    fit_delay, make_records, statistics, summarize_records,
)


class LinearModel:
    """以前三关节作为位置的FK替身，用于检验误差分解的符号与加和。"""

    def forward_kinematics(self, joints):
        """返回前三个关节组成的位置及单位旋转，不加载机器人模型。"""
        return np.asarray(joints[:3]), np.eye(3)


class AnalysisTests(TestCase):
    """检查分析的数学关系与时间语义，避免用当前命令代替历史响应。"""

    def test_decomposition_and_feedback_reuse(self):
        """补偿项与跟踪项可以互相抵消，同一样本重复发布只算一个独立反馈。"""
        fields = dict(sequence=1, input_seq=2, ideal_arm_rad=[.1]*14,
                      arm_rad=[.12]*14, actual_arm_rad=[.11]*14,
                      arm_feedback_serial=4, arm_feedback_stamp_s=30.,
                      arm_feedback_received_monotonic_s=10.,
                      publish_begin_monotonic_s=10.01, publish_end_monotonic_s=10.011)
        arm = dict(enabled=False, requested_position_m=[.1]*3,
                   requested_rotation_matrix=np.eye(3).tolist())
        data = dict(config={}, commands=[dict(fields=fields, monotonic_s=10.02, line=1)]*2,
                    accepted={2: dict(right=arm)}, mappings={})
        with patch('aimdk_functions.upper_body_teleop_analyze.make_model', return_value=LinearModel()):
            rows = make_records(data, 'right')
        np.testing.assert_allclose(rows[0]['compensation_mm'], [20.]*3)
        np.testing.assert_allclose(rows[0]['tracking_mm'], [-10.]*3)
        np.testing.assert_allclose(rows[0]['task_mm'], [10.]*3)
        self.assertEqual(rows[0]['phase'], 'holding')
        summary = summarize_records(rows)
        self.assertEqual(summary['unique_feedback_samples'], 1)
        self.assertEqual(summary['reused_feedback_samples'], 1)

    def test_delay_recovers_known_history_and_rejects_legacy_time(self):
        """非周期命令延后50ms得到的反馈可恢复该延迟；无发送时刻时不伪造拟合。"""
        times = np.arange(0., 3., .01)
        commands = []
        records = []
        for index, time in enumerate(times):
            target = [float(np.sin(time*3)+time*.2)]*14
            commands.append(dict(fields=dict(publish_begin_monotonic_s=float(time), arm_rad=target),
                                 monotonic_s=float(time)+.001))
            if index >= 10:
                records.append(dict(feedback_serial=index, feedback_received_monotonic_s=float(time)+.055,
                                    feedback_rad=[value+.02 for value in target[7:]]))
        result = fit_delay(dict(commands=commands), records, 'right', .1, .01, .08)
        self.assertEqual(result['status'], 'estimated')
        self.assertAlmostEqual(result['delay_s'], .05)
        self.assertLess(result['validation_demeaned_rmse_deg'], 1e-10)
        self.assertLess(result['validation_demeaned_rmse_deg'],
                        result['zero_delay_validation_demeaned_rmse_deg'])
        del commands[0]['fields']['publish_begin_monotonic_s']
        result = fit_delay(dict(commands=commands), records, 'right', .1, .01, .08)
        self.assertEqual(result['status'], 'unavailable')

    def test_approach_lag_is_separate_from_ik_residual_including_canceled_goal(self):
        """当前保持目标可已撤销终点，IK残差仍取原候选，四项有符号分解闭合。"""
        fields = dict(sequence=1, input_seq=2, ideal_arm_rad=[.04]*14,
                      ik_goal_arm_rad=[.04]*14, arm_rad=[.06]*14,
                      actual_arm_rad=[.05]*14, joint_approach=dict(paused=True))
        arm = dict(enabled=True, candidate_rad=[.1]*7, requested_position_m=[.1]*3,
                   requested_rotation_matrix=np.eye(3).tolist())
        data = dict(config={}, commands=[dict(fields=fields, monotonic_s=10., line=1)],
                    accepted={2: dict(right=arm)}, mappings={})
        with patch('aimdk_functions.upper_body_teleop_analyze.make_model', return_value=LinearModel()):
            row = make_records(data, 'right')[0]
        np.testing.assert_allclose(row['ik_mm'], [0.]*3)
        np.testing.assert_allclose(row['approach_mm'], [-60.]*3)
        np.testing.assert_allclose(np.sum([row[key] for key in
                                          ('ik_mm', 'approach_mm', 'compensation_mm', 'tracking_mm')], axis=0),
                                   row['task_mm'])
        self.assertTrue(row['joint_approach_paused'])

    def test_signed_statistics_preserve_mean_and_absolute_percentile(self):
        """统计保留误差符号，P95使用绝对值而非有符号排序。"""
        result = statistics([-2., 1.])
        self.assertEqual(result['mean'], -.5)
        self.assertEqual(result['maximum_abs'], 2.)
        self.assertIsNone(statistics([]))
