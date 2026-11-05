"""只读身体状态记录与补偿必需反馈边界回归；由用户在机器人容器执行。"""

from types import SimpleNamespace
from unittest import TestCase
from unittest.mock import Mock, patch

from aimdk_functions.upper_body_ros import UpperBodyFeedbackPort


class BodyRecordingTests(TestCase):
    """验证额外记录不会参与就绪和错误锁存，补偿启用时仍走原有校验。"""

    def make_port(self, compensation):
        """注入ROS消息和QoS替身，返回实际初始化的反馈适配器。"""
        messages = SimpleNamespace(JointStateArray=object, HandStateArray=object,
                                   McCommonState=object)
        qos = SimpleNamespace(QoSProfile=Mock(), ReliabilityPolicy=Mock(), DurabilityPolicy=Mock())
        with patch.dict('sys.modules', {'aimdk_msgs.msg': messages, 'rclpy.qos': qos,
                                       'sensor_msgs.msg': SimpleNamespace(Imu=object)}):
            return UpperBodyFeedbackPort(Mock(), Mock(), compensation=compensation, record_body=True)

    def test_record_only_never_joins_required_streams(self):
        """关闭补偿时即使原始身体消息不可解码也只记录，不加入反馈就绪门槛。"""
        port = self.make_port(False)
        self.assertNotIn('waist', port.streams)
        self.assertNotIn('pelvis_imu', port.streams)
        convert = SimpleNamespace(message_to_ordereddict=Mock(return_value={'invalid': True}))
        port.node.get_clock.return_value.now.return_value.nanoseconds = 1000000000
        with patch.dict('sys.modules', {'rosidl_runtime_py.convert': convert}):
            port.receive('waist', object())
        fields = port.diagnostics.record.call_args.kwargs
        self.assertTrue(fields['observation_only'])
        self.assertEqual(fields['serial'], 1)
        self.assertEqual(fields['raw'], {'invalid': True})

    def test_compensation_keeps_body_streams_required(self):
        """开启补偿时IMU与腰仍走原有必需反馈流，不被只读选项降级。"""
        port = self.make_port(True)
        self.assertIn('waist', port.streams)
        self.assertIn('pelvis_imu', port.streams)
        self.assertEqual(port.observation_serials, {})
