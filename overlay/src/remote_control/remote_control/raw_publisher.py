"""将原始XR回调封装为ROS消息，供独立采集和遥操共用。"""

import json


class XRRawPublisher:
    """发布原始SDK事件，分别统计发布尝试和成功返回的调用。"""

    def __init__(self, node, session_id=None):
        """在node上创建原始XR话题，session_id为可选的试次编号；创建失败向调用方传播。"""
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = node.create_publisher(String, '/remote_control/xr/raw', qos)
        self.message_type, self.session_id = String, session_id
        self.attempted = self.publish_calls_returned = 0
        self.first_error = None

    def __call__(self, event, drained):
        """发布单条SDKEvent；drained标记退出排空，序列化或发布异常记录为首次错误。"""
        self.attempted += 1
        try:
            message = self.message_type(data=json.dumps(dict(
                schema='remote_control.xr_raw.v1', session_id=self.session_id,
                drained=drained, **event.to_dict()), ensure_ascii=False, allow_nan=False))
            self.publisher.publish(message)
        except Exception as error:
            if self.first_error is None:
                self.first_error = dict(sdk_connection_id=event.sdk_connection_id,
                                        callback_seq=event.callback_seq,
                                        message=f'{type(error).__name__}: {error}')
        else:
            self.publish_calls_returned += 1

    def record_summary(self, log):
        """将发布调用计数和首次错误写入XR日志；写入失败向调用方传播。"""
        log.record('xr_raw_publish_summary', attempted=self.attempted,
                   publish_calls_returned=self.publish_calls_returned, first_error=self.first_error)
