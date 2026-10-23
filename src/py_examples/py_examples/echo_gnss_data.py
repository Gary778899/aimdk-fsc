#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix, NavSatStatus


STATUS_STR = {
    NavSatStatus.STATUS_NO_FIX: 'NO_FIX',
    NavSatStatus.STATUS_FIX: 'FIX',
}


class GnssPrinter(Node):
    NAVSAT_TOPIC = '/aima/hal/sensor/gnss'

    def __init__(self):
        super().__init__('gnss_printer')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )

        self._last_recv = None
        self.sub = self.create_subscription(
            NavSatFix, self.NAVSAT_TOPIC, self.cb, qos
        )
        self.get_logger().info(f'Subscribing NavSatFix: {self.NAVSAT_TOPIC}')

    def cb(self, msg: NavSatFix):
        now = time.perf_counter()
        dt_ms = 0.0 if self._last_recv is None else (
            now - self._last_recv) * 1000.0
        self._last_recv = now

        t_sec = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        status_str = STATUS_STR.get(msg.status.status, 'UNKNOWN')
        cov = msg.position_covariance

        self.get_logger().info(
            "\n========== NavSatFix ==========\n"
            f"stamp={t_sec:.6f}s\n"
            f"frame={msg.header.frame_id}\n"
            f"recv_dt={dt_ms:.3f}ms\n"
            f"latitude:  {msg.latitude:.8f} deg\n"
            f"longitude: {msg.longitude:.8f} deg\n"
            f"altitude:  {msg.altitude:.3f} m\n"
            f"status: {status_str}\n"
            f"service: {msg.status.service}\n"
            f"position_covariance: [{cov[0]:.4f}, {cov[1]:.4f}, {cov[2]:.4f}, "
            f"{cov[3]:.4f}, {cov[4]:.4f}, {cov[5]:.4f}, "
            f"{cov[6]:.4f}, {cov[7]:.4f}, {cov[8]:.4f}]\n"
            f"position_covariance_type: {msg.position_covariance_type}"
        )


def main():
    rclpy.init()
    node = GnssPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
