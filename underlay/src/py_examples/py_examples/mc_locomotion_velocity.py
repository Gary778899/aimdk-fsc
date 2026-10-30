#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import signal
import sys

from aimdk_msgs.msg import McLocomotionVelocity, McInputAction, MessageHeader
from aimdk_msgs.srv import SetMcInputSource


class DirectVelocityControl(Node):
    def __init__(self):
        super().__init__('direct_velocity_control')

        self.publisher = self.create_publisher(
            McLocomotionVelocity, '/aima/mc/locomotion/velocity', 10)
        self.client = self.create_client(
            SetMcInputSource, '/aimdk_5Fmsgs/srv/SetMcInputSource')

        self.forward_velocity = 0.0
        self.lateral_velocity = 0.0
        self.angular_velocity = 0.0

        # Maximum speed limits
        self.max_forward_speed = 1.0  # m/s
        self.min_forward_speed = 0.2  # m/s

        self.max_lateral_speed = 1.0  # m/s
        self.min_lateral_speed = 0.2  # m/s

        self.max_angular_speed = 1.0  # rad/s
        self.min_angular_speed = 0.1  # rad/s

        self.timer = None

        self.get_logger().info("Direct velocity control node started!")

    def start_publish(self):
        if not self.timer:
            self.timer = self.create_timer(0.02, self.publish_velocity)

    def _call_input_source(self, action_value):
        action_name = {
            McInputAction.INPUTACTION_ADD: "ADD",
            McInputAction.INPUTACTION_ENABLE: "ENABLE",
        }.get(action_value, str(action_value))

        req = SetMcInputSource.Request()
        req.action.value = action_value
        req.input_source.name = "node.locomotion_velocity"
        req.input_source.priority = 40
        req.input_source.timeout = 1000

        future = None
        for i in range(8):
            req.request.header.stamp = self.get_clock().now().to_msg()
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)
            if future.done():
                break
            # retry as remote peer is NOT handled well by ROS
            self.get_logger().info(
                f"trying input source action {action_name} ({action_value})... [{i}]")

        if future is None or not future.done():
            return False
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call exception: {str(e)}")
            return False
        return response is not None and response.response.header.code == 0

    def register_input_source(self):
        self.get_logger().info("Registering input source...")

        timeout_sec = 8.0
        start = self.get_clock().now().nanoseconds / 1e9

        while not self.client.wait_for_service(timeout_sec=2.0):
            now = self.get_clock().now().nanoseconds / 1e9
            if now - start > timeout_sec:
                self.get_logger().error("Waiting for service timed out")
                return False
            self.get_logger().info("Waiting for input source service...")

        # Try ADD first; if already registered, fall back to ENABLE
        if self._call_input_source(McInputAction.INPUTACTION_ADD):  # ADD (1001)
            self.get_logger().info("✅ Input source registered (ADD).")
            return True
        self.get_logger().info(
            "ADD failed (likely already registered), trying ENABLE...")
        if self._call_input_source(McInputAction.INPUTACTION_ENABLE):  # ENABLE (2001)
            self.get_logger().info("✅ Input source enabled.")
            return True
        self.get_logger().error(
            "❌ Failed to register input source via ADD or ENABLE.")
        return False

    def publish_velocity(self):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "node.locomotion_velocity"
        msg.forward_velocity = self.forward_velocity
        msg.lateral_velocity = self.lateral_velocity
        msg.angular_velocity = self.angular_velocity

        self.publisher.publish(msg)

        self.get_logger().info(
            f"Publishing velocity: forward {self.forward_velocity:.2f} m/s, "
            f"lateral {self.lateral_velocity:.2f} m/s, "
            f"angular {self.angular_velocity:.2f} rad/s"
        )

    def set_forward(self, forward):
        # check value range, mc has thresholds to start movement
        if abs(forward) < 0.005:
            self.forward_velocity = 0.0
            return True
        elif abs(forward) > self.max_forward_speed or abs(forward) < self.min_forward_speed:
            raise ValueError("out of range")
        else:
            self.forward_velocity = forward
            return True

    def set_lateral(self, lateral):
        # check value range, mc has thresholds to start movement
        if abs(lateral) < 0.005:
            self.lateral_velocity = 0.0
            return True
        elif abs(lateral) > self.max_lateral_speed or abs(lateral) < self.min_lateral_speed:
            raise ValueError("out of range")
        else:
            self.lateral_velocity = lateral
            return True

    def set_angular(self, angular):
        # check value range, mc has thresholds to start movement
        if abs(angular) < 0.005:
            self.angular_velocity = 0.0
            return True
        elif abs(angular) > self.max_angular_speed or abs(angular) < self.min_angular_speed:
            raise ValueError("out of range")
        else:
            self.angular_velocity = angular
            return True

    def clear_velocity(self):
        self.forward_velocity = 0.0
        self.lateral_velocity = 0.0
        self.angular_velocity = 0.0


# Global node instance for signal handling
global_node = None


def signal_handler(sig, frame):
    global global_node
    if global_node is not None:
        global_node.clear_velocity()
        global_node.get_logger().info(
            f"Received signal {sig}, clearing velocity and shutting down")
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0)


def main():
    global global_node
    rclpy.init()

    node = DirectVelocityControl()
    global_node = node

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    if not node.register_input_source():
        node.get_logger().error("❌ Input source registration failed, exiting")
        if rclpy.ok():
            rclpy.shutdown()
        return

    # get and check control values
    # notice that mc has thresholds to start movement
    try:
        # get input forward
        forward = float(
            input("Please enter forward velocity 0 or ±(0.2 ~ 1.0) m/s: "))
        node.set_forward(forward)
        # get input lateral
        lateral = float(
            input("Please enter lateral velocity 0 or ±(0.2 ~ 1.0) m/s: "))
        node.set_lateral(lateral)
        # get input angular
        angular = float(
            input("Please enter angular velocity 0 or ±(0.1 ~ 1.0) rad/s: "))
        node.set_angular(angular)
    except Exception as e:
        node.get_logger().error(f"Invalid input: {e}")
        if rclpy.ok():
            rclpy.shutdown()
        return

    node.get_logger().info("Setting velocity, moving for 5 seconds")
    node.start_publish()

    start = node.get_clock().now()
    while (node.get_clock().now() - start).nanoseconds / 1e9 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.001)

    node.clear_velocity()
    node.get_logger().info("5-second motion finished, robot stopped")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
