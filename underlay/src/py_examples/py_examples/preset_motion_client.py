#!/usr/bin/env python3

import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.srv import SetMcPresetMotion, SetMcInputSource
from aimdk_msgs.msg import (McPresetMotion, McControlArea, McInputAction,
                            RequestHeader, CommonState)

INPUT_SOURCE_NAME = "node.preset_motion"
INPUT_SOURCE_PRIORITY = 40
INPUT_SOURCE_TIMEOUT = 1000


class SetMcPresetMotionClient(Node):
    def __init__(self):
        super().__init__('preset_motion_client')
        self.client = self.create_client(
            SetMcPresetMotion, '/aimdk_5Fmsgs/srv/SetMcPresetMotion')
        self.input_source_client = self.create_client(
            SetMcInputSource, '/aimdk_5Fmsgs/srv/SetMcInputSource')
        self.get_logger().info('✅ SetMcPresetMotion client node created.')

        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('⏳ Service unavailable, waiting...')

        self.get_logger().info('🟢 Service available, ready to send request.')

    def _call_input_source(self, action_value: int) -> bool:
        action_name = {
            McInputAction.INPUTACTION_ADD: "ADD",
            McInputAction.INPUTACTION_ENABLE: "ENABLE",
        }.get(action_value, str(action_value))

        req = SetMcInputSource.Request()
        req.action.value = action_value
        req.input_source.name = INPUT_SOURCE_NAME
        req.input_source.priority = INPUT_SOURCE_PRIORITY
        req.input_source.timeout = INPUT_SOURCE_TIMEOUT

        future = None
        for i in range(8):
            req.request.header.stamp = self.get_clock().now().to_msg()
            future = self.input_source_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)
            if future.done():
                break
            # retry as remote peer is NOT handled well by ROS
            self.get_logger().info(
                f'trying input source action {action_name} ({action_value})... [{i}]')

        if future is None or not future.done():
            return False
        response = future.result()
        return response is not None and response.response.header.code == 0

    def register_input_source(self) -> bool:
        if not self.input_source_client.wait_for_service(timeout_sec=8.0):
            self.get_logger().error('❌ SetMcInputSource service unavailable.')
            return False

        # Try ADD first; if already registered, fall back to ENABLE
        if self._call_input_source(McInputAction.INPUTACTION_ADD):  # ADD (1001)
            self.get_logger().info(
                f'✅ Input source "{INPUT_SOURCE_NAME}" registered (ADD).')
            return True
        self.get_logger().info(
            'ADD failed (likely already registered), trying ENABLE...')
        if self._call_input_source(McInputAction.INPUTACTION_ENABLE):  # ENABLE (2001)
            self.get_logger().info(
                f'✅ Input source "{INPUT_SOURCE_NAME}" enabled.')
            return True
        self.get_logger().error('❌ Failed to register input source via ADD or ENABLE.')
        return False

    def send_request(self, area_id: int, motion_id: int) -> bool:
        request = SetMcPresetMotion.Request()
        request.header = RequestHeader()

        request.input_source.name = INPUT_SOURCE_NAME
        request.input_source.priority = INPUT_SOURCE_PRIORITY
        request.input_source.timeout = INPUT_SOURCE_TIMEOUT

        motion = McPresetMotion()
        area = McControlArea()

        motion.value = motion_id
        area.value = area_id

        request.motion = motion
        request.area = area
        request.interrupt = False

        self.get_logger().info(
            f'📨 Sending request to set preset motion: motion={motion_id}, area={area_id}')

        for i in range(8):
            request.header.stamp = self.get_clock().now().to_msg()
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)

            if future.done():
                break

            # retry as remote peer is NOT handled well by ROS
            self.get_logger().info(f'trying ... [{i}]')

        response = future.result()
        if response is None:
            self.get_logger().error('❌ Service call failed or timed out.')
            return False

        if response.response.header.code == 0:
            self.get_logger().info(
                f'✅ Preset motion set successfully: {response.response.task_id}')
            return True
        # RUNNING (400)
        elif response.response.state.value == CommonState.RUNNING:
            self.get_logger().info(
                f'⏳ Preset motion executing: {response.response.task_id}')
            return True
        else:
            self.get_logger().error(
                f'❌ Failed to set preset motion: {response.response.task_id}'
            )
            return False


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        area = int(input("Enter arm area ID (1-left, 2-right): "))
        motion = int(input(
            "Enter preset motion ID (1001-raise, 1002-wave, 1003-handshake, 1004-airkiss): "))

        node = SetMcPresetMotionClient()
        if not node.register_input_source():
            node.get_logger().error("❌ Input source registration failed, exiting")
        else:
            node.send_request(area, motion)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        rclpy.logging.get_logger('main').error(
            f'Program exited with exception: {e}')

    if node:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
