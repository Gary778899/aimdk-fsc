#!/usr/bin/env python3

import os
import sys
import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.srv import PlayAudioFile
from aimdk_msgs.msg import CommonState


class PlayAudioClient(Node):
    def __init__(self):
        super().__init__('play_audio_client')
        self.client = self.create_client(
            PlayAudioFile, '/aimdk_5Fmsgs/srv/PlayAudioFile')
        self.get_logger().info('✅ PlayAudio client node created.')

        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('⏳ Service unavailable, waiting...')

        self.get_logger().info('🟢 Service available, ready to send request.')

    def send_request(self, audio_path):
        req = PlayAudioFile.Request()

        req.file.pkg_name = 'demo_client'
        req.file.file_name = os.path.basename(audio_path)
        req.file.file_path = os.path.dirname(audio_path) + '/'
        req.file.priority = 8
        req.file.priority_weight = 0

        self.get_logger().info(
            f'📨 Sending request to play audio: {audio_path}')
        for i in range(5):
            req.request.header.stamp = self.get_clock().now().to_msg()
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

            if future.done():
                break

            if not rclpy.ok():
                self.get_logger().warn('Interrupted while waiting')
                return False

            # retry as remote peer is NOT handled well by ROS
            self.get_logger().info(f'trying ... [{i}]')

        resp = future.result()
        if resp is None:
            self.get_logger().error('❌ Service call not completed or timed out.')
            return False

        if resp.reponse.status.value == CommonState.SUCCESS:  # SUCCESS (1)
            self.get_logger().info('✅ Request to play audio file recorded successfully.')
            return True
        else:
            # FAILURE(2): focus-apply failed; INVALID(5): invalid params;
            # other: see CommonState for details.
            reasons = {CommonState.FAILURE: 'focus apply failed',  # FAILURE (2)
                       CommonState.INVALID: 'invalid params'}      # INVALID (5)
            reason = reasons.get(resp.reponse.status.value, 'unknown')
            self.get_logger().error(
                f'❌ Failed to record play-audio request '
                f'(status={resp.reponse.status.value}, {reason}): '
                f'{resp.reponse.message}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = None

    default_audio = '/agibot/data/var/hal_audio/file/bingo.wav'
    try:
        if len(sys.argv) > 1:
            audio_path = sys.argv[1]
        else:
            audio_path = input(
                f'Enter audio file path to play (default: {default_audio}): ').strip()
            if not audio_path:
                audio_path = default_audio

        node = PlayAudioClient()
        node.send_request(audio_path)
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
