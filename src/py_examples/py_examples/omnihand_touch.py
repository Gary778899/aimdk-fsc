#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from aimdk_msgs.msg import HandStateArray, HandCommandArray, HandType


class HandStateSubscriber(Node):
    def __init__(self):
        super().__init__('hand_state_subscriber')

        # Create publisher
        self.publisher_ = self.create_publisher(
            HandCommandArray,
            '/aima/hal/joint/hand/command',
            10
        )

        # Create QoS profile with BEST_EFFORT reliability
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        # Create subscriber
        self.subscription = self.create_subscription(
            HandStateArray,
            '/aima/hal/joint/hand/state',
            self.topic_callback,
            qos_profile
        )

        # Create a timer to publish once per second
        self.timer = self.create_timer(1.0, self.publish_command)

        self.get_logger().info(
            "Subscriber started, listening to /aima/hal/joint/hand/state topic..."
        )

    def topic_callback(self, msg):
        """Callback function for handling incoming messages"""
        # Print message header information
        self.get_logger().info(
            f"Message received - Sequence: {msg.header.sequence}, "
            f"Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        )

        # Print left hand touch sensor data
        self.print_touch_sensor_data("Left Hand", msg.left_touch_sensors)

        # Print right hand touch sensor data
        self.print_touch_sensor_data("Right Hand", msg.right_touch_sensors)

        print()

    def print_touch_sensor_data(self, hand_name, sensor_data):
        """
        Print touch sensor data for a specific hand

        Args:
            hand_name: Name of the hand (Left Hand/Right Hand)
            sensor_data: Touch sensor data structure
        """
        print(f"=== {hand_name} Touch Sensor Data ===")

        # Print palm touch data
        print("Palm Touch Data (36 elements): ", end="")
        self.print_array(sensor_data.palm_touch_data)

        # Print back of hand touch data
        print("Back of Hand Touch Data (36 elements): ", end="")
        self.print_array(sensor_data.back_of_hand_touch_data)

        # Print finger touch data
        print("Thumb Touch Data (16 elements): ", end="")
        self.print_array(sensor_data.thumb_touch_data)

        print("Index Finger Touch Data (16 elements): ", end="")
        self.print_array(sensor_data.index_finger_touch_data)

        print("Middle Finger Touch Data (16 elements): ", end="")
        self.print_array(sensor_data.middle_finger_touch_data)

        print("Ring Finger Touch Data (16 elements): ", end="")
        self.print_array(sensor_data.ring_finger_touch_data)

        print("Little Finger Touch Data (16 elements): ", end="")
        self.print_array(sensor_data.little_finger_touch_data)

    def print_array(self, arr):
        """
        Print array of uint8_t elements

        Args:
            arr: Array to print (list or tuple)
        """
        print("[", end="")
        for i, val in enumerate(arr):
            print(f"{val:3d}", end="")
            if i < len(arr) - 1:
                print(" ", end="")
        print("]")

    def publish_command(self):
        """Publish hand command message"""
        message = HandCommandArray()

        # Set header
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "hand_command"

        # Set the hand type
        # dexterous hands (1)
        message.left_hand_type.value = HandType.NIMBLE_HANDS
        # dexterous hands (1)
        message.right_hand_type.value = HandType.NIMBLE_HANDS

        # Create hand command arrays
        from aimdk_msgs.msg import HandCommand

        # Joint names for readability, mirroring the /aima/hal/joint/hand/state feedback (URDF names).
        # The driver routes by array index/order (see endeffector.md), not by these name strings.
        left_names = ["L_thumb_roll_joint", "L_thumb_abad_joint", "L_thumb_mcp_joint",
                      "L_index_abad_joint", "L_index_pip_joint", "L_middle_pip_joint",
                      "L_ring_abad_joint", "L_ring_pip_joint", "L_pinky_abad_joint", "L_pinky_pip_joint"]
        right_names = [n.replace("L_", "R_", 1) for n in left_names]

        def build_cmd(name):
            cmd = HandCommand()
            cmd.name = name
            cmd.position = 0.0
            cmd.velocity = 0.1
            cmd.acceleration = 0.0
            cmd.deceleration = 0.0
            cmd.effort = 0.0
            return cmd

        message.left_hands = [build_cmd(n) for n in left_names]
        message.right_hands = [build_cmd(n) for n in right_names]

        # Publish the message
        self.publisher_.publish(message)

        self.get_logger().info("Published hand command")


def main(args=None):
    rclpy.init(args=args)
    node = HandStateSubscriber()
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
