import rclpy
from rclpy.node import Node
from aimdk_msgs.msg import HandCommandArray, HandCommand, HandType


class HandControl(Node):
    def __init__(self):
        super().__init__('hand_control')

        # Create publisher
        self.publisher_ = self.create_publisher(
            HandCommandArray,
            '/aima/hal/joint/hand/command',
            10
        )

        self.timer_ = self.create_timer(
            1.0,
            self.publish_hand_commands
        )

        # Initialize variables
        self.target_finger = 0
        self.step = 1
        self.increasing = True
        self.get_logger().info("Hand control node started!")

    def build_hand_cmd(self, name: str) -> HandCommand:
        cmd = HandCommand()
        cmd.name = name
        cmd.position = 0.0
        cmd.velocity = 0.1
        cmd.acceleration = 0.0
        cmd.deceleration = 0.0
        cmd.effort = 0.0
        return cmd

    def publish_hand_commands(self):
        msg = HandCommandArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'hand_command'
        # dexterous hands (1)
        msg.left_hand_type.value = HandType.NIMBLE_HANDS
        # dexterous hands (1)
        msg.right_hand_type.value = HandType.NIMBLE_HANDS

        left_names = ['L_thumb_roll_joint', 'L_thumb_abad_joint', 'L_thumb_mcp_joint',
                      'L_index_abad_joint', 'L_index_pip_joint', 'L_middle_pip_joint',
                      'L_ring_abad_joint', 'L_ring_pip_joint', 'L_pinky_abad_joint', 'L_pinky_pip_joint']
        right_names = [n.replace('L_', 'R_', 1) for n in left_names]

        # left hand
        msg.left_hands = [self.build_hand_cmd(
            left_names[i]) for i in range(10)]

        # right hand
        msg.right_hands = [self.build_hand_cmd(
            right_names[i]) for i in range(10)]

        # Absolute joint targets in rad = half-grip
        # Joint order: [thumb_roll, thumb_abad, thumb_mcp, index_abad, index_pip,
        #               middle_pip, ring_abad, ring_pip, pinky_abad, pinky_pip]
        left_hand_target = [-0.6, 0.8, -0.4,
                            0.1, 0.7, 0.7, -0.1, 0.7, -0.1, 0.7]
        right_hand_target = [0.6, -0.8, 0.4,
                             -0.1, 0.7, 0.7, 0.1, 0.7, 0.1, 0.7]

        if self.target_finger < 10:
            msg.right_hands[self.target_finger].position = right_hand_target[self.target_finger]
        else:
            target_finger_ = self.target_finger - 10
            msg.left_hands[target_finger_].position = left_hand_target[target_finger_]

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published hand command with target_finger: {self.target_finger}')
        self.update_target_finger()

    def update_target_finger(self):
        if self.increasing:
            self.target_finger += self.step
            if self.target_finger >= 19:
                self.target_finger = 19
                self.increasing = False
        else:
            self.target_finger -= self.step
            if self.target_finger <= 0:
                self.target_finger = 0
                self.increasing = True


def main(args=None):
    rclpy.init(args=args)
    hand_control_node = HandControl()

    try:
        rclpy.spin(hand_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        hand_control_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
