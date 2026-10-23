#include <aimdk_msgs/msg/hand_command_array.hpp>
#include <aimdk_msgs/msg/hand_type.hpp>
#include <array>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class HandCommandPublisher : public rclcpp::Node {
public:
  HandCommandPublisher() : Node("hand_command_publisher") {
    publisher_ = this->create_publisher<aimdk_msgs::msg::HandCommandArray>(
        "/aima/hal/joint/hand/command", 10);

    // Create a timer to publish once per second
    timer_ = this->create_wall_timer(
        1s, std::bind(&HandCommandPublisher::publish_command, this));

    RCLCPP_INFO(this->get_logger(), "Hand control node started!");
  }

private:
  void publish_command() {
    auto message = aimdk_msgs::msg::HandCommandArray();

    // Set header
    message.header.stamp = this->now();
    message.header.frame_id = "hand_command";

    // Set the hand type
    message.left_hand_type.value =
        aimdk_msgs::msg::HandType::NIMBLE_HANDS; // dexterous hands (1)
    message.right_hand_type.value =
        aimdk_msgs::msg::HandType::NIMBLE_HANDS; // dexterous hands (1)

    const char *left_names[10] = {"L_thumb_roll_joint", "L_thumb_abad_joint",
                                  "L_thumb_mcp_joint",  "L_index_abad_joint",
                                  "L_index_pip_joint",  "L_middle_pip_joint",
                                  "L_ring_abad_joint",  "L_ring_pip_joint",
                                  "L_pinky_abad_joint", "L_pinky_pip_joint"};
    const char *right_names[10] = {"R_thumb_roll_joint", "R_thumb_abad_joint",
                                   "R_thumb_mcp_joint",  "R_index_abad_joint",
                                   "R_index_pip_joint",  "R_middle_pip_joint",
                                   "R_ring_abad_joint",  "R_ring_pip_joint",
                                   "R_pinky_abad_joint", "R_pinky_pip_joint"};

    // Create left hand command array
    message.left_hands.resize(10);
    for (int i = 0; i < 10; i++) {
      message.left_hands[i].name = left_names[i];
      message.left_hands[i].position = 0.0;
      message.left_hands[i].velocity = 0.1;
      message.left_hands[i].acceleration = 0.0;
      message.left_hands[i].deceleration = 0.0;
      message.left_hands[i].effort = 0.0;
    }

    // Create right hand command array
    message.right_hands.resize(10);
    for (int i = 0; i < 10; i++) {
      message.right_hands[i].name = right_names[i];
      message.right_hands[i].position = 0.0;
      message.right_hands[i].velocity = 0.1;
      message.right_hands[i].acceleration = 0.0;
      message.right_hands[i].deceleration = 0.0;
      message.right_hands[i].effort = 0.0;
    }

    // Absolute joint targets in rad = half-grip
    // Joint order: [thumb_roll, thumb_abad, thumb_mcp, index_abad, index_pip,
    //               middle_pip, ring_abad, ring_pip, pinky_abad, pinky_pip]
    const std::array<double, 10> left_hand_target = {
        -0.6, 0.8, -0.4, 0.1, 0.7, 0.7, -0.1, 0.7, -0.1, 0.7};
    const std::array<double, 10> right_hand_target = {
        0.6, -0.8, 0.4, -0.1, 0.7, 0.7, 0.1, 0.7, 0.1, 0.7};

    if (target_finger < 10) {
      message.right_hands[target_finger].position =
          right_hand_target[target_finger];
    } else {
      int target_finger_ = target_finger - 10;
      message.left_hands[target_finger_].position =
          left_hand_target[target_finger_];
    }

    // Publish the message
    publisher_->publish(message);

    RCLCPP_INFO(this->get_logger(),
                "Published hand command with target_finger: %d", target_finger);

    update_target_finger();
  }

  void update_target_finger() {
    if (increasing_) {
      target_finger += step_;
      if (target_finger >= 19) {
        target_finger = 19;
        increasing_ = false;
      }
    } else {
      target_finger -= step_;
      if (target_finger <= 0) {
        target_finger = 0;
        increasing_ = true;
      }
    }
  }

  rclcpp::Publisher<aimdk_msgs::msg::HandCommandArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  int target_finger = 0;
  int step_ = 1;
  bool increasing_ = true;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HandCommandPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
