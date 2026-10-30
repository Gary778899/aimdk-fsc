// Upper body control example.
//
// Usage:
//   ros2 run examples upper_body_control <mode>
//
//   mode:
//     head    — head center pose,          requires HEAD_ONLY mode
//     claw    — claw half-open,            requires UPPERBODY_REMOTE_SPLIT mode
//     joint   — dexterous hand half-open,  requires UPPERBODY_REMOTE_SPLIT mode
//     gesture — dexterous hand gesture 1,  requires UPPERBODY_REMOTE_SPLIT mode
//
// Switch to the desired MC mode before running, e.g. for head:
//   ros2 run examples set_mc_action SD     # → STAND_DEFAULT
//   ros2 run examples set_mc_action HO     # → HEAD_ONLY
// or for claw / joint / gesture:
//   ros2 run examples set_mc_action URS    # → UPPERBODY_REMOTE_SPLIT

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "aimdk_msgs/msg/message_header.hpp"
#include "aimdk_msgs/msg/upper_body_command_array.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using Msg = aimdk_msgs::msg::UpperBodyCommandArray;

// hand_sub_mode, head_pos, arm_pos, hand_pos
struct ModeConfig {
  uint8_t hand_sub_mode;
  std::array<double, 2> head_pos;
  std::array<double, 14> arm_pos;
  std::vector<double> hand_pos;
};

static const std::map<std::string, ModeConfig> &modes() {
  static const std::map<std::string, ModeConfig> kModes = {
      // head_yaw=0, head_pitch=0 (center)
      {"head",
       {0,
        {{0.0, 0.0}},
        {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0}},
        {}}},
      // left_open=0.5, right_open=0.5
      {"claw",
       {Msg::HAND_CLAW_OPEN_CLOSE, // claw (1)
        {{0.0, 0.0}},
        {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0}},
        {0.5, 0.5}}},
      // left 10 joints + right 10 joints, in rad, all half-open
      // (each joint at half its URDF range limit). Right is the mirror of left.
      // Joint order: [thumb_roll, thumb_abad, thumb_mcp, index_abad, index_pip,
      //               middle_pip, ring_abad, ring_pip, pinky_abad, pinky_pip]
      {"joint",
       {Msg::HAND_DEXTEROUS_JOINT, // dexterous joint (2)
        {{0.0, 0.0}},
        {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0}},
        {-0.6, 0.8,  -0.4, 0.1,  0.7, 0.7, -0.1, 0.7, -0.1, 0.7,
         0.6,  -0.8, 0.4,  -0.1, 0.7, 0.7, 0.1,  0.7, 0.1,  0.7}}},
      // left_gesture=1, left_open=1.0, right_gesture=1, right_open=1.0
      {"gesture",
       {Msg::HAND_DEXTEROUS_GESTURE, // dexterous gesture (3)
        {{0.0, 0.0}},
        {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0}},
        {1.0, 1.0, 1.0, 1.0}}},
  };
  return kModes;
}

class UpperBodyControlNode : public rclcpp::Node {
public:
  UpperBodyControlNode(const std::string &mode)
      : Node("upper_body_control"), seq_(0) {
    const auto &cfg = modes().at(mode);
    hand_sub_mode_ = cfg.hand_sub_mode;
    head_pos_ = cfg.head_pos;
    arm_pos_ = cfg.arm_pos;
    hand_pos_ = cfg.hand_pos;
    pub_ = create_publisher<Msg>("/mc/upper_body_command", 10);
    timer_ = create_wall_timer(20ms, [this]() { publish(); });
    RCLCPP_INFO(get_logger(), "mode=%s  hand_sub_mode=%d", mode.c_str(),
                hand_sub_mode_);
  }

private:
  void publish() {
    Msg msg;
    auto t = this->now();
    msg.header.stamp.sec = static_cast<int32_t>(t.seconds());
    msg.header.stamp.nanosec =
        static_cast<uint32_t>(t.nanoseconds() % 1000000000);
    msg.header.frame_id = "mc_upper_body";
    msg.header.sequence = seq_++;
    msg.source = "upper_body_example";
    msg.hand_sub_mode = hand_sub_mode_;
    msg.head_pos = head_pos_;
    msg.arm_pos = arm_pos_;
    msg.hand_pos = hand_pos_;
    pub_->publish(msg);
  }

  rclcpp::Publisher<Msg>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint32_t seq_;
  uint8_t hand_sub_mode_;
  std::array<double, 2> head_pos_;
  std::array<double, 14> arm_pos_;
  std::vector<double> hand_pos_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::string mode;
  if (argc >= 2)
    mode = argv[1];

  if (modes().find(mode) == modes().end()) {
    std::string usage = "Usage: upper_body_control <";
    bool first = true;
    for (const auto &[k, _] : modes()) {
      if (!first)
        usage += "|";
      usage += k;
      first = false;
    }
    usage += ">";
    std::printf("%s\n", usage.c_str());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(std::make_shared<UpperBodyControlNode>(mode));
  rclcpp::shutdown();
  return 0;
}
