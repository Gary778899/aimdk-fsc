#include "locomotion_wrapper/hardware_wrapper_node.hpp"
#include <chrono>
#include <stdexcept>
namespace locomotion_wrapper
{
namespace
{
int64_t steady_now()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}
HardwareWrapperNode::HardwareWrapperNode(const rclcpp::NodeOptions & options)
: Node("hardware_wrapper_node", options)
{
  const auto duration_parameter = [this](const std::string & name, double value) {
      double seconds = declare_parameter<double>(name, value);
      if (!std::isfinite(seconds) || seconds < 0 || seconds > 3600) {
        throw std::invalid_argument(name + " must be finite and within [0, 3600]");
      }
      return static_cast<int64_t>(seconds * 1e9);
    };
  limits_.max_age_ns = duration_parameter("max_age_sec", 0.1);
  limits_.max_skew_ns = duration_parameter("max_group_skew_sec", 0.02);
  limits_.future_tolerance_ns = duration_parameter("future_tolerance_sec", 0.005);
  joint_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
    declare_parameter<std::string>("joint_state_topic", "/humanoid/joint_states"),
    rclcpp::SensorDataQoS());
  imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
    declare_parameter<std::string>("imu_topic", "/humanoid/imu"), rclcpp::SensorDataQoS());
  for (const std::string name : {"head", "arm", "waist", "leg"}) {
    JointGroup group;
    group.name = name;
    group.required = declare_parameter<bool>("require_" + name, name == "leg");
    group.expected_names = declare_parameter<std::vector<std::string>>(
      name + "_expected_names", name == "leg" ? wbc_leg_names() : std::vector<std::string>{});
    const auto index = groups_.size();
    groups_.push_back(group);
    subscriptions_.push_back(create_subscription<aimdk_msgs::msg::JointStateArray>(
      declare_parameter<std::string>(name + "_joint_state_topic", "/aima/hal/joint/" + name + "/state"),
      rclcpp::SensorDataQoS(),
      [this, index](aimdk_msgs::msg::JointStateArray::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        groups_[index].message = *message;
        groups_[index].received_ns = steady_now();
        ++joint_generation_;
      }));
  }
  imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
    declare_parameter<std::string>("imu_input_topic", "/aima/hal/imu/chest/state"),
    rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
      std::lock_guard<std::mutex> lock(mutex_);
      imu_ = *message;
      imu_received_ = steady_now();
      imu_pending_ = true;
    });
  timer_ = create_wall_timer(std::chrono::milliseconds(2), [this]() {publish_valid_states();});
  RCLCPP_INFO(get_logger(), "Read-only state observer; timestamps are preserved and stale data is suppressed");
}
void HardwareWrapperNode::publish_valid_states()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto ros_now = now().nanoseconds();
  const auto steady = steady_now();
  sensor_msgs::msg::JointState joints;
  const auto error = compose_joints(groups_, ros_now, steady, limits_, joints);
  if (!error.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Joint state suppressed: %s", error.c_str());
  } else if (joint_generation_ != published_generation_) {
    joint_publisher_->publish(joints);
    published_generation_ = joint_generation_;
  }
  const auto imu_error = imu_ ? validate_imu(*imu_, imu_received_, ros_now, steady, limits_) : "missing IMU";
  if (!imu_error.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "IMU suppressed: %s", imu_error.c_str());
  } else if (imu_pending_) {
    imu_publisher_->publish(*imu_);
    imu_pending_ = false;
  }
}
}  // namespace locomotion_wrapper
