#ifndef LOCOMOTION_WRAPPER__HARDWARE_WRAPPER_NODE_HPP_
#define LOCOMOTION_WRAPPER__HARDWARE_WRAPPER_NODE_HPP_
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "locomotion_wrapper/state_validation.hpp"
namespace locomotion_wrapper
{
class HardwareWrapperNode : public rclcpp::Node
{
public:
  explicit HardwareWrapperNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
private:
  void publish_valid_states();
  std::mutex mutex_;
  Limits limits_;
  std::vector<JointGroup> groups_;
  std::vector<rclcpp::Subscription<aimdk_msgs::msg::JointStateArray>::SharedPtr> subscriptions_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<sensor_msgs::msg::Imu> imu_;
  int64_t imu_received_ = 0;
  uint64_t joint_generation_ = 0, published_generation_ = 0;
  bool imu_pending_ = false;
};
}  // namespace locomotion_wrapper
#endif
