#ifndef LOCOMOTION_WRAPPER__STATE_VALIDATION_HPP_
#define LOCOMOTION_WRAPPER__STATE_VALIDATION_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <vector>
#include "aimdk_msgs/msg/joint_state_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace locomotion_wrapper
{
inline int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}
struct Limits
{
  int64_t max_age_ns = 100000000;
  int64_t max_skew_ns = 20000000;
  int64_t future_tolerance_ns = 5000000;
};
inline std::string time_error(
  const builtin_interfaces::msg::Time & stamp, int64_t received, int64_t now,
  int64_t steady_now, const Limits & limits)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000 || stamp_ns(stamp) <= 0) {
    return "invalid source timestamp";
  }
  const auto age = now - stamp_ns(stamp);
  if (age < -limits.future_tolerance_ns) {return "source timestamp is in future";}
  if (age > limits.max_age_ns || steady_now < received ||
    steady_now - received > limits.max_age_ns)
  {return "stale data";}
  return {};
}
inline std::vector<std::string> wbc_leg_names()
{
  return {"left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"};
}
struct JointGroup
{
  std::string name;
  bool required = true;
  std::vector<std::string> expected_names;
  std::optional<aimdk_msgs::msg::JointStateArray> message;
  int64_t received_ns = 0;
};
inline std::string compose_joints(
  const std::vector<JointGroup> & groups, int64_t now, int64_t steady_now,
  const Limits & limits, sensor_msgs::msg::JointState & output)
{
  sensor_msgs::msg::JointState candidate;
  int64_t earliest = std::numeric_limits<int64_t>::max();
  int64_t latest = 0;
  std::set<std::string> seen;
  for (const auto & group : groups) {
    if (group.required && group.expected_names.empty()) {
      return group.name + ": required group has no configured expected_names";
    }
    if (!group.message) {
      if (group.required) {return group.name + ": missing group";}
      continue;
    }
    const auto & message = *group.message;
    const auto error = time_error(message.header.stamp, group.received_ns, now, steady_now, limits);
    if (!error.empty()) {return group.name + ": " + error;}
    if (message.state.value != 0) {return group.name + ": domain error state";}
    if (message.joints.empty()) {return group.name + ": empty group";}
    std::set<std::string> group_names;
    for (const auto & joint : message.joints) {
      if (joint.name.empty() || !seen.insert(joint.name).second) {
        return group.name + ": empty or duplicate joint name";
      }
      if (joint.error_code != 0) {return group.name + ": joint error_code";}
      if (!std::isfinite(joint.position) || !std::isfinite(joint.velocity) ||
        !std::isfinite(joint.effort)) {return group.name + ": non-finite joint value";}
      group_names.insert(joint.name);
      candidate.name.push_back(joint.name);
      candidate.position.push_back(joint.position);
      candidate.velocity.push_back(joint.velocity);
      candidate.effort.push_back(joint.effort);
    }
    if (!group.expected_names.empty() &&
      group_names != std::set<std::string>(group.expected_names.begin(), group.expected_names.end()))
    {return group.name + ": joint set differs from expected_names";}
    const auto stamp = stamp_ns(message.header.stamp);
    if (stamp < earliest) {earliest = stamp; candidate.header = std_msgs::msg::Header();
      candidate.header.stamp = message.header.stamp;}
    latest = std::max(latest, stamp);
  }
  if (candidate.name.empty()) {return "no joint data";}
  if (latest - earliest > limits.max_skew_ns) {return "joint group source timestamp skew";}
  output = std::move(candidate);
  return {};
}
inline std::string validate_imu(
  const sensor_msgs::msg::Imu & message, int64_t received, int64_t now,
  int64_t steady_now, const Limits & limits)
{
  const auto error = time_error(message.header.stamp, received, now, steady_now, limits);
  if (!error.empty()) {return error;}
  const auto & q = message.orientation;
  const auto & a = message.linear_acceleration;
  const auto & w = message.angular_velocity;
  const std::vector<double> values = {q.x, q.y, q.z, q.w, a.x, a.y, a.z, w.x, w.y, w.z};
  for (auto value : values) {if (!std::isfinite(value)) {return "non-finite IMU value";}}
  for (const auto * covariance : {&message.orientation_covariance,
      &message.angular_velocity_covariance, &message.linear_acceleration_covariance})
  {
    if ((*covariance)[0] < 0) {return "IMU field unavailable";}
    for (auto value : *covariance) {
      if (!std::isfinite(value)) {return "non-finite IMU covariance";}
    }
  }
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  if (std::abs(norm - 1.0) > 0.01) {return "IMU quaternion is not unit length";}
  return {};
}
}  // namespace locomotion_wrapper
#endif
