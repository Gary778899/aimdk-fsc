#include <gtest/gtest.h>
#include "locomotion_wrapper/state_validation.hpp"
using namespace locomotion_wrapper;
namespace
{
constexpr int64_t NOW = 1000000000LL;
JointGroup group(const std::string & name = "leg", const std::string & joint_name = "knee")
{
  JointGroup result;
  result.name = name;
  result.expected_names = {joint_name};
  result.received_ns = NOW;
  result.message.emplace();
  result.message->header.stamp.sec = 1;
  aimdk_msgs::msg::JointState joint;
  joint.name = joint_name;
  joint.position = 0.2;
  joint.velocity = 0.3;
  joint.effort = 0.4;
  result.message->joints.push_back(joint);
  return result;
}
std::string compose(const std::vector<JointGroup> & groups, int64_t now = NOW,
  int64_t steady = NOW)
{
  sensor_msgs::msg::JointState output;
  return compose_joints(groups, now, steady, Limits{}, output);
}
}
TEST(StateValidation, MapsValuesAndPreservesOldestSourceStamp)
{
  auto first = group();
  auto second = group("arm", "elbow");
  second.message->header.stamp.nanosec = 5000000;
  sensor_msgs::msg::JointState output;
  ASSERT_TRUE(compose_joints({first, second}, NOW + 5000000, NOW, Limits{}, output).empty());
  EXPECT_EQ(output.header.stamp, first.message->header.stamp);
  ASSERT_EQ(output.name, (std::vector<std::string>{"knee", "elbow"}));
  EXPECT_DOUBLE_EQ(output.position[0], 0.2);
  EXPECT_DOUBLE_EQ(output.velocity[1], 0.3);
  EXPECT_DOUBLE_EQ(output.effort[0], 0.4);
}
TEST(StateValidation, RequiredAndOptionalMissingGroups)
{
  JointGroup missing;
  missing.name = "head";
  missing.expected_names = {"head_joint"};
  EXPECT_NE(compose({group(), missing}).find("missing group"), std::string::npos);
  missing.required = false;
  EXPECT_TRUE(compose({group(), missing}).empty());
}
TEST(StateValidation, SourceAndReceiptAgeAreBothEnforced)
{
  EXPECT_EQ(compose({group()}, NOW + 100000001), "leg: stale data");
  EXPECT_EQ(compose({group()}, NOW, NOW + 100000001), "leg: stale data");
  EXPECT_EQ(compose({group()}, NOW - 10000000), "leg: source timestamp is in future");
  EXPECT_EQ(compose({group()}, NOW, NOW - 1), "leg: stale data");
}
TEST(StateValidation, RejectsZeroStampSkewAndIncompleteJointSet)
{
  auto value = group();
  value.message->header.stamp.sec = 0;
  EXPECT_EQ(compose({value}), "leg: invalid source timestamp");
  value = group();
  value.expected_names = {"knee", "ankle"};
  EXPECT_EQ(compose({value}), "leg: joint set differs from expected_names");
  auto second = group("arm", "elbow");
  second.message->header.stamp.nanosec = 30000000;
  EXPECT_EQ(compose({group(), second}, NOW + 30000000), "joint group source timestamp skew");
}
TEST(StateValidation, RejectsErrorStatesDuplicateAndInvalidValues)
{
  auto value = group();
  value.message->state.value = 4;
  EXPECT_EQ(compose({value}), "leg: domain error state");
  value = group();
  value.message->joints[0].error_code = 1;
  EXPECT_EQ(compose({value}), "leg: joint error_code");
  value = group();
  value.message->joints[0].effort = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(compose({value}), "leg: non-finite joint value");
  EXPECT_EQ(compose({group(), group("arm")}), "arm: empty or duplicate joint name");
  value = group();
  value.message->joints.clear();
  EXPECT_EQ(compose({value}), "leg: empty group");
}
TEST(StateValidation, ImuSourceAgeQuaternionCovarianceAndValues)
{
  sensor_msgs::msg::Imu message;
  message.header.stamp.sec = 1;
  message.orientation.w = 1;
  EXPECT_TRUE(validate_imu(message, NOW, NOW, NOW, Limits{}).empty());
  EXPECT_EQ(validate_imu(message, NOW, NOW, NOW + 100000001, Limits{}), "stale data");
  message.orientation.w = 0;
  EXPECT_EQ(validate_imu(message, NOW, NOW, NOW, Limits{}), "IMU quaternion is not unit length");
  message.orientation.w = 1;
  message.angular_velocity_covariance[0] = -1;
  EXPECT_EQ(validate_imu(message, NOW, NOW, NOW, Limits{}), "IMU field unavailable");
  message.angular_velocity_covariance[0] = 0;
  message.linear_acceleration.x = std::numeric_limits<double>::infinity();
  EXPECT_EQ(validate_imu(message, NOW, NOW, NOW, Limits{}), "non-finite IMU value");
}

TEST(StateValidation, DefaultWbcLegConfigurationRejectsPartialLegs)
{
  auto value = group("leg", "left_knee_joint");
  value.expected_names = wbc_leg_names();
  EXPECT_EQ(compose({value}), "leg: joint set differs from expected_names");
  value.message->joints.clear();
  for (const auto & name : wbc_leg_names()) {
    aimdk_msgs::msg::JointState joint;
    joint.name = name;
    value.message->joints.push_back(joint);
  }
  EXPECT_TRUE(compose({value}).empty());
}
TEST(StateValidation, RequiredGroupsFailClosedWithoutExplicitJointSet)
{
  auto value = group();
  value.expected_names.clear();
  EXPECT_EQ(compose({value}), "leg: required group has no configured expected_names");
}
