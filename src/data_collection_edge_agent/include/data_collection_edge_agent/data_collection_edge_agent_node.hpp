#pragma once

#include <atomic>
#include <mutex>
#include <thread>
#include <future>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <rosbag2_cpp/writer.hpp>

namespace data_collection_edge_agent
{

class DataCollectionEdgeAgentNode : public rclcpp::Node
{
public:
  explicit DataCollectionEdgeAgentNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DataCollectionEdgeAgentNode() override;

private:
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg);

  void start_recording_locked();
  void stop_recording_locked();
  void zmq_server_loop();

  std::string make_timestamped_bag_uri() const;
  static std::string normalize_command(std::string command);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  mutable std::mutex writer_mutex_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;

  std::string bag_root_dir_;
  std::string joint_state_topic_;
  std::string imu_topic_;
  std::string zmq_bind_address_;
  int zmq_port_{5555};
  std::string zmq_bind_endpoint_;

  std::atomic<bool> is_recording_{false};
  std::atomic<bool> shutdown_requested_{false};
  std::thread zmq_thread_;
  std::promise<void> zmq_ready_;
  std::string last_error_;
};

}  // namespace data_collection_edge_agent
