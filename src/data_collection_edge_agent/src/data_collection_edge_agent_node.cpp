#include "data_collection_edge_agent/data_collection_edge_agent_node.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <ctime>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <memory>
#include <future>
#include <stdexcept>
#include <unistd.h>
#include <sstream>
#include <thread>

#include <rmw/rmw.h>
#include <rosbag2_storage/topic_metadata.hpp>
#include <zmq.hpp>

namespace data_collection_edge_agent
{

DataCollectionEdgeAgentNode::DataCollectionEdgeAgentNode(const rclcpp::NodeOptions & options)
: Node("data_collection_edge_agent_node", options)
{
  bag_root_dir_ = this->declare_parameter<std::string>("bag_root_dir", "./");
  joint_state_topic_ = this->declare_parameter<std::string>("joint_state_topic", "/humanoid/joint_states");
  imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/humanoid/imu");
  zmq_bind_address_ = this->declare_parameter<std::string>("zmq_bind_address", "127.0.0.1");
  zmq_port_ = this->declare_parameter<int>("zmq_port", 5555);
  zmq_bind_endpoint_ = "tcp://" + zmq_bind_address_ + ":" + std::to_string(zmq_port_);

  auto qos = rclcpp::SensorDataQoS().best_effort().keep_last(10);

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic_,
    qos,
    std::bind(&DataCollectionEdgeAgentNode::on_joint_state, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_,
    qos,
    std::bind(&DataCollectionEdgeAgentNode::on_imu, this, std::placeholders::_1));

  if (zmq_port_ < 1 || zmq_port_ > 65535 || joint_state_topic_ == imu_topic_) {
    throw std::invalid_argument("Invalid ZMQ port or identical sensor topics");
  }
  auto ready = zmq_ready_.get_future();
  zmq_thread_ = std::thread([this]() { this->zmq_server_loop(); });
  try {
    ready.get();  // Surface bind failures synchronously to the launcher.
  } catch (...) {
    shutdown_requested_.store(true);
    zmq_thread_.join();
    throw;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Data Collection Edge Agent ready. ZMQ REP listening on %s",
    zmq_bind_endpoint_.c_str());
}

DataCollectionEdgeAgentNode::~DataCollectionEdgeAgentNode()
{
  shutdown_requested_.store(true, std::memory_order_release);

  if (zmq_thread_.joinable()) {
    zmq_thread_.join();
  }
  std::lock_guard<std::mutex> lock(writer_mutex_);
  try {
    stop_recording_locked();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Bag close failed: %s", ex.what());
  }
}

void DataCollectionEdgeAgentNode::on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!is_recording_.load(std::memory_order_relaxed)) {
    return;
  }

  std::lock_guard<std::mutex> lock(writer_mutex_);
  if (!is_recording_.load(std::memory_order_relaxed) || !writer_) {
    return;
  }

  try {
    writer_->write(*msg, joint_state_topic_, this->get_clock()->now());
  } catch (const std::exception & ex) {
    last_error_ = ex.what();
    is_recording_.store(false);
    RCLCPP_ERROR(get_logger(), "Recording write failed: %s", ex.what());
    try {stop_recording_locked();} catch (const std::exception &) {}
  }
}

void DataCollectionEdgeAgentNode::on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (!is_recording_.load(std::memory_order_relaxed)) {
    return;
  }

  std::lock_guard<std::mutex> lock(writer_mutex_);
  if (!is_recording_.load(std::memory_order_relaxed) || !writer_) {
    return;
  }

  try {
    writer_->write(*msg, imu_topic_, this->get_clock()->now());
  } catch (const std::exception & ex) {
    last_error_ = ex.what();
    is_recording_.store(false);
    RCLCPP_ERROR(get_logger(), "Recording write failed: %s", ex.what());
    try {stop_recording_locked();} catch (const std::exception &) {}
  }
}

void DataCollectionEdgeAgentNode::start_recording_locked()
{
  if (is_recording_.load(std::memory_order_relaxed)) {
    RCLCPP_WARN(this->get_logger(), "START ignored: already recording");
    return;
  }

  if (writer_) {
    stop_recording_locked();
  }
  const std::string bag_uri = make_timestamped_bag_uri();
  std::filesystem::create_directories(std::filesystem::path(bag_uri).parent_path());

  auto writer = std::make_unique<rosbag2_cpp::Writer>();

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_uri;
  storage_options.storage_id = "mcap";

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = rmw_get_serialization_format();
  converter_options.output_serialization_format = rmw_get_serialization_format();

  writer->open(storage_options, converter_options);

  const std::string serialization_format = rmw_get_serialization_format();
  rosbag2_storage::TopicMetadata joint_metadata;
  joint_metadata.name = joint_state_topic_;
  joint_metadata.type = "sensor_msgs/msg/JointState";
  joint_metadata.serialization_format = serialization_format;
  writer->create_topic(joint_metadata);
  rosbag2_storage::TopicMetadata imu_metadata;
  imu_metadata.name = imu_topic_;
  imu_metadata.type = "sensor_msgs/msg/Imu";
  imu_metadata.serialization_format = serialization_format;
  writer->create_topic(imu_metadata);
  writer_ = std::move(writer);
  last_error_.clear();

  is_recording_.store(true, std::memory_order_release);
  RCLCPP_INFO(this->get_logger(), "Recording started: %s", bag_uri.c_str());
}

void DataCollectionEdgeAgentNode::stop_recording_locked()
{
  if (!is_recording_.load(std::memory_order_relaxed) && !writer_) {
    return;
  }

  is_recording_.store(false, std::memory_order_release);

  if (writer_) {
    auto writer = std::move(writer_);
    writer->close();
  }

  RCLCPP_INFO(this->get_logger(), "Recording stopped");
}

void DataCollectionEdgeAgentNode::zmq_server_loop()
{
  bool ready = false;
  try {
    zmq::context_t context(1);
    zmq::socket_t socket(context, zmq::socket_type::rep);
    socket.set(zmq::sockopt::linger, 0);
    socket.set(zmq::sockopt::sndtimeo, 200);
    socket.set(zmq::sockopt::rcvtimeo, 200);
    socket.bind(zmq_bind_endpoint_);
    zmq_ready_.set_value();
    ready = true;

    while (!shutdown_requested_.load(std::memory_order_acquire)) {
      zmq::pollitem_t poll_items[] = {
        {socket.handle(), 0, ZMQ_POLLIN, 0}
      };
      zmq::poll(poll_items, 1, std::chrono::milliseconds(100));

      if (!(poll_items[0].revents & ZMQ_POLLIN)) {
        continue;
      }

      zmq::message_t request;
      const auto received = socket.recv(request, zmq::recv_flags::none);
      if (!received.has_value()) {
        continue;
      }

      std::string command(static_cast<const char *>(request.data()), request.size());
      command = normalize_command(std::move(command));

      std::string response = "ACK";
      try {
        std::lock_guard<std::mutex> lock(writer_mutex_);
        if (command == "START") {
          start_recording_locked();
        } else if (command == "STOP") {
          stop_recording_locked();
          if (!last_error_.empty()) {
            response = "ERROR " + last_error_;
          }
        } else {
          response = "ERROR unknown command";
        }
      } catch (const std::exception & ex) {
        response = std::string("ERROR ") + ex.what();
        RCLCPP_ERROR(get_logger(), "Command failed: %s", ex.what());
      }

      if (!socket.send(zmq::buffer(response), zmq::send_flags::none)) {
        throw std::runtime_error("ZMQ reply timed out");
      }
    }

    socket.close();
    context.close();
  } catch (const std::exception & ex) {
    if (!ready) {
      zmq_ready_.set_exception(std::current_exception());
    } else {
      RCLCPP_ERROR(this->get_logger(), "ZMQ loop exception: %s", ex.what());
      // A lost control server must not leave an unattended recording running.
      std::lock_guard<std::mutex> lock(writer_mutex_);
      try {stop_recording_locked();} catch (const std::exception &) {}
    }
  }
}

std::string DataCollectionEdgeAgentNode::make_timestamped_bag_uri() const
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  std::tm tm_now;
#if defined(_WIN32)
  localtime_s(&tm_now, &now_c);
#else
  localtime_r(&now_c, &tm_now);
#endif

  std::ostringstream oss;
  static std::atomic<unsigned long> sequence{0};
  oss << "data_log_" << std::put_time(&tm_now, "%Y%m%d_%H%M%S") << "_"
      << std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count()
      << "_" << getpid() << "_" << sequence.fetch_add(1);

  return (std::filesystem::path(bag_root_dir_) / oss.str()).string();
}

std::string DataCollectionEdgeAgentNode::normalize_command(std::string command)
{
  command.erase(
    std::remove_if(command.begin(), command.end(), [](unsigned char c) {
      return std::isspace(c) != 0;
    }),
    command.end());

  std::transform(command.begin(), command.end(), command.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });

  return command;
}

}  // namespace data_collection_edge_agent

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<data_collection_edge_agent::DataCollectionEdgeAgentNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
