#include "data_collection/data_collection_node.hpp"

#include <chrono>
#include <cctype>
#include <fstream>
#include <set>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <aimdk_msgs/msg/hand_command_array.hpp>
#include <aimdk_msgs/msg/joint_command_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

#include <rclcpp/serialization.hpp>
#include <zmq.hpp>

#ifndef DATA_COLLECTION_SDK_VERSION
#define DATA_COLLECTION_SDK_VERSION "unknown"
#endif
#ifndef DATA_COLLECTION_CODE_VERSION
#define DATA_COLLECTION_CODE_VERSION "unknown"
#endif

namespace data_collection
{
namespace
{

/** @brief 返回单调时钟的整数纳秒。 @return 仅用于同一进程内间隔的单调时间。 */
int64_t monotonic_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

/** @brief 按配置创建 QoS，未知参数拒绝启动。
 * @param settings reliability、durability、depth 配置。 @return 订阅 QoS。
 * @throws std::invalid_argument 配置取值非法。
 */
rclcpp::QoS configured_qos(const Json & settings)
{
  const auto depth = settings.at("depth").get<int64_t>();
  if (depth <= 0) {
    throw std::invalid_argument("QoS depth must be positive");
  }
  rclcpp::QoS qos{rclcpp::KeepLast(static_cast<std::size_t>(depth))};
  const auto reliability = settings.at("reliability").get<std::string>();
  const auto durability = settings.at("durability").get<std::string>();
  if (reliability == "best_effort") {
    qos.best_effort();
  } else if (reliability == "reliable") {
    qos.reliable();
  } else {
    throw std::invalid_argument("Unsupported QoS reliability: " + reliability);
  }
  if (durability == "volatile") {
    qos.durability_volatile();
  } else if (durability == "transient_local") {
    qos.transient_local();
  } else {
    throw std::invalid_argument("Unsupported QoS durability: " + durability);
  }
  return qos;
}

/** @brief 序列化完整消息。 @tparam Message 消息类型。
 * @param message 原消息。 @return 独占序列化消息。
 */
template<typename Message>
std::shared_ptr<rclcpp::SerializedMessage> serialize_raw(const Message & message)
{
  auto serialized = std::make_shared<rclcpp::SerializedMessage>();
  rclcpp::Serialization<Message> serialization;
  serialization.serialize_message(&message, serialized.get());
  return serialized;
}

}  // namespace

DataCollectionNode::DataCollectionNode(const rclcpp::NodeOptions & options)
: Node("data_collection_node", options)
{
  const auto mapping_file = declare_parameter<std::string>("mapping_file", "");
  const auto data_root = declare_parameter<std::string>("data_root", "");
  const auto queue_capacity = declare_parameter<int64_t>("queue_capacity", 512);
  const auto bind_address = declare_parameter<std::string>("zmq_bind_address", "127.0.0.1");
  const auto port = declare_parameter<int>("zmq_port", 5555);
  const auto model = declare_parameter<std::string>("robot_model", "unknown");
  const auto firmware = declare_parameter<std::string>("firmware", "unknown");
  if (mapping_file.empty() || data_root.empty() || queue_capacity <= 0 ||
    port < 1 || port > 65535 || bind_address.empty())
  {
    throw std::invalid_argument("mapping_file/data_root required; invalid queue capacity or ZMQ endpoint");
  }
  std::ifstream input(mapping_file);
  if (!input) {
    throw std::runtime_error("Cannot open mapping file: " + mapping_file);
  }
  mapping_ = Json::parse(input);
  auto selected = declare_parameter<std::vector<std::string>>("selected_streams", {"arms", "hands"});
  std::set<std::string> requested(selected.begin(), selected.end());
  for (std::size_t index = 0; index < selected.size(); ++index) {
    for (const auto & stream : mapping_.at("streams")) {
      if (stream.at("id") != selected[index]) {continue;}
      for (const auto & dependency : stream.value("includes", Json::array())) {
        const auto id = dependency.get<std::string>();
        if (requested.insert(id).second) {selected.push_back(id);}
      }
    }
  }
  std::set<std::string> resolved;
  Json active = Json::array();
  for (const auto & stream : mapping_.at("streams")) {
    const auto id = stream.at("id").get<std::string>();
    if (requested.count(id)) {
      active.push_back(stream);
      resolved.insert(id);
    }
  }
  if (requested.empty() || resolved != requested) {
    throw std::invalid_argument("selected_streams must contain known stream ids");
  }
  mapping_["streams"] = active;
  const auto storage_config = declare_parameter<std::string>("mcap_storage_config", "");
  if (storage_config.empty()) {
    throw std::invalid_argument("mcap_storage_config is required for compressed MCAP");
  }
  const Json metadata{{"sdk_version", DATA_COLLECTION_SDK_VERSION},
    {"code_version_at_build", DATA_COLLECTION_CODE_VERSION}, {"robot_model", model},
    {"firmware", firmware}, {"mapping_file", mapping_file},
    {"mcap_storage_config", storage_config}, {"selected_streams", selected},
    {"acquisition", Json::parse(declare_parameter<std::string>("acquisition_metadata_json", "{}"))},
    {"use_sim_time", get_parameter("use_sim_time").as_bool()},
    {"source_time_clock", "unverified"}, {"receive_time_clock", "ROS"},
    {"receive_monotonic_clock", "steady_process_local"}};
  writer_ = std::make_unique<SessionWriter>(
    mapping_, data_root, static_cast<std::size_t>(queue_capacity), metadata);
  create_inputs();
  zmq_bind_endpoint_ = "tcp://" + bind_address + ":" + std::to_string(port);
  auto ready = zmq_ready_.get_future();
  zmq_thread_ = std::thread(&DataCollectionNode::zmq_server_loop, this);
  try {
    ready.get();
  } catch (...) {
    shutdown_requested_.store(true);
    zmq_thread_.join();
    throw;
  }
  RCLCPP_INFO(get_logger(), "Collector ready: %zu sources, control %s, compressed MCAP",
    mapping_.at("streams").size(), zmq_bind_endpoint_.c_str());
}

DataCollectionNode::~DataCollectionNode()
{
  shutdown_requested_.store(true);
  if (zmq_thread_.joinable()) {
    zmq_thread_.join();
  }
  try {
    stop_recording();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Session close failed: %s", error.what());
  }
}

/** @brief 转交完整消息，按类型保存源时间和图像属性，异常锁存到当前会话。 */
template<typename MessageT>
void DataCollectionNode::on_sensor(const Json & stream, std::shared_ptr<MessageT> message)
{
  const auto context = receive_context();
  if (!context.active) {return;}
  try {
    Json description{{"receive_monotonic_ns", context.monotonic_ns}};
    if constexpr (!std::is_same_v<MessageT, std_msgs::msg::String>) {
      const auto & stamp = message->header.stamp;
      description["source_stamp_ns"] = static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
      description["frame_id"] = message->header.frame_id;
    }
    if constexpr (std::is_same_v<MessageT, aimdk_msgs::msg::JointStateArray> ||
      std::is_same_v<MessageT, aimdk_msgs::msg::HandStateArray> ||
      std::is_same_v<MessageT, aimdk_msgs::msg::JointCommandArray> ||
      std::is_same_v<MessageT, aimdk_msgs::msg::HandCommandArray>)
    {
      const auto & measured = message->header.meas_stamp;
      description["meas_stamp_ns"] = static_cast<int64_t>(measured.sec) * 1000000000LL + measured.nanosec;
      description["sequence"] = message->header.sequence;
    }
    if constexpr (std::is_same_v<MessageT, sensor_msgs::msg::Image>) {
      description["width"] = message->width;
      description["height"] = message->height;
      description["encoding"] = message->encoding;
      description["step"] = message->step;
      description["is_bigendian"] = message->is_bigendian;
    }
    const auto serialize_begin = monotonic_now_ns();
    auto serialized = serialize_raw(*message);
    description["serialization_wall_ns"] = monotonic_now_ns() - serialize_begin;
    accept_raw(stream, std::move(serialized), context, description);
  } catch (const std::exception & error) {
    fail_session(error.what(), context.generation);
  }
}

void DataCollectionNode::accept_raw(
  const Json & stream, std::shared_ptr<rclcpp::SerializedMessage> message,
  const ReceiveContext & context, const Json & description)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!recording_enabled_ || generation_ != context.generation || !writer_->is_recording()) {
    return;
  }
  writer_->enqueue_raw(stream.at("topic").get<std::string>(),
    stream.at("message_type").get<std::string>(), std::move(message), context.time_ns,
    description);
}

void DataCollectionNode::create_inputs()
{
  for (const auto & stream : mapping_.at("streams")) {
    const auto type = stream.at("message_type").get<std::string>();
    const auto topic = stream.at("topic").get<std::string>();
    const auto qos = configured_qos(stream.at("qos"));
    // Humble 对 std::bind 的类型推导保留原成员函数参数；用单消息参数的 lambda 适配。
    // 按值捕获当前来源配置，使回调在循环结束后仍持有正确的映射。
    if (type == "sensor_msgs/msg/Image") {
      /** @brief 转交原始图像，不进行图像编码。 */
      const auto callback = [this, stream](sensor_msgs::msg::Image::SharedPtr message) {
          on_sensor(stream, std::move(message));
        };
      subscriptions_.push_back(create_subscription<sensor_msgs::msg::Image>(topic, qos, callback));
    } else if (type == "sensor_msgs/msg/CameraInfo") {
      /** @brief 转交相机内参。 */
      const auto callback = [this, stream](sensor_msgs::msg::CameraInfo::SharedPtr message) {
          on_sensor(stream, std::move(message));
        };
      subscriptions_.push_back(create_subscription<sensor_msgs::msg::CameraInfo>(topic, qos, callback));
    } else if (type == "sensor_msgs/msg/Imu") {
      /** @brief 转交原始 IMU。 */
      const auto callback = [this, stream](sensor_msgs::msg::Imu::SharedPtr message) {
          on_sensor(stream, std::move(message));
        };
      subscriptions_.push_back(create_subscription<sensor_msgs::msg::Imu>(topic, qos, callback));
    } else if (type == "aimdk_msgs/msg/JointStateArray") {
      /** @brief 转交所配置身体部位的完整原始关节反馈。 */
      const auto callback = [this, stream](aimdk_msgs::msg::JointStateArray::SharedPtr message) {
          on_sensor(stream, std::move(message));
        };
      subscriptions_.push_back(create_subscription<aimdk_msgs::msg::JointStateArray>(topic, qos, callback));
    } else if (type == "aimdk_msgs/msg/HandStateArray") {
      /** @brief 转交夹爪原始反馈。 */
      const auto callback = [this, stream](aimdk_msgs::msg::HandStateArray::SharedPtr message) {
          on_sensor(stream, std::move(message));
        };
      subscriptions_.push_back(create_subscription<aimdk_msgs::msg::HandStateArray>(topic, qos, callback));
    } else if (type == "aimdk_msgs/msg/JointCommandArray") {
      /** @brief 只读转交 HAL 原始关节指令。 */
      const auto callback = [this, stream](aimdk_msgs::msg::JointCommandArray::SharedPtr message) {
          on_sensor(stream, std::move(message));
        };
      subscriptions_.push_back(create_subscription<aimdk_msgs::msg::JointCommandArray>(topic, qos, callback));
    } else if (type == "aimdk_msgs/msg/HandCommandArray") {
      /** @brief 只读转交夹爪类型及左右夹爪完整原始指令。 */
      const auto callback = [this, stream](aimdk_msgs::msg::HandCommandArray::SharedPtr message) {
          on_sensor(stream, std::move(message));
        };
      subscriptions_.push_back(create_subscription<aimdk_msgs::msg::HandCommandArray>(topic, qos, callback));
    } else if (type == "std_msgs/msg/String") {
      /** @brief 转交原始 XR 事件信封，由读回工具解析载荷和 SDK 时间。 */
      const auto callback = [this, stream](std_msgs::msg::String::SharedPtr message) {
          on_sensor(stream, std::move(message));
        };
      subscriptions_.push_back(create_subscription<std_msgs::msg::String>(topic, qos, callback));
    } else {
      throw std::invalid_argument("No input adapter for message_type: " + type);
    }
  }
}

DataCollectionNode::ReceiveContext DataCollectionNode::receive_context()
{
  // 时间在回调入口捕获，磁盘线程不得重写接收时间。
  const auto entry_generation = generation_.load();
  const auto received_ros = get_clock()->now().nanoseconds();
  const auto received_monotonic = monotonic_now_ns();
  std::lock_guard<std::mutex> lock(state_mutex_);
  return ReceiveContext{recording_enabled_ && generation_.load() == entry_generation,
    entry_generation, received_ros, received_monotonic};
}

Json DataCollectionNode::clock_anchor()
{
  const auto before = monotonic_now_ns();
  const auto system = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  const auto ros = get_clock()->now().nanoseconds();
  const auto after = monotonic_now_ns();
  return Json{{"monotonic_before_ns", before}, {"system_time_ns", system},
    {"ros_time_ns", ros}, {"monotonic_after_ns", after}};
}

void DataCollectionNode::fail_session(const std::string & reason, uint64_t generation)
{
  std::lock_guard<std::mutex> control_lock(control_mutex_);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (generation_ != generation) {
      return;
    }
    recording_enabled_ = false;
    ++generation_;
    writer_->record_clock_anchor("gate_closed", clock_anchor());
    node_error_ = reason;
  }
  writer_->fail(reason);
  RCLCPP_ERROR(get_logger(), "Session input failed: %s", reason.c_str());
}

void DataCollectionNode::start_recording()
{
  std::lock_guard<std::mutex> control_lock(control_mutex_);
  if (writer_->is_recording()) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    recording_enabled_ = false;
    ++generation_;
  }
  const auto path = writer_->start();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    node_error_.clear();
    // 开门时再次推进代号，拒绝 writer 初始化期间已进入但尚未取锁的回调。
    ++generation_;
    recording_enabled_ = true;
    writer_->record_clock_anchor("gate_opened", clock_anchor());
  }
  RCLCPP_INFO(get_logger(), "Recording started: %s", path.c_str());
}

void DataCollectionNode::stop_recording()
{
  std::lock_guard<std::mutex> control_lock(control_mutex_);
  const bool already_closed = writer_->status().at("closed").get<bool>();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool was_recording = recording_enabled_;
    recording_enabled_ = false;
    ++generation_;
    if (was_recording) {
      writer_->record_clock_anchor("gate_closed", clock_anchor());
    }
  }
  // 门控关闭之后再排空，避免在接收锁内等待磁盘 I/O。
  // 只有排空、关闭和摘要写入成功返回后才输出停止日志；异常仍交给调用者报告。
  const auto summary = writer_->stop();
  const auto path = summary.at("session_dir").get<std::string>();
  if (path.empty()) {
    RCLCPP_INFO(get_logger(), "Recording already stopped: no session has been started");
    return;
  }
  const auto raw_counts = summary.at("raw").dump();
  RCLCPP_INFO(get_logger(), "Recording %s: %s; raw=%s; data_complete=%s",
    already_closed ? "already stopped" : "stopped", path.c_str(), raw_counts.c_str(),
    summary.at("data_complete").get<bool>() ? "true" : "false");
}

Json DataCollectionNode::status()
{
  auto result = writer_->status();
  std::lock_guard<std::mutex> lock(state_mutex_);
  result["recording_enabled"] = recording_enabled_ && writer_->is_recording();
  const auto now = monotonic_now_ns();
  for (auto & topic : result.at("raw").items()) {
    auto & counts = topic.value();
    if (counts.contains("last_description")) {
      counts["receive_age_ns"] = now - counts.at("last_description").at("receive_monotonic_ns").get<int64_t>();
    } else {
      counts["receive_age_ns"] = nullptr;
    }
  }
  result["control_ready"] = control_ready_.load();
  result["node_error"] = node_error_.empty() ? Json(nullptr) : Json(node_error_);
  return result;
}

void DataCollectionNode::zmq_server_loop()
{
  bool ready = false;
  try {
    zmq::context_t context(1);
    zmq::socket_t socket(context, zmq::socket_type::rep);
    socket.set(zmq::sockopt::linger, 0);
    socket.set(zmq::sockopt::sndtimeo, 200);
    socket.set(zmq::sockopt::rcvtimeo, 200);
    socket.bind(zmq_bind_endpoint_);
    control_ready_.store(true);
    zmq_ready_.set_value();
    ready = true;
    while (!shutdown_requested_.load() && rclcpp::ok()) {
      zmq::pollitem_t items[] = {{socket.handle(), 0, ZMQ_POLLIN, 0}};
      zmq::poll(items, 1, std::chrono::milliseconds(100));
      if (!(items[0].revents & ZMQ_POLLIN)) {
        continue;
      }
      zmq::message_t request;
      if (!socket.recv(request, zmq::recv_flags::none)) {
        continue;
      }
      const auto command = normalize_command(
        std::string(static_cast<const char *>(request.data()), request.size()));
      std::string response;
      try {
        if (command == "START") {
          start_recording();
          response = "ACK";
        } else if (command == "STOP") {
          stop_recording();
          response = "ACK";
        } else if (command == "STATUS") {
          response = status().dump();
        } else {
          response = "ERROR unknown command";
        }
      } catch (const std::exception & error) {
        response = std::string("ERROR ") + error.what();
        RCLCPP_ERROR(get_logger(), "Control command failed: %s", error.what());
      }
      if (!socket.send(zmq::buffer(response), zmq::send_flags::none)) {
        throw std::runtime_error("ZMQ reply timed out");
      }
    }
    control_ready_.store(false);
    socket.close();
    context.close();
  } catch (const std::exception & error) {
    control_ready_.store(false);
    if (!ready) {
      zmq_ready_.set_exception(std::current_exception());
    } else {
      // 控制通道故障须使会话标记为未完整完成。
      RCLCPP_ERROR(get_logger(), "Control server failed: %s", error.what());
      {
        std::lock_guard<std::mutex> control_lock(control_mutex_);
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          if (recording_enabled_) {
            writer_->record_clock_anchor("gate_closed", clock_anchor());
          }
          recording_enabled_ = false;
          ++generation_;
          node_error_ = error.what();
        }
        writer_->fail(std::string("Control server failed: ") + error.what());
      }
      try {
        stop_recording();
      } catch (const std::exception & close_error) {
        RCLCPP_ERROR(get_logger(), "Failed session close: %s", close_error.what());
      }
    }
  }
}

std::string DataCollectionNode::normalize_command(std::string command)
{
  const auto first = command.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  command = command.substr(first, command.find_last_not_of(" \t\r\n") - first + 1);
  for (char & value : command) {
    value = static_cast<char>(std::toupper(static_cast<unsigned char>(value)));
  }
  return command;
}

}  // namespace data_collection

/** @brief 初始化 ROS 并运行独立采集节点，构造或执行失败时返回非零。
 * @param argc 参数数量。 @param argv ROS 参数。 @return 正常为 0，错误为 1。
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<data_collection::DataCollectionNode>();
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("data_collection"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
}
