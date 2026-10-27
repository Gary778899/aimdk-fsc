#include "data_collection/data_collection_node.hpp"

#include <chrono>
#include <cctype>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

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

/** @brief 保留 SDK 时间的秒及纳秒，不替换零时间。
 * @param time SDK 时间。 @return 原字段对象。
 */
Json time_json(const builtin_interfaces::msg::Time & time)
{
  return Json{{"sec", time.sec}, {"nanosec", time.nanosec}};
}

/** @brief 完整保留 SDK 消息头。 @param header 原消息头。 @return 消息头对象。 */
Json header_json(const aimdk_msgs::msg::MessageHeader & header)
{
  return Json{{"stamp", time_json(header.stamp)}, {"meas_stamp", time_json(header.meas_stamp)},
    {"sequence", header.sequence}, {"frame_id", header.frame_id}};
}

/** @brief 转换双臂反馈，不重排、不裁剪、不过滤故障。
 * @param message SDK 双臂状态。 @return 可由 JSON 路径寻址的对象。
 */
Json arm_json(const aimdk_msgs::msg::JointStateArray & message)
{
  Json joints = Json::array();
  for (const auto & joint : message.joints) {
    joints.push_back(Json{{"name", joint.name}, {"position", joint.position},
      {"velocity", joint.velocity}, {"effort", joint.effort}, {"error_code", joint.error_code}});
  }
  return Json{{"header", header_json(message.header)}, {"state", {{"value", message.state.value}}},
    {"joints", std::move(joints)}};
}

/** @brief 转换一侧手部数组，保留 SDK 名称及数组顺序。
 * @param hands SDK 手部状态。 @return 原始手部对象数组。
 */
Json hands_json(const std::vector<aimdk_msgs::msg::HandState> & hands)
{
  Json result = Json::array();
  for (const auto & hand : hands) {
    result.push_back(Json{{"name", hand.name}, {"position", hand.position},
      {"velocity", hand.velocity}, {"effort", hand.effort},
      {"state", hand.state}, {"faultcode", hand.faultcode}});
  }
  return result;
}

/** @brief 转换双夹爪反馈；本阶段不提取触觉，原始 MCAP 仍保留整条消息。
 * @param message SDK 双手状态。 @return 原消息头、手型及状态字段。
 */
Json hand_json(const aimdk_msgs::msg::HandStateArray & message)
{
  return Json{{"header", header_json(message.header)},
    {"left_hand_type", {{"value", message.left_hand_type.value}}},
    {"right_hand_type", {{"value", message.right_hand_type.value}}},
    {"left_hands", hands_json(message.left_hands)},
    {"right_hands", hands_json(message.right_hands)}};
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

/** @brief 按需序列化完整 SDK 消息。 @tparam Message 消息类型。
 * @param message 原消息。 @param enabled 原始录制开关。 @return 消息或空指针。
 */
template<typename Message>
std::shared_ptr<rclcpp::SerializedMessage> serialize_raw(const Message & message, bool enabled)
{
  if (!enabled) {
    return nullptr;
  }
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
  record_raw_ = declare_parameter<bool>("record_raw_mcap", false);
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
  mapper_ = std::make_unique<StateMapper>(mapping_);
  sample_rate_hz_ = mapping_.at("sample_rate_hz").get<double>();
  if (!std::isfinite(sample_rate_hz_) || sample_rate_hz_ <= 0.0) {
    throw std::invalid_argument("sample_rate_hz must be positive and finite");
  }
  const double period_ns = 1000000000.0 / sample_rate_hz_;
  // 这里只检查整数纳秒的可表达范围，不设置额外的采集频率门槛。
  if (!std::isfinite(period_ns) || period_ns < 1.0 ||
    period_ns >= static_cast<double>(std::numeric_limits<int64_t>::max()))
  {
    throw std::invalid_argument("sample_rate_hz produces an unrepresentable sampling period");
  }
  const auto period = std::chrono::nanoseconds(static_cast<int64_t>(period_ns));
  const Json metadata{{"sdk_version", DATA_COLLECTION_SDK_VERSION},
    {"code_version_at_build", DATA_COLLECTION_CODE_VERSION}, {"robot_model", model},
    {"firmware", firmware}, {"mapping_file", mapping_file},
    {"use_sim_time", get_parameter("use_sim_time").as_bool()},
    {"source_time_clock", "unverified"}, {"sampling_clock", "steady"},
    {"sample_time_clock", "ROS"}, {"receive_time_clock", "ROS"}};
  writer_ = std::make_unique<SessionWriter>(
    mapping_, data_root, record_raw_, static_cast<std::size_t>(queue_capacity), metadata);
  create_inputs();
  sample_timer_ = create_wall_timer(period, std::bind(&DataCollectionNode::sample, this));
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
  RCLCPP_INFO(get_logger(), "Collector ready: %.3f Hz, control %s, raw MCAP %s",
    sample_rate_hz_, zmq_bind_endpoint_.c_str(), record_raw_ ? "enabled" : "disabled");
}

DataCollectionNode::~DataCollectionNode()
{
  shutdown_requested_.store(true);
  if (sample_timer_) {
    sample_timer_->cancel();
  }
  if (zmq_thread_.joinable()) {
    zmq_thread_.join();
  }
  try {
    stop_recording();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Session close failed: %s", error.what());
  }
}

void DataCollectionNode::create_inputs()
{
  for (const auto & stream : mapping_.at("streams")) {
    const auto type = stream.at("message_type").get<std::string>();
    const auto topic = stream.at("topic").get<std::string>();
    const auto qos = configured_qos(stream.at("qos"));
    // Humble 对 std::bind 的类型推导保留原成员函数参数；用单消息参数的 lambda 适配。
    // 按值捕获当前来源配置，使回调在循环结束后仍持有正确的映射。
    if (type == "aimdk_msgs/msg/JointStateArray") {
      /** @brief 将双臂消息交给当前来源的适配器。 @param message SDK 双臂反馈。 */
      const auto callback = [this, stream](aimdk_msgs::msg::JointStateArray::SharedPtr message) {
          on_arm(stream, std::move(message));
        };
      subscriptions_.push_back(create_subscription<aimdk_msgs::msg::JointStateArray>(
        topic, qos, callback));
    } else if (type == "aimdk_msgs/msg/HandStateArray") {
      /** @brief 将夹爪消息交给当前来源的适配器。 @param message SDK 双夹爪反馈。 */
      const auto callback = [this, stream](aimdk_msgs::msg::HandStateArray::SharedPtr message) {
          on_hand(stream, std::move(message));
        };
      subscriptions_.push_back(create_subscription<aimdk_msgs::msg::HandStateArray>(
        topic, qos, callback));
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

void DataCollectionNode::on_arm(
  const Json & stream, aimdk_msgs::msg::JointStateArray::SharedPtr message)
{
  const auto context = receive_context();
  if (!context.active) {
    return;
  }
  try {
    accept_message(stream, arm_json(*message), serialize_raw(*message, record_raw_), context);
  } catch (const std::exception & error) {
    fail_session(error.what(), context.generation);
  }
}

void DataCollectionNode::on_hand(
  const Json & stream, aimdk_msgs::msg::HandStateArray::SharedPtr message)
{
  const auto context = receive_context();
  if (!context.active) {
    return;
  }
  try {
    accept_message(stream, hand_json(*message), serialize_raw(*message, record_raw_), context);
  } catch (const std::exception & error) {
    fail_session(error.what(), context.generation);
  }
}

void DataCollectionNode::accept_message(
  const Json & stream, const Json & message,
  std::shared_ptr<rclcpp::SerializedMessage> serialized, const ReceiveContext & context)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!recording_enabled_ || generation_ != context.generation || !writer_->is_recording()) {
    return;
  }
  mapper_->accept(stream.at("id").get<std::string>(), message, context.time_ns, context.monotonic_ns);
  if (serialized) {
    writer_->enqueue_raw(stream.at("topic").get<std::string>(),
      stream.at("message_type").get<std::string>(), std::move(serialized), context.time_ns);
  }
}

void DataCollectionNode::sample()
{
  uint64_t generation = 0;
  try {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!recording_enabled_ || !writer_->is_recording()) {
      return;
    }
    generation = generation_;
    const auto sample_ros = get_clock()->now().nanoseconds();
    const auto sample_monotonic = monotonic_now_ns();
    writer_->enqueue_frame(mapper_->frame(frame_index_++, sample_ros, sample_monotonic));
  } catch (const std::exception & error) {
    fail_session(error.what(), generation);
  }
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
    node_error_ = reason;
  }
  writer_->fail(reason);
  RCLCPP_ERROR(get_logger(), "Session input/sampling failed: %s", reason.c_str());
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
    mapper_->reset();
    frame_index_ = 0;
    node_error_.clear();
    // 开门时再次推进代号，拒绝 writer 初始化期间已进入但尚未取锁的回调。
    ++generation_;
    recording_enabled_ = true;
  }
  RCLCPP_INFO(get_logger(), "Recording started: %s", path.c_str());
}

void DataCollectionNode::stop_recording()
{
  std::lock_guard<std::mutex> control_lock(control_mutex_);
  const bool already_closed = writer_->status().at("closed").get<bool>();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    recording_enabled_ = false;
    ++generation_;
  }
  // 门控关闭之后再排空，避免在映射锁内等待磁盘 I/O。
  // 只有排空、关闭和摘要写入成功返回后才输出停止日志；异常仍交给调用者报告。
  const auto summary = writer_->stop();
  const auto path = summary.at("session_dir").get<std::string>();
  if (path.empty()) {
    RCLCPP_INFO(get_logger(), "Recording already stopped: no session has been started");
    return;
  }
  const auto frame_counts = summary.at("frames").dump();
  RCLCPP_INFO(get_logger(), "Recording %s: %s; frames=%s; data_complete=%s",
    already_closed ? "already stopped" : "stopped", path.c_str(), frame_counts.c_str(),
    summary.at("data_complete").get<bool>() ? "true" : "false");
}

Json DataCollectionNode::status()
{
  auto result = writer_->status();
  std::lock_guard<std::mutex> lock(state_mutex_);
  result["sampling_enabled"] = recording_enabled_ && writer_->is_recording();
  result["sample_rate_hz"] = sample_rate_hz_;
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
