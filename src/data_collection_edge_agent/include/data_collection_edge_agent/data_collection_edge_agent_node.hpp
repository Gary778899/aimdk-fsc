#pragma once

#include <atomic>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <aimdk_msgs/msg/hand_state_array.hpp>
#include <aimdk_msgs/msg/joint_state_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include "data_collection_edge_agent/session_writer.hpp"
#include "data_collection_edge_agent/state_mapper.hpp"

namespace data_collection_edge_agent
{

/** @brief 独立订阅 SDK 双臂和夹爪状态，按固定频率生成统一采集帧。 */
class DataCollectionEdgeAgentNode : public rclcpp::Node
{
public:
  /** @brief 加载映射并创建订阅、采样定时器及控制端口；配置或绑定失败抛出异常。
   * @param options ROS 节点选项及参数覆盖。
   */
  explicit DataCollectionEdgeAgentNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /** @brief 停止控制线程并尝试排空当前会话，错误记录到日志。 */
  ~DataCollectionEdgeAgentNode() override;

private:
  /** @brief 保存一次回调所属会话及入口接收时间，防止跨会话写入。 */
  struct ReceiveContext
  {
    bool active{false};
    uint64_t generation{0};
    int64_t time_ns{0};
    int64_t monotonic_ns{0};
  };

  /** @brief 捕获会话与入口时间。 @return 当前回调上下文，未录制时 active 为 false。 */
  ReceiveContext receive_context();
  /** @brief 按配置创建已支持的 SDK 订阅；未知类型或 QoS 抛出异常。 */
  void create_inputs();
  /** @brief 转换双臂反馈并按需提交原始消息。
   * @param stream 来源配置。 @param message SDK 反馈。
   */
  void on_arm(const Json & stream, aimdk_msgs::msg::JointStateArray::SharedPtr message);
  /** @brief 转换双夹爪反馈并按需提交原始消息。
   * @param stream 来源配置。 @param message SDK 反馈。
   */
  void on_hand(const Json & stream, aimdk_msgs::msg::HandStateArray::SharedPtr message);
  /** @brief 在同一会话门控内更新缓存并提交原始消息。
   * @param stream 来源配置。 @param message 原字段对象。
   * @param serialized 可选原始序列化消息。 @param context 回调入口上下文。
   */
  void accept_message(
    const Json & stream, const Json & message,
    std::shared_ptr<rclcpp::SerializedMessage> serialized, const ReceiveContext & context);
  /** @brief 单次定时回调最多生成一帧，不追补错过的采样周期。 */
  void sample();
  /** @brief 锁存当前会话的适配或采样异常并停止接收。
   * @param reason 原因。 @param generation 产生异常的会话代号。
   */
  void fail_session(const std::string & reason, uint64_t generation);
  /** @brief 幂等开始会话；文件就绪后清除缓存并开放接收，失败抛出异常。 */
  void start_recording();
  /** @brief 关闭接收门后排空、关闭文件并记录最终摘要；失败抛出异常。 */
  void stop_recording();
  /** @brief 返回写入及节点采样状态。 @return STATUS 响应对象。 */
  Json status();
  /** @brief 执行有限收发等待的 ZMQ 控制循环，绑定失败传给构造函数。 */
  void zmq_server_loop();
  /** @brief 去除首尾空白并转大写。 @param command 请求。 @return 规范化命令。 */
  static std::string normalize_command(std::string command);

  Json mapping_;
  std::unique_ptr<StateMapper> mapper_;
  std::unique_ptr<SessionWriter> writer_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr sample_timer_;
  std::mutex state_mutex_;
  std::mutex control_mutex_;
  bool recording_enabled_{false};
  std::atomic<uint64_t> generation_{0};
  uint64_t frame_index_{0};
  bool record_raw_{false};
  double sample_rate_hz_{50.0};
  std::string node_error_;
  std::string zmq_bind_endpoint_;
  std::atomic<bool> shutdown_requested_{false};
  std::atomic<bool> control_ready_{false};
  std::thread zmq_thread_;
  std::promise<void> zmq_ready_;
};

}  // namespace data_collection_edge_agent
