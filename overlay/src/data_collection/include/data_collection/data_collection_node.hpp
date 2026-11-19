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

#include "data_collection/session_writer.hpp"

namespace data_collection
{

/** @brief 只读订阅所选机器人数据源，按来源频率保存 MCAP。 */
class DataCollectionNode : public rclcpp::Node
{
public:
  /** @brief 加载来源配置并创建订阅及控制端口；配置或绑定失败抛出异常。
   * @param options ROS 节点选项及参数覆盖。
   */
  explicit DataCollectionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /** @brief 停止控制线程并尝试排空当前会话，错误记录到日志。 */
  ~DataCollectionNode() override;

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
  /** @brief 返回接收门边界附近的一组机器人时钟，含采样的单调时间上下界。 */
  Json clock_anchor();
  /** @brief 按配置创建已支持的 SDK 订阅；未知类型或 QoS 抛出异常。 */
  void create_inputs();
  /** @brief 锁存当前会话的输入适配异常并停止接收。
   * @param reason 原因。 @param generation 产生异常的会话代号。
   */
  void fail_session(const std::string & reason, uint64_t generation);
  /** @brief 幂等开始会话；文件就绪后清除缓存并开放接收，失败抛出异常。 */
  void start_recording();
  /** @brief 关闭接收门后排空、关闭文件并记录最终摘要；失败抛出异常。 */
  void stop_recording();
  /** @brief 返回写入及节点录制状态。 @return STATUS 响应对象。 */
  Json status();
  /** @brief 执行有限收发等待的 ZMQ 控制循环，绑定失败传给构造函数。 */
  void zmq_server_loop();
  /** @brief 去除首尾空白并转大写。 @param command 请求。 @return 规范化命令。 */
  static std::string normalize_command(std::string command);

  /** @brief 按来源类型接收反馈、指令和传感器消息，保留源时间与图像描述并提交 MCAP。 */
  template<typename MessageT>
  void on_sensor(const Json & stream, std::shared_ptr<MessageT> message);

  /** @brief 提交原始消息及描述；保持会话代号门控。 */
  void accept_raw(const Json & stream, std::shared_ptr<rclcpp::SerializedMessage> message,
    const ReceiveContext & context, const Json & description);

  Json mapping_;
  std::unique_ptr<SessionWriter> writer_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::mutex state_mutex_;
  std::mutex control_mutex_;
  bool recording_enabled_{false};
  std::atomic<uint64_t> generation_{0};
  std::string node_error_;
  std::string zmq_bind_endpoint_;
  std::atomic<bool> shutdown_requested_{false};
  std::atomic<bool> control_ready_{false};
  std::thread zmq_thread_;
  std::promise<void> zmq_ready_;
};

}  // namespace data_collection
