#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>
#include <rclcpp/serialized_message.hpp>

namespace data_collection_edge_agent
{

using Json = nlohmann::json;

/** @brief 管理统一帧与可选原始 MCAP 的独立采集会话。 */
class SessionWriter
{
public:
  /** @brief 保存映射、绝对数据根目录、原始录制开关、队列容量和来源元数据；配置非法抛出异常。 */
  SessionWriter(
    const Json & mapping, const std::string & root, bool record_raw = false,
    std::size_t queue_capacity = 512, const Json & metadata = Json::object());
  /** @brief 停止接收并尝试排空、关闭当前会话，不向外抛出异常。 */
  ~SessionWriter();
  /** @brief 创建会话并返回绝对目录；录制中幂等，初始化失败抛出异常。 */
  std::string start();
  /** @brief 停止入队、排空并关闭文件，返回摘要；写入或关闭失败抛出异常。 */
  Json stop();
  /** @brief 返回当前运行状态、会话目录、计数与数据完整性摘要。 */
  Json status() const;
  /** @brief 将统一帧移入有界队列；未录制或队列已满时返回 false。 */
  bool enqueue_frame(Json frame);
  /** @brief 按话题、类型和接收时刻提交原始序列化消息；未录制、未启用或队列已满返回 false。 */
  bool enqueue_raw(
    const std::string & topic, const std::string & type,
    std::shared_ptr<rclcpp::SerializedMessage> message, int64_t receive_time_ns);
  /** @brief 返回当前是否允许接收新记录。 */
  bool is_recording() const;
  /** @brief 记录不可恢复原因并关闭接收门；不阻塞等待写盘，已有记录由 STOP 收尾。 */
  void fail(const std::string & reason);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace data_collection_edge_agent
