#pragma once

#include <cstdint>
#include <map>
#include <string>

#include <nlohmann/json.hpp>

namespace data_collection
{

using Json = nlohmann::json;

/** @brief 根据冻结的 JSON 配置整理双臂和夹爪反馈；调用方负责并发互斥。 */
class StateMapper
{
public:
  /** @brief 校验并保存配置；配置不符合双臂七轴契约时抛出 std::invalid_argument。
   * @param config 含来源、关节名称、字段路径及数值转换的映射配置。
   */
  explicit StateMapper(const Json & config);

  /** @brief 更新一个来源的缓存并记录时间异常，不因设备故障丢弃反馈。
   * @param stream_id 配置中的来源标识。
   * @param message 由输入适配器转换的原始消息对象。
   * @param receive_time_ns 本地接收 ROS 时间，整数纳秒。
   * @param receive_monotonic_ns 本地接收单调时间，整数纳秒。
   * @throws std::invalid_argument 来源未配置或消息不是对象。
   */
  void accept(
    const std::string & stream_id, const Json & message,
    int64_t receive_time_ns, int64_t receive_monotonic_ns);

  /** @brief 组装一帧固定顺序状态；缺失或非有限字段输出 null 并附质量说明。
   * @param frame_index 会话内帧序号。
   * @param sample_time_ns 当前采样 ROS 时间。
   * @param sample_monotonic_ns 当前采样单调时间，仅用于计算接收年龄。
   * @return 含双臂、双夹爪及各来源时间的统一 JSON 帧。
   */
  Json frame(uint64_t frame_index, int64_t sample_time_ns, int64_t sample_monotonic_ns);

  /** @brief 清除所有缓存、来源序号和复用记录，用于开始新会话。 */
  void reset();

  /** @brief 返回构造时通过校验的映射配置。 @return 冻结配置的只读引用。 */
  const Json & config() const;

private:
  /** @brief 保存单个来源的最新反馈及两帧之间发生的时间异常。 */
  struct Cache
  {
    Json message;
    int64_t receive_time_ns{0};
    int64_t receive_monotonic_ns{0};
    uint64_t sample_id{0};
    uint64_t previous_frame_sample_id{0};
    Json stamp_ns;
    Json meas_stamp_ns;
    Json sequence;
    Json pending_issues = Json::array();
    Json history_issues = Json::array();
    bool received{false};
  };

  Json config_;
  std::map<std::string, Json> streams_;
  std::map<std::string, Cache> caches_;
};

}  // namespace data_collection
