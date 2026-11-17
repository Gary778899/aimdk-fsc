#include "data_collection/session_writer.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include <rclcpp/time.hpp>
#include <rmw/rmw.h>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

namespace data_collection
{
namespace
{

/** @brief 返回系统 UTC 时钟的整数纳秒；仅用于会话生命周期。 */
int64_t system_time_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

/** @brief 返回单调时钟纳秒，用于阶段耗时。 */
int64_t steady_time_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

/** @brief 返回当前线程累计 CPU 纳秒；Linux 线程时钟读取失败时返回零。 */
int64_t thread_cpu_ns()
{
  timespec value{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) != 0) {return 0;}
  return static_cast<int64_t>(value.tv_sec) * 1000000000LL + value.tv_nsec;
}

/** @brief 累计阶段耗时，不保留逐消息日志以减少诊断开销。 */
struct DurationStats
{
  uint64_t calls{0};
  int64_t wall_ns{0};
  int64_t cpu_ns{0};
  int64_t max_wall_ns{0};

  /** @brief 累加一次阶段的墙钟和线程 CPU 耗时，单位纳秒。 */
  void observe(int64_t wall, int64_t cpu = 0)
  {
    ++calls;
    wall_ns += wall;
    cpu_ns += cpu;
    max_wall_ns = std::max(max_wall_ns, wall);
  }

  /** @brief 返回累计次数、总耗时、平均值和最大值；CPU 为调用线程时间。 */
  Json json(bool cpu_measured = false) const
  {
    return Json{{"calls", calls}, {"total_wall_ns", wall_ns}, {"total_cpu_ns", cpu_measured ? Json(cpu_ns) : Json(nullptr)},
      {"mean_wall_ns", calls ? Json(static_cast<double>(wall_ns) / calls) : Json(nullptr)},
      {"max_wall_ns", max_wall_ns}};
  }
};

/** @brief 将 Unix 纳秒时间转为固定北京时间的目录时间字段；转换失败时抛出异常。 */
std::string session_time_label(int64_t system_ns)
{
  const auto elapsed = std::chrono::nanoseconds(system_ns);
  const auto seconds = std::chrono::floor<std::chrono::seconds>(elapsed);
  // 固定加八小时后按 UTC 格式化，避免受到容器默认时区影响。
  const auto beijing_seconds = static_cast<std::time_t>(seconds.count() + 8 * 60 * 60);
  std::tm calendar{};
  char date_time[32];
  if (gmtime_r(&beijing_seconds, &calendar) == nullptr ||
    std::strftime(date_time, sizeof(date_time), "%Y%m%d_%H%M%S", &calendar) == 0)
  {
    throw std::runtime_error("cannot format session start time");
  }
  return date_time;
}

/** @brief 将计数器初始化为提交、成功写入、丢弃均为零的对象。 */
Json empty_counts()
{
  return Json{{"submitted", 0u}, {"written", 0u}, {"dropped", 0u}};
}

/** @brief 对给定 JSON 对象的非负计数字段加一。 */
void increment(Json & object, const std::string & key)
{
  object[key] = object.at(key).get<uint64_t>() + 1u;
}

/** @brief 将 JSON 严格写入临时文件后重命名；写入、关闭或替换失败抛出异常。 */
void write_json_atomically(const std::filesystem::path & destination, const Json & value)
{
  const auto temporary = std::filesystem::path(destination.string() + ".tmp");
  std::ofstream output;
  output.exceptions(std::ios::badbit | std::ios::failbit);
  output.open(temporary, std::ios::out | std::ios::trunc);
  output << value.dump(2) << '\n';
  output.flush();
  output.close();
  std::filesystem::rename(temporary, destination);
}

}  // namespace

/** @brief 持有文件、写入线程及会话统计；生命周期与队列使用不同互斥量。 */
class SessionWriter::Impl
{
public:
  /** @brief 保存不可变来源配置并校验数据根目录、队列容量及原始话题。 */
  Impl(
    const Json & mapping, const std::string & root,
    std::size_t queue_capacity, const Json & metadata)
  : mapping_(mapping), root_(root), capacity_(queue_capacity),
    metadata_(metadata)
  {
    if (root.empty() || !root_.is_absolute()) {
      throw std::invalid_argument("data root must be an explicit absolute path");
    }
    if (capacity_ == 0) {
      throw std::invalid_argument("queue capacity must be positive");
    }
    if (!mapping_.contains("streams") || !mapping_.at("streams").is_array() ||
      mapping_.at("streams").empty() || !metadata_.is_object())
    {
      throw std::invalid_argument("mapping streams and object metadata are required");
    }
    for (const auto & stream : mapping_.at("streams")) {
      const auto id = stream.at("id").get<std::string>();
      const auto topic = stream.at("topic").get<std::string>();
      const auto type = stream.at("message_type").get<std::string>();
      if (id.empty() || topic.empty() || type.empty() || topics_.count(topic) != 0 ||
        stream_ids_.count(id) != 0)
      {
        throw std::invalid_argument("stream ids and topics must be unique and nonempty");
      }
      topics_.emplace(topic, type);
      stream_ids_.emplace(id, topic);
    }
    std::ifstream config_input(metadata_.at("mcap_storage_config").get<std::string>());
    if (!config_input) {throw std::runtime_error("cannot read MCAP storage config");}
    std::ostringstream contents;
    contents << config_input.rdbuf();
    storage_config_text_ = contents.str();
    reset_statistics();
  }

  /** @brief 创建新会话并开放入队；录制中返回原目录，失败会话保留。 */
  std::string start()
  {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (accepting_) {
        return session_dir_.string();
      }
    }
    if (worker_.joinable() || raw_writer_) {
      finish(false);
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      reset_statistics();
      session_dir_.clear();
      started_ns_ = system_time_ns();
      ended_ns_ = 0;
      state_ = "STARTING";
      last_error_.clear();
      closed_ = false;
    }
    try {
      std::filesystem::create_directories(root_);
      const auto time_label = session_time_label(started_ns_);
      std::filesystem::path selected;
      for (uint64_t index = 0; ; ++index) {
        selected = root_ / (time_label + (index == 0 ? "" : "_" + std::to_string(index)));
        if (std::filesystem::create_directory(selected)) {
          break;
        }
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        session_dir_ = selected;
      }
      {
        auto raw = std::make_unique<rosbag2_cpp::Writer>();
        rosbag2_storage::StorageOptions storage;
        storage.uri = (selected / "raw").string();
        storage.storage_id = "mcap";
        std::ofstream config_copy(selected / "mcap_storage.yaml");
        config_copy.exceptions(std::ios::badbit | std::ios::failbit);
        config_copy << storage_config_text_;
        config_copy.close();
        storage.storage_config_uri = (selected / "mcap_storage.yaml").string();
        rosbag2_cpp::ConverterOptions converter;
        converter.input_serialization_format = rmw_get_serialization_format();
        converter.output_serialization_format = rmw_get_serialization_format();
        raw->open(storage, converter);
        for (const auto & entry : topics_) {
          rosbag2_storage::TopicMetadata topic;
          topic.name = entry.first;
          topic.type = entry.second;
          topic.serialization_format = rmw_get_serialization_format();
          raw->create_topic(topic);
        }
        raw_writer_ = std::move(raw);
      }
      write_json_atomically(selected / "session.json", status());
      worker_ = std::thread(&Impl::worker_loop, this);
      std::string initialization_error;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        worker_started_ = true;
        if (last_error_.empty()) {
          state_ = "RECORDING";
          accepting_ = true;
        } else {
          initialization_error = last_error_;
        }
      }
      condition_.notify_all();
      if (!initialization_error.empty()) {
        throw std::runtime_error(initialization_error);
      }
      return selected.string();
    } catch (const std::exception & error) {
      fail(std::string("session initialization failed: ") + error.what());
      finish(false);
      throw;
    }
  }

  /** @brief 串行停止并返回最终摘要；任何会话错误均向调用者报告。 */
  Json stop()
  {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    return finish(true);
  }

  /** @brief 在状态互斥量保护下生成快照，不等待磁盘或线程结束。 */
  Json status() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    bool has_data = false;
    bool data_complete = closed_ && last_error_.empty();
    for (const auto & topic : raw_counts_.items()) {
      has_data = has_data || topic.value().at("written").get<uint64_t>() > 0;
      data_complete = data_complete && topic.value().at("dropped").get<uint64_t>() == 0 &&
        topic.value().at("written").get<uint64_t>() > 0;
    }
    Json by_topic = Json::object();
    for (const auto & entry : write_timings_) {by_topic[entry.first] = entry.second.json(true);}
    return Json{
      {"timing", {{"write_call", write_timing_.json(true)}, {"write_by_topic", by_topic},
        {"queue_wait", queue_wait_timing_.json()}, {"serialization", serialization_timing_.json()},
        {"close_call", close_timing_.json(true)}, {"stop_drain_wall_ns", stop_drain_wall_ns_}}},
      {"schema_version", 2}, {"state", state_}, {"session_dir", session_dir_.string()},
      {"started_system_time_ns", started_ns_ == 0 ? Json(nullptr) : Json(started_ns_)},
      {"ended_system_time_ns", ended_ns_ == 0 ? Json(nullptr) : Json(ended_ns_)},
      {"closed", closed_}, {"complete", closed_ && last_error_.empty()},
      {"data_complete", data_complete},
      {"no_data", !has_data},
      {"record_raw", true}, {"storage_format", "mcap"}, {"queue_capacity", capacity_},
      {"queue_pending", queue_.size()}, {"queue_peak", queue_peak_},
      {"queue_bytes", queue_bytes_}, {"queue_peak_bytes", queue_peak_bytes_},
      {"raw", raw_counts_}, {"last_error", last_error_},
      {"mapping", mapping_}, {"metadata", metadata_}};
  }

  /** @brief 提交配置话题的独占序列化样本及接收时刻；参数非法抛出异常。 */
  bool enqueue_raw(
    const std::string & topic, const std::string & type,
    std::shared_ptr<rclcpp::SerializedMessage> message, int64_t receive_time_ns, const Json & description)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!accepting_) {
      return false;
    }
    const auto found = topics_.find(topic);
    if (found == topics_.end() || found->second != type || !message) {
      throw std::invalid_argument("raw message does not match a configured topic/type");
    }
    serialization_timing_.observe(description.at("serialization_wall_ns").get<int64_t>());
    auto & counts = raw_counts_.at(topic);
    if (counts.at("submitted") == 0) {
      counts["first_receive_time_ns"] = receive_time_ns;
      counts["first_receive_monotonic_ns"] = description.at("receive_monotonic_ns");
      counts["first_source_stamp_ns"] = description.value("source_stamp_ns", Json(nullptr));
    }
    const auto previous_time = counts.value("last_receive_time_ns", receive_time_ns);
    if (receive_time_ns < previous_time) {
      counts["receive_time_backwards"] = counts.value("receive_time_backwards", uint64_t{0}) + 1;
    }
    for (const auto & field : {"source_stamp_ns", "meas_stamp_ns", "sequence"}) {
      if (!description.contains(field)) {continue;}
      const std::string key(field);
      const auto value = description.at(key).get<int64_t>();
      if (value == 0) {
        counts[key + "_zero"] = counts.value(key + "_zero", uint64_t{0}) + 1;
      }
      if (counts.contains("last_description") && counts.at("last_description").contains(key)) {
        const auto previous = counts.at("last_description").at(key).get<int64_t>();
        if (value == previous) {
          counts[key + "_repeated"] = counts.value(key + "_repeated", uint64_t{0}) + 1;
        } else if (value < previous) {
          counts[key + "_backwards"] = counts.value(key + "_backwards", uint64_t{0}) + 1;
        }
      }
    }
    counts["last_receive_time_ns"] = receive_time_ns;
    counts["last_source_stamp_ns"] = description.value("source_stamp_ns", Json(nullptr));
    counts["last_description"] = description;
    const auto duration_ns = description.at("receive_monotonic_ns").get<int64_t>() -
      counts.at("first_receive_monotonic_ns").get<int64_t>();
    counts["receive_frequency_hz"] = duration_ns > 0 ?
      Json(counts.at("submitted").get<uint64_t>() * 1000000000.0 / duration_ns) : Json(nullptr);
    const auto size = message->size();
    counts["received_bytes"] = counts.value("received_bytes", uint64_t{0}) + size;
    Record record;
    record.bytes = size;
    record.topic = topic;
    record.type = type;
    record.message = std::move(message);
    record.receive_time_ns = receive_time_ns;
    return enqueue_locked(std::move(record), raw_counts_.at(topic));
  }

  /** @brief 返回接收门状态；错误或 STOP 后立即为 false。 */
  bool is_recording() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return accepting_;
  }

  /** @brief 保留首次不可恢复原因，关闭接收门并通知写盘线程排空。 */
  void fail(const std::string & reason)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    fail_locked(reason);
  }

private:
  /** @brief 单条原始序列化消息及接收时间。 */
  struct Record
  {
    std::size_t bytes{0};
    std::string topic;
    std::string type;
    std::shared_ptr<rclcpp::SerializedMessage> message;
    int64_t receive_time_ns{0};
    int64_t enqueue_steady_ns{0};
  };

  /** @brief 统一执行容量检查、入队及计数；调用方持状态锁且已校验接收门和记录类型。
   * @param record 待写记录。 @param counts 对应原始话题的计数。
   * @return 成功入队为 true；队列满则丢弃新记录并计数，返回 false。
   */
  bool enqueue_locked(Record record, Json & counts)
  {
    increment(counts, "submitted");
    if (queue_.size() >= capacity_) {
      increment(counts, "dropped");
      return false;
    }
    record.enqueue_steady_ns = steady_time_ns();
    queue_bytes_ += record.bytes;
    queue_peak_bytes_ = std::max(queue_peak_bytes_, queue_bytes_);
    queue_.push_back(std::move(record));
    queue_peak_ = std::max(queue_peak_, queue_.size());
    condition_.notify_one();
    return true;
  }

  /** @brief 锁存首次错误、关闭接收门并唤醒写盘线程；调用方持状态锁。
   * @param reason 不可恢复错误，空值使用通用采集错误描述。
   */
  void fail_locked(const std::string & reason)
  {
    set_error_locked(reason.empty() ? "unspecified collection failure" : reason);
    accepting_ = false;
    condition_.notify_all();
  }

  /** @brief 清空当前会话计数，并预登记全部预期流以识别从未到达的来源。 */
  void reset_statistics()
  {
    raw_counts_ = Json::object();
    queue_.clear();
    queue_peak_ = 0;
    queue_bytes_ = 0;
    queue_peak_bytes_ = 0;
    worker_started_ = false;
    write_timing_ = {};
    queue_wait_timing_ = {};
    serialization_timing_ = {};
    close_timing_ = {};
    stop_drain_wall_ns_ = 0;
    write_timings_.clear();
    for (const auto & topic : topics_) {
      write_timings_[topic.first] = {};
      raw_counts_[topic.first] = empty_counts();
    }
  }

  /** @brief 在调用方已持有状态锁时记录首次错误。 */
  void set_error_locked(const std::string & reason)
  {
    if (last_error_.empty()) {
      last_error_ = reason;
    }
    state_ = "ERROR";
  }

  /** @brief 对已接收但无法写入的原始消息计入丢弃；调用方持状态锁。 */
  void count_discard_locked(const Record & record)
  {
    increment(raw_counts_.at(record.topic), "dropped");
  }

  /** @brief 单线程写盘；STOP 后排空，写入失败则锁存错误并丢弃剩余队列。 */
  void worker_loop()
  {
    while (true) {
      Record record;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        while (queue_.empty() && (!worker_started_ || accepting_)) {
          condition_.wait(lock);
        }
        if (queue_.empty()) {
          return;
        }
        record = std::move(queue_.front());
        queue_.pop_front();
        queue_bytes_ -= record.bytes;
        queue_wait_timing_.observe(steady_time_ns() - record.enqueue_steady_ns);
      }
      bool written = false;
      try {
        const auto wall_begin = steady_time_ns();
        const auto cpu_begin = thread_cpu_ns();
        raw_writer_->write(
          record.message, record.topic, record.type,
          rclcpp::Time(record.receive_time_ns, RCL_ROS_TIME));
        const auto cpu_elapsed = thread_cpu_ns() - cpu_begin;
        const auto wall_elapsed = steady_time_ns() - wall_begin;
        written = true;
        std::lock_guard<std::mutex> lock(mutex_);
        write_timing_.observe(wall_elapsed, cpu_elapsed);
        write_timings_.at(record.topic).observe(wall_elapsed, cpu_elapsed);
        increment(raw_counts_.at(record.topic), "written");
      } catch (const std::exception & error) {
        std::lock_guard<std::mutex> lock(mutex_);
        fail_locked(std::string("recording write failed: ") + error.what());
        if (!written) {
          count_discard_locked(record);
        }
        for (const auto & pending : queue_) {
          count_discard_locked(pending);
        }
        queue_.clear();
        queue_bytes_ = 0;
        return;
      }
    }
  }

  /** @brief 调用方持生命周期锁时排空和关闭；report_error 控制是否向调用者抛出会话错误。 */
  Json finish(bool report_error)
  {
    // 幂等 STOP 不重写已完成的摘要，避免后续状态查询扰动持久化结果。
    if (!worker_.joinable() && !raw_writer_) {
      const Json previous = status();
      if (previous.at("closed").get<bool>() ||
        previous.at("session_dir").get<std::string>().empty())
      {
        if (report_error && !previous.at("last_error").get<std::string>().empty()) {
          throw std::runtime_error(previous.at("last_error").get<std::string>());
        }
        return previous;
      }
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      accepting_ = false;
      worker_started_ = true;
      if (state_ != "ERROR" && !session_dir_.empty() && !closed_) {
        state_ = "STOPPING";
      }
    }
    condition_.notify_all();
    const auto drain_begin = steady_time_ns();
    if (worker_.joinable()) {
      worker_.join();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_drain_wall_ns_ = steady_time_ns() - drain_begin;
    }
    bool close_success = true;
    if (raw_writer_) {
      try {
        const auto wall_begin = steady_time_ns();
        const auto cpu_begin = thread_cpu_ns();
        raw_writer_->close();
        const auto cpu_elapsed = thread_cpu_ns() - cpu_begin;
        const auto wall_elapsed = steady_time_ns() - wall_begin;
        std::lock_guard<std::mutex> lock(mutex_);
        close_timing_.observe(wall_elapsed, cpu_elapsed);
      } catch (const std::exception & error) {
        close_success = false;
        fail(std::string("MCAP close failed: ") + error.what());
      }
      raw_writer_.reset();
    }
    std::filesystem::path directory;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      directory = session_dir_;
      if (!directory.empty() && !closed_) {
        ended_ns_ = system_time_ns();
        closed_ = close_success;
        state_ = last_error_.empty() ? "READY" : "ERROR";
      }
    }
    if (!directory.empty()) {
      try {
        write_json_atomically(directory / "session.json", status());
      } catch (const std::exception & error) {
        fail(std::string("session summary write failed: ") + error.what());
      }
    }
    Json summary = status();
    if (report_error && !summary.at("last_error").get<std::string>().empty()) {
      throw std::runtime_error(summary.at("last_error").get<std::string>());
    }
    return summary;
  }

  Json mapping_;
  std::filesystem::path root_;
  std::size_t capacity_;
  Json metadata_;
  std::string storage_config_text_;
  std::map<std::string, std::string> topics_;
  std::map<std::string, std::string> stream_ids_;
  mutable std::mutex mutex_;
  std::mutex lifecycle_mutex_;
  std::condition_variable condition_;
  std::deque<Record> queue_;
  std::thread worker_;
  std::unique_ptr<rosbag2_cpp::Writer> raw_writer_;
  std::filesystem::path session_dir_;
  bool accepting_{false};
  bool worker_started_{false};
  bool closed_{false};
  std::string state_{"READY"};
  std::string last_error_;
  int64_t started_ns_{0};
  int64_t ended_ns_{0};
  Json raw_counts_;
  DurationStats write_timing_;
  DurationStats queue_wait_timing_;
  DurationStats serialization_timing_;
  DurationStats close_timing_;
  std::map<std::string, DurationStats> write_timings_;
  int64_t stop_drain_wall_ns_{0};
  std::size_t queue_peak_{0};
  std::size_t queue_bytes_{0};
  std::size_t queue_peak_bytes_{0};
};

/** @brief 构造内部存储管理器；参数含义同公开声明，非法参数抛出异常。 */
SessionWriter::SessionWriter(
  const Json & mapping, const std::string & root,
  std::size_t queue_capacity, const Json & metadata)
: impl_(std::make_unique<Impl>(mapping, root, queue_capacity, metadata))
{
}

/** @brief 析构时尽力完成排空关闭，保留已记录错误而不抛出异常。 */
SessionWriter::~SessionWriter()
{
  try {
    impl_->stop();
  } catch (const std::exception &) {
    // 停止失败已在状态和可写入的会话摘要中记录。
  }
}

/** @brief 开始会话并返回会话目录；失败抛出异常。 */
std::string SessionWriter::start()
{
  return impl_->start();
}

/** @brief 停止会话并返回最终摘要；失败抛出异常。 */
Json SessionWriter::stop()
{
  return impl_->stop();
}

/** @brief 返回当前运行状态与统计快照。 */
Json SessionWriter::status() const
{
  return impl_->status();
}

/** @brief 提交原始样本并返回是否成功入队；参数错误抛出异常。 */
bool SessionWriter::enqueue_raw(
  const std::string & topic, const std::string & type,
  std::shared_ptr<rclcpp::SerializedMessage> message, int64_t receive_time_ns, const Json & description)
{
  return impl_->enqueue_raw(topic, type, std::move(message), receive_time_ns, description);
}

/** @brief 返回当前是否开放接收门。 */
bool SessionWriter::is_recording() const
{
  return impl_->is_recording();
}

/** @brief 非阻塞记录不可恢复原因并关闭接收门。 */
void SessionWriter::fail(const std::string & reason)
{
  impl_->fail(reason);
}

}  // namespace data_collection
