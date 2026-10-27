#include "data_collection/session_writer.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <unistd.h>

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
  /** @brief 保存不可变配置并校验数据根目录、队列容量及原始话题。 */
  Impl(
    const Json & mapping, const std::string & root, bool record_raw,
    std::size_t queue_capacity, const Json & metadata)
  : mapping_(mapping), root_(root), record_raw_(record_raw), capacity_(queue_capacity),
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
    if (worker_.joinable() || frames_.is_open() || raw_writer_) {
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
      static std::atomic<uint64_t> sequence{0};
      std::filesystem::path selected;
      for (std::size_t attempt = 0; attempt != 100; ++attempt) {
        selected = root_ / ("collection_" + std::to_string(started_ns_) + "_" +
          std::to_string(getpid()) + "_" + std::to_string(sequence.fetch_add(1)));
        if (std::filesystem::create_directory(selected)) {
          break;
        }
        selected.clear();
      }
      if (selected.empty()) {
        throw std::runtime_error("cannot allocate a unique session directory");
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        session_dir_ = selected;
      }
      frames_.exceptions(std::ios::badbit | std::ios::failbit);
      frames_.open(selected / "frames.jsonl", std::ios::out | std::ios::trunc);
      if (record_raw_) {
        auto raw = std::make_unique<rosbag2_cpp::Writer>();
        rosbag2_storage::StorageOptions storage;
        storage.uri = (selected / "raw").string();
        storage.storage_id = "mcap";
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
    bool data_complete = closed_ && last_error_.empty() &&
      frame_counts_.at("written").get<uint64_t>() > 0 &&
      frame_counts_.at("dropped").get<uint64_t>() == 0 && invalid_frames_ == 0;
    for (const auto & stream : streams_.items()) {
      data_complete = data_complete && stream.value().at("seen").get<bool>() &&
        stream.value().at("missing_frames").get<uint64_t>() == 0 &&
        stream.value().at("observed_quality_issues").empty();
    }
    for (const auto & topic : raw_counts_.items()) {
      data_complete = data_complete && topic.value().at("dropped").get<uint64_t>() == 0;
    }
    return Json{
      {"schema_version", 1}, {"state", state_}, {"session_dir", session_dir_.string()},
      {"started_system_time_ns", started_ns_ == 0 ? Json(nullptr) : Json(started_ns_)},
      {"ended_system_time_ns", ended_ns_ == 0 ? Json(nullptr) : Json(ended_ns_)},
      {"closed", closed_}, {"complete", closed_ && last_error_.empty()},
      {"data_complete", data_complete},
      {"no_data", frame_counts_.at("written").get<uint64_t>() == 0},
      {"record_raw", record_raw_}, {"queue_capacity", capacity_},
      {"queue_pending", queue_.size()}, {"queue_peak", queue_peak_},
      {"frames", frame_counts_}, {"raw", raw_counts_}, {"streams", streams_},
      {"invalid_frames", invalid_frames_}, {"last_error", last_error_},
      {"mapping", mapping_}, {"metadata", metadata_}};
  }

  /** @brief 提交统一帧；队列满时丢弃新帧并计数，回调不执行磁盘操作。 */
  bool enqueue_frame(Json frame)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!accepting_) {
      return false;
    }
    increment(frame_counts_, "submitted");
    if (queue_.size() >= capacity_) {
      increment(frame_counts_, "dropped");
      return false;
    }
    Record record;
    record.frame = std::move(frame);
    queue_.push_back(std::move(record));
    queue_peak_ = std::max(queue_peak_, queue_.size());
    condition_.notify_one();
    return true;
  }

  /** @brief 提交配置话题的独占序列化样本及接收时刻；参数非法抛出异常。 */
  bool enqueue_raw(
    const std::string & topic, const std::string & type,
    std::shared_ptr<rclcpp::SerializedMessage> message, int64_t receive_time_ns)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!accepting_ || !record_raw_) {
      return false;
    }
    const auto found = topics_.find(topic);
    if (found == topics_.end() || found->second != type || !message) {
      throw std::invalid_argument("raw message does not match a configured topic/type");
    }
    auto & counts = raw_counts_.at(topic);
    increment(counts, "submitted");
    if (queue_.size() >= capacity_) {
      increment(counts, "dropped");
      return false;
    }
    Record record;
    record.raw = true;
    record.topic = topic;
    record.type = type;
    record.message = std::move(message);
    record.receive_time_ns = receive_time_ns;
    queue_.push_back(std::move(record));
    queue_peak_ = std::max(queue_peak_, queue_.size());
    condition_.notify_one();
    return true;
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
    set_error_locked(reason.empty() ? "unspecified collection failure" : reason);
    accepting_ = false;
    condition_.notify_all();
  }

private:
  /** @brief 单条队列记录；统一帧与序列化消息由 raw 字段区分。 */
  struct Record
  {
    bool raw{false};
    Json frame;
    std::string topic;
    std::string type;
    std::shared_ptr<rclcpp::SerializedMessage> message;
    int64_t receive_time_ns{0};
  };

  /** @brief 清空当前会话计数，并预登记全部预期流以识别从未到达的来源。 */
  void reset_statistics()
  {
    frame_counts_ = empty_counts();
    raw_counts_ = Json::object();
    streams_ = Json::object();
    queue_.clear();
    queue_peak_ = 0;
    invalid_frames_ = 0;
    worker_started_ = false;
    for (const auto & stream : stream_ids_) {
      streams_[stream.first] = Json{
        {"seen", false}, {"missing_frames", 0u}, {"reused_frames", 0u},
        {"issue_frames", 0u}, {"quality_issue_frames", 0u},
        {"observed_quality_issues", Json::array()}};
    }
    if (record_raw_) {
      for (const auto & topic : topics_) {
        raw_counts_[topic.first] = empty_counts();
      }
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

  /** @brief 统计成功写入帧中的缺流、重复、异常和字段有效性；调用方持状态锁。 */
  void observe_frame_locked(const Json & frame)
  {
    bool invalid = false;
    const auto stream_values = frame.value("streams", Json::object());
    for (auto & stream : streams_.items()) {
      auto & counts = stream.value();
      if (!stream_values.contains(stream.key()) ||
        !stream_values.at(stream.key()).value("received", false))
      {
        increment(counts, "missing_frames");
        invalid = true;
        continue;
      }
      const auto & source = stream_values.at(stream.key());
      counts["seen"] = true;
      if (source.value("reused", false)) {
        increment(counts, "reused_frames");
      }
      if (source.contains("issues") && !source.at("issues").empty()) {
        increment(counts, "issue_frames");
      }
      bool quality_issue = false;
      for (const auto & issue : source.value("issues", Json::array())) {
        quality_issue = quality_issue || issue != "cached_sample_reused";
      }
      if (quality_issue) {
        increment(counts, "quality_issue_frames");
      }
      // 累计集合保留被丢弃采样帧中的源异常，但不将后续正常帧反复计为异常帧。
      for (const auto & field : {"issues", "history_issues"}) {
        for (const auto & issue : source.value(field, Json::array())) {
          auto & history = counts.at("observed_quality_issues");
          if (issue != "cached_sample_reused" &&
            std::find(history.begin(), history.end(), issue) == history.end())
          {
            history.push_back(issue);
          }
        }
      }
    }
    for (const std::string side : {"left", "right"}) {
      const auto & arm = frame.at("arms").at(side);
      for (const auto & valid : arm.at("valid")) {
        invalid = invalid || !valid.get<bool>();
      }
      invalid = invalid || !frame.at("grippers").at(side).at("valid").get<bool>();
    }
    if (invalid) {
      ++invalid_frames_;
    }
  }

  /** @brief 对已接收但无法写入的记录计入丢弃；调用方持状态锁。 */
  void count_discard_locked(const Record & record)
  {
    if (record.raw) {
      increment(raw_counts_.at(record.topic), "dropped");
    } else {
      increment(frame_counts_, "dropped");
    }
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
      }
      bool written = false;
      try {
        if (record.raw) {
          raw_writer_->write(
            record.message, record.topic, record.type,
            rclcpp::Time(record.receive_time_ns, RCL_ROS_TIME));
        } else {
          frames_ << record.frame.dump() << '\n';
        }
        written = true;
        std::lock_guard<std::mutex> lock(mutex_);
        if (record.raw) {
          increment(raw_counts_.at(record.topic), "written");
        } else {
          increment(frame_counts_, "written");
          observe_frame_locked(record.frame);
        }
      } catch (const std::exception & error) {
        std::lock_guard<std::mutex> lock(mutex_);
        set_error_locked(std::string("recording write failed: ") + error.what());
        accepting_ = false;
        if (!written) {
          count_discard_locked(record);
        }
        for (const auto & pending : queue_) {
          count_discard_locked(pending);
        }
        queue_.clear();
        return;
      }
    }
  }

  /** @brief 调用方持生命周期锁时排空和关闭；report_error 控制是否向调用者抛出会话错误。 */
  Json finish(bool report_error)
  {
    // 幂等 STOP 不重写已完成的摘要，避免后续状态查询扰动持久化结果。
    if (!worker_.joinable() && !frames_.is_open() && !raw_writer_) {
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
    if (worker_.joinable()) {
      worker_.join();
    }
    bool close_success = true;
    if (frames_.is_open()) {
      try {
        frames_.flush();
        frames_.close();
      } catch (const std::exception & error) {
        close_success = false;
        fail(std::string("frame file close failed: ") + error.what());
        // 失败的流仍须关闭句柄，但不将二次无异常关闭解释为数据成功。
        frames_.exceptions(std::ios::goodbit);
        frames_.close();
      }
    }
    frames_.clear();
    if (raw_writer_) {
      try {
        raw_writer_->close();
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
  bool record_raw_;
  std::size_t capacity_;
  Json metadata_;
  std::map<std::string, std::string> topics_;
  std::map<std::string, std::string> stream_ids_;
  mutable std::mutex mutex_;
  std::mutex lifecycle_mutex_;
  std::condition_variable condition_;
  std::deque<Record> queue_;
  std::thread worker_;
  std::ofstream frames_;
  std::unique_ptr<rosbag2_cpp::Writer> raw_writer_;
  std::filesystem::path session_dir_;
  bool accepting_{false};
  bool worker_started_{false};
  bool closed_{false};
  std::string state_{"READY"};
  std::string last_error_;
  int64_t started_ns_{0};
  int64_t ended_ns_{0};
  Json frame_counts_;
  Json raw_counts_;
  Json streams_;
  std::size_t queue_peak_{0};
  uint64_t invalid_frames_{0};
};

/** @brief 构造内部存储管理器；参数含义同公开声明，非法参数抛出异常。 */
SessionWriter::SessionWriter(
  const Json & mapping, const std::string & root, bool record_raw,
  std::size_t queue_capacity, const Json & metadata)
: impl_(std::make_unique<Impl>(mapping, root, record_raw, queue_capacity, metadata))
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

/** @brief 提交统一帧并返回是否成功入队。 */
bool SessionWriter::enqueue_frame(Json frame)
{
  return impl_->enqueue_frame(std::move(frame));
}

/** @brief 提交原始样本并返回是否成功入队；参数错误抛出异常。 */
bool SessionWriter::enqueue_raw(
  const std::string & topic, const std::string & type,
  std::shared_ptr<rclcpp::SerializedMessage> message, int64_t receive_time_ns)
{
  return impl_->enqueue_raw(topic, type, std::move(message), receive_time_ns);
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
