#include "data_collection/state_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace data_collection
{
namespace
{

/** @brief 读取点号分隔的对象路径。 @param value 根对象。 @param path 字段路径。
 * @return 存在时返回字段指针，否则返回 nullptr；不执行表达式或数组下标代码。
 */
const Json * lookup(const Json & value, const std::string & path)
{
  const Json * current = &value;
  std::size_t start = 0;
  while (start < path.size()) {
    const auto end = path.find('.', start);
    const auto key = path.substr(start, end == std::string::npos ? end : end - start);
    if (!current->is_object() || !current->contains(key)) {
      return nullptr;
    }
    current = &current->at(key);
    if (end == std::string::npos) {
      return current;
    }
    start = end + 1;
  }
  return nullptr;
}

/** @brief 向说明数组加入不重复的问题。 @param issues 目标数组。 @param issue 问题标识。 */
void add_issue(Json & issues, const std::string & issue)
{
  if (std::find(issues.begin(), issues.end(), Json(issue)) == issues.end()) {
    issues.push_back(issue);
  }
}

/** @brief 检查配置条件。 @param condition 条件。 @param message 失败原因。
 * @throws std::invalid_argument 配置条件不成立。
 */
void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::invalid_argument("mapping: " + message);
  }
}

/** @brief 判断字符串是否为非空、分段完整的点路径。 @param path 待检查值。 @return 是否有效。 */
bool valid_path(const Json & path)
{
  if (!path.is_string()) {
    return false;
  }
  const auto text = path.get<std::string>();
  return !text.empty() && text.front() != '.' && text.back() != '.' &&
         text.find("..") == std::string::npos;
}

/** @brief 判断值是否为有限数值。 @param value JSON 值。 @return 是否可用于数值转换。 */
bool finite_number(const Json & value)
{
  return value.is_number() && std::isfinite(value.get<double>());
}

/** @brief 校验一个字段的映射规则。 @param field 映射对象。 @param label 错误位置。
 * @throws std::invalid_argument 路径、比例、偏置或单位无效。
 */
void validate_field(const Json & field, const std::string & label)
{
  require(field.is_object(), label + " must be an object");
  require(field.contains("path") && valid_path(field.at("path")), label + ".path invalid");
  require(field.contains("scale") && finite_number(field.at("scale")) &&
    field.at("scale").get<double>() != 0.0, label + ".scale must be finite and nonzero");
  require(field.contains("offset") && finite_number(field.at("offset")), label + ".offset invalid");
  require(field.contains("unit") && field.at("unit").is_string() &&
    !field.at("unit").get<std::string>().empty(), label + ".unit missing");
}

/** @brief 校验统一字段的单位及原值保留规则。
 * @param field 输出字段名。 @param rule 完整规则。 @param arm 是否属于手臂。
 * @throws std::invalid_argument 单位或原值转换不符合数据契约。
 */
void validate_semantics(const std::string & field, const Json & rule, bool arm)
{
  if (arm && field == "position") {
    require(rule.at("unit") == "rad", "arm position unit must be rad");
  } else if (arm && field == "velocity") {
    require(rule.at("unit") == "rad/s", "arm velocity unit must be rad/s");
  } else if (!arm && field == "opening") {
    require(rule.at("unit") == "normalized_opening", "opening unit must be normalized_opening");
  }
  if (field == "effort_raw" || field == "velocity_raw" || field == "error_code" ||
    field == "state" || field == "faultcode")
  {
    require(rule.at("scale") == 1 && rule.at("offset") == 0,
      field + " must preserve the original value");
  }
}

/** @brief 查找某关节的字段覆盖规则，未覆盖时使用组默认。
 * @param group 手臂组配置。 @param joint 统一关节名。 @param field 输出字段名。
 * @return 构造时已校验的完整字段规则引用。
 */
const Json & joint_rule(const Json & group, const std::string & joint, const std::string & field)
{
  if (group.contains("joint_fields") && group.at("joint_fields").contains(joint) &&
    group.at("joint_fields").at(joint).contains(field))
  {
    return group.at("joint_fields").at(joint).at(field);
  }
  return group.at("fields").at(field);
}

/** @brief 校验来源选择及规定字段集合。 @param group 组配置。 @param fields 必需输出字段。
 * @param streams 已声明来源。 @param label 配置位置。
 * @throws std::invalid_argument 来源、数组或字段规则无效。
 */
void validate_group(
  const Json & group, const std::vector<std::string> & fields,
  const std::map<std::string, Json> & streams, const std::string & label)
{
  require(group.is_object(), label + " must be an object");
  require(group.contains("stream") && group.at("stream").is_string() &&
    streams.count(group.at("stream").get<std::string>()) != 0, label + ".stream unknown");
  require(group.contains("array") && valid_path(group.at("array")), label + ".array invalid");
  require(group.contains("name_field") && valid_path(group.at("name_field")),
    label + ".name_field invalid");
  require(group.contains("fields") && group.at("fields").is_object(), label + ".fields missing");
  for (const auto & field : fields) {
    require(group.at("fields").contains(field), label + ".fields." + field + " missing");
    validate_field(group.at("fields").at(field), label + ".fields." + field);
  }
}

/** @brief 将 ROS 风格秒与纳秒对象转换为有符号整数纳秒，不替换无效时间。
 * @param message 来源对象。 @param path 时间对象路径。 @param issues 问题数组。
 * @param label 时间字段标识。 @return 有效整数纳秒或 null。
 */
Json timestamp_ns(
  const Json & message, const std::string & path, Json & issues, const std::string & label)
{
  const auto * stamp = lookup(message, path);
  if (!stamp || !stamp->is_object() || !stamp->contains("sec") ||
    !stamp->contains("nanosec") || !stamp->at("sec").is_number_integer() ||
    !stamp->at("nanosec").is_number_integer())
  {
    add_issue(issues, label + "_missing_or_invalid");
    return nullptr;
  }
  if ((stamp->at("sec").is_number_unsigned() &&
    stamp->at("sec").get<uint64_t>() > static_cast<uint64_t>(std::numeric_limits<int32_t>::max())) ||
    (stamp->at("nanosec").is_number_unsigned() &&
    stamp->at("nanosec").get<uint64_t>() >= 1000000000ULL))
  {
    add_issue(issues, label + "_invalid");
    return nullptr;
  }
  const auto sec = stamp->at("sec").get<int64_t>();
  const auto ns = stamp->at("nanosec").get<int64_t>();
  // SDK 的 sec 是 int32；限制范围以避免迁移适配器提供异常大整数造成溢出。
  if (sec < std::numeric_limits<int32_t>::min() ||
    sec > std::numeric_limits<int32_t>::max() || ns < 0 || ns >= 1000000000LL)
  {
    add_issue(issues, label + "_invalid");
    return nullptr;
  }
  const int64_t result = sec * 1000000000LL + ns;
  if (result <= 0) {
    add_issue(issues, label + (result == 0 ? "_zero" : "_negative"));
  }
  return result;
}

/** @brief 按名字选择数组元素，重复命名时拒绝歧义映射但保留问题说明。
 * @param message 来源对象。 @param group 数组与名称字段规则。 @param name 来源名称。
 * @param issues 问题数组。 @return 唯一匹配项或 nullptr。
 */
const Json * select_item(
  const Json & message, const Json & group, const std::string & name, Json & issues)
{
  const auto * array = lookup(message, group.at("array").get<std::string>());
  if (!array || !array->is_array()) {
    add_issue(issues, "array_missing_or_invalid");
    return nullptr;
  }
  const Json * selected = nullptr;
  for (const auto & item : *array) {
    const auto * observed = lookup(item, group.at("name_field").get<std::string>());
    if (observed && observed->is_string() && observed->get<std::string>() == name) {
      if (selected) {
        add_issue(issues, name + ":duplicate_name");
        return nullptr;
      }
      selected = &item;
    }
  }
  if (!selected) {
    add_issue(issues, name + ":missing");
  }
  return selected;
}

/** @brief 读取并转换一个数值字段；原始整数字段保持整数表示。
 * @param item 匹配项或 nullptr。 @param rule 字段规则。 @param integer 是否要求原始整数。
 * @param label 问题位置。 @param issues 问题数组。 @return 转换数值或 null。
 */
Json mapped_number(
  const Json * item, const Json & rule, bool integer, const std::string & label, Json & issues)
{
  const auto * value = item ? lookup(*item, rule.at("path").get<std::string>()) : nullptr;
  if (!value || !finite_number(*value) || (integer && !value->is_number_integer())) {
    add_issue(issues, label + ":missing_or_nonfinite");
    return nullptr;
  }
  if (integer || (rule.at("scale") == 1 && rule.at("offset") == 0)) {
    return *value;
  }
  const double transformed = value->get<double>() * rule.at("scale").get<double>() +
    rule.at("offset").get<double>();
  if (!std::isfinite(transformed)) {
    add_issue(issues, label + ":conversion_nonfinite");
    return nullptr;
  }
  return transformed;
}

/** @brief 返回配置字段或 null。 @param message 来源对象。 @param path 路径。
 * @return 原始 JSON 值副本；缺失时返回 null。
 */
Json raw_field(const Json & message, const std::string & path)
{
  const auto * value = lookup(message, path);
  return value ? *value : Json(nullptr);
}

}  // namespace

/** @brief 校验并冻结配置；参数与异常契约见头文件。 */
StateMapper::StateMapper(const Json & config)
: config_(config)
{
  require(config_.is_object(), "root must be an object");
  require(config_.contains("schema_version") && config_.at("schema_version") == 1,
    "schema_version must be 1");
  require(config_.contains("platform") && config_.at("platform").is_string() &&
    !config_.at("platform").get<std::string>().empty(), "platform missing");
  require(config_.contains("sample_rate_hz") && finite_number(config_.at("sample_rate_hz")) &&
    config_.at("sample_rate_hz").get<double>() > 0.0, "sample_rate_hz must be positive");
  const std::vector<std::string> expected_order = {
    "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_yaw", "wrist_pitch", "wrist_roll"};
  require(config_.contains("joint_order") && config_.at("joint_order") == Json(expected_order),
    "joint_order must match the seven canonical joint names");
  require(config_.contains("streams") && config_.at("streams").is_array() &&
    !config_.at("streams").empty(), "streams must be a nonempty array");
  std::set<std::string> topics;
  for (const auto & stream : config_.at("streams")) {
    require(stream.is_object(), "stream must be an object");
    for (const auto & key : {"id", "topic", "message_type"}) {
      require(stream.contains(key) && stream.at(key).is_string() &&
        !stream.at(key).get<std::string>().empty(), std::string("stream.") + key + " missing");
    }
    const auto id = stream.at("id").get<std::string>();
    require(streams_.count(id) == 0, "duplicate stream id " + id);
    require(topics.insert(stream.at("topic").get<std::string>()).second, "duplicate source topic");
    require(stream.contains("qos") && stream.at("qos").is_object(), id + ".qos missing");
    const auto & qos = stream.at("qos");
    require(qos.contains("reliability") &&
      (qos.at("reliability") == "best_effort" || qos.at("reliability") == "reliable"),
      id + ".qos.reliability invalid");
    require(qos.contains("durability") &&
      (qos.at("durability") == "volatile" || qos.at("durability") == "transient_local"),
      id + ".qos.durability invalid");
    require(qos.contains("depth") && qos.at("depth").is_number_integer() &&
      qos.at("depth").get<int64_t>() > 0, id + ".qos.depth must be positive integer");
    require(stream.contains("timestamps") && stream.at("timestamps").is_object(),
      id + ".timestamps missing");
    for (const auto & key : {"stamp", "meas_stamp", "sequence"}) {
      require(stream.at("timestamps").contains(key) && valid_path(stream.at("timestamps").at(key)),
        id + ".timestamps." + key + " invalid");
    }
    if (stream.contains("state_path")) {
      require(valid_path(stream.at("state_path")), id + ".state_path invalid");
    }
    if (stream.contains("frame_id_path")) {
      require(valid_path(stream.at("frame_id_path")), id + ".frame_id_path invalid");
    }
    streams_.emplace(id, stream);
  }
  require(config_.contains("arms") && config_.at("arms").is_object(), "arms missing");
  require(config_.contains("grippers") && config_.at("grippers").is_object(), "grippers missing");
  std::set<std::pair<std::string, std::string>> selected_names;
  std::set<std::pair<std::string, std::string>> selected_grippers;
  for (const auto & side : {"left", "right"}) {
    require(config_.at("arms").contains(side), std::string("arms.") + side + " missing");
    const auto & arm = config_.at("arms").at(side);
    validate_group(arm, {"position", "velocity", "effort_raw", "error_code"}, streams_,
      std::string("arms.") + side);
    require(arm.contains("names") && arm.at("names").is_array() && arm.at("names").size() == 7,
      std::string("arms.") + side + ".names must contain seven entries");
    for (const auto & name : arm.at("names")) {
      require(name.is_string() && !name.get<std::string>().empty(), "arm source name invalid");
      require(selected_names.emplace(arm.at("stream").get<std::string>() + ":" +
        arm.at("array").get<std::string>(), name.get<std::string>()).second,
        "arm source selected more than once");
    }
    const std::set<std::string> arm_fields = {"position", "velocity", "effort_raw", "error_code"};
    for (const auto & field : arm_fields) {
      validate_semantics(field, arm.at("fields").at(field), true);
    }
    if (arm.contains("joint_fields")) {
      require(arm.at("joint_fields").is_object(), "joint_fields must be an object");
      for (auto it = arm.at("joint_fields").begin(); it != arm.at("joint_fields").end(); ++it) {
        require(std::find(expected_order.begin(), expected_order.end(), it.key()) != expected_order.end(),
          "unknown canonical joint in joint_fields: " + it.key());
        require(it.value().is_object(), "joint_fields entries must be objects");
        for (auto field = it.value().begin(); field != it.value().end(); ++field) {
          require(arm_fields.count(field.key()) != 0, "unknown joint field: " + field.key());
          validate_field(field.value(), "joint_fields." + it.key() + "." + field.key());
          validate_semantics(field.key(), field.value(), true);
        }
      }
    }
    require(config_.at("grippers").contains(side), std::string("grippers.") + side + " missing");
    const auto & gripper = config_.at("grippers").at(side);
    validate_group(gripper, {"opening", "velocity_raw", "effort_raw", "state", "faultcode"},
      streams_, std::string("grippers.") + side);
    require(gripper.contains("name") && gripper.at("name").is_string() &&
      !gripper.at("name").get<std::string>().empty(), "gripper.name missing");
    require(selected_grippers.emplace(gripper.at("stream").get<std::string>() + ":" +
      gripper.at("array").get<std::string>(), gripper.at("name").get<std::string>()).second,
      "gripper source selected more than once");
    require(gripper.contains("type_path") && valid_path(gripper.at("type_path")),
      "gripper.type_path invalid");
    require(gripper.contains("expected_type") && gripper.at("expected_type").is_number_integer(),
      "gripper.expected_type must be integer");
    for (const auto & field : {"opening", "velocity_raw", "effort_raw", "state", "faultcode"}) {
      validate_semantics(field, gripper.at("fields").at(field), false);
    }
  }
  reset();
}

/** @brief 接收指定来源的原始对象并记录时间推进情况；参数与异常契约见头文件。 */
void StateMapper::accept(
  const std::string & stream_id, const Json & message,
  int64_t receive_time_ns, int64_t receive_monotonic_ns)
{
  const auto stream = streams_.find(stream_id);
  if (stream == streams_.end() || !message.is_object()) {
    throw std::invalid_argument("unknown stream or non-object feedback: " + stream_id);
  }
  auto & cache = caches_.at(stream_id);
  const auto & times = stream->second.at("timestamps");
  const auto stamp = timestamp_ns(message, times.at("stamp").get<std::string>(),
    cache.pending_issues, "stamp");
  const auto meas_stamp = timestamp_ns(message, times.at("meas_stamp").get<std::string>(),
    cache.pending_issues, "meas_stamp");
  auto sequence = raw_field(message, times.at("sequence").get<std::string>());
  if (!sequence.is_number_integer()) {
    sequence = nullptr;
    add_issue(cache.pending_issues, "sequence_missing_or_invalid");
  }
  if (cache.received) {
    if (stamp.is_number_integer() && cache.stamp_ns.is_number_integer()) {
      if (stamp == cache.stamp_ns) {
        add_issue(cache.pending_issues, "stamp_repeated");
      } else if (stamp.get<int64_t>() < cache.stamp_ns.get<int64_t>()) {
        add_issue(cache.pending_issues, "stamp_moved_backwards");
      }
    }
    if (meas_stamp.is_number_integer() && cache.meas_stamp_ns.is_number_integer()) {
      if (meas_stamp == cache.meas_stamp_ns) {
        add_issue(cache.pending_issues, "meas_stamp_repeated");
      } else if (meas_stamp.get<int64_t>() < cache.meas_stamp_ns.get<int64_t>()) {
        add_issue(cache.pending_issues, "meas_stamp_moved_backwards");
      }
    }
    if (!sequence.is_null() && sequence == cache.sequence) {
      add_issue(cache.pending_issues, "sequence_repeated");
    }
    if (receive_time_ns < cache.receive_time_ns) {
      add_issue(cache.pending_issues, "receive_ros_time_moved_backwards");
    }
    if (receive_monotonic_ns < cache.receive_monotonic_ns) {
      add_issue(cache.pending_issues, "receive_monotonic_time_moved_backwards");
    }
  }
  cache.message = message;
  cache.stamp_ns = stamp;
  cache.meas_stamp_ns = meas_stamp;
  cache.sequence = sequence;
  cache.receive_time_ns = receive_time_ns;
  cache.receive_monotonic_ns = receive_monotonic_ns;
  for (const auto & issue : cache.pending_issues) {
    add_issue(cache.history_issues, issue.get<std::string>());
  }
  ++cache.sample_id;
  cache.received = true;
}

/** @brief 按固定双臂顺序生成当前采样帧并更新复用记录；参数与返回契约见头文件。 */
Json StateMapper::frame(
  uint64_t frame_index, int64_t sample_time_ns, int64_t sample_monotonic_ns)
{
  Json result = {{"schema_version", 1}, {"frame_index", frame_index},
    {"sample_time_ns", sample_time_ns}, {"sample_monotonic_ns", sample_monotonic_ns},
    {"streams", Json::object()}, {"arms", Json::object()}, {"grippers", Json::object()}};
  for (auto & pair : caches_) {
    const auto & id = pair.first;
    auto & cache = pair.second;
    Json issues = cache.pending_issues;
    const bool reused = cache.received && cache.previous_frame_sample_id == cache.sample_id;
    if (!cache.received) {
      add_issue(issues, "not_received");
    } else if (reused) {
      add_issue(issues, "cached_sample_reused");
    }
    Json metadata = {{"received", cache.received},
      {"receive_time_ns", cache.received ? Json(cache.receive_time_ns) : Json(nullptr)},
      {"receive_monotonic_ns", cache.received ? Json(cache.receive_monotonic_ns) : Json(nullptr)},
      {"sample_id", cache.received ? Json(cache.sample_id) : Json(nullptr)},
      {"stamp_ns", cache.stamp_ns}, {"meas_stamp_ns", cache.meas_stamp_ns},
      {"sequence", cache.sequence}, {"reused", reused},
      {"age_ns", cache.received ? Json(sample_monotonic_ns - cache.receive_monotonic_ns) : Json(nullptr)}};
    if (cache.received && sample_monotonic_ns < cache.receive_monotonic_ns) {
      add_issue(issues, "sample_precedes_receive_monotonic_time");
    }
    const auto & stream = streams_.at(id);
    metadata["frame_id"] = nullptr;
    if (cache.received && stream.contains("frame_id_path")) {
      const auto value = raw_field(cache.message, stream.at("frame_id_path").get<std::string>());
      if (value.is_string()) {
        metadata["frame_id"] = value;
      } else {
        add_issue(issues, "frame_id_missing_or_invalid");
      }
    }
    if (stream.contains("state_path")) {
      metadata["source_state"] = cache.received ?
        raw_field(cache.message, stream.at("state_path").get<std::string>()) : Json(nullptr);
      if (cache.received && !metadata.at("source_state").is_number_integer()) {
        add_issue(issues, "source_state_missing_or_invalid");
      } else if (metadata.at("source_state").is_number_integer() && metadata.at("source_state") != 0) {
        add_issue(issues, "source_state_nonzero");
      }
    }
    metadata["issues"] = std::move(issues);
    metadata["history_issues"] = cache.history_issues;
    result["streams"][id] = std::move(metadata);
    cache.previous_frame_sample_id = cache.sample_id;
    cache.pending_issues = Json::array();
  }
  for (const auto & side : {"left", "right"}) {
    const auto & group = config_.at("arms").at(side);
    const auto & cache = caches_.at(group.at("stream").get<std::string>());
    Json arm = {{"joint_names", config_.at("joint_order")}, {"position", Json::array()},
      {"velocity", Json::array()}, {"effort_raw", Json::array()}, {"error_code", Json::array()},
      {"valid", Json::array()}, {"issues", Json::array()}};
    std::size_t joint_index = 0;
    for (const auto & name : group.at("names")) {
      Json joint_issues = Json::array();
      const auto source_name = name.get<std::string>();
      const auto * item = cache.received ? select_item(cache.message, group, source_name, joint_issues) : nullptr;
      if (!cache.received) {
        add_issue(joint_issues, source_name + ":not_received");
      }
      bool valid = item != nullptr;
      for (const auto & field : {"position", "velocity", "effort_raw", "error_code"}) {
        const auto canonical_name = config_.at("joint_order").at(joint_index).get<std::string>();
        const auto value = mapped_number(item, joint_rule(group, canonical_name, field),
          std::string(field) == "error_code", source_name + ":" + field, joint_issues);
        arm[field].push_back(value);
        valid = valid && !value.is_null();
        if (std::string(field) == "error_code" && !value.is_null() && value != 0) {
          valid = false;
          add_issue(joint_issues, source_name + ":error_code_nonzero");
        }
      }
      const auto & stream_meta = result.at("streams").at(group.at("stream").get<std::string>());
      if (stream_meta.contains("source_state")) {
        if (!stream_meta.at("source_state").is_number_integer()) {
          valid = false;
          add_issue(joint_issues, source_name + ":source_state_missing_or_invalid");
        } else if (stream_meta.at("source_state") != 0) {
          valid = false;
          add_issue(joint_issues, source_name + ":source_state_nonzero");
        }
      }
      arm["valid"].push_back(valid);
      for (const auto & issue : joint_issues) {
        add_issue(arm["issues"], issue.get<std::string>());
      }
      ++joint_index;
    }
    result["arms"][side] = std::move(arm);

    const auto & hand_group = config_.at("grippers").at(side);
    const auto & hand_cache = caches_.at(hand_group.at("stream").get<std::string>());
    Json hand = {{"issues", Json::array()}};
    const auto * item = hand_cache.received ? select_item(hand_cache.message, hand_group,
      hand_group.at("name").get<std::string>(), hand["issues"]) : nullptr;
    bool valid = item != nullptr;
    if (!hand_cache.received) {
      add_issue(hand["issues"], "not_received");
    }
    for (const auto & field : {"opening", "velocity_raw", "effort_raw", "state", "faultcode"}) {
      const bool integer = std::string(field) == "state" || std::string(field) == "faultcode";
      hand[field] = mapped_number(item, hand_group.at("fields").at(field), integer, field, hand["issues"]);
      valid = valid && !hand.at(field).is_null();
    }
    hand["type"] = hand_cache.received ? raw_field(hand_cache.message,
      hand_group.at("type_path").get<std::string>()) : Json(nullptr);
    if (!hand.at("type").is_number_integer() || hand.at("type") != hand_group.at("expected_type")) {
      valid = false;
      add_issue(hand["issues"], "unexpected_hand_type");
    }
    if (!hand.at("opening").is_null() &&
      (hand.at("opening").get<double>() < 0.0 || hand.at("opening").get<double>() > 1.0))
    {
      valid = false;
      add_issue(hand["issues"], "opening_out_of_range");
    }
    if (!hand.at("faultcode").is_null() && hand.at("faultcode") != 0) {
      valid = false;
      add_issue(hand["issues"], "faultcode_nonzero");
    }
    hand["valid"] = valid;
    result["grippers"][side] = std::move(hand);
  }
  return result;
}

/** @brief 重置会话缓存，不改变已校验映射配置。 */
void StateMapper::reset()
{
  caches_.clear();
  for (const auto & stream : streams_) {
    caches_.emplace(stream.first, Cache{});
  }
}

/** @brief 返回冻结配置的只读引用，不进行复制。 */
const Json & StateMapper::config() const
{
  return config_;
}

}  // namespace data_collection
