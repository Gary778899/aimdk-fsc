#!/usr/bin/env python3
"""检查 MCAP 会话摘要，可选在 ROS 环境读回消息；兼容历史 JSONL 会话。"""

import argparse
import base64
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import sys


XR_TOPIC = "/remote_control/xr/raw"
XR_SCHEMA = "remote_control.xr_raw.v1"
XR_EVENT_FIELDS = (
    "sdk_connection_id", "callback_seq", "received_system_time_ns", "received_monotonic_ns",
    "received_monotonic_s", "kind", "callback_type", "status", "device_id", "payload_base64", "error",
)


def require(condition, message):
    """条件不满足时抛出带定位信息的 ValueError，用于拒绝损坏的采集文件。"""
    if not condition:
        raise ValueError(message)


def reject_constant(value):
    """拒绝 JSON 标准以外的 NaN、Infinity 常量；始终抛出 ValueError。"""
    raise ValueError(f"JSON 含非标准数值常量：{value}")


def finite_float(value):
    """解析 JSON 小数并拒绝溢出产生的非有限值，返回有限浮点数。"""
    result = float(value)
    require(math.isfinite(result), f"JSON 含非有限小数：{value}")
    return result


def unique_object(pairs):
    """由 JSON 键值对构造对象，遇到重复键抛出 ValueError。"""
    result = {}
    for key, value in pairs:
        require(key not in result, f"JSON 存在重复键：{key}")
        result[key] = value
    return result


def parse_json(text):
    """严格解析 JSON 文本，拒绝重复键、非标准常量和浮点溢出。"""
    return json.loads(
        text, parse_constant=reject_constant, parse_float=finite_float,
        object_pairs_hook=unique_object,
    )


def integer(value, location, nullable=False):
    """校验整数纳秒、序号或代码；nullable 允许 null，布尔值始终不视为整数。"""
    require((nullable and value is None) or type(value) is int, f"{location} 必须为整数")
    return value


def number(value, location, nullable=True):
    """校验有限数值；缺失数据可为 null，布尔值与非有限数值均被拒绝。"""
    if nullable and value is None:
        return value
    require(type(value) in (int, float) and math.isfinite(value), f"{location} 必须为有限数值")
    return value


def issues(value, location):
    """校验问题标识数组，返回原数组；格式错误抛出 ValueError。"""
    require(isinstance(value, list) and all(isinstance(item, str) for item in value),
            f"{location} 必须为字符串数组")
    return value


def compare_summary(condition, message, finalized, mismatches):
    """记录摘要与读回的差异；已收尾摘要矛盾时抛出异常，过时快照仅报告差异。"""
    if not condition:
        mismatches.append(message)
        require(not finalized, message)


def validate_frame(frame, mapping, location):
    """按映射校验一帧的双臂、夹爪、来源及整数时间；返回是否含无效数据。"""
    require(isinstance(frame, dict), f"{location} 必须为对象")
    require(type(frame["schema_version"]) is int and frame["schema_version"] == 1,
            f"{location}.schema_version 不受支持")
    require(integer(frame["frame_index"], location + ".frame_index") >= 0,
            f"{location}.frame_index 不得为负")
    integer(frame["sample_time_ns"], location + ".sample_time_ns")
    integer(frame["sample_monotonic_ns"], location + ".sample_monotonic_ns")
    require(set(frame["arms"]) == {"left", "right"}, f"{location}.arms 必须包含左右臂")
    require(set(frame["grippers"]) == {"left", "right"}, f"{location}.grippers 必须包含左右夹爪")
    invalid = False
    for side in ("left", "right"):
        arm = frame["arms"][side]
        prefix = f"{location}.arms.{side}"
        require(arm["joint_names"] == mapping["joint_order"], f"{prefix} 关节顺序与映射不一致")
        for field in ("position", "velocity", "effort_raw", "error_code", "valid"):
            values = arm[field]
            require(isinstance(values, list) and len(values) == 7, f"{prefix}.{field} 长度必须为 7")
            for index, value in enumerate(values):
                item_location = f"{prefix}.{field}[{index}]"
                if field == "valid":
                    require(type(value) is bool, f"{item_location} 必须为布尔值")
                    invalid = invalid or not value
                elif field == "error_code":
                    integer(value, item_location, nullable=True)
                else:
                    number(value, item_location)
        issues(arm["issues"], prefix + ".issues")
        for index, valid in enumerate(arm["valid"]):
            if valid:
                require(all(arm[field][index] is not None for field in
                            ("position", "velocity", "effort_raw", "error_code")),
                        f"{prefix}[{index}] 标为有效但含 null")
                require(arm["error_code"][index] == 0,
                        f"{prefix}[{index}] 标为有效但错误码非零")

        hand = frame["grippers"][side]
        prefix = f"{location}.grippers.{side}"
        require(type(hand["valid"]) is bool, f"{prefix}.valid 必须为布尔值")
        invalid = invalid or not hand["valid"]
        for field in ("opening", "velocity_raw", "effort_raw"):
            number(hand[field], prefix + "." + field)
        for field in ("type", "state", "faultcode"):
            integer(hand[field], prefix + "." + field, nullable=True)
        issues(hand["issues"], prefix + ".issues")
        if hand["valid"]:
            require(all(hand[field] is not None for field in
                        ("opening", "velocity_raw", "effort_raw", "type", "state", "faultcode")),
                    f"{prefix} 标为有效但含 null")
            require(0 <= hand["opening"] <= 1 and hand["faultcode"] == 0 and
                    hand["type"] == mapping["grippers"][side]["expected_type"],
                    f"{prefix} 有效标志与开度、故障或手型矛盾")

    expected_streams = {entry["id"] for entry in mapping["streams"]}
    require(set(frame["streams"]) == expected_streams, f"{location} 来源集合与映射不一致")
    for stream_id, stream in frame["streams"].items():
        prefix = f"{location}.streams.{stream_id}"
        for field in ("received", "reused"):
            require(type(stream[field]) is bool, f"{prefix}.{field} 必须为布尔值")
        if "frame_id" in stream:
            require(stream["frame_id"] is None or isinstance(stream["frame_id"], str),
                    f"{prefix}.frame_id 必须为字符串或 null")
        for field in ("receive_time_ns", "receive_monotonic_ns", "sample_id", "stamp_ns",
                      "meas_stamp_ns", "sequence", "age_ns"):
            integer(stream[field], prefix + "." + field, nullable=True)
        issues(stream["issues"], prefix + ".issues")
        issues(stream["history_issues"], prefix + ".history_issues")
        if stream["received"]:
            require(all(stream[field] is not None for field in
                        ("receive_time_ns", "receive_monotonic_ns", "sample_id", "age_ns")),
                    f"{prefix} 已接收但缺少接收时间或样本序号")
            require(stream["age_ns"] == frame["sample_monotonic_ns"] - stream["receive_monotonic_ns"],
                    f"{prefix}.age_ns 与单调接收时间不一致")
        else:
            invalid = True
            require(not stream["reused"], f"{prefix} 从未收到却标为复用")
    return invalid


def inspect_session(directory):
    """逐行读回会话文件并核对摘要；返回检查报告，损坏文件抛出异常。"""
    session = parse_json((directory / "session.json").read_text(encoding="utf-8"))
    if session.get("schema_version") == 2:
        return inspect_mcap_summary(directory, session)
    require(type(session["schema_version"]) is int and session["schema_version"] == 1,
            "session.schema_version 不受支持")
    mapping = session["mapping"]
    order = mapping["joint_order"]
    require(isinstance(order, list) and len(order) == 7 and len(set(order)) == 7 and
            all(isinstance(name, str) and name for name in order), "映射必须声明 7 个唯一关节标识")
    require(type(mapping["schema_version"]) is int and mapping["schema_version"] == 1,
            "mapping.schema_version 不受支持")
    expected_ids = [entry["id"] for entry in mapping["streams"]]
    require(expected_ids and len(set(expected_ids)) == len(expected_ids), "映射来源标识不得为空或重复")
    source_stats = {stream_id: {
        "seen": False, "missing_frames": 0, "reused_frames": 0, "issue_frames": 0,
        "quality_issue_frames": 0, "received_new_callback_frames": 0, "source_stamp_repeated_frames": 0,
        "age_min_ns": None, "age_max_ns": None, "age_sum_ns": 0,
        "observed_quality_issues": [],
    } for stream_id in expected_ids}
    previous_sources = {}
    source_issue_counts = {stream_id: Counter() for stream_id in expected_ids}
    faults = Counter()
    count = 0
    invalid_frames = 0
    previous_index = None
    previous_mono = None
    first_mono = None
    interval_min = None
    interval_max = None
    missing_frame_indices = 0
    with (directory / "frames.jsonl").open(encoding="utf-8") as input_file:
        for line_number, line in enumerate(input_file, start=1):
            location = f"frames.jsonl:{line_number}"
            require(line.endswith("\n"), f"{location} 缺少结束换行，可能未完整写入")
            try:
                frame = parse_json(line)
                invalid_frames += validate_frame(frame, mapping, location)
            except (ValueError, KeyError, TypeError, OverflowError) as error:
                raise ValueError(f"{location}: {error}") from error
            index = frame["frame_index"]
            mono = frame["sample_monotonic_ns"]
            if previous_index is not None:
                require(index > previous_index, f"{location} frame_index 未严格推进")
                require(mono > previous_mono, f"{location} 单调采样时间未严格推进")
                missing_frame_indices += index - previous_index - 1
                delta = mono - previous_mono
                interval_min = delta if interval_min is None else min(interval_min, delta)
                interval_max = delta if interval_max is None else max(interval_max, delta)
            else:
                first_mono = mono
            consecutive_frame = previous_index is not None and index == previous_index + 1
            previous_index, previous_mono = index, mono
            for stream_id, stream in frame["streams"].items():
                stats = source_stats[stream_id]
                if not stream["received"]:
                    stats["missing_frames"] += 1
                    continue
                stats["seen"] = True
                stats["reused_frames"] += stream["reused"]
                stats["received_new_callback_frames"] += not stream["reused"]
                stats["issue_frames"] += bool(stream["issues"])
                stats["quality_issue_frames"] += any(
                    issue != "cached_sample_reused" for issue in stream["issues"]
                )
                for issue in stream["issues"] + stream["history_issues"]:
                    if issue != "cached_sample_reused" and issue not in stats["observed_quality_issues"]:
                        stats["observed_quality_issues"].append(issue)
                source_issue_counts[stream_id].update(stream["issues"])
                age = stream["age_ns"]
                stats["age_sum_ns"] += age
                stats["age_min_ns"] = age if stats["age_min_ns"] is None else min(stats["age_min_ns"], age)
                stats["age_max_ns"] = age if stats["age_max_ns"] is None else max(stats["age_max_ns"], age)
                previous = previous_sources.get(stream_id)
                if previous is not None:
                    require(stream["sample_id"] >= previous["sample_id"],
                            f"{location}.{stream_id} 本地样本序号回退")
                    if consecutive_frame or stream["sample_id"] == previous["sample_id"]:
                        require(stream["reused"] == (stream["sample_id"] == previous["sample_id"]),
                                f"{location}.{stream_id} reused 与样本序号矛盾")
                    if not stream["reused"] and stream["stamp_ns"] is not None:
                        stats["source_stamp_repeated_frames"] += stream["stamp_ns"] == previous["stamp_ns"]
                previous_sources[stream_id] = stream
            for side in ("left", "right"):
                faults[f"{side}_arm_fault_frames"] += any(
                    value not in (None, 0) for value in frame["arms"][side]["error_code"]
                )
                faults[f"{side}_gripper_fault_frames"] += frame["grippers"][side]["faultcode"] not in (None, 0)
            count += 1

    for field in ("closed", "complete", "data_complete", "no_data", "record_raw"):
        require(type(session[field]) is bool, f"session.{field} 必须为布尔值")
    for field in ("started_system_time_ns", "ended_system_time_ns"):
        integer(session[field], "session." + field, nullable=True)
    summary_finalized = session["ended_system_time_ns"] is not None
    metadata_mismatches = []
    frames = session["frames"]
    for field in ("submitted", "written", "dropped"):
        require(integer(frames[field], "session.frames." + field) >= 0, "帧计数不得为负")
    compare_summary(frames["written"] == count,
                    "frames.jsonl 帧数与 session.frames.written 不一致",
                    summary_finalized, metadata_mismatches)
    compare_summary(integer(session["invalid_frames"], "session.invalid_frames") == invalid_frames,
                    "无效帧数与 session.invalid_frames 不一致",
                    summary_finalized, metadata_mismatches)
    compare_summary(session["no_data"] == (count == 0), "session.no_data 与实际帧数矛盾",
                    summary_finalized, metadata_mismatches)
    if summary_finalized:
        require(frames["submitted"] == frames["written"] + frames["dropped"],
                "已收尾会话的帧提交、写入和丢弃计数不守恒")
    require(set(session["streams"]) == set(expected_ids), "会话来源统计集合与映射不一致")
    for stream_id, stats in source_stats.items():
        saved = session["streams"][stream_id]
        for field in ("seen", "missing_frames", "reused_frames", "issue_frames", "quality_issue_frames"):
            if field == "seen":
                require(type(saved[field]) is bool, f"session.streams.{stream_id}.seen 必须为布尔值")
            else:
                integer(saved[field], f"session.streams.{stream_id}.{field}")
            compare_summary(saved[field] == stats[field],
                            f"来源 {stream_id} 的 {field} 统计与读回不一致",
                            summary_finalized, metadata_mismatches)
        issues(saved["observed_quality_issues"], f"session.streams.{stream_id}.observed_quality_issues")
        compare_summary(saved["observed_quality_issues"] == stats["observed_quality_issues"],
                        f"来源 {stream_id} 累计异常集合与读回不一致",
                        summary_finalized, metadata_mismatches)
        received_frames = count - stats["missing_frames"]
        stats["age_mean_ns"] = stats.pop("age_sum_ns") / received_frames if received_frames else None
        stats["issues"] = dict(source_issue_counts[stream_id])

    raw_dropped = 0
    expected_raw = {entry["topic"] for entry in mapping["streams"]} if session["record_raw"] else set()
    require(set(session["raw"]) == expected_raw, "原始话题统计集合与 record_raw／映射不一致")
    for topic, counters in session["raw"].items():
        for field in ("submitted", "written", "dropped"):
            require(integer(counters[field], f"session.raw.{topic}.{field}") >= 0, "原始计数不得为负")
        if summary_finalized:
            require(counters["submitted"] == counters["written"] + counters["dropped"],
                    f"原始话题 {topic} 计数不守恒")
        raw_dropped += counters["dropped"]
    require(isinstance(session["last_error"], str), "session.last_error 必须为字符串")
    complete = summary_finalized and session["closed"] and not session["last_error"]
    compare_summary(session["complete"] == complete,
                    "session.complete 与收尾／关闭状态／错误矛盾",
                    summary_finalized, metadata_mismatches)
    computed_data_complete = (
        complete and count > 0 and frames["dropped"] == 0 and raw_dropped == 0 and
        invalid_frames == 0 and all(stats["seen"] and stats["missing_frames"] == 0 and
                                    not stats["observed_quality_issues"] for stats in source_stats.values())
    )
    compare_summary(session["data_complete"] == computed_data_complete,
                    "session.data_complete 与收尾状态、读回质量、缺流或丢弃计数矛盾",
                    summary_finalized, metadata_mismatches)
    elapsed = previous_mono - first_mono if count >= 2 else None
    return {
        "inspection_scope": "frames_jsonl_and_session_metadata",
        "session_dir": str(directory.resolve()), "structure_valid": True,
        "summary_finalized": summary_finalized,
        "metadata_matches_readback": not metadata_mismatches,
        "metadata_mismatches": metadata_mismatches,
        "summary_frame_counts": frames,
        "closed": summary_finalized and session["closed"], "complete": complete,
        "data_complete": computed_data_complete, "no_data": count == 0,
        "frames": count, "invalid_frames": invalid_frames,
        "dropped_frames": frames["dropped"] if summary_finalized else None,
        "missing_frame_indices_between_written": missing_frame_indices,
        "duration_ns": elapsed,
        "output_frequency_hz": (count - 1) * 1_000_000_000 / elapsed if elapsed else None,
        "target_frequency_hz": mapping["sample_rate_hz"],
        "interval_ns": {"min": interval_min, "max": interval_max,
                        "mean": elapsed / (count - 1) if elapsed else None},
        "streams": source_stats, "faults": dict(faults), "last_error": session["last_error"],
        "raw_mcap_enabled": session["record_raw"],
        "raw_mcap_verified": False if session["record_raw"] else None,
    }


def inspect_mcap_summary(directory, session):
    """用标准库核对新版 MCAP 会话摘要；不读取 MCAP 消息内容，返回检查报告。"""
    expected = {entry["topic"] for entry in session["mapping"]["streams"]}
    require(expected and set(session["raw"]) == expected, "话题统计与所选来源不一致")
    finalized = session["ended_system_time_ns"] is not None
    written = 0
    dropped = 0
    for topic, counts in session["raw"].items():
        for field in ("submitted", "written", "dropped"):
            require(integer(counts[field], f"raw.{topic}.{field}") >= 0, "消息计数不得为负")
        if finalized:
            require(counts["submitted"] == counts["written"] + counts["dropped"],
                    f"{topic} 消息计数不守恒")
        written += counts["written"]
        dropped += counts["dropped"]
    complete = finalized and session["closed"] and not session["last_error"]
    data_complete = complete and dropped == 0 and all(
        counts["written"] > 0 for counts in session["raw"].values())
    require(session["no_data"] == (written == 0), "no_data 与写入消息数不一致")
    require(session["complete"] == complete, "complete 与会话收尾状态不一致")
    require(session["data_complete"] == data_complete, "data_complete 与来源统计不一致")
    return dict(inspection_scope="session_metadata", session_dir=str(directory.resolve()),
                structure_valid=True, closed=session["closed"], complete=complete,
                data_complete=data_complete, no_data=written == 0, messages=written,
                dropped=dropped, raw=session["raw"], raw_mcap_enabled=True,
                raw_mcap_verified=False, last_error=session["last_error"])


def validate_xr_event(event):
    """校验原始 XR 消息信封与时间字段，返回事件；载荷内容在读回阶段统计。"""
    require(isinstance(event, dict) and event.get("schema") == XR_SCHEMA, "XR 原始事件格式不受支持")
    require(isinstance(event["sdk_connection_id"], str) and event["sdk_connection_id"], "XR 缺少 SDK 连接标识")
    require(integer(event["callback_seq"], "XR callback_seq") > 0, "XR callback_seq 必须为正")
    require(type(event["drained"]) is bool, "XR drained 必须为布尔值")
    for field in ("received_system_time_ns", "received_monotonic_ns", "callback_type", "status"):
        integer(event[field], f"XR {field}")
    number(event["received_monotonic_s"], "XR received_monotonic_s", nullable=False)
    for field in ("kind", "device_id", "payload_base64"):
        require(isinstance(event[field], str), f"XR {field} 必须为字符串")
    for field, value in (("error", event["error"]), ("session_id", event.get("session_id"))):
        require(value is None or isinstance(value, str), f"XR {field} 必须为字符串或 null")
    return event


def parse_xr_event(text):
    """从 String 话题的 JSON 文本读取原始 XR 信封；格式错误抛出 ValueError 或 KeyError。"""
    return validate_xr_event(parse_json(text))


def xr_event_digest(event):
    """返回包含载荷、时间、排空阶段和会话标识的事件摘要，用于比较 MCAP 与 XR JSONL。"""
    values = [event[field] for field in (*XR_EVENT_FIELDS, "drained")] + [event.get("session_id")]
    return hashlib.sha256(json.dumps(values, ensure_ascii=True).encode("ascii")).digest()


def read_xr_reference(path):
    """读取同次 XR JSONL 的元数据及按连接和序号索引的事件摘要；损坏数据带行号报错。"""
    metadata, reference = None, {}
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                row = parse_json(line)
                require(isinstance(row, dict) and row.get("schema") == "remote_control.xr.v1",
                        "XR JSONL 格式不受支持")
                if line_number == 1:
                    require(row["event"] == "metadata", "XR JSONL 首行必须是元数据")
                    metadata = row["fields"]
                    require(isinstance(metadata, dict), "XR JSONL 元数据必须为对象")
                if row["event"] != "sdk_event":
                    continue
                fields = row["fields"]
                event = validate_xr_event(dict(fields["raw"], schema=XR_SCHEMA, drained=fields["drained"],
                                               session_id=metadata.get("session_id")))
                key = (event["sdk_connection_id"], event["callback_seq"])
                require(key not in reference, "XR JSONL 存在重复事件标识")
                reference[key] = xr_event_digest(event)
            except (ValueError, KeyError, TypeError) as error:
                raise ValueError(f"{path}:{line_number}: {error}") from error
    require(isinstance(metadata, dict), f"{path}: 缺少 XR JSONL 元数据")
    return metadata, reference


class XrReadback:
    """核对原始 XR 信封和事件覆盖，对指定设备按原输入协议验证双手帧。"""

    def __init__(self, device_id, reference_path, input_config_path):
        """读取可选对账日志和输入协议，显式设备或配置与同次日志冲突时抛出 ValueError。"""
        metadata, self.reference = ({}, None) if reference_path is None else read_xr_reference(reference_path)
        logged_device = metadata.get("device_id")
        require(device_id is None or logged_device in (None, device_id), "指定的 XR 设备与 JSONL 元数据不一致")
        self.device_id = device_id or logged_device
        configuration = metadata.get("input_config")
        if input_config_path is not None:
            require(self.device_id, "使用 XR 输入配置时须指定设备或提供含设备标识的 XR JSONL")
            explicit = parse_json(input_config_path.read_text(encoding="utf-8"))
            require(configuration is None or configuration == explicit, "XR 输入配置与 JSONL 元数据不一致")
            configuration = explicit
        self.config = None
        if configuration is not None and self.device_id:
            from remote_control.protocol import InputConfig

            self.config = InputConfig.from_dict(configuration)
        self.counts = Counter()
        self.kinds = Counter()
        self.sequences = defaultdict(Counter)
        self.devices = defaultdict(Counter)
        self.digests = {}

    def observe(self, text):
        """检查一条 MCAP XR 消息，累计事件覆盖、载荷可解码性与设备帧计数。"""
        event = parse_xr_event(text)
        connection, seq = event["sdk_connection_id"], event["callback_seq"]
        self.sequences[connection][seq] += 1
        if self.reference is not None:
            self.digests[connection, seq] = xr_event_digest(event)
        drained = event["drained"]
        kind = event["kind"]
        self.kinds[kind] += 1
        self.counts["events"] += 1
        self.counts["drained_events"] += drained
        self.counts["callback_errors"] += event["error"] is not None
        if kind in ("server_disconnected", "device_missing") and not drained:
            self.counts["active_disconnect_events"] += (kind == "server_disconnected"
                                                         or event["device_id"] == self.device_id)
        try:
            raw = base64.b64decode(event["payload_base64"], validate=True)
        except ValueError:
            self.counts["invalid_base64"] += 1
            return
        if kind == "device_state":
            self.observe_tracking(event, raw)

    def observe_tracking(self, event, raw):
        """统计 Tracking 和结构双手帧，对目标设备复用 SDK 解析器核对完整帧。"""
        counts = self.devices[event["device_id"]]
        try:
            outer = parse_json(raw.decode("utf-8"))
            require(isinstance(outer, dict), "设备状态须为对象")
            if outer.get("functionName", outer.get("function")) != "Tracking":
                return
            counts["tracking_callbacks"] += 1
            if event["drained"]:
                return
            counts["active_tracking_callbacks"] += 1
            value = outer["value"]
            if isinstance(value, str):
                value = parse_json(value)
            require(isinstance(value, dict), "Tracking.value 须为对象")
            controllers = value.get("Controller", {})
            if isinstance(controllers, dict) and all(
                    isinstance(controllers.get(side), dict) for side in ("left", "right")):
                counts["active_dual_hand_fields"] += 1
            if self.config is not None and event["device_id"] == self.device_id:
                from remote_control.protocol import parse_tracking
                from remote_control.sdk import SDKEvent

                sdk_event = SDKEvent.from_dict({field: event[field] for field in XR_EVENT_FIELDS})
                frame = parse_tracking(sdk_event, event["callback_seq"], self.config)
                counts["active_complete_dual_hand_frames"] += frame is not None
        except (ValueError, KeyError, TypeError, AttributeError, OverflowError):
            counts["invalid_payloads"] += 1

    def report(self):
        """返回按连接、设备和 JSONL 对账的报告，区分信封读回与目标设备可用性。"""
        connections = {}
        for connection, sequences in self.sequences.items():
            first, last = min(sequences), max(sequences)
            connections[connection] = dict(first_callback_seq=first, last_callback_seq=last,
                                           internal_missing=last - first + 1 - len(sequences),
                                           duplicates=sum(sequences.values()) - len(sequences))
        target = self.devices.get(self.device_id, {})
        tracking = target.get("active_tracking_callbacks", 0) if self.device_id else None
        hands = target.get("active_dual_hand_fields", 0) if self.device_id else None
        complete = target.get("active_complete_dual_hand_frames", 0) if self.config else None
        comparable = None
        if self.reference is not None:
            missing = self.reference.keys() - self.digests.keys()
            extra = self.digests.keys() - self.reference.keys()
            changed = {key for key in self.digests.keys() & self.reference.keys()
                       if self.digests[key] != self.reference[key]}
            comparable = dict(reference_events=len(self.reference), missing_from_mcap=len(missing),
                              extra_in_mcap=len(extra), content_mismatches=len(changed),
                              missing_examples=sorted(missing)[:10], extra_examples=sorted(extra)[:10],
                              mismatch_examples=sorted(changed)[:10], matches=not (missing or extra or changed))
        quality_ok = (self.counts["invalid_base64"] == 0 and
                      all(row["internal_missing"] == row["duplicates"] == 0 for row in connections.values()) and
                      (comparable is None or comparable["matches"]) and
                      (hands is None or hands > 0) and (complete is None or complete > 0))
        return dict(counts=dict(self.counts), kinds=dict(self.kinds), connections=connections,
                    devices={device: dict(counts) for device, counts in self.devices.items()},
                    target_device_id=self.device_id, target_tracking_callbacks=tracking,
                    target_dual_hand_fields=hands, target_complete_dual_hand_frames=complete,
                    jsonl_comparison=comparable, quality_ok=quality_ok)


def verify_mcap(directory, xr_device_id=None, xr_jsonl=None, xr_input_config=None):
    """在 Humble 环境读回 MCAP，核对类型、条数、图像结构及所选 XR；读取损坏时抛异常。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    session = parse_json((directory / "session.json").read_text(encoding="utf-8"))
    require(session["record_raw"], "会话未启用 MCAP")
    require(session["closed"], "先 STOP 并关闭会话，再读回 MCAP")
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(directory / "raw"), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    streams = session["mapping"]["streams"]
    expected = {entry["topic"]: entry["message_type"] for entry in streams}
    xr_topic = next((entry["topic"] for entry in streams if entry["id"] == "xr_raw"), None)
    require(xr_topic is not None or not (xr_device_id or xr_jsonl or xr_input_config), "会话未选择 xr_raw")
    require(xr_topic is None or expected[xr_topic] == "std_msgs/msg/String", "xr_raw 必须使用 std_msgs/msg/String")
    xr = None if xr_topic is None else XrReadback(xr_device_id, xr_jsonl, xr_input_config)
    actual = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    require(actual == expected, "MCAP 话题或消息类型与会话配置不一致")
    classes = {topic: get_message(kind) for topic, kind in actual.items()}
    counts = Counter()
    stamps = {}
    while reader.has_next():
        topic, payload, received_ns = reader.read_next()
        message = deserialize_message(payload, classes[topic])
        counts[topic] += 1
        if topic == xr_topic:
            xr.observe(message.data)
        if hasattr(message, "header"):
            stamp = message.header.stamp
            source_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            stats = stamps.setdefault(topic, dict(first_source_stamp_ns=source_ns,
                                                 first_receive_time_ns=received_ns,
                                                 source_repeats=0, source_backwards=0))
            previous = stats.get("last_source_stamp_ns")
            if previous is not None:
                stats["source_repeats"] += source_ns == previous
                stats["source_backwards"] += source_ns < previous
            stats["last_source_stamp_ns"] = source_ns
            stats["last_receive_time_ns"] = received_ns
        if actual[topic] == "sensor_msgs/msg/Image":
            require(message.width > 0 and message.height > 0 and message.step > 0 and
                    len(message.data) == message.step * message.height,
                    f"{topic} 图像尺寸、步长或数据长度无效")
            stamps[topic].update(width=message.width, height=message.height, encoding=message.encoding)
    for topic in expected:
        require(counts[topic] == session["raw"][topic]["written"], f"{topic} MCAP 条数与摘要不一致")
    report = dict(verified=True, counts={topic: counts[topic] for topic in expected}, sources=stamps)
    if xr is not None:
        report["xr_raw"] = xr.report()
    return report


def main():
    """解析会话目录并输出 JSON；完整返回 0，质量不完整返回 1，文件损坏返回 2。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", type=Path, help="包含 session.json 的会话目录")
    parser.add_argument("--verify-mcap", action="store_true", help="在机器人 ROS 环境实际读回并反序列化 MCAP")
    parser.add_argument("--xr-device-id", help="核对指定设备在录制期的 Tracking；默认使用 XR JSONL 元数据")
    parser.add_argument("--xr-jsonl", type=Path, help="对账同次遥操的 XR JSONL，并复用其中的输入协议")
    parser.add_argument("--xr-input-config", type=Path, help="验证完整双手帧所用的 XR 输入协议 JSON")
    options = parser.parse_args()
    if not options.verify_mcap and (options.xr_device_id or options.xr_jsonl or options.xr_input_config):
        parser.error("XR 选项需要 --verify-mcap")
    try:
        report = inspect_session(options.session_dir)
        if options.verify_mcap:
            report["mcap_readback"] = verify_mcap(options.session_dir, options.xr_device_id,
                                                 options.xr_jsonl, options.xr_input_config)
            report["raw_mcap_verified"] = True
        print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
        xr = report.get("mcap_readback", {}).get("xr_raw")
        return 0 if report["data_complete"] and (xr is None or xr["quality_ok"]) else 1
    except (OSError, ValueError, KeyError, TypeError, OverflowError, ImportError, RuntimeError) as error:
        print(json.dumps({"structure_valid": False, "error": str(error)}, ensure_ascii=False))
        return 2


if __name__ == "__main__":
    sys.exit(main())
