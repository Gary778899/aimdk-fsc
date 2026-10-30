#!/usr/bin/env python3
"""仅用 Python 标准库读回统一采集帧，核对结构、计数、频率和数据质量。"""

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import sys


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


def main():
    """解析会话目录并输出 JSON；完整返回 0，质量不完整返回 1，文件损坏返回 2。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", type=Path, help="包含 session.json 和 frames.jsonl 的会话目录")
    options = parser.parse_args()
    try:
        report = inspect_session(options.session_dir)
        print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
        return 0 if report["data_complete"] else 1
    except (OSError, ValueError, KeyError, TypeError, OverflowError) as error:
        print(json.dumps({"structure_valid": False, "error": str(error)}, ensure_ascii=False))
        return 2


if __name__ == "__main__":
    sys.exit(main())
