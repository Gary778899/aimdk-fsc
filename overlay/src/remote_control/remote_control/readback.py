"""解释原始 XR 回调，并与同一次录制的 JSONL 进行对账。"""

import base64
from collections import Counter, defaultdict
import hashlib
import json
import math

from .sdk import SDKEvent


XR_SCHEMA = "remote_control.xr_raw.v1"
XR_EVENT_FIELDS = (
    "sdk_connection_id", "callback_seq", "received_system_time_ns", "received_monotonic_ns",
    "received_monotonic_s", "kind", "callback_type", "status", "device_id", "payload_base64", "error",
)


def require(condition, message):
    """XR 协议条件不满足时抛出带定位信息的 ValueError。"""
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


def integer(value, location):
    """校验整数纳秒、序号或代码并返回原值，类型不符时抛出 ValueError。"""
    require(type(value) is int, f"{location} 必须为整数")
    return value


def validate_xr_event(event):
    """校验原始 XR 消息信封与时间字段，返回事件；载荷内容在读回阶段统计。"""
    require(isinstance(event, dict) and event.get("schema") == XR_SCHEMA, "XR 原始事件格式不受支持")
    require(isinstance(event["sdk_connection_id"], str) and event["sdk_connection_id"], "XR 缺少 SDK 连接标识")
    require(integer(event["callback_seq"], "XR callback_seq") > 0, "XR callback_seq 必须为正")
    require(type(event["drained"]) is bool, "XR drained 必须为布尔值")
    for field in ("received_system_time_ns", "received_monotonic_ns", "callback_type", "status"):
        integer(event[field], f"XR {field}")
    received = event["received_monotonic_s"]
    require(type(received) in (int, float) and math.isfinite(received),
            "XR received_monotonic_s 必须为有限数值")
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
            except (ValueError, KeyError, TypeError, OverflowError) as error:
                raise ValueError(f"{path}:{line_number}: {error}") from error
    require(isinstance(metadata, dict), f"{path}: 缺少 XR JSONL 元数据")
    return metadata, reference


class XrReadback:
    """核对原始 XR 信封和事件覆盖，对指定设备按原输入协议验证双手帧。"""

    def __init__(self, device_id, reference_path, input_config_path, *, expected_session_id=None):
        """读取对账日志和输入协议，设备、会话编号或协议冲突时抛出 ValueError。

        expected_session_id 绑定日志和消息的试次归属，并要求对账日志声明设备和输入协议。
        """
        metadata, self.reference = ({}, None) if reference_path is None else read_xr_reference(reference_path)
        logged_device = metadata.get("device_id")
        require(logged_device is None or isinstance(logged_device, str), "XR JSONL 设备标识必须为字符串")
        require(device_id is None or logged_device in (None, device_id), "指定的 XR 设备与 JSONL 元数据不一致")
        self.device_id = device_id or logged_device
        self.expected_session_id = expected_session_id
        configuration = metadata.get("input_config")
        if expected_session_id is not None and reference_path is not None:
            require(metadata.get("session_id") == expected_session_id,
                    "XR JSONL 的会话编号与实验清单不一致")
            require(logged_device and configuration is not None, "XR JSONL 须声明目标设备和输入协议")
        if input_config_path is not None:
            require(self.device_id, "使用 XR 输入配置时须指定设备或提供含设备标识的 XR JSONL")
            explicit = parse_json(input_config_path.read_text(encoding="utf-8"))
            require(configuration is None or configuration == explicit, "XR 输入配置与 JSONL 元数据不一致")
            configuration = explicit
        self.config = None
        if configuration is not None and self.device_id:
            from .protocol import InputConfig

            self.config = InputConfig.from_dict(configuration)
        self.counts = Counter()
        self.kinds = Counter()
        self.sequences = defaultdict(Counter)
        self.devices = defaultdict(Counter)
        self.digests = {}
        self.first_envelope_error = None

    def observe(self, text):
        """检查一条 XR 文本并累计计数，坏信封记入报告后继续接收后续回调。"""
        try:
            event = parse_xr_event(text)
            require(self.expected_session_id is None or event.get("session_id") == self.expected_session_id,
                    "XR 信封的会话编号与实验清单不一致")
        except (ValueError, KeyError, TypeError, OverflowError) as error:
            self.counts["invalid_envelopes"] += 1
            if self.first_envelope_error is None:
                self.first_envelope_error = f"原始 XR 信封无效：{error}"
            return
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
                from .protocol import parse_tracking

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
        quality_ok = (self.first_envelope_error is None and self.counts["invalid_base64"] == 0 and
                      all(row["internal_missing"] == row["duplicates"] == 0 for row in connections.values()) and
                      (comparable is None or comparable["matches"]) and
                      complete is not None and complete > 0)
        report = dict(counts=dict(self.counts), kinds=dict(self.kinds), connections=connections,
                      devices={device: dict(counts) for device, counts in self.devices.items()},
                      target_device_id=self.device_id, target_tracking_callbacks=tracking,
                      target_dual_hand_fields=hands, target_complete_dual_hand_frames=complete,
                      jsonl_comparison=comparable, quality_ok=quality_ok)
        if self.first_envelope_error is not None:
            report["error"] = self.first_envelope_error
        return report
