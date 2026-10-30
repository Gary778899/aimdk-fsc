"""Enumerate topics / services and optionally measure topic publish frequency.

Design:
  - `Inspector` is a rclpy Node added to the shared RosBridge executor.
  - `refresh()` is called by the UI on a timer; it cheaply re-enumerates
    topics/services and their publisher/subscriber counts.
  - `start_measure(topic, type_str)` creates a subscription with sensor-data
    QoS that counts arrival timestamps in a deque; `get_hz(topic)` returns the
    current rolling rate. `stop_measure(topic)` tears it down.

  Measurement is opt-in per topic because subscribing to every topic at once
  is wasteful (big msgs like PointCloud2 / Image consume real CPU/network).
"""
from __future__ import annotations

import collections
import time
from threading import Lock
from typing import Any, Dict, List, Optional

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message


_SENSOR_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)


class _Meter:
    __slots__ = ("times", "latencies", "sub", "count_total", "no_header")

    def __init__(self) -> None:
        self.times: collections.deque[float] = collections.deque(maxlen=50)
        # Rolling window of per-message latencies in seconds (reception - header.stamp).
        self.latencies: collections.deque[float] = collections.deque(maxlen=50)
        self.sub = None
        self.count_total = 0
        # Set once we've seen a message with no usable header stamp, so the UI can
        # show "n/a" instead of "…" forever for header-less message types.
        self.no_header = False


class Inspector(Node):
    def __init__(self, include_hidden: bool = False) -> None:
        super().__init__("robotop_comm_inspector")
        self._topic_rows: List[Dict[str, Any]] = []
        self._service_rows: List[Dict[str, Any]] = []
        self._meters: Dict[str, _Meter] = {}
        self._errors: Dict[str, str] = {}
        self._lock = Lock()
        self._include_hidden = include_hidden

    def refresh(self) -> None:
        topics = self.get_topic_names_and_types()
        services = self.get_service_names_and_types()
        topic_rows = []
        for name, types in topics:
            if not self._include_hidden and _is_hidden(name):
                continue
            topic_rows.append({
                "name": name,
                "type": types[0] if types else "",
                "all_types": types,
                "pub_count": self.count_publishers(name),
                "sub_count": self.count_subscribers(name),
            })
        topic_rows.sort(key=lambda r: r["name"])
        service_rows = [
            {"name": name, "type": types[0]
                if types else "", "all_types": types}
            for name, types in sorted(services)
            if self._include_hidden or not _is_hidden(name)
        ]
        with self._lock:
            self._topic_rows = topic_rows
            self._service_rows = service_rows

    def topics(self) -> List[Dict[str, Any]]:
        with self._lock:
            return list(self._topic_rows)

    def list_services(self) -> List[Dict[str, Any]]:
        with self._lock:
            return list(self._service_rows)

    def is_measuring(self, topic: str) -> bool:
        with self._lock:
            return topic in self._meters

    def start_measure(self, topic: str, type_str: str) -> Optional[str]:
        """Returns None on success, error string on failure (persisted in get_error)."""
        with self._lock:
            self._errors.pop(topic, None)
            if topic in self._meters:
                return None
        try:
            msg_cls = get_message(type_str)
        except (ImportError, AttributeError, ValueError) as e:
            err = f"type not installed: {type_str}"
            with self._lock:
                self._errors[topic] = err
            return err
        sub_qos = self._match_publisher_qos(topic)
        meter = _Meter()
        try:
            meter.sub = self.create_subscription(
                msg_cls, topic, lambda m, t=topic: self._on_msg(t, m), sub_qos
            )
        except Exception as e:
            err = f"subscribe failed: {e}"
            with self._lock:
                self._errors[topic] = err
            return err
        with self._lock:
            self._meters[topic] = meter
        return None

    def _match_publisher_qos(self, topic: str) -> QoSProfile:
        """Match the first publisher's reliability+durability so the subscription
        actually receives messages. Falls back to sensor-data QoS if no publisher."""
        try:
            pubs = self.get_publishers_info_by_topic(topic)
        except Exception:
            pubs = []
        if not pubs:
            return _SENSOR_QOS
        pq = pubs[0].qos_profile
        return QoSProfile(
            depth=5,
            reliability=pq.reliability,
            durability=pq.durability,
            history=HistoryPolicy.KEEP_LAST,
        )

    def get_error(self, topic: str) -> Optional[str]:
        with self._lock:
            return self._errors.get(topic)

    def stop_measure(self, topic: str) -> None:
        with self._lock:
            meter = self._meters.pop(topic, None)
        if meter is not None and meter.sub is not None:
            try:
                self.destroy_subscription(meter.sub)
            except Exception:
                pass

    def _on_msg(self, topic: str, msg: Any) -> None:
        now = time.monotonic()
        latency = _header_latency(msg)
        with self._lock:
            meter = self._meters.get(topic)
            if meter is None:
                return
            meter.times.append(now)
            meter.count_total += 1
            if latency is None:
                meter.no_header = True
            else:
                meter.latencies.append(latency)

    def get_hz(self, topic: str) -> Optional[float]:
        with self._lock:
            meter = self._meters.get(topic)
            if meter is None or len(meter.times) < 2:
                return None
            span = meter.times[-1] - meter.times[0]
            if span <= 0:
                return None
            return (len(meter.times) - 1) / span

    def get_count(self, topic: str) -> int:
        with self._lock:
            meter = self._meters.get(topic)
            return meter.count_total if meter else 0

    def get_latency(self, topic: str) -> Optional[float]:
        """Rolling mean latency in seconds (reception - header.stamp), or None if
        not yet available. Use has_header() to tell "no data yet" from "no header"."""
        with self._lock:
            meter = self._meters.get(topic)
            if meter is None or not meter.latencies:
                return None
            return sum(meter.latencies) / len(meter.latencies)

    def has_header(self, topic: str) -> bool:
        """False once a message without a usable header stamp has been seen."""
        with self._lock:
            meter = self._meters.get(topic)
            return not (meter is not None and meter.no_header)

    def stop_all(self) -> None:
        with self._lock:
            topics = list(self._meters)
        for t in topics:
            self.stop_measure(t)


def _is_hidden(name: str) -> bool:
    """Match `ros2 topic list` default filter: any path segment starts with `_`."""
    return any(seg.startswith("_") for seg in name.split("/") if seg)


def _header_latency(msg: Any) -> Optional[float]:
    """Latency in seconds = wall clock now - msg.header.stamp, mirroring
    `ros2 topic delay`. Returns None when the message has no usable header stamp.

    Uses time.time() (wall) rather than time.monotonic() because header stamps are
    epoch-based; this matches system time when use_sim_time is false."""
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None:
        return None
    try:
        sec = stamp.sec
        nanosec = stamp.nanosec
    except AttributeError:
        return None
    if sec == 0 and nanosec == 0:
        # Unset stamp; publisher didn't fill it in.
        return None
    return time.time() - (sec + nanosec * 1e-9)
