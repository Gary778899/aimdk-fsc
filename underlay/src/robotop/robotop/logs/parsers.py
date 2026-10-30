"""Parse three log formats into a uniform LogLine.

Sources observed:
  - ROS2 node logs (~/.ros/log/<session>/*.log):
      `<unix_ts> [LEVEL] [node]: msg`           (launch log)
      `[LEVEL] [<unix_ts>] [node]: msg`         (node log)
  - AIMA framework (~/.aima/log/aima.log):
      `YYYY-MM-DD HH:MM:SS,mmm  LEVEL  msg`
  - AIMRT app logs (e.g. ~/qiu/face_ui.log):
      `[YYYY-MM-DD HH:MM:SS.uuuuuu][Level][pid][file.cc:line] msg`
"""
from __future__ import annotations

import re
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Optional


@dataclass
class LogLine:
    timestamp: datetime
    level: str
    source: str
    message: str
    raw: str
    file_path: Optional[str] = None
    line_num: Optional[int] = None


_LEVEL_MAP = {
    "DEBUG": "DEBUG", "INFO": "INFO", "WARN": "WARN", "WARNING": "WARN",
    "ERROR": "ERROR", "FATAL": "FATAL", "TRACE": "DEBUG",
}


def _norm_level(s: str) -> str:
    return _LEVEL_MAP.get(s.upper(), s.upper())


_ROS_BRACKET = re.compile(
    r"^\[(?P<level>[A-Z]+)\] \[(?P<ts>[\d.]+)\] \[(?P<node>[^\]]+)\]: ?(?P<msg>.*)$"
)
_ROS_INLINE = re.compile(
    r"^(?P<ts>[\d.]+) \[(?P<level>[A-Z]+)\] \[(?P<node>[^\]]+)\]: ?(?P<msg>.*)$"
)
_AIMA = re.compile(
    r"^(?P<date>\d{4}-\d{2}-\d{2}) (?P<time>\d{2}:\d{2}:\d{2}),(?P<ms>\d+)\s+(?P<level>\w+)\s+(?P<msg>.*)$"
)
_AIMRT = re.compile(
    r"^\[(?P<date>\d{4}-\d{2}-\d{2}) (?P<time>\d{2}:\d{2}:\d{2})\.(?P<us>\d+)\]"
    r"\[(?P<level>\w+)\]\[(?P<pid>\d+)\]\[(?P<file>[^\]]+)\] ?(?P<msg>.*)$"
)


def _local_to_utc(naive: datetime) -> datetime:
    return naive.astimezone(timezone.utc)


def parse_ros(raw: str) -> Optional[LogLine]:
    m = _ROS_BRACKET.match(raw) or _ROS_INLINE.match(raw)
    if not m:
        return None
    return LogLine(
        timestamp=datetime.fromtimestamp(
            float(m.group("ts")), tz=timezone.utc),
        level=_norm_level(m.group("level")),
        source=f"ros:{m.group('node')}",
        message=m.group("msg"),
        raw=raw,
    )


def parse_aima(raw: str) -> Optional[LogLine]:
    m = _AIMA.match(raw)
    if not m:
        return None
    naive = datetime.strptime(
        f"{m.group('date')} {m.group('time')}", "%Y-%m-%d %H:%M:%S"
    ).replace(microsecond=int(m.group("ms")) * 1000)
    return LogLine(
        timestamp=_local_to_utc(naive),
        level=_norm_level(m.group("level")),
        source="aima",
        message=m.group("msg"),
        raw=raw,
    )


def parse_aimrt(raw: str) -> Optional[LogLine]:
    m = _AIMRT.match(raw)
    if not m:
        return None
    us = m.group("us").ljust(6, "0")[:6]
    naive = datetime.strptime(
        f"{m.group('date')} {m.group('time')}.{us}", "%Y-%m-%d %H:%M:%S.%f"
    )
    return LogLine(
        timestamp=_local_to_utc(naive),
        level=_norm_level(m.group("level")),
        source=f"aimrt:{m.group('file')}",
        message=m.group("msg"),
        raw=raw,
    )


_PARSERS = (parse_ros, parse_aima, parse_aimrt)


def parse(raw: str, *, fallback_source: str = "unknown") -> Optional[LogLine]:
    """Try parsers in order; return None if no parser matches."""
    if not raw or not raw.strip():
        return None
    for fn in _PARSERS:
        result = fn(raw)
        if result is not None:
            return result
    return None


def parse_or_raw(raw: str, *, source: str = "unknown") -> LogLine:
    """Like parse() but always returns a LogLine (current-time fallback)."""
    line = parse(raw)
    if line is not None:
        return line
    return LogLine(
        timestamp=datetime.now(timezone.utc),
        level="INFO",
        source=source,
        message=raw,
        raw=raw,
    )
