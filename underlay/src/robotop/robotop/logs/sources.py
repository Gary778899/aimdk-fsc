"""LogSource implementations: tail files / scan history / subscribe ROS topics."""
from __future__ import annotations

import os
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, Iterator, List, Optional, Protocol

from .parsers import LogLine, parse, parse_or_raw


class LogSource(Protocol):
    def name(self) -> str: ...
    def follow(self) -> Iterator[LogLine]: ...
    def scan(self, since: datetime,
             until: Optional[datetime] = None) -> Iterable[LogLine]: ...


def _read_last_n_lines(path: Path, n: int, block_size: int = 8192) -> List[str]:
    """Read the last `n` lines of a text file efficiently (reverse block scan)."""
    if n <= 0:
        return []
    try:
        size = path.stat().st_size
    except FileNotFoundError:
        return []
    if size == 0:
        return []
    with open(path, "rb") as f:
        end = size
        chunks: List[bytes] = []
        newline_count = 0
        while end > 0 and newline_count <= n:
            read_size = min(block_size, end)
            end -= read_size
            f.seek(end)
            chunk = f.read(read_size)
            chunks.append(chunk)
            newline_count += chunk.count(b"\n")
        data = b"".join(reversed(chunks))
    text = data.decode("utf-8", errors="replace")
    lines = text.splitlines()
    return lines[-n:]


def _tail_lines(
    path: Path,
    *,
    from_end: bool,
    poll_interval: float = 0.2,
    backfill: int = 0,
) -> Iterator[tuple[int, str]]:
    """Generator yielding (line_num, raw) — like `tail -F`. Re-opens on truncation.

    When `from_end=True` and `backfill>0`, first emit the last `backfill` lines
    of the file (so the viewer isn't empty before new lines arrive), then tail.
    """
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        total_lines = sum(1 for _ in fh)
    n = 0
    if from_end and backfill > 0:
        backfilled = _read_last_n_lines(path, backfill)
        start_line = max(1, total_lines - len(backfilled) + 1)
        for i, raw in enumerate(backfilled):
            yield start_line + i, raw
        n = total_lines

    with open(path, "r", encoding="utf-8", errors="replace") as f:
        if from_end:
            f.seek(0, os.SEEK_END)
            if not backfill:
                n = total_lines
        last_inode = os.fstat(f.fileno()).st_ino
        while True:
            line = f.readline()
            if line:
                n += 1
                yield n, line.rstrip("\n")
                continue
            time.sleep(poll_interval)
            try:
                st = os.stat(path)
                if st.st_ino != last_inode or st.st_size < f.tell():
                    f.close()
                    f = open(path, "r", encoding="utf-8", errors="replace")
                    last_inode = os.fstat(f.fileno()).st_ino
                    n = 0
            except FileNotFoundError:
                pass


class FileTailSource:
    """Generic single-file tail + scan source."""

    BACKFILL_LINES = 100

    def __init__(self, path: Path, label: str) -> None:
        self.path = path
        self.label = label

    def name(self) -> str:
        return f"{self.label}:{self.path.name}"

    def follow(self) -> Iterator[LogLine]:
        if not self.path.exists():
            return
        for n, raw in _tail_lines(self.path, from_end=True, backfill=self.BACKFILL_LINES):
            line = parse_or_raw(raw, source=self.label)
            line.file_path = str(self.path)
            line.line_num = n
            yield line

    def scan(self, since: datetime, until: Optional[datetime] = None) -> Iterable[LogLine]:
        if not self.path.exists():
            return
        with open(self.path, "r", encoding="utf-8", errors="replace") as f:
            for n, raw in enumerate(f, 1):
                line = parse(raw.rstrip("\n"))
                if line is None:
                    continue
                if line.timestamp < since:
                    continue
                if until is not None and line.timestamp > until:
                    return
                line.file_path = str(self.path)
                line.line_num = n
                yield line


class RosSessionSource:
    """Tail the *.log files in the latest ~/.ros/log/<session>/ dir; scan all sessions."""

    def __init__(self, log_root: Path = Path.home() / ".ros" / "log") -> None:
        self.root = log_root

    def name(self) -> str:
        return "ros_logs"

    def _latest_session(self) -> Optional[Path]:
        if not self.root.exists():
            return None
        sessions = [p for p in self.root.iterdir() if p.is_dir()]
        if not sessions:
            return None
        return max(sessions, key=lambda p: p.stat().st_mtime)

    def follow(self) -> Iterator[LogLine]:
        session = self._latest_session()
        if session is None:
            return
        files = sorted(session.glob("*.log"), key=lambda p: p.stat().st_mtime)
        if not files:
            return
        # MVP: tail only the most recently modified file
        target = files[-1]
        for n, raw in _tail_lines(target, from_end=True, backfill=FileTailSource.BACKFILL_LINES):
            line = parse_or_raw(raw, source=f"ros:{target.stem}")
            line.file_path = str(target)
            line.line_num = n
            yield line

    def scan(self, since: datetime, until: Optional[datetime] = None) -> Iterable[LogLine]:
        if not self.root.exists():
            return
        sessions = sorted(
            (p for p in self.root.iterdir() if p.is_dir()),
            key=lambda p: p.stat().st_mtime,
        )
        for session in sessions:
            session_mtime = datetime.fromtimestamp(
                session.stat().st_mtime, tz=timezone.utc)
            if until is not None and session_mtime < since:
                continue
            for f in sorted(session.glob("*.log")):
                src = FileTailSource(f, label=f"ros:{f.stem}")
                yield from src.scan(since, until)


class AimaLogSource:
    """Tail aima.log; scan rotated aima.log.YYYY-MM-DD files."""

    def __init__(self, log_dir: Path = Path.home() / ".aima" / "log") -> None:
        self.dir = log_dir

    def name(self) -> str:
        return "aima"

    def follow(self) -> Iterator[LogLine]:
        return FileTailSource(self.dir / "aima.log", label="aima").follow()

    def scan(self, since: datetime, until: Optional[datetime] = None) -> Iterable[LogLine]:
        if not self.dir.exists():
            return
        files = sorted(self.dir.glob("aima.log*"))
        for f in files:
            yield from FileTailSource(f, label="aima").scan(since, until)


class RosoutTopicSource:
    """Subscribe to /rosout, yield each message as a LogLine.

    Requires `rclpy.init()` already done (we hook into the shared RosBridge).
    Historical scan: not supported — /rosout is volatile.
    """

    def __init__(self, node) -> None:
        from rcl_interfaces.msg import Log

        self._buf: List[LogLine] = []
        self._node = node
        self._sub = node.create_subscription(Log, "/rosout", self._on_log, 100)

    def name(self) -> str:
        return "rosout"

    def _on_log(self, msg) -> None:
        level_map = {10: "DEBUG", 20: "INFO",
                     30: "WARN", 40: "ERROR", 50: "FATAL"}
        line = LogLine(
            timestamp=datetime.fromtimestamp(
                msg.stamp.sec + msg.stamp.nanosec / 1e9, tz=timezone.utc
            ),
            level=level_map.get(int(msg.level), "INFO"),
            source=f"ros:{msg.name}",
            message=msg.msg,
            raw=f"[{msg.name}] {msg.msg}",
            file_path=msg.file or None,
            line_num=int(msg.line) or None,
        )
        self._buf.append(line)

    def follow(self) -> Iterator[LogLine]:
        while True:
            if self._buf:
                yield self._buf.pop(0)
            else:
                time.sleep(0.05)

    def scan(self, since: datetime, until: Optional[datetime] = None) -> Iterable[LogLine]:
        return iter(())
