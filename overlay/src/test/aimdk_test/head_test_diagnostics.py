"""Buffered event diagnostics for head_test; disk writes occur outside motion."""
from collections import deque
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import time


class DiagnosticLog:
    """Collect bounded JSONL events in memory and save them when the run exits."""

    def __init__(self, directory=None):
        """在 directory 或工作区 log 创建北京时间日志；重名追加短序号。

        未指定目录且无法定位工作区时抛出 ValueError，其他文件错误直接传播。
        """
        if directory is None:
            candidates = (Path.cwd(), *Path.cwd().parents,
                          *Path(__file__).resolve().parents)
            root = next((p for marker in ('overlay/src/test/package.xml', 'src/test/package.xml')
                         for p in candidates
                         if (p / marker).is_file()), None)
            if root is None:
                raise ValueError('workspace not found; pass --log-dir /path/to/project/log')
            directory = root / 'log'
        directory = Path(directory).expanduser().resolve()
        directory.mkdir(parents=True, exist_ok=True)
        label = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
        index = 0
        while True:
            self.path = directory / (label + (f'_{index}' if index else '') + '.jsonl')
            try:
                self.stream = self.path.open('x', encoding='utf-8')
                break
            except FileExistsError:
                index += 1
        self.started = time.monotonic()
        self.events = deque(maxlen=20000)
        self.total = 0
        self.record('log_open', path=str(self.path), buffered=True)

    def record(self, event, **fields):
        """缓冲 event 及 fields，保留墙钟纳秒和单调经过秒数。"""
        now = time.monotonic()
        self.events.append(dict(event=event, elapsed_s=now-self.started,
                                monotonic_s=now, wall_time_ns=time.time_ns(), **fields))
        self.total += 1

    def close(self):
        """保存缓冲事件及截断计数，关闭文件；重复调用直接返回。"""
        if self.stream.closed:
            return
        self.record('log_close', dropped_events=max(0, self.total + 1 - self.events.maxlen))
        try:
            for event in self.events:
                self.stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + '\n')
        finally:
            self.stream.close()
