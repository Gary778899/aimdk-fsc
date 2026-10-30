"""双臂功能的有界内存 JSONL 日志；运动结束后集中写盘。"""
from collections import deque
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import uuid


class DiagnosticLog:
    """在内存中缓存有界事件，退出时写入 JSONL 文件。"""

    def __init__(self, directory=None, prefix='dual_arm_control'):
        """在 directory 或自动探测的功能包工作区 log 目录创建唯一日志；无法探测或写入时抛出 ValueError/OSError。"""
        if directory is None:
            candidates = (Path.cwd(), *Path.cwd().parents,
                          *Path(__file__).resolve().parents)
            root = next((p for marker in ('overlay/src/functions/package.xml', 'src/functions/package.xml')
                         for p in candidates
                         if (p / marker).is_file()), None)
            if root is None:
                raise ValueError('workspace not found; pass --log-dir /path/to/project/log')
            directory = root / 'log'
        directory = Path(directory).expanduser().resolve()
        directory.mkdir(parents=True, exist_ok=True)
        label = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        self.path = directory / f'{prefix}_{label}_{uuid.uuid4().hex[:8]}.jsonl'
        self.stream = self.path.open('x', encoding='utf-8')
        self.started = time.monotonic()
        self.events = deque(maxlen=20000)
        self.total = 0
        self.record('log_open', path=str(self.path), buffered=True)

    def record(self, event, **fields):
        """缓存事件与字段，附加墙钟时间和单调经过时间；超过容量时丢弃最早事件。"""
        now = time.monotonic()
        self.events.append(dict(event=event, elapsed_s=now-self.started,
                                monotonic_s=now, wall_time_ns=time.time_ns(), **fields))
        self.total += 1

    def close(self):
        """将缓存和丢弃计数写入日志并关闭；重复调用不重复写入，写盘错误向调用者传播。"""
        if self.stream.closed:
            return
        self.record('log_close', dropped_events=max(0, self.total + 1 - self.events.maxlen))
        try:
            for event in self.events:
                self.stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + '\n')
        finally:
            self.stream.close()
