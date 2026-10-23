"""Buffered event diagnostics for head_test; disk writes occur outside motion."""
from collections import deque
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import uuid


class DiagnosticLog:
    """Collect bounded JSONL events in memory and save them when the run exits."""

    def __init__(self, directory=None, prefix='head_test'):
        """Create a unique log in directory or the detected workspace log folder.

        Prefix labels the tool in the filename. Raise OSError if the destination cannot be created, or ValueError if the
        workspace cannot be found and no explicit directory was supplied.
        """
        if directory is None:
            candidates = (Path.cwd(), *Path.cwd().parents,
                          *Path(__file__).resolve().parents)
            root = next((p for p in candidates
                         if (p / 'src/test/package.xml').is_file()), None)
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
        """Buffer an event with UTC wall time and monotonic elapsed seconds."""
        now = time.monotonic()
        self.events.append(dict(event=event, elapsed_s=now-self.started,
                                monotonic_s=now, wall_time_ns=time.time_ns(), **fields))
        self.total += 1

    def close(self):
        """Save buffered events and any truncation count; close the file once."""
        if self.stream.closed:
            return
        self.record('log_close', dropped_events=max(0, self.total + 1 - self.events.maxlen))
        try:
            for event in self.events:
                self.stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + '\n')
        finally:
            self.stream.close()
