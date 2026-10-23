"""Textual log viewer: live tail or historical scan with filter DSL.

Filter DSL (space-separated tokens, all must match):
  level=INFO              exact level
  level>=WARN             level threshold (DEBUG<INFO<WARN<ERROR<FATAL)
  node~=slam              substring match against the source field
  any_other_text          substring match against message (case-insensitive)
"""
from __future__ import annotations

import queue
import re
import threading
import time
from collections import deque
from datetime import datetime, timezone
from typing import Callable, List, Optional

from textual.app import ComposeResult
from textual.containers import Vertical
from textual.screen import Screen
from textual.widgets import Footer, Header, Input, RichLog, Static
from rich.markup import escape

from .parsers import LogLine
from .sources import LogSource


_LEVEL_ORDER = {"DEBUG": 10, "INFO": 20, "WARN": 30, "ERROR": 40, "FATAL": 50}


def _compile_filter(expr: str) -> Callable[[LogLine], bool]:
    expr = (expr or "").strip()
    if not expr:
        return lambda _l: True
    text_terms: List[str] = []
    predicates: List[Callable[[LogLine], bool]] = []
    for token in expr.split():
        if m := re.match(r"^level=(\w+)$", token, re.I):
            wanted = m.group(1).upper()
            predicates.append(lambda l, w=wanted: l.level == w)
        elif m := re.match(r"^level>=(\w+)$", token, re.I):
            thr = _LEVEL_ORDER.get(m.group(1).upper(), 0)
            predicates.append(
                lambda l, t=thr: _LEVEL_ORDER.get(l.level, 0) >= t)
        elif m := re.match(r"^node~=(.+)$", token, re.I):
            needle = m.group(1).lower()
            predicates.append(lambda l, n=needle: n in l.source.lower())
        elif m := re.match(r"^source~=(.+)$", token, re.I):
            needle = m.group(1).lower()
            predicates.append(lambda l, n=needle: n in l.source.lower())
        else:
            text_terms.append(token.lower())
    if text_terms:
        predicates.append(
            lambda l, ts=text_terms: all(t in l.message.lower() for t in ts)
        )
    return lambda line: all(p(line) for p in predicates)


_LEVEL_STYLE = {
    "DEBUG": "dim",
    "INFO": "white",
    "WARN": "yellow",
    "ERROR": "red",
    "FATAL": "bold red",
}


def _render(line: LogLine) -> str:
    ts = line.timestamp.astimezone().strftime("%H:%M:%S.%f")[:-3]
    style = _LEVEL_STYLE.get(line.level, "white")
    src = escape(line.source[:30])
    msg = escape(line.message)
    return f"[dim]{ts}[/dim] [{style}]{line.level:5}[/{style}] [cyan]{src:30}[/cyan] {msg}"


class LogScreen(Screen):
    DEFAULT_CSS = """
    Input { dock: top; }
    RichLog { height: 1fr; }
    #status { dock: bottom; height: 1; background: $boost; padding: 0 1; }
    """
    BINDINGS = [
        ("q", "back_or_quit", "Back"),
        ("slash", "focus_filter", "Filter"),
        ("escape", "blur_filter", "Unfocus"),
        ("ctrl+l", "clear_log", "Clear"),
    ]

    def __init__(
        self,
        sources: List[LogSource],
        mode: str = "live",
        since: Optional[datetime] = None,
        until: Optional[datetime] = None,
        initial_filter: str = "",
    ) -> None:
        super().__init__()
        self._sources = sources
        self._mode = mode
        self._since = since
        self._until = until
        self._filter_expr = initial_filter
        self._predicate = _compile_filter(initial_filter)
        self._q: "queue.Queue[LogLine]" = queue.Queue(maxsize=10000)
        self._buffer: "deque[LogLine]" = deque(maxlen=5000)
        self._total = 0
        self._shown = 0
        self._threads: List[threading.Thread] = []

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        yield Input(
            placeholder="filter: level=INFO  level>=WARN  node~=slam  text...",
            value=self._filter_expr,
            id="filter",
        )
        yield RichLog(id="log", highlight=False, markup=True, auto_scroll=True, wrap=False)
        yield Static("", id="status")

    def on_mount(self) -> None:
        self.sub_title = f"logs ({self._mode})"
        self._update_status()
        if self._mode == "live":
            for src in self._sources:
                t = threading.Thread(
                    target=self._pump_source, args=(src,), name=f"src-{src.name()}", daemon=True
                )
                t.start()
                self._threads.append(t)
            self.set_interval(0.1, self._drain_queue)
        else:
            self.call_after_refresh(self._load_history)

    def _pump_source(self, src: LogSource) -> None:
        try:
            for line in src.follow():
                try:
                    self._q.put_nowait(line)
                except queue.Full:
                    pass
        except Exception:
            pass

    def _drain_queue(self) -> None:
        log = self.query_one("#log", RichLog)
        n = 0
        while not self._q.empty() and n < 200:
            try:
                line = self._q.get_nowait()
            except queue.Empty:
                break
            self._total += 1
            self._buffer.append(line)
            if self._predicate(line):
                log.write(_render(line))
                self._shown += 1
            n += 1
        if n > 0:
            self._update_status()

    def _load_history(self) -> None:
        log = self.query_one("#log", RichLog)
        all_lines: List[LogLine] = []
        for src in self._sources:
            try:
                all_lines.extend(src.scan(self._since, self._until))
            except Exception as e:
                log.write(f"[red]source {src.name()} scan error: {e}[/red]")
        all_lines.sort(key=lambda l: l.timestamp)
        self._total = len(all_lines)
        for line in all_lines:
            if self._predicate(line):
                log.write(_render(line))
                self._shown += 1
        self._update_status()

    def on_input_submitted(self, event: Input.Submitted) -> None:
        if event.input.id != "filter":
            return
        self._filter_expr = event.value
        self._predicate = _compile_filter(event.value)
        if self._mode == "history":
            self._shown = 0
            self.query_one("#log", RichLog).clear()
            self.call_after_refresh(self._load_history)
        else:
            self._rerender_buffer()
        self._update_status()

    def _rerender_buffer(self) -> None:
        """Live mode: re-apply the current filter to the retained buffer."""
        log = self.query_one("#log", RichLog)
        log.clear()
        self._shown = 0
        for line in self._buffer:
            if self._predicate(line):
                log.write(_render(line))
                self._shown += 1

    def _update_status(self) -> None:
        srcs = ", ".join(s.name() for s in self._sources)
        self.query_one("#status", Static).update(
            f"mode={self._mode}  total={self._total}  shown={self._shown}  "
            f"filter=[yellow]{self._filter_expr or '*'}[/yellow]  sources=[dim]{srcs}[/dim]"
        )

    def action_focus_filter(self) -> None:
        self.query_one("#filter", Input).focus()

    def action_blur_filter(self) -> None:
        self.query_one("#log", RichLog).focus()

    def action_clear_log(self) -> None:
        self.query_one("#log", RichLog).clear()
        self._buffer.clear()
        self._shown = 0
        self._update_status()

    def action_back_or_quit(self) -> None:
        if len(self.app.screen_stack) > 2:
            self.app.pop_screen()
        else:
            self.app.exit()


LogApp = LogScreen
