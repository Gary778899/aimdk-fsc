"""Textual screens for HDS faults: live active codes + historical event query."""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

from rich.markup import escape
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Vertical
from textual.screen import Screen
from textual.widgets import DataTable, Footer, Header, Static, TabbedContent, TabPane

from ..status.collector import StatusCollector, StatusSnapshot


def _ns_to_local(ns: int) -> str:
    if not ns:
        return ""
    dt = datetime.fromtimestamp(ns / 1e9).astimezone()
    return dt.strftime("%H:%M:%S.%f")[:-3]


_DIAG_TYPE_NAMES = {0: "NORMAL", 1: "TRIGGER+", 2: "TRIGGER-"}


class LiveFaultsScreen(Screen):
    """Live view of /aima/hds/{diag_code_list, alert_code_list, post_proc/signal}."""

    DEFAULT_CSS = """
    DataTable { height: 1fr; }
    #status { dock: bottom; height: 1; background: $boost; padding: 0 1; }
    """
    BINDINGS = [
        Binding("q", "back_or_quit", "Back"),
        Binding("r", "refresh", "Refresh"),
    ]

    def __init__(self, collector: StatusCollector) -> None:
        super().__init__()
        self._collector = collector

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with TabbedContent(initial="tab-diag"):
            with TabPane("Diag", id="tab-diag"):
                yield DataTable(id="tbl-diag", cursor_type="row")
            with TabPane("Alert", id="tab-alert"):
                yield DataTable(id="tbl-alert", cursor_type="row")
            with TabPane("PostProc", id="tab-pp"):
                yield DataTable(id="tbl-pp", cursor_type="row")
        yield Static("", id="status")
        yield Footer()

    def on_mount(self) -> None:
        self.sub_title = "faults (live)"
        diag = self.query_one("#tbl-diag", DataTable)
        diag.add_columns("first", "last", "type", "module",
                         "code", "diag_code", "info")
        alert = self.query_one("#tbl-alert", DataTable)
        alert.add_columns("first", "last", "code")
        pp = self.query_one("#tbl-pp", DataTable)
        pp.add_columns("ts", "diag_code", "robot", "motion",
                       "sound", "led", "facial", "funcdis")

        self.set_interval(0.5, self.action_refresh)
        self.action_refresh()

    def action_back_or_quit(self) -> None:
        if len(self.app.screen_stack) > 2:
            self.app.pop_screen()
        else:
            self.app.exit()

    def action_refresh(self) -> None:
        snap = self._collector.snapshot()
        self._refresh_diag(snap)
        self._refresh_alert(snap)
        self._refresh_pp(snap)
        self.query_one("#status", Static).update(
            f"diag={len(snap.diag_list)}  alert={len(snap.alert_list)}  "
            f"pp={len(snap.postproc_list)}  hds_available={snap.hds_available}"
        )

    def _refresh_diag(self, snap: StatusSnapshot) -> None:
        tbl = self.query_one("#tbl-diag", DataTable)
        tbl.clear()
        for d in snap.diag_list:
            tbl.add_row(
                _ns_to_local(d["first_timestamp"]),
                _ns_to_local(d["timestamp"]),
                _DIAG_TYPE_NAMES.get(d["type"], str(d["type"])),
                str(d["module_id"]),
                str(d["code"]),
                str(d["diag_code"]),
                escape((d.get("info") or "")[:80]),
            )

    def _refresh_alert(self, snap: StatusSnapshot) -> None:
        tbl = self.query_one("#tbl-alert", DataTable)
        tbl.clear()
        for a in snap.alert_list:
            tbl.add_row(
                _ns_to_local(a["first_timestamp"]),
                _ns_to_local(a["timestamp"]),
                escape(a.get("code") or ""),
            )

    def _refresh_pp(self, snap: StatusSnapshot) -> None:
        tbl = self.query_one("#tbl-pp", DataTable)
        tbl.clear()
        for p in snap.postproc_list:
            tbl.add_row(
                _ns_to_local(p["timestamp"]),
                str(p["diag_code"]),
                str(p["robot_sig"]),
                str(p["motion_sig"]),
                str(p["sound_sig"]),
                str(p["led_sig"]),
                str(p["facial_sig"]),
                f"0x{p['funcdisable_sig']:x}",
            )


class HistoryFaultsScreen(Screen):
    """Static table of HdsEventRecord rows returned by GetHdsEventHistory."""

    DEFAULT_CSS = """
    DataTable { height: 1fr; }
    #status { dock: bottom; height: 1; background: $boost; padding: 0 1; }
    """
    BINDINGS = [Binding("q", "back_or_quit", "Back")]

    def __init__(
        self,
        records: List[Dict[str, Any]],
        total_count: int,
        has_more: bool,
        query_summary: str,
    ) -> None:
        super().__init__()
        self._records = records
        self._total = total_count
        self._has_more = has_more
        self._summary = query_summary

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        yield DataTable(id="tbl", cursor_type="row")
        yield Static("", id="status")
        yield Footer()

    def on_mount(self) -> None:
        self.sub_title = "faults (history)"
        tbl = self.query_one("#tbl", DataTable)
        tbl.add_columns(
            "id", "first", "last", "duration", "recovered", "diag_code", "alert_code", "loc"
        )
        for r in self._records:
            tbl.add_row(
                str(r["id"]),
                _ns_to_local(r["first_timestamp"]),
                _ns_to_local(r["last_timestamp"]),
                _fmt_duration(r["duration_ns"]),
                "✓" if r["recovered"] else "[red]active[/red]",
                str(r["diag_code"]) if r["diag_code"] else "",
                escape(r.get("alert_code") or ""),
                escape(r.get("alert_location") or ""),
            )
        more = " (more available)" if self._has_more else ""
        self.query_one("#status", Static).update(
            f"total={self._total}  shown={len(self._records)}{more}  query=[yellow]{self._summary}[/yellow]"
        )

    def action_back_or_quit(self) -> None:
        if len(self.app.screen_stack) > 2:
            self.app.pop_screen()
        else:
            self.app.exit()


# Backwards-compat aliases
LiveFaultsApp = LiveFaultsScreen
HistoryFaultsApp = HistoryFaultsScreen


def _fmt_duration(ns: int) -> str:
    if not ns:
        return ""
    s = ns / 1e9
    if s < 60:
        return f"{s:.1f}s"
    if s < 3600:
        return f"{s/60:.1f}m"
    return f"{s/3600:.1f}h"
