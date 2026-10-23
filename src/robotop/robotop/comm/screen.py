"""Textual screen: list topics/services + on-demand topic hz measurement."""
from __future__ import annotations

from typing import Dict, List

from rich.markup import escape
from textual.app import ComposeResult
from textual.binding import Binding
from textual.coordinate import Coordinate
from textual.screen import Screen
from textual.widgets import DataTable, Footer, Header, Input, Static, TabbedContent, TabPane

from .inspector import Inspector


class CommScreen(Screen):
    DEFAULT_CSS = """
    Input { dock: top; height: 3; }
    DataTable { height: 1fr; }
    #status { dock: bottom; height: 1; background: $boost; padding: 0 1; }
    """
    BINDINGS = [
        Binding("q", "back_or_quit", "Back"),
        Binding("slash", "focus_filter", "Filter"),
        Binding("escape", "blur_filter", "Unfocus"),
        Binding("m", "toggle_measure", "Measure"),
        Binding("M", "stop_all_measure", "Stop all"),
        Binding("r", "refresh_now", "Refresh"),
    ]

    # Column index of the "hz", "count" and "lat" cells in the topics table
    _COL_M = 0
    _COL_HZ = 5
    _COL_COUNT = 6
    _COL_LAT = 7

    def __init__(self, inspector: Inspector) -> None:
        super().__init__()
        self._insp = inspector
        self._filter = ""
        self._topic_row_keys: List[str] = []
        self._service_row_keys: List[str] = []
        self._last_user_msg = ""

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        yield Input(placeholder="filter substring (e.g. 'slam' or 'hal/sensor')", id="filter")
        with TabbedContent(initial="tab-topics"):
            with TabPane("Topics", id="tab-topics"):
                yield DataTable(id="tbl-topics", cursor_type="row")
            with TabPane("Services", id="tab-services"):
                yield DataTable(id="tbl-services", cursor_type="row")
        yield Static("", id="status")
        yield Footer()

    def on_mount(self) -> None:
        self.sub_title = "comm"
        topics = self.query_one("#tbl-topics", DataTable)
        topics.add_columns("M", "topic", "type", "pub",
                           "sub", "hz", "count", "lat(ms)")
        services = self.query_one("#tbl-services", DataTable)
        services.add_columns("service", "type")
        self._insp.refresh()
        self._render_full()
        self.set_interval(1.0, self._on_refresh_tick)
        self.set_interval(0.5, self._tick_values_only)

    def _on_refresh_tick(self) -> None:
        """Re-enumerate from rclpy. If the list shape changed, full rebuild;
        otherwise only refresh values to preserve cursor + scroll."""
        before = self._topic_row_keys + self._service_row_keys
        self._insp.refresh()
        new_topic_keys = [r["name"] for r in self._visible_topics()]
        new_service_keys = [r["name"] for r in self._visible_services()]
        after = new_topic_keys + new_service_keys
        if before != after:
            self._render_full()
        else:
            self._tick_values_only()

    def action_refresh_now(self) -> None:
        self._insp.refresh()
        self._render_full()

    def action_focus_filter(self) -> None:
        self.query_one("#filter", Input).focus()

    def action_blur_filter(self) -> None:
        self.query_one("#tbl-topics", DataTable).focus()

    def on_input_changed(self, event: Input.Changed) -> None:
        if event.input.id == "filter":
            self._filter = event.value.strip().lower()
            self._render_full()

    def action_toggle_measure(self) -> None:
        tbl = self.query_one("#tbl-topics", DataTable)
        if tbl.cursor_row is None:
            return
        rows = self._visible_topics()
        if not (0 <= tbl.cursor_row < len(rows)):
            return
        row = rows[tbl.cursor_row]
        name = row["name"]
        if self._insp.is_measuring(name):
            self._insp.stop_measure(name)
            self._last_user_msg = ""
        else:
            err = self._insp.start_measure(name, row["type"])
            self._last_user_msg = (
                f"[red]start failed for {name}: {err}[/red]" if err else ""
            )
        self._tick_values_only()

    def action_stop_all_measure(self) -> None:
        self._insp.stop_all()
        self._tick_values_only()

    def action_back_or_quit(self) -> None:
        if len(self.app.screen_stack) > 2:
            self.app.pop_screen()
        else:
            self.app.exit()

    def _visible_topics(self) -> List[Dict]:
        rows = self._insp.topics()
        if self._filter:
            rows = [r for r in rows if self._filter in r["name"].lower(
            ) or self._filter in r["type"].lower()]
        return rows

    def _visible_services(self) -> List[Dict]:
        rows = self._insp.list_services()
        if self._filter:
            rows = [r for r in rows if self._filter in r["name"].lower(
            ) or self._filter in r["type"].lower()]
        return rows

    def _render_full(self) -> None:
        """Rebuild both DataTables. Preserves cursor row index across rebuild."""
        topics = self._visible_topics()
        services = self._visible_services()

        tbl = self.query_one("#tbl-topics", DataTable)
        prev_cursor = tbl.cursor_row
        tbl.clear()
        for r in topics:
            name = r["name"]
            is_meas = self._insp.is_measuring(name)
            err = self._insp.get_error(name)
            hz = self._insp.get_hz(name) if is_meas else None
            count = self._insp.get_count(name) if is_meas else 0
            tbl.add_row(
                _m_glyph(is_meas, err),
                escape(name),
                escape(r["type"]),
                str(r["pub_count"]),
                str(r["sub_count"]),
                f"{hz:.2f}" if hz is not None else ("…" if is_meas else "-"),
                str(count) if is_meas else "-",
                self._lat_cell(name, is_meas),
            )
        self._topic_row_keys = [r["name"] for r in topics]
        if topics and prev_cursor is not None:
            tbl.move_cursor(
                row=min(prev_cursor, len(topics) - 1), animate=False)

        stbl = self.query_one("#tbl-services", DataTable)
        prev_scursor = stbl.cursor_row
        stbl.clear()
        for r in services:
            stbl.add_row(escape(r["name"]), escape(r["type"]))
        self._service_row_keys = [r["name"] for r in services]
        if services and prev_scursor is not None:
            stbl.move_cursor(
                row=min(prev_scursor, len(services) - 1), animate=False)

        self._update_status(topics, services)

    def _tick_values_only(self) -> None:
        tbl = self.query_one("#tbl-topics", DataTable)
        rows = self._visible_topics()
        for idx, r in enumerate(rows):
            name = r["name"]
            is_meas = self._insp.is_measuring(name)
            err = self._insp.get_error(name)
            hz = self._insp.get_hz(name) if is_meas else None
            count = self._insp.get_count(name) if is_meas else 0
            try:
                tbl.update_cell_at(
                    Coordinate(idx, self._COL_M), _m_glyph(is_meas, err), update_width=False
                )
                tbl.update_cell_at(
                    Coordinate(idx, self._COL_HZ),
                    f"{hz:.2f}" if hz is not None else (
                        "…" if is_meas else "-"),
                    update_width=False,
                )
                tbl.update_cell_at(
                    Coordinate(idx, self._COL_COUNT),
                    str(count) if is_meas else "-",
                    update_width=False,
                )
                tbl.update_cell_at(
                    Coordinate(idx, self._COL_LAT),
                    self._lat_cell(name, is_meas),
                    update_width=False,
                )
            except Exception:
                return
        self._update_status(rows, self._visible_services())

    def _lat_cell(self, name: str, is_meas: bool) -> str:
        """Format the lat(ms) cell: '-' when idle, 'n/a' for header-less types,
        '…' while waiting for the first stamped message, else colored ms."""
        if not is_meas:
            return "-"
        if not self._insp.has_header(name):
            return "[dim]n/a[/dim]"
        lat = self._insp.get_latency(name)
        if lat is None:
            return "…"
        ms = lat * 1000.0
        color = "green" if ms < 100 else ("yellow" if ms < 500 else "red")
        return f"[{color}]{ms:.0f}[/{color}]"

    def _update_status(self, topics: List[Dict], services: List[Dict]) -> None:
        topics_total = len(self._insp.topics())
        services_total = len(self._insp.list_services())
        measuring = sum(
            1 for r in topics if self._insp.is_measuring(r["name"]))
        msg = self._last_user_msg or "[dim](m=toggle measure, M=stop all, /=filter, r=refresh)[/dim]"
        self.query_one("#status", Static).update(
            f"topics={len(topics)}/{topics_total}  services={len(services)}/{services_total}  "
            f"measuring={measuring}  filter=[yellow]{escape(self._filter) or '*'}[/yellow]  {msg}"
        )


def _m_glyph(is_measuring: bool, error: str) -> str:
    if is_measuring:
        return "●"
    if error:
        return "[red]✗[/red]"
    return ""


# Backwards-compat alias
CommApp = CommScreen
