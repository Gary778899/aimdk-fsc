"""typer CLI entrypoint: `robotop` (menu) + dash/logs/comm/faults/orin-monitor subcommands."""
from __future__ import annotations

from pathlib import Path
from typing import List, Optional

import typer
from textual.app import App
from textual.screen import Screen


app = typer.Typer(
    no_args_is_help=False,
    add_completion=False,
    help="robotop — robot debug TUI",
    invoke_without_command=True,
)


class _SingleScreenApp(App):
    """Minimal Textual App that just pushes one Screen — used by subcommands so
    the same Screen classes work standalone or inside the main menu app."""

    TITLE = "robotop"

    def __init__(self, screen: Screen) -> None:
        super().__init__()
        self._initial_screen = screen

    def on_mount(self) -> None:
        self.push_screen(self._initial_screen)


@app.callback()
def _root(ctx: typer.Context) -> None:
    if ctx.invoked_subcommand is None:
        menu()


@app.command()
def menu() -> None:
    """Top-level menu: pick Dashboard / Logs / Comm / Faults."""
    from .main_app import RobotopMainApp

    RobotopMainApp().run()


@app.command()
def dash() -> None:
    """System status dashboard (MC / System / Faults / PMU / Orin)."""
    from .ros_bridge import RosBridge
    from .status.collector import StatusCollector
    from .status.screen import StatusScreen

    bridge = RosBridge.get()
    collector = StatusCollector()
    bridge.add_node(collector)
    try:
        _SingleScreenApp(StatusScreen(collector)).run()
    finally:
        bridge.shutdown()


@app.command(name="orin-monitor")
def orin_monitor(
    rate: float = typer.Option(1.0, help="publish rate in Hz"),
) -> None:
    """Run Orin stats publisher (foreground). Use tmux/systemd to background it."""
    from .orin_monitor_node import main as monitor_main

    raise typer.Exit(monitor_main(rate=rate))


@app.command()
def logs(
    follow: bool = typer.Option(
        False, "--follow", "-f", help="live tail (default unless --since is given)"
    ),
    since: Optional[str] = typer.Option(
        None, help="historical mode start time (e.g. '10m', '1h', '2026-06-05 14:00')"
    ),
    until: Optional[str] = typer.Option(None, help="historical mode end time"),
    grep: Optional[str] = typer.Option(None, help="initial filter text"),
    level: Optional[str] = typer.Option(
        None, help="initial level filter (e.g. WARN)"),
    add_path: List[Path] = typer.Option(
        [], "--add-path", help="extra log file to tail (e.g. ~/.aima/log/aima.log, ~/qiu/face_ui.log)"
    ),
    no_ros: bool = typer.Option(False, help="skip ~/.ros/log/ source"),
    rosout: bool = typer.Option(
        False, help="subscribe /rosout (live mode only)"),
) -> None:
    """Aggregate ROS / app logs with a filter DSL. AIMA log: pass it via --add-path."""
    from .logs.screen import LogScreen
    from .logs.sources import FileTailSource, RosSessionSource, RosoutTopicSource

    mode = "history" if since else "live"
    initial_filter = " ".join(t for t in [
        f"level>={level.upper()}" if level else "",
        grep or "",
    ] if t).strip()

    sources = []
    if not no_ros:
        sources.append(RosSessionSource())
    for p in add_path:
        sources.append(FileTailSource(Path(p).expanduser(), label=str(p)))

    bridge = None
    if rosout and mode == "live":
        from .ros_bridge import RosBridge
        from rclpy.node import Node

        bridge = RosBridge.get()
        node = Node("robotop_rosout_listener")
        bridge.add_node(node)
        sources.append(RosoutTopicSource(node))

    if not sources:
        typer.secho("no log sources enabled", fg="red")
        raise typer.Exit(2)

    since_dt = _parse_when(since) if since else None
    until_dt = _parse_when(until) if until else None

    try:
        _SingleScreenApp(
            LogScreen(
                sources=sources,
                mode=mode,
                since=since_dt,
                until=until_dt,
                initial_filter=initial_filter,
            )
        ).run()
    finally:
        if bridge is not None:
            bridge.shutdown()


@app.command()
def comm(
    include_hidden: bool = typer.Option(
        False, "--include-hidden", help="show hidden topics/services (path segments starting with `_`)"
    ),
) -> None:
    """List topics & services; measure hz on demand (press `m` on a row)."""
    from .comm.inspector import Inspector
    from .comm.screen import CommScreen
    from .ros_bridge import RosBridge

    bridge = RosBridge.get()
    insp = Inspector(include_hidden=include_hidden)
    bridge.add_node(insp)
    try:
        _SingleScreenApp(CommScreen(insp)).run()
    finally:
        insp.stop_all()
        bridge.shutdown()


@app.command()
def faults(
    history: bool = typer.Option(
        False, "--history", help="query historical events instead of live tail"),
    since: Optional[str] = typer.Option(
        None, help="history start (e.g. '10m', '1h')"),
    until: Optional[str] = typer.Option(None, help="history end"),
    state: str = typer.Option(
        "all", help="history state filter: all / active / recovered"),
    diag_code: List[int] = typer.Option(
        [], help="filter by diag_code (repeatable)"),
    alert_code: List[str] = typer.Option(
        [], help="filter by alert code string (repeatable)"),
    limit: int = typer.Option(200, help="max records to return"),
    asc: bool = typer.Option(
        False, help="ascending time order (default: descending)"),
) -> None:
    """HDS faults viewer (live by default, or --history for past records)."""
    try:
        from .status.collector import HDS_AVAILABLE
    except ImportError:
        HDS_AVAILABLE = False

    if not HDS_AVAILABLE:
        typer.secho(
            "HDS message types not installed in aimdk_msgs. "
            "Need to add interface/ros2/robot/hds/ to src/aimdk_msgs/interface/ and rebuild.",
            fg="red",
        )
        raise typer.Exit(2)

    if history:
        _run_faults_history(
            since=since,
            until=until,
            state=state,
            diag_codes=diag_code,
            alert_codes=alert_code,
            limit=limit,
            asc=asc,
        )
    else:
        _run_faults_live()


def _run_faults_live() -> None:
    from .faults.screen import LiveFaultsScreen
    from .ros_bridge import RosBridge
    from .status.collector import StatusCollector

    bridge = RosBridge.get()
    collector = StatusCollector()
    bridge.add_node(collector)
    try:
        _SingleScreenApp(LiveFaultsScreen(collector)).run()
    finally:
        bridge.shutdown()


def _run_faults_history(
    *,
    since: Optional[str],
    until: Optional[str],
    state: str,
    diag_codes: List[int],
    alert_codes: List[str],
    limit: int,
    asc: bool,
) -> None:
    from .faults.history import query_history
    from .faults.screen import HistoryFaultsScreen

    state_map = {"all": 0, "active": 1, "recovered": 2}
    if state not in state_map:
        raise typer.BadParameter(f"--state must be one of {list(state_map)}")

    start_ns = _to_ns(since) if since else 0
    end_ns = _to_ns(until) if until else 0

    summary_parts = [f"state={state}", f"limit={limit}"]
    if since:
        summary_parts.append(f"since={since}")
    if until:
        summary_parts.append(f"until={until}")
    if diag_codes:
        summary_parts.append(f"diag_codes={diag_codes}")
    if alert_codes:
        summary_parts.append(f"alert_codes={alert_codes}")

    typer.echo(f"querying GetHdsEventHistory ({' '.join(summary_parts)})...")
    records, total, has_more = query_history(
        start_time_ns=start_ns,
        end_time_ns=end_ns,
        diag_codes=diag_codes,
        alert_codes=alert_codes,
        state=state_map[state],
        order_desc=not asc,
        limit=limit,
    )
    _SingleScreenApp(HistoryFaultsScreen(
        records, total, has_more, " ".join(summary_parts))).run()


def _to_ns(s: str) -> int:
    dt = _parse_when(s)
    return int(dt.timestamp() * 1e9)


def _parse_when(s: str):
    from datetime import datetime, timedelta, timezone
    import re

    s = s.strip()
    if m := re.match(r"^(\d+)([smhd])$", s):
        n = int(m.group(1))
        unit = {"s": "seconds", "m": "minutes",
                "h": "hours", "d": "days"}[m.group(2)]
        return datetime.now(timezone.utc) - timedelta(**{unit: n})
    if s.lower() == "now":
        return datetime.now(timezone.utc)
    import dateparser

    dt = dateparser.parse(s)
    if dt is None:
        raise typer.BadParameter(f"could not parse time: {s!r}")
    if dt.tzinfo is None:
        dt = dt.astimezone()
    return dt.astimezone(timezone.utc)


if __name__ == "__main__":
    app()
