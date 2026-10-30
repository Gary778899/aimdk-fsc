"""Top-level Textual app — RobotopMainApp + MenuScreen.

Hosts a navigable menu over all sub-screens. Selecting a menu item pushes the
matching Screen on the App's screen stack; the screen's `q` binding pops back
to this menu.

ROS resources (StatusCollector, comm Inspector) are created lazily on first
use and cached on the App instance — that keeps startup cheap when the user
only wants to look at logs (no ROS needed).
"""
from __future__ import annotations

from typing import Optional

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.screen import Screen
from textual.widgets import Footer, Header, Label, ListItem, ListView, Static

from .ros_bridge import RosBridge


_MENU_ITEMS = [
    ("dash", "System status (MC / System / Faults / PMU / Orin)"),
    ("logs", "Aggregate ROS / app logs, live follow"),
    ("comm", "topic / service list + on-demand rate"),
    ("faults", "HDS diagnostics / alerts, live table"),
]


class MenuScreen(Screen):
    DEFAULT_CSS = """
    ListView { height: auto; padding: 1 2; }
    ListItem { padding: 0 1; }
    #hint { dock: bottom; height: 2; padding: 0 1; color: $text-muted; }
    """
    BINDINGS = [
        Binding("q", "app.quit", "Quit", priority=True),
        Binding("escape", "app.quit", "Quit", priority=True),
    ]

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        yield ListView(
            *(
                ListItem(
                    Label(f"[b]{key.upper()}[/b]\n  [dim]{desc}[/dim]"), id=f"menu-{key}")
                for key, desc in _MENU_ITEMS
            ),
            id="menu",
        )
        yield Static("[dim]↑↓ select · Enter open · q quit[/dim]", id="hint")
        yield Footer()

    def on_mount(self) -> None:
        self.sub_title = "menu"
        self.query_one("#menu", ListView).focus()

    def on_list_view_selected(self, event: ListView.Selected) -> None:
        item_id = event.item.id or ""
        key = item_id.removeprefix("menu-")
        app: RobotopMainApp = self.app  # type: ignore[assignment]
        screen = app.build_screen(key)
        if screen is None:
            return
        app.push_screen(screen)


class RobotopMainApp(App):
    TITLE = "robotop"
    CSS = """
    Screen { background: $background; }
    """
    BINDINGS = [
        Binding("ctrl+c", "quit", "Quit", priority=True),
    ]

    def __init__(self) -> None:
        super().__init__()
        self._bridge: Optional[RosBridge] = None
        self._collector = None
        self._inspector = None

    def on_mount(self) -> None:
        self.push_screen(MenuScreen())

    def on_unmount(self) -> None:
        if self._inspector is not None:
            try:
                self._inspector.stop_all()
            except Exception:
                pass
        if self._bridge is not None:
            try:
                self._bridge.shutdown()
            except Exception:
                pass

    def _ensure_bridge(self) -> RosBridge:
        if self._bridge is None:
            self._bridge = RosBridge.get()
        return self._bridge

    def _ensure_collector(self):
        if self._collector is None:
            bridge = self._ensure_bridge()
            from .status.collector import StatusCollector
            self._collector = StatusCollector()
            bridge.add_node(self._collector)
        return self._collector

    def _ensure_inspector(self):
        if self._inspector is None:
            bridge = self._ensure_bridge()
            from .comm.inspector import Inspector
            self._inspector = Inspector(include_hidden=False)
            bridge.add_node(self._inspector)
        return self._inspector

    def build_screen(self, key: str) -> Optional[Screen]:
        if key == "dash":
            from .status.screen import StatusScreen
            return StatusScreen(self._ensure_collector())
        if key == "logs":
            from .logs.screen import LogScreen
            from .logs.sources import RosSessionSource
            return LogScreen(
                sources=[RosSessionSource()],
                mode="live",
                initial_filter="",
            )
        if key == "comm":
            from .comm.screen import CommScreen
            return CommScreen(self._ensure_inspector())
        if key == "faults":
            from .status.collector import HDS_AVAILABLE
            if not HDS_AVAILABLE:
                self._show_modal_error(
                    "HDS message types not installed in aimdk_msgs.\n"
                    "Add interface/ros2/robot/hds/ to src/aimdk_msgs/interface/ and rebuild."
                )
                return None
            from .faults.screen import LiveFaultsScreen
            return LiveFaultsScreen(self._ensure_collector())
        return None

    def _show_modal_error(self, message: str) -> None:
        self.bell()
        self.notify(message, severity="warning", timeout=6)
