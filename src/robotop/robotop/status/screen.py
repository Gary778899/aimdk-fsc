"""Textual dashboard for system status."""
from __future__ import annotations

import time

from textual.app import ComposeResult
from textual.containers import Grid
from textual.screen import Screen
from textual.widgets import Footer, Header, Static
from rich.markup import escape

from .collector import StatusCollector, StatusSnapshot, HDS_AVAILABLE


STALE_AFTER_S = 3.0


def _stale(last_at: float) -> bool:
    return last_at == 0.0 or (time.time() - last_at) > STALE_AFTER_S


def _fmt_mc(s: StatusSnapshot) -> str:
    if _stale(s.last_mc_at):
        return "[dim][stale - GetMcAction not responding][/dim]"
    desc = s.mc_action_desc or "?"
    name = s.mc_action_name or "?"
    desc_line = f"action: [b]{desc}[/b]"
    if name != desc and s.mc_action_value not in (0, None):
        desc_line += f" [dim](#{s.mc_action_value} = {name})[/dim]"
    elif s.mc_action_value is not None:
        desc_line += f" [dim](#{s.mc_action_value})[/dim]"
    status_color = {"RUNNING": "green", "TRANSITION": "yellow", "IDLE": "dim"}.get(
        s.mc_status_name or "", "white"
    )
    return f"{desc_line}\nstatus: [{status_color}]{s.mc_status_name}[/{status_color}]"


def _fmt_system(s: StatusSnapshot) -> str:
    if _stale(s.last_state_at):
        return "[dim][stale][/dim]"
    return (
        f"state: [b]{s.system_state}[/b]\n"
        f"status: {s.system_status_name} ({s.system_status_value})"
    )


def _fmt_pmu(s: StatusSnapshot) -> str:
    if _stale(s.last_pmu_at):
        return "[dim][stale - /aima/hal/pmu/state not received][/dim]"

    # Battery flow direction from BMS bits (bit 0 = charging, bit 2 = discharging)
    bms = s.bms_status_bits or 0
    if bms & 0x01:
        flow = "[green]⚡ charging[/green]"
    elif bms & 0x04:
        flow = f"[cyan]↓ discharging[/cyan]"
    else:
        flow = "[dim]idle[/dim]"

    bat_temp = f"  bat:{s.battery_temperature:.1f}°C" if s.battery_temperature is not None else ""

    lines = [
        f"battery: [b]{s.battery_pct}%[/b]  {flow}",
        f"  {s.battery_voltage:.2f}V  {s.battery_current:+.2f}A  {s.battery_power:.1f}W",
        f"orin:    {s.orin_voltage:.2f}V / {s.orin_current:.2f}A",
        f"rk3588:  {s.rk3588_voltage:.2f}V / {s.rk3588_current:.2f}A",
        f"48V bus: {s.bus_48v_voltage:.2f}V / {s.bus_48v_current:.2f}A",
        f"fan: {s.fan_pct}% ({s.fan_speed:.0f} RPM)  pmu:{s.pmu_temperature:.1f}°C{bat_temp}",
    ]

    warnings = _decode_pmu_warnings(s.pmu_bool_status, s.bms_status_bits)
    if warnings:
        lines.append(f"[red]⚠ {'  '.join(warnings)}[/red]")
    return "\n".join(lines)


_PMU_BIT_WARNINGS = [
    (6, "48v_over_current"),
    (7, "48v_over_temp"),
]
_BMS_BIT_WARNINGS = [
    (1, "charge_over_current"),
    (3, "discharge_over_current"),
    (4, "short_circuit"),
    (5, "cell_over_volt"),
    (6, "cell_under_volt"),
    (7, "pack_over_volt"),
    (8, "pack_under_volt"),
    (9, "cell_open"),
    (11, "discharge_over_temp"),
    (12, "charge_over_temp"),
    (13, "discharge_under_temp"),
    (14, "charge_under_temp"),
    (16, "cell_volt_diff_high"),
    (19, "mosfet_over_temp"),
]


def _decode_pmu_warnings(pmu_bits, bms_bits):
    out = []
    if pmu_bits is not None:
        for bit, label in _PMU_BIT_WARNINGS:
            if pmu_bits & (1 << bit):
                out.append(label)
    if bms_bits is not None:
        for bit, label in _BMS_BIT_WARNINGS:
            if bms_bits & (1 << bit):
                out.append(label)
    return out


def _fmt_orin(s: StatusSnapshot) -> str:
    if _stale(s.last_orin_at):
        return "[dim][stale - run `ros2 run robotop orin_monitor_node`][/dim]"
    parts = []
    if s.orin_cpu_pct_avg is not None:
        cores = f"x{s.orin_cpu_cores}" if s.orin_cpu_cores else ""
        parts.append(f"CPU: {s.orin_cpu_pct_avg:.1f}% {cores}")
    if s.orin_gpu_pct is not None:
        parts.append(f"GPU: {s.orin_gpu_pct:.1f}%")
    if s.orin_mem_used_mb is not None and s.orin_mem_total_mb is not None:
        parts.append(
            f"MEM: {s.orin_mem_used_mb:.0f}/{s.orin_mem_total_mb:.0f} MB")
    if s.orin_temp_cpu is not None:
        parts.append(f"T_cpu: {s.orin_temp_cpu:.1f}°C")
    if s.orin_temp_gpu is not None:
        parts.append(f"T_gpu: {s.orin_temp_gpu:.1f}°C")
    if s.orin_provider:
        parts.append(f"[dim](via {s.orin_provider})[/dim]")
    return "\n".join(parts)


def _fmt_faults(s: StatusSnapshot) -> str:
    if not s.hds_available:
        return "[dim](HDS msgs not in aimdk_msgs install)[/dim]"
    if _stale(s.last_diag_at) and _stale(s.last_alert_at):
        return "[dim][stale - no HDS topic received][/dim]"
    diag_n = len(s.diag_list)
    alert_n = len(s.alert_list)
    pp_n = len(s.postproc_list)
    diag_color = "green" if diag_n == 0 else "yellow" if diag_n < 3 else "red"
    alert_color = "green" if alert_n == 0 else "yellow" if alert_n < 3 else "red"
    lines = [
        f"diag:  [{diag_color}]{diag_n}[/{diag_color}]   alert: [{alert_color}]{alert_n}[/{alert_color}]   pp_sig: {pp_n}",
    ]
    if s.diag_list:
        d = s.diag_list[0]
        info = escape(d.get("info") or "")[:60]
        lines.append(f"[red]last diag[/red]  #{d['diag_code']}: {info}")
    if s.alert_list:
        a = s.alert_list[0]
        code = escape(a.get("code") or "")[:60]
        lines.append(f"[yellow]last alert[/yellow] {code}")
    return "\n".join(lines)


class Card(Static):
    def __init__(self, title: str, **kwargs) -> None:
        super().__init__("", **kwargs)
        self.border_title = title


class StatusScreen(Screen):
    DEFAULT_CSS = """
    Grid { grid-size: 3 2; grid-gutter: 1 2; padding: 1; }
    #card-orin { column-span: 2; }
    Card { border: round cyan; padding: 1 2; height: 100%; }
    """
    BINDINGS = [
        ("q", "back_or_quit", "Back"),
        ("r", "force_refresh", "Refresh"),
    ]

    def __init__(self, collector: StatusCollector) -> None:
        super().__init__()
        self._collector = collector

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with Grid():
            yield Card("MC Action", id="card-mc")
            yield Card("System", id="card-system")
            yield Card("Faults / HDS", id="card-faults")
            yield Card("PMU / Battery", id="card-pmu")
            yield Card("Orin", id="card-orin")
        yield Footer()

    def on_mount(self) -> None:
        self.sub_title = "dash"
        self.set_interval(0.5, self.action_force_refresh)
        self.action_force_refresh()

    def action_force_refresh(self) -> None:
        snap = self._collector.snapshot()
        self.query_one("#card-mc", Card).update(_fmt_mc(snap))
        self.query_one("#card-system", Card).update(_fmt_system(snap))
        self.query_one("#card-faults", Card).update(_fmt_faults(snap))
        self.query_one("#card-pmu", Card).update(_fmt_pmu(snap))
        self.query_one("#card-orin", Card).update(_fmt_orin(snap))

    def action_back_or_quit(self) -> None:
        if len(self.app.screen_stack) > 2:
            self.app.pop_screen()
        else:
            self.app.exit()


# Backwards-compat alias for the old subcommand entry point shape
StatusApp = StatusScreen
