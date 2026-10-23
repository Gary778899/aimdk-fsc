"""Publish Jetson Orin runtime stats as diagnostic_msgs/DiagnosticArray on /orin/stats.

Uses psutil for CPU/MEM, tegrastats popen output for GPU/temp/power.
"""
from __future__ import annotations

import argparse
import re
import subprocess
import threading
from typing import Dict

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class StatsProvider:
    _RE_GPU = re.compile(r"GR3D_FREQ (\d+)%")
    _RE_T_GPU = re.compile(r"GPU@([\d.]+)C")
    _RE_T_CPU = re.compile(r"CPU@([\d.]+)C")
    _RE_T_TH = re.compile(r"tj@([\d.]+)C|thermal@([\d.]+)C")
    _RE_PWR = re.compile(r"VDD_IN (\d+)/")

    def __init__(self) -> None:
        import psutil  # noqa: WPS433
        self._psutil = psutil
        # prime psutil so first read returns non-zero
        psutil.cpu_percent(percpu=True)
        self._latest = ""
        self._proc: subprocess.Popen | None = None
        try:
            self._proc = subprocess.Popen(
                ["tegrastats", "--interval", "1000"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                bufsize=1,
            )
            threading.Thread(target=self._reader, daemon=True).start()
        except FileNotFoundError:
            pass  # non-Jetson: psutil-only mode

    def name(self) -> str:
        return "psutil+tegrastats" if self._proc is not None else "psutil"

    def _reader(self) -> None:
        if self._proc.stdout is None:
            return
        for line in self._proc.stdout:
            self._latest = line.rstrip("\n")

    def read(self) -> Dict[str, str]:
        per_core = self._psutil.cpu_percent(percpu=True)
        mem = self._psutil.virtual_memory()
        out: Dict[str, str] = {
            "cpu_pct_avg": f"{(sum(per_core) / len(per_core)) if per_core else 0:.1f}",
            "cpu_cores": str(len(per_core)),
            "mem_used_mb": f"{mem.used / 1024 / 1024:.0f}",
            "mem_total_mb": f"{mem.total / 1024 / 1024:.0f}",
            "mem_pct": f"{mem.percent:.1f}",
        }
        for i, p in enumerate(per_core):
            out[f"cpu_pct_core_{i}"] = f"{p:.1f}"
        line = self._latest
        if line:
            if m := self._RE_GPU.search(line):
                out["gpu_pct"] = m.group(1)
            if m := self._RE_T_GPU.search(line):
                out["temp_gpu"] = m.group(1)
            if m := self._RE_T_CPU.search(line):
                out["temp_cpu"] = m.group(1)
            if m := self._RE_T_TH.search(line):
                out["temp_thermal"] = m.group(1) or m.group(2)
            if m := self._RE_PWR.search(line):
                out["power_total_mw"] = m.group(1)
        return out

    def close(self) -> None:
        try:
            if self._proc is not None:
                self._proc.terminate()
        except Exception:
            pass


class OrinMonitor(Node):
    def __init__(self, provider: StatsProvider, rate_hz: float) -> None:
        super().__init__("orin_monitor")
        self._provider = provider
        self._pub = self.create_publisher(DiagnosticArray, "/orin/stats", 10)
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f"orin_monitor up via provider={provider.name()} rate={rate_hz}Hz")

    def _tick(self) -> None:
        kv = self._provider.read()
        if not kv:
            return
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "orin"
        status.hardware_id = "jetson_orin"
        status.level = DiagnosticStatus.OK
        status.message = self._provider.name()
        status.values = [KeyValue(key=k, value=v) for k, v in kv.items()]
        msg.status = [status]
        self._pub.publish(msg)

    def destroy_node(self) -> bool:
        self._provider.close()
        return super().destroy_node()


def main(rate: float = 1.0) -> int:
    if rate <= 0:
        raise ValueError("rate must be > 0")
    rclpy.init()
    node = OrinMonitor(StatsProvider(), rate_hz=rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def _cli_entry() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--rate", type=float, default=1.0)
    args, _ = p.parse_known_args()
    main(rate=args.rate)


if __name__ == "__main__":
    _cli_entry()
