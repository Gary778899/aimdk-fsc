"""Shared rclpy lifecycle: init once, spin in a background thread."""
from __future__ import annotations

import threading
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor


class RosBridge:
    _instance: Optional["RosBridge"] = None

    def __init__(self) -> None:
        if not rclpy.ok():
            rclpy.init()
        self._executor = MultiThreadedExecutor()
        self._nodes: list = []
        self._thread = threading.Thread(
            target=self._executor.spin, name="rclpy-spin", daemon=True
        )
        self._thread.start()

    @classmethod
    def get(cls) -> "RosBridge":
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def add_node(self, node) -> None:
        self._executor.add_node(node)
        self._nodes.append(node)

    def shutdown(self) -> None:
        for node in self._nodes:
            try:
                self._executor.remove_node(node)
                node.destroy_node()
            except Exception:
                pass
        self._executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        type(self)._instance = None
