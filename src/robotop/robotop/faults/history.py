"""Synchronous client for /aimdk_5Fmsgs/srv/GetHdsEventHistory."""
from __future__ import annotations

import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node


def query_history(
    *,
    start_time_ns: int = 0,
    end_time_ns: int = 0,
    diag_codes: Optional[List[int]] = None,
    alert_codes: Optional[List[str]] = None,
    state: int = 0,
    order_desc: bool = True,
    offset: int = 0,
    limit: int = 200,
    timeout_sec: float = 30.0,
) -> Tuple[List[Dict[str, Any]], int, bool]:
    """Call GetHdsEventHistory once. Returns (records, total_count, has_more).

    Imports aimdk_msgs.srv.GetHdsEventHistory lazily so callers that don't have
    HDS msgs installed can still import this module.
    """
    from aimdk_msgs.srv import GetHdsEventHistory  # noqa: WPS433

    own_init = not rclpy.ok()
    if own_init:
        rclpy.init()
    node = Node("robotop_faults_history_client")
    try:
        client = node.create_client(
            GetHdsEventHistory, "/aimdk_5Fmsgs/srv/GetHdsEventHistory"
        )
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("GetHdsEventHistory service not available")

        req = GetHdsEventHistory.Request()
        now = node.get_clock().now()
        req.header.stamp.sec = now.nanoseconds // 1_000_000_000
        req.header.stamp.nanosec = now.nanoseconds % 1_000_000_000
        req.query.start_time = start_time_ns
        req.query.end_time = end_time_ns
        req.query.diag_codes = list(diag_codes or [])
        req.query.alert_codes = list(alert_codes or [])
        req.query.alert_locations = []
        req.query.state = state
        req.query.order_desc = order_desc
        req.query.offset = offset
        req.query.limit = limit

        fut = client.call_async(req)
        end = time.time() + timeout_sec
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if fut.done():
                break
        if not fut.done():
            raise RuntimeError("GetHdsEventHistory timed out")
        resp = fut.result()
        records: List[Dict[str, Any]] = []
        for r in resp.records:
            records.append({
                "id": int(r.id),
                "diag_code": int(r.diag_code),
                "alert_code": r.alert_code,
                "alert_location": r.alert_location,
                "first_timestamp": int(r.first_timestamp),
                "last_timestamp": int(r.last_timestamp),
                "recover_timestamp": int(r.recover_timestamp),
                "duration_ns": int(r.duration_ns),
                "recovered": bool(r.recovered),
            })
        return records, int(resp.total_count), bool(resp.has_more)
    finally:
        node.destroy_node()
        if own_init and rclpy.ok():
            rclpy.shutdown()
