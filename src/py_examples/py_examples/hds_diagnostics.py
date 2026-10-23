#!/usr/bin/env python3
"""
HDS diagnostics tool - two modes:

  echo       Subscribe to diag / alert / post_proc topics in real time.
  history    Query historical alert / diagnostic events via service.

See docs/cn/example/Python.md section "Subscribe HDS diagnostics /
query history events" for full usage and field descriptions.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import aimdk_msgs.msg as aimdk_msg
import aimdk_msgs.srv as aimdk_srv


DIAG_TYPE_NAMES = {
    aimdk_msg.DiagnosticType.NORMAL: "NORMAL",
    aimdk_msg.DiagnosticType.TRIGGER_APPEAR: "APPEAR",
    aimdk_msg.DiagnosticType.TRIGGER_DISAPPEAR: "DISAPPEAR",
}

STATE_NAMES = {
    aimdk_msg.HdsEventQuery.STATE_ALL: "ALL",
    aimdk_msg.HdsEventQuery.STATE_ACTIVE: "ACTIVE",
    aimdk_msg.HdsEventQuery.STATE_RECOVERED: "RECOVERED",
}


# ---------- echo mode ----------

class HdsDiagnosticsEcho(Node):
    """Subscribe to three HDS topics and print incoming messages."""

    def __init__(self):
        super().__init__('hds_diagnostics_echo')

        # HDS publishers use BEST_EFFORT; match it to receive messages.
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            aimdk_msg.DiagnosticInfoArray,
            '/aima/hds/diag_code_list',
            self.on_diagnostics, qos)

        self.create_subscription(
            aimdk_msg.AlertCodeArray,
            '/aima/hds/alert_code_list',
            self.on_alerts, qos)

        self.create_subscription(
            aimdk_msg.PostProcSigArray,
            '/aima/hds/post_proc/signal',
            self.on_post_proc, qos)

        self.get_logger().info(
            'Subscribed: diag_code_list / alert_code_list / post_proc/signal')

    def on_diagnostics(self, msg: aimdk_msg.DiagnosticInfoArray):
        if not msg.diagnostics:
            return
        lines = [f'[Diagnostics] count={len(msg.diagnostics)}']
        for d in msg.diagnostics:
            type_name = DIAG_TYPE_NAMES.get(
                d.type.value, f'UNKNOWN({d.type.value})')
            lines.append(
                f'  diag_code=0x{d.diag_code:010X}  type={type_name}  '
                f'module_id={d.module_id}  ts={d.timestamp}  info="{d.info}"')
        self.get_logger().info('\n'.join(lines))

    def on_alerts(self, msg: aimdk_msg.AlertCodeArray):
        if not msg.list:
            return
        lines = [f'[Alerts] count={len(msg.list)}']
        for a in msg.list:
            lines.append(
                f'  code="{a.code}"  ts={a.timestamp}  '
                f'first_ts={a.first_timestamp}')
        self.get_logger().info('\n'.join(lines))

    def on_post_proc(self, msg: aimdk_msg.PostProcSigArray):
        if not msg.post_proc_sig:
            return
        lines = [f'[PostProc] count={len(msg.post_proc_sig)}']
        for p in msg.post_proc_sig:
            s = p.sig_set
            lines.append(
                f'  diag_code=0x{p.diag_code:010X}  ts={p.timestamp}  '
                f'robot={s.robot_sig} motion={s.motion_sig} '
                f'sound={s.sound_sig} led={s.led_sig} facial={s.facial_sig} '
                f'funcdisable=0x{s.funcdisable_sig:016X}')
        self.get_logger().info('\n'.join(lines))


def run_echo(_args) -> int:
    node = HdsDiagnosticsEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    return 0


# ---------- history mode ----------

class HdsHistoryClient(Node):
    """Call the GetHdsEventHistory service to query historical events."""

    SERVICE_NAME = '/aimdk_5Fmsgs/srv/GetHdsEventHistory'

    def __init__(self):
        super().__init__('hds_history_client')
        self.client = self.create_client(
            aimdk_srv.GetHdsEventHistory, self.SERVICE_NAME)

    def call(self, query: aimdk_msg.HdsEventQuery, timeout_sec: float = 10.0):
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f'Service {self.SERVICE_NAME} not available')
            return None

        request = aimdk_srv.GetHdsEventHistory.Request()
        now = self.get_clock().now()
        request.header.stamp.sec = now.nanoseconds // 1_000_000_000
        request.header.stamp.nanosec = now.nanoseconds % 1_000_000_000
        request.query = query

        future = self.client.call_async(request)
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                return future.result()

        self.get_logger().error('Service call timeout')
        future.cancel()
        return None


def build_query(args) -> aimdk_msg.HdsEventQuery:
    q = aimdk_msg.HdsEventQuery()
    q.start_time = args.start_ns
    q.end_time = args.end_ns
    q.diag_codes = args.diag_codes
    q.alert_codes = args.alert_codes
    q.alert_locations = []
    q.state = args.state
    q.order_desc = args.desc
    q.offset = args.offset
    q.limit = args.limit
    return q


def print_records(response) -> None:
    print(f'total_count = {response.total_count}, '
          f'returned = {len(response.records)}, '
          f'has_more = {response.has_more}')
    if not response.records:
        return
    print('-' * 80)
    for r in response.records:
        recovered = 'YES' if r.recovered else 'NO'
        duration_s = r.duration_ns / 1e9
        print(f'id={r.id}  diag=0x{r.diag_code:010X}  '
              f'alert="{r.alert_code}"')
        print(f'  first_ts={r.first_timestamp}  '
              f'last_ts={r.last_timestamp}')
        print(f'  recovered={recovered}  '
              f'recover_ts={r.recover_timestamp}  '
              f'duration={duration_s:.3f}s')
        if r.alert_location:
            print(f'  location="{r.alert_location}"')
        print('-' * 80)


def run_history(args) -> int:
    node = HdsHistoryClient()
    try:
        query = build_query(args)
        node.get_logger().info(
            f'Querying history: state={STATE_NAMES.get(query.state)}, '
            f'offset={query.offset}, limit={query.limit}, '
            f'desc={query.order_desc}')

        response = node.call(query)
        if response is None:
            return 1
        print_records(response)
        return 0
    finally:
        node.destroy_node()


# ---------- entry point ----------

def parse_args():
    parser = argparse.ArgumentParser(
        description='HDS diagnostics tool - echo topics or query history')
    sub = parser.add_subparsers(dest='mode', required=True)

    sub.add_parser(
        'echo', help='subscribe diag/alert/post_proc topics and print live')

    h = sub.add_parser('history', help='query GetHdsEventHistory service')
    h.add_argument('--start-ns', type=int, default=0,
                   help='start timestamp (ns), 0 means unlimited')
    h.add_argument('--end-ns', type=int, default=0,
                   help='end timestamp (ns), 0 means unlimited')
    h.add_argument('--diag-codes', type=lambda s: int(s, 0), nargs='*',
                   default=[], help='filter diag codes (e.g. 0x6200000001)')
    h.add_argument('--alert-codes', nargs='*', default=[],
                   help='filter alert code strings')
    h.add_argument('--state', type=int, default=aimdk_msg.HdsEventQuery.STATE_ALL,
                   choices=[0, 1, 2],
                   help='0=ALL, 1=ACTIVE, 2=RECOVERED')
    h.add_argument('--offset', type=int, default=0)
    h.add_argument('--limit', type=int, default=20)
    h.add_argument('--desc', action='store_true',
                   help='order by time descending')

    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    try:
        if args.mode == 'echo':
            return run_echo(args)
        if args.mode == 'history':
            return run_history(args)
        return 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
