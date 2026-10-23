"""X2 Ultra left arm axis-7 test: default read-only; --execute enables one small round trip."""
import argparse
import math
import sys
import time

from aimdk_test.arm_test_core import Stream, decode_arm, decode_hand, decode_body
from aimdk_test.head_test_core import TestFailure
from aimdk_test.head_test_diagnostics import DiagnosticLog


UNRESOLVED = (
    'Execution profile: left wrist roll only, near the reviewed posture; confirm direction and clearance.',
    'Claw position replay in mode 1 is a position target, not a dedicated hold command.',
    'Empty load needs operator confirmation; feedback cannot prove the claws are empty.',
    'URS claw behavior before first command and after exit still requires verification.',
    'FSM raw value is recorded only; its conflicting encodings are not interpreted.',
)


def diagnostic_value(value):
    """Convert non-finite raw feedback to explicit strings so fault logs remain valid JSON."""
    if isinstance(value, float) and not math.isfinite(value):
        return repr(value)
    if isinstance(value, dict):
        return {key: diagnostic_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [diagnostic_value(item) for item in value]
    return value


def parser():
    """Return CLI options; only --execute enables one left wrist-roll round trip."""
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--execute', action='store_true', help='enable one left arm axis-7 round trip; confirm empty claws and clearance on site')
    result.add_argument('--amplitude-deg', type=float, default=2., help='signed wrist-roll offset: magnitude 0.5..2 degrees')
    result.add_argument('--leg-seconds', type=float, default=3., help='duration of each leg: 3..10 seconds')
    result.add_argument('--timeout', type=float, default=5., help='startup/service timeout: 1..10 s')
    result.add_argument('--observe-seconds', type=float, default=2., help='continuous valid baseline: 1..10 s')
    result.add_argument('--log-dir', help='default: detected project root log/')
    return result


class BaselinePort:
    """ROS adapter containing only three subscribers and a GetMcAction client."""

    def __init__(self, rclpy, node, options, diagnostics):
        """Construct read-only ROS resources with SDK feedback QoS; retain supplied diagnostics."""
        from aimdk_msgs.msg import JointStateArray, HandStateArray, McCommonState
        from aimdk_msgs.srv import GetMcAction
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        self.rclpy, self.node, self.options = rclpy, node, options
        self.diagnostics = diagnostics
        self.streams = dict(arm=Stream('arm', decode_arm), hand=Stream('hand', decode_hand),
                            mc=Stream('mc', decode_body))
        self.last_log = dict.fromkeys(self.streams, float('-inf'))
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions = [
            node.create_subscription(JointStateArray, '/aima/hal/joint/arm/state', self.on_arm, qos),
            node.create_subscription(HandStateArray, '/aima/hal/joint/hand/state', self.on_hand, qos),
            node.create_subscription(McCommonState, '/aima/mc/common/state', self.on_mc, qos),
        ]
        self.get_type = GetMcAction
        self.client = node.create_client(GetMcAction, '/aimdk_5Fmsgs/srv/GetMcAction')

    def receive(self, name, msg):
        """Decode feedback, preserving original header and received data even when validation fails."""
        from rosidl_runtime_py.convert import message_to_ordereddict
        now = time.monotonic()
        stream = self.streams[name]
        stream.accept(msg, now)
        if stream.error or now - self.last_log[name] >= .1:
            self.last_log[name] = now
            self.diagnostics.record('feedback', stream=name, received_monotonic_s=now,
                                    ros_time_s=self.node.get_clock().now().nanoseconds / 1e9,
                                    raw=diagnostic_value(message_to_ordereddict(msg)),
                                    error=str(stream.error) if stream.error else None)
        if stream.error:
            raise TestFailure(f'{name}: {stream.error}')

    def on_arm(self, msg):
        """Accept one full dual-arm feedback message."""
        self.receive('arm', msg)

    def on_hand(self, msg):
        """Accept one dual-claw feedback message."""
        self.receive('hand', msg)

    def on_mc(self, msg):
        """Accept standing/mode feedback without interpreting FSM codes."""
        self.receive('mc', msg)

    def spin(self):
        """Process callbacks for at most 10 ms; track ROS clock independently for every stream."""
        self.rclpy.spin_once(self.node, timeout_sec=.01)
        if not self.rclpy.ok():
            raise TestFailure('ROS context stopped')
        now, ros_now = time.monotonic(), self.node.get_clock().now().nanoseconds / 1e9
        for stream in self.streams.values():
            stream.monitor.observe_clock(now, ros_now)
            if stream.error:
                raise TestFailure(f'{stream.name}: {stream.error}')

    def checked(self):
        """Return simultaneously fresh arm, hand and MC samples or raise TestFailure."""
        now, ros_now = time.monotonic(), self.node.get_clock().now().nanoseconds / 1e9
        return {name: stream.checked(now, ros_now) for name, stream in self.streams.items()}

    def mode(self):
        """Bound GetMcAction availability/response waits and require STAND_DEFAULT / RUNNING."""
        request = self.get_type.Request()
        request.request.header.stamp = self.node.get_clock().now().to_msg()
        deadline = time.monotonic() + self.options.timeout
        while not self.client.service_is_ready():
            if time.monotonic() >= deadline:
                raise TestFailure('GetMcAction service unavailable')
            self.spin()
        self.diagnostics.record('mode_query_sent')
        future = self.client.call_async(request)
        try:
            while not future.done():
                if time.monotonic() >= deadline:
                    raise TestFailure('GetMcAction response timeout')
                self.spin()
            response = future.result()
        finally:
            if not future.done():
                self.client.remove_pending_request(future)
        if response is None:
            raise TestFailure('GetMcAction returned no response')
        self.diagnostics.record('mode_response', code=response.header.code,
                                action=response.info.action_desc, status=response.info.status.value)
        if (response.header.code != 0 or response.info.action_desc != 'STAND_DEFAULT'
                or response.info.status.value != 100):
            raise TestFailure('GetMcAction must confirm STAND_DEFAULT / RUNNING')

    def collect(self):
        """Wait for advancing sources, query mode, then collect a bounded continuously valid baseline."""
        deadline = time.monotonic() + self.options.timeout
        while not all(s.monitor.latest is not None and s.monitor.latest.stamp > s.first_stamp
                      for s in self.streams.values()):
            if time.monotonic() >= deadline:
                raise TestFailure('startup timeout: all arm/hand/MC source stamps must advance')
            self.spin()
        self.checked()
        self.mode()
        self.checked()
        deadline = time.monotonic() + self.options.observe_seconds
        initial = self.checked()
        low = {name: list(value.positions) for name, value in initial.items()}
        high = {name: list(value.positions) for name, value in initial.items()}
        stationary = all(h['stationary_candidate'] for h in initial['hand'].data['hands'])
        while time.monotonic() < deadline:
            self.spin()
            values = self.checked()
            stationary = stationary and all(h['stationary_candidate'] for h in values['hand'].data['hands'])
            for name, value in values.items():
                low[name] = [min(a, b) for a, b in zip(low[name], value.positions)]
                high[name] = [max(a, b) for a, b in zip(high[name], value.positions)]
        self.mode()
        values = self.checked()
        stationary = stationary and all(h['stationary_candidate'] for h in values['hand'].data['hands'])
        for name, value in values.items():
            low[name] = [min(a, b) for a, b in zip(low[name], value.positions)]
            high[name] = [max(a, b) for a, b in zip(high[name], value.positions)]
        ranges = {name: [b-a for a, b in zip(low[name], high[name])] for name in low}
        self.diagnostics.record('baseline', samples={name: value.data for name, value in values.items()},
                                position_ranges=ranges, claws_stationary_candidate=stationary,
                                unresolved=list(UNRESOLVED))
        print('STAND_DEFAULT / RUNNING; body=STAND; source/local clocks and advancing feedback checked.')
        for index, joint in enumerate(values['arm'].data['joints']):
            print(f"arm[{index:2d}] {joint['name']}: {joint['position']:.6f} rad "
                  f"({math.degrees(joint['position']):.3f} deg), velocity={joint['velocity']:.6f}, "
                  f"error={joint['error_code']}, observed range={ranges['arm'][index]:.6f} rad")
        for index, hand in enumerate(values['hand'].data['hands']):
            print(f"{hand['side']} claw {hand['name']!r}: type={hand['type']}, opening={hand['position']:.6f}, "
                  f"velocity={hand['velocity']:.6f}, state={hand['state']}, fault={hand['faultcode']}, "
                  f"observed range={ranges['hand'][index]:.6f}")
        for name, value in values.items():
            print(f'{name}: source age={self.node.get_clock().now().nanoseconds/1e9-value.stamp:.4f}s, '
                  f'receive age={time.monotonic()-value.received:.4f}s; header={value.data["header"]}')
        print(f'Claws stationary candidate throughout observation: {stationary}; FSM raw={values["mc"].data["fsm_raw"]}')
        for item in UNRESOLVED:
            print(f'PENDING: {item}')
        print('Read-only baseline collected. No command publisher or SetMcAction client created during collection.')
        return dict(samples=values, position_ranges=ranges, stationary=stationary)


def main(args=None):
    """Collect baseline, optionally execute one wrist-roll test; return 0, failure 1 or interruption 130."""
    options = parser().parse_args(args)
    diagnostics = None
    node = None
    initialized = False
    try:
        for name, value in (('timeout', options.timeout), ('observe-seconds', options.observe_seconds)):
            if not math.isfinite(value) or not 1 <= value <= 10:
                raise TestFailure(f'{name} must be 1..10 seconds')
        from aimdk_test.arm_test_core import validate_motion_options, execute_arm_test
        amplitude = math.radians(options.amplitude_deg)
        validate_motion_options(amplitude, options.leg_seconds)
        diagnostics = DiagnosticLog(options.log_dir, prefix='arm_test')
        diagnostics.record('run_start', read_only=not options.execute, timeout_s=options.timeout,
                           observe_seconds=options.observe_seconds, amplitude_deg=options.amplitude_deg,
                           leg_seconds=options.leg_seconds)
        print(f'Diagnostic log: {diagnostics.path}', flush=True)
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = rclpy.create_node('arm_baseline_acceptance')
        if node.get_parameter('use_sim_time').value:
            raise TestFailure('real-robot tool refuses use_sim_time')
        port = BaselinePort(rclpy, node, options, diagnostics)
        baseline = port.collect()
        if options.execute:
            if not baseline['stationary'] or any(v > .005 for v in baseline['position_ranges']['hand']):
                raise TestFailure('claws were not stationary during baseline observation')
            if any(v > math.radians(.2) for v in baseline['position_ranges']['arm']):
                raise TestFailure('arms were not stationary during baseline observation')
            from aimdk_test.arm_test_motion import ArmExecution
            execute_arm_test(ArmExecution(port), amplitude, options.leg_seconds)
            print('PASS: left arm axis 7 outbound/return endpoints measured; STAND_DEFAULT confirmed; claw openings checked.')
        else:
            print('READ ONLY: no commands sent; use --execute only after confirming empty claws and clearance.')
        diagnostics.record('run_pass', read_only=not options.execute)
        return 0
    except KeyboardInterrupt:
        if diagnostics:
            diagnostics.record('run_interrupted')
        return 130
    except Exception as error:
        if diagnostics:
            diagnostics.record('run_failed', error=str(error), error_type=type(error).__name__)
        print(f'FAIL: {error}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        if diagnostics:
            diagnostics.close()


if __name__ == '__main__':
    sys.exit(main())
