"""X2 Ultra yaw acceptance tool. Default read-only; ROS imports are lazy."""
import argparse
import math
import sys
import time

from aimdk_test.head_test_core import (
    FeedbackMonitor, HEAD, STAND, TestFailure, decode_feedback, execute_test,
    target_at, tracking_check, validate_options, EndpointMonitor,
)


def parser():
    """Return the CLI parser; motion requires the explicit --execute flag."""
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--execute', action='store_true', help='explicitly enable one yaw round trip')
    result.add_argument('--amplitude-deg', type=float, default=3., help='signed yaw offset, magnitude 1..5 deg')
    result.add_argument('--leg-seconds', type=float, default=3., help='each motion leg: 3..10 s')
    result.add_argument('--timeout', type=float, default=5., help='bounded startup/service/transition timeout: 1..10 s')
    result.add_argument('--log-dir', help='diagnostic directory; default: workspace log/')
    return result


def make_port(rclpy, node, args):
    """Build the ROS adapter from rclpy, node and CLI options; return a Port."""
    from aimdk_msgs.msg import JointStateArray, McCommonState, McBodyPoseStatus, McActionStatus, UpperBodyCommandArray
    from aimdk_msgs.srv import GetMcAction, SetMcAction
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

    class Port:
        def __init__(self):
            """Initialize feedback subscriptions and mode clients; enable commands only for execution."""
            self.monitor = FeedbackMonitor()
            self.diagnostics = getattr(args, 'diagnostics', None)
            self.last_sample_log = float('-inf')
            self.last_publish_at = None
            self.last_command = None
            self.last_mc_state = None
            self.feedback_error = None
            self.body_error = None
            self.serial = 0
            self.observed_names = []
            self.body = None
            self.body_received = 0.
            self.body_stamp = None
            self.body_advanced = 0.
            self.mode_value = None
            self.mode_received = 0.
            self.pending = None
            self.pending_at = 0.
            self.sequence = 0
            self.head_request_sent = False
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.subscription = node.create_subscription(JointStateArray,
                '/aima/hal/joint/head/state', self.on_feedback, qos)
            self.body_subscription = node.create_subscription(McCommonState,
                '/aima/mc/common/state', self.on_body, qos)
            self.get = node.create_client(GetMcAction, '/aimdk_5Fmsgs/srv/GetMcAction')
            self.set = None
            self.publisher = None
            if args.execute:
                self.set = node.create_client(SetMcAction, '/aimdk_5Fmsgs/srv/SetMcAction')
                command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                        durability=DurabilityPolicy.VOLATILE)
                self.publisher = node.create_publisher(UpperBodyCommandArray,
                    '/mc/upper_body_command', command_qos)

        def event(self, name, **fields):
            """Buffer a diagnostic event with the latest feedback and ownership ages."""
            if self.diagnostics is None:
                return
            now = time.monotonic()
            fb = self.monitor.latest
            ros_now = node.get_clock().now().nanoseconds / 1e9
            self.diagnostics.record(name, ros_time_s=ros_now,
                mode=self.mode_value, mode_age_s=now-self.mode_received,
                head_position_rad=list(fb.positions) if fb else None,
                head_source_stamp_s=fb.stamp if fb else None,
                head_source_age_s=ros_now-fb.stamp if fb else None,
                head_receive_age_s=now-fb.received if fb else None,
                head_source_frozen_s=(now-self.monitor.source_advanced_at
                                      if self.monitor.source_advanced_at is not None else None),
                ros_frozen_s=(now-self.monitor.clock_advanced_at
                              if self.monitor.clock_advanced_at is not None else None),
                mc_receive_age_s=now-self.body_received if self.body else None,
                mc_source_stamp_s=self.body_stamp,
                mc_source_age_s=ros_now-self.body_stamp if self.body_stamp is not None else None,
                mc_source_frozen_s=now-self.body_advanced if self.body else None,
                body=self.body.body_status.value if self.body else None,
                fsm_raw=self.body.fsm_state.current_state if self.body else None,
                last_command_rad=self.last_command, **fields)

        def stop_commands(self, reason):
            """Mark the last published command and gap before a normal or aborted handoff."""
            self.event('commands_stopped', reason=reason, published_count=self.sequence,
                       last_sequence=self.sequence-1 if self.sequence else None,
                       last_publish_monotonic_s=self.last_publish_at,
                       gap_since_last_publish_s=(time.monotonic()-self.last_publish_at
                                                 if self.last_publish_at is not None else None))

        def on_body(self, msg):
            """Record an MC message and latch any source timestamp regression."""
            now = time.monotonic()
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self.body_stamp is not None and stamp < self.body_stamp:
                self.body_error = TestFailure('MC state timestamp moved backwards')
            if self.body_stamp is None or stamp > self.body_stamp:
                self.body_advanced = now
            self.body_stamp = stamp
            self.body, self.body_received = msg, now
            state = (msg.action_info.action_desc, msg.action_info.status.value,
                     msg.body_status.value, msg.fsm_state.current_state)
            if state != self.last_mc_state:
                self.last_mc_state = state
                self.event('mc_state_changed', action=state[0], action_status=state[1])

        def on_feedback(self, msg):
            """Decode head feedback and latch TestFailure on invalid joint data."""
            self.observed_names = [joint.name for joint in msg.joints]
            self.serial += 1
            try:
                self.monitor.accept(decode_feedback(msg, time.monotonic(), self.serial))
                if time.monotonic() - self.last_sample_log >= .1:
                    self.last_sample_log = time.monotonic()
                    self.event('feedback_sample', feedback_serial=self.serial)
                # Faults remain latched for this process; a later valid frame cannot mask one.
            except TestFailure as error:
                self.feedback_error = error
                self.event('head_feedback_error', error=str(error))

        def spin(self, seconds=.005):
            """Process ROS callbacks for at most seconds; raise TestFailure if ROS stops."""
            rclpy.spin_once(node, timeout_sec=seconds)
            self.monitor.observe_clock(time.monotonic(), node.get_clock().now().nanoseconds / 1e9)
            if not rclpy.ok():
                raise TestFailure('ROS context stopped')

        def call(self, client, request):
            """Send a service request and return its response within the configured timeout. Raise TestFailure on unavailability, timeout or an empty response."""
            if not client.wait_for_service(timeout_sec=args.timeout):
                raise TestFailure(f'service unavailable: {client.srv_name}')
            started = time.monotonic()
            self.event('service_send', service=client.srv_name,
                       requested_mode=request.command.action_desc if client is self.set else None)
            future = client.call_async(request)
            if client is self.set and request.command.action_desc == HEAD:
                self.head_request_sent = True
            deadline = time.monotonic() + args.timeout
            while not future.done() and time.monotonic() < deadline:
                self.spin()
            if not future.done():
                self.event('service_timeout', service=client.srv_name, duration_s=time.monotonic()-started)
                client.remove_pending_request(future)
                raise TestFailure('service response timeout; remote completion is unknown')
            result = future.result()
            self.event('service_response', service=client.srv_name,
                       duration_s=time.monotonic()-started)
            if result is None:
                raise TestFailure('empty service response')
            return result

        def mode(self, allow_transition=False):
            """Query GetMcAction and return its RUNNING mode; optionally allow TRANSITION as None."""
            request = GetMcAction.Request()
            request.request.header.stamp = node.get_clock().now().to_msg()
            response = self.call(self.get, request)
            return self.record_mode(response, allow_transition)

        def record_mode(self, response, allow_transition=False):
            """Validate a GetMcAction response and cache its RUNNING mode. Return None for an allowed transition; raise TestFailure for errors or other statuses."""
            self.event('mode_response', action=response.info.action_desc,
                       action_status=response.info.status.value, response_code=response.header.code)
            if response.header.code != 0:
                raise TestFailure('mode query response error')
            if allow_transition and response.info.status.value == McActionStatus.TRANSITION:
                return None
            if response.info.status.value != McActionStatus.RUNNING:
                raise TestFailure('mode query failed or action is not RUNNING')
            self.mode_value = response.info.action_desc
            self.mode_received = time.monotonic()
            return self.mode_value

        def feedback(self):
            """Return checked head feedback; reject stale MC data or a non-standing body. FSM values are not interpreted."""
            if self.body_error:
                raise self.body_error
            if self.feedback_error:
                raise self.feedback_error
            fb = self.monitor.checked(time.monotonic(), node.get_clock().now().nanoseconds / 1e9)
            body = self.body
            if body is None or time.monotonic() - self.body_received > .3:
                raise TestFailure(f'MC receive gap={time.monotonic()-self.body_received:.6f}s > 0.300000s or state unavailable')
            stamp = body.header.stamp.sec + body.header.stamp.nanosec * 1e-9
            age = node.get_clock().now().nanoseconds / 1e9 - stamp
            if not -.05 <= age <= .3 or time.monotonic() - self.body_advanced > .3:
                raise TestFailure(f'MC source age={age:.6f}s (allowed -0.05..0.3s); source frozen={time.monotonic()-self.body_advanced:.6f}s (limit 0.3s)')
            if body.body_status.value != McBodyPoseStatus.MC_BODY_POSE_STAND:
                raise TestFailure('MC body state does not report STAND')
            return fb

        def transition(self, mode, expected):
            """Switch to mode only from expected and confirm RUNNING; raise TestFailure on failure."""
            if self.set is None:
                raise TestFailure('read-only mode cannot switch modes')
            # Recheck current state immediately before sending each transition.
            current = self.mode()
            if current != expected:
                raise TestFailure(f'control mode changed to {current}; refusing takeover')
            self.feedback()
            self.event('transition_begin', target_mode=mode, expected_mode=expected)
            if mode == STAND:
                self.stop_commands('normal_handoff')
            request = SetMcAction.Request()
            request.header.stamp = node.get_clock().now().to_msg()
            request.source = 'node.head_yaw_acceptance'
            request.command.action_desc = mode
            response = self.call(self.set, request)
            self.event('transition_accepted_response', target_mode=mode,
                       response_code=response.response.header.code,
                       response_status=response.response.status.value,
                       response_message=response.response.message)
            if response.response.header.code != 0 or response.response.status.value != 1:
                raise TestFailure(f'mode switch rejected: {response.response.message}')
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline:
                if self.mode(allow_transition=True) == mode:
                    self.feedback()
                    self.event('transition_confirmed', target_mode=mode)
                    return
                self.spin(.02)
            raise TestFailure('mode switch confirmation timed out')

        def new_feedback(self):
            """Return advancing head feedback captured after mode confirmation or raise TestFailure."""
            after = time.monotonic()
            previous = self.monitor.latest
            deadline = after + args.timeout
            while time.monotonic() < deadline:
                self.spin()
                fb = self.feedback()
                if fb.received > after and fb.serial > previous.serial and fb.stamp > previous.stamp:
                    self.event('start_captured', feedback_serial=fb.serial)
                    return fb
            raise TestFailure('no advancing head feedback after mode confirmation')

        def poll_mode(self):
            """Poll HEAD_ONLY ownership without blocking; raise TestFailure on loss or expiry."""
            now = time.monotonic()
            if self.pending is not None and now - self.pending_at > .3:
                self.get.remove_pending_request(self.pending)
                self.pending = None
                raise TestFailure(f'mode polling response expired: age={now-self.pending_at:.6f}s > 0.300000s')
            if self.pending is not None and self.pending.done():
                self.event('async_mode_response', duration_s=now-self.pending_at)
                result = self.pending.result()
                self.pending = None
                if result is None:
                    raise TestFailure('empty async mode response')
                if self.record_mode(result) != HEAD:
                    raise TestFailure('HEAD_ONLY lost; stop publishing')
            if now - self.mode_received > .4:
                raise TestFailure(f'mode confirmation expired: age={now-self.mode_received:.6f}s > 0.400000s')
            if self.pending is None and now - self.mode_received >= .1:
                request = GetMcAction.Request()
                request.request.header.stamp = node.get_clock().now().to_msg()
                self.pending = self.get.call_async(request)
                self.pending_at = now
                self.event('async_mode_send')

        def publish(self, position):
            """Publish a two-slot head position target in radians using the HEAD_ONLY payload."""
            msg = UpperBodyCommandArray()
            # Documented zero stamp means MC receive time. Feedback stamps stay intact.
            msg.header.frame_id = 'mc_upper_body'
            msg.header.sequence = self.sequence
            self.sequence += 1
            msg.source = 'head_yaw_acceptance'
            msg.head_pos = list(position)
            msg.arm_pos = [0.] * 14  # ignored ONLY in confirmed HEAD_ONLY
            msg.hand_sub_mode = 0
            msg.hand_pos = []
            self.publisher.publish(msg)
            published_at = time.monotonic()
            gap = published_at-self.last_publish_at if self.last_publish_at is not None else None
            self.last_publish_at = published_at
            self.last_command = list(position)
            self.event('command_published', command_sequence=msg.header.sequence,
                       publish_gap_s=gap, publish_monotonic_s=published_at)
            if self.sequence == 1:
                self.event('first_command', command_sequence=msg.header.sequence)

        def motion(self, start, amplitude, duration):
            # Publish q0 exactly, then require a newer measured sample before moving.
            """Execute a yaw round trip from start by amplitude radians over duration seconds per leg. Raise TestFailure on feedback, ownership, tracking or timing failures."""
            before = self.feedback()
            if any(abs(a-b) > math.radians(.5) for a, b in zip(before.positions, start)):
                raise TestFailure('head drifted from captured start before first command')
            self.poll_mode()
            self.publish(start)
            began_capture = time.monotonic()
            next_capture = began_capture + .02
            while True:
                self.spin(.002)
                now = time.monotonic()
                self.poll_mode()
                sample = self.feedback()
                if sample.serial > before.serial and sample.stamp > before.stamp:
                    if any(abs(a-b) > math.radians(.5) for a, b in zip(sample.positions, start)):
                        raise TestFailure('head moved away from captured start before first-frame validation')
                    break
                if now - began_capture > .2:
                    raise TestFailure('no advancing feedback after first command')
                if now >= next_capture:
                    self.publish(start)
                    next_capture = now + .02
            # Fixed-rate event loop; asynchronous mode polling never blocks publishing.
            for leg, end in enumerate(((start[0] + amplitude, start[1]), start), start=1):
                origin = start if end != start else (start[0] + amplitude, start[1])
                delta = end[0] - origin[0]
                began = time.monotonic()
                next_tick = began
                self.event('leg_begin', leg=leg, target_rad=list(end))
                settling = EndpointMonitor(end, began + duration)
                while True:
                    self.spin(.002)
                    now = time.monotonic()
                    self.poll_mode()
                    actual = self.feedback().positions
                    elapsed = now - began
                    # Single outbound quintic for this segment, then hold endpoint.
                    position = target_at(origin, delta, duration, min(elapsed, duration))
                    tracking_check(actual, position)
                    if now >= next_tick:
                        if now - next_tick > .06:
                            raise TestFailure(f'publisher scheduling gap exceeded 80 ms: lateness={now-next_tick:.6f}s > 0.060000s')
                        self.publish(position)
                        next_tick = now + .02
                    if elapsed >= duration:
                        if settling.checked(actual, now):
                            self.event('endpoint_confirmed', leg=leg, target_rad=list(end))
                            break
            self.stop_commands('round_trip_complete')

        def abort_handoff(self):
            """Attempt bounded handoff after a HEAD_ONLY request; report an unconfirmed handoff."""
            if not self.head_request_sent:
                print('No HEAD_ONLY request was sent; leaving current controller untouched.')
                return
            self.stop_commands('abort')
            try:
                if self.pending is not None:
                    self.get.remove_pending_request(self.pending)
                    self.pending = None
                # Fresh query, not a cached HEAD_ONLY claim. No return trajectory.
                if self.mode() != HEAD:
                    raise TestFailure('another mode/controller owns control')
                self.feedback()
                self.transition(STAND, expected=HEAD)
                self.event('abort_handoff_confirmed')
                print('Aborted motion; STAND_DEFAULT confirmed. Inspect robot before retry.')
            except BaseException as error:
                self.event('abort_handoff_failed', error=str(error))
                print(f'Automatic handoff unconfirmed: {error}. Operator must inspect/control robot.', file=sys.stderr)

    return Port()


def main(args=None):
    """Run read-only checks or an explicitly requested yaw test; return a process exit code."""
    options = parser().parse_args(args)
    diagnostics = None
    port = None
    try:
        amplitude = math.radians(options.amplitude_deg)
        validate_options(amplitude, options.leg_seconds)
        if not math.isfinite(options.timeout) or not 1 <= options.timeout <= 10:
            raise TestFailure('timeout must be 1..10 seconds')
        from aimdk_test.head_test_diagnostics import DiagnosticLog
        diagnostics = DiagnosticLog(options.log_dir)
        options.diagnostics = diagnostics
        diagnostics.record('run_start', execute=options.execute,
                           amplitude_deg=options.amplitude_deg, leg_seconds=options.leg_seconds,
                           timeout_s=options.timeout)
        print(f'Diagnostic log: {diagnostics.path} (buffered; saved on exit)', flush=True)
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        # Keep context alive for bounded handoff on Ctrl-C.
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('head_yaw_acceptance')
        try:
            if node.get_parameter('use_sim_time').value:
                raise TestFailure('real-robot tool refuses use_sim_time')
            port = make_port(rclpy, node, options)
            deadline = time.monotonic() + options.timeout
            while port.monitor.latest is None or port.body is None:
                if time.monotonic() >= deadline:
                    raise TestFailure('startup head/MC feedback timeout')
                port.spin()
                if port.feedback_error or port.body_error:
                    raise port.feedback_error or port.body_error
            mode = port.mode()
            print(f'GetMcAction mode={mode}; status=RUNNING; '
                  f'MC body raw={port.body.body_status.value}; FSM is not checked.')
            if mode != STAND:
                raise TestFailure(f'execution precondition requires STAND_DEFAULT; current mode={mode}')
            now_ros = node.get_clock().now().nanoseconds / 1e9
            latest = port.monitor.latest
            print(f'Head feedback names={port.observed_names}; '
                  f'head source age={now_ros-latest.stamp:.4f}s, '
                  f'head receive age={time.monotonic()-latest.received:.4f}s; '
                  f'MC source age={now_ros-port.body_stamp:.4f}s, '
                  f'MC receive age={time.monotonic()-port.body_received:.4f}s.')
            fb = port.feedback()
            print(f'Mode={mode}; RUNNING; body=STAND; yaw={math.degrees(fb.positions[0]):.3f} deg; '
                  f'pitch slot={math.degrees(fb.positions[1]):.3f} deg; source/local clock checked.')
            if options.execute:
                execute_test(port, amplitude, options.leg_seconds)
                port.event('run_pass')
                print('PASS: yaw outbound/return endpoints measured; STAND_DEFAULT confirmed.')
            else:
                print('READ ONLY: no command publisher or SetMcAction client created. '
                      'Original X2 Ultra profile; only yaw is exercised.')
        except BaseException as error:
            if port is not None:
                port.event('run_error_snapshot', error=str(error), error_type=type(error).__name__)
            raise
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    except KeyboardInterrupt:
        if diagnostics is not None:
            diagnostics.record('run_interrupted')
        print('Interrupted. Inspect robot and confirm control mode before retry.', file=sys.stderr)
        return 130
    except Exception as error:
        if diagnostics is not None:
            diagnostics.record('run_failed', error=str(error), error_type=type(error).__name__)
        print(f'FAIL: {error}', file=sys.stderr)
        return 1
    finally:
        if diagnostics is not None:
            try:
                diagnostics.close()
            except Exception as error:
                print(f'Could not save diagnostic log: {error}', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
