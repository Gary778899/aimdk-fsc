"""Explicit X2 Ultra left wrist-roll execution with complete arm and claw position targets."""
import math
import sys
import time

from aimdk_test.arm_test_core import (
    ARM_NAMES, STAND, URS, TEST_INDEX, decode_mc, validate_arm_path,
    validate_start, arm_target, require_static_hands, check_holding,
    ArmEndpoint, ARM_DRIFT_TOLERANCE,
)
from aimdk_test.head_test_core import TestFailure


class ArmExecution:
    """Compose the read-only adapter with command resources only for explicit execution."""

    def __init__(self, port):
        """Create URS publisher and Set client, retaining all baseline streams and diagnostics."""
        from aimdk_msgs.msg import UpperBodyCommandArray
        from aimdk_msgs.srv import SetMcAction
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        self.port = port
        self.node = port.node
        self.message_type = UpperBodyCommandArray
        self.set_type = SetMcAction
        self.set_client = self.node.create_client(SetMcAction, '/aimdk_5Fmsgs/srv/SetMcAction')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.node.create_publisher(UpperBodyCommandArray, '/mc/upper_body_command', qos)
        self.port.streams['mc'].decoder = decode_mc
        self.expected = STAND
        self.transition_pair = None
        self.request_sent = False
        self.ownership_lost = False
        self.mode_value = None
        self.mode_received = 0.
        self.pending = None
        self.pending_at = 0.
        self.sequence = 0
        self.last_publish = None
        self.last_command = None
        self.hands = None
        self.handoff = False
        self.before = None
        self.aborting = False

    def event(self, name, **fields):
        """Record control events plus independent source/receipt ages for every feedback stream."""
        now = time.monotonic()
        ros_now = self.node.get_clock().now().nanoseconds / 1e9
        samples = {}
        for name_key, stream in self.port.streams.items():
            value = stream.monitor.latest
            if value is not None:
                samples[name_key] = dict(positions=list(value.positions), source_stamp_s=value.stamp,
                                        receive_age_s=now-value.received, source_age_s=ros_now-value.stamp,
                                        serial=value.serial)
        self.port.diagnostics.record(name, mode=self.mode_value, mode_age_s=now-self.mode_received,
                                     samples=samples, **fields)

    def checked(self):
        """Require fresh standing feedback and expected mode; latch evidence of external takeover."""
        values = self.port.checked()
        mc = values['mc'].data
        allowed = self.transition_pair or (self.expected,)
        statuses = (100, 200) if self.transition_pair else (100,)
        if mc['action'] not in allowed or mc['action_status'] not in statuses:
            self.ownership_lost = True
            raise TestFailure(f'MC mode changed: {mc["action"]} / {mc["action_status"]}; refusing takeover')
        return values

    def tick(self):
        """Process feedback without blocking publication, then check current feedback and mode invariants."""
        self.port.spin()
        values = self.checked()
        # Recovery retains freshness, body, hardware-fault and ownership checks,
        # but the motion error that caused an abort must not block mode handoff.
        if self.aborting:
            return values
        require_static_hands(values['hand'])
        if self.before is not None and self.request_sent:
            if any(abs(a-b) > .01 for a, b in zip(values['hand'].positions, self.before['hand'].positions)):
                raise TestFailure('claw opening changed during URS takeover or execution')
            if values['mc'].data['action'] == URS:
                for index, (actual, original) in enumerate(zip(values['arm'].positions, self.before['arm'].positions)):
                    if index != TEST_INDEX and abs(actual-original) > ARM_DRIFT_TOLERANCE:
                        raise TestFailure(f'{ARM_NAMES[index]} moved more than 3 degrees during takeover or execution')
        if self.handoff and self.last_command is not None:
            mc = values['mc'].data
            if mc['action'] == URS and mc['action_status'] == 100 and self.mode_value == URS:
                if time.monotonic() - self.mode_received > .4:
                    raise TestFailure('ownership confirmation expired during handoff')
                check_holding(values, self.last_command, self.hands)
                if time.monotonic() - self.last_publish >= .02:
                    self.publish(self.last_command, self.hands)
            else:
                self.handoff = False
                self.event('handoff_commands_stopped', action=mc['action'], status=mc['action_status'])
        return values

    def call(self, client, request, deadline=None, target=None):
        """Return a bounded service response while pumping callbacks and optional final-pose targets."""
        deadline = min(deadline, time.monotonic()+self.port.options.timeout) if deadline else time.monotonic()+self.port.options.timeout
        while not client.service_is_ready():
            if time.monotonic() >= deadline:
                raise TestFailure(f'service unavailable: {client.srv_name}')
            self.tick()
        self.event('service_send', service=client.srv_name, target_mode=target)
        future = client.call_async(request)
        if target == URS:
            self.request_sent = True
        try:
            while not future.done():
                if time.monotonic() >= deadline:
                    raise TestFailure(f'service timeout: {client.srv_name}; remote completion unknown')
                self.tick()
            if time.monotonic() >= deadline:
                raise TestFailure('service response arrived after deadline')
            response = future.result()
            if response is None:
                raise TestFailure('empty service response')
            return response
        finally:
            if not future.done():
                client.remove_pending_request(future)

    def record_mode(self, response, allow_transition=False):
        """Validate a query result without treating TRANSITION or late responses as RUNNING ownership."""
        self.event('mode_response', action=response.info.action_desc,
                   status=response.info.status.value, code=response.header.code)
        if response.header.code != 0:
            raise TestFailure('GetMcAction error response')
        action, status = response.info.action_desc, response.info.status.value
        allowed = self.transition_pair or (self.expected,)
        if action not in allowed:
            self.ownership_lost = True
            raise TestFailure(f'external mode {action}; will not take over')
        if allow_transition and status == 200:
            return None
        if status != 100:
            if not self.transition_pair:
                self.ownership_lost = True
            raise TestFailure(f'mode is not RUNNING: {action} / {status}')
        self.mode_value, self.mode_received = action, time.monotonic()
        return action

    def mode(self, deadline=None, allow_transition=False):
        """Query fresh mode within deadline and return a validated RUNNING action or transition None."""
        request = self.port.get_type.Request()
        request.request.header.stamp = self.node.get_clock().now().to_msg()
        response = self.call(self.port.client, request, deadline)
        return self.record_mode(response, allow_transition)

    def cancel_poll(self):
        """Remove an outstanding asynchronous Get request before synchronous service operations."""
        if self.pending is not None:
            self.port.client.remove_pending_request(self.pending)
            self.pending = None

    def poll_mode(self):
        """Poll URS asynchronously; reject ownership loss, late replies and expired confirmations."""
        now = time.monotonic()
        if self.pending is not None and now-self.pending_at > .3:
            self.cancel_poll()
            raise TestFailure('asynchronous mode response expired after 0.3 s')
        if self.pending is not None and self.pending.done():
            response = self.pending.result()
            self.pending = None
            if response is None or self.record_mode(response) != URS:
                raise TestFailure('URS ownership lost')
        if now-self.mode_received > .4:
            raise TestFailure('URS ownership confirmation expired after 0.4 s')
        if self.pending is None and now-self.mode_received >= .1:
            request = self.port.get_type.Request()
            request.request.header.stamp = self.node.get_clock().now().to_msg()
            self.pending = self.port.client.call_async(request)
            self.pending_at = now

    def preflight(self, amplitude, duration):
        """Check the empirical posture envelope and stationary claw openings before any Set request."""
        if self.mode() != STAND:
            raise TestFailure('must start in STAND_DEFAULT / RUNNING')
        initial = self.checked()
        validate_arm_path(initial['arm'].positions, amplitude, duration)
        require_static_hands(initial['hand'])
        deadline = time.monotonic() + .3
        while time.monotonic() < deadline:
            values = self.tick()
            validate_start(values['arm'].positions)
            check_holding(values, initial['arm'].positions, initial['hand'].positions,
                          arm_tolerance=math.radians(.2))
            if abs(values['arm'].positions[TEST_INDEX]-initial['arm'].positions[TEST_INDEX]) > math.radians(.2):
                raise TestFailure('wrist roll is not stationary before URS')
        self.before = self.checked()
        self.event('execution_preflight', joint=ARM_NAMES[TEST_INDEX], index=TEST_INDEX,
                   amplitude_rad=amplitude, leg_seconds=duration,
                   arm_drift_tolerance_deg=math.degrees(ARM_DRIFT_TOLERANCE),
                   profile='reviewed-baseline +/-3 degrees; wrist path +/-5 degrees')
        print('EXECUTE: left arm axis 7 / left_wrist_roll_joint, index 6; '
              f'offset={math.degrees(amplitude):+.3f} deg, {duration:g} s/leg, one round trip. '
              'All other joints hold measured start; both claws replay measured openings.', flush=True)

    def transition(self, target, expected):
        """Request one mode change from freshly verified expected mode, confirm target within one timeout."""
        self.cancel_poll()
        self.expected = expected
        self.handoff = target == STAND and self.last_command is not None
        if self.mode() != expected:
            raise TestFailure('mode changed before transition')
        self.checked()
        self.transition_pair = (expected, target)
        deadline = time.monotonic() + self.port.options.timeout
        self.event('transition_begin', target_mode=target)
        request = self.set_type.Request()
        request.header.stamp = self.node.get_clock().now().to_msg()
        request.source = 'node.arm_wrist_roll_acceptance'
        request.command.action_desc = target
        response = self.call(self.set_client, request, deadline, target)
        self.event('transition_response', target_mode=target, code=response.response.header.code,
                   status=response.response.status.value, message=response.response.message)
        if response.response.header.code != 0 or response.response.status.value != 1:
            raise TestFailure(f'mode change rejected: {response.response.message}')
        while time.monotonic() < deadline:
            action = self.mode(deadline, allow_transition=True)
            values = self.checked()
            if action == target and values['mc'].data['action'] == target and values['mc'].data['action_status'] == 100:
                self.expected, self.transition_pair, self.handoff = target, None, False
                self.event('transition_confirmed', target_mode=target,
                           gap_since_last_publish_s=(time.monotonic()-self.last_publish
                                                     if self.last_publish is not None else None))
                return
            next_query = min(deadline, time.monotonic()+.02)
            while time.monotonic() < next_query:
                self.tick()
        raise TestFailure('mode transition confirmation timed out')

    def capture_start(self, amplitude, duration):
        """Capture fresh advancing post-URS samples and reject arm/claw jumps during takeover."""
        after = time.monotonic()
        previous = self.checked()
        deadline = after + min(.3, self.port.options.timeout)
        while time.monotonic() < deadline:
            values = self.tick()
            self.poll_mode()
            if all(values[name].received > after and values[name].stamp > previous[name].stamp
                   for name in ('arm', 'hand', 'mc')):
                start, hands = values['arm'].positions, values['hand'].positions
                validate_arm_path(start, amplitude, duration,
                                  takeover_reference=self.before['arm'].positions)
                check_holding(values, self.before['arm'].positions, self.before['hand'].positions)
                if abs(start[TEST_INDEX]-self.before['arm'].positions[TEST_INDEX]) > math.radians(.5):
                    raise TestFailure('wrist roll jumped during URS takeover')
                self.hands = hands
                self.event('start_captured', start_rad=list(start), hand_openings=list(hands))
                print(f'Post-URS start arm_pos={list(start)}; hand_pos={list(hands)}; '
                      f'wrist peak={math.degrees(start[TEST_INDEX]+amplitude):.3f} deg.', flush=True)
                return start, hands
        raise TestFailure('no advancing arm/hand/MC feedback within 0.3 s after URS')

    def publish(self, positions, hands):
        """Publish full measured-start arm targets and mode-1 left/right claw openings at receive-time stamp."""
        if self.mode_value != URS or time.monotonic()-self.mode_received > .4:
            raise TestFailure('cannot publish without fresh confirmed URS ownership')
        if (len(positions) != 14 or len(hands) != 2
                or not all(math.isfinite(v) for v in (*positions, *hands))
                or not all(0 <= v <= 1 for v in hands)):
            raise TestFailure('invalid full arm/claw target')
        msg = self.message_type()
        msg.header.frame_id = 'mc_upper_body'
        msg.header.sequence = self.sequence
        msg.source = 'arm_wrist_roll_acceptance'
        # All-zero command stamp is the SDK receive-time option; feedback stamps stay original.
        msg.head_pos = [0., 0.]  # URS head targets are ineffective per upper-body chapter.
        msg.arm_pos = list(positions)
        msg.hand_sub_mode = self.message_type.HAND_CLAW_OPEN_CLOSE
        msg.hand_pos = list(hands)
        now = time.monotonic()
        gap = now-self.last_publish if self.last_publish is not None else None
        if gap is not None and gap > .08:
            raise TestFailure(f'command publication gap {gap:.6f}s exceeds 0.08 s')
        self.publisher.publish(msg)
        self.last_publish, self.last_command = now, tuple(positions)
        self.sequence += 1
        self.event('command_published', command_sequence=msg.header.sequence,
                   arm_pos=list(positions), hand_sub_mode=msg.hand_sub_mode,
                   hand_pos=list(hands), publish_gap_s=gap)

    def motion(self, start, hands, amplitude, duration):
        """Publish q0 first, validate newer feedback, then run quintic outbound/return with measured endpoints."""
        values = self.checked()
        if any(abs(a-b) > (math.radians(.2) if i == TEST_INDEX else ARM_DRIFT_TOLERANCE)
               for i, (a, b) in enumerate(zip(values['arm'].positions, start))):
            raise TestFailure('arm drifted before first command')
        check_holding(values, start, hands)
        self.poll_mode()
        self.publish(start, hands)
        first_publish_at = self.last_publish
        self.event('first_command', arm_pos=list(start), hand_pos=list(hands))
        deadline = time.monotonic()+.2
        while True:
            current = self.tick()
            self.poll_mode()
            check_holding(current, start, hands)
            if any(abs(a-b) > (math.radians(.5) if i == TEST_INDEX else ARM_DRIFT_TOLERANCE)
                   for i, (a, b) in enumerate(zip(current['arm'].positions, start))):
                raise TestFailure('arm moved before first-frame validation')
            if all(current[n].stamp > values[n].stamp and current[n].received > first_publish_at
                   for n in ('arm', 'hand')):
                break
            if time.monotonic() >= deadline:
                raise TestFailure('no advancing arm/claw feedback after first command')
            if time.monotonic()-self.last_publish >= .02:
                self.publish(start, hands)
        peak = arm_target(start, amplitude, duration, duration)
        for leg, origin, delta, end in ((1, start, amplitude, peak), (2, peak, -amplitude, start)):
            began = time.monotonic()
            endpoint = ArmEndpoint(end, began+duration)
            self.event('leg_begin', leg=leg, target_rad=list(end))
            while True:
                values = self.tick()
                self.poll_mode()
                now = time.monotonic()
                elapsed = now-began
                commanded = arm_target(origin, delta, duration, elapsed)
                check_holding(values, commanded, hands)
                if now-self.last_publish >= .02:
                    self.publish(commanded, hands)
                if elapsed >= duration and endpoint.checked(values['arm'].positions, now):
                    self.event('endpoint_confirmed', leg=leg, target_rad=list(end))
                    break
        self.event('round_trip_complete', published_count=self.sequence)

    def verify_exit(self, hands):
        """Observe fresh STAND feedback for 0.3 s and report claw opening changes after normal handoff."""
        deadline = time.monotonic()+.3
        while time.monotonic() < deadline:
            values = self.tick()
            require_static_hands(values['hand'])
            if any(abs(a-b) > .01 for a, b in zip(values['hand'].positions, hands)):
                raise TestFailure('claw opening changed after returning to STAND_DEFAULT')
        self.mode()
        self.event('exit_verified', hand_openings=list(values['hand'].positions))

    def abort_handoff(self):
        """Stop commands and try one bounded standing handoff only if fresh evidence still identifies our URS."""
        self.aborting = True
        self.handoff = False
        self.last_command = None
        self.cancel_poll()
        self.event('commands_stopped', reason='abort', published_count=self.sequence)
        if not self.request_sent or self.ownership_lost:
            self.event('abort_handoff_skipped', reason='no URS request or external takeover')
            return
        try:
            # No return trajectory, no reuse of stale ownership. Faulted feedback
            # prevents automatic Set: operator intervention may be necessary.
            self.expected, self.transition_pair = URS, (URS, STAND)
            action = self.mode()
            if action == STAND:
                self.expected, self.transition_pair = STAND, None
                self.event('abort_already_standing')
                return
            self.transition_pair = None
            self.expected = URS
            self.checked()
            # Recovery never resumes target publication or the return trajectory.
            self.transition(STAND, URS)
            self.event('abort_handoff_confirmed')
            print('Motion aborted; STAND_DEFAULT confirmed. Inspect robot before retry.', file=sys.stderr)
        except BaseException as error:
            self.event('abort_handoff_failed', error=str(error))
            print(f'Automatic handoff unconfirmed: {error}. Operator must inspect/control robot.', file=sys.stderr)
