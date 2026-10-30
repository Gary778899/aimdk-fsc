"""Read-only X2 Ultra arm/claw baseline validation; no ROS dependencies."""
from dataclasses import dataclass
import math

from aimdk_test.head_test_core import FeedbackMonitor, TestFailure

ARM_NAMES = tuple(f'{side}_{joint}_joint' for side in ('left', 'right')
                  for joint in ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw',
                                'elbow', 'wrist_yaw', 'wrist_pitch', 'wrist_roll'))


@dataclass(frozen=True)
class Sample:
    """Decoded values plus original source time and local monotonic receipt time."""
    positions: tuple
    stamp: float
    received: float
    serial: int
    data: dict


def header_data(header):
    """Return both original source timestamps and sequence for JSON diagnostics."""
    return dict(stamp_s=header.stamp.sec + header.stamp.nanosec * 1e-9,
                meas_stamp_s=header.meas_stamp.sec + header.meas_stamp.nanosec * 1e-9,
                sequence=header.sequence)


def sample(msg, received, serial, positions, data):
    """Return a Sample with original header; reject zero/invalid world timestamps."""
    header = header_data(msg.header)
    if not math.isfinite(header['stamp_s']) or header['stamp_s'] <= 0:
        raise TestFailure('source timestamp must be nonzero and finite')
    return Sample(tuple(positions), header['stamp_s'], received, serial,
                  dict(header=header, **data))


def decode_arm(msg, received, serial):
    """Reorder exactly 14 unique arm joints; raise TestFailure for malformed/faulted data."""
    names = [joint.name for joint in msg.joints]
    if len(names) != 14 or len(set(names)) != 14 or set(names) != set(ARM_NAMES):
        raise TestFailure(f'expected exactly 14 unique arm joints; observed {names}')
    if msg.state.value != 0:
        raise TestFailure(f'arm domain error={msg.state.value}')
    found = {joint.name: joint for joint in msg.joints}
    joints = []
    for name in ARM_NAMES:
        joint = found[name]
        if joint.error_code or not all(math.isfinite(v) for v in (
                joint.position, joint.velocity, joint.effort)):
            raise TestFailure(f'invalid arm joint {name}: error={joint.error_code}')
        joints.append(dict(name=name, position=joint.position, velocity=joint.velocity,
                           effort=joint.effort, error_code=joint.error_code))
    return sample(msg, received, serial, [j['position'] for j in joints],
                  dict(observed_names=names, domain_state=msg.state.value, joints=joints))


def decode_hand(msg, received, serial):
    """Validate both CLAW feedback arrays, rejecting cached NONE/ERROR values and faults.

    Moving/blocked states are recorded in read-only mode and mark the sample
    unsuitable for a future position-hold start; they do not trigger commands.
    """
    hands = []
    for side in ('left', 'right'):
        kind = getattr(msg, f'{side}_hand_type').value
        values = getattr(msg, f'{side}_hands')
        if kind != 2 or len(values) != 1:
            raise TestFailure(f'{side}: expected CLAW=2 and one hand; type={kind}, count={len(values)}')
        hand = values[0]
        if (hand.faultcode or not all(math.isfinite(v) for v in (
                hand.position, hand.velocity, hand.effort)) or not 0 <= hand.position <= 1):
            raise TestFailure(f'{side}: invalid claw feedback, faultcode={hand.faultcode}')
        hands.append(dict(side=side, type=kind, name=hand.name, position=hand.position,
                          velocity=hand.velocity, effort=hand.effort,
                          state=hand.state, faultcode=hand.faultcode,
                          stationary_candidate=hand.state == 0 and abs(hand.velocity) <= .01))
    return sample(msg, received, serial, [h['position'] for h in hands], dict(hands=hands))


def decode_body(msg, received, serial):
    """Decode MC body/mode diagnostics; require standing RUNNING STAND_DEFAULT, ignore FSM encoding."""
    if (msg.body_status.value != 1 or msg.action_info.action_desc != 'STAND_DEFAULT'
            or msg.action_info.status.value != 100):
        raise TestFailure('MC feedback must report body=STAND and STAND_DEFAULT / RUNNING')
    return sample(msg, received, serial, (), dict(body=msg.body_status.value,
                  action=msg.action_info.action_desc, action_status=msg.action_info.status.value,
                  fsm_raw=msg.fsm_state.current_state))


class Stream:
    """Reuse head-test clock/freshness checks with a permanent per-stream error latch."""

    def __init__(self, name, decoder):
        """Initialize a named feedback stream using decoder(msg, receipt, serial)."""
        self.name, self.decoder = name, decoder
        self.monitor = FeedbackMonitor()
        self.error = None
        self.serial = 0
        self.first_stamp = None

    def accept(self, msg, received):
        """Decode one message and latch errors; later healthy frames cannot clear faults."""
        self.serial += 1
        try:
            value = self.decoder(msg, received, self.serial)
            self.monitor.accept(value)
            if self.first_stamp is None:
                self.first_stamp = value.stamp
        except TestFailure as error:
            self.error = self.error or error

    def checked(self, now, ros_now):
        """Return fresh Sample after observing actual source advancement; raise TestFailure otherwise."""
        if self.error:
            raise TestFailure(f'{self.name}: {self.error}')
        try:
            value = self.monitor.checked(now, ros_now)
        except TestFailure as error:
            raise TestFailure(f'{self.name}: {str(error).replace("head", "stream")}') from error
        if value.stamp <= self.first_stamp:
            raise TestFailure(f'{self.name}: waiting for source timestamp advancement')
        return value


STAND = 'STAND_DEFAULT'
URS = 'UPPERBODY_REMOTE_SPLIT'
TEST_INDEX = 6
# Reviewed 2026-10-20 robot feedback, NOT a command or a manufacturer's limit.
REFERENCE = (.38684511184692383, -.06087923049926758, -.01831197738647461,
             -1.159193515777588, .004122257232666016, .00438690185546875,
             -.00324249267578125, .39125537872314453, .06528902053833008,
             .0267486572265625, -1.1568927764892578, .0004792213439941406,
             .01125335693359375, -.00553131103515625)


def decode_mc(msg, received, serial):
    """Decode standing MC feedback during mode transitions without imposing one mode name."""
    if msg.body_status.value != 1:
        raise TestFailure('MC body must remain STAND')
    return sample(msg, received, serial, (), dict(body=msg.body_status.value,
                  action=msg.action_info.action_desc, action_status=msg.action_info.status.value,
                  fsm_raw=msg.fsm_state.current_state))


def validate_motion_options(amplitude, duration):
    """Validate signed radians and seconds; permit only 0.5..2 degrees and 3..10 s per leg."""
    if not math.isfinite(amplitude) or not math.radians(.5) <= abs(amplitude) <= math.radians(2):
        raise TestFailure('arm amplitude magnitude must be 0.5..2 degrees')
    if not math.isfinite(duration) or not 3 <= duration <= 10:
        raise TestFailure('arm leg duration must be 3..10 seconds')
    if 1.875 * abs(amplitude) / duration > math.radians(2):
        raise TestFailure('arm peak velocity exceeds 2 degrees/s')


def validate_start(positions):
    """Restrict this first-test profile to within three degrees of each reviewed joint position.

    This is an empirical acceptance envelope, not a mechanical limit table. In
    particular the measured right shoulder roll is preserved without clipping
    it to the conflicting bundled URDF. Refuse other starting configurations.
    """
    if len(positions) != 14 or not all(math.isfinite(v) for v in positions):
        raise TestFailure('expected 14 finite arm positions')
    for name, actual, reference in zip(ARM_NAMES, positions, REFERENCE):
        if abs(actual-reference) > math.radians(3):
            raise TestFailure(f'{name}: start differs from reviewed baseline by more than 3 degrees')


ARM_DRIFT_TOLERANCE = math.radians(3)


def validate_arm_path(start, amplitude, duration, takeover_reference=None):
    """Check posture and wrist path in radians, allowing 3-degree held-joint takeover drift.

    With takeover_reference, validate that pre-switch posture against the reviewed
    baseline and bound the post-switch start relative to it. The tested wrist
    may jump at most 0.5 degrees; its entire path must remain within +/-5 degrees.
    """
    validate_motion_options(amplitude, duration)
    if takeover_reference is None:
        validate_start(start)
    else:
        validate_start(takeover_reference)
        if len(start) != 14 or not all(math.isfinite(v) for v in start):
            raise TestFailure('expected 14 finite arm positions')
        for index, (actual, reference) in enumerate(zip(start, takeover_reference)):
            tolerance = math.radians(.5) if index == TEST_INDEX else ARM_DRIFT_TOLERANCE
            if abs(actual-reference) > tolerance:
                raise TestFailure(f'{ARM_NAMES[index]} jumped during URS takeover')
    if max(abs(start[TEST_INDEX]), abs(start[TEST_INDEX] + amplitude)) > math.radians(5):
        raise TestFailure('left wrist roll path exceeds +/-5 degree first-test window')


def arm_target(start, delta, duration, elapsed):
    """Return a full 14-joint quintic leg; change only left wrist roll, preserving the other 13 values."""
    t = max(0., min(1., elapsed / duration))
    blend = t ** 3 * (10 - 15 * t + 6 * t * t)
    target = list(start)
    target[TEST_INDEX] += delta * blend
    return tuple(target)


def require_static_hands(value):
    """Require healthy stationary claws for capture and execution; fault/type checks occur in decode_hand."""
    if not all(h['stationary_candidate'] for h in value.data['hands']):
        raise TestFailure('both claws must remain stationary, state=0, abs(velocity)<=0.01')


def check_holding(values, commanded, hands, arm_tolerance=ARM_DRIFT_TOLERANCE):
    """Check all 14 joint errors and both claw openings; reject claw drift above 0.01 travel."""
    require_static_hands(values['hand'])
    for index, (actual, target) in enumerate(zip(values['arm'].positions, commanded)):
        tolerance = math.radians(1) if index == TEST_INDEX else arm_tolerance
        if abs(actual-target) > tolerance:
            raise TestFailure(f'{ARM_NAMES[index]} tracking error exceeds {math.degrees(tolerance):g} degrees')
    if any(abs(a-b) > .01 for a, b in zip(values['hand'].positions, hands)):
        raise TestFailure('claw opening changed by more than 0.01 travel')


def execute_arm_test(port, amplitude, duration):
    """Execute one left wrist-roll round trip through a bounded hardware port; attempt handoff on error."""
    validate_motion_options(amplitude, duration)
    port.preflight(amplitude, duration)
    try:
        port.transition(URS, STAND)
        start, hands = port.capture_start(amplitude, duration)
        port.motion(start, hands, amplitude, duration)
        port.transition(STAND, URS)
        port.verify_exit(hands)
    except BaseException as error:
        port.event('execution_failure', error=str(error), error_type=type(error).__name__)
        port.abort_handoff()
        raise


class ArmEndpoint:
    """Require the tested wrist within 0.2 degrees and every held joint within 0.3 degrees for 0.3 s."""

    def __init__(self, target, arrival_time):
        """Initialize endpoint positions in radians and monotonic planned arrival in seconds."""
        self.target, self.arrival_time = target, arrival_time
        self.since = None

    def checked(self, actual, now):
        """Return True after settling; raise TestFailure if not settled within two seconds of arrival."""
        reached = all(abs(a-b) <= math.radians(.2 if i == TEST_INDEX else .3)
                      for i, (a, b) in enumerate(zip(actual, self.target)))
        self.since = (self.since if self.since is not None else now) if reached else None
        if self.since is not None and now-self.since >= .3:
            return True
        if now > self.arrival_time+2:
            raise TestFailure('arm endpoint not settled within 2 s (wrist 0.2 deg, held joints 0.3 deg)')
        return False
