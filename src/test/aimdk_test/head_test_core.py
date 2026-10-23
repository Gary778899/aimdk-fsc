"""Offline safety checks for the X2 Ultra yaw acceptance test; no ROS imports."""
from dataclasses import dataclass
import math

NAMES = ('head_yaw_joint', 'head_pitch_joint')
STAND = 'STAND_DEFAULT'
HEAD = 'HEAD_ONLY'
YAW_LIMIT = math.radians(19)  # documented Ultra +/-20 deg, retain 1 deg margin
PITCH_LIMIT = 0.3838  # SDK motocontrol example; pitch is held, never exercised
MAX_SPEED = math.radians(5)
MAX_TRACKING_ERROR = math.radians(5)


class TestFailure(RuntimeError):
    """An acceptance check failed; no promise of physical safety is implied."""


@dataclass(frozen=True)
class Feedback:
    positions: tuple
    stamp: float
    received: float
    serial: int


def validate_options(amplitude, duration):
    """Validate amplitude in radians and leg duration in seconds; raise TestFailure outside limits."""
    if not math.isfinite(amplitude) or not math.radians(1) <= abs(amplitude) <= math.radians(5):
        raise TestFailure('amplitude magnitude must be 1..5 degrees')
    if not math.isfinite(duration) or not 3 <= duration <= 10:
        raise TestFailure('each leg duration must be 3..10 seconds')
    if 1.875 * abs(amplitude) / duration > MAX_SPEED:
        raise TestFailure('trajectory peak velocity exceeds 5 degrees/s')


def decode_feedback(msg, received, serial):
    """Decode a joint message with receipt time and serial; return Feedback or raise TestFailure on invalid data."""
    if msg.state.value != 0:
        raise TestFailure(f'head domain error state={msg.state.value}')
    found = {}
    for joint in msg.joints:
        if joint.name in NAMES:
            if joint.name in found:
                raise TestFailure(f'duplicate joint: {joint.name}')
            if joint.error_code or not all(math.isfinite(x) for x in (
                    joint.position, joint.velocity, joint.effort)):
                raise TestFailure(f'bad joint feedback: {joint.name}')
            found[joint.name] = joint.position
    if NAMES[0] not in found:
        raise TestFailure(f'yaw feedback missing; observed {sorted(found)}')
    # Original X2 Ultra has no effective pitch axis (joint_control manual).
    # The fixed protocol slot is zero only when that inactive joint is absent.
    positions = (found[NAMES[0]], found.get(NAMES[1], 0.0))
    validate_pose(positions)
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if not math.isfinite(stamp) or stamp <= 0:
        raise TestFailure('feedback source timestamp must be nonzero')
    return Feedback(positions, stamp, received, serial)


def validate_pose(positions):
    """Check two-slot joint positions in radians; raise TestFailure outside accepted limits."""
    if len(positions) != 2 or not all(math.isfinite(v) for v in positions):
        raise TestFailure('invalid two-slot head position')
    if abs(positions[0]) > YAW_LIMIT or abs(positions[1]) > PITCH_LIMIT:
        raise TestFailure('head position outside conservative acceptance limits')


class FeedbackMonitor:
    def __init__(self):
        """Initialize head feedback and ROS/source clock observations."""
        self.latest = None
        self.source_advanced_at = None
        self.last_ros = None
        self.clock_advanced_at = None

    def accept(self, feedback):
        """Store a Feedback sample and track source time advancement; raise TestFailure on regression."""
        previous = self.latest
        if previous and feedback.stamp < previous.stamp:
            raise TestFailure('feedback source clock moved backwards')
        if previous is None or feedback.stamp > previous.stamp:
            self.source_advanced_at = feedback.received
        self.latest = feedback

    def observe_clock(self, now, ros_now):
        """Observe ROS time independently of feedback checks; reject invalid or regressing time."""
        if not math.isfinite(ros_now) or ros_now <= 0:
            raise TestFailure('invalid ROS clock')
        if self.last_ros is not None and ros_now < self.last_ros:
            raise TestFailure('ROS clock moved backwards')
        if self.last_ros is None or ros_now > self.last_ros:
            self.clock_advanced_at = now
        self.last_ros = ros_now

    def checked(self, now, ros_now):
        """Return fresh feedback or raise TestFailure with the failed age and limit."""
        self.observe_clock(now, ros_now)
        fb = self.latest
        if fb is None:
            raise TestFailure('head feedback unavailable')
        ages = {
            'head receive gap': now - fb.received,
            'head source timestamp frozen': now - self.source_advanced_at,
            'ROS clock frozen': now - self.clock_advanced_at,
        }
        failures = [f'{name}={age:.6f}s > 0.200000s'
                    for name, age in ages.items() if age > .2]
        if failures:
            raise TestFailure('; '.join(failures))
        age = ros_now - fb.stamp
        if not -.05 <= age <= .2:
            raise TestFailure(f'feedback source/local clock mismatch or stale stamp: {age:.3f}s')
        return fb


def target_at(start, amplitude, duration, elapsed):
    """Quintic outbound and inbound; zero velocity/acceleration at joins."""
    if elapsed <= 0 or elapsed >= 2 * duration:
        return tuple(start)
    t = elapsed / duration if elapsed <= duration else (2 * duration - elapsed) / duration
    blend = t ** 3 * (10 - 15 * t + 6 * t * t)
    return (start[0] + amplitude * blend, start[1])


def validate_path(start, amplitude, duration):
    """Validate the start, endpoint, amplitude and duration; raise TestFailure outside limits."""
    validate_options(amplitude, duration)
    validate_pose(start)
    validate_pose((start[0] + amplitude, start[1]))


def tracking_check(actual, commanded):
    """Raise TestFailure when actual joint positions differ from commanded radians by more than five degrees."""
    if any(abs(a - b) > MAX_TRACKING_ERROR for a, b in zip(actual, commanded)):
        raise TestFailure('head tracking error exceeds 5 degrees')


def execute_test(port, amplitude, duration):
    """Port abstracts hardware; all mode operations must be bounded by its adapter."""
    validate_options(amplitude, duration)
    if port.mode() != STAND:
        raise TestFailure('start must already be STAND_DEFAULT; will not stand up or take over')
    port.feedback()
    entered = False
    try:
        entered = True  # request may reach MC even when local confirmation times out
        port.transition(HEAD, expected=STAND)
        start = port.new_feedback().positions
        validate_path(start, amplitude, duration)
        port.motion(start, amplitude, duration)
        port.transition(STAND, expected=HEAD)
        entered = False
    except BaseException as error:
        if hasattr(port, 'event'):
            port.event('execution_failure', error=str(error), error_type=type(error).__name__)
        # Never issue a return trajectory on an error. Adapter must obtain a new
        # successful mode response plus valid feedback before handing back to stand.
        if entered:
            port.abort_handoff()
        raise


class EndpointMonitor:
    def __init__(self, target, arrival_time):
        """Track a target in radians from the planned monotonic arrival_time in seconds."""
        self.target = target
        self.arrival_time = arrival_time
        self.since = None

    def checked(self, actual, now):
        """Check actual positions at monotonic time now; return True after settling or raise TestFailure on timeout."""
        reached = all(abs(a-b) <= math.radians(.5) for a, b in zip(actual, self.target))
        self.since = (self.since if self.since is not None else now) if reached else None
        if self.since is not None and now - self.since >= .3:
            return True
        if now > self.arrival_time + 2:
            raise TestFailure('endpoint not reached within 0.5 deg and 2 s settling')
        return False
