"""遥操已发布目标的轨迹导出、具名读回与时间轴采样；无ROS导入。"""

from bisect import bisect_right
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

from .dual_arm_control_core import ARM_NAMES, ControlFailure, interpolate
from .upper_body_cartesian_core import CapturedTarget


SCHEMA = 'aimdk_functions.trajectory.v1'


def check_target(target):
    """沿用双臂执行限位校验完整目标；非有限值和夹爪范围由CapturedTarget校验。"""
    for index, side in enumerate(('left', 'right')):
        target.for_arm(side, target.arm_rad[index*7:index*7+7])
    return target


@dataclass(frozen=True)
class TrajectoryPoint:
    """保存相对首条所选发布的秒数、完整目标和原日志行号及命令序号。"""

    time_s: float
    target: CapturedTarget
    source_line: int
    source_sequence: int


@dataclass(frozen=True)
class RecordedTrajectory:
    """已校验的轨迹元数据、按时间排列的点及查表时间轴。"""

    metadata: dict
    points: tuple
    times: tuple

    def index_at(self, elapsed):
        """返回elapsed秒之前最近的轨迹点索引；越过两端时保持首点或末点。"""
        return max(0, min(len(self.points)-1, bisect_right(self.times, elapsed)-1))

    def summary(self):
        """返回时长、首末目标、关节范围和相邻最大步长，单位为秒及rad。"""
        joints = [point.target.arm_rad for point in self.points]
        return dict(point_count=len(self.points), duration_s=self.times[-1],
                    first_arm_rad=list(joints[0]), last_arm_rad=list(joints[-1]),
                    first_hand_openings=list(self.points[0].target.hand_openings),
                    last_hand_openings=list(self.points[-1].target.hand_openings),
                    minimum_arm_rad=[min(values) for values in zip(*joints)],
                    maximum_arm_rad=[max(values) for values in zip(*joints)],
                    maximum_step_rad=max((abs(b-a) for previous, current in zip(joints, joints[1:])
                                          for a, b in zip(previous, current)), default=0.))


def load_trajectory(path):
    """读取v1 JSONL并按头部关节名重排；格式、时间或目标无效时报告路径及行号。"""
    points, metadata, names = [], None, None
    with Path(path).open(encoding='utf-8') as stream:
        for number, line in enumerate(stream, 1):
            try:
                row = json.loads(line)
                if number == 1:
                    if (row['schema'] != SCHEMA or row['type'] != 'metadata'
                            or row['robot_model'] != 'x2-ultra' or row['urdf_version'] != 'v1.3.0'
                            or row['arm_unit'] != 'rad' or row['target_field'] != 'arm_rad'):
                        raise ValueError('要求x2-ultra/v1.3.0的v1已发布rad目标轨迹')
                    names = row['arm_names']
                    if len(names) != 14 or set(names) != set(ARM_NAMES):
                        raise ValueError('需要完整且唯一的14个关节名称')
                    metadata = row
                    continue
                if row['type'] != 'point' or len(row['arm_rad']) != 14:
                    raise ValueError('需要完整轨迹点')
                stamp = float(row['time_s'])
                if (not math.isfinite(stamp) or (not points and stamp != 0.)
                        or (points and stamp <= points[-1].time_s)):
                    raise ValueError('首点时间须为0，后续时间须有限且严格推进')
                positions = dict(zip(names, row['arm_rad']))
                hands = row['hand_openings']
                target = check_target(CapturedTarget(tuple(positions[name] for name in ARM_NAMES),
                                                     (hands['left'], hands['right'])))
                points.append(TrajectoryPoint(stamp, target, row['source_line'], row['source_sequence']))
            except (KeyError, TypeError, ValueError, ControlFailure) as error:
                raise ControlFailure(f'{path}:{number}: {error}') from error
    if metadata is None or len(points) < 2:
        raise ControlFailure('轨迹至少需要头部及两个时间推进的目标点')
    return RecordedTrajectory(metadata, tuple(points), tuple(point.time_s for point in points))


def export_trajectory(source, output, start=0., end=None):
    """导出遥操command_published的已发送目标；窗口相对首条发布，重置所选首点为0。

    保留源日志SHA256、代码/模型元数据和退出记录；历史日志缺发布调用时间时拒绝导出。
    output使用独占创建，避免覆盖已有轨迹；返回严格读回后的RecordedTrajectory。
    """
    if not math.isfinite(start) or start < 0 or (end is not None and (
            not math.isfinite(end) or end <= start)):
        raise ControlFailure('导出窗口要求0 <= start < end，单位为有限秒数')
    source, output = Path(source).resolve(), Path(output)
    points, original, config, exits = [], None, None, []
    origin = previous = selected_origin = None
    digest = hashlib.sha256()
    with source.open('rb') as stream:
        for number, raw in enumerate(stream, 1):
            digest.update(raw)
            try:
                row = json.loads(raw)
                event, fields = row['event'], row['fields']
                if event == 'metadata':
                    original = fields
                elif event == 'online_ik_config':
                    config = fields
                elif event in ('error', 'exit'):
                    exits.append(dict(event=event, fields=fields))
                elif event == 'command_published':
                    stamp = float(fields['publish_begin_monotonic_s'])
                    if not math.isfinite(stamp) or (previous is not None and stamp <= previous):
                        raise ValueError('发布调用时间须有限且严格推进')
                    origin = stamp if origin is None else origin
                    previous = stamp
                    elapsed = stamp-origin
                    if elapsed < start or (end is not None and elapsed > end):
                        continue
                    selected_origin = stamp if selected_origin is None else selected_origin
                    target = check_target(CapturedTarget(fields['arm_rad'], fields['hand_openings']))
                    points.append(dict(type='point', time_s=stamp-selected_origin,
                                       arm_rad=list(target.arm_rad),
                                       hand_openings=dict(zip(('left', 'right'), target.hand_openings)),
                                       source_line=number, source_sequence=fields['sequence'],
                                       source_publish_monotonic_s=stamp))
            except (KeyError, TypeError, ValueError, ControlFailure) as error:
                raise ControlFailure(f'{source}:{number}: {error}') from error
    if original is None or original.get('command') != 'upper_body_teleop_observe':
        raise ControlFailure('导出源须为upper_body_teleop_observe日志')
    if len(points) < 2:
        raise ControlFailure('所选窗口至少需要两条已发布命令')
    metadata = dict(schema=SCHEMA, type='metadata', robot_model='x2-ultra',
                    urdf_version='v1.3.0', arm_unit='rad', arm_names=list(ARM_NAMES),
                    target_field='arm_rad', additional_compensation=False,
                    created_at_utc=datetime.now(timezone.utc).isoformat(),
                    source_log=str(source), source_sha256=digest.hexdigest(),
                    source_metadata=original, source_ik_config=config, source_exits=exits,
                    window_s=dict(start=start, end=end),
                    selected_start_s=points[0]['source_publish_monotonic_s']-origin)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x', encoding='utf-8') as stream:
        for row in (metadata, *points):
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
    return load_trajectory(output)


class ReplayTrajectory:
    """从实时采样目标五次过渡至首点，随后按指定倍速零阶保持原轨迹。"""

    kind = 'trajectory_replay'

    def __init__(self, captured, recording, start_duration, speed):
        """保存实时起点、记录及显式正过渡秒数和倍速；非法参数抛出ControlFailure。"""
        if any(not math.isfinite(value) or value <= 0 for value in (start_duration, speed)):
            raise ControlFailure('起点过渡时长和倍速须为有限正数')
        self.captured, self.recording = captured, recording
        self.start_duration, self.speed = start_duration, speed
        self.duration = start_duration + recording.times[-1]/speed

    def sample(self, elapsed):
        """返回本次阶段、原轨迹相对时间和源点索引；过渡阶段无源点。"""
        if elapsed < self.start_duration:
            return dict(phase='approach', trajectory_time_s=None, point_index=None)
        stamp = min(self.recording.times[-1], (elapsed-self.start_duration)*self.speed)
        return dict(phase='replay' if elapsed < self.duration else 'endpoint',
                    trajectory_time_s=stamp, point_index=self.recording.index_at(stamp))

    def at(self, elapsed):
        """返回完整双臂及夹爪目标；过渡零增量首帧，回放按绝对时间查表并保持末点。"""
        if elapsed <= 0:
            return self.captured
        if elapsed < self.start_duration:
            target = self.recording.points[0].target
            return CapturedTarget(
                interpolate(self.captured.arm_rad, target.arm_rad, elapsed, self.start_duration),
                interpolate(self.captured.hand_openings, target.hand_openings, elapsed, self.start_duration))
        return self.recording.points[self.sample(elapsed)['point_index']].target
