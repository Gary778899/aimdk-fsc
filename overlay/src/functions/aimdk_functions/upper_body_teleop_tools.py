"""只读遥操的映射显示、参考状态快照及原节奏FIFO回放调度。"""

from bisect import bisect_right
import math
import time

import numpy as np


def rotation_vector_deg(rotation):
    """把诊断用旋转矩阵转为父坐标系旋转向量，单位度；180°轴符号等价。"""
    value = np.asarray(rotation, dtype=float)
    angle = math.acos(float(np.clip((np.trace(value) - 1.) / 2., -1., 1.)))
    skew = np.array([value[2, 1] - value[1, 2], value[0, 2] - value[2, 0],
                     value[1, 0] - value[0, 1]])
    if angle < 1e-8:
        result = skew / 2.
    elif math.pi - angle < 1e-6:
        # 对称部分的最大特征向量给出旋转轴，避免接近180°时除以sin(angle)。
        _, vectors = np.linalg.eigh(value + value.T)
        axis = vectors[:, -1]
        if np.dot(axis, skew) < 0:
            axis = -axis
        result = axis * angle
    else:
        result = skew * angle / (2. * math.sin(angle))
    return np.degrees(result).tolist()


def tracking_state(solver):
    """复制双侧已接受理想关节、范围起点、SEW及分支，用于失败前后对照。"""
    return dict(
        revision=solver.revision, last_input_seq=solver.last_input_seq,
        arm_rad=list(solver.accepted.arm_rad), hand_openings=list(solver.accepted.hand_openings),
        origin_arm_rad=list(solver.origin.arm_rad),
        command_reference_sides=sorted(solver.command_reference_sides),
        arms={side: dict(seed_rad=list(state.joints), sew_rad=state.sew, branch=state.branch)
              for side, state in solver.states.items()})


class MappingDisplay:
    """按显示间隔输出映射候选，Grip事件和失败立即输出；完整记录仍逐帧保存。"""

    def __init__(self, interval=1., clock=time.monotonic, output=print):
        """interval为秒，0关闭终端输出；clock及output可注入测试替身。"""
        if not math.isfinite(interval) or interval < 0:
            raise ValueError('display-interval须为有限非负秒数')
        self.interval, self.clock, self.output = interval, clock, output
        self.last = None

    def show(self, frame, report, accepted, reason=None, *, transient=False):
        """显示增量及结果；transient使连续可恢复拒绝沿用显示间隔，避免20Hz刷屏。"""
        if self.interval == 0:
            return
        now = self.clock()
        event = report is not None and any(
            row['event'] in ('reference_established', 'released') for row in report.values())
        if (accepted or transient) and not event and self.last is not None and now - self.last < self.interval:
            return
        self.last = now
        rows = [f'映射 seq={frame.seq} 结果={"accepted" if accepted else "rejected"}']
        if reason is not None:
            rows.append(f'原因={reason}')
        for side, row in (report or {}).items():
            text = f'{side}: {row["event"]} grip={row["grip"]:.3f} trigger={row["trigger"]:.3f}'
            if row.get('head_yaw_rad') is not None:
                text += f' Grip参考水平角°={math.degrees(row["head_yaw_rad"]):.2f}'
            if row['enabled']:
                text += (f' 手柄Δm={np.round(row["input_translation_m"], 4).tolist()}'
                         f' Δrot°={np.round(rotation_vector_deg(row["input_rotation_matrix"]), 2).tolist()}'
                         f' TCPΔm={np.round(row["tcp_translation_m"], 4).tolist()}'
                         f' Δrot°={np.round(rotation_vector_deg(row["tcp_rotation_matrix"]), 2).tolist()}')
            rows.append(text)
        self.output('\n'.join(rows), flush=True)


class ReplayPacer:
    """完整帧预载后的FIFO调度；recorded模式锚定首帧接收时间，不跳帧或重新计时。"""

    def __init__(self, frames, diagnostics, mode='immediate', clock=time.monotonic,
                 sleeper=time.sleep):
        """frames为按原顺序预载的帧；mode为immediate或recorded，时间函数可注入。"""
        if mode not in ('immediate', 'recorded'):
            raise ValueError('replay-timing须为immediate或recorded')
        self.frames, self.diagnostics, self.mode = frames, diagnostics, mode
        self.clock, self.sleeper = clock, sleeper
        first = frames[0].received_monotonic_s if frames else 0.
        self.offsets = [frame.received_monotonic_s - first for frame in frames]
        if any(b <= a for a, b in zip(self.offsets, self.offsets[1:])):
            raise ValueError('回放接收时间须严格推进')
        self.current = None
        self.started = False

    def __iter__(self):
        """逐帧等待绝对计划时刻；处理落后时直接消费下一帧，记录已到期等待帧数。"""
        if self.started:
            raise ValueError('同一回放调度器不能重复消费')
        self.started = True
        anchor = self.clock()
        for index, frame in enumerate(self.frames):
            due = anchor + self.offsets[index] if self.mode == 'recorded' else None
            if due is not None:
                while due > self.clock():
                    self.sleeper(min(.05, max(0., due - self.clock())))
            began = self.clock()
            backlog = (max(0, bisect_right(self.offsets, began - anchor) - index - 1)
                       if due is not None else None)
            self.current = dict(input_seq=frame.seq, timing_context=f'replay_{self.mode}',
                                processing_started_monotonic_s=began,
                                scheduled_monotonic_s=due)
            self.diagnostics.record(
                'input_timing', **self.current,
                receive_gap_s=self.offsets[index] - self.offsets[index-1] if index else None,
                processing_age_s=max(0., began-due) if due is not None else None,
                backlog_frames=backlog, remaining_frames=len(self.frames)-index-1,
                original_received_monotonic_s=frame.received_monotonic_s)
            yield frame

    def result_timing(self, frame):
        """返回本帧至结果日志写入后的耗时；原日志单调时间不与回放进程相减。"""
        if self.current is None or self.current['input_seq'] != frame.seq:
            raise ValueError('结果与当前回放输入不匹配')
        now = self.clock()
        due = self.current['scheduled_monotonic_s']
        return dict(timing_context=self.current['timing_context'],
                    completed_monotonic_s=now,
                    service_seconds=now-self.current['processing_started_monotonic_s'],
                    input_to_result_s=now-due if due is not None else None)


def live_result_timing(frame):
    """返回本机XR回调接收到结果记录完成的耗时，不计算跨设备网络延迟。"""
    now = time.monotonic()
    return dict(timing_context='live', completed_monotonic_s=now,
                input_to_result_s=now-frame.received_monotonic_s)
