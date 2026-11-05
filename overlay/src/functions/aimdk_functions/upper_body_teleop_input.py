"""连续接收完整XR输入，保留Grip边沿并合并普通位姿；不导入ROS。"""

from collections import deque
from dataclasses import dataclass
import threading
import time

from remote_control.mapping import InputUnavailable, Session


@dataclass(frozen=True)
class InputSelection:
    """交给主线程的完整帧、Grip边沿标记与边沿代数。"""

    frame: object
    edge: bool
    generation: int


class LatestInput:
    """接收线程独占录制器，主线程消费边沿FIFO和单槽最新位姿。"""

    def __init__(self, source, recorder, config, diagnostics, options, lock=None, on_error=None):
        """准备接收器；lock与执行提交共用，on_error在输入失效时立即停发。"""
        self.source, self.recorder = source, recorder
        self.config, self.diagnostics, self.options = config, diagnostics, options
        self.lock = lock if lock is not None else threading.RLock()
        self.on_error = on_error
        self.validator = Session(config)
        self.edges = deque()
        self.latest = self.selected = None
        self.pressed = (False, False)
        self.generation = self.consumed_seq = self.coalesced = 0
        self.next_pose = 0.
        self.last_received = self.source_advanced = self.last_source = None
        self.error = None
        self.finished = False
        self.active = not options.execute
        self.closed = threading.Event()
        self.thread = None

    def start(self):
        """启动唯一的SDK事件消费线程，持续接收与记录，不等待主线程IK。"""
        self.thread = threading.Thread(target=self._receive, name='teleop_input')
        self.thread.start()

    def close(self):
        """结束消费线程并等待退出；调用后才能关闭SDK和排空剩余事件。"""
        self.closed.set()
        if self.thread is not None:
            self.thread.join()

    def activate(self):
        """首次Grip接管时启用执行输入时限，保留首次Grip前持续等待的约定。"""
        with self.lock:
            self.active = True

    def check(self, now=None):
        """在锁内传播首次接收异常并检查原始接收及源时间；超时抛出InputUnavailable。"""
        with self.lock:
            if self.error is not None:
                raise self.error
            timeout = self.options.input_timeout
            if not self.active or timeout is None or self.last_received is None:
                return
            now = time.monotonic() if now is None else now
            if now - self.last_received > timeout:
                raise InputUnavailable('XR接收／处理年龄超过input-timeout')
            if self.last_source is not None and now - self.source_advanced > timeout:
                raise InputUnavailable('XR源时间停滞超过input-timeout')

    def accept(self, frame, now):
        """逐帧校验并更新原始时间；保留完整边沿帧，同会话普通位姿只保留最新一帧。"""
        with self.lock:
            self.validator.observe_input(frame)
            gap = (None if self.last_received is None
                   else frame.received_monotonic_s - self.last_received)
            age = now - frame.received_monotonic_s
            timeout = self.options.input_timeout
            if self.active and timeout is not None:
                if gap is not None and gap > timeout:
                    raise InputUnavailable('XR回调接收间隔超过input-timeout')
                if age > timeout:
                    raise InputUnavailable('XR积压帧年龄超过input-timeout')
            self.last_received = frame.received_monotonic_s
            if frame.source_time_ns is not None:
                if self.last_source is None or frame.source_time_ns > self.last_source:
                    self.source_advanced = frame.received_monotonic_s
                self.last_source = frame.source_time_ns
            pressed = tuple(getattr(frame, side).grip >= self.config.grip_threshold
                            for side in ('left', 'right'))
            edge = pressed != self.pressed
            if edge:
                self.generation += 1
                self.edges.append(InputSelection(frame, True, self.generation))
            if self.latest is not None and self.latest.frame.seq > self.consumed_seq:
                if not self.latest.edge:
                    self.coalesced += 1
            self.pressed = pressed
            self.latest = InputSelection(frame, edge, self.generation)
            self.check(now)

    def take(self, now):
        """优先取最早边沿；普通位姿最多20Hz，每次取最新值，超期周期不补算。"""
        with self.lock:
            self.check(now)
            if self.edges:
                selected = self.edges.popleft()
            elif (self.latest is not None and self.latest.frame.seq > self.consumed_seq
                  and now >= self.next_pose):
                selected = self.latest
                self.next_pose = now + .05
            else:
                return None
            self.consumed_seq = selected.frame.seq
            self.selected = selected
            return selected

    def can_commit(self):
        """提交锁内检查接收状态；边沿保序提交，普通候选遇到更新的Grip边沿则丢弃。"""
        with self.lock:
            self.check()
            return self.selected.edge or self.selected.generation == self.generation

    def _receive(self):
        """持续录制与校验事件，记录接收耗时；输入失效锁存并立即调用停发回调。"""
        began = time.monotonic()
        try:
            while not self.closed.is_set():
                if (self.options.duration is not None
                        and time.monotonic() - began >= self.options.duration):
                    break
                event = self.source.read(timeout=.01)
                if event is None:
                    self.check()
                    continue
                frame = self.recorder.consume(event, self.source.backlog)
                if event.kind == 'server_disconnected' or (
                        event.kind == 'device_missing' and event.device_id == self.options.device_id):
                    raise InputUnavailable(f'XR输入失效：{event.kind}')
                if frame is None:
                    continue
                now = time.monotonic()
                previous = self.last_received
                self.accept(frame, now)
                fields = dict(input_seq=frame.seq,
                              receive_gap_s=(None if previous is None
                                             else frame.received_monotonic_s - previous),
                              processing_age_s=now-frame.received_monotonic_s,
                              backlog_events=self.source.backlog, timing_context='live')
                self.recorder.log.record('input_timing', **fields)
                self.diagnostics.record('input_timing', **fields)
        except Exception as error:
            with self.lock:
                self.error = error
                if self.on_error is not None:
                    self.on_error()
        finally:
            with self.lock:
                self.finished = True
