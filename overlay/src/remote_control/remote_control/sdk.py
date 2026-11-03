"""通过官方C ABI复制完整XR回调；本模块导入时不加载动态库。"""

import base64
import ctypes
from dataclasses import asdict, dataclass
import importlib.util
from pathlib import Path
import queue
import threading
import time


SERVER_CONNECT = 1 << 2
SERVER_DISCONNECT = 1 << 3
DEVICE_FIND = 1 << 4
DEVICE_MISSING = 1 << 5
DEVICE_CONNECT = 1 << 9
DEVICE_STATE_JSON = 1 << 25
EVENT_NAMES = {
    SERVER_CONNECT: 'server_connected', SERVER_DISCONNECT: 'server_disconnected',
    DEVICE_FIND: 'device_found', DEVICE_MISSING: 'device_missing',
    DEVICE_CONNECT: 'device_connected', DEVICE_STATE_JSON: 'device_state',
}
CALLBACK_MASK = sum(EVENT_NAMES)


class DeviceStateJson(ctypes.Structure):
    """对应官方PXREADevStateJson的固定长度字符数组。"""

    _fields_ = [('devID', ctypes.c_char * 32), ('stateJson', ctypes.c_char * 16352)]


SDK_CALLBACK = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_int,
                               ctypes.c_int, ctypes.c_void_p)


@dataclass(frozen=True)
class SDKEvent:
    """在回调内复制的原始事件；序号属于本次SDK连接，时间使用本机单调秒。"""

    callback_seq: int
    received_monotonic_s: float
    kind: str
    callback_type: int
    status: int
    device_id: str
    payload_base64: str
    error: str | None = None

    def to_dict(self):
        """返回可直接写入JSONL的原始事件字段。"""
        return asdict(self)

    @classmethod
    def from_dict(cls, value):
        """从日志重建原始事件；缺失或额外字段按构造异常报告。"""
        return cls(**value)

    def payload(self):
        """还原回调字节副本；非法Base64抛出ValueError。"""
        return base64.b64decode(self.payload_base64, validate=True)


def resolve_library(path=None):
    """返回显式库路径或xrobotoolkit-sdk随包库；缺失时抛出FileNotFoundError。"""
    if path is not None:
        candidate = Path(path).expanduser().resolve()
    else:
        spec = importlib.util.find_spec('xrobotoolkit_sdk')
        if spec is None or not spec.submodule_search_locations:
            raise FileNotFoundError('请安装xrobotoolkit-sdk或指定--sdk-library')
        candidate = Path(next(iter(spec.submodule_search_locations))) / 'libPXREARobotSDK.so'
    if not candidate.is_file():
        raise FileNotFoundError(f'未找到PXREARobotSDK动态库：{candidate}')
    return candidate


class SDKSource:
    """单实例SDK连接及无覆盖的FIFO回调队列；消费端负责记录积压。"""

    _active_lock = threading.Lock()
    _active = None

    def __init__(self, library=None):
        """准备库路径和回调，不连接服务；library可指向ARM64或本机匹配架构的库。"""
        self.library_path = resolve_library(library)
        self.events = queue.Queue()
        self._lock = threading.Lock()
        self._sequence = 0
        self._error = None
        self._library = None
        self._started = False
        self._callback = SDK_CALLBACK(self._receive)

    def _receive(self, context, callback_type, status, user_data):
        """复制回调生命周期内的数据并入队；异常锁存为事件，不向C调用栈抛出。"""
        del context
        with self._lock:
            self._sequence += 1
            received = time.monotonic()
            device_id, payload, error = '', b'', None
            try:
                if callback_type == DEVICE_STATE_JSON:
                    if not user_data:
                        raise ValueError('设备JSON回调指针为空')
                    state = ctypes.cast(user_data, ctypes.POINTER(DeviceStateJson)).contents
                    payload = bytes(state.stateJson).split(b'\0', 1)[0]
                    device_id = bytes(state.devID).split(b'\0', 1)[0].decode('utf-8')
                elif callback_type in (DEVICE_FIND, DEVICE_MISSING, DEVICE_CONNECT):
                    if not user_data:
                        raise ValueError('设备状态回调指针为空')
                    payload = ctypes.string_at(user_data)
                    device_id = payload.decode('utf-8')
            except Exception as exc:
                error = f'{type(exc).__name__}: {exc}'
                if self._error is None:
                    self._error = error
            event = SDKEvent(self._sequence, received,
                             EVENT_NAMES.get(callback_type, 'unknown'), callback_type,
                             status, device_id, base64.b64encode(payload).decode('ascii'), error)
            self.events.put(event)

    def start(self):
        """初始化SDK；失败后确认终止才释放回调所有权，终止失败时保留强引用。"""
        with self._active_lock:
            if SDKSource._active is not None:
                raise RuntimeError('当前进程已有SDKSource连接')
            SDKSource._active = self
        try:
            self._library = ctypes.CDLL(str(self.library_path))
            self._library.PXREAInit.argtypes = [ctypes.c_void_p, SDK_CALLBACK, ctypes.c_uint]
            self._library.PXREAInit.restype = ctypes.c_int
            self._library.PXREADeinit.argtypes = []
            self._library.PXREADeinit.restype = ctypes.c_int
            result = self._library.PXREAInit(None, self._callback, CALLBACK_MASK)
            if result != 0:
                cleanup = self._library.PXREADeinit()
                if cleanup != 0:
                    self._started = True
                    raise RuntimeError(f'PXREAInit失败，code={result}；终止失败，code={cleanup}')
                raise RuntimeError(f'PXREAInit失败，code={result}')
            self._started = True
        except Exception:
            if not self._started:
                with self._active_lock:
                    SDKSource._active = None
            raise
        return self

    def read(self, timeout=None):
        """按FIFO取出一条事件；timeout秒内无事件返回None，回调异常在事件字段保留。"""
        try:
            return self.events.get(timeout=timeout)
        except queue.Empty:
            return None

    @property
    def error(self):
        """返回首次回调异常文本；成功帧不会清除此错误。"""
        with self._lock:
            return self._error

    @property
    def backlog(self):
        """返回当前尚未消费的回调数，供日志观测。"""
        return self.events.qsize()

    @property
    def is_closed(self):
        """返回SDK是否尚未初始化或已确认终止；关闭失败保持False。"""
        return not self._started

    def close(self):
        """确认SDK终止后释放回调所有权；返回码非零保留引用并抛出RuntimeError。"""
        if not self._started:
            return
        result = self._library.PXREADeinit()
        if result != 0:
            raise RuntimeError(f'PXREADeinit失败，code={result}')
        self._started = False
        with self._active_lock:
            if SDKSource._active is self:
                SDKSource._active = None
