"""遥操阶段事件增量 JSONL 日志；显式日志目录沿用现有工作区 log 下载流程。"""

import hashlib
import json
import math
from pathlib import Path
import time
import threading
import uuid


def json_value(value):
    """递归保留字典/列表中的原始字段，将非有限浮点明确编码为字符串；不改写为零。"""
    if isinstance(value, float) and not math.isfinite(value):
        return 'NaN' if math.isnan(value) else ('inf' if value > 0 else '-inf')
    if isinstance(value, dict):
        return {key: json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_value(item) for item in value]
    return value


def log_directory(value):
    """返回显式目录或按功能包标记探测的工作区log；探测失败要求显式指定目录。"""
    if value:
        return Path(value).expanduser().resolve()
    candidates = (Path.cwd(), *Path.cwd().parents, *Path(__file__).resolve().parents)
    for marker in ('overlay/src/functions/package.xml', 'src/functions/package.xml'):
        for candidate in candidates:
            if (candidate / marker).is_file():
                return candidate / 'log'
    raise ValueError('无法探测工作区，请指定 --log-dir /实际工作区/log')


def code_manifest():
    """记录实际加载安装位置及相关Python文件摘要，不依赖git或开发者源码绝对路径。"""
    root = Path(__file__).resolve().parent
    paths = (list(root.glob('upper_body_*.py'))
             + list((root / 'upper_body_kinematics').rglob('*.py'))
             + [root / 'dual_arm_control_core.py'])
    return {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(paths)}


class UpperBodyLog:
    """逐事件写入并 flush，保留已完成事件；不使用有界内存覆盖历史事件。"""

    def __init__(self, directory, **metadata):
        """在指定 directory 创建独立文件并记录版本等 metadata；文件错误直接传播。"""
        directory = Path(directory).expanduser().resolve()
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / f'upper_body_{time.time_ns()}_{uuid.uuid4().hex[:8]}.jsonl'
        self.stream = self.path.open('x', encoding='utf-8')
        self.lock = threading.RLock()
        self.started = time.monotonic()
        try:
            self.record('metadata', **metadata)
        except Exception:
            self.stream.close()
            raise

    def record(self, event, **fields):
        """增量记录 JSON 兼容 fields；非有限浮点保留为字符串，不支持的对象明确报错。"""
        with self.lock:
            now = time.monotonic()
            line = json.dumps(dict(event=event, wall_time_ns=time.time_ns(),
                                   monotonic_s=now, elapsed_s=now - self.started,
                                   fields=json_value(fields)), ensure_ascii=False, allow_nan=False)
            self.stream.write(line + '\n')
            self.stream.flush()

    def close(self, reason='closed'):
        """写入退出原因并关闭；可重复关闭，写盘错误传播，强制断电持久性不保证。"""
        with self.lock:
            if self.stream.closed:
                return
            try:
                self.record('exit', reason=reason)
            finally:
                self.stream.close()
