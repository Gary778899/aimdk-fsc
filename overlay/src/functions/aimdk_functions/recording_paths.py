"""功能包记录路径：北京时间到秒，重名追加短序号。"""

from datetime import datetime, timedelta, timezone
from pathlib import Path


def recording_label(started=None):
    """将带时区的 started 转为北京时间标签；省略时使用当前记录开始时间。"""
    return (started or datetime.now(timezone.utc)).astimezone(
        timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')


def create_recording(directory, *, suffix='', started=None):
    """在 directory 独占创建记录目录或带 suffix 的文本文件，返回路径及文件流。

    started 为带时区的开始时间；重名追加递增序号，目录返回的文件流为 None。
    其他文件系统异常直接传播。
    """
    root = Path(directory).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    label = recording_label(started)
    index = 0
    while True:
        path = root / (label + (f'_{index}' if index else '') + suffix)
        try:
            if suffix:
                return path, path.open('x', encoding='utf-8')
            path.mkdir()
            return path, None
        except FileExistsError:
            index += 1
