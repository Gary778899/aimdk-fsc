"""固定设备的只读常驻监听，复用录制器保存原始回调和标准XR帧。"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from uuid import uuid4

from .cli import run_recording


class WatchStatus:
    """显示服务、固定设备和接收窗口状态，不按等待时长推断断链。"""

    def __init__(self, device_id, interval=1.0, output=print, clock=time.monotonic):
        """保存设备、显示间隔秒、输出函数和单调时钟；非法间隔抛出ValueError。"""
        if not math.isfinite(interval) or interval <= 0:
            raise ValueError('显示间隔须为有限正秒数')
        self.device_id = device_id
        self.interval = interval
        self.output = output
        self.clock = clock
        self.last_report = clock()
        self.server_connected = False
        self.device_seen = False
        self.latest = None
        self.total_frames = 0
        self.window_times = []

    def started(self, path):
        """立即显示日志位置及等待对象；服务连接由后续实际回调确认。"""
        self.output(f'只读监听设备 {self.device_id}；等待Service及头显输入。', flush=True)
        self.output(f'日志：{path}；Ctrl-C结束。', flush=True)

    def consume(self, event, frame):
        """处理实际连接事件与归一化帧；失联后清除显示缓存并继续等待。"""
        labels = {
            'server_connected': 'Service已连接，等待头显发送Controller数据',
            'server_disconnected': 'Service已断开，等待SDK后续连接事件',
            'device_found': '发现设备',
            'device_connected': '设备连接事件',
            'device_missing': '设备离线，继续等待',
        }
        if event.kind == 'server_connected':
            self.server_connected = True
        elif event.kind == 'server_disconnected':
            self.server_connected = False
            self.device_seen = False
            self.latest = None
            self.window_times.clear()
        if event.device_id == self.device_id:
            if event.kind in ('device_found', 'device_connected', 'device_state'):
                self.device_seen = True
            elif event.kind == 'device_missing':
                self.device_seen = False
                self.latest = None
                self.window_times.clear()
        if event.kind in labels:
            self.output(f'{labels[event.kind]}；device={event.device_id!r} status={event.status}',
                        flush=True)
        if frame is not None:
            if self.latest is None:
                self.output(f'开始接收 {self.device_id} 的完整双手柄帧。', flush=True)
            self.latest = frame
            self.total_frames += 1
            self.window_times.append(frame.received_monotonic_s)

    def report(self, backlog):
        """定期显示窗口帧率、接收年龄及最新手柄状态；无新帧时不重复显示旧输入。"""
        now = self.clock()
        if now - self.last_report < self.interval:
            return
        count = len(self.window_times)
        hz = None
        if count > 1 and self.window_times[-1] > self.window_times[0]:
            hz = (count - 1) / (self.window_times[-1] - self.window_times[0])
        age = None if self.latest is None else now - self.latest.received_monotonic_s
        status = dict(service_connected=self.server_connected, device_seen=self.device_seen,
                      device_id=self.device_id, total_frames=self.total_frames,
                      window_frames=count, receive_hz=hz, last_frame_age_s=age, backlog=backlog)
        self.output(json.dumps(status, ensure_ascii=False), flush=True)
        if count and self.latest is not None:
            for side in ('left', 'right'):
                controller = getattr(self.latest, side)
                self.output(f'{side}: ' + json.dumps(controller.to_dict(), ensure_ascii=False),
                            flush=True)
        else:
            self.output('本显示窗口没有新Controller帧；检查头显连接、Controller和Send。', flush=True)
        self.window_times.clear()
        self.last_report = now


def watch_parser():
    """定义已确认机器人部署的默认路径；允许显式覆盖设备、库、配置和日志目录。"""
    parser = argparse.ArgumentParser(description='常驻等待PICO，显示输入并保存XR JSONL日志')
    parser.add_argument('--device-id', default='TestDevice', help='固定设备标识，默认TestDevice')
    parser.add_argument('--input-config', default='/home/agi/aimdk-fsc/data/pico_config/input.json',
                        help='已核对的输入协议配置')
    parser.add_argument('--sdk-library',
                        default='/opt/apps/roboticsservice/SDK/arm64/libPXREARobotSDK.so',
                        help='SDK动态库，默认使用已安装ARM64服务随包库')
    parser.add_argument('--log-dir', default='/home/agi/aimdk-fsc/log', help='自动命名日志所在目录')
    parser.add_argument('--display-interval', type=float, default=1.0, help='终端显示间隔秒，默认1')
    parser.add_argument('--duration', type=float, help='监听总秒数，含等待；省略时持续至Ctrl-C')
    return parser


def watch_main(args=None):
    """启动常驻录制及状态显示；配置错误返回1，其他退出码与xr_record一致。"""
    options = watch_parser().parse_args(args)
    try:
        observer = WatchStatus(options.device_id, options.display_interval)
    except ValueError as error:
        print(f'XR监听失败：{error}', flush=True)
        return 1
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    options.output = str(Path(options.log_dir) / f'xr_watch_{stamp}_{uuid4().hex[:8]}.jsonl')
    return run_recording(options, observer=observer)


if __name__ == '__main__':
    raise SystemExit(watch_main())
