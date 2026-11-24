#!/usr/bin/env python3
"""在机器人容器中录制单次示教，并维护该会话的实验清单。"""

import argparse
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


DEFAULT_ENDPOINT = 'tcp://127.0.0.1:5555'
MANAGED_OPTIONS = ('--publish-xr-raw', '--session-id', '--xr-output', '--log-dir', '--help')
READBACK_ERRORS = (OSError, ValueError, KeyError, TypeError, OverflowError, ImportError, RuntimeError)


def read_object(path):
    """读取 path 所指的 JSON 对象；文件或结构无效时向调用方报错。"""
    value = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise ValueError(f'{path} 须包含 JSON 对象')
    return value


def write_json(path, value):
    """在同一目录写入临时文件后原子替换 path；写入失败时保留上一次结果。"""
    with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                     prefix=f'.{path.name}.', delete=False) as stream:
        temporary = Path(stream.name)
        try:
            json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
            stream.close()
            temporary.replace(path)
        finally:
            temporary.unlink(missing_ok=True)


@contextmanager
def update_manifest(directory):
    """独占更新会话清单；读回结果与人工标注的并发更新各自保留。"""
    descriptor = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX)
        path = directory / 'experiment.json'
        manifest = read_object(path)
        yield manifest
        write_json(path, manifest)
    finally:
        os.close(descriptor)


def clock_anchor():
    """采集本机系统与单调时钟锚点，时间及采样区间长度均为整数纳秒。"""
    before = time.monotonic_ns()
    system = time.time_ns()
    after = time.monotonic_ns()
    return dict(system_time_ns=system, monotonic_time_ns=(before + after) // 2,
                sample_span_ns=after - before)


def control(command, options):
    """用现有客户端发送采集命令，STATUS 返回对象；客户端失败抛出 RuntimeError。"""
    result = subprocess.run(
        ['ros2', 'run', 'data_collection', 'collection_control', command,
         '--endpoint', options.endpoint, '--timeout-ms', str(options.timeout_ms)],
        capture_output=True, text=True, start_new_session=True, check=False)
    if result.returncode:
        raise RuntimeError(f'{command} 请求失败：{result.stderr.strip() or result.stdout.strip()}')
    return json.loads(result.stdout) if command == 'STATUS' else None


def finished(status):
    """根据采集状态判断会话是否已经关门、排空并关闭 MCAP。"""
    return (status.get('recording_enabled') is False and status.get('closed') is True
            and status.get('queue_pending') == 0)


def finish_capture(options, directory):
    """查询并停止 directory 指定的会话，返回停止后的状态和可选 STOP 错误。"""
    status = control('STATUS', options)
    if status.get('session_dir') != str(directory):
        raise RuntimeError('采集器当前会话已变化，请核对待收尾的目录')
    error = None
    if not finished(status):
        try:
            control('STOP', options)
        except RuntimeError as exc:
            error = str(exc)
        status = control('STATUS', options)
        if status.get('session_dir') != str(directory):
            raise RuntimeError('STOP 后采集器会话已变化')
    return status, error


def log_exit_reason(path):
    """从日志尾部读取最后一条 exit 的 reason；缺日志或尾条不完整时返回 None。"""
    if not path.is_file():
        return None
    with path.open('rb') as stream:
        offset = stream.seek(0, os.SEEK_END)
        tail = b''
        while offset and b'\n' not in tail.rstrip(b'\n'):
            size = min(offset, 4096)
            offset -= size
            stream.seek(offset)
            tail = stream.read(size) + tail
    try:
        row = json.loads(tail.rstrip(b'\n').rsplit(b'\n', 1)[-1])
        return row['fields']['reason'] if row['event'] == 'exit' else None
    except (KeyError, TypeError, ValueError, UnicodeError):
        return None


def record(options):
    """解析实验条件和遥操参数，以该端点的单实例锁运行一次会话并返回退出码。"""
    conditions = read_object(options.conditions.expanduser())
    json.dumps(conditions, allow_nan=False)
    args = options.teleop_args
    if args[:1] != ['--']:
        raise ValueError('遥操参数须放在 -- 之后')
    args = args[1:]
    for arg in args:
        flag = arg.split('=', 1)[0]
        if flag in ('--', '-h') or (flag.startswith('--') and
                                  any(option.startswith(flag) for option in MANAGED_OPTIONS)):
            raise ValueError(f'{flag} 由会话入口管理')
    settings = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    settings.add_argument('--device-id', required=True)
    teleop, _ = settings.parse_known_args(args)
    endpoint_key = hashlib.sha256(options.endpoint.encode()).hexdigest()[:16]
    lock_path = Path(tempfile.gettempdir()) / f'aimdk-fsc-trial-{endpoint_key}.lock'
    with lock_path.open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError('该采集端点已有实验录制入口') from error
        return record_session(options, conditions, args, teleop.device_id)


def record_session(options, conditions, args, device_id):
    """协调一次 START、遥操、STOP 和清单落盘，返回会话进程退出码。"""
    directory = child = manifest = None
    start_error = None
    interrupted = None
    interrupt_forwarded = False
    returncode = 1

    def interrupt_child():
        """向已启动的遥操进程组转交一次 SIGINT，触发遥操自己的正常收尾。"""
        nonlocal interrupt_forwarded
        if child is not None and not interrupt_forwarded and child.poll() is None:
            interrupt_forwarded = True
            try:
                os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError:
                pass

    def forward_interrupt(signum, _frame):
        """记住首次中断信号；遥操已启动时转交 SIGINT 触发正常收尾。"""
        nonlocal interrupted
        if interrupted is None:
            interrupted = signum
            interrupt_child()

    handlers = {signum: signal.signal(signum, forward_interrupt)
                for signum in (signal.SIGINT, signal.SIGTERM)}
    try:
        previous = control('STATUS', options)
        if previous['recording_enabled']:
            raise RuntimeError('采集器已有录制会话')
        if 'xr_raw' not in previous['metadata']['selected_streams']:
            raise ValueError('请先用 streams 选中 xr_raw 启动采集器')
        if interrupted is not None:
            return 128 + interrupted
        start = clock_anchor()
        try:
            control('START', options)
        except RuntimeError as error:
            start_error = str(error)
        try:
            active = control('STATUS', options)
        except (RuntimeError, ValueError) as error:
            start_error = start_error or str(error)
            active = control('STATUS', options)
        path = active.get('session_dir')
        if not path or path == previous.get('session_dir') or not Path(path).is_absolute():
            raise RuntimeError(start_error or '采集器未提供新会话的绝对路径')
        directory = Path(path)
        if (directory / 'experiment.json').exists():
            directory = None
            raise RuntimeError('新会话目录已有实验清单')
        command = ['ros2', 'run', 'aimdk_functions', 'upper_body_teleop_observe', *args,
                   '--publish-xr-raw', '--session-id', directory.name,
                   '--xr-output', str(directory / 'xr.jsonl'),
                   '--log-dir', str(directory / 'control')]
        manifest = dict(
            schema_version=1, trial_id=directory.name, conditions=conditions,
            teleop=dict(command=command, device_id=device_id, returncode=None, exit_reason=None),
            capture=dict(state='recording', endpoint=options.endpoint, closed=None, data_complete=None,
                         readback_returncode=None),
            artifacts=dict(session='session.json', raw='raw', mcap_storage='mcap_storage.yaml',
                           xr=None, control=None, console=None, readback=None),
            outcome=dict(status='unlabeled'), clocks=dict(start=start))
        write_json(directory / 'experiment.json', manifest)
        print(f'会话目录 {directory}', flush=True)
        if start_error:
            raise RuntimeError(start_error)
        if not active['recording_enabled']:
            raise RuntimeError('采集器已停止接收，请查看会话摘要')
        if interrupted is None:
            with (directory / 'console.txt').open('x', encoding='utf-8', buffering=1) as console:
                child = subprocess.Popen(
                    command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                    encoding='utf-8', errors='replace', start_new_session=True,
                    env=dict(os.environ, PYTHONUNBUFFERED='1'))
                if interrupted is not None:
                    interrupt_child()
                with child.stdout:
                    for line in child.stdout:
                        console.write(line)
                        print(line, end='', flush=True)
                returncode = child.wait()
    except (OSError, RuntimeError, ValueError, KeyError) as error:
        if manifest is not None:
            manifest['teleop']['error'] = str(error)
        print(f'实验录制失败：{error}', file=sys.stderr)
    finally:
        try:
            if child is not None:
                interrupt_child()
                child.wait()
            if directory is not None:
                if not finalize(directory, options, manifest, child):
                    returncode = returncode or 1
        finally:
            for signum, handler in handlers.items():
                signal.signal(signum, handler)
    if interrupted is not None:
        return 128 + interrupted
    return returncode if returncode >= 0 else 128 - returncode


def finalize(directory, options, manifest, child):
    """收尾采集并记录可用产物；返回采集存储是否确认正常关闭。"""
    status, error = {}, None
    try:
        status, error = finish_capture(options, directory)
    except (OSError, RuntimeError, ValueError) as exc:
        error = str(exc)
        try:
            summary = read_object(directory / 'session.json')
            if summary.get('session_dir') == str(directory) and summary.get('closed') is True:
                status = dict(summary, recording_enabled=False)
        except (OSError, ValueError):
            pass
    if error:
        print(f'会话收尾 {directory}：{error}', file=sys.stderr)
    if manifest is not None:
        capture = manifest['capture']
        capture.update({name: status.get(name) for name in (
            'closed', 'complete', 'data_complete', 'queue_pending', 'last_error')})
        capture.update(state='finished' if finished(status) else 'unconfirmed', stop_error=error)
        xr_path = directory / 'xr.jsonl'
        control_path = next(iter(sorted((directory / 'control').glob('*.jsonl'))), None)
        manifest['teleop'].update(
            returncode=None if child is None else child.returncode,
            exit_reason=(log_exit_reason(control_path) if control_path else None)
            or log_exit_reason(xr_path))
        manifest['artifacts'].update(xr='xr.jsonl' if xr_path.is_file() else None,
                                     control=str(control_path.relative_to(directory)) if control_path else None,
                                     console='console.txt' if (directory / 'console.txt').is_file() else None)
        manifest['clocks']['end'] = clock_anchor()
        write_json(directory / 'experiment.json', manifest)
    return finished(status) and status.get('complete') is True


def prepare_xr_readback(directory, session, manifest, options):
    """根据摘要、实验清单和显式选项返回 XR 话题及校验器，配置错误向调用方传播。"""
    from remote_control.readback import XrReadback

    stream = next((entry for entry in session['mapping']['streams'] if entry['id'] == 'xr_raw'), None)
    if stream is None:
        raise ValueError('会话未选择 xr_raw')
    if stream['message_type'] != 'std_msgs/msg/String':
        raise ValueError('xr_raw 必须使用 std_msgs/msg/String')

    if manifest is not None:
        device = manifest['teleop']['device_id']
        trial_id = manifest['trial_id']
        if not isinstance(device, str) or not device or not isinstance(trial_id, str) or not trial_id:
            raise ValueError('实验清单须声明有效的目标设备和试次编号')
        artifact = manifest['artifacts']['xr']
        reference = directory / artifact if artifact else None
        return stream['topic'], XrReadback(device, reference, None, expected_session_id=trial_id)

    reference, config = options.xr_jsonl, options.xr_input_config
    if reference is not None:
        reference = reference.expanduser()
    elif (directory / 'xr.jsonl').exists():
        reference = directory / 'xr.jsonl'
    if config is not None:
        config = config.expanduser()
    return stream['topic'], XrReadback(options.xr_device_id, reference, config)


def inspect_trial(directory, manifest, options):
    """流式核对会话的 MCAP 和 XR，返回组合报告及退出码 0、1 或 2。"""
    try:
        from data_collection.readback import inspect_session, read_session, verify_mcap

        session = read_session(directory)
        report = inspect_session(directory, session=session)
    except READBACK_ERRORS as error:
        return dict(structure_valid=False, error=str(error), raw_mcap_verified=False, xr_readback=None), 2

    xr = xr_report = None
    handlers = {}
    try:
        xr_topic, xr = prepare_xr_readback(directory, session, manifest, options)
    except READBACK_ERRORS as error:
        xr_report = dict(quality_ok=False, error=str(error))
    else:
        def observe_xr_message(message, _received_ns):
            """将已反序列化的 String 消息正文交给 XR 协议校验器。"""
            xr.observe(message.data)

        handlers[xr_topic] = observe_xr_message

    try:
        report['mcap_readback'] = verify_mcap(directory, session=session, message_handlers=handlers)
    except READBACK_ERRORS as error:
        report.update(mcap_readback=dict(verified=False, error=str(error)), xr_readback=xr_report)
        return report, 2
    report['raw_mcap_verified'] = True
    if xr is not None:
        xr_report = xr.report()
    report['xr_readback'] = xr_report
    if 'error' in xr_report:
        return report, 2
    incomplete = (not report['data_complete'] or not xr_report['quality_ok']
                  or xr_report['jsonl_comparison'] is None)
    return report, int(incomplete)


def verify(directory, manifest, options):
    """保存组合读回报告，更新已有实验清单并返回核验退出码。"""
    report, returncode = inspect_trial(directory, manifest, options)
    if manifest is None:
        write_json(directory / 'readback.json', report)
    else:
        with update_manifest(directory) as current:
            write_json(directory / 'readback.json', report)
            current['artifacts']['readback'] = 'readback.json'
            current['capture']['readback_returncode'] = returncode
    print(directory / 'readback.json')
    return returncode


def main():
    """分派录制、读回或任务标注命令，错误返回非零退出码。"""
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest='action', required=True)
    recording = actions.add_parser('record', help='录制一次完整示教')
    recording.add_argument('--conditions', type=Path, required=True, help='实验条件 JSON 对象文件')
    recording.add_argument('--endpoint', default=DEFAULT_ENDPOINT)
    recording.add_argument('--timeout-ms', type=int, default=5000)
    recording.add_argument('teleop_args', nargs=argparse.REMAINDER, help='-- 后传入遥操参数')
    checking = actions.add_parser('verify', help='读回 MCAP 和 XR 并保存报告，支持独立 XR 会话')
    checking.add_argument('session_dir', type=Path)
    checking.add_argument('--xr-jsonl', type=Path, help='独立会话的同次 XR JSONL，默认会话内的 xr.jsonl')
    checking.add_argument('--xr-device-id', help='独立会话的目标设备标识，默认取 XR JSONL 元数据')
    checking.add_argument('--xr-input-config', type=Path, help='独立会话的 XR 输入协议 JSON，默认取 XR JSONL 元数据')
    labeling = actions.add_parser('label', help='更新本次示教的任务结果')
    labeling.add_argument('session_dir', type=Path)
    labeling.add_argument('--outcome', required=True, choices=('success', 'failure', 'aborted', 'unlabeled'))
    labeling.add_argument('--stage')
    labeling.add_argument('--reason')
    options = parser.parse_args()
    try:
        if options.action == 'record':
            if options.timeout_ms <= 0:
                parser.error('--timeout-ms 必须大于 0')
            return record(options)
        directory = options.session_dir.expanduser().resolve()
        manifest_path = directory / 'experiment.json'
        if options.action == 'verify':
            if not manifest_path.exists():
                return verify(directory, None, options)
            if any(value is not None for value in (options.xr_jsonl, options.xr_device_id, options.xr_input_config)):
                checking.error('--xr-jsonl、--xr-device-id 和 --xr-input-config 仅适用于无 experiment.json 的会话')
        manifest = read_object(manifest_path)
        if manifest['capture']['state'] == 'recording':
            raise ValueError('请等待录制入口写入会话终态')
        if options.action == 'verify':
            return verify(directory, manifest, options)
        with update_manifest(directory) as current:
            current['outcome'] = dict(status=options.outcome, stage=options.stage, reason=options.reason,
                                      labeled_system_time_ns=time.time_ns())
        return 0
    except (OSError, RuntimeError, ValueError, KeyError) as error:
        print(f'实验会话处理失败：{error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
