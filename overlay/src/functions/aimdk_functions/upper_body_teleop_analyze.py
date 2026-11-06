"""离线分析遥操关节及名义TCP误差、补偿分段和命令至反馈表观延迟；不导入ROS。"""

import argparse
import csv
import hashlib
import json
import math
from operator import itemgetter
from pathlib import Path
import sys

import numpy as np

from .upper_body_kinematics.parameters import model_path
from .upper_body_kinematics.vendor.x2_arm_model import ArmModel


def statistics(values):
    """返回沿样本轴的有符号均值、RMSE、绝对值P95及最大值；空样本返回None。"""
    array = np.asarray(values, dtype=float)
    if not array.size:
        return None
    return dict(samples=len(array), mean=np.mean(array, axis=0).tolist(),
                rmse=np.sqrt(np.mean(array**2, axis=0)).tolist(),
                p95_abs=np.percentile(np.abs(array), 95, axis=0).tolist(),
                maximum_abs=np.max(np.abs(array), axis=0).tolist())


def rotation_error(actual, desired):
    """返回两个旋转矩阵之间的最短角，单位度，裁剪仅处理反余弦浮点误差。"""
    cosine = (np.trace(actual @ np.asarray(desired).T) - 1.) / 2.
    return float(np.degrees(np.arccos(np.clip(cosine, -1., 1.))))


def load_log(path):
    """逐行读取需要的事件，保留源行号；损坏记录报错，返回命令、目标、配置及反馈索引。"""
    commands, accepted, mappings, feedback = [], {}, {}, {}
    metadata, config, counts, exits = {}, None, dict(arm=0, pelvis_imu=0, waist=0), []
    with Path(path).open(encoding='utf-8') as stream:
        for number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                event, fields = row['event'], row['fields']
                if event == 'metadata':
                    metadata = fields
                elif event == 'online_ik_config':
                    config = fields
                elif event == 'command_published':
                    commands.append(dict(fields=fields, monotonic_s=row['monotonic_s'], line=number))
                elif event == 'ik_accepted':
                    accepted[fields['input_seq']] = fields['result']['arms']
                elif event == 'mapping_candidate':
                    mappings[fields['input_seq']] = fields['arms']
                elif event == 'feedback':
                    name = fields['stream']
                    counts[name] = counts.get(name, 0) + 1
                    if name == 'arm':
                        feedback[fields['serial']] = fields
                elif event in ('error', 'exit'):
                    exits.append(dict(event=event, fields=fields))
            except (ValueError, TypeError, KeyError) as error:
                raise ValueError(f'{path}:{number}: {error}') from error
    if not commands or config is None:
        raise ValueError('需要含online_ik_config及command_published的遥操执行日志')
    return dict(commands=commands, accepted=accepted, mappings=mappings, feedback=feedback,
                metadata=metadata, config=config, feedback_counts=counts, exits=exits)


def make_model(config, side):
    """核对日志模型摘要，按日志工具变换构造指定侧模型；不使用其他工具配置代替。"""
    path = model_path()
    if hashlib.sha256(path.read_bytes()).hexdigest() != config['model_sha256']:
        raise ValueError('当前URDF摘要与日志不一致，不能用当前模型解释该日志')
    tool = config['tools'][side]
    return ArmModel(side, urdf_path=path, mjcf_path=None,
                    tcp_offset=np.asarray(tool['translation_m']),
                    tcp_rotation=np.asarray(tool['rotation_matrix']))


def command_time(command):
    """返回发布调用开始时间；历史日志缺字段时使用日志记录时间并由报告标明局限。"""
    return command['fields'].get('publish_begin_monotonic_s', command['monotonic_s'])


def make_records(data, side, start=0., end=None):
    """在相对首条发布的时间窗口内重算分侧误差；返回记录和逐条所用反馈序号。"""
    model = make_model(data['config'], side)
    offset = 0 if side == 'left' else 7
    origin = command_time(data['commands'][0])
    records = []
    for command in data['commands']:
        fields = command['fields']
        elapsed = command_time(command) - origin
        if elapsed < start or (end is not None and elapsed > end):
            continue
        joints = {name: np.asarray(fields[key][offset:offset+7], dtype=float)
                  for name, key in (('ideal', 'ideal_arm_rad'), ('sent', 'arm_rad'),
                                    ('feedback', 'actual_arm_rad'))}
        poses = {name: model.forward_kinematics(q) for name, q in joints.items()}
        seq = fields.get('input_seq')
        arm = data['accepted'].get(seq, {}).get(side)
        if seq is not None and arm is None:
            raise ValueError(f"第{command['line']}行发布目标找不到已接受IK输入{seq}")
        # 保持时存储目标可已撤销远端终点，IK残差仍用原接受事件的真实候选计算。
        ik_joints = np.asarray(arm['candidate_rad'] if arm is not None and 'joint_approach' in fields
                               else joints['ideal'], dtype=float)
        ik_pose = model.forward_kinematics(ik_joints)
        desired = (poses['ideal'][0] if arm is None else np.asarray(arm['requested_position_m']))
        desired_rotation = (poses['ideal'][1] if arm is None else arm['requested_rotation_matrix'])
        compensation = fields.get('compensation')
        detail = (compensation or {}).get('arms', {}).get(side, {})
        blend = detail.get('blend')
        phase = ('startup' if seq is None else 'ramp' if blend is not None and blend < 1.
                 else 'holding' if not arm['enabled'] else 'tracking')
        record = dict(time_s=elapsed, sequence=fields['sequence'], input_seq=seq, phase=phase,
                      feedback_serial=fields.get('arm_feedback_serial'), blend=blend,
                      publish_begin_monotonic_s=fields.get('publish_begin_monotonic_s'),
                      publish_end_monotonic_s=fields.get('publish_end_monotonic_s'),
                      feedback_received_monotonic_s=fields.get('arm_feedback_received_monotonic_s'),
                      feedback_stamp_s=fields.get('arm_feedback_stamp_s'),
                      publish_gap_s=fields.get('publish_gap_s'),
                      saturated_joints=sum(detail.get('bias_saturated', [])),
                      rotation_error_deg=rotation_error(poses['feedback'][1], desired_rotation),
                      ik_rotation_error_deg=rotation_error(ik_pose[1], desired_rotation),
                      joint_approach_paused=fields.get('joint_approach', {}).get('paused', False))
        for name, vector in (
                ('ideal_rad', joints['ideal']), ('sent_rad', joints['sent']),
                ('feedback_rad', joints['feedback']),
                ('ik_solution_rad', ik_joints),
                ('approach_error_deg', np.degrees(joints['ideal']-ik_joints)),
                ('bias_deg', np.degrees(joints['sent']-joints['ideal'])),
                ('send_error_deg', np.degrees(joints['feedback']-joints['sent'])),
                ('ideal_error_deg', np.degrees(joints['feedback']-joints['ideal'])),
                ('ik_mm', (ik_pose[0]-desired)*1000),
                ('approach_mm', (poses['ideal'][0]-ik_pose[0])*1000),
                ('compensation_mm', (poses['sent'][0]-poses['ideal'][0])*1000),
                ('tracking_mm', (poses['feedback'][0]-poses['sent'][0])*1000),
                ('task_mm', (poses['feedback'][0]-desired)*1000),
                ('gravity_torque_nm', detail.get('gravity_torque_nm', [])),
                ('raw_bias_deg', np.degrees(detail.get('raw_bias_rad', [])))):
            record[name] = np.asarray(vector).tolist()
        if record['feedback_received_monotonic_s'] is not None:
            record['feedback_age_s'] = (command_time(command)
                                        - record['feedback_received_monotonic_s'])
        else:
            record['feedback_age_s'] = None
        record['mapping_event'] = data['mappings'].get(seq, {}).get(side, {}).get('event')
        records.append(record)
    return records


def summarize_records(records):
    """对所选窗口及阶段统计关节、TCP分解与反馈复用；按发布计数，另报唯一反馈数。"""
    metrics = {}
    for key in ('bias_deg', 'send_error_deg', 'ideal_error_deg', 'ideal_rad', 'ik_mm',
                'approach_mm', 'approach_error_deg',
                'compensation_mm', 'tracking_mm', 'task_mm', 'rotation_error_deg',
                'ik_rotation_error_deg', 'feedback_age_s', 'publish_gap_s'):
        metrics[key] = statistics([row[key] for row in records if row[key] is not None])
    for key in ('ik_mm', 'approach_mm', 'compensation_mm', 'tracking_mm', 'task_mm'):
        metrics[key+'_norm'] = statistics([np.linalg.norm(row[key]) for row in records])
    serials = [row['feedback_serial'] for row in records if row['feedback_serial'] is not None]
    return dict(published_samples=len(records), unique_feedback_samples=len(set(serials)),
                samples_without_feedback_identity=len(records)-len(serials),
                reused_feedback_samples=len(serials)-len(set(serials)),
                saturated_publish_samples=sum(row['saturated_joints'] > 0 for row in records),
                metrics=metrics)


def fit_delay(data, records, side, maximum, step, max_gap):
    """用独立反馈时间与历史零阶保持命令拟合非负表观延迟及偏差，后1/3片段独立评价。

    maximum由操作者按停走响应设置，step为搜索分辨率，max_gap为允许跨越的命令间隔秒数。
    仅使用有精确反馈标识的记录；历史日志返回不可拟合说明，不猜测采样时刻。
    """
    commands = data['commands']
    if any('publish_begin_monotonic_s' not in item['fields'] for item in commands):
        return dict(status='unavailable', reason='历史日志缺发布调用时间，不能精确拟合')
    offset = 0 if side == 'left' else 7
    times = np.array([command_time(item) for item in commands])
    sent = np.array([item['fields']['arm_rad'][offset:offset+7] for item in commands])
    seen, samples = set(), []
    for row in records:
        serial = row['feedback_serial']
        if serial is None or serial in seen or row['feedback_received_monotonic_s'] is None:
            continue
        seen.add(serial)
        samples.append(row)
    samples.sort(key=itemgetter('feedback_received_monotonic_s'))
    if len(samples) < 6:
        return dict(status='unavailable', reason='独立反馈不足以划分拟合与验证片段')
    received = np.array([row['feedback_received_monotonic_s'] for row in samples])
    actual = np.array([row['feedback_rad'] for row in samples])
    delays = np.arange(0., maximum + step/2., step)
    delays = delays[delays <= maximum]
    valid = np.ones(len(samples), dtype=bool)
    indices = []
    for delay in delays:
        query = received-delay
        index = np.searchsorted(times, query, side='right')-1
        bounded = np.clip(index, 0, len(times)-1)
        valid &= (index >= 0) & (query <= times[-1]) & (query-times[bounded] <= max_gap)
        following = np.minimum(bounded+1, len(times)-1)
        valid &= times[following]-times[bounded] <= max_gap
        indices.append(bounded)
    actual, received = actual[valid], received[valid]
    if len(actual) < 6:
        return dict(status='unavailable', reason='共同有效时间区间内反馈不足；核对窗口、缺口和延迟上界')
    split = len(actual)*2//3
    baseline = sent[indices[0][valid]]
    active = np.ptp(baseline[:split], axis=0) > 1e-9
    if not active.any():
        return dict(status='unavailable', reason='拟合片段没有变化的发送关节目标')
    fits = []
    for delay, index in zip(delays, indices):
        error = actual-sent[index[valid]]
        bias = error[:split].mean(axis=0)
        score = np.mean((error[:split, active]-bias[active])**2)
        fits.append((score, float(delay), bias, error))
    best = min(fits, key=itemgetter(0))
    score, delay, bias, error = best
    return dict(status='estimated', delay_s=delay, search_max_s=maximum, search_step_s=step,
                at_search_boundary=bool(delay == delays[-1]), feedback_time='local_receive_monotonic',
                fit_samples=split, validation_samples=len(actual)-split,
                active_joint_indices=np.flatnonzero(active).tolist(),
                fitted_bias_deg=np.degrees(bias).tolist(),
                zero_delay_error_deg=statistics(np.degrees(fits[0][3])),
                aligned_error_deg=statistics(np.degrees(error)),
                zero_delay_validation_demeaned_rmse_deg=math.degrees(float(np.sqrt(np.mean(
                    (fits[0][3][split:, active]-fits[0][2][active])**2)))),
                fit_demeaned_rmse_deg=math.degrees(math.sqrt(score)),
                validation_demeaned_rmse_deg=math.degrees(float(np.sqrt(np.mean(
                    (error[split:, active]-bias[active])**2)))))


def write_csv(path, records):
    """将向量展开为带索引列的CSV，保持原始关节rad和误差单位，空字段留空。"""
    flattened = []
    for row in records:
        result = {}
        for name, value in row.items():
            if isinstance(value, list):
                result.update({f'{name}_{index}': number for index, number in enumerate(value)})
            else:
                result[name] = value
        flattened.append(result)
    columns = list(dict.fromkeys(key for row in flattened for key in row))
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(flattened)


def plot_records(path, records):
    """使用可选matplotlib导出关节与TCP分解曲线；未安装时明确报错，不影响无图分析。"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    times = [row['time_s'] for row in records]
    figure, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)
    for index, key in enumerate(('ideal_error_deg', 'send_error_deg', 'bias_deg')):
        axes[index].plot(times, [row[key] for row in records])
        axes[index].set_ylabel(key)
        axes[index].grid(alpha=.3)
    for key in ('ik_mm', 'approach_mm', 'compensation_mm', 'tracking_mm', 'task_mm'):
        axes[3].plot(times, [np.linalg.norm(row[key]) for row in records], label=key)
    axes[3].legend()
    axes[3].set_ylabel('TCP norm (mm)')
    axes[3].set_xlabel('Time since first publish (s)')
    figure.tight_layout()
    figure.savefig(path, dpi=150)
    plt.close(figure)


def parser():
    """定义日志、侧别、分析窗口、延迟搜索及可选绘图参数，不提供运动执行功能。"""
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('log', help='遥操执行JSONL')
    result.add_argument('--output', required=True, help='新建输出目录，拒绝覆盖已有目录')
    result.add_argument('--side', choices=('left', 'right', 'both'), default='both')
    result.add_argument('--start', type=float, default=0., help='相对首条发布的窗口起点秒数')
    result.add_argument('--end', type=float, help='窗口终点秒数，默认到日志结束')
    result.add_argument('--delay-max', type=float, help='显式启用延迟拟合，提供有依据的搜索上界秒数')
    result.add_argument('--delay-step', type=float, default=.002, help='延迟搜索步长秒数，默认0.002')
    result.add_argument('--max-command-gap', type=float, default=.08, help='历史命令允许间隔秒数，沿用0.08')
    result.add_argument('--plots', action='store_true', help='另输出PNG，需要本地安装matplotlib')
    return result


def main(args=None):
    """离线生成分侧CSV、JSON及解释报告；成功返回0，格式或文件错误返回1。"""
    options = parser().parse_args(args)
    try:
        for value in (options.start, options.end, options.delay_max,
                      options.delay_step, options.max_command_gap):
            if value is not None and not math.isfinite(value):
                raise ValueError('时间参数必须有限')
        if (options.start < 0 or (options.end is not None and options.end <= options.start)
                or (options.delay_max is not None and options.delay_max <= 0)
                or options.delay_step <= 0 or options.max_command_gap <= 0):
            raise ValueError('窗口需满足0≤start<end，延迟上界、步长及间隔须为正')
        source = Path(options.log).expanduser().resolve()
        data = load_log(source)
        sides = ('left', 'right') if options.side == 'both' else (options.side,)
        records = {side: make_records(data, side, options.start, options.end) for side in sides}
        if any(not rows for rows in records.values()):
            raise ValueError('所选窗口没有发布记录')
        report = dict(source=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                      options=vars(options), frame='torso_link', model=data['config'],
                      metadata=data['metadata'], feedback_counts=data['feedback_counts'],
                      exits=data['exits'], sides={})
        for side, rows in records.items():
            report['sides'][side] = dict(
                all=summarize_records(rows),
                phases={phase: summarize_records([row for row in rows if row['phase'] == phase])
                        for phase in ('startup', 'ramp', 'holding', 'tracking')})
            if options.delay_max is not None:
                report['sides'][side]['delay'] = fit_delay(
                    data, rows, side, options.delay_max, options.delay_step, options.max_command_gap)
        output = Path(options.output).expanduser().resolve()
        output.mkdir(parents=True, exist_ok=False)
        for side, rows in records.items():
            write_csv(output / f'{side}.csv', rows)
            if options.plots:
                plot_records(output / f'{side}.png', rows)
        (output / 'summary.json').write_text(
            json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
        (output / 'report.md').write_text(
            '# 遥操误差分析\n\n'
            f'源日志：`{source}`。结果见`summary.json`及分侧CSV。\n\n'
            '位置单位mm，关节误差单位deg，原始关节rad；向量按该侧关节0..6或XYZ排列。\n'
            '统计按发布事件计数，重复反馈数量另列。holding表示Grip释放后的目标保持，'
            '不证明实体已稳定；ramp覆盖补偿渐入。startup的IK残差以初始理想FK定义为零。\n'
            '新执行日志的approach_mm为已发布理想FK减原接受IK候选FK，包含渐进滞后及保持时撤销终点的差异。'
            '四项有符号位置分解ik+approach+compensation+tracking等于task；历史日志approach为零。\n'
            '历史日志缺反馈标识和发布时刻时不拟合延迟。延迟为发送至反馈接收的表观估计，'
            '不代表内部执行时刻；前2/3拟合，后1/3验证，边界最优需复核搜索范围。\n'
            '名义FK误差不包含独立实体标定和世界坐标测量。\n', encoding='utf-8')
        print(f'分析完成：{output}')
        return 0
    except (OSError, ValueError, TypeError, KeyError, ImportError) as error:
        print(f'遥操分析失败：{error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
