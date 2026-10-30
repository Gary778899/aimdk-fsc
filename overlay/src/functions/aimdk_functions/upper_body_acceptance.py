"""交互式收集遥操第一阶段前五步验收证据；运动逐项预览并显式确认。"""

import argparse
import json
import math
import os
from pathlib import Path
import platform
import shlex
import signal
import subprocess
import sys
import time
import uuid


def ask(message, default=''):
    """读取一行现场输入；空行采用default，EOF向上传播并保留已有报告。"""
    return input(f'{message}' + (f' [{default}]' if default else '') + ': ').strip() or default


def read_events(directory):
    """读取一个用例目录中的控制器JSONL；格式损坏直接报错，不把缺失记录视为通过。"""
    return [json.loads(line) for path in sorted(directory.glob('upper_body_*.jsonl'))
            for line in path.read_text(encoding='utf-8').splitlines() if line.strip()]


def summarize(events):
    """提取控制器完成证据、原始错误、周期分布和按名称对齐的接收时跟踪误差。

    跟踪误差以反馈接收前最近一条已发送命令为参照，仅作离线观测，不代表硬件同步。
    """
    from .dual_arm_control_core import ARM_NAMES
    from .upper_body_acceptance_replay import distribution

    result = dict(event_count=len(events), errors=[], feedback_errors=[], published_count=0)
    maxima = [0.] * 14
    last_command, gaps, calculations, error_samples = None, [], [], 0
    bias_values, sensor_skews = [], []
    unchanged_arm, unchanged_hand = True, True
    for row in events:
        fields = row['fields']
        event = row['event']
        if event == 'error':
            result['errors'].append(fields)
        if event == 'feedback' and fields.get('error'):
            result['feedback_errors'].append(fields['error'])
        if event in ('metadata', 'plan', 'motion_begin', 'endpoint_confirmed', 'execution_result', 'summary', 'exit'):
            result[event] = fields
        if event == 'commands_stopped':
            result['commands_stopped'] = fields
        if event == 'compensation_config':
            result['compensation_config'] = fields
        if event == 'round_trip':
            result.setdefault('round_trips', []).append(fields)
        if event == 'command_published':
            result['published_count'] += 1
            last_command = fields
            if fields.get('publish_gap_s') is not None:
                gaps.append(fields['publish_gap_s'])
            calculations.append(fields['calculation_seconds'])
            details = fields.get('compensation')
            if details:
                for values in details['arms'].values():
                    bias_values.extend(abs(math.degrees(value)) for value in values['bias_rad'])
                    sensor_skews.append(values['input_source_skew_s'])
            metadata = result.get('metadata', {}).get('options', {})
            plan = result.get('plan')
            if plan:
                side = metadata.get('side')
                held = range(7, 14) if side == 'left' else range(7) if side == 'right' else ()
                if metadata.get('action') == 'gripper':
                    held = range(14)
                unchanged_arm &= all(fields['arm_rad'][i] == plan['start_arm_rad'][i] for i in held)
                unchanged_hand &= fields['hand_openings'] == plan['hand_openings']
        if event == 'feedback' and fields['stream'] == 'arm' and last_command is not None:
            joints = {item['name']: item['position'] for item in fields['raw']['joints']}
            for index, name in enumerate(ARM_NAMES):
                maxima[index] = max(maxima[index], abs(math.degrees(joints[name] - last_command['arm_rad'][index])))
            error_samples += 1
    result.update(publish_gap_s=distribution(gaps), calculation_seconds=distribution(calculations),
                  receive_time_tracking_samples=error_samples,
                  held_arm_command_unchanged=unchanged_arm if last_command else None,
                  planned_hand_command_unchanged=unchanged_hand if last_command else None,
                  compensation_abs_bias_deg=distribution(bias_values),
                  compensation_input_skew_s=distribution(sensor_skews),
                  max_abs_tracking_error_deg=dict(zip(ARM_NAMES, maxima)) if error_samples else {})
    plan, execution = result.get('plan'), result.get('execution_result')
    if plan and execution:
        result['actual_delta_deg'] = [math.degrees(b-a) for a, b in zip(
            plan['start_arm_rad'], execution['actual_arm_rad'])]
        result['endpoint_sent_error_deg'] = [math.degrees(a-b) for a, b in zip(
            execution['actual_arm_rad'], execution['sent_arm_rad'])]
        result['endpoint_ideal_error_deg'] = [math.degrees(a-b) for a, b in zip(
            execution['actual_arm_rad'], execution['ideal_target_rad'])]
    return result


class AcceptanceSession:
    """保存会话、逐项命令与现场判断，汇总覆盖缺口，不自动宣告完整阶段验收。"""

    def __init__(self, options):
        """创建唯一会话目录；默认沿用工作区log及现有日志下载任务。"""
        from .upper_body_kinematics_check import code_manifest, log_directory
        from .upper_body_kinematics.parameters import parameter_report

        self.options = options
        self.root = log_directory(options.log_dir) / (
            time.strftime('upper_body_acceptance_%Y%m%dT%H%M%S_') + uuid.uuid4().hex[:8])
        self.root.mkdir(parents=True)
        self.data = dict(schema_version=1, started=time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                         options=vars(options), python=sys.version, platform=platform.platform(),
                         ros_distro=os.environ.get('ROS_DISTRO', 'unknown'),
                         package_path=str(Path(__file__).resolve().parent),
                         code_sha256=code_manifest(), parameters=parameter_report(),
                         context={}, cases=[], reviews={}, limitations=[
                             '步骤4：当前统一入口无纯旋转及完整位姿目标，相关实机用例未覆盖。',
                             '实体TCP误差、轴向、保持及退出表现须由独立测量和现场观察评价。',
                             '边界与故障用例仅含数学/参数拒绝；断流和反馈故障注入尚未覆盖。'])
        self.save()

    def save(self):
        """原子更新结构化报告和可读报告；每项完成及中断时调用，保留已收集证据。"""
        temporary = self.root / 'report.json.tmp'
        temporary.write_text(json.dumps(self.data, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        temporary.replace(self.root / 'report.json')
        lines = ['# 上肢交互验收记录', '', '结论：待开发者结合覆盖、指标与现场测量审阅；不自动判定前五步全部验收。',
                 '', '完整输入、版本、指标及观察见 report.json；每项原始日志及终端输出在同名子目录。', '',
                 '| 用例 | 自动证据 | 现场判断 |', '| --- | --- | --- |']
        for case in self.data['cases']:
            lines.append(f"| {case['id']} | {case.get('evidence', '未完成')} | {case.get('observation', {}).get('verdict', '未评估')} |")
        lines += ['', '## 分步审阅', '']
        for step in range(1, 6):
            review = self.data['reviews'].get(str(step), {})
            lines.append(f"- 步骤 {step}：{review.get('verdict', '未评估')}；{review.get('reason', '尚无人工审阅记录')}")
        lines += ['', '## 执行指标', '',
                  '| 用例 | 发送帧数 | 最大发布间隔/s | 最大接收时跟踪误差/° |',
                  '| --- | --- | --- | --- |']
        for case in self.data['cases']:
            metrics = case.get('metrics', {})
            if metrics.get('published_count'):
                error = metrics.get('max_abs_tracking_error_deg', {})
                lines.append(f"| {case['id']} | {metrics['published_count']} | "
                             f"{metrics['publish_gap_s'].get('max', '未记录')} | "
                             f"{max(error.values()) if error else '未记录'} |")
        lines += ['', '## 已知缺口', ''] + ['- ' + value for value in self.data['limitations']]
        (self.root / 'report.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')

    def run(self, identifier, executable, arguments, execute=False, interrupt=False):
        """启动独立进程组并保存stdout/stderr、退出码和JSONL；Ctrl+C转发至整个子进程组。

        execute仅控制证据判定，运动授权由motion()逐项获取。中断后的记录也必须人工检查。
        """
        directory = self.root / identifier
        directory.mkdir()
        command = ['ros2', 'run', 'aimdk_functions', executable, *arguments]
        if '--help' not in arguments:
            command += ['--log-dir', str(directory)]
        case = dict(id=identifier, command=command, executed=execute, expected_interrupt=interrupt,
                    evidence='未完成', directory=str(directory))
        self.data['cases'].append(case)
        self.save()
        print('\n' + shlex.join(command), flush=True)
        print(f'终端输出：{directory / "console.txt"}；Ctrl+C会转发给本次命令。', flush=True)
        process = None
        try:
            with (directory / 'console.txt').open('w', encoding='utf-8') as output:
                process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                           start_new_session=True)
                try:
                    code = process.wait()
                except KeyboardInterrupt:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        code = process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGTERM)
                        code = process.wait()
                    case['operator_interrupted'] = True
            case['returncode'] = code
            case['metrics'] = summarize(read_events(directory))
            metrics = case['metrics']
            reason = metrics.get('exit', {}).get('reason')
            if '--help' in arguments:
                good = code == 0
            elif executable == 'upper_body_kinematics_check':
                summary = metrics.get('summary', {})
                good = (code == 0 and reason == 'completed' and summary.get('round_trip_count') == 6
                        and summary.get('diagnostic_case_count', 0) > 0
                        and summary.get('incomplete_or_unexpected') == 0)
            elif interrupt:
                good = (reason == 'interrupted' and metrics['published_count'] > 0
                        and 'commands_stopped' in metrics)
            else:
                good = (code == 0 and reason == 'completed' and not metrics['errors']
                        and not metrics['feedback_errors'] and 'plan' in metrics)
                if execute:
                    good = (good and metrics['published_count'] > 0
                            and 'endpoint_confirmed' in metrics and 'execution_result' in metrics
                            and 'commands_stopped' in metrics)
                else:
                    good = good and metrics['published_count'] == 0
            case['evidence'] = '完整' if good else '失败或证据不完整'
            print((directory / 'console.txt').read_text(encoding='utf-8')[-5000:])
        except Exception as error:
            case['evidence'] = '失败或证据不完整'
            case['error'] = f'{type(error).__name__}: {error}'
        finally:
            # 会话异常或再次中断时回收整个命令进程组，防止遗留后台发布进程。
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                case['forced_process_cleanup'] = True
                case['evidence'] = '失败或证据不完整'
            self.save()
        return case

    def input_cases(self):
        """直接调用现有参数解析/校验验证明确拒绝的输入；不初始化ROS或创建发布器。"""
        from .dual_arm_control_core import ControlFailure
        from .upper_body_control import parser as control_parser, validate_options

        cases = {
            'nonfinite': ['joint', '--side', 'left', '--tag', 'elbow', '--delta-deg', 'nan'],
            'over_10_deg': ['joint', '--side', 'left', '--tag', 'elbow', '--delta-deg', '11'],
            'missing_right': ['joint', '--side', 'both', '--tag', 'elbow', '--left-delta-deg', '1'],
            'mixed_tcp': ['tcp', '--side', 'left', '--delta-mm', '1', '--translation-mm', '1', '0', '0'],
            'opening_range': ['gripper', '--side', 'left', '--opening', '1.1'],
            'gripper_compensation': ['gripper', '--side', 'left', '--opening', '0.5', '--compensation'],
        }
        import contextlib
        import io

        for name, arguments in cases.items():
            case = dict(id='input_' + name, command=arguments, executed=False, evidence='失败或证据不完整')
            console = io.StringIO()
            try:
                with contextlib.redirect_stderr(console):
                    options = control_parser().parse_args(arguments)
                    validate_options(options)
            except (ControlFailure, SystemExit) as error:
                case.update(evidence='完整', expected_rejection=str(error), console=console.getvalue())
            self.data['cases'].append(case)
            self.save()

    def observe(self, case):
        """记录现场判断、实物测量及退出表现；人工结论与自动证据分别保存。"""
        verdict = ask('现场判断 pass/fail/pending（方向、幅度、另一侧保持、夹爪、连续性及停发后表现）', 'pending')
        if verdict not in ('pass', 'fail', 'pending'):
            verdict = 'pending'
        case['observation'] = dict(verdict=verdict, notes=ask('观察及独立测量结果，写明单位；未测填未测', '未测'))
        self.save()

    def motion(self, identifier, arguments, interrupt=False):
        """先只读预览，成功后要求输入EXECUTE才运行一次动作；失败后不自动重试或返程。"""
        choice = ask(f'{identifier}：输入p预览，s跳过', 's')
        if choice != 'p':
            self.data['cases'].append(dict(id=identifier, evidence='跳过'))
            self.save()
            return
        common = ['--load-description', self.data['context']['condition']]
        preview = self.run(identifier + '_preview', 'upper_body_control', arguments + common)
        if preview['evidence'] != '完整':
            return
        print('核对预览；操作者预先进入URS/RUNNING，确认动作空间及没有其他上肢发布者。')
        if interrupt:
            print('本项执行后，在轨迹结束前按Ctrl+C；随后记录实体保持及固件超时表现。')
        if ask('输入EXECUTE执行；其他输入仅保留预览', 'skip') != 'EXECUTE':
            return
        case = self.run(identifier + '_execute', 'upper_body_control', arguments + common + ['--execute'],
                        execute=True, interrupt=interrupt)
        self.observe(case)

    def review(self, step, requirement):
        """要求逐步审阅完成条件；只保存操作者判断，不把跳过或命令退出0转换成验收通过。"""
        print('\n步骤', step, '完成条件：', requirement)
        self.data['reviews'][str(step)] = dict(
            verdict=ask('审阅结论 accepted/incomplete/failed', 'incomplete'),
            reason=ask('依据、适用范围、日志或缺失项', '待审阅'))
        cases = self.data['cases']
        required = []
        if step == 1:
            required = ['help_' + action for action in ('joint', 'tcp', 'gripper')]
            required += ['input_' + name for name in ('nonfinite', 'over_10_deg', 'missing_right',
                                                       'mixed_tcp', 'opening_range', 'gripper_compensation')]
        elif step == 2:
            required = ['fk_ik']
        elif step == 3:
            replay_cases = [case for case in cases if case['id'].startswith('replay_')]
            required = [case['id'] for case in replay_cases] or ['replay_missing']
        elif step == 5:
            required = [f'comp_{side}_{repeat}_{state}_execute' for side in ('left', 'right')
                        for repeat in range(self.options.repeats) for state in ('off', 'on')]
        missing = [identifier for identifier in required
                   if not any(case['id'] == identifier and case.get('evidence') == '完整'
                              for case in cases)]
        if missing and self.data['reviews'][str(step)]['verdict'] == 'accepted':
            self.data['reviews'][str(step)]['verdict'] = 'incomplete'
            self.data['reviews'][str(step)]['reason'] += '；缺少完整证据：' + ', '.join(missing)
        if step == 4 and self.data['reviews']['4']['verdict'] == 'accepted':
            self.data['reviews']['4']['verdict'] = 'incomplete'
            self.data['reviews']['4']['reason'] += '；统一入口尚无纯旋转和完整位姿实机验证。'
        self.save()

    def readonly(self):
        """运行现有只读FK/IK和边界诊断，保留两侧三快照供后续离线回放。"""
        if ask('步骤2：运行两侧三快照与数学边界/拒绝用例？y/s', 's') != 'y':
            return
        context = self.data['context']
        case = self.run('fk_ik', 'upper_body_kinematics_check', [
            '--side', 'both', '--samples', '3', '--check-cases', '--label', context['posture'],
            '--sdk-version', context['sdk'], '--firmware-version', context['firmware'],
            '--load-description', context['condition']])
        self.observe(case)

    def offline(self):
        """从本会话或显式指定的实际快照日志回放两侧三类轨迹，不连接机器人或发布命令。"""
        from .upper_body_acceptance_replay import replay

        source = self.options.snapshot_log
        if not source:
            paths = sorted((self.root / 'fk_ik').glob('upper_body_*.jsonl'))
            source = str(paths[0]) if paths else ask('步骤3：已有实际快照JSONL路径；空行跳过')
        if not source:
            return
        snapshots = [row['fields'] for row in (json.loads(line) for line in Path(source).read_text().splitlines())
                     if row['event'] == 'snapshot']
        if not snapshots:
            raise ValueError('日志中没有snapshot事件；请使用upper_body_kinematics_check日志')
        for index, snapshot in enumerate(snapshots):
            for side in ('left', 'right'):
                for kind in ('translation', 'rotation', 'combined'):
                    identifier = f'replay_{index}_{side}_{kind}'
                    case = dict(id=identifier, evidence='未完成', snapshot_source=str(source))
                    self.data['cases'].append(case)
                    try:
                        case['metrics'] = replay(snapshot, side, kind, self.options.translation_mm,
                                                 self.options.rotation_deg, self.root / (identifier + '.jsonl'))
                        case['evidence'] = '完整' if case['metrics']['complete'] else '失败或证据不完整'
                    except Exception as error:
                        case['evidence'], case['error'] = '失败或证据不完整', str(error)
                    self.save()
                    print(identifier, case['evidence'])

    def movements(self):
        """覆盖左右单关节、正反向轴平移、endpoint、夹爪及人工中断；各动作独立确认。"""
        from .dual_arm_control_core import JOINT_TAGS

        for side in ('left', 'right'):
            for tag in JOINT_TAGS:
                for sign in (1, -1):
                    self.motion(f'joint_{side}_{tag}_{sign:+}', ['joint', '--side', side, '--tag', tag,
                                '--delta-deg', str(sign * self.options.joint_deg), '--no-compensation'])
            for axis in ('x', 'y', 'z'):
                for sign in (1, -1):
                    self.motion(f'tcp_{side}_{axis}_{sign:+}', ['tcp', '--side', side, '--axis', axis,
                                '--delta-mm', str(sign * self.options.translation_mm), '--no-compensation'])
            self.motion(f'endpoint_{side}', ['tcp', '--side', side, '--path', 'endpoint',
                        '--translation-mm', str(self.options.translation_mm), '0', '0', '--no-compensation'])
            opening = ask(f'{side}夹爪测试目标0..1', '0.5')
            self.motion(f'gripper_{side}', ['gripper', '--side', side, '--opening', opening])
            self.motion(f'interrupt_{side}', ['joint', '--side', side, '--tag', 'wrist_roll',
                        '--delta-deg', str(self.options.joint_deg), '--duration', '10', '--no-compensation'],
                        interrupt=True)
        # 双臂用例为功能包补充覆盖，仍逐项预览和授权。
        for identifier, arguments in (
            ('dual_joint_common', ['joint', '--side', 'both', '--tag', 'wrist_yaw',
                                   '--delta-deg', str(self.options.joint_deg), '--no-compensation']),
            ('dual_joint_independent', ['joint', '--side', 'both', '--tag', 'wrist_yaw',
                                        '--left-delta-deg', str(self.options.joint_deg),
                                        '--right-delta-deg', str(-self.options.joint_deg), '--no-compensation']),
            ('dual_tcp_independent', ['tcp', '--side', 'both', '--left-translation-mm',
                                      '0', str(self.options.translation_mm), '0', '--right-translation-mm',
                                      '0', str(-self.options.translation_mm), '0', '--no-compensation']),
            ('dual_gripper_independent', ['gripper', '--side', 'both', '--left-opening', '0.5',
                                          '--right-opening', '0.3'])):
            self.motion(identifier, arguments)

    def compensation(self):
        """按左右、重复次数生成开关补偿对照，记录实际负载及起点差异，不自动返回起点。"""
        for side in ('left', 'right'):
            mass = ask(f'{side}附加载荷kg（包含模型外的夹爪/工具/工件，未知则跳过该侧）', 'unknown')
            if mass == 'unknown':
                continue
            com = shlex.split(ask('负载质心在该侧腕坐标系XYZ/m', '0 0 0'))
            if len(com) != 3:
                raise ValueError('负载质心需要三个数值')
            for repeat in range(self.options.repeats):
                for enabled in (False, True):
                    print('\n操作者恢复相同起点、姿态、工具和负载；脚本不自动返程。')
                    condition = ask('记录本次配对起点及负载一致性，独立测量方法；s跳过', 's')
                    if condition == 's':
                        continue
                    arguments = ['tcp', '--side', side, '--translation-mm', str(self.options.translation_mm), '0', '0']
                    arguments += (['--compensation', '--payload-mass-kg', mass, '--payload-com-m', *com]
                                  if enabled else ['--no-compensation'])
                    identifier = f'comp_{side}_{repeat}_{"on" if enabled else "off"}'
                    self.data.setdefault('pair_conditions', {})[identifier] = condition
                    self.motion(identifier, arguments)
            pairs = []
            for repeat in range(self.options.repeats):
                cases = {state: next((case for case in self.data['cases']
                                     if case['id'] == f'comp_{side}_{repeat}_{state}_execute'), None)
                         for state in ('off', 'on')}
                if all(cases.values()):
                    plans = [cases[state].get('metrics', {}).get('plan') for state in ('off', 'on')]
                    if all(plans):
                        pairs.append(dict(repeat=repeat, cases=[cases[state]['id'] for state in ('off', 'on')],
                                          start_difference_deg=[math.degrees(b-a) for a, b in zip(
                                              plans[0]['start_arm_rad'], plans[1]['start_arm_rad'])]))
            self.data.setdefault('compensation_pairs', {})[side] = pairs
            self.save()


def parser():
    """定义阶段选择与实验幅度；默认仅预览，所有实机动作仍逐项要求EXECUTE。"""
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--log-dir', help='会话父目录，默认工作区log')
    result.add_argument('--steps', type=int, nargs='+', choices=range(1, 6), default=[1, 2, 3, 4, 5])
    result.add_argument('--snapshot-log', help='步骤3使用已有只读snapshot日志，可单独离线运行')
    result.add_argument('--joint-deg', type=float, default=2.)
    result.add_argument('--translation-mm', type=float, default=5.)
    result.add_argument('--rotation-deg', type=float, default=1., help='仅用于无硬件数学回放')
    result.add_argument('--repeats', type=int, default=3, help='每侧补偿开关配对次数')
    return result


def main(args=None):
    """运行交互验收；返回0表示报告保存，1异常，130会话中断；退出0不表示阶段验收。"""
    options = parser().parse_args(args)
    if (not all(math.isfinite(value) for value in (options.joint_deg, options.translation_mm, options.rotation_deg))
            or not 0 < options.joint_deg <= 10 or options.repeats < 1):
        raise SystemExit('幅度必须有限，关节幅度须为(0,10]度，重复次数须大于0')
    session = None
    try:
        session = AcceptanceSession(options)
        print('会话目录：', session.root)
        print('所有动作均为相对当前采样的单程；正反请求不是自动返程。输入s跳过，Ctrl+D结束会话。')
        session.data['context'] = dict(posture=ask('实际起始姿态与TCP/工具版本', 'unknown'),
                                       condition=ask('现场条件、工具、负载及独立测量方法', 'unknown'),
                                       sdk=ask('实际SDK版本', 'unknown'), firmware=ask('实际固件版本', 'unknown'))
        session.save()
        if 1 in options.steps:
            for action in ('joint', 'tcp', 'gripper'):
                session.run('help_' + action, 'upper_body_control', [action, '--help'])
            session.input_cases()
            session.review(1, '核对report.json中的包路径、代码/模型哈希、参考来源、限位、Humble/Python版本及依赖安装。')
        if 2 in options.steps:
            session.readonly()
            session.review(2, '左右实际快照、FK→IK→FK、分支、SEW、残差和数学边界/不可达用例可复核；多姿态覆盖须声明。')
        if 3 in options.steps:
            session.offline()
            session.review(3, '两侧平移/旋转/组合回放完整，核对失败帧、最后接受状态、周期耗时、最大步长及残差分布。')
        if 4 in options.steps:
            session.movements()
            session.review(4, '核对左右动作与另一侧保持、请求/计划/发送/反馈、实体测量、正常退出和中断；纯旋转/完整位姿尚缺入口。')
        if 5 in options.steps:
            session.compensation()
            session.review(5, '核对同起点和负载的重复配对、偏置与目标、反馈跟踪/FK及实体误差，明确限定工况下启用或暂不启用结论。')
        return 0
    except (KeyboardInterrupt, EOFError):
        print('\n会话结束，保留已完成证据。')
        return 130
    except Exception as error:
        if session is not None:
            session.data['error'] = f'{type(error).__name__}: {error}'
        print(f'验收会话失败：{error}', file=sys.stderr)
        return 1
    finally:
        if session is not None:
            session.save()
            print('报告：', session.root / 'report.md')


if __name__ == '__main__':
    sys.exit(main())
