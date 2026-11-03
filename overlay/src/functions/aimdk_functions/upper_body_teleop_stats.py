"""读回只读遥操日志，统计映射与双侧IK耗时、残差及输入关联。"""

import argparse
import json
from pathlib import Path
import sys

from remote_control.recording import distribution


def summarize(path):
    """返回只读日志的结果与分布；损坏记录报错，20ms只作为观测分界。"""
    inputs, accepted, rejected, errors, exits = [], [], [], [], []
    metrics = {'mapping_and_ik_seconds': [], 'dual_ik_seconds': []}
    for side in ('left', 'right'):
        for key in ('solve_seconds', 'position_residual_m', 'rotation_residual_rad',
                    'step_rad', 'excursion_rad'):
            metrics[f'{side}.{key}'] = []
    with Path(path).expanduser().open(encoding='utf-8') as stream:
        for number, line in enumerate(stream, 1):
            try:
                row = json.loads(line)
                event, fields = row['event'], row['fields']
                if event == 'xr_input':
                    inputs.append(fields['frame']['seq'])
                elif event == 'ik_accepted':
                    accepted.append(fields['input_seq'])
                    metrics['mapping_and_ik_seconds'].append(fields['calculation_seconds'])
                    report = fields['result']
                    metrics['dual_ik_seconds'].append(report['calculation_seconds'])
                    for side, arm in report['arms'].items():
                        for key in ('solve_seconds', 'position_residual_m', 'rotation_residual_rad',
                                    'step_rad', 'excursion_rad'):
                            metrics[f'{side}.{key}'].append(arm[key])
                elif event == 'ik_rejected':
                    rejected.append(fields)
                elif event == 'error':
                    errors.append(fields)
                elif event == 'exit':
                    exits.append(fields)
            except (ValueError, TypeError, KeyError) as error:
                raise ValueError(f'{path}:{number}: {error}') from error
    resolved = accepted + [value['input_seq'] for value in rejected]
    missing = sorted(set(inputs) - set(resolved))
    extra = sorted(set(resolved) - set(inputs))
    duplicates = len(resolved) - len(set(resolved))
    return dict(scope='read_only_mapping_and_ik', input_frames=len(inputs),
                accepted_frames=len(accepted), rejected_frames=rejected, errors=errors,
                exit_reasons=exits, unresolved_inputs=missing, unknown_results=extra,
                repeated_results=duplicates,
                replay_complete=bool(inputs and exits and exits[-1].get('reason') == 'completed'
                                     and not rejected and not errors and not missing
                                     and not extra and duplicates == 0),
                distributions={key: distribution(values) for key, values in metrics.items()},
                mapping_and_ik_over_20ms=sum(v > .02 for v in metrics['mapping_and_ik_seconds']))


def main(args=None):
    """输出日志统计JSON；返回0表示读回成功、1表示日志格式或读取失败。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', help='upper_body_teleop_replay或observe的JSONL日志')
    options = parser.parse_args(args)
    try:
        print(json.dumps(summarize(options.input), ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    except Exception as error:
        print(f'遥操统计失败：{error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
