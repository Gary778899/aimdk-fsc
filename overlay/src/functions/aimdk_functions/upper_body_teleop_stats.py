"""读回遥操观测及执行日志，统计映射、双侧IK、发布耗时和输入关联。"""

import argparse
import json
from pathlib import Path
import sys

from remote_control.recording import distribution
from .upper_body_teleop_tools import rotation_vector_deg


def summarize(path):
    """返回遥操日志的结果与分布；损坏记录报错，20ms只作为观测分界。"""
    inputs, accepted, rejected, errors, exits = [], [], [], [], []
    superseded, selections = [], []
    recoveries = []
    input_errors, mapping_events, timings, backlog = [], [], {}, {}
    event_counts, mapping_ranges = {}, {}
    commands, ten_degree_observations = [], []
    execute = False
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
                if event == 'metadata':
                    execute = fields.get('execute', False)
                elif event == 'command_published':
                    commands.append(fields)
                    for key in ('publish_gap_s', 'input_age_s', 'latest_receive_age_s', 'calculation_seconds'):
                        if fields.get(key) is not None:
                            metrics.setdefault(f'command.{key}', []).append(fields[key])
                    if fields.get('tracking_error_rad'):
                        metrics.setdefault('command.max_tracking_error_rad', []).append(
                            max(abs(value) for value in fields['tracking_error_rad']))
                    if fields.get('previous_target_tracking_error_rad') is not None:
                        metrics.setdefault('command.max_previous_target_tracking_error_rad', []).append(
                            max(abs(value) for value in fields['previous_target_tracking_error_rad']))
                elif event == 'xr_input':
                    inputs.append(fields['frame']['seq'])
                elif event == 'input_selected':
                    selections.append(fields)
                    if fields.get('selection_age_s') is not None:
                        metrics.setdefault('selection.age_s', []).append(fields['selection_age_s'])
                elif event == 'ik_superseded':
                    superseded.append(fields['input_seq'])
                elif event == 'mapping_candidate':
                    for side, arm in fields['arms'].items():
                        label = f'{side}.{arm["event"]}'
                        event_counts[label] = event_counts.get(label, 0) + 1
                        if arm['event'] in ('reference_established', 'released'):
                            mapping_events.append(dict(input_seq=fields['input_seq'], side=side,
                                                       event=arm['event']))
                        if arm['enabled']:
                            values = dict(input_translation_m=arm['input_translation_m'],
                                          tcp_translation_m=arm['tcp_translation_m'],
                                          input_rotation_deg=rotation_vector_deg(arm['input_rotation_matrix']),
                                          tcp_rotation_deg=rotation_vector_deg(arm['tcp_rotation_matrix']))
                            for key, vector in values.items():
                                for axis, value in zip('xyz', vector):
                                    mapping_ranges.setdefault(f'{side}.{key}.{axis}', []).append(value)
                elif event in ('input_timing', 'frame_result_timing'):
                    context = fields.get('timing_context', 'legacy')
                    values = timings.setdefault(context, {})
                    for key in ('receive_gap_s', 'processing_age_s', 'input_to_result_s',
                                'service_seconds', 'backlog_frames', 'backlog_events'):
                        if fields.get(key) is not None:
                            values.setdefault(key, []).append(fields[key])
                    queue_key = 'backlog_events' if context == 'live' else 'backlog_frames'
                    if event == 'input_timing' and fields.get(queue_key) is not None:
                        backlog.setdefault(context, []).append((fields['input_seq'], fields[queue_key]))
                    if event == 'frame_result_timing':
                        values.setdefault('completed_monotonic_s', []).append(fields['completed_monotonic_s'])
                elif event == 'replay_input_error':
                    input_errors.append(fields)
                elif event == 'ik_accepted':
                    accepted.append(fields['input_seq'])
                    metrics['mapping_and_ik_seconds'].append(fields['calculation_seconds'])
                    report = fields['result']
                    metrics['dual_ik_seconds'].append(report['calculation_seconds'])
                    for side, arm in report['arms'].items():
                        if arm.get('exceeds_10deg_observation'):
                            ten_degree_observations.append(dict(input_seq=fields['input_seq'], side=side,
                                                                step_rad=arm['step_rad'],
                                                                excursion_rad=arm['excursion_rad']))
                        for key in ('solve_seconds', 'position_residual_m', 'rotation_residual_rad',
                                    'step_rad', 'excursion_rad'):
                            metrics[f'{side}.{key}'].append(arm[key])
                elif event == 'ik_rejected':
                    rejected.append(fields)
                elif event == 'ik_recovery':
                    recoveries.append(fields)
                elif event == 'error':
                    errors.append(fields)
                elif event == 'exit':
                    exits.append(fields)
            except (ValueError, TypeError, KeyError) as error:
                raise ValueError(f'{path}:{number}: {error}') from error
    resolved = accepted + [value['input_seq'] for value in rejected] + superseded
    missing = sorted(set(inputs) - set(resolved))
    extra = sorted(set(resolved) - set(inputs))
    duplicates = len(resolved) - len(set(resolved))
    timing_summary = {}
    for context, values in timings.items():
        completed = values.pop('completed_monotonic_s', [])
        values['result_interval_s'] = [b-a for a, b in zip(completed, completed[1:])]
        queued = backlog.get(context, [])
        timing_summary[context] = dict(
            distributions={key: distribution(items) for key, items in values.items()},
            samples={key: len(items) for key, items in values.items()},
            backlog=dict(unit='events' if context == 'live' else 'frames',
                         first=dict(zip(('input_seq', 'count'), queued[0])) if queued else None,
                         last=dict(zip(('input_seq', 'count'), queued[-1])) if queued else None,
                         nonzero_samples=sum(value > 0 for _, value in queued),
                         increases=sum(b[1] > a[1] for a, b in zip(queued, queued[1:]))))
    first_failure = None
    if rejected:
        first = rejected[0]
        first_failure = dict(input_seq=first['input_seq'], reason=first['reason'],
                             recoverable=first.get('recoverable', False),
                             previous_input_seq=first.get('ik_before', {}).get('last_input_seq'),
                             ik_state_preserved=(first['ik_before'] == first['ik_after']
                                                 if 'ik_before' in first and 'ik_after' in first else None),
                             mapping_reference_preserved=(
                                 first['mapping_before']['sides'] == first['mapping_after']['sides']
                                 if 'mapping_before' in first and 'mapping_after' in first else None))
    published_inputs = {row['input_seq'] for row in commands if row.get('input_seq') is not None}
    return dict(scope='mapping_ik_and_execution' if execute else 'read_only_mapping_and_ik', input_frames=len(inputs),
                accepted_frames=len(accepted), rejected_frames=rejected, errors=errors,
                superseded_inputs=superseded,
                input_selection=dict(selected_frames=len(selections),
                                     grip_edge_frames=sum(value['grip_edge'] for value in selections),
                                     coalesced_pose_frames=(selections[-1]['coalesced_pose_frames']
                                                            if selections else 0)),
                execution=dict(published_frames=len(commands),
                               tracking_observation_frames=sum(
                                   'tracking_exceeds_5deg_observation' in row for row in commands),
                               previous_target_tracking_observation_frames=sum(
                                   row.get('previous_target_tracking_error_rad') is not None
                                   for row in commands),
                               tracking_over_5deg_observation_frames=sum(
                                   row.get('tracking_exceeds_5deg_observation', False) for row in commands),
                               previous_target_tracking_over_5deg_observation_frames=sum(
                                   row.get('previous_target_tracking_exceeds_5deg_observation') is True
                                   for row in commands),
                               first_command=commands[0] if commands else None,
                               last_command=commands[-1] if commands else None,
                               accepted_inputs_not_published=sorted(set(accepted)-published_inputs),
                               unknown_published_inputs=sorted(published_inputs-set(accepted))),
                ten_degree_observations=ten_degree_observations,
                ik_recovery=dict(recoverable_rejections=sum(
                    row.get('recoverable', False) for row in rejected),
                    transition_frames=recoveries,
                    completed_transitions=sum(row['phase'] == 'tracking' for row in recoveries)),
                replay_input_errors=input_errors, first_failure=first_failure,
                mapping_candidate_events=mapping_events, mapping_candidate_event_counts=event_counts,
                mapping_candidate_ranges={key: dict(min=min(values), max=max(values))
                                          for key, values in mapping_ranges.items()},
                timing=timing_summary,
                exit_reasons=exits, unresolved_inputs=missing, unknown_results=extra,
                repeated_results=duplicates,
                replay_complete=bool(inputs and exits and exits[-1].get('reason') == 'completed'
                                     and all(row.get('recoverable', False) for row in rejected)
                                     and not errors and not input_errors and not missing
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
