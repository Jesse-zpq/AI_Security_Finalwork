"""Audit original P4/T2 CARLA logs and fresh model runs without changing policies."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
HANDOFF = ROOT / 'CARLA_Member3-4-5_Handoff' / 'CARLA_repro_handoff'
sys.path.insert(0, str(HANDOFF))
from replay import load_case, reproduce, frame_input
from carla_pilot import PARAMS, DECISION_DT, CORRIDOR, Plan, Policy, check_plan, planned_state, follow_acceleration
from carla_preflight import make_runs
from carla_metrics import score_run


def check_pair(reference, stress):
    ignored = {'run_id', 'pair_with', 'actor_overrides'}
    assert {k: v for k, v in reference.items() if k not in ignored} == {k: v for k, v in stress.items() if k not in ignored}
    assert stress['pair_with'] == reference['run_id']
    assert reference['actor_overrides'] == {}
    assert stress['actor_overrides'] == {'R': {'x_m': -10.0}}
    assert reference['intent'] == 'follow' and reference['left_observation_lost']
    assert reference['readiness_event_s'] is None and not reference['expand_left_prior_to_middle']


def validate_window(rows, deadline):
    selected = [r for r in rows if r['start_s'] <= deadline + 1e-7]
    assert len(selected) == round(deadline / DECISION_DT) + 1
    assert all(r['shoulder_blocker'] == 'R' and r['in_lane_available'] for r in selected)


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def write_csv(path, rows):
    with path.open('w', encoding='utf-8-sig', newline='') as out:
        writer = csv.DictWriter(out, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def summary(record):
    events = {e['event']: e for e in record['events']}
    fallback = events.get('fallback_started', {})
    stop = events.get('observed_stop', {})
    row = dict(run_id=record['run_id'], backend=record['backend'],
               matched_run=('T2' if record['run_id'].startswith('P4') else 'P4') + '-' + record['case']['strategy'],
               right_initial_x_m=record['frames'][0]['states']['R']['x'],
               request_s=events.get('takeover_requested', {}).get('time_s'),
               handover_s=events.get('scripted_handover', {}).get('time_s'),
               initial_deadline_s=next((e['deadline_s'] for e in record['events'] if e['event'] == 'deadline_updated'), None),
               fallback_s=fallback.get('time_s'), fallback=fallback.get('action'),
               launch_deadline_s=fallback.get('deadline_s'),
               stop_threshold_s=stop.get('first_below_threshold_s'),
               end_s=record['score']['terminal_time_s'], outcome=record['score']['outcome'],
               collisions=len(record['collisions']))
    for window, prefix in [('shared_window_s', 'first2s'), ('until_endpoint', 'full')]:
        for metric in ['front_gap_m', 'rear_gap_m', 'left_gap_m', 'right_gap_m', 'longitudinal_ttc_s', 'ego_acceleration_mps2']:
            value = record['score'][window][metric]
            row[prefix + '_' + metric] = value['value'] if value else None
    return row


def audit(record):
    policy = Policy(record['case'])
    launch = next(e for e in record['events'] if e['event'] == 'fallback_started')
    decisions, forecasts = [], []
    previous = math.inf
    for frame in record['frames'][::round(DECISION_DT / record['physics_step_s'])]:
        now = frame['time_s']
        if now > launch['time_s'] + 1e-7:
            break
        ego, observed, rear_a = frame_input(record, now)
        options, blocked = policy.stop_options(ego, now, now, observed, rear_a)
        assert options
        wait = Plan('wait', now, ego, min(0, follow_acceleration(ego, observed)))
        safe_until, still_safe = now, True
        for tick in range(1 + max(0, round((PARAMS['max_wait_s'] - now) / DECISION_DT))):
            future = now + tick * DECISION_DT
            _, conflict = check_plan(wait, now, future - now, observed, rear_a, policy.blind_prior)
            future_options, reasons = policy.stop_options(planned_state(wait, future), future, now, observed, rear_a)
            still_safe = still_safe and not conflict and bool(future_options)
            if still_safe:
                safe_until = future
            if now == 0:
                available = {p.name for _, p in future_options}
                forecasts.append(dict(run_id=record['run_id'], backend=record['backend'], start_s=round(future, 6),
                    in_lane_available='in_lane_stop' in available,
                    shoulder_available='shoulder_stop' in available,
                    shoulder_blocker=reasons.get('shoulder_stop', {}).get('object'),
                    shoulder_conflict_s=reasons.get('shoulder_stop', {}).get('time_s'),
                    in_lane_blocker=reasons.get('in_lane_stop', {}).get('object'),
                    waiting_and_fallback_valid=still_safe))
        deadline = min(previous, max(now, safe_until - PARAMS['mrm_launch_margin_s']))
        assert math.isclose(deadline, frame['deadline_s'], abs_tol=1e-7)
        decisions.append(dict(run_id=record['run_id'], backend=record['backend'], decision_s=now,
            available_now='/'.join(p.name for _, p in options),
            shoulder_blocker=blocked.get('shoulder_stop', {}).get('object'),
            feasible_until_s=round(safe_until, 6), recalculated_deadline_s=round(deadline, 6),
            logged_deadline_s=frame['deadline_s'], action=frame['action']))
        previous = deadline
    if record['run_id'] == 'T2-C':
        validate_window(forecasts, decisions[0]['recalculated_deadline_s'])
        assert launch['time_s'] == 0 and launch['action'] == 'in_lane_stop'
        assert decisions[0]['recalculated_deadline_s'] < 2 + PARAMS['handover_buffer_s']
        assert not any(e['event'] == 'takeover_requested' for e in record['events'])
    return decisions, forecasts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-dir', type=Path, default=HERE / 'results' / 'model_runs')
    parser.add_argument('--output', type=Path, default=HERE / 'results')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = {c['run_id']: c for c in make_runs()}
    decisions, forecasts, sources = [], [], []
    for strategy in 'ABC':
        check_pair(manifest['P4-' + strategy], manifest['T2-' + strategy])
    for backend in ['carla', 'model']:
        rows = []
        for group in ['P4', 'T2']:
            for strategy in 'ABC':
                rid = group + '-' + strategy
                path = (HANDOFF / 'CARLA_formal_results' if backend == 'carla' else args.model_dir) / (rid + '.json')
                record = json.loads(path.read_text(encoding='utf-8'))
                assert record['case'] == manifest[rid] and record['backend'] == backend
                if backend == 'carla':
                    reproduce(record)
                assert score_run(record, PARAMS['longitudinal_clearance_margin_m'], PARAMS['lateral_clearance_margin_m'], CORRIDOR) == record['score']
                assert not any(e['event'] in ('driver_ready', 'scripted_handover', 'scripted_transfer_started') for e in record['events'])
                assert all('L' not in f['observed'] and 'L' in f['states'] for f in record['frames'])
                rows.append(summary(record))
                sources.append(dict(path=str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
                if strategy == 'C':
                    d, f = audit(record)
                    decisions.extend(d)
                    forecasts.extend(f)
        write_csv(args.output / ('formal_pairs.csv' if backend == 'carla' else 'model_pairs.csv'), rows)
    write_csv(args.output / 'deadline_audit.csv', decisions)
    write_csv(args.output / 'initial_wait_forecast.csv', forecasts)
    # The timely branch is supplementary context for Member4's page, not a new stress run.
    timely = load_case('P3-C')
    reproduce(timely)
    for path in sorted((HANDOFF / 'core').glob('*.py')):
        sources.append(dict(path=str(path.relative_to(ROOT)), sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    path = HANDOFF / 'CARLA_formal_results' / 'P3-C.json'
    sources.append(dict(path=str(path.relative_to(ROOT)), sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    write_json(args.output / 'verification.json', dict(formal_pairs_replayed=6, timely_reference_replayed=1,
        fresh_model_runs_checked=6, deadline_decisions_checked=len(decisions),
        forecast_samples=len(forecasts), single_variable_pair_checks=3,
        note='CARLA records are existing evidence; model_runs are new 2D simulations. Forecasts are not executed trajectories.', sources=sources))
    print(f'PASS: 6 formal pairs + P3-C replayed; 6 fresh model records scored; {len(decisions)} deadlines; {len(forecasts)} forecast samples.')


if __name__ == '__main__':
    main()
