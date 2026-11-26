"""Usage: python member3_action_filter.py [--cases N1-A P2-A P2-C T2-C T3-C] [--at 0]

Checks action filtering and stop candidates at decision ticks from CARLA logs.
Examples: --cases T3-C --at 2.6 evaluates both stop candidates at launch time.
Without --at, the default snapshots are t=0 and the stop launch frame if present.
"""

import argparse
import json

from replay import frame_input, load_case, reproduce
from carla_pilot import Plan, Policy, SEGMENT, check_plan


def explain_frame(record, time):
    """Use the original stop and clearance functions with the exact decision inputs."""
    case = record['case']
    ego, observed, rear_acceleration = frame_input(record, time)
    policy = Policy(case)
    output = {'time_s': time, 'observed': list(observed)}
    if case['intent'] == 'left' and time == 0:
        # The original strategy A applies this geometric check to its left candidate.
        if case['strategy'] == 'A':
            score, rejection = check_plan(Plan('left_change', time, ego), time, SEGMENT, observed)
            output['left_candidate'] = {'score_m': round(score, 6), 'rejection': rejection}
        if case['strategy'] == 'C':
            original = next(e for e in record['events'] if e['event'] == 'left_rejected')
            output['left_candidate'] = {'rejection': original['trigger']}
    if case['strategy'] == 'C':
        # Scores and blockers are recomputed with C's original blind-area prior.
        options, blocked = policy.stop_options(ego, time, time, observed, rear_acceleration)
        output['available_stop_candidates'] = {plan.name: round(score, 6) for score, plan in options}
        output['blocked_stop_candidates'] = blocked
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', nargs='+', default=['N1-A', 'P2-A', 'P2-C', 'T2-C', 'T3-C', 'X1-C'])
    parser.add_argument('--at', type=float, help='one recorded decision tick, in seconds')
    args = parser.parse_args()
    for run_id in args.cases:
        record = load_case(run_id)
        reproduced, _ = reproduce(record)
        events = reproduced['events']
        chosen = [e for e in events if e['event'] in ('left_accepted', 'left_rejected', 'fallback_started',
                  'shoulder_second_segment', 'shoulder_replan', 'no_verified_fallback')]
        times = {0.0} | {e['time_s'] for e in events if e['event'] == 'fallback_started'}
        if args.at is not None:
            times = {args.at}
        print(json.dumps({'run_id': run_id, 'decision_snapshots': [explain_frame(record, t) for t in sorted(times)],
                          'verified_events': chosen}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
