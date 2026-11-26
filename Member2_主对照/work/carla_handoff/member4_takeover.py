"""Usage: python member4_takeover.py [--cases P1-C P2-C P3-C P4-C T3-C]

Replays the original C policy on recorded CARLA states. Prints the request,
readiness, stable transfer, deadline updates and fallback events. Time is in s.
"""

import argparse
import json

from replay import load_case, reproduce

TIMELINE = {'deadline_updated', 'takeover_requested', 'driver_ready', 'scripted_transfer_started',
            'scripted_handover', 'transfer_deadline_elapsed', 'fallback_started', 'no_verified_fallback',
            'shoulder_second_segment', 'observed_stop'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', nargs='+', default=['P1-C', 'P2-C', 'P3-C', 'P4-C', 'T3-C'])
    args = parser.parse_args()
    for run_id in args.cases:
        original = load_case(run_id)
        if original['case']['strategy'] != 'C':
            parser.error(f'{run_id} does not use policy C')
        # This calls the same Policy.decide and transfer confirmation loop used by CARLA.
        replayed, metrics = reproduce(original)
        print(json.dumps({'run_id': run_id, 'outcome': replayed['outcome'],
                          'terminal_time_s': metrics['terminal_time_s'],
                          'timeline': [e for e in replayed['events'] if e['event'] in TIMELINE]},
                         ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
