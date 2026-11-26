"""Usage: python member5_baselines.py [--mode replay|model|carla] [--cases N1-A N2-A P2-A P4-B]

Default cases are the two normal references, 12 core comparisons, and T3-B.
Replay audits the original logs. Model and CARLA invoke the original simulator
entry point and require --output with a new result folder. CARLA also needs a
dedicated running server and the matching CARLA Python client.
"""

import argparse
from pathlib import Path
import subprocess
import sys

from replay import ROOT, load_case, reproduce
from carla_preflight import build_manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('replay', 'model', 'carla'), default='replay')
    parser.add_argument('--cases', nargs='+', help='run IDs from the formal matrix')
    parser.add_argument('--all', action='store_true', help='select all 23 formal runs')
    parser.add_argument('--output', type=Path, help='new folder for freshly simulated traces')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2200)
    args = parser.parse_args()
    if args.cases and args.all:
        parser.error('Select either --cases or --all.')
    matrix = [c['run_id'] for c in build_manifest()['runs'] if c['role'] != 'boundary']
    chosen = args.cases or (matrix if args.all else [key for key in matrix if key[0] in 'NP'] + ['T3-B'])
    if len(chosen) != len(set(chosen)) or not set(chosen) <= {*matrix, 'X1-C'}:
        parser.error('Provide unique run IDs from the matrix, or X1-C for the boundary example.')
    if args.mode == 'replay':
        for run_id in chosen:
            original = load_case(run_id)
            result, score = reproduce(original)
            target = score['first_collision']
            contact = f" | collision={target['other']}@{target['time_s']:.2f}s" if target else ''
            print(f"{run_id} | {result['outcome']} @ {score['terminal_time_s']:.2f}s{contact} | exact replay")
        return
    if args.output is None:
        parser.error('--output is required when running a simulator.')
    # Delegate to the unchanged experiment CLI so both backends use the original matrix and scorer.
    command = [sys.executable, '-B', str(ROOT / 'core' / 'carla_pilot.py'), '--mode', args.mode,
               '--host', args.host, '--port', str(args.port), '--output', str(args.output.resolve()),
               '--cache', str((args.output.resolve() / 'carla_cache')), '--cases', *chosen]
    if args.all:
        command[command.index('--cases'):] = ['--suite', 'formal']
    raise SystemExit(subprocess.run(command, check=False).returncode)


if __name__ == '__main__':
    main()
