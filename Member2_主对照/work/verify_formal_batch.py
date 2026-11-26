"""Verify completeness and matching experimental inputs in the formal CARLA logs."""

from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import sys


workspace = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(workspace / 'outputs'))
from carla_preflight import LANES, REFERENCE, build_manifest
from carla_metrics import score_run


def load(path):
    return json.loads(path.read_text(encoding='utf-8'))


folder = Path(sys.argv[1])
summary = load(folder / 'summary.json')
manifest = build_manifest()
expected = {case['run_id']: case for case in manifest['runs'] if case['role'] != 'boundary'}
assert len(expected) == 23
assert summary['suite'] == 'formal' and summary['backend'] == 'carla' and summary['carla_version'] == '1.0'
assert len(summary['runs']) == len(expected) and {row['run_id'] for row in summary['runs']} == set(expected)
assert {item.stem for item in folder.glob('*.json')} == {*expected, 'summary'}
for key, value in manifest['parameters'].items():
    assert summary['parameters'][key] == value, key
assert summary['parameters']['brake_pedal_scale_mps2'] == 15
corridor = (min(y - width / 2 for _, y, width, _ in LANES.values()),
            max(y + width / 2 for _, y, width, _ in LANES.values()))
records = {}
outcomes = defaultdict(Counter)
max_errors = defaultdict(float)
for summary_row in summary['runs']:
    case_id = summary_row['run_id']
    record = load(folder / (case_id + '.json'))
    case = expected[case_id]
    frames = record['frames']
    assert record['case'] == case
    assert record['backend'] == 'carla' and record['carla_version'] == '1.0'
    assert record['observation_limit_s'] == 25 and record['brake_pedal_scale_mps2'] == 15
    assert frames[0]['time_s'] == 0 and frames[-1]['time_s'] <= 25
    assert all(math.isclose(frame['time_s'], index * .05, abs_tol=1e-8)
               for index, frame in enumerate(frames))
    assert all(frame['world_frame'] == frames[0]['world_frame'] + index
               for index, frame in enumerate(frames))
    assert all(frame['controls_for_this_frame'] for frame in frames[1:])
    actors = frames[0]['states']
    present = {key for key in REFERENCE if case['actor_overrides'].get(key, {}) is not None}
    assert set(actors) == present
    assert all(set(frame['states']) == present for frame in frames)
    observed = present - {'E'}
    if case['left_observation_lost']:
        observed.discard('L')
    assert set(frames[0]['observed']) == observed
    for key, actor in actors.items():
        target = {**REFERENCE[key], **case['actor_overrides'].get(key, {})}
        for field, desired, tolerance in (('x', target['x_m'], .25), ('y', LANES[target['lane']][1], .25),
                                           ('v', target['speed_mps'], .30), ('yaw', 0, math.radians(1))):
            error = abs(actor[field] - desired)
            max_errors[field] = max(max_errors[field], error)
            assert error <= tolerance, (case_id, key, field, error)
    recalculated = score_run(record, 2.0, .2, corridor)
    assert recalculated == record['score']
    assert {key: summary_row[key] for key in recalculated} == recalculated
    assert summary_row['events'] == record['events']
    outcome = recalculated['outcome']
    assert bool(record['collisions']) == (outcome == 'collision'), case_id
    if outcome == 'observation_limit':
        assert frames[-1]['time_s'] == 25
    if outcome.startswith('stopped_'):
        event = next(event for event in record['events'] if event['event'] == 'observed_stop')
        assert math.isclose(event['time_s'] - event['first_below_threshold_s'], 3.5, abs_tol=.051)
    records[case_id] = record
    if case['role'] == 'fault':
        outcomes[case['strategy']][outcome] += 1

paired_differences = {}
for first, second in (('P1-C', 'P2-C'), ('P3-C', 'P4-C')):
    maxima = defaultdict(float)
    for left, right in zip(records[first]['frames'], records[second]['frames']):
        if left['time_s'] >= 2:
            break
        assert left['time_s'] == right['time_s'] and left['action'] == right['action']
        assert left['observed'] == right['observed'] and left['deadline_s'] == right['deadline_s']
        for key in left['states']:
            for field in ('x', 'y', 'v'):
                difference = abs(left['states'][key][field] - right['states'][key][field])
                maxima[field] = max(maxima[field], difference)
    paired_differences[f'{first}/{second}'] = dict(maxima)

print('Verified formal outputs:', len(records), '| maximum initial errors:', dict(max_errors))
print('Pre-event paired deviations:', paired_differences)
for strategy, counts in sorted(outcomes.items()):
    print('Fault cases', strategy, dict(counts))
for case_id, record in records.items():
    relevant = [{'event': event['event'], **{k: event[k] for k in ('time_s', 'deadline_s', 'action', 'trigger') if k in event}}
                for event in record['events'] if event['event'] in ('fallback_started', 'scripted_handover')]
    print(case_id, record['outcome'], record['score']['terminal_time_s'], record['collisions'], relevant)
