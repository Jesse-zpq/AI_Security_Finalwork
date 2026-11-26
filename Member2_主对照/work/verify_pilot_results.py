import json
from pathlib import Path
import sys

folder = Path(sys.argv[1]) if len(sys.argv) > 1 else Path('outputs/CARLA_pilot_results')
records = {path.stem: json.loads(path.read_text(encoding='utf-8')) for path in folder.glob('*.json')}
assert set(records) == {'N1-A', 'P2-A', 'P1-C', 'P2-C'}
assert all(record['backend'] == 'carla' and record['carla_version'] == '1.0' for record in records.values())

normal, fault, take, fallback = (records[key] for key in ('N1-A', 'P2-A', 'P1-C', 'P2-C'))
assert normal['events'][0]['event'] == 'left_rejected' and normal['events'][0]['trigger']['object'] == 'L'
assert fault['events'][0]['event'] == 'left_accepted'
assert set(normal['frames'][0]['observed']) - set(fault['frames'][0]['observed']) == {'L'}
assert fault['collisions'][0]['other'] == 'L' and fault['collisions'][0]['time_s'] == 2.0
assert all(not item['collisions'] for item in (normal, take, fallback))
for result in (take, fallback):
    assert result['events'][0]['trigger'] == 'left_observation_unknown'
    assert result['events'][1]['deadline_s'] == 4.7
assert take['events'][-1]['event'] == 'scripted_handover' and take['events'][-1]['time_s'] == 2.5
selection = next(event for event in fallback['events'] if event['event'] == 'fallback_started')
assert selection['time_s'] == 4.7 and selection['action'] == 'in_lane_stop'
assert selection['candidates_rejected']['shoulder_stop']['object'] == 'R'
assert fallback['events'][-1]['event'] == 'observed_stop'
stop_time = next(frame['time_s'] for frame in fallback['frames'] if frame['states']['E']['v'] < 0.2)
assert fallback['frames'][-1]['time_s'] >= stop_time + 3.5 - 1e-6

paired = zip(take['frames'], fallback['frames'])
max_delta = {'x': 0.0, 'y': 0.0, 'v': 0.0}
for first, second in paired:
    if first['time_s'] >= 2.0:
        break
    assert first['time_s'] == second['time_s'] and first['action'] == second['action'] == 'wait'
    for field in max_delta:
        for key, state in first['states'].items():
            max_delta[field] = max(max_delta[field], abs(state[field] - second['states'][key][field]))
assert max(max_delta.values()) < 0.05
print('CARLA 1.0 verified, all four logs, event order, collision identity, stop hold, matched trajectories.')
print('Maximum paired state deviation before readiness:', max_delta)
print('First below 0.2 m/s:', stop_time, 'confirmed:', fallback['frames'][-1]['time_s'])
print('First collision:', fault['collisions'][0])
print('Rejected shoulder candidate:', selection['candidates_rejected']['shoulder_stop'])
