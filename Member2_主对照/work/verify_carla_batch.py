from pathlib import Path
import json
import sys


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


physical = Path(sys.argv[1])
model = Path(sys.argv[2])
summary = read(physical / 'summary.json')
assert summary['backend'] == 'carla' and summary['carla_version'] == '1.0'
assert summary['parameters']['brake_pedal_scale_mps2'] == 15
results = {row['run_id']: read(physical / (row['run_id'] + '.json')) for row in summary['runs']}
assert len(results) == 10
assert all(result['observation_limit_s'] == 25 for result in results.values())
assert all(result['score']['shared_window_s']['observed_until_s'] <= 2 for result in results.values())
assert all(result['score']['until_endpoint']['first_outside_corridor_s'] is None for result in results.values())


def event(run_id, name):
    return next(item for item in results[run_id]['events'] if item['event'] == name)


assert event('N1-A', 'left_rejected')['trigger']['object'] == 'L'
assert results['P2-A']['collisions'][0]['other'] == 'L'
assert event('P1-C', 'scripted_handover')['time_s'] == 2.5
assert event('P2-C', 'fallback_started')['time_s'] == 4.7
assert event('P4-B', 'fallback_started')['time_s'] == 0.0
calibrated = results['P4-B']['frames']
low_speed = next(frame for frame in calibrated if frame['states']['E']['v'] < 0.2)
acceleration = (low_speed['states']['E']['v'] - calibrated[0]['states']['E']['v']) / low_speed['time_s']
assert abs(acceleration + 4) < 0.15
assert results['T1-A']['outcome'] == 'observation_limit'
assert all('L' not in frame['states'] for frame in results['T1-A']['frames'])
assert event('T2-C', 'fallback_started')['candidates_rejected']['shoulder_stop']['object'] == 'R'
assert abs(results['T2-C']['frames'][0]['states']['R']['x'] + 10) < 0.25
assert abs(results['T3-B']['frames'][0]['states']['Bk']['x'] + 20) < 0.25
assert results['T3-B']['collisions'][0]['other'] == 'Bk'
deadlines = [item['deadline_s'] for item in results['T3-C']['events'] if item['event'] == 'deadline_updated']
assert deadlines[-1] < deadlines[0] and deadlines == sorted(deadlines, reverse=True)
assert event('T3-C', 'fallback_started')['time_s'] <= deadlines[-1] + 0.1
assert event('T3-C', 'shoulder_second_segment')['time_s'] > event('T3-C', 'fallback_started')['time_s']
assert results['T3-C']['outcome'] == 'stopped_shoulder' and not results['T3-C']['collisions']
assert event('X1-C', 'no_verified_fallback')['time_s'] == 0.0
assert 'L' not in results['X1-C']['frames'][0]['observed']
assert any('L' in frame['observed'] for frame in results['X1-C']['frames'])

formal = read(model / 'summary.json')
ids = {row['run_id'] for row in formal['runs']}
assert formal['backend'] == 'model' and len(ids) == 23
assert {'N1-A', 'N2-A'} <= ids and not any(key.startswith('X1-') for key in ids)
assert len([key for key in ids if key.startswith(('P', 'T'))]) == 21
assert all(read(model / (run_id + '.json'))['observation_limit_s'] == 25 for run_id in ids)
print('Validated 10 CARLA qualification runs and all 23 formal entries in the model backend.')
print('P4-B actual mean until low speed:', round(acceleration, 3), 'm/s2; distance:', round(low_speed['states']['E']['x'], 3), 'm')
print('P4-B minimum instantaneous acceleration:', results['P4-B']['score']['until_endpoint']['ego_acceleration_mps2'])
print('T3-C deadlines', deadlines[0], deadlines[-1])
for run_id, result in results.items():
    print(run_id, result['outcome'], result['frames'][-1]['time_s'])
