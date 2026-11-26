from dataclasses import replace

from member1_n1_n2 import LEFT_CHANGE, assess_candidate, first_contacts, normal_observation, rollout
from member1_scenario import (
    BRAKE_ACCELERATION, DT, LANE_CHANGE_DURATION, LEFT_LANE_Y, RIGHT_LANE_Y,
    SHOULDER_Y, VEHICLE_LENGTH, LaneChange, initial_states, step_reference_scene,
)


start = initial_states()
masked = {key: value for key, value in normal_observation(start).items() if key != 'L'}
left_maneuver = LaneChange(0.0, 0.0, LEFT_LANE_Y)
print('Masked A candidate:', assess_candidate(start['E'], masked, LEFT_CHANGE))
print('Same candidate, true scoring:', first_contacts(rollout(start, LANE_CHANGE_DURATION, left_maneuver)))

for rear_x in (-60.0, -20.0):
    for acceleration in (0.0, BRAKE_ACCELERATION):
        states = {**start, 'Bk': replace(start['Bk'], x=rear_x)}
        rear_only = []
        for tick in range(95):
            time = round(tick * DT, 10)
            rear_only.append((time, {key: states[key] for key in ('E', 'Bk')}))
            states = step_reference_scene(states, time, ego_acceleration=acceleration)
        time, at_two = rear_only[20]
        rear_gap = at_two['E'].x - at_two['Bk'].x - VEHICLE_LENGTH
        print(f'{rear_x=}, {acceleration=}, gap at {time} s={rear_gap:.3f}, '
              f'contacts through {rear_only[-1][0]} s={first_contacts(rear_only)}')

for rear_x in (-50.0, -10.0):
    result = []
    for start_tick in range(48):
        start_time = start_tick * DT
        segments = (LaneChange(start_time, 0.0, RIGHT_LANE_Y),
                    LaneChange(start_time + LANE_CHANGE_DURATION, RIGHT_LANE_Y, SHOULDER_Y))
        frames = []
        for tick in range(start_tick, start_tick + 71):
            time = tick * DT
            segment = segments[time >= segments[1].start_time]
            ego = replace(start['E'], x=start['E'].speed * time, y=segment.y_at(time))
            right_rear = replace(start['R'], x=rear_x + start['R'].speed * time)
            frames.append((round(time, 10), {'E': ego, 'R': right_rear}))
        result.append(first_contacts(frames)['margin_conflict'] is not None)
    print(f'Right rear {rear_x=} m, constant-25 m/s right shifts with clearance violation: '
          f'{sum(result)}/{len(result)} launch times, only R checked')
