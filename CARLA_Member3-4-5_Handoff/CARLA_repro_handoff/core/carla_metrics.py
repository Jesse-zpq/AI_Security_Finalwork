"""Read CARLA traces using the same metrics for each experimental strategy."""

import math


def box_projection(ego, target):
    def size(actor):
        cosine, sine = abs(math.cos(actor['yaw'])), abs(math.sin(actor['yaw']))
        return (actor['length'] * cosine + actor['width'] * sine,
                actor['width'] * cosine + actor['length'] * sine)

    ego_x, ego_y = size(ego)
    target_x, target_y = size(target)
    dx, dy = target['x'] - ego['x'], target['y'] - ego['y']
    return dx, dy, abs(dx) - (ego_x + target_x) / 2, abs(dy) - (ego_y + target_y) / 2


def measure(frames, collision_s, longitudinal_margin, lateral_margin, corridor):
    summary = dict.fromkeys(('front_gap_m', 'rear_gap_m', 'left_gap_m', 'right_gap_m',
                             'longitudinal_ttc_s', 'ego_acceleration_mps2'))
    summary['observed_until_s'] = frames[-1]['time_s']
    summary['first_outside_corridor_s'] = None

    def update(key, value, time, target=None):
        if summary[key] is None or value < summary[key]['value']:
            summary[key] = {'value': value, 'time_s': time, 'actor': target}

    previous = None
    for frame in frames:
        time = frame['time_s']
        actors = frame['states']
        ego = actors['E']
        half_width = (ego['width'] * abs(math.cos(ego['yaw'])) + ego['length'] * abs(math.sin(ego['yaw']))) / 2
        if ((ego['y'] - half_width < corridor[0] or ego['y'] + half_width > corridor[1])
                and summary['first_outside_corridor_s'] is None):
            summary['first_outside_corridor_s'] = time
        if previous and (collision_s is None or time < collision_s - 1e-8):
            dt = time - previous['time_s']
            acceleration = (ego['v'] - previous['states']['E']['v']) / dt
            update('ego_acceleration_mps2', acceleration, time)
        for actor, target in actors.items():
            if actor == 'E':
                continue
            dx, dy, gap_x, gap_y = box_projection(ego, target)
            if gap_y <= lateral_margin:
                key = 'front_gap_m' if dx >= 0 else 'rear_gap_m'
                update(key, gap_x, time, actor)
                closing = ego['v'] - target['v'] if dx >= 0 else target['v'] - ego['v']
                if closing > 0.05 and gap_x > 0:
                    update('longitudinal_ttc_s', gap_x / closing, time, actor)
            if gap_x <= longitudinal_margin and abs(dy) >= 0.5:
                key = 'left_gap_m' if dy > 0 else 'right_gap_m'
                update(key, gap_y, time, actor)
        previous = frame
    return summary


def score_run(record, longitudinal_margin, lateral_margin, corridor, shared_window_s=2.0):
    contacts = record['collisions']
    collision = next((hit for hit in contacts if hit.get('time_s') is not None), None)
    collision_s = collision['time_s'] if collision else None
    frames = record['frames']
    shared = [frame for frame in frames if frame['time_s'] <= shared_window_s + 1e-8]
    return {
        'outcome': record['outcome'],
        'collision': bool(contacts),
        'first_collision': contacts[0] if contacts else None,
        'terminal_time_s': frames[-1]['time_s'],
        'shared_window_s': measure(shared, collision_s, longitudinal_margin, lateral_margin, corridor),
        'until_endpoint': measure(frames, collision_s, longitudinal_margin, lateral_margin, corridor),
    }
