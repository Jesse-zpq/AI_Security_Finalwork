"""Run the reference, fault and takeover pilot on a dedicated CARLA server."""

import argparse
import json
import math
import socket
from dataclasses import asdict, dataclass
from pathlib import Path

from carla_preflight import EGO_X_ORIGIN, LANES, REFERENCE, build_manifest, build_road, check_shoulder_support
from carla_metrics import score_run


RUN_IDS = ('N1-A', 'P2-A', 'P1-C', 'P2-C')
QUALIFICATION = ('N1-A', 'P2-A', 'P1-C', 'P2-C', 'P4-B', 'T1-A', 'T2-C', 'T3-B', 'T3-C', 'X1-C')
PARAMS = build_manifest()['parameters']
PHYSICS_DT = PARAMS['physics_step_s']
DECISION_DT = PARAMS['decision_step_s']
SEGMENT = PARAMS['lateral_segment_s']
BRAKE = PARAMS['braking_target_mps2']
# Model 3 pedal scale measured in the straight-road calibration, in m/s² per unit brake.
BRAKE_PEDAL_SCALE = 15.0
FAULT_LEFT_LIMIT = LANES['left'][1] / 2
STOP_ACTIONS = ('in_lane_stop', 'shoulder_stop')
CORRIDOR = (min(y - width / 2 for _, y, width, _ in LANES.values()),
            max(y + width / 2 for _, y, width, _ in LANES.values()))


@dataclass(frozen=True)
class State:
    x: float
    y: float
    v: float
    length: float = 5.0
    width: float = 1.8
    yaw: float = 0.0


@dataclass(frozen=True)
class Plan:
    name: str
    start: float
    ego: State
    acceleration: float = 0.0
    hold_y: float = 0.0


def clamp(value, lower, upper):
    return min(upper, max(lower, value))


def move(x, speed, acceleration, duration):
    moving_time = min(duration, speed / -acceleration) if acceleration < 0 else duration
    return (x + speed * moving_time + 0.5 * acceleration * moving_time**2,
            max(0.0, speed + acceleration * moving_time))


def lateral(start, end, elapsed, duration=SEGMENT):
    u = clamp(elapsed / duration, 0.0, 1.0)
    return start + (end - start) * u**3 * (10 - 15 * u + 6 * u**2)


def lateral_at(plan, time):
    elapsed = max(0.0, time - plan.start)
    if plan.name == 'left_change':
        return lateral(plan.ego.y, LANES['left'][1], elapsed)
    if plan.name == 'shoulder_stop':
        if elapsed <= SEGMENT:
            return lateral(plan.ego.y, LANES['right'][1], elapsed)
        return lateral(LANES['right'][1], LANES['shoulder'][1], elapsed - SEGMENT)
    return plan.hold_y


def planned_state(plan, time):
    elapsed = max(0.0, time - plan.start)
    ego = plan.ego
    coast = min(elapsed, 2 * SEGMENT) if plan.name == 'shoulder_stop' else 0.0
    x, speed = move(ego.x + coast * ego.v, ego.v, plan.acceleration, elapsed - coast)
    y = lateral_at(plan, time)
    lateral_speed = (lateral_at(plan, time + 0.02) - y) / 0.02
    yaw = math.atan2(lateral_speed, max(speed, 0.1))
    return State(x, y, speed, ego.length, ego.width, yaw)


def gaps(ego, other):
    def projected_size(state):
        cosine, sine = abs(math.cos(state.yaw)), abs(math.sin(state.yaw))
        return (state.length * cosine + state.width * sine,
                state.width * cosine + state.length * sine)

    ego_length, ego_width = projected_size(ego)
    other_length, other_width = projected_size(other)
    return (abs(ego.x - other.x) - (ego_length + other_length) / 2,
            abs(ego.y - other.y) - (ego_width + other_width) / 2)


def clearance(ego, target):
    longitudinal, lateral_gap = gaps(ego, target)
    return max(longitudinal - PARAMS['longitudinal_clearance_margin_m'],
               lateral_gap - PARAMS['lateral_clearance_margin_m'])


def observed_targets(states, left_lost):
    return {key: target for key, target in states.items()
            if key != 'E' and (not left_lost or target.y <= FAULT_LEFT_LIMIT)}


def target_prediction(key, target, elapsed, until_ego_brake, rear_acceleration):
    if key != 'Bk':
        return State(target.x + target.v * elapsed, target.y, target.v, target.length, target.width)
    if rear_acceleration < -0.5:
        x, speed = move(target.x, target.v, max(-6.0, rear_acceleration), elapsed)
    else:
        delay = PARAMS['predicted_rear_response_delay_s'][1]
        brake = PARAMS['predicted_rear_braking_magnitude_mps2'][0]
        coast = min(elapsed, until_ego_brake + delay)
        x, speed = move(target.x + target.v * coast, target.v, -brake, elapsed - coast)
    return State(x, target.y, speed, target.length, target.width)


def check_plan(plan, now, duration, targets, rear_acceleration=0.0, blind_prior=None, current_ego=None):
    score = math.inf
    until_brake = math.inf
    start = max(now, plan.start)
    nominal = planned_state(plan, start)
    dx = current_ego.x - nominal.x if current_ego else 0.0
    dy = current_ego.y - nominal.y if current_ego else 0.0
    if plan.acceleration < 0.0:
        until_brake = max(0.0, plan.start - now + (2 * SEGMENT if plan.name == 'shoulder_stop' else 0.0))
    steps = math.ceil(duration / DECISION_DT - 1e-8)
    for index in range(steps + 1):
        elapsed = min(index * DECISION_DT, duration)
        time = start + elapsed
        ego = planned_state(plan, time)
        if current_ego:
            ego = State(ego.x + dx, ego.y + dy, ego.v, ego.length, ego.width, ego.yaw)
        half_width = (ego.length * abs(math.sin(ego.yaw)) + ego.width * abs(math.cos(ego.yaw))) / 2
        if ego.y - half_width < CORRIDOR[0] or ego.y + half_width > CORRIDOR[1]:
            return -1.0, {'object': 'road_edge', 'time_s': time}
        for key, target in targets.items():
            predicted = target_prediction(key, target, time - now, until_brake, rear_acceleration)
            separation = clearance(ego, predicted)
            if separation < 0:
                return separation, {'object': key, 'time_s': time}
            score = min(score, separation)
        if blind_prior:
            min_x, max_x = PARAMS['left_prior_x_m']
            min_v, max_v = PARAMS['left_prior_v_mps']
            hidden = State(clamp(ego.x, min_x + min_v * time, max_x + max_v * time),
                           clamp(ego.y, *blind_prior), 0.0)
            separation = clearance(ego, hidden)
            if separation < 0:
                return separation, {'object': 'left_unknown', 'time_s': time}
            score = min(score, separation)
    return score, None


def follow_acceleration(ego, targets):
    front_gaps = [gaps(ego, state)[0] for state in targets.values()
                  if state.x > ego.x and abs(state.y - ego.y) < 1.8]
    target_gap = PARAMS['nominal_follow_standoff_m'] + PARAMS['nominal_follow_headway_s'] * ego.v
    speed_term = PARAMS['nominal_follow_speed_gain_per_s'] * (25.0 - ego.v)
    gap_term = PARAMS['nominal_follow_gap_gain_per_s2'] * min(0.0, min(front_gaps, default=math.inf) - target_gap)
    return clamp(speed_term + gap_term, -4.0, 2.0)


class Policy:
    def __init__(self, case):
        self.case = case
        self.plan = None
        self.events = []
        self.deadline = None
        self.requested = False
        self.ready = False
        self.transfer_attempted = False
        self.shoulder_phase = 1
        offset = PARAMS['left_prior_lane_center_offset_m']
        self.blind_prior = (0.0 if case['expand_left_prior_to_middle'] else LANES['left'][1] - offset,
                            LANES['left'][1] + offset)

    def log(self, time, event, **details):
        self.events.append({'time_s': round(time, 6), 'event': event, **details})

    def stop_options(self, ego, start, now, targets, rear_acceleration):
        options = []
        reasons = {}
        for action in STOP_ACTIONS:
            plan = Plan(action, start, ego, BRAKE)
            duration = (2 * SEGMENT if action == 'shoulder_stop' else 0.0) + ego.v / -BRAKE
            duration += PARAMS['post_stop_observation_s']
            score, rejection = check_plan(plan, now, duration, targets, rear_acceleration, self.blind_prior)
            if rejection is None:
                options.append((score, plan))
            else:
                reasons[action] = rejection
        return options, reasons

    def monitor_shoulder(self, time, ego, targets, rear_acceleration):
        plan = self.plan
        remaining = max(0.0, plan.start + 2 * SEGMENT + plan.ego.v / -BRAKE
                        + PARAMS['post_stop_observation_s'] - time)
        _, rejection = check_plan(plan, time, remaining, targets, rear_acceleration, self.blind_prior, ego)
        if rejection:
            self.log(time, 'shoulder_replan', trigger=rejection)
            for name in ('middle', 'right', 'shoulder'):
                _, center, width, _ = LANES[name]
                actual_width = ego.width * abs(math.cos(ego.yaw)) + ego.length * abs(math.sin(ego.yaw))
                if abs(ego.y - center) + actual_width / 2 > width / 2:
                    continue
                action = {'middle': 'in_lane_stop', 'right': 'right_lane_stop', 'shoulder': 'hold_shoulder_stop'}[name]
                candidate = Plan(action, time, ego, BRAKE, center)
                _, blocked = check_plan(candidate, time, ego.v / -BRAKE + PARAMS['post_stop_observation_s'],
                                        targets, rear_acceleration, self.blind_prior)
                if not blocked:
                    self.log(time, 'alternate_stop_started', action=candidate.name)
                    return candidate
            self.log(time, 'no_verified_fallback', trigger=rejection)
            return Plan('emergency_stop', time, ego, BRAKE, ego.y)
        if self.shoulder_phase == 1 and time >= plan.start + SEGMENT - 1e-6:
            self.shoulder_phase = 2
            self.log(time, 'shoulder_second_segment')
        return plan

    def decide(self, time, ego, targets, rear_acceleration):
        ready_time = self.case['readiness_event_s']
        if not self.ready and ready_time is not None and time >= ready_time:
            self.ready = True
            self.log(time, 'driver_ready')
        if self.case['strategy'] == 'B':
            if self.plan is None:
                self.log(time, 'fallback_started', action='in_lane_stop', trigger='immediate_braking_baseline')
                self.plan = Plan('in_lane_stop', time, ego, BRAKE)
            return self.plan
        if self.plan is not None and self.plan.name == 'driver_follow':
            if time < self.deadline - 1e-6:
                return Plan('driver_follow', time, ego, follow_acceleration(ego, targets))
            self.log(time, 'transfer_deadline_elapsed')
            self.plan = None
        if self.plan is not None and self.plan.name == 'shoulder_stop':
            self.plan = self.monitor_shoulder(time, ego, targets, rear_acceleration)
            return self.plan
        if self.plan is not None and self.plan.name not in ('left_change', 'left_lane_follow'):
            return self.plan
        if self.case['strategy'] == 'A':
            if self.plan is not None and time >= self.plan.start + SEGMENT - 1e-6:
                self.plan = Plan('left_lane_follow', time, ego, hold_y=LANES['left'][1])
            if self.plan is not None and self.plan.name == 'left_lane_follow':
                return Plan('left_lane_follow', time, ego, follow_acceleration(ego, targets), LANES['left'][1])
            if self.plan is None and self.case['intent'] == 'left' and time == 0:
                candidate = Plan('left_change', time, ego)
                _, rejection = check_plan(candidate, time, SEGMENT, targets)
                self.log(time, 'left_accepted' if rejection is None else 'left_rejected',
                         trigger=rejection, observed=list(targets))
                if rejection is None:
                    self.plan = candidate
            return self.plan or Plan('follow', time, ego, follow_acceleration(ego, targets))

        if self.case['intent'] == 'left' and time == 0:
            self.log(time, 'left_rejected', trigger='left_observation_unknown', observed=list(targets))
        options, rejected = self.stop_options(ego, time, time, targets, rear_acceleration)
        if not options:
            self.log(time, 'no_verified_fallback', candidates=rejected)
            self.plan = Plan('emergency_stop', time, ego, BRAKE)
            return self.plan

        wait = Plan('wait', time, ego, min(0.0, follow_acceleration(ego, targets)))
        cap = PARAMS['max_wait_s']
        feasible_until = time
        limiting_condition = 'wait_cap'
        for tick in range(1, max(0, round((cap - time) / DECISION_DT)) + 1):
            future = time + tick * DECISION_DT
            future_ego = planned_state(wait, future)
            _, conflict = check_plan(wait, time, future - time, targets, rear_acceleration, self.blind_prior)
            future_options, reasons = self.stop_options(future_ego, future, time, targets, rear_acceleration)
            if conflict or not future_options:
                limiting_condition = conflict or reasons
                break
            feasible_until = future
        deadline = max(time, feasible_until - PARAMS['mrm_launch_margin_s'])
        if self.requested:
            deadline = min(self.deadline, deadline)
        if self.deadline is None or abs(deadline - self.deadline) > 1e-6:
            self.log(time, 'deadline_updated', deadline_s=round(deadline, 6), trigger=limiting_condition)
        self.deadline = deadline
        required_window = max(0.0, 2.0 - time) + PARAMS['handover_buffer_s']
        if not self.requested and deadline - time >= required_window:
            self.requested = True
            self.log(time, 'takeover_requested', deadline_s=round(deadline, 6))
        if (self.requested and self.ready and not self.transfer_attempted and time + 0.5 < deadline - 1e-6
                and abs(ego.y) < 0.3 and abs(ego.yaw) < 0.05):
            self.log(time, 'scripted_transfer_started')
            self.transfer_attempted = True
            self.plan = Plan('driver_follow', time, ego, follow_acceleration(ego, targets))
            return self.plan
        if time >= deadline - 1e-6 or not self.requested:
            best_score, selected = max(options, key=lambda item: item[0])
            for score, candidate in options:
                if candidate.name == 'shoulder_stop' and best_score - score <= PARAMS['fallback_score_tie_margin_m']:
                    selected = candidate
            self.log(time, 'fallback_started', action=selected.name, candidates_rejected=rejected,
                     deadline_s=round(deadline, 6))
            self.plan = selected
            return selected
        return wait


def initial_states(case):
    result = {}
    for key, reference in REFERENCE.items():
        override = case['actor_overrides'].get(key, {})
        if override is not None:
            value = {**reference, **override}
            result[key] = State(value['x_m'], LANES[value['lane']][1], value['speed_mps'])
    return result


def npc_reference(key, state, time, case):
    if key == 'Bk':
        coast = min(time, 2.0)
        x, speed = move(state.x + state.v * coast, state.v, -5.0, time - coast)
        return State(x, state.y, speed, state.length, state.width)
    target_y = state.y
    script = case['actor_overrides'].get(key, {})
    cut_in_at = script.get('cut_in_at_s')
    if cut_in_at is not None:
        target_y = lateral(state.y, LANES['middle'][1], time - cut_in_at, script['cut_in_duration_s'])
    return State(state.x + state.v * time, target_y, state.v, state.length, state.width)


class ModelWorld:
    def __init__(self, case):
        self.case = case
        self.start = initial_states(case)
        self.collisions = []
        self.states = self.start
        self.frame = 0
        self.controls = None

    def sample(self):
        return self.states

    def tick(self, plan, next_time):
        self.frame += 1
        self.states = {key: npc_reference(key, state, next_time, self.case) for key, state in self.start.items()}
        self.states['E'] = planned_state(plan, next_time)
        for key, state in self.states.items():
            if key != 'E' and max(gaps(self.states['E'], state)) < 0:
                self.collisions.append({'other': key, 'time_s': round(next_time, 6), 'source': 'model_boxes'})

    def close(self):
        pass


class CarlaWorld:
    def __init__(self, carla, client, world, case):
        self.carla = carla
        self.client = client
        self.world = world
        self.case = case
        self.actors = {}
        self.sensor = None
        self.frame_zero = None
        self.frame = None
        self.controls = None
        self.collisions = []
        self.start = initial_states(case)
        self.road = world.get_map()
        origin = self.road.get_waypoint_xodr(1, -2, EGO_X_ORIGIN).transform
        self.origin = origin.location
        try:
            self.spawn()
        except BaseException:
            self.close()
            raise

    def spawn(self):
        carla = self.carla
        definitions = self.world.get_blueprint_library()
        vehicle = definitions.find('vehicle.tesla.model3')
        for key, state in self.start.items():
            extent_y = self.origin.y - state.y
            location = carla.Location(x=self.origin.x + state.x - state.v * PHYSICS_DT,
                                      y=extent_y, z=self.origin.z + 0.25)
            vehicle.set_attribute('role_name', f'pilot_{key}')
            actor = self.world.spawn_actor(vehicle, carla.Transform(location, carla.Rotation()))
            self.actors[key] = actor
            actor.apply_control(carla.VehicleControl(brake=0.5))
        for _ in range(15):
            self.world.tick()

        def on_collision(event):
            other = next((key for key, actor in self.actors.items() if actor.id == event.other_actor.id),
                         event.other_actor.type_id)
            time = None if self.frame_zero is None else round((event.frame - self.frame_zero) * PHYSICS_DT, 6)
            self.collisions.append({'other': other, 'time_s': time, 'frame': event.frame, 'source': 'carla_sensor'})

        self.sensor = self.world.spawn_actor(definitions.find('sensor.other.collision'),
                                             carla.Transform(), attach_to=self.actors['E'])
        self.sensor.listen(on_collision)
        for actor in self.actors.values():
            actor.apply_control(carla.VehicleControl(throttle=0.3))
        self.world.tick()
        starts = [carla.command.ApplyTargetVelocity(actor.id, carla.Vector3D(x=self.start[key].v))
                  for key, actor in self.actors.items()]
        for response in self.client.apply_batch_sync(starts, False):
            if response.error:
                raise RuntimeError(f'CARLA velocity initialization failed: {response.error}')
        self.world.tick()
        samples = self.sample()
        self.origin.x += samples['E'].x
        samples = self.sample()
        for key, actual in samples.items():
            target = self.start[key]
            if (abs(actual.x - target.x) > 0.25 or abs(actual.y - target.y) > 0.25
                    or abs(actual.v - target.v) > 0.3 or abs(actual.yaw) > math.radians(1)):
                raise RuntimeError(f'Spawn/speed calibration failed for {key}: {actual}')
        self.start = samples
        self.frame_zero = self.world.get_snapshot().frame
        self.frame = self.frame_zero

    def sample(self):
        states = {}
        for key, actor in self.actors.items():
            transform = actor.get_transform()
            box = actor.bounding_box
            center = transform.transform(self.carla.Location(x=box.location.x, y=box.location.y, z=box.location.z))
            velocity = actor.get_velocity()
            states[key] = State(center.x - self.origin.x, self.origin.y - center.y,
                                max(0.0, velocity.x), 2 * box.extent.x, 2 * box.extent.y,
                                -math.radians(transform.rotation.yaw))
        return states

    def tick(self, plan, next_time):
        current = self.sample()
        self.controls = {}
        for key, actor in self.actors.items():
            state = current[key]
            if key == 'E':
                target = planned_state(plan, next_time)
                lookahead = planned_state(plan, next_time + 0.30)
                brake_action = plan.acceleration < 0 and (plan.name != 'shoulder_stop'
                                or next_time >= plan.start + 2 * SEGMENT)
                feedforward = plan.acceleration if brake_action else max(0.0, plan.acceleration)
            else:
                target = npc_reference(key, self.start[key], next_time, self.case)
                lookahead = npc_reference(key, self.start[key], next_time + 0.30, self.case)
                feedforward = -5.0 if key == 'Bk' and next_time >= 2.0 and target.v > 0 else 0.0
            acceleration = feedforward + 1.8 * (target.v - state.v)
            desired_heading = math.atan2(lookahead.y - state.y, max(3.0, 0.32 * state.v))
            steer = clamp(-1.05 * (desired_heading - state.yaw), -0.3, 0.3)
            hold = target.v < 0.1 and state.v < 0.25
            throttle = 0.0 if acceleration < 0 or hold else clamp(0.19 + acceleration / 4.0, 0.0, 1.0)
            brake = 1.0 if hold else clamp(-acceleration / BRAKE_PEDAL_SCALE, 0.0, 1.0)
            actor.apply_control(self.carla.VehicleControl(throttle=throttle, brake=brake, steer=steer, hand_brake=hold))
            self.controls[key] = {'throttle': throttle, 'brake': brake, 'steer': steer, 'hand_brake': hold}
        self.frame = self.world.tick()

    def close(self):
        if self.sensor is not None:
            self.sensor.stop()
            self.sensor.destroy()
        for actor in self.actors.values():
            actor.destroy()
        self.world.tick()


def run_case(case, backend, duration):
    policy = Policy(case)
    frames = []
    interval = round(DECISION_DT / PHYSICS_DT)
    prior_rear_v = backend.sample()['Bk'].v
    stop_started = None
    stable_transfer_since = None
    minimum = {}
    plan = None
    outcome = 'observation_limit'
    for tick in range(round(duration / PHYSICS_DT) + 1):
        time = round(tick * PHYSICS_DT, 8)
        states = backend.sample()
        ego = states['E']
        observations = observed_targets(states, case['left_observation_lost'])
        if tick % interval == 0:
            rear_acceleration = (observations['Bk'].v - prior_rear_v) / DECISION_DT
            prior_rear_v = observations['Bk'].v
            plan = policy.decide(time, ego, observations, rear_acceleration)
        frames.append({'time_s': time, 'world_frame': backend.frame,
                       'states': {key: asdict(state) for key, state in states.items()},
                       'observed': list(observations), 'action': plan.name, 'deadline_s': policy.deadline,
                       'controls_for_this_frame': backend.controls})
        for key, target in states.items():
            if key != 'E':
                gap_x, gap_y = gaps(ego, target)
                score = max(gap_x - PARAMS['longitudinal_clearance_margin_m'],
                            gap_y - PARAMS['lateral_clearance_margin_m'])
                if key not in minimum or score < minimum[key]['score_m']:
                    minimum[key] = {'time_s': time, 'score_m': score, 'gap_x_m': gap_x, 'gap_y_m': gap_y}
        if backend.collisions:
            outcome = 'collision'
            break
        if plan.name == 'driver_follow':
            stable = abs(ego.y) < 0.3 and abs(ego.yaw) < 0.05 and ego.v > 1.0
            stable &= all(clearance(ego, target) >= 0 for key, target in states.items() if key != 'E')
            stable_transfer_since = (time if stable_transfer_since is None else stable_transfer_since) if stable else None
            if stable_transfer_since is not None and time - stable_transfer_since >= 0.5 - 1e-6:
                policy.log(time, 'scripted_handover')
                outcome = 'scripted_handover'
                break
        stopping = (*STOP_ACTIONS, 'right_lane_stop', 'hold_shoulder_stop', 'emergency_stop')
        if plan.name in stopping and ego.v < 0.2:
            stop_started = time if stop_started is None else stop_started
            if time - stop_started >= PARAMS['post_stop_observation_s'] + 0.5:
                extents = ego.width * abs(math.cos(ego.yaw)) + ego.length * abs(math.sin(ego.yaw))
                parked = next((name for name, (_, center, width, _) in LANES.items()
                               if abs(ego.y - center) + extents / 2 <= width / 2), 'straddled_boundary')
                unverified = any(event['event'] == 'no_verified_fallback' for event in policy.events)
                outcome = f'unverified_stop_{parked}' if unverified else f'stopped_{parked}'
                policy.log(time, 'observed_stop', y_m=ego.y, parked_region=parked,
                           first_below_threshold_s=stop_started)
                break
        else:
            stop_started = None
        if tick < round(duration / PHYSICS_DT):
            backend.tick(plan, time + PHYSICS_DT)
    return {'run_id': case['run_id'], 'case': case, 'outcome': outcome,
            'events': policy.events, 'collisions': list(backend.collisions),
            'minimum_projected_clearance': minimum, 'frames': frames}


def load_carla(host, port, cache):
    import carla

    with socket.create_connection((host, port), timeout=4):
        pass
    client = carla.Client(host, port)
    cache.mkdir(parents=True, exist_ok=True)
    client.set_files_base_folder(str(cache.resolve()))
    client.set_timeout(8.0)
    server = client.get_server_version()
    if server != client.get_client_version():
        raise RuntimeError(f'CARLA client/server mismatch: {client.get_client_version()} / {server}')
    client.set_timeout(75.0)
    generation = carla.OpendriveGenerationParameters(wall_height=0.0, additional_width=0.0,
                                                      enable_pedestrian_navigation=False)
    world = client.generate_opendrive_world(build_road(), generation)
    road = world.get_map()
    for lane, (lane_id, _, width, kind) in LANES.items():
        waypoint = road.get_waypoint_xodr(1, lane_id, EGO_X_ORIGIN)
        if waypoint is None or waypoint.lane_type != getattr(carla.LaneType, kind) or abs(waypoint.lane_width - width) > 0.05:
            raise RuntimeError(f'Unexpected generated lane geometry for {lane}')
    check_shoulder_support(carla, world, road)
    return carla, client, world, server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('carla', 'model'), default='carla')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', default=2200, type=int)
    parser.add_argument('--cache', type=Path, default=Path(__file__).resolve().parents[1] / 'work' / 'pilot_carla_cache')
    parser.add_argument('--suite', choices=('pilot', 'qualification', 'formal'), default='pilot')
    parser.add_argument('--cases', nargs='+', help='explicit case IDs for a limited run')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    cases = {case['run_id']: case for case in build_manifest()['runs']}
    complete = tuple(key for key, case in cases.items() if case['role'] != 'boundary')
    suites = {'pilot': RUN_IDS, 'qualification': QUALIFICATION, 'formal': complete}
    requested = args.cases or suites[args.suite]
    if not set(requested).issubset(cases) or len(requested) != len(set(requested)):
        parser.error('Provide unique case IDs from the manifest.')
    args.output.mkdir(parents=True, exist_ok=True)
    if any((args.output / f'{run_id}.json').exists() for run_id in requested):
        parser.error('The output already contains one or more selected case IDs; specify a fresh directory.')
    version = None
    world = None
    if args.mode == 'carla':
        carla, client, world, version = load_carla(args.host, args.port, args.cache)
        saved = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = PHYSICS_DT
        settings.no_rendering_mode = True
        world.apply_settings(settings)
    try:
        aggregate = []
        for run_id in requested:
            case = cases[run_id]
            duration = PARAMS['horizon_s']
            if args.suite == 'pilot' and args.cases is None:
                duration = 4.0 if case['strategy'] == 'A' else 22.0
            backend = ModelWorld(case) if world is None else CarlaWorld(carla, client, world, case)
            try:
                result = run_case(case, backend, duration)
            finally:
                backend.close()
            result.update(backend=args.mode, carla_version=version, physics_step_s=PHYSICS_DT,
                          decision_step_s=DECISION_DT, observation_limit_s=duration,
                          brake_pedal_scale_mps2=BRAKE_PEDAL_SCALE if world is not None else None)
            result['score'] = score_run(result, PARAMS['longitudinal_clearance_margin_m'],
                                        PARAMS['lateral_clearance_margin_m'], CORRIDOR)
            aggregate.append({'run_id': run_id, **result['score'], 'events': result['events']})
            output = args.output / f'{run_id}.json'
            output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
            print(f'{run_id}: ' + ', '.join(event['event'] for event in result['events'])
                  + f", conflicts={len(result['collisions'])}, duration={result['frames'][-1]['time_s']:.2f}s", flush=True)
        summary = {'suite': args.suite, 'backend': args.mode, 'carla_version': version,
                   'parameters': {**PARAMS, 'brake_pedal_scale_mps2': BRAKE_PEDAL_SCALE}, 'runs': aggregate}
        (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    finally:
        if world is not None:
            world.apply_settings(saved)


if __name__ == '__main__':
    main()
