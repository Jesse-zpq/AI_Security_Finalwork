"""Usage: import this adapter from the member 3, 4 or 5 entry point.

The logged physical states and controls stay fixed during replay. The original
runner recomputes perception, decisions, endpoints and scoring from those states.
"""

import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'core'))

from carla_metrics import score_run
from carla_pilot import CORRIDOR, DECISION_DT, PARAMS, State, observed_targets, run_case


def load_case(run_id):
    """Read a formal trace, or the independent X1-C pre-batch trace."""
    folder = 'CARLA_批跑前验证' if run_id == 'X1-C' else 'CARLA_formal_results'
    path = ROOT / folder / f'{run_id}.json'
    if not path.is_file():
        raise ValueError(f'No recorded run named {run_id}')
    return json.loads(path.read_text(encoding='utf-8'))


def frame_input(record, time):
    """Return the exact ego/observed states and rear acceleration at a decision tick."""
    frames = record['frames']
    interval = round(DECISION_DT / record['physics_step_s'])
    index = round(time / record['physics_step_s'])
    if (index < 0 or index >= len(frames) or index % interval
            or not math.isclose(frames[index]['time_s'], time, abs_tol=1e-8)):
        raise ValueError(f'{time:.3f} s is outside the recorded decision ticks for {record["run_id"]}')
    frame = frames[index]
    states = {actor: State(**value) for actor, value in frame['states'].items()}
    previous = frames[max(index - interval, 0)]
    observed = observed_targets(states, record['case']['left_observation_lost'])
    rear_acceleration = (observed['Bk'].v - previous['states']['Bk']['v']) / DECISION_DT
    return states['E'], observed, rear_acceleration


class RecordedWorld:
    """Provide the original runner's backend interface from a saved CARLA trace."""

    def __init__(self, record):
        self.record = record
        self.index = 0
        self.frames = record['frames']

    @property
    def frame(self):
        return self.frames[self.index]['world_frame']

    @property
    def controls(self):
        return self.frames[self.index]['controls_for_this_frame']

    @property
    def collisions(self):
        return [item for item in self.record['collisions'] if item['frame'] <= self.frame]

    def sample(self):
        return {actor: State(**value) for actor, value in self.frames[self.index]['states'].items()}

    def tick(self, plan, next_time):
        """Advance to the next recorded physical state. The runner records the new decision."""
        self.index += 1
        if self.index >= len(self.frames) or not math.isclose(self.frames[self.index]['time_s'], next_time, abs_tol=1e-8):
            raise RuntimeError(f'Recomputed run requested a frame beyond the trace at {next_time:.3f} s')


def reproduce(record):
    """Recompute a saved run and fail if any event, action, endpoint or score differs."""
    if record['backend'] != 'carla':
        raise ValueError('The handoff replay expects a trace from the CARLA backend.')
    recomputed = run_case(record['case'], RecordedWorld(record), record['observation_limit_s'])
    metrics = score_run(recomputed, PARAMS['longitudinal_clearance_margin_m'],
                        PARAMS['lateral_clearance_margin_m'], CORRIDOR)
    for key in ('outcome', 'events', 'collisions', 'frames', 'minimum_projected_clearance'):
        if recomputed[key] != record[key]:
            raise AssertionError(f'{record["run_id"]}: recomputed {key} does not match')
    if metrics != record['score']:
        raise AssertionError(f'{record["run_id"]}: recomputed score does not match')
    return recomputed, metrics
