r"""Member 5 · 交叉复核：独立重放一条压力工况，核对评分口径。

按汇报方案 §5，5 号交叉复核 6 号的一个压力条件。6 号产物到位前，先用公共交接包里的
既有日志对 T3-B（本车道后车逼近、立即制动基线）做同样复核；6 号的新目录可直接用
--log-root 指向。

用法（在仓库根目录运行，Python 3.10+，标准库）：

    python -B ".\Member5_基线与主对照\cross_check.py"
    python -B ".\Member5_基线与主对照\cross_check.py" --cases T3-B T2-C
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
HANDOFF = ROOT / 'CARLA_Member3-4-5_Handoff' / 'CARLA_repro_handoff'
sys.path.insert(0, str(HANDOFF))

from replay import load_case, reproduce  # noqa: E402

from carla_metrics import score_run  # noqa: E402
from carla_pilot import CORRIDOR, PARAMS  # noqa: E402

TIMELINE = ('driver_ready', 'takeover_requested', 'deadline_updated', 'scripted_handover',
            'fallback_started', 'shoulder_second_segment', 'no_verified_fallback', 'observed_stop')


def projected_gap(ego, target):
    """按评分口径重写一次：两个车身在各自航向下的轴对齐投影间距。"""
    def extent(actor):
        cosine, sine = abs(math.cos(actor['yaw'])), abs(math.sin(actor['yaw']))
        return (actor['length'] * cosine + actor['width'] * sine,
                actor['width'] * cosine + actor['length'] * sine)

    ego_x, ego_y = extent(ego)
    target_x, target_y = extent(target)
    dx, dy = target['x'] - ego['x'], target['y'] - ego['y']
    return abs(dx) - (ego_x + target_x) / 2, abs(dy) - (ego_y + target_y) / 2, dx


def independent_metrics(record, window_s=2.0):
    """不用 carla_metrics，直接从帧里重算前 window_s 的最小前向与后向净空。"""
    log_margin, lat_margin = (PARAMS['longitudinal_clearance_margin_m'],
                              PARAMS['lateral_clearance_margin_m'])
    best = {'front_gap_m': None, 'rear_gap_m': None}
    for frame in record['frames']:
        if frame['time_s'] > window_s + 1e-8:
            break
        ego = frame['states']['E']
        for name, target in frame['states'].items():
            if name == 'E':
                continue
            gap_x, gap_y, dx = projected_gap(ego, target)
            if gap_y > lat_margin:
                continue
            key = 'front_gap_m' if dx >= 0 else 'rear_gap_m'
            if best[key] is None or gap_x < best[key]['value']:
                best[key] = {'value': gap_x, 'time_s': frame['time_s'], 'actor': name}
    return best


def check(run_id, log_root):
    record = json.loads((log_root / f'{run_id}.json').read_text(encoding='utf-8'))
    replayed, metrics = reproduce(record)

    reported = record['score']['shared_window_s']
    independent = independent_metrics(record)
    differences = {}
    for key, item in independent.items():
        if item is None and reported[key] is None:
            continue
        assert item is not None and reported[key] is not None, f'{run_id}: {key} 一侧为空'
        assert math.isclose(item['value'], reported[key]['value'], abs_tol=1e-9), (
            f'{run_id}: {key} 重算 {item["value"]} 与日志 {reported[key]["value"]} 不一致')
        differences[key] = 0.0

    stops = [event for event in record['events'] if event['event'] == 'observed_stop']
    observation = None
    if stops:
        observation = round(stops[0]['time_s'] - stops[0]['first_below_threshold_s'], 3)
        assert math.isclose(observation, 3.5, abs_tol=0.051), f'{run_id}: 停后观察时长 {observation}'

    return {
        'run_id': run_id,
        'log_sha256': hashlib.sha256((log_root / f'{run_id}.json').read_bytes()).hexdigest(),
        'outcome': record['score']['outcome'],
        'collision': record['collisions'][:1],
        'terminal_time_s': record['score']['terminal_time_s'],
        'first2s_front_gap_m': round(reported['front_gap_m']['value'], 3) if reported['front_gap_m'] else None,
        'first2s_rear_gap_m': round(reported['rear_gap_m']['value'], 3) if reported['rear_gap_m'] else None,
        'independent_rear_gap_m': round(independent['rear_gap_m']['value'], 6) if independent['rear_gap_m'] else None,
        'independent_front_gap_m': round(independent['front_gap_m']['value'], 6) if independent['front_gap_m'] else None,
        'post_stop_observation_s': observation,
        'replay_events_identical': replayed['events'] == record['events'],
        'replay_score_identical': metrics == record['score'],
        'recomputed': score_run(record, PARAMS['longitudinal_clearance_margin_m'],
                                PARAMS['lateral_clearance_margin_m'], CORRIDOR) == record['score'],
        'timeline': [{'time_s': event['time_s'], 'event': event['event'],
                      'action': event.get('action')}
                     for event in record['events'] if event['event'] in TIMELINE],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', nargs='+', default=['T3-B'])
    parser.add_argument('--log-root', type=Path,
                        default=HANDOFF / 'CARLA_formal_results')
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parent / 'results')
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    payload = []
    for run_id in args.cases:
        item = check(run_id, args.log_root)
        payload.append(item)
        print(json.dumps(item, ensure_ascii=False, indent=2))
    (args.output / 'cross_check.json').write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(f'\n已写出 {args.output / "cross_check.json"}')


if __name__ == '__main__':
    main()
