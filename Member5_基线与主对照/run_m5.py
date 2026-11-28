r"""Member 5 · 基线与主对照：2 条正常观测参照 + 12 条主对照（A/B/C × 意图 × 接管）。

用法（在仓库根目录运行，Python 3.10+，绘图另需 matplotlib）：

    python -B ".\Member5_基线与主对照\run_m5.py"
    python -B ".\Member5_基线与主对照\run_m5.py" --output <另一目录>

脚本读取公共交接包里的既有 CARLA 正式日志，用原评分器重新计算每条轨迹的指标，
核对 summary.json、事件与动作，再写出配对结果表和主结果图。它不重新仿真。
"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
HANDOFF = ROOT / 'CARLA_Member3-4-5_Handoff' / 'CARLA_repro_handoff'
sys.path.insert(0, str(HANDOFF / 'core'))

from carla_metrics import score_run                       # noqa: E402
from carla_pilot import CORRIDOR, PARAMS                  # noqa: E402
from carla_preflight import LANES, REFERENCE, build_manifest  # noqa: E402

LOGS = HANDOFF / 'CARLA_formal_results'
BACKEND = 'carla'
REFERENCES = ['N1-A', 'N2-A']
MAIN = ['P1-A', 'P1-B', 'P1-C', 'P2-A', 'P2-B', 'P2-C',
        'P3-A', 'P3-B', 'P3-C', 'P4-A', 'P4-B', 'P4-C']
COLOURS = {'A': '#2F6FA8', 'B': '#C9762B', 'C': '#2E8B57'}
LABELS = {'A': 'A 常规规划', 'B': 'B 立即停车', 'C': 'C 动作筛选'}
INTENT = {'P1': '左变道', 'P2': '左变道', 'P3': '跟驰', 'P4': '跟驰',
          'N1': '左变道', 'N2': '跟驰'}
TAKEOVER = {'P1': '及时接管', 'P2': '无人接管', 'P3': '及时接管', 'P4': '无人接管',
            'N1': '正常观测', 'N2': '正常观测'}


def sha256(path):
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def load(run_id, log_root=LOGS):
    path = log_root / f'{run_id}.json'
    if not path.is_file():
        raise SystemExit(f'缺少正式日志：{path}')
    return json.loads(path.read_text(encoding='utf-8'))


def event_times(record):
    table = {}
    for event in record['events']:
        table.setdefault(event['event'], []).append(event)
    return table


def first(table, name, field='time_s'):
    rows = table.get(name)
    return rows[0][field] if rows else None


def stop_position(record, time_s):
    if time_s is None:
        return None
    frames = record['frames']
    return next((round(f['states']['E']['x'], 3) for f in frames
                 if math.isclose(f['time_s'], time_s, abs_tol=1e-6)), None)


def metric(block, key, field='value'):
    item = block.get(key)
    if not item:
        return None
    value = item[field]
    return round(value, 3) if isinstance(value, float) else value


def reason_text(value):
    """把否决原因从日志里的字典压成一行文本，便于表格阅读。"""
    if isinstance(value, dict) and 'object' in value:
        return f"{value['object']}@{value['time_s']:.2f}s"
    return value


def build_rows(records):
    rows = []
    for run_id, record in records.items():
        case = record['case']
        name = run_id.split('-')[0]
        strategy = case['strategy']
        events = event_times(record)
        score = record['score']
        shared, full = score['shared_window_s'], score['until_endpoint']
        collision = record['collisions'][0] if record['collisions'] else {}
        handover = first(events, 'scripted_handover')
        fallback = events.get('fallback_started', [])
        stop_event = events.get('observed_stop', [])
        rows.append({
            'run_id': run_id,
            'backend': record['backend'],
            'intent': INTENT[name],
            'takeover_condition': TAKEOVER[name],
            'strategy': strategy,
            'matched_run': case['pair_with'],
            'action_at_0': record['frames'][0]['action'],
            'left_decision': ('accepted' if 'left_accepted' in events
                              else 'rejected' if 'left_rejected' in events else None),
            'left_reject_reason': reason_text(
                next((e.get('trigger') for e in events.get('left_rejected', [])), None)),
            'deadline_0_s': first(events, 'deadline_updated', 'deadline_s'),
            'takeover_request_s': first(events, 'takeover_requested'),
            'driver_ready_s': first(events, 'driver_ready'),
            'handover_s': handover,
            'fallback_action': fallback[0]['action'] if fallback else None,
            'fallback_s': fallback[0]['time_s'] if fallback else None,
            'first_below_stop_s': stop_event[0]['first_below_threshold_s'] if stop_event else None,
            'stop_s': stop_event[0]['time_s'] if stop_event else None,
            'stop_x_m': stop_position(record, stop_event[0]['time_s'] if stop_event else None),
            'collision_actor': collision.get('other'),
            'collision_s': collision.get('time_s'),
            'outcome': score['outcome'],
            'end_s': score['terminal_time_s'],
            **{f'first2s_{key}': metric(shared, key) for key in
               ('front_gap_m', 'rear_gap_m', 'left_gap_m', 'right_gap_m',
                'longitudinal_ttc_s', 'ego_acceleration_mps2')},
            **{f'full_{key}': metric(full, key) for key in
               ('front_gap_m', 'rear_gap_m', 'left_gap_m', 'right_gap_m',
                'longitudinal_ttc_s', 'ego_acceleration_mps2')},
        })
    return rows


def verify(records, log_root=LOGS):
    """核对 14 条日志：清单、指标重算、事件与终点。任一不符即中止。"""
    manifest = {case['run_id']: case for case in build_manifest()['runs']}
    summary = json.loads((log_root / 'summary.json').read_text(encoding='utf-8'))
    summary_rows = {row['run_id']: row for row in summary['runs']}
    max_recompute = 0.0
    for run_id, record in records.items():
        case = manifest[run_id]
        assert record['case'] == case, run_id
        assert record['backend'] == BACKEND and record['carla_version'] == '1.0', run_id
        recalculated = score_run(record, PARAMS['longitudinal_clearance_margin_m'],
                                 PARAMS['lateral_clearance_margin_m'], CORRIDOR)
        assert recalculated == record['score'], f'{run_id}: 指标重算不一致'
        assert {key: summary_rows[run_id][key] for key in recalculated} == recalculated, run_id
        assert summary_rows[run_id]['events'] == record['events'], run_id
        for key in ('front_gap_m', 'rear_gap_m', 'left_gap_m', 'right_gap_m'):
            for block in ('shared_window_s', 'until_endpoint'):
                item = record['score'][block][key]
                if item:
                    max_recompute = max(max_recompute, 0.0)
    return max_recompute, manifest


def write_csv(rows, path):
    columns = list(rows[0])
    with path.open('w', encoding='utf-8-sig', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def build_figure(records, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    plt.rcParams.update({
        'font.sans-serif': ['Microsoft YaHei', 'SimHei', 'Noto Sans CJK SC'],
        'axes.unicode_minus': False,
        'font.size': 11.5,
        'axes.titlesize': 13.6,
        'axes.titleweight': 'bold',
        'axes.labelsize': 11.5,
        'xtick.labelsize': 11.0,
        'ytick.labelsize': 11.0,
        'axes.edgecolor': '#7A8A8F',
        'axes.linewidth': 0.8,
        'figure.facecolor': 'white',
        'axes.facecolor': '#FCFDFD',
    })

    def trace(run_id, key):
        record = records[run_id]
        collision = record['collisions'][0]['time_s'] if record['collisions'] else None
        frames = [f for f in record['frames']
                  if collision is None or f['time_s'] < collision - 1e-9]
        return [f['time_s'] for f in frames], [f['states']['E'][key] for f in frames]

    def panel_title(ax, title, note):
        ax.set_title(title, loc='left', pad=19)
        ax.text(0.0, 1.015, note, transform=ax.transAxes, fontsize=10.4,
                color='#5B6B70', va='bottom', ha='left')

    fig, axes = plt.subplots(2, 2, figsize=(13.2, 5.4), dpi=190)
    (ax_end, ax_speed), (ax_lat, ax_gap) = axes

    # (a) 终点位置 ---------------------------------------------------------
    cases = [('P2-A', 'A 常规规划', 'A'), ('P2-B', 'B 立即停车', 'B'), ('P2-C', 'C 动作筛选', 'C')]
    ends = [records[run]['frames'][-1]['states']['E']['x'] for run, _, _ in cases]
    bars = ax_end.bar([0, 1, 2], ends, width=0.52,
                      color=[COLOURS[s] for _, _, s in cases], zorder=3)
    bars[0].set_edgecolor('#C0392B')
    bars[0].set_linewidth(2.0)
    bars[0].set_hatch('///')
    for index, (value, (run_id, _, strategy)) in enumerate(zip(ends, cases)):
        label = f'{value:.0f} m' if strategy != 'A' else f'{value:.0f} m\n碰撞终止'
        ax_end.annotate(label, xy=(index, value), xytext=(0, 5), textcoords='offset points',
                        ha='center', va='bottom', fontsize=11.5,
                        color='#C0392B' if strategy == 'A' else '#183941',
                        fontweight='bold' if strategy == 'A' else 'normal')
    ax_end.set_xticks([0, 1, 2])
    ax_end.set_xticklabels([name for _, name, _ in cases])
    ax_end.set_ylim(0, 205)
    ax_end.set_ylabel('终点纵向位置 / m')
    ax_end.grid(axis='y', color='#E3EAEC', lw=0.8)
    ax_end.set_axisbelow(True)
    panel_title(ax_end, f'a  终点：碰撞、{ends[1]:.0f} m、{ends[2]:.0f} m',
                '左变道意图、无人接管；A 的轨迹在 2.00 s 被 L 终止')

    # (b) 速度 -------------------------------------------------------------
    for run_id, style in (('P2-A', dict(color=COLOURS['A'], lw=2.2)),
                          ('P4-A', dict(color=COLOURS['A'], lw=1.3, ls=(0, (3, 2)), alpha=0.55)),
                          ('P2-B', dict(color=COLOURS['B'], lw=2.2)),
                          ('P2-C', dict(color=COLOURS['C'], lw=2.2))):
        t, v = trace(run_id, 'v')
        ax_speed.plot(t, v, **style)
    for run_id, when, colour in (('P2-B', 6.3, COLOURS['B']), ('P2-C', 10.2, COLOURS['C'])):
        ax_speed.plot([when], [0.0], marker='o', ms=6.5, color=colour, zorder=6)
    box = dict(facecolor='white', edgecolor='none', pad=1.2)
    ax_speed.text(6.65, 3.0, 'B 6.30 s', fontsize=10.4, color=COLOURS['B'], bbox=box)
    ax_speed.text(11.7, 3.0, 'C 10.20 s', fontsize=10.4, color=COLOURS['C'], bbox=box)
    ax_speed.set_xlim(-0.6, 25.6)
    ax_speed.set_ylim(-0.8, 26.8)
    ax_speed.set_xlabel('场景时间 / s')
    ax_speed.set_ylabel('自车速度 / (m/s)')
    ax_speed.grid(axis='y', color='#E3EAEC', lw=0.8)
    ax_speed.set_axisbelow(True)
    panel_title(ax_speed, 'b  速度：B 在 0 s 制动，C 在 4.70 s 才制动',
                '圆点为首次低于 0.2 m/s；虚线为跟驰分支 P4-A')

    # (c) 横向位置 ---------------------------------------------------------
    for run_id, style in (('P2-A', dict(color=COLOURS['A'], lw=2.2)),
                          ('P4-A', dict(color=COLOURS['A'], lw=1.3, ls=(0, (3, 2)), alpha=0.55)),
                          ('P2-B', dict(color=COLOURS['B'], lw=2.2)),
                          ('P2-C', dict(color=COLOURS['C'], lw=2.2))):
        t, y = trace(run_id, 'y')
        ax_lat.plot(t, y, **style)
    ax_lat.axhline(LANES['left'][1], color='#B9C6CA', lw=0.9, ls='--')
    ax_lat.text(15.5, LANES['left'][1] + 0.10, '左道中心 +3.60 m', fontsize=10.2, color='#5B6B70')
    ax_lat.axhline(0.0, color='#B9C6CA', lw=0.9, ls='--')
    ax_lat.text(15.5, 0.10, '中道中心 0 m', fontsize=10.2, color='#5B6B70')
    ax_lat.annotate('A 在 2.00 s 越过车道线', xy=(2.0, 2.32), xytext=(3.6, 3.02),
                    fontsize=10.4, color=COLOURS['A'],
                    arrowprops=dict(arrowstyle='-', color=COLOURS['A'], lw=0.9))
    ax_lat.set_xlim(-0.6, 25.6)
    ax_lat.set_ylim(-0.45, 4.05)
    ax_lat.set_xlabel('场景时间 / s')
    ax_lat.set_ylabel('横向位置 / m')
    panel_title(ax_lat, 'c  横向：只有 A 向左侧移动',
                'B、C 全程保持中道；A 的左移在 2.00 s 被打断')

    # (d) 前 2 s 最小后向净空 ---------------------------------------------
    groups = ['A', 'B', 'C']
    width = 0.34
    for index, (intent_prefix, colour, label) in enumerate(
            (('P2', '#4C8FBF', '左变道意图 P2'), ('P4', '#D98C4A', '跟驰意图 P4'))):
        values = []
        for strategy in groups:
            item = records[f'{intent_prefix}-{strategy}']['score']['shared_window_s']['rear_gap_m']
            values.append(item['value'] if item else 0.0)
        positions = [i + (index - 0.5) * width for i in range(3)]
        bars = ax_gap.bar(positions, values, width=width, color=colour, label=label, zorder=3)
        for bar, value, strategy in zip(bars, values, groups):
            if value < 0:
                bar.set_color('#C0392B')
                bar.set_hatch('///')
                ax_gap.annotate('碰撞\n−0.51', xy=(bar.get_x() + width / 2, 0.0), xytext=(0, 4),
                                textcoords='offset points', ha='center', va='bottom',
                                fontsize=10.0, color='#C0392B', fontweight='bold', linespacing=1.15,
                                bbox=dict(facecolor='white', edgecolor='none', pad=1.0))
            elif strategy == 'B':
                ax_gap.annotate(f'{value:.1f}', xy=(bar.get_x() + width / 2, value), xytext=(0, 3),
                                textcoords='offset points', ha='center', fontsize=10.4,
                                color='#183941', fontweight='bold')
            else:
                ax_gap.annotate(f'{value:.1f}', xy=(bar.get_x() + width / 2, value), xytext=(0, 3),
                                textcoords='offset points', ha='center', fontsize=10.4,
                                color='#39515A')
    ax_gap.set_xticks(range(3))
    ax_gap.set_xticklabels([LABELS[s] for s in groups])
    ax_gap.set_ylim(-9, 80)
    ax_gap.set_ylabel('前 2 s 最小后向净空 / m')
    ax_gap.grid(axis='y', color='#E3EAEC', lw=0.8)
    ax_gap.set_axisbelow(True)
    ax_gap.legend(loc='upper left', frameon=False, fontsize=10.4, ncol=2,
                  handlelength=1.4, columnspacing=1.8, borderaxespad=0.2)
    panel_title(ax_gap, 'd  后向代价：B 立即制动少 7.9 m',
                '净空为投影车身间距；A 在左变道分支与 L 碰撞')

    fig.tight_layout(h_pad=2.9, w_pad=2.2, rect=(0.008, 0.008, 0.992, 0.99))
    fig.savefig(path, dpi=190)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parent / 'results')
    parser.add_argument('--log-root', type=Path, default=LOGS)
    args = parser.parse_args()
    log_root = args.log_root

    args.output.mkdir(parents=True, exist_ok=True)
    records = {}
    for run_id in REFERENCES + MAIN:
        records[run_id] = load(run_id, log_root)

    verify(records, log_root)
    rows = build_rows(records)
    write_csv(rows, args.output / 'main_pairs.csv')
    build_figure(records, args.output / 'main_contrast.png')

    payload = {
        'suite': 'member5_main_and_references',
        'backend': BACKEND,
        'runs': {run_id: {'sha256': sha256(LOGS / f'{run_id}.json'),
                          'outcome': records[run_id]['score']['outcome'],
                          'end_s': records[run_id]['score']['terminal_time_s']}
                 for run_id in records},
        'source_code': {'carla_metrics.py': sha256(HANDOFF / 'core' / 'carla_metrics.py'),
                        'carla_pilot.py': sha256(HANDOFF / 'core' / 'carla_pilot.py'),
                        'carla_preflight.py': sha256(HANDOFF / 'core' / 'carla_preflight.py')},
        'checks': {
            'reference_runs': len(REFERENCES),
            'main_runs': len(MAIN),
            'score_recomputed_from_frames': True,
            'summary_rows_matched': True,
            'events_matched': True,
        },
        'expectations': {
            'A_left_collisions': [r['run_id'] for r in rows
                                  if r['left_decision'] == 'accepted' and r['collision_s']],
            'B_immediate_stops': [r['run_id'] for r in rows if r['fallback_s'] == 0.0],
            'C_handovers': [r['run_id'] for r in rows if r['handover_s']],
            'C_wait_then_stop': [r['run_id'] for r in rows if r['fallback_s'] and r['fallback_s'] > 0],
        },
    }
    (args.output / 'verification.json').write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')

    for row in rows:
        print(f"{row['run_id']:6s} {row['outcome']:16s} end={row['end_s']:>5} "
              f"fallback={row['fallback_s']} handover={row['handover_s']} "
              f"collision={row['collision_actor']}@{row['collision_s']} "
              f"first2s_rear={row['first2s_rear_gap_m']}")
    print(f'\n已写出 {args.output}')


if __name__ == '__main__':
    main()
