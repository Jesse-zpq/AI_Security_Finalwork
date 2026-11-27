"""Create the report figure from verified formal data (no model/CARLA mixing)."""
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
RESULTS = HERE / 'results'
LOGS = HERE.parent / 'CARLA_Member3-4-5_Handoff' / 'CARLA_repro_handoff' / 'CARLA_formal_results'
plt.rcParams.update({'font.family': 'Microsoft YaHei', 'axes.unicode_minus': False, 'font.size': 11})
fig, axes = plt.subplots(2, 2, figsize=(14, 9), layout='constrained')
colors = {'A': '#0072B2', 'B': '#D55E00', 'C': '#009E73'}
for strategy in 'ABC':
    r = json.loads((LOGS / f'T2-{strategy}.json').read_text(encoding='utf-8'))
    axes[0, 0].plot([f['time_s'] for f in r['frames']], [f['states']['E']['v'] for f in r['frames']],
                    label=f'T2-{strategy}', color=colors[strategy], linestyle='--' if strategy == 'C' else '-', linewidth=2.5)
r = json.loads((LOGS / 'P4-C.json').read_text(encoding='utf-8'))
axes[0, 0].plot([f['time_s'] for f in r['frames']], [f['states']['E']['v'] for f in r['frames']], color='#666666', linestyle=':', linewidth=2.5, label='P4-C 配对主组')
axes[0, 0].set(title='实际速度：T2 的 C 与立即停车 B 基本重合', xlabel='场景时间 / s', ylabel='自车速度 / (m/s)', xlim=(0, 25), ylim=(0, 27))
axes[0, 0].legend()
rows = list(csv.DictReader((RESULTS / 'initial_wait_forecast.csv').open(encoding='utf-8-sig')))
rows = [r for r in rows if r['run_id'] == 'T2-C' and r['backend'] == 'carla']
ax = axes[0, 1]
for index, key in enumerate(['shoulder_available', 'in_lane_available']):
    for state, color, marker in [('True', '#009E73', 'o'), ('False', '#D55E00', 'x')]:
        selected = [float(r['start_s']) for r in rows if r[key] == state]
        ax.scatter(selected, [index] * len(selected), color=color, marker=marker, s=28,
                   label=('可行' if state == 'True' else '否决') if index == 0 else None)
ax.axvline(1.9, color='#333333', linestyle='--', label='截止 1.90 s')
ax.axvspan(1.9, 5, color='grey', alpha=0.1)
ax.set(yticks=[0, 1], yticklabels=['向右靠边', '原车道停车'], ylim=(-0.5, 1.6),
       title='起点预测：等待中可启动的后备（非实际轨迹）', xlabel='假设等待后的启动时刻 / s')
ax.legend(loc='upper right', fontsize=9)
ax.text(2.45, 0.7, '2.30 s 起两项均被否决\n不应等待到这里', fontsize=10)
ax = axes[1, 0]
for y, label, end, color in [(2, 'P3-C 及时接管', 2.5, '#0072B2'), (1, 'P4-C 无人接管', 4.7, '#666666'), (0, 'T2-C 右侧不足', 0, '#D55E00')]:
    ax.plot([0, end], [y, y], linewidth=5, color=color)
    ax.scatter([end], [y], color=color, s=70, zorder=3)
ax.scatter([2], [2], marker='|', s=150, color='#333333')
ax.text(2, 2.15, '2.00 就绪 → 2.50 移交', fontsize=10)
ax.text(1, 1.15, '0 s 请求；4.70 s 启动停车', fontsize=10)
ax.text(0.08, 0.15, '0 s 启动停车；不发出接管请求', fontsize=10)
ax.plot([1.9, 1.9], [-0.2, 0.12], color='#D55E00', linestyle='--')
ax.text(1.9, -0.4, '预测截止 1.90 s', fontsize=10)
ax.set(yticks=[0, 1, 2], yticklabels=['T2-C', 'P4-C', 'P3-C'], xlim=(-0.2, 5.6), ylim=(-0.65, 2.6),
       xlabel='场景时间 / s', title='实际事件：预测截止不等于后备启动时间')
rows = list(csv.DictReader((RESULTS / 'formal_pairs.csv').open(encoding='utf-8-sig')))
ax = axes[1, 1]
for offset, group, color in [(-0.18, 'P4', '#0072B2'), (0.18, 'T2', '#D55E00')]:
    values = [float(next(r for r in rows if r['run_id'] == group + '-' + s)['first2s_rear_gap_m']) for s in 'ABC']
    bars = ax.bar([i + offset for i in range(3)], values, width=0.36, color=color, label=group)
    ax.bar_label(bars, fmt='%.2f', fontsize=10)
ax.set(xticks=[0, 1, 2], xticklabels=['A 常规跟驰', 'B 立即停车', 'C 动作筛选'], ylim=(0, 60),
       ylabel='最小后向车身净空 / m', title='共同前 2 s：提前制动压缩后向余量')
ax.legend()
for ax in axes.flat:
    ax.spines[['top', 'right']].set_visible(False)
fig.suptitle('右侧间隙压力测试｜只改变 R 初始位置 −50 → −10 m\n既有 CARLA 正式日志复算；每种工况每策略 1 条轨迹', fontsize=17)
fig.savefig(RESULTS / 'right_gap_comparison.png', dpi=180)
plt.close(fig)
print('Created right_gap_comparison.png')
