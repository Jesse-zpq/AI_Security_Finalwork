# Member4 · 右侧间隙压力测试

**汇报使用[第6页 PPT](第6页_接管窗口与右侧间隙.pptx)，讲稿已放在讲者备注中。** 按[汇报方案](../自动驾驶安全降级_期末汇报方案.md) §4.2，T2-A/B/C对照P4-A/B/C“跟驰意图、无人接管”，仅将右后车初始位置由−50 m改为−10 m，速度仍为27 m/s，其他输入一致。

## 必要文件

| 文件 | 用途 |
|---|---|
| [第6页 PPT](第6页_接管窗口与右侧间隙.pptx) / [汇报图](results/right_gap_comparison.png) | 可编辑时间线、讲稿及结果图 |
| [正式配对表](results/formal_pairs.csv) | 既有CARLA六条轨迹的复算指标，按 `run_id` 合并 |
| [二维配对表](results/model_pairs.csv) / [原始轨迹](results/model_runs) | 新跑T2三条和P4三条，含完整状态、事件及评分 |
| [截止审计](results/deadline_audit.csv) / [等待预测](results/initial_wait_forecast.csv) | 96个决策点截止复算、204个起点假设等待采样点 |
| [验证与来源](results/verification.json) | 原始数据和核心代码哈希、核验数量 |
| [run_t2.py](run_t2.py) / [plot_results.py](plot_results.py) | 配对与窗口核验、指标汇总、绘图 |

**CARLA部分是已有日志复算，新运行使用二维ModelWorld。** 物理原始记录留在[公共交接目录](../CARLA_Member3-4-5_Handoff/CARLA_repro_handoff/CARLA_formal_results)，不重复复制。P3-C仅用于及时接管时间线说明，不计入新增实验。事件可直接读取原始轨迹的 `events` 字段。

## 结果与解释

下表采用既有CARLA结果，单位为秒。全部无人接管且未观察到碰撞，“—”表示无该量或事件。

| 运行 | 初始截止 | 后备启动 | 首次低于停车阈值 | 观察终点 | 结果 |
|---|---:|---:|---:|---:|---|
| P4-A / T2-A | — | — | — | 25.00 | 继续跟驰 |
| P4-B / T2-B | — | 0.00 | 6.30 | 9.80 | 中道停车 |
| P4-C | 4.70 | 4.70 | 10.20 | 13.70 | 中道停车 |
| T2-C | 1.90 | **0.00** | 6.30 | 9.80 | 中道停车 |

T2-C起点预测的0–1.90 s窗口内，全部20个决策采样点的靠边候选均由R否决，原车道停车均可行。后者最后可行采样时刻为2.20 s，扣除0.30 s启动裕度得到1.90 s截止。窗口不足设定的2.00+0.50=2.50 s接管流程，因此C**没有发出接管请求，在0 s立即停车**。P4-C则在0 s请求接管，等待至4.70 s启动停车。

新二维结果的分支一致：T2-C初始截止1.80 s、0 s启动，B/C在9.75 s完成停车观察；P4-C截止从4.70 s收缩至4.50 s并启动，13.50 s结束。汇报统一采用CARLA时刻。

**本项展示等待窗口与启动时刻改变，最终停车位置均为中道。** 共同前2 s最小后向净空由P4-C的50.908 m降至T2-C的43.035 m，提前制动也有后向代价。T2中C与B行为基本一致，不能据此声称C全面优于B。

## 参数与指标口径

- 初始中心位置/速度（m、m/s）：E=0/25，F=50/20，L=−20/33，Bk=−60/27；R仅位置变化。Bk在2 s后以−5 m/s²制动。故障从0 s持续屏蔽左侧观测，L仍存在于真值。
- 车道宽3.6 m、路肩宽3.0 m；物理步长0.05 s、决策步长0.10 s；等待上限5 s、目标制动加速度−4 m/s²。停车阈值0.2 m/s，稳定确认0.5 s加停后观察3 s，运行上限25 s。
- 规划裕度为纵向2.0 m、横向0.2 m。CARLA使用逐帧车身尺寸和航向，二维默认5.0×1.8 m；完整参数与策略沿用公共交接包。
- CSV的 `first2s_` 为共同前2 s，`full_` 为各自至终点，后者观察时长不同。净空为投影车身间距，候选裕度另扣规划余量；加速度为带符号最小值，空白表示无适用样本。
- 等待预测是假设轨迹，不是实际执行等待或连续时间安全证明。结论限于一次确定配置、脚本交通和有限观察期。基础23次中仅计T2三条，不重复计入P4复跑或二维验证。

## 复现

在仓库根目录运行，Python3.10+，绘图另需Matplotlib。核验读取保留的六条二维轨迹和公共CARLA日志，不重复仿真。

```powershell
python -B 'Member4_右侧间隙压力测试/run_t2.py'
python -B 'Member4_右侧间隙压力测试/plot_results.py'
```

重新仿真时使用新目录，保留本次原始记录：

```powershell
python -B 'CARLA_Member3-4-5_Handoff/CARLA_repro_handoff/core/carla_pilot.py' --mode model --cases P4-A P4-B P4-C T2-A T2-B T2-C --output 'Member4_右侧间隙压力测试/results/model_rerun'
python -B 'Member4_右侧间隙压力测试/run_t2.py' --model-dir 'Member4_右侧间隙压力测试/results/model_rerun' --output 'Member4_右侧间隙压力测试/results/rerun_audit'
```

新CARLA实跑按[交接说明](../CARLA_Member3-4-5_Handoff/CARLA_repro_handoff/README_Usage.md)配置独占服务端。
