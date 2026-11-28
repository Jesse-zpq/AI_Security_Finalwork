# Member6 · 压力测试与结论（无需 CARLA 安装）

本目录沿用 Member3 / Member4 的实验口径：既有 CARLA 正式日志复算 + 独立二维 ModelWorld 新仿真。复用 Member5 的 `member5_baselines.py` 运行入口以及公共 `core/`，不修改策略、初值、盲区先验或评分器。不运行 CARLA 服务端、不安装 CARLA Python 客户端、不运行完整 Bench2Drive 框架。

**直接提交使用：** `第9-10页_压力测试与结论.pptx`（两页、可编辑、含讲者备注）；`实验结论.md`；`results/verified_offline/`（本次实际执行得到的完整输出）。既有 CARLA 轨迹仍保存在公共目录，避免重复打包。

## 1. 实验矩阵

| 测试 | 新增轨迹 | 配对参照 | 改动 |
|---|---|---|---|
| T1 | T1-A/B/C | P2-A/B/C | 仅删除实际左后车 L，盲区先验不变 |
| T2 | T2-A/B/C | P4-A/B/C | R 初始中心 x：−50→−10 m |
| T3 | T3-A/B/C | P4-A/B/C | Bk 初始中心 x：−60→−20 m |
| X1 | X1-C | P4-C（背景比较） | F=+10 m，R=−10 m，L 从1 s开始3.5 s切入，左先验拓展至中道 |
| 独立复跑 | P4-C | 既有 P4-C | 不改输入，复核主对照 |

个人任务 9+1+1=11 条；为同后端配对额外重跑 P2-A/B/C、P4-A/B，共16条新二维轨迹。16条既有 CARLA 日志为相同工况的参照，不是新 CARLA 运行。全组23条基础轨迹计数不变，X1另计。

## 2. 本地运行（Windows PowerShell）

将整个 `Member6_压力测试与结论` 文件夹放进原仓库根目录，和 `Member3_左盲区压力测试`、`Member4_右侧间隙压力测试`、`CARLA_Member3-4-5_Handoff` 同级。务必保留公共交接目录和前两位同学的 CSV，因为交叉核验需要它们。路径包含中文或空格时保留引号。Python 3.10+。

在项目根目录运行：

```powershell
python -m pip install -r '.\Member6_压力测试与结论\requirements.txt'
python -B '.\Member6_压力测试与结论\run_offline.py'
```

默认创建 `results/run_年月日_时分秒/`，每次使用新目录，不覆盖已提供结果。程序先复算历史记录，再通过 Member5 入口运行 ModelWorld，输出两类数据、配对表、图、核验表、两页 PPT。候选等待扫描需要重复计算，不是程序卡死；以 `Complete:` 和 `offline_verification.json` 为完成标志。

如需指定目录（必须不存在）：

```powershell
python -B '.\Member6_压力测试与结论\run_offline.py' --output '.\Member6_压力测试与结论\results\my_run'
```

仅复查/重新生成已有结果目录的跨后端核对和 PPT（不生成新轨迹）：

```powershell
python -B '.\Member6_压力测试与结论\run_offline.py' --summarize-only --output '.\Member6_压力测试与结论\results\verified_offline'
```

仅生成两页 PPT（要求目录内已有两类完整结果）：

```powershell
python -B '.\Member6_压力测试与结论\make_ppt.py' '.\Member6_压力测试与结论\results\verified_offline'
```

如需只运行一类实验，`run_member6.py --mode replay --output 新目录` 或 `--mode model --output 新目录`；两类完整运行请优先使用 `run_offline.py`。如果中断，保留部分结果并换新的输出目录重新运行。若已有完整两个结果子目录，可以使用 `--summarize-only`。

## 3. 参数及评分口径

公用场景参数表初版建议 0.1 s 并在门限附近以0.05 s复核；公共正式实现采用物理/运动采样0.05 s，决策0.10 s，Member3/4也使用这一口径。E/F/L/R/Bk 参考中心 x=0/50/−20/−50/−60 m；速度25/20/33/27/27 m/s。车道宽3.6 m、路肩3.0 m。二维车辆5×1.8 m，CARLA用实际包围盒/航向。Bk在2 s后以−5 m/s²制动到零，外部交通脚本不响应自车。故障0 s，等待上限5 s，启动余量0.3 s，预计接管2 s+0.5 s缓冲，停车目标−4 m/s²，单段横移3.5 s；纵横规划裕度2.0/0.2 m，速度阈值0.2 m/s、稳定0.5 s、停后观察3 s，最大观察25 s。以公共实现为准，所有工况与 Member3/4 的配对条件一致。

方向净空为空=无适用样本，并非0或无限安全；净空与扣除规划裕度后的候选分数不同；全程观察时长不同，不能直接作为等时对比。历史 CARLA 碰撞来自传感器，二维来自包围盒；即使分支一致也不能将精确时刻/净空混写。X1 是组合反例而非单因素对照。

## 4. 输出文件

| 文件（相对本次结果目录） | 用途 |
|---|---|
| `formal_replay/paired_results.csv` | 历史 CARLA 16条的正式配对指标，PPT主数据来源 |
| `model_local/paired_results.csv` | 新二维16条指标（独立复跑） |
| `model_local/raw_runs/*.json` | 新二维原始帧、状态、事件及评分 |
| 两个子目录的 `pair_deltas.csv`、`candidate_audit.csv`、`initial_wait_forecast.csv`、`decision_events.json` | 同后端配对变化、候选审计、假设等待预测及真实事件 |
| 两个子目录的 `boundary_comparison.png` | 后向净空、横向轨迹、截止时刻变化、全程各方向净空 |
| 两个子目录的 `X1_failure_trace.json`、`X1_failure_trajectory.png` | 未验证后备的完整失败轨迹及图 |
| `model_local/P4_C_review.csv` | P4-C历史 vs 独立二维复跑 |
| `backend_comparison.csv` | 16条跨后端分支及数值差异（不混合计算同工况净空） |
| `member34_crosscheck.csv` | 与 Member3、4 已有结果逐项核对，共同指标允许1e-6舍入误差 |
| `deadline_audit.csv` | 起点、截止发生变化、后备启动或无后备时刻的独立重算；历史逐帧一致性由公共 replay 检查 |
| `experiment_manifest.json` | 完整工况和参数 |
| `offline_verification.json` | 核验计数及差异 |
| 两个子目录的 `verification.json`、`本次实验事实表.md` | 单批来源与哈希、自动生成的可读事实表 |
| `第9-10页_压力测试与结论.pptx` | 生成两页幻灯片及讲者备注 |

## 5. 验收与结论

本次实际执行：16条正式历史日志精确复算通过；16条新二维轨迹完成；16/16终点/碰撞对象/首帧动作/后备类型/停车区域分支一致；384/384 与前两位组员共同指标核对一致；38/38关键截止重算一致。复算与二维的数值不完全相同，属于后端差异。参考 `实验结论.md` 与 `结论模板.md`，本地再运行后以实际输出为准。PPT沿用前面成员以正式CARLA历史数据为主、二维为复核的口径，不能宣称本地新跑CARLA或完整Bench2Drive。

AI辅助说明：代码核查、脚本封装、结果复算与图表/演示页生成使用ChatGPT辅助；历史CARLA轨迹来自小组公共交接包。成员本人需复核本地输出、合稿中填写真实姓名、进行计时串讲（本人实际完成情况据实记录）。
