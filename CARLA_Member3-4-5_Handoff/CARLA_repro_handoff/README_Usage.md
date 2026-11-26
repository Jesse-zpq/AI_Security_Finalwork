# CARLA 主对照复现交接

解压后在本目录打开终端。以下命令使用 `python`，也可以换成你电脑上对应的 Python 可执行文件。日志复算和二维试跑使用 Python 3.10 或以上版本及标准库。实跑需要 CARLA 1.0 和与服务端版本匹配的 Python 客户端。

三个入口都调用 `core/` 中与正式批次逐字节一致的源代码。`CARLA_formal_results/` 包含 23 条正式轨迹和汇总，`CARLA_批跑前验证/X1-C.json` 是独立的组合反例。Excel 文件提供共同参数和 21 条故障对照的人工可读结果。

| 模式 | 输入 | 输出和核查用途 |
| --- | --- | --- |
| 日志复算 | 保存的 CARLA 帧及碰撞记录 | 逐帧重新运行原观测和策略逻辑，核对动作、事件、终点和评分。 |
| `model` | 原工况配置 | 用原二维后端重新生成轨迹，核查配置和程序流程。 |
| `carla` | 原工况配置及独占 CARLA 服务端 | 重新生成物理轨迹，比较碰撞对象、终点及决策时序。 |

## 成员 3 · 动作筛选与停车后备

```powershell
python member3_action_filter.py
python member3_action_filter.py --cases N1-A P2-A --at 0
python member3_action_filter.py --cases T3-C --at 2.6
```

入口复用原函数 `check_plan` 与 `Policy.stop_options`，显示当时的观测车辆、候选得分、首个否决对象与预测时刻。默认核对初始帧和实际启动停车的决策帧。`--at` 接受日志内每 0.10 s 一个的决策帧。`verified_events` 来自重新运行的原策略。候选得分为米，记录的是原判据使用的裕度分数。

可先看 N1-A 与 P2-A，验证 L 可见时 A 拒绝左移，故障遮蔽后 A 放行。随后看 T2-C 的右后车限制、T3-C 的两段靠边和 X1-C 的未验证后备。

## 成员 4 · 接管窗口与策略切换

```powershell
python member4_takeover.py
python member4_takeover.py --cases P1-C P2-C
python member4_takeover.py --cases T2-C T3-C
```

入口在正式日志的逐帧状态上重新运行 `Policy.decide` 和原移交确认逻辑。输出 `time_s` 表示事件发生时刻，`deadline_s` 表示该次计算得到的截止时刻。`driver_ready` 是脚本就绪，`scripted_handover` 是稳定移交。`fallback_started` 标记实际启动停车后备。

参考交通中，P1-C 在 2.00 s 就绪、2.50 s 确认移交，P2-C 在 4.70 s 启动中道停车。T3-C 的截止从 4.20 s 提前到 2.60 s，可用来对照滚动更新的原因字段。

## 成员 5 · 基线与主对照

```powershell
# 默认检查两条正常参照、12 条主对照和 T3-B 压力工况
python member5_baselines.py

# 用正式日志复算完整批次
python member5_baselines.py --all

# 用二维模型重跑选定工况，写到新目录
python member5_baselines.py --mode model --cases N1-A P2-A P4-B --output replay_model
```

CARLA 实跑先在专用终端启动本机 CARLA 1.0 服务端。Windows 发布包的启动示例如下，`CARLA_ROOT` 指向解压目录。复现入口会在指定端口加载自建地图，使用独立服务端可以避免替换其他实验的地图。

```powershell
$env:CARLA_ROOT = 'D:\CARLA\Carla-1.0-Win64-Shipping'
$serverData = (New-Item -ItemType Directory -Path .\carla_server_data -Force).FullName
$game = Join-Path $env:CARLA_ROOT 'CarlaUnreal\Binaries\Win64\CarlaUnreal-Win64-Shipping.exe'
Set-Location -LiteralPath (Split-Path -Parent $game)
& $game -carla-rpc-port=2200 -carla-streaming-port=2201 -nullrhi -nosound "-UserDir=$serverData"
```

在解压目录的另一个终端运行。使用可 import `carla` 的 Python，输出目录用新名称，程序会拒绝覆盖所选工况的旧记录。

```powershell
python member5_baselines.py --mode carla --cases N1-A P2-A P4-B T3-B T3-C --port 2200 --output replay_carla
python member5_baselines.py --mode carla --all --port 2200 --output replay_formal
```

比对新旧 `summary.json` 和单组日志时，优先看 `outcome`、`first_collision`、`events`、`shared_window_s` 和 `until_endpoint`。重跑 CARLA 时帧编号和逐帧物理量可能变化。原论文与本组数据中的基线参数和局限见随包正式参数表。
