# Member2 主对照实验代码

本目录汇集左侧感知退化实验使用的仿真、评分、校验、组员复现及表格生成代码。正式运行使用的三个 Python 核心文件保存在 `outputs/`，与原实验代码逐字节一致。目录名沿用原工作区布局，便于运行辅助脚本。

| 位置 | 用途 |
| --- | --- |
| `outputs/carla_pilot.py` | 工况构建、A/B/C 策略、动作与接管判定、二维和 CARLA 执行。 |
| `outputs/carla_preflight.py` | 运行清单、自建 OpenDRIVE 道路、CARLA 版本与地图预检。 |
| `outputs/carla_metrics.py` | 碰撞、前后左右净空、纵向 TTC、减速度和终点评分。 |
| `work/start_pilot_server.ps1` | Windows 独立服务端启动与试跑、验证、正式批次调度。 |
| `work/verify_*.py` | 最小试跑、批跑前验证和正式批次的日志校验。 |
| `work/member1_review/` | 成员 1 的原始二维参照脚本，以及本组用来核查压力工况的复算脚本。 |
| `work/carla_handoff/` | 成员 3、4、5 的复现入口和各自的 [Usage](work/carla_handoff/README_Usage.md)。 |
| `work/*.mjs`、`work/read_handoff.py` | 场景参数表与正式结果表的生成和读取。 |

## 运行

从本目录打开 PowerShell。二维模式使用 Python 3.10 或以上版本及标准库。

```powershell
python -B .\outputs\carla_preflight.py --write-dir .\work\carla_inputs
python -B .\outputs\carla_pilot.py --mode model --cases N1-A P2-A P4-B --output .\work\model_smoke
```

正式批次需要 CARLA 1.0 和匹配 Python 解释器的 CARLA 客户端 wheel。下面的入口启动独立服务端，保存进程日志，运行后关闭该服务端。若 Python 客户端安装在独立目录，可用 `-Python` 与 `-PythonClientPath` 指定。`-Output` 使用新目录名可保留之前的运行。

```powershell
.\work\start_pilot_server.ps1 -CarlaRoot 'D:\CARLA\Carla-1.0-Win64-Shipping' -Suite formal -Port 2200 -Output 'outputs\CARLA_formal_results'
python -B .\work\verify_formal_batch.py .\outputs\CARLA_formal_results
```

`formal` 包含 21 条故障对照和 2 条正常参照。组合反例可用 `-Cases X1-C` 输出到独立目录。若服务端已经启动，可直接调用 `carla_pilot.py --mode carla --host 127.0.0.1 --port 2200 --suite formal --output <新目录>`。

## 原始日志复算

组员可以用课程目录内已有的 CARLA 交接日志。`CARLA_LOG_ROOT` 指向 `CARLA_formal_results` 和 `CARLA_批跑前验证` 的共同上级。

```powershell
$env:CARLA_LOG_ROOT = (Resolve-Path '..\CARLA_Member3-4-5_Handoff\CARLA_repro_handoff').Path
python -B .\work\carla_handoff\member3_action_filter.py
python -B .\work\carla_handoff\member4_takeover.py
python -B .\work\carla_handoff\member5_baselines.py --all
python -B .\work\verify_formal_batch.py (Join-Path $env:CARLA_LOG_ROOT 'CARLA_formal_results')
```

`verify_pilot_results.py <最小试跑目录>` 对应原四条试跑记录。`verify_carla_batch.py <校准后验证目录> <二维正式清单结果目录>` 对应批跑前的两套验证记录。`pilot_server_probe.py <端口> [道路文件]` 用于查 RPC、可用地图或地图生成错误。

## 表格工具

三个 `.mjs` 脚本使用 Node.js 与 `@oai/artifact-tool`。若包由工作区运行时提供，设置 `CARLA_SHEET_DEPENDENCIES` 为包含其 `node_modules` 的目录。表格脚本从项目根目录运行。`read_handoff.py <原始表格目录>` 需要 `openpyxl`。

```powershell
node .\work\update_carla_workbook.mjs <成员1交接表.xlsx> <CARLA试跑参数表.xlsx>
node .\work\sync_formal_workbook.mjs <CARLA试跑参数表.xlsx> <正式参数表.xlsx>
node .\work\build_carla_results_workbook.mjs .\work\21组故障结果.xlsx
```

正式参数表脚本读取原批跑前验证中的 `CARLA_批跑前验证/P4-B.json`。生成结果表时，使用 `CARLA_LOG_ROOT` 下的正式批次；也可使用项目 `outputs/` 下的新运行结果。
