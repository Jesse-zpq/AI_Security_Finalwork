import fs from 'node:fs/promises';
import path from 'node:path';
import { createRequire } from 'node:module';
import { pathToFileURL } from 'node:url';

const runtime = process.env.CARLA_SHEET_DEPENDENCIES;
const bundled = createRequire(runtime ? path.resolve(runtime, 'resolver.cjs') : import.meta.url);
const { FileBlob, SpreadsheetFile } = await import(pathToFileURL(bundled.resolve('@oai/artifact-tool')).href);
const [source, target] = process.argv.slice(2);
const workbook = await SpreadsheetFile.importXlsx(await FileBlob.load(source));
const name = 'CARLA 共用参数';
const sheet = workbook.worksheets.getItem(name);
const validationPath = path.join(process.env.CARLA_LOG_ROOT ?? 'outputs', 'CARLA_批跑前验证', 'P4-B.json');
const validation = JSON.parse(await fs.readFile(validationPath, 'utf8'));
const metric = validation.score.until_endpoint;
const stopped = validation.frames.find((frame) => frame.states.E.v < 0.2);
const first = validation.frames[0];
const mean = (stopped.states.E.v - first.states.E.v) / (stopped.time_s - first.time_s);
const round = (number, digits) => Number(number.toFixed(digits));

const rows = [
  ['制动', '共用控制器踏板比例', validation.brake_pedal_scale_mps2, 'm/s²', '正式配置', '每单位归一化制动指令的换算比例。A/B/C 共用。'],
  ['制动', '策略 B 目标纵向加速度', -4, 'm/s²', '正式配置', '故障帧开始在中道制动。指令目标与实测响应分别记录。'],
  ['校准', 'B 参考组平均纵向加速度', round(mean, 3), 'm/s²', 'P4-B 实测', '从故障帧至首次低于 0.20 m/s，用首末速度差计算。'],
  ['校准', 'B 参考组最小采样加速度', round(metric.ego_acceleration_mps2.value, 3), 'm/s²', 'P4-B 实测', '用相邻物理帧的速度差计算。'],
  ['校准', 'B 参考组至首次低速距离', round(stopped.states.E.x - first.states.E.x, 2), 'm', 'P4-B 实测', '终点为首次低于 0.20 m/s。'],
  ['评分', '统一观察上限', validation.observation_limit_s, 's', '正式配置', '碰撞、稳定移交或停车观察完成时提前终止，记录实际时长。'],
  ['评分', '配对统计窗口上限', 2, 's', '评分口径', '同时汇报窗口内和终点前指标。窗口遇提前终止时按实际时长报告。'],
  ['评分', '前后净空横向间隙上限', 0.20, 'm', '评分口径', '横向投影间隙不超过此值时，分别统计前后最小纵向间隙。'],
  ['评分', '左右净空纵向间隙上限', 2.00, 'm', '评分口径', '纵向间隙不超过此值、且横向中心差满足下列门限时统计。'],
  ['评分', '左右净空横向中心差下限', 0.50, 'm', '评分口径', '按目标位于本车左侧或右侧分别统计。'],
  ['评分', '计算纵向 TTC 的接近速度下限', 0.05, 'm/s', '评分口径', '相对接近速度须大于此值，且前后净空适用、纵向间隙为正。'],
  ['评分', '碰撞与越界记录', '首个事件', '—', '评分口径', '碰撞取本车 CARLA 传感器；越界用投影车身与三车道及右肩边界。'],
  ['批次', '故障对照运行数', 21, '条', '正式清单', '含 12 条主对照和 9 条单项压力工况。'],
  ['批次', '正常观测参照运行数', 2, '条', '正式清单', 'N1-A 与 N2-A。X1-C 组合反例保留在预验证记录。'],
];

const start = 37;
const end = start + rows.length - 1;
sheet.getRange('A2').values = [['CARLA 1.0 试跑参数与正式批次设置。正式制动校准和共用评分口径见下方新增区块。']];
sheet.getRange('A35:F35').merge();
sheet.getRange('A35').values = [['正式批次｜制动校准与评分']];
sheet.getRange('A36:F36').copyFrom(sheet.getRange('A3:F3'), 'all');
sheet.getRange('A36:F36').values = [['类别','参数','数值或配置','单位','来源类型','用途与边界']];
sheet.getRange(`A${start}:F${end}`).values = rows;

const title = sheet.getRange('A35:F35');
title.format.font.name = 'Microsoft YaHei';
title.format.font.size = 14;
title.format.font.bold = true;
title.format.font.color = '#163956';
title.format.verticalAlignment = 'center';
title.format.rowHeightPx = 36;
sheet.getRange('A36:F36').format.rowHeightPx = 29;
const body = sheet.getRange(`A${start}:F${end}`);
body.format.font.name = 'Microsoft YaHei';
body.format.font.size = 10;
body.format.font.color = '#172A3A';
body.format.verticalAlignment = 'center';
body.format.rowHeightPx = 29;
sheet.getRange(`D${start}:E${end}`).format.horizontalAlignment = 'center';
rows.forEach((entry, index) => {
  const row = start + index;
  const band = sheet.getRange(`A${row}:F${row}`);
  if (index % 2) band.format.fill = '#F1F6FA';
  if (index === 0 || rows[index - 1][0] !== entry[0]) {
    band.format.borders = { top: { style: 'thin', color: '#CBDBE8' } };
  }
  sheet.getRange(`A${row}`).format.font.bold = true;
  const cell = sheet.getRange(`C${row}`);
  if (typeof entry[2] === 'number') {
    cell.format.horizontalAlignment = 'right';
    cell.setNumberFormat([2, 3].includes(index) ? '0.000' : Number.isInteger(entry[2]) ? '0' : '0.00');
    if (entry[0] !== '校准') cell.format.font.color = '#1766AB';
  }
  if (entry[0] === '校准') sheet.getRange(`E${row}`).format.fill = '#E7F1EF';
});
workbook.recalculate();
const view = await workbook.render({ sheetName: name, range: `A35:F${end}`, scale: 1, format: 'png' });
await fs.writeFile('work/carla_formal_params_preview.png', new Uint8Array(await view.arrayBuffer()));
const blob = await SpreadsheetFile.exportXlsx(workbook);
await blob.save(target);
