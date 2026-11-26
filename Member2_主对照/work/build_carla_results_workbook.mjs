import fs from 'node:fs/promises';
import path from 'node:path';
import { createRequire } from 'node:module';
import { pathToFileURL } from 'node:url';

const runtime = process.env.CARLA_SHEET_DEPENDENCIES;
const bundled = createRequire(runtime ? path.resolve(runtime, 'resolver.cjs') : import.meta.url);
const { Workbook, SpreadsheetFile } = await import(pathToFileURL(bundled.resolve('@oai/artifact-tool')).href);
const results = path.join(process.env.CARLA_LOG_ROOT ?? 'outputs', 'CARLA_formal_results');
const summary = JSON.parse(await fs.readFile(path.join(results, 'summary.json'), 'utf8'));

const groupLabels = {
  P1: 'P1｜主对照', P2: 'P2｜主对照', P3: 'P3｜主对照', P4: 'P4｜主对照',
  T1: 'T1｜左侧无实车', T2: 'T2｜右侧间隙不足', T3: 'T3｜同道后车近',
};
const strategies = { A: 'A 常规规划', B: 'B 立即制动', C: 'C 动作筛选与后备' };
const resultLabels = {
  collision: '碰撞', observation_limit: '持续行驶', stopped_middle: '中道停车',
  stopped_shoulder: '路肩停车', scripted_handover: '脚本接管',
};
const notes = {
  'P1-A': '碰撞与脚本就绪信号同为 2.00 s。',
  'P2-A': '与 N1-A 配对。屏蔽观测，左后车实体仍在。',
  'P2-C': '右后车被预测会侵入靠边候选的净空门限。',
  'P3-C': '与 P4-C 配对，交通初值和故障设定相同。',
  'P4-C': '4.70 s 由等待上限扣除启动余量得到。',
  'T1-A': '移除真实左后车 L，左侧故障标志仍在。',
  'T1-C': '盲区实际为空，C 仍因左侧未知拒绝变道。',
  'T2-C': '初始截止 1.90 s，接管所需窗口不足。',
  'T3-B': '前 2 s 最小后向净空约 3.03 m。',
  'T3-C': '截止从 4.20 s 提前至 2.60 s；前 2 s 最小后向净空约 10.91 m。',
};
const time = (number) => number.toFixed(2);

function describe(run) {
  const event = (name) => run.events.find((entry) => entry.event === name);
  const collision = run.score.first_collision;
  const outcome = run.outcome;
  const strategy = run.case.strategy;
  const left = run.case.intent === 'left';
  const end = time(run.score.terminal_time_s);
  if (strategy === 'A') {
    if (collision) {
      return `屏蔽左后车后放行左变道，${time(collision.time_s)} s 与左后车 L 碰撞。`;
    }
    return left ? `左变道完成，之后在左道继续行驶。${end} s 观察期内未记录碰撞。`
      : `保持中道跟驰，到 ${end} s 观察期结束，未记录碰撞。`;
  }
  if (strategy === 'B') {
    if (collision) {
      return `0 s 开始在中道制动，${time(collision.time_s)} s 与同道后车 Bk 碰撞。`;
    }
    return `0 s 开始在中道制动，${end} s 完成停稳观察。`;
  }
  if (outcome === 'scripted_handover') {
    const ready = time(event('driver_ready').time_s);
    return `${left ? '拒绝左变道后保道' : '继续保道'}并请求接管，${ready} s 脚本就绪，${end} s 确认稳定移交。`;
  }
  const start = time(event('fallback_started').time_s);
  if (outcome === 'stopped_shoulder') {
    const second = time(event('shoulder_second_segment').time_s);
    return `${start} s 启动靠边，${second} s 进入第二段，${end} s 完成路肩停稳观察。`;
  }
  if (start === '0.00') {
    return `右侧候选受右后车限制，0 s 启动中道制动，${end} s 完成停稳观察。`;
  }
  return `${left ? '拒绝左变道并保道等待' : '保道等待'}，${start} s 启动中道制动，${end} s 完成停稳观察。`;
}

const ids = summary.runs.filter(({ run_id: id }) => /^[PT]/.test(id)).map(({ run_id: id }) => id);
const rows = [];
for (const id of ids) {
  const run = JSON.parse(await fs.readFile(path.join(results, `${id}.json`), 'utf8'));
  rows.push([
    id, groupLabels[id.split('-')[0]], run.case.intent === 'left' ? '左变道' : '中道跟驰',
    run.case.readiness_event_s === null ? '无人接管' : `${time(run.case.readiness_event_s)} s 脚本就绪`,
    strategies[run.case.strategy], resultLabels[run.outcome], run.score.terminal_time_s, describe(run),
    notes[id] ?? null,
  ]);
}
if (rows.length !== 21 || rows.some((row) => row.some((value) => value === undefined))) {
  throw new Error('The formal batch must provide exactly 21 complete fault cases.');
}

const workbook = Workbook.create();
const sheetName = '21组故障对照';
const sheet = workbook.worksheets.add(sheetName);
const firstRow = 5;
const lastRow = firstRow + rows.length - 1;
const used = sheet.getRange(`A1:I${lastRow}`);
used.format.font.name = 'Microsoft YaHei';
used.format.font.size = 10;
used.format.font.color = '#182B3A';
used.format.verticalAlignment = 'center';
used.format.rowHeightPx = 27;
sheet.showGridLines = false;
sheet.freezePanes.freezeRows(4);
sheet.tabColor = '#245578';
const widths = [70, 149, 87, 124, 153, 88, 95, 397, 330];
widths.forEach((width, col) => {
  sheet.getRangeByIndexes(0, col, lastRow, 1).format.columnWidthPx = width;
});

sheet.getRange('A1:I1').merge();
sheet.getRange('A2:I2').merge();
sheet.getRange('A3:I3').merge();
sheet.getRange('A1').values = [['CARLA 正式批次｜21 条故障对照']];
sheet.getRange('A2').values = [['每个工况按 A、B、C 配对。持续行驶表示到达 25 s 观察上限。停车时刻包括停稳确认和停车后的观察期。']];
sheet.getRange('A3').values = [['终点与过程来自正式逐帧日志。就绪事件由脚本注入，A/B 按既定基线继续控制，C 按接管窗口处理。']];
const title = sheet.getRange('A1:I1');
title.format.font.size = 16;
title.format.font.bold = true;
title.format.font.color = '#163956';
title.format.rowHeightPx = 41;
sheet.getRange('A2:I3').format.font.color = '#456477';

const header = sheet.getRange('A4:I4');
header.values = [['运行编号', '分组', '原计划', '接管条件', '策略', '仿真结果', '终点时刻（s）', '仿真过程简述', '备注']];
header.format.fill = '#245578';
header.format.font.bold = true;
header.format.font.color = '#FFFFFF';
header.format.rowHeightPx = 32;
header.format.horizontalAlignment = 'center';
sheet.getRange(`A${firstRow}:I${lastRow}`).values = rows;
sheet.getRange(`A${firstRow}:I${lastRow}`).format.rowHeightPx = 42;
sheet.getRange(`C${firstRow}:G${lastRow}`).format.horizontalAlignment = 'center';
sheet.getRange(`G${firstRow}:G${lastRow}`).setNumberFormat('0.00');
sheet.getRange(`H${firstRow}:I${lastRow}`).format.wrapText = true;

const fillFor = { 碰撞: '#FCE4E2', 持续行驶: '#EEF2F6', 中道停车: '#E8F1FA', 路肩停车: '#E3F2EF', 脚本接管: '#E8F3E3' };
for (const [index, rowData] of rows.entries()) {
  const rowIndex = firstRow + index;
  const band = sheet.getRange(`A${rowIndex}:I${rowIndex}`);
  if (Math.floor(index / 3) % 2) band.format.fill = '#F3F7FB';
  if (index % 3 === 0) band.format.borders = { top: { style: 'thin', color: '#CDDBE8' } };
  sheet.getRange(`A${rowIndex}`).format.font.bold = true;
  const result = sheet.getRange(`F${rowIndex}`);
  result.format.fill = fillFor[rowData[5]];
  result.format.font.bold = true;
  if (rowData[5] === '碰撞') result.format.font.color = '#982E28';
}

workbook.recalculate();
for (const [label, range] of [['overview', 'A1:I14'], ['pressure', `A15:I${lastRow}`]]) {
  const rendered = await workbook.render({ sheetName, range, scale: 1, format: 'png' });
  await fs.writeFile(path.join('work', `results_${label}.png`), new Uint8Array(await rendered.arrayBuffer()));
}
const exported = await SpreadsheetFile.exportXlsx(workbook);
await exported.save(process.argv[2]);
