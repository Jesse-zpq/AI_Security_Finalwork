import fs from 'node:fs/promises';
import path from 'node:path';
import { createRequire } from 'node:module';
import { pathToFileURL } from 'node:url';

const dependencyRoot = process.env.CARLA_SHEET_DEPENDENCIES;
const requireBundled = createRequire(dependencyRoot ? path.resolve(dependencyRoot, 'resolver.cjs') : import.meta.url);
const { FileBlob, SpreadsheetFile } = await import(pathToFileURL(requireBundled.resolve('@oai/artifact-tool')).href);

const [input, output] = process.argv.slice(2);
const workbook = await SpreadsheetFile.importXlsx(await FileBlob.load(input));
const originalName = '公用参数';
const newName = 'CARLA 共用参数';

async function render(sheetName, range, filename) {
  const preview = await workbook.render({ sheetName, range, scale: 1, format: 'png' });
  await fs.writeFile(path.join('work', filename), new Uint8Array(await preview.arrayBuffer()));
}

if (!output) {
  const snapshot = await workbook.inspect({ kind: 'sheet', include: 'id,name', maxChars: 3000 });
  console.log(snapshot.ndjson);
  await render(originalName, 'A1:F17', 'carla_params_source.png');
  process.exit(0);
}

const rows = [
  ['环境', '仿真器版本', 'CARLA 1.0', '—', '试跑配置', 'Windows 安装包及对应 Python 客户端。'],
  ['环境', '交通车辆蓝图', 'vehicle.tesla.model3', '—', '试跑配置', '五辆车共用。更换车型后复测包围盒与控制响应。'],
  ['几何', '车辆包围盒长度', 4.694, 'm', 'CARLA 实测', '以包围盒中心及当前航向计算净空。'],
  ['几何', '车辆包围盒宽度', 2.088, 'm', 'CARLA 实测', '以包围盒中心及当前航向计算净空。'],
  ['几何', 'OpenDRIVE 道路总长', 1200, 'm', '试跑配置', '三条同向车道及右肩的宽度沿用「公用参数」。'],
  ['几何', '本车起点道路里程 s', 100, 'm', '试跑配置', '课程坐标以各次本车首帧包围盒中心为纵向原点，向前、向左为正。'],
  ['时间', '物理步长', 0.05, 's', '试跑配置', 'CARLA 同步步进，位置、速度和碰撞按仿真帧记录。'],
  ['时间', '决策更新周期', 0.10, 's', '试跑配置', '每两个物理步更新一次决策。故障时钟与首帧对齐。'],
  ['初值', '纵向及横向位置容差', 0.25, 'm', '验收设定', '每车、每轴分别检查实际首帧与目标初值之差的绝对值。'],
  ['初值', '速度容差', 0.30, 'm/s', '验收设定', '每车分别检查实际首帧与目标初值之差的绝对值。'],
  ['初值', '航向容差', 1, '°', '验收设定', '参考道路行驶方向，逐车检查首帧绝对航向误差。'],
  ['接管', '稳定移交确认时长', 0.50, 's', '试跑判定', '以下三项及净空检查同时满足后开始计时。'],
  ['接管', '距中道中心距离上限', 0.30, 'm', '试跑判定', '稳定移交时要求绝对横向偏差小于此值。'],
  ['接管', '航向偏差上限', 0.05, 'rad', '试跑判定', '稳定移交时要求绝对航向偏差小于此值。'],
  ['接管', '移交最低行驶车速', 1.0, 'm/s', '试跑判定', '稳定移交时要求车速大于此值。'],
  ['停车', '停稳车速门限', 0.20, 'm/s', '试跑判定', '低于此值后开始累计停稳时长。'],
  ['停车', '连续低速确认时长', 0.50, 's', '试跑判定', '连续低速达到此长后，按「公用参数」继续观察 3.00 s。'],
  ['跟驰', '目标静态净空', 5, 'm', '试跑控制', '试跑目标净空等于该值加车速乘以下列目标车头时距。'],
  ['跟驰', '目标车头时距', 1.4, 's', '试跑控制', '目标净空按本车速度计算。'],
  ['跟驰', '速度反馈增益', 0.60, 's⁻¹', '试跑控制', '乘以目标车速与实测车速之差。'],
  ['跟驰', '净空反馈增益', 0.30, 's⁻²', '试跑控制', '实际净空低于目标净空时参与参考加速度计算。'],
  ['预测', '同道后车预测响应延迟', 2.00, 's', '试跑模型', '尚未观测到后车制动时，相对本车候选制动开始计时。'],
  ['预测', '同道后车预测制动加速度', -5.00, 'm/s²', '试跑模型', '预测模型取值。交通真值脚本按仿真时钟在第 2 秒制动。'],
  ['校准', '本车首帧实测速度', 25.012, 'm/s', '四组试跑', '目标 25 m/s，用于核对初始化。'],
  ['校准', '后备停车启动时车速', 21.70, 'm/s', 'P2-C 实测', '原车道停车在 4.70 s 启动。'],
  ['校准', '从启动停车至首次低速的距离', 53.27, 'm', 'P2-C 实测', '从 4.70 s 到 9.85 s，终点为首次低于 0.20 m/s。'],
  ['校准', '停车段平均纵向加速度', -4.18, 'm/s²', 'P2-C 复算', '用 4.70–9.85 s 首末实测车速计算，停车目标为 −4 m/s²。'],
  ['校准', '左移片段最大横向跟踪偏差', 0.236, 'm', 'P2-A 复算', '碰撞前的短片段，供核查。两段右移的跟踪偏差待测。'],
];

const sheet = workbook.worksheets.add(newName);
const end = 3 + rows.length;
sheet.showGridLines = false;
sheet.tabColor = '#2563A5';
sheet.getRange('A1:F1').merge();
sheet.getRange('A2:F2').merge();
sheet.getRange('A1').values = [['高速公路左侧感知退化｜CARLA 共用参数']];
sheet.getRange('A2').values = [['适用于 CARLA 1.0 最小试跑。正式实验沿用「公用参数」的场景初值与 25 s 观察上限。校准行供复核。']];
sheet.getRange('A3:F3').values = [['类别','参数','数值或配置','单位','来源类型','用途与边界']];
sheet.getRange(`A4:F${end}`).values = rows;

const body = sheet.getRange(`A1:F${end + 2}`);
body.format.font.name = 'Microsoft YaHei';
body.format.font.size = 10;
body.format.font.color = '#172A3A';
body.format.verticalAlignment = 'center';
const widths = [76, 262, 166, 64, 113, 525];
widths.forEach((width, index) => {
  sheet.getRangeByIndexes(0, index, end + 2, 1).format.columnWidthPx = width;
});
sheet.getRange('A1:F1').format.font.color = '#163956';
sheet.getRange('A1:F1').format.font.bold = true;
sheet.getRange('A1:F1').format.font.size = 16;
sheet.getRange('A1:F1').format.rowHeightPx = 42;
sheet.getRange('A2:F2').format.font.color = '#486075';
sheet.getRange('A2:F2').format.rowHeightPx = 30;
sheet.getRange('A3:F3').format.fill = '#245578';
sheet.getRange('A3:F3').format.font.color = '#FFFFFF';
sheet.getRange('A3:F3').format.font.bold = true;
sheet.getRange('A3:F3').format.horizontalAlignment = 'center';
sheet.getRange('A3:F3').format.rowHeightPx = 29;
sheet.getRange(`A4:F${end}`).format.rowHeightPx = 27;
sheet.getRange(`D4:E${end}`).format.horizontalAlignment = 'center';
for (const [index, row] of rows.entries()) {
  const line = index + 4;
  if (index % 2) sheet.getRange(`A${line}:F${line}`).format.fill = '#F1F6FA';
  if (index === 0 || row[0] !== rows[index - 1][0]) {
    sheet.getRange(`A${line}:F${line}`).format.borders = { top: { style: 'thin', color: '#CBDBE8' } };
  }
  sheet.getRange(`A${line}`).format.font.bold = true;
  if (typeof row[2] === 'number') {
    const number = sheet.getRange(`C${line}`);
    number.format.horizontalAlignment = 'right';
    const pattern = [4.694, 2.088, 25.012, 0.236].includes(row[2]) ? '0.000'
      : [1200, 100, 1, 5].includes(row[2]) ? '0' : '0.00';
    number.setNumberFormat(pattern);
    if (!['几何', '校准'].includes(row[0])) number.format.font.color = '#1766AB';
  }
  if (row[0] === '校准') sheet.getRange(`E${line}`).format.fill = '#E7F1EF';
}
sheet.getRange(`A${end + 2}:F${end + 2}`).merge();
sheet.getRange(`A${end + 2}`).values = [['依据为 carla_pilot.py、carla_preflight.py 与 CARLA_pilot_results 的逐帧记录。']];
sheet.getRange(`A${end + 2}:F${end + 2}`).format.rowHeightPx = 29;
sheet.getRange(`A${end + 2}:F${end + 2}`).format.font.italic = true;
sheet.freezePanes.freezeRows(3);

workbook.recalculate();
console.log((await workbook.inspect({ kind: 'region', sheetId: newName, range: 'A3:F11', maxChars: 2300, tableMaxRows: 9 })).ndjson);
await render(newName, 'A1:F17', 'carla_params_top.png');
await render(newName, `A18:F${end + 2}`, 'carla_params_bottom.png');
const xlsx = await SpreadsheetFile.exportXlsx(workbook);
await xlsx.save(output);
