"""Generate two editable slides from a complete run_offline output folder."""
import argparse
import csv
import json
from pathlib import Path
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE, MSO_CONNECTOR
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.util import Inches, Pt

BG='FBFAF6'; DARK='132C37'; TEAL='226B71'; GRAY='39515A'; MUTED='63777D'; LIGHT='DCE7E3'; WHITE='FFFFFF'; RED='AD583D'; PALE='F4E4D9'; GOLD='BE8650'; LINE='C4D1CE'
FONT='Microsoft YaHei'
def rgb(h): return RGBColor.from_string(h)
def txt(s,x,y,w,h,t,size=16,color=DARK,bold=False,align=PP_ALIGN.LEFT):
    sh=s.shapes.add_textbox(Inches(x),Inches(y),Inches(w),Inches(h)); tf=sh.text_frame; tf.clear(); tf.word_wrap=True
    tf.margin_left=tf.margin_right=Inches(0); tf.margin_top=tf.margin_bottom=Inches(0); tf.vertical_anchor=MSO_ANCHOR.MIDDLE
    p=tf.paragraphs[0]; p.alignment=align; p.space_after=Pt(0); r=p.add_run(); r.text=str(t); r.font.name=FONT; r.font.size=Pt(size); r.font.bold=bold; r.font.color.rgb=rgb(color)
    return sh

def rect(s,x,y,w,h,fill,stroke=None):
    sh=s.shapes.add_shape(MSO_SHAPE.RECTANGLE,Inches(x),Inches(y),Inches(w),Inches(h)); sh.fill.solid(); sh.fill.fore_color.rgb=rgb(fill)
    if stroke: sh.line.color.rgb=rgb(stroke); sh.line.width=Pt(.7)
    else: sh.line.fill.background()
    return sh

def line(s,x1,y1,x2,y2,c=LINE,width=1):
    sh=s.shapes.add_connector(MSO_CONNECTOR.STRAIGHT,Inches(x1),Inches(y1),Inches(x2),Inches(y2)); sh.line.color.rgb=rgb(c); sh.line.width=Pt(width); return sh

def base(prs,num,section,title,sub):
    s=prs.slides.add_slide(prs.slide_layouts[6]); s.background.fill.solid(); s.background.fill.fore_color.rgb=rgb(BG)
    txt(s,.68,.34,11.1,.34,section,13.5,TEAL,True); txt(s,12.04,.34,.58,.34,num,15,TEAL,True,PP_ALIGN.RIGHT)
    line(s,.68,.79,12.65,.79,LINE,.8)
    txt(s,.68,.95,12,.70,title,30,DARK,True); txt(s,.68,1.73,12,.46,sub,16,GRAY)
    return s

def read(path):
    with path.open(encoding='utf-8-sig') as f: return {r['run_id']:r for r in csv.DictReader(f)}
def f(r,k,n=2): return '—' if r[k]=='' else f'{float(r[k]):.{n}f}'
def note(s,t): s.notes_slide.notes_text_frame.text=t

def build(root, output):
    root=Path(root); formal=read(root/'formal_replay'/'paired_results.csv'); model=read(root/'model_local'/'paired_results.csv')
    prs=Presentation(); prs.slide_width=Inches(13.333333); prs.slide_height=Inches(7.5)
    a,b,c=[formal['T3-'+x] for x in 'ABC']; p,t2=formal['P4-C'],formal['T2-C']; mc=model['T3-C']; mx=model['X1-C']; x=formal['X1-C']
    names={'stopped_middle':'中道停车','stopped_shoulder':'路肩停车','observation_limit':'继续跟驰','collision':'追尾','unverified_stop_middle':'未验证中道停车'}
    s=base(prs,'09','实验六  /  压力测试与结论','后车逼近：立即制动并非风险最小','T3 对照 P4（跟驰、无人接管）：仅将同道后车 Bk 的初始中心位置由 −60 m 改为 −20 m。')
    txt(s,.68,2.30,6.2,.35,'T3：共同前 2 s 最小后向净空 / m',18,TEAL,True)
    # editable bar chart; consistent scaling with real data
    colors=[MUTED,RED,TEAL]; scale=max(float(r['first2s_rear_gap_m']) for r in [a,b,c])*1.1
    for i,(r,col) in enumerate(zip([a,b,c],colors)):
        y=2.88+i*.70; txt(s,.72,y,.46,.36,r['run_id'][-1],19,DARK,True); rect(s,1.25,y+.07,3.35,.25,LIGHT); rect(s,1.25,y+.07,3.35*float(r['first2s_rear_gap_m'])/scale,.25,col)
        txt(s,4.76,y,.69,.36,f(r,'first2s_rear_gap_m'),17,col,True,PP_ALIGN.RIGHT); txt(s,5.61,y,1.40,.36,(names.get(r['outcome'],r['outcome']) if r['outcome']!='collision' else f(r,'collision_s')+' s 追尾'),14,col,True)
    txt(s,.72,5.05,6.25,.72,f"C 全程：前向 {f(c,'full_front_gap_m')} m，后向 {f(c,'full_rear_gap_m')} m；\n左向 {f(c,'full_left_gap_m')} m，右向 {f(c,'full_right_gap_m')}（无适用样本）",14,GRAY)
    line(s,7.14,2.35,7.14,5.80,LINE,.9)
    txt(s,7.42,2.30,5.20,.35,'C：截止时刻与实际启动 / s',18,TEAL,True)
    # comparison matrix
    widths=[1.03,1.18,1.33,1.57]; starts=[7.42,8.45,9.63,10.96]; total=sum(widths)
    rect(s,7.42,2.84,total,.42,TEAL)
    for xx,ww,tt in zip(starts,widths,['工况','初始截止','启动时刻','停车位置']): txt(s,xx+.07,2.84,ww-.14,.42,tt,13,WHITE,True,PP_ALIGN.CENTER)
    for i,r in enumerate([p,t2,c]):
        y=3.26+i*.62; rect(s,7.42,y,total,.62,WHITE if i%2==0 else LIGHT)
        for j,(xx,ww,tt) in enumerate(zip(starts,widths,[r['run_id'][:2],f(r,'initial_deadline_s'),f(r,'fallback_s'),names.get(r['outcome'],r['outcome'])])): txt(s,xx+.06,y,ww-.12,.62,tt,15,TEAL if i==2 else DARK,i==2,PP_ALIGN.CENTER)
    txt(s,7.42,5.23,5.13,.68,f"T3 截止 {f(c,'initial_deadline_s')} → {f(c,'final_deadline_s')} s；二维 {f(mc,'final_deadline_s')} s。\nP4-C 独立二维复跑 {f(model['P4-C'],'fallback_s')} s 启动，分支一致。",13.5,GRAY)
    rect(s,.68,6.10,11.97,.65,LIGHT); txt(s,.88,6.10,11.60,.65,'T1 有保守代价；T2 窗口不足立即停车；T3 后向风险改变了 C 的停车选择。',16,TEAL,True)
    txt(s,.68,6.98,11.98,.24,'来源：仓库正式 CARLA 日志复算；二维 ModelWorld 独立复跑。净空按车身投影计，完整观察时长因终点而异。',10.5,MUTED)
    note(s,f"【约55秒】前三类压力测试每类三条。T1移除真实左车，C仍取消左移，有保守代价；T2右间隙不足，C不等待，立即中道停车。重点看T3，同道后车初始中心距从60米缩至20米，其他输入保持不变。前两秒，A、B、C后向净空分别是{f(a,'first2s_rear_gap_m')}、{f(b,'first2s_rear_gap_m')}、{f(c,'first2s_rear_gap_m')}米。B在{f(b,'collision_s')}秒被后车追尾；C的截止从{f(c,'initial_deadline_s')}秒收缩到{f(c,'final_deadline_s')}秒，启动两段靠边停车。二维重跑在{f(mc,'fallback_s')}秒启动，决策分支一致。P4-C独立复跑也是中道停车。不过C的全程左向净空仍需检查，右向无适用样本不是零，更不能由此证明横向绝对安全。")
    reasons=json.loads(x['unverified_reasons']); left_t=reasons['in_lane_stop']['time_s']; right_t=reasons['shoulder_stop']['time_s']; r2=json.loads(mx['unverified_reasons'])
    s=base(prs,'10','实验六  /  组合反例与小组结论','无可验证后备时，应识别能力边界','X1：前车 F +10 m、右后车 R −10 m、左车 1 s 开始切入；规划先验扩展至中道。')
    txt(s,.68,2.29,5.55,.35,'X1-C：0 s 判定两种后备均不可验证',17.5,TEAL,True)
    for i,(name,reason,when) in enumerate([('中道停车','左侧未知占用',left_t),('靠边停车','前车 F 限制',right_t)]):
        y=2.82+i*.67; rect(s,.68,y,5.54,.53,PALE); txt(s,.82,y,1.49,.53,name,16,RED,True); txt(s,2.30,y,2.14,.53,reason,15,DARK); txt(s,4.52,y,1.54,.53,f'预测 {when:.2f} s',14,RED,True,PP_ALIGN.RIGHT)
    rect(s,.68,4.20,5.54,.90,LIGHT); txt(s,.86,4.22,5.15,.83,f"进入紧急制动分支；{f(x,'terminal_s')} s 结束观察。\n无记录碰撞，但停车结果未通过安全验证。",15,TEAL,True)
    txt(s,.72,5.20,5.50,.44,f"二维复跑：同样 0 s 判定无后备；F 预测冲突 {r2['shoulder_stop']['time_s']:.2f} s。",13,GRAY)
    line(s,6.48,2.35,6.48,5.78,LINE,.9)
    txt(s,6.76,2.29,5.80,.35,'本组判断与适用范围',18,TEAL,True)
    for y,hd,detail in [(2.85,'01  动作筛选','缺失观测 ≠ 道路无车；允许动作须有证据。'),(3.66,'02  接管窗口','以可启动的停车后备滚动限制等待，而非固定倒计时。'),(4.47,'03  风险边界','不能验证后备时退出正常等待；不承诺全工况安全。')]:
        txt(s,6.77,y,5.64,.30,hd,16,DARK,True); txt(s,6.77,y+.31,5.70,.43,detail,14,GRAY)
    txt(s,.68,5.91,11.97,.32,'组内分工（按汇报方案角色）',14,TEAL,True)
    roles=['1 场景与运动参数','2 基准与观测接口','3 动作筛选与停车后备','4 接管窗口与策略切换','5 基线与主对照','6 压力测试与结论']
    for i,role in enumerate(roles):
        xx=.68+(i%3)*4.0; yy=6.32+(i//3)*.30; txt(s,xx,yy,3.94,.28,role,12.5,GRAY,i==5)
    txt(s,.68,7.04,11.97,.23,'来源：X1 既有 CARLA 反例与新二维复跑；结论限于确定性脚本交通、给定盲区包络及有限观察期。',10.5,MUTED)
    note(s,f"【约45秒】最后看组合反例X1。前车靠近、右侧间隙不足，左侧又可能切入。系统在零秒判断没有可验证后备：中道停车受左侧未知占用限制，预计{left_t:.2f}秒违反判据；靠边候选先受前车限制，预计{right_t:.2f}秒违反判据。这些是预测时刻，不是实际碰撞时刻。系统进入未覆盖的紧急制动分支，虽然实际没有碰撞，但仍不能把它算成验证安全的停车。二维复跑同样得到该分支。本组的结论不是C永远最优，而是用可用证据筛选动作、依据后备可行性调整等待，并在证据不足时明确报告未覆盖风险。以上是组内分工。")
    prs.save(output)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('result_dir',type=Path); p.add_argument('--output',type=Path); args=p.parse_args(); build(args.result_dir,args.output or args.result_dir/'第9-10页_压力测试与结论.pptx')
