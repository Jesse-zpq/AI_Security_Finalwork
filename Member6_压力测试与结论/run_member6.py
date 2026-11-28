"""Reuse Member5's unchanged simulator and score Member6 pressure tests."""
import argparse,csv,hashlib,json,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
H=HERE.parent/'CARLA_Member3-4-5_Handoff'/'CARLA_repro_handoff'
sys.path.insert(0,str(H))
from replay import load_case,reproduce,frame_input
from carla_preflight import make_runs
from carla_pilot import Policy,PARAMS,CORRIDOR,DECISION_DT,Plan,planned_state,follow_acceleration,check_plan
from carla_metrics import score_run
STRESS=[f'T{i}-{s}' for i in (1,2,3) for s in 'ABC']+['X1-C']
REFS=[f'P{i}-{s}' for i in (2,4) for s in 'ABC']; ALL=REFS+STRESS
MANIFEST={c['run_id']:c for c in make_runs()}
def event(r,n): return next((e for e in r['events'] if e['event']==n),{})
def dump(p,o): p.write_text(json.dumps(o,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def csvout(p,rows):
    with p.open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
def row(r,source):
    fb=event(r,'fallback_started'); st=event(r,'observed_stop'); fail=event(r,'no_verified_fallback'); hit=r['score']['first_collision'] or {}
    d=dict(run_id=r['run_id'],source=source,backend=r['backend'],pair_with=r['case']['pair_with'],outcome=r['outcome'],initial_action=r['frames'][0]['action'],collision=r['score']['collision'],collision_actor=hit.get('other'),collision_s=hit.get('time_s'),request_s=event(r,'takeover_requested').get('time_s'),handover_s=event(r,'scripted_handover').get('time_s'),initial_deadline_s=event(r,'deadline_updated').get('deadline_s'),final_deadline_s=next((e['deadline_s'] for e in reversed(r['events']) if e['event']=='deadline_updated'),None),fallback_s=fb.get('time_s'),fallback_action=fb.get('action'),fallback_rejected=json.dumps(fb.get('candidates_rejected',{})),unverified_s=fail.get('time_s'),unverified_reasons=json.dumps(fail.get('candidates',{})),shoulder_replan_s=event(r,'shoulder_replan').get('time_s'),parked_region=st.get('parked_region'),stop_threshold_s=st.get('first_below_threshold_s'),terminal_s=r['score']['terminal_time_s'])
    for win,prefix in [('shared_window_s','first2s'),('until_endpoint','full')]:
        d[prefix+'_observed_until_s']=r['score'][win]['observed_until_s']; d[prefix+'_outside_corridor_s']=r['score'][win]['first_outside_corridor_s']
        for m in ['front_gap_m','rear_gap_m','left_gap_m','right_gap_m','longitudinal_ttc_s','ego_acceleration_mps2']:
            v=r['score'][win][m] or {}; d[prefix+'_'+m]=v.get('value'); d[prefix+'_'+m+'_actor']=v.get('actor'); d[prefix+'_'+m+'_at_s']=v.get('time_s')
    return d

def check(r,k):
    if r['case']!=MANIFEST[k] or r['run_id']!=k: raise ValueError(k+' manifest mismatch')
    if score_run(r,PARAMS['longitudinal_clearance_margin_m'],PARAMS['lateral_clearance_margin_m'],CORRIDOR)!=r['score']: raise ValueError(k+' score mismatch')
    if any('L' in f['observed'] for f in r['frames'] if 'L' in f['states'] and f['states']['L']['y']>1.8): raise ValueError(k+' hidden actor visible')
    if k.startswith('T1') and any('L' in f['states'] for f in r['frames']): raise ValueError(k+' L not removed')
def verify_design():
    changes={'T1':{'L':None},'T2':{'R':{'x_m':-10.0}},'T3':{'Bk':{'x_m':-20.0}}}; ignore={'run_id','pair_with','actor_overrides'}
    for k in STRESS[:-1]:
        c=MANIFEST[k]; ref=MANIFEST[c['pair_with']]; assert {i:v for i,v in c.items() if i not in ignore}=={i:v for i,v in ref.items() if i not in ignore}; assert c['actor_overrides']==changes[k[:2]] and ref['actor_overrides']=={}
    x=MANIFEST['X1-C']; assert x['expand_left_prior_to_middle'] and x['actor_overrides']=={'F':{'x_m':10.0},'R':{'x_m':-10.0},'L':{'cut_in_at_s':1.0,'cut_in_duration_s':3.5}}
def audit(r,source):
    p=Policy(r['case']); end=event(r,'fallback_started') or event(r,'no_verified_fallback'); limit=end.get('time_s',r['frames'][-1]['time_s']); rows=[]
    for f in r['frames'][::round(DECISION_DT/r['physics_step_s'])]:
        t=f['time_s']
        if t>limit+1e-8: break
        ego,obs,a=frame_input(r,t); opts,rej=p.stop_options(ego,t,t,obs,a); scores={q.name:s for s,q in opts}
        for act in ('in_lane_stop','shoulder_stop'):
            reason=rej.get(act,{}); rows.append(dict(run_id=r['run_id'],source=source,time_s=t,observed='/'.join(obs),actual_action=f['action'],deadline_s=f['deadline_s'],candidate=act,available=act in scores,score_m=scores.get(act),blocker=reason.get('object'),predicted_conflict_s=reason.get('time_s')))
    return rows

def forecast(r,source):
    p=Policy(r['case']); ego,obs,a=frame_input(r,0); wait=Plan('wait',0,ego,min(0,follow_acceleration(ego,obs))); rows=[]; continuous=True
    for i in range(round(PARAMS['max_wait_s']/DECISION_DT)+1):
        t=round(i*DECISION_DT,8); _,conflict=check_plan(wait,0,t,obs,a,p.blind_prior); opts,rej=p.stop_options(planned_state(wait,t),t,0,obs,a); scores={q.name:s for s,q in opts}; continuous=continuous and not conflict and bool(opts)
        rows.append(dict(run_id=r['run_id'],source=source,hypothetical_launch_s=t,continuous_wait_and_backup_valid=continuous,in_lane_available='in_lane_stop' in scores,shoulder_available='shoulder_stop' in scores,in_lane_blocker=rej.get('in_lane_stop',{}).get('object'),shoulder_blocker=rej.get('shoulder_stop',{}).get('object'),wait_conflict=json.dumps(conflict)))
    return rows

def analyze(records,sources,old,out,local):
    rows=[row(records[k],sources[k]) for k in ALL]; csvout(out/'paired_results.csv',rows); by={r['run_id']:r for r in rows}; pairs=[]
    for k in STRESS:
        a,b=by[MANIFEST[k]['pair_with']],by[k]; d=dict(run_id=k,reference=a['run_id'],reference_source=a['source'],stress_source=b['source'],reference_outcome=a['outcome'],stress_outcome=b['outcome'],reference_collision=a['collision'],stress_collision=b['collision'],reference_fallback=a['fallback_action'],stress_fallback=b['fallback_action'])
        for m in ['first2s_front_gap_m','first2s_rear_gap_m','first2s_left_gap_m','first2s_right_gap_m','initial_deadline_s','fallback_s']:
            x,y=a[m],b[m]; d['ref_'+m]=x; d['stress_'+m]=y; d['delta_'+m]=None if x is None or y is None else y-x
        pairs.append(d)
    csvout(out/'pair_deltas.csv',pairs)
    ids=('P2-C','P4-C','T1-C','T2-C','T3-C','X1-C'); csvout(out/'candidate_audit.csv',[v for k in ids for v in audit(records[k],sources[k])]); csvout(out/'initial_wait_forecast.csv',[v for k in ids for v in forecast(records[k],sources[k])]); dump(out/'X1_failure_trace.json',records['X1-C']); dump(out/'decision_events.json',{k:records[k]['events'] for k in ALL})
    a,b=row(old['P4-C'],'archive'),by['P4-C']; fields=['outcome','collision','initial_action','initial_deadline_s','final_deadline_s','fallback_s','fallback_action','parked_region','terminal_s','first2s_rear_gap_m','full_front_gap_m','full_rear_gap_m','full_left_gap_m','full_right_gap_m']; csvout(out/'P4_C_review.csv',[dict(metric=k,archived=a[k],current=b[k],current_source=b['source'],delta=b[k]-a[k] if type(a[k]) in (int,float) and type(b[k]) in (int,float) else None,equal=a[k]==b[k]) for k in fields])
    from plot_results import plot
    plot(records,sources,out)
    fmt=lambda x:'—' if x is None else f'{x:.3f}' if isinstance(x,float) else str(x)
    lines=['# 本次实验事实表（自动生成）','',f'本地轨迹数量：{len(local)}；archive_replay 是既有 CARLA 日志复算，不是新仿真。','', '| 工况 | 来源 | 终点 | 碰撞对象/时刻 | 初始/最终截止(s) | 后备启动(s)/动作 | 前2s后向净空(m) |','|---|---|---|---|---|---|---|']
    for r in rows: lines.append('| '+' | '.join(map(fmt,[r['run_id'],r['source'],r['outcome'],fmt(r['collision_actor'])+'/'+fmt(r['collision_s']),fmt(r['initial_deadline_s'])+'/'+fmt(r['final_deadline_s']),fmt(r['fallback_s'])+'/'+fmt(r['fallback_action']),r['first2s_rear_gap_m']]))+' |')
    lines+=['','X1 无可验证后备：'+fmt(by['X1-C']['unverified_s'])+' s；原因：`'+by['X1-C']['unverified_reasons']+'`。','', '结合 paired_results.csv 的全程四方向净空及观察时长填写结论模板。无碰撞不等于安全证明；空白不是零。']; (out/'本次实验事实表.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')

def main():
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--mode',choices=['replay','model','carla','analyze'],default='replay'); p.add_argument('--output',type=Path,required=True); p.add_argument('--input',type=Path); p.add_argument('--paired',action='store_true'); p.add_argument('--formal-output',type=Path,help='Also write the verified historical comparison to this new directory'); p.add_argument('--host',default='127.0.0.1'); p.add_argument('--port',type=int,default=2200); args=p.parse_args(); verify_design(); out=args.output.resolve(); out.mkdir(parents=True,exist_ok=False); old={}; sources={}; hashes={}
    for k in ALL:
        r=load_case(k); check(r,k); reproduce(r); old[k]=r; sources[k]='archive_replay'; path=H/('CARLA_批跑前验证' if k=='X1-C' else 'CARLA_formal_results')/(k+'.json'); hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest(); print(k,'archive exact replay OK',flush=True)
    records=old.copy(); local=[]
    if args.mode!='replay':
        if args.mode=='analyze':
            if args.input is None: p.error('--input required')
            raw=args.input.resolve(); local=[k for k in ALL if (raw/(k+'.json')).exists()]
        else:
            raw=out/'raw_runs'; local=ALL if args.paired or args.mode=='model' else STRESS+['P4-C']; subprocess.run([sys.executable,'-B',str(H/'member5_baselines.py'),'--mode',args.mode,'--cases',*local,'--host',args.host,'--port',str(args.port),'--output',str(raw)],check=True)
        if not set(STRESS+['P4-C'])<=set(local): raise ValueError('Need 10 stress runs and P4-C')
        for k in local:
            path=raw/(k+'.json'); r=json.loads(path.read_text(encoding='utf-8')); check(r,k); records[k]=r; sources[k]='local_'+r['backend']; hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
        if len({records[k]['backend'] for k in local})!=1: raise ValueError('Mixed local backends')
        if records['P4-C']['backend']=='model' and set(local)!=set(ALL): raise ValueError('Model comparison requires all 16 cases')
    for path in list((H/'core').glob('*.py'))+[H/'replay.py',H/'member5_baselines.py',Path(__file__)]: hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    dump(out/'verification.json',dict(mode=args.mode,archived_exact_replays=ALL,local_ids=local,independent_review_new_or_imported='P4-C' in local,sources=sources,parameters=PARAMS,sha256=hashes)); analyze(records,sources,old,out,local)
    if args.formal_output:
        formal=args.formal_output.resolve(); formal.mkdir(parents=True,exist_ok=False); archived_sources={k:'archive_replay' for k in ALL}; analyze(old,archived_sources,old,formal,[])
        dump(formal/'verification.json',dict(mode='replay',archived_exact_replays=ALL,local_ids=[],sources=archived_sources,parameters=PARAMS,sha256={k:v for k,v in hashes.items() if str(out) not in k}))
    print('Complete:',out)
if __name__=='__main__': main()
