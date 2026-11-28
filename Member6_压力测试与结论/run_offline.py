"""Member6 offline end-to-end experiment. Requires no CARLA client/server."""
import argparse
import csv
import json
import math
import subprocess
import sys
from datetime import datetime
from pathlib import Path

from run_member6 import (HERE, H, ALL, STRESS, MANIFEST, csvout, dump, event,
                         Policy, PARAMS, DECISION_DT, Plan, check_plan,
                         planned_state, follow_acceleration, frame_input, load_case)
from carla_preflight import build_manifest


def read_csv(path):
    with path.open(encoding='utf-8-sig', newline='') as f:
        return {r['run_id']: r for r in csv.DictReader(f)}


def equal(a, b, tol=1e-6):
    if a == b:
        return True
    try:
        return math.isclose(float(a), float(b), rel_tol=0, abs_tol=tol)
    except (ValueError, TypeError):
        return False


def compare_previous(out, formal, model):
    rows = []
    for pattern in ('Member3*', 'Member4*'):
        dirs = sorted(p for p in HERE.parent.glob(pattern) if p.is_dir())
        if len(dirs) != 1:
            raise RuntimeError(f'Expected one teammate directory matching {pattern}: {dirs}')
        for fname, current in [('formal_pairs.csv', formal), ('model_pairs.csv', model)]:
            old = read_csv(dirs[0] / 'results' / fname)
            for rid, ref in old.items():
                if rid not in current:
                    continue
                metrics = {k:k for k in ref if k in current[rid] and k.startswith(('first2s_', 'full_'))}
                metrics.update({'outcome':'outcome','fallback_s':'fallback_s','fallback':'fallback_action','end_s':'terminal_s'})
                for a,b in metrics.items():
                    x,y = ref[a],current[rid][b]
                    rows.append(dict(member=dirs[0].name,table=fname,run_id=rid,metric=b,previous=x,current=y,match=equal(x,y)))
    csvout(out/'member34_crosscheck.csv', rows)
    return rows


def audit_deadline(record, source):
    """Re-enumerate wait feasibility at initial, deadline-change and launch frames as Member4 does; previous deadline is the logged preceding frame. Full historical replay checks every frame."""
    case = record['case']; policy = Policy(case)
    stop = event(record, 'fallback_started') or event(record, 'no_verified_fallback')
    requested = event(record, 'takeover_requested').get('time_s')
    previous = None; rows = []
    selected_times={0.0,stop['time_s']} | {e['time_s'] for e in record['events'] if e['event']=='deadline_updated'}
    for f in record['frames'][::round(DECISION_DT/record['physics_step_s'])]:
        now = f['time_s']
        if now > stop['time_s'] + 1e-8:
            break
        if not any(math.isclose(now,t,abs_tol=1e-8) for t in selected_times):
            previous=f['deadline_s']; continue
        ego, obs, a = frame_input(record, now)
        options, rejected = policy.stop_options(ego, now, now, obs, a)
        feasible = now; deadline = None; limiting = rejected
        if options:
            wait = Plan('wait', now, ego, min(0.0, follow_acceleration(ego, obs)))
            limiting = 'wait_cap'
            for tick in range(1, max(0, round((PARAMS['max_wait_s']-now)/DECISION_DT))+1):
                future = now + tick*DECISION_DT
                _, conflict = check_plan(wait, now, future-now, obs, a, policy.blind_prior)
                opts, reasons = policy.stop_options(planned_state(wait, future), future, now, obs, a)
                if conflict or not opts:
                    limiting = conflict or reasons
                    break
                feasible = future
            deadline = max(now, feasible-PARAMS['mrm_launch_margin_s'])
            if requested is not None and requested < now-1e-8:
                deadline = min(previous, deadline)
        match = equal(deadline, f['deadline_s'], 1e-7)
        if not match:
            raise AssertionError(f'{source}/{record["run_id"]} at {now}: deadline {deadline} != {f["deadline_s"]}')
        rows.append(dict(source=source,run_id=record['run_id'],decision_s=now,
                         available_now='/'.join(p.name for _,p in options),
                         feasible_until_s=feasible if options else None,
                         recalculated_deadline_s=deadline,logged_deadline_s=f['deadline_s'],
                         match=match,action=f['action'],limiting=json.dumps(limiting)))
        previous = deadline
    return rows


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,default=None,help='New output directory; default is results/run_<timestamp>.')
    p.add_argument('--summarize-only',action='store_true',help='Reuse complete formal_replay and model_local folders under --output; do not simulate again.')
    args=p.parse_args()
    if args.summarize_only and args.output is None: p.error('--summarize-only needs --output')
    out=(args.output or HERE/'results'/('run_'+datetime.now().strftime('%Y%m%d_%H%M%S'))).resolve()
    if not args.summarize_only:
        out.mkdir(parents=True,exist_ok=False)
        subprocess.run([sys.executable,'-B',str(HERE/'run_member6.py'),'--mode','model','--output',str(out/'model_local'),'--formal-output',str(out/'formal_replay')],check=True)
    # Reject incomplete or mislabelled result folders before aggregation.
    for sub, mode, n in [('formal_replay','replay',0),('model_local',None,len(ALL))]:
        meta=json.loads((out/sub/'verification.json').read_text(encoding='utf-8'))
        if set(meta['archived_exact_replays'])!=set(ALL) or len(meta['local_ids'])!=n or (mode and meta['mode']!=mode):
            raise ValueError(f'Incomplete verification metadata: {sub}')
        if sub=='model_local':
            if set(meta['local_ids'])!=set(ALL) or any(meta['sources'][k]!='local_model' for k in ALL):
                raise ValueError('Expected all new ModelWorld results')
            if any(not (out/sub/'raw_runs'/f'{k}.json').is_file() for k in ALL):
                raise ValueError('Incomplete raw ModelWorld trajectories')
    formal=read_csv(out/'formal_replay'/'paired_results.csv')
    model=read_csv(out/'model_local'/'paired_results.csv')
    if set(formal)!=set(model) or set(formal)!=set(ALL):
        raise AssertionError('Incomplete experiment matrix')
    fields=['outcome','collision','collision_actor','initial_action','fallback_action','parked_region','collision_s','initial_deadline_s','final_deadline_s','fallback_s','terminal_s']
    for win in ('first2s','full'):
        fields.extend(f'{win}_{m}' for m in ('front_gap_m','rear_gap_m','left_gap_m','right_gap_m','longitudinal_ttc_s','ego_acceleration_mps2'))
    comp=[]
    for rid in ALL:
        a,b=formal[rid],model[rid]; row={'run_id':rid,'role':'pressure' if rid in STRESS[:-1] else 'combined_counterexample' if rid=='X1-C' else 'independent_review' if rid=='P4-C' else 'extra_paired_reference'}
        for k in fields:
            row['carla_'+k]=a[k]; row['model_'+k]=b[k]
            if k.endswith(('_s','_m','_mps2')):
                try: row['delta_'+k]=float(b[k])-float(a[k])
                except ValueError: row['delta_'+k]=None
        row['branch_match']=all(a[k]==b[k] for k in ('outcome','collision','collision_actor','initial_action','fallback_action','parked_region'))
        comp.append(row)
    csvout(out/'backend_comparison.csv',comp)
    teammates=compare_previous(out,formal,model)
    audits=[]
    for rid in [r for r in ALL if r.endswith('-C')]:
        r=json.loads((out/'model_local'/'raw_runs'/f'{rid}.json').read_text(encoding='utf-8'))
        audits+=audit_deadline(load_case(rid),'archive_replay')+audit_deadline(r,'local_model')
    csvout(out/'deadline_audit.csv',audits)
    dump(out/'experiment_manifest.json',build_manifest())
    dump(out/'offline_verification.json',dict(formal_exact_replays=len(formal),new_model_runs=len(model),required_pressure_runs=9,combined_counterexample_runs=1,independent_reference_runs=1,additional_paired_reference_runs=5,branch_matches=sum(r['branch_match'] for r in comp),deadline_audit_points=len(audits),deadline_audit_matches=sum(r['match'] for r in audits),teammate_metric_checks=len(teammates),teammate_metric_matches=sum(r['match'] for r in teammates),teammate_mismatches=[r for r in teammates if not r['match']],requires_carla=False,new_runs_use_carla=False))
    from make_ppt import build
    build(out, out/'第9-10页_压力测试与结论.pptx')
    print(f'Complete: {out}')
    print((out/'offline_verification.json').read_text(encoding='utf-8'))

if __name__=='__main__':
    main()
