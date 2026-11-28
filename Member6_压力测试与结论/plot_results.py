import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from carla_metrics import box_projection
from carla_pilot import PARAMS

def rear_series(r):
    times=[]; gaps=[]
    for f in r['frames']:
        e=f['states']['E']; other=[]
        for key,s in f['states'].items():
            if key=='E': continue
            dx,dy,gx,gy=box_projection(e,s)
            if dx<0 and gy<=PARAMS['lateral_clearance_margin_m']: other.append(gx)
        times.append(f['time_s']); gaps.append(min(other) if other else float('nan'))
    return times,gaps

def plot(records,sources,out):
    fig,ax=plt.subplots(2,2,figsize=(12,8),layout='constrained'); colors={'A':'#64748b','B':'#c97724','C':'#197b79'}
    for s in 'ABC':
        k='T3-'+s; r=records[k]; t,g=rear_series(r); ax[0,0].plot([x for x in t if x<=5],[v for x,v in zip(t,g) if x<=5],label=k,color=colors[s]); ax[0,1].plot(t,[f['states']['E']['y'] for f in r['frames']],label=k,color=colors[s]); hit=r['score']['first_collision']
        if hit: ax[0,0].axvline(hit['time_s'],color=colors[s],ls=':')
    ax[0,0].axhline(2,ls='--',color='gray',label='2 m planning margin'); ax[0,0].set(xlim=(0,5),xlabel='Time (s)',ylabel='Rear gap (m)',title='T3: rear clearance'); ax[0,0].legend(fontsize=8)
    for y in [3.6,0,-3.6,-6.9]: ax[0,1].axhline(y,ls=':',color='gray',alpha=.6)
    ax[0,1].set(xlabel='Time (s)',ylabel='Lateral y (m)',title='T3: actual lateral trajectories'); ax[0,1].legend()
    for k,style in [('P4-C','-'),('T2-C','--'),('T3-C','-.')]:
        r=records[k]; e=next((e for e in r['events'] if e['event']=='fallback_started'),None); f=[v for v in r['frames'] if v['deadline_s'] is not None and (not e or v['time_s']<=e['time_s']+1e-8)]
        if f: ax[1,0].step([v['time_s'] for v in f],[v['deadline_s'] for v in f],where='post',ls=style,label=k)
        if e and e.get('deadline_s') is not None: ax[1,0].scatter(e['time_s'],e['deadline_s'],marker='x',s=55)
    ax[1,0].plot([0,5],[0,5],':',color='gray',label='Current time'); ax[1,0].set(xlim=(-.1,5.1),ylim=(-.1,5.1),xlabel='Time (s)',ylabel='Absolute deadline (s)',title='C: rolling deadline (cross = launch time)'); ax[1,0].legend(fontsize=8)
    groups=['P4','T2','T3']; width=.18
    for j,m in enumerate(['front','rear','left','right']):
        for i,g in enumerate(groups):
            v=records[g+'-C']['score']['until_endpoint'][m+'_gap_m']; x=i+(j-1.5)*width
            if v: ax[1,1].bar(x,v['value'],width,label=m if i==0 else None,color=['#4477aa','#ee9944','#228833','#aa3377'][j])
            else: ax[1,1].text(x,0,'NA',ha='center',fontsize=7)
    ax[1,1].set(xticks=range(3),xticklabels=groups,ylabel='Minimum gap (m)',title='C: full-run gaps (unequal durations)'); ax[1,1].legend(fontsize=8)
    fig.suptitle('Member 6 | Boundary comparison | '+sources['T3-C']); fig.savefig(out/'boundary_comparison.png',dpi=200); plt.close(fig)
    r=records['X1-C']; fig,ax=plt.subplots(2,1,figsize=(10,7),layout='constrained')
    for key in ('E','F','L','R','Bk'):
        fs=[f for f in r['frames'] if key in f['states']]; ax[0].plot([f['time_s'] for f in fs],[f['states'][key]['y'] for f in fs],label=key); ax[1].plot([f['time_s'] for f in fs],[f['states'][key]['x']-f['states']['E']['x'] for f in fs],label=key)
    for a in ax:
        for e in r['events']:
            if e['event']=='no_verified_fallback': a.axvline(e['time_s'],color='red',ls='--',label='No verified fallback'); break
        a.legend(ncol=3,fontsize=8); a.set_xlabel('Time (s)')
    ax[0].set(ylabel='Lateral y (m)',title='Actual lateral positions'); ax[1].set(ylabel='Relative longitudinal center x (m)',title='Relative longitudinal centers (not bumper gaps)'); fig.suptitle('X1-C | '+r['outcome']+' | '+sources['X1-C']); fig.savefig(out/'X1_failure_trajectory.png',dpi=200); plt.close(fig)
