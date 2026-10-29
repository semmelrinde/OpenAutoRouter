import json
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from analysis.evaluation import rate
from analysis.topology import topology

TOPO=('parallel_rest_mm','star','sharp','kelvin_close_mm','tap_close_mm',
      'tap_via_in_pad','kelvin_beside_current_mm')


def check(B,R0,R1,K,D0,D1):
    E0,E1=[rate(B,R,K,v3=True) for R in (R0,R1)]
    T0,T1=[topology(R,B) for R in (R0,R1)]
    def drc(d):
        return dict(faults=sum(v['severity']=='error' for v in d['violations']),open_items=len(d['unconnected_items']))
    C0,C1=drc(D0),drc(D1)
    reasons=[]
    if C1['faults']>C0['faults'] or C1['open_items']>C0['open_items']:
        reasons.append('KiCad DRC worse')
    if E0['is_valid'] and not E1['is_valid']:
        reasons.append('JSON connection gate worse')
    if E1['score_v3']>E0['score_v3']+1e-9:
        reasons.append('Score v3 worse')
    for key in TOPO:
        if T1[key]>T0[key]+1e-9:
            reasons.append('Topology worse: '+key)

    for n in sorted(set(E0['measures']['net_list'])|set(E1['measures']['net_list'])):
        for key in ('xt_victim','xt_disturb'):
            a=(E0['measures']['net_list'].get(n) or {}).get('m',{}).get(key) or 0.
            b=(E1['measures']['net_list'].get(n) or {}).get('m',{}).get(key) or 0.
            if b>a+1e-9:
                reasons.append('Coupling worse: '+n+' '+key)
    return dict(accept=not reasons,reasons=reasons,prior=dict(drc=C0,evaluation=E0,topology=T0),
                new=dict(drc=C1,evaluation=E1,topology=T1),
                limit='Score/topology/coupling use the track scaffold, not area edges; DC separately in native copper.')


def main():
    b,r0,r1,k,d0,d1,out=sys.argv[1:8]
    B,R0,R1,K,D0,D1=[json.loads(Path(f).read_text()) for f in (b,r0,r1,k,d0,d1)]
    result=check(B,R0,R1,K,D0,D1)
    Path(out).write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
    print('CurrentAreas-Gate:', 'accept' if result['accept'] else 'reject',
          'Score v3',result['prior']['evaluation']['score_v3'],'->',result['new']['evaluation']['score_v3'],
          '; '.join(result['reasons'][:4]))
    return 0 if result['accept'] else 1


if __name__=='__main__':sys.exit(main())
