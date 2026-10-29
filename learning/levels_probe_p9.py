import json
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from learning import endurance_run as DL, collect as S
from learning.levels import drc
from analysis import corpus as KO


def probe(d, originals, out):
    S.LEVELS.update(an=True, originals=originals); S.V2['an']=True
    info=S.prepare(d,out)
    if not info or not info['gnd_layers']: raise RuntimeError('no occupied GND layer')
    DL._TL.gnd_layers=info['gnd_layers'];DL._TL.style='standard';DL._TL.cfg=dict(pfmul=1.25,short_via=0,single_test=0,critique=0)
    KO.ROUTER_EXTRA=tuple(KO.ROUTER_EXTRA)+('cpu_max=240',)
    bj=os.path.join(out,'board.json');kj=os.path.join(out,'netclasses.json');rj=os.path.join(out,'routes.json')
    rc,cpu=DL.route_run_style(out,bj,kj,rj,40,240,os.path.join(out,'route.log'))
    B=json.load(open(bj));K=json.load(open(kj));R=json.load(open(rj));RG=json.load(open(os.path.join(out,'design_rules.json')))
    E=S.rule_measurement(B,R,K,info['gnd_layers'])
    E.update(gnd_layers=info['gnd_layers'],cpu_s=cpu,rc=rc,code_sha256=S.code_state())
    E['drc']=drc(info['pcb'],rj,os.path.join(out,'finished.kicad_pcb'),info,RG,[],KO)
    json.dump(E,open(os.path.join(out,'outcome.json'),'w'),indent=1)
    print(json.dumps(E))


if __name__=='__main__':probe(*sys.argv[1:4])
