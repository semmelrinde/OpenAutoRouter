""
import json
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from lernen import dauerlauf as DL, sammeln as S
from lernen.ebenen import drc
from analyse import korpus as KO


def probe(d, originals, out):
    S.EBENEN.update(an=True, originale=originals); S.V2['an']=True
    info=S.vorbereiten(d,out)
    if not info or not info['gndlagen']: raise RuntimeError('keine belegte GND-Ebene')
    DL._TL.gndlagen=info['gndlagen'];DL._TL.stil='standard';DL._TL.cfg=dict(pfmul=1.25,kurz_via=0,einzeltest=0,kritik=0)
    KO.ROUTER_EXTRA=tuple(KO.ROUTER_EXTRA)+('cpu_max=240',)
    bj=os.path.join(out,'board.json');kj=os.path.join(out,'klassen.json');rj=os.path.join(out,'routes.json')
    rc,cpu=DL.route_lauf_stil(out,bj,kj,rj,40,240,os.path.join(out,'route.log'))
    B=json.load(open(bj));K=json.load(open(kj));R=json.load(open(rj));RG=json.load(open(os.path.join(out,'regeln.json')))
    E=S.regelmessung(B,R,K,info['gndlagen'])
    E.update(gndlagen=info['gndlagen'],cpu_s=cpu,rc=rc,code_sha256=S.code_stand())
    E['drc']=drc(info['pcb'],rj,os.path.join(out,'fertig.kicad_pcb'),info,RG,[],KO)
    json.dump(E,open(os.path.join(out,'ergebnis.json'),'w'),indent=1)
    print(json.dumps(E))


if __name__=='__main__':probe(*sys.argv[1:4])
