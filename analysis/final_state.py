import hashlib, json, subprocess, sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
KPY = '/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3'
KC = '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'


def export(pcb, netclasses, metadata):
    pcb = Path(pcb).resolve(); out = pcb.parent
    sha = hashlib.sha256(pcb.read_bytes()).hexdigest()
    bj, rj, dj = [out / p for p in ('board_finished.json','routes_finished.json','drc_finished.json')]
    for p in (bj,rj,dj): p.unlink(missing_ok=True)
    for script, goal, extra in (('export_board.py',bj,[]),('routes_export.py',rj,['entire=1'])):
        subprocess.run([KPY,str(ROOT/'kicad'/script),str(pcb),str(goal)]+extra,check=True)
    subprocess.run([KC,'pcb','drc','--format','json','--severity-all','-o',str(dj),str(pcb)],check=True)
    if hashlib.sha256(pcb.read_bytes()).hexdigest() != sha: raise ValueError('Final board changed during check')
    json.loads(bj.read_text()); R=json.loads(rj.read_text()); meta=json.load(open(metadata))
    for k in ('options','gnd_layers','area_nets'):
        if k in meta: R[k]=meta[k]

    K=json.load(open(netclasses)); gnd=set(K.get('gnd') or [])
    prior={(v['net'],round(v['x'],5),round(v['y'],5)):v for v in meta.get('vias',[])}
    for v in R['vias']:
        av=prior.get((v['net'],round(v['x'],5),round(v['y'],5)),{})
        for k in ('rv','gr'):
            if av.get(k): v[k]=True
        if v['net'] in gnd: v['rv']=True

    from shapely.geometry import LineString
    from shapely.ops import unary_union
    sig={}
    for t in meta.get('tracks',[]):
        if t.get('sig'): sig.setdefault((t['net'],t['layer']),[]).append(LineString(t['pts']))
    sig={k:unary_union(gs).buffer(0.000002) for k,gs in sig.items()}
    for t in R['tracks']:
        g=sig.get((t['net'],t['layer']))
        if g is not None and g.covers(LineString(t['pts'])): t['sig']=True
    json.dump(R,open(rj,'w'),indent=1)
    from analysis.variants import drc_numbers
    faults,open_items,loose=drc_numbers(dj)
    if faults==99999: raise ValueError('Final DRC incomplete')
    provenance=dict(circuit_board=str(pcb),sha256=sha,drc_sha256=hashlib.sha256(dj.read_bytes()).hexdigest(),
                    board=str(bj),routes=str(rj),drc=dict(faults=faults,open_items=open_items,loose=loose),
                    arc_chord_error_mm=0.00025,copper_map='final tracks/vias/pads and filled zones')
    json.dump(provenance,open(out/'final_state.json','w'),indent=1)
    return provenance


if __name__=='__main__':
    sys.path.insert(0,str(ROOT))
    print(json.dumps(export(*sys.argv[1:4]),indent=1))
