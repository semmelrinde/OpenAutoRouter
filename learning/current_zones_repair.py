import copy
import math
from pathlib import Path

from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

ROOT = Path(__file__).resolve().parents[1]


def prepare(B, R, K, design_rules, folder):
    from learning.current_zones import keepout_area, _layers, _via_ground
    from learning.copper_bars import track_geometry
    from common.shift import relocate
    original_src = B.get('task_origin') or B.get('src', '')
    folder = Path(folder); folder.mkdir(parents=True, exist_ok=True)
    B, R = copy.deepcopy(B), copy.deepcopy(R)
    oldpads=copy.deepcopy(B['pads'])
    moved = {}; affected = set(); reasons = []
    outline = unary_union([Polygon(q) for q in B['outline']]).buffer(-.2)

    footprints = {f['ref']:f for f in B.get('fps', [])}
    courtyards = {c['ref']:unary_union([Polygon(q) for q in c['polys']]) for c in B.get('courtyards', [])}
    blocked = []
    for ref, part in sorted(footprints.items()):
        own = courtyards.get(ref)
        if own is None: continue
        for k in B.get('keepouts', []):
            if k.get('ref')==ref or part['side'] not in k.get('cu_layers', _layers(B)): continue
            if not (k.get('prohibitions') or {}).get('part_list'): continue
            geo = unary_union([Polygon(q, (k.get('hole_list') or [[]]*len(k['polys']))[i]) for i,q in enumerate(k['polys'])])
            if own.intersection(geo).area>1e-7: blocked.append(ref); break
    for ref in blocked:
        if not original_src.startswith('task seed='):
            raise ValueError('Native keepout area requires placement repair: '+ref)
        if footprints[ref].get('locked') or (K.get('part_list',{}).get(ref) or {}).get('pinned'):
            raise ValueError('Fixed part in native keepout area: '+ref)
        own = courtyards[ref]
        foreign = unary_union([g for rr,g in courtyards.items() if rr!=ref and footprints[rr]['side']==footprints[ref]['side']]).buffer(.006)
        keep = unary_union([keepout_area(dict(B,keepouts=[k]), footprints[ref]['side'], 'part_list') for k in B.get('keepouts',[]) if k.get('ref')!=ref])
        from shapely.affinity import translate
        candidates=sorted((math.hypot(x*.2,y*.2),x*.2,y*.2) for x in range(-40,41) for y in range(-40,41) if x or y)
        for _,dx,dy in candidates:
            g=translate(own,dx,dy)
            if outline.covers(g) and not g.intersects(foreign) and not g.intersects(keep):
                B,_=relocate(B,{ref:(dx,dy,0)});moved[ref]=[dx,dy,0];courtyards[ref]=g;break
        else: raise ValueError('No legal slot near '+ref)
        affected.update(p['net'] for p in B['pads'] if p['ref']==ref and p['net'])
        reasons.append(dict(category='Component in native keepout area',ref=ref,offset=moved[ref]))
    options=R.get('options') or {}; rg=(design_rules or {}).get('router') or {}
    clr=max(float(q.get(k) or 0) for q,k in ((R,'clr'),(options,'clr'),(options,'drc'),(rg,'clr'),(rg,'drc')))
    hole=max(float(options.get('h2h') or .2),float(rg.get('h2h') or .2))
    holes=[(p['net'],Point(p['x'],p['y']).buffer(p['drill']/2,quad_segs=32)) for p in B['pads'] if p.get('drill')]
    shapes=[(t,track_geometry(t)) for t in R['tracks']]
    for t,g in shapes:
        bad = g.intersection(keepout_area(B,t['layer'],'track_list')).area>1e-8
        bad |= any(g.distance(h)<hole-.0001 for n,h in holes if n!=t['net'])
        if bad: affected.add(t['net']);reasons.append(dict(category='Track keepout/drill',net=t['net'],layer=t['layer']))

    from shapely.strtree import STRtree
    for layer in _layers(B):
        ts=[(t,g) for t,g in shapes if t['layer']==layer];tree=STRtree([g for t,g in ts])
        for i,(t,g) in enumerate(ts):
            for j in tree.query(g.buffer(clr)):
                if j<=i or t['net']==ts[j][0]['net']:continue
                if g.distance(ts[j][1])<clr-.0001:

                    pair=[t['net'],ts[j][0]['net']]
                    power=[n for n in pair if K['net_list'].get(n,{}).get('category') in ('current_val','supply','sw')]
                    n=min(power or pair,key=lambda n:K['net_list'].get(n,{}).get('current_A') or 0)
                    affected.add(n);reasons.append(dict(category='TrackSpacing',net=n,layer=layer))
    vd,_=_via_ground(R)
    for v in R['vias']:
        disk=Point(v['x'],v['y']).buffer(v.get('d',v.get('diameter',vd))/2)
        if any(disk.intersection(keepout_area(B,L,'vias')).area>1e-8 for L in v.get('layers',_layers(B))):
            affected.add(v['net']);reasons.append(dict(category='Via in keepout area',net=v['net']))
    affected.discard('')
    if not affected and not moved:return B,R,dict(applied=False,reasons=[])
    if not original_src.startswith('task seed='):
        raise ValueError('Scaffold repair is released only for synthetic tasks')

    if moved:
        B['src']=original_src
        from learning.copper_kicad_native import library_geometry
        B,_=library_geometry(folder/'library',B,R,design_rules)
    from learning.current_zones_local import repair
    R,local=repair(B,R,oldpads,moved,clr,hole)
    reasons += [dict(category='Local legal corridor',**q) for q in local]
    R['native_repair']=dict(applied=True,net_list=sorted(affected),offset=moved,reasons=reasons)
    return B,R,R['native_repair']
