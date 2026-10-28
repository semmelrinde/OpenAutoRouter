""
import copy
import math
from pathlib import Path

from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

ROOT = Path(__file__).resolve().parents[1]


def vorbereiten(B, R, K, regeln, folder):
    from lernen.stromzonen import sperrflaeche, _lagen, _via_masse
    from lernen.kupferbalken import track_geometry
    from gemeinsam.verschieben import bewegen
    original_src = B.get('aufgabe_ursprung') or B.get('src', '')
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
            if k.get('ref')==ref or part['side'] not in k.get('lagen', _lagen(B)): continue
            if not (k.get('verbote') or {}).get('teile'): continue
            geo = unary_union([Polygon(q, (k.get('loecher') or [[]]*len(k['polys']))[i]) for i,q in enumerate(k['polys'])])
            if own.intersection(geo).area>1e-7: blocked.append(ref); break
    for ref in blocked:
        if not original_src.startswith('aufgabe seed='):
            raise ValueError('Native Sperrflaeche erfordert Placement-Reparatur: '+ref)
        if footprints[ref].get('locked') or (K.get('teile',{}).get(ref) or {}).get('fest'):
            raise ValueError('Festes Teil im nativen Sperrbereich: '+ref)
        own = courtyards[ref]
        foreign = unary_union([g for rr,g in courtyards.items() if rr!=ref and footprints[rr]['side']==footprints[ref]['side']]).buffer(.006)
        keep = unary_union([sperrflaeche(dict(B,keepouts=[k]), footprints[ref]['side'], 'teile') for k in B.get('keepouts',[]) if k.get('ref')!=ref])
        from shapely.affinity import translate
        candidates=sorted((math.hypot(x*.2,y*.2),x*.2,y*.2) for x in range(-40,41) for y in range(-40,41) if x or y)
        for _,dx,dy in candidates:
            g=translate(own,dx,dy)
            if outline.covers(g) and not g.intersects(foreign) and not g.intersects(keep):
                B,_=bewegen(B,{ref:(dx,dy,0)});moved[ref]=[dx,dy,0];courtyards[ref]=g;break
        else: raise ValueError('Kein legaler Platz nahe '+ref)
        affected.update(p['net'] for p in B['pads'] if p['ref']==ref and p['net'])
        reasons.append(dict(art='Bauteil im nativen Sperrbereich',ref=ref,versatz=moved[ref]))
    options=R.get('optionen') or {}; rg=(regeln or {}).get('router') or {}
    clr=max(float(q.get(k) or 0) for q,k in ((R,'clr'),(options,'clr'),(options,'drc'),(rg,'clr'),(rg,'drc')))
    hole=max(float(options.get('h2h') or .2),float(rg.get('h2h') or .2))
    holes=[(p['net'],Point(p['x'],p['y']).buffer(p['drill']/2,quad_segs=32)) for p in B['pads'] if p.get('drill')]
    shapes=[(t,track_geometry(t)) for t in R['tracks']]
    for t,g in shapes:
        bad = g.intersection(sperrflaeche(B,t['layer'],'bahnen')).area>1e-8
        bad |= any(g.distance(h)<hole-.0001 for n,h in holes if n!=t['net'])
        if bad: affected.add(t['net']);reasons.append(dict(art='Bahn Sperre/Bohrung',net=t['net'],layer=t['layer']))

    from shapely.strtree import STRtree
    for layer in _lagen(B):
        ts=[(t,g) for t,g in shapes if t['layer']==layer];tree=STRtree([g for t,g in ts])
        for i,(t,g) in enumerate(ts):
            for j in tree.query(g.buffer(clr)):
                if j<=i or t['net']==ts[j][0]['net']:continue
                if g.distance(ts[j][1])<clr-.0001:

                    pair=[t['net'],ts[j][0]['net']]
                    power=[n for n in pair if K['netze'].get(n,{}).get('art') in ('strom','versorgung','sw')]
                    n=min(power or pair,key=lambda n:K['netze'].get(n,{}).get('strom_A') or 0)
                    affected.add(n);reasons.append(dict(art='Bahnabstand',net=n,layer=layer))
    vd,_=_via_masse(R)
    for v in R['vias']:
        disk=Point(v['x'],v['y']).buffer(v.get('d',v.get('diameter',vd))/2)
        if any(disk.intersection(sperrflaeche(B,L,'vias')).area>1e-8 for L in v.get('layers',_lagen(B))):
            affected.add(v['net']);reasons.append(dict(art='Via im Sperrbereich',net=v['net']))
    affected.discard('')
    if not affected and not moved:return B,R,dict(angewendet=False,gruende=[])
    if not original_src.startswith('aufgabe seed='):
        raise ValueError('Geruestreparatur ist nur fuer synthetische Aufgaben freigegeben')

    if moved:
        B['src']=original_src
        from lernen.kupfer_nativ import bibliotheksgeometrie
        B,_=bibliotheksgeometrie(folder/'bibliothek',B,R,regeln)
    from lernen.stromzonen_lokal import reparieren
    R,local=reparieren(B,R,oldpads,moved,clr,hole)
    reasons += [dict(art='Lokaler legaler Korridor',**q) for q in local]
    R['native_reparatur']=dict(angewendet=True,netze=sorted(affected),versetzt=moved,gruende=reasons)
    return B,R,R['native_reparatur']
