""
import copy
import math
import time
from collections import Counter
from pathlib import Path

from shapely.geometry import Point, Polygon, LineString
from shapely.ops import unary_union

from analyse.bewertung import bewerte
from analyse import regelkennzahlen as RK
from nachbearbeitung.zugregeln import annahme

VERSION = 'restkonflikte-v2-option'


def individuelle_vias(R):
    ""
    d, drill = map(float, (R.get('optionen') or {}).get('via', '0.45/0.20').split('/'))
    for v in R.get('vias', []):
        if any(abs(float(v[k])-d)>1e-7 for k in ('d','diameter') if k in v): return True
        if any(abs(float(v[k])-drill)>1e-7 for k in ('drill','bohrung') if k in v): return True
        if v.get('layers') and list(v['layers']) not in (['F','B'], ['F.Cu','B.Cu']): return True
        if v.get('type') not in (None, 'through', 'through_hole'): return True
    return False


def reparatur_sperre(R):
    if individuelle_vias(R): return 'individuelle Via-Geometrie braucht nativen Zug'
    if any(t.get('items') for t in R.get('tracks',[])):
        return 'native Linien/Boegen brauchen nativen Zug'
    if (R.get('optionen') or {}).get('fix') or R.get('zonenplan_id'):
        return 'festes Fremdkupfer/Zonenplan braucht abgestimmten nativen Zug'
    return None


def zugregeln(R):
    import json
    opt = R.get('optionen') or {}
    rules = {}
    if opt.get('regeln'): rules.update(json.loads(Path(opt['regeln']).read_text()).get('router', {}))
    rules.update(opt)
    return rules


def loch(p):
    if p.get('hole_polys'): return unary_union([Polygon(q) for q in p['hole_polys']])
    if p.get('drill', 0): return Point(p['x'], p['y']).buffer(float(p['drill'])/2, 32)
    return None


def harte_vias(B, R0, R1):
    ""
    rules=zugregeln(R1);d,h=map(float,rules.get('via','0.45/0.20').split('/'))
    edge=float(rules.get('edge','.3'));clr=float(rules.get('drcpad',rules.get('drc','.127')))
    hc=float(rules.get('loch_clr','0'));h2h=float(rules.get('h2h','.25'))
    holes=B.get('outline_holes') or []
    allowed=unary_union([Polygon(q,holes[i] if i<len(holes) else []) for i,q in enumerate(B.get('outline',[]))])
    old=Counter((v['net'],v['x'],v['y']) for v in R0.get('vias',[]))
    def fail(gate,dist,need):return dict(ok=False,tor=gate,abstand_mm=round(dist,8),minimum_mm=need)
    for v in R1.get('vias',[]):
        key=(v['net'],v['x'],v['y'])
        if old[key]:old[key]-=1;continue
        q=Point(v['x'],v['y']);radius=float(v.get('diameter',v.get('d',d)))/2;hr=float(v.get('drill',v.get('bohrung',h)))/2
        dist=q.distance(allowed.boundary)
        if not allowed.covers(q) or dist<radius+edge-1e-8:return fail('Via-Rand',dist,radius+edge)
        for k in B.get('keepouts',[]):
            if not (k.get('verbote') or {}).get('vias',True):continue
            for poly in k['polys']:
                dist=q.distance(Polygon(poly))
                if dist<=max(radius,hr)+1e-8:return fail('Via-only-Sperre',dist,max(radius,hr))
        for pad in B['pads']:
            if pad['net']!=v['net']:
                for polys in pad['polys'].values():
                    for poly in polys:
                        dist=q.distance(Polygon(poly))
                        if dist<radius+clr-1e-8:return fail('Via-Pad-Kupfer',dist,radius+clr)
                hole=loch(pad)
                if hole is not None:
                    dist=q.distance(hole)
                    if dist<radius+hc-1e-8:return fail('Via-Kupfer-Bohrung',dist,radius+hc)
            hole=loch(pad)
            if hole is not None:
                dist=q.distance(hole)
                if dist<hr+h2h-1e-8:return fail('Via-Bohrung-Bohrung',dist,hr+h2h)
        for vs in B.get('via_sperre',[]):
            own=[pad for pad in B['pads'] if pad['ref']==vs.get('ref') and pad['net']==v['net']]
            if any(Polygon(poly).covers(q) for pad in own for polys in pad['polys'].values() for poly in polys):continue
            for poly in vs['polys']:
                dist=q.distance(Polygon(poly))
                if dist<hr+float(rules.get('drc','.127'))-1e-8:return fail('Via-Net-Tie-Bohrung',dist,hr+float(rules.get('drc','.127')))
        for other in R1.get('vias',[]):
            if other is v:continue
            dist=q.distance(Point(other['x'],other['y']))
            need=hr+float(other.get('drill',other.get('bohrung',h)))/2+h2h
            if dist<need-1e-8:return fail('Via-Bohrung-Bohrung',dist,need)
    return dict(ok=True)


def harte_segmente(B, R0, R1, diagnose=False):
    ""
    import json
    rules = {}
    opt = R1.get('optionen') or R0.get('optionen') or {}
    if opt.get('regeln'):
        rules.update(json.loads(Path(opt['regeln']).read_text()).get('router', {}))
    rules.update(opt)
    edge = float(rules.get('edge', '.3')); padclr = float(rules.get('drcpad', rules.get('drc', '.127')))
    holeclr = float(rules.get('loch_clr', '0'))
    holes=B.get('outline_holes') or []
    allowed = unary_union([Polygon(q,holes[i] if i<len(holes) else []) for i,q in enumerate(B.get('outline', []))])
    def segments(R):
        for t in R['tracks']:
            for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']):
                yield (t['net'],t['layer'],tuple(sorted((tuple(a),tuple(b)))),float(w))
    def reject(gate,dist,need):
        detail=dict(ok=False,tor=gate,abstand_mm=round(dist,8),minimum_mm=need)
        return detail if diagnose else False
    old = Counter(segments(R0))
    for net,layer,ab,w in segments(R1):
        key=(net,layer,ab,w)
        if old[key]: old[key]-=1; continue
        g=LineString(ab); radius=w/2
        if w<=0 or not math.isfinite(w) or allowed.is_empty: return reject('Segmentgeometrie',0,0)
        if not allowed.covers(g) or g.distance(allowed.boundary)<radius+edge-1e-8: return reject('Segment-Rand',g.distance(allowed.boundary),radius+edge)
        for k in B.get('keepouts', []):
            if not (k.get('verbote') or {}).get('bahnen', True): continue
            layers=k.get('lagen') or k.get('layers') or ['*']
            if layer not in {q.replace('.Cu','') for q in layers} and not set(layers)&{'*','*.Cu'}: continue
            for q in k['polys']:
                dist=g.distance(Polygon(q))
                if dist<=radius+1e-8: return reject('Segment-Sperre',dist,radius)
        for p in B['pads']:
            if p['net']==net: continue
            for q in p['polys'].get(layer, []):
                dist=g.distance(Polygon(q))
                if dist<radius+padclr-1e-8: return reject('Segment-Pad',dist,radius+padclr)
            hole=loch(p)
            if hole is not None and g.distance(hole)<radius+holeclr-1e-8: return reject('Segment-Bohrung',g.distance(hole),radius+holeclr)
    return dict(ok=True) if diagnose else True


def offsets(maximum=.4):

    for d in (.1, .2, .3, .4):
        for dx, dy in ((d, 0), (-d, 0), (0, d), (0, -d), (d, d), (-d, d), (d, -d), (-d, -d)):
            if math.hypot(dx, dy) <= maximum + 1e-8:
                yield dx, dy


def octilinear(a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    h = min(abs(dx), abs(dy))
    q = [round(a[0] + math.copysign(h, dx), 5), round(a[1] + math.copysign(h, dy), 5)]
    out = [list(a)]
    if math.dist(a, q) > 1e-7: out.append(q)
    if math.dist(out[-1], b) > 1e-7: out.append(list(b))
    return out


def via_zug(B, unit, index, target, via_d=.45, vip=True, ep=False, stub_w=.1):
    ""
    v = unit['vias'][index]; q = [v['x'], v['y']]; net = unit['net']
    if any(v.get(k) for k in ('n', 'zu', 'gr', 'rv', 'strom_via', 'rueck_via')): return None
    if any(math.dist(q, (w['x'], w['y'])) < 1e-6 for j, w in enumerate(unit['vias']) if j != index): return None
    own = []
    for p in B['pads']:
        if p['net'] != net: continue
        for L, polys in p['polys'].items():
            g = unary_union([Polygon(a) for a in polys if len(a) >= 3])
            if g.distance(Point(q)) < via_d/2 - 1e-6: own.append((p, L, g))

    if any(min(p['bbox'][2]-p['bbox'][0],p['bbox'][3]-p['bbox'][1])>=1 and
           sum(1 for z in B['pads'] if z['ref']==p['ref'])>=8 for p,_,_ in own): ep=True
    if ep and not any(g.buffer(-via_d/2+1e-6).covers(Point(target)) for _, _, g in own): return None
    if not vip and own and not ep:
        if any(g.distance(Point(target)) < via_d/2 + 1e-6 for _, _, g in own): return None
    changed = copy.deepcopy(unit); connections = set()
    for t in changed['tracks']:
        ps, ws = t['pts'], t['w']
        if any(math.dist(p, q) < 1e-6 for p in ps[1:-1]): return None

        for a, b in zip(ps[:-1], ps[1:]):
            if min(math.dist(a, q), math.dist(b, q)) > 1e-6 and LineString([a,b]).distance(Point(q)) < 1e-6: return None
        if len(ps) < 2: return None
        if math.dist(ps[0], q) < 1e-6:
            p = octilinear(target, ps[1]); t['pts'] = p + ps[2:]; t['w'] = [ws[0]]*(len(p)-1)+ws[1:]
            connections.add(t['layer'])
        elif math.dist(ps[-1], q) < 1e-6:
            p = octilinear(ps[-2], target); t['pts'] = ps[:-2]+p; t['w'] = ws[:-1]+[ws[-1]]*(len(p)-1)
            connections.add(t['layer'])
    for p, L, g in own:
        if g.distance(Point(target)) < via_d/2 - 1e-6: continue

        copper=unary_union([Point(target).buffer(via_d/2,32)]+[
            LineString([a,b]).buffer(w/2,32) for t in changed['tracks'] if t['layer']==L
            for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w'])])
        pieces=list(copper.geoms) if hasattr(copper,'geoms') else [copper]
        if any(c.covers(Point(target)) and c.distance(g)<1e-8 for c in pieces): continue
        ps = octilinear([p['x'],p['y']], target)
        if len(ps) > 1:
            changed['tracks'].append(dict(layer=L, pts=ps, w=[stub_w]*(len(ps)-1), sig=True))
            connections.add(L)

    if not connections and not own: return None
    changed['vias'][index].update(x=round(target[0],5), y=round(target[1],5))
    return changed

def _via_netz(unit, v):
    return v['net'] if v.get('rv') and v.get('net') else unit['net']


def vias_nach_zug(vias0, alt, neu):
    ""
    weg = Counter((_via_netz(alt, w), w['x'], w['y']) for w in alt['vias'])
    rest = []
    for z in vias0:
        k = (z['net'], z['x'], z['y'])
        if weg[k]: weg[k] -= 1; continue
        rest.append(z)
    return [dict(z, net=_via_netz(neu, z)) for z in neu['vias']] + rest


def teil_freiraum(B0, B1, R1):
    ""
    alt = {f['ref']: pose(f) for f in B0['fps']}
    moved = {f['ref'] for f in B1['fps'] if alt.get(f['ref']) != pose(f)}
    if not moved: return dict(ok=True)
    rules = zugregeln(R1); d, h = map(float, rules.get('via', '0.45/0.20').split('/'))
    padclr = float(rules.get('drcpad', rules.get('drc', '.127'))); holeclr = float(rules.get('loch_clr', '0'))
    h2h = float(rules.get('h2h', '.25'))
    def fail(gate, dist, need): return dict(ok=False, tor=gate, abstand_mm=round(dist, 8), minimum_mm=need)
    pads = [(p['net'], {L: unary_union([Polygon(q) for q in qs if len(q) >= 3]) for L, qs in p['polys'].items()}, loch(p))
            for p in B1['pads'] if p['ref'] in moved]
    keep = []
    for k in B1.get('keepouts', []):
        if k.get('ref') not in moved: continue
        lagen = {q.replace('.Cu', '') for q in (k.get('lagen') or k.get('layers') or ['*'])}
        keep.append((lagen, (k.get('verbote') or {}), unary_union([Polygon(q) for q in k['polys'] if len(q) >= 3])))
    geos = [g for _, by, hole in pads for g in list(by.values()) + ([hole] if hole is not None else [])] + [g for _, _, g in keep]
    if not geos: return dict(ok=True)
    nah = unary_union(geos).buffer(1.0)
    for t in R1.get('tracks', []):
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            g = LineString([a, b]) if math.dist(a, b) > 1e-9 else Point(a)
            if not g.intersects(nah): continue
            r = float(w) / 2
            for net, by, hole in pads:
                if net == t['net']: continue
                if t['layer'] in by:
                    dist = g.distance(by[t['layer']])
                    if dist < r + padclr - 1e-8: return fail('Teilzug: Bahn-Pad', dist, r + padclr)
                if hole is not None and g.distance(hole) < r + holeclr - 1e-8: return fail('Teilzug: Bahn-Bohrung', g.distance(hole), r + holeclr)
            for lagen, verbote, poly in keep:
                if not verbote.get('bahnen', True) or not (t['layer'] in lagen or lagen & {'*'}): continue
                if g.distance(poly) <= r + 1e-8: return fail('Teilzug: Bahn-Footprint-Sperre', g.distance(poly), r)
    for v in R1.get('vias', []):
        q = Point(v['x'], v['y'])
        if not q.intersects(nah): continue
        r = float(v.get('diameter', v.get('d', d))) / 2; hr = float(v.get('drill', v.get('bohrung', h))) / 2
        for net, by, hole in pads:
            if hole is not None and q.distance(hole) < hr + h2h - 1e-8: return fail('Teilzug: Via-Bohrung-Bohrung', q.distance(hole), hr + h2h)
            if net == v['net']: continue
            for g in by.values():
                if q.distance(g) < r + padclr - 1e-8: return fail('Teilzug: Via-Pad', q.distance(g), r + padclr)
            if hole is not None and q.distance(hole) < r + holeclr - 1e-8: return fail('Teilzug: Via-Bohrung', q.distance(hole), r + holeclr)
        for lagen, verbote, poly in keep:
            if verbote.get('vias', True) and q.distance(poly) <= max(r, hr) + 1e-8: return fail('Teilzug: Via-Footprint-Sperre', q.distance(poly), max(r, hr))
    return dict(ok=True)


def tor(B0, R0, E0, B1, R1, E1, K):
    ""
    via_gate=harte_vias(B1,R0,R1)
    if not via_gate['ok']: return dict(besser=False,grund=via_gate['tor'],geometrie=via_gate)
    segment_gate=harte_segmente(B1,R0,R1,diagnose=True)
    if not segment_gate['ok']: return dict(besser=False,grund='geaenderte Segmente verletzen harte Hindernisse',geometrie=segment_gate)
    free_gate=teil_freiraum(B0,B1,R1)
    if not free_gate['ok']: return dict(besser=False,grund='unveraendertes Kupfer verletzt das bewegte Teil',geometrie=free_gate)
    if E1['konflikte'] >= E0['konflikte']: return dict(besser=False, grund='Konflikte sinken nicht')
    if E1['offen'] > E0['offen'] or not set(E1['unverbunden']) <= set(E0['unverbunden']):
        return dict(besser=False, grund='Verbindungen schlechter')
    if not set(R1['failed']) <= set(R0['failed']): return dict(besser=False, grund='neue offene Einheiten')
    if not set(R1['conflicts']) <= set(R0['conflicts']): return dict(besser=False, grund='neue Konflikteinheiten')
    counts0 = Counter(v['net'] for v in R0['vias'] if v['net'] in K['gnd'] or K['netze'].get(v['net'],{}).get('art') in ('strom','versorgung','sw'))
    counts1 = Counter(v['net'] for v in R1['vias'])
    if any(counts1[n] < c for n,c in counts0.items()): return dict(besser=False, grund='geschuetzte Via-Zahl sinkt')
    pads0=massepad_zahlen(B0,R0,K);pads1=massepad_zahlen(B1,R1,K)
    if any(pads1.get(p,0)<n for p,n in pads0.items()):return dict(besser=False,grund='R16: Via-Zahl eines Massepads sinkt')
    c = annahme(E0, E1, B0, R0, B1, R1, K, note='v3', mindest=0)
    rg = RK.annahme(E0['regeln'], E1['regeln'])
    if not rg['nicht_schlechter']: return dict(besser=False, grund=rg['grund'])
    if not math.isfinite(E1['note_v3']) or E1['note_v3'] > E0['note_v3'] + 1e-6:
        return dict(besser=False, grund='Note v3 schlechter')
    return c


def loese_vias(G):
    ""
    args, B, K, U = G['args'], G['D'], G['KL'], G['U']
    t0 = time.process_time(); limit = max(0, float(args.get('rest_cpu','30')))
    teile_an=args.get('rest_teile','1')=='1' and bool(args.get('klassen'))
    if teile_an: limit *= .25
    log = dict(version=VERSION, versuche=[], verworfen=[], uebernommen=0, cpu_s=0.0)
    def aus(): return time.process_time()-t0 >= limit or G['zeit_knapp'](1.0)
    source=G.get('ST') or {}
    def state():
        c = G['exact_conflicts'](); ts, vs = G['ausgabe_geometrie']()
        result=G['ausgabe_daten'](ts,vs,{u:len(v) for u,v in c.items()},roh=True)
        for field in ('zones','zonen','r41_fuellung_lagen'):
            if source.get(field): result[field]=source[field]
        return result
    if not K or not limit: log['grund']='Klassen oder Budget fehlen'; return log
    blocked=reparatur_sperre(dict(source,optionen=dict(args,via=args.get('via',str(G['VIA_D'])+'/0.20'))))
    if blocked:
        log['grund']=blocked;return log
    if G.get('ZONENPLAN') or G.get('PAAR_NETZE'):
        log['grund']='gebundener Zonenplan/Paarnetze brauchen gemeinsame Zuege'; return log
    G['_REST_LOKAL_KONFLIKT']=True
    R0 = state(); paare0=list(G['PAIRS'])
    if not R0['conflicts']: log['grund']='konfliktfrei'; return log
    try:
        E0 = bewerte(B,R0,K,v3=True,minimax=False)
    except Exception as error:
        log.update(grund='Bewertung fehlt: '+repr(error)[:180],cpu_s=round(time.process_time()-t0,4));return log
    nmax = max(0,int(args.get('rest_kand','32')))
    visited=set()
    while not aus() and len(log['versuche']) < nmax:
        G['exact_conflicts'](); stellen = [(x,y) for _,_,_,_,x,y in G['PAIRS']]
        candidates=[]
        for u in sorted(U):
            loaded_own = u.startswith('FIX#') and 'start' in args and 'fix' not in args and U[u]['net'] in G['FLNETZE']
            if (u.startswith(('FIX#','HEFT#')) or u in G['FIXED']) and not loaded_own: continue
            r=U[u]
            for vi,v in enumerate(r['vias']):
                d=min((math.dist((v['x'],v['y']),p) for p in stellen),default=math.inf)
                if d <= float(args.get('rest_radius','.8')): candidates.append((d,u,vi))
        accepted=False
        for _,u,vi in sorted(candidates):
            if accepted or aus(): break
            v=U[u]['vias'][vi]
            for dx,dy in offsets():
                if aus() or len(log['versuche']) >= nmax: break
                key=(u,vi,round(v['x']+dx,5),round(v['y']+dy,5))
                if key in visited: continue
                visited.add(key)
                new=via_zug(B,U[u],vi,key[2:],G['VIA_D'],G['VIP'],u.startswith('GND#') and int(u[4:]) in G['EPM'],max(float(args.get('bahn_min','0')),G['cls']('/__signal__')['wmin'] if 'cls' in G else .1))
                loaded_own = u.startswith('FIX#') and 'start' in args and 'fix' not in args and U[u]['net'] in G['FLNETZE']
                stitch=False
                if (new is None and loaded_own and U[u]['net'] in K['gnd'] and source.get('zones') and not U[u]['tracks'] and
                        any(z.get('net')==U[u]['net'] and z.get('filled') is True and not z.get('holes') and Polygon(z['pts']).covers(Point(v['x'],v['y']).buffer(G['VIA_D']/2)) and Polygon(z['pts']).covers(Point(key[2:]).buffer(G['VIA_D']/2)) for z in source['zones'])):


                    new=copy.deepcopy(U[u]);new['vias'][vi].update(x=key[2],y=key[3]);stitch=True
                if new is None:
                    log['verworfen'].append(dict(einheit=u,via=vi,ziel=list(key[2:]),tor='Anbindung/Gruppenschutz',abstand_mm=math.hypot(dx,dy),vorher=len(R0['conflicts']),nachher=None))
                    continue
                vg=harte_vias(B, R0, dict(R0,vias=vias_nach_zug(R0['vias'],U[u],new)))
                if not vg['ok']:
                    log['verworfen'].append(dict(einheit=u,via=vi,ziel=list(key[2:]),vorher=len(R0['conflicts']),nachher=None,**vg))
                    continue


                ix=int((key[2]-G['X0'])/G['RES']);iy=int((key[3]-G['Y0'])/G['RES'])
                if not (0<=ix<G['NX'] and 0<=iy<G['NY']): continue
                import numpy as np
                if not G['via_hart'](new['net'],np.asarray([iy*G['NX']+ix],dtype=np.int64)):
                    log['verworfen'].append(dict(einheit=u,via=vi,ziel=list(key[2:]),tor='Raster-Hartkarte',abstand_mm=math.hypot(dx,dy),vorher=len(R0['conflicts']),nachher=None))
                    continue


                lokal=[p for p in stellen if math.dist((v['x'],v['y']),p)<=float(args.get('rest_radius','.8'))]
                lokal.extend(((v['x'],v['y']),key[2:]))
                group=[u]+[w for w in sorted(U) if w!=u and not w.startswith(('FIX#','HEFT#')) and w not in G['FIXED'] and G['_nah_an'](U[w],lokal,1.0)]
                if len(group)>int(args.get('rest_gruppe','40')):
                    log['verworfen'].append(dict(einheit=u,via=vi,ziel=list(key[2:]),tor='Gruppengroesse',abstand_mm=math.hypot(dx,dy),vorher=len(R0['conflicts']),nachher=None,gruppe_n=len(group)))
                    continue
                save={w:copy.deepcopy(U[w]) for w in group}; order=list(U); fails=set(G['fail']); hs=G['hist'].copy(); pf=G['PF'][0]
                via_alt=copy.deepcopy(G.get('VIA_ALT',{}))
                weich=G['VG_WEICH'][0]; G['VG_WEICH'][0]=False
                ok=False; reason='CPU-Budget'; R1=E1=None;decision={};paare1=None
                try:
                    G['del_unit'](u); G['add_unit'](u,new['net'],new['tracks'],new['vias'])

                    R1=state();paare1=list(G['PAIRS'])
                    if R1['conflicts'] and not aus():
                        for w in group[1:]: G['del_unit'](w)
                        for w in G['paar_reihe'](sorted(group[1:],key=lambda w:(G['grp'](w),w))):
                            if aus(): break
                            if not G['route_unit'](w): G['fail'].add(w)
                            else: G['fail'].discard(w)
                        R1=state();paare1=list(G['PAIRS'])
                    if not aus():
                        E1=bewerte(B,R1,K,v3=True,minimax=False)
                        decision=tor(B,R0,E0,B,R1,E1,K); ok=decision['besser']; reason=decision['grund']
                        if stitch:
                            coverage=deckungs_schutz(B,R0,R1,K)
                            if not coverage['ok']:ok=False;reason=coverage['grund']
                except Exception as e: reason='Pruefung fehlgeschlagen: '+repr(e)[:180]
                finally:
                    G['hist'][:]=hs; G['PF'][0]=pf; G['VG_WEICH'][0]=weich
                    if not ok:
                        for w in group:
                            if w in U: G['del_unit'](w)
                        for w in group: G['add_unit'](w,save[w]['net'],save[w]['tracks'],save[w]['vias'])
                        G['fail'].clear(); G['fail'].update(fails)
                        if 'VIA_ALT' in G:G['VIA_ALT'].clear();G['VIA_ALT'].update(via_alt)

                    ordered={w:U[w] for w in order}; U.clear(); U.update(ordered); G['_KIDX'][0]=None
                log['versuche'].append(dict(einheit=u, via=vi, ziel=list(key[2:]), gruppe=group, angenommen=ok, grund=reason,
                    vorher=dict(konflikte=E0['konflikte'], note_v3=E0['note_v3']),
                    nachher=dict(konflikte=len(R1['conflicts'])) if E1 is None and R1 is not None else (None if E1 is None else dict(konflikte=E1['konflikte'],note_v3=E1['note_v3'])),
                    tor=reason,abstand_mm=round(math.hypot(dx,dy),5),zug_mm=round(math.hypot(dx,dy),5),
                    konflikt_paare_vorher=paare0,konflikt_paare_nachher=paare1,pruefung_vollstaendig=E1 is not None,
                    geometrie=decision.get('geometrie') if E1 is not None else None))
                if ok:
                    R0,E0,paare0=R1,E1,paare1; log['uebernommen']+=1; accepted=True; break
                if teile_an and next(teile_kandidaten(B,K,lokal),None) is not None:
                    log['teile_vorrang']=lokal
                    log['grund']='Via-Zug gescheitert: lokale C/R zuerst'
                    log['cpu_s']=round(time.process_time()-t0,4)
                    G['exact_conflicts']()
                    return log
        if not accepted or not R0['conflicts']: break
    log['cpu_s']=round(time.process_time()-t0,4); log['budget_erreicht']=aus()
    G['exact_conflicts']()
    return log


def teile_kandidaten(B, K, stellen):
    ""
    import re
    from gemeinsam import wanderweite as WW
    from gemeinsam.teile import lade_teile, platine, Belegung
    from gemeinsam.verschieben import bewegen
    from nachbearbeitung.zugregeln import sperre
    W=WW.tabelle(K); _,_,inside,_=platine(B,.2)
    fps={f['ref']:f for f in B['fps']}

    raw_groups=B.get('platzierungsgruppen') or []
    groups=raw_groups.values() if isinstance(raw_groups,dict) else raw_groups
    grouped={r for g in groups if isinstance(g,dict) for r in g.get('refs', [])}
    for r in sorted(fps, key=lambda r:(min((math.dist((fps[r]['x'],fps[r]['y']),s) for s in stellen),default=math.inf),r)):
        f=fps[r]; t=K['teile'].get(r,{})
        size=re.search(r'(?:^|[_:])([CR])_(0201|0402|0603|0805)(?:_|$)',f.get('fp',''))
        if not size or r in grouped or f.get('locked') or t.get('fest') or t.get('rolle')!='klein' or r not in W or W[r]['max']<=0: continue
        if not any(math.dist((f['x'],f['y']),s)<=1.5 for s in stellen): continue
        nearest=min(stellen,key=lambda s:math.dist((f['x'],f['y']),s))
        ax,ay=f['x']-nearest[0],f['y']-nearest[1]

        direction=(0,1 if ay>=0 else -1) if abs(ay)>=abs(ax) else (1 if ax>=0 else -1,0)
        preferred=[(round(direction[0]*d,5),round(direction[1]*d,5),0) for d in (.1,.2,.3,.4,.5)]
        rotations=[(0,0,th) for th in (90,180,270)] if W[r]['drehen'] else []
        poses=preferred+rotations+[(dx,dy,0) for dx,dy in offsets(.5) if (dx,dy,0) not in preferred]
        for dx,dy,th in poses:
            original=(B.get('wander_basis') or {}).get(r) or f
            if isinstance(original,(list,tuple)):original=dict(x=original[0],y=original[1])
            old_distance=math.dist((f['x'],f['y']),(original['x'],original['y']))
            distance=math.dist((f['x']+dx,f['y']+dy),(original['x'],original['y']))
            if distance>W[r]['max']+1e-8: continue
            moves={r:(dx,dy,th)}; B1,P=bewegen(B,moves)
            T1=lade_teile(B1,.15); f1={q['ref']:q for q in B1['fps']}; bel=Belegung()
            for q in T1:
                if q!=r: bel.add(q,T1[q].geo(f1[q]['x'],f1[q]['y'],0,f1[q]['side']))
            geo=T1[r].geo(f1[r]['x'],f1[r]['y'],0,f1[r]['side'])
            if bel.kollision(geo) or not all(inside.contains(g) for g in geo.values()) or sperre(B1,geo,(r,)): continue
            yield dict(ref=r,moves=moves,board=B1,placement=P,preis=WW.preis(W[r],old_distance,distance))


def teil_netze(B,B1,K,ref,paare):
    ""
    pads=[p for b in (B,B1) for p in b['pads'] if p['ref']==ref]
    affected={p['net'] for p in pads}
    for pair in paare:
        if any(math.dist((p['x'],p['y']),pair[-2:])<=1.5 for p in pads):
            affected.update(n for n in pair[2:4] if n in K['netze'])
    return affected


def teile_nachlauf(board_path, klassen_path, routes_path, budget, mit_platz=False):
    ""
    import json
    import os
    import resource
    import sys
    import tempfile
    from pathlib import Path
    from gemeinsam import plaetze as PL
    root=Path(__file__).resolve().parents[1]
    B=json.loads(Path(board_path).read_text()); K=json.loads(Path(klassen_path).read_text()); R0=json.loads(Path(routes_path).read_text())
    B=effektives_board(B,R0)
    log=dict(version=VERSION,versuche=[],verworfen=[],uebernommen=0,cpu_s=0.0)
    blocked=reparatur_sperre(R0)
    if blocked:
        log['grund']=blocked;return log
    if not R0.get('conflicts') or budget<1: log['grund']='konfliktfrei/Budget'; return log
    if any((R0.get('optionen') or {}).get(k) for k in ('fix','gnd_ohne_via')):
        log['grund']='Feste/ausgelassene Flaechenpads brauchen abgestimmte Placement-Uebernahme';return log
    if R0.get('zones') or R0.get('zonen') or R0.get('zonenplan_id'):
        log['grund']='Zonen brauchen Neufuellung'; return log
    pairs_path=Path(routes_path).with_name(Path(routes_path).stem+'_konflikte.json')
    if not pairs_path.exists(): log['grund']='Konfliktkarte fehlt'; return log
    paare=json.loads(pairs_path.read_text());stellen=[(p[-2],p[-1]) for p in paare]
    priority=(R0.get('restkonflikte') or {}).get('teile_vorrang')
    if priority: stellen=priority
    start_cpu=time.process_time(); children0=resource.getrusage(resource.RUSAGE_CHILDREN)
    def spent():
        ch=resource.getrusage(resource.RUSAGE_CHILDREN)
        return time.process_time()-start_cpu+ch.ru_utime+ch.ru_stime-children0.ru_utime-children0.ru_stime
    opt=dict(R0.get('optionen') or {})
    for key in ('png','etiketten','verlauf','pre','fix','nur','neu','neu_netze','start','lerndaten','lernpolicy','zonenplan','rest_reparatur'):
        opt.pop(key,None)
    opt.update(restkonflikte='0',kurzschluss_schutz='0',rest_pruefung='1',klassen=str(Path(klassen_path).resolve()),kurz='0',kurz_via='0',via_am_pad_raeumen='0',hypo='0',r19_form='0')
    if opt.get('regeln'): opt['regeln']=str(Path(opt['regeln']).resolve())
    def run(bj,sj,rj,check=False,ref=None,netze=()):
        remaining=budget-spent()
        if remaining<1: return False
        o=dict(opt,start=str(sj),cpu_max=str(max(1,int(remaining))),iter='12')
        if check: o['rest_check']='1'
        if ref: o.update(neu=ref,neu_netze=','.join(sorted(netze)),rest_teil_lokal='1')
        rr=PL.lauf([sys.executable,str(root/'router/route_pf.py'),str(bj),str(rj)]+[f'{k}={v}' for k,v in o.items()],
                   cpu_s=max(1,int(remaining)),log=str(rj)+'.log',mit_platz=mit_platz,env=dict(os.environ,PYTHONHASHSEED='0'))
        log.setdefault('laeufe',[]).append(rr)
        return rr['zustand']=='ok' and rj.exists() and spent()<=budget
    with tempfile.TemporaryDirectory(prefix='autorouter_r45_') as td:
        d=Path(td); bj=d/'board.json'; sj=d/'start.json'; check=d/'check.json'; out=d/'routes.json'
        bj.write_text(json.dumps(B));sj.write_text(json.dumps(R0))


        if not run(bj,sj,check,check=True): log['grund']='Basispruefung/Budget'; log['cpu_s']=spent();return log
        C0=json.loads(check.read_text()); R0=dict(R0,conflicts=C0['conflicts'])
        E0=bewerte(B,R0,K,v3=True,minimax=False)
        for c in teile_kandidaten(B,K,stellen):
            if spent()>=budget or len(log['versuche'])>=int(opt.get('rest_teil_kand','8')): break
            B1=c['board'];bj.write_text(json.dumps(B1))
            moved_start=start_nach_teilzug(B,R0,c['ref'])
            if moved_start is None:
                log['verworfen'].append(dict(ref=c['ref'],moves=c['moves'],tor='geteilte/geschuetzte Pad-Anbindung',zug_mm=math.hypot(*c['moves'][c['ref']][:2]),vorher=dict(konflikte=E0['konflikte']),nachher=None,konflikt_paare_vorher=C0.get('rest_check_paare'),konflikt_paare_nachher=None,pruefung_vollstaendig=False))
                continue
            sj.write_text(json.dumps(moved_start))
            affected=teil_netze(B,B1,K,c['ref'],paare)
            if not run(bj,sj,out,ref=c['ref'],netze=affected-set(K['gnd'])):
                log['versuche'].append(dict(ref=c['ref'],moves=c['moves'],angenommen=False,grund='Router/Budget',tor='Router/Budget',abstand_mm=math.hypot(*c['moves'][c['ref']][:2]),vorher=dict(konflikte=E0['konflikte']),nachher=None,konflikt_paare_vorher=C0.get('rest_check_paare'),konflikt_paare_nachher=None,pruefung_vollstaendig=False));continue
            R1=json.loads(out.read_text())

            affected=teil_netze(B,B1,K,c['ref'],paare)
            for field in ('tracks','vias'):
                R1[field]=[q for q in R0[field] if q['net'] not in affected]+[q for q in R1[field] if q['net'] in affected]
            sj.write_text(json.dumps(R1))
            if not run(bj,sj,check,check=True):
                log['versuche'].append(dict(ref=c['ref'],moves=c['moves'],angenommen=False,grund='Abschlusspruefung/Budget',tor='Abschlusspruefung/Budget',abstand_mm=math.hypot(*c['moves'][c['ref']][:2]),vorher=dict(konflikte=E0['konflikte']),nachher=None,konflikt_paare_vorher=C0.get('rest_check_paare'),konflikt_paare_nachher=None,pruefung_vollstaendig=False));continue
            C1=json.loads(check.read_text());R1['rest_check_paare']=C1.get('rest_check_paare',[]);R1['conflicts']=C1['conflicts'];R1['failed']=sorted(set(R1['failed'])|set(C1['failed']))
            E1=bewerte(B1,R1,K,v3=True,minimax=False);decision=tor(B,R0,E0,B1,R1,E1,K)
            ok=decision['besser'] and E1['note_v3']+max(0,c['preis'])<=E0['note_v3']+1e-6 and spent()<=budget
            log['versuche'].append(dict(ref=c['ref'],moves=c['moves'],angenommen=ok,grund=decision['grund'],
                vorher=dict(konflikte=E0['konflikte'],note_v3=E0['note_v3']),nachher=dict(konflikte=E1['konflikte'],note_v3=E1['note_v3']),
                tor=decision['grund'] if not decision['besser'] else ('Wanderpreis/CPU' if not ok else 'angenommen'),
                abstand_mm=round(math.hypot(*c['moves'][c['ref']][:2]),5),zug_mm=round(math.hypot(*c['moves'][c['ref']][:2]),5),
                konflikt_paare_vorher=C0.get('rest_check_paare'),konflikt_paare_nachher=C1.get('rest_check_paare'),pruefung_vollstaendig=True,geometrie=decision.get('geometrie')))
            if ok:
                R1['optionen']=R0['optionen'];R1['restkonflikte']=R0.get('restkonflikte')
                R1.update(placement_marker(B,B1))
                R1['rest_teile']=log
                log['uebernommen']=1;log['cpu_s']=spent()

                dest=Path(routes_path);tmp=dest.with_name(dest.name+'.r45.tmp');tmp.write_text(json.dumps(R1));tmp.replace(dest)
                dest.with_name(dest.stem+'_rest_board.json').write_text(json.dumps(B1))
                dest.with_name(dest.stem+'_rest_placement.json').write_text(json.dumps(R1['rest_placement']))
                break
    log['cpu_s']=round(spent(),4)
    return log


def effektives_board(B,R):
    ""
    if not R.get('rest_board'): return B
    from gemeinsam.verschieben import bewegen
    fps={f['ref']:f for f in B['fps']};target={f['ref']:f for f in R['rest_board']['fps']}
    moves={};sides={}
    for r,source in R['rest_quelle'].items():
        current=pose(fps[r]);end=pose(target[r])
        known=(R.get('rest_native_sources') or {}).get(r) or [source]
        if current not in known+[end]:raise ValueError('R45-Placement gehoert zu anderer Quellpose: '+r)
        th=(current['rot']-end['rot'])%360 if current['side']==end['side'] else (180-current['rot']-end['rot'])%360
        moves[r]=(end['x']-current['x'],end['y']-current['y'],th);sides[r]=end['side']
    expected,_=bewegen(B,moves,seiten=sides)
    import json
    target_pads=R['rest_board']['pads']
    names={(p['ref'],p['num'],p['net']) for p in target_pads}
    expected_pads=[p for p in expected['pads'] if not (p['ref'].startswith('SPERR_') and p['net'].startswith('#SPERR_')) or (p['ref'],p['num'],p['net']) in names]
    if len(expected_pads)!=len(target_pads):raise ValueError('R45-Zielboard hat andere Padinstanzen')
    def canonical(q):return json.dumps(q,sort_keys=True,default=lambda x:x.tolist())
    if any(canonical({k:p.get(k) for k in q})!=canonical(q) for p,q in zip(expected_pads,target_pads)):
        raise ValueError('R45-Zielpads passen nicht zur Transformation')
    for field in ('wander_basis','wander_poses'):
        if field in R['rest_board']:expected[field]=copy.deepcopy(R['rest_board'][field])
    return expected

def lokales_board_uebernehmen(board_path, routes_path):
    ""
    import json
    from pathlib import Path
    R=json.loads(Path(routes_path).read_text())
    if R.get('rest_board'):
        B=effektives_board(json.loads(Path(board_path).read_text()),R)
        Path(board_path).write_text(json.dumps(B))


def reparatur_lauf(root, board, routes, out, klassen, regeln, lagen, via, gndlagen, budget):
    ""
    import json
    import os
    import sys
    from pathlib import Path
    R=json.loads(Path(routes).read_text()); opt=dict(R.get('optionen') or {})
    for k,v in dict(klassen=klassen,regeln=regeln,lagen=lagen,via=via,gndlagen=gndlagen).items():
        if v: opt.setdefault(k,str(v))
    for k in ('start','neu','neu_netze','png','etiketten','verlauf','pre','fix','nur','lerndaten','lernpolicy'):
        opt.pop(k,None)
    opt.update(start=str(routes),rest_reparatur='1',restkonflikte='1',rest_cpu=str(budget))
    os.execv(sys.executable,[sys.executable,str(Path(root)/'router/route_pf.py'),str(board),str(out)]+[f'{k}={v}' for k,v in opt.items()])


def reparatur_ausgabe(G):
    ""
    import json
    import tempfile
    from pathlib import Path
    source=copy.deepcopy(G['ST'])

    def near(r,places,radius):
        return any(Point(v['x'],v['y']).distance(Point(q))<=radius for v in r['vias'] for q in places) or any(
            LineString([a,b]).distance(Point(q))<=radius for t in r['tracks'] for a,b in zip(t['pts'][:-1],t['pts'][1:]) for q in places)
    G['_nah_an']=near
    log=loese_vias(G)
    if log['uebernommen']:
        cf=G['exact_conflicts']();ts,vs=G['ausgabe_geometrie']()
        result=dict(source,**G['ausgabe_daten'](ts,vs,{u:len(v) for u,v in cf.items()},roh=True))
    else: result=source
    result['optionen']=source['optionen'];result['restkonflikte']=log
    budget=max(0,float(G['args'].get('rest_cpu','30'))-log['cpu_s'])
    if G['CPU_MAX']>0: budget=min(budget,max(0,G['CPU_MAX']-time.process_time()))
    if G['args'].get('rest_teile','1')=='1' and G['args'].get('klassen'):
        with tempfile.TemporaryDirectory(prefix='autorouter_r45_only_') as td:
            d=Path(td);rp=d/'routes.json';bp=d/'board.json'
            rp.write_text(json.dumps(result));bp.write_text(Path(G['sys'].argv[1]).read_text())
            rp.with_name('routes_konflikte.json').write_text(json.dumps(G.get('PAIRS',[])))
            parts=teile_nachlauf(bp,G['args']['klassen'],rp,budget)
            result=json.loads(rp.read_text());result['rest_teile']=parts
    result=G['rest_abschluss'](result)
    dest=Path(G['OUT']);dest.write_text(json.dumps(result))
    if result.get('rest_board'):
        dest.with_name(dest.stem+'_rest_board.json').write_text(json.dumps(result['rest_board']))
        dest.with_name(dest.stem+'_rest_placement.json').write_text(json.dumps(result['rest_placement']))
    print('R45 Reparatur: %d Via-Zuege, %d Teil-Zuege' % (log['uebernommen'],result.get('rest_teile',{}).get('uebernommen',0)),flush=True)



def start_nach_teilzug(B,R,ref):
    ""
    plane=set(R.get('flaechennetze') or [])
    remove_tracks=set();remove_vias=set()
    vd=float((R.get('optionen') or {}).get('via','0.45/0.20').split('/')[0])
    for net in sorted(plane):
        own=[p for p in B['pads'] if p['ref']==ref and p['net']==net]
        if not own: continue
        nodes=[]
        for i,t in enumerate(R['tracks']):
            if t['net']==net:
                geo=unary_union([LineString([a,b]).buffer(w/2) for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w'])])
                nodes.append(('t',i,{t['layer']:geo}))
        layers=['F']+['In%d'%i for i in range(1,B.get('lagen',2)-1)]+['B']
        for i,v in enumerate(R['vias']):
            if v['net']==net:nodes.append(('v',i,{l:Point(v['x'],v['y']).buffer(vd/2) for l in layers}))
        def touches_pad(node,p):
            return any(l in node[2] and any(node[2][l].intersects(Polygon(q)) for q in qs) for l,qs in p['polys'].items())
        connected={i for i,n in enumerate(nodes) if any(touches_pad(n,p) for p in own)}; queue=list(connected)
        while queue:
            a=queue.pop()
            for j,n in enumerate(nodes):
                if j in connected:continue
                if any(l in n[2] and g.intersects(n[2][l]) for l,g in nodes[a][2].items()): connected.add(j);queue.append(j)
        if any(touches_pad(nodes[j],p) for j in connected for p in B['pads'] if p['net']==net and p['ref']!=ref):return None
        remove_tracks.update(nodes[j][1] for j in connected if nodes[j][0]=='t')
        remove_vias.update(nodes[j][1] for j in connected if nodes[j][0]=='v')
    out=copy.deepcopy(R);out['tracks']=[t for i,t in enumerate(out['tracks']) if i not in remove_tracks];out['vias']=[v for i,v in enumerate(out['vias']) if i not in remove_vias]
    return out


def deckungs_schutz(B,R0,R1,K):
    ""
    from analyse.r41 import messen
    ground=set(K.get('gnd') or [])
    zones=[z for z in R0.get('zones',[]) if z.get('net') in ground]
    if not zones or any(z.get('filled') is not True or z.get('holes') for z in zones):return dict(ok=False,grund='native Fuellung fehlt/hat Loecher')
    for z in zones:
        g=Polygon(z['pts'])
        if not g.is_valid or g.symmetric_difference(g.convex_hull).area>1e-10:return dict(ok=False,grund='konkave Insel braucht geodaetischen Nachweis')
        vv=[v for v in R1['vias'] if v['net']==z['net'] and g.covers(Point(v['x'],v['y']))]
        if not vv:return dict(ok=False,grund='K2: Insel ohne Via')
        circles=[Point(v['x'],v['y']).buffer(1.5,quad_segs=32) for v in vv]
        if g.difference(unary_union(circles)).area>1e-10:return dict(ok=False,grund='K3: nicht vollstaendig innerhalb 1,5 mm')
        if len(vv)==1 and g.difference(Point(vv[0]['x'],vv[0]['y']).buffer(2,quad_segs=32)).area>1e-10:return dict(ok=False,grund='K2: grosse Insel braucht zwei Vias')
    a,b=messen(B,R0,K),messen(B,R1,K)
    if not a.get('fuellung_gemessen') or not b.get('fuellung_gemessen'):return dict(ok=False,grund='R41-Fuellnachweis fehlt')
    if b['fehler_mm']>a['fehler_mm']+1e-6:return dict(ok=False,grund='R41 schlechter')
    return dict(ok=True,grund='K2/K3 kontinuierlich und R41 gehalten')


def massepad_zahlen(B,R,K):
    ""
    grouped={}
    for p in B['pads']:
        if p['net'] not in K['gnd'] or K['teile'].get(p['ref'],{}).get('rolle')!='chip':continue
        box=p['bbox']
        if min(box[2]-box[0],box[3]-box[1])<1:continue
        key=(p['ref'],p['num'],p['net'])
        grouped.setdefault(key,[]).extend(Polygon(q) for qs in p['polys'].values() for q in qs)
    out={}
    for key,geos in grouped.items():
        g=unary_union(geos).buffer(.000001)
        out[key]=sum(v['net']==key[2] and g.covers(Point(v['x'],v['y'])) for v in R['vias'])
    return out



def pose(f):
    return {k:f[k] for k in ('x','y','rot','side')}


def placement_marker(B0,B1):
    ""
    basis=copy.deepcopy(B0.get('wander_basis') or {f['ref']:pose(f) for f in B0['fps']})
    history=copy.deepcopy(B0.get('wander_poses') or {r:[q] for r,q in basis.items()})
    for f in B0['fps']+B1['fps']:
        q=pose(f);h=history.setdefault(f['ref'],[])
        if q not in h:h.append(q)
    B1['wander_basis']=basis;B1['wander_poses']=history
    changed={f['ref']:f for f in B1['fps'] if pose(f)!=basis[f['ref']]}
    source={r:basis[r] for r in changed};moves={};parts={}
    for r,f in changed.items():
        q=source[r];th=(q['rot']-f['rot'])%360 if q['side']==f['side'] else (180-q['rot']-f['rot'])%360
        moves[r]=(f['x']-q['x'],f['y']-q['y'],th)

        ps=[p for p in B1['pads'] if p['ref']==r]
        parts[r]=dict(side=f['side'],x=f['x'],y=f['y'],drehung=th,transformation_eindeutig=True,
            pads={p['num']:[p['x'],p['y']] for p in ps},
            padinstanzen=[dict(num=p['num'],net=p['net'],x=p['x'],y=p['y'],side='FB' if p.get('drill',0) or len(p['layers'])>1 else f['side']) for p in ps])
    return dict(rest_board=B1,rest_quelle=source,rest_native_sources={r:history[r] for r in changed},rest_moves=moves,rest_placement=dict(parts=parts))
