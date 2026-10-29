import json, math, os, re, sys
from pathlib import Path

import shapely
from shapely.errors import GEOSException
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

CURRENT_KINDS = ('current_val', 'supply', 'sw')
QS = 32
TOL = 0.008


def _via_ground(R):
    try: d, b = [float(x) for x in str((R.get('options') or {}).get('via') or '0.45/0.20').split('/')]
    except ValueError: d, b = 0.45, 0.20
    return d, b


def _gnd_layers(R, gnd_layers):
    gl = gnd_layers if gnd_layers is not None else (R.get('gnd_layers') or (R.get('options') or {}).get('gnd_layers') or [])
    gl = gl if isinstance(gl, list) else str(gl).split(',')
    return [str(q).split('=')[0] for q in gl if q]


def _layers(B):
    nl = int(B.get('cu_layers') or 2)
    return ['F'] + ['In%d' % i for i in range(1, nl - 1)] + ['B']


def _parts(B, R, L):
    from common.zones import via_geometry
    R = via_geometry(R); out = []
    for t in R.get('tracks', []):
        if t['layer'] != L or len(t['pts']) < 2: continue
        from learning.copper_bars import track_geometry
        out.append((t.get('net', ''), track_geometry(t)))
    for v in R.get('vias', []):
        if L in (v.get('layers') or _layers(B)):
            out.append((v.get('net', ''), Point(v['x'], v['y']).buffer(v['diameter'] / 2, QS)))
    for p in B['pads']:
        for q in (p.get('polys') or {}).get(L, []):
            if len(q) >= 3: out.append((p['net'], Polygon(q).buffer(0)))
    for z in R.get('zone_list', []):
        if z['layer'] == L: out.append((z['net'], _geo(z)))
    for z in R.get('zones', []):
        if z['layer'] == L and z.get('filled'):
            out.append((z['net'], Polygon(z['pts'], z.get('holes', [])).buffer(0)))
    return out


def _geo(z):
    return unary_union([Polygon(p['outer'], p.get('inner') or []).buffer(0) for p in z['polys']])


def mask_clearance(B, n, L, clr, qs=QS):
    if L not in ('F', 'B'): return Polygon()
    design = B.get('design') or {}
    web = design.get('mask_web')
    if not isinstance(web, (int, float)) or not math.isfinite(web): return Polygon()
    own = {n} if isinstance(n, str) else set(n)
    out = []
    for p in B['pads']:
        if p['net'] in own or p.get('graphic') or p['ref'].startswith(('NTBAR_', 'CU_')): continue
        mk = (p['mask'] or {}).get(L) if 'mask' in p else design.get('mask')
        if not isinstance(mk, (int, float)) or not math.isfinite(mk): continue
        spacing = max(clr, mk + max(0., web))
        if spacing <= clr: continue
        out.extend(Polygon(q).buffer(0).buffer(spacing + .003, qs)
                   for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3)
    return unary_union(out) if out else Polygon()


def mask_free(vacant, B, n, L, clr, qs=QS):
    keepout = mask_clearance(B, n, L, clr, qs)
    return vacant.difference(keepout) if not keepout.is_empty else vacant


def keepout_area(B, L, ban):
    return unary_union([Polygon(q, (k.get('hole_list') or [[]]*len(k['polys']))[i]).buffer(0)
        for k in B.get('keepouts', []) if L in (k.get('cu_layers') or _layers(B)) and (k.get('prohibitions') or {}).get(ban)
        for i,q in enumerate(k.get('polys', [])) if len(q)>=3])


def _polys(g):
    out = []
    for p in sorted(getattr(g, 'geoms', [g]), key=lambda p: -p.area):
        if p.is_empty or p.geom_type != 'Polygon': continue
        out.append(dict(outer=[[round(x, 6), round(y, 6)] for x, y in p.exterior.coords],
                        inner=[[[round(x, 6), round(y, 6)] for x, y in i.coords] for i in p.interiors]))
    return out


def net_current(K, n):
    from analysis.measures import _current
    if (K.get('net_list') or {}).get(n, {}).get('current_A') is not None or (K.get('net_list') or {}).get(n, {}).get('flow'):
        return _current(n, K)[0]
    v = (K.get('net_list') or {}).get(n) or {}
    for k in ('current_val', 'amps_a', 'I', 'ampere'):
        if isinstance(v.get(k), (int, float)): return float(v[k])
    st = (K.get('current_val') or {}).get(n)
    if isinstance(st, (int, float)): return float(st)
    if isinstance(st, dict): return float(sum(x for x in st.values() if isinstance(x, (int, float))))
    return 1.0 if v.get('category') == 'sw' else 0.5


def precedence(B, R, K):
    from analysis.copper_model import ir_values
    from analysis.measures import voltage_with_source, budgets
    physics = ir_values(B, dict(R, current_zones_model='r36-dc-v1'), K)
    result = {}; dc_budgets, _ = budgets(K)
    for n in current_nets(K, set(K.get('gnd') or ['GND'])):
        u, _ = voltage_with_source(n, K)
        budget = dc_budgets.get(n, max(10., 10*abs(u)) if u else 30.)
        drop = physics['max_drop_mV'].get(n)
        result[n] = dict(current_A=net_current(K, n), drop_mV=drop, budget_mV=budget,
                         weight=net_current(K, n)*(1+(drop/budget if drop is not None else 1)))
    return result


def tangential_rounds(g, core_set, vacant, radius=.4):
    import numpy as np
    from shapely.geometry.polygon import orient
    safe = vacant.buffer(2e-7)
    core = core_set.buffer(-2e-7)
    candidates = []
    for poly in getattr(g, 'geoms', [g]):
        if poly.geom_type != 'Polygon': continue
        poly = orient(poly, sign=1)
        for ring in [poly.exterior, *poly.interiors]:
            xy = np.asarray(ring.coords[:-1])
            for a,b,c in zip(np.roll(xy,1,axis=0),xy,np.roll(xy,-1,axis=0)):
                la,lc=np.linalg.norm(a-b),np.linalg.norm(c-b)
                if min(la,lc)<1e-8: continue
                ua,uc=(a-b)/la,(c-b)/lc
                turn=float(ua[1]*uc[0]-ua[0]*uc[1])
                angle=math.acos(float(np.clip(ua@uc,-1,1)))
                if abs(math.pi-angle)<math.radians(8) or angle<1e-4: continue
                r=min(radius,.44*min(la,lc)*math.tan(angle/2))
                while r>=.0025:
                    d=r/math.tan(angle/2)
                    ta,tb=b+ua*d,b+uc*d
                    bisector=(ua+uc)/np.linalg.norm(ua+uc)
                    center=b+bisector*r/math.sin(angle/2)
                    sa=math.atan2(*(ta-center)[::-1]);ea=math.atan2(*(tb-center)[::-1])
                    delta=(ea-sa)%(2*math.pi) if turn>0 else -((sa-ea)%(2*math.pi))
                    steps=max(4,math.ceil(abs(delta)*r/.004),math.ceil(abs(delta)/math.radians(2)))
                    arc=[center+r*np.array([math.cos(t),math.sin(t)]) for t in np.linspace(sa,sa+delta,steps+1)]
                    wedge=Polygon([b,ta,*arc,tb,b]).buffer(0)
                    if turn>0:
                        ok=wedge.intersection(core).area<1e-10
                    else:
                        ok=safe.covers(wedge)
                    if ok:
                        candidates.append((turn,wedge));break
                    r*=.5
    remove=unary_union([w for turn,w in candidates if turn>0])
    add=unary_union([w for turn,w in candidates if turn<0])
    return g.difference(remove).union(add).buffer(0)


I_ZONE = 0.2


def _current_known(v):
    fl = v.get('flow') or {}
    if fl.get('unit') == 'A' and fl.get('total') is not None: return float(fl['total'])
    return float(v['current_A']) if v.get('current_A') is not None else None


def current_nets(K, gnd, i_zone=I_ZONE):
    N = K.get('net_list') or {}
    return sorted((n for n, v in N.items() if n not in gnd and
                   (v.get('category') in CURRENT_KINDS or (_current_known(v) or 0.0) >= i_zone)),
                  key=lambda n: (-net_current(K, n), n))


def measurement_branches(B, R, K, L, n):
    from common.flow import analysis
    ts=[t for t in R['tracks'] if t['net']==n]
    ps=[p for p in B['pads'] if p['net']==n]



    sense=re.compile(r'SENS|SNS|KELVIN|\bFB\b|FEEDBACK|REMOTE', re.I)
    tap_r=K.get('taps') or {}
    if not (tap_r.get(n) if isinstance(tap_r, dict) else n in tap_r) and \
            not any(sense.search(str(p.get('pinfunc') or '') + ' ' + str(p.get('pintype') or '')) for p in ps):
        return Polygon(), Polygon()
    f=K['net_list'][n].get('flow') or {}
    total=sum(f.get('load',{}).values())
    quiet_rel=float((R.get('r36_protection') or {}).get(n,.001))
    if not total or (quiet_rel<=.001 and not any(0<v<total*.001 for v in f.get('load',{}).values())):return Polygon(),Polygon()
    info=analysis(ts,[v for v in R['vias'] if v['net']==n],ps,f,endpoints=True)
    if not info:return Polygon(),Polygon()



    import numpy as np
    from scipy.spatial import cKDTree
    from codex.zones_ir.solver_fast import geometry,Grid,ModelError
    from analysis.measures import structure
    rr=dict(R,tracks=[{k:v for k,v in t.items() if k!='items'} for t in R['tracks']])
    layers=_layers(B);vd,drill=_via_ground(R)
    polys,padmap,barrels,_,_=geometry(B,rr,n,layers,vd)
    polys={l:g.simplify(.001,preserve_topology=True) for l,g in polys.items()}
    A=structure(len(layers),_gnd_layers(R,None))
    stack=B.get('stackup') or {l:dict(copper_um=v['t']*1000,z_mm=v['z']) for l,v in A['cu_layers'].items()}
    try:
        grid=Grid(polys,padmap,barrels,layers,.05,stack,via_drill=drill)
        field,state=grid.solve(f.get('source',[]),f.get('load',{}))
        ix=np.array([i for i,l in enumerate(grid.node_layers) if l==L]);tree=cKDTree(grid.coords[ix]) if len(ix) else None
        small=max(float(np.nanmax(state['u']))*quiet_rel,1e-9)
        protected={key for key,v in field['drop_V'].items() if quiet_rel>.001 and v<small}
        protected_share=max([float(f['load'][key])/total for key in protected] or [0.])
        node_current=np.zeros(grid.n)
        np.maximum.at(node_current,grid.a,np.nan_to_num(np.abs(state['current'])))
        np.maximum.at(node_current,grid.b,np.nan_to_num(np.abs(state['current'])))
    except ModelError:
        tree=None;protected=set();protected_share=0.
    contacts=unary_union([Polygon(q) for p in ps for q in p['polys'].get(L,[])])
    shapely.prepare(contacts)
    pieces=[];definite=[];skin=.005 if quiet_rel>.001 else .04
    share_limit=max(.001,protected_share+1e-9)
    for ti,t in enumerate(ts):
        if t['layer']!=L:continue
        shares=[info['share'].get((ti,si),0) for si in range(len(t['w']))]
        private=bool(shares) and 0<max(shares)<=share_limit
        for si,(a,b,w) in enumerate(zip(t['pts'][:-1],t['pts'][1:],t['w'])):
            share=info['share'].get((ti,si),0)
            quiet=False;current_weak=True
            if tree is not None:
                sample=np.linspace(a,b,max(2,math.ceil(math.dist(a,b)/.1)+1))
                dist,ids=tree.query(sample);u=state['u'][ix[ids]]
                outside=~shapely.intersects_xy(contacts,sample[:,0],sample[:,1])
                current_weak=bool(np.all(dist<.2) and (not np.any(outside) or np.max(node_current[ix[ids[outside]]])<total*.001))
                quiet=bool(np.all(dist<.2) and np.all(np.isfinite(u)) and np.max(u)<small and (quiet_rel>.001 or current_weak))
            branch=(private or 0<share<=share_limit) and (quiet_rel>.001 or current_weak)
            if quiet or branch:
                h=LineString([a,b]).buffer(w/2+skin,QS);pieces.append(h)
                if branch:definite.append(h)



    if quiet_rel>.001 and tree is not None:
        for v in R['vias']:
            if v['net']!=n or L not in v.get('layers',layers):continue
            distance,vid=tree.query([v['x'],v['y']])
            if distance<.3 and np.isfinite(state['u'][ix[vid]]) and state['u'][ix[vid]]<small:
                h=Point(v['x'],v['y']).buffer(v.get('d',v.get('diameter',vd))/2+skin,QS)
                pieces.append(h);definite.append(h)
    for p in ps:
        load=f.get('load',{}).get(p['ref']+':'+str(p['num']),0)
        if 0<load<info['total']*.001 or p['ref']+':'+str(p['num']) in protected:
            hs=[Polygon(q).buffer(skin,QS) for q in p['polys'].get(L,[])]
            pieces.extend(hs);definite.extend(hs)
    anchors=unary_union([Polygon(q).buffer(.005,QS) for p in ps if p['ref']+':'+str(p['num']) in f.get('source',[]) for q in p['polys'].get(L,[])])
    own=unary_union(pieces).difference(anchors)
    certain=unary_union(definite).difference(anchors)



    own=unary_union([part for part in getattr(own,'geoms',[own]) if part.intersects(certain)])



    own=own.union(unary_union([Polygon(q).buffer(skin,QS) for p in ps for q in p['polys'].get(L,[])
        if p['ref']+':'+str(p['num']) not in f.get('source',[]) and own.intersects(Polygon(q))]))
    anchors=unary_union([Polygon(q).buffer(.005,QS) for p in ps if p['ref']+':'+str(p['num']) in f.get('source',[]) for q in p['polys'].get(L,[])])
    return own,own.buffer(.15,QS).difference(anchors)


def _groups_hull(pads, spacing, local=False):
    if local:


        return [a.union(b).convex_hull for i,a in enumerate(pads)
                for b in pads[i+1:] if a.distance(b) <= spacing]
    n = len(pads); parent = list(range(n))
    def f(i):
        while parent[i] != i: parent[i] = parent[parent[i]]; i = parent[i]
        return i
    for i in range(n):
        for j in range(i + 1, n):
            if pads[i].distance(pads[j]) <= spacing: parent[f(i)] = f(j)
    gr = {}
    for i in range(n): gr.setdefault(f(i), []).append(pads[i])
    return [unary_union(g).convex_hull for g in gr.values() if len(g) >= 2]


def _segment_hulls(B, RR, L, n, grow):
    from common.zones import via_geometry
    RR = via_geometry(RR)
    pads = [Polygon(q).buffer(0) for p in B['pads'] if p['net'] == n for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3]
    vias = [Point(v['x'], v['y']).buffer(v['diameter'] / 2, QS)
            for v in RR.get('vias', []) if v.get('net') == n and L in (v.get('layers') or _layers(B))]
    out = []
    for t in RR.get('tracks', []):
        if t['layer'] != L or t.get('net') != n: continue
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            ends = []
            for q in (a, b):
                P = Point(q); e = P.buffer(w / 2, QS)
                for g in pads + vias:
                    if g.distance(P) < 1e-3: e = e.union(g)
                ends.append(e.buffer(grow, QS))
            out.append(unary_union(ends).convex_hull)
    return out


_RICHT = [(math.cos(math.radians(45 * k)), math.sin(math.radians(45 * k))) for k in range(8)]
def _octagon(g, border=0.0):
    import numpy as np
    pts = np.asarray(g.convex_hull.exterior.coords if g.convex_hull.geom_type == 'Polygon' else g.convex_hull.coords)
    U = np.asarray(_RICHT); c = (pts @ U.T).max(axis=0) + border
    corners = []
    for k in range(8):
        (a1, b1), (a2, b2) = U[k], U[(k + 1) % 8]; det = a1 * b2 - a2 * b1
        corners.append(((c[k] * b2 - c[(k + 1) % 8] * b1) / det, (a1 * c[(k + 1) % 8] - a2 * c[k]) / det))
    return Polygon(corners).buffer(0)


def _segment_octagons(B, RR, L, n, grow):
    from common.zones import via_geometry
    RR = via_geometry(RR)
    pads = [Polygon(q).buffer(0) for p in B['pads'] if p['net'] == n for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3]
    vias = [Point(v['x'], v['y']).buffer(v['diameter'] / 2, 8)
            for v in RR.get('vias', []) if v.get('net') == n and L in (v.get('layers') or _layers(B))]
    out = []
    for t in RR.get('tracks', []):
        if t['layer'] != L or t.get('net') != n: continue
        ws = t['w']
        for i, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], ws)):
            ends = []
            for j, q in enumerate((a, b)):

                width = w
                if RR.get('k20_corridor'):
                    neighbor = i-1 if j == 0 else i+1
                    if 0 <= neighbor < len(ws): width = max(w, ws[neighbor])
                P = Point(q); e = P.buffer(width / 2, 8)
                for g in pads + vias:
                    if g.distance(P) < 1e-3: e = e.union(g)
                ends.append(e)
            if RR.get('k20_corridor'):


                out.append(unary_union([_octagon(e, grow) for e in ends]).convex_hull)
                continue

            dx, dy = b[0] - a[0], b[1] - a[1]; ang = math.degrees(math.atan2(dy, dx)) % 45
            if math.hypot(dx, dy) < 1e-6 or min(ang, 45 - ang) < 3: out.append(_octagon(unary_union(ends), grow))
            else: out += [_octagon(e, grow) for e in ends]
    return out


def _close_g(g, r):
    return g.buffer(r, join_style=2, mitre_limit=2.0).buffer(-r, join_style=2, mitre_limit=2.0)


def _open_g(g, r):
    return g.buffer(-r, join_style=2, mitre_limit=2.0).buffer(r, join_style=2, mitre_limit=2.0)


def _close(g, r):
    return g.buffer(r, QS).buffer(-r, QS)


def _open(g, r):
    return g.buffer(-r, QS).buffer(r, QS)


PROTECTION = ('link_chain', 'tap', 'kelvin', 'sig')

def track_protection_on(R):
    o = R.get('options') or {}
    return str(o.get('track_protection', '0')).lower() in ('1', 'true') or str(o.get('r39', '0')) == '1'


def _track_protection(R, L, n):
    from learning.copper_bars import track_geometry
    tracks = [t for t in R.get('tracks', []) if t.get('net') == n and t['layer'] == L
              and any(t.get(k) for k in PROTECTION)]
    copper = unary_union([track_geometry(t) for t in tracks])
    clearance = float(R.get('clr') or (R.get('options') or {}).get('clr') or .1)
    return copper, copper.buffer(clearance + .005, QS)


def _zones_copper(R, n, L):
    return unary_union([_geo(q) for q in R.get('zone_list', []) if q['net'] == n and q['layer'] == L] +
                       [Polygon(q['pts'], q.get('holes') or []).buffer(0) for q in R.get('zones', [])
                        if q['net'] == n and q['layer'] == L and q.get('filled')])


def _protected_zones(B, R, zones):
    out = []; part_list = {}
    for z in zones:
        copper, moat = _track_protection(R, z['layer'], z['net'])
        if copper.is_empty:
            out.append(z); continue
        if z['layer'] not in part_list: part_list[z['layer']] = _parts(B, R, z['layer'])
        seed = unary_union([g for n, g in part_list[z['layer']] if n == z['net']]).difference(moat)
        if seed.is_empty: continue

        existing = _zones_copper(R, z['net'], z['layer'])
        g = _geo(z).difference(moat).union(_geo(z).intersection(copper.union(existing)))
        g = unary_union([q for q in getattr(g, 'geoms', [g]) if q.intersects(seed)])
        if g.is_empty: continue
        out.append(dict(z, polys=_polys(g), area_mm2=round(g.area, 2)))
    return out








SKELETON = None


def skeleton_apply(R, zs):
    from learning.current_zones_topo import skeleton_apply as _a
    return _a(R, zs)


def skeleton(R):
    w = R.get('r36_skeleton', SKELETON) if isinstance(R, dict) else SKELETON
    return float(w) if w else None


def zone_list(B, R, K, gnd_layers=None, clr=None, border=0.25, r_smoothed=0.6, r_free=0.2, r_open=0.2, group=None, min_area=0.05,
          min_track=1.0, priority=None, measurement_branch=True, compact=True, shaping=None, zones_judge=None):
    from learning.zones_judge_selection import an, choose
    if an(R, zones_judge):
        return choose(B, R, K, gnd_layers, clr=clr, border=border, r_smoothed=r_smoothed, r_free=r_free,
            r_open=r_open, group=group, min_area=min_area, min_track=min_track,
            priority=priority, measurement_branch=measurement_branch, compact=compact, shaping=shaping)
    shaping = shaping or (R.get('r36_shape') or 'topo')
    from analysis.switch_node import an as _sw_on
    if _sw_on(R) and shaping != 'topo':
        raise ValueError("sw_compact=1 is only implemented for shaping='topo'")
    if str(R.get('r36_k17', '0')).lower() in ('1', 'true') and shaping != 'topo':
        raise ValueError("r36_k17=1 is only implemented for shaping='topo'")
    if R.get('r36_corridor') and shaping == 'topo':
        raise ValueError("r36_corridor=1 is not implemented with shaping='topo'")
    if str((R.get('options') or {}).get('r39', '0')) == '1' and shaping != 'straight':
        raise ValueError("r39=1 is only implemented for shaping='straight' (r36_shape='straight')")
    if shaping == 'topo':



        from learning.current_zones_topo import zone_list as zones_topo
        zones = zones_topo(B, R, K, gnd_layers, clr=clr, border=border, kappa=1.0, skeleton=skeleton(R),
                           only_area=float(R.get('r36_only_area', 0.2)),
                           lam_min_mw=float(R.get('r36_lam_min', 0.0)),
                           lam_max_mw=float(R.get('r36_lam_max', 2.0)))
        return _protected_zones(B, R, zones) if track_protection_on(R) and any(any(t.get(k) for k in PROTECTION) for t in R.get('tracks', [])) else zones
    if shaping == 'minimax':



        from learning.current_zones_minimax import zone_list as zones_minimax
        zones = zones_minimax(B, R, K, gnd_layers, clr=clr, border=border)
        return _protected_zones(B, R, zones) if track_protection_on(R) and any(any(t.get(k) for k in PROTECTION) for t in R.get('tracks', [])) else zones
    corridor = bool(R.get('r36_corridor'))
    gnd = set(K.get('gnd') or ['GND']); gl = set(_gnd_layers(R, gnd_layers))
    clr = clr if clr is not None else max(0.15, float(R.get('clr') or (R.get('options') or {}).get('clr') or 0.1))
    contour = unary_union([Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3]).buffer(-border, QS)
    _, b_via = _via_ground(R)
    hole_list = [(v.get('net', ''), Point(v['x'], v['y']).buffer(b_via / 2, QS)) for v in R.get('vias', [])]
    hole_list += [(p['net'], Point(p['x'], p['y']).buffer(p['drill'] / 2, QS)) for p in B['pads'] if p.get('drill')]
    priority = priority if priority is not None else precedence(B, R, K)
    net_list = sorted(current_nets(K, gnd), key=lambda n:(-priority[n]['weight'], n))
    RR = dict(R, zone_list=list(R.get('zone_list') or [])); out = []
    protection = track_protection_on(RR)
    for L in _layers(B):
        if L in gl: continue
        for n in net_list:
            part_list = _parts(B, RR, L)
            owned = unary_union([g for m, g in part_list if m == n])
            if owned.is_empty: continue
            protected, moat = _track_protection(RR, L, n) if protection else (Polygon(), Polygon())
            growth = dict(RR, tracks=[t for t in RR.get('tracks', [])
                            if not (t.get('net') == n and t['layer'] == L and
                                    any(t.get(k) for k in PROTECTION))]) if protection else RR
            seed = owned.difference(moat) if not protected.is_empty else owned
            if seed.is_empty: continue
            segs = [(a, b, w) for t in growth.get('tracks', []) if t['layer'] == L and t.get('net') == n
                    for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w'])]
            tracks_pads = unary_union([(LineString([a, b]) if a != b else Point(a)).buffer(w / 2, QS) for a, b, w in segs] +
                                      [Polygon(q).buffer(0) for p in B['pads'] if p['net'] == n for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3])
            if tracks_pads.is_empty: continue
            pads = [Polygon(q).buffer(0) for p in B['pads'] if p['net'] == n for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3]
            if len(pads) < 2 and not segs: continue
            foreign = unary_union([g for m, g in part_list if m != n] + [g for m, g in hole_list if m != n])
            keepout = keepout_area(B, L, 'areas')
            free_pad = contour.difference(foreign.buffer(clr + .003, QS)).difference(keepout)
            free_pad = mask_free(free_pad, B, n, L, clr)
            hole = float((R.get('options') or {}).get('h2h') or .2)
            free_pad = free_pad.difference(unary_union([h.buffer(hole+.003, QS) for m,h in hole_list if m != n]))
            if not protected.is_empty: free_pad = free_pad.difference(moat)
            vacant = _open(free_pad, float((R.get('r36_free') or {}).get(n,r_free)))
            I = net_current(K, n)
            if compact:


                grow = min(0.6, max(0.2, 0.15 + 0.1 * I)); rich = 1.0; grp = 1.5 if group is None else group
            else:
                grow = min(1.5, max(0.5, 0.4 + 0.25 * I))
                rich = min(3.5, 2.0 + 0.4 * I)
                grp = 4.0 if group is None else group



            pieces_e = [s for s in getattr(seed, 'geoms', [seed]) if not s.is_empty]
            hulls = [] if compact else [s.convex_hull.intersection(s.buffer(rich, QS)) for s in pieces_e]
            if shaping == 'straight' and compact:


                from common.zones import via_geometry
                vias_n = [Point(v['x'], v['y']).buffer(v['diameter'] / 2, 8)
                          for v in via_geometry(RR)['vias'] if v.get('net') == n and L in (v.get('layers') or _layers(B))]
                cand = unary_union([_octagon(p, grow) for p in pads + vias_n] + [_octagon(h, grow) for h in _groups_hull(pads, grp, local=corridor)] +
                                   _segment_octagons(B, growth, L, n, grow))
                z = _open_g(cand.intersection(vacant), r_open)
                z = unary_union([z, owned])
                z = _close_g(z, r_smoothed).intersection(unary_union([vacant, owned]))
            else:
                cand = unary_union([seed.buffer(grow, QS)] + hulls + _groups_hull(pads, grp, local=corridor) + _segment_hulls(B, growth, L, n, grow))
                z = _open(cand.intersection(vacant), r_open)
                z = unary_union([z, owned])
                z = _close(z, r_smoothed).intersection(unary_union([vacant, owned]))
            pieces = [s for s in getattr(z, 'geoms', [z]) if s.geom_type == 'Polygon' and s.area >= min_area
                       and s.buffer(1e-3).intersects(tracks_pads)]
            if not pieces: continue
            g = unary_union(pieces + [owned] + [p.buffer(.04, QS).intersection(free_pad) for p in pads]).buffer(0)


            W = min(2.0, float((K.get('net_list', {}).get(n) or {}).get('w') or 0.3))
            thin = g.difference(g.buffer(-W / 2 + 1e-4, QS).buffer(W / 2 - 1e-4, QS))
            if not thin.is_empty and shaping == 'straight' and compact:


                corr = [LineString([a, b]).buffer(W / 2, cap_style=2, join_style=2) for a, b, w in segs
                        if a != b and w < W and LineString([a, b]).intersects(thin.buffer(1e-3))]
                if corr: g = g.union(unary_union(corr).intersection(free_pad)).buffer(0)
            elif not thin.is_empty: g = g.union(thin.buffer(W / 2, QS).intersection(free_pad)).buffer(0)
            if R.get('r36_smoothed'):
                from learning.r36_shape import transitions
                g = g.union(transitions(RR, L, n, grow).intersection(free_pad)).buffer(0)




            safe = free_pad.union(owned)
            g = g.buffer(.02, QS).intersection(safe).simplify(.0005, preserve_topology=True)
            g = _open(_close(g,.01),.01).intersection(safe)
            g = g.intersection(safe).union(owned)
            for _ in range(2):
                g = tangential_rounds(g.simplify(.002,preserve_topology=True).union(owned), owned, free_pad)
            if measurement_branch:
                sense,moat=measurement_branches(B,R,K,L,n)
                g=g.difference(moat).union(sense.intersection(free_pad)).union(owned)
                g=tangential_rounds(g.simplify(.0005,preserve_topology=True).union(owned),owned,free_pad)
            if corridor:



                hem = max(.75, float((K.get('net_list',{}).get(n) or {}).get('w') or .3)/2)
                g = g.intersection(owned.buffer(hem, QS)).union(owned).buffer(0)




            if str((R.get('options') or {}).get('r39', '0')) == '1':
                g = r39_corridor(B, RR, K, L, n, g, owned, free_pad, measurement_branch)
            zz = dict(net=n, layer=L, source='current_zones', amps_a=round(I, 3), polys=_polys(g),
                      area_mm2=round(g.area, 2), owned_mm2=round(owned.area, 2), precedence=priority[n])
            out.append(zz); RR['zone_list'].append(zz)
    return out


def r39_corridor(B, R, K, L, n, g, owned, vacant, measurement_branch=True):
    from common.arc import points
    W = float(K['net_list'][n].get('w') or .3)
    if not math.isfinite(W) or W <= 0: raise ValueError('R39: invalid target width ' + n)
    paths = []
    for t in R.get('tracks', []):
        if t.get('net') != n or t['layer'] != L or any(t.get(k) for k in ('tap', 'link_chain', 'kelvin', 'sig')): continue
        if 'items' in t:
            for item in t['items']:
                ps = item[1:3] if item[0] == 'L' else points(*item[1:4])
                paths.append(LineString(ps).buffer(W / 2, QS))
        else:
            for a, b in zip(t['pts'][:-1], t['pts'][1:]):
                if a != b: paths.append(LineString([a, b]).buffer(W / 2, QS))
    if not paths: return g
    allowed = vacant
    chains, moat = _track_protection(R, L, n)
    if not chains.is_empty:

        allowed = allowed.difference(moat)
    if measurement_branch:
        _, moat = measurement_branches(B, R, K, L, n)
        allowed = allowed.difference(moat)
    add = unary_union(paths).intersection(allowed)
    add = unary_union([p for p in getattr(add, 'geoms', [add])
                       if p.geom_type == 'Polygon' and p.intersects(owned)])


    original = [p for p in getattr(g, 'geoms', [g]) if p.geom_type == 'Polygon']
    safe_add = [p for p in getattr(add, 'geoms', [add])
                if sum(p.intersects(q) for q in original) <= 1]
    return g.union(unary_union(safe_add)).buffer(0)

def fast_nets(K, from_rank=3):
    return {n for n, k in sorted((K.get('net_list') or {}).items())
            if k.get('category') == 'sw' or (k.get('category') == 'fast' and (k.get('rank') or 0) >= from_rank)}

def _geo_grow(seed, M, k):
    from scipy import ndimage
    s4, s8 = ndimage.generate_binary_structure(2, 1), ndimage.generate_binary_structure(2, 2)
    x = seed & M
    for i in range(k):
        y = ndimage.binary_dilation(x, structure=s8 if i % 2 else s4, mask=M)
        if (y == x).all(): break
        x = y
    return x


def ends_stitch(s, close, anchor, allowed, inner, keepout_vias, d_via, clr, r=1.5, r_cut=2.0, h=0.1, min_cells=4, max_new=None):
    import numpy as np
    from scipy.signal import fftconvolve
    x0, y0, x1, y1 = s.bounds
    nx, ny = int(math.ceil((x1 - x0) / h)) + 1, int(math.ceil((y1 - y0) / h)) + 1
    X, Y = np.meshgrid(x0 + h * np.arange(nx), y0 + h * np.arange(ny))
    for g in (s, close, inner, allowed): shapely.prepare(g)
    M = shapely.contains_xy(s, X, Y)
    U = M & shapely.contains_xy(close, X, Y)
    if U.sum() < min_cells: return [], None
    U2 = U.copy() if r_cut is not None else None
    OK = M & shapely.contains_xy(inner, X, Y) & shapely.contains_xy(allowed, X, Y)
    k, k2 = int(math.ceil(r / h)), int(math.ceil((r_cut or r) / h)); ks = int(math.ceil((d_via + clr) / h))
    yy, xx = np.mgrid[-k:k + 1, -k:k + 1]; disc = ((xx * xx + yy * yy) * h * h <= r * r).astype(np.float32)

    def window(ax, ay, kk):
        cx, cy = int(round((ax - x0) / h)), int(round((ay - y0) / h))
        return slice(max(0, cy - kk - 1), max(0, min(ny, cy + kk + 2))), slice(max(0, cx - kk - 1), max(0, min(nx, cx + kk + 2)))
    def cover(ax, ay, probe=False):
        n = 0
        for UU, rr, kk in ((U, r, k), (U2, r_cut, k2)) if U2 is not None else ((U, r, k),):
            a, b = window(ax, ay, kk)
            if a.start >= a.stop or b.start >= b.stop: continue
            d2 = (X[a, b] - ax) ** 2 + (Y[a, b] - ay) ** 2
            seed = M[a, b] & (d2 <= (d_via / 2 + h) ** 2)
            if not seed.any(): continue
            dk = _geo_grow(seed, M[a, b], kk) & (d2 <= rr * rr)
            if UU is U: n = int((U[a, b] & dk).sum())
            if probe: return n
            UU[a, b] &= ~dk
        return n

    def keepout_list(ax, ay):
        a, b = window(ax, ay, ks)
        if a.start < a.stop and b.start < b.stop:
            OK[a, b] &= ~(((X[a, b] - ax) ** 2 + (Y[a, b] - ay) ** 2) < (d_via + clr) ** 2)

    for ax, ay in anchor: cover(ax, ay)
    for ax, ay in keepout_vias:
        if x0 - 1 <= ax <= x1 + 1 and y0 - 1 <= ay <= y1 + 1: keepout_list(ax, ay)
    new = []
    for _ in range(2000):
        if U.sum() < min_cells or (max_new is not None and len(new) >= max_new): break
        iy, ix = np.nonzero(U)
        a = slice(max(0, iy.min() - k), iy.max() + k + 1); b = slice(max(0, ix.min() - k), ix.max() + k + 1)
        G = fftconvolve(U[a, b].astype(np.float32), disc, mode='same')
        G = np.where(OK[a, b], np.rint(G), 0)
        j = int(np.argmax(G))
        if G.flat[j] < min_cells: break
        cy, cx = divmod(j, G.shape[1]); cy += a.start; cx += b.start
        vx, vy = float(X[cy, cx]), float(Y[cy, cx])
        if cover(vx, vy, probe=True) < min_cells: OK[cy, cx] = False; continue
        cover(vx, vy); keepout_list(vx, vy); new.append((round(vx, 4), round(vy, 4)))
    if U2 is None or not U2.any(): return new, None
    iy, ix = np.nonzero(U2)
    e = h * 0.55
    rest = shapely.union_all(shapely.box(X[iy, ix] - e, Y[iy, ix] - e, X[iy, ix] + e, Y[iy, ix] + e))
    return new, rest


def _polys_from(g):
    return [q for q in getattr(g, 'geoms', [g]) if q.geom_type == 'Polygon' and not q.is_empty]


def open_pocket(s, open_items, anchor, d_via, w_max=2.0, r_end=0.8):
    way = []
    wide = s.buffer(-w_max / 2, QS).buffer(w_max / 2, QS)
    for o in sorted(_polys_from(open_items), key=lambda o: (-o.area, o.bounds)):
        if o.area < 0.05 or any(o.intersects(t) for t in way): continue
        t = None
        free_from = lambda g: all(g.distance(Point(a)) > r_end for a in anchor)
        for w in (0.3, 0.5, 0.8, 1.2, 1.6, w_max):
            next_ = [c for c in _polys_from(s.buffer(-w / 2, QS)) if c.distance(o) <= w / 2]
            if not next_: break
            if any(c.distance(Point(a)) <= w / 2 for c in next_ for a in anchor): continue
            for g in (unary_union(next_).buffer(w, QS).intersection(s), unary_union(next_).buffer(w / 2, QS).intersection(s)):
                if free_from(g) and g.area <= 4 * o.area + 4.0: t = g; break
            if t is not None: break
        if t is None and wide.intersection(o).area < 0.01:
            nearby = [a for a in anchor if Point(a).distance(o) <= 3.0]
            if nearby:
                cut = s.difference(unary_union([Point(a).buffer(r_end, QS) for a in nearby])).buffer(-0.1, QS).buffer(0.1, QS)
                t = unary_union([q for q in _polys_from(cut) if q.intersects(o)])
                if t.area > 4 * o.area + 2.0: t = None
        if t is not None and not t.is_empty and t.area >= 0.1: way.append(t)
    return unary_union(way) if way else None


GND_GRID = 1e-6


def _gnd_difference(a, logbook, b, *args, **kwargs):
    try:
        return a.difference(b, *args, **kwargs)
    except GEOSException as exc:
        if not a.is_valid or not b.is_valid:
            raise
        g = shapely.difference(a, b, grid_size=GND_GRID)
        if not g.is_valid:
            raise ValueError('Invalid GND overlay post-run') from exc
        if shapely.difference(g, a.buffer(2*GND_GRID), grid_size=GND_GRID).area > 1e-9:
            raise ValueError('GND overlay adds copper outside the original space') from exc
        if shapely.intersection(g, b.buffer(-2*GND_GRID), grid_size=GND_GRID).area > 1e-9:
            raise ValueError('GND overlay fills foreign keepout area') from exc
        match = re.search(r' at ([\d.eE+-]+) ([\d.eE+-]+)', str(exc))
        logbook.append(dict(faults=str(exc), location=[float(v) for v in match.groups()] if match else None,
                             grid_mm=GND_GRID, a_area_mm2=a.area, b_area_mm2=b.area,
                             export_area_mm2=g.area, a_valid=True, b_valid=True))
        return g


def _gnd_buffer(a, logbook, *args, **kwargs):
    eligible = a.geom_type == 'MultiPolygon' and args and args[0] > 0
    try:
        g = a.buffer(*args, **kwargs)
        if g.is_valid or not eligible or not a.is_valid: return g
        reason = shapely.is_valid_reason(g)
    except GEOSException as exc:
        if not eligible or not a.is_valid: raise
        g = None
        reason = str(exc)



    pieces = [part.buffer(*args, **kwargs) for part in a.geoms]
    if any(not part.is_valid for part in pieces):
        raise ValueError('Invalid single GND component buffer')
    result = shapely.union_all(pieces)
    if not result.is_valid:
        raise ValueError('Invalid union of the GND component buffers')
    cover = result.buffer(2*GND_GRID)
    if any(part.difference(cover).area > 1e-9 for part in pieces):
        raise ValueError('GND component buffer loses keepout area')
    match = re.search(r'(?:\[| at )([\d.eE+-]+) ([\d.eE+-]+)', reason)
    logbook.append(dict(category='components_buffer', faults=reason,
                         location=[float(v) for v in match.groups()] if match else None,
                         radius_mm=args[0], part_list=len(pieces), input_valid=True,
                         original_area_mm2=g.area if g is not None else None,
                         export_area_mm2=result.area))
    return result


def gnd_fill(B, R, K, gnd_layers=None, clr=None, border=0.25, raster=2.0, min_width=0.25, min_area=0.3,
                 r41=True, r41_r=1.5, r41_cut=2.0, r41_close=1.0, r41_bands=2.0, k3=True, k3_r=3.0, k2_round=2.0, geos_logbook=None):
    geos_log = geos_logbook if geos_logbook is not None else []
    gnd = set(K.get('gnd') or ['GND']); gn = sorted(gnd)[0]; gl = set(_gnd_layers(R, gnd_layers))
    clr = clr if clr is not None else max(0.15, float(R.get('clr') or (R.get('options') or {}).get('clr') or 0.1))
    d_via, b_via = _via_ground(R)
    contour = _gnd_buffer(unary_union([_gnd_buffer(Polygon(o), geos_log, 0) for o in B['outline'] if len(o) >= 3]), geos_log, -border, QS)
    cu_layers = [L for L in _layers(B) if L not in gl]
    hole_list = [_gnd_buffer(Point(v['x'], v['y']), geos_log, b_via / 2, QS) for v in R.get('vias', []) if v.get('net') not in gnd]
    hole_list += [_gnd_buffer(Point(p['x'], p['y']), geos_log, p['drill'] / 2, QS) for p in B['pads'] if p.get('drill') and p['net'] not in gnd]
    foreign_l, owned_l, free_l = {}, {}, {}
    for L in cu_layers:
        t = _parts(B, R, L)
        foreign_l[L] = unary_union([g for m, g in t if m not in gnd] + hole_list)
        owned_l[L] = unary_union([g for m, g in t if m in gnd])
        keepout = keepout_area(B, L, 'areas')
        f = _gnd_difference(_gnd_difference(contour, geos_log, _gnd_buffer(foreign_l[L], geos_log, clr + .003, QS)), geos_log, keepout)
        mask_keepout = mask_clearance(B, gnd, L, clr)
        if not mask_keepout.is_empty: f = _gnd_difference(f, geos_log, mask_keepout)
        f = _gnd_difference(f, geos_log, _gnd_buffer(unary_union(hole_list), geos_log, float((R.get('options') or {}).get('h2h') or .2)+.003, QS))
        free_l[L] = _gnd_buffer(_gnd_buffer(f, geos_log, -min_width / 2, QS), geos_log, min_width / 2, QS)

    all_items = unary_union([g for L in _layers(B) for n,g in _parts(B,R,L) if n not in gnd])
    via_mask_keepout = unary_union([mask_clearance(B, gnd, L, clr) for L in _layers(B)])
    hole = max(.2, float((R.get('options') or {}).get('h2h') or .2))
    bore_foreign = unary_union(hole_list)
    via_keepout = unary_union([keepout_area(B,L,'vias').union(keepout_area(B,L,'areas')) for L in _layers(B)])
    vias = []
    def via_ok(x, y):
        c = _gnd_buffer(Point(x, y), geos_log, d_via / 2 + clr, QS)
        drill = _gnd_buffer(Point(x,y), geos_log, b_via/2 + hole, QS)
        return _gnd_buffer(contour, geos_log, -d_via / 2).contains(Point(x, y)) and not c.intersects(all_items) and \
            (via_mask_keepout.is_empty or not Point(x,y).buffer(d_via/2, QS).intersects(via_mask_keepout)) and not drill.intersects(bore_foreign) and not c.intersects(via_keepout) and \
            all(Point(v['x'], v['y']).distance(Point(x, y)) >= d_via + clr for v in R.get('vias', []) + vias)
    SCH = fast_nets(K) if r41 else set()
    gnd_anchor = [(v['x'], v['y']) for v in R.get('vias', []) if v.get('net') in gnd] + \
                [(p['x'], p['y']) for p in B['pads'] if p.get('drill') and p['net'] in gnd]
    bore_all = unary_union([_gnd_buffer(Point(p['x'], p['y']), geos_log, p['drill'] / 2, QS) for p in B['pads'] if p.get('drill')] + [bore_foreign])
    allowed = None

    def allowed_fetch():
        nonlocal allowed
        if allowed is None:
            verb = [_gnd_buffer(all_items, geos_log, d_via / 2 + clr, QS), _gnd_buffer(bore_all, geos_log, b_via / 2 + hole, QS)]
            if not via_keepout.is_empty: verb.append(_gnd_buffer(via_keepout, geos_log, d_via / 2 + clr, QS))
            allowed = _gnd_difference(_gnd_buffer(contour, geos_log, -d_via / 2, QS), geos_log, unary_union(verb))
        return allowed
    zones_out = []
    for L in cu_layers:
        pieces = [s for s in getattr(free_l[L], 'geoms', [free_l[L]]) if s.geom_type == 'Polygon' and s.area >= min_area]
        keep = []
        for s in sorted(pieces, key=lambda s: -s.area):
            if not owned_l[L].is_empty and s.intersects(owned_l[L]): keep.append(s); continue
            if any(s.contains(Point(v['x'], v['y'])) for v in vias): keep.append(s); continue
            inner = _gnd_buffer(s, geos_log, -(d_via / 2 + 0.05), QS)
            if inner.is_empty: continue
            x0, y0, x1, y1 = inner.bounds; assigned = False
            ys = [y0 + (y1 - y0) / 2] if y1 - y0 < raster else [y0 + raster / 2 + i * raster for i in range(int((y1 - y0) / raster) + 1)]
            xs = [x0 + (x1 - x0) / 2] if x1 - x0 < raster else [x0 + raster / 2 + i * raster for i in range(int((x1 - x0) / raster) + 1)]
            for y in ys:
                for x in xs:
                    if inner.contains(Point(x, y)) and via_ok(x, y):
                        vias.append(dict(x=round(x, 4), y=round(y, 4), net=gn, source='gnd_fill')); assigned = True
            if not assigned:
                p = inner.representative_point()
                if via_ok(p.x, p.y): vias.append(dict(x=round(p.x, 4), y=round(p.y, 4), net=gn, source='gnd_fill')); assigned = True
            if assigned: keep.append(s)
        r41_info = None
        if r41 and SCH and keep:
            fast = unary_union([g for m, g in _parts(B, R, L) if m in SCH])
            if not fast.is_empty:
                allowed = allowed_fetch()
                close_L = _gnd_buffer(fast, geos_log, r41_close, QS); r41_info = dict(vias=0, cut_mm2=0.0)


                from learning.copper_bars import track_geometry
                track_list = [track_geometry(t) for t in R.get('tracks', []) if t['layer'] == L and t.get('net') in SCH and len(t['pts']) >= 2]
                close_cut = _gnd_buffer(unary_union(track_list), geos_log, r41_close, QS) if track_list else None
                new_b = []
                for s in keep:
                    close = s.intersection(close_L)
                    if close.area < 0.01: new_b.append(s); continue
                    sp = _gnd_buffer(s, geos_log, 0.01); shapely.prepare(sp)
                    anchor = [a for a in gnd_anchor + [(v['x'], v['y']) for v in vias] if sp.contains(Point(a))]
                    nv, rest = ends_stitch(s, close, anchor, allowed, _gnd_buffer(s, geos_log, -(d_via / 2 + 0.05), QS),
                                              [(v['x'], v['y']) for v in R.get('vias', []) + vias], d_via, clr, r41_r, r41_cut)
                    nv = [p for p in nv if via_ok(*p)]
                    vias += [dict(x=x, y=y, net=gn, source='gnd_fill', r41=1) for x, y in nv]; r41_info['vias'] += len(nv)
                    if rest is not None and not rest.is_empty:
                        rest = rest.intersection(close_cut) if close_cut is not None else None
                    if rest is not None and not rest.is_empty:
                        rest = open_pocket(s, rest, anchor + nv, d_via, r41_bands)
                    if rest is None or rest.is_empty: new_b.append(s); continue
                    t = _gnd_difference(s, geos_log, _gnd_buffer(rest, geos_log, 0.05, QS))
                    t = _gnd_buffer(_gnd_buffer(t, geos_log, -min_width / 2, QS), geos_log, min_width / 2, QS)
                    t = _gnd_buffer(_gnd_buffer(t, geos_log, min_width / 2, QS), geos_log, -min_width / 2, QS).intersection(s)
                    a_nv = anchor + nv; kept = 0.0
                    for q in getattr(t, 'geoms', [t]):
                        if q.geom_type != 'Polygon' or q.area < min_area: continue
                        qp = _gnd_buffer(q, geos_log, 0.01)
                        if (not owned_l[L].is_empty and q.intersects(owned_l[L])) or any(qp.contains(Point(a)) for a in a_nv):
                            new_b.append(q); kept += q.area
                    r41_info['cut_mm2'] += s.area - kept
                keep = new_b
        k3_n = 0
        if k3 and keep:
            permitted = allowed_fetch()
            for s in keep:
                inner_s = _gnd_buffer(s, geos_log, -(d_via / 2 + 0.05), QS)
                if inner_s.is_empty: continue
                sp = _gnd_buffer(s, geos_log, 0.01); shapely.prepare(sp)
                anchor = [a for a in gnd_anchor + [(v['x'], v['y']) for v in vias] if sp.contains(Point(a))]
                occupied = [(v['x'], v['y']) for v in R.get('vias', []) + vias]
                nv, _ = ends_stitch(s, s, anchor, permitted, inner_s, occupied, d_via, clr, k3_r, None, h=0.2)
                if len(anchor) + len(nv) == 1 and s.hausdorff_distance(Point((anchor + nv)[0])) > k2_round:
                    nv += ends_stitch(s, s, anchor + nv, permitted, inner_s, occupied + nv, d_via, clr, k2_round, None, max_new=1)[0]
                nv = [p for p in nv if via_ok(*p)]
                vias += [dict(x=x, y=y, net=gn, source='gnd_fill', k3=1) for x, y in nv]; k3_n += len(nv)
        if keep:
            g = unary_union(keep).simplify(0.00002, preserve_topology=True)
            z = dict(net=gn, layer=L, source='gnd_fill', polys=_polys(g), area_mm2=round(g.area, 2), prio=-1)
            if r41_info: z['r41'] = dict(vias=r41_info['vias'], cut_mm2=round(r41_info['cut_mm2'], 2))
            if k3: z['k3_vias'] = k3_n
            zones_out.append(z)
    return zones_out, vias


def _suggestion(rj, R):
    p = str(os.path.splitext(rj)[0]) + '_current_zones/suggestion.json'
    try: V = json.load(open(p))
    except (OSError, ValueError): return None


    k17 = lambda q: str(q.get('r36_k17', '0')).lower() in ('1', 'true')
    if k17(V) != k17(R): return None
    basis = lambda vs: sorted((round(v['x'], 4), round(v['y'], 4), v.get('net', '')) for v in vs if v.get('source') not in ('gnd_fill', 'current_via'))
    if V.get('tracks') != R.get('tracks') or basis(V.get('vias', [])) != basis(R.get('vias', [])): return None
    if not any(z.get('source') == 'current_zones' for z in V.get('zone_list', [])): return None
    return V


def _routing_verify(B, R, K, gnd_layers=None):
    from analysis.evaluation import not_connected, gnd_areas, native_areas
    from analysis.corpus import normalized_route
    from analysis.measures import review_active
    from common.zones import board_geometry, unify, via_geometry
    diagnose = 'conflicts' in R and 'failed' in R
    info = dict(is_valid=False, diagnose=diagnose, conflict_list=len(R.get('conflicts') or {}),
                open_items=len(R.get('failed') or []), not_connected=[], under_minimum=[])
    if not diagnose or info['conflict_list'] or info['open_items']: return info
    gl = gnd_layers if gnd_layers is not None else (R.get('gnd_layers') or (R.get('options') or {}).get('gnd_layers') or [])
    gl = [q for q in (gl.split(',') if isinstance(gl, str) else list(gl)) if q]
    R = normalized_route(B, K, R, gl)
    R = dict(R, gnd_layers=gl, options=dict(R.get('options') or {}, gnd_layers=gl))
    B = board_geometry(B, R)
    R = unify(via_geometry(R))
    covered = gnd_areas(R, set(K.get('gnd') or []), K) | native_areas(R, K)
    info['not_connected'] = not_connected(B, R, covered, K=K)
    rg = K.get('design_rules') or {}; rg = rg.get('router', rg)
    minimum = max(float(rg.get('track_min') or 0),
                  float((R.get('options') or {}).get('track_min') or 0)) if review_active(K) else 0.
    info['under_minimum'] = [dict(track=ti, segment=si, width=w, minimum=minimum)
                            for ti, t in enumerate(R.get('tracks', []))
                            for si, w in enumerate(t['w']) if w < minimum - 1e-9]
    info['is_valid'] = not info['not_connected'] and not info['under_minimum']
    return info


def _geometric(bj, kj, rj, gnd_layers=None, off=None, gnd=True, cpu_s=720):
    import tempfile
    from common import slots as PL
    bj, kj, rj = (str(Path(p).resolve()) for p in (bj, kj, rj))
    if off is not None: off = str(Path(off).resolve())
    original = Path(rj).read_bytes()
    with tempfile.TemporaryDirectory(prefix='autorouter-r36-geometric-') as folder:
        candidate = Path(folder)/'routes.json'; result = Path(folder)/'info.json'
        cmd = [str(Path(__file__).resolve().parents[1]/'.venv/bin/python'),
               '-m', 'learning.current_zones', '_geometric_worker', str(bj), str(kj), str(rj),
               str(candidate), str(result), json.dumps(gnd_layers), str(int(gnd))]
        run = PL.job(['caffeinate', '-i', '-s'] + cmd, cpu_s=cpu_s,
                      cwd=str(Path(__file__).resolve().parents[1]),
                      log=str(Path(folder)/'worker.log'), env=dict(os.environ, PYTHONHASHSEED='0'))
        info = json.loads(result.read_text()) if run['cur_state'] == 'ok' and result.exists() else dict(
            applied=False, reason='R36 geometry: ' + run['cur_state'])
        info['compute_slot'] = run
        if info.get('applied'):
            if not candidate.exists():
                return dict(info, applied=False, reason='R36 geometry without route output')
            if Path(rj).read_bytes() != original:
                return dict(info, applied=False, reason='Route changed during follow-up')
            target = Path(off or rj)
            with tempfile.NamedTemporaryFile(dir=target.parent, prefix='.r36_', delete=False) as f:
                tmp = f.name; f.write(candidate.read_bytes())
            try: os.replace(tmp, target)
            finally:
                if os.path.exists(tmp): os.unlink(tmp)
        return info

def _geometric_internal(bj, kj, rj, gnd_layers=None, off=None, gnd=True, _k17_proposal=False):
    B, K, R = [json.loads(Path(p).read_text()) for p in (bj,kj,rj)]
    from learning.zones_judge_selection import an as judge_on
    if judge_on(R):
        raise ValueError('zones_judge=1: geometric substitute without native acceptance blocked')
    R['zone_list'] = [z for z in R.get('zone_list', []) if z.get('source') not in ('current_zones', 'gnd_fill')]
    R['vias'] = [v for v in R.get('vias', []) if v.get('source') not in ('gnd_fill', 'current_via')]
    input_ = _routing_verify(B, R, K, gnd_layers)
    if not input_['is_valid']:
        return dict(applied=False, fallback_blocked='invalid_input_route', input_check=input_,
                    reason='R36: geometric substitute only on valid input route')
    if str(R.get('r36_k17', '0')).lower() in ('1', 'true') and not _k17_proposal:
        return _geometric_k17(bj,kj,rj,R,K,B,gnd_layers,off,gnd)
    prior = _suggestion(rj, R) if gnd else None
    if prior is not None:
        R['zone_list'], R['vias'] = prior['zone_list'], prior['vias']
        zs = [z for z in R['zone_list'] if z.get('source') == 'current_zones']; gz = [z for z in R['zone_list'] if z.get('source') == 'gnd_fill']
        info = dict(applied=True, input_check=input_, current_zones=len(zs), net_list=sorted({z['net'] for z in zs}), reused=True,
                    area_mm2=round(sum(z.get('area_mm2', 0) for z in zs), 1), owned_mm2=round(sum(z.get('owned_mm2', 0) for z in zs), 1),
                    gnd_zones=len(gz), gnd_area_mm2=round(sum(z.get('area_mm2', 0) for z in gz), 1))
        json.dump(R, open(off or rj, 'w'))
        return info
    zs = zone_list(B, R, K, gnd_layers)
    from learning.current_zones_topo import skeleton_apply
    n_skeleton = skeleton_apply(R, zs); R['zone_list'] += zs
    info = dict(applied=True, input_check=input_, current_zones=len(zs), net_list=sorted({z['net'] for z in zs}),
                **({'skeleton_tracks': n_skeleton} if n_skeleton else {}),
                area_mm2=round(sum(z['area_mm2'] for z in zs), 1), owned_mm2=round(sum(z['owned_mm2'] for z in zs), 1))
    if gnd:
        gz, gv = gnd_fill(B, R, K, gnd_layers); R['zone_list'] += gz; R['vias'] += gv
        info.update(gnd_zones=len(gz), gnd_vias=len(gv), gnd_area_mm2=round(sum(z['area_mm2'] for z in gz), 1))
    Path(off or rj).write_text(json.dumps(R))
    return info


def _geometric_k17(bj,kj,rj,R,K,B,gnd_layers,off,gnd):
    import copy
    import tempfile
    from analysis.evaluation import rate
    from analysis.k17 import measure
    from learning.k17_contour import assumption
    with tempfile.TemporaryDirectory(prefix='autorouter-k17-pair-') as tmp:
        tmp = Path(tmp)
        before = copy.deepcopy(R)
        before.pop('r36_k17',None)
        (tmp/'basis.json').write_text(json.dumps(before))
        (tmp/'suggestion.json').write_text(json.dumps(R))
        ia = _geometric_internal(bj,kj,str(tmp/'basis.json'),gnd_layers,str(tmp/'basis_off.json'),gnd)
        if not ia.get('applied') or not (tmp/'basis_off.json').exists():
            return dict(applied=False,reason='K17: default completion missing',basis=ia)
        try:
            ib = _geometric_internal(bj,kj,str(tmp/'suggestion.json'),gnd_layers,str(tmp/'suggestion_off.json'),gnd,
                                    _k17_proposal=True)
        except Exception as exc:
            ib = dict(applied=False,faults=repr(exc))
        a = json.loads((tmp/'basis_off.json').read_text())
        b = json.loads((tmp/'suggestion_off.json').read_text()) if ib.get('applied') else None
        try:
            if b is None:
                raise ValueError('Proposal completion missing: '+str(ib.get('faults') or ib.get('reason')))
            gate = assumption(rate(B,a,K,v3=True),rate(B,b,K,v3=True),measure(B,a,K),measure(B,b,K))
        except Exception as exc:
            gate = dict(passed=False,faults=repr(exc),missing=['complete pair acceptance'])
        chosen,info = (b,ib) if gate['passed'] else (a,ia)
        chosen['r36_k17'] = True
        chosen['r36_k17_acceptance'] = gate
        Path(off or rj).write_text(json.dumps(chosen))
        return dict(info,k17_acceptance=gate,k17_fallback=not gate['passed'])


def _receiver_allowed(B,K,before,after):
    from analysis.length_impedance import length_list,receiver_protection,receiver_netclasses
    kk=receiver_netclasses(K)
    return receiver_protection(length_list(B,before,kk),length_list(B,after,kk))


def apply(bj, kj, rj, gnd_layers=None, off=None, gnd=True, kicad_native=True, design_rules=None, cpu_s=720, zone_plan=None, k20=False, zones_judge=None, zones_judge_model=None):
    bj, kj, rj = (str(Path(p).resolve()) for p in (bj, kj, rj))
    if off is not None: off = str(Path(off).resolve())
    if design_rules: design_rules = str(Path(design_rules).resolve())
    if zone_plan: zone_plan = str(Path(zone_plan).resolve())
    original=Path(rj).read_bytes()
    route = json.loads(original)
    if zones_judge is not None:
        from learning.zones_judge_selection import an as _judge_on
        route['options']=dict(route.get('options') or {}, zones_judge=str(int(_judge_on(route,zones_judge))))
    if zones_judge_model is not None:
        route['options']=dict(route.get('options') or {}, zones_judge_model=str(zones_judge_model))
    if route.get('r36_corridor') and (route.get('r36_shape') or 'topo') == 'topo':
        raise ValueError("r36_corridor=1 is not implemented with shaping='topo'")
    if k20 and (not kicad_native or zone_plan):
        raise ValueError('K20 corridor needs native acceptance without local zone plan')
    from learning.zones_judge_selection import an as judge_on
    if judge_on(route) and (not kicad_native or zone_plan or k20):
        raise ValueError('zones_judge=1 needs native acceptance without local plan/K20')
    if not kicad_native:
        return _geometric(bj,kj,rj,gnd_layers,off,gnd,cpu_s=cpu_s)
    from common import slots as PL
    import tempfile
    root=Path(__file__).resolve().parents[1]
    target=Path(off or rj)
    planfile=Path(rj).parent/'zone_plan/zone_plan.json'
    local_plan=False
    if k20 and planfile.exists():
        try: local_plan=(json.loads(planfile.read_text()).get('context') or {}).get('mode') != 'diagnose'
        except (OSError,ValueError): local_plan=True
    if k20 and (route.get('zone_plan_id') or (route.get('current_zones_acceptance') or {}).get('zone_plan_id') or
                local_plan):
        return dict(applied=False,reason='K20 blocked with local zone plan',
                    compute_slot=dict(cpu_s=0.0,cur_state='not_started'))
    artifact=Path(str(Path(rj).with_suffix(''))+('_k20_current_zones' if k20 else '_current_zones'))
    artifact.mkdir(parents=True,exist_ok=True)
    source=artifact/'input_.json'
    if k20: route['k20_corridor'] = True
    if gnd_layers is not None:route['gnd_layers']=gnd_layers.split(',') if isinstance(gnd_layers,str) else list(gnd_layers)
    source.write_text(json.dumps(route))
    result=artifact/'acceptance.json';candidate=artifact/'filled_routes.json'
    result.unlink(missing_ok=True);candidate.unlink(missing_ok=True)
    cmd=['caffeinate','-i','-s',str(root/'.venv/bin/python'),'-m','learning.current_zones_post_run',
         str(Path(bj).resolve()),str(Path(kj).resolve()),str(source.resolve()),design_rules or '',
         str(result.resolve()),str(candidate.resolve()),str(int(gnd))]
    if zone_plan: cmd.append(str(Path(zone_plan).resolve()))

    run=PL.job(cmd,cpu_s=cpu_s,log=str(artifact/'worker.log'),cwd=str(root))
    info=json.loads(result.read_text()) if run['cur_state']=='ok' and result.exists() else dict(
        applied=False,reason='Native current zones: '+run['cur_state'])
    import hashlib
    info.update(compute_slot=run, artifact_folder=str(artifact.resolve()),
                input_sha256=hashlib.sha256(original).hexdigest(),
                worker_input_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                export_sha256=hashlib.sha256(candidate.read_bytes()).hexdigest() if candidate.exists() else None)
    if k20 and run['cpu_s'] >= cpu_s:
        info.update(applied=False, reason='K20: process tree CPU budget exhausted')
    if info['applied']:
        if k20 and (not (info.get('k20_gate') or {}).get('taken') or not info.get('receiver_gate')):
            info.update(applied=False, reason='K20: worker certificate missing')
            result.write_text(json.dumps(info,indent=1))
            return info
        if Path(rj).read_bytes()!=original:
            info.update(applied=False,reason='Route changed during native check')
            return info


        try:
            if not k20:
                B=json.loads(Path(bj).read_text());K=json.loads(Path(kj).read_text())
                from learning.shaping import receiver_gate_on
                info['receiver_gate']=_receiver_allowed(B,K,json.loads(original),json.loads(candidate.read_bytes())) if receiver_gate_on(K) else True
        except Exception as exc:
            info.update(receiver_gate=False,receiver_error=repr(exc))
        if not info['receiver_gate']:
            info.update(applied=False,reason='Receiver gate: native zone post-run')
            result.write_text(json.dumps(info,indent=1))
            return info
        target.parent.mkdir(parents=True,exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=target.parent,prefix='.current_zones_',delete=False) as f:
            tmp=f.name;f.write(candidate.read_bytes())
        try:
            if target.resolve()==Path(rj).resolve():
                Path(str(Path(rj).with_suffix(''))+('_before_k20_current_zones.json' if k20 else '_before_current_zones.json')).write_bytes(original)
            os.replace(tmp,target)
        finally:
            if os.path.exists(tmp):os.unlink(tmp)
    result.write_text(json.dumps(info,indent=1))
    return info


def verify(B, R, K, gnd_layers=None, clr=None, border=0.25):
    clr = clr if clr is not None else max(0.15, float(R.get('clr') or (R.get('options') or {}).get('clr') or 0.1))
    clr_r = float(R.get('clr') or (R.get('options') or {}).get('clr') or 0.09)
    contour = unary_union([Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3]).buffer(-border + 1e-3, QS)
    faults = []; R0 = dict(R, zone_list=[])
    parts_l = {}
    for z in R.get('zone_list', []):
        g = _geo(z)
        if z['layer'] not in parts_l: parts_l[z['layer']] = _parts(B, R0, z['layer'])
        t = parts_l[z['layer']]
        owned = unary_union([h for m, h in t if m == z['net']]); new = g.difference(owned.buffer(TOL))
        if not new.is_empty and not contour.contains(new.buffer(-1e-4)): faults.append((z['net'], z['layer'], 'border', round(new.difference(contour).area, 4)))
        other = [h for m, h in t if m != z['net']] + \
                 [_geo(y) for y in R.get('zone_list', []) if y is not z and y['layer'] == z['layer'] and y['net'] != z['net']]
        if other:
            a = unary_union(other)
            if not new.is_empty and new.distance(a) < clr - TOL: faults.append((z['net'], z['layer'], 'spacing_new', round(new.distance(a), 4)))


            if g.distance(a) < min(clr_r, owned.distance(a)) - TOL: faults.append((z['net'], z['layer'], 'spacing', round(g.distance(a), 4)))
    return faults


def main():
    if sys.argv[1:2] == ['_geometric_worker']:
        bj, kj, rj, off, result, gl, gnd = sys.argv[2:]
        info = _geometric_internal(bj, kj, rj, json.loads(gl), off, gnd == '1')
        Path(result).write_text(json.dumps(info))
        return
    a = sys.argv[1:]; kw = dict(x.split('=', 1) for x in a if '=' in x); p = [x for x in a if '=' not in x]
    gl = kw.get('gnd_layers'); gl = gl.split(',') if gl else None
    info = apply(p[0], p[1], p[2], gl, off=p[3] if len(p) > 3 else None, gnd=kw.get('gnd', '1') == '1',
                    kicad_native=kw.get('kicad_native','1')=='1',design_rules=kw.get('design_rules'),
                    zones_judge=kw.get('zones_judge'),
                    zones_judge_model=kw.get('zones_judge_model'))
    print(json.dumps(info, ensure_ascii=False))


if __name__ == '__main__':
    main()
