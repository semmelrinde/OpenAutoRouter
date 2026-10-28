""
import json, math, os, re, sys
from pathlib import Path

import shapely
from shapely.errors import GEOSException
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

STROMARTEN = ('strom', 'versorgung', 'sw')
QS = 32
TOL = 0.008


def _via_masse(R):
    try: d, b = [float(x) for x in str((R.get('optionen') or {}).get('via') or '0.45/0.20').split('/')]
    except ValueError: d, b = 0.45, 0.20
    return d, b


def _gndlagen(R, gndlagen):
    gl = gndlagen if gndlagen is not None else (R.get('gndlagen') or (R.get('optionen') or {}).get('gndlagen') or [])
    gl = gl if isinstance(gl, list) else str(gl).split(',')
    return [str(q).split('=')[0] for q in gl if q]


def _lagen(B):
    nl = int(B.get('lagen') or 2)
    return ['F'] + ['In%d' % i for i in range(1, nl - 1)] + ['B']


def _teile(B, R, L):
    ""
    from gemeinsam.zonen import viageometrie
    R = viageometrie(R); out = []
    for t in R.get('tracks', []):
        if t['layer'] != L or len(t['pts']) < 2: continue
        from lernen.kupferbalken import track_geometry
        out.append((t.get('net', ''), track_geometry(t)))
    for v in R.get('vias', []):
        if L in (v.get('layers') or _lagen(B)):
            out.append((v.get('net', ''), Point(v['x'], v['y']).buffer(v['diameter'] / 2, QS)))
    for p in B['pads']:
        for q in (p.get('polys') or {}).get(L, []):
            if len(q) >= 3: out.append((p['net'], Polygon(q).buffer(0)))
    for z in R.get('zonen', []):
        if z['layer'] == L: out.append((z['net'], _geo(z)))
    for z in R.get('zones', []):
        if z['layer'] == L and z.get('filled'):
            out.append((z['net'], Polygon(z['pts'], z.get('holes', [])).buffer(0)))
    return out


def _geo(z):
    return unary_union([Polygon(p['aussen'], p.get('innen') or []).buffer(0) for p in z['polys']])


def maskenfreiraum(B, n, L, clr, qs=QS):
    ""
    if L not in ('F', 'B'): return Polygon()
    design = B.get('design') or {}
    steg = design.get('maske_steg')
    if not isinstance(steg, (int, float)) or not math.isfinite(steg): return Polygon()
    eigene = {n} if isinstance(n, str) else set(n)
    out = []
    for p in B['pads']:
        if p['net'] in eigene or p.get('graphic') or p['ref'].startswith(('NTBAR_', 'CU_')): continue
        mk = (p['maske'] or {}).get(L) if 'maske' in p else design.get('maske')
        if not isinstance(mk, (int, float)) or not math.isfinite(mk): continue
        abstand = max(clr, mk + max(0., steg))
        if abstand <= clr: continue
        out.extend(Polygon(q).buffer(0).buffer(abstand + .003, qs)
                   for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3)
    return unary_union(out) if out else Polygon()


def maskenfrei(frei, B, n, L, clr, qs=QS):
    ""
    sperre = maskenfreiraum(B, n, L, clr, qs)
    return frei.difference(sperre) if not sperre.is_empty else frei


def sperrflaeche(B, L, verbot):
    return unary_union([Polygon(q, (k.get('loecher') or [[]]*len(k['polys']))[i]).buffer(0)
        for k in B.get('keepouts', []) if L in (k.get('lagen') or _lagen(B)) and (k.get('verbote') or {}).get(verbot)
        for i,q in enumerate(k.get('polys', [])) if len(q)>=3])


def _polys(g):
    out = []
    for p in sorted(getattr(g, 'geoms', [g]), key=lambda p: -p.area):
        if p.is_empty or p.geom_type != 'Polygon': continue
        out.append(dict(aussen=[[round(x, 6), round(y, 6)] for x, y in p.exterior.coords],
                        innen=[[[round(x, 6), round(y, 6)] for x, y in i.coords] for i in p.interiors]))
    return out


def netz_strom(K, n):
    from analyse.metriken import _strom
    if (K.get('netze') or {}).get(n, {}).get('strom_A') is not None or (K.get('netze') or {}).get(n, {}).get('fluss'):
        return _strom(n, K)[0]
    v = (K.get('netze') or {}).get(n) or {}
    for k in ('strom', 'strom_a', 'I', 'ampere'):
        if isinstance(v.get(k), (int, float)): return float(v[k])
    st = (K.get('strom') or {}).get(n)
    if isinstance(st, (int, float)): return float(st)
    if isinstance(st, dict): return float(sum(x for x in st.values() if isinstance(x, (int, float))))
    return 1.0 if v.get('art') == 'sw' else 0.5


def vorrang(B, R, K):
    ""
    from analyse.kupfermodell import ir_werte
    from analyse.metriken import spannung_mit_quelle, budgets
    physics = ir_werte(B, dict(R, stromzonen_modell='r36-dc-v1'), K)
    result = {}; dc_budgets, _ = budgets(K)
    for n in stromnetze(K, set(K.get('gnd') or ['GND'])):
        u, _ = spannung_mit_quelle(n, K)
        budget = dc_budgets.get(n, max(10., 10*abs(u)) if u else 30.)
        drop = physics['max_abfall_mV'].get(n)
        result[n] = dict(strom_A=netz_strom(K, n), abfall_mV=drop, budget_mV=budget,
                         gewicht=netz_strom(K, n)*(1+(drop/budget if drop is not None else 1)))
    return result


def tangential_runden(g, kern, frei, radius=.4):
    ""
    import numpy as np
    from shapely.geometry.polygon import orient
    safe = frei.buffer(2e-7)
    core = kern.buffer(-2e-7)
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


def _strom_bekannt(v):
    ""
    fl = v.get('fluss') or {}
    if fl.get('einheit') == 'A' and fl.get('gesamt') is not None: return float(fl['gesamt'])
    return float(v['strom_A']) if v.get('strom_A') is not None else None


def stromnetze(K, gnd, i_zone=I_ZONE):
    ""
    N = K.get('netze') or {}
    return sorted((n for n, v in N.items() if n not in gnd and
                   (v.get('art') in STROMARTEN or (_strom_bekannt(v) or 0.0) >= i_zone)),
                  key=lambda n: (-netz_strom(K, n), n))


def messzweige(B, R, K, L, n):
    ""
    from gemeinsam.fluss import analyse
    ts=[t for t in R['tracks'] if t['net']==n]
    ps=[p for p in B['pads'] if p['net']==n]



    sense=re.compile(r'SENS|SNS|KELVIN|\bFB\b|FEEDBACK|REMOTE', re.I)
    abgr=K.get('abgriffe') or {}
    if not (abgr.get(n) if isinstance(abgr, dict) else n in abgr) and \
            not any(sense.search(str(p.get('pinfunc') or '') + ' ' + str(p.get('pintype') or '')) for p in ps):
        return Polygon(), Polygon()
    f=K['netze'][n].get('fluss') or {}
    total=sum(f.get('last',{}).values())
    quiet_rel=float((R.get('r36_schutz') or {}).get(n,.001))
    if not total or (quiet_rel<=.001 and not any(0<v<total*.001 for v in f.get('last',{}).values())):return Polygon(),Polygon()
    info=analyse(ts,[v for v in R['vias'] if v['net']==n],ps,f,endpunkte=True)
    if not info:return Polygon(),Polygon()



    import numpy as np
    from scipy.spatial import cKDTree
    from codex.zonen_ir.solver_fast import geometry,Grid,ModelError
    from analyse.metriken import aufbau
    rr=dict(R,tracks=[{k:v for k,v in t.items() if k!='items'} for t in R['tracks']])
    layers=_lagen(B);vd,drill=_via_masse(R)
    polys,padmap,barrels,_,_=geometry(B,rr,n,layers,vd)
    polys={l:g.simplify(.001,preserve_topology=True) for l,g in polys.items()}
    A=aufbau(len(layers),_gndlagen(R,None))
    stack=B.get('stackup') or {l:dict(copper_um=v['t']*1000,z_mm=v['z']) for l,v in A['lagen'].items()}
    try:
        grid=Grid(polys,padmap,barrels,layers,.05,stack,via_drill=drill)
        field,state=grid.solve(f.get('quelle',[]),f.get('last',{}))
        ix=np.array([i for i,l in enumerate(grid.node_layers) if l==L]);tree=cKDTree(grid.coords[ix]) if len(ix) else None
        small=max(float(np.nanmax(state['u']))*quiet_rel,1e-9)
        protected={key for key,v in field['drop_V'].items() if quiet_rel>.001 and v<small}
        protected_share=max([float(f['last'][key])/total for key in protected] or [0.])
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
        shares=[info['anteil'].get((ti,si),0) for si in range(len(t['w']))]
        private=bool(shares) and 0<max(shares)<=share_limit
        for si,(a,b,w) in enumerate(zip(t['pts'][:-1],t['pts'][1:],t['w'])):
            share=info['anteil'].get((ti,si),0)
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
        load=f.get('last',{}).get(p['ref']+':'+str(p['num']),0)
        if 0<load<info['gesamt']*.001 or p['ref']+':'+str(p['num']) in protected:
            hs=[Polygon(q).buffer(skin,QS) for q in p['polys'].get(L,[])]
            pieces.extend(hs);definite.extend(hs)
    anchors=unary_union([Polygon(q).buffer(.005,QS) for p in ps if p['ref']+':'+str(p['num']) in f.get('quelle',[]) for q in p['polys'].get(L,[])])
    own=unary_union(pieces).difference(anchors)
    certain=unary_union(definite).difference(anchors)



    own=unary_union([part for part in getattr(own,'geoms',[own]) if part.intersects(certain)])



    own=own.union(unary_union([Polygon(q).buffer(skin,QS) for p in ps for q in p['polys'].get(L,[])
        if p['ref']+':'+str(p['num']) not in f.get('quelle',[]) and own.intersects(Polygon(q))]))
    anchors=unary_union([Polygon(q).buffer(.005,QS) for p in ps if p['ref']+':'+str(p['num']) in f.get('quelle',[]) for q in p['polys'].get(L,[])])
    return own,own.buffer(.15,QS).difference(anchors)


def _gruppen_huelle(pads, abstand, lokal=False):
    ""
    if lokal:


        return [a.union(b).convex_hull for i,a in enumerate(pads)
                for b in pads[i+1:] if a.distance(b) <= abstand]
    n = len(pads); eltern = list(range(n))
    def f(i):
        while eltern[i] != i: eltern[i] = eltern[eltern[i]]; i = eltern[i]
        return i
    for i in range(n):
        for j in range(i + 1, n):
            if pads[i].distance(pads[j]) <= abstand: eltern[f(i)] = f(j)
    gr = {}
    for i in range(n): gr.setdefault(f(i), []).append(pads[i])
    return [unary_union(g).convex_hull for g in gr.values() if len(g) >= 2]


def _segment_huellen(B, RR, L, n, wachsen):
    ""
    from gemeinsam.zonen import viageometrie
    RR = viageometrie(RR)
    pads = [Polygon(q).buffer(0) for p in B['pads'] if p['net'] == n for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3]
    vias = [Point(v['x'], v['y']).buffer(v['diameter'] / 2, QS)
            for v in RR.get('vias', []) if v.get('net') == n and L in (v.get('layers') or _lagen(B))]
    out = []
    for t in RR.get('tracks', []):
        if t['layer'] != L or t.get('net') != n: continue
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            enden = []
            for q in (a, b):
                P = Point(q); e = P.buffer(w / 2, QS)
                for g in pads + vias:
                    if g.distance(P) < 1e-3: e = e.union(g)
                enden.append(e.buffer(wachsen, QS))
            out.append(unary_union(enden).convex_hull)
    return out


_RICHT = [(math.cos(math.radians(45 * k)), math.sin(math.radians(45 * k))) for k in range(8)]
def _achteck(g, rand=0.0):
    ""
    import numpy as np
    pts = np.asarray(g.convex_hull.exterior.coords if g.convex_hull.geom_type == 'Polygon' else g.convex_hull.coords)
    U = np.asarray(_RICHT); c = (pts @ U.T).max(axis=0) + rand
    ecken = []
    for k in range(8):
        (a1, b1), (a2, b2) = U[k], U[(k + 1) % 8]; det = a1 * b2 - a2 * b1
        ecken.append(((c[k] * b2 - c[(k + 1) % 8] * b1) / det, (a1 * c[(k + 1) % 8] - a2 * c[k]) / det))
    return Polygon(ecken).buffer(0)


def _segment_achtecke(B, RR, L, n, wachsen):
    ""
    from gemeinsam.zonen import viageometrie
    RR = viageometrie(RR)
    pads = [Polygon(q).buffer(0) for p in B['pads'] if p['net'] == n for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3]
    vias = [Point(v['x'], v['y']).buffer(v['diameter'] / 2, 8)
            for v in RR.get('vias', []) if v.get('net') == n and L in (v.get('layers') or _lagen(B))]
    out = []
    for t in RR.get('tracks', []):
        if t['layer'] != L or t.get('net') != n: continue
        ws = t['w']
        for i, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], ws)):
            enden = []
            for j, q in enumerate((a, b)):

                width = w
                if RR.get('k20_korridor'):
                    neighbor = i-1 if j == 0 else i+1
                    if 0 <= neighbor < len(ws): width = max(w, ws[neighbor])
                P = Point(q); e = P.buffer(width / 2, 8)
                for g in pads + vias:
                    if g.distance(P) < 1e-3: e = e.union(g)
                enden.append(e)
            if RR.get('k20_korridor'):


                out.append(unary_union([_achteck(e, wachsen) for e in enden]).convex_hull)
                continue

            dx, dy = b[0] - a[0], b[1] - a[1]; ang = math.degrees(math.atan2(dy, dx)) % 45
            if math.hypot(dx, dy) < 1e-6 or min(ang, 45 - ang) < 3: out.append(_achteck(unary_union(enden), wachsen))
            else: out += [_achteck(e, wachsen) for e in enden]
    return out


def _schliessen_g(g, r):
    return g.buffer(r, join_style=2, mitre_limit=2.0).buffer(-r, join_style=2, mitre_limit=2.0)


def _oeffnen_g(g, r):
    return g.buffer(-r, join_style=2, mitre_limit=2.0).buffer(r, join_style=2, mitre_limit=2.0)


def _schliessen(g, r):
    return g.buffer(r, QS).buffer(-r, QS)


def _oeffnen(g, r):
    return g.buffer(-r, QS).buffer(r, QS)


SCHUTZ = ('kette', 'abg', 'kelvin', 'sig')

def bahnschutz_an(R):
    ""
    o = R.get('optionen') or {}
    return str(o.get('bahnschutz', '0')).lower() in ('1', 'true') or str(o.get('r39', '0')) == '1'


def _bahnschutz(R, L, n):
    ""
    from lernen.kupferbalken import track_geometry
    tracks = [t for t in R.get('tracks', []) if t.get('net') == n and t['layer'] == L
              and any(t.get(k) for k in SCHUTZ)]
    copper = unary_union([track_geometry(t) for t in tracks])
    clearance = float(R.get('clr') or (R.get('optionen') or {}).get('clr') or .1)
    return copper, copper.buffer(clearance + .005, QS)


def _zonen_kupfer(R, n, L):
    ""
    return unary_union([_geo(q) for q in R.get('zonen', []) if q['net'] == n and q['layer'] == L] +
                       [Polygon(q['pts'], q.get('holes') or []).buffer(0) for q in R.get('zones', [])
                        if q['net'] == n and q['layer'] == L and q.get('filled')])


def _geschuetzte_zonen(B, R, zones):
    ""
    out = []; teile = {}
    for z in zones:
        copper, moat = _bahnschutz(R, z['layer'], z['net'])
        if copper.is_empty:
            out.append(z); continue
        if z['layer'] not in teile: teile[z['layer']] = _teile(B, R, z['layer'])
        saat = unary_union([g for n, g in teile[z['layer']] if n == z['net']]).difference(moat)
        if saat.is_empty: continue

        existing = _zonen_kupfer(R, z['net'], z['layer'])
        g = _geo(z).difference(moat).union(_geo(z).intersection(copper.union(existing)))
        g = unary_union([q for q in getattr(g, 'geoms', [g]) if q.intersects(saat)])
        if g.is_empty: continue
        out.append(dict(z, polys=_polys(g), flaeche_mm2=round(g.area, 2)))
    return out








SKELETT = None


def skelett_anwenden(R, zs):
    from lernen.stromzonen_topo import skelett_anwenden as _a
    return _a(R, zs)


def skelett(R):
    ""
    w = R.get('r36_skelett', SKELETT) if isinstance(R, dict) else SKELETT
    return float(w) if w else None


def zonen(B, R, K, gndlagen=None, clr=None, rand=0.25, r_glatt=0.6, r_frei=0.2, r_oeffnen=0.2, gruppe=None, min_flaeche=0.05,
          min_bahn=1.0, prioritaet=None, messzweig=True, kompakt=True, form=None, zonen_richter=None):
    ""
    from lernen.zonen_richter_auswahl import an, waehlen
    if an(R, zonen_richter):
        return waehlen(B, R, K, gndlagen, clr=clr, rand=rand, r_glatt=r_glatt, r_frei=r_frei,
            r_oeffnen=r_oeffnen, gruppe=gruppe, min_flaeche=min_flaeche, min_bahn=min_bahn,
            prioritaet=prioritaet, messzweig=messzweig, kompakt=kompakt, form=form)
    form = form or (R.get('r36_form') or 'topo')
    from analyse.schaltknoten import an as _sw_an
    if _sw_an(R) and form != 'topo':
        raise ValueError("sw_kompakt=1 ist nur fuer form='topo' implementiert")
    if str(R.get('r36_k17', '0')).lower() in ('1', 'true') and form != 'topo':
        raise ValueError("r36_k17=1 ist nur fuer form='topo' implementiert")
    if R.get('r36_korridor') and form == 'topo':
        raise ValueError("r36_korridor=1 ist mit form='topo' nicht implementiert")
    if str((R.get('optionen') or {}).get('r39', '0')) == '1' and form != 'gerade':
        raise ValueError("r39=1 ist nur fuer form='gerade' (r36_form='gerade') implementiert")
    if form == 'topo':



        from lernen.stromzonen_topo import zonen as zonen_topo
        zones = zonen_topo(B, R, K, gndlagen, clr=clr, rand=rand, kappa=1.0, skelett=skelett(R),
                           nur_flaeche=float(R.get('r36_nur_flaeche', 0.2)),
                           lam_min_mw=float(R.get('r36_lam_min', 0.0)),
                           lam_max_mw=float(R.get('r36_lam_max', 2.0)))
        return _geschuetzte_zonen(B, R, zones) if bahnschutz_an(R) and any(any(t.get(k) for k in SCHUTZ) for t in R.get('tracks', [])) else zones
    if form == 'minimax':



        from lernen.stromzonen_minimax import zonen as zonen_minimax
        zones = zonen_minimax(B, R, K, gndlagen, clr=clr, rand=rand)
        return _geschuetzte_zonen(B, R, zones) if bahnschutz_an(R) and any(any(t.get(k) for k in SCHUTZ) for t in R.get('tracks', [])) else zones
    korridor = bool(R.get('r36_korridor'))
    gnd = set(K.get('gnd') or ['GND']); gl = set(_gndlagen(R, gndlagen))
    clr = clr if clr is not None else max(0.15, float(R.get('clr') or (R.get('optionen') or {}).get('clr') or 0.1))
    umriss = unary_union([Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3]).buffer(-rand, QS)
    _, b_via = _via_masse(R)
    loecher = [(v.get('net', ''), Point(v['x'], v['y']).buffer(b_via / 2, QS)) for v in R.get('vias', [])]
    loecher += [(p['net'], Point(p['x'], p['y']).buffer(p['drill'] / 2, QS)) for p in B['pads'] if p.get('drill')]
    prioritaet = prioritaet if prioritaet is not None else vorrang(B, R, K)
    netze = sorted(stromnetze(K, gnd), key=lambda n:(-prioritaet[n]['gewicht'], n))
    RR = dict(R, zonen=list(R.get('zonen') or [])); out = []
    schutz = bahnschutz_an(RR)
    for L in _lagen(B):
        if L in gl: continue
        for n in netze:
            teile = _teile(B, RR, L)
            eigen = unary_union([g for m, g in teile if m == n])
            if eigen.is_empty: continue
            protected, moat = _bahnschutz(RR, L, n) if schutz else (Polygon(), Polygon())
            wachstum = dict(RR, tracks=[t for t in RR.get('tracks', [])
                            if not (t.get('net') == n and t['layer'] == L and
                                    any(t.get(k) for k in SCHUTZ))]) if schutz else RR
            saat = eigen.difference(moat) if not protected.is_empty else eigen
            if saat.is_empty: continue
            segs = [(a, b, w) for t in wachstum.get('tracks', []) if t['layer'] == L and t.get('net') == n
                    for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w'])]
            bahnen_pads = unary_union([(LineString([a, b]) if a != b else Point(a)).buffer(w / 2, QS) for a, b, w in segs] +
                                      [Polygon(q).buffer(0) for p in B['pads'] if p['net'] == n for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3])
            if bahnen_pads.is_empty: continue
            pads = [Polygon(q).buffer(0) for p in B['pads'] if p['net'] == n for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3]
            if len(pads) < 2 and not segs: continue
            fremd = unary_union([g for m, g in teile if m != n] + [g for m, g in loecher if m != n])
            sperre = sperrflaeche(B, L, 'flaechen')
            frei_pad = umriss.difference(fremd.buffer(clr + .003, QS)).difference(sperre)
            frei_pad = maskenfrei(frei_pad, B, n, L, clr)
            hole = float((R.get('optionen') or {}).get('h2h') or .2)
            frei_pad = frei_pad.difference(unary_union([h.buffer(hole+.003, QS) for m,h in loecher if m != n]))
            if not protected.is_empty: frei_pad = frei_pad.difference(moat)
            frei = _oeffnen(frei_pad, float((R.get('r36_frei') or {}).get(n,r_frei)))
            I = netz_strom(K, n)
            if kompakt:


                wachsen = min(0.6, max(0.2, 0.15 + 0.1 * I)); reich = 1.0; grp = 1.5 if gruppe is None else gruppe
            else:
                wachsen = min(1.5, max(0.5, 0.4 + 0.25 * I))
                reich = min(3.5, 2.0 + 0.4 * I)
                grp = 4.0 if gruppe is None else gruppe



            stuecke_e = [s for s in getattr(saat, 'geoms', [saat]) if not s.is_empty]
            huellen = [] if kompakt else [s.convex_hull.intersection(s.buffer(reich, QS)) for s in stuecke_e]
            if form == 'gerade' and kompakt:


                from gemeinsam.zonen import viageometrie
                vias_n = [Point(v['x'], v['y']).buffer(v['diameter'] / 2, 8)
                          for v in viageometrie(RR)['vias'] if v.get('net') == n and L in (v.get('layers') or _lagen(B))]
                kand = unary_union([_achteck(p, wachsen) for p in pads + vias_n] + [_achteck(h, wachsen) for h in _gruppen_huelle(pads, grp, lokal=korridor)] +
                                   _segment_achtecke(B, wachstum, L, n, wachsen))
                z = _oeffnen_g(kand.intersection(frei), r_oeffnen)
                z = unary_union([z, eigen])
                z = _schliessen_g(z, r_glatt).intersection(unary_union([frei, eigen]))
            else:
                kand = unary_union([saat.buffer(wachsen, QS)] + huellen + _gruppen_huelle(pads, grp, lokal=korridor) + _segment_huellen(B, wachstum, L, n, wachsen))
                z = _oeffnen(kand.intersection(frei), r_oeffnen)
                z = unary_union([z, eigen])
                z = _schliessen(z, r_glatt).intersection(unary_union([frei, eigen]))
            stuecke = [s for s in getattr(z, 'geoms', [z]) if s.geom_type == 'Polygon' and s.area >= min_flaeche
                       and s.buffer(1e-3).intersects(bahnen_pads)]
            if not stuecke: continue
            g = unary_union(stuecke + [eigen] + [p.buffer(.04, QS).intersection(frei_pad) for p in pads]).buffer(0)


            W = min(2.0, float((K.get('netze', {}).get(n) or {}).get('w') or 0.3))
            duenn = g.difference(g.buffer(-W / 2 + 1e-4, QS).buffer(W / 2 - 1e-4, QS))
            if not duenn.is_empty and form == 'gerade' and kompakt:


                korr = [LineString([a, b]).buffer(W / 2, cap_style=2, join_style=2) for a, b, w in segs
                        if a != b and w < W and LineString([a, b]).intersects(duenn.buffer(1e-3))]
                if korr: g = g.union(unary_union(korr).intersection(frei_pad)).buffer(0)
            elif not duenn.is_empty: g = g.union(duenn.buffer(W / 2, QS).intersection(frei_pad)).buffer(0)
            if R.get('r36_glatt'):
                from lernen.r36_form import uebergaenge
                g = g.union(uebergaenge(RR, L, n, wachsen).intersection(frei_pad)).buffer(0)




            safe = frei_pad.union(eigen)
            g = g.buffer(.02, QS).intersection(safe).simplify(.0005, preserve_topology=True)
            g = _oeffnen(_schliessen(g,.01),.01).intersection(safe)
            g = g.intersection(safe).union(eigen)
            for _ in range(2):
                g = tangential_runden(g.simplify(.002,preserve_topology=True).union(eigen), eigen, frei_pad)
            if messzweig:
                sense,moat=messzweige(B,R,K,L,n)
                g=g.difference(moat).union(sense.intersection(frei_pad)).union(eigen)
                g=tangential_runden(g.simplify(.0005,preserve_topology=True).union(eigen),eigen,frei_pad)
            if korridor:



                saum = max(.75, float((K.get('netze',{}).get(n) or {}).get('w') or .3)/2)
                g = g.intersection(eigen.buffer(saum, QS)).union(eigen).buffer(0)




            if str((R.get('optionen') or {}).get('r39', '0')) == '1':
                g = r39_korridor(B, RR, K, L, n, g, eigen, frei_pad, messzweig)
            zz = dict(net=n, layer=L, quelle='stromzonen', strom_a=round(I, 3), polys=_polys(g),
                      flaeche_mm2=round(g.area, 2), eigen_mm2=round(eigen.area, 2), vorrang=prioritaet[n])
            out.append(zz); RR['zonen'].append(zz)
    return out


def r39_korridor(B, R, K, L, n, g, eigen, frei, messzweig=True):
    ""
    from gemeinsam.bogen import punkte
    W = float(K['netze'][n].get('w') or .3)
    if not math.isfinite(W) or W <= 0: raise ValueError('R39: ungueltige Sollbreite ' + n)
    paths = []
    for t in R.get('tracks', []):
        if t.get('net') != n or t['layer'] != L or any(t.get(k) for k in ('abg', 'kette', 'kelvin', 'sig')): continue
        if 'items' in t:
            for item in t['items']:
                ps = item[1:3] if item[0] == 'L' else punkte(*item[1:4])
                paths.append(LineString(ps).buffer(W / 2, QS))
        else:
            for a, b in zip(t['pts'][:-1], t['pts'][1:]):
                if a != b: paths.append(LineString([a, b]).buffer(W / 2, QS))
    if not paths: return g
    allowed = frei
    chains, moat = _bahnschutz(R, L, n)
    if not chains.is_empty:

        allowed = allowed.difference(moat)
    if messzweig:
        _, moat = messzweige(B, R, K, L, n)
        allowed = allowed.difference(moat)
    add = unary_union(paths).intersection(allowed)
    add = unary_union([p for p in getattr(add, 'geoms', [add])
                       if p.geom_type == 'Polygon' and p.intersects(eigen)])


    original = [p for p in getattr(g, 'geoms', [g]) if p.geom_type == 'Polygon']
    safe_add = [p for p in getattr(add, 'geoms', [add])
                if sum(p.intersects(q) for q in original) <= 1]
    return g.union(unary_union(safe_add)).buffer(0)

def schnelle_netze(K, ab_rang=3):
    ""
    return {n for n, k in sorted((K.get('netze') or {}).items())
            if k.get('art') == 'sw' or (k.get('art') == 'schnell' and (k.get('rang') or 0) >= ab_rang)}

def _geo_wachsen(saat, M, k):
    ""
    from scipy import ndimage
    s4, s8 = ndimage.generate_binary_structure(2, 1), ndimage.generate_binary_structure(2, 2)
    x = saat & M
    for i in range(k):
        y = ndimage.binary_dilation(x, structure=s8 if i % 2 else s4, mask=M)
        if (y == x).all(): break
        x = y
    return x


def enden_stitchen(s, nah, anker, erlaubt, innen, sperr_vias, d_via, clr, r=1.5, r_schnitt=2.0, h=0.1, min_zellen=4, max_neu=None):
    ""
    import numpy as np
    from scipy.signal import fftconvolve
    x0, y0, x1, y1 = s.bounds
    nx, ny = int(math.ceil((x1 - x0) / h)) + 1, int(math.ceil((y1 - y0) / h)) + 1
    X, Y = np.meshgrid(x0 + h * np.arange(nx), y0 + h * np.arange(ny))
    for g in (s, nah, innen, erlaubt): shapely.prepare(g)
    M = shapely.contains_xy(s, X, Y)
    U = M & shapely.contains_xy(nah, X, Y)
    if U.sum() < min_zellen: return [], None
    U2 = U.copy() if r_schnitt is not None else None
    OK = M & shapely.contains_xy(innen, X, Y) & shapely.contains_xy(erlaubt, X, Y)
    k, k2 = int(math.ceil(r / h)), int(math.ceil((r_schnitt or r) / h)); ks = int(math.ceil((d_via + clr) / h))
    yy, xx = np.mgrid[-k:k + 1, -k:k + 1]; scheibe = ((xx * xx + yy * yy) * h * h <= r * r).astype(np.float32)

    def fenster(ax, ay, kk):
        cx, cy = int(round((ax - x0) / h)), int(round((ay - y0) / h))
        return slice(max(0, cy - kk - 1), max(0, min(ny, cy + kk + 2))), slice(max(0, cx - kk - 1), max(0, min(nx, cx + kk + 2)))
    def abdecken(ax, ay, probe=False):
        n = 0
        for UU, rr, kk in ((U, r, k), (U2, r_schnitt, k2)) if U2 is not None else ((U, r, k),):
            a, b = fenster(ax, ay, kk)
            if a.start >= a.stop or b.start >= b.stop: continue
            d2 = (X[a, b] - ax) ** 2 + (Y[a, b] - ay) ** 2
            saat = M[a, b] & (d2 <= (d_via / 2 + h) ** 2)
            if not saat.any(): continue
            dk = _geo_wachsen(saat, M[a, b], kk) & (d2 <= rr * rr)
            if UU is U: n = int((U[a, b] & dk).sum())
            if probe: return n
            UU[a, b] &= ~dk
        return n

    def sperren(ax, ay):
        a, b = fenster(ax, ay, ks)
        if a.start < a.stop and b.start < b.stop:
            OK[a, b] &= ~(((X[a, b] - ax) ** 2 + (Y[a, b] - ay) ** 2) < (d_via + clr) ** 2)

    for ax, ay in anker: abdecken(ax, ay)
    for ax, ay in sperr_vias:
        if x0 - 1 <= ax <= x1 + 1 and y0 - 1 <= ay <= y1 + 1: sperren(ax, ay)
    neu = []
    for _ in range(2000):
        if U.sum() < min_zellen or (max_neu is not None and len(neu) >= max_neu): break
        iy, ix = np.nonzero(U)
        a = slice(max(0, iy.min() - k), iy.max() + k + 1); b = slice(max(0, ix.min() - k), ix.max() + k + 1)
        G = fftconvolve(U[a, b].astype(np.float32), scheibe, mode='same')
        G = np.where(OK[a, b], np.rint(G), 0)
        j = int(np.argmax(G))
        if G.flat[j] < min_zellen: break
        cy, cx = divmod(j, G.shape[1]); cy += a.start; cx += b.start
        vx, vy = float(X[cy, cx]), float(Y[cy, cx])
        if abdecken(vx, vy, probe=True) < min_zellen: OK[cy, cx] = False; continue
        abdecken(vx, vy); sperren(vx, vy); neu.append((round(vx, 4), round(vy, 4)))
    if U2 is None or not U2.any(): return neu, None
    iy, ix = np.nonzero(U2)
    e = h * 0.55
    rest = shapely.union_all(shapely.box(X[iy, ix] - e, Y[iy, ix] - e, X[iy, ix] + e, Y[iy, ix] + e))
    return neu, rest


def _polys_von(g):
    return [q for q in getattr(g, 'geoms', [g]) if q.geom_type == 'Polygon' and not q.is_empty]


def offene_tasche(s, offen, anker, d_via, w_max=2.0, r_ende=0.8):
    ""
    weg = []
    breit = s.buffer(-w_max / 2, QS).buffer(w_max / 2, QS)
    for o in sorted(_polys_von(offen), key=lambda o: (-o.area, o.bounds)):
        if o.area < 0.05 or any(o.intersects(t) for t in weg): continue
        t = None
        frei_von = lambda g: all(g.distance(Point(a)) > r_ende for a in anker)
        for w in (0.3, 0.5, 0.8, 1.2, 1.6, w_max):
            dran = [c for c in _polys_von(s.buffer(-w / 2, QS)) if c.distance(o) <= w / 2]
            if not dran: break
            if any(c.distance(Point(a)) <= w / 2 for c in dran for a in anker): continue
            for g in (unary_union(dran).buffer(w, QS).intersection(s), unary_union(dran).buffer(w / 2, QS).intersection(s)):
                if frei_von(g) and g.area <= 4 * o.area + 4.0: t = g; break
            if t is not None: break
        if t is None and breit.intersection(o).area < 0.01:
            nahe = [a for a in anker if Point(a).distance(o) <= 3.0]
            if nahe:
                schnitt = s.difference(unary_union([Point(a).buffer(r_ende, QS) for a in nahe])).buffer(-0.1, QS).buffer(0.1, QS)
                t = unary_union([q for q in _polys_von(schnitt) if q.intersects(o)])
                if t.area > 4 * o.area + 2.0: t = None
        if t is not None and not t.is_empty and t.area >= 0.1: weg.append(t)
    return unary_union(weg) if weg else None


GND_GRID = 1e-6


def _gnd_differenz(a, protokoll, b, *args, **kwargs):
    try:
        return a.difference(b, *args, **kwargs)
    except GEOSException as exc:
        if not a.is_valid or not b.is_valid:
            raise
        g = shapely.difference(a, b, grid_size=GND_GRID)
        if not g.is_valid:
            raise ValueError('Ungültiger GND-Overlay-Nachlauf') from exc
        if shapely.difference(g, a.buffer(2*GND_GRID), grid_size=GND_GRID).area > 1e-9:
            raise ValueError('GND-Overlay fügt Kupfer außerhalb des Ausgangsraums hinzu') from exc
        if shapely.intersection(g, b.buffer(-2*GND_GRID), grid_size=GND_GRID).area > 1e-9:
            raise ValueError('GND-Overlay füllt fremde Sperrfläche') from exc
        match = re.search(r' at ([\d.eE+-]+) ([\d.eE+-]+)', str(exc))
        protokoll.append(dict(fehler=str(exc), ort=[float(v) for v in match.groups()] if match else None,
                             grid_mm=GND_GRID, a_flaeche_mm2=a.area, b_flaeche_mm2=b.area,
                             ausgabe_flaeche_mm2=g.area, a_gueltig=True, b_gueltig=True))
        return g


def _gnd_puffer(a, protokoll, *args, **kwargs):
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
        raise ValueError('Ungültiger einzelner GND-Komponentenpuffer')
    result = shapely.union_all(pieces)
    if not result.is_valid:
        raise ValueError('Ungültige Vereinigung der GND-Komponentenpuffer')
    cover = result.buffer(2*GND_GRID)
    if any(part.difference(cover).area > 1e-9 for part in pieces):
        raise ValueError('GND-Komponentenpuffer verliert Sperrfläche')
    match = re.search(r'(?:\[| at )([\d.eE+-]+) ([\d.eE+-]+)', reason)
    protokoll.append(dict(art='komponenten_puffer', fehler=reason,
                         ort=[float(v) for v in match.groups()] if match else None,
                         radius_mm=args[0], teile=len(pieces), input_gueltig=True,
                         original_flaeche_mm2=g.area if g is not None else None,
                         ausgabe_flaeche_mm2=result.area))
    return result


def gnd_fuellung(B, R, K, gndlagen=None, clr=None, rand=0.25, raster=2.0, min_breite=0.25, min_flaeche=0.3,
                 r41=True, r41_r=1.5, r41_schnitt=2.0, r41_nah=1.0, r41_streifen=2.0, k3=True, k3_r=3.0, k2_rund=2.0, geos_protokoll=None):
    ""
    geos_log = geos_protokoll if geos_protokoll is not None else []
    gnd = set(K.get('gnd') or ['GND']); gn = sorted(gnd)[0]; gl = set(_gndlagen(R, gndlagen))
    clr = clr if clr is not None else max(0.15, float(R.get('clr') or (R.get('optionen') or {}).get('clr') or 0.1))
    d_via, b_via = _via_masse(R)
    umriss = _gnd_puffer(unary_union([_gnd_puffer(Polygon(o), geos_log, 0) for o in B['outline'] if len(o) >= 3]), geos_log, -rand, QS)
    lagen = [L for L in _lagen(B) if L not in gl]
    loecher = [_gnd_puffer(Point(v['x'], v['y']), geos_log, b_via / 2, QS) for v in R.get('vias', []) if v.get('net') not in gnd]
    loecher += [_gnd_puffer(Point(p['x'], p['y']), geos_log, p['drill'] / 2, QS) for p in B['pads'] if p.get('drill') and p['net'] not in gnd]
    fremd_l, eigen_l, frei_l = {}, {}, {}
    for L in lagen:
        t = _teile(B, R, L)
        fremd_l[L] = unary_union([g for m, g in t if m not in gnd] + loecher)
        eigen_l[L] = unary_union([g for m, g in t if m in gnd])
        sperre = sperrflaeche(B, L, 'flaechen')
        f = _gnd_differenz(_gnd_differenz(umriss, geos_log, _gnd_puffer(fremd_l[L], geos_log, clr + .003, QS)), geos_log, sperre)
        maskensperre = maskenfreiraum(B, gnd, L, clr)
        if not maskensperre.is_empty: f = _gnd_differenz(f, geos_log, maskensperre)
        f = _gnd_differenz(f, geos_log, _gnd_puffer(unary_union(loecher), geos_log, float((R.get('optionen') or {}).get('h2h') or .2)+.003, QS))
        frei_l[L] = _gnd_puffer(_gnd_puffer(f, geos_log, -min_breite / 2, QS), geos_log, min_breite / 2, QS)

    alle = unary_union([g for L in _lagen(B) for n,g in _teile(B,R,L) if n not in gnd])
    via_maskensperre = unary_union([maskenfreiraum(B, gnd, L, clr) for L in _lagen(B)])
    hole = max(.2, float((R.get('optionen') or {}).get('h2h') or .2))
    bohr_fremd = unary_union(loecher)
    via_sperre = unary_union([sperrflaeche(B,L,'vias').union(sperrflaeche(B,L,'flaechen')) for L in _lagen(B)])
    vias = []
    def via_ok(x, y):
        c = _gnd_puffer(Point(x, y), geos_log, d_via / 2 + clr, QS)
        drill = _gnd_puffer(Point(x,y), geos_log, b_via/2 + hole, QS)
        return _gnd_puffer(umriss, geos_log, -d_via / 2).contains(Point(x, y)) and not c.intersects(alle) and \
            (via_maskensperre.is_empty or not Point(x,y).buffer(d_via/2, QS).intersects(via_maskensperre)) and not drill.intersects(bohr_fremd) and not c.intersects(via_sperre) and \
            all(Point(v['x'], v['y']).distance(Point(x, y)) >= d_via + clr for v in R.get('vias', []) + vias)
    SCH = schnelle_netze(K) if r41 else set()
    gnd_anker = [(v['x'], v['y']) for v in R.get('vias', []) if v.get('net') in gnd] + \
                [(p['x'], p['y']) for p in B['pads'] if p.get('drill') and p['net'] in gnd]
    bohr_alle = unary_union([_gnd_puffer(Point(p['x'], p['y']), geos_log, p['drill'] / 2, QS) for p in B['pads'] if p.get('drill')] + [bohr_fremd])
    erlaubt = None

    def erlaubt_holen():
        nonlocal erlaubt
        if erlaubt is None:
            verb = [_gnd_puffer(alle, geos_log, d_via / 2 + clr, QS), _gnd_puffer(bohr_alle, geos_log, b_via / 2 + hole, QS)]
            if not via_sperre.is_empty: verb.append(_gnd_puffer(via_sperre, geos_log, d_via / 2 + clr, QS))
            erlaubt = _gnd_differenz(_gnd_puffer(umriss, geos_log, -d_via / 2, QS), geos_log, unary_union(verb))
        return erlaubt
    zonen_out = []
    for L in lagen:
        stuecke = [s for s in getattr(frei_l[L], 'geoms', [frei_l[L]]) if s.geom_type == 'Polygon' and s.area >= min_flaeche]
        behalten = []
        for s in sorted(stuecke, key=lambda s: -s.area):
            if not eigen_l[L].is_empty and s.intersects(eigen_l[L]): behalten.append(s); continue
            if any(s.contains(Point(v['x'], v['y'])) for v in vias): behalten.append(s); continue
            innen = _gnd_puffer(s, geos_log, -(d_via / 2 + 0.05), QS)
            if innen.is_empty: continue
            x0, y0, x1, y1 = innen.bounds; gesetzt = False
            ys = [y0 + (y1 - y0) / 2] if y1 - y0 < raster else [y0 + raster / 2 + i * raster for i in range(int((y1 - y0) / raster) + 1)]
            xs = [x0 + (x1 - x0) / 2] if x1 - x0 < raster else [x0 + raster / 2 + i * raster for i in range(int((x1 - x0) / raster) + 1)]
            for y in ys:
                for x in xs:
                    if innen.contains(Point(x, y)) and via_ok(x, y):
                        vias.append(dict(x=round(x, 4), y=round(y, 4), net=gn, quelle='gnd_fuellung')); gesetzt = True
            if not gesetzt:
                p = innen.representative_point()
                if via_ok(p.x, p.y): vias.append(dict(x=round(p.x, 4), y=round(p.y, 4), net=gn, quelle='gnd_fuellung')); gesetzt = True
            if gesetzt: behalten.append(s)
        r41_info = None
        if r41 and SCH and behalten:
            schnell = unary_union([g for m, g in _teile(B, R, L) if m in SCH])
            if not schnell.is_empty:
                erlaubt = erlaubt_holen()
                nah_L = _gnd_puffer(schnell, geos_log, r41_nah, QS); r41_info = dict(vias=0, geschnitten_mm2=0.0)


                from lernen.kupferbalken import track_geometry
                bahnen = [track_geometry(t) for t in R.get('tracks', []) if t['layer'] == L and t.get('net') in SCH and len(t['pts']) >= 2]
                nah_schnitt = _gnd_puffer(unary_union(bahnen), geos_log, r41_nah, QS) if bahnen else None
                neu_b = []
                for s in behalten:
                    nah = s.intersection(nah_L)
                    if nah.area < 0.01: neu_b.append(s); continue
                    sp = _gnd_puffer(s, geos_log, 0.01); shapely.prepare(sp)
                    anker = [a for a in gnd_anker + [(v['x'], v['y']) for v in vias] if sp.contains(Point(a))]
                    nv, rest = enden_stitchen(s, nah, anker, erlaubt, _gnd_puffer(s, geos_log, -(d_via / 2 + 0.05), QS),
                                              [(v['x'], v['y']) for v in R.get('vias', []) + vias], d_via, clr, r41_r, r41_schnitt)
                    nv = [p for p in nv if via_ok(*p)]
                    vias += [dict(x=x, y=y, net=gn, quelle='gnd_fuellung', r41=1) for x, y in nv]; r41_info['vias'] += len(nv)
                    if rest is not None and not rest.is_empty:
                        rest = rest.intersection(nah_schnitt) if nah_schnitt is not None else None
                    if rest is not None and not rest.is_empty:
                        rest = offene_tasche(s, rest, anker + nv, d_via, r41_streifen)
                    if rest is None or rest.is_empty: neu_b.append(s); continue
                    t = _gnd_differenz(s, geos_log, _gnd_puffer(rest, geos_log, 0.05, QS))
                    t = _gnd_puffer(_gnd_puffer(t, geos_log, -min_breite / 2, QS), geos_log, min_breite / 2, QS)
                    t = _gnd_puffer(_gnd_puffer(t, geos_log, min_breite / 2, QS), geos_log, -min_breite / 2, QS).intersection(s)
                    a_nv = anker + nv; kept = 0.0
                    for q in getattr(t, 'geoms', [t]):
                        if q.geom_type != 'Polygon' or q.area < min_flaeche: continue
                        qp = _gnd_puffer(q, geos_log, 0.01)
                        if (not eigen_l[L].is_empty and q.intersects(eigen_l[L])) or any(qp.contains(Point(a)) for a in a_nv):
                            neu_b.append(q); kept += q.area
                    r41_info['geschnitten_mm2'] += s.area - kept
                behalten = neu_b
        k3_n = 0
        if k3 and behalten:
            erl = erlaubt_holen()
            for s in behalten:
                innen_s = _gnd_puffer(s, geos_log, -(d_via / 2 + 0.05), QS)
                if innen_s.is_empty: continue
                sp = _gnd_puffer(s, geos_log, 0.01); shapely.prepare(sp)
                anker = [a for a in gnd_anker + [(v['x'], v['y']) for v in vias] if sp.contains(Point(a))]
                belegt = [(v['x'], v['y']) for v in R.get('vias', []) + vias]
                nv, _ = enden_stitchen(s, s, anker, erl, innen_s, belegt, d_via, clr, k3_r, None, h=0.2)
                if len(anker) + len(nv) == 1 and s.hausdorff_distance(Point((anker + nv)[0])) > k2_rund:
                    nv += enden_stitchen(s, s, anker + nv, erl, innen_s, belegt + nv, d_via, clr, k2_rund, None, max_neu=1)[0]
                nv = [p for p in nv if via_ok(*p)]
                vias += [dict(x=x, y=y, net=gn, quelle='gnd_fuellung', k3=1) for x, y in nv]; k3_n += len(nv)
        if behalten:
            g = unary_union(behalten).simplify(0.00002, preserve_topology=True)
            z = dict(net=gn, layer=L, quelle='gnd_fuellung', polys=_polys(g), flaeche_mm2=round(g.area, 2), prio=-1)
            if r41_info: z['r41'] = dict(vias=r41_info['vias'], geschnitten_mm2=round(r41_info['geschnitten_mm2'], 2))
            if k3: z['k3_vias'] = k3_n
            zonen_out.append(z)
    return zonen_out, vias


def _vorschlag(rj, R):
    ""
    p = str(os.path.splitext(rj)[0]) + '_stromzonen/vorschlag.json'
    try: V = json.load(open(p))
    except (OSError, ValueError): return None


    k17 = lambda q: str(q.get('r36_k17', '0')).lower() in ('1', 'true')
    if k17(V) != k17(R): return None
    basis = lambda vs: sorted((round(v['x'], 4), round(v['y'], 4), v.get('net', '')) for v in vs if v.get('quelle') not in ('gnd_fuellung', 'strom_via'))
    if V.get('tracks') != R.get('tracks') or basis(V.get('vias', [])) != basis(R.get('vias', [])): return None
    if not any(z.get('quelle') == 'stromzonen' for z in V.get('zonen', [])): return None
    return V


def _routing_pruefen(B, R, K, gndlagen=None):
    ""
    from analyse.bewertung import unverbunden, gnd_flaechen, native_flaechen
    from analyse.korpus import normalisierte_route
    from analyse.metriken import review_aktiv
    from gemeinsam.zonen import boardgeometrie, vereinheitlichen, viageometrie
    diagnose = 'conflicts' in R and 'failed' in R
    info = dict(gueltig=False, diagnose=diagnose, konflikte=len(R.get('conflicts') or {}),
                offen=len(R.get('failed') or []), unverbunden=[], unter_minimum=[])
    if not diagnose or info['konflikte'] or info['offen']: return info
    gl = gndlagen if gndlagen is not None else (R.get('gndlagen') or (R.get('optionen') or {}).get('gndlagen') or [])
    gl = [q for q in (gl.split(',') if isinstance(gl, str) else list(gl)) if q]
    R = normalisierte_route(B, K, R, gl)
    R = dict(R, gndlagen=gl, optionen=dict(R.get('optionen') or {}, gndlagen=gl))
    B = boardgeometrie(B, R)
    R = vereinheitlichen(viageometrie(R))
    gedeckt = gnd_flaechen(R, set(K.get('gnd') or []), K) | native_flaechen(R, K)
    info['unverbunden'] = unverbunden(B, R, gedeckt, K=K)
    rg = K.get('regeln') or {}; rg = rg.get('router', rg)
    minimum = max(float(rg.get('bahn_min') or 0),
                  float((R.get('optionen') or {}).get('bahn_min') or 0)) if review_aktiv(K) else 0.
    info['unter_minimum'] = [dict(bahn=ti, segment=si, breite=w, minimum=minimum)
                            for ti, t in enumerate(R.get('tracks', []))
                            for si, w in enumerate(t['w']) if w < minimum - 1e-9]
    info['gueltig'] = not info['unverbunden'] and not info['unter_minimum']
    return info


def _geometrisch(bj, kj, rj, gndlagen=None, aus=None, gnd=True, cpu_s=720):
    ""
    import tempfile
    from gemeinsam import plaetze as PL
    bj, kj, rj = (str(Path(p).resolve()) for p in (bj, kj, rj))
    if aus is not None: aus = str(Path(aus).resolve())
    original = Path(rj).read_bytes()
    with tempfile.TemporaryDirectory(prefix='autorouter-r36-geometrisch-') as folder:
        candidate = Path(folder)/'routes.json'; result = Path(folder)/'info.json'
        cmd = [str(Path(__file__).resolve().parents[1]/'.venv/bin/python'),
               '-m', 'lernen.stromzonen', '_geometrisch_worker', str(bj), str(kj), str(rj),
               str(candidate), str(result), json.dumps(gndlagen), str(int(gnd))]
        run = PL.lauf(['caffeinate', '-i', '-s'] + cmd, cpu_s=cpu_s,
                      cwd=str(Path(__file__).resolve().parents[1]),
                      log=str(Path(folder)/'worker.log'), env=dict(os.environ, PYTHONHASHSEED='0'))
        info = json.loads(result.read_text()) if run['zustand'] == 'ok' and result.exists() else dict(
            angewendet=False, grund='R36-Geometrie: ' + run['zustand'])
        info['rechenplatz'] = run
        if info.get('angewendet'):
            if not candidate.exists():
                return dict(info, angewendet=False, grund='R36-Geometrie ohne Routenausgabe')
            if Path(rj).read_bytes() != original:
                return dict(info, angewendet=False, grund='Route waehrend Nachlauf geaendert')
            target = Path(aus or rj)
            with tempfile.NamedTemporaryFile(dir=target.parent, prefix='.r36_', delete=False) as f:
                tmp = f.name; f.write(candidate.read_bytes())
            try: os.replace(tmp, target)
            finally:
                if os.path.exists(tmp): os.unlink(tmp)
        return info

def _geometrisch_intern(bj, kj, rj, gndlagen=None, aus=None, gnd=True, _k17_proposal=False):
    ""
    B, K, R = [json.loads(Path(p).read_text()) for p in (bj,kj,rj)]
    from lernen.zonen_richter_auswahl import an as richter_an
    if richter_an(R):
        raise ValueError('zonen_richter=1: geometrischer Ersatz ohne native Abnahme gesperrt')
    R['zonen'] = [z for z in R.get('zonen', []) if z.get('quelle') not in ('stromzonen', 'gnd_fuellung')]
    R['vias'] = [v for v in R.get('vias', []) if v.get('quelle') not in ('gnd_fuellung', 'strom_via')]
    eingang = _routing_pruefen(B, R, K, gndlagen)
    if not eingang['gueltig']:
        return dict(angewendet=False, ersatz_gesperrt='ungueltige_eingaberoute', eingangspruefung=eingang,
                    grund='R36: geometrischer Ersatz nur auf gueltiger Eingaberoute')
    if str(R.get('r36_k17', '0')).lower() in ('1', 'true') and not _k17_proposal:
        return _geometrisch_k17(bj,kj,rj,R,K,B,gndlagen,aus,gnd)
    alt = _vorschlag(rj, R) if gnd else None
    if alt is not None:
        R['zonen'], R['vias'] = alt['zonen'], alt['vias']
        zs = [z for z in R['zonen'] if z.get('quelle') == 'stromzonen']; gz = [z for z in R['zonen'] if z.get('quelle') == 'gnd_fuellung']
        info = dict(angewendet=True, eingangspruefung=eingang, stromzonen=len(zs), netze=sorted({z['net'] for z in zs}), wiederverwendet=True,
                    flaeche_mm2=round(sum(z.get('flaeche_mm2', 0) for z in zs), 1), eigen_mm2=round(sum(z.get('eigen_mm2', 0) for z in zs), 1),
                    gnd_zonen=len(gz), gnd_flaeche_mm2=round(sum(z.get('flaeche_mm2', 0) for z in gz), 1))
        json.dump(R, open(aus or rj, 'w'))
        return info
    zs = zonen(B, R, K, gndlagen)
    from lernen.stromzonen_topo import skelett_anwenden
    n_skelett = skelett_anwenden(R, zs); R['zonen'] += zs
    info = dict(angewendet=True, eingangspruefung=eingang, stromzonen=len(zs), netze=sorted({z['net'] for z in zs}),
                **({'skelett_bahnen': n_skelett} if n_skelett else {}),
                flaeche_mm2=round(sum(z['flaeche_mm2'] for z in zs), 1), eigen_mm2=round(sum(z['eigen_mm2'] for z in zs), 1))
    if gnd:
        gz, gv = gnd_fuellung(B, R, K, gndlagen); R['zonen'] += gz; R['vias'] += gv
        info.update(gnd_zonen=len(gz), gnd_vias=len(gv), gnd_flaeche_mm2=round(sum(z['flaeche_mm2'] for z in gz), 1))
    Path(aus or rj).write_text(json.dumps(R))
    return info


def _geometrisch_k17(bj,kj,rj,R,K,B,gndlagen,aus,gnd):
    ""
    import copy
    import tempfile
    from analyse.bewertung import bewerte
    from analyse.k17 import messen
    from lernen.k17_kontur import annahme
    with tempfile.TemporaryDirectory(prefix='autorouter-k17-paar-') as tmp:
        tmp = Path(tmp)
        before = copy.deepcopy(R)
        before.pop('r36_k17',None)
        (tmp/'basis.json').write_text(json.dumps(before))
        (tmp/'vorschlag.json').write_text(json.dumps(R))
        ia = _geometrisch_intern(bj,kj,str(tmp/'basis.json'),gndlagen,str(tmp/'basis_aus.json'),gnd)
        if not ia.get('angewendet') or not (tmp/'basis_aus.json').exists():
            return dict(angewendet=False,grund='K17: Standardabschluss fehlt',basis=ia)
        try:
            ib = _geometrisch_intern(bj,kj,str(tmp/'vorschlag.json'),gndlagen,str(tmp/'vorschlag_aus.json'),gnd,
                                    _k17_proposal=True)
        except Exception as exc:
            ib = dict(angewendet=False,fehler=repr(exc))
        a = json.loads((tmp/'basis_aus.json').read_text())
        b = json.loads((tmp/'vorschlag_aus.json').read_text()) if ib.get('angewendet') else None
        try:
            if b is None:
                raise ValueError('Vorschlagsabschluss fehlt: '+str(ib.get('fehler') or ib.get('grund')))
            gate = annahme(bewerte(B,a,K,v3=True),bewerte(B,b,K,v3=True),messen(B,a,K),messen(B,b,K))
        except Exception as exc:
            gate = dict(bestanden=False,fehler=repr(exc),fehlt=['vollstaendige Paarabnahme'])
        chosen,info = (b,ib) if gate['bestanden'] else (a,ia)
        chosen['r36_k17'] = True
        chosen['r36_k17_abnahme'] = gate
        Path(aus or rj).write_text(json.dumps(chosen))
        return dict(info,k17_abnahme=gate,k17_rueckfall=not gate['bestanden'])


def _empfaenger_erlaubt(B,K,vorher,nachher):
    from analyse.laengenimpedanz import laengen,empfaenger_schutz,empfaenger_klassen
    kk=empfaenger_klassen(K)
    return empfaenger_schutz(laengen(B,vorher,kk),laengen(B,nachher,kk))


def anwenden(bj, kj, rj, gndlagen=None, aus=None, gnd=True, nativ=True, regeln=None, cpu_s=720, zonenplan=None, k20=False, zonen_richter=None, zonen_richter_modell=None):
    ""
    bj, kj, rj = (str(Path(p).resolve()) for p in (bj, kj, rj))
    if aus is not None: aus = str(Path(aus).resolve())
    if regeln: regeln = str(Path(regeln).resolve())
    if zonenplan: zonenplan = str(Path(zonenplan).resolve())
    original=Path(rj).read_bytes()
    route = json.loads(original)
    if zonen_richter is not None:
        from lernen.zonen_richter_auswahl import an as _richter_an
        route['optionen']=dict(route.get('optionen') or {}, zonen_richter=str(int(_richter_an(route,zonen_richter))))
    if zonen_richter_modell is not None:
        route['optionen']=dict(route.get('optionen') or {}, zonen_richter_modell=str(zonen_richter_modell))
    if route.get('r36_korridor') and (route.get('r36_form') or 'topo') == 'topo':
        raise ValueError("r36_korridor=1 ist mit form='topo' nicht implementiert")
    if k20 and (not nativ or zonenplan):
        raise ValueError('K20-Korridor braucht native Abnahme ohne lokalen Zonenplan')
    from lernen.zonen_richter_auswahl import an as richter_an
    if richter_an(route) and (not nativ or zonenplan or k20):
        raise ValueError('zonen_richter=1 braucht native Abnahme ohne lokalen Plan/K20')
    if not nativ:
        return _geometrisch(bj,kj,rj,gndlagen,aus,gnd,cpu_s=cpu_s)
    from gemeinsam import plaetze as PL
    import tempfile
    root=Path(__file__).resolve().parents[1]
    target=Path(aus or rj)
    planfile=Path(rj).parent/'zonenplan/zonenplan.json'
    local_plan=False
    if k20 and planfile.exists():
        try: local_plan=(json.loads(planfile.read_text()).get('kontext') or {}).get('modus') != 'diagnose'
        except (OSError,ValueError): local_plan=True
    if k20 and (route.get('zonenplan_id') or (route.get('stromzonen_abnahme') or {}).get('zonenplan_id') or
                local_plan):
        return dict(angewendet=False,grund='K20 mit lokalem Zonenplan gesperrt',
                    rechenplatz=dict(cpu_s=0.0,zustand='nicht_gestartet'))
    artifact=Path(str(Path(rj).with_suffix(''))+('_k20_stromzonen' if k20 else '_stromzonen'))
    artifact.mkdir(parents=True,exist_ok=True)
    source=artifact/'eingabe.json'
    if k20: route['k20_korridor'] = True
    if gndlagen is not None:route['gndlagen']=gndlagen.split(',') if isinstance(gndlagen,str) else list(gndlagen)
    source.write_text(json.dumps(route))
    result=artifact/'abnahme.json';candidate=artifact/'gefuellt_routes.json'
    result.unlink(missing_ok=True);candidate.unlink(missing_ok=True)
    cmd=['caffeinate','-i','-s',str(root/'.venv/bin/python'),'-m','lernen.stromzonen_nachlauf',
         str(Path(bj).resolve()),str(Path(kj).resolve()),str(source.resolve()),regeln or '',
         str(result.resolve()),str(candidate.resolve()),str(int(gnd))]
    if zonenplan: cmd.append(str(Path(zonenplan).resolve()))

    run=PL.lauf(cmd,cpu_s=cpu_s,log=str(artifact/'worker.log'),cwd=str(root))
    info=json.loads(result.read_text()) if run['zustand']=='ok' and result.exists() else dict(
        angewendet=False,grund='Native Stromzonen: '+run['zustand'])
    import hashlib
    info.update(rechenplatz=run, artefaktordner=str(artifact.resolve()),
                eingabe_sha256=hashlib.sha256(original).hexdigest(),
                worker_eingabe_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                ausgabe_sha256=hashlib.sha256(candidate.read_bytes()).hexdigest() if candidate.exists() else None)
    if k20 and run['cpu_s'] >= cpu_s:
        info.update(angewendet=False, grund='K20: Prozessbaum-CPU-Budget erschoepft')
    if info['angewendet']:
        if k20 and (not (info.get('k20_tor') or {}).get('angenommen') or not info.get('empfaengertor')):
            info.update(angewendet=False, grund='K20: Worker-Zertifikat fehlt')
            result.write_text(json.dumps(info,indent=1))
            return info
        if Path(rj).read_bytes()!=original:
            info.update(angewendet=False,grund='Route während nativer Prüfung geändert')
            return info


        try:
            if not k20:
                B=json.loads(Path(bj).read_text());K=json.loads(Path(kj).read_text())
                from lernen.form import empfaengertor_an
                info['empfaengertor']=_empfaenger_erlaubt(B,K,json.loads(original),json.loads(candidate.read_bytes())) if empfaengertor_an(K) else True
        except Exception as exc:
            info.update(empfaengertor=False,empfaenger_fehler=repr(exc))
        if not info['empfaengertor']:
            info.update(angewendet=False,grund='Empfaengertor: nativer Zonennachlauf')
            result.write_text(json.dumps(info,indent=1))
            return info
        target.parent.mkdir(parents=True,exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=target.parent,prefix='.stromzonen_',delete=False) as f:
            tmp=f.name;f.write(candidate.read_bytes())
        try:
            if target.resolve()==Path(rj).resolve():
                Path(str(Path(rj).with_suffix(''))+('_vor_k20_stromzonen.json' if k20 else '_vor_stromzonen.json')).write_bytes(original)
            os.replace(tmp,target)
        finally:
            if os.path.exists(tmp):os.unlink(tmp)
    result.write_text(json.dumps(info,indent=1))
    return info


def pruefen(B, R, K, gndlagen=None, clr=None, rand=0.25):
    ""
    clr = clr if clr is not None else max(0.15, float(R.get('clr') or (R.get('optionen') or {}).get('clr') or 0.1))
    clr_r = float(R.get('clr') or (R.get('optionen') or {}).get('clr') or 0.09)
    umriss = unary_union([Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3]).buffer(-rand + 1e-3, QS)
    fehler = []; R0 = dict(R, zonen=[])
    teile_l = {}
    for z in R.get('zonen', []):
        g = _geo(z)
        if z['layer'] not in teile_l: teile_l[z['layer']] = _teile(B, R0, z['layer'])
        t = teile_l[z['layer']]
        eigen = unary_union([h for m, h in t if m == z['net']]); neu = g.difference(eigen.buffer(TOL))
        if not neu.is_empty and not umriss.contains(neu.buffer(-1e-4)): fehler.append((z['net'], z['layer'], 'rand', round(neu.difference(umriss).area, 4)))
        andere = [h for m, h in t if m != z['net']] + \
                 [_geo(y) for y in R.get('zonen', []) if y is not z and y['layer'] == z['layer'] and y['net'] != z['net']]
        if andere:
            a = unary_union(andere)
            if not neu.is_empty and neu.distance(a) < clr - TOL: fehler.append((z['net'], z['layer'], 'abstand_neu', round(neu.distance(a), 4)))


            if g.distance(a) < min(clr_r, eigen.distance(a)) - TOL: fehler.append((z['net'], z['layer'], 'abstand', round(g.distance(a), 4)))
    return fehler


def main():
    if sys.argv[1:2] == ['_geometrisch_worker']:
        bj, kj, rj, aus, result, gl, gnd = sys.argv[2:]
        info = _geometrisch_intern(bj, kj, rj, json.loads(gl), aus, gnd == '1')
        Path(result).write_text(json.dumps(info))
        return
    a = sys.argv[1:]; kw = dict(x.split('=', 1) for x in a if '=' in x); p = [x for x in a if '=' not in x]
    gl = kw.get('gndlagen'); gl = gl.split(',') if gl else None
    info = anwenden(p[0], p[1], p[2], gl, aus=p[3] if len(p) > 3 else None, gnd=kw.get('gnd', '1') == '1',
                    nativ=kw.get('nativ','1')=='1',regeln=kw.get('regeln'),
                    zonen_richter=kw.get('zonen_richter'),
                    zonen_richter_modell=kw.get('zonen_richter_modell'))
    print(json.dumps(info, ensure_ascii=False))


if __name__ == '__main__':
    main()
