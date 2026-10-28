""
import json, math, sys, collections
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gemeinsam.bahnitems import arc_geometry
import shapely
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import nearest_points
from shapely.strtree import STRtree

PAR_GRAD, PAR_D, PAR_L = 15.0, 0.6, 0.8
SPITZ_GRAD = 60.0
RAND_PAD, RAND_STAMM = 0.3, 0.6
NEBEN_D = 0.8
ARM_R = 0.15


def _measurement_routes(R):
    ""
    out=dict(R); tracks=[]
    for t in R['tracks']:
        if 'items' not in t:
            tracks.append(t); continue
        pts=[]; widths=[]
        for it in t['items']:
            if it[0]=='L': part=[it[1],it[2]]
            elif it[0]=='A':
                c,r,ang,sweep=arc_geometry(it)
                step=2*math.acos(max(-1.,min(1.,1-.00025/r)))
                n=max(1,math.ceil(abs(sweep)/max(step,1e-8)))
                part=[(c[0]+r*math.cos(ang+sweep*i/n),c[1]+r*math.sin(ang+sweep*i/n)) for i in range(n+1)]
                part[0],part[-1]=it[1],it[3]
            else: raise ValueError('unbekanntes Bahnitem: '+str(it[0]))
            if not pts: pts.append(part[0])
            pts.extend(part[1:]); widths.extend([it[-1]]*(len(part)-1))
        tracks.append(dict(t,pts=pts,w=widths))
    out['tracks']=sorted(tracks,key=lambda t:json.dumps(t,sort_keys=True))
    return out


def _parallel_runs(S, R, flat_segments=False, self_pairs=False):
    ""
    geometries=[LineString([s[1],s[2]]) for s in S]
    tree=STRtree(geometries); offset=[]; totals=collections.defaultdict(float)
    for seg,g in zip(S,geometries):
        offset.append(totals[seg[0]]); totals[seg[0]]+=g.length
    intervals=collections.defaultdict(list)
    paths={ti:LineString(R['tracks'][ti]['pts']) for ti in totals}
    outer={ti:g.buffer(PAR_D,cap_style='flat') for ti,g in paths.items()}
    inner={ti:g.buffer(.02) for ti,g in paths.items()}


    bends = collections.defaultdict(list)
    for i, (ti, a, b, *_rest) in enumerate(S):
        heading = math.atan2(b[1]-a[1], b[0]-a[0])
        rows = bends[ti]
        turn = abs((heading-rows[-1][1]+math.pi) % (2*math.pi)-math.pi) if rows else 0.
        rows.append((i, heading, (rows[-1][2] if rows else 0.)+turn))
    bend_at = {i: total for rows in bends.values() for i, _, total in rows}
    bend_total = {}
    for ti, rows in bends.items():
        seam = abs((rows[0][1]-rows[-1][1]+math.pi) % (2*math.pi)-math.pi)
        bend_total[ti] = rows[-1][2]+seam
    for i,(ti,a,b,wa_,sa) in enumerate(S):
        la=geometries[i]; wa=math.atan2(b[1]-a[1],b[0]-a[0]); puf=la.buffer(PAR_D, cap_style='flat' if flat_segments else 'round')
        for j in tree.query(puf):
            j=int(j); tj,c,d,wb_,sb=S[j]
            if tj==ti:
                if not self_pairs or i == j: continue
                turning = abs(bend_at[i]-bend_at[j])
                if paths[ti].is_closed: turning = min(turning, bend_total[ti]-turning)
                if turning < math.radians(60)-1e-8: continue
            wb=math.atan2(d[1]-c[1],d[0]-c[0]); dw=abs((wa-wb+math.pi/2)%math.pi-math.pi/2)
            if dw>math.radians(PAR_GRAD): continue
            lb=geometries[j]
            near=puf.intersection(lb)
            near = near.difference(la.buffer(.02)) if ti == tj else near.intersection(outer[ti]).difference(inner[ti])
            pieces=[near] if near.geom_type=='LineString' else list(getattr(near,'geoms',[]))
            for g in pieces:
                if g.geom_type!='LineString' or g.is_empty: continue
                lo,hi=sorted(lb.project(Point(p)) for p in (g.coords[0],g.coords[-1]))
                if hi-lo>1e-8: intervals[(ti,tj)].append((offset[j]+lo,offset[j]+hi))
    directed={}
    for pair,spans in sorted(intervals.items()):
        merged=[]
        for lo,hi in sorted(spans):
            if merged and lo<=merged[-1][1]+1e-7: merged[-1][1]=max(merged[-1][1],hi)
            else: merged.append([lo,hi])
        directed[pair]=math.fsum(hi-lo for lo,hi in merged if hi-lo>PAR_L)
    for ti,tj in sorted({tuple(sorted(p)) for p in directed}):
        a,b=R['tracks'][ti],R['tracks'][tj]
        art='abgriff' if a.get('sig') or b.get('sig') else 'kette' if a.get('kette') or b.get('kette') else 'rest'
        value = directed.get((ti, ti), 0.)/2 if ti == tj else (directed.get((ti,tj),0.)+directed.get((tj,ti),0.))/2
        if value>0: yield ti,tj,art,value


def _segmente(R):
    d = collections.defaultdict(list)
    for ti, t in enumerate(R['tracks']):
        for si, (a, b) in enumerate(zip(t['pts'][:-1], t['pts'][1:])):
            if math.dist(a, b) > 1e-6: d[(t['net'], t['layer'])].append((ti, tuple(a), tuple(b), t['w'][si] if si < len(t['w']) else t['w'][-1], bool(t.get('sig'))))
    return d


def _pads(B):
    ""
    P = collections.defaultdict(list)
    if not B: return P
    for p in B['pads']:
        if not p.get('net'): continue
        for L, polys in p['polys'].items():
            g = shapely.union_all([Polygon(q) for q in polys if len(q) >= 3])
            if not g.is_empty: P[(p['net'], L.replace('.Cu', ''))].append(g)
    return P


def _fuss(p, a, b):
    ""
    dx, dy = b[0] - a[0], b[1] - a[1]; ll = dx * dx + dy * dy
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / ll)) if ll > 0 else 0.0
    return (a[0] + t * dx, a[1] + t * dy)


def _abst_punkt_seg(p, a, b):
    return math.dist(p, _fuss(p, a, b))


def _arm(pts, i, schritt, p, r=ARM_R):
    ""
    q = None; k = i
    while 0 <= k < len(pts):
        q = pts[k]
        if math.dist(q, p) >= r: break
        k += schritt
    if q is None or math.dist(q, p) < 0.02: return None
    return math.degrees(math.atan2(q[1] - p[1], q[0] - p[0])) % 360.0


def topologie(R, B=None, bsp_max=8):
    R = _measurement_routes(R)
    D = _segmente(R); PADS = _pads(B)
    clr = float(R.get('clr') or 0.13)
    try: via_d = float(str((R.get('optionen') or {}).get('via', '0.45/0.20')).split('/')[0])
    except ValueError: via_d = 0.45
    vias = collections.defaultdict(list)
    for v in R['vias']: vias[v['net']].append((v['x'], v['y']))
    par = dict(abgriff=collections.Counter(), kette=collections.Counter(), rest=collections.Counter()); bsp_par = []
    kn = collections.Counter(); bsp_kn = []
    abg = []
    via_im_pad = 0; via_gezaehlt = set()
    for (net, L), S in sorted(D.items()):

        for ti,tj,art,nah in _parallel_runs(S,R):
            par[art][net]+=nah
            a,b=R['tracks'][ti],R['tracks'][tj]
            bsp_par.append((round(nah,2),art,net,L,
                            [round(v,2) for p in (a['pts'][0],a['pts'][-1]) for v in p],
                            [round(v,2) for p in (b['pts'][0],b['pts'][-1]) for v in p],
                            round(LineString(a['pts']).distance(LineString(b['pts'])),3)))

        pk = PADS.get((net, L), [])
        ts = [t for t in R['tracks'] if t['net'] == net and t['layer'] == L]
        zuege = [[tuple(q) for q in t['pts']] for t in ts]


        kand = sorted({tuple(q) for t in ts for q in (
            [p for it in t['items'] for p in (it[1], it[2] if it[0]=='L' else it[3])]
            if 'items' in t else t['pts'])})


        geometrien=[LineString([s[1],s[2]]) for s in S]
        baum=STRtree(geometrien)
        kreuze=set()
        for i,g in enumerate(geometrien):
            for j in baum.query(g):
                j=int(j)
                if j<=i:continue
                if S[i][0]==S[j][0] and any(math.dist(a,b)<1e-8 for a in S[i][1:3] for b in S[j][1:3]):continue
                hit=g.intersection(geometrien[j])
                if hit.geom_type=='Point' and not hit.is_empty:kreuze.add((hit.x,hit.y))
                elif hit.geom_type=='MultiPoint':kreuze.update((p.x,p.y) for p in hit.geoms)
        kand=sorted(set(kand)|kreuze);fertig=[]
        for p in kand:
            if any(math.dist(p, q) < 0.05 for q in fertig): continue
            roh = []; mitte = False; enden = 0
            for P in zuege:
                nahe = []
                for k, (a, b) in enumerate(zip(P[:-1], P[1:])):
                    if min(a[0], b[0]) - 0.04 > p[0] or max(a[0], b[0]) + 0.04 < p[0] or min(a[1], b[1]) - 0.04 > p[1] or max(a[1], b[1]) + 0.04 < p[1]: continue
                    f = _fuss(p, a, b)
                    d = math.dist(p, f)
                    if d <= 0.03: nahe.append((k, d, f))


                gruppen=[]
                for x in nahe:
                    if not gruppen or x[0] > gruppen[-1][-1][0]+1: gruppen.append([])
                    gruppen[-1].append(x)
                for gruppe in gruppen:
                    k, _, f = min(gruppe, key=lambda x: (x[1], x[0]))
                    roh += [_arm(P, k + 1, 1, f), _arm(P, k, -1, f)]
                if math.dist(P[0], p) <= 0.03 or math.dist(P[-1], p) <= 0.03: enden += 1
                elif any(_abst_punkt_seg(p, a, b) <= 0.03 for a, b in zip(P[:-1], P[1:])) and all(math.dist(q, p) > 0.03 for q in P): mitte = True
            arme = []
            for x in sorted(x for x in roh if x is not None):
                if not arme or (x - arme[-1]) > 3.0: arme.append(x)
            if len(arme) > 1 and (arme[0] + 360.0 - arme[-1]) <= 3.0: arme.pop()
            if len(arme) < 3: continue
            fertig.append(p)
            lu = [(arme[(k + 1) % len(arme)] - arme[k]) % 360 for k in range(len(arme))]
            im_pad = any(g.buffer(0.03).contains(Point(p)) for g in pk) if pk else False
            kn['knoten'] += 1; kn['knoten_pad'] += im_pad; kn['abzweig_mitte'] += mitte
            if len(arme) > 3: kn['stern'] += 1; kn['stern_pad'] += im_pad
            if min(lu) < SPITZ_GRAD: kn['spitz'] += 1; kn['spitz_pad'] += im_pad
            if len(arme) > 3 or min(lu) < SPITZ_GRAD:
                bsp_kn.append((len(arme), round(min(lu), 1), net, L, (round(p[0], 2), round(p[1], 2)), 'im Pad' if im_pad else 'frei'))

        sig_ti = sorted({s[0] for s in S if s[4]})
        if not sig_ti: continue
        nsig = [s for s in S if not s[4]]
        sig_enden = [tuple(q) for t in R['tracks'] if t['net'] == net and t.get('sig') for q in (t['pts'][0], t['pts'][-1])]
        eig_vias = []; abg_vias = []
        for v in vias[net]:
            (abg_vias if any(math.dist(v, e) < 0.05 for e in sig_enden) else eig_vias).append(v)
        netz_pads = [g for (n2, L2), gs in PADS.items() if n2 == net for g in gs]
        end_pads = [g for g in netz_pads if any(g.buffer(0.03).contains(Point(e)) for e in sig_enden)]
        for v in abg_vias:
            if v not in via_gezaehlt and any(g.contains(Point(v)) for g in netz_pads) and not any(g.contains(Point(v)) for g in end_pads):
                via_gezaehlt.add(v); via_im_pad += 1
        kupfer = [LineString([a, b]).buffer(w / 2) for _, a, b, w, _ in nsig] + [Point(v).buffer(via_d / 2) for v in eig_vias]
        for ti in sig_ti:
            t = R['tracks'][ti]; pts = [tuple(q) for q in t['pts']]; anker = []; art = 'kelvin'; eigene_pads = []
            for e in (pts[0], pts[-1]):
                hit = [g for g in pk if g.buffer(0.03).contains(Point(e))]
                if hit: anker += [g.buffer(RAND_PAD) for g in hit]; eigene_pads += hit; continue
                if any(math.dist(e, v) < 0.05 for v in abg_vias): continue
                if not t.get('kelvin') and any(_abst_punkt_seg(e, a, b) < w / 2 + 0.03 for _, a, b, w, _ in nsig):
                    art = 'abzweig'; anker.append(Point(e).buffer(RAND_STAMM))

            A = shapely.union_all(anker) if anker else None
            O = shapely.union_all(kupfer + [g for g in pk if not any(g is h for h in eigene_pads)]) if (kupfer or pk) else None
            ws = t['w']; teile = [LineString([a, b]).buffer((ws[i] if i < len(ws) else ws[-1]) / 2) for i, (a, b) in enumerate(zip(pts[:-1], pts[1:])) if math.dist(a, b) > 1e-6]
            if not teile: continue
            T = shapely.union_all(teile); lin = LineString(pts)
            if A is not None: T = T.difference(A); lin = lin.difference(A)
            if O is not None and A is not None: O = O.difference(A)
            lg = lin.length if not lin.is_empty else 0.0
            if O is None or O.is_empty or T.is_empty:
                abg.append(dict(net=net, lage=L, art=art, abstand=None, nah_mm=0.0, ort=None, laenge=lg, neben_mm=0.0)); continue
            Ks = shapely.union_all([LineString([a, b]).buffer(w / 2) for _, a, b, w, _ in nsig]) if nsig else None
            if Ks is not None and A is not None: Ks = Ks.difference(A)
            neben = lin.intersection(Ks.buffer(NEBEN_D)).length if (Ks is not None and not Ks.is_empty and not lin.is_empty) else 0.0
            d = T.distance(O); nah = lin.intersection(O.buffer(clr + min(ws) / 2 - 1e-6)).length if not lin.is_empty else 0.0
            ort = None
            if d < clr:
                q = nearest_points(T, O)[0]; ort = (round(q.x, 2), round(q.y, 2))
            abg.append(dict(net=net, lage=L, art=art, abstand=round(d, 3), nah_mm=round(nah, 2), ort=ort, laenge=lg, neben_mm=neben, flag=bool(t.get('kelvin'))))
    kel = [a for a in abg if a['art'] == 'kelvin' and a['abstand'] is not None]
    kfl = [a for a in kel if a.get('flag')]
    alle = par['abgriff'] + par['kette'] + par['rest']
    erg = dict(version='topologie-v4-symmetrisch', parallel_abgriff_mm=round(sum(par['abgriff'].values()), 1), parallel_kette_mm=round(sum(par['kette'].values()), 1),
               parallel_rest_mm=round(sum(par['rest'].values()), 1), parallel_netze=dict(sorted((n, round(v, 1)) for n, v in alle.items())),
               knoten=kn['knoten'], knoten_pad=kn['knoten_pad'], abzweig_mitte=kn['abzweig_mitte'],
               stern=kn['stern'], stern_pad=kn['stern_pad'], spitz=kn['spitz'], spitz_pad=kn['spitz_pad'],
               abgriffe=len(abg), abgriffe_kelvin=sum(1 for a in abg if a['art'] == 'kelvin'), abgriffe_abzweig=sum(1 for a in abg if a['art'] == 'abzweig'),
               abgriff_abstand_min=min((a['abstand'] for a in kel), default=None),
               abgriff_nah_mm=round(sum(a['nah_mm'] for a in kel), 1), abgriff_nah=sum(1 for a in kel if a['abstand'] < clr - 1e-6),
               abgriff_laenge_mm=round(sum(a['laenge'] for a in abg if a['art'] == 'kelvin'), 1),
               abgriff_neben_strom_mm=round(sum(a['neben_mm'] for a in abg if a['art'] == 'kelvin'), 1),
               kelvin_gekennzeichnet=len(kfl), kelvin_abstand_min=min((a['abstand'] for a in kfl), default=None),
               kelvin_nah=sum(1 for a in kfl if a['abstand'] < clr - 1e-6), kelvin_nah_mm=round(sum(a['nah_mm'] for a in kfl), 1),
               kelvin_laenge_mm=round(sum(a['laenge'] for a in kfl), 1), kelvin_neben_strom_mm=round(sum(a['neben_mm'] for a in kfl), 1),
               abgriff_via_im_pad=via_im_pad, mit_board=bool(B), clr=clr)
    erg['beispiele_parallel'] = sorted(bsp_par, reverse=True)[:bsp_max]
    erg['beispiele_knoten'] = sorted(bsp_kn, key=lambda k: (-k[0], k[1]))[:bsp_max]
    erg['beispiele_abgriff'] = sorted((a for a in kel if a['abstand'] < clr - 1e-6), key=lambda a: a['abstand'])[:bsp_max]
    return erg
