import json, math, sys, collections
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.track_items import arc_geometry
import shapely
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import nearest_points
from shapely.strtree import STRtree

PAR_DEG, PAR_D, PAR_L = 15.0, 0.6, 0.8
SHARP_DEG = 60.0
BORDER_PAD, BORDER_TRUNK = 0.3, 0.6
BESIDE_D = 0.8
ARM_R = 0.15


def _measurement_routes(R):
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
            else: raise ValueError('unknown track item: '+str(it[0]))
            if not pts: pts.append(part[0])
            pts.extend(part[1:]); widths.extend([it[-1]]*(len(part)-1))
        tracks.append(dict(t,pts=pts,w=widths))
    out['tracks']=sorted(tracks,key=lambda t:json.dumps(t,sort_keys=True))
    return out


def _parallel_runs(S, R, flat_segments=False, self_pairs=False):
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
            if dw>math.radians(PAR_DEG): continue
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
        category='tap' if a.get('sig') or b.get('sig') else 'link_chain' if a.get('link_chain') or b.get('link_chain') else 'rest'
        value = directed.get((ti, ti), 0.)/2 if ti == tj else (directed.get((ti,tj),0.)+directed.get((tj,ti),0.))/2
        if value>0: yield ti,tj,category,value


def _segments(R):
    d = collections.defaultdict(list)
    for ti, t in enumerate(R['tracks']):
        for si, (a, b) in enumerate(zip(t['pts'][:-1], t['pts'][1:])):
            if math.dist(a, b) > 1e-6: d[(t['net'], t['layer'])].append((ti, tuple(a), tuple(b), t['w'][si] if si < len(t['w']) else t['w'][-1], bool(t.get('sig'))))
    return d


def _pads(B):
    P = collections.defaultdict(list)
    if not B: return P
    for p in B['pads']:
        if not p.get('net'): continue
        for L, polys in p['polys'].items():
            g = shapely.union_all([Polygon(q) for q in polys if len(q) >= 3])
            if not g.is_empty: P[(p['net'], L.replace('.Cu', ''))].append(g)
    return P


def _foot(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]; ll = dx * dx + dy * dy
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / ll)) if ll > 0 else 0.0
    return (a[0] + t * dx, a[1] + t * dy)


def _dist_point_seg(p, a, b):
    return math.dist(p, _foot(p, a, b))


def _arm(pts, i, step, p, r=ARM_R):
    q = None; k = i
    while 0 <= k < len(pts):
        q = pts[k]
        if math.dist(q, p) >= r: break
        k += step
    if q is None or math.dist(q, p) < 0.02: return None
    return math.degrees(math.atan2(q[1] - p[1], q[0] - p[0])) % 360.0


def topology(R, B=None, example_max=8):
    R = _measurement_routes(R)
    D = _segments(R); PADS = _pads(B)
    clr = float(R.get('clr') or 0.13)
    try: via_d = float(str((R.get('options') or {}).get('via', '0.45/0.20')).split('/')[0])
    except ValueError: via_d = 0.45
    vias = collections.defaultdict(list)
    for v in R['vias']: vias[v['net']].append((v['x'], v['y']))
    par = dict(tap=collections.Counter(), link_chain=collections.Counter(), rest=collections.Counter()); example_par = []
    kn = collections.Counter(); example_kn = []
    tap = []
    via_in_pad = 0; via_counted = set()
    for (net, L), S in sorted(D.items()):

        for ti,tj,category,close in _parallel_runs(S,R):
            par[category][net]+=close
            a,b=R['tracks'][ti],R['tracks'][tj]
            example_par.append((round(close,2),category,net,L,
                            [round(v,2) for p in (a['pts'][0],a['pts'][-1]) for v in p],
                            [round(v,2) for p in (b['pts'][0],b['pts'][-1]) for v in p],
                            round(LineString(a['pts']).distance(LineString(b['pts'])),3)))

        pk = PADS.get((net, L), [])
        ts = [t for t in R['tracks'] if t['net'] == net and t['layer'] == L]
        move_records = [[tuple(q) for q in t['pts']] for t in ts]


        cand = sorted({tuple(q) for t in ts for q in (
            [p for it in t['items'] for p in (it[1], it[2] if it[0]=='L' else it[3])]
            if 'items' in t else t['pts'])})


        geometries=[LineString([s[1],s[2]]) for s in S]
        tree_idx=STRtree(geometries)
        crosses=set()
        for i,g in enumerate(geometries):
            for j in tree_idx.query(g):
                j=int(j)
                if j<=i:continue
                if S[i][0]==S[j][0] and any(math.dist(a,b)<1e-8 for a in S[i][1:3] for b in S[j][1:3]):continue
                hit=g.intersection(geometries[j])
                if hit.geom_type=='Point' and not hit.is_empty:crosses.add((hit.x,hit.y))
                elif hit.geom_type=='MultiPoint':crosses.update((p.x,p.y) for p in hit.geoms)
        cand=sorted(set(cand)|crosses);finished=[]
        for p in cand:
            if any(math.dist(p, q) < 0.05 for q in finished): continue
            raw_data = []; middle = False; ends = 0
            for P in move_records:
                nearby = []
                for k, (a, b) in enumerate(zip(P[:-1], P[1:])):
                    if min(a[0], b[0]) - 0.04 > p[0] or max(a[0], b[0]) + 0.04 < p[0] or min(a[1], b[1]) - 0.04 > p[1] or max(a[1], b[1]) + 0.04 < p[1]: continue
                    f = _foot(p, a, b)
                    d = math.dist(p, f)
                    if d <= 0.03: nearby.append((k, d, f))


                group_list=[]
                for x in nearby:
                    if not group_list or x[0] > group_list[-1][-1][0]+1: group_list.append([])
                    group_list[-1].append(x)
                for group in group_list:
                    k, _, f = min(group, key=lambda x: (x[1], x[0]))
                    raw_data += [_arm(P, k + 1, 1, f), _arm(P, k, -1, f)]
                if math.dist(P[0], p) <= 0.03 or math.dist(P[-1], p) <= 0.03: ends += 1
                elif any(_dist_point_seg(p, a, b) <= 0.03 for a, b in zip(P[:-1], P[1:])) and all(math.dist(q, p) > 0.03 for q in P): middle = True
            arms = []
            for x in sorted(x for x in raw_data if x is not None):
                if not arms or (x - arms[-1]) > 3.0: arms.append(x)
            if len(arms) > 1 and (arms[0] + 360.0 - arms[-1]) <= 3.0: arms.pop()
            if len(arms) < 3: continue
            finished.append(p)
            lu = [(arms[(k + 1) % len(arms)] - arms[k]) % 360 for k in range(len(arms))]
            in_pad = any(g.buffer(0.03).contains(Point(p)) for g in pk) if pk else False
            kn['nodes'] += 1; kn['nodes_pad'] += in_pad; kn['branch_middle'] += middle
            if len(arms) > 3: kn['star'] += 1; kn['star_pad'] += in_pad
            if min(lu) < SHARP_DEG: kn['sharp'] += 1; kn['sharp_pad'] += in_pad
            if len(arms) > 3 or min(lu) < SHARP_DEG:
                example_kn.append((len(arms), round(min(lu), 1), net, L, (round(p[0], 2), round(p[1], 2)), 'in pad' if in_pad else 'vacant'))

        sig_ti = sorted({s[0] for s in S if s[4]})
        if not sig_ti: continue
        nsig = [s for s in S if not s[4]]
        sig_ends = [tuple(q) for t in R['tracks'] if t['net'] == net and t.get('sig') for q in (t['pts'][0], t['pts'][-1])]
        mine_vias = []; tap_vias = []
        for v in vias[net]:
            (tap_vias if any(math.dist(v, e) < 0.05 for e in sig_ends) else mine_vias).append(v)
        net_pads = [g for (n2, L2), gs in PADS.items() if n2 == net for g in gs]
        end_pads = [g for g in net_pads if any(g.buffer(0.03).contains(Point(e)) for e in sig_ends)]
        for v in tap_vias:
            if v not in via_counted and any(g.contains(Point(v)) for g in net_pads) and not any(g.contains(Point(v)) for g in end_pads):
                via_counted.add(v); via_in_pad += 1
        copper_map = [LineString([a, b]).buffer(w / 2) for _, a, b, w, _ in nsig] + [Point(v).buffer(via_d / 2) for v in mine_vias]
        for ti in sig_ti:
            t = R['tracks'][ti]; pts = [tuple(q) for q in t['pts']]; anchor = []; category = 'kelvin'; own_pads = []
            for e in (pts[0], pts[-1]):
                hit = [g for g in pk if g.buffer(0.03).contains(Point(e))]
                if hit: anchor += [g.buffer(BORDER_PAD) for g in hit]; own_pads += hit; continue
                if any(math.dist(e, v) < 0.05 for v in tap_vias): continue
                if not t.get('kelvin') and any(_dist_point_seg(e, a, b) < w / 2 + 0.03 for _, a, b, w, _ in nsig):
                    category = 'branch'; anchor.append(Point(e).buffer(BORDER_TRUNK))

            A = shapely.union_all(anchor) if anchor else None
            O = shapely.union_all(copper_map + [g for g in pk if not any(g is h for h in own_pads)]) if (copper_map or pk) else None
            ws = t['w']; part_list = [LineString([a, b]).buffer((ws[i] if i < len(ws) else ws[-1]) / 2) for i, (a, b) in enumerate(zip(pts[:-1], pts[1:])) if math.dist(a, b) > 1e-6]
            if not part_list: continue
            T = shapely.union_all(part_list); lin = LineString(pts)
            if A is not None: T = T.difference(A); lin = lin.difference(A)
            if O is not None and A is not None: O = O.difference(A)
            lg = lin.length if not lin.is_empty else 0.0
            if O is None or O.is_empty or T.is_empty:
                tap.append(dict(net=net, layer_name=L, category=category, spacing=None, close_mm=0.0, location=None, total_length=lg, beside_mm=0.0)); continue
            Ks = shapely.union_all([LineString([a, b]).buffer(w / 2) for _, a, b, w, _ in nsig]) if nsig else None
            if Ks is not None and A is not None: Ks = Ks.difference(A)
            beside = lin.intersection(Ks.buffer(BESIDE_D)).length if (Ks is not None and not Ks.is_empty and not lin.is_empty) else 0.0
            d = T.distance(O); close = lin.intersection(O.buffer(clr + min(ws) / 2 - 1e-6)).length if not lin.is_empty else 0.0
            location = None
            if d < clr:
                q = nearest_points(T, O)[0]; location = (round(q.x, 2), round(q.y, 2))
            tap.append(dict(net=net, layer_name=L, category=category, spacing=round(d, 3), close_mm=round(close, 2), location=location, total_length=lg, beside_mm=beside, flag=bool(t.get('kelvin'))))
    kel = [a for a in tap if a['category'] == 'kelvin' and a['spacing'] is not None]
    kfl = [a for a in kel if a.get('flag')]
    all_items = par['tap'] + par['link_chain'] + par['rest']
    erg = dict(version='topology-v4-symmetric', parallel_tap_mm=round(sum(par['tap'].values()), 1), parallel_chain_mm=round(sum(par['link_chain'].values()), 1),
               parallel_rest_mm=round(sum(par['rest'].values()), 1), parallel_nets=dict(sorted((n, round(v, 1)) for n, v in all_items.items())),
               nodes=kn['nodes'], nodes_pad=kn['nodes_pad'], branch_middle=kn['branch_middle'],
               star=kn['star'], star_pad=kn['star_pad'], sharp=kn['sharp'], sharp_pad=kn['sharp_pad'],
               taps=len(tap), taps_kelvin=sum(1 for a in tap if a['category'] == 'kelvin'), taps_branch=sum(1 for a in tap if a['category'] == 'branch'),
               tap_spacing_min=min((a['spacing'] for a in kel), default=None),
               tap_close_mm=round(sum(a['close_mm'] for a in kel), 1), tap_close=sum(1 for a in kel if a['spacing'] < clr - 1e-6),
               tap_length_mm=round(sum(a['total_length'] for a in tap if a['category'] == 'kelvin'), 1),
               tap_beside_current_mm=round(sum(a['beside_mm'] for a in tap if a['category'] == 'kelvin'), 1),
               kelvin_marked=len(kfl), kelvin_spacing_min=min((a['spacing'] for a in kfl), default=None),
               kelvin_close=sum(1 for a in kfl if a['spacing'] < clr - 1e-6), kelvin_close_mm=round(sum(a['close_mm'] for a in kfl), 1),
               kelvin_length_mm=round(sum(a['total_length'] for a in kfl), 1), kelvin_beside_current_mm=round(sum(a['beside_mm'] for a in kfl), 1),
               tap_via_in_pad=via_in_pad, with_board=bool(B), clr=clr)
    erg['examples_parallel'] = sorted(example_par, reverse=True)[:example_max]
    erg['examples_nodes'] = sorted(example_kn, key=lambda k: (-k[0], k[1]))[:example_max]
    erg['examples_tap'] = sorted((a for a in kel if a['spacing'] < clr - 1e-6), key=lambda a: a['spacing'])[:example_max]
    return erg
