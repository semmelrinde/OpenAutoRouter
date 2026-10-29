import collections
import math

import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import spsolve


def canonical(tracks):
    bands = collections.defaultdict(list)
    for t in tracks:
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            if math.dist(a, b) > 1e-10:
                a,b = sorted(tuple(round(x,6) for x in p) for p in (a,b))
                angle = round(math.atan2(b[1]-a[1],b[0]-a[0]),12)
                c,s = math.cos(angle),math.sin(angle)
                x0,x1 = (round(c*p[0]+s*p[1],6) for p in (a,b))
                y = round(-s*a[0]+c*a[1],6)
                bands[t['net'],t['layer'],angle,bool(t.get('sig'))].append((x0,x1,y-w/2,y+w/2))
    group_list = collections.defaultdict(list)
    for (n,L,angle,sig), strips in sorted(bands.items()):
        c,s = math.cos(angle),math.sin(angle)
        cuts = sorted({x for strip in strips for x in strip[:2]})
        for x0,x1 in zip(cuts[:-1],cuts[1:]):
            spans = sorted((y0,y1) for a,b,y0,y1 in strips if a<x1 and b>x0)
            merged = []
            for y0,y1 in spans:
                if merged and y0<=merged[-1][1]+1e-10: merged[-1][1]=max(merged[-1][1],y1)
                else: merged.append([y0,y1])
            for y0,y1 in merged:
                y=(y0+y1)/2; w=round(y1-y0,6)
                pts=[(round(c*x-s*y,6),round(s*x+c*y,6)) for x in (x0,x1)]
                group_list[n,L,w,sig].append(LineString(pts))
    out = []
    for (n, L, w, sig), ls in sorted(group_list.items()):
        g = shapely.line_merge(shapely.union_all(ls))
        for line in getattr(g, 'geoms', [g]):

            pts = list(line.simplify(2e-6).coords)
            pts = min(pts, pts[::-1])
            out.append(dict(net=n, layer=L, pts=pts, w=[w] * (len(pts)-1), sig=sig))

    kept=[];occupied={}
    ordered=sorted(out,key=lambda t:(-LineString(t['pts']).length*t['w'][0],t['net'],t['layer'],t['pts'],t['w']))
    for t in ordered:
        key=t['net'],t['layer'],t['sig']
        copper=LineString(t['pts']).buffer(t['w'][0]/2)
        old=occupied.get(key)
        if old is not None and old.buffer(1e-9).covers(copper):continue
        kept.append(t);occupied[key]=copper if old is None else old.union(copper)
    return sorted(kept, key=lambda t: (t['net'], t['layer'], t['pts'], t['w']))


def I_via_max(drill_hole, current_03=1.):
    return current_03*drill_hole/.3


def copper_contacts(tracks, vias=(), stack=()):
    from shapely.ops import nearest_points
    segments = [(t, a, b, w) for t in tracks
                for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w'])]
    lines = [LineString([a,b]) for _,a,b,_ in segments]
    copper = [line.buffer(w/2) for line,(_,_,_,w) in zip(lines,segments)]
    tree = shapely.STRtree(copper)
    cuts = [{0.,1.} for _ in lines]; left=[]
    for i,(t,_,_,_) in enumerate(segments):
        for j in sorted(tree.query(copper[i],predicate='intersects')):
            if j<=i or (t['net'],t['layer']) != (segments[j][0]['net'],segments[j][0]['layer']):continue
            p,q=nearest_points(lines[i],lines[j])
            cuts[i].add(lines[i].project(p,normalized=True))
            cuts[j].add(lines[j].project(q,normalized=True))
            left.append((t['layer'],tuple(p.coords[0]),tuple(q.coords[0])))


    from analysis.reference import via_layers
    for v in vias:
        point=Point(v['x'],v['y']);land=point.buffer(float(v.get('d') or .45)/2)
        span=via_layers(v,list(stack))
        for i in sorted(tree.query(land,predicate='intersects')):
            t=segments[i][0]
            if t['net']!=v['net'] or t['layer'] not in span:continue
            s=lines[i].project(point,normalized=True);q=lines[i].interpolate(s,normalized=True)
            cuts[i].add(s);left.append((t['layer'],tuple(q.coords[0]),(v['x'],v['y'])))
    out=[]
    for (t,_,_,w),line,parts in zip(segments,lines,cuts):
        out.append(dict(t,pts=[list(line.interpolate(s,normalized=True).coords[0]) for s in sorted(parts)],
                        w=[w]*(len(parts)-1)))
    return out,left


def solve(tracks, vias, pads, flow, A, drill, I, inductive=False):
    from common.flow import net_graph
    from analysis.measures import RHO, VIA_SLEEVE, l_plane, via_lc


    TOL = 1e-7
    from analysis.reference import via_layers
    if any(via_layers(v,A['stack']) is None for v in vias):return None
    tracks, contacts = copper_contacts(canonical(tracks),vias,A['stack'])
    if not flow or (not tracks and not vias): return None
    g = net_graph(tracks, vias, pads, TOL)


    extra={L for p in pads for L in p['layers'] if L in A['cu_layers']}-set(g['cu_layers'])
    g['cu_layers']=sorted(set(g['cu_layers'])|extra)
    from analysis.reference import via_layers
    for v in vias:
        span=via_layers(v,A['stack'])
        if span is None:return None
        for L in g['cu_layers']:
            if L not in span:continue
            node=(L,round(v['x']/TOL)*TOL,round(v['y']/TOL)*TOL)
            g['adj'].setdefault(node,[])
            for p in pads:
                if L not in p['layers']:continue
                poly=shapely.union_all([Polygon(q) for q in p.get('polys',{}).get(L,[])])
                if poly.distance(Point(v['x'],v['y']))>float(v.get('d') or .45)/2+TOL:continue
                pk=g['pad_pts'][p['ref']+':'+p['num']]
                g['adj'].setdefault(pk,[]).append((node,None));g['adj'][node].append((pk,None))
    parent = {k: k for k in g['adj']}
    def find(k):
        while parent[k] != k:
            parent[k] = parent[parent[k]]; k = parent[k]
        return k
    def union(a, b):
        a, b = find(a), find(b)
        if a != b: parent[max(a, b)] = min(a, b)
    for L,p,q in contacts:
        u=(L,round(p[0]/TOL)*TOL,round(p[1]/TOL)*TOL)
        v=(L,round(q[0]/TOL)*TOL,round(q[1]/TOL)*TOL)
        if u in parent and v in parent:union(u,v)
    pad_layers = {(p['ref'], p['num']): set(p['layers']) for p in pads}
    edges = []
    for u in sorted(g['adj']):
        for v, kid in g['adj'][u]:
            if u >= v: continue
            if kid is None:

                pk, ck = (u, v) if u[0] == 'PAD' else (v, u)
                if pk[0] == 'PAD' and ck[0] != 'PAD' and ck[0] not in pad_layers[pk[1:]]: continue
                union(u, v)
            elif kid[0] == 's':
                t = tracks[kid[1]]; si = kid[2]; a, b = t['pts'][si:si+2]; w = t['w'][si]
                length = math.dist(a, b) * (kid[4] - kid[3])
                r = l_plane(A, t['layer'], w) * length if inductive else RHO * length / (w * A['cu_layers'][t['layer']]['t'])
                edges.append((u, v, max(r, 1e-15), kid))

    cu_layers = sorted(g['cu_layers'], key=lambda L: A['cu_layers'][L]['z'])
    for vi, v in enumerate(vias):
        span=via_layers(v,A['stack'])
        if span is None:return None
        d = float(v.get('drill') or drill)
        vl=[L for L in cu_layers if L in span]
        for L1, L2 in zip(vl[:-1], vl[1:]):
            u = (L1, round(v['x']/TOL)*TOL, round(v['y']/TOL)*TOL)
            q = (L2, round(v['x']/TOL)*TOL, round(v['y']/TOL)*TOL)
            h = abs(A['cu_layers'][L2]['z'] - A['cu_layers'][L1]['z'])
            r = via_lc(h, d, A, float(v.get('d') or .45))[0] if inductive else RHO*h/(math.pi*(d+VIA_SLEEVE)*VIA_SLEEVE)
            edges.append((u, q, max(r, 1e-15), ('v', vi)))
    edges = [(find(u), find(v), r, kid) for u, v, r, kid in edges if find(u) != find(v)]
    starts = {find(g['pad_pts'][q]) for q in flow.get('source', []) if q in g['pad_pts'] and g['pad_pts'][q] in parent}
    if not starts: return None
    adj = collections.defaultdict(set)
    for u, v, _, _ in edges: adj[u].add(v); adj[v].add(u)
    seen = set(starts); front = list(sorted(starts))
    for u in front:
        for v in sorted(adj[u]):
            if v not in seen: seen.add(v); front.append(v)
    load_list = collections.defaultdict(float); absent = []
    for p, a in sorted((flow.get('load') or {}).items()):
        pk = g['pad_pts'].get(p)
        if pk not in parent or find(pk) not in seen:
            if a > 0: absent.append(p)
        else: load_list[find(pk)] += float(a)
    total = sum(load_list.values())
    if I > 0 and total <= 0: return None
    idx = {k: j for j, k in enumerate(sorted(seen - starts))}
    rhs = np.zeros(len(idx)); rows = []; cols = []; vals = []
    for k, j in idx.items(): rhs[j] = load_list[k] / total * I if total else 0.
    for u, v, r, _ in edges:
        if u not in seen: continue
        c = 1/r
        for a, b in ((u, v), (v, u)):
            if a not in idx: continue
            rows.append(idx[a]); cols.append(idx[a]); vals.append(c)
            if b in idx: rows.append(idx[a]); cols.append(idx[b]); vals.append(-c)
    sol = spsolve(coo_matrix((vals, (rows, cols)), shape=(len(idx), len(idx))).tocsr(), rhs) if idx else []
    U = {k: float(sol[j]) for k, j in idx.items()}; U.update({k: 0. for k in starts})
    currents = collections.defaultdict(float); via_I = [0.] * len(vias); power = 0.
    for u, v, r, kid in edges:
        if u not in seen: continue
        cur = abs(U[u] - U[v])/r; power += cur*cur*r
        if kid[0] == 'v': via_I[kid[1]] = max(via_I[kid[1]], cur)
        else: currents[kid[1:3]] = max(currents[kid[1:3]], cur)
    seg = [(ti, si, t['layer'], a, b, w, math.dist(a,b), currents[(ti,si)], A['cu_layers'][t['layer']]['t'])
           for ti,t in enumerate(tracks) for si,(a,b,w) in enumerate(zip(t['pts'][:-1],t['pts'][1:],t['w']))]
    rest = np.linalg.norm(coo_matrix((vals,(rows,cols)),shape=(len(idx),len(idx))).tocsr() @ np.asarray(sol) - rhs, ord=np.inf) if idx else 0.
    return dict(P=power, dV=max(U.values(), default=0.), seg=seg, via_I=via_I,
                reached=sum(pk in parent and find(pk) in seen for pk in g['pad_pts'].values()), pads=len(g['pad_pts']),
                absent=absent, kcl_rest_A=float(rest), model='ladder_net-L' if inductive else 'ladder_net-R')
