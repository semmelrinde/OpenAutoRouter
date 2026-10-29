import collections
import math

import numpy as np
import shapely
from shapely.geometry import Polygon, Point, LineString
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import spsolve


def via_layers(v, stack):
    type_=v.get('via_type',v.get('type'))
    if type_ not in (None,'through','blind_buried','blind','buried',0,1):return None
    layers=v.get('layers')
    if not layers:return list(stack) if type_ in (None,'through',0) else None
    if any(L not in stack for L in layers) or len(set(layers))<2:return None
    ids=[stack.index(L) for L in layers]
    return stack[min(ids):max(ids)+1]


def pad_copper(p,L):
    return shapely.union_all([Polygon(q['outer'],q.get('hole_list',[])) if isinstance(q,dict) else Polygon(q)
                              for q in p.get('polys',{}).get(L,[])])


def areas(B, R, A, GND, via_d=.45, drill=.2, all_items=False):
    cu = R.get('copper_map', B.get('copper_map'))
    final = isinstance(cu, dict)
    if final and any(via_layers(v,A['stack']) is None for v in R.get('vias',[])+cu.get('vias',[]) if v['net'] in GND):
        return {L:Polygon() for L in A['levels']},'via_span_unknown'
    if final and not export_fits(B,R,cu,GND):
        return {L:Polygon() for L in A['levels']},'export_outdated'
    outline = [Polygon(o).buffer(0) for o in B.get('outline', []) if len(o) >= 3]
    contour = shapely.union_all(outline)
    out = {}
    cu_layers=sorted(set(A['levels']) | ({z['layer'] for z in cu.get('zones',[])} | {t['layer'] for t in cu.get('tracks',[])} | {L for p in cu.get('pads',[]) for L in p.get('polys',{})} if final else set()))
    for L in cu_layers:
        if final:
            polys = [Polygon(p['outer'], p.get('hole_list', [])).buffer(0)
                     for z in cu.get('zones', []) if z['layer'] == L and z['net'] in GND for p in z['polys']]

            for t in cu.get('tracks', []):
                if t['layer'] == L and t['net'] in GND:
                    w = t['w'] if isinstance(t['w'], (int,float)) else min(t['w'])
                    polys.append(LineString(t['pts']).buffer(w/2))
            for f in cu.get('shapes', []):
                if f['layer'] == L and f['net'] in GND:
                    polys.extend(Polygon(p['outer'], p.get('hole_list', [])) for p in f['polys'])
            for p in cu.get('pads',[]):
                if p['net'] in GND:
                    polys.extend(Polygon(q['outer'],q.get('hole_list',[])) for q in p.get('polys',{}).get(L,[]))
            for v in cu.get('vias',[]):
                if v['net'] in GND and L in via_layers(v,A['stack']):polys.append(Point(v['x'],v['y']).buffer(float(v.get('d') or via_d)/2))
            out[L] = shapely.union_all(polys).intersection(contour)
        else: out[L] = contour
        holes = []
        for k in B.get('keepouts', []):
            lg = k.get('cu_layers') or k.get('layers') or A['stack']
            if (L in lg or '*.Cu' in lg or '*' in lg) and (k.get('prohibitions') or {}).get('areas'):
                holes.extend(Polygon(q).buffer(0) for q in k['polys'] if len(q) >= 3)
        for p in B.get('pads', []):
            if not final and p.get('drill') and (p.get('net') not in GND or not p.get('net')):
                holes.append(Point(p['x'],p['y']).buffer(float(p['drill'])/2 + .15))
            if not final and p.get('net') not in GND and L in p.get('layers', []):
                holes.extend(Polygon(q).buffer(.15) for q in p.get('polys', {}).get(L, []) if len(q)>=3)
        for v in R.get('vias', []):
            if not final and v['net'] not in GND: holes.append(Point(v['x'],v['y']).buffer(float(v.get('d') or via_d)/2+.15))
        for t in R.get('tracks', []):
            if not final and t['layer'] == L and t['net'] not in GND:
                holes.extend(LineString([a,b]).buffer(w/2+.15) for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']) if a!=b)
        if final:
            for p in B.get('pads',[]):
                if p.get('hole_polys'):holes.extend(Polygon(q) for q in p['hole_polys'])
                elif p.get('drill'):holes.append(Point(p['x'],p['y']).buffer(float(p['drill'])/2))
            holes.extend(Point(v['x'],v['y']).buffer(float(v.get('drill') or drill)/2) for v in cu.get('vias',[]) if v['net'] in GND and L in via_layers(v,A['stack']))
        if holes: out[L] = out[L].difference(shapely.union_all(holes))
    if final:

        parts = [(L,p) for L,g in out.items() for p in getattr(g,'geoms',[g]) if not p.is_empty]
        adj = collections.defaultdict(set); roots = set()
        for v in R.get('vias', []) + cu.get('vias', []):
            if v['net'] not in GND: continue
            contact=Point(v['x'],v['y']).buffer(float(v.get('drill') or drill)/2+1e-5)
            ids = [i for i,(L,p) in enumerate(parts) if L in via_layers(v,A['stack']) and p.intersects(contact)]
            for i in ids: adj[i].update(ids)
        source_list = R['reference_sources'] if 'reference_sources' in R else [p for p in B.get('pads', []) if p['net'] in GND and p['ref'].startswith(('J','P'))]
        for q in source_list:
            for i,(L,p) in enumerate(parts):
                if L in q.get('layers',[]) and p.intersects(pad_copper(q,L)):roots.add(i)
        seen = set(roots); front = sorted(roots)
        for i in front:
            for j in sorted(adj[i]):
                if j not in seen: seen.add(j); front.append(j)
        out = {L: shapely.union_all([p for i,(ll,p) in enumerate(parts) if ll==L and i in seen]) for L in out}
    return {L:g for L,g in out.items() if all_items or L in A['levels']}, 'final_copper' if final else 'preliminary_estimate'


def export_fits(B,R,cu,GND):
    for field,items in (('tracks',R.get('tracks',[])),('vias',R.get('vias',[])),('pads',B.get('pads',[]))):
        if items and field not in cu:return False
    if 'tracks' in cu:
        groups=collections.defaultdict(list)
        for t in cu['tracks']:
            w=t['w'] if isinstance(t['w'],(int,float)) else min(t['w'])
            groups[t['net'],t['layer'],round(w,5)].append(LineString(t['pts']))
        lines={k:shapely.union_all(v).buffer(2e-5) for k,v in groups.items()}
        for t in R.get('tracks',[]):
            for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']):
                if a==b:continue
                g=lines.get((t['net'],t['layer'],round(w,5)))
                if g is None or not g.covers(LineString([a,b])):return False
    if 'vias' in cu:
        for v in R.get('vias',[]):
            via,bore = str((R.get('options') or {}).get('via') or '0.45/0.20').split('/')
            if not any(v['net']==q['net'] and math.dist((v['x'],v['y']),(q['x'],q['y']))<=2e-5 and
                       abs(float(v.get('d') or via)-float(q.get('d') or via))<=2e-5 and
                       abs(float(v.get('drill') or bore)-float(q.get('drill') or bore))<=2e-5 and
                       sorted(v.get('layers') or cu.get('cu_layers') or [])==sorted(q.get('layers') or cu.get('cu_layers') or [])
                       for q in cu['vias']):return False
    if 'pads' in cu:
        pads=collections.defaultdict(list)
        for p in cu['pads']:pads[p['ref'],p['num']].append(p)
        def geometry(p,L):
            return shapely.union_all([Polygon(poly['outer'],poly.get('hole_list',[])) if isinstance(poly,dict) else Polygon(poly)
                                      for poly in p.get('polys',{}).get(L,[])])
        def fits(p,q):
            if q['net']!=p['net'] or math.dist((p['x'],p['y']),(q['x'],q['y']))>2e-5:return False
            if abs(float(p.get('drill') or 0)-float(q.get('drill') or 0))>2e-5:return False
            if p.get('pad_type') and p['pad_type']!=q.get('pad_type'):return False
            if p.get('hole_polys'):
                a=shapely.union_all([Polygon(poly) for poly in p['hole_polys']])
                b=shapely.union_all([Polygon(poly['outer'],poly.get('hole_list',[])) if isinstance(poly,dict) else Polygon(poly) for poly in q.get('hole_polys',[])])
                if b.is_empty or not a.buffer(2e-5).covers(b) or not b.buffer(2e-5).covers(a):return False
            if set(p.get('layers',p.get('polys',{})))!=set(q.get('layers',q.get('polys',{}))):return False
            if set(p.get('polys',{}))!=set(q.get('polys',{})):return False


            for L in p.get('polys',{}):
                a,b=geometry(p,L),geometry(q,L)
                if a.is_empty != b.is_empty:return False
                if not a.is_empty and (not a.buffer(2e-5).covers(b) or not b.buffer(2e-5).covers(a)):return False
            return True
        for p in B.get('pads',[]):
            if not any(fits(p,q) for q in pads[p['ref'],p['num']]):return False
    return True

def plate_potential(surface, source_list, measurement_points, rq, raster=.5, connections=(), contact_radii=None):

    if not isinstance(surface,dict):surface={'E':surface}
    if not isinstance(rq,dict):rq={L:rq for L in surface}
    union=shapely.union_all(list(surface.values()))
    if union.is_empty:return None
    x0,y0,x1,y1 = union.bounds
    nx,ny = math.ceil((x1-x0)/raster)+1,math.ceil((y1-y0)/raster)+1
    if nx*ny*len(surface) > 120000: return None
    xy = [(x0+i*raster,y0+j*raster) for i in range(nx) for j in range(ny)]
    cells={}
    for L,g in sorted(surface.items()):
        mask=shapely.covers(g,shapely.points(xy))
        cells.update({(L,i//ny,i%ny):xy[i] for i in np.flatnonzero(mask)})
    if not cells: return None
    points = list(cells.values()); keys = list(cells)
    contact_radii=contact_radii or {}
    def nearest(pt,L=None):
        if isinstance(pt,dict):
            el=pt;pt=el.get('xy');layers=el.get('layers') or []
            if len(layers)!=1:return None
            L=layers[0]
            contact=pad_copper(el,L)
            rad=float(el.get('radius') or 0.)
        else:
            if L is None and len(surface)!=1:return None
            contact=Polygon();rad=contact_radii.get(tuple(pt),0.)
        if L is not None and L not in surface:return None
        contact=contact.union(Point(pt).buffer(rad+1e-5))
        candidates=[i for i,k in enumerate(keys) if (L is None or k[0]==L) and
                    LineString([pt,points[i]]).difference(surface[k[0]]).difference(contact).is_empty]
        if not candidates:return None
        j = min(candidates,key=lambda i: math.dist(pt,points[i]))
        return keys[j] if math.dist(pt,points[j])<=raster else None
    qs=[(nearest(p),I) for p,I in source_list]; ms=[nearest(p) for p in measurement_points]
    if any(k is None for k,_ in qs) or any(k is None for k in ms): return None
    edges=[]; adj=collections.defaultdict(set)
    for a,p in cells.items():
        for b in ((a[0],a[1]+1,a[2]),(a[0],a[1],a[2]+1)):
            if b in cells and surface[a[0]].covers(LineString([p,cells[b]])):
                edges.append((a,b,rq[a[0]])); adj[a].add(b);adj[b].add(a)
    for pt,L1,L2,r in connections:
        a,b=nearest(pt,L1),nearest(pt,L2)
        if a is not None and b is not None and a!=b:
            edges.append((a,b,max(r,1e-12)));adj[a].add(b);adj[b].add(a)
    root=qs[0][0]; seen={root};front=[root]
    for a in front:
        for b in sorted(adj[a]):
            if b not in seen:seen.add(b);front.append(b)
    if any(k not in seen for k,_ in qs) or any(k not in seen for k in ms):return None
    idx={k:i for i,k in enumerate(sorted(seen-{root}))};rhs=np.zeros(len(idx));rows=[];cols=[];vals=[]
    for k,I in qs:
        if k in idx:rhs[idx[k]]+=I
    for a,b,r in edges:
        if a not in seen:continue
        for u,v in ((a,b),(b,a)):
            if u in idx:
                rows.append(idx[u]);cols.append(idx[u]);vals.append(1/r)
                if v in idx:rows.append(idx[u]);cols.append(idx[v]);vals.append(-1/r)
    s=spsolve(coo_matrix((vals,(rows,cols)),shape=(len(idx),len(idx))).tocsr(),rhs)
    return [float(s[idx[k]]) if k in idx else 0. for k in ms]
