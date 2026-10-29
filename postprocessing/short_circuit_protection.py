import json, math
import numpy as np


class R44Error(RuntimeError):
    ""


def protection_on(*source_list):
    amount = '1'
    for q in source_list:
        if isinstance(q, dict):
            if q.get('short_circuit_protection') is not None: amount = q['short_circuit_protection']
        elif q:
            for a in q:
                if isinstance(a, str) and a.startswith('short_circuit_protection='): amount = a.split('=', 1)[1]
    return str(amount).strip().lower() not in ('0', 'false', 'off', 'no')


def protection_option(an):
    return ['short_circuit_protection=%d' % bool(an)]

def track_preview(t):
    if 'items' not in t: return t, 0.0
    from common.track_items import arc_geometry, native_arc_representable, validate
    points, widths = [], []
    for it in t['items']:
        if it[0] == 'A':
            c, r, ang, sweep = arc_geometry(it)
            if not native_arc_representable(it): raise ValueError('degenerate native arc')
            step = 2 * math.acos(max(-1., min(1., 1 - .00025 / r)))
            count = max(1, math.ceil(abs(sweep) / max(step, 1e-10)))
            if count > 100000: raise ValueError('Arc cannot be sampled reliably')
            ps = [[c[0]+r*math.cos(ang+sweep*k/count), c[1]+r*math.sin(ang+sweep*k/count)] for k in range(count+1)]
            ps[0], ps[-1] = it[1], it[3]
        elif it[0] == 'L': ps = it[1:3]
        else: raise ValueError('unknown track item')
        if points and math.dist(points[-1], ps[0]) > 2e-6: raise ValueError('interrupted item chain')
        points.extend(ps if not points else ps[1:]); widths.extend([it[-1]]*(len(ps)-1))
    q = dict(t, pts=points, w=widths)
    validate(q, error=.00025)
    return q, .000252

def _null(a, b):
    return abs(a[0] - b[0]) < 1e-6 and abs(a[1] - b[1]) < 1e-6


def export_contacts(board, data):
    from shapely.geometry import Polygon, LineString, Point
    from shapely.strtree import STRtree
    stack = ['F'] + ['In%d' % i for i in range(1, int(board.get('cu_layers') or 2) - 1)] + ['B']
    geoms = {L: [] for L in stack}
    def add(L, net, g, category, location, unit=None):
        if not g.is_empty: geoms.setdefault(L, []).append((net, g, category, location, unit or net))
    def rounded(g, r): return g.buffer(r / math.cos(math.pi / 64) + 1e-9, quad_segs=16)
    tie_groups = {f['ref']: [set(g) for g in f.get('net_tie_pad_groups', [])]
                  for f in board.get('fps', []) if f.get('net_tie')}
    pad_refs = {}
    seen = set()
    for p in list(board['pads']) + list(board.get('footprint_copper') or []):
        if (p not in (board.get('footprint_copper') or []) and not p.get('graphic') and not p.get('pad_type')
            and ((str(p.get('net', '')).startswith('#KEEPOUT_') and str(p.get('ref', '')).startswith('KEEPOUT_'))
                 or (str(p.get('net', '')).startswith('#NTBAR_') and str(p.get('ref', '')).startswith('NTBAR_')))): continue
        net = p.get('net') or '#pad:%s:%s' % (p.get('ref'), p.get('num'))
        polys = dict(p.get('polys') or {})


        if p.get('pad_type') == 'pth' and not p.get('padstack_complete') and p.get('drill') and any(float(q) > 0 for q in (p['drill'] if isinstance(p['drill'], list) else [p['drill']])):
            outer = polys.get('F') or polys.get('B') or []
            for L in stack: polys.setdefault(L, outer)
        for L, qs in polys.items():
            for q in qs:
                g = Polygon(q)
                key = (L, net, p.get('ref'), p.get('num'), g.wkb)
                if key in seen: continue
                seen.add(key); pad_refs[id(g)] = (p.get('ref'), p.get('num'))
                add(L, net, g, 'pad', [p['x'], p['y']])
    for t in data['tracks']:
        t, error = track_preview(t)
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            if _null(a, b): continue
            add(t['layer'], t['net'], rounded(LineString([a, b]), w / 2 + error + (2e-6 if error else 0)), 'track', a, t.get('unit'))
    d = float(str(data.get('options', {}).get('via', '0.45/0.20')).split('/')[0])
    for v in data['vias']:
        g = rounded(Point(v['x'], v['y']), float(v.get('d', v.get('diameter', d))) / 2)


        for L in v.get('layers') or stack: add(L, v['net'], g, 'via', [v['x'], v['y']], v.get('unit'))
    for z in data.get('zone_list', []) + data.get('zones', []):
        if z.get('filled') is True:


            if not z.get('pts'): raise ValueError('R44: fill zone without checkable copper geometry')
            q = Polygon(z['pts'], z.get('holes', []))
            add(z['layer'], z['net'], q, 'zone', list(q.representative_point().coords)[0], z.get('unit'))
            continue
        qs = [Polygon(q['outer'], q.get('inner', [])) for q in z.get('polys', [])]
        qs += [Polygon(a, h) for a, h in z.get('filled', [])] if isinstance(z.get('filled'), list) else []
        if z.get('native_outlines'):
            qs = [Polygon(q['pts'], q.get('holes', [])) for q in z['native_outlines']]
        elif z.get('pts') and not qs: qs = [Polygon(z['pts'], z.get('holes', []))]
        if not qs: raise ValueError('R44: zone without checkable copper geometry')
        for L in z.get('native_layers') or [z['layer']]:
            for q in qs: add(L, z['net'], q, 'zone', list(q.representative_point().coords)[0], z.get('unit'))
    out = []; inherited = []
    for L, G in sorted(geoms.items()):
        if not G: continue
        gs = np.asarray([q[1] for q in G], dtype=object)
        ai, bi = STRtree(gs).query(gs, predicate='intersects')
        for i, j in sorted(zip(ai.tolist(), bi.tolist())):
            if i >= j or G[i][0] == G[j][0]: continue
            a, b = G[i], G[j]
            if a[2] == b[2] == 'pad':
                ref, num = pad_refs[id(a[1])]
                other_ref, other_num = pad_refs[id(b[1])]
                if ref == other_ref and any({num, other_num} <= group for group in tie_groups.get(ref, [])): continue
            c = a[1].intersection(b[1]).representative_point()
            e = dict(layer_name=L, net_list=[a[0], b[0]], kinds=[a[2], b[2]], unit_list=[a[4], b[4]], location=[round(c.x, 6), round(c.y, 6)])
            (inherited if a[2] == b[2] == 'pad' else out).append(e)
    return out, inherited


def export_selection(edges, force, important, solver=True):
    import collections
    force = set(force); edges = {tuple(sorted(e)) for e in edges if not set(e) & force}
    nodes = sorted({n for e in edges for n in e})
    chosen = set(force); rest = set(edges)
    while rest:
        degree = collections.Counter(n for e in rest for n in e)
        n = min(degree, key=lambda n: (-degree[n], important[n], n))
        chosen.add(n); rest = {e for e in rest if n not in e}

    for n in sorted(chosen - force, key=lambda n: (-important[n], n)):
        if all(set(e) & (chosen - {n}) for e in edges): chosen.remove(n)
    optimal = not edges
    if nodes and solver:
        from scipy.optimize import milp, Bounds, LinearConstraint
        from scipy.sparse import lil_matrix
        ix = {n:i for i,n in enumerate(nodes)}; N = len(nodes)
        mx = max(important.values(), default=1); base = mx + 2
        A = lil_matrix((len(edges) + N, N + 1)); low = []; high = []
        for k, (a, b) in enumerate(sorted(edges)):
            A[k, ix[a]] = A[k, ix[b]] = 1; low.append(1); high.append(np.inf)
        for i, n in enumerate(nodes):
            A[len(edges)+i, i] = important[n]; A[len(edges)+i, N] = -1
            low.append(-np.inf); high.append(0)
        c = np.array([base + important[n] / ((N+1)*(mx+1)) for n in nodes] + [1.0])
        result = milp(c, integrality=np.array([1]*N + [0]),
            bounds=Bounds(np.array([0]*N + [max((important[n] for n in force), default=0)]), np.array([1]*N + [mx])),
            constraints=LinearConstraint(A.tocsr(), low, high), options=dict(node_limit=2000))
        def key(s): return (len(s), max((important[n] for n in s), default=0), sum(important[n] for n in s), sorted(s))
        if result.x is not None:
            s = force | {n for n,i in ix.items() if result.x[i] > .5}
            if all(set(e) & s for e in edges) and key(s) <= key(chosen): chosen = s
        optimal = bool(result.success)
    return chosen, optimal


def export_ratsnest(board, net_list):
    out = []
    for n in sorted(net_list):
        ps = sorted({(p['x'], p['y']) for p in board['pads'] if p.get('net') == n})
        if not ps: continue


        best = {b: (math.dist(ps[0], b), ps[0], b) for b in ps[1:]}
        while best:
            _, a, b = min(best.values())
            out.append(dict(net=n, pts=[list(a),list(b)], reason='open_items'))
            del best[b]
            for c in best: best[c] = min(best[c], (math.dist(b, c), b, c))
    return out


def _export_safe(board, data, unit_list=None, important=None, pairs=(), checkpoint=False):
    import hashlib
    unit_list = unit_list or {}; important = important or {}
    owners = {q.get('unit') for k in ('tracks', 'vias') for q in data.get(k, []) if q.get('unit')}
    raw_sha = hashlib.sha256(json.dumps(dict(tracks=data['tracks'], vias=data['vias']), sort_keys=True).encode()).hexdigest()
    contacts, inherited = export_contacts(board, data)
    edges = set(); force = set(); faults = list(contacts)
    for e in contacts:
        a,b = e.get('unit_list', e['net_list']); aa,bb = e['kinds']
        if aa == 'pad': force.add(b)
        elif bb == 'pad': force.add(a)
        else: edges.add(tuple(sorted((a,b))))


    for L,d,a,b,x,y in pairs:
        na,nb = unit_list.get(a),unit_list.get(b)
        if a.startswith('GND#') and na and a in owners: na = a
        if b.startswith('GND#') and nb and b in owners: nb = b
        if a == 'PAD' and nb: force.add(nb)
        elif b == 'PAD' and na: force.add(na)
        elif na and nb:
            if na == nb: force.add(na)
            else: edges.add(tuple(sorted((na,nb))))
        else: continue
        faults.append(dict(layer_name=L, net_list=[na or a,nb or b], kinds=['rule','rule'], location=[x,y], spacing=d))
    affected = force | {n for e in edges for n in e}
    important = {n:int(important.get(n, 1)) for n in affected}
    way, optimal = export_selection(edges, force, important, solver=not checkpoint)
    if not way and not inherited:
        if data.get('failed'):
            result = dict(data)
            if not checkpoint: result['ratsnest'] = export_ratsnest(board, {unit_list.get(n,n) for n in data['failed']})
            return result
        return data
    result = dict(data)
    for k in ('tracks','vias','zone_list','zones'):
        if k in data: result[k] = [q for q in data[k] if q.get('unit', q['net']) not in way]
    failed = set(data.get('failed', [])) | way
    result['failed'] = sorted(failed)
    if not checkpoint: result['ratsnest'] = export_ratsnest(board, {unit_list.get(n,n) for n in failed})
    rest, _ = export_contacts(board, result)
    if rest: raise ValueError('R44: remaining contacts cannot be resolved reliably')
    result['short_circuit_protection'] = dict(version=1, nets_removed=sorted({unit_list.get(u, u) for u in way}), units_removed=sorted(way), faults=faults,
        input_pad_contacts=inherited, remaining_contacts=0, minimal_proven=optimal,
        before_cleanup=dict(sha=raw_sha, track_list=len(data['tracks']), vias=len(data['vias'])),
        pairs_before_cleanup=list(pairs))
    return result


def export_safe(board, data, unit_list=None, important=None, pairs=(), checkpoint=False):
    units = dict(unit_list or {})
    bad, errors = set(), []
    work = dict(data)
    for key in ('tracks', 'vias', 'zone_list', 'zones'):
        if key not in data: continue
        work[key] = []
        for i, obj in enumerate(data[key]):
            owner = obj.get('unit') or obj.get('net') or '#unknown:%s:%d' % (key, i)
            units.setdefault(owner, obj.get('net') or owner)
            try:
                if not obj.get('net'): raise ValueError('Copper without net')
                if key == 'tracks':
                    q, _ = track_preview(obj)
                    if len(q['pts'])<2 or len(q['w'])!=len(q['pts'])-1 or not q.get('layer'): raise ValueError('invalid track preview')
                    if not all(len(p)==2 and all(math.isfinite(v) for v in p) for p in q['pts']): raise ValueError('invalid track points')
                    if not all(math.isfinite(w) and w>0 for w in q['w']): raise ValueError('invalid track width')

                    if 'items' in obj:
                        from common.track_items import validate
                        try: validate(obj, error=.00025); q = obj
                        except ValueError: pass
                else: q = obj
                if key == 'vias':
                    if not all(math.isfinite(q[v]) for v in ('x','y')): raise ValueError('invalid via position')
                    diameter = float(q.get('d',q.get('diameter',str(data.get('options',{}).get('via','.45/.20')).split('/')[0])))
                    if not math.isfinite(diameter) or diameter<=0: raise ValueError('invalid via diameter')
                if key in ('zone_list','zones'):
                    export_contacts(dict(pads=[], cu_layers=board.get('cu_layers',2)), dict(tracks=[],vias=[],**{key:[q]}))
                work[key].append(q)
            except Exception as exc:
                bad.add(owner); errors.append(dict(unit=owner, reason=str(exc)[:180]))
    for key in ('tracks','vias','zone_list','zones'):
        if key in work: work[key] = [q for q in work[key] if (q.get('unit') or q.get('net')) not in bad]
    work['failed'] = sorted(set(data.get('failed', [])) | bad)
    try:
        result = _export_safe(board, work, units, important, pairs, checkpoint)
    except Exception as exc:
        errors.append(dict(reason='Overall check: '+str(exc)[:180]))
        result = dict(work)
        for key in ('tracks','vias','zone_list','zones'):
            for q in result.get(key, []): bad.add(q.get('unit') or q.get('net'))
            if key in result: result[key] = []
        result['failed'] = sorted(set(result['failed']) | bad)
    if errors:
        result = dict(result)
        info = dict(result.get('short_circuit_protection') or {})
        info.update(geometry_error=errors, units_uncheckable=sorted(bad), remaining_contacts=0)
        info['nets_removed'] = sorted(set(info.get('nets_removed',[])) | {units.get(u,u) for u in bad})
        result['short_circuit_protection'] = info
    if not checkpoint and result.get('failed'):
        try: result['ratsnest'] = export_ratsnest(board, {units.get(n,n) for n in result['failed']})
        except Exception: result['ratsnest'] = []

    return data if result == data else result



def output_assignment(data, cu_layers, origin, shape, raster=.25):
    from shapely.geometry import LineString, Point, Polygon
    from shapely import box, intersects
    result = np.zeros((len(cu_layers), *shape), bool)
    forms = []
    for t in data.get('tracks', []):
        t, error = track_preview(t)
        for a,b,w in zip(t['pts'],t['pts'][1:],t['w']):
            if _null(a, b): continue
            forms.append((t['layer'], LineString([a,b]).buffer(w/2+error)))
    diameter = float(str(data.get('options',{}).get('via','.45/.20')).split('/')[0])
    for v in data.get('vias', []):
        g = Point(v['x'],v['y']).buffer(float(v.get('d',v.get('diameter',diameter)))/2)
        forms.extend((L,g) for L in v.get('layers') or cu_layers)
    for z in data.get('zone_list', [])+data.get('zones', []):
        if z.get('filled') is True: qs, ls = [Polygon(z['pts'],z.get('holes',[]))], [z['layer']]
        else:
            qs = [Polygon(q['outer'],q.get('inner',[])) for q in z.get('polys',[])]
            qs += [Polygon(a,h) for a,h in z.get('filled',[])] if isinstance(z.get('filled'),list) else []
            if z.get('native_outlines'): qs = [Polygon(q['pts'],q.get('holes',[])) for q in z['native_outlines']]
            elif z.get('pts') and not qs: qs = [Polygon(z['pts'],z.get('holes',[]))]
            ls = z.get('native_layers') or [z['layer']]
        forms.extend((L,g) for L in ls for g in qs)
    for L,g in forms:
        if L not in cu_layers or g.is_empty: continue
        bounds = np.asarray(g.bounds).reshape(2,2)
        lo = np.maximum(0,np.floor((bounds[0]-origin-1e-9)/raster).astype(int))
        hi = np.minimum([shape[1],shape[0]],np.floor((bounds[1]-origin)/raster).astype(int)+1)
        if np.any(hi<=lo): continue
        yy,xx = np.mgrid[lo[1]:hi[1],lo[0]:hi[0]]
        x,y = origin[0]+xx*raster,origin[1]+yy*raster
        result[cu_layers.index(L),lo[1]:hi[1],lo[0]:hi[0]] |= intersects(g,box(x,y,x+raster,y+raster))
    return result.astype(np.float16)


def labels_save(board, routes, routes_path, labels=None):
    from pathlib import Path
    import hashlib, os, tempfile
    path = Path(routes_path)
    paths = {Path(labels)} if labels else {path.with_name(path.stem+'_labels.npz'), path.parent/'labels.npz'}
    if not labels and routes.get('options',{}).get('labels'):
        q = Path(routes['options']['labels'])

        if q.resolve().parent == path.resolve().parent: paths.add(q)
    for dest in sorted(paths):
        if not dest.exists(): continue
        try:
            with np.load(dest,allow_pickle=False) as z: d = {k:z[k] for k in z.files}
            layers = list(d['cu_layers']); origin = d['origin_mm']; shape = d['occupancy'].shape[-2:]


            observer = int(d.get('observer_version', 1))



            d.setdefault('source_observer_version', np.asarray(1 if observer == 4 else observer, np.int16))
            d.setdefault('occupancy_search', d['occupancy'].copy())
            if 'occupancy_mask' in d: d.setdefault('occupancy_search_mask', np.asarray(d['occupancy_mask'], bool).copy())
            d['occupancy'] = output_assignment(routes,layers,origin,shape,float(d['raster_mm']))
            d['occupancy_contract'] = np.asarray('output_copper_cell_contact')
            d['label_version'] = np.asarray(2,np.int16); d['observer_version'] = np.asarray(4,np.int16)
            d['export_routes_sha256'] = np.asarray(hashlib.sha256(path.read_bytes()).hexdigest())
            values = d['net_values'].copy(); lm = np.zeros_like(d['net_layers']); vm = np.zeros_like(d['vias_raster'])
            names = list(d['net_list']); ix = {n:i for i,n in enumerate(names)}
            values[:,1] = 0; values[:,4] = 0
            failed = {q.get('net') or q.get('unit') for key in ('tracks','vias') for q in routes.get(key,[]) if q.get('unit') in routes.get('failed',[])} | set(routes.get('failed',[]))
            failed |= set((routes.get('short_circuit_protection') or {}).get('nets_removed',[]))
            for unit in routes.get('failed',[]):
                if unit.startswith('GND#'):
                    try: failed.add(board['pads'][int(unit[4:])].get('net'))
                    except (ValueError,IndexError): pass
            for n in failed:
                if n in ix: values[ix[n],0] = 0
            for t in routes.get('tracks',[]):
                t,_ = track_preview(t)
                if t['net'] in ix:
                    i = ix[t['net']]; values[i,1] += sum(math.dist(a,b) for a,b in zip(t['pts'],t['pts'][1:]))
                    if t['layer'] in layers: lm[i,layers.index(t['layer'])] = True
            for v in routes.get('vias',[]):
                if v['net'] in ix: values[ix[v['net']],4] += 1; lm[ix[v['net']],:] = True
                x,y = np.floor((np.array([v['x'],v['y']])-origin)/float(d['raster_mm'])).astype(int)
                if 0<=x<shape[1] and 0<=y<shape[0]: vm[:,y,x] += 1
            values[:,3] = np.divide(values[:,1],values[:,2],out=np.zeros(len(values)),where=values[:,2]>0)
            values[:,5] = lm.sum(1)
            d.update(net_values=values,net_layers=lm,vias_raster=vm)
            fd,tmp = tempfile.mkstemp(dir=dest.parent,suffix='.npz'); os.close(fd)
            try: np.savez_compressed(tmp,**d); os.replace(tmp,dest)
            finally:
                if os.path.exists(tmp): os.unlink(tmp)
        except Exception as exc:

            print('R44: labels cannot be synchronized:',repr(exc)[:180])

            os.replace(dest,dest.with_name(dest.name+'.r44_uncheckable'))


def _file_save(board_path, routes_path, output_path=None, state=None, data=None):
    from pathlib import Path
    import os, tempfile
    state = {} if state is None else state
    b = json.loads(Path(board_path).read_text())
    r = data if data is not None else json.loads(Path(routes_path).read_text())
    if not isinstance(r, dict) or not isinstance(b, dict): raise ValueError('Routes/board not a JSON object')
    units = {q.get('unit') or q.get('net'): q.get('net') for key in ('tracks', 'vias', 'zone_list', 'zones') for q in r.get(key, []) if q.get('net')}
    units.update({'GND#%d' % i: p.get('net') for i, p in enumerate(b['pads'])})
    safe = export_safe(b, r, units)
    dest = Path(output_path or routes_path)
    if safe is r and output_path is None:
        state['written'] = True
        labels_save(b, safe, routes_path)
        return safe
    fd, tmp = tempfile.mkstemp(dir=dest.parent, prefix='.r44_')
    try:
        with os.fdopen(fd, 'w') as f: json.dump(safe, f)
        os.replace(tmp, dest)
    finally:
        if os.path.exists(tmp): os.unlink(tmp)
    state['written'] = True
    if output_path is None: labels_save(b, safe, routes_path)
    return safe


def file_save(board_path, routes_path, output_path=None, data=None):
    from pathlib import Path
    import os
    path = Path(routes_path); state = {}
    try: return _file_save(board_path, routes_path, output_path, state, data)
    except Exception as exc:
        reason = 'R44: output not secured: %s: %s' % (type(exc).__name__, str(exc)[:200])
        if output_path is None and not state.get('written') and path.exists():
            blocked = path.with_name(path.name + '.r44_unchecked')
            try:
                os.replace(path, blocked)
                try: blocked.with_name(blocked.name + '.reason').write_text(reason)
                except OSError: pass
            except OSError as keepout: reason += '; keepouts failed: %s' % str(keepout)[:80]
        raise R44Error(reason) from exc


def candidate_save(board_path, routes_path):
    from pathlib import Path
    path = Path(routes_path)
    if not path.exists():
        blocked = path.with_name(path.name + '.r44_unchecked')
        if not blocked.exists(): return None
        try: return blocked.with_name(blocked.name + '.reason').read_text()[:300]
        except OSError: return 'R44: output locked unchecked (%s)' % blocked.name
    try: data = json.loads(path.read_text())
    except Exception: data = None
    if isinstance(data, dict) and not protection_on(data.get('options')): return None
    try: file_save(board_path, routes_path, data=data if isinstance(data, dict) else None)
    except R44Error as exc: return str(exc)
    return None


def candidate_save_measurement(board_path, routes_path):
    import time
    start = time.thread_time()
    reason = candidate_save(board_path, routes_path)
    return reason, dict(cpu_s=time.thread_time()-start, cpu_kind='owned_thread')


if __name__ == '__main__':
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    try: file_save(*sys.argv[1:4])
    except R44Error as exc:
        print(exc, file=sys.stderr); sys.exit(1)
