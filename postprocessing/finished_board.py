import collections, json, math, os, re, shutil, subprocess, sys, time

A = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, A)
from common.drc_events import event, new_events
from common.copper_net import Obstacles, NetCopper, ZonesGuard, geo_hash, zones_fill as _zones_fill, zones_contours
from kicad.sexpr_swap import swap
from postprocessing.keepout_nets import net_spacings, keepout_nets, keepout_extent

KPY = '/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3'
CLI = shutil.which('kicad-cli') or '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
VPY = sys.executable


def log(*a):
    print(time.strftime('%H:%M:%S'), *a, flush=True)


def copy(src_pcb, dst_pcb, pro, dru):
    shutil.copy(src_pcb, dst_pcb)
    b = dst_pcb[:-10]; shutil.copy(pro, b + '.kicad_pro'); shutil.copy(dru, b + '.kicad_dru')


def drc(pcb, pro, dru):
    b = pcb[:-10]; shutil.copy(pro, b + '.kicad_pro'); shutil.copy(dru, b + '.kicad_dru'); bj = b + '_drc.json'
    if os.path.exists(bj): os.remove(bj)
    r = subprocess.run([CLI, 'pcb', 'drc', '--format', 'json', '--severity-all', '--refill-zones', '--save-board', '-o', bj, pcb],
                       capture_output=True, text=True)
    shutil.copy(pro, b + '.kicad_pro'); shutil.copy(dru, b + '.kicad_dru')
    if r.returncode != 0 or not os.path.exists(bj): raise RuntimeError('kicad-cli DRC failed (%d): %s' % (r.returncode, r.stderr[-500:]))
    d = json.load(open(bj))
    if 'violations' not in d or 'unconnected_items' not in d: raise RuntimeError('DRC report without violations/unconnected_items: ' + bj)
    c = collections.Counter((v['severity'], v['type']) for v in d['violations'])
    ev = collections.Counter(event(v) for v in d['violations']) + collections.Counter(event(v, 'open_items') for v in d['unconnected_items'])
    return dict(E=collections.Counter({k[1]: n for k, n in c.items() if k[0] == 'error'}), U=len(d['unconnected_items']),
                W=collections.Counter({k[1]: n for k, n in c.items() if k[0] == 'warning'}), ev=ev, d=d)


def obst(pcb, out, box):
    if os.path.exists(out): os.remove(out)
    r = subprocess.run([KPY, os.path.join(A, 'kicad', 'obst_export.py'), pcb] + [str(x) for x in box] + [out], capture_output=True, text=True)
    if r.returncode != 0 or not os.path.exists(out): raise RuntimeError('obst_export failed: ' + r.stderr[-500:])
    return json.load(open(out))


def frame(H):
    xs = [p[0] for e in H['edges'] for p in e]; ys = [p[1] for e in H['edges'] for p in e]
    return [math.floor(min(xs)) - 1, math.floor(min(ys)) - 1, math.ceil(max(xs)) + 1, math.ceil(max(ys)) + 1]


def zones_fill(text):
    return [(f['net'], f['layer'], f['poly']) for f in _zones_fill(text)]


def nets_the_message(new_ev, change_nets, D=None, text_before=None):
    net_list, foreign, places = set(), set(), []
    def off(items):
        owned = False
        for i in items:
            ds = i.get('description', '')
            for part in ds.split('[')[1:]:
                n = part.split(']')[0]
                if n in change_nets: net_list.add(n); owned = True
                else: foreign.add(n)
        if not owned: places.extend((i['pos']['x'], i['pos']['y']) for i in items if i.get('pos'))
        return owned
    for v in new_ev: off(v['items'])
    if net_list or D is None: return net_list
    from shapely.geometry import LineString, Point
    new = [(s['net'], s['layer'], LineString([s['a'], s['b']]).buffer(s['w'] / 2 + 0.02)) for s in D['add'] if s['a'] != s['b']]
    if text_before and foreign:
        for zn, L, g in zones_fill(text_before):
            if zn not in foreign: continue
            net_list |= {n for n, L2, b in new if L2 == L and b.intersects(g)}
    if not net_list and places:
        net_list |= {n for n, _, b in new if any(b.distance(Point(o)) < 1.5 for o in places)}
    return net_list


def good(R, ref):
    return not new_events(R, ref) and R['U'] <= ref['U']


def apply_verify(cur, D, goal, pro, dru, ref, sieve):
    text = open(cur, encoding='utf-8').read()
    net_of = {}
    H_uuid = D.get('_uuid_net', {})
    for u in D['remove']: net_of[u] = H_uuid.get(u)
    out = set()
    for round_no in range(sieve + 1):
        Dk = dict(remove=[u for u in D['remove'] if net_of.get(u) not in out], add=[s for s in D['add'] if s['net'] not in out],
                  add_generated=[g for g in D.get('add_generated', []) if g.get('net_name') not in out])
        if not Dk['remove'] and not Dk['add']: return False, Dk, dict(reason='everything reverted', back=sorted(out))
        new, info = swap(text, Dk)
        if info['absent']: return False, Dk, dict(reason='Tracks not found (no half delta): %s' % info['absent'][:5])
        if os.path.exists(goal) and os.path.samefile(goal, cur): raise RuntimeError('Target is the input: ' + goal)
        open(goal, 'w', encoding='utf-8').write(new)
        R = drc(goal, pro, dru); nev = new_events(R, ref)
        log('   DRC: errors %s, open %d, warnings %d, new reports %d' % (dict(R['E']) or 0, R['U'], sum(R['W'].values()), len(nev)),
            '(reverted: %d nets)' % len(out) if out else '')
        if good(R, ref): return True, Dk, dict(info, back=sorted(out), drc=dict(faults=dict(R['E']), open_items=R['U'], warnings=sum(R['W'].values())), _R=R)
        change = {s['net'] for s in Dk['add']} | {net_of.get(u) for u in Dk['remove']} - {None}
        new_out = nets_the_message(nev, change, Dk, text)
        if not new_out: return False, Dk, dict(reason='DRC report cannot be assigned to a changed net', faults=dict(R['E']), open_items=R['U'])
        out |= new_out
        log('   reverting:', ', '.join(sorted(n.split('/')[-1] for n in new_out)))
    return False, Dk, dict(reason='not clean after %d sieve rounds' % sieve, back=sorted(out))




def step_tidy_up(H, S, fwd, o, cur, tmp):
    from postprocessing.tidy_up import tidy_up
    return tidy_up(H, keepout_list=S, net_list=o.get('_nets'), fwd=fwd, micro=float(o.get('micro', 0.03)), fix=o.get('_fix', ()))


def step_middle(H, S, fwd, o, cur, tmp):
    from postprocessing.middle_approach import middle
    he = float(o.get('hmax_plane', 0.3))
    hm = {(n, L): he for n, Ls in (o.get('_hmax_layers') or {}).items() for L in Ls} if he > 0 else {}
    return middle(H, keepout_list=S, net_list=o.get('_nets'), fwd=fwd, fix=o.get('_fix', ()), hmax=hm)


def step_shorten(H, S, fwd, o, cur, tmp):
    hj = os.path.join(tmp, 'shortcut_h.json'); json.dump(H, open(hj, 'w'))
    box = frame(H); zone_list = o['_zone_nets']
    net_list = sorted(n for n in ({t['net'] for t in H['tracks'] if t['net']} if o.get('_nets') is None else set(o['_nets'])) if n not in S)
    D = dict(remove=[], add=[], stat={})
    fj = os.path.join(tmp, 'shortcut_fix.json'); json.dump(sorted(o.get('_fix', ())), open(fj, 'w'))
    zj = os.path.join(tmp, 'shortcut_without_dead.json'); json.dump(sorted(n for n in net_list if n in zone_list), open(zj, 'w'))
    if not net_list: return D


    oj = os.path.join(tmp, 'shortcut_o.json')
    hl = os.path.join(tmp, 'shortcut_hmax_layers.json'); json.dump(o.get('_hmax_layers') or {}, open(hl, 'w'))
    bj = os.path.join(tmp, 'shortcut_budget.json'); json.dump(lengths_budget(o.get('_L0'), H, o.get('_gate_nets')), open(bj, 'w'))
    opt = ['drc=' + o.get('drc', '0.09'), 'drcpad=' + o.get('drcpad', '0.10'), 'drcvia=' + o.get('drcvia', o.get('drc', '0.09')),
           'border=' + o.get('edge', '0.2'), 'hole_size=' + o.get('hole_size', '0.25'), 'hmax=' + o.get('hmax', '0'), 'dead=1', 'without_dead=' + zj,
           'hmax_layers=' + hl, 'hmax_plane=' + o.get('hmax_plane', '0.3'), 'budget=' + bj,
           'pcb=' + cur, 'fix=' + fj] + (['netclasses=' + o['netclasses']] if o.get('netclasses') else []) + (['keepout_list=' + o['keepout_list']] if o.get('keepout_list') else [])
    r = subprocess.run([VPY, os.path.join(A, 'postprocessing', 'shorten.py'), hj] + [str(x) for x in box] + [';'.join(net_list), oj] + opt,
                       capture_output=True, text=True)
    if r.returncode != 0: raise RuntimeError('shorten: ' + r.stderr[-800:])
    O = json.load(open(oj)); D['remove'] += O['remove']; D['add'] += O['add']
    D['stat']['shorten'] = r.stdout.strip().split('\n')[-1][:400]
    return D


def lengths_budget(L0, H, gate_nets, percent=2.0, mm=1.0):
    if not L0 or not gate_nets: return {}
    from analysis.finished_metrics import length_list
    L = length_list(H)
    return {n: max(0.0, min(mm, percent / 100.0 * L0.get(n, 0.0)) - abs(L.get(n, 0.0) - L0.get(n, 0.0)) - 1e-6) for n in sorted(gate_nets)}


def step_smooth(H, S, fwd, o, cur, tmp):
    from shapely.geometry import LineString, Point, Polygon
    from postprocessing.smooth45 import smooth
    clr = max(float(o.get('drc', 0.09)), float(o.get('drcpad', 0.10)), float(o.get('drcvia', o.get('drc', 0.09))))
    hole_size, edge = float(o.get('hole_size', 0.25)), float(o.get('edge', 0.2))
    rings = sorted((Polygon(e).buffer(0) for e in H['edges'] if len(e) > 2), key=lambda g: -g.area)
    outer = rings[0]; slots = [g for g in rings[1:] if outer.contains(g.representative_point())]
    pads = collections.defaultdict(lambda: dict(polys=collections.defaultdict(list)))
    for p in H['pads']:
        k = (p.get('ref'), p.get('num'), p['net']); pads[k]['net'] = p['net']; pads[k]['polys'][p['layer']].append(p['poly'])
        pads[k]['x'], pads[k]['y'] = p.get('x', 0), p.get('y', 0)
    cu_layers = sorted({t['layer'] for t in H['tracks']} | {p['layer'] for p in H['pads']})
    Bp = [dict(net=v['net'], polys=dict(v['polys']), x=v['x'], y=v['y']) for v in pads.values()]
    for h in H.get('holes', []):
        c = list(Point(h['x'], h['y']).buffer(h['d'] / 2 + max(0.0, hole_size - clr), 16).exterior.coords)
        Bp.append(dict(net=h.get('net') or '#HOLE', polys={L: [c] for L in cu_layers}, x=h['x'], y=h['y']))
    blocked = [dict(prohibitions=dict(track_list=True), polys=[list(g.buffer(edge, 8).exterior.coords)]) for g in slots]
    blocked += [dict(prohibitions=dict(track_list=True), polys=[k['poly']]) for k in H.get('keepouts', []) if k.get('tracks')]
    B = dict(pads=Bp, outline=[list(outer.exterior.coords)], keepouts=blocked,
             areas=[(n, L, g.buffer(max(0.0, c - clr), 8)) for n, L, g, c in o.get('_areas', ())])
    na = net_spacings(o.get('_dru'), {t['net'] for t in H['tracks']})
    tracks = []; choice = set(o['_nets']) if o.get('_nets') is not None else None
    for t in H['tracks']:
        w = t['w'] + (2 * max(0.0, na.get(t['net'], 0) - clr) if t['net'] in S else 0.0)
        tracks.append(dict(net=t['net'], layer=t['layer'], pts=[list(t['a']), list(t['b'])], w=[w],
                           **({'fix': True} if t.get('arc') or t['net'] in S or (choice is not None and t['net'] not in choice)
                              or t.get('uuid') in o.get('_fix', ()) else {})))
    dmax = max([v['d'] for v in H['vias']] + [0.45])
    R = dict(tracks=tracks, vias=[dict(net=v['net'], x=v['x'], y=v['y'], d=v['d']) for v in H['vias']],
             options=dict(via='%g/0.2' % dmax, clr=clr, edge=edge))

    arc = {(t['net'], t['layer']) for t in H['tracks'] if t.get('arc')}
    for t in R['tracks']:
        if (t['net'], t['layer']) in arc: t['fix'] = True

    from postprocessing.smooth45 import implicit_anchor
    rd = lambda p: (round(p[0], 4), round(p[1], 4))
    R['tracks'], anchor = implicit_anchor(R, B)
    before = collections.defaultdict(set)
    for t in R['tracks']:
        before[(t['net'], t['layer'])].add(tuple(sorted([rd(t['pts'][0]), rd(t['pts'][-1])])))
    he = float(o.get('hmax_plane', 0.3))
    hm = {(n, L): he for n, Ls in (o.get('_hmax_layers') or {}).items() for L in Ls} if he > 0 else {}
    info = smooth(B, R, clr=clr, border=edge, keepout_list=set(S), anchor=anchor, prev_obstacle=True, hmax=hm, corners_protection=True)
    after = collections.defaultdict(list)
    for t in R['tracks']: after[(t['net'], t['layer'])].append(t)
    D = dict(remove=[], add=[], stat=info)
    for key, ts in sorted(after.items()):
        if key[0] in S or key in arc or (choice is not None and key[0] not in choice): continue
        segs = {tuple(sorted([rd(a), rd(b)])) for t in ts for a, b in zip(t['pts'][:-1], t['pts'][1:]) if a != b}
        if segs == before[key]: continue
        D['remove'] += sorted({t['uuid'] for t in H['tracks'] if (t['net'], t['layer']) == key})
        for t in ts:
            for (a, b), w in zip(zip(t['pts'][:-1], t['pts'][1:]), t['w']):
                if a != b: D['add'].append(dict(net=key[0], layer=key[1], a=list(a), b=list(b), w=w))

    budget = lengths_budget(o.get('_L0'), H, o.get('_gate_nets'))
    if budget:
        u_l = {t['uuid']: math.dist(t['a'], t['b']) for t in H['tracks']}; u_n = {t['uuid']: t['net'] for t in H['tracks']}
        dl = collections.defaultdict(float)
        for u in D['remove']: dl[u_n[u]] -= u_l[u]
        for s_ in D['add']: dl[s_['net']] += math.dist(s_['a'], s_['b'])
        too_long = sorted(n for n, d in dl.items() if n in budget and abs(d) > budget[n])
        if too_long:
            D['remove'] = [u for u in D['remove'] if u_n[u] not in too_long]; D['add'] = [s_ for s_ in D['add'] if s_['net'] not in too_long]
            info = dict(info, rejected_length_gate=too_long)

    NET_COPPER = NetCopper(H); rm = set(D['remove']); broken = set()
    for net in sorted({s['net'] for s in D['add']}):
        prior = NET_COPPER.tracks.get(net, [])
        new = [t for t in prior if t['uuid'] not in rm] + [s for s in D['add'] if s['net'] == net]
        if NET_COPPER.separates(net, prior, new): broken.add(net)
    if broken:
        log('   smooth: splits, reverted:', ', '.join(sorted(broken)))
        u_net = {t['uuid']: t['net'] for t in H['tracks']}
        D['remove'] = [u for u in D['remove'] if u_net.get(u) not in broken]; D['add'] = [s for s in D['add'] if s['net'] not in broken]
    D['stat'] = dict(info, nets_layers=len({(s['net'], s['layer']) for s in D['add']}), rejected_separates=sorted(broken))
    return D

def shape_gate(H, H_before=None, L0=None, gate_nets=()):
    from analysis.finished_metrics import lengths_violation, middle_loss, gate_check
    t = gate_check(H)
    t['middle_lost'] = sum(middle_loss(H_before, H).values()) if H_before is not None and H_before is not H else 0
    t['length_gate'] = len(lengths_violation(L0, H, gate_nets)) if L0 is not None and gate_nets else 0
    return t


def shape_nets(H_new, H_before, L0, gate_nets):
    from analysis.finished_metrics import lengths_violation, middle_loss
    return set(middle_loss(H_before, H_new)) | set(lengths_violation(L0, H_new, gate_nets) if gate_nets else {})


def nets_worse(H, H_new, L0, gate_nets, changed):
    return set(shape_reasons(H, H_new, L0, gate_nets, changed))


SHAPE_REASONS = ('angle', 'r32', 'hairpin', 'corner90', 'ends', 'goal', 'middle', 'total_length')


def shape_reasons(H, H_new, L0, gate_nets, changed, fill=None, fill_new=None):
    from analysis.finished_metrics import corners90_new, ends_new, lengths_violation, middle_loss, gate_check, goal_leave
    changed = sorted(n for n in changed if n)
    if not changed: return {}
    out = collections.defaultdict(set)
    def per(Hx, n):
        Hn = dict(Hx, tracks=[t for t in Hx['tracks'] if t['net'] == n], vias=[v for v in Hx['vias'] if v['net'] == n],
                  pads=[p for p in Hx['pads'] if p['net'] == n])
        t = gate_check(Hn); return (t['angle'], t['r32'], t['hairpins'])
    for n in changed:
        for g, a, b in zip(('angle', 'r32', 'hairpin'), per(H_new, n), per(H, n)):
            if a > b: out[n].add(g)
    for (n, L) in corners90_new(H, H_new, net_list=changed): out[n].add('corner90')
    for n in ends_new(H, H_new, fill, fill_new, net_list=changed): out[n].add('ends')
    for n in goal_leave(H, H_new, net_list=changed): out[n].add('goal')
    for n in middle_loss(H, H_new): out[n].add('middle')
    for n in (lengths_violation(L0, H_new, gate_nets) if gate_nets else {}): out[n].add('total_length')
    return {n: sorted(g) for n, g in sorted(out.items())}


def _layers_union(fill):
    from shapely.ops import unary_union
    z = collections.defaultdict(list)
    for f in fill: z[(f['net'], f['layer'])].append(f['poly'])
    return {k: unary_union(v) for k, v in z.items()}


def _parts(g, amin=0.0):
    return [p for p in (g.geoms if hasattr(g, 'geoms') else [g]) if not p.is_empty and p.area > amin]


def zones_gate(fill_ref, fill_new, H_ref, H_new, changed, white_max=0.1, bands=0.1, bands_min=0.01, area_max=0.1, clr=0.15,
              border=0.5, total=False, contours=None, fill_start=None, H_start=None):
    from shapely.geometry import LineString, Point, Polygon
    from shapely.ops import unary_union
    changed = set(n for n in changed if n)
    if not changed or not (fill_ref or fill_new): return {}, []
    r6 = lambda p: (round(p[0], 6), round(p[1], 6))
    key = lambda t: (t['net'], t['layer'], tuple(sorted([r6(t['a']), r6(t['b'])])), round(t['w'], 6))
    prior = collections.Counter(key(t) for t in H_ref['tracks'] if t['net'] in changed)
    new = collections.Counter(key(t) for t in H_new['tracks'] if t['net'] in changed)
    diff = collections.defaultdict(list)
    for (n, L, (a, b), w) in list((prior - new).elements()) + list((new - prior).elements()):
        diff[L].append((n, (LineString([a, b]) if a != b else Point(a)).buffer(w / 2, 8)))
    Z0, Z1 = _layers_union(fill_ref), _layers_union(fill_new)
    ZS = _layers_union(fill_start) if fill_start is not None else None
    blame = collections.defaultdict(set); findings = []
    if total:
        for (zn, L) in sorted(set(Z0) | set(Z1)):
            a0 = Z0[(zn, L)].area if (zn, L) in Z0 else 0.0; a1 = Z1[(zn, L)].area if (zn, L) in Z1 else 0.0
            if a1 < a0 - area_max: findings.append(dict(category='area_total', net_name=zn, layer_name=L, before=round(a0, 3), after=round(a1, 3)))
    if not diff: return {}, findings
    def culprits(L, g, r, without=None):
        return sorted({n for n, s_ in diff.get(L, []) if n != without and s_.distance(g) < r})
    for L in sorted(diff):
        window = unary_union([s_ for _, s_ in diff[L]]).buffer(2.0, 8); fb = window.buffer(0.5, 8)
        zl = sorted({n for n, L2 in set(Z0) | set(Z1) if L2 == L})

        per_net = collections.defaultdict(list)
        for n, s_ in diff[L]: per_net[n].append(s_)
        costs = collections.defaultdict(list)
        for n in sorted(per_net):
            surround = unary_union(per_net[n]).buffer(border, 8)
            for zn in zl:
                if zn == n: continue
                a0 = Z0[(zn, L)].intersection(surround).area if (zn, L) in Z0 else 0.0
                a1 = Z1[(zn, L)].intersection(surround).area if (zn, L) in Z1 else 0.0
                aS = (ZS[(zn, L)].intersection(surround).area if (zn, L) in ZS else 0.0) if ZS is not None else a0
                if a1 < a0 - 1e-6: costs[zn].append(n)
                if a1 < a0 - area_max or (a1 < a0 - 1e-6 and a1 < aS - area_max):
                    findings.append(dict(category='surface', net_name=zn, layer_name=L, blame=[n], before=round(a0, 3), after=round(a1, 3),
                                        start=round(aS, 3)))
                    blame[n].add('surface')
        if ZS is not None and H_start is not None:
            ks = collections.Counter(key(t) for t in H_start['tracks'] if t['layer'] == L)
            kn = collections.Counter(key(t) for t in H_new['tracks'] if t['layer'] == L)
            since = collections.defaultdict(list)
            for (m, L2, (a, b), w) in list((ks - kn).elements()) + list((kn - ks).elements()):
                since[m].append((LineString([a, b]) if a != b else Point(a)).buffer(w / 2, 8))
            now = {n: unary_union(g) for n, g in per_net.items()}
            for m in sorted(since):
                surround = unary_union(since[m]).buffer(border, 8)
                nearby = [n for n, g in now.items() if g.intersects(surround)]
                if not nearby: continue
                for zn in zl:
                    if zn == m: continue
                    a0 = Z0[(zn, L)].intersection(surround).area if (zn, L) in Z0 else 0.0
                    a1 = Z1[(zn, L)].intersection(surround).area if (zn, L) in Z1 else 0.0
                    aS = ZS[(zn, L)].intersection(surround).area if (zn, L) in ZS else 0.0
                    if a1 < aS - area_max and a1 < a0 - 1e-6:
                        findings.append(dict(category='surface', net_name=zn, layer_name=L, surroundings=m, blame=sorted(nearby), before=round(a0, 3),
                                            after=round(a1, 3), start=round(aS, 3)))
                        for n in nearby: blame[n].add('surface')
        if ZS is not None:
            for zn in zl:
                aS = ZS[(zn, L)].area if (zn, L) in ZS else 0.0; a1 = Z1[(zn, L)].area if (zn, L) in Z1 else 0.0
                a0 = Z0[(zn, L)].area if (zn, L) in Z0 else 0.0
                if a1 < aS - area_max and a1 < a0 - 1e-6:


                    sch = sorted(costs.get(zn) or per_net)
                    findings.append(dict(category='area_total', net_name=zn, layer_name=L, blame=sch, start=round(aS, 3), before=round(a0, 3), after=round(a1, 3)))
                    for n in sch: blame[n].add('surface')

        cu0, cu1 = [], []
        for Hx, cu in ((H_ref, cu0), (H_new, cu1)):
            for t in Hx['tracks']:
                if t['layer'] != L: continue
                g = (LineString([t['a'], t['b']]) if t['a'] != t['b'] else Point(t['a']))
                if g.distance(window) > 0.5: continue
                cu.append(g.buffer(t['w'] / 2, 8))
            for v in Hx['vias']:
                if (not v.get('layers') or L in v['layers']) and Point(v['x'], v['y']).distance(window) <= 0.5:
                    cu.append(Point(v['x'], v['y']).buffer(v['d'] / 2, 16))
            for p in Hx['pads']:
                if p['layer'] == L and len(p.get('poly') or []) >= 3:
                    g = Polygon(p['poly']).buffer(0)
                    if g.distance(window) <= 0.5: cu.append(g)
        board = unary_union([Polygon(e).buffer(0) for e in H_new.get('edges', []) if len(e) > 2])
        inner = board.buffer(-0.5).intersection(window) if not board.is_empty else window

        group_list = collections.defaultdict(list)
        for u in contours or ():
            if u['layer'] == L: group_list[round(max(clr, u.get('clr', 0.0)), 4)].append(u['poly'])
        regions = [(c_, inner.intersection(unary_union(gs))) for c_, gs in sorted(group_list.items())]
        regions = [(c_, b_) for c_, b_ in regions if not b_.is_empty]
        inner = unary_union([b_ for _, b_ in regions] or [Polygon()])
        blocked = unary_union([Polygon(k['poly']).buffer(0) for k in H_new.get('keepouts', []) if L in (k.get('layers') or [])] or [Polygon()])
        def white(cu, Z):
            zf = [g.intersection(fb) for (n, L2), g in Z.items() if L2 == L]
            k = unary_union(cu + zf)
            return unary_union([b_.difference(k.buffer(c_ + 0.01, 8)).difference(blocked.buffer(c_)) for c_, b_ in regions] or [Polygon()])
        w0, w1 = (white(cu0, Z0), white(cu1, Z1)) if not inner.is_empty else (Polygon(), Polygon())
        r_w = max([0.4] + [c_ + 0.25 for c_, _ in regions])
        for g in _parts(w1.difference(w0.buffer(0.01)), white_max):
            c = g.representative_point(); sch = culprits(L, g, r_w)
            findings.append(dict(category='white', layer_name=L, mm2=round(g.area, 3), at=[round(c.x, 2), round(c.y, 2)], blame=sch))
            for n in sch: blame[n].add('white')
        def thin(g):
            return g.difference(g.buffer(-bands / 2, 8).buffer(bands / 2, 8)) if not g.is_empty else g
        for zn in sorted({n for n, L2 in Z1 if L2 == L}):
            g1 = Z1[(zn, L)].intersection(fb); g0 = Z0[(zn, L)].intersection(fb) if (zn, L) in Z0 else Polygon()
            s1, s0 = thin(g1), thin(g0)
            for g in _parts(s1.difference(s0.buffer(0.01)).intersection(window), bands_min):
                c = g.representative_point(); sch = culprits(L, g, 0.4, without=zn)
                findings.append(dict(category='bands', net_name=zn, layer_name=L, mm2=round(g.area, 4), at=[round(c.x, 2), round(c.y, 2)], blame=sch))
                for n in sch: blame[n].add('bands')
    return {n: sorted(g) for n, g in sorted(blame.items())}, findings


SUPPLY = re.compile(r'(?i)(^|/)[+]|(^|[/_.])(vcc|vdd|vee|vbus|vbat|vio|vin|vsys|vref|v_dram)|(^|[/_.])p_?\d|(^|[/_.])\d+v\d*($|[_ ])|(^|[/_.])\d+v\d+')


def levels_hmax(H, fill, share=0.3):
    from shapely.geometry import Polygon
    from postprocessing.keepout_nets import GROUND
    rings = [Polygon(e).buffer(0) for e in H.get('edges', []) if len(e) > 2]
    if not rings or not fill: return {}
    surface = max(g.area for g in rings)
    per = collections.defaultdict(float)
    for f in fill: per[f['layer']] += f['poly'].area
    levels = {L for L, a in per.items() if a >= share * surface}
    supply = {f['net'] for f in fill if f['net'] and not GROUND.search(f['net'])}
    supply |= {t['net'] for t in H['tracks'] if t['net'] and SUPPLY.search(t['net']) and not GROUND.search(t['net'])
                   and not re.search(r'(?i)_EN$|ENABLE', t['net'])}
    out = collections.defaultdict(set)
    for t in H['tracks']:
        if t['net'] in supply and t['layer'] in levels: out[t['net']].add(t['layer'])
    return {n: sorted(v) for n, v in sorted(out.items())}


def fix_hash(H, fix):
    import hashlib
    r = lambda v: round(float(v), 6)
    T = sorted((t['uuid'], t['layer'], r(t['a'][0]), r(t['a'][1]), r(t['b'][0]), r(t['b'][1]), r(t['w'])) for t in H['tracks'] if t.get('uuid') in fix)
    return hashlib.sha256(json.dumps([len(fix), T]).encode()).hexdigest()


def via_sieve(H, D):
    from common.copper_net import apply
    u_net = {t['uuid']: t['net'] for t in H['tracks']}
    net_list = sorted({a_['net'] for a_ in D['add']} | {u_net.get(u) for u in D['remove']} - {None})
    pads = collections.defaultdict(list)
    for p in H.get('pads', []):
        if len(p.get('poly') or []) >= 3: pads[(p['net'], p['layer'])].append(p['poly'])
    def cu_layers(v, ts, stub=False):
        c = (v['x'], v['y']); r = v['d'] / 2; out = set()
        for t in ts:
            if t['layer'] in out or (v.get('layers') and t['layer'] not in v['layers']): continue
            a, b = t['a'], t['b']
            if not stub and math.dist(a, c) < r - 1e-6 and math.dist(b, c) < r - 1e-6: continue
            if math.dist(a, c) <= r + t['w'] / 2 + 1e-6 or math.dist(b, c) <= r + t['w'] / 2 + 1e-6 or \
                    (a != b and _stretch_spacing(c, a, b) <= r + t['w'] / 2 + 1e-6):
                out.add(t['layer'])
        return out
    out = []
    for n in net_list:
        prior = [t for t in H['tracks'] if t['net'] == n]; new = apply(prior, D['remove'], [a_ for a_ in D['add'] if a_['net'] == n])
        def at_pad(v, L):
            from shapely.geometry import Point, Polygon
            return any(Polygon(q).buffer(0).distance(Point(v['x'], v['y'])) <= v['d'] / 2 + 1e-6 for q in pads.get((n, L), ()))
        def loose(v):
            if any(not at_pad(v, L) for L in cu_layers(v, prior) - cu_layers(v, new)): return True
            pl = {L for L in (v.get('layers') or []) if at_pad(v, L)}
            return len(cu_layers(v, new, True) | pl) < 2 <= len(cu_layers(v, prior, True) | pl)
        if any(loose(v) for v in H['vias'] if v['net'] == n): out.append(n)
    if not out: return D, []
    st = dict(D.get('stat') or {}, via_back=out)
    return dict(D, remove=[u for u in D['remove'] if u_net.get(u) not in out], add=[a_ for a_ in D['add'] if a_['net'] not in out], stat=st), out


def _stretch_spacing(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]; L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def _chords(D):
    if not any('m' in a_ for a_ in D.get('add', [])): return D
    add = []
    for a_ in D['add']:
        if 'm' not in a_: add.append(a_); continue
        b_ = {k_: v_ for k_, v_ in a_.items() if k_ not in ('m', 'uuid')}
        add += [dict(b_, b=a_['m']), dict(b_, a=a_['m'])]
    return dict(D, add=add)

def _without_nets(D, net_list, H):
    if not net_list: return D
    out = set(net_list); u_net = {t['uuid']: t['net'] for t in H['tracks']}
    return dict(D, remove=[u for u in D['remove'] if u_net.get(u) not in out], add=[a_ for a_ in D['add'] if a_['net'] not in out],
                add_generated=[g for g in D.get('add_generated', []) if (g.get('net_name') if isinstance(g, dict) else None) not in out])


def zones_sieve(H, D, fill, drc_=0.09):
    if not fill or not D.get('add'): return D, []
    ZW = ZonesGuard(H, fill, drc=drc_); u_net = {t['uuid']: t['net'] for t in H['tracks']}
    rm = collections.defaultdict(list); ad = collections.defaultdict(list)
    for u in D['remove']: rm[u_net.get(u)].append(u)
    for a_ in D['add']: ad[a_['net']].append(a_)
    out = []
    for n in sorted(set(rm) | set(ad), key=lambda x: x or ''):
        if ad.get(n) and ZW.verify(n, ad[n]): out.append(n); continue
        ZW.commit(n, ad.get(n, []), rm.get(n, []))
    if not out: return D, []
    st = dict(D.get('stat') or {}, zones_back=out)
    return dict(D, remove=[u for u in D['remove'] if u_net.get(u) not in out], add=[a_ for a_ in D['add'] if a_['net'] not in out], stat=st), out


def run_contract(one, pro, dru, o):
    import hashlib, platform, shapely
    def sha(p): return hashlib.sha256(open(p, 'rb').read()).hexdigest()
    git = lambda *x: subprocess.run(['git', '-C', A] + list(x), capture_output=True, text=True).stdout.strip()
    cli = subprocess.run([CLI, '--version'], capture_output=True, text=True).stdout.strip()
    return dict(commit=git('rev-parse', 'HEAD'), worktree_diff_sha256=hashlib.sha256(git('diff', 'HEAD').encode()).hexdigest(),
                options={k: (sorted(v) if isinstance(v, set) else v) for k, v in o.items() if not k.startswith('_')},
                input_sha256=dict(pcb=sha(one), pro=sha(pro), dru=sha(dru)), python=platform.python_version(),
                shapely=shapely.__version__, kicad_cli=cli)


def step_corners45(H, S, fwd, o, cur, tmp):
    from postprocessing.corners45 import corners45
    fl = [(f['net'], f['layer'], f['poly']) for f in (o.get('_fill') or [])]
    return corners45(H, keepout_list=S, net_list=o.get('_nets'), fix=o.get('_fix', ()), fwd=fwd, fill=fl, cmax=float(o.get('corners_cmax', 0.4)))


def step_meander(H, S, fwd, o, cur, tmp):
    from postprocessing.meander import meander_change
    back = {n for n, g in S.items() if g == 'withdrawn'}
    D = meander_change(H, fwd, o.get('_pairs') or [], o, keepout_list=back)
    return D



GR_POLY = re.compile(r'\n\t\(gr_poly\n.*?\n\t\)', re.S)


def shapes_generated(text, max_mm2=0.2):
    out = []
    for m in GR_POLY.finditer(text):
        b = m.group(0)
        n = re.search(r'\(net "((?:[^"\\]|\\.)*)"\)', b); L = re.search(r'\(layer "([^"]+\.Cu)"\)', b)
        if not n or not n.group(1) or not L: continue
        pts = [(float(x), float(y)) for x, y in re.findall(r'\(xy ([-0-9.]+) ([-0-9.]+)\)', b)]
        if len(pts) < 3: continue
        A = abs(sum(pts[i][0] * pts[i - 1][1] - pts[i - 1][0] * pts[i][1] for i in range(len(pts)))) / 2
        if A < max_mm2:
            u = re.search(r'\(uuid "([^"]+)"\)', b)
            out.append((m.start(), m.end(), u.group(1) if u else None, n.group(1), L.group(1), pts, b))
    return out


def shapes_remove(text, max_mm2=0.2, only=None):
    part_list = []; pos = 0; n = 0
    for a, e, u, *_r in shapes_generated(text, max_mm2):
        if only is not None and u not in only: continue
        part_list.append(text[pos:a]); pos = e; n += 1
    part_list.append(text[pos:])
    return ''.join(part_list), n


def teardrop_new(src, dst, pro, dru, o, tmp):
    from common.copper_net import zones_fill as zf
    clr = max([f.get('clr') or 0.0 for f in zf(open(src, encoding='utf-8').read())] + [0.0]) or 0.2
    vacant = o.get('teardrop_free') or '%g' % round(max(float(o.get('drc', 0.09)), float(o.get('drcpad', 0.10))) + 0.012, 3)
    clearance = o.get('teardrop_clearance') or '%g' % round(clr + 0.005, 3)
    border, hole_size = o.get('edge', '0.2'), o.get('hole_size', '0.25'); nc = ['netclasses=' + o['netclasses']] if o.get('netclasses') else []
    topt = ['r=' + o.get('teardrop_r', '0.4'), 'vacant=' + clearance, 'free_hard=' + clearance, 'border=' + border, 'hole_size=' + hole_size] + nc
    def job(cmd, name):
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0: raise RuntimeError('%s: %s' % (name, r.stderr[-600:]))
        return (r.stdout.strip().split('\n') or [''])[-1]
    def ex(pcb, k):
        j = lambda x: os.path.join(tmp, '%s_%s.json' % (x, k))
        job([KPY, os.path.join(A, 'kicad', 'fillet_post_run.py'), 'export', pcb, j('k')], 'fillet_post_run export')
        job([KPY, os.path.join(A, 'kicad', 'copper_export.py'), pcb, j('cu')], 'copper_export')
        job([KPY, os.path.join(A, 'kicad', 'keepout_export.py'), pcb, j('blocked')], 'keepout_export')
        return j('k'), j('blocked'), j('cu')
    def appendix(pcb, shapes, off):
        x = job([KPY, os.path.join(A, 'kicad', 'shapes_append.py'), pcb, shapes, off], 'shapes_append')
        b = off[:-10]; shutil.copy(pro, b + '.kicad_pro'); shutil.copy(dru, b + '.kicad_dru'); return x
    log_ = {}
    z = [os.path.join(tmp, 'tr_z%d.kicad_pcb' % i) for i in range(4)]
    k, sp, cu = ex(src, 0)
    log_['signal'] = job([VPY, os.path.join(A, 'postprocessing', 'white_teardrop.py'), k, sp, os.path.join(tmp, 'tr_t1.json')] + topt +
                          ['net_list=signal', 'zone_list=soft', 'pads=' + cu], 'white_teardrop signal')
    appendix(src, os.path.join(tmp, 'tr_t1.json'), z[1])
    k, sp, cu = ex(z[1], 1)
    log_['surface'] = job([VPY, os.path.join(A, 'postprocessing', 'white_teardrop.py'), k, sp, os.path.join(tmp, 'tr_t2.json')] + topt +
                           ['net_list=surface', 'pads=' + cu], 'white_teardrop area')
    appendix(z[1], os.path.join(tmp, 'tr_t2.json'), z[2])
    k, sp, cu = ex(z[2], 2)
    log_['notches'] = job([VPY, os.path.join(A, 'postprocessing', 'notches_fill.py'), k, sp, os.path.join(tmp, 'tr_t3.json'),
                           'free_hard=' + vacant, 'vacant=' + clearance, 'border=' + border, 'hole_size=' + hole_size, 'pads=' + cu] + nc, 'notches_fill')
    appendix(z[2], os.path.join(tmp, 'tr_t3.json'), z[3])
    log_['loose'] = job([KPY, os.path.join(A, 'kicad', 'teardrops_loose_remove.py'), z[3], dst], 'teardrop_loose_way')
    b = dst[:-10]; shutil.copy(pro, b + '.kicad_pro'); shutil.copy(dru, b + '.kicad_dru')
    return dict(log_, vacant=vacant, clearance=clearance, border=border, hole_size=hole_size)


def shapes_matching(old, H, text, r=0.5, rest=0.002):
    from shapely.geometry import LineString, Point, Polygon
    from shapely.ops import unary_union
    from common.copper_net import zones_fill as zf
    Lk = lambda L: L[:-3] if L.endswith('.Cu') else L
    cu = collections.defaultdict(list)
    for t in H['tracks']: cu[(t['net'], t['layer'])].append(LineString([t['a'], t['b']]).buffer(t['w'] / 2, 16) if t['a'] != t['b'] else Point(t['a']).buffer(t['w'] / 2))
    for p_ in H.get('pads', []):
        if len(p_.get('poly') or []) >= 3: cu[(p_['net'], p_['layer'])].append(Polygon(p_['poly']).buffer(0))
    cu_layers = {t['layer'] for t in H['tracks']} | {p_['layer'] for p_ in H.get('pads', [])}
    for v in H.get('vias', []):
        for L in v.get('layers') or cu_layers: cu[(v['net'], L)].append(Point(v['x'], v['y']).buffer(v['d'] / 2, 32))
    fill = collections.defaultdict(list)
    for f in zf(text): fill[(f['net'], f['layer'])].append(f['poly'])
    outlines = collections.defaultdict(list)
    for z in zones_contours(text): outlines[(z['net'], z['layer'])].append(z['poly'])
    cache = {}
    def geo(n, L):
        if (n, L) not in cache:
            c = unary_union(cu[(n, L)]) if cu[(n, L)] else None
            cache[(n, L)] = (c, c.buffer(r, 16).buffer(-r, 16) if c is not None else None,
                             unary_union(fill[(n, L)]) if fill[(n, L)] else None, unary_union(outlines[(n, L)]) if outlines[(n, L)] else None)
        return cache[(n, L)]
    matching, loose, shifted = [], [], []
    for x in old:
        n, L, pts = x[3], Lk(x[4]), x[5]
        F = Polygon(pts).buffer(0)
        c, too, fu, um = geo(n, L)
        an = [g for g in (c, fu) if g is not None]
        if not an or not any(g.buffer(1e-4).intersects(F) for g in an): loose.append(x); continue
        inner = unary_union([g for g in (too, um) if g is not None])
        if F.difference(inner.buffer(1e-4)).area > rest: shifted.append(x); continue
        matching.append(x)
    return matching, loose, shifted


def shapes_deploy(text, blocks):
    i = text.find('\n\t(zone')
    if i < 0: i = text.rstrip().rfind(')')
    return text[:i] + ''.join(blocks) + text[i:]


def teardrop_phase(cur, goal, pro, dru, ref, o, tmp, rounds_n=3, old=None, H=None):
    info0 = {}
    if old:
        text = open(cur, encoding='utf-8').read()
        matching, loose, moved = shapes_matching(old, H, text)
        info0 = dict(old=len(old), old_keep=len(matching), old_loose=len(loose), old_shifted=len(moved),
                     loose_at=[[round(sum(q[0] for q in x[5]) / len(x[5]), 2), round(sum(q[1] for q in x[5]) / len(x[5]), 2), x[3]] for x in loose][:30],
                     shifted_at=[[round(sum(q[0] for q in x[5]) / len(x[5]), 2), round(sum(q[1] for q in x[5]) / len(x[5]), 2), x[3]] for x in moved][:30])
        log('   old teardrops: %d fit, %d loose, %d shifted (only the fitting ones come back)' % (len(matching), len(loose), len(moved)))
        with_ = os.path.join(tmp, 'tr_prev.kicad_pcb'); copy(cur, with_, pro, dru)
        open(with_, 'w', encoding='utf-8').write(shapes_deploy(text, [x[6] for x in matching]))
        R_ = drc(with_, pro, dru); cur = with_
    info = dict(info0, **teardrop_new(cur, goal, pro, dru, o, tmp))
    text = open(goal, encoding='utf-8').read(); info['shapes'] = len(shapes_generated(text)); info['back'] = 0
    for round_no in range(rounds_n + 1):
        R = drc(goal, pro, dru); nev = new_events(R, ref)
        log('   Teardrops: %d shapes, DRC errors %s, open %d, new reports %d' % (info['shapes'] - info['back'], dict(R['E']) or 0, R['U'], len(nev)))
        if good(R, ref): return True, R, info
        if round_no == rounds_n: break
        from shapely.geometry import Point, Polygon
        places = [Point(i['pos']['x'], i['pos']['y']) for v in nev for i in v.get('items', []) if i.get('pos')]
        text = open(goal, encoding='utf-8').read()
        way = {u for _, _, u, n, L, pts, _b in shapes_generated(text) if u and any(Polygon(pts).buffer(0).distance(q) < 0.3 for q in places)}
        if not way: break
        text, n = shapes_remove(text, only=way); info['back'] += n
        open(goal, 'w', encoding='utf-8').write(text)
        log('   Teardrops: %d shapes reverted due to new reports' % n)
    return False, R, info


STANDARD_STEPS = 'tidy_up,shorten,smooth,middle,corners45,tidy_up,meander'
SHAPE_EXCEPTIONS = dict(meander=('angle', 'hairpin'))
ZONES_EXCEPTIONS = dict(meander=('surface', 'white'))
STEPS = dict(tidy_up=step_tidy_up, shorten=step_shorten, smooth=step_smooth, middle=step_middle,
                corners45=step_corners45, meander=step_meander)


def main():
    a = sys.argv[1:]; o = dict(x.split('=', 1) for x in a[2:] if '=' in x)
    one = os.path.abspath(a[0]); off = os.path.abspath(a[1])
    if os.path.realpath(os.path.dirname(one)) == os.path.realpath(off):
        raise SystemExit('Output folder must be a different folder than the input folder')
    os.makedirs(off, exist_ok=True)
    pro, dru = one[:-10] + '.kicad_pro', one[:-10] + '.kicad_dru'
    if not (os.path.exists(pro) and os.path.exists(dru)): raise SystemExit('.kicad_pro/.kicad_dru missing next to ' + one)
    o['_dru'] = dru
    if o.get('net_list'): o['_nets'] = [n for n in o['net_list'].split(';') if n]
    steps = o.get('steps', STANDARD_STEPS).split(',')
    sieve = int(o.get('sieve', 3)); rounds_n = int(o.get('rounds_n', 3))
    tmp = os.path.join(off, 'tmp'); os.makedirs(tmp, exist_ok=True)
    cur = os.path.join(off, 'f00_start.kicad_pcb'); copy(one, cur, pro, dru)


    text_one = open(one, encoding='utf-8').read(); tmax = float(o.get('teardrop_max', 0.2))
    n_gen = len(shapes_generated(text_one, tmax))
    teardrop_on = o.get('teardrop', 'auto') == '1' or (o.get('teardrop', 'auto') == 'auto' and n_gen > 0)
    R_one = None; old_shapes = []
    if teardrop_on:
        if n_gen:
            before = os.path.join(off, 'f00_input.kicad_pcb'); copy(one, before, pro, dru)
            log('Input with %d generated copper shapes (teardrops): refill zones, DRC' % n_gen, before)
            R_one = drc(before, pro, dru)
            log('Input: errors %s, open %d, warnings %d' % (dict(R_one['E']) or 0, R_one['U'], sum(R_one['W'].values())))
            old_shapes = shapes_generated(text_one, tmax)
            text0, n_way = shapes_remove(text_one, tmax)
            open(cur, 'w', encoding='utf-8').write(text0)
            log('Teardrops: %d generated shapes removed (below %g mm²), new at the end' % (n_way, tmax))
        else:
            log('Teardrops: input without generated shapes, new at the end (teardrop=1)')
    log('Start: refill zones, DRC', cur)
    R0 = drc(cur, pro, dru); ref = R0; E0, U0, W0 = R0['E'], R0['U'], R0['W']
    log('Initial board: errors %s, open %d, warnings %d %s' % (dict(E0) or 0, U0, sum(W0.values()), dict(W0)))
    H0 = obst(cur, os.path.join(off, 'h_start.json'), [0, 0, 1000, 1000]); box = frame(H0)

    U = keepout_extent(H0, pcb=cur, pro=pro, dru=dru, netclasses=o.get('netclasses'), keepout_list=o.get('keepout_list'))
    S = U['net_list']; o['_fix'] = set(U['track_list'])
    gate_nets = sorted((set(U['group_list']) | set(U['pairs'])) - set(S))
    o['_gate_nets'] = gate_nets
    if 'meander' in steps:
        from common.pair_nets import pairs as _pairs
        from postprocessing.meander import skew_design_rules, tolerance
        _sr = skew_design_rules(open(dru, encoding='utf-8').read())
        o['_pairs'] = [(p_, n_, tolerance(p_, n_, _sr, float(o.get('skew_tol', 0.1))))
                       for p_, n_ in _pairs(sorted({t['net'] for t in H0['tracks'] if t['net']}), o.get('netclasses'))]
        log('Meander: %d pairs:' % len(o['_pairs']), ', '.join('%s (±%g)' % (p_.split('/')[-1], t_) for p_, n_, t_ in o['_pairs']))
    json.dump(dict(S, **{'#track_list': U['track_list'], '#length_groups': U['group_list']}), open(os.path.join(off, 'keepout_nets.json'), 'w'),
              indent=1, ensure_ascii=False)
    log('blocked:', len(S), 'Nets:', ', '.join('%s (%s)' % (n.split('/')[-1], g.split(' ')[0]) for n, g in S.items()))
    log('fixed tracks (Kelvin):', len(U['track_list']), '; length gate for', len(gate_nets), 'Nets:', ', '.join(n.split('/')[-1] for n in gate_nets)[:400])
    from postprocessing.keepout_nets import _pcb_read
    o['_zone_nets'] = _pcb_read(cur)[2]
    for n_ in U.get('without_path', []): log('WARNING: Kelvin sense line without copper path from IC to shunt, net fully locked:', n_)
    hash0 = geo_hash(H0, S); fix0 = fix_hash(H0, o['_fix'])
    from analysis.finished_metrics import length_list
    L0 = length_list(H0); o['_L0'] = L0

    o['_hmax_layers'] = levels_hmax(H0, _zones_fill(open(cur, encoding='utf-8').read())) if float(o.get('hmax_plane', 0.3)) > 0 else {}
    if o['_hmax_layers']: log('Supply on plane layers (hmax %s):' % o.get('hmax_plane', '0.3'),
                             ', '.join('%s %s' % (n.split('/')[-1], '/'.join(Ls)) for n, Ls in o['_hmax_layers'].items())[:400])
    logbook = []; k = 0; H = H0; meander_nets = set()
    back = collections.defaultdict(set)
    faults = collections.defaultdict(lambda: collections.defaultdict(set))
    zone_gate_on = o.get('zone_gate', '1') == '1'; step_nr = 0
    fill_start = _zones_fill(open(cur, encoding='utf-8').read())
    for name in steps:
        for rnd in range(rounds_n if name in ('shorten', 'middle') else 1):
            text_cur = open(cur, encoding='utf-8').read()
            fill = _zones_fill(text_cur)
            contours = zones_contours(text_cur)
            o['_areas'] = [(f['net'], f['layer'], f['poly'], f['clr']) for f in fill if f['net'] in S]
            o['_fill'] = fill
            fwd = Obstacles(H, drc=float(o.get('drc', 0.09)), drcpad=float(o.get('drcpad', 0.10)), drcbga=float(o.get('drcbga', 0.09)),
                              drcvia=float(o.get('drcvia', o.get('drc', 0.09))), hole_size=float(o.get('hole_size', 0.25)), edge=float(o.get('edge', 0.2)),
                              net_spacing=net_spacings(dru, {t['net'] for t in H['tracks']}), areas=o['_areas'])
            step_nr += 1; sname = '%02d %s R%d' % (step_nr, name, rnd + 1)
            log('Step %s%s' % (name, ' round %d' % (rnd + 1) if rnd else ''))
            D = STEPS[name](H, dict(S, **{n: 'withdrawn' for n in back[name]}), fwd, o, cur, tmp)
            zk = []
            if name != 'shorten':
                _, zk = zones_sieve(H, _chords(D), fill, float(o.get('drc', 0.09))); D = _without_nets(D, zk, H)
                if zk: log('   Zones: reverted (cuts foreign zone or costs fill):', ', '.join(sorted(n.split('/')[-1] for n in zk)))
                for n_ in zk: faults[sname]['zones_sieve'].add(n_)
            _, via_cost = via_sieve(H, _chords(D)); D = _without_nets(D, via_cost, H)
            if via_cost: log('   Vias: reverted (via would lose a layer):', ', '.join(sorted(n.split('/')[-1] for n in via_cost)))
            for n_ in via_cost: faults[sname]['via_sieve (loose via)'].add(n_)
            D['_uuid_net'] = {t['uuid']: t['net'] for t in H['tracks']}
            log('   Proposal: removed %d, new %d' % (len(D['remove']), len(D['add'])), json.dumps(D.get('stat'), ensure_ascii=False)[:600])
            if not D['remove'] and not D['add']:
                if zk or via_cost: logbook.append(dict(step=name, round_no=rnd + 1, nr=step_nr, accepted=False, reason='everything screened out',
                                                   stat=D.get('stat')))
                break
            goal = os.path.join(off, 'f%02d_%s.kicad_pcb' % (k + 1, name))
            ok, Dk, info = apply_verify(cur, D, goal, pro, dru, ref, sieve)
            R_new = info.pop('_R', None)
            logbook.append(dict(step=name, round_no=rnd + 1, nr=step_nr, suggestion=dict(removed=len(D['remove']), new=len(D['add'])), stat=D.get('stat'),
                                  accepted=ok, accepted_change=dict(removed=len(Dk['remove']), new=len(Dk['add'])) if ok else None, **info))
            back[name] |= set(info.get('back') or [])
            for n_ in info.get('back') or []: faults[sname]['drc_sieve'].add(n_)
            if not ok:
                log('   rejected:', info.get('reason')); break
            H_new = obst(goal, os.path.join(tmp, 'h_%02d.json' % (k + 1)), [0, 0, 1000, 1000])


            t0 = shape_gate(H, H, L0, gate_nets)
            reasons_all = {}; zones_findings = []
            for attempt in range(3):
                changed = {s_['net'] for s_ in Dk['add']} | {D['_uuid_net'].get(u) for u in Dk['remove']} - {None}
                fill_new = _zones_fill(open(goal, encoding='utf-8').read())
                gr = {n_: ['shape_gate:' + g_ for g_ in v_ if g_ not in SHAPE_EXCEPTIONS.get(name, ())]
                      for n_, v_ in shape_reasons(H, H_new, L0, gate_nets, changed, fill, fill_new).items()}
                gr = {n_: g_ for n_, g_ in gr.items() if g_}
                if zone_gate_on:
                    zt, cmd = zones_gate(fill, fill_new, H, H_new, changed, white_max=float(o.get('white_max', 0.1)),
                                        bands=float(o.get('bands', 0.1)), area_max=float(o.get('area_max', 0.1)), contours=contours,
                                        fill_start=fill_start, H_start=H0)
                    zones_findings += cmd
                    for n_, g_ in zt.items():
                        g_ = [x for x in g_ if x not in ZONES_EXCEPTIONS.get(name, ())]
                        if g_: gr.setdefault(n_, []).extend('zone_gate:' + x for x in g_)
                if not gr: break
                for n_, g_ in gr.items():
                    reasons_all.setdefault(n_, []).extend(x for x in g_ if x not in reasons_all.get(n_, []))
                    for x in g_: faults[sname][x].add(n_)
                log('   Shape gate/zone gate: reverting:', '; '.join('%s (%s)' % (n_.split('/')[-1], ', '.join(g_)) for n_, g_ in sorted(gr.items()))[:900])
                back[name] |= set(gr)
                u_net = D['_uuid_net']
                D2 = dict(D, remove=[u for u in Dk['remove'] if u_net.get(u) not in gr], add=[a_ for a_ in Dk['add'] if a_['net'] not in gr],
                          add_generated=[g for g in Dk.get('add_generated', []) if g.get('net_name') not in gr])
                ok2 = bool(D2['remove'] or D2['add'])
                if ok2:
                    ok2, Dk, info2 = apply_verify(cur, D2, goal, pro, dru, ref, sieve); R_new = info2.pop('_R', None)
                    for n_ in info2.get('back') or []: faults[sname]['drc_sieve'].add(n_); back[name].add(n_)
                if not ok2: break
                H_new = obst(goal, os.path.join(tmp, 'h_%02d.json' % (k + 1)), [0, 0, 1000, 1000])
            else:
                ok2 = False
            if reasons_all: logbook[-1].update(shape_gate_back=sorted(reasons_all), shape_gate_reasons=reasons_all)
            if zones_findings: logbook[-1]['zone_gate_findings'] = zones_findings[:50]
            if reasons_all and not ok2:
                log('   rejected: shape gate/zone gate'); logbook[-1].update(accepted=False, reason='ShapeGate/ZoneGate'); break
            t1 = shape_gate(H_new, H, L0, gate_nets)
            if reasons_all: logbook[-1].update(accepted_change=dict(removed=len(Dk['remove']), new=len(Dk['add'])))
            if name == 'meander':
                meander_nets.update(s_['net'] for s_ in Dk['add']); meander_nets.update(D['_uuid_net'].get(u) for u in Dk['remove'])
                meander_nets.discard(None)
            k += 1; cur = goal; H = H_new; ref = R_new
            log('   accepted ->', os.path.basename(cur), 'ShapeGate', t0, '->', t1)
    teardrop_info = None; before_teardrop = cur
    if teardrop_on:
        log('Teardrops new (zones.sh step 6)')
        goal = os.path.join(off, 'f%02d_teardrop.kicad_pcb' % (k + 1))
        ok_t, R_t, teardrop_info = teardrop_phase(cur, goal, pro, dru, ref, o, tmp, old=old_shapes,
                                                H=obst(cur, os.path.join(tmp, 'h_before_teardrop.json'), [0, 0, 1000, 1000]) if old_shapes else None)
        teardrop_info['accepted'] = ok_t
        logbook.append(dict(step='teardrop', round_no=1, nr=step_nr + 1, accepted=ok_t, stat=teardrop_info))
        if ok_t: k += 1; cur = goal; ref = R_t; log('   accepted ->', os.path.basename(cur))
        else: log('   rejected: teardrops brought new DRC reports; result without generated teardrops')
    finished = os.path.join(off, 'finished.kicad_pcb'); copy(cur, finished, pro, dru)
    R1 = drc(finished, pro, dru); E1, U1, W1 = R1['E'], R1['U'], R1['W']
    H1 = obst(finished, os.path.join(off, 'h_finished.json'), [0, 0, 1000, 1000])

    NK0, NK1 = NetCopper(H0), NetCopper(H1)
    separated = {}
    for net in sorted({t['net'] for t in H0['tracks'] if t['net']}):
        k0, k1 = NK0.components(net), NK1.components(net)
        g = collections.defaultdict(set)
        for pid, c in k0.items(): g[c].add(pid)
        sch = [sorted(ms) for ms in g.values() if len({k1.get(m) for m in ms}) > 1]
        if sch: separated[net] = sch
    S_hash = {n: g for n, g in S.items() if n not in meander_nets}
    hash0 = geo_hash(H0, S_hash); hash1 = geo_hash(H1, S_hash); fix1 = fix_hash(H1, o['_fix'])
    from analysis.finished_metrics import lengths_violation, middle_loss
    mv = middle_loss(H0, H1); lv = lengths_violation(L0, H1, gate_nets)

    from analysis.finished_metrics import corners90_new, ends_new, goal_leave


    with_input = bool(R_one and teardrop_info and teardrop_info.get('accepted'))
    fill0 = _zones_fill(open(os.path.join(off, 'f00_input.kicad_pcb' if with_input else 'f00_start.kicad_pcb'), encoding='utf-8').read())
    final = before_teardrop if teardrop_info and teardrop_info.get('accepted') and not with_input else finished
    fill1 = _zones_fill(open(final, encoding='utf-8').read())
    changed_all = {t['net'] for t in H0['tracks']} | {t['net'] for t in H1['tracks']}
    en = ends_new(H0, H1, fill0, fill1)
    e90 = {'%s %s' % k_: z_ for k_, z_ in sorted(corners90_new(H0, H1).items())}
    zv = goal_leave(H0, H1)
    _, zone_findings = zones_gate(fill0, fill1, H0, H1, changed_all, white_max=float(o.get('white_max', 0.1)), bands=float(o.get('bands', 0.1)),
                        area_max=float(o.get('area_max', 0.1)), total=True,
                        contours=zones_contours(open(finished, encoding='utf-8').read())) if o.get('zone_gate', '1') == '1' else ({}, [])


    def _mae(b):
        if not meander_nets: return False
        sch = set(b.get('blame') or [])
        if b['category'] in ('white', 'surface') and sch and sch <= meander_nets: return True
        if b['category'] == 'area_total' and not sch:
            fl = [x for x in zone_findings if x['category'] == 'surface' and (x.get('net_name'), x.get('layer_name')) == (b.get('net_name'), b.get('layer_name'))]
            return bool(fl) and all(set(x.get('blame') or []) and set(x['blame']) <= meander_nets for x in fl)
        return False
    zone_findings_mae = [b for b in zone_findings if _mae(b)]; zone_findings = [b for b in zone_findings if not _mae(b)]
    end = dict(ends_new=en, corners90_new=e90, goal_leave=zv, zones_findings=zone_findings, meander_findings=zone_findings_mae,
                bands_new=sum(1 for b in zone_findings if b['category'] == 'bands'), white_new=sum(1 for b in zone_findings if b['category'] == 'white'),
                area_loss=[b for b in zone_findings if b['category'] in ('surface', 'area_total')])
    sieve_nets = sorted({n for p_ in logbook for n in (p_.get('back') or [])})
    counter = dict(sieve_interventions=sum(1 for p_ in logbook if p_.get('back')), sieve_back_nets=len(sieve_nets), sieve_nets=sieve_nets,
                   shape_gate_back_nets=sorted({n for p_ in logbook for n in (p_.get('shape_gate_back') or [])}),
                   zones_back_nets=sorted({n for gg in faults.values() for n in gg.get('zones_sieve', ())}),
                   via_back_nets=sorted({n for gg in faults.values() for n in gg.get('via_sieve (loose via)', ())}),
                   middle_lost=sum(mv.values()), middle_lost_per_net=mv, length_gate_violations=lv,
                   tool_error={st: {g: sorted(ns) for g, ns in sorted(gg.items())} for st, gg in faults.items() if gg},
                   tool_error_total_sum=dict(collections.Counter(g.split(':')[0] if g.startswith('zone_gate') else g
                                                                 for gg in faults.values() for g, ns in gg.items() for _ in ns)),
                   shape_gate_interventions_per_reason=dict(collections.Counter(g for gg in faults.values() for g, ns in gg.items()
                                                                       if g.startswith(('shape_gate', 'zone_gate')) for _ in ns)))

    def surface(fl, per_layer=False):
        z = collections.defaultdict(float)
        for f in fl: z[(f['net'], f['layer']) if per_layer else f['net']] += f['poly'].area
        return z
    z0, z1 = surface(fill0), surface(fill1)
    zones_diff = {n: dict(before_mm2=round(z0[n], 2), after_mm2=round(z1[n], 2), diff_mm2=round(z1[n] - z0[n], 2))
                  for n in sorted(set(z0) | set(z1)) if abs(z1[n] - z0[n]) > 0.05}
    zl0, zl1 = surface(fill0, True), surface(fill1, True)
    zones_layer = {'%s %s' % k_: dict(before_mm2=round(zl0[k_], 3), after_mm2=round(zl1[k_], 3), diff_mm2=round(zl1[k_] - zl0[k_], 3))
                  for k_ in sorted(set(zl0) | set(zl1)) if abs(zl1[k_] - zl0[k_]) > 0.01}
    from analysis.finished_metrics import measure
    metric = dict(before=dict(all_items=measure(H0), without_blocked=measure(H0, S)), after=dict(all_items=measure(H1), without_blocked=measure(H1, S)))
    report = dict(input_=one, outputs=finished, steps=logbook, blocked=S,
                   drc_before=dict(faults=dict(E0), open_items=U0, warnings=dict(W0)), drc_after=dict(faults=dict(E1), open_items=U1, warnings=dict(W1)),
                   drc_ok=good(R1, R0), drc_clean=U1 == 0 and not E1, drc_new_messages=len(new_events(R1, R0)), connection_separated=separated,
                   drc_input=dict(faults=dict(R_one['E']), open_items=R_one['U'], warnings=dict(R_one['W'])) if R_one else None,
                   drc_new_against_input=len(new_events(R1, R_one)) if R_one else None, teardrop=teardrop_info,
                   meander=dict(net_list=sorted(meander_nets), skew_error_before=E0.get('skew_out_of_range', 0),
                                 skew_error_after=E1.get('skew_out_of_range', 0),
                                 pairs=next((p_['stat'].get('pairs') for p_ in logbook if p_['step'] == 'meander' and p_.get('accepted')), None)),
                   blocked_hash=dict(before=hash0, after=hash1, equal=hash0 == hash1),
                   fixed_tracks=dict(count=len(o['_fix']), before=fix0, after=fix1, equal=fix0 == fix1), length_gate_nets=gate_nets,
                   counter=counter, zone_area=zones_diff, zone_area_per_layer=zones_layer, end=end, metrics=metric,
                   job=run_contract(one, pro, dru, o))
    json.dump(report, open(os.path.join(off, 'report.json'), 'w'), indent=1, ensure_ascii=False)
    log('FINISHED: DRC errors %s, open %d, warnings %d (before %d), new reports %d; split %d nets; locked unchanged: %s, '
        'fixed tracks unchanged: %s; sieve reverted %d nets (%d interventions); center lost %d; length gate violations %d'
        % (dict(E1) or 0, U1, sum(W1.values()), sum(W0.values()), report['drc_new_messages'], len(separated), hash0 == hash1, fix0 == fix1,
           counter['sieve_back_nets'], counter['sieve_interventions'], counter['middle_lost'], len(lv)))
    log('  Tool errors per cause:', json.dumps(counter['tool_error_total_sum'], ensure_ascii=False),
        '; new compared to start: offsets/loose ends %d, 90° corners %d, left pad/via %d, strips %d, white %d, area loss %d'
        % (sum(en.values()), sum(e90.values()), sum(zv.values()), end['bands_new'], end['white_new'], len(end['area_loss'])))
    for part in ('all_items', 'without_blocked'):
        v, n = metric['before'][part], metric['after'][part]
        log('  %s: length %.1f -> %.1f, segments %d -> %d, moves %d -> %d, bends %d -> %d, stairs %s -> %s, micro<0,08 %d -> %d, R37 via %s -> %s, pad %s -> %s'
            % (part, v['length_mm'], n['length_mm'], v['segments'], n['segments'], v['wobble']['move_records'], n['wobble']['move_records'], v['r18']['bends'],
               n['r18']['bends'], v['stairs'].get('stairs'), n['stairs'].get('stairs'), v['micro']['under_0_08'], n['micro']['under_0_08'],
               v['r37'].get('via_beside_middle', 0), n['r37'].get('via_beside_middle', 0), v['r37'].get('pad_beside_middle', 0), n['r37'].get('pad_beside_middle', 0)))
    shape_ok = not en and not e90 and not zv and not end['bands_new'] and not end['white_new'] and not end['area_loss']
    return 0 if report['drc_ok'] and shape_ok and not separated and hash0 == hash1 and fix0 == fix1 and not mv and not lv else 1


if __name__ == '__main__':
    sys.exit(main())
