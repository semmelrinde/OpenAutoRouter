import json, math, os, sys, collections
R0 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, R0)
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import unary_union
from shapely.strtree import STRtree
from common.flow import analysis as flow_analysis


def main():
    B = json.load(open(sys.argv[1])); R = json.load(open(sys.argv[2])); Z = json.load(open(sys.argv[3])); K = json.load(open(sys.argv[4]))
    OUT = sys.argv[5]; a = dict(x.split('=', 1) for x in sys.argv[6:] if '=' in x)
    VD, VDR = [float(v) for v in a.get('via', '0.45/0.20').split('/')]; VR = VD / 2
    CLR = float(a.get('clr', '0.2')); H2H = float(a.get('h2h', '0.2')); EDGE = float(a.get('edge', '0.3'))
    PITCH = max(float(a.get('spacing', '0.7')), VDR + H2H + 0.02)
    I03 = float(a.get('current_03', '1.0')); RES = float(a.get('reserve', '1.3')); IMIN = float(a.get('minimum_a', '0.5'))
    VIP = a.get('vip', '0') == '1'; REL = a.get('relative', '0') == '1'
    I_VIA = I03 * VDR / 0.3
    board = unary_union([Polygon(o) for o in B['outline'] if len(o) >= 3]).buffer(-(EDGE + VR))
    GND = set(K.get('gnd') or [])

    geo = collections.defaultdict(list)
    for t in R['tracks']:
        for p, q, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            geo[t['layer']].append((t['net'], LineString([p, q]).buffer(w / 2, cap_style='round')))
    for p in B['pads']:
        for L, polys in p['polys'].items():
            for q in polys:
                if len(q) >= 3: geo[L].append((p['net'], Polygon(q).buffer(0)))
    smd = [(p['net'], Polygon(q).buffer(0)) for p in B['pads'] if not p.get('drill') for L, polys in p['polys'].items() for q in polys if len(q) >= 3]
    cu_layers = sorted(geo)
    tree_idx = {L: (STRtree([g for _, g in geo[L]]), [n for n, _ in geo[L]]) for L in cu_layers}
    zone_list = collections.defaultdict(list)
    foreign_zone = collections.defaultdict(list)
    for z in Z:
        if z.get('gnd_signal') or (z.get('prio', 1) == 0 and z['net'] in GND) or len(z['pts']) < 3: continue
        g = Polygon(z['pts'], z.get('holes') or []).buffer(0)
        zone_list[(z['net'], z['layer'])].append(g); foreign_zone[z['layer']].append((z['net'], g))
    bore = [(v['x'], v['y']) for v in R['vias']] + [(p['x'], p['y']) for p in B['pads'] if p.get('drill', 0) > 0]

    def vacant(x, y, net_name):
        pt = Point(x, y)
        if not board.contains(pt): return False
        if any(math.hypot(x - bx, y - by) < VDR + H2H for bx, by in bore): return False
        if not VIP and any(n == net_name and g.distance(pt) < VR for n, g in smd): return False
        circle = pt.buffer(VR + CLR)
        for L in cu_layers:
            tr, nets = tree_idx[L]
            for i in tr.query(circle):
                n = nets[int(i)]; g = geo[L][int(i)][1]
                if n == net_name: continue
                if g.distance(pt) < VR + CLR: return False
            for n, g in foreign_zone[L]:
                if n != net_name and g.distance(pt) < VR + CLR: return False
        return True

    def owned(x, y, net_name, L):
        pt = Point(x, y)
        if any(g.buffer(-VR * 0.5).contains(pt) for g in zone_list[(net_name, L)]): return True
        tr, nets = tree_idx[L]
        return any(nets[int(i)] == net_name and geo[L][int(i)][1].contains(pt) for i in tr.query(pt))

    new = []; report = []
    tracks_n = collections.defaultdict(list)
    for ti, t in enumerate(R['tracks']):
        if not t.get('sig'): tracks_n[t['net']].append(t)
    for net_name, k in sorted(K['net_list'].items()):
        if k.get('category') not in ('current_val', 'supply', 'sw') or net_name in GND: continue
        fl = k.get('flow') or {}
        I_net = float(fl.get('total') or 0.0)
        if I_net <= 0 or (fl.get('unit') != 'A' and not REL): continue
        tr = tracks_n.get(net_name) or []
        vs = [v for v in R['vias'] if v['net'] == net_name]
        if not tr or not vs: continue
        info = flow_analysis([dict(layer=t['layer'], pts=t['pts'], w=t['w']) for t in tr], vs,
                             [p for p in B['pads'] if p['net'] == net_name], fl)
        share = (info or {}).get('share') or {}
        for v in vs:

            seg = []; ant = 0.0; vp = Point(v['x'], v['y'])
            for ti, t in enumerate(tr):
                for si, (p, q) in enumerate(zip(t['pts'][:-1], t['pts'][1:])):
                    if LineString([p, q]).distance(vp) < 0.06:
                        far = q if math.dist(q, (v['x'], v['y'])) > math.dist(p, (v['x'], v['y'])) else p
                        seg.append((t['layer'], far, t['w'][si])); ant = max(ant, share.get((ti, si), 1.0 if not share else 0.0))
            lag = {L for L, _, _ in seg}
            for pp in B['pads']:
                if pp['net'] == net_name:
                    for L, polys in pp['polys'].items():
                        if any(len(q) >= 3 and Polygon(q).buffer(0.05).contains(vp) for q in polys): lag.add(L)
            if not seg or len(lag) < 2: continue
            I = ant * I_net
            if I < IMIN: continue
            n_nominal = max(2, math.ceil(RES * I / I_VIA))
            already = sum(1 for w in vs + new if w['net'] == net_name and math.hypot(w['x'] - v['x'], w['y'] - v['y']) < 2.5 * PITCH)
            missing = n_nominal - already
            if missing <= 0: continue
            wmin = min(w for _, _, w in seg)

            cand = []
            for L, far, w in seg:
                d = math.dist(far, (v['x'], v['y']))
                for k_ in (1, 2, 3):
                    r = k_ * PITCH
                    if r > d - 0.05: break
                    f = r / d; cand.append((r, L, v['x'] + f * (far[0] - v['x']), v['y'] + f * (far[1] - v['y'])))
            for r in (PITCH, 1.4 * PITCH, 2 * PITCH):
                for j in range(12):
                    ph = 2 * math.pi * j / 12; cand.append((r + 0.01, None, v['x'] + r * math.cos(ph), v['y'] + r * math.sin(ph)))
            assigned = 0; reason = collections.Counter(); bridges = []
            for r, LA, x, y in sorted(cand):
                if assigned >= missing: break
                if any(math.hypot(x - w['x'], y - w['y']) < PITCH * 0.95 for w in vs + new if w['net'] == net_name): reason['too near to via'] += 1; continue
                if not vacant(x, y, net_name): reason['foreign copper/drill/edge'] += 1; continue

                st = []; ok = True
                for L in sorted(lag - {LA}):
                    if owned(x, y, net_name, L): continue
                    line = LineString([(x, y), (v['x'], v['y'])]); band = line.buffer(wmin / 2 + CLR)
                    tr_, nets_ = tree_idx.get(L, (None, []))
                    disturb = tr_ is not None and any(nets_[int(i)] != net_name and geo[L][int(i)][1].intersects(band) for i in tr_.query(band))
                    disturb = disturb or any(n != net_name and g.intersects(band) for n, g in foreign_zone[L])
                    if disturb: ok = False; break
                    st.append(dict(net=net_name, layer=L, pts=[[round(x, 4), round(y, 4)], [v['x'], v['y']]], w=[wmin], sig=False, current_web=True))
                if not ok: reason['Web blocked'] += 1; continue
                nv = dict(net=net_name, x=round(x, 4), y=round(y, 4), current_via=True); new.append(nv); bore.append((x, y)); bridges += st; assigned += 1
            R['tracks'] += bridges
            report.append(dict(net_name=net_name, x=v['x'], y=v['y'], current_A=round(I, 3), nominal=n_nominal, before=already, new=assigned,
                                cu_layers=sorted(lag), bridges=len(bridges), declined=dict(reason)))
    R['vias'] = R['vias'] + new
    json.dump(R, open(OUT, 'w'))
    json.dump(dict(i_via_A=round(I_VIA, 3), reserve=RES, spacing=PITCH, sites=report), open(OUT.replace('.json', '_current_vias.json'), 'w'), indent=1)
    short = [b for b in report if b['before'] + b['new'] < b['nominal']]
    print('Current vias: %d spots, %d vias new (I_via %.2f A, reserve %.1f); too little room at %d spots' % (
        len(report), len(new), I_VIA, RES, len(short)))
    for b in sorted(report, key=lambda b: -b['current_A'])[:12]:
        print('  %-14s %5.2f A  target %d  before %d  new %d  %s  %s' % (b['net_name'][-14:], b['current_A'], b['nominal'], b['before'], b['new'], ','.join(b['cu_layers']), b['declined']))


if __name__ == '__main__':
    main()
