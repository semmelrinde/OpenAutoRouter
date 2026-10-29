import collections
import math
from shapely import affinity
from shapely.geometry import Polygon
from shapely.ops import unary_union
from common.values import type_


def hull(pts):
    pts = sorted(set(pts))
    if len(pts) < 3: return 0.0
    def cross(a, b, c): return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    def half(ps):
        out = []
        for p in ps:
            while len(out) > 1 and cross(out[-2], out[-1], p) <= 0: out.pop()
            out.append(p)
        return out[:-1]
    h = half(pts) + half(pts[::-1])
    return abs(sum(a[0]*b[1]-a[1]*b[0] for a, b in zip(h, h[1:]+h[:1]))) / 2


class IslandDesignRules:
    def __init__(self, part_list, netclasses, board=None):
        self.t = part_list; self.k = netclasses; self.gnd = set(netclasses['gnd'])
        self.pin = {}; self.loops = []; self.boost_loops = []; self.group_list = {}
        self.polys = {}
        if board:
            for p in board['pads']:
                r = p['ref']
                if r not in part_list: continue
                qs = [Polygon(q).buffer(0) for v in p['polys'].values() for q in v if len(q) >= 3]
                if qs:
                    g = affinity.translate(unary_union(qs), -part_list[r].x0, -part_list[r].y0)
                    key = (r,p['num'])
                    self.polys[key] = unary_union([self.polys[key],g]) if key in self.polys else g
        for a, island in sorted(netclasses['islands'].items()):
            if a not in part_list: continue
            used = collections.Counter()
            for e in sorted(island['decoupling'], key=lambda e: (e['C'], e['ref'])):
                r = e['ref']
                if r not in part_list: continue
                if e.get('over_coil'):
                    s = netclasses['part_list'][r].get('supports') or {}
                    if s.get('ref') in part_list and s.get('pin'):
                        self.pin[r] = (s['ref'], e['net_name'], {s['pin']})
                    continue
                pins = sorted(set(e['pins']))
                if not pins: continue

                p = min(pins, key=lambda p: (used[e['net_name'], p], p))
                used[e['net_name'], p] += 1
                self.pin[r] = (a, e['net_name'], {p})
            sw = [r for r in island['members'] if r in part_list and
                  netclasses['part_list'][r].get('function') == 'coil_sw']
            if not sw: continue
            core_set = {}
            for e in sorted(island['decoupling'],key=lambda e:(e['C'],e['ref'])):
                if e['ref'] in part_list: core_set.setdefault((e['net_name'],bool(e.get('over_coil'))),e['ref'])
            self.group_list[a] = sorted({a, *sw, *core_set.values(),
                                     *(r for r in island['members'] if r in part_list and
                                       netclasses['part_list'][r].get('function') == 'bootstrap')})
            for r in island['members']:
                t = netclasses['part_list'].get(r,{})
                s = t.get('supports') or {}
                if r in part_list and t.get('function') in ('coil_sw','bootstrap') and s.get('ref') in part_list and s.get('pin'):
                    self.pin[r] = (s['ref'],s['net_name'],{s['pin']})

            inside = {}
            for e in sorted(island['decoupling'], key=lambda e: (e['C'], e['ref'])):
                if not e.get('over_coil') and e['ref'] in part_list and e['pins']:
                    inside.setdefault(e['net_name'], e)
            for n, e in sorted(inside.items()): self.loops.append((a, e['ref'], n, set(e['pins'])))


            sw_net = {n for l in sw for _,n,*_ in part_list[l].pads if netclasses['net_list'].get(n,{}).get('category') == 'sw'}
            boost = any(n in inside for l in sw for _,n,*_ in part_list[l].pads if n not in sw_net)
            if boost:
                for d in sorted(island['members']):
                    if d not in part_list or type_(d,'') != 'D': continue
                    dp = part_list[d].pads
                    ds = [(num,n) for num,n,*_ in dp if n in sw_net]
                    do = [(num,n) for num,n,*_ in dp if n not in sw_net and n not in self.gnd and n]
                    if not ds or not do: continue
                    sn, so = ds[0][1],do[0][1]
                    sp = {num for num,n,*_ in part_list[a].pads if n == sn}
                    if not sp: continue
                    self.pin[d] = (a,sn,sp)
                    self.group_list[a] = sorted(set(self.group_list[a])|{d})
                    for e in sorted(island['decoupling'],key=lambda e:(e['C'],e['ref'])):
                        if e['net_name'] != so or e['ref'] not in part_list: continue
                        self.pin[e['ref']] = (d,so,{do[0][0]})
                    cout = next((e['ref'] for e in sorted(island['decoupling'],key=lambda e:(e['C'],e['ref']))
                                 if e['net_name'] == so and e['ref'] in part_list),None)
                    if cout: self.boost_loops.append((a,cout,d,sn,so))
        self.of = collections.defaultdict(list)
        for c, (a, _, _) in self.pin.items(): self.of[a].append(c)
        self.series = {}
        for a, i in netclasses['islands'].items():
            groups = collections.defaultdict(list)
            for r in i['members']:
                if r in part_list and netclasses['part_list'][r].get('role') == 'small':
                    groups[netclasses['part_list'][r].get('function')].append(r)
            self.series[a] = [sorted(rs) for rs in groups.values() if len(rs) > 1]

    def loop(self, pads, a, c, n, pins):
        vin = [(x, y) for num, net, x, y, _ in pads[a] if num in pins and net == n]
        pg = [(x, y) for _, net, x, y, _ in pads[a] if net in self.gnd]
        cv = [(x, y) for _, net, x, y, _ in pads[c] if net == n]
        cg = [(x, y) for _, net, x, y, _ in pads[c] if net in self.gnd]
        if not all((vin, pg, cv, cg)): return None

        v, u = min(((v, u) for v in vin for u in cv), key=lambda q: math.dist(*q))
        g, h = min(((g, h) for g in pg for h in cg), key=lambda q: math.dist(*q))
        return hull([v, u, g, h]), math.dist(v, u) + math.dist(g, h)
    def cost(self, ch, pads, loop_w=2.0, compact=0.4, ordering=0.04):
        c = 0.0
        for a, r, n, pins in self.loops:
            if not ({a, r} & ch.keys()) or a not in pads or r not in pads: continue
            z = self.loop(pads, a, r, n, pins)
            if z: c += loop_w * z[0] + 2.0 * z[1]
        for a,r,d,sn,so in self.boost_loops:
            if not ({a,r,d} & ch.keys()) or not all(q in pads for q in (a,r,d)): continue
            z = self.boost_loop(pads,a,r,d,sn,so)
            if z: c += loop_w*z[0]+2.0*z[1]
        for a, rs in self.group_list.items():
            if not (set(rs) & ch.keys()): continue

            pts = [(x, y) for r in rs if r in pads for _, n, x, y, _ in pads[r] if n not in self.gnd]
            if pts: c += compact * ((max(x for x, _ in pts)-min(x for x, _ in pts)) +
                                    (max(y for _, y in pts)-min(y for _, y in pts)))
        for groups in self.series.values():
            for rs in groups:
                for r, q in zip(rs, rs[1:]):
                    if not ({r,q} & ch.keys()) or r not in pads or q not in pads: continue
                    p = [(sum(z[2] for z in pads[t])/len(pads[t]), sum(z[3] for z in pads[t])/len(pads[t]))
                         for t in (r,q) if pads[t]]
                    if len(p) == 2: c += ordering * min(abs(p[0][0]-p[1][0]), abs(p[0][1]-p[1][1]))
                    if len(pads[r]) == len(pads[q]) == 2:
                        pp = [sorted(pads[t],key=lambda p:p[0]) for t in (r,q)]
                        v = [(p[1][2]-p[0][2],p[1][3]-p[0][3]) for p in pp]
                        norm=math.hypot(*v[0])*math.hypot(*v[1])
                        if norm: c += ordering*(1-(v[0][0]*v[1][0]+v[0][1]*v[1][1])/norm)
        return c

    def boost_loop(self,pads,a,c,d,sn,so):
        def pts(r,n): return [(x,y) for _,net,x,y,_ in pads[r] if net in n]
        sets = [pts(a,{sn}),pts(d,{sn}),pts(d,{so}),pts(c,{so}),pts(c,self.gnd),pts(a,self.gnd)]
        if not all(sets): return None
        pairs=[min(((p,q) for p in sets[i] for q in sets[i+1]),key=lambda z:math.dist(*z)) for i in (0,2,4)]
        return hull([p for pair in pairs for p in pair]),sum(math.dist(*pair) for pair in pairs)

    def measure(self, layer_name):
        pads = {r: self.t[r].pads_abs(*s) for r, s in layer_name.items()}
        geo = {}
        for (r,num), g in self.polys.items():
            x,y,th,side = layer_name[r]
            if side != self.t[r].side0: g = affinity.scale(g,-1,1,origin=(0,0))
            geo[r,num] = affinity.translate(affinity.rotate(g,th,origin=(0,0)),x,y)
        per_pin = []
        for a, i in sorted(self.k['islands'].items()):
            for n in sorted({e['net_name'] for e in i['decoupling'] if not e.get('over_coil')}):
                es = [e for e in i['decoupling'] if e['net_name'] == n and not e.get('over_coil') and e['ref'] in pads]
                for num, net, x, y, sd in pads.get(a, []):
                    if net != n or num not in {p for e in es for p in e['pins']}: continue
                    assigned = [e for e in es if e['ref'] in self.pin and self.pin[e['ref']][0] == a and num in self.pin[e['ref']][2]]
                    pool = assigned or es
                    ds = [(math.hypot(x-u, y-v), e['C'], e['ref'], cs) for e in pool
                          for _, cn, u, v, cs in pads[e['ref']] if cn == n]
                    small = min((e['C'] for e in pool), default=None)
                    ks = [q for q in ds if q[1] == small]
                    q = min(ks) if ks else None
                    edge = min((geo[a,num].distance(geo[e['ref'],cn]) for e in pool if q and e['ref'] == q[2]
                                for cn, net, *_ in pads[e['ref']] if net == n and (e['ref'],cn) in geo), default=None) if (a,num) in geo else None
                    per_pin.append(dict(ref=a, pin=num, net_name=n, smallest_C=small,
                                        spacing_mm=q[0] if q else None, capacitor=q[2] if q else None,
                                        pad_edge_mm=edge, opposite_side=q[3] != sd if q else None, assigned=bool(assigned)))
        loops = [dict(regulator=a, cin=c, net_name=n, category='VIN-PadHull', area_mm2=z[0] if z else None,
                      connection_mm=z[1] if z else None) for a, c, n, p in self.loops
                 if a in pads and c in pads for z in [self.loop(pads, a, c, n, p)]]
        loops += [dict(regulator=a,cout=c,diode=d,net_name=so,category='Boost-SW-Diode-COUT-PGND-PadHull',
                       area_mm2=z[0] if z else None,connection_mm=z[1] if z else None)
                  for a,c,d,sn,so in self.boost_loops if all(r in pads for r in (a,c,d))
                  for z in [self.boost_loop(pads,a,c,d,sn,so)]]
        sw = {}
        for n, k in sorted(self.k['net_list'].items()):
            if k.get('category') != 'sw': continue
            pts = [(x,y) for ps in pads.values() for _, net, x,y,_ in ps if net == n]
            sw[n] = max((math.dist(a,b) for a in pts for b in pts), default=0)
        model = {a for a, *_ in self.loops} | {a for a,*_ in self.boost_loops}
        return dict(decoupling_per_pin=per_pin, hot_loop=loops, switch_node_mm=sw,
                    power_without_loop_model=sorted(a for a,i in self.k['islands'].items() if i.get('category')=='power' and a not in model),
                    spacing_kind='PadCenter-PadCenter', hot_loop_kind='convex hull VIN/PGND/CIN pad centers')
