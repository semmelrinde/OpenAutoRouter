import copy
import json
import math
from pathlib import Path
import shutil
import sys
import time
from collections import Counter
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.track_items import native_arc_representable

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
from shapely.strtree import STRtree


def line(points):
    return LineString(points) if len(set(map(tuple, points))) > 1 else Point(points[0])


def length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def fillet(a, b, c, radius):
    la, lc = math.dist(a, b), math.dist(b, c)
    if min(la, lc) < 1e-8 or radius <= 0:
        return None
    u = ((a[0]-b[0])/la, (a[1]-b[1])/la)
    v = ((c[0]-b[0])/lc, (c[1]-b[1])/lc)
    angle = math.acos(max(-1., min(1., u[0]*v[0]+u[1]*v[1])))
    if angle < 1e-5 or angle > math.pi-1e-5:
        return None
    d = radius/math.tan(angle/2)
    if d >= min(la, lc):
        return None
    bi = (u[0]+v[0], u[1]+v[1]); bl = math.hypot(*bi)
    bi = (bi[0]/bl, bi[1]/bl)
    center = (b[0]+bi[0]*radius/math.sin(angle/2), b[1]+bi[1]*radius/math.sin(angle/2))
    p = (b[0]+u[0]*d, b[1]+u[1]*d)
    q = (b[0]+v[0]*d, b[1]+v[1]*d)
    mid = (center[0]-bi[0]*radius, center[1]-bi[1]*radius)
    start = math.atan2(p[1]-center[1], p[0]-center[0])
    end = math.atan2(q[1]-center[1], q[0]-center[0])
    sweep = (end-start+math.pi) % (2*math.pi)-math.pi
    return p, mid, q, center, radius, start, sweep


def sample_arc(f, error=0.00025):
    p, mid, q, center, radius, start, sweep = f
    step = 2*math.acos(max(-1., min(1., 1-error/radius)))
    n = max(2, math.ceil(abs(sweep)/max(step, 1e-8)))
    pts = [(center[0]+radius*math.cos(start+sweep*k/n),
            center[1]+radius*math.sin(start+sweep*k/n)) for k in range(n+1)]
    pts[0], pts[-1] = p, q
    return pts


class Guard:
    def __init__(self, board, routes, rules, margin=0.01):
        self.rules = rules; self.margin = margin
        self.obs = {}; self.new = {}; self.tests = Counter()
        self.layers = sorted({'F', 'B'} | {t['layer'] for t in routes['tracks']})
        self.clr = float(rules.get('drc', 0.09))
        self.padclr = float(rules.get('drcpad', self.clr))
        self.edge = float(rules.get('edge', 0.2))
        self.hole = float(rules.get('hole_clr', 0.0))
        self.allowed = unary_union([Polygon(p) for p in board.get('outline', []) if len(p) >= 3])
        vd, vb = map(float, str(rules.get('via', '0.45/0.2')).split('/'))
        def add(L, owner, net, g, radius, clr, contact=True):
            self.obs.setdefault(L, []).append((owner, net, g, radius, clr, contact))
        for ti, t in enumerate(routes['tracks']):
            for a, b, w in zip(t['pts'], t['pts'][1:], t['w']):
                add(t['layer'], ti, t['net'], line([a, b]), w/2, self.clr)
        for vi, v in enumerate(routes['vias']):
            radius = float(v.get('d', vd))/2
            for L in self.layers:
                add(L, ('via', vi), v['net'], Point(v['x'], v['y']), radius, self.clr)
                add(L, ('viahole', vi), v['net'], Point(v['x'], v['y']), float(v.get('drill', vb))/2, self.hole, False)
        for pi, p in enumerate(board['pads']):
            net = p['net'] if p['net'] and not p['net'].startswith('unconnected') else ('pad', pi)
            for L, polys in p['polys'].items():
                for poly in polys:

                    mask = float((p.get('mask') or {}).get(L, 0))
                    add(L, ('pad', pi), net, Polygon(poly), 0., max(self.padclr, mask), True)
            if p.get('drill', 0) > 0:
                for L in self.layers:
                    add(L, ('hole', pi), net, Point(p['x'], p['y']), p['drill']/2, self.hole, False)
        for ai, k in enumerate(board.get('keepouts', [])):
            if not (k.get('prohibitions') or {}).get('track_list', True): continue
            for poly in k.get('polys', []):
                for L in [L_ for L_ in self.layers if not k.get('cu_layers') or L_ in k['cu_layers']]:
                    add(L, ('keepout', ai), ('keepout', ai), Polygon(poly), 0., 0., False)
        self.trees = {L: STRtree([o[2] for o in obs]) for L, obs in self.obs.items()}
        self.maxreach = {L: max((o[3]+o[4] for o in obs), default=0.) for L, obs in self.obs.items()}

    def clear(self, pts, w, layer, net, owner, error=0.):
        self.tests['spacing'] += 1
        g = line(pts); rr = w/2 + self.margin + error
        if self.allowed.is_empty or not self.allowed.covers(g) or g.distance(self.allowed.boundary) < rr+self.edge:
            self.tests['edge_declined'] += 1
            return False
        reach = rr+self.maxreach.get(layer, 0.)
        if layer in self.trees:
            for i in self.trees[layer].query(g.buffer(reach)):
                idx, n, other, r, clr, contact = self.obs[layer][int(i)]
                if idx == owner or n == net or idx in getattr(self, 'ignore_tracks', set()):
                    continue
                if g.distance(other) < rr+r+clr:
                    self.tests['copper_declined'] += 1
                    return False
        for n, other, r, err in self.new.get(layer, []):
            if n != net and g.distance(other) < rr+r+self.clr+err:
                self.tests['new_copper_declined'] += 1
                return False
        return True

    def contacts(self, pts, w, layer, net, owner):
        g = line(pts); found = []
        reach = w/2+self.maxreach.get(layer, 0.)+0.001
        for i in self.trees[layer].query(g.buffer(reach)) if layer in self.trees else []:
            idx, n, other, r, clr, contact = self.obs[layer][int(i)]
            if contact and idx != owner and n == net and g.distance(other) <= w/2+r+1e-8:
                found.append((other, w/2+r, g.distance(other)))
        for n, other, r, err in self.new.get(layer, []):
            if n == net and g.distance(other) <= w/2+r+err+1e-8:
                found.append((other, w/2+r-err, g.distance(other)))
        return found

    def retain(self, pts, contacts, error=0.):
        g = line(pts)
        for other, reach, old_dist in contacts:


            limit = min(reach-1e-6, max(0., old_dist)+0.001)
            if old_dist >= reach-1e-6:
                limit = old_dist+1e-9
            if g.distance(other)+error > limit:
                self.tests['contact_declined'] += 1
                return False
        return True

    def add(self, pts, w, layer, net, error=0.):
        self.new.setdefault(layer, []).append((net, line(pts), w/2, error))


def straighten(pts, w, layer, net, owner, guard, contacts):
    pts = list(map(tuple, pts)); changes = 0
    while len(pts) > 2:
        prefix = [0.]
        for a, b in zip(pts, pts[1:]):
            prefix.append(prefix[-1]+math.dist(a, b))
        proposals = sorted(((prefix[j]-prefix[i]-math.dist(pts[i], pts[j]), i, j)
                            for i in range(len(pts)-2) for j in range(i+2, len(pts))), reverse=True)
        accepted = False
        for gain, i, j in proposals:

            if gain < -1e-8:
                continue
            if not guard.clear([pts[i], pts[j]], w, layer, net, owner):
                continue
            trial = pts[:i+1]+pts[j:]
            if not guard.retain(trial, contacts):
                continue
            pts = trial; changes += j-i-1; accepted = True
            break
        if not accepted:
            break
    return pts, changes


def round_track(pts, w, layer, net, owner, guard, contacts, rmax, error):
    cuts = {}; dense = list(pts); count = 0
    def build():
        its = []; points = [pts[0]]; cursor = pts[0]
        for i in range(1, len(pts)):
            if i in cuts:
                f = cuts[i]; ap = sample_arc(f, error)
                if math.dist(cursor, f[0]) > 1e-9:
                    its.append(['L', cursor, f[0], w]); points.append(f[0])
                its.append(['A', f[0], f[1], f[2], w]); points.extend(ap[1:]); cursor = f[2]
            else:
                if math.dist(cursor, pts[i]) > 1e-9:
                    its.append(['L', cursor, pts[i], w]); points.append(pts[i])
                cursor = pts[i]
        return its, points
    for i in range(1, len(pts)-1):
        la, lc = math.dist(pts[i-1], pts[i]), math.dist(pts[i], pts[i+1])
        if min(la, lc) < 1e-6:
            continue
        u = ((pts[i-1][0]-pts[i][0])/la, (pts[i-1][1]-pts[i][1])/la)
        v = ((pts[i+1][0]-pts[i][0])/lc, (pts[i+1][1]-pts[i][1])/lc)
        ang = math.acos(max(-1., min(1., u[0]*v[0]+u[1]*v[1])))
        if ang < 1e-5 or ang > math.pi-1e-5:
            continue
        cap = min(rmax, 0.45*min(la, lc)*math.tan(ang/2))


        for k in range(14):
            radius = cap*(0.65**k)
            if radius < 0.025:
                break
            f = fillet(pts[i-1], pts[i], pts[i+1], radius)
            if f is None or not native_arc_representable(['A', f[0], f[1], f[2], w]):
                continue
            if not guard.clear(sample_arc(f, error), w, layer, net, owner, error):
                continue
            cuts[i] = f; its, trial = build()
            if guard.retain(trial, contacts, error):
                dense = trial; count += 1
                break
            del cuts[i]
    its, dense = build()
    return its, dense, count


def receiver_end_gate(board, before, after, classes):
    from analysis.length_impedance import length_list, receiver_protection, receiver_netclasses
    K = receiver_netclasses(classes or dict(net_list={}))
    good = receiver_protection(length_list(board, before, K), length_list(board, after, K))
    info = after['organic_post_run']
    info['receiver_gate'] = good
    if good: return after
    out = copy.deepcopy(before)
    info['changes'] = []
    info['rollback_nets'] = sorted({t['net'] for t in before['tracks']})
    for key in ('tracks_changed', 'arcs', 'vertices_removed'): info['statistics'][key] = 0
    info['statistics']['receiver_gate_declined'] = 1
    out['organic_post_run'] = info
    return out


def optimize(board, routes, rules, classes=None, pull=True, rmax=3., error=0.00025,
             connectivity=None, ideal=False, per_net_kind=False):
    if ideal:
        from postprocessing.net_shape import optimize_ideal
        return receiver_end_gate(board, routes, optimize_ideal(board, routes, rules, classes or {}, error, connectivity), classes)
    t0 = time.process_time(); out = copy.deepcopy(routes)
    guard = Guard(board, routes, rules)
    if any(t.get('kelvin') for t in routes['tracks']):


        from postprocessing.net_shape import IdealGuard
        guard = IdealGuard(board, routes, rules)
    stat = Counter(); changed = []
    power = {n for n, k in (classes or {}).get('net_list', {}).items()
             if k.get('category') in ('supply', 'current_val', 'sw')}
    for ti, t in enumerate(out['tracks']):
        P, W = t['pts'], t['w']
        if len(P) < 2 or len(W) != len(P)-1:
            raise ValueError('invalid pts/w lengths')
        if t['net'] in power or t.get('current_web') or t.get('fix') or 'items' in t:
            stat['guarded'] += 1; continue
        layer, net = t['layer'], t['net']
        from common.net_style import organic
        net_pull = pull if not per_net_kind else organic(net, classes)
        net_radius = rmax if not per_net_kind else (3. if net_pull else .5)

        starts=[0]+[j for j in range(1,len(W)) if abs(W[j]-W[j-1])>1e-9]+[len(W)]
        allpts=[tuple(P[0])]; allwidth=[]; allitems=[]; removed=arcs_total=0
        stat['fixed_width_anchor'] += len(starts)-2
        for begin,end in zip(starts,starts[1:]):
            part=list(map(tuple,P[begin:end+1]));w=W[begin]
            contacts=guard.contacts(part,w,layer,net,ti)
            Q,n=straighten(part,w,layer,net,ti,guard,contacts) if net_pull else (part,0)
            its,dense,arcs=round_track(Q,w,layer,net,ti,guard,contacts,net_radius,error) if net_radius>0 else (
                [['L',a,b,w] for a,b in zip(Q,Q[1:])],Q,0)
            if length(dense)>length(part)+1e-8 or not guard.retain(dense,contacts,error if arcs else 0.):
                stat['end_gate_declined']+=1
                its=[['L',a,b,w] for a,b in zip(part,part[1:])];dense=part;n=arcs=0
            allpts.extend(dense[1:]);allwidth.extend([w]*(len(dense)-1));allitems.extend(its)
            removed+=n;arcs_total+=arcs
            if n or arcs:
                guard.add(dense,w,layer,net,error if arcs else 0.)
        if not removed and not arcs_total:
            continue
        t['pts']=[list(q) for q in allpts];t['w']=allwidth;t['items']=allitems
        stat['vertices_removed']+=removed;stat['arcs']+=arcs_total;stat['tracks_changed']+=1
        changed.append(dict(track=ti,net=net,layer=layer,before_mm=length(P),after_chords_mm=length(allpts),
                            arcs=arcs_total,vertices_removed=removed))

    from postprocessing.net_shape import kelvin_end_gate
    rollback=sorted(kelvin_end_gate(board, routes, out, rules, changed, error))
    if rollback:
        out['tracks']=[copy.deepcopy(routes['tracks'][i]) if t['net'] in rollback else t for i,t in enumerate(out['tracks'])]
        changed=[c for c in changed if c['net'] not in rollback]
        stat['kelvin_end_gate_declined']=len(rollback)
        stat['tracks_changed']=len(changed)
        stat['arcs']=sum(c['arcs'] for c in changed)
        stat['vertices_removed']=sum(c['vertices_removed'] for c in changed)
    if connectivity is not None:
        old_bad=set(connectivity(board,routes,set(routes.get('area_nets',[]))))
        new_bad=set(connectivity(board,out,set(routes.get('area_nets',[]))))
        rollback=sorted(set(rollback) | (new_bad-old_bad))
        if rollback:
            out['tracks']=[copy.deepcopy(routes['tracks'][i]) if t['net'] in rollback else t for i,t in enumerate(out['tracks'])]
            changed=[c for c in changed if c['net'] not in rollback]
            stat['tracks_changed']=len(changed)
            stat['arcs']=sum(c['arcs'] for c in changed)
            stat['vertices_removed']=sum(c['vertices_removed'] for c in changed)
            final_bad=set(connectivity(board,out,set(routes.get('area_nets',[]))))
            if final_bad-old_bad:
                raise RuntimeError('Connection gate violated after rollback')
    out['organic_post_run'] = dict(version=2, cpu_s=time.process_time()-t0, pull=pull, rmax_mm=rmax,
        chord_error_mm=error, spacing_reserve_mm=guard.margin, statistics=dict(stat), checks=dict(guard.tests),
        changes=changed, rollback_nets=rollback, connection_gate_checked=connectivity is not None,
        native_drc_required=True)
    return receiver_end_gate(board, routes, out, classes)


def main():
    bj, rj, out = sys.argv[1:4]
    args = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
    if args.get('off') == '1':
        shutil.copyfile(rj, out)
        return
    B = json.loads(Path(bj).read_text()); R = json.loads(Path(rj).read_text())
    declared = json.loads(Path(args['design_rules']).read_text())
    rules = declared if args.get('style') == 'ideal' else declared['router']
    classes = json.loads(Path(args['netclasses']).read_text()) if args.get('netclasses') else None
    root=Path(args['basis']) if args.get('basis') else Path(__file__).resolve().parents[1]
    sys.path.insert(0,str(root))
    from analysis.evaluation import not_connected
    result = optimize(B, R, rules, classes, pull=args.get('pull', '1') != '0',
                      rmax=float(args.get('rmax', '3')), connectivity=not_connected, ideal=args.get('style') == 'ideal', per_net_kind=args.get('style') == 'per_net_kind')
    if args.get('style') == 'ideal' and args.get('rule_gate', '1') != '0':
        from postprocessing.net_shape import rule_gate
        gate = rule_gate(B, R, result, classes or {}, topology_version=args.get('topology_version'),
                         si_score=args.get('si_score') == '1', si_measure=args.get('si_measure') == '1')
        result['organic_post_run']['rule_gate'] = {k: v for k, v in gate.items() if k not in ('before', 'after')}
        if not gate['taken']:
            result['tracks'] = R['tracks']
            result['organic_post_run']['statistics']['tracks_changed'] = 0
            result['organic_post_run']['statistics']['arcs'] = 0
            result['organic_post_run']['statistics']['vertices_removed'] = 0
            result['organic_post_run']['changes'] = []
    if args.get('style') == 'ideal':
        from postprocessing.net_shape import native_shape_metrics
        result['organic_post_run']['shape_kicad_native'] = native_shape_metrics(result,B)
        f = result['organic_post_run']['shape_kicad_native']
        result['organic_post_run']['acute_angle_fulfilled'] = f['sharp_corners'] == f['sharp_branches'] == 0
    Path(out).write_text(json.dumps(result, ensure_ascii=False))
    print(json.dumps(result['organic_post_run'], ensure_ascii=False))


if __name__ == '__main__':
    main()
