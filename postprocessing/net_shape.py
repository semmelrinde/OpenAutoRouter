import copy
import math
import json
from pathlib import Path
import time
from collections import Counter, defaultdict

from shapely.geometry import Point, Polygon, box
from shapely.strtree import STRtree
from shapely.ops import unary_union
from analysis.netclasses import net_shape
from common.track_items import arc_geometry, native_arc_representable, primitive_intersections, split_native_contacts
from postprocessing.pulltight_routes import Guard, fillet, length, line, round_track, sample_arc, straighten


def ideal_rules(board, rules):
    full = 'router' in rules
    out = dict(rules['router'] if full else rules)
    hole = (rules.get('raw_data') or {}).get('min_hole_clearance') if full else None
    if hole is None: hole = (board.get('design') or {}).get('hole_spacing')
    if hole is not None:
        if not math.isfinite(float(hole)) or float(hole)<0: raise ValueError('invalid hole spacing')
        out['hole_clr'] = max(float(out.get('hole_clr',0)),float(hole))
    return out


class IdealGuard(Guard):
    def __init__(self, board, routes, rules):
        super().__init__(board,routes,ideal_rules(board,rules))
        self.kelvin={i:t for i,t in enumerate(routes['tracks']) if t.get('kelvin')}
        self.endcaps={}
        for i,t in self.kelvin.items():
            caps=[]
            for e in (t['pts'][0],t['pts'][-1]):
                rr=.3
                for p in board['pads']:
                    if p['net']==t['net'] and p.get('bbox') and p['bbox'][0]-.03<=e[0]<=p['bbox'][2]+.03 and p['bbox'][1]-.03<=e[1]<=p['bbox'][3]+.03:
                        rr=max(rr,max(math.dist(e,q) for q in [(p['bbox'][0],p['bbox'][1]),(p['bbox'][2],p['bbox'][3])])+.3)
                caps.append(Point(e).buffer(rr))
            self.endcaps[i]=unary_union(caps)

    def clear(self,pts,w,layer,net,owner,error=0.):
        if not super().clear(pts,w,layer,net,owner,error): return False
        g=line(pts)
        for idx,n,other,r,clr,contact in self.obs.get(layer,[]):
            if n==net and not contact and g.distance(other)<w/2+r+clr+self.margin+error:
                self.tests['own_hole_declined']+=1;return False
        if owner in self.kelvin:
            check=g.difference(self.endcaps[owner])
            for idx,n,other,r,clr,contact in self.obs.get(layer,[]):
                if idx==owner or n!=net or not contact: continue
                if check.distance(other)<w/2+r+self.clr+self.margin+error:
                    self.tests['kelvin_declined']+=1;return False
        else:
            for idx,t in self.kelvin.items():
                if t['net']!=net or t['layer']!=layer: continue
                check=line(t['pts']).difference(self.endcaps[idx])
                if g.distance(check)<w/2+max(t['w'])/2+self.clr+self.margin+error:
                    self.tests['kelvin_declined']+=1;return False
        return True


def kelvin_end_gate(board, before, after, rules, changed, error=.00025):
    if not any(t.get('kelvin') for t in before['tracks']): return set()
    from analysis.topology_native import _measurement_routes
    measured = _measurement_routes(after)
    guard = IdealGuard(board, measured, rules)
    bad = set()
    changed_nets = {c['net'] for c in changed}
    for i, t in enumerate(measured['tracks']):
        if t['net'] not in changed_nets: continue
        for a, b, w in zip(t['pts'], t['pts'][1:], t['w']):
            if not guard.clear([a, b], w, t['layer'], t['net'], i, error):
                bad.add(t['net']); break
    return bad


def analytic_length(t):
    if 'items' not in t:
        return length(t['pts'])
    total = 0.
    for it in t['items']:
        if it[0] == 'L': total += math.dist(it[1], it[2])
        elif it[0] == 'A':
            _, r, _, sweep = arc_geometry(it); total += r*abs(sweep)
        else: raise ValueError('unknown track item: %s' % it[0])
    return total


def render_cuts(pts, w, cuts, error):
    its, dense, cursor = [], [tuple(pts[0])], tuple(pts[0])
    for i in range(1, len(pts)):
        if i in cuts:
            f = cuts[i]
            if math.dist(cursor, f[0]) > 1e-9:
                its.append(['L', cursor, f[0], w]); dense.append(f[0])
            its.append(['A', f[0], f[1], f[2], w]); dense.extend(sample_arc(f, error)[1:]); cursor = f[2]
        else:
            if math.dist(cursor, pts[i]) > 1e-9:
                its.append(['L', cursor, pts[i], w]); dense.append(tuple(pts[i]))
            cursor = tuple(pts[i])
    return dict(items=its, pts=[list(p) for p in dense], w=[w]*(len(dense)-1))


def paired(a, b, ia, ib, guard, rmax, error, partner_rmax=None):
    if a['layer'] != b['layer'] or len(a['pts']) != len(b['pts']) or len(a['pts']) < 3:
        return None
    if any(abs(w-a['w'][0]) > 1e-9 for w in a['w']) or any(abs(w-b['w'][0]) > 1e-9 for w in b['w']): return None
    P, Q = a['pts'], b['pts']; offsets = []
    for p, pp, q, qq in zip(P, P[1:], Q, Q[1:]):
        lp, lq = math.dist(p, pp), math.dist(q, qq)
        if min(lp, lq) < 1e-6: return None
        u = ((pp[0]-p[0])/lp, (pp[1]-p[1])/lp)
        v = ((qq[0]-q[0])/lq, (qq[1]-q[1])/lq)
        if math.dist(u, v) > 1e-5: return None
        offsets.append(u[0]*(q[1]-p[1])-u[1]*(q[0]-p[0]))
    d = sum(offsets)/len(offsets)
    if max(abs(x-d) for x in offsets) > 1e-5: return None
    wa, wb = a['w'][0], b['w'][0]
    if abs(d) < (wa+wb)/2+guard.clr+2*error: return None
    partner_rmax = rmax if partner_rmax is None else partner_rmax
    ca, cb = {}, {}; guard.ignore_tracks = {ia, ib}
    try:
        contacts = [guard.contacts(t['pts'], t['w'][0], t['layer'], t['net'], i) for t, i in ((a, ia), (b, ib))]
        for j in range(1, len(P)-1):
            u = (P[j][0]-P[j-1][0], P[j][1]-P[j-1][1]); v = (P[j+1][0]-P[j][0], P[j+1][1]-P[j][1])
            turn = 1 if u[0]*v[1]-u[1]*v[0] > 0 else -1

            upper = min(rmax, partner_rmax+turn*d)
            lower = max(.025, .025+turn*d)
            if upper < lower-1e-9:
                guard.tests['pair_radius_incompatible']+=1
                return None
            for k in range(18):
                ra = upper*(.75**k); rb = ra-turn*d
                if ra < lower-1e-9 or min(ra, rb) < .025: continue
                fa = fillet(P[j-1], P[j], P[j+1], ra); fb = fillet(Q[j-1], Q[j], Q[j+1], rb)
                if fa is None or fb is None or math.dist(fa[3], fb[3]) > 1e-5: continue
                if not all(native_arc_representable(['A',f[0],f[1],f[2],w]) for f,w in [(fa,wa),(fb,wb)]):continue
                if any(x in cuts and math.dist(f[0], pts[j]) + math.dist(cuts[x][2], pts[j-1]) >= math.dist(pts[j-1], pts[j])-1e-6
                       for cuts, f, pts in ((ca,fa,P),(cb,fb,Q)) for x in [j-1]): continue
                ca[j], cb[j] = fa, fb
                aa, bb = render_cuts(P, wa, ca, error), render_cuts(Q, wb, cb, error)
                if (guard.clear(aa['pts'], wa, a['layer'], a['net'], ia, error) and
                    guard.clear(bb['pts'], wb, b['layer'], b['net'], ib, error) and
                    guard.retain(aa['pts'], contacts[0], error) and guard.retain(bb['pts'], contacts[1], error) and
                    line(aa['pts']).distance(line(bb['pts'])) >= (wa+wb)/2+guard.clr+2*error): break
                del ca[j]; del cb[j]
        if not ca: return None
        return render_cuts(P, wa, ca, error), render_cuts(Q, wb, cb, error), len(ca)
    finally:
        guard.ignore_tracks = set()


def anchors(routes):
    out = defaultdict(set)
    for t in routes['tracks']:
        for p in (t['pts'][0], t['pts'][-1]): out[(t['net'], t['layer'])].add(tuple(p))
    for v in routes['vias']:
        for L in {t['layer'] for t in routes['tracks']}: out[(v['net'], L)].add((v['x'], v['y']))
    return out


def split_anchors(t, node_points):
    P, W = [tuple(t['pts'][0])], []
    for a, b, w in zip(t['pts'], t['pts'][1:], t['w']):
        g = line([a,b]); ll = math.dist(a,b)
        nodes = sorted((g.project(Point(p)), p) for p in node_points if 1e-6 < math.dist(a,p) < ll-1e-6 and g.distance(Point(p)) < 1e-7)
        for _, p in nodes: P.append(p); W.append(w)
        P.append(tuple(b)); W.append(w)
    starts = [0]+[j for j in range(1,len(W)) if abs(W[j]-W[j-1])>1e-9 or P[j] in node_points]+[len(W)]
    return P, W, starts


def group_errors(routes, classes):
    totals = defaultdict(float)
    for t in routes['tracks']: totals[t['net']] += analytic_length(t)
    groups = defaultdict(set)
    for n, k in classes.get('net_list', {}).items():
        if k.get('length_group'): groups['L:'+str(k['length_group'])].add(n)
        if k.get('pair'): groups['P:'+min(n,k['pair'])+'|'+max(n,k['pair'])].update((n,k['pair']))
    return {g: max(totals[n] for n in ns)-min(totals[n] for n in ns) for g, ns in groups.items()}, groups


def bga_regions(board):
    refs={f['ref'] for f in board.get('fps',[]) if 'BGA' in f.get('fp','').upper() or f.get('lib')=='Package_BGA'}
    regions=[]
    for r in sorted(refs):
        ps=[p for p in board['pads'] if p['ref']==r]
        if ps:
            regions.append((min(p['x'] for p in ps)-1.,min(p['y'] for p in ps)-1.,
                            max(p['x'] for p in ps)+1.,max(p['y'] for p in ps)+1.))
    return regions


def is_fanout(pts,regions):
    return length(pts)<=4. and any(all(x0<=p[0]<=x1 and y0<=p[1]<=y1 for p in pts) for x0,y0,x1,y1 in regions)


def pair_contracts(nets, by_net):
    normalized={n:dict(k) for n,k in nets.items()}
    graph=defaultdict(set)
    for n,k in nets.items():
        partner=k.get('pair')
        if partner:
            graph[n].add(partner);graph[partner].add(n)
    blocked=set();conflicts=[];seen=set()
    for n in sorted(graph):
        if n in seen: continue
        members=set();pending=[n]
        while pending:
            member=pending.pop()
            if member in members: continue
            members.add(member);pending.extend(graph[member]-members)
        seen.update(members)
        reason=None
        if len(members)!=2 or any(len(graph[m])!=1 for m in members):
            reason='incompatible pair membership'
        elif any(not by_net.get(m) for m in members):
            reason='Partner copper missing'
        else:
            a,b=sorted(members)
            normalized.setdefault(a,{})['pair']=b
            normalized.setdefault(b,{})['pair']=a
            if net_shape(a,normalized[a])['shaping'] != net_shape(b,normalized[b])['shaping']:
                reason='incompatible shape/pair profiles'
        if reason:
            blocked.update(members);conflicts.append(dict(net_list=sorted(members),reason=reason))
    return normalized,blocked,conflicts


def optimize_ideal(board, routes, rules, classes, error=.00025, connectivity=None):
    t0 = time.process_time(); out = copy.deepcopy(routes); guard = IdealGuard(board, out, rules)
    stat = Counter(); changed = []; done = set(); nodes = anchors(routes); nets = classes.get('net_list', {})
    by_net = defaultdict(list)
    for i, t in enumerate(routes['tracks']): by_net[t['net']].append(i)
    nets,blocked,profile_conflicts=pair_contracts(nets,by_net)
    matched = bool(routes.get('length_matching')); regions=bga_regions(board)


    fixed_nets=set()
    fix_file=(routes.get('options') or {}).get('fix')
    if fix_file:
        try: fixed_nets={t['net'] for t in json.loads(Path(fix_file).read_text())['tracks']}
        except (OSError,ValueError,KeyError): fixed_nets=set(by_net);stat['fix_source_missing']=1
    for ti, t in enumerate(out['tracks']):
        if ti in done: continue
        P, W = t['pts'], t['w']; net, layer = t['net'], t['layer']; k = nets.get(net, {})
        if len(P)<2 or len(W)!=len(P)-1: raise ValueError('invalid pts/w lengths')
        profile = net_shape(net, k)
        stat['profile_'+profile['shaping']] += 1
        if net in blocked:
            stat['pair_profile_guarded']+=1;continue
        if net in fixed_nets or t.get('fix') or t.get('current_web') or 'items' in t or t.get('meander') or matched and (k.get('pair') or k.get('length_group')):
            stat['guarded']+=1; continue
        if k.get('pair'):
            partner = k['pair']; candidates = [j for j in by_net[partner] if j not in done and j != ti and out['tracks'][j]['layer']==layer]
            result = None
            partner_profile = net_shape(partner,nets.get(partner,{}))
            if profile['shaping'] != partner_profile['shaping'] or nets.get(partner,{}).get('pair',net) != net:
                stat['pair_profile_guarded']+=1
                profile_conflicts.append(dict(net_list=sorted((net,partner)),reason='incompatible shape/pair profiles'))
                continue
            for tj in candidates:
                b = out['tracks'][tj]
                if partner in fixed_nets or b.get('fix') or b.get('current_web') or b.get('meander') or 'items' in b: continue
                radius = min(.5,profile['shape_rmax_mm']) if is_fanout(t['pts'],regions) else profile['shape_rmax_mm']
                partner_radius = min(.5,partner_profile['shape_rmax_mm']) if is_fanout(b['pts'],regions) else partner_profile['shape_rmax_mm']
                radius_failures=guard.tests['pair_radius_incompatible']
                result = paired(t,b,ti,tj,guard,radius,error,partner_rmax=partner_radius)
                if guard.tests['pair_radius_incompatible']>radius_failures:
                    stat['pair_profile_guarded']+=1
                    profile_conflicts.append(dict(net_list=sorted((net,partner)),reason='Radius limits do not allow concentric arcs',
                                                  radius_max_mm={net:radius,partner:partner_radius}))
                if result:
                    aa, bb, arcs = result
                    for idx, geo in ((ti,aa),(tj,bb)):
                        out['tracks'][idx].update(geo); done.add(idx)
                        guard.add(geo['pts'], geo['w'][0],layer,out['tracks'][idx]['net'],error)
                        changed.append(dict(track=idx,net=out['tracks'][idx]['net'],arcs=arcs,vertices_removed=0))
                    stat['pairs_shaped']+=1; break
            if not result: stat['pair_topology_guarded']+=1
            continue
        PP, WW, starts = split_anchors(t,nodes[(net,layer)])
        allpts = [PP[0]]; allwidth=[]; allitems=[]; removed=arcs_total=0
        stat['fixed_width_anchor'] += sum(abs(W[j]-W[j-1])>1e-9 for j in range(1,len(W)))
        for begin,end in zip(starts,starts[1:]):
            part=PP[begin:end+1]; w=WW[begin]; contacts=guard.contacts(part,w,layer,net,ti)
            fanout=is_fanout(part,regions)
            if fanout: stat['bga_partial_chains_45r']+=1
            pull=profile['shaping']=='organic' and not k.get('length_group') and not fanout
            Q,n=straighten(part,w,layer,net,ti,guard,contacts) if pull else (part,0)
            its,dense,arcs=round_track(Q,w,layer,net,ti,guard,contacts,min(.5,profile['shape_rmax_mm']) if fanout else profile['shape_rmax_mm'],error)
            test=dict(items=its)
            if analytic_length(test)>length(part)+1e-8 or not guard.retain(dense,contacts,error if arcs else 0):
                stat['end_gate_declined']+=1; its=[['L',a,b,w] for a,b in zip(part,part[1:])]; dense=part; n=arcs=0
            allpts.extend(dense[1:]); allwidth.extend([w]*(len(dense)-1)); allitems.extend(its)
            removed+=n; arcs_total+=arcs
            if n or arcs: guard.add(dense,w,layer,net,error if arcs else 0)
        if removed or arcs_total:
            t.update(pts=[list(q) for q in allpts],w=allwidth,items=allitems)
            changed.append(dict(track=ti,net=net,arcs=arcs_total,vertices_removed=removed))
            if k.get('category') in ('current_val','supply','sw'): stat['current_tracks_shaped']+=1
    rollback=set()
    rollback.update(kelvin_end_gate(board, routes, out, rules, changed, error))


    e0,groups=group_errors(routes,classes); e1,_=group_errors(out,classes)
    for g, old in e0.items():
        if e1[g]>old+1e-7: rollback.update(groups[g]); stat['length_group_declined']+=1
    if connectivity:
        old=set(connectivity(board,routes,set(routes.get('area_nets',[]))))
        rollback.update(set(connectivity(board,out,set(routes.get('area_nets',[]))))-old)


    while True:
        extended = set(rollback)
        for members in groups.values():
            if rollback.intersection(members): extended.update(members)
        if extended == rollback: break
        rollback = extended
    out['tracks']=[copy.deepcopy(routes['tracks'][i]) if t['net'] in rollback else t for i,t in enumerate(out['tracks'])]
    changed=[c for c in changed if c['net'] not in rollback]
    final_errors,_ = group_errors(out,classes)
    if any(final_errors[g] > value+1e-7 for g,value in e0.items()):
        raise RuntimeError('Length gate violated after rollback')
    if connectivity and set(connectivity(board,out,set(routes.get('area_nets',[]))))-old:
        raise RuntimeError('Connection gate violated after rollback')
    stat.update(tracks_changed=len(changed),arcs=sum(c['arcs'] for c in changed),vertices_removed=sum(c['vertices_removed'] for c in changed))
    out['organic_post_run']=dict(version=3,style='ideal',cpu_s=time.process_time()-t0,chord_error_mm=error,
        spacing_reserve_mm=guard.margin,statistics=dict(stat),checks=dict(guard.tests),changes=changed,
        profile_conflicts=profile_conflicts,length_differences_before=e0,length_differences_after=final_errors,
        rollback_nets=sorted(rollback),connection_gate_checked=connectivity is not None,native_drc_required=True,
        order='Shape -> length matching',width_profile='original widths and width anchors (no tapering)')
    return out


def rule_gate(board, before, after, classes, gnd_layers=None, *, topology_version=None,
              si_score=False, si_measure=False):
    from analysis import rule_metrics as RK
    options=dict(gnd_layers=gnd_layers,si_score=si_score,si_measure=si_measure)
    if topology_version is not None: options['topology_version']=topology_version
    a=RK.measure(board,before,classes,**options); b=RK.measure(board,after,classes,**options)
    contract=RK.assumption(a,b); comparison=contract['comparison']
    missing=comparison['absent']
    worse=[k for k,t in comparison['per'].items() if t and t['gate_check'] and t['verdict']>0]
    fa,fb=native_shape_metrics(before,board),native_shape_metrics(after,board)
    for key in ('sharp_corners','sharp_branches','sharp_corners_free','sharp_branches_free'):
        if fb[key]>fa[key]: worse.append(key)
    return dict(taken=contract['not_worse'] and not worse,absent=missing,worse=worse,
                rule_contract=contract,before=a,after=b)


def native_shape_metrics(routes, board=None):
    rays=defaultdict(list); radius=[]; node_points=anchors(routes)
    primitives=defaultdict(list)
    for t in routes['tracks']:
        key=t['net'],t['layer']
        its=t.get('items') or [['L',a,b,w] for a,b,w in zip(t['pts'],t['pts'][1:],t['w'])]
        primitives[key].extend(its)
        for it in its:
            node_points[key].update((tuple(it[1]),tuple(it[2] if it[0]=='L' else it[3])))
            if it[0]=='A': radius.append(arc_geometry(it)[1])
    for key,items in primitives.items():
        bounds=[]
        for it in items:
            if it[0]=='L':
                a,b=it[1:3]; bounds.append(box(min(a[0],b[0]),min(a[1],b[1]),max(a[0],b[0]),max(a[1],b[1])).buffer(1e-8))
            else:
                c,r,_,_=arc_geometry(it); bounds.append(box(c[0]-r,c[1]-r,c[0]+r,c[1]+r))
        tree=STRtree(bounds)
        for i,g in enumerate(bounds):
            for j in tree.query(g):
                if j>i: node_points[key].update(primitive_intersections(items[i],items[int(j)]))
        for original in items:
            if original[0]=='A':
                center,_,_,original_sweep=arc_geometry(original)
                sign=1 if original_sweep>0 else -1
            for it in split_native_contacts(original,node_points[key]):
                a=it[1]; b=it[2] if it[0]=='L' else it[3]
                if it[0]=='L':
                    if math.dist(a,b)<1e-8: continue
                    start=math.atan2(b[1]-a[1],b[0]-a[0]); end=start+math.pi
                else:


                    start=math.atan2(a[1]-center[1],a[0]-center[0])+sign*math.pi/2
                    end=math.atan2(b[1]-center[1],b[0]-center[0])-sign*math.pi/2
                for p,v in ((a,start),(b,end)):
                    rays[(*key,round(p[0],6),round(p[1],6))].append(math.degrees(v)%360)
    pads=defaultdict(list)
    if board:
        for p in board['pads']:
            for L,polys in p['polys'].items():
                pads[(p['net'],L)].extend(Polygon(q) for q in polys)
    kn=acute=branch=acute_pad=branch_pad=0
    for key,values in rays.items():
        vals=[]
        for v in sorted(values):
            if not vals or v-vals[-1]>1e-4: vals.append(v)
        if len(vals)>1 and vals[0]+360-vals[-1]<1e-4: vals.pop()
        in_pad=any(g.covers(Point(key[2],key[3])) for g in pads[(key[0],key[1])])
        if len(vals)==2:
            a=min((vals[1]-vals[0])%360,(vals[0]-vals[1])%360)
            kn+=a<179; acute+=a<90-1e-4;acute_pad+=(a<90-1e-4 and in_pad)
        elif len(vals)>=3:
            small=min((vals[(i+1)%len(vals)]-vals[i])%360 for i in range(len(vals)))<60-1e-4
            branch+=small;branch_pad+=(small and in_pad)
    return dict(bends_kicad_native=kn,sharp_corners=acute,sharp_branches=branch,
                sharp_corners_free=acute-acute_pad,sharp_corners_pad=acute_pad,sharp_branches_free=branch-branch_pad,
                sharp_branches_pad=branch_pad,with_board=board is not None,arcs=len(radius),
                radius_min_mm=min(radius,default=None),radius_max_mm=max(radius,default=None),
                length_analytical_mm=sum(analytic_length(t) for t in routes['tracks']))
