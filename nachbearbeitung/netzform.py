""
import copy
import math
import json
from pathlib import Path
import time
from collections import Counter, defaultdict

from shapely.geometry import Point, Polygon, box
from shapely.strtree import STRtree
from shapely.ops import unary_union
from analyse.klassen import netzform
from gemeinsam.bahnitems import arc_geometry, native_arc_representable, primitive_intersections, split_native_contacts
from nachbearbeitung.pulltight_routes import Guard, fillet, length, line, round_track, sample_arc, straighten


def ideal_rules(board, rules):
    ""
    full = 'router' in rules
    out = dict(rules['router'] if full else rules)
    hole = (rules.get('roh') or {}).get('min_hole_clearance') if full else None
    if hole is None: hole = (board.get('design') or {}).get('loch_abstand')
    if hole is not None:
        if not math.isfinite(float(hole)) or float(hole)<0: raise ValueError('ungueltiger Lochabstand')
        out['loch_clr'] = max(float(out.get('loch_clr',0)),float(hole))
    return out


class IdealGuard(Guard):
    ""
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
                self.tests['eigenes_loch_abgelehnt']+=1;return False
        if owner in self.kelvin:
            check=g.difference(self.endcaps[owner])
            for idx,n,other,r,clr,contact in self.obs.get(layer,[]):
                if idx==owner or n!=net or not contact: continue
                if check.distance(other)<w/2+r+self.clr+self.margin+error:
                    self.tests['kelvin_abgelehnt']+=1;return False
        else:
            for idx,t in self.kelvin.items():
                if t['net']!=net or t['layer']!=layer: continue
                check=line(t['pts']).difference(self.endcaps[idx])
                if g.distance(check)<w/2+max(t['w'])/2+self.clr+self.margin+error:
                    self.tests['kelvin_abgelehnt']+=1;return False
        return True


def kelvin_endtor(board, before, after, rules, changed, error=.00025):
    ""
    if not any(t.get('kelvin') for t in before['tracks']): return set()
    from analyse.topologie_native import _measurement_routes
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
        else: raise ValueError('unbekanntes Bahnitem: %s' % it[0])
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
    ""
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
                guard.tests['paar_radius_unvereinbar']+=1
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
    ""
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
    for n, k in classes.get('netze', {}).items():
        if k.get('laengengruppe'): groups['L:'+str(k['laengengruppe'])].add(n)
        if k.get('paar'): groups['P:'+min(n,k['paar'])+'|'+max(n,k['paar'])].update((n,k['paar']))
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


def pair_contracts(nets, bynet):
    ""
    normalized={n:dict(k) for n,k in nets.items()}
    graph=defaultdict(set)
    for n,k in nets.items():
        partner=k.get('paar')
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
            reason='unvereinbare Paarzugehoerigkeit'
        elif any(not bynet.get(m) for m in members):
            reason='Partnerkupfer fehlt'
        else:
            a,b=sorted(members)
            normalized.setdefault(a,{})['paar']=b
            normalized.setdefault(b,{})['paar']=a
            if netzform(a,normalized[a])['form'] != netzform(b,normalized[b])['form']:
                reason='unvereinbare Form-/Paarprofile'
        if reason:
            blocked.update(members);conflicts.append(dict(netze=sorted(members),grund=reason))
    return normalized,blocked,conflicts


def optimize_ideal(board, routes, rules, classes, error=.00025, connectivity=None):
    t0 = time.process_time(); out = copy.deepcopy(routes); guard = IdealGuard(board, out, rules)
    stat = Counter(); changed = []; done = set(); nodes = anchors(routes); nets = classes.get('netze', {})
    bynet = defaultdict(list)
    for i, t in enumerate(routes['tracks']): bynet[t['net']].append(i)
    nets,blocked,profile_conflicts=pair_contracts(nets,bynet)
    matched = bool(routes.get('laengenabgleich')); regions=bga_regions(board)


    fixed_nets=set()
    fixfile=(routes.get('optionen') or {}).get('fix')
    if fixfile:
        try: fixed_nets={t['net'] for t in json.loads(Path(fixfile).read_text())['tracks']}
        except (OSError,ValueError,KeyError): fixed_nets=set(bynet);stat['fixquelle_fehlt']=1
    for ti, t in enumerate(out['tracks']):
        if ti in done: continue
        P, W = t['pts'], t['w']; net, layer = t['net'], t['layer']; k = nets.get(net, {})
        if len(P)<2 or len(W)!=len(P)-1: raise ValueError('ungueltige pts/w-Laengen')
        profile = netzform(net, k)
        stat['profil_'+profile['form']] += 1
        if net in blocked:
            stat['paar_profil_geschuetzt']+=1;continue
        if net in fixed_nets or t.get('fix') or t.get('strom_steg') or 'items' in t or t.get('maeander') or matched and (k.get('paar') or k.get('laengengruppe')):
            stat['geschuetzt']+=1; continue
        if k.get('paar'):
            partner = k['paar']; candidates = [j for j in bynet[partner] if j not in done and j != ti and out['tracks'][j]['layer']==layer]
            result = None
            partner_profile = netzform(partner,nets.get(partner,{}))
            if profile['form'] != partner_profile['form'] or nets.get(partner,{}).get('paar',net) != net:
                stat['paar_profil_geschuetzt']+=1
                profile_conflicts.append(dict(netze=sorted((net,partner)),grund='unvereinbare Form-/Paarprofile'))
                continue
            for tj in candidates:
                b = out['tracks'][tj]
                if partner in fixed_nets or b.get('fix') or b.get('strom_steg') or b.get('maeander') or 'items' in b: continue
                radius = min(.5,profile['form_rmax_mm']) if is_fanout(t['pts'],regions) else profile['form_rmax_mm']
                partner_radius = min(.5,partner_profile['form_rmax_mm']) if is_fanout(b['pts'],regions) else partner_profile['form_rmax_mm']
                radius_failures=guard.tests['paar_radius_unvereinbar']
                result = paired(t,b,ti,tj,guard,radius,error,partner_rmax=partner_radius)
                if guard.tests['paar_radius_unvereinbar']>radius_failures:
                    stat['paar_profil_geschuetzt']+=1
                    profile_conflicts.append(dict(netze=sorted((net,partner)),grund='Radiusgrenzen erlauben keine konzentrischen Boegen',
                                                  radius_max_mm={net:radius,partner:partner_radius}))
                if result:
                    aa, bb, arcs = result
                    for idx, geo in ((ti,aa),(tj,bb)):
                        out['tracks'][idx].update(geo); done.add(idx)
                        guard.add(geo['pts'], geo['w'][0],layer,out['tracks'][idx]['net'],error)
                        changed.append(dict(track=idx,net=out['tracks'][idx]['net'],boegen=arcs,eckpunkte_entfernt=0))
                    stat['paare_geformt']+=1; break
            if not result: stat['paar_topologie_geschuetzt']+=1
            continue
        PP, WW, starts = split_anchors(t,nodes[(net,layer)])
        allpts = [PP[0]]; allwidth=[]; allitems=[]; removed=arcs_total=0
        stat['feste_breitenanker'] += sum(abs(W[j]-W[j-1])>1e-9 for j in range(1,len(W)))
        for begin,end in zip(starts,starts[1:]):
            part=PP[begin:end+1]; w=WW[begin]; contacts=guard.contacts(part,w,layer,net,ti)
            fanout=is_fanout(part,regions)
            if fanout: stat['bga_teilketten_45r']+=1
            pull=profile['form']=='organisch' and not k.get('laengengruppe') and not fanout
            Q,n=straighten(part,w,layer,net,ti,guard,contacts) if pull else (part,0)
            its,dense,arcs=round_track(Q,w,layer,net,ti,guard,contacts,min(.5,profile['form_rmax_mm']) if fanout else profile['form_rmax_mm'],error)
            test=dict(items=its)
            if analytic_length(test)>length(part)+1e-8 or not guard.retain(dense,contacts,error if arcs else 0):
                stat['endtor_abgelehnt']+=1; its=[['L',a,b,w] for a,b in zip(part,part[1:])]; dense=part; n=arcs=0
            allpts.extend(dense[1:]); allwidth.extend([w]*(len(dense)-1)); allitems.extend(its)
            removed+=n; arcs_total+=arcs
            if n or arcs: guard.add(dense,w,layer,net,error if arcs else 0)
        if removed or arcs_total:
            t.update(pts=[list(q) for q in allpts],w=allwidth,items=allitems)
            changed.append(dict(track=ti,net=net,boegen=arcs_total,eckpunkte_entfernt=removed))
            if k.get('art') in ('strom','versorgung','sw'): stat['strombahnen_geformt']+=1
    rollback=set()
    rollback.update(kelvin_endtor(board, routes, out, rules, changed, error))


    e0,groups=group_errors(routes,classes); e1,_=group_errors(out,classes)
    for g, old in e0.items():
        if e1[g]>old+1e-7: rollback.update(groups[g]); stat['laengengruppe_abgelehnt']+=1
    if connectivity:
        old=set(connectivity(board,routes,set(routes.get('flaechennetze',[]))))
        rollback.update(set(connectivity(board,out,set(routes.get('flaechennetze',[]))))-old)


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
        raise RuntimeError('Laengentor nach Ruecknahme verletzt')
    if connectivity and set(connectivity(board,out,set(routes.get('flaechennetze',[]))))-old:
        raise RuntimeError('Verbindungstor nach Ruecknahme verletzt')
    stat.update(bahnen_geaendert=len(changed),boegen=sum(c['boegen'] for c in changed),eckpunkte_entfernt=sum(c['eckpunkte_entfernt'] for c in changed))
    out['organisch_nachlauf']=dict(version=3,stil='ideal',cpu_s=time.process_time()-t0,sehnenfehler_mm=error,
        abstandsreserve_mm=guard.margin,statistik=dict(stat),pruefungen=dict(guard.tests),aenderungen=changed,
        profilkonflikte=profile_conflicts,laengendifferenzen_vorher=e0,laengendifferenzen_nachher=final_errors,
        ruecknahme_netze=sorted(rollback),verbindungstor_geprueft=connectivity is not None,native_drc_erforderlich=True,
        reihenfolge='Form -> Laengenabgleich',breitenprofil='urspruengliche Breiten und Breitenanker (keine Verjuengung)')
    return out


def rule_gate(board, before, after, classes, gndlagen=None, *, topologie_version=None,
              si_note=False, si_messen=False):
    ""
    from analyse import regelkennzahlen as RK
    options=dict(gndlagen=gndlagen,si_note=si_note,si_messen=si_messen)
    if topologie_version is not None: options['topologie_version']=topologie_version
    a=RK.messen(board,before,classes,**options); b=RK.messen(board,after,classes,**options)
    contract=RK.annahme(a,b); comparison=contract['vergleich']
    missing=comparison['fehlend']
    worse=[k for k,t in comparison['je'].items() if t and t['tor'] and t['urteil']>0]
    fa,fb=native_form_metrics(before,board),native_form_metrics(after,board)
    for key in ('spitze_ecken','spitze_abzweige','spitze_ecken_frei','spitze_abzweige_frei'):
        if fb[key]>fa[key]: worse.append(key)
    return dict(angenommen=contract['nicht_schlechter'] and not worse,fehlend=missing,schlechter=worse,
                regelvertrag=contract,vorher=a,nachher=b)


def native_form_metrics(routes, board=None):
    ""
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
    return dict(knicke_nativ=kn,spitze_ecken=acute,spitze_abzweige=branch,
                spitze_ecken_frei=acute-acute_pad,spitze_ecken_pad=acute_pad,spitze_abzweige_frei=branch-branch_pad,
                spitze_abzweige_pad=branch_pad,mit_board=board is not None,boegen=len(radius),
                radius_min_mm=min(radius,default=None),radius_max_mm=max(radius,default=None),
                laenge_analytisch_mm=sum(analytic_length(t) for t in routes['tracks']))
