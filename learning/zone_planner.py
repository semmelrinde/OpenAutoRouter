import copy
import hashlib
import json
import math
import os
from pathlib import Path
import sys
from typing import TypedDict

from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

ROOT = Path(__file__).resolve().parents[1]
SENSE = {'feedback', 'feedback_c', 'divider', 'setting_r'}


class Plan(TypedDict):
    version: str
    id: str
    net_list: dict
    terminals: list
    core_set: list
    growth: list
    via_candidates: list
    audit: dict


def sha(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True, allow_nan=False).encode()).hexdigest()


def check_plan(plan):
    if plan.get('version') != 'zone_plan-local-v2' or plan.get('id') != sha(
            {k:v for k,v in plan.items() if k not in ('id','board_file')}):
        raise ValueError('Zone plan content/version changed')


def check_inputs(plan, board, netclasses, design_rules, options=None):
    check_plan(plan)
    if plan.get('routing_board_sha256') != sha(board):
        raise ValueError('Zone plan does not match the router board contract')
    for name, obj in (('netclasses', netclasses), ('design_rules', design_rules)):
        if obj is None or plan.get('input_hashes', {}).get(name) != sha(obj):
            raise ValueError('Zone plan: current '+name+'-contract changed/missing')
    if not plan.get('native_geometry'):
        raise ValueError('Local zone plan needs native pad geometry')
    if plan.get('board_file'):
        native = json.loads(Path(plan['board_file']).read_text())
        if sha(native) != plan['input_hashes']['board']:
            raise ValueError('Zone plan: native pad geometry changed')
    if options is not None:
        gnd = options.get('gnd_layers', '').split(',') if options.get('gnd_layers') else []
        if sorted(gnd) != sorted(plan['reference_protection']['gnd_layers']):
            raise ValueError('Zone plan: GND context changed')


        from learning.zone_planner_contract import geometry
        expected = geometry(design_rules, plan.get('context',{}).get('options'))
        actual = geometry(design_rules, dict(plan.get('context',{}).get('options') or {}, **{k:v for k,v in options.items() if k!='gnd_layers'}))
        if expected != actual: raise ValueError('Zone plan: geometry options changed')



def acceptance_gate(z, info, required):
    accepted = (info or {}).get('applied') if required == 'legacy' else (info or {}).get('native_acceptance')
    if required and not accepted:
        z.update(is_valid=False, invalid_reason='R36: '+str(
            (info or {}).get('reason') or (info or {}).get('faults') or 'native acceptance missing'))
    if 'is_valid' in (z.get('rule') or {}):
        z['rule']['is_valid'] = z['is_valid']
    return z


def current_val(v):
    f = v.get('flow') or {}
    x = v.get('current_A')
    source = v.get('current_A_source')
    if x is None and f.get('unit') == 'A':
        loads = list((f.get('load') or {}).values())
        if any(not isinstance(a, (int, float)) or not math.isfinite(a) or a < 0 for a in loads):
            return None, 'invalid loads'
        x, source = sum(loads), f.get('unit_source', 'flow_A')
    if x is None:
        return None, 'unknown'
    if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) or x < 0:
        return None, 'invalid current'
    return float(x), source or 'current_A without origin'


def selection(K):
    out = {}
    for n, v in sorted(K.get('net_list', {}).items()):
        amps, origin = current_val(v)
        functional = any(t.get('decoupling') and (t.get('supports') or {}).get('net_name') == n
                         for t in K.get('part_list', {}).values())
        power = v.get('category') in ('supply', 'current_val', 'sw')
        sense = v.get('kelvin') or n in K.get('gnd', []) or v.get('category') == 'gnd'
        selected = not sense and power and (functional or v.get('category') == 'sw' or amps is None or amps >= .01)
        reason = ('GND/Kelvin separated' if sense else 'Signal' if not power else
                  'FunctionDuty' if functional or v.get('category') == 'sw' else
                  'Current unknown, local precaution' if amps is None else
                  'R36 mandatory >=100mA' if amps >= .1 else
                  'R36 preference >=10mA' if amps >= .01 else 'Small current, track allowed')
        out[n] = dict(chosen=selected, category=v.get('category'), current_A=amps, current_source=origin,
                      reason=reason, function_duty=functional, local_without_layer_change=v.get('category') == 'sw')
    return out


def padid(p):
    return '%s:%s' % (p['ref'], p['num'])


def shape_check(g):
    corners = []
    for p in getattr(g, 'geoms', [g]):
        if p.geom_type != 'Polygon':
            continue
        for ring in [p.exterior, *p.interiors]:
            xy = list(ring.coords)[:-1]
            for i, b in enumerate(xy):
                a, c = xy[i-1], xy[(i+1) % len(xy)]
                u, v = (b[0]-a[0], b[1]-a[1]), (c[0]-b[0], c[1]-b[1])
                the = math.hypot(*u)*math.hypot(*v)
                if the < 1e-14:
                    continue
                turn = math.degrees(math.acos(max(-1, min(1, (u[0]*v[0]+u[1]*v[1])/the))))
                if turn > 8.01:
                    corners.append(dict(xy=b, direction_change_deg=round(turn, 3)))
    return dict(passed=not corners, free_bends=corners, chord_angle_max_deg=8.01)


def _rounds(g, vacant, collar=.06, oct_=False):
    from learning.current_zones import tangential_rounds


    if oct_:
        from learning.zone_planner_oct import hull
        convex=hull(g).buffer(max(.1,collar),quad_segs=48)
    else:convex=g.convex_hull.buffer(collar,quad_segs=48)
    if vacant.covers(convex):return convex
    shape = g.buffer(collar, quad_segs=48).intersection(vacant)
    for _ in range(3):
        shape = tangential_rounds(shape.simplify(.0001, preserve_topology=True), Polygon(), vacant, .12)
    return shape


def via_middle_items(R):
    from common.track_items import validate
    via_d=float(str((R.get('options') or {}).get('via') or '0.45/0.20').split('/')[0])
    changed=0
    for t in R['tracks']:
        if not t.get('items'):continue
        items=t['items'];pts=t['pts'];widths=t['w']
        first,load=items[0][1],items[-1][2 if items[-1][0]=='L' else 3]
        starts=[i for i,p in enumerate(pts) if math.dist(p,first)<1e-7]
        ends=[i for i,p in enumerate(pts) if math.dist(p,load)<1e-7]
        if not starts or not ends:continue
        lo,hi=starts[0],ends[-1]
        if lo>hi:continue
        additions=[]
        for i in list(range(lo))+list(range(hi,len(pts)-1)):
            from shapely.geometry import LineString
            stub=LineString([pts[i],pts[i+1]]).buffer(widths[i]/2,quad_segs=48)
            rings=[Point(v['x'],v['y']).buffer(v.get('d',v.get('diameter',via_d))/2,quad_segs=48)
                   for v in R['vias'] if v['net']==t['net']]




            endpoint=first if i<lo else load
            olditem=items[0] if i<lo else items[-1]
            endwidth=olditem[3] if olditem[0]=='L' else olditem[4]
            cap=Point(*endpoint).buffer(endwidth/2,quad_segs=48)
            if not any(g.covers(LineString([pts[i],pts[i+1]])) and
                       g.union(cap).buffer(.00025).covers(stub) for g in rings):
                raise ValueError('R37 item extension does not lie fully within the via copper')
            additions.append((i,['L',pts[i],pts[i+1],widths[i]]))
        if additions:
            t['items']=[it for i,it in additions if i<lo]+items+[it for i,it in additions if i>=hi]
            changed+=len(additions)
        validate(t,error=float((R.get('organic_post_run') or {}).get('chord_error_mm',.00025)))
    return changed


def schedule(B, K, RG, context=None) -> Plan:
    from learning import current_zones as SZ
    ctx = dict(context or {})
    op = dict((RG or {}).get('router') or {}, **(ctx.get('options') or {}))
    from learning.zone_planner_contract import geometry
    normalized = geometry(RG, op)
    if int(normalized['cu_layers']) != int(B['cu_layers']):
        raise ValueError('Plan/router layers differ; specify cu_layers explicitly in the rules')
    op = dict(op, **{k:v for k,v in normalized.items() if k!='via'}, via='/'.join(map(str,normalized['via'])))
    layers = SZ._layers(B)
    clr = max(float(op.get('clr') or .1), float(op.get('drcpad') or 0),
              float((B.get('design') or {}).get('min_spacing') or 0)) + .004
    edge = float(op['edge']) + .036
    drill_gap = max(float(op.get('h2h') or .2), float((B.get('design') or {}).get('hole_hole') or 0))
    vd, drill = SZ._via_ground(dict(options=op))
    vd = max(vd, float((B.get('design') or {}).get('via_min') or 0),
             drill + 2*float((B.get('design') or {}).get('ring_min') or 0))
    drill = max(drill, float((B.get('design') or {}).get('bore_min') or 0))
    vd = max(vd, drill+2*float((B.get('design') or {}).get('ring_min') or 0))
    hole_clr = max(float(op.get('hole_clr') or 0), float((B.get('design') or {}).get('hole_spacing') or 0))
    from analysis.areas import contour
    board = contour(B).buffer(-edge)
    fps = {f['ref']: f for f in B.get('fps', [])}
    selected = selection(K)
    plan = Plan(version='zone_plan-local-v2', id='', net_list=selected, terminals=[], core_set=[],
                growth=[], via_candidates=[], audit=dict(rejections=[], shaping=[], kicad_native_confirmed=False))
    plan.update(context=ctx, input_hashes=dict(board=sha(B), netclasses=sha(K), design_rules=sha(RG), context=sha(ctx)),
                pad_geometry=ctx.get('pad_geometry', 'declared export polygons'),
                native_geometry=bool(ctx.get('native_geometry')), start_spacing_mm=1.5,
                reference_protection=dict(gnd_layers=ctx.get('gnd_layers', []), repurposing=False),
                electrical_budgets=copy.deepcopy((K.get('specs') or {}).get('budgets', {})))
    occupied = {L: [] for L in layers}
    vias = []
    def rejected(n, L, why, **extra):
        plan['audit']['rejections'].append(dict(net=n, layer=L, reason=why, **extra))
    for n in sorted(selected, key=lambda n: (-(selected[n]['current_A'] or 0), n)):
        info = selected[n]
        if not info['chosen']:
            continue
        v = K['net_list'][n]
        f = v.get('flow') or {}
        sense = {q['pad'] for q in K.get('taps', {}).get(n, [])}
        pads = sorted([p for p in B['pads'] if p['net'] == n], key=lambda p: (padid(p), p['x'], p['y']))
        sense |= {padid(p) for p in pads if K.get('part_list',{}).get(p['ref'],{}).get('function') in SENSE
                  or any(x in str(p.get('pinfunc') or '').upper() for x in ('SENSE','FB','VSNS','ISNS'))}
        local_layers = set.intersection(*(set(p.get('layers', [])) for p in pads)) if pads else set()
        if info['local_without_layer_change']:
            common = sorted(local_layers & {'F', 'B'})
            info['local_layer'] = common[0] if common else None
            if not common:
                rejected(n, None, 'SW/BST without a common outer layer')
        for L in ('F', 'B'):
            entries = []
            for p in pads:
                if L not in p.get('polys', {}):
                    continue
                ti = K.get('part_list', {}).get(p['ref'], {})
                is_sense = padid(p) in sense or ti.get('function') in SENSE or any(
                    x in str(p.get('pinfunc') or '').upper() for x in ('SENSE', 'FB', 'VSNS', 'ISNS'))
                bga = 'BGA' in str(fps.get(p['ref'], {}).get('fp', '')).upper()
                support = ti.get('supports') or {}
                group = support.get('ref') if support.get('net_name') == n else ti.get('island') or p['ref']
                if not group:
                    group = p['ref']
                if is_sense:
                    rejected(n, L, 'Sense anchor stays a track', pad=padid(p))
                pg = unary_union([Polygon(q).buffer(0) for q in p['polys'][L]])
                terminal = dict(id='%s:%s:%d' % (padid(p), L, len(plan['terminals'])), net=n, layer=L,
                                pad=padid(p), ref=p['ref'], xy=[p['x'], p['y']], group=group,
                                source=padid(p) in f.get('source', []), load_A=(f.get('load') or {}).get(padid(p)) if f.get('unit') == 'A' else None,
                                sense=is_sense, bga=bga, polys=SZ._polys(pg))
                plan['terminals'].append(terminal)
                if not is_sense:
                    entries.append((terminal, pg))
            if not entries:
                continue

            obstacles = [Polygon(q).buffer(clr, quad_segs=48) for p in B['pads']
                         if p['net'] != n or padid(p) in sense or K.get('part_list', {}).get(p['ref'], {}).get('function') in SENSE
                         for q in p.get('polys', {}).get(L, [])]
            obstacles += [Point(p['x'], p['y']).buffer(p['drill']/2+hole_clr+.004, quad_segs=48)
                          for p in B['pads'] if p.get('drill') and p['net'] != n]
            obstacles += [g.buffer(clr, quad_segs=48) for m, g in occupied[L] if m != n]
            obstacles += [SZ.keepout_area(B, L, 'areas'), SZ.keepout_area(B, L, 'track_list')]
            free = board.difference(unary_union(obstacles))

            normal = [(t, g) for t, g in entries if not t['bga']]

            free=free.difference(unary_union([g.buffer(clr,quad_segs=48) for t,g in entries if t['bga']]))
            for t, _ in entries:
                if t['bga']:
                    rejected(n, L, 'BGA single ball/fanout, no pad hull', pad=t['pad'])
            groups = []
            for t, g in normal:
                merged = False
                for members, geometries, bridges in groups:
                    if members[0]['group'] != t['group']:
                        continue
                    choices = sorted([(g.distance(q), j) for j, q in enumerate(geometries)])
                    for dist, j in choices:
                        if dist > 1.5:
                            break
                        pair = g.union(geometries[j]).convex_hull
                        foreign_groups=unary_union([h.buffer(clr) for u,h in normal if u['group']!=t['group']])
                        if free.covers(pair.buffer(.06, quad_segs=48)) and not pair.intersects(foreign_groups):
                            members.append(t); geometries.append(g); bridges.append(pair); merged = True; break
                    if merged:
                        break
                if not merged:
                    groups.append(([t], [g], []))
            for members, geometries, bridges in groups:
                body = unary_union(geometries+bridges)
                group_free=free.difference(unary_union([g.buffer(clr,quad_segs=48) for t,g in normal
                                                       if t['group']!=members[0]['group']]))
                core = _rounds(body, group_free, oct_=ctx.get('stage3',False))
                manufacturing = max(float((B.get('design') or {}).get('min_width') or 0),
                                    float((RG or {}).get('raw_data', {}).get('min_track_width') or 0), float(op.get('track_min') or 0))
                minimum = max(manufacturing, .25 if info['category'] == 'sw' or core.area >= 20 else .1)
                audit = shape_check(core)
                if (core.is_empty or not core.buffer(1e-6).covers(body) or not audit['passed']
                        or len(SZ._polys(core.buffer(-minimum/2))) != 1):
                    rejected(n, L, 'Local core: connection/shape/minimum width not satisfiable', pads=[t['pad'] for t in members])
                    continue
                ident = '%s:%s:%d' % (n, L, len(plan['core_set']))
                zone = dict(net=n, layer=L, polys=SZ._polys(core), source='zone_planner', terminals=[t['id'] for t in members],
                            core_id=ident, min_thickness=minimum, shaping=audit, r36=True)
                occupied[L].append((n, core)); plan['core_set'].append(zone)
                plan['growth'].append(dict(net=n, layer=L, polys=SZ._polys(core.buffer(.25).intersection(free)), source='zone_planner-growth'))
                plan['audit']['shaping'].append(dict(core_id=ident, **audit))
                if info['local_without_layer_change'] or int(B['cu_layers']) < 4:
                    continue
                amps = info['current_A'] if any(t['source'] for t in members) else sum(t['load_A'] or 0 for t in members)
                if not amps:
                    amps = info['current_A']
                rule = max(2, math.ceil(4*amps)) if amps is not None else 2
                physics = math.ceil(1.3*amps/(drill/.3)) if amps is not None and drill > 0 else None
                demand = max(rule, physics or 0)
                field = dict(net=n, layer=L, core_id=ident, current_A=amps, rule=rule, physics_demand=physics,
                             nominal=demand, vias=[], raster_mm=.5, taken=False)
                from shapely.ops import nearest_points


                field_free=free.difference(unary_union([h.buffer(clr) for u,h in normal if u not in members]))
                search=core.buffer(1.5).intersection(field_free)
                x0, y0, x1, y1 = search.bounds
                center = core.representative_point()
                additions=[]
                candidates = [(math.dist((x*.5, y*.5), (center.x, center.y)), x*.5, y*.5)
                              for x in range(math.ceil(x0/.5), math.floor(x1/.5)+1)
                              for y in range(math.ceil(y0/.5), math.floor(y1/.5)+1)]
                hints=[(0,v['x'],v['y']) for f in B.get('copper_reserve',[]) if f['net']==n and any(t['ref']==f['ref'] for t in members) for v in f['vias']]
                for _, x, y in hints+sorted(candidates):
                    pt = Point(x, y); disk = pt.buffer(vd/2, quad_segs=48)
                    if any(math.dist((x,y),(q['x'],q['y'])) < 1e-6 for q in field['vias']):continue
                    if not field_free.covers(disk.buffer(.06)) or any(math.dist((x,y),(q['x'],q['y'])) < (drill+q['drill'])/2+drill_gap+.004 for q in vias+field['vias']):
                        continue
                    a,b=nearest_points(core,disk)
                    from shapely.geometry import LineString
                    bridge=LineString([a,b]).buffer(max(.25,minimum)/2,quad_segs=48) if a.distance(b)>1e-9 else disk
                    extension=disk.union(bridge).buffer(.06,quad_segs=48)
                    if not field_free.covers(extension):continue
                    legal = True
                    for lay in layers:
                        if disk.buffer(clr).intersects(SZ.keepout_area(B, lay, 'vias')):
                            legal = False; break
                        for p in B['pads']:
                            ps = unary_union([Polygon(q) for q in p.get('polys', {}).get(lay, [])])
                            if p['net'] != n and disk.distance(ps) < clr:
                                legal = False; break
                            if p.get('drill') and pt.distance(Point(p['x'],p['y'])) < (drill+p['drill'])/2+drill_gap+.004:
                                legal = False; break
                            if float(op.get('vip', 0)) != 1 and lay in ('F','B') and disk.distance(ps) < .1:
                                legal = False; break
                        if any(m != n and disk.distance(g) < clr for m,g in occupied[lay]):
                            legal = False
                        if not legal:
                            break
                    if legal:
                        field['vias'].append(dict(net=n,x=x,y=y,d=vd,drill=drill,layers=layers,source='zone_planner',core_id=ident))
                        additions.append(extension)
                    if len(field['vias']) == demand:
                        break
                field['actual'] = len(field['vias']); field['taken'] = field['actual'] == demand
                if field['taken'] and additions:
                    enriched=_rounds(unary_union([core]+additions),field_free,collar=.01,oct_=ctx.get('stage3',False))
                    shapeaudit=shape_check(enriched)
                    if not enriched.buffer(1e-6).covers(body) or not shapeaudit['passed']:
                        field['taken']=False
                    else:
                        core=enriched;zone.update(polys=SZ._polys(core),shaping=shapeaudit)
                        occupied[L][-1]=(n,core)
                        plan['growth'][-1]['polys']=SZ._polys(core.buffer(.25).intersection(field_free))
                plan['via_candidates'].append(field)
                if field['taken']:
                    vias += field['vias']
                    for lay in layers:
                        occupied[lay].extend((n, Point(q['x'],q['y']).buffer(vd/2)) for q in field['vias'])
                else:
                    rejected(n, L, 'Via field lacks room', core_id=ident, nominal=demand, actual=field['actual'])
    plan['audit'].update(cores=len(plan['core_set']), terminals=len(plan['terminals']),
                         via_fields=sum(f['taken'] for f in plan['via_candidates']),
                         current_unknown=[n for n,i in selected.items() if i['chosen'] and i['current_A'] is None],
                         stage=2, inner_corridors=False, load_pad_J='required separately in the final step')
    if ctx.get('stage3'):
        from learning.zone_planner_corridor import complete
        complete(B,K,RG,plan,board,clr)
    plan['id'] = sha({k:v for k,v in plan.items() if k != 'id'})
    return plan


def prepare(bj, kj, design_rules, mode, gnd_layers, cpu_s=120):
    from common import slots as PL
    if mode not in ('diagnose', 'local'):
        raise ValueError('zone_plan=diagnose|local')
    folder = Path(bj).parent/'zone_plan'; folder.mkdir(parents=True, exist_ok=True)
    out = folder/'zone_plan.json'; out.unlink(missing_ok=True)
    cmd = ['caffeinate','-i','-s',sys.executable,'-m','learning.zone_planner','plan',str(Path(bj).resolve()),
           str(Path(kj).resolve()),str(Path(design_rules).resolve()),str(out.resolve()),mode,','.join(gnd_layers)]
    run = PL.job(cmd, cpu_s=cpu_s, log=str(folder/'plan.log'), cwd=str(ROOT), env=dict(os.environ,PYTHONHASHSEED='0'))
    info = dict(mode=mode, compute_slot=run, file=str(out), applied=run['cur_state']=='ok' and out.exists())
    if info['applied']:
        plan = json.loads(out.read_text()); info.update(id=plan['id'],audit=plan['audit'])
    return info


def final(bj, kj, rj, design_rules, gnd_layers, mode='kicad_native', cpu_s=720):
    from learning import current_zones as SZ
    if mode not in ('legacy', 'kicad_native', 'local', 'geometric'): raise ValueError('r36=legacy|kicad_native|geometric|0')
    plan = str(Path(bj).parent/'zone_plan/zone_plan.json') if mode == 'local' else None
    if mode == 'geometric':


        info = SZ._geometric(bj,kj,rj,gnd_layers,cpu_s=cpu_s)
        info.update(fallback='geometric', native_acceptance=False, kicad_native_reason='not attempted (r36=geometric)')
        mode = 'legacy'
    else:
        info = SZ.apply(bj,kj,rj,gnd_layers,design_rules=design_rules,cpu_s=cpu_s,zone_plan=plan)
        info['native_acceptance'] = bool(info.get('applied'))
    if mode == 'legacy':


        if not info['native_acceptance'] and info.get('fallback') != 'geometric':
            nat = info

            native_run = nat.get('compute_slot') or {}
            rest = max(0., cpu_s - native_run.get('cpu_s', 0.))
            if rest < 1:
                return dict(nat, applied=False, reason='R36 replacement: CPU budget exhausted')
            geo = SZ._geometric(bj,kj,rj,gnd_layers,cpu_s=rest)
            geo_run = geo.get('compute_slot') or {}
            info = dict(geo, native_acceptance=False,
                        compute_slot_kicad_native=native_run, compute_slot_geometric=geo_run,
                        compute_slot=dict(geo_run, cpu_s=native_run.get('cpu_s',0.) + geo_run.get('cpu_s',0.)),
                        kicad_native_reason=str(nat.get('reason') or nat.get('faults'))[:200])
            if not geo.get('applied'): return info
            info['fallback'] = 'geometric'
        if not info.get('applied'): return info
        B,K,R = [json.loads(Path(p).read_text()) for p in (bj,kj,rj)]
        failures = SZ.verify(B,R,K,gnd_layers)
        if failures:
            way = {(n,L) for n,L,*_ in failures}
            R['zone_list'] = [q for q in R.get('zone_list',[]) if (q['net'],q['layer']) not in way]
            if any(n in set(K.get('gnd') or []) for n,_ in way):
                R['vias'] = [v for v in R['vias'] if v.get('source') != 'gnd_fill']
                R['zone_list'] = [q for q in R['zone_list'] if q.get('source') != 'gnd_fill']
            Path(rj).write_text(json.dumps(R)); info['rejected'] = failures[:8]
        return info
    if not info['native_acceptance']:

        target = str(Path(rj).with_suffix(''))+'_geometric_fallback.json'
        try:
            native_run = info.get('compute_slot') or {}
            rest = max(0., cpu_s - native_run.get('cpu_s',0.))
            if rest < 1: return dict(info, fallback_blocked='cpu_budget')
            info['geometry'] = SZ._geometric(bj,kj,rj,gnd_layers,off=target,cpu_s=rest)
            geo_run = info['geometry'].get('compute_slot') or {}
            info['compute_slot_kicad_native'] = native_run
            info['compute_slot_geometric'] = geo_run
            info['compute_slot'] = dict(geo_run, cpu_s=native_run.get('cpu_s',0.) + geo_run.get('cpu_s',0.))
            if info['geometry'].get('applied'):
                info['fallback'] = 'geometric'
                info['fallback_file'] = target
            else:
                info['fallback_blocked'] = info['geometry'].get('fallback_blocked')
        except Exception as exc:
            info['fallback_error'] = str(exc)
    return info


def main():
    _, bj,kj,rg,out,mode,gnd = sys.argv[1:]
    from learning.copper_kicad_native import library_geometry
    B,K,RG = [json.loads(Path(p).read_text()) for p in (bj,kj,rg)]
    routing_hash = sha(B)
    folder = Path(out).parent
    B, geom = library_geometry(folder/'library',B,dict(options=RG.get('router',{})),RG)
    Path(folder/'native_board.json').write_text(json.dumps(B))
    plan = schedule(B,K,RG,dict(mode=mode,gnd_layers=gnd.split(',') if gnd else [],native_geometry=True,
                             pad_geometry=geom.get('source','native board export'),gnd_nets=K.get('gnd',[]),stage3=mode=='local' and int(B['cu_layers'])>=6))
    plan['board_file'] = str(folder/'native_board.json')
    plan['routing_board_sha256'] = routing_hash
    plan['id'] = sha({k:v for k,v in plan.items() if k not in ('id','board_file')})
    Path(out).write_text(json.dumps(plan,indent=1,ensure_ascii=False))


if __name__ == '__main__':
    main()
