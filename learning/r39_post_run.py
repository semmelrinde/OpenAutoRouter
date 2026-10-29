import copy
import json
import math
import os
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def export_zones(R):
    from collections import defaultdict
    import uuid
    from shapely.geometry import Polygon
    from shapely.geometry.polygon import orient
    from shapely.ops import unary_union
    from common.zones import unify
    r=copy.deepcopy(unify(R))
    groups=defaultdict(list);kept=[]
    for z in r.get('zones',[]):
        if z.get('source')=='R36-FullCopperModel':groups[z['net'],z['layer']].append(z)
        else:kept.append(z)
    for (net,layer),zones in sorted(groups.items()):
        parameters=('full','min_thickness','local_clearance','prio')
        if len({tuple(z.get(k) for k in parameters) for z in zones})!=1:
            raise ValueError('R39: different zone parameters at '+net+'/'+layer)
        g=unary_union([Polygon(z['pts'],z.get('holes',[])).buffer(0) for z in zones])
        contours=[]
        for poly in sorted(getattr(g,'geoms',[g]),key=lambda p:(p.bounds,p.area)):
            if poly.is_empty or poly.geom_type!='Polygon':continue
            poly=orient(poly,sign=1)
            contours.append(dict(pts=list(poly.exterior.coords),holes=[list(i.coords) for i in poly.interiors]))
        meta={k:v for k,v in zones[0].items() if k not in ('pts','holes','native_outlines','zone_id')}
        meta.update(native_layers=[layer],native_outlines=contours,
                    zone_id=str(uuid.uuid5(uuid.NAMESPACE_URL,'autorouter:r39-export:'+net+':'+layer)))
        kept.extend(dict(meta,**p) for p in contours)
    r['zones']=kept
    return r


def unknown_gate(B, before, after, measurement):
    from learning.copper_bars import copper_map
    from shapely.geometry import box, Polygon
    a, b = copper_map(B, before), copper_map(B, after)
    for row in measurement.get('not_measurable', []):
        key = row['net_name'], row['layer_name']
        region = box(*row['bounds']).buffer(1e-6)
        if a.get(key, Polygon()).symmetric_difference(b.get(key, Polygon())).intersection(region).area > 1e-8:
            return False
    return True


def export_equal(B, before, after):
    from learning.copper_bars import copper_map
    from common.zones import unify
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    representation = {'zones', 'zone_list'}
    canonical = unify(before)
    if ({k:v for k,v in canonical.items() if k not in representation} !=
            {k:v for k,v in after.items() if k not in representation}): return False


    groups = {}
    for z in after.get('zones', []):
        if z.get('source') == 'R36-FullCopperModel':
            groups.setdefault((z['net'], z['layer']), []).append(z)
    for (_, layer), zones in groups.items():
        filled = unary_union([Polygon(z['pts'], z.get('holes', [])) for z in zones])
        for z in zones:
            if z.get('native_layers') != [layer] or not z.get('native_outlines'): return False
            outlines = unary_union([Polygon(q['pts'], q.get('holes', [])) for q in z['native_outlines']])
            if filled.symmetric_difference(outlines).area > 1e-8: return False
    a, b = copper_map(B, before), copper_map(B, after)
    return all(a.get(key, Polygon()).symmetric_difference(b.get(key, Polygon())).area <= 1e-8
               for key in set(a) | set(b))


def path_gate(a, b):
    return (a['version'] == b['version'] and a['parameter'] == b['parameter']
            and a['coverage'] == b['coverage']
            and a['not_measurable'] == b['not_measurable']
            and b['unnecessary_mm2'] < a['unnecessary_mm2'] - .001)


def rule_gate(a, b):
    from analysis.rule_metrics import assumption, gate_fields
    gate = assumption(a, b)

    return gate['not_worse'] and all(
        isinstance(a.get(k), (int, float)) and isinstance(b.get(k), (int, float))
        and math.isfinite(a[k]) and math.isfinite(b[k]) and b[k] <= a[k] + 1e-9
        for k in set(gate_fields(a)) | set(gate_fields(b)))


def suggestion(B, R, K, net_name, gnd_layers):
    from shapely.geometry import Point, Polygon
    from shapely.ops import unary_union
    from learning import current_zones as SZ
    candidate = copy.deepcopy(R)


    candidate['zone_list'] = [z for z in candidate.get('zone_list', []) if z.get('source') != 'gnd_fill']
    candidate['vias'] = [v for v in candidate.get('vias', []) if v.get('source') != 'gnd_fill']
    from common.zones import unify
    candidate = copy.deepcopy(unify(candidate))
    extra = []
    outline = unary_union([Polygon(q).buffer(0) for q in B['outline']]).buffer(-.25, SZ.QS)
    clr = max(.15, float(R.get('clr') or (R.get('options') or {}).get('clr') or .1))
    _, drill = SZ._via_ground(R)
    hole = float((R.get('options') or {}).get('h2h') or .2)
    for z in candidate.get('zones', []):
        if not z.get('r36') or z['net'] != net_name: continue
        L = z['layer']
        copper = SZ._parts(B, candidate, L)
        own = unary_union([g for n, g in copper if n == net_name])
        foreign = unary_union([g for n, g in copper if n != net_name])
        holes = [Point(p['x'], p['y']).buffer(float(p['drill'])/2, SZ.QS)
                 for p in B['pads'] if p.get('drill') and p['net'] != net_name]
        holes += [Point(v['x'], v['y']).buffer(float(v.get('drill') or drill)/2, SZ.QS)
                  for v in candidate['vias'] if v['net'] != net_name and L in (v.get('layers') or SZ._layers(B))]
        free = outline.difference(foreign.buffer(clr+.003, SZ.QS))
        free = free.difference(unary_union(holes).buffer(hole+.003, SZ.QS))
        free = free.difference(SZ.keepout_area(B, L, 'areas'))
        old = Polygon(z['pts'], z.get('holes', [])).buffer(0)
        g = SZ.r39_corridor(B, candidate, K, L, net_name, old, own, free)
        delta = g.difference(old)
        if not delta.is_empty:
            extra.append(dict(net=net_name, layer=L, source='r39_corridor',
                polys=SZ._polys(delta), area_mm2=round(delta.area, 4)))
    candidate['zone_list'] += extra
    if any(z.get('source') == 'gnd_fill' for z in R.get('zone_list', [])):
        gz, gv = SZ.gnd_fill(B, candidate, K, gnd_layers)
        candidate['zone_list'] += gz
        candidate['vias'] += gv
    return candidate


def repair(B, R, K, gnd_layers=None, attempts=3):
    from shapely.geometry import Polygon
    from analysis.current_bottlenecks import bottlenecks
    from learning.shaping import is_valid
    from learning.collect import rule_measurement
    from learning.current_zones import verify
    from learning.copper_bars import copper_map
    from analysis.copper_model import ir_values
    from learning.current_zones_post_run import physics_gate
    gl = gnd_layers if gnd_layers is not None else R.get('gnd_layers') or []
    baseline = copy.deepcopy(R)

    baseline['options'] = dict(baseline.get('options') or {}, r39_diagnose='1')
    E = bottlenecks(B, baseline, K, gl)
    info = dict(applied=False, native_acceptance=False, measurement_version=E['version'], before=E, attempts=[])
    valid, _ = is_valid(B, K, None, gl, routes=baseline)
    if not valid:
        info['reason'] = 'Invalid routing: no additional current copper'
        info['after'] = E
        return baseline, info
    scores = {}
    for u in E['unnecessary']: scores[u['net_name']] = scores.get(u['net_name'], 0) + u['area_mm2']
    nets = sorted(scores, key=lambda n: (-scores[n], n))[:attempts]
    accepted = baseline
    m0 = rule_measurement(B, accepted, K, gl)['design_rules'] if nets else None
    ir0 = ir_values(B, accepted, K) if nets else None
    for n in nets:
        candidate = suggestion(B, accepted, K, n, gl)
        before_cu, after_cu = copper_map(B, accepted), copper_map(B, candidate)

        additive = all(g.difference(after_cu.get(key, Polygon())).area <= 1e-8
                       for key, g in before_cu.items() if key[0] not in (K.get('gnd') or ['GND']))
        e1 = bottlenecks(B, candidate, K, gl)
        net_area = lambda e: sum(u['area_mm2'] for u in e['unnecessary'] if u['net_name'] == n)
        early = dict(unknown=unknown_gate(B, accepted, candidate, E), additive=additive, r39=path_gate(E, e1), target_net=net_area(e1)<net_area(E)-.001,
                     spacing=not verify(B,candidate,K,gl))
        if not all(early.values()):
            info['attempts'].append(dict(net_name=n,gates=dict(early,is_valid=None,design_rules=None,physics=None),
                taken=False,before_mm2=E['unnecessary_mm2'],after_mm2=e1['unnecessary_mm2'],
                before_score=m0.get('score_v3'),after_score=None,reason='Early geometry/R39 gate'))
            continue
        valid1, _ = is_valid(B, K, None, gl, routes=candidate)
        m1 = rule_measurement(B, candidate, K, gl)['design_rules']
        ir1 = ir_values(B, candidate, K)
        gates = dict(unknown=early['unknown'], is_valid=valid1, additive=additive, spacing=not verify(B, candidate, K, gl),
                     design_rules=rule_gate(m0, m1), r39=path_gate(E, e1),
                     target_net=net_area(e1) < net_area(E)-.001,
                     physics=physics_gate(ir0, ir1)['passed'])
        take = all(gates.values())
        info['attempts'].append(dict(net_name=n, gates=gates, taken=take,
            before_mm2=E['unnecessary_mm2'], after_mm2=e1['unnecessary_mm2'],
            before_score=m0.get('score_v3'), after_score=m1.get('score_v3')))
        if take:
            accepted, E, m0, ir0 = candidate, e1, m1, ir1
            info['applied'] = True
    if info['applied']:
        pre_export = accepted
        accepted = export_zones(pre_export)
        info['export_copper_equal'] = export_equal(B, pre_export, accepted)
        final_e = bottlenecks(B, accepted, K, gl)
        if not (info['export_copper_equal'] and path_gate(info['before'], final_e)
                and final_e['unnecessary_mm2'] <= E['unnecessary_mm2']+.001):
            info.update(applied=False, export_gate=False)
            accepted = baseline; E = info['before']
        else:
            E = final_e; info['export_gate'] = True
            accepted.pop('current_zones_acceptance', None)
            accepted.pop('native_acceptance', None)
    info.update(after=E, reason='Secured geometric post-run' if info['applied'] else 'No candidate passes all gates')
    return accepted, info


def apply(bj, kj, rj, gnd_layers=None, cpu_s=240):
    from common import slots as PL
    from learning.r11_post_run import file_lock
    rj = str(Path(rj).resolve())
    with file_lock(rj):
        original = Path(rj).read_bytes()
    parent = Path(str(Path(rj).with_suffix(''))+'_r39')
    parent.mkdir(parents=True, exist_ok=True)
    artifact = Path(tempfile.mkdtemp(prefix='run_', dir=parent))
    source = artifact/'input_.json'; source.write_bytes(original)
    result, route = artifact/'acceptance.json', artifact/'routes.json'
    result.unlink(missing_ok=True); route.unlink(missing_ok=True)
    run = PL.job(['caffeinate', '-i', '-s', ROOT/'.venv/bin/python', __file__, bj, kj, source,
                   result, route, json.dumps(gnd_layers)], cpu_s=cpu_s, log=str(artifact/'worker.log'), cwd=str(ROOT))
    info = json.loads(result.read_text()) if run['cur_state'] == 'ok' and result.exists() else dict(applied=False, reason=run['cur_state'])
    info['compute_slot'] = run
    info['workdir'] = str(artifact)


    with file_lock(rj):
        if Path(rj).read_bytes() != original:
            info.update(applied=False, reason='Route changed during R39 check')
        elif route.exists() and run['cur_state'] == 'ok':
            with tempfile.NamedTemporaryFile(dir=Path(rj).parent, delete=False) as f:
                tmp = f.name; f.write(route.read_bytes())
            try: os.replace(tmp, rj)
            finally:
                if os.path.exists(tmp): os.unlink(tmp)
    result.write_text(json.dumps(info, indent=1))
    return info


if __name__ == '__main__':
    bj, kj, rj, result, route, gl = sys.argv[1:]
    B, K, R = [json.loads(Path(p).read_text()) for p in (bj, kj, rj)]
    try:
        output, info = repair(B, R, K, json.loads(gl))
        Path(route).write_text(json.dumps(output))
    except Exception as e:
        info = dict(applied=False, reason=repr(e))
    Path(result).write_text(json.dumps(info, indent=1))
