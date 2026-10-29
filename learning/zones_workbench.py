import glob, json, math, os, shutil, statistics, sys, time
from concurrent.futures import ProcessPoolExecutor

R_ = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); sys.path.insert(0, R_)
FOLDER = os.path.expanduser(os.environ.get('AUTOROUTER_ZONES_WORKBENCH', '~/Library/Caches/Autorouter/zones_workbench'))
ITSELF = os.path.expanduser('~/Library/Caches/Autorouter/selfplay')
MIX = {2: 3, 4: 8, 6: 7, 8: 6}


def cases():
    return sorted(d for d in glob.glob(os.path.join(FOLDER, 'cases', '*')) if os.path.exists(os.path.join(d, 'routes.json')))


def collect(n=24):
    from learning.current_zones import current_nets
    cand = sorted(glob.glob(os.path.join(ITSELF, 'task_*', 'k*', 'routes_current_zones', 'input_.json')), key=os.path.getmtime, reverse=True)
    per = {k: 0 for k in MIX}; goal = {k: round(v * n / 24) for k, v in MIX.items()}; new = 0; per_task = {}
    os.makedirs(os.path.join(FOLDER, 'cases'), exist_ok=True)
    for e in cand:
        k = os.path.dirname(os.path.dirname(e)); a = os.path.dirname(k)
        try: B = json.load(open(os.path.join(k, 'board.json'))); K = json.load(open(os.path.join(k, 'netclasses.json')))
        except (OSError, ValueError): continue
        L = int(B.get('cu_layers') or 2)
        if L not in per or per[L] >= goal[L] or per_task.get(a, 0) >= 2 or not current_nets(K, set(K.get('gnd') or ['GND'])): continue
        per_task[a] = per_task.get(a, 0) + 1
        name = '%s_%s' % (os.path.basename(a).replace('task_', 'a'), os.path.basename(k))
        d = os.path.join(FOLDER, 'cases', name)
        if os.path.exists(d): per[L] += 1; continue
        os.makedirs(d)
        for f in ('board.json', 'netclasses.json'): shutil.copyfile(os.path.join(k, f), os.path.join(d, f))
        shutil.copyfile(e, os.path.join(d, 'routes.json'))
        if os.path.exists(os.path.join(a, 'design_rules.json')): shutil.copyfile(os.path.join(a, 'design_rules.json'), os.path.join(d, 'design_rules.json'))
        json.dump(dict(source=k, cu_layers=L, time_s=time.strftime('%Y-%m-%d %H:%M')), open(os.path.join(d, 'meta.json'), 'w'))
        per[L] += 1; new += 1
        if all(per[x] >= goal[x] for x in per): break
    print('Cases: %d new, per layer count %s -> %s' % (new, per, os.path.join(FOLDER, 'cases')))


def _angle(poly):
    sharp = notch = 0
    for ring, outer in [(poly.exterior, True)] + [(i, False) for i in poly.interiors]:
        pts = list(ring.simplify(0.02).coords)[:-1]
        if len(pts) < 3: continue
        surface = sum(pts[i][0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * pts[i][1] for i in range(len(pts)))
        ccw = surface > 0
        for i in range(len(pts)):
            a, b, c = pts[i - 1], pts[i], pts[(i + 1) % len(pts)]
            v1, v2 = (a[0] - b[0], a[1] - b[1]), (c[0] - b[0], c[1] - b[1])
            l1, l2 = math.hypot(*v1), math.hypot(*v2)
            if l1 < 0.05 or l2 < 0.05: continue
            w = math.degrees(math.acos(max(-1, min(1, (v1[0] * v2[0] + v1[1] * v2[1]) / (l1 * l2)))))
            cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
            convex = (cross > 0) == ccw if outer else (cross > 0) != ccw
            if w < 80: sharp += convex; notch += not convex
    return sharp, notch


def wedges(B, R, K, clr=0.15, w_sharp=45.0, w_tight=80.0, n_places=10):
    from shapely.geometry import Polygon, Point
    from shapely.ops import unary_union
    from learning import current_zones as SZ
    contour = unary_union([Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3])
    zs = [z for z in R.get('zone_list', []) if z.get('source') == 'current_zones']
    n_sharp = n_tight = 0; places = []
    for L in sorted({z['layer'] for z in zs}):
        zl = unary_union([SZ._geo(z) for z in zs if z['layer'] == L])
        part_list = SZ._parts(B, R, L)
        per = {}
        for m, g in part_list: per.setdefault(m, []).append(g)
        copper_map = unary_union([unary_union(v).buffer(clr / 2 + .01, 8) for _, v in sorted(per.items())])
        white = contour.difference(copper_map)

        router = unary_union([g for _, g in SZ._parts(B, dict(R, zone_list=[z for z in R.get('zone_list', []) if z.get('source') != 'current_zones']), L)])
        close = zl.difference(router.buffer(.01, 8)).buffer(clr / 2 + .04, 8)
        for poly in getattr(white, 'geoms', [white]):
            if poly.geom_type != 'Polygon' or poly.area < 1e-3: continue
            for ring, outer in [(poly.exterior, True)] + [(i, False) for i in poly.interiors]:
                pts = list(ring.simplify(0.01).coords)[:-1]
                if len(pts) < 3: continue
                fl = sum(pts[i][0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * pts[i][1] for i in range(len(pts)))
                for i in range(len(pts)):
                    a, b, c = pts[i - 1], pts[i], pts[(i + 1) % len(pts)]
                    v1, v2 = (a[0] - b[0], a[1] - b[1]), (c[0] - b[0], c[1] - b[1])
                    l1, l2 = math.hypot(*v1), math.hypot(*v2)


                    if l1 < 0.1 or l2 < 0.1: continue
                    w = math.degrees(math.acos(max(-1, min(1, (v1[0] * v2[0] + v1[1] * v2[1]) / (l1 * l2)))))
                    cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
                    convex = (cross > 0) == (fl > 0) if outer else (cross > 0) != (fl > 0)
                    if not convex or w >= w_tight or not close.contains(Point(b)): continue
                    n_tight += 1; n_sharp += w < w_sharp; places.append((round(w, 1), L, round(b[0], 2), round(b[1], 2)))
    return n_sharp, n_tight, sorted(places)[:n_places]


def measure(B, R, K, gnd_layers, design_rules=None, pair_model=None, *, current_chain=None, bottlenecks=None, only_shape=False):
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    from learning import current_zones as SZ
    contour = unary_union([Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3])
    zs = [z for z in R.get('zone_list', []) if z.get('source') == 'current_zones']
    pads = [(p['net'], L, Polygon(q).buffer(0)) for p in B['pads'] for L, qs in (p.get('polys') or {}).items() for q in qs if len(q) >= 3]
    pieces, per_nl, sharp, notch, outer_area, edges_45, edges = [], {}, 0, 0, 0.0, 0.0, 0.0
    for z in zs:
        g = SZ._geo(z)
        for p in getattr(g, 'geoms', [g]):
            if p.geom_type != 'Polygon' or p.area < 1e-4: continue
            npads = sum(1 for n, L, q in pads if n == z['net'] and L == z['layer'] and p.intersects(q))
            pieces.append(dict(net_name=z['net'], layer_name=z['layer'], surface=p.area, pads=npads))
            per_nl[(z['net'], z['layer'])] = per_nl.get((z['net'], z['layer']), 0) + 1
            s, k = _angle(p); sharp += s; notch += k
            if z['layer'] in ('F', 'B'): outer_area += p.area
            for a, b in zip(list(p.exterior.coords)[:-1], list(p.exterior.coords)[1:]):
                l = math.dist(a, b); edges += l
                w = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 45
                if min(w, 45 - w) < 1.0: edges_45 += l
    fl = sorted(s['surface'] for s in pieces)
    q = lambda xs, p: xs[min(len(xs) - 1, int(p * len(xs)))] if xs else None
    try: tight = bottlenecks if bottlenecks is not None else __import__('analysis.current_bottlenecks', fromlist=['bottlenecks']).bottlenecks(B, R, K, gnd_layers)
    except Exception as e: tight = dict(unnecessary_n=None, unnecessary_mm2=None, faults=repr(e)[:120])
    try: k_sharp, k_tight, k_places = wedges(B, R, K)
    except Exception as e: k_sharp = k_tight = None; k_places = repr(e)[:120]
    from learning.zones_geometry import measure as geometry_measure
    from learning.zones_hard_checking import verify as hard_verify
    t_chk = time.process_time()
    try:
        geometry = geometry_measure(B,R,K)
    except Exception as e:
        geometry = dict(faults=repr(e), counter={})
    hard = None
    hard_details = dict(status='not_measured')
    if not only_shape:
        try:
            hard_details = hard_verify(B,R,K,gnd_layers,design_rules=design_rules)
            hard = len(hard_details['faults'])
        except Exception as e:
            hard_details = dict(passed=False, error_checking=repr(e))
    checker_cpu = time.process_time()-t_chk
    try:
        from analysis.current_chain import link_chain
        KE = current_chain if current_chain is not None else link_chain(B, R, K)
    except Exception as e: KE = dict(faults=repr(e)[:120])
    from learning.shape_features import chain_valid
    result = dict(geometry=geometry, hard_checking=hard_details, checker_cpu_s=checker_cpu,
                side_strips=geometry['counter'].get('side_strips'),
                own_net_slots=geometry['counter'].get('own_net_slot'),
                via_bites=geometry['counter'].get('via_biss'), pieces=len(pieces), z2_outer_pct=round(100 * outer_area / max(1e-6, 2 * contour.area), 2),
                z2_median_mm2=round(q(fl, 0.5), 2) if fl else None, z2_p90_mm2=round(q(fl, 0.9), 2) if fl else None,
                z2_pads_median=statistics.median([s['pads'] for s in pieces]) if pieces else None,
                z2_without_pad_pct=round(100 * sum(1 for s in pieces if s['pads'] == 0) / max(1, len(pieces)), 1),
                z3_edges45_pct=round(100 * edges_45 / max(1e-6, edges), 1),
                z4_per_net_layer=round(statistics.mean(per_nl.values()), 2) if per_nl else None,
                z4_one_piece_pct=round(100 * sum(1 for v in per_nl.values() if v == 1) / max(1, len(per_nl)), 1),
                z4_splitter=sum(1 for s in pieces if s['surface'] < 0.5),
                sharp_corners=sharp, notches=notch, wedges_45=k_sharp, wedges_80=k_tight, wedges_places=k_places, r39_unnecessary_n=tight.get('unnecessary_n'), r39_unnecessary_mm2=tight.get('unnecessary_mm2'),
                hard_violations=hard, current_chain=KE,
                chain_coverage=dict(complete=chain_valid(KE),
                                     skipped=KE.get('skipped'), net_list=len(KE.get('net_list') or {}),
                                     **({'faults': KE['faults']} if 'faults' in KE else {})),
                chain_eta=KE.get('efficiency'), chain_eta_median=KE.get('efficiency_median'),
                chain_element_max=KE.get('element_max'), copper_current_mm2=KE.get('copper_mm2'), chain_nets=KE.get('net_list'))
    if only_shape: return result
    from learning.zones_judge import diagnose
    zr = diagnose(B, R)
    result.update(zones_judge_score=zr['zones_judge_score'], zones_judge=zr)
    from learning.judge_pair import Service, components, component_occupancy, measurement_signature
    judge = Service(model=pair_model, goal='current_score') if pair_model else Service(goal='current_score')

    if (judge.model/'current_score_champion.json').is_file():
        from analysis.evaluation import rate
        try:
            E = rate(B, R, K, v3=True, minimax=False)
            z = dict(E, is_valid=bool(E['is_valid']) and hard == 0, rule=E.get('design_rules') or {},
                     current_chain=KE, v3_components=components(E['measures']), v3_components_evidence=component_occupancy(E['measures']), pair_measurement_version=measurement_signature())
            judge.apply(z)
            result['current_pair'] = z
        except Exception as e: result['current_pair'] = dict(pair_status='faults', faults=repr(e)[:160])
    else: result['current_pair'] = dict(pair_status='no_scores', pair_released=False)
    return result


def _design_rules_case(d):
    p = os.path.join(d,'design_rules.json')
    return json.load(open(p)) if os.path.exists(p) else None


def _one_case(arg):
    d, variant, params, gnd = arg
    try: os.nice(10)
    except OSError: pass
    from learning import current_zones as SZ
    t0 = time.process_time()
    B, K, R = (json.load(open(os.path.join(d, f))) for f in ('board.json', 'netclasses.json', 'routes.json'))
    gl = R.get('gnd_layers')
    R['zone_list'] = [z for z in R.get('zone_list', []) if z.get('source') not in ('current_zones', 'gnd_fill')]
    R['vias'] = [v for v in R.get('vias', []) if v.get('source') != 'gnd_fill']
    try:
        params = dict(params); module = params.pop('module', None); pair_model = params.pop('pair_model', None)
        diagnose=params.pop('zones_judge_diagnose', '0')
        R['options']=dict(R.get('options') or {}, zones_judge_diagnose=diagnose)
        judge_model=params.pop('zones_judge_model', None)
        if judge_model:R['options']=dict(R.get('options') or {}, zones_judge_model=judge_model)
        if module:
            zs = __import__('learning.current_zones_' + module, fromlist=['zone_list']).zone_list(B, R, K, gl, **params)
        else: zs = SZ.zone_list(B, R, K, gl, **params)
        from learning.current_zones_topo import skeleton_apply
        skeleton_apply(R, zs)
        R['zone_list'] += zs
        if gnd: gz, gv = SZ.gnd_fill(B, R, K, gl); R['zone_list'] += gz; R['vias'] += gv
        cpu = time.process_time() - t0
        m = measure(B, R, K, gl, design_rules=_design_rules_case(d), pair_model=pair_model); m.update(cpu_s=round(cpu, 1), cu_layers=B.get("cu_layers"))
    except Exception as e:
        m = dict(faults=repr(e)[:300])
    json.dump(R, open(os.path.join(d, 'routes_%s.json' % variant), 'w'))
    json.dump(m, open(os.path.join(d, 'mass_%s.json' % variant), 'w'), indent=1)
    return os.path.basename(d), m


def _remeasure(arg):
    d, variant = arg
    try: os.nice(10)
    except OSError: pass
    p = os.path.join(d, 'routes_%s.json' % variant)
    if not os.path.exists(p): return os.path.basename(d), dict(faults='no route')
    B, K, R = json.load(open(os.path.join(d, 'board.json'))), json.load(open(os.path.join(d, 'netclasses.json'))), json.load(open(p))
    prior = json.load(open(os.path.join(d, 'mass_%s.json' % variant))) if os.path.exists(os.path.join(d, 'mass_%s.json' % variant)) else {}
    t0 = time.process_time()
    try:
        m = measure(B, R, K, R.get('gnd_layers'), design_rules=_design_rules_case(d))
        m.update(cpu_s=prior.get('cpu_s'), measure_cpu_s=time.process_time()-t0, cu_layers=B.get('cu_layers'))
    except Exception as e: m = dict(faults=repr(e)[:300])
    json.dump(m, open(os.path.join(d, 'mass_%s.json' % variant), 'w'), indent=1)
    return os.path.basename(d), m


def remeasure(variant, par=3):
    erg = {}
    with ProcessPoolExecutor(par) as ex:
        for name, m in ex.map(_remeasure, [(d, variant) for d in cases()]): erg[name] = m
    p = os.path.join(FOLDER, 'variant_%s.json' % variant)
    V = json.load(open(p)) if os.path.exists(p) else dict(variant=variant)
    V.update(cases=erg, remeasured=time.strftime('%Y-%m-%d %H:%M')); json.dump(V, open(p, 'w'), indent=1)
    merge(variant, erg)


def _value(v):
    if v in ('0', '1') : return v == '1'
    try: return float(v) if '.' in v else int(v)
    except ValueError: return v


def compute(variant, par=3, gnd=False, **params):
    params = {k: _value(v) for k, v in params.items()}
    F = cases(); erg = {}
    with ProcessPoolExecutor(par) as ex:
        for name, m in ex.map(_one_case, [(d, variant, params, gnd) for d in F]): erg[name] = m
    json.dump(dict(variant=variant, params=params, gnd=gnd, time_s=time.strftime('%Y-%m-%d %H:%M'), cases=erg),
              open(os.path.join(FOLDER, 'variant_%s.json' % variant), 'w'), indent=1)
    merge(variant, erg)


KEY = ('chain_eta', 'chain_eta_median', 'chain_element_max', 'copper_current_mm2', 'pieces', 'z2_outer_pct', 'z2_median_mm2', 'z2_p90_mm2', 'z2_without_pad_pct', 'z3_edges45_pct', 'z4_per_net_layer',
              'z4_one_piece_pct', 'z4_splitter', 'sharp_corners', 'notches', 'wedges_45', 'wedges_80', 'r39_unnecessary_n', 'hard_violations', 'side_strips', 'own_net_slots', 'via_bites', 'checker_cpu_s', 'cpu_s', 'zones_judge_score')


def merge(variant, erg):
    counts = dict(cases=len(erg), case_error=0, geometry_error=0, geometry_unknown=0,
                  hard_check_error=0, invalid=0, status_unknown=0, passed=0,
                  complete_passed=0)
    for m in erg.values():
        geo = m.get('geometry') or {}
        hard = m.get('hard_checking') or {}
        error = 'error_checking' in hard
        valid = hard.get('passed')
        counts['case_error'] += 'faults' in m
        counts['geometry_error'] += 'faults' in geo
        counts['geometry_unknown'] += not geo
        counts['hard_check_error'] += error
        counts['invalid'] += valid is False and not error
        counts['status_unknown'] += valid is not True and valid is not False and not error
        counts['passed'] += valid is True and not error
        counts['complete_passed'] += (valid is True and not error and bool(geo)
                                            and 'faults' not in geo and 'faults' not in m)
    print('Variant %s: %d cases, %d fully passed' %
          (variant, len(erg), counts['complete_passed']))
    print('  Status (check kinds may overlap): ' +
          ', '.join('%s=%d'%(k,v) for k,v in counts.items() if k not in ('cases','complete_passed')))
    metrics = {}
    for k in KEY:
        xs = [m[k] for m in erg.values() if 'faults' not in m and type(m.get(k)) in (int,float)
              and math.isfinite(m[k])]
        metrics[k] = dict(n=len(xs), denominator=len(erg), absent=len(erg)-len(xs),
                          median=statistics.median(xs) if xs else None, total_sum=sum(xs) if xs else None)
        values = 'Median %8.2f  Total %9.1f'%(statistics.median(xs),sum(xs)) if xs else 'Median --  Total --'
        print('  %-20s %s  n=%d/%d  absent=%d'%(k,values,len(xs),len(erg),len(erg)-len(xs)))
    return dict(status=counts, metrics=metrics)


def image(d, variants, goal, width=700):
    from PIL import Image, ImageDraw
    B = json.load(open(os.path.join(d, 'board.json')))
    xs = [p[0] for o in B['outline'] for p in o]; ys = [p[1] for o in B['outline'] for p in o]
    x0, y0, x1, y1 = min(xs) - .5, min(ys) - .5, max(xs) + .5, max(ys) + .5
    S = width / (x1 - x0); W, H = int((x1 - x0) * S), int((y1 - y0) * S)
    tr = lambda p: ((p[0] - x0) * S, (p[1] - y0) * S)
    PAL = [(230, 90, 40), (40, 150, 220), (150, 80, 200), (60, 170, 90), (220, 170, 30), (200, 60, 140), (90, 120, 120), (170, 110, 60)]
    lines = []
    for v in variants:
        p = os.path.join(d, 'routes_%s.json' % v)
        if not os.path.exists(p): continue
        R = json.load(open(p)); net_list = sorted({z['net'] for z in R.get('zone_list', []) if z.get('source') == 'current_zones'})
        color = {n: PAL[i % len(PAL)] for i, n in enumerate(net_list)}
        images = []
        for L in ('F', 'B'):
            in_ = Image.new('RGB', (W, H + 18), (255, 255, 255)); dr = ImageDraw.Draw(in_, 'RGBA')
            off = lambda p: (tr(p)[0], tr(p)[1] + 18)
            for o in B['outline']: dr.polygon([off(p) for p in o], fill=(246, 246, 241), outline=(0, 0, 0))
            for z in R.get('zone_list', []):
                if z['layer'] != L: continue
                c = (205, 228, 205) if z.get('source') == 'gnd_fill' else color.get(z['net'], (200, 200, 200))
                for q in z['polys']:
                    m = Image.new('L', in_.size, 0); dm = ImageDraw.Draw(m); dm.polygon([off(p) for p in q['outer']], fill=255)
                    for h in q.get('inner') or []: dm.polygon([off(p) for p in h], fill=0)
                    in_.paste(c, (0, 0), m)
            dr = ImageDraw.Draw(in_, 'RGBA')
            for pd in B['pads']:
                for q in (pd.get('polys') or {}).get(L, []): dr.polygon([off(x) for x in q], fill=(120, 120, 120, 200))
            for t in R.get('tracks', []):
                if t['layer'] != L: continue
                c = color.get(t['net'], (60, 60, 60)); c = tuple(int(x * .6) for x in c) + (255,)
                for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']): dr.line([off(a), off(b)], fill=c, width=max(1, int(w * S)))
            for via in R.get('vias', []):
                x, y = off((via['x'], via['y'])); r = max(1.5, .225 * S); dr.ellipse([x - r, y - r, x + r, y + r], fill=(20, 20, 20))
            m = json.load(open(os.path.join(d, 'mass_%s.json' % v))) if os.path.exists(os.path.join(d, 'mass_%s.json' % v)) else {}
            for n, kv in (m.get('chain_nets') or {}).items():
                o = kv.get('element_location')
                if not o: continue
                x, y = off((o[0], o[1])); r = 9
                dr.ellipse([x - r, y - r, x + r, y + r], outline=(255, 0, 0), width=3)
                dr.text((x + r + 2, y - 6), '%s %.0f%% x%s %s' % (n, 100 * kv['efficiency'], kv['element'], o[2] if o[2] != L else ''), fill=(200, 0, 0))
            dr.text((4, 3), '%s  %s  | chain eta %s, link x%s, copper %s mm2 | pieces %s, outer %s %%, notches %s, R39 %s' % (v, L,
                    m.get('chain_eta'), m.get('chain_element_max'), m.get('copper_current_mm2'), m.get('pieces'), m.get('z2_outer_pct'),
                    m.get('notches'), m.get('r39_unnecessary_n')), fill=(0, 0, 0))
            images.append(in_)
        row = Image.new('RGB', (2 * W + 8, H + 18), (255, 255, 255)); row.paste(images[0], (0, 0)); row.paste(images[1], (W + 8, 0))
        lines.append(row)
    if not lines: return None
    out = Image.new('RGB', (lines[0].width, sum(z.height + 6 for z in lines)), (255, 255, 255)); y = 0
    for z in lines: out.paste(z, (0, y)); y += z.height + 6
    out.save(goal); return goal


def comparison(a, b, with_image=True, only=None, pair_model=None):
    for v in (a, b):
        p = os.path.join(FOLDER, 'variant_%s.json' % v)
        if os.path.exists(p): merge(v, json.load(open(p))['cases'])
    from learning.judge_pair import Service
    judge = Service(model=pair_model,goal='current_score') if pair_model else Service(goal='current_score')
    for d in cases():
        if only and os.path.basename(d) not in only: continue
        try:
            rows = [json.load(open(os.path.join(d, 'mass_%s.json' % v))).get('current_pair') or {} for v in (a,b)]
            decision = judge.comparison(*rows)
            print(os.path.basename(d), 'Current-Pair:', decision)
        except (OSError, ValueError): pass
    if not with_image: return
    os.makedirs(os.path.join(FOLDER, 'images'), exist_ok=True)
    for d in cases():
        if only and os.path.basename(d) not in only: continue
        z = image(d, [a, b], os.path.join(FOLDER, 'images', '%s_%s_%s.png' % (os.path.basename(d), a, b)))
        if z: print(z)


def main():
    a = sys.argv[1:]; kw = dict(x.split('=', 1) for x in a if '=' in x); pos = [x for x in a if '=' not in x]
    if not pos: print(__doc__); return
    if pos[0] == 'collect': collect(int(kw.get('n', '24')))
    elif pos[0] == 'compute':
        par = int(kw.pop('parallel', '3')); gnd = kw.pop('gnd', '0') == '1'
        compute(pos[1], par, gnd, **kw)
    elif pos[0] == 'measure': remeasure(pos[1], int(kw.get('parallel', '3')))
    elif pos[0] == 'comparison':
        comparison(pos[1], pos[2], kw.get('image', '1') == '1', set(kw['cases'].split(',')) if kw.get('cases') else None, kw.get('pair_model'))


if __name__ == '__main__':
    main()
