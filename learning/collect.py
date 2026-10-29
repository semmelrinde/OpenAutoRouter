import hashlib, json, os, sys, time, random, glob, threading, subprocess
from concurrent.futures import ThreadPoolExecutor
R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, R)
from analysis import corpus as KO
from learning.features import feats



SPACE = dict(cross=[1, 2, 4], separate=[3.0], via_cost=[0.6, 1.2], fine=['1'], via_f=[1.0, 1.5])
LOCK = threading.Lock()
V2 = dict(an=False, code='')
V3 = dict(an=True)
LEVELS = dict(an=False, layers68=False, kicad_native=False)
DENSITY = False
PLACEMENT_PROTECTION = False
R11_IR = False
R36_ZONES = False
R36_SMOOTHED = False
CURRENT_MAX = 20.0


def code_state():
    h = hashlib.sha256()
    for folder in ('learning', 'router', 'placement', 'analysis', 'common', 'kicad', 'postprocessing'):
        for p in sorted(glob.glob(os.path.join(R, folder, '*.py')) + glob.glob(os.path.join(R, folder, '*.json')) +
                        (glob.glob(os.path.join(R, 'learning/ai', '*.py')) + glob.glob(os.path.join(R, 'learning/ai', '*.json')) if folder == 'learning' else []) +
                        (glob.glob(os.path.join(R, 'analysis/k20', '*.py')) if folder == 'analysis' else [])):
            h.update(os.path.relpath(p, R).encode())
            with open(p, 'rb') as f: h.update(f.read())
    return h.hexdigest()

def k19_fallback(B, K, o, L):
    import math
    gn = K.get('gnd_main') or (K.get('gnd') or [None])[0]
    mp = os.path.join(o, 'human.json')
    if L < 4 or not gn or not os.path.exists(mp): return [], []
    R = json.load(open(mp)); long = {}
    for t in R.get('tracks', []):
        long[t['layer']] = long.get(t['layer'], 0.0) + sum(math.dist(a, b) for a, b in zip(t['pts'][:-1], t['pts'][1:]))
    ges = sum(long.values()) or 1.0
    inner = ['In%d' % i for i in range(1, L - 1)]
    if sum(long.get(x, 0.0) for x in inner) / ges >= 0.05: return [], []
    cu_layers = ['In1'] if L < 6 else ['In1', 'In%d' % (L - 2)]
    zone_list = [dict(net=gn, layer=x, pts=[list(p) for p in q], holes=[], prio=0, full=False, k19_fallback=True)
             for x in cu_layers for q in B['outline'] if len(q) >= 3]
    with open(os.path.join(o, 'levels_export.log'), 'a') as f:
        f.write('K19 substitute: no original areas, inner layers on the human %.1f %% of track length -> %s = %s\n' % (
            100 * sum(long.get(x, 0.0) for x in inner) / ges, ','.join(cu_layers), gn))
    return cu_layers, zone_list


def current_follow_up(bj, kj, rj, rg, gnd_layers, zone_list=False, smoothed=False, ir=False):
    if not (zone_list or smoothed or ir) or not os.path.exists(rj): return {}
    if smoothed:
        with open(rj) as f: route = json.load(f)
        route['r36_smoothed'] = True
        with open(rj, 'w') as f: json.dump(route, f)
    out = {}
    if ir:






        from learning.current_zones import apply
        out['current_zones'] = apply(bj, kj, rj, gnd_layers, design_rules=rg)
    elif zone_list or smoothed:
        from learning.zone_planner import final
        out['current_zones'] = final(bj, kj, rj, rg, gnd_layers, mode='legacy')
    if ir:
        from learning.r11_post_run import apply
        out['r11_ir'] = apply(bj, kj, rj, rg)
    return out


def rule_measurement(B, routes, K, gnd_layers, dense=False, flow_cache=None, link_chain=False, *, v3_evaluation=None):
    from analysis.evaluation import rate
    from analysis.topology import topology
    from analysis.coupling import coupling

    KK, limited = current_limited(K)
    RR = dict(routes, gnd_layers=list(gnd_layers))
    from analysis.standard import goal
    minimax = goal(K.get('specs'), explicit=V3.get('goal')) == 'minimax'
    if v3_evaluation is not None and not minimax and not limited:
        import copy
        E = copy.deepcopy(v3_evaluation)
    else:
        E = rate(B, RR, K if minimax else KK, v3=True, minimax=minimax, flow_cache=flow_cache)
    diagnose = rate(B, RR, KK, v3=True, minimax=False, flow_cache=flow_cache) if minimax and limited else E
    if dense and 'k20' not in diagnose['design_rules']:


        from analysis.k20.metric import diagnose as k20_diagnose
        k20_diagnose(diagnose['design_rules'], B, RR, K)
    T = topology(routes, B)
    from analysis.rule_metrics import gate_fields
    from analysis.current_bottlenecks import learned_values
    from analysis.signal_integrity import short
    rt = {k: E['design_rules'].get(k) for k in gate_fields(E['design_rules'])}
    if E['score_v3_version'] == 'v3.2': rt['score_v3_version'] = 'v3.2'
    extra = dict(rule_gate=rt, **{k: E[k] for k in ('minimax', 'minimax_version', 'minimax_profile', 'minimax_rank', 'minimax_total_sum', 'minimax_current_paths',
                                    'minimax_complete', 'minimax_board_contract', 'minimax_sources', 'minimax_valid', 'minimax_status', 'chokepoint', 'safety', 'evaluation_target')}) if minimax else {}


    _N = ((diagnose.get('measures') or {}).get('net_list') or {})
    current_x = max([max(v['x'].get('dc', 0.0), v['x'].get('j', 0.0)) for v in _N.values() if v.get('net_class') in ('current_val', 'supply', 'sw')] or [0.0])
    from learning.judge_pair import components, component_occupancy, measurement_signature
    from learning.pair_contract import board_id
    chain = (E.get('measures') or {}).get('current_chain')
    if link_chain and chain is None and E['is_valid']:
        from analysis.current_chain import link_chain as measure_chain
        try: chain = measure_chain(B, RR, K)
        except Exception as e: chain = dict(status='unknown', faults=repr(e)[:160])
    return dict(board_id=board_id(B), score_v3_version=diagnose['score_v3_version'], pair_measurement_version=measurement_signature(), v3_components=components(diagnose['measures']), v3_components_evidence=component_occupancy(diagnose['measures']), current_chain=chain or dict(status='unknown'), **(dict(gate_check=E['gate_check'], measurement_version=E['gate_check'].get('measurement_version')) if 'gate_check' in E else {}), **({'_physics': dict(diagnose['measures'], not_connected=E['not_connected'], area_nets=routes.get('area_nets', []), current_limited=limited, score_v3_whole=diagnose['score_v3'])} if dense else {}), si=short(diagnose['measures']), design_rules=diagnose['design_rules'], **extra, current_x_max=round(current_x, 2),
                is_valid=E['is_valid'], score_v3=diagnose['score_v3_nets'], version=diagnose['score_v3_version'],
                topology={k: T[k] for k in ('parallel_rest_mm', 'branch_middle', 'sharp', 'tap_close_mm')},
                r27=E['design_rules'].get('r27'), coupling=coupling(routes, KK)['total_sum'],
                current_bottlenecks=learned_values(diagnose['design_rules'].get('current_bottlenecks')),
                **({'k20': diagnose['design_rules']['k20']} if 'k20' in diagnose['design_rules'] else {}),
                r39_coverage=diagnose['design_rules'].get('r39_coverage'),
                current_bottlenecks_mm2=diagnose['design_rules'].get('current_bottlenecks_mm2'))

def current_limited(K, i_max=CURRENT_MAX):
    import copy
    from analysis.measures import _current
    N = K.get('net_list') or {}
    too = sorted(n for n in N if _current(n, K)[0] > i_max and not N[n].get('current_required') and (N[n].get('flow') or {}).get('unit_source') != 'spec' and not any('spec' in str(h) for h in (N[n].get('flow') or {}).get('provenance', {}).values()))
    if not too: return K, []
    KK = dict(K, net_list=dict(N))
    for n in too:
        k = copy.deepcopy(N[n]); prior, _ = _current(n, K); factor = i_max / prior
        k['current_A'] = i_max
        k['current_A_source'] = str(k.get('current_A_source') or 'estimate') + ' (learning-limited)'
        fl = k.get('flow') or {}
        if fl.get('unit') == 'A':
            fl['total'] = i_max
            fl['load'] = {p: float(i) * factor for p, i in (fl.get('load') or {}).items()}
            fl['learning_limit'] = dict(original_A=prior, factor=factor, limit_A=i_max)
        KK['net_list'][n] = k
    return KK, too


def score_v3(B, R, K, gnd_layers=None):
    t0 = time.time()
    try:
        from analysis.evaluation import rate
        K, _too = current_limited(K)
        E = rate(B, dict(R, gnd_layers=list(gnd_layers or [])), K, v3=True)
        M = E['measures']
        top = []
        for n, v in sorted(M['net_list'].items(), key=lambda kv: (-kv[1]['contribution'], kv[0]))[:3]:
            gr = sorted(v['part_list'].items(), key=lambda kv: (-kv[1], kv[0]))[:2]
            top.append(dict(net_name=n, net_class=v['net_class'], contribution=v['contribution'],
                            reason=', '.join('%s %.2f (x %.2f)' % (k, w, v['x'].get(k, 0.0)) for k, w in gr)))
        return M['score_v3'], top, round(time.time() - t0, 3)
    except Exception as e:
        return None, [dict(faults=repr(e)[:200])], round(time.time() - t0, 3)


def v3_rel(rating, human):
    try:
        return round(float(rating) / float(human), 4) if rating is not None and human is not None and float(human) > 1e-6 else None
    except (TypeError, ValueError):
        return None


def state_from(rc, log):
    if rc in ('time_limit', 'wall_limit'): return rc, False
    if rc != 0: return 'faults', False
    try: txt = open(log, errors='replace').read()
    except OSError: txt = ''
    return ('time_limit', True) if 'Compute time budget' in txt else ('ok', False)


def conflict_history(log):
    import re
    try: return [[int(a), int(b)] for a, b in re.findall(r'Iteration \d+: conflicts (\d+), incomplete (\d+)', open(log, errors='replace').read())]
    except OSError: return []


def chance_cfg(rnd):
    return dict(seed=rnd.randint(1, 10 ** 6), **{k: rnd.choice(v) for k, v in SPACE.items()})


def kicad_native(cmd, log, cpu=300):
    from common import slots as PL
    cmd = list(cmd)
    if cmd[0] == KO.KPY: cmd[0] = '/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3'
    r = PL.job(cmd, cpu_s=cpu, log=log)
    if r['cur_state'] != 'ok': raise RuntimeError('KiCad: ' + repr(r))
    return r


def prepare(d, o):
    os.makedirs(o, exist_ok=True)
    pcb = os.path.join(d, 'processed.kicad_pcb')
    if not os.path.exists(pcb): return None
    for cmd in ([KO.KPY, os.path.join(R, 'kicad/export_board.py'), pcb, os.path.join(o, 'board.json')],
                [KO.KPY, os.path.join(R, 'kicad/routes_export.py'), pcb, os.path.join(o, 'human.json')]):
        kicad_native(cmd, os.path.join(o, os.path.basename(cmd[1]) + '.log'))
    if not os.path.exists(os.path.join(o, 'board.json')): return None
    B = json.load(open(os.path.join(o, 'board.json'))); L = B.get('cu_layers') or 2
    RG = KO.design_rules_off(pcb, L, B); json.dump(RG, open(os.path.join(o, 'design_rules.json'), 'w'))
    vj = os.path.join(o, 'specs.json')
    cons_original = os.path.join(d, 'specs.json')
    CONS = json.load(open(cons_original)) if os.path.exists(cons_original) else {}
    CONS.setdefault('width', RG.get('width') or {})
    json.dump(CONS, open(vj, 'w'))
    K = KO.netclasses_run(o, os.path.join(o, 'board.json'), os.path.join(o, 'netclasses.json'), vj)
    gnd_layers, gnd_zones = [], []
    with_levels = (LEVELS['an'] and L == 4) or (LEVELS.get('layers68') and L in (6, 8))
    if with_levels:
        from learning.levels import off_original
        source = os.path.join(LEVELS.get('originals') or d, os.path.basename(d), 'original.kicad_pcb') if LEVELS.get('originals') else pcb
        source = source if os.path.exists(source) else pcb
        gnd_layers, gnd_zones = off_original(source, B, K, o, kicad_native=LEVELS.get('kicad_native', False))
        if not gnd_layers and LEVELS.get('k19_fallback', True):
            gnd_layers, gnd_zones = k19_fallback(B, K, o, L)
    json.dump(KO.scramble(B, 1), open(os.path.join(o, 'board_dice.json'), 'w'))
    KW = KO.netclasses_run(o, os.path.join(o, 'board_dice.json'), os.path.join(o, 'netclasses_dice.json'), vj)
    movable = [r for r, t in KW['part_list'].items() if not t['pinned'] and t['role'] in ('small', 'chip')]
    if len(movable) < 3: return None
    M = KO.rate_who(B, K, os.path.join(o, 'human.json'))
    if M and LEVELS.get('kicad_native'):
        from learning.levels import original_drc
        M['drc'] = original_drc(pcb, os.path.join(o, 'human.kicad_pcb'), KO)
    elif M: M['drc'] = KO.drc_chk(pcb, None, os.path.join(o, 'human.kicad_pcb'), RG['router']['via'], L)
    sides = ''.join(sorted({f['side'] for f in B['fps'] if f.get('n_pads')}, reverse=True)) or 'F'
    metric_human = None
    if V2['an']:
        try:
            from analysis.metrics import metrics
            metric_human = metrics(json.load(open(os.path.join(o, 'human.json'))))
        except Exception: metric_human = None
    v3m = dict(rating=None, top=None, s=None)
    if V3['an']:
        try: v3m['rating'], v3m['top'], v3m['s'] = score_v3(B, json.load(open(os.path.join(o, 'human.json'))), K, gnd_layers=gnd_layers)
        except Exception as e: v3m['top'] = [dict(faults=repr(e)[:200])]
    from analysis.density import measure
    from analysis.standard import goal
    return dict(**(dict(density_original=measure(B, RG)) if DENSITY else {}), profile_data=CONS.get('profile_data', 'balanced'), evaluation_target=goal(CONS, explicit=V3.get('goal')), pcb=pcb, cu_layers=L, sides=sides, human=M, features_human=feats(B, K, RG, gnd_layers=gnd_layers), movable=len(movable),
                via=RG['router']['via'], metrics_human=metric_human, v3_human=v3m, gnd_layers=gnd_layers, gnd_zones=gnd_zones, gnd_source=source if with_levels else None,
                gnd_zones_sha256=hashlib.sha256(json.dumps(gnd_zones, sort_keys=True).encode()).hexdigest())


def variant_tight(d, o, rnd):
    import shutil as _sh
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    with open(os.path.join(o, 'board.json')) as f: B = json.load(f)
    with open(os.path.join(o, 'netclasses_dice.json')) as f: KW = json.load(f)
    if (KW.get('specs') or {}).get('contour_fixed'): return None, None
    if len(B['outline']) != 1: return None, None
    if any(B.get('outline_holes') or []): return None, None
    native_p = os.path.join(o, 'contour_native.json')
    native_python = os.path.expanduser('/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3')
    proc = subprocess.run([native_python, os.path.join(R, 'analysis/density_native.py'), os.path.join(d, 'processed.kicad_pcb'), native_p],
                          capture_output=True, text=True, timeout=300)
    if proc.returncode != 0: raise RuntimeError('Native outline check failed: ' + proc.stderr[-300:])
    with open(native_p) as f: native = json.load(f)
    if not native['tight_allowed']: return None, None
    U = unary_union([Polygon(q) for q in B['outline'] if len(q) >= 3]); x0, y0, x1, y1 = U.bounds
    hx, hy = (x1 - x0) / 2, (y1 - y0) / 2

    pinned = {r for r, t in KW['part_list'].items() if t['pinned']}
    pts = [pt for c in B.get('courtyards', []) if c['ref'] in pinned for q in c['polys'] for pt in q]
    from common.parts import load_parts
    TT = load_parts(B)
    fp = {f['ref']: f for f in B['fps']}
    pts += [pt for r in sorted(pinned & set(TT)) for g in TT[r].geo(fp[r]['x'], fp[r]['y'], 0, fp[r]['side']).values()
            for pt in ((g.bounds[0], g.bounds[1]), (g.bounds[2], g.bounds[3]))]

    pts += [pt for k in B.get('keepouts', []) if not k.get('ref') for poly in k['polys'] for pt in poly]
    pts += [pt for k in B.get('copper_graphic', []) for poly in k['polys'] for pt in poly]
    static = [Polygon(poly).buffer(0) for key in ('keepouts', 'copper_graphic')
                for k in B.get(key, []) if key == 'copper_graphic' or not k.get('ref')
                for poly in k['polys'] if len(poly) >= 3]

    vacant = dict(l=min([px - x0 for px, py in pts] + [2 * hx]), r=min([x1 - px for px, py in pts] + [2 * hx]),
                o=min([py - y0 for px, py in pts] + [2 * hy]), u=min([y1 - py for px, py in pts] + [2 * hy]))
    move = {k: max(0.0, min(v - 1.0, 0.2 * (2 * (hx if k in 'lr' else hy)))) * rnd.uniform(0.6, 1.0) for k, v in vacant.items()}
    X0, X1, Y0, Y1 = x0 + move['l'], x1 - move['r'], y0 + move['o'], y1 - move['u']
    f = (X1 - X0) * (Y1 - Y0) / ((x1 - x0) * (y1 - y0))
    if f > 0.92: return None, None



    from shapely.affinity import affine_transform
    fx, fy = (X1 - X0) / (x1 - x0), (Y1 - Y0) / (y1 - y0)
    tight_u = affine_transform(U, [fx, 0, 0, fy, X0 - fx * x0, Y0 - fy * y0])
    if any(g.difference(tight_u).area > g.difference(U).area + 1e-6 for g in static): return None, None
    for r in sorted(pinned & set(TT)):
        for g in TT[r].geo(fp[r]['x'], fp[r]['y'], 0, fp[r]['side']).values():
            if g.difference(tight_u).area > g.difference(U).area + 1e-6: return None, None
    q = os.path.join(os.path.dirname(o), '_sources', os.path.basename(o) + '_tight'); os.makedirs(q, exist_ok=True)
    target = os.path.join(q, 'processed.kicad_pcb')
    from kicad.project_copy import design_rules_save, design_rules_goal_clear, design_rules_copy
    project_rules = design_rules_save(os.path.join(d, 'processed.kicad_pcb'))
    design_rules_goal_clear(target)
    if os.path.exists(target): os.remove(target)
    proc = subprocess.run([KO.KPY, os.path.join(R, 'kicad/contour_scale.py'), os.path.join(d, 'processed.kicad_pcb'), target,
                    'box'] + ['%.4f' % v for v in (x0, y0, x1, y1, X0, Y0, X1, Y1)],
                   capture_output=True, text=True, timeout=300)
    if proc.returncode != 0: return None, None
    design_rules_copy(project_rules, target, generated_pro=True)


    probe = os.path.join(q, 'contour_chk.json')
    if os.path.exists(probe): os.remove(probe)
    proc = subprocess.run([native_python, '-c',
                           "import wx,runpy,sys; app=wx.GetApp() or wx.App(False); lz=wx.LogStderr(); wx.Log.SetActiveTarget(lz); sys.argv=sys.argv[1:]; runpy.run_path(sys.argv[0],run_name='__main__')",
                           os.path.join(R, 'kicad/export_board.py'), target, probe],
                          capture_output=True, text=True, timeout=300)
    if proc.returncode != 0 or not os.path.exists(probe):
        raise RuntimeError('Tight export failed: rc=%s %s' % (proc.returncode, proc.stderr[-500:]))
    with open(probe) as h: BN = json.load(h)
    UN = unary_union([Polygon(poly) for poly in BN['outline'] if len(poly) >= 3]).buffer(0)
    if any(g.difference(UN).area > g.difference(U).area + 1e-6 for g in static): return None, None
    fn = {p['ref']: p for p in BN['fps']}
    for r in sorted(pinned):
        if r not in fn or any(fn[r].get(k) != fp[r].get(k) for k in ('x', 'y', 'side', 'rot')): return None, None
        if r in TT:
            for g in TT[r].geo(fp[r]['x'], fp[r]['y'], 0, fp[r]['side']).values():
                if g.difference(UN).area > g.difference(U).area + 1e-6: return None, None
    f = UN.area / U.area
    if not 0 < f <= 0.92: return None, None
    if os.path.exists(os.path.join(d, 'specs.json')): _sh.copy(os.path.join(d, 'specs.json'), q)
    if os.path.exists(os.path.join(d, 'metadata.json')): _sh.copy(os.path.join(d, 'metadata.json'), q)
    return (q, round(f, 3)) if os.path.exists(os.path.join(q, 'processed.kicad_pcb')) else (None, None)


def tight_reference(info, original, source, factor):
    for key in ('human', 'v3_human', 'metrics_human', 'density_original'):
        info[key] = original.get(key)
    info['family'] = os.path.basename(source)
    info['variant'] = dict(category='tight', factor=factor, from_=os.path.basename(source))
    return info


def _candidate(d, o, info, k, cfg, it, time_s, drc, only_place=False, prepared=False):
    c = os.path.join(o, 'k%d' % k); os.makedirs(c, exist_ok=True); t0 = time.time()
    row = dict(cu_layers=info.get('cu_layers'), gnd_layers=info.get('gnd_layers', []), gnd_source=info.get('gnd_source'), gnd_zones_sha256=info.get('gnd_zones_sha256'), board=os.path.basename(o), candidate=k, cfg=cfg, movable=info['movable'], sides=info['sides'],
                 human=info['human'], features_human=info['features_human'], variant=info.get('variant'))
    if V3['an']:
        v3m = info.get('v3_human') or {}
        row.update(score_v3=None, score_v3_human=v3m.get('rating'), v3_rel=None, v3_worst=None,
                     v3_worst_human=v3m.get('top'), v3_s=None, v3_s_human=v3m.get('s'))
    from learning.families import board_family
    row['family'] = info.get('family') or board_family(d)
    if V2['an']:
        row.update(format='v2', family=row['family'], code=V2['code'], code_sha256=V2['code'] if len(V2['code']) == 64 else None,
                     router_sha256=hashlib.sha256(open(os.path.join(R, 'router/route_pf.py'), 'rb').read()).hexdigest(),
                     netclasses_sha256=hashlib.sha256(open(os.path.join(R, 'analysis/netclasses.py'), 'rb').read()).hexdigest(),
                     load=os.getloadavg()[0], budget=dict(cpu_s=time_s, iter=it),
                     metrics_human=info.get('metrics_human'), cfg_source=info.get('cfg_source', {}).get(k, 'chance'))
    try:
        if not prepared:
            args = ['seed=%d' % cfg['seed'], 'cross=%g' % cfg['cross'], 'separate=%g' % cfg['separate'], 'via_cost=%g' % cfg['via_cost'],
                    'fine=' + cfg['fine'], 'sides=' + info['sides'], 'sides_original=1']
            if cfg.get('dense'): args += ['dense=%g' % cfg['dense']]
            pcmd = [KO.VPY, os.path.join(R, 'placement/placer.py'), os.path.join(o, 'board.json'), os.path.join(o, 'netclasses_dice.json'),
                    os.path.join(c, 'placement.json'), 'design_rules=' + os.path.join(o, 'design_rules.json'), 'mode=new', 'chance=1'] + args

            from common import slots as PL
            rp = PL.job(pcmd, cpu_s=time_s, log=os.path.join(c, 'placer.log'), env=dict(os.environ, PYTHONHASHSEED='0'))
            row['placer'] = dict(cur_state=rp['cur_state'], cpu_s=rp['cpu_s'], wait_s=rp['wait_s'])
            row['placer_attempts'] = [dict(seed=cfg['seed'], cur_state=rp['cur_state'], cpu_s=rp['cpu_s'])]
            if rp['cur_state'] != 'ok':
                row['cur_state'] = 'placer_' + rp['cur_state']; raise RuntimeError('Placer: ' + rp['cur_state'])


            def collisions():
                try: return (json.load(open(os.path.join(c, 'placement.json'))).get('metrics') or {}).get('after', {}).get('collisions')
                except (OSError, ValueError): return None
            for retry in range(2 if PLACEMENT_PROTECTION else 0):
                if collisions() == 0: break
                cmd = [x if not x.startswith('seed=') else 'seed=%d' % (cfg['seed'] + 7919 * (retry + 1)) for x in pcmd]
                rp = PL.job(cmd, cpu_s=time_s, log=os.path.join(c, 'placer_repeat%d.log' % (retry + 1)), env=dict(os.environ, PYTHONHASHSEED='0'))
                row['placer']['cpu_s'] += rp['cpu_s']
                row['placer']['cur_state'] = rp['cur_state']
                row['placer_attempts'].append(dict(seed=cfg['seed'] + 7919 * (retry + 1), cur_state=rp['cur_state'], cpu_s=rp['cpu_s']))
                row['placer_repeat'] = retry + 1
                if rp['cur_state'] != 'ok': break
            if PLACEMENT_PROTECTION and (collisions() != 0 or rp['cur_state'] != 'ok'):
                row.update(is_valid=False, cur_state='placer_' + rp['cur_state'] if rp['cur_state'] != 'ok' else 'placer_collision',
                    invalid_reason='Placement collides or collision measurement missing')
                raise RuntimeError(row['invalid_reason'])
            ap = os.path.join(c, 'auto.kicad_pcb')
            kicad_native([KO.KPY, os.path.join(R, 'placement/apply_place.py'), info['pcb'], os.path.join(c, 'placement.json'), ap], os.path.join(c, 'apply.log'))
            kicad_native([KO.KPY, os.path.join(R, 'kicad/export_board.py'), ap, os.path.join(c, 'board.json'), 'placement=' + os.path.join(c, 'placement.json')], os.path.join(c, 'export.log'))
            BA = json.load(open(os.path.join(c, 'board.json')))
            KA = KO.netclasses_run(o, os.path.join(c, 'board.json'), os.path.join(c, 'netclasses.json'), os.path.join(o, 'specs.json'))
            vf = float(cfg.get('via_f', 1.0))
            if vf != 1.0:
                for k_ in KA['net_list'].values():
                    if 'via' in k_: k_['via'] = round(k_['via'] * vf, 3)
                json.dump(KA, open(os.path.join(c, 'netclasses.json'), 'w'))
            RG = json.load(open(os.path.join(o, 'design_rules.json')))
            row['place_s'] = round(time.time() - t0, 1)
            row['feats'] = dict(feats(BA, KA, RG, gnd_layers=info.get('gnd_layers', [])), via_f=vf)
            row['placer_metrics'] = (json.load(open(os.path.join(c, 'placement.json'))).get('metrics') or {}).get('after')
            row['s'] = round(time.time() - t0, 1)
            json.dump(row, open(os.path.join(c, 'prepared.json'), 'w'))
        else:
            row = json.load(open(os.path.join(c, 'prepared.json')))
            t0 -= row.get('s', 0)
            BA = json.load(open(os.path.join(c, 'board.json')))
            KA = json.load(open(os.path.join(c, 'netclasses.json')))
            RG = json.load(open(os.path.join(o, 'design_rules.json')))
            ap = os.path.join(c, 'auto.kicad_pcb')
        if DENSITY or cfg.get('dense') or info.get('density_original'):
            from analysis.density import against
            row['density'] = against(BA, info.get('density_original') or json.load(open(os.path.join(o, 'board.json'))), RG)
        if only_place: return row
        from learning.ai.data import route_context
        rc, ts = KO.route_run(route_context(o, c), os.path.join(c, 'board.json'), os.path.join(c, 'netclasses.json'), os.path.join(c, 'routes.json'), it, time_s,
                               os.path.join(c, 'route.log'), **({'gnd_layers': info.get('gnd_layers', [])} if LEVELS.get('kicad_native') else {}))
        row.update(router_rc=rc, route_s=ts, router=dict(cpu_s=ts))
        row['cur_state'], row['budget_end'] = state_from(rc, os.path.join(c, 'route.log'))
        row.update(current_follow_up(*[os.path.join(c, f) for f in ('board.json', 'netclasses.json', 'routes.json')],
                                   os.path.join(o, 'design_rules.json'), info.get('gnd_layers', []),
                                   zone_list=R36_ZONES, smoothed=R36_SMOOTHED, ir=R11_IR))
        if cfg.get('k20_zones') and os.path.exists(os.path.join(c, 'routes.json')):
            from learning.k20_post_run import apply as k20_apply
            row['k20_zones'] = k20_apply(os.path.join(c, 'board.json'), os.path.join(c, 'netclasses.json'),
                                           os.path.join(c, 'routes.json'), os.path.join(o, 'design_rules.json'),
                                           info.get('gnd_layers', []), cpu_s=time_s)
        if cfg.get('r19_post_run') and os.path.exists(os.path.join(c, 'routes.json')):
            from learning.r19_post_run import apply as r19_apply
            row['r19_post_run'] = r19_apply(os.path.join(c, 'board.json'), os.path.join(c, 'netclasses.json'),
                                               os.path.join(c, 'routes.json'), os.path.join(o, 'design_rules.json'),
                                               info.get('gnd_layers', []))
        from postprocessing.short_circuit_protection import candidate_save_measurement
        r44_reason, row['r44'] = candidate_save_measurement(os.path.join(c, 'board.json'), os.path.join(c, 'routes.json'))
        if r44_reason:
            row.update(is_valid=False, cur_state='r44_unchecked', invalid_reason=r44_reason[:300], outcome=None,
                         s=round(time.time() - t0, 1))
            return row
        E = KO.rate_who(BA, KA, os.path.join(c, 'routes.json'))
        if V2['an']:
            row['cur_state'], row['budget_end'] = state_from(rc, os.path.join(c, 'route.log'))
            row['conflict_history'] = conflict_history(os.path.join(c, 'route.log'))
            if E:
                try:
                    from analysis.metrics import metrics
                    row['metrics'] = metrics(json.load(open(os.path.join(c, 'routes.json'))))
                except Exception as e_: row['metrics_error'] = repr(e_)[:200]
        if V3['an']:
            n3, top3, s3 = None, None, 0.0
            if E:
                try: n3, top3, s3 = score_v3(BA, json.load(open(os.path.join(c, 'routes.json'))), KA, gnd_layers=info.get('gnd_layers', []))
                except Exception as e_: top3 = [dict(faults=repr(e_)[:200])]
            row.update(score_v3=n3, v3_rel=v3_rel(n3, row.get('score_v3_human')), v3_worst=top3, v3_s=s3,
                         v3_current_limited=current_limited(KA)[1][:10] or None)
        pinned = [r for r, t in json.load(open(os.path.join(o, 'netclasses_dice.json')))['part_list'].items() if t['pinned']]
        if E and info.get('gnd_layers'):
            from analysis.evaluation import rate
            E['is_valid'] = rate(BA, json.load(open(os.path.join(c, 'routes.json'))), KA, minimax=False)['is_valid']
            E['not_connected'] = len(__import__('analysis.evaluation', fromlist=['not_connected']).not_connected(BA, json.load(open(os.path.join(c, 'routes.json'))), set(KA['gnd'])))
        native_routes = None
        if E and drc:
            if LEVELS.get('kicad_native') or info.get('gnd_layers'):
                from learning.levels import drc as levels_drc
                E['drc'] = levels_drc(ap, os.path.join(c, 'routes.json'), os.path.join(c, 'finished.kicad_pcb'), info, RG, pinned, KO, kicad_native=LEVELS.get('kicad_native', False))
            else:
                rv = json.load(open(os.path.join(c, 'routes.json'))).get('options', {}).get('via', info['via'])
                E['drc'] = KO.drc_chk(ap, os.path.join(c, 'routes.json'), os.path.join(c, 'finished.kicad_pcb'), rv, info['cu_layers'], pinned)
            E['drc']['slot_usable'] = KO.slot_usable(E['drc'], (info.get('human') or {}).get('drc'))
            if LEVELS.get('kicad_native') and info['cu_layers'] in (6, 8):
                from analysis.evaluation import native_areas, rate
                RR = dict(json.load(open(os.path.join(c, 'routes.json'))), drc=E['drc'])
                RR['native_areas'] = dict(version='native-areas-v1', net_list=sorted({n for q in info.get('gnd_layers', []) if '=' in q for n in q.split('=', 1)[1].split('|')}))
                if native_areas(RR, KA):
                    native_routes = RR
                    checked = rate(BA, RR, KA, minimax=False)
                    E['not_connected'], E['is_valid'] = len(checked['not_connected']), checked['is_valid']
        row.update(router_rc=rc, route_s=ts, router=dict(cpu_s=ts), outcome=E, is_valid=KO.valid_whole(E, True, kicad_native=LEVELS.get('kicad_native', False)) if E else False)
        if E:
            row['cost'] = KO.cost(E)
            if V3['an']:
                try:
                    row['rule'] = rule_measurement(BA, native_routes or dict(json.load(open(os.path.join(c, 'routes.json'))), **({'drc': E.get('drc')} if drc else {})), KA, info.get('gnd_layers', []), dense=True)
                    if 'gate_check' in row['rule']:
                        row['gate_check'] = row['rule']['gate_check']
                    row['evaluation_version'] = row['rule']['minimax_version'] if row['rule'].get('evaluation_target', 'v3') == 'minimax' else row['rule']['version']
                    if row['rule'].get('evaluation_target', 'v3') == 'minimax':
                        row['valid_routing'] = row['is_valid']
                        row['is_valid'] = (row['is_valid'] if LEVELS.get('kicad_native') else E['is_valid']) and row['rule']['minimax_valid']


                    if LEVELS.get('kicad_native'): row['rule']['is_valid'] = row['is_valid']
                except Exception as e_:
                    row['rule_error'] = repr(e_)[:300]
                    if LEVELS.get('kicad_native'): row['is_valid'] = False
    except Exception as e:
        row['faults'] = repr(e)[:300]
        if V2['an']: row.setdefault('cur_state', 'faults')
    if R36_ZONES or R36_SMOOTHED:
        from learning.zone_planner import acceptance_gate
        row.setdefault('is_valid', False)
        acceptance_gate(row, row.get('current_zones'), 'legacy')
    row['s'] = round(time.time() - t0, 1)
    return row


def candidate(d, o, info, k, cfg, it, time_s, drc, only_place=False, prepared=False, before_intake=None):
    tc = time.thread_time()
    before = code_state()
    z = _candidate(d, o, info, k, cfg, it, time_s, drc, only_place, prepared)
    if before_intake is not None: z = before_intake(z)
    after = code_state()
    z.setdefault('code_sha256', before)
    if before != after or (V2['an'] and V2['code'] != after) or z.get('code_sha256') != after:
        z.update(code_changed=True, code_after=after)
    from learning.learning_effect import complete, children_cpu_s
    complete(z, os.path.join(o, 'k%d' % k))
    try:
        from learning.ai.data import capture
        metrics = (z.get('rule') or {}).pop('_physics', None)
        z.setdefault('family', info.get('family') or os.path.basename(o).removesuffix('_tight'))
        z['dataset'] = capture(os.path.join(o, 'k%d' % k), z, os.path.join(o, 'board.json'),
                                  provenance=info.get('labels_provenance', 'endurance_run'), metrics=metrics)
    except Exception as e: z['dataset'] = dict(cur_state='faults', faults=repr(e)[:200])
    z['cpu_total'] = children_cpu_s(z) + time.thread_time() - tc
    z['cpu_measurement'] = 'partially'
    return z


def main():
    tr, out = sys.argv[1], sys.argv[2]
    kw = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a)
    K_ = int(kw.get('candidates', '4')); P = int(kw.get('parallel', '8')); time_s = int(kw.get('time_s', '240')); it = int(kw.get('iter', '40'))
    maxp = int(kw.get('max_pads', '220')); nb = int(kw.get('boards', '60')); drc = kw.get('drc', '1') == '1'
    tight = kw.get('tight', '0') == '1'
    global DENSITY; DENSITY = tight or kw.get('density_measure') == '1'
    global PLACEMENT_PROTECTION; PLACEMENT_PROTECTION = kw.get('placement_protection', '0') == '1'
    global R11_IR; R11_IR = kw.get('r11_ir', '0') == '1'
    global R36_ZONES, R36_SMOOTHED
    R36_ZONES = kw.get('r36_zones', '0') == '1'
    R36_SMOOTHED = kw.get('r36_smoothed', '0') == '1'
    data = kw.get('data', os.path.join(out, 'attempts.jsonl')); rnd = random.Random(int(kw.get('seed', '1')))
    V2['an'] = kw.get('v2', '0') == '1'; V3['an'] = kw.get('v3', '1') == '1'
    if V2['an']:
        KO.ROUTER_EXTRA = tuple(KO.ROUTER_EXTRA) + ('cpu_max=%d' % time_s,)
        try: V2['code'] = code_state()
        except Exception: V2['code'] = ''
    for k_ in list(SPACE):
        if k_ in kw: SPACE[k_] = [kw[k_] if k_ == 'fine' else float(kw[k_])]
    cfg_prev = {}
    if kw.get('cfg_from'):
        for z_ in open(kw['cfg_from']):
            try: j_ = json.loads(z_); cfg_prev[(j_['board'], j_['candidate'])] = j_['cfg']
            except Exception: pass
    end = time.time() + float(kw['duration']) if kw.get('duration') else None; started = set()
    os.makedirs(out, exist_ok=True); os.makedirs(os.path.dirname(os.path.abspath(data)), exist_ok=True)
    dirs = sorted(d for d in glob.glob(os.path.join(tr, '*')) if os.path.isdir(d))
    rnd.shuffle(dirs)
    finished = set()
    if os.path.exists(data):
        for z in open(data):
            try: j = json.loads(z); finished.add((j['board'], j['candidate']))
            except Exception: pass
    t0 = time.time(); n_ok = 0

    def board(d):
        o = os.path.join(out, os.path.basename(d))
        try:
            meta = json.load(open(os.path.join(d, 'metadata.json')))
            if int(meta.get('layers') or 2) != 2: return []
            txt = open(os.path.join(d, 'processed.kicad_pcb'), errors='replace').read()
            if txt.count('(pad ') > maxp * 1.15: return []
            info = prepare(d, o)
        except Exception:
            return []
        if not info or info['features_human']['n_pads'] > maxp: return []
        from learning.families import board_family
        info['family'] = board_family(d)
        jobs = [(d, o, info, k) for k in range(K_)]
        if tight:
            try:
                q, f = variant_tight(d, o, random.Random(__import__("zlib").crc32(os.path.basename(d).encode())))
                if q:
                    o2 = os.path.join(out, os.path.basename(q)); info2 = prepare(q, o2)
                    if info2:
                        tight_reference(info2, info, d, f)
                        jobs += [(q, o2, info2, k) for k in range(K_)]
            except Exception as e:
                print('Variant tight', os.path.basename(d), repr(e)[:120], flush=True)
        return jobs

    with ThreadPoolExecutor(P) as ex:
        jobs = [j for js in ex.map(board, dirs[:nb]) for j in js]
    print('Tasks: %d boards, %d candidates (%.0fs preparation)' % (len({j[0] for j in jobs}), len(jobs), time.time() - t0), flush=True)
    cfgs = {(d, k): chance_cfg(rnd) for d, o, info, k in jobs}
    for d, o, info, k in jobs:
        if (os.path.basename(d), k) in cfg_prev:
            cfgs[(d, k)] = dict(cfg_prev[(os.path.basename(d), k)]); info.setdefault('cfg_source', {})[k] = 'cfg_from'
        if kw.get('k20_zones', '0') == '1': cfgs[(d, k)]['k20_zones'] = True
        if kw.get('r19_post_run', '0') == '1': cfgs[(d, k)]['r19_post_run'] = True

    def job(j):
        d, o, info, k = j
        if (os.path.basename(d), k) in finished: return None
        with LOCK:
            if end and time.time() > end and o not in started: return None
            started.add(o)
        z = candidate(d, o, info, k, cfgs[(d, k)], it, time_s, drc)
        with LOCK:
            open(data, 'a').write(json.dumps(z) + '\n')
        if kw.get('tidy_up', '1') == '1':
            for f in ('auto.kicad_pcb', 'finished.kicad_pcb', 'finished.kicad_pro', 'auto.kicad_pro'):
                try: os.remove(os.path.join(o, 'k%d' % k, f))
                except OSError: pass
        return z
    with ThreadPoolExecutor(P) as ex:
        for z in ex.map(job, jobs):
            if z is None: continue
            n_ok += bool(z.get('is_valid'))
            print('%-40s k%d %s cost %s  %4.0fs  (valid so far %d)' % (z['board'][:40], z['candidate'], 'yes  ' if z.get('is_valid') else 'no',
                  z.get('cost'), z['s'], n_ok), flush=True)
    print('finished in %.0fs -> %s' % (time.time() - t0, data))


if __name__ == '__main__':
    main()
