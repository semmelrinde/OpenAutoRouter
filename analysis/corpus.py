from pathlib import Path
import json, os, re, sys, subprocess, time, glob, shutil, random, zlib
from concurrent.futures import ThreadPoolExecutor
R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, R)
from common.process_guard import bootstrap
if __name__ == "__main__": bootstrap(heavy=False)
from analysis.evaluation import rate, not_connected
from analysis import density as DM

from kicad.python_call import KPY
VPY = os.path.join(R, '.venv', 'bin', 'python')


def net_classes_off(pcb):
    txt = open(pcb, encoding='utf-8', errors='replace').read(); out = []
    for m in re.finditer(r'\(net_class (\S+)(.*?)\n  \)', txt, re.S):
        b = m.group(2); w = {}
        for k in ('clearance', 'trace_width', 'via_dia', 'via_drill'):
            q = re.search(r'\(' + k + r' ([\d.]+)\)', b)
            if q: w[k] = float(q.group(1))
        nets = [n.strip('"') for n in re.findall(r'\(add_net ("[^"]*"|[^)\s]+)\)', b)]
        out.append((m.group(1).strip('"'), w, nets))
    return out


def design_rules_off(pcb, cu_layers, B=None, sigmin_uniform=False):
    NKB = (B or {}).get('net_classes') or {}
    if NKB:
        used = set((B.get('netclass') or {}).values()) | {'Default'}
        NET_COPPER = [(n, dict(clearance=k.get('clr'), trace_width=k.get('w'), via_dia=k.get('via_d'), via_drill=k.get('via_b')),
               [nt for nt, c in (B.get('netclass') or {}).items() if c == n] if n != 'Default' else ['*'])
              for n, k in NKB.items() if n in used]
    else:
        NET_COPPER = net_classes_off(pcb)
    D = next((w for n, w, _ in NET_COPPER if n == 'Default'), {})
    def amount(k, d): return D.get(k) or d
    clr, w, vd, vb = amount('clearance', 0.2), amount('trace_width', 0.25), amount('via_dia', 0.6), amount('via_drill', 0.4)
    clr = max([clr] + [k.get('clearance') or 0 for n, k, nets in NET_COPPER if nets])
    width = {nt: k['trace_width'] for n, k, nets in NET_COPPER if n != 'Default' and (k.get('trace_width') or 0) > w for nt in nets}
    ds = (B or {}).get('design') or {}
    from analysis.design_rules import dru_global, dru_rules, edge_rules, border_stroke_width
    dru, dru_conditional = dru_global(os.path.splitext(pcb)[0] + '.kicad_dru')
    border_dru=dru_rules(os.path.splitext(pcb)[0] + '.kicad_dru')
    edge_contract = edge_rules(ds.get('border') or 0.0, border_dru,border_stroke_width(pcb,border_dru))
    ds = dict(ds)
    for kind, key in (('clearance', 'min_spacing'), ('track_width', 'min_width'),
                      ('edge_clearance', 'border'), ('hole_clearance', 'hole_spacing'), ('hole_to_hole', 'hole_hole'),
                      ('hole_size', 'bore_min'), ('via_diameter', 'via_min'), ('annular_width', 'ring_min')):
        if kind in dru: ds[key] = max(ds.get(key) or 0.0, dru[kind])
    min_width = ds.get('min_width') or 0.0; clr = max(clr, ds.get('min_spacing') or 0.0)
    w = max(w, min_width); vd = max(vd, ds.get('via_min') or 0.0); vb = max(vb, ds.get('bore_min') or 0.0)



    vd = max(vd, vb + 2 * (ds.get('ring_min') or 0.0), vb + 2 * ((ds.get('hole_spacing') or 0.0) - clr))
    border = max(0.25, clr, edge_contract['edge_mm'] + 0.05)
    mask = 2 * (ds.get('mask') or 0.0) + (ds.get('mask_web') or 0.0)
    pad_spacing = round(max(0.15, mask + 0.02 if mask else 0.0), 3)
    pad_spacing_drc = round(max(0.15, clr + 0.005, mask + 0.02 if mask else 0.0), 4)
    from analysis.spacings import contract_spec
    from analysis.design_rules import dru_rules
    contract_board = B or dict(net_classes={n:dict(clr=k.get('clearance')) for n,k,_ in NET_COPPER},
                              netclass={nt:n for n,_,nets in NET_COPPER for nt in nets if nt!='*'})
    contract = contract_spec(contract_board, dru=dru_rules(os.path.splitext(pcb)[0]+'.kicad_dru'))
    return dict(spacings=contract, router=dict(cu_layers=cu_layers, drc=clr, drcpad=clr, clr=round(clr + 0.005, 4), edge=border, h2h=max(0.25, ds.get('hole_hole') or 0.0),
                            via='%s/%s' % (vd, vb), sigw=w, sigmin=w if sigmin_uniform else min(w, max(min_width, 0.15)), track_min=min_width, hole_clr=ds.get('hole_spacing') or 0.0, edge_exact='1',
                            angle45='1', vip='0'),
                placer=dict(pad_spacing=pad_spacing, pad_spacing_drc=pad_spacing_drc, border=round(max(0.2, border - pad_spacing / 2 + 0.02), 3)),
                width=width, dru_global=dru, dru_conditional=dru_conditional, edge_rules=edge_contract,
                source='Board setup, netclasses and global .kicad_dru of ' + os.path.basename(pcb))


VIA_F = 1.0
FIXED_FROM = ''
DRC_REQUIRED = True
ROUTER_EXTRA = ()
SIGMIN_UNIFORM = False
DENSITY = False
ROUTE = 1
ROUTE_ONLY_AT_ERROR = False
REPAIR = 0
CANDIDATES = 1
SELECTION = 'heuristic'
PAIR_MODEL = None
SETTINGS = ''
STEERING = ''
PLACER_STANDARD = ()
_MOD = []
def ranking(cands):
    sys.path.insert(0, R)
    from learning import model as LM
    import numpy as np
    if SELECTION == 'first': return list(cands)
    if SELECTION == 'heuristic':
        return sorted(cands, key=lambda k: k['feats']['mst'] * (1 + 0.02 * k['feats']['crossings']) * (1 + max(0.0, k['feats']['rudy_max'] - 1)))
    if not _MOD:
        import joblib; _MOD.append(joblib.load(os.path.join(R, 'learning/model/selection.joblib')))
    P, C = LM.prediction(_MOD[0], [k['feats'] for k in cands]); med = float(np.median(C))
    E = P * C + (1 - P) * 2 * med
    return [cands[i] for i in np.argsort(E, kind='stable')]


def selection(cands):
    sys.path.insert(0, R)
    from learning import model as LM
    if SELECTION == 'first': return cands[0]
    if SELECTION == 'heuristic': return LM.choice_heuristic(cands)
    if not _MOD:
        import joblib; _MOD.append(joblib.load(os.path.join(R, 'learning/model/selection.joblib')))
    return LM.choice_model(_MOD[0])(cands)
KC = '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'

SLOT_TYPES = {'courtyards_overlap', 'copper_edge_clearance', 'npth_inside_courtyard', 'pth_inside_courtyard'}


def drc_chk(pcb_src, routes, out_pcb, via, cu_layers, pinned=None, zone_list=None):
    import shutil
    try:
        from postprocessing.short_circuit_kicad_native import NativeBudget
        native = NativeBudget(300)
        if routes is None:
            shutil.copy(pcb_src, out_pcb)
            from kicad.project_copy import design_rules_save, design_rules_copy
            design_rules_copy(design_rules_save(pcb_src), out_pcb)
        else:
            zj = out_pcb + '.zones.json'; json.dump(list(zone_list or []), open(zj, 'w'))
            imp = native.run([KPY, os.path.join(R, 'kicad/import_routes.py'), pcb_src, routes, zj, out_pcb, 'cu_layers=%d' % cu_layers, 'via=' + via])
            if imp.returncode: raise RuntimeError('KiCad import failed: ' + imp.stderr[-400:])
        if routes is not None:
            from postprocessing.short_circuit_protection import protection_on
            try: r44_on = protection_on(json.load(open(routes)).get('options'))
            except Exception: r44_on = True
        if routes is not None and r44_on:
            from postprocessing.short_circuit_kicad_native import save
            guard = save(out_pcb, out_pcb, out_pcb + '.r44.json', cpu_s=max(1,300-native.spent))
            native.spent += guard['cpu_s']
            if not guard.get('checked'): return dict(faults='R44: '+guard.get('reason','unchecked'), r44=guard)
        if routes is not None:
            from postprocessing.thermal_island import finish
            def thermal_call(argv, log):
                result = native.run(list(map(str, argv)))
                Path(log).write_text(result.stdout + result.stderr)
                if result.returncode: raise RuntimeError('Thermal island: '+result.stderr[-400:])
            route_data=json.load(open(routes))
            options=route_data.get('options') or {}
            context=dict(board=Path(routes).with_name('board.json'),netclasses=options.get('netclasses',''),design_rules=options.get('design_rules',''),routes=routes)
            if not all(Path(context[k]).is_file() for k in ('board','netclasses','design_rules')): context=None
            finish(out_pcb, route_data, thermal_call, context)
        js = out_pcb + '.drc.json'
        if os.path.exists(js): os.remove(js)
        native.run([KC, 'pcb', 'drc', '--format', 'json', '--severity-error', '-o', js, out_pcb])
        geo = native.run([KPY, os.path.join(R, 'kicad/drc_geometry.py'), out_pcb, js])
        if geo.returncode: raise RuntimeError('DRC geometry failed: ' + geo.stderr[-400:])
        D = json.load(open(js))
        if routes is not None:
            import hashlib
            checked = out_pcb + '.routes_checked.json'
            native.run([KPY,os.path.join(R,'kicad/routes_export.py'),out_pcb,checked,'entire=1','unit_list=1'])
            with open(routes, 'rb') as fh: D['autorouter_input_routes_sha256'] = hashlib.sha256(fh.read()).hexdigest()
            with open(checked, 'rb') as fh: D['autorouter_routes_sha256'] = hashlib.sha256(fh.read()).hexdigest()
            with open(out_pcb, 'rb') as fh: D['autorouter_pcb_sha256'] = hashlib.sha256(fh.read()).hexdigest()
            D['autorouter_routes_file'] = os.path.basename(checked)
            D['autorouter_routes_contract'] = 'native-termination_copper-v2'
            D['autorouter_native_cpu_s'] = native.spent
            with open(js, 'w') as fh: json.dump(D, fh, indent=2)
    except Exception as e:
        return dict(faults=repr(e)[:200])
    return drc_evaluate(D, pinned)


def drc_evaluate(D, pinned=None, kicad_native=False):
    import collections
    types = collections.Counter(v['type'] for v in D['violations'])
    def routing(v): return any(re.match(r'(Track|Via|Arc) \[', i['description']) for i in v['items'])
    def part_list(v): return {m.group(1) for i in v['items'] for m in [re.search(r'(?: of |^Footprint )(\S+?)(?: on \S+)?$', i['description'])] if m}
    def slot(v):
        if routing(v): return False
        t = part_list(v)
        if pinned is not None and (not t or t <= set(pinned)): return False
        if v['type'] in SLOT_TYPES: return True
        if v['type'] in ('clearance', 'shorting_items', 'hole_clearance') and t: return True
        return v['type'] == 'solder_mask_bridge' and len(t) > 1
    pt = collections.Counter(v['type'] for v in D['violations'] if slot(v))

    ev = sorted({'%s|%s' % (v['type'], ','.join(sorted(part_list(v)))) for v in D['violations'] if slot(v) or (pinned is None and not routing(v) and part_list(v))})



    evidence = D.get('autorouter_geometry') or {}
    complete = evidence.get('version') == 2 and 'environment' in evidence and 'part_list' in evidence
    geom = []
    for v in D['violations']:
        if not slot(v): continue
        refs = part_list(v)

        proven = complete and bool(refs) and refs <= evidence['part_list'].keys()
        geom.append(json.dumps([v['type'], v.get('description', ''), sorted(
            (i['description'], round(i['pos']['x'], 6), round(i['pos']['y'], 6)) for i in v['items']),
            [evidence['environment'], sorted((ref, evidence['part_list'][ref]) for ref in refs)] if proven else None], ensure_ascii=False))
    geom.sort()


    schema = dict(version='corpus-drc-v1') if isinstance(D.get('unconnected_items'), list) else {}
    zones_copper = sum(1 for v in D['violations'] if v['type'].startswith('zone') or
                       any(i['description'].startswith('Zone ') for i in v['items']))
    return dict(**schema, **({'zones_copper': zones_copper} if kicad_native else {}), routing=sum(1 for v in D['violations'] if routing(v)), slot=sum(pt.values()), slot_types=dict(pt), slot_events=ev, slot_geometry=geom,
                slot_geometry_version=2 if complete else 0,
                not_connected=len(D.get('unconnected_items', [])), all_items=len(D['violations']), types=dict(types))


def slot_usable(d, basis):
    if not d: return 0
    if d.get('slot_geometry_version') != 2 or (basis or {}).get('slot_geometry_version') != 2:
        return d.get('slot', 0)

    from collections import Counter
    b = Counter((basis or {}).get('slot_geometry') or [])
    return sum((Counter(e for e in d['slot_geometry'] if json.loads(e)[-1] is not None) - b).values()) + sum(
        json.loads(e)[-1] is None for e in d['slot_geometry'])


def scramble(B, seed):
    import random, copy
    rnd = random.Random(seed); B = copy.deepcopy(B)
    xs = [p['x'] for p in B['pads']]; ys = [p['y'] for p in B['pads']]
    d = {}
    for f in B['fps']:
        d[f['ref']] = (rnd.uniform(min(xs), max(xs)) - f['x'], rnd.uniform(min(ys), max(ys)) - f['y'])
        f['x'] += d[f['ref']][0]; f['y'] += d[f['ref']][1]
    for p in B['pads']:
        dx, dy = d.get(p['ref'], (0, 0)); p['x'] += dx; p['y'] += dy
    return B

def shaping(R):
    import math as _m
    sharp = 0; L = L45 = 0.0
    for t in R.get('tracks', []):
        P = t['pts']
        for a, b in zip(P[:-1], P[1:]):
            l = _m.dist(a, b); L += l
            w = _m.degrees(_m.atan2(b[1] - a[1], b[0] - a[0])) % 45.0
            if min(w, 45.0 - w) < 1.0: L45 += l
        for a, b, c in zip(P[:-2], P[1:-1], P[2:]):
            u = (a[0] - b[0], a[1] - b[1]); v = (c[0] - b[0], c[1] - b[1]); nu, nv = _m.hypot(*u), _m.hypot(*v)
            if nu < 1e-6 or nv < 1e-6: continue
            if (u[0] * v[0] + u[1] * v[1]) / (nu * nv) > 1e-6: sharp += 1
    return dict(sharp_bends=sharp, share_45=round(L45 / L, 3) if L else None)


def normalized_route(B, K, Rr, gnd_layers=None):
    from analysis.measures import review_active
    if gnd_layers is not None and review_active(K):
        from analysis.copper_model import areas
        gl = [x for x in gnd_layers.split(',') if x] if isinstance(gnd_layers, str) else list(gnd_layers)
        planes, _, _ = areas(B, Rr, K, gnd_layers=gl)
        if (Rr.get('gnd_layers') == gl and (Rr.get('options') or {}).get('gnd_layers') == gl
                and Rr.get('area_nets') == sorted(set(planes.values()))): return Rr


        Rr = dict(Rr, gnd_layers=gl, options=dict(Rr.get('options') or {}, gnd_layers=gl),
                  area_nets=sorted(set(planes.values())))
    return Rr


def rate_who(B, K, f, gnd_layers=None, flow_cache=None, routes=None):
    if routes is None and not os.path.exists(f): return None
    RR = routes if routes is not None else json.load(open(f))
    from common.zones import board_geometry
    B = board_geometry(B, RR)
    Rr = normalized_route(B, K, RR, gnd_layers)
    E = rate(B, Rr, K, flow_cache=flow_cache, minimax=False)
    from analysis.measures import review_active
    if review_active(K):
        from analysis.copper_model import ground, ground_unconnected
        planes, _ = ground(B, Rr, K)
        unconn = len(set(not_connected(B, Rr, set(planes.values()), flow_cache=flow_cache, K=K)) | set(ground_unconnected(B, Rr, K)))
        ok = E['is_valid']
    else:
        unconn = len(not_connected(B, Rr, set(), flow_cache=flow_cache, K=K))
        ok = E['diagnose'] and E['conflict_list'] == 0 and E['open_items'] == 0
    return dict(is_valid=bool(ok and not unconn), not_connected=unconn,
                conflict_list=E['conflict_list'], open_items=E['open_items'], vias=E['vias'], rating=E['rating'], weighted=E['weighted'],
                length_mm=round(sum(E['total_length'].values()), 1), length_total_mm=E.get('length_total'), detour_mm=E['detour_total_sum'], **shaping(Rr))


def netclasses_run(o, bj, kj, vj, via_f=None):
    vf = VIA_F if via_f is None else float(via_f)
    subprocess.run([VPY, os.path.join(R, 'analysis/netclasses.py'), bj, kj, 'design_rules=' + os.path.join(o, 'design_rules.json'), 'specs=' + vj],
                   capture_output=True, text=True, timeout=300, env=dict(os.environ, PYTHONHASHSEED='0'))
    K = json.load(open(kj))
    if vf != 1.0:
        for k in K['net_list'].values():
            if 'via' in k: k['via'] = round(k['via'] * vf, 3)
        json.dump(K, open(kj, 'w'))
    return K


def route_run(o, bj, kj, rj, it, time_s, log, style=None, gnd_layers=None):
    from common import slots as PL
    from postprocessing.short_circuit_protection import protection_on, protection_option
    extra = ['labels=' + os.path.splitext(rj)[0] + '_labels.npz'] + list(ROUTER_EXTRA) + (['style=' + style] if style and style != 'standard' else [])



    opt = dict(x.split('=', 1) for x in extra if '=' in x)
    if opt.get('labels_budget') == '1' and 'cpu_max' not in opt:
        extra += ['cpu_max=%d' % time_s]; opt['cpu_max'] = str(time_s)
    reserve = 60 if float(opt.get('cpu_max', '0')) > 0 else 0
    r = PL.job([VPY, os.path.join(R, 'router/route_pf.py'), bj, rj, 'design_rules=' + os.path.join(o, 'design_rules.json'), 'netclasses=' + kj, 'gnd_layers=' + ','.join(gnd_layers or []),
                 'iter=%d' % it, 'pf0=1', 'pfmul=1.25', 'hinc=1.0', 'goal_from=20', 'short=1'] + extra + protection_option(protection_on(extra)), cpu_s=time_s + reserve, log=log,
                env=dict(os.environ, PYTHONHASHSEED='0'))
    rc = r['rc'] if r['cur_state'] in ('ok', 'faults') else r['cur_state']
    return rc, r['cpu_s']


def placer_command(bj, kj, pj, rg, seed, sides, learned=(), explicit=()):
    return [VPY, os.path.join(R, 'placement/placer.py'), bj, kj, pj, 'design_rules=' + rg, 'mode=new', 'chance=1', 'seed=%d' % seed,
            'sides=' + sides, 'sides_original=1'] + list(PLACER_STANDARD) + list(learned) + list(explicit)


def placer_run(cmd, time_s, log):
    from common import slots as PL
    return PL.job(cmd, cpu_s=time_s, log=log, env=dict(os.environ, PYTHONHASHSEED='0'))


def design_rules_measure(B, K, f, gnd_layers=None, area_nets=None):
    if not f or not os.path.exists(f): return None
    try:
        from analysis import rule_metrics as RK
        from learning.judge_pair import measurement_signature
        RR = json.load(open(f))
        from common.zones import board_geometry
        B = board_geometry(B, RR)
        result = RK.measure(B, RR, K, gnd_layers=gnd_layers, area_nets=area_nets, minimax=False)
        result['pair_measurement_version'] = measurement_signature()
        return result
    except Exception as e:
        return dict(faults=dict(everything=repr(e)[:200]))


def learned_settings(B, K, RG, L, placer_args=()):
    from learning.features import feats
    from learning import settings as EI
    pinned = {}
    for a in placer_args:
        k_, _, v_ = a.partition('=')
        if k_ in EI.PLACER_DIM: pinned[k_] = v_
    for a in ROUTER_EXTRA:
        k_, _, v_ = a.partition('=')
        if k_ == 'style': pinned['style'] = v_
        elif k_ == 'angle45': pinned['style'] = 'standard' if v_ == '1' else 'organic'
    if VIA_F != 1.0: pinned['via_f'] = VIA_F
    return EI.settings(feats(B, K, RG), L, fpath=STEERING or EI.PATH, spec='test_bench', pinned=pinned)


def placer_options_learned(EG):
    if not EG: return []
    from learning import settings as EI
    return EI.placer_options(EG)


def settings_short(EG):
    from learning import settings as EI
    return EI.short(EG)


def one_board(d, out, it, time_s, drc=True, auto=False, seed=1, placer_args=(), original_ref=None):
    name = os.path.basename(d.rstrip('/')); o = os.path.join(out, name); os.makedirs(o, exist_ok=True)
    pcb = os.path.join(d, 'processed.kicad_pcb')
    if not os.path.exists(pcb): pcb = sorted(glob.glob(os.path.join(d, '*.kicad_pcb')))[0]
    res = dict(board=name); t0 = time.time()
    try:
        for cmd in ([KPY, os.path.join(R, 'kicad/export_board.py'), pcb, os.path.join(o, 'board.json')],
                    [KPY, os.path.join(R, 'kicad/routes_export.py'), pcb, os.path.join(o, 'human.json')]):
            subprocess.run(cmd, capture_output=True, text=True, timeout=300)
        B = json.load(open(os.path.join(o, 'board.json'))); L = B.get('cu_layers') or 2
        RG = design_rules_off(pcb, L, B, sigmin_uniform=SIGMIN_UNIFORM); json.dump(RG, open(os.path.join(o, 'design_rules.json'), 'w'))
        vj = os.path.join(o, 'specs.json')
        p26 = DENSITY or original_ref or any(x.startswith('dense=') and float(x.split('=',1)[1]) for x in placer_args)
        specs = json.load(open(os.path.join(d, 'specs.json'))) if p26 and os.path.exists(os.path.join(d, 'specs.json')) else {}
        json.dump(dict(specs, width=RG.get('width') or {}), open(vj, 'w'))
        K = netclasses_run(o, os.path.join(o, 'board.json'), os.path.join(o, 'netclasses.json'), vj)
        EG = learned_settings(B, K, RG, L, placer_args) if SETTINGS == 'learned' else None
        style = EG['value_list']['style'] if EG and EG['provenance']['style'] == 'learned' else None
        via_f = EG['value_list']['via_f'] if EG and EG['provenance']['via_f'] == 'learned' else None
        pl_learned = placer_options_learned(EG)
        ST = dict(style=style) if style else {}
        if EG:
            res['settings'] = dict(net_class=EG['net_class'], value_list=EG['value_list'], provenance=EG['provenance'], reason=EG['reason'],
                                        steering=EG['steering'], steering_time=EG['steering_time'], n_min=EG['n_min'])
            print('%s settings %s' % (name[:40], settings_short(EG)), flush=True)
        ref = os.path.join(original_ref, 'outcome.json') if original_ref else (os.path.join(FIXED_FROM, name, 'outcome.json') if FIXED_FROM else '')
        accepted = bool(ref) and os.path.exists(ref)
        if accepted:
            R0 = json.load(open(ref))
            for k_ in ('router_rc', 'route_s', 'human', 'our'): res[k_] = R0.get(k_)
            ref_folder = os.path.dirname(ref)
            res['fixed_from'] = original_ref or FIXED_FROM
            if original_ref:
                res['fixed_original_only_baseline'] = True
                if os.path.exists(os.path.join(ref_folder, 'board.json')):
                    shutil.copy(os.path.join(ref_folder, 'board.json'), os.path.join(o, 'board_original.json'))
            for f_ in ('human.kicad_pcb.drc.json', 'our.kicad_pcb.drc.json', 'our.json'):
                if os.path.exists(os.path.join(ref_folder, f_)): shutil.copy(os.path.join(ref_folder, f_), os.path.join(o, f_))
        else:
            res['router_rc'], res['route_s'] = route_run(o, os.path.join(o, 'board.json'), os.path.join(o, 'netclasses.json'),
                                                          os.path.join(o, 'our.json'), it, time_s, os.path.join(o, 'route.log'), **ST)
            for who in ('human', 'our'):
                res[who] = rate_who(B, K, os.path.join(o, who + '.json'))
        for who in ('human', 'our'):
            if res.get(who) and not res[who].get('design_rules'):
                res[who]['design_rules'] = design_rules_measure(B, K, os.path.join(o, who + '.json'))
        if p26:
            D0 = ((res.get('human') or {}).get('density') if original_ref else None) or DM.measure(B, RG)
            for who in ('human', 'our'):
                if res.get(who): res[who]['density'] = dict(D0)
        res['net_list'] = sum(1 for n in set(p['net'] for p in B['pads'] if p['net']) if sum(1 for p in B['pads'] if p['net'] == n) > 1)
        res['pads'] = len(B['pads']); res['cu_layers'] = L; res['part_list'] = len(B['fps'])
        if drc and not accepted:
            via = RG['router']['via']
            if res['human']: res['human']['drc'] = drc_chk(pcb, None, os.path.join(o, 'human.kicad_pcb'), via, L)
            if res['our']: res['our']['drc'] = drc_chk(pcb, os.path.join(o, 'our.json'), os.path.join(o, 'our.kicad_pcb'), via, L)
        if auto:
            ta = time.time()
            json.dump(scramble(B, seed), open(os.path.join(o, 'board_dice.json'), 'w'))

            sides = ''.join(sorted({f['side'] for f in B['fps'] if f.get('n_pads')}, reverse=True)) or 'F'
            res['sides'] = sides
            netclasses_run(o, os.path.join(o, 'board_dice.json'), os.path.join(o, 'netclasses_dice.json'), vj)

            cands = []; generated = []
            for k in range(CANDIDATES):
                tag = '' if k == 0 else '_k%d' % k
                pj = os.path.join(o, 'auto_placement%s.json' % tag); bjk = os.path.join(o, 'auto_board%s.json' % tag)
                for prior in (pj, bjk):
                    if os.path.exists(prior): os.remove(prior)
                pl = placer_run(placer_command(os.path.join(o, 'board.json'), os.path.join(o, 'netclasses_dice.json'), pj,
                                               os.path.join(o, 'design_rules.json'), seed + 1000 * k, sides, pl_learned, placer_args),
                                 time_s, os.path.join(o, 'auto_placer%s.log' % tag))
                res.setdefault('auto_placer', []).append(dict(candidate=tag or 'k0', cur_state=pl['cur_state'], cpu_s=pl['cpu_s'], wall_s=pl['wall_s'], wait_s=pl['wait_s']))
                if pl['cur_state'] != 'ok' or not os.path.exists(pj): continue
                apk = os.path.join(o, 'auto%s.kicad_pcb' % tag)
                subprocess.run([KPY, os.path.join(R, 'placement/apply_place.py'), pcb, pj, apk],
                               capture_output=True, text=True, timeout=300)
                subprocess.run([KPY, os.path.join(R, 'kicad/export_board.py'), apk, bjk, 'placement=' + pj],
                               capture_output=True, text=True, timeout=300)
                if not os.path.exists(bjk): continue
                generated.append(tag)
                if CANDIDATES > 1:
                    from learning.features import feats
                    BK = json.load(open(bjk))
                    KK = netclasses_run(o, bjk, os.path.join(o, 'auto_netclasses%s.json' % tag), vj, via_f=via_f)
                    cands.append(dict(tag=tag, feats=feats(BK, KK, RG), candidate=k))
            pinned = [r for r, t in json.load(open(os.path.join(o, 'netclasses_dice.json')))['part_list'].items() if t['pinned']]
            order = [k['tag'] for k in ranking(cands)] if CANDIDATES > 1 else generated
            if CANDIDATES > 1: res['auto_candidates'] = [dict(k, rank=order.index(k['tag'])) for k in cands]
            res['auto_place_s'] = round(time.time() - ta, 1)
            attempts = []
            for tag in order[:max(1, ROUTE)]:
                ap = os.path.join(o, 'auto%s.kicad_pcb' % tag); bj = os.path.join(o, 'auto_board%s.json' % tag)
                kj = os.path.join(o, 'auto_netclasses%s.json' % tag); rj = os.path.join(o, 'auto%s.json' % tag)
                BA = json.load(open(bj)); KA = netclasses_run(o, bj, kj, vj, via_f=via_f)
                rc, ts = route_run(o, bj, kj, rj, it, time_s, os.path.join(o, 'auto_route%s.log' % tag), **ST)
                E = rate_who(BA, KA, rj); BE = BA
                if REPAIR and E and not E['is_valid']:

                    rep = os.path.join(o, 'rep%s' % tag); os.makedirs(rep, exist_ok=True); tr_ = time.time()
                    for a_, b_ in ((bj, 'board.json'), (rj, 'routes.json'), (kj, 'netclasses.json'), (os.path.join(o, 'design_rules.json'), 'design_rules.json')):
                        shutil.copy(a_, os.path.join(rep, b_))
                    try:
                        subprocess.run([VPY, os.path.join(R, 'postprocessing/improve.py'), rep, 'rounds_n=%d' % REPAIR, 'parallel=3', 'ir=0'],
                                       capture_output=True, text=True, timeout=time_s, env=dict(os.environ, PYTHONHASHSEED='0'))
                    except subprocess.TimeoutExpired:
                        pass
                    rn = os.path.join(rep, 'routes_after.json')
                    if os.path.exists(rn):
                        BN = json.load(open(os.path.join(rep, 'board_after.json'))); EN = rate_who(BN, KA, rn)
                        if EN and (EN['is_valid'] or EN['conflict_list'] + EN['open_items'] < E['conflict_list'] + E['open_items']):
                            ap2 = os.path.join(o, 'auto_rep%s.kicad_pcb' % tag)
                            subprocess.run([KPY, os.path.join(R, 'placement/apply_place.py'), ap, os.path.join(rep, 'placement_after.json'), ap2],
                                           capture_output=True, text=True, timeout=300)
                            if os.path.exists(ap2):
                                shutil.copy(rn, rj); ap = ap2; EN['repaired'] = True; E = EN; BE = BN
                                if p26: json.dump(BN, open(bj, 'w'))
                    res['auto_rep_s'] = round(res.get('auto_rep_s', 0) + time.time() - tr_, 1)
                if E:
                    E['router_rc'] = rc; E['route_s'] = ts; E['candidate'] = tag or 'k0'
                    E['design_rules'] = design_rules_measure(BE, KA, rj)
                    if p26:
                        E['density'] = DM.against(BE, D0, RG)
                        E['density_board'] = os.path.basename(bj)
                    if drc:
                        E['drc'] = drc_chk(ap, rj, os.path.join(o, 'auto_finished%s.kicad_pcb' % tag), RG['router']['via'], L, pinned)
                        E['drc']['slot_usable'] = slot_usable(E['drc'], (res.get('human') or {}).get('drc'))
                attempts.append((tag, E))
                if E and valid_whole(E, True) and len(attempts) >= 1 and ROUTE_ONLY_AT_ERROR: break
            ok = [(t_, e_) for t_, e_ in attempts if e_]
            if ok:
                tag, E = ok[0]
                for tk, ek in ok[1:]:
                    if candidate_better(E, ek): tag, E = tk, ek
                res['auto'] = E; res['auto_routed'] = [t_ or 'k0' for t_, _ in attempts]
                if tag:
                    for x in ('auto_board', 'auto_placement', 'auto_netclasses'):
                        if os.path.exists(os.path.join(o, '%s%s.json' % (x, tag))): shutil.copy(os.path.join(o, '%s%s.json' % (x, tag)), os.path.join(o, x + '.json'))
                    shutil.copy(os.path.join(o, 'auto%s.json' % tag), os.path.join(o, 'auto.json'))
                    if os.path.exists(os.path.join(o, 'auto_finished%s.kicad_pcb.drc.json' % tag)):
                        shutil.copy(os.path.join(o, 'auto_finished%s.kicad_pcb.drc.json' % tag), os.path.join(o, 'auto_finished.kicad_pcb.drc.json'))
            else:
                res['auto'] = None
    except Exception as e:
        import traceback; res['faults'] = repr(e)[:300]; open(os.path.join(o, 'errors.txt'), 'w').write(traceback.format_exc())
    try:
        from learning.ai.data import test_bench_capture
        res['_labels_seed'] = seed
        res['_labels_auto'] = [e for _, e in attempts if e] if auto and 'attempts' in locals() else []
        res['dataset'] = test_bench_capture(o, res)
    except Exception as _e: res['dataset'] = [dict(cur_state='faults', faults=repr(_e)[:200])]
    finally:
        res.pop('_labels_seed', None); res.pop('_labels_auto', None)
    res['total_s'] = round(time.time() - t0, 1)
    json.dump(res, open(os.path.join(o, 'outcome.json'), 'w'), indent=1)
    return res


def valid_whole(x, slot=False, kicad_native=False):
    if not x or not x.get('is_valid'): return False
    d = x.get('drc')
    if d is None: return not DRC_REQUIRED
    if 'faults' in d: return False
    if kicad_native and (d.get('version') != 'corpus-drc-v1' or not all(type(d.get(k)) is int and d[k] >= 0 for k in ('all_items', 'slot', 'routing', 'not_connected', 'zones_copper'))): return False
    return d['routing'] == 0 and d['not_connected'] == 0 and (not kicad_native or d.get('zones_copper') == 0) and (not slot or d.get('slot_usable', d['slot']) == 0)


def cost(x): return x['length_mm'] + 3.0 * x['vias']


def score_v3(x):
    v = ((x or {}).get('design_rules') or {}).get('score_v3')
    return float(v) if isinstance(v, (int, float)) else None


def candidate_better(a, b):
    from analysis import rule_metrics as RK
    ga, gb = valid_whole(a, True), valid_whole(b, True)
    if ga != gb:
        if not any((e or {}).get('gate_check') or ((e or {}).get('design_rules') or {}).get('measurement_version') for e in (a,b)):return gb
        from analysis import contract_spec as VT
        gate_check=VT.comparison(dict(a,geometric_valid=ga,design_rules=a.get('design_rules') or {}),
                         dict(b,geometric_valid=gb,design_rules=b.get('design_rules') or {}))
        return gb and gate_check['not_worse']
    decision = None
    if ga and gb:
        from learning.judge_pair import Service
        za = dict(a, is_valid=True, rule=a.get('design_rules') or {})
        zb = dict(b, is_valid=True, rule=b.get('design_rules') or {})
        judge = Service(model=PAIR_MODEL) if PAIR_MODEL else Service()
        try:
            judge.load_data()
            if judge.applicable(za) and judge.applicable(zb):
                decision = judge.comparison(za,zb)
                if not decision['not_worse']: return False
        except (OSError,ValueError,KeyError): pass
    if RK.assumption(a.get('design_rules'), b.get('design_rules'))['better']: return True
    if decision is not None and decision['pair']: return decision['better']
    return False


def choice_key(E):
    v = score_v3(E)
    return (not valid_whole(E, True), v is None, v if v is not None else 0.0, cost(E))


def board_value(r, who, slot=False):
    x, m = r.get(who), r.get('human')
    if not valid_whole(x, slot): return 2.0
    if not m: return 1.0
    return round(min(2.0, cost(x) / max(1e-6, cost(m))), 4)


def rule_value(r, who, slot=False):
    x = r.get(who)
    if not valid_whole(x, slot): return 2.0
    v = score_v3(x)
    if v is None: return 2.0
    m = score_v3(r.get('human'))
    if m is None: return 1.0
    return round(min(2.0, v / max(1.0, m)), 4)


def load_data(list_, goal):
    import urllib.request, urllib.parse
    L = json.load(open(list_)); missing = []
    def one(b):
        d = os.path.join(goal, b['name']); os.makedirs(d, exist_ok=True); n = 0
        for f in L['files']:
            url = 'https://raw.githubusercontent.com/PCBench/PCBench/%s/PCBs/%s/%s' % (L['sha'], urllib.parse.quote(b['pcbench']), f)
            try:
                data = urllib.request.urlopen(url, timeout=60).read(); open(os.path.join(d, f), 'wb').write(data); n += len(data)
            except Exception as e:
                missing.append((b['name'], f, repr(e)[:60]))
        return n
    with ThreadPoolExecutor(8) as ex: n = sum(ex.map(one, L['boards']))
    print('loaded: %d boards, %.1f MB to %s, %d files missing' % (len(L['boards']), n / 1e6, goal, len(missing)))
    for x in missing[:10]: print('  missing', x)


RULE_FILES = dict(human=('board.json', 'netclasses.json', 'human.json'), our=('board.json', 'netclasses.json', 'our.json'),
                     auto=('auto_board.json', 'auto_netclasses.json', 'auto.json'))


def design_rules_recount(r, o):
    for who, (bj, kj, rj) in RULE_FILES.items():
        if r.get('fixed_original_only_baseline') and who in ('human', 'our'): continue
        x = r.get(who)
        if not x or x.get('repaired') or not all(os.path.exists(os.path.join(o, f)) for f in (bj, kj, rj)): continue
        with open(os.path.join(o, bj)) as f: B = json.load(f)
        with open(os.path.join(o, kj)) as f: K = json.load(f)
        x['design_rules'] = design_rules_measure(B, K, os.path.join(o, rj))


def density_recount(r, o):
    bj = os.path.join(o, 'board.json'); rg = os.path.join(o, 'design_rules.json')
    if not os.path.exists(bj): return
    B = json.load(open(bj)); RG = json.load(open(rg)) if os.path.exists(rg) else None
    D0 = ((r.get('human') or {}).get('density') if r.get('fixed_original_only_baseline') else None) or DM.measure(B, RG)
    if r.get('fixed_original_only_baseline'):


        sources = [os.path.join(o, 'board_original.json'), os.path.join(r.get('fixed_from') or o, 'board.json')]
        for source in sources:
            if source == bj: continue
            if os.path.exists(source):
                D0 = DM.measure(json.load(open(source)), RG); break
    for who in ('human', 'our'):
        if r.get(who): r[who]['density'] = dict(D0)
    ap = os.path.join(o, 'auto_board.json')
    if (r.get('auto') or {}).get('density_board'):
        ap = os.path.join(o, r['auto']['density_board'])
    elif (r.get('auto') or {}).get('repaired'):
        tag = r['auto'].get('candidate', 'k0')

        end = os.path.join(o, 'rep' + ('' if tag == 'k0' else tag), 'board_after.json')
        if os.path.exists(end): ap = end
        else: return
    if r.get('auto') and os.path.exists(ap):
        r['auto']['density'] = DM.against(json.load(open(ap)), D0, RG)


def narrow_task(d, out, it, time_s, seed, pa, drc):
    from learning import collect as S
    name = os.path.basename(d); o = os.path.join(out, name)
    try:
        q, factor = S.variant_tight(d, o, random.Random(zlib.crc32((name + ':' + str(seed)).encode())))
        if q is None: return dict(board=name, tight_skipped='no free edge')
        r = one_board(q, os.path.join(out, 'tight'), it, time_s, drc, True, seed, pa, original_ref=o)
        r['variant'] = dict(category='tight', from_=name, factor=factor)
        json.dump(r, open(os.path.join(out, 'tight', r['board'], 'outcome.json'), 'w'), indent=1)
        return r
    except Exception as e:
        return dict(board=name, tight_error=repr(e)[:300])
    finally:

        shutil.rmtree(os.path.join(out, '_sources', name + '_tight'), ignore_errors=True)


def evaluate(out, design_rules=True):
    res = []
    for f in sorted(glob.glob(os.path.join(out, '*', 'outcome.json'))):
        r = json.load(open(f)); o = os.path.dirname(f)
        for who, file, fj in (('human', 'human.kicad_pcb.drc.json', None), ('our', 'our.kicad_pcb.drc.json', None),
                               ('auto', 'auto_finished.kicad_pcb.drc.json', 'netclasses_dice.json')):
            if r.get(who) and os.path.exists(os.path.join(o, file)):
                pinned = [x for x, t in json.load(open(os.path.join(o, fj)))['part_list'].items() if t['pinned']] if fj else None
                r[who]['drc'] = drc_evaluate(json.load(open(os.path.join(o, file))), pinned)
                if who == 'auto': r[who]['drc']['slot_usable'] = slot_usable(r[who]['drc'], (r.get('human') or {}).get('drc'))
        if design_rules: design_rules_recount(r, o)
        if DENSITY or any((r.get(w) or {}).get('density') for w in ('human','our','auto')):
            density_recount(r, o)
        json.dump(r, open(f, 'w'), indent=1); res.append(r)
    return res


def main():
    if sys.argv[1] == 'load_data': return load_data(sys.argv[2], sys.argv[3])
    if sys.argv[1] == 'table':
        res = [json.load(open(f)) for f in sorted(glob.glob(os.path.join(sys.argv[2], '*', 'outcome.json')))]
        table(res); print(json.dumps(summary(res, any(r.get('auto') for r in res)))); return
    if sys.argv[1] == 'evaluate':
        kw = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a)
        res = evaluate(sys.argv[2], design_rules=kw.get('design_rules', '1') == '1'); z = summary(res, any(r.get('auto') for r in res))
        table(res)
        json.dump(z, open(os.path.join(sys.argv[2], 'test_bench.json'), 'w'), indent=1); print(json.dumps(z, indent=1)); return
    if sys.argv[1] == 'comparison':
        kw = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
        A, Bv = (run_load(x) for x in sys.argv[2:4])
        T = gate_check(A, Bv, float(kw['tol']) if kw.get('tol') else None)
        print('\n'.join(gate_text(T, sys.argv[2], sys.argv[3])))
        if kw.get('json'): json.dump(T, open(kw['json'], 'w'), indent=1)
        return
    corpus, out = sys.argv[1], sys.argv[2]
    kw = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a)
    only = set(kw['only'].split(',')) if kw.get('only') else None
    global VIA_F, CANDIDATES, SELECTION; VIA_F = float(kw.get('via_factor', '1'))
    CANDIDATES = int(kw.get('candidates', '1')); SELECTION = kw.get('selection', 'heuristic')
    global PAIR_MODEL; PAIR_MODEL = kw.get('pair_model')
    global FIXED_FROM, REPAIR, ROUTE, ROUTE_ONLY_AT_ERROR; FIXED_FROM = kw.get('fixed_from', ''); REPAIR = int(kw.get('repair', '0'))
    global SETTINGS, STEERING; SETTINGS = kw.get('settings', ''); STEERING = kw.get('steering', '')
    if SETTINGS not in ('', 'off', 'learned'): sys.exit('settings=%s unknown (learned or off)' % SETTINGS)
    global ROUTER_EXTRA; ROUTER_EXTRA = tuple(x for x in kw.get('router', '').split(',') if x)
    global SIGMIN_UNIFORM; SIGMIN_UNIFORM = kw.get('sigmin_uniform', '0') == '1'
    ROUTE = int(kw.get('route', '1')); ROUTE_ONLY_AT_ERROR = kw.get('route_sparing', '0') == '1'
    auto = kw.get('auto', '0') == '1'; drc = kw.get('drc', '1') == '1'
    global DRC_REQUIRED; DRC_REQUIRED = drc
    pa = tuple(x for x in kw.get('placer', '').split(',') if x)
    global DENSITY; DENSITY = kw.get('tight') == '1' or kw.get('density_measure') == '1'
    dirs = sorted(d for d in glob.glob(os.path.join(corpus, '*')) if os.path.isdir(d) and (not only or os.path.basename(d) in only))
    os.makedirs(out, exist_ok=True)
    json.dump(dict(boards=[os.path.basename(d) for d in dirs], part_list=['our', 'auto'] if auto else ['our']),
              open(os.path.join(out, 'manifest.json'), 'w'), indent=1)
    with ThreadPoolExecutor(int(kw.get('parallel', '4'))) as ex:
        res = list(ex.map(lambda d: one_board(d, out, int(kw.get('iter', '60')), int(kw.get('time_s', '600')), drc, auto,
                                              int(kw.get('seed', '1')), pa), dirs))
    json.dump(res, open(os.path.join(out, 'corpus_result.json'), 'w'), indent=1)
    table(res)
    extra = summary(res, auto)
    json.dump(extra, open(os.path.join(out, 'test_bench.json'), 'w'), indent=1)
    print(json.dumps(extra, indent=1))
    if kw.get('tight', '0') == '1':
        enur = set(kw['tight_only'].split(',')) if kw.get('tight_only') else None
        edirs = [d for d in dirs if not enur or os.path.basename(d) in enur]
        with ThreadPoolExecutor(min(4, int(kw.get('parallel', '4')))) as ex:
            er = list(ex.map(lambda d: narrow_task(d, out, int(kw.get('iter', '60')), int(kw.get('time_s', '600')),
                                                  int(kw.get('seed', '1')), pa, drc), edirs))
        eo = os.path.join(out, 'tight'); os.makedirs(eo, exist_ok=True)
        json.dump(er, open(os.path.join(eo, 'corpus_result.json'), 'w'), indent=1)
        json.dump(dict(tasks=len(er), generated=sum('variant' in r for r in er),
                       auto_valid=sum(valid_whole(r.get('auto'), True) for r in er),
                       design_rules=rule_block(er, 'auto', True), density=density_block(er, 'auto', True)),
                  open(os.path.join(eo, 'test_bench.json'), 'w'), indent=1)


def table(res):
    def f(x, slot=False):
        if not x: return '-'
        dd = x.get('drc') or {}; rg = x.get('design_rules') or {}
        v = rg.get('score_v3')
        return '%s %4d %6.0f d%s v3 %s T%s' % ('yes  ' if valid_whole(x, slot) else 'NO', x['vias'], x['length_mm'],
                                              '%d/%d' % (dd.get('routing', -1), dd.get('not_connected', -1)) + ('/p%d' % dd.get('slot_usable', dd.get('slot', -1)) if slot else ''),
                                              '-' if v is None else '%.1f' % v, '-' if rg.get('topology') is None else '%g' % rg['topology'])
    print('%-34s %5s | %-40s | %-40s | %-44s' % ('Board', 'Nets', 'Human ok/Vias/mm dDRC v3 Topology', 'Ours (fixed)', 'Ours (autonomous)'))
    for r in res:
        print('%-34s %5s | %-40s | %-40s | %-44s' % (r['board'][:34], r.get('net_list'), f(r.get('human')), f(r.get('our')), f(r.get('auto'), True)))


PARTS = dict(pinned=('our', False), auto=('auto', True))


def rule_block(res, who, slot):
    from analysis import rule_metrics as RK
    n = len(res); ok = [r for r in res if valid_whole(r.get(who), slot)]
    with_ = [r for r in ok if (r[who].get('design_rules') or {}).get('score_v3') is not None]
    mm = [r for r in with_ if score_v3(r.get('human')) is not None]
    S = RK.totals([r[who]['design_rules'] for r in with_]); SM = RK.totals([r['human']['design_rules'] for r in mm])
    SW = RK.totals([r[who]['design_rules'] for r in mm])
    rel = {k: (round(SW[k] / SM[k], 3) if SW.get(k) is not None and SM.get(k) else None) for k in RK.GATE + RK.BESIDE}
    return dict(amount=round(sum(rule_value(r, who, slot) for r in res) / max(1, n), 4), valid_measured=len(with_),
                without_measurement=len(ok) - len(with_), totals=S, with_human=len(mm), totals_human=SM, totals_we_with_human=SW, rel_human=rel)


def density_block(res, who, slot=False):
    rows = [r[who]['density'] for r in res if r.get(who) and r[who].get('density')]
    valid = [r[who]['density'] for r in res if valid_whole(r.get(who), slot) and r[who].get('density')]
    def block(rows):
        return dict(measured=len(rows), totals={k: round(sum(x[k] for x in rows), 6) for k in ('board_mm2', 'occupied_mm2', 'hull_mm2')},
                    medium={k: round(sum(x[k] for x in rows if x.get(k) is not None) / max(1, sum(x.get(k) is not None for x in rows)), 6)
                            if any(x.get(k) is not None for x in rows) else None
                            for k in ('density', 'density_rel_original', 'board_mm2_rel_original', 'hull_mm2_rel_original')})
    return dict(all_items=block(rows), valid_ones=block(valid))


def summary(res, auto):
    n = len(res); z = dict(boards=n)
    z['human_valid'] = sum(valid_whole(r.get('human')) for r in res)
    z['fixed_valid'] = sum(valid_whole(r.get('our')) for r in res)
    z['fixed_value'] = round(sum(board_value(r, 'our') for r in res) / max(1, n), 4)
    ok = [r for r in res if valid_whole(r.get('our')) and r.get('human')]
    if ok:
        z['fixed_vias_rel'] = round(sum(r['our']['vias'] for r in ok) / max(1, sum(r['human']['vias'] for r in ok)), 3)
        z['fixed_length_rel'] = round(sum(r['our']['length_mm'] for r in ok) / max(1e-6, sum(r['human']['length_mm'] for r in ok)), 3)
    z['fixed_time_median_s'] = sorted(r.get('route_s') or 0 for r in res)[n // 2] if n else 0
    if auto:
        z['auto_valid'] = sum(valid_whole(r.get('auto'), True) for r in res)
        z['auto_value'] = round(sum(board_value(r, 'auto', True) for r in res) / max(1, n), 4)
        oka = [r for r in res if valid_whole(r.get('auto'), True) and r.get('human')]
        if oka:
            z['auto_vias_rel'] = round(sum(r['auto']['vias'] for r in oka) / max(1, sum(r['human']['vias'] for r in oka)), 3)
            z['auto_length_rel'] = round(sum(r['auto']['length_mm'] for r in oka) / max(1e-6, sum(r['human']['length_mm'] for r in oka)), 3)
        z['total'] = round(z['fixed_value'] + z['auto_value'], 4)

    z['design_rules'] = {t: rule_block(res, who, slot) for t, (who, slot) in PARTS.items() if t == 'pinned' or auto}
    z['fixed_rule_value'] = z['design_rules']['pinned']['amount']
    if auto:
        z['auto_rule_value'] = z['design_rules']['auto']['amount']
        z['rule_total'] = round(z['fixed_rule_value'] + z['auto_rule_value'], 4)
    if DENSITY or any((r.get(w) or {}).get('density') for r in res for w in ('human','our','auto')):
        z['density'] = {t: density_block(res, who, slot) for t, (who, slot) in PARTS.items() if t == 'pinned' or auto}


    z['rank'] = [-(z['fixed_valid'] + z.get('auto_valid', 0)), z.get('rule_total', z['fixed_rule_value']), z.get('total', z['fixed_value'])]
    return z


def run_load(o):
    data = {json.load(open(f))['board']: json.load(open(f)) for f in sorted(glob.glob(os.path.join(o, '*', 'outcome.json')))}
    m = os.path.join(o, 'manifest.json')
    if os.path.exists(m):
        manifest = json.load(open(m))
        for n in manifest['boards']:
            data.setdefault(n, dict(board=n, _missing=True))
            data[n]['_parts_expected'] = manifest['part_list']
    return data


def gate_check(A, Bv, tol_rel=None):
    from analysis import rule_metrics as RK
    names = sorted(set(A) & set(Bv))
    a = [A[n] for n in names]; b = [Bv[n] for n in names]
    auto = any(r.get('auto') for r in a + b)
    out = dict(boards=len(names), only_baseline=sorted(set(A) - set(Bv)), only_variant=sorted(set(Bv) - set(A)), part_list={})
    faults = []
    if set(A) != set(Bv) or not names: faults.append('Board set differs or is empty')
    expected = {'our', 'auto'} if auto else {'our'}
    for board_side, data in (('baseline', A), ('new', Bv)):
        for n, r in sorted(data.items()):
            for who in expected | set(r.get('_parts_expected') or []):
                if not r.get(who): faults.append('%s %s: result %s missing' % (board_side, n, who)); continue
                if valid_whole(r[who], who == 'auto'):
                    for k in RK.gate_fields(r[who].get('design_rules')):
                        if RK.amount(r[who].get('design_rules'), k) is None: faults.append('%s %s %s: %s missing/not finite' % (board_side, n, who, k))
    ga = gb = 0; bad = []; good = []
    for t, (who, slot) in PARTS.items():
        if t == 'auto' and not auto: continue
        va = [n for n in names if valid_whole(A[n].get(who), slot)]; vb = [n for n in names if valid_whole(Bv[n].get(who), slot)]
        ga += len(va); gb += len(vb)
        both = sorted(set(va) & set(vb))
        V = comparison_geo(RK, {n: A[n][who].get('design_rules') for n in both}, {n: Bv[n][who].get('design_rules') for n in both}, tol_rel)
        wa = round(sum(board_value(A[n], who, slot) for n in names) / max(1, len(names)), 4)
        wb = round(sum(board_value(Bv[n], who, slot) for n in names) / max(1, len(names)), 4)
        ra = round(sum(rule_value(A[n], who, slot) for n in names) / max(1, len(names)), 4)
        rb = round(sum(rule_value(Bv[n], who, slot) for n in names) / max(1, len(names)), 4)
        beside = {}
        for k_, f_ in (('length_mm', lambda x: x['length_mm']), ('vias', lambda x: x['vias'])):
            sa = sum(f_(A[n][who]) for n in both); sb = sum(f_(Bv[n][who]) for n in both)
            beside[k_] = dict(baseline=round(sa, 1), new=round(sb, 1), rel=round((sb - sa) / sa, 4) if sa else None)
        out['part_list'][t] = dict(valid_baseline=len(va), valid_new=len(vb), lost=sorted(set(va) - set(vb)), won=sorted(set(vb) - set(va)),
                               both=len(both), design_rules=V, rule_value=dict(baseline=ra, new=rb), old_value=dict(baseline=wa, new=wb), beside=beside)
        for k, v in V['per'].items():
            if not v or not v['gate_check']: continue
            if v['verdict'] > 0: bad.append('%s %s' % (t, k))
            elif v['verdict'] < 0: good.append('%s %s' % (t, k))
    out.update(valid_baseline=ga, valid_new=gb, worse_in=bad, better_in=good)
    out['complete'] = not faults; out['faults'] = faults
    if faults: out['verdict'] = 'incomplete'
    elif gb < ga or bad: out['verdict'] = 'worse'
    elif gb > ga or good: out['verdict'] = 'better'
    else: out['verdict'] = 'equal'
    return out


def comparison_geo(RK, A, Bv, tol_rel=None):
    return RK.comparison(A, Bv, RK.TOL_REL if tol_rel is None else tol_rel)


def gate_text(T, na='Baseline', nb='Variant'):
    o = ['Gate (H6): %s -> %s, %d common boards: valid %d -> %d; verdict: %s' % (os.path.basename(na.rstrip('/')), os.path.basename(nb.rstrip('/')),
                                                                                      T['boards'], T['valid_baseline'], T['valid_new'], T['verdict'].upper())]
    if T.get('faults'): o.append('  incomplete: ' + '; '.join(T['faults']))
    if T['worse_in']: o.append('  worse: ' + ', '.join(T['worse_in']))
    if T['better_in']: o.append('  better: ' + ', '.join(T['better_in']))
    for t, x in T['part_list'].items():
        o.append('%s: valid %d -> %d (lost %s, gained %s), valid in both %d; rule value %.4f -> %.4f, old value %.4f -> %.4f' % (
            t, x['valid_baseline'], x['valid_new'], ','.join(n[:4] for n in x['lost']) or '-', ','.join(n[:4] for n in x['won']) or '-',
            x['both'], x['rule_value']['baseline'], x['rule_value']['new'], x['old_value']['baseline'], x['old_value']['new']))
        o.append('  %-14s %4s %10s %10s %8s %7s  %s' % ('Size', 'n', 'Baseline', 'new', 'rel', 'geo', 'better/worse/equal per board, p, verdict'))
        for k, v in x['design_rules']['per'].items():
            if not v: continue
            o.append('  %-14s %4d %10.2f %10.2f %8s %7s  %d/%d/%d p %.2f/%.2f %s%s' % (k, v['n'], v['baseline'], v['new'], '-' if v['rel'] is None else '%+.1f%%' % (100 * v['rel']),
                     '-' if 'geo' not in v else '%.3f' % v['geo'], v['better'], v['worse'], v['equal'], v['p_better'], v['p_worse'],
                     {1: 'WORSE', -1: 'better', 0: 'equal'}[v['verdict']], '' if v['gate_check'] else ' (secondary metric)'))
        for k, v in x['beside'].items():
            o.append('  %-14s %4d %10.1f %10.1f %8s          (secondary measure)' % (k, x['both'], v['baseline'], v['new'], '-' if v['rel'] is None else '%+.1f%%' % (100 * v['rel'])))
    return o


if __name__ == '__main__':
    main()
