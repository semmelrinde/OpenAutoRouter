import copy
from contextlib import contextmanager
import fcntl
import json
import math
import os
import resource
from pathlib import Path
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def suggestion(B, R, K, net, grow):
    from shapely.geometry import Polygon, LineString
    from shapely.ops import unary_union
    from learning import current_zones as SZ
    from common.flow import analysis
    from common.zones import unify
    from analysis.copper_model import settings
    R = unify(R)
    gnd = set(K.get('gnd') or [])
    clr, edge = settings(K)
    outline = unary_union([Polygon(q) for q in B['outline']]).buffer(-edge)
    tracks = [t for t in R['tracks'] if t['net'] == net]
    flow = analysis(tracks, [v for v in R['vias'] if v['net'] == net],
                   [p for p in B['pads'] if p['net'] == net], K['net_list'][net].get('flow') or {})
    if not flow: return None
    bare = dict(R, zones=[z for z in R.get('zones', []) if z['net'] not in gnd])
    additions = {}
    for L in SZ._layers(B):
        if L in SZ._gnd_layers(R, None): continue
        seed = []
        for ti, t in enumerate(tracks):
            if t['layer'] != L or t.get('sig') or t.get('kelvin') or t.get('tap'): continue
            for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
                share = flow['share'].get((ti, si), 0)
                if share <= 0 or a == b: continue

                seed.append(LineString([a, b]).buffer(w/2 + grow*math.sqrt(share), quad_segs=16))
        if not seed: continue
        foreign = unary_union([g for n, g in SZ._parts(B, bare, L) if n != net])
        free = outline.difference(foreign.buffer(clr+.005)).difference(SZ.keepout_area(B, L, 'areas'))
        free = free.difference(SZ.keepout_area(B, L, 'track_list'))
        g = unary_union(seed).intersection(free)
        if not g.is_empty: additions[L] = g
    if not additions: return None
    out = copy.deepcopy(R)


    out['zones'] = [z for z in out.get('zones', [])
                    if not (z.get('source') == 'r11_corridor' and z['net'] == net
                            and z['layer'] in additions)]

    prio = 1 + max((z.get('prio', 0) for z in out.get('zones', [])), default=0)
    for L, g in additions.items():
        for p in SZ._polys(g):
            out.setdefault('zones', []).append(dict(net=net, layer=L, source='r11_corridor',
                pts=p['outer'], holes=p['inner'], filled=True, full=True, r36=True,
                prio=prio, min_thickness=.025, local_clearance=None))
    out['current_zones_model'] = 'r36-dc-v1'
    return out


def rule_gate(a, b):
    from analysis.rule_metrics import gate_fields
    keys = set(gate_fields(a['design_rules'])) | set(gate_fields(b['design_rules']))
    missing, worse = [], []
    for key in sorted(keys):
        x, y = a['design_rules'].get(key), b['design_rules'].get(key)
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in (x, y)):
            missing.append(key)
        elif y > x + 1e-8: worse.append(key)
    return dict(passed=isinstance(a.get('is_valid'), bool) and b.get('is_valid') is True and not missing and not worse,
                missing=missing, worse=worse)


def execute(bj, kj, rj, design_rules, folder, cpu_s=240):
    def consumed():
        child = resource.getrusage(resource.RUSAGE_CHILDREN)
        return time.process_time() + child.ru_utime + child.ru_stime
    cpu_start = consumed()
    from analysis.evaluation import rate
    from analysis.r11 import measure
    from common.zones import board_geometry
    from learning.copper_kicad_native import library_geometry, verify
    from learning.current_zones_post_run import physics_comparison
    from analysis.copper_model import ground_corridors
    B, K, R = [json.loads(Path(p).read_text()) for p in (bj, kj, rj)]
    B = board_geometry(B, R)




    R['current_zones_model'] = 'r36-dc-v1'
    B0 = B
    e = rate(B, R, K, v3=True)
    diag = measure(K, e['measures'], B)
    info = dict(applied=False, before=diag, attempts=[], reason='no complete source/load model')
    if diag['status'] != 'measured': return R, info
    info['reason'] = 'no passed IR/rule candidate'
    RG = json.loads(Path(design_rules).read_text()) if design_rules and Path(design_rules).exists() else None



    B, pad_info = library_geometry(Path(folder)/'library', B, R, RG)
    info['pad_source'] = pad_info
    ranked = sorted(diag['net_list'], key=lambda n: (-diag['net_list'][n]['goal_x'], -diag['net_list'][n]['drop_mV'], n))
    last_effort = 0
    for net in ranked[:2]:
        if diag['net_list'][net]['current_A'] <= 0: continue
        for growth in (.15, .35):
            rest = cpu_s - (consumed()-cpu_start)
            if rest < max(5, 1.2*last_effort):
                info['budget_exhausted'] = True
                break
            attempt_start = consumed()
            candidate = suggestion(B, R, K, net, growth)
            if candidate is None: continue
            path = Path(folder)/('%02d' % len(info['attempts']))
            path.mkdir(parents=True, exist_ok=True)
            try:
                a, b, native = verify(path, B, R, candidate, RG, cpu_s=min(120, int(rest)))
                a['native_pad_geometry'] = B['pads']
                b['native_pad_geometry'] = B['pads']
                ea, eb = [rate(B, r, K, v3=True) for r in (a, b)]
                physics = physics_comparison(B, a, B, b, K, ea['ir'], eb['ir'])
                gate = rule_gate(ea, eb)
                original_gate = rule_gate(e, eb)
                original_physics = physics_comparison(B0, R, B, b, K, e['ir'], eb['ir'])
                after = measure(K, eb['measures'], B)
                ground = ground_corridors(B, a, b, K, RG)
                improve = (after['status'] == 'measured' and after['dc_x_max'] <= diag['dc_x_max']+1e-8
                           and eb['score_v3_nets'] < ea['score_v3_nets']-1e-8)
                good = (native['suggestion']['is_valid'] and physics['passed'] and gate['passed']
                        and original_gate['passed'] and original_physics['passed']
                        and ground['passed'] and improve
                        and eb['score_v3_nets'] < e['score_v3_nets']-1e-8)
                row = dict(net_name=net, grow_mm=growth, kicad_native=native, physics=physics,
                           rule_gate=gate, input_gate=original_gate, input_physics=original_physics,
                           ground=ground, before=measure(K, ea['measures'], B), after=after,
                           score_v3_before=ea['score_v3_nets'], score_v3_after=eb['score_v3_nets'], taken=good)
                if good:
                    R, e, diag = b, eb, after
                    B0 = B
                    info.update(applied=True, reason='IR/rule gate passed')
            except Exception as exc:
                row = dict(net_name=net, grow_mm=growth, taken=False, faults=str(exc)[:500])
            info['attempts'].append(row)
            last_effort = consumed()-attempt_start
    info['after'] = diag
    return R, info


@contextmanager
def file_lock(rj):
    with Path(str(Path(rj).resolve())+'.r11.lock').open('a+b') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(lock, fcntl.LOCK_UN)


def apply(bj, kj, rj, design_rules=None, cpu_s=240):
    from common import slots as PL
    rj = str(Path(rj).resolve())
    with file_lock(rj):
        original = Path(rj).read_bytes()
    parent = Path(str(Path(rj).with_suffix(''))+'_r11')
    parent.mkdir(parents=True, exist_ok=True)
    folder = Path(tempfile.mkdtemp(prefix='run_', dir=parent))
    source, target, result = [folder/n for n in ('input_.json','candidate.json','info.json')]
    source.write_bytes(original)
    target.unlink(missing_ok=True)
    result.unlink(missing_ok=True)
    cmd = [str(ROOT/'.venv/bin/python'), '-m', 'learning.r11_post_run', bj, kj,
           str(source), design_rules or '', str(target), str(result), str(cpu_s)]
    run = PL.job(cmd, cpu_s=cpu_s, cwd=str(ROOT), log=str(folder/'worker.log'),
                  env=dict(os.environ, PYTHONHASHSEED='0'))
    info = json.loads(result.read_text()) if run['cur_state'] == 'ok' and result.exists() else dict(applied=False, reason=run['cur_state'])
    info['compute_slot'] = run
    info['workdir'] = str(folder)
    if info['applied']:
        with file_lock(rj):
            if Path(rj).read_bytes() != original:
                info.update(applied=False, reason='Input changed during post-run')
            else:
                with tempfile.NamedTemporaryFile(dir=Path(rj).parent, delete=False) as f:
                    tmp = f.name
                    f.write(target.read_bytes())
                try: os.replace(tmp, rj)
                finally:
                    if os.path.exists(tmp): os.unlink(tmp)
    result.write_text(json.dumps(info, indent=1))
    return info


if __name__ == '__main__':
    bj, kj, rj, rg, target, result, budget = sys.argv[1:]
    out, info = execute(bj, kj, rj, rg, Path(result).parent, cpu_s=float(budget))
    Path(target).write_text(json.dumps(out))
    Path(result).write_text(json.dumps(info, indent=1))
