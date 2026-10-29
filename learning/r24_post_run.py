import json
import math
import os
import shutil
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def apply(bj, kj, rj, design_rules, *, cpu_s=180):
    from common import slots as PL
    original = Path(rj).read_bytes()
    artifact=Path(rj[:-5]+'_r24_checking')
    artifact.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='autorouter-r24-') as td:
        candidate, result = Path(td)/'routes.json', Path(td)/'info.json'
        candidate.write_bytes(original)
        run = PL.job([str(ROOT/'.venv/bin/python'), str(Path(__file__)), bj, kj, str(candidate),
                       design_rules, str(result)], cpu_s=cpu_s, cwd=str(ROOT),
                      log=str(Path(td)/'worker.log'), env=dict(os.environ, PYTHONHASHSEED='0'))
        info = json.loads(result.read_text()) if run['cur_state']=='ok' and result.exists() else dict(
            applied=False, reason='R24-Worker: '+run['cur_state'])
        info['compute_slot'] = run
        info['artifact'] = str(artifact)
        shutil.copytree(td, artifact, dirs_exist_ok=True)
        if info.get('applied'):
            if Path(rj).read_bytes() != original:
                return dict(info, applied=False, reason='Route changed during follow-up')
            with tempfile.NamedTemporaryFile(dir=Path(rj).parent, prefix='.r24_', delete=False) as f:
                tmp = Path(f.name); f.write(candidate.read_bytes())
            try:
                Path(rj[:-5]+'_before_r24.json').write_bytes(original)
                os.replace(tmp, rj)
            finally: tmp.unlink(missing_ok=True)
        return info


def r24_preserved(a, b):
    keys=('dead_mm', 'free_mm', 'visible_bends', 'hairpins', 'zigzag')
    return bool(a and b and all(isinstance(m.get(k),(int,float)) and not isinstance(m[k],bool)
        and math.isfinite(m[k]) for m in (a,b) for k in keys) and all(b[k]<=a[k] for k in keys))


def model_check(B, old, after, K):
    from learning.collect import rule_measurement
    from analysis.rule_metrics import assumption
    gnd_layers = old.get('gnd_layers') or []
    m0, m1 = [rule_measurement(B, q, K, gnd_layers, dense=True) for q in (old, after)]
    gate = assumption(m0.get('rule_gate') or m0['design_rules'], m1.get('rule_gate') or m1['design_rules'])

    notes = [m.get('score_v3') for m in (m0, m1)]
    known = all(isinstance(n, (int, float)) and not isinstance(n, bool) and math.isfinite(n) for n in notes)
    score_ok = known and notes[1] <= notes[0] + 1e-9
    gate.update(valid_before=m0['is_valid'], valid_after=m1['is_valid'],
                learning_score_before=notes[0], learning_score_after=notes[1], learning_score_preserved=score_ok)
    gate['not_worse'] = gate['not_worse'] and m1['is_valid'] and score_ok
    if not m1['is_valid']: gate['reason'] = 'Learning model invalid'
    elif not known: gate['reason'] = 'Learning score unknown'
    elif not score_ok: gate['reason'] = 'Learning score worsened'
    from learning.rank_model import is_valid
    model_valid = is_valid(dict(is_valid=m1['is_valid'], rule=m1))
    gate['not_worse'] = bool(gate['not_worse'] and model_valid)
    if not model_valid: gate['reason'] = 'Learning model/target contract invalid'
    return gate


def _apply(bj, kj, rj, design_rules, folder):
    from postprocessing.r24 import suggestion, measure
    from learning.copper_kicad_native import library_geometry, verify, run, KC
    from analysis.evaluation import rate
    from analysis.rule_metrics import assumption
    B, K, old, RG = [json.loads(Path(p).read_text()) for p in (bj, kj, rj, design_rules)]
    model_board = B

    B, binfo = library_geometry(Path(folder)/'library', B, old, RG)
    new, info = suggestion(B, old, K)
    info.update(applied=False, library=binfo)
    if not info['dead_pieces'] and not info['shortenable_chains']:
        info['reason'] = 'No safe geometric proposal'; return info
    if new.get('vias') != old.get('vias'):
        raise ValueError('R24 changes vias')
    before, after, native = verify(Path(folder)/'kicad_native', B, old, new, RG)
    info['kicad_native'] = native


    drc = Path(folder)/'loose.json'
    run([KC, 'pcb', 'drc', '--format', 'json', '--severity-all', '--all-track-errors',
         '-o', str(drc), str(Path(folder)/'kicad_native/suggestion.kicad_pcb')], Path(folder)/'loose.log')
    all_drc = json.loads(drc.read_text())
    loose = sum(v['type'] in ('track_dangling','via_dangling') for v in all_drc['violations'])
    info['loose'] = loose
    if not native['suggestion']['is_valid'] or loose:
        info['reason'] = 'Native DRC blocks (errors/open/loose)'; return info

    e0, e1 = [rate(B, q, K, v3=True) for q in (before, after)]
    gate = assumption(e0['design_rules'], e1['design_rules'])
    info['rule_gate'] = gate



    model_gate = model_check(model_board, old, after, K)
    info['model_check'] = model_gate
    info['valid_before'], info['valid_after'] = e0['is_valid'], e1['is_valid']
    from analysis.length_impedance import length_list, receiver_protection, receiver_netclasses
    lk = receiver_netclasses(K)
    info['receiver_gate'] = receiver_protection(length_list(B,before,lk), length_list(B,after,lk))

    r0, r1 = (measure(B, q, K, search=True) for q in (before, after))
    good_r24 = r24_preserved(r0, r1)
    info['r24_preserved'] = good_r24
    if not (e1['is_valid'] and gate['not_worse'] and model_gate['not_worse'] and info['receiver_gate'] and good_r24):
        info['reason'] = 'Rule/receiver/R24 gate blocks'; return info
    info.update(applied=True, reason='Native DRC 0/0/0 and rule gate passed')
    after['r24_post_run'] = {k:v for k,v in info.items() if k not in ('rule_gate', 'model_check')}
    Path(rj).write_text(json.dumps(after))
    return info

if __name__ == '__main__':
    bj, kj, rj, rg, result = sys.argv[1:]
    try:
        td=Path(result).parent/'native'
        td.mkdir(exist_ok=True)
        info = _apply(bj,kj,rj,rg,str(td))
    except Exception as exc:
        info = dict(applied=False, reason='R24 check failed', faults=repr(exc))
    Path(result).write_text(json.dumps(info, allow_nan=False))
