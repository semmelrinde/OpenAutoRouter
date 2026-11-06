from __future__ import annotations

import ast
import hashlib
import json
import math
import random
from pathlib import Path


SEED_MIN, SEED_MAX = 20_000_000, 99_999_999
FILES = frozenset({'router/pf_head.py', 'router/pf_body.py', 'router/pf_main.py'})


def seeds(run_seed: int, round_no: int, count: int, holdout: int) -> tuple[list[int], list[int]]:
    if not 0 <= run_seed < 1000 or not 0 <= round_no < 80 or not 0 < count <= 1000 or not 0 < holdout <= 1000 or count + holdout > 1000:
        raise ValueError('invalid run seed, round or scope')
    start = SEED_MIN + (run_seed * 80 + round_no) * 1000
    value_list = random.Random((run_seed << 16) | round_no).sample(range(start, start + 1000), count + holdout)
    assert max(value_list) <= SEED_MAX
    return value_list[:count], value_list[count:]


def patch_range(source_text: str, name: str, content: str) -> str:
    if not name.isidentifier():
        raise ValueError('invalid marker name')
    begin, end = f'# EVOLUTION:{name}:BEGIN', f'# EVOLUTION:{name}:END'
    lines = source_text.splitlines(keepends=True)
    a = [i for i, z in enumerate(lines) if z.strip() == begin]
    b = [i for i, z in enumerate(lines) if z.strip() == end]
    if len(a) != 1 or len(b) != 1 or a[0] >= b[0]:
        raise ValueError('Markers missing, duplicated or swapped')
    new = ''.join(lines[:a[0] + 1]) + content.rstrip('\n') + '\n' + ''.join(lines[b[0]:])
    ast.parse(new)
    return new


def file_hash(fpath: Path) -> str:
    return hashlib.sha256(fpath.read_bytes()).hexdigest()


def archive(folder: Path, id_: str, entry: dict) -> Path:
    if not id_ or not all(c.isascii() and (c.isalnum() or c in '-_') for c in id_):
        raise ValueError('invalid identifier')
    folder.mkdir(parents=True, exist_ok=True)
    fpath = folder / (id_ + '.json')
    with fpath.open('x', encoding='utf-8') as f:
        json.dump(entry, f, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False)
        f.write('\n')
    return fpath


def pair_pre_gate(basis: list[dict], candidate: list[dict]) -> dict:
    def index(lines):
        out = {}
        for z in lines:
            ident = z.get('seed')
            if ident is None or ident in out or type(z.get('is_valid')) is not bool:
                raise ValueError('missing/duplicated task or unresolved validity')
            for key in ('remaining_conflicts', 'open_items'):
                value = z.get(key)
                if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value < 0:
                    raise ValueError('missing or invalid error value')
            out[ident] = z
        return out
    a, b = index(basis), index(candidate)
    if not a or set(a) != set(b):
        raise ValueError('incomplete paired tasks')
    lost = sorted(s for s in a if a[s]['is_valid'] and not b[s]['is_valid'])
    def totals(d):
        return dict(is_valid=sum(z['is_valid'] for z in d.values()),
                    remaining_conflicts=sum(z['remaining_conflicts'] for z in d.values()),
                    open_items=sum(z['open_items'] for z in d.values()))
    x, y = totals(a), totals(b)
    hard = (y['is_valid'], -y['remaining_conflicts'], -y['open_items']) >= (x['is_valid'], -x['remaining_conflicts'], -x['open_items'])
    return dict(pre_gate=not lost and hard, lost=lost, basis=x, candidate=y,
                quality='not_checked', release=False)





MARKER = {
    'cost_function': ('router/pf_main.py', "        return (grp(u)[0], learn_series_value(u,rng.border()))\n"),
    'rip_up': ('router/pf_main.py', "        series = units if all_items else sorted(todo)\n"),
    'presence_hist': ('router/pf_main.py', "    PF[0] *= float(args.get('pfmul', '1.5'))\n"),
    'stall': ('router/pf_main.py', "    if FAIL_STILL and not conf and fail:\n        _fs[0] = _fs[0] + 1 if fail == _fs[1] else 1; _fs[1] = set(fail)\n        if _fs[0] >= FAIL_STILL:\n            print('Negotiation ends: no conflicts, %d units unchanged without a route for %d passes' % (len(fail), _fs[0]), flush=True); break\n    else: _fs[0] = 0\n"),
    'via_cost': ('router/pf_head.py', "        _via = float(args.get('cost_via_' + _kind, '1'))\n"),
}


def copy_generate(source: Path, goal: Path) -> Path:
    import os
    import shutil
    import subprocess
    source, goal = source.resolve(), goal.resolve()
    if goal == source or (source in goal.parents and (source / 'codex' / 'evolution' / 'results') not in goal.parents):
        raise ValueError('Create output outside the project or under codex/evolution/results')
    goal.mkdir(parents=True, exist_ok=False)
    ignore = shutil.ignore_patterns('__pycache__', '*.pyc', 'data', 'model', 'duration_*', 'results', '.git')
    for name in ('analysis', 'router', 'placement', 'kicad', 'common', 'learning', 'postprocessing', 'tests'):
        shutil.copytree(source / name, goal / name, ignore=ignore, symlinks=True)
    (goal / 'codex').mkdir()
    for name in ('ir_drop', 'zones_ir', 'generator_real'):
        shutil.copytree(source / 'codex' / name, goal / 'codex' / name, ignore=ignore, symlinks=True)
    for name in ('finish.sh', 'zones.sh'):
        shutil.copy2(source / name, goal / name)
    os.symlink(source / '.venv', goal / '.venv', target_is_directory=True)
    for name, (file, row) in MARKER.items():
        fpath = goal / file
        text = fpath.read_text()
        if text.count(row) != 1:
            raise ValueError('Marker location not unambiguous: ' + name)
        inset = row[:len(row) - len(row.lstrip())]
        fpath.write_text(text.replace(row, inset + '# EVOLUTION:' + name + ':BEGIN\n' + row + inset + '# EVOLUTION:' + name + ':END\n'))
    subprocess.run(['zsh', 'router/build.sh'], cwd=goal, check=True, capture_output=True, text=True)
    compile((goal / 'router/route_pf.py').read_bytes(), str(goal / 'router/route_pf.py'), 'exec')
    return goal


def suggestion_codex(marker: str, context: str, archive: list[dict], cwd: Path) -> str:
    import subprocess
    if marker not in MARKER:
        raise ValueError('unknown marker')
    file = cwd / MARKER[marker][0]
    lines = file.read_text().splitlines()
    pos = next(i for i, z in enumerate(lines) if '# EVOLUTION:' + marker + ':BEGIN' in z)
    end = next(i for i, z in enumerate(lines) if '# EVOLUTION:' + marker + ':END' in z)
    surroundings = '\n'.join(lines[max(0, pos-35):min(len(lines), end+36)])
    roles = {
        'cost_function': 'Sort key for nets before re-routing in every PathFinder negotiation round.',
        'rip_up': 'Selection of the units that are re-routed in this negotiation round; sorting follows.',
        'stall': 'Abort when conflict-free failed units repeatedly find no route.',
        'via_cost': 'Net-class-specific via price when building the search costs.',
        'presence_hist': 'Raises the presence price after a negotiation round.',
    }
    prompt = ('Generate a structural, deterministic PathFinder improvement proposal. '
              'No pure factor change of existing options. Without classes the previous behavior must remain bit-identical. '
              'Change only the content between EVOLUTION:' + marker + ':BEGIN and :END in ' + MARKER[marker][0] +
              '. Do not write files or run commands. Respond only with a valid Unified Diff '
              'with a/ and b/ file paths, without Markdown or explanation.\nRole: ' + roles[marker] +
              '\nCode context:\n' + surroundings + '\n' + context + '\nRecent attempts:\n' + json.dumps(archive[-10:], ensure_ascii=False))
    keepout = Path.home() / 'Library/Application Support/Autorouter/codex_keepout.sb'
    if not keepout.is_file():
        raise RuntimeError('Codex lock profile missing: ' + str(keepout))
    result = subprocess.run(['sandbox-exec', '-f', str(keepout), 'codex', 'exec', '-m', 'gpt-6.1-terra', '--sandbox', 'danger-full-access', '--skip-git-repo-check', prompt],
                            cwd=cwd, text=True, capture_output=True, check=True)
    return result.stdout


def structural(prior: str, new: str) -> bool:
    import copy
    class Norm(ast.NodeTransformer):
        def visit_Constant(self, node):
            if type(node.value) in (int, float):
                return ast.copy_location(ast.Constant(value=1), node)
            if isinstance(node.value, str):
                try:
                    float(node.value)
                except ValueError:
                    pass
                else:
                    return ast.copy_location(ast.Constant(value='1'), node)
            return node
        def visit_BinOp(self, node):
            node = self.generic_visit(node)
            if isinstance(node.op, ast.Mult):
                if isinstance(node.left, ast.Constant) and type(node.left.value) in (int, float): return node.right
                if isinstance(node.right, ast.Constant) and type(node.right.value) in (int, float): return node.left
            return node
        def visit_IfExp(self, node):
            node = self.generic_visit(node)
            if ast.dump(node.body) == ast.dump(node.orelse):
                return node.body
            return node
        def visit_If(self, node):
            node = self.generic_visit(node)
            if node.orelse and [ast.dump(x) for x in node.body] == [ast.dump(x) for x in node.orelse]:
                return node.body
            return node
    def normal(text):
        return ast.dump(Norm().visit(copy.deepcopy(ast.parse(text))), include_attributes=False)
    return normal(prior) != normal(new)


def diff_normalize(copy: Path, marker: str, diff: str) -> str:
    import difflib
    if marker not in MARKER: raise ValueError('unknown marker')
    file = MARKER[marker][0]
    lines = diff.splitlines(keepends=True)
    if lines and lines[0].strip() in ('```diff', '```') and lines[-1].strip() == '```':
        lines = lines[1:-1]
    oldheaders = [z for z in lines if z.startswith('--- ')]
    newheaders = [z for z in lines if z.startswith('+++ ')]
    if len(oldheaders) != 1 or len(newheaders) != 1 or oldheaders[0].strip() != '--- a/'+file or newheaders[0].strip() != '+++ b/'+file:
        raise ValueError('Diff must affect exactly the marker file')
    original = (copy/file).read_text()
    before = original.splitlines(keepends=True)
    after = list(before)
    idx = 0; hunks = 0; load = 0
    while idx < len(lines):
        if not lines[idx].startswith('@@ '):
            idx += 1; continue
        hunks += 1; idx += 1; prior = []; new = []
        while idx < len(lines) and not lines[idx].startswith('@@ '):
            line = lines[idx]; idx += 1
            if line.startswith(' '): prior.append(line[1:]); new.append(line[1:])
            elif line.startswith('-'): prior.append(line[1:])
            elif line.startswith('+'): new.append(line[1:])
            else: raise ValueError('Invalid hunk content')
        if not prior: raise ValueError('Hunk without original context')
        hits = [j for j in range(load, len(after)-len(prior)+1) if after[j:j+len(prior)] == prior]
        if len(hits) != 1: raise ValueError('Hunk context missing or not unambiguous')
        j = hits[0]; after[j:j+len(prior)] = new; load = j+len(new)
    if not hunks or after == before: raise ValueError('Diff without change')
    return ''.join(difflib.unified_diff(before, after, fromfile='a/'+file, tofile='b/'+file))


def patch_apply(copy: Path, marker: str, diff: str, only_structural: bool = True) -> None:
    import os
    import subprocess
    git_env = dict(os.environ, GIT_CEILING_DIRECTORIES=str(copy.resolve().parent))
    if marker not in MARKER: raise ValueError('unknown marker')
    file = MARKER[marker][0]
    prior = (copy / file).read_text()

    probe = subprocess.run(['git', 'apply', '--numstat', '-'], input=diff, cwd=copy,
                           text=True, capture_output=True, check=True, env=git_env)
    lines = probe.stdout.splitlines()
    if len(lines) != 1 or lines[0].split('\t')[-1] != file:
        raise ValueError('Patch must affect exactly the marker file')
    subprocess.run(['git', 'apply', '--check', '-'], input=diff, cwd=copy, text=True, capture_output=True, check=True, env=git_env)
    subprocess.run(['git', 'apply', '-'], input=diff, cwd=copy, text=True, capture_output=True, check=True, env=git_env)
    new = (copy / file).read_text()
    begin, end = '# EVOLUTION:' + marker + ':BEGIN', '# EVOLUTION:' + marker + ':END'
    try:
        a, b = prior.split(begin); _, c = b.split(end)
        x, y = new.split(begin); _, z = y.split(end)
        if a != x or c != z: raise ValueError('Patch outside the marker')
        if only_structural and not structural(prior, new):
            raise ValueError('No structural change')
        subprocess.run(['zsh', 'router/build.sh'], cwd=copy, check=True, capture_output=True, text=True)
        compile((copy / 'router/route_pf.py').read_bytes(), str(copy / 'router/route_pf.py'), 'exec')
    except Exception:
        (copy / file).write_text(prior)
        subprocess.run(['zsh', 'router/build.sh'], cwd=copy, check=True, capture_output=True, text=True)
        raise



def pair_comparison(basis, candidate):
    from analysis import rule_metrics as RK, contract_spec as VT
    before = pair_pre_gate(basis, candidate)
    a = {x['seed']: x for x in basis}; b = {x['seed']: x for x in candidate}
    common = [s for s in sorted(a) if a[s]['is_valid'] and b[s]['is_valid']]
    contradiction = [dict(seed=s, board_side=board_side) for board_side, rows in (('basis', a), ('candidate', b)) for s in sorted(rows)
                   if rows[s]['is_valid'] and (rows[s].get('kicad_native_before_final') or {}).get('is_valid') is False]
    def gate_check(z):
        t = VT.from_row(z)


        r = z.get('rule') or {}
        if not t.get('measurement_version') and r.get('pair_measurement_version'):
            t['measurement_version'] = r['pair_measurement_version']
        return t
    pairs, strict = {}, {}
    for s in common:
        x, y = a[s].get('raw_data') or {}, b[s].get('raw_data') or {}
        tx, ty = gate_check(x), gate_check(y)
        pairs[str(s)] = VT.comparison(tx, ty)
        strict[str(s)] = VT.comparison(tx, ty, rigid=True)
    new = {str(s): gate_check(b[s].get('raw_data') or {}) for s in sorted(a) if not a[s]['is_valid'] and b[s]['is_valid']}
    rg = RK.comparison({str(s): (a[s]['raw_data'].get('rule') or {}).get('design_rules') for s in common},
                      {str(s): (b[s]['raw_data'].get('rule') or {}).get('design_rules') for s in common})
    bad = [k for k, v in rg['per'].items() if v and v.get('gate_check') and not v.get('not_worse', False)]
    new_measured = all(t.get('measurement_version') and all(v is not None for v in t['design_rules'].values())
                        and t['electrical'] != 'violated' for t in new.values())
    comparable = not contradiction and bool(common) and rg['complete'] and not bad and all(p['not_worse'] for p in pairs.values()) and new_measured
    rigid = comparable and all(p['not_worse'] for p in strict.values()) and all(t['complete_valid'] for t in new.values())
    return dict(pre_gate=before, rule_comparison=rg, single_contract=pairs, single_contract_strict=strict,
                new_valid=new, rule_bad=bad, native_contradictions=contradiction,
                quality_not_worse=bool(before['pre_gate'] and comparable),
                release=bool(before['pre_gate'] and rigid),
                reason='Native routing before completion contradicts self-play' if contradiction else ('No common valid boards' if not common else ('Rule gate missing or violated' if not comparable else ('Mandatory proofs unknown or violated' if not rigid else 'Rule gate satisfied'))))


def cell(lines, level=0):
    if any(z.get('is_valid') is None for z in lines): faults = 'unknown'
    elif any(z.get('open_items', 0) for z in lines): faults = 'open_items'
    elif any(z.get('remaining_conflicts', 0) for z in lines): faults = 'conflict_list'
    elif any(not z.get('is_valid') for z in lines): faults = 'otherwise'
    else: faults = 'is_valid'
    return f'{level}:{faults}'


def parent_selection(entries, chance):
    grid_cells = {}
    for e in entries:
        if e.get('cell') and e.get('patch') and e.get('status') == 'measured':
            grid_cells.setdefault(e['cell'], []).append(e)
    if not grid_cells: return None
    cell = chance.choice(sorted(grid_cells))
    return chance.choice(sorted(grid_cells[cell], key=lambda e: e['id']))


def elite_update(folder, entry):
    import os
    if any(type(z.get('is_valid')) is not bool or z.get('remaining_conflicts') is None or z.get('open_items') is None for z in entry['lines']):
        return False
    folder.mkdir(parents=True, exist_ok=True)
    file = folder / 'grid_cells.json'
    grid_cells = json.loads(file.read_text()) if file.exists() else {}
    key = entry['cell']; prior = grid_cells.get(key)
    if prior:
        try:
            v = pair_comparison(prior['lines'], entry['lines'])
        except ValueError:
            return False
        if not v['quality_not_worse']:
            return False
        v_gate = v['pre_gate']
        improved = (v_gate['candidate']['is_valid'] > v_gate['basis']['is_valid'] or
                      any(t and t.get('gate_check') and t.get('verdict') == -1 for t in v['rule_comparison']['per'].values()))
        if not improved: return False
    grid_cells[key] = entry
    temp = folder / 'grid_cells.json.tmp'
    temp.write_text(json.dumps(grid_cells, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False))
    os.replace(temp, file)
    return True


def suggestion_structural(copy, marker, context, history):
    answer = suggestion_codex(marker, context, history, copy)
    diff = diff_normalize(copy, marker, answer)
    patch_apply(copy, marker, diff)
    return dict(source='codex', marker=marker, patch=diff)


def hand_patch(copy, factor):
    import difflib
    file, _ = MARKER['presence_hist']
    prior = (copy / file).read_text()
    row = "    PF[0] *= float(args.get('pfmul', '1.5'))\n"
    new = prior.replace(row, "    PF[0] *= float(args.get('pfmul', '1.5')) * (" + str(factor) + " if NC is not None else 1.0)\n")
    if new == prior: raise ValueError('Marker line missing')
    return ''.join(difflib.unified_diff(prior.splitlines(True), new.splitlines(True), fromfile='a/'+file, tofile='b/'+file))


def next_candidate(source, goal, elite_path, run_seed, factor=None, context='', marker=None):
    import difflib
    allowed = ('cost_function', 'rip_up', 'stall', 'via_cost')
    marker = marker or allowed[run_seed % len(allowed)]
    if marker not in allowed: raise ValueError('Marker not in the structural search space')
    file = MARKER[marker][0]
    entries = list(json.loads(elite_path.read_text()).values()) if elite_path.exists() else []
    parent = parent_selection([e for e in entries if e.get('marker') == marker], random.Random(run_seed))
    copy_generate(source, goal)
    p = goal / file; original = p.read_text()
    if parent:
        patch_apply(goal, marker, parent['patch'])
    mutation = suggestion_structural(goal, marker, context, entries)
    total = ''.join(difflib.unified_diff(original.splitlines(True), p.read_text().splitlines(True), fromfile='a/'+file, tofile='b/'+file))
    if not structural(original, p.read_text()): raise ValueError('Overall patch is not structural')
    return dict(parent=parent['id'] if parent else 'basis', parent_cell=parent['cell'] if parent else None,
                source='codex', marker=marker, faults=None, patch=total, mutation=mutation['patch'], seed=run_seed)


def stop_guard(stop):
    import os, threading
    from common.cpu_tree import CpuTree
    from common.process_guard import ORIGINAL_POPEN
    def watch():
        tree = CpuTree(ORIGINAL_POPEN)
        while True:
            tree.sample(os.getpid())
            if Path(stop).exists(): tree.kill(os.getpid()); os._exit(130)
            threading.Event().wait(.3)
    t = threading.Thread(target=watch, daemon=True); t.start()


def worker(category, task, outputs, stop):
    import os, shutil
    stop_guard(stop)
    if Path(stop).exists(): raise SystemExit(130)
    if category == 'generate':
        from learning import tasks as AU
        AU.generate(outputs, int(task), level=0)
        return
    from learning import selfplay as SS, learning_curve as LK
    from analysis import corpus as KO
    shutil.copytree(task, outputs)
    A = json.loads(Path(outputs, 'task.json').read_text())
    A['evaluation_target'] = 'v3'
    vp = Path(outputs, 'specs.json'); cons = json.loads(vp.read_text()); cons['evaluation'] = 'v3'; vp.write_text(json.dumps(cons))
    KO.netclasses_run(outputs, str(Path(outputs, 'board.json')), str(Path(outputs, 'netclasses.json')), str(vp))
    cfg = LK.meas_cfg(dict(LK.REF, legal_lp=True, champion='0'), 'work', 59147, 60000, ())
    z = SS.attempt(outputs, A, cfg, 0, 300, 40)
    Path(outputs, 'measurement.json').write_text(json.dumps(z, ensure_ascii=False, indent=1, allow_nan=False))


def measurement_run(code, task, outputs, stop, generate=False):
    import os
    from common import slots as PL
    cmd = [str(code / '.venv/bin/python'), str(code / 'learning/evolution.py'), 'worker',
           'generate' if generate else 'measure', str(task), str(outputs), str(stop)]
    return PL.job(cmd, cpu_s=60000, log=str(outputs.parent / (outputs.name + '.log')),
                   env=dict(os.environ, PYTHONHASHSEED='0', AUTOROUTER_MAIN=str(code)), cwd=str(code))


def kicad_native_before_final(folder):
    p = folder / 'k0' / 'routes.json'
    if not p.exists(): return None
    data = json.loads(p.read_text()).get('multi_attempts') or {}
    candidates = [x for x in data.get('attempts', []) if x.get('seed') == data.get('chosen')]
    if len(candidates) != 1: return None
    item = candidates[0]
    if type(item.get('is_valid')) is not bool: return None
    return dict(seed=item['seed'], is_valid=item['is_valid'], faults=item.get('faults'),
                open_pairs=item.get('open_pairs'), phase='before shape and zones')


def measurement_line(seed, folder, pl):
    p = folder / 'measurement.json'
    z = json.loads(p.read_text()) if p.exists() else {}
    k = z.get('metrics') or {}
    return dict(seed=seed, is_valid=z.get('is_valid'), remaining_conflicts=k.get('conflict_list'), open_items=k.get('open_items'),
                score_v3=z.get('score_v3'), rule=z.get('rule'), kicad_native_before_final=kicad_native_before_final(folder), cur_state=z.get('cur_state', pl['cur_state']),
                cpu_s=pl['cpu_s'], pl=pl, raw_data=z)


if __name__ == '__main__':
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from common.process_guard import bootstrap
    bootstrap(heavy=False if sys.argv[1:2] != ['worker'] else True)
    if sys.argv[1:2] == ['worker'] and len(sys.argv) == 6:
        worker(*sys.argv[2:])
    elif sys.argv[1:2] == ['probe'] and len(sys.argv) == 3:
        import runpy
        script = Path(__file__).resolve().parents[1] / 'codex/evolution/resume.py'
        sys.argv = [str(script), sys.argv[2]]
        runpy.run_path(str(script), run_name='__main__')
    else:
        raise SystemExit('Usage: evolution.py probe <new results folder>')
