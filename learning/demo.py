"""One-command demo: invent a board, place it, route it, score it.

Runs the whole chain (learning/tasks.py, analysis/netclasses.py, placement/placer.py, applying the placement without KiCad, router/route_pf.py, analysis/evaluation.py) for one or more seeds. Each seed gets a folder with preview images (placement pl_*.png, routed layers rt_*.png, scoring by net kind bw_*.png). At the end a Markdown table is printed that can be pasted into a pull request: same seed and setup, same numbers. No own board needed; KiCad is only used for its footprint library.

usage: python3 learning/demo.py <folder> [seed=1] [n=1] [level=0] [cu_layers=4] [iter=30] [move=1]
"""
import json, os, subprocess, sys, time

R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, R)


def step(cmd, log):
    t = time.time()
    with open(log, 'a') as f:
        r = subprocess.run([sys.executable] + cmd, stdout=f, stderr=subprocess.STDOUT, env=dict(os.environ, PYTHONHASHSEED='0'))
    if r.returncode: raise RuntimeError('%s failed (exit %d), see %s' % (os.path.basename(cmd[0]), r.returncode, log))
    return time.time() - t


def apply(d):
    from common.shift import relocate
    from placement.p3_groups import board_with_groups
    B = json.load(open(os.path.join(d, 'board.json'))); P = json.load(open(os.path.join(d, 'placement.json')))
    fps = {f['ref']: f for f in B['fps']}; T = {r: p for r, p in sorted(P['parts'].items()) if r in fps}
    moves = {r: (p['x'] - fps[r]['x'], p['y'] - fps[r]['y'], p.get('rotation', 0.0)) for r, p in T.items()}
    B, _ = relocate(B, moves, sides={r: p['side'] for r, p in T.items()})
    json.dump(board_with_groups(B, P), open(os.path.join(d, 'board_placed.json'), 'w'))


def one(goal, seed, kw):
    d = os.path.join(goal, 'task_%06d' % seed); log = os.path.join(d, 'demo.log')
    step([os.path.join(R, 'learning/tasks.py'), goal, 'seed=%d' % seed, 'level=' + kw.get('level', '0'), 'cu_layers=' + kw.get('cu_layers', '4')], os.devnull)
    A = json.load(open(os.path.join(d, 'task.json')))
    t = dict(place=0.0, route=0.0)
    os.chdir(d)
    step([os.path.join(R, 'analysis/netclasses.py'), 'board.json', 'netclasses.json', 'design_rules=design_rules.json', 'specs=specs.json'], log)
    t['place'] = step([os.path.join(R, 'placement/placer.py'), 'board.json', 'netclasses.json', 'placement.json',
                               'design_rules=design_rules.json', 'mode=new', 'image=pl'], log)
    apply(d)
    t['route'] = step([os.path.join(R, 'router/route_pf.py'), 'board_placed.json', 'routes.json', 'design_rules=design_rules.json',
                           'netclasses=netclasses.json', 'iter=' + kw.get('iter', '30'), 'move=' + kw.get('move', '1'), 'png=rt'], log)
    try:
        step([os.path.join(R, 'analysis/evaluation.py'), 'board_placed.json', 'routes.json', 'netclasses.json', 'json=evaluation.json', 'image=bw'], log)
        E = json.load(open('evaluation.json'))
    except RuntimeError:
        Rt = json.load(open('routes.json')); open_items = len(Rt.get('failed') or []) + len(Rt.get('unreachable') or [])
        E = dict(is_valid=not Rt.get('conflicts') and not open_items, conflict_list=len(Rt.get('conflicts') or {}), open_items=open_items, rating='router check only')
    return dict(seed=seed, cu_layers=A.get('cu_layers'), part_list=len(json.load(open('board.json'))['fps']), is_valid=bool(E.get('is_valid')),
                conflict_list=E.get('conflict_list'), open_items=(E.get('open_items') or 0) + len(E.get('not_connected') or []),
                rating=E.get('score_v3', E.get('rating')), place_s=round(t['place']), route_s=round(t['route']), folder=d)


def main(argv):
    if not argv or '=' in argv[0]: raise SystemExit(__doc__)
    goal = os.path.abspath(argv[0]); kw = dict(a.split('=', 1) for a in argv[1:] if '=' in a)
    os.makedirs(goal, exist_ok=True)
    s0, n = int(kw.get('seed', '1')), int(kw.get('n', '1'))
    lines = []
    for s in range(s0, s0 + n):
        try: z = one(goal, s, kw)
        except Exception as e: z = dict(seed=s, is_valid=False, faults=str(e)[:200])
        lines.append(z)
        print('seed %d: %s' % (s, 'is_valid' if z.get('is_valid') else z.get('faults') or 'invalid (%s conflicts, %s open)' % (z.get('conflict_list'), z.get('open_items'))), flush=True)
    print('\n| seed | layers | parts | valid | conflicts | open | score | place s | route s |')
    print('|---|---|---|---|---|---|---|---|---|')
    for z in lines:
        print('| %s |' % ' | '.join(str(z.get(k, '')) for k in ('seed', 'cu_layers', 'part_list', 'is_valid', 'conflict_list', 'open_items', 'rating', 'place_s', 'route_s')))
    print('\n%d/%d valid. Images: <folder>/task_*/pl_*.png (placement), rt_*.png (routed layers), bw_*.png (scoring by net kind)' % (sum(bool(z.get('is_valid')) for z in lines), n))
    json.dump(lines, open(os.path.join(goal, 'demo.json'), 'w'), indent=1)


if __name__ == '__main__':
    main(sys.argv[1:])
