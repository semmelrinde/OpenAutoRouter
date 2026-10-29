"""Learned move selection for the router's repair moves (move=1).

Reads the logged moves (<OUT>_move_records.jsonl written by the router) and estimates, per spot situation (type, group
size, pad density) and strategy, the chance of success and the CPU cost. The result is a policy file that the router
reads with move_policy=<file>: per spot it tries the strategies in order of expected improvement per CPU second
instead of the fixed order. Situations without enough data keep the fixed order.

usage: python3 learning/move_choice.py <moves.jsonl> <policy.json> [min_n=5]
       python3 learning/move_choice.py collect <folder> <moves.jsonl>   (append all *_move_records.jsonl files)
"""
import collections, glob, hashlib, json, os, sys

VERSION = 'move_choice-v1'


def strategy_key(z):
    return '%s|%s|%s|%d' % (z['R'], z['series'], int(bool(z.get('repair'))), int(bool(z.get('warm'))))


def layer_key(z):
    k = z.get('core_set', 1); p = z.get('pads2', 0)
    return '%s|k%d|p%d' % (z['type_'], min(k, 3), 0 if p <= 8 else 1 if p <= 20 else 2)


def learning(lines, min_n=5):
    st = collections.defaultdict(lambda: [0, 0, 0.0])
    for z in lines:
        if z.get('outcome') not in ('taken', 'rejected'): continue
        for lk in (layer_key(z), z['type_']):
            s = st[(lk, strategy_key(z))]; s[0] += 1; s[1] += z['outcome'] == 'taken'; s[2] += float(z.get('cpu_s') or 0)
    tab = {}
    for (lk, sk), (n, ok, cpu) in sorted(st.items()):
        if n < min_n: continue
        p = (ok + 1) / (n + 2); c = max(cpu / n, 0.05)
        tab.setdefault(lk, {})[sk] = dict(n=n, ok=ok, p=round(p, 4), cpu=round(c, 3), amount=round(p / c, 6))
    return tab


def main(argv):
    if argv and argv[0] == 'collect':
        folder, goal = argv[1], argv[2]; n = 0
        prior = set()
        if os.path.exists(goal):
            prior = {json.loads(l).get('_source_row') for l in open(goal)}
        with open(goal, 'a') as out:
            for f in sorted(glob.glob(os.path.join(folder, '**', '*_moves.jsonl'), recursive=True)):
                for i, l in enumerate(open(f)):
                    key = hashlib.sha256(('%s:%d:%s' % (f, i, l)).encode()).hexdigest()[:16]
                    if key in prior: continue
                    z = json.loads(l); z['_source_row'] = key; out.write(json.dumps(z, sort_keys=True) + '\n'); n += 1
        print('collected', n, 'new moves to', goal); return
    source, goal = argv[0], argv[1]
    kw = dict(a.split('=', 1) for a in argv[2:])
    lines = [json.loads(l) for l in open(source)]
    tab = learning(lines, int(kw.get('min_n', '5')))
    out = dict(version=VERSION, move_records=len(lines), taken=sum(z.get('outcome') == 'taken' for z in lines),
               source_sha=hashlib.sha256(open(source, 'rb').read()).hexdigest(), table=tab)
    os.makedirs(os.path.dirname(os.path.abspath(goal)), exist_ok=True)
    with open(goal + '.tmp', 'w') as f: json.dump(out, f, indent=1, sort_keys=True)
    os.replace(goal + '.tmp', goal)
    print('%d moves, %d accepted, %d layers in %s' % (out['move_records'], out['taken'], len(tab), goal))


if __name__ == '__main__':
    main(sys.argv[1:])
