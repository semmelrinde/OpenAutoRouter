import json, os, sys, copy
R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, R)
from learning.features import feats
from learning import model as LM


def board_off_placement(B, P, exact=False):
    import numpy as np
    from collections import defaultdict
    B = copy.deepcopy(B); parts = P['parts']
    fps = {f['ref']: f for f in B['fps']}
    by = defaultdict(list)
    for p in B['pads']: by[p['ref']].append(p)
    for r in (parts if exact else by):
        ps=by.get(r,[])
        t = parts.get(r)
        if not t or r not in fps: continue
        flip = t['side'] != fps[r]['side']; ox = fps[r]['x']
        nums = defaultdict(list)
        for p in ps: nums[p['num']].append(p)
        pairs = [(p, t.get('pads',{})[n]) for n, L in nums.items() if len(L) == 1 and n in t.get('pads',{}) for p in L]
        def source(x, y): return ((2 * ox - x) if flip else x, y)
        if exact:


            import math
            if not all(k in t for k in ('x','y','rotation')):raise ValueError('exact placement transform missing: '+r)
            th=math.radians(t['rotation']);c,s=math.cos(th),math.sin(th)
            Rm=np.array([[c,-s],[s,c]])
            ca=np.array([fps[r]['x'],fps[r]['y']]);cz=np.array([t['x'],t['y']])

            rot0=fps[r].get('rot',0)
            fps[r]['rot']=((180-rot0 if flip else rot0)-t['rotation'])%360
        elif len(pairs) >= 2:
            A = np.array([source(p['x'], p['y']) for p, _ in pairs]); Z = np.array([q for _, q in pairs], float)
            ca, cz = A.mean(0), Z.mean(0); H = (A - ca).T @ (Z - cz); U, _, Vt = np.linalg.svd(H)
            Rm = Vt.T @ U.T
            if np.linalg.det(Rm) < 0: Vt[1] *= -1; Rm = Vt.T @ U.T
        else:
            Rm = np.eye(2); ca = np.array(source(ps[0]['x'], ps[0]['y'])) if ps else np.zeros(2)
            cz = np.array(pairs[0][1], float) if pairs else np.array([t.get('x', ps[0]['x']), t.get('y', ps[0]['y'])], float)
        def T(x, y):
            q = Rm @ (np.array(source(x, y)) - ca) + cz; return [float(q[0]), float(q[1])]
        for p in ps:
            p['x'], p['y'] = T(p['x'], p['y'])
            new = {}
            for L, polys in p['polys'].items():
                L2 = ('B' if L == 'F' else 'F' if L == 'B' else L) if flip and len(p['layers']) == 1 else L
                new[L2] = [[T(x, y) for x, y in poly] for poly in polys]
            p['polys'] = new
            if flip and len(p['layers']) == 1: p['layers'] = ['B' if p['layers'][0] == 'F' else 'F']
        for c in B.get('courtyards', []) + B.get('keepouts', []):
            if c.get('ref') == r:
                c['polys'] = [[T(x, y) for x, y in q] for q in c['polys']]

                if 'hole_list' in c: c['hole_list'] = [[[T(x, y) for x, y in q] for q in lo] for lo in c['hole_list']]
                if flip and 'cu_layers' in c:
                    c['cu_layers'] = ['B' if L == 'F' else 'F' if L == 'B' else L for L in c['cu_layers']]
                if flip and c.get('side') in ('F', 'B'): c['side'] = 'B' if c['side'] == 'F' else 'F'
        f = fps[r]; f['x'], f['y'], f['side'] = t.get('x', f['x']), t.get('y', f['y']), t['side']
    return B

def choose(B, K, RG, placements, category='model'):
    ks = []
    for i, pj in enumerate(placements):
        P = json.load(open(pj))
        ks.append(dict(candidate=i, fpath=pj, feats=dict(feats(board_off_placement(B, P), K, RG), via_f=1.0)))
    if category == 'heuristic':
        best = LM.choice_heuristic(ks); P_ = C_ = [None] * len(ks)
    else:
        import joblib
        mod = joblib.load(os.path.join(R, 'learning/model/selection.joblib'))
        P_, C_ = LM.prediction(mod, [k['feats'] for k in ks])
        best = LM.choice_model(mod)(ks)
    for k, p, c in zip(ks, P_, C_): k['p_valid'] = None if p is None else round(float(p), 3); k['cost'] = None if c is None else round(float(c), 1)
    return best, ks


if __name__ == '__main__':
    B = json.load(open(sys.argv[1])); K = json.load(open(sys.argv[2])); RG = json.load(open(sys.argv[3]))
    kw = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
    best, ks = choose(B, K, RG, [a for a in sys.argv[4:] if '=' not in a], kw.get('selection', 'model'))
    for k in ks:
        print('Candidate %d  P(valid) %s  Cost %s  Airline %.0f  Crossings %d  %s' % (
            k['candidate'], k['p_valid'], k['cost'], k['feats']['mst'], k['feats']['crossings'], '<- chosen' if k is best else ''))
    print('CHOSEN', best['candidate'], best['fpath'])
