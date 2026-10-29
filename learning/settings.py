import json, math, os, sys
R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

DIMENSIONS = ('cross', 'separate', 'via_cost', 'via_f', 'style')
PLACER_DIM = ('cross', 'separate', 'via_cost')





DEFAULT = dict(
    test_bench=dict(cross=0.0, separate=3.0, via_cost=0.6, via_f=1.0, style='standard'),
    pipeline=dict(cross=2.0, separate=3.0, via_cost=0.6, via_f=1.0, style='standard'))
PATH = 'learning/model/steering.json'
N_MIN = 10
Z = 1.645
SPACING = 0.02
SIGMA, N0, PRIOR0 = 0.5, 1.0, 1.2

def net_class(mm, cu_layers=2):
    mm = mm or {}
    d = mm.get('pads_per_cm2') or 0.0; n = mm.get('n_parts') or 0
    dk = 'd0' if d < 1.5 else 'd1' if d < 3.0 else 'd2'
    tk = 't0' if n < 25 else 't1' if n < 60 else 't2'
    return 'L%d_%s_%s' % (int(mm.get('cu_layers') or cu_layers or 2), dk, tk)


def _arm(a):
    try: return '%g' % float(a)
    except (TypeError, ValueError): return str(a)


def _value(dim, a, model_example):
    return float(a) if isinstance(model_example, (int, float)) else str(a)


def path_resolve(fpath):
    if not fpath: fpath = PATH
    if os.path.isabs(fpath) or os.path.exists(fpath): return os.path.abspath(fpath)
    return os.path.join(R, fpath)


def steering_load(fpath=PATH):
    p = path_resolve(fpath)
    try:
        with open(p) as f: return json.load(f), p
    except Exception: return None, p


def posterior(L, k, dim, a):
    st = L.get('statistics') or {}
    ng, sg = st.get('*|%s|%s' % (dim, _arm(a)), [0, 0.0])
    prior = sg / ng if ng else PRIOR0
    n, s = st.get('%s|%s|%s' % (k, dim, _arm(a)), [0, 0.0])
    return (s + N0 * prior) / (n + N0), SIGMA / math.sqrt(n + N0), int(n)


def settings(feats, cu_layers=2, fpath=PATH, n_min=N_MIN, spec='test_bench', pinned=None, z=Z, spacing=SPACING):
    V = dict(DEFAULT[spec] if isinstance(spec, str) else spec)
    pinned = {d: v for d, v in (pinned or {}).items() if d in DIMENSIONS and v not in (None, '')}
    k = net_class(feats, cu_layers)
    L, p = steering_load(fpath)
    out = dict(net_class=k, value_list={}, provenance={}, reason={}, steering=p, steering_time=(L or {}).get('time_s'), n_min=n_min)
    best = ((L or {}).get('best') or {}).get(k) or {}
    for d in DIMENSIONS:
        v0 = V[d]
        if d in pinned:
            out['value_list'][d] = _value(d, pinned[d], v0); out['provenance'][d] = 'explicit'; out['reason'][d] = 'explicitly set'; continue
        out['value_list'][d] = v0; out['provenance'][d] = 'default'
        if L is None:
            out['reason'][d] = 'steering.json missing'; continue
        if d not in best:
            out['reason'][d] = 'no data for class %s' % k; continue
        if L.get('format') == 'steering.v3':


            ab = best[d][1]
            c = ((L.get('contrasts') or {}).get(k, {}).get(d, {}).get('%s|%s' % (_arm(ab), _arm(v0))))
            out['reason'][d] = 'debiased, but no gate release for the pipeline'
            if not L.get('released') or not c or not c.get('ci'): continue
            from learning.collect import code_state
            if L.get('code') != code_state():
                out['reason'][d] = 'different code state'; continue
            if c['n'] < n_min or c['n_baseline'] < n_min or c['ci'][1] > -spacing:
                out['reason'][d] = 'Contrast not safely ahead or too little data'; continue
            out['value_list'][d] = _value(d, ab, v0); out['provenance'][d] = 'learned'
            out['reason'][d] = 'debiased contrast %.3f, 95 %% %s' % (c['diff'], c['ci'])
            continue
        mb, ab, nb = best[d][0], best[d][1], int(best[d][2])
        if _arm(ab) == _arm(v0):
            out['reason'][d] = 'Default %s is the best arm (%.3f, n %d)' % (_arm(v0), mb, nb); continue
        if nb < n_min:
            out['reason'][d] = 'best arm %s (%.3f) has only n %d < %d' % (_arm(ab), mb, nb, n_min); continue
        mb2, sb, nb2 = posterior(L, k, d, ab)
        m0, s0, n0 = posterior(L, k, d, v0)
        diff = m0 - mb2; required = max(spacing, z * math.hypot(sb, s0))
        txt = '%s %.3f (n %d) versus default %s %.3f (n %d): lead %.3f, required %.3f' % (_arm(ab), mb2, nb2, _arm(v0), m0, n0, diff, required)
        if diff >= required:
            out['value_list'][d] = _value(d, ab, v0); out['provenance'][d] = 'learned'; out['reason'][d] = txt
        else:
            out['reason'][d] = 'not clearly ahead: ' + txt
    return out


def placer_options(E):
    return ['%s=%g' % (d, E['value_list'][d]) for d in PLACER_DIM if E['provenance'][d] == 'learned']


def short(E):
    return '%s: ' % E['net_class'] + ' '.join('%s=%s (%s)' % (d, _arm(E['value_list'][d]), E['provenance'][d]) for d in DIMENSIONS)


def main():
    pos = [a for a in sys.argv[1:] if '=' not in a]; kw = dict(a.split('=', 1) for a in sys.argv[1:] if '=' in a)
    if len(pos) < 2:
        print(__doc__); sys.exit(2)
    sys.path.insert(0, R)
    from learning.features import feats
    B = json.load(open(pos[0])); K = json.load(open(pos[1])); RG = json.load(open(pos[2])) if len(pos) > 2 else None
    cu_layers = int(kw.get('cu_layers') or B.get('cu_layers') or 2)
    mm = feats(B, K, RG)
    if kw.get('cu_layers'): mm = dict(mm, cu_layers=cu_layers)
    E = settings(mm, cu_layers, fpath=kw.get('steering', PATH), n_min=int(kw.get('n_min', N_MIN)), spec=kw.get('spec', 'test_bench'))
    fmt = kw.get('format', 'text')
    if fmt == 'json':
        print(json.dumps(E, indent=1)); return
    if fmt == 'sh':
        print('E_NET_CLASS=%s' % E['net_class'])
        for d in DIMENSIONS: print('E_%s=%s' % (d.upper(), _arm(E['value_list'][d])))
        for d in DIMENSIONS: print('E_PROVENANCE_%s=%s' % (d.upper(), E['provenance'][d]))
        for d in DIMENSIONS: print('# %s: %s (%s)' % (d, E['provenance'][d], E['reason'][d]), file=sys.stderr)
        return
    print('Class %s, steering.json from %s' % (E['net_class'], E['steering_time']))
    for d in DIMENSIONS: print('  %-10s %-9s %-14s %s' % (d, _arm(E['value_list'][d]), E['provenance'][d], E['reason'][d]))


if __name__ == '__main__':
    main()
