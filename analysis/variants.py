import json, os, sys
R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, R)


def key_heuristic(m):
    return m['mst'] * (1 + 0.02 * m['crossings']) * (1 + max(0.0, m['rudy_max'] - 1))


def ranking(B, K, RG, placements, selection='heuristic'):
    from learning.selection import board_off_placement
    from learning.features import feats
    ks = [dict(fpath=p, feats=feats(board_off_placement(B, json.load(open(p))), K, RG)) for p in placements]
    if selection == 'model':
        import joblib, numpy as np
        from learning import model as LM
        mod = joblib.load(os.path.join(R, 'learning/model/selection.joblib'))
        P, C = LM.prediction(mod, [dict(k['feats'], via_f=1.0) for k in ks]); med = float(np.median(C))
        E = P * C + (1 - P) * 2 * med
        for k, e in zip(ks, E): k['keyname'] = float(e)
    else:
        for k in ks: k['keyname'] = key_heuristic(k['feats'])
    return sorted(ks, key=lambda k: (k['keyname'], k['fpath']))


def zone_opt():
    return {k: os.environ[k] for k in ('ZONE_EDGE', 'ZONE_GAP', 'ZONE_FAIR', 'ZONE_PRECEDENCE', 'ZONE_GND_SIGNAL', 'GND_ALL',
                                       'FILL_NET', 'TONGUE', 'ZCLR') if k in os.environ}


def drc_numbers(fpath):
    try:
        d = json.load(open(fpath))
        v = d['violations']; open_items = d['unconnected_items']
        if not isinstance(v, list) or not isinstance(open_items, list) or any(not isinstance(x, dict) for x in open_items):
            raise ValueError('invalid DRC format')
        if any(not isinstance(x, dict) or x.get('severity') not in ('error', 'warning', 'info') or not isinstance(x.get('type'), str) for x in v):
            raise ValueError('invalid DRC entry')
    except (OSError, ValueError, KeyError, TypeError):
        return 99999, 99999, 0
    return (sum(1 for x in v if x.get('severity') == 'error'), len(open_items),
            sum(1 for x in v if x.get('type') in ('track_dangling', 'via_dangling')))


def load(folder, routes='routes_shape.json', drc='drc_finished.json', ir=True):
    from analysis.evaluation import rate
    rp = os.path.join(folder, routes)
    if not os.path.exists(rp): rp = os.path.join(folder, 'routes.json')
    B = json.load(open(os.path.join(folder, 'board.json'))); K = json.load(open(os.path.join(folder, 'netclasses.json')))
    Rt = json.load(open(rp))


    Rt = dict(Rt, tracks=[{k: x for k, x in t.items() if k != 'items'} for t in Rt['tracks']])
    E = rate(B, Rt, K, ir=ir, zone_list=zone_opt() if ir else None, v3=True)
    from learning.judge_pair import measurement_signature, components, component_occupancy
    from learning.pair_contract import board_id
    E.update(board_id=board_id(B), pair_measurement_version=measurement_signature(), v3_components=components(E.get('measures') or {}), v3_components_evidence=component_occupancy(E.get('measures') or {}))
    fe, of, lo = drc_numbers(os.path.join(folder, drc))
    return dict(folder=folder, routes=rp, B=B, K=K, R=Rt, E=E, drc_error=fe, drc_open=of, drc_loose=lo,
                valid_whole=bool(E['is_valid']) and fe == 0 and of == 0)


def gate_key(v):
    return (not v['valid_whole'], v['drc_error'] + v['drc_open'], tuple(v['E']['rank'][:3]))
def choose(vs, rating='v3', pair=None):
    from analysis.evaluation import comparison
    vs = sorted(vs, key=lambda v: (gate_key(v), v['folder']))
    upper = [v for v in vs if gate_key(v) == gate_key(vs[0])]
    best = upper[0]; prot = []
    for v in upper[1:]:
        cons = comparison(best['E'], v['E'], best['B'], best['R'], v['B'], v['R'], best['K'], rating=rating, K1=v['K'])
        prot.append(dict(against=best['folder'], candidate=v['folder'], better=cons['better'], reason=cons['reason'], delta=cons.get('delta')))
        if cons['better']: best = v

    if pair is None:
        from learning.judge_pair import Service
        pair = Service()
    def row(v):
        return dict(v['E'], is_valid=v['valid_whole'], rule=v['E'].get('design_rules') or {})
    for v in upper:
        if v is best: continue
        decision = pair.comparison(row(best), row(v))
        if decision['pair']:
            prot.append(dict(against=best['folder'], candidate=v['folder'], **decision))
            if decision['better']: best = v
    return best, prot


def row(v):
    E = v['E']; sp = E.get('current_paths') or {}; ir = E.get('ir') or {}
    mx = max((ir.get('max_drop_mV') or {}).values() or [0.0])
    return '%-28s %s DRC %d/%d (loose %d) | Score v3 %8.1f | v2 %s | Vias %4d | Length %7.1f mm | Loss %s mW | largest drop %.1f mV' % (
        os.path.basename(v['folder'].rstrip('/')), 'VALID  ' if v['valid_whole'] else 'invalid', v['drc_error'], v['drc_open'],
        v['drc_loose'], E.get('score_v3') or 0.0, '%.1f' % E['score_v2'] if E.get('score_v2') is not None else '-', E['vias'],
        E['length_total'], '%.1f' % sp['loss_mW'] if sp.get('loss_mW') is not None else '-', mx)


if __name__ == '__main__':
    a = [x for x in sys.argv[1:] if '=' not in x]; kw = dict(x.split('=', 1) for x in sys.argv[1:] if '=' in x)
    if not a: sys.exit(__doc__)
    if a[0] == 'ranking':
        B, K, RG = (json.load(open(p)) for p in a[1:4])
        for i, k in enumerate(ranking(B, K, RG, a[4:], kw.get('selection', 'heuristic')), 1):
            m = k['feats']
            print('RANK %d %s  Key %.1f  Straight-line %.0f  Crossings %d' % (i, k['fpath'], k['keyname'], m['mst'], m['crossings']))
    elif a[0] == 'choose':
        vs = [load(o, kw.get('routes', 'routes_shape.json'), kw.get('drc', 'drc_finished.json'), kw.get('ir', '1') == '1') for o in a[2:]]
        for v in vs: print(row(v))
        from learning.judge_pair import Service
        pair = Service(model=kw['pair_model']) if kw.get('pair_model') else None
        best, prot = choose(vs, kw.get('rating', 'v3'), pair)
        for p in prot: print('  Comparison %s against %s: %s (%s)' % (os.path.basename(p['candidate']), os.path.basename(p['against']),
                                                                   'better' if p['better'] else 'not better', p['reason']))
        json.dump(dict(chosen=best['folder'], gate_check='valid (evaluation + KiCad DRC), then score v3 (evaluation.comparison)',
                       variants=[dict(folder=v['folder'], routes=v['routes'], is_valid=v['valid_whole'], drc=[v['drc_error'], v['drc_open'], v['drc_loose']],
                                       rank=v['E']['rank'], score_v3=v['E'].get('score_v3'), score_v2=v['E'].get('score_v2'), rating=v['E']['rating'],
                                       vias=v['E']['vias'], total_length=v['E']['length_total'], current_paths=v['E'].get('current_paths'))
                                  for v in vs], comparisons=prot), open(a[1], 'w'), indent=1)
        print('CHOSEN', best['folder'])
    else:
        sys.exit(__doc__)
