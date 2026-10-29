import json, math, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

RHO = 1.72e-5
T_OUTER, T_INNER = 0.035, 0.0175


def _pads(B):
    return {'%s:%s' % (p['ref'], p.get('num')): p for p in B['pads']}


def _loads(K, n, i_tot):
    fl = (K['net_list'].get(n) or {}).get('flow') or {}
    load = {p: float(q) for p, q in (fl.get('load') or {}).items()}
    s = sum(load.values())
    if s <= 0: return {}, fl.get('source') or []
    return {p: q * i_tot / s for p, q in load.items() if q > 0}, fl.get('source') or []


def _area_preparation(B, R):
    from common.zones import unify, board_geometry, via_geometry
    from analysis.copper_model import geometry
    R = via_geometry(unify(R))
    B = board_geometry(B, R)
    d_via = float(str((R.get('options') or {}).get('via') or '0.45/0.20').split('/')[0])
    cu_layers = ['F'] + ['In%d' % i for i in range(1, int(B.get('cu_layers') or 2) - 1)] + ['B']
    return B, R, {L: geometry(dict(B, pads=[]), R, L, via_d=d_via) for L in cu_layers}


def copper_area(B, R, n, *, _prepared=None):
    from shapely.geometry import Point
    from shapely.ops import unary_union
    from learning.copper_bars import pad_polygon
    B, R, geometries = _prepared if _prepared is not None else _area_preparation(B, R)
    via_spec = str((R.get('options') or {}).get('via') or '0.45/0.20').split('/')
    d_bore = float(via_spec[1]) if len(via_spec) > 1 else 0.2
    cu_layers = ['F'] + ['In%d' % i for i in range(1, int(B.get('cu_layers') or 2) - 1)] + ['B']
    per, eq = {}, 0.0
    for L in cu_layers:
        g = [q for net, q in geometries[L] if net == n]
        if not g: continue
        pads = [pad_polygon(B, p, q) for p in B['pads'] if p['net'] == n for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3]
        a = unary_union(g).difference(unary_union(pads)).area if pads else unary_union(g).area

        bore = [Point(v['x'], v['y']).buffer(float(v.get('drill') or d_bore) / 2, 32) for v in R.get('vias', [])
                if v.get('net') == n and L in (v.get('layers') or cu_layers)]
        if bore: a -= unary_union(bore).intersection(unary_union(g)).area
        if a <= 0: continue
        t = (B.get('stackup') or {}).get(L, {}).get('copper_um')
        t = float(t) / 1000 if t is not None else (T_OUTER if L in ('F', 'B') else T_INNER)
        per[L] = round(a, 3); eq += a * t / T_OUTER
    return eq, per


def _tot(ir, n, j_star, pad_tot=0.0, share=0.1):
    h = (ir.get('J_hist') or {}).get(n)
    if not h: return None
    tot = sum(f for f, upper in zip(h['area_mm2'], h['edges'][1:]) if upper <= share * j_star)
    ges = sum(h['area_mm2'])
    tot, ges = max(0.0, tot - pad_tot), ges - pad_tot
    return round(tot / ges, 3) if ges > 0 else None

def _pads_without_current(B, K, n, source):
    from shapely.geometry import Polygon
    load = ((K['net_list'].get(n) or {}).get('flow') or {}).get('load') or {}
    a = 0.0
    for p in B['pads']:
        k = '%s:%s' % (p['ref'], p.get('num'))
        if p['net'] != n or k in source or float(load.get(k, 0.0)) > 0: continue
        a += sum(Polygon(q).buffer(0).area for qs in (p.get('polys') or {}).values() for q in qs if len(q) >= 3)
    return a


def link_chain(B, R, K, ir=None):
    if ir is None or 'J_max_A_mm2' not in ir:
        from analysis.copper_model import ir_values
        from analysis.minimax import load_error

        kk = dict(K, net_list={n:k for n,k in K['net_list'].items() if not load_error(k)})
        ir = ir_values(B, R, kk) if kk['net_list'] else {}
    P = _pads(B); rq = RHO / T_OUTER; out = {}; skipped = {}; prepared = None
    for n, p_mw in sorted((ir.get('loss_per_net_mW') or {}).items()):
        i_tot = (ir.get('current_per_net_A') or {}).get(n) or 0.0
        if i_tot <= 1e-6 or p_mw <= 1e-9: continue
        load, source = _loads(K, n, i_tot)
        qs = [P[q] for q in source if q in P]
        if not load or not qs or any(p not in P for p in load) or any(q not in P for q in source):

            skipped[n] = 'no load' if not load else 'no source' if not qs else 'Pad missing on the board'
            continue
        s_id = 0.0
        for p, i in load.items():
            if p not in P: continue
            d = min(math.hypot(P[p]['x'] - q['x'], P[p]['y'] - q['y']) for q in qs)
            s_id += i * d
        if prepared is None: prepared = _area_preparation(B, R)
        a_eq, per = copper_area(B, R, n, _prepared=prepared)
        if a_eq <= 1e-6 or s_id <= 1e-9: continue
        p_ideal = rq * s_id ** 2 / a_eq * 1000
        j_star = s_id / (a_eq * T_OUTER)
        jmax = (ir.get('J_max_A_mm2') or {}).get(n)
        out[n] = dict(raw_data=dict(efficiency=min(1.0, p_ideal / p_mw), element=jmax / j_star if jmax is not None else None,
                               copper_mm2=a_eq, l_eff_mm=s_id / i_tot, current_A=i_tot,
                               loss_mW=p_mw, loss_ideal_mW=p_ideal), efficiency=round(min(1.0, p_ideal / p_mw), 4), element=round(jmax / j_star, 2) if jmax else None, j_star=round(j_star, 3),
                      copper_mm2=round(a_eq, 2), copper_per_layer=per, l_eff_mm=round(s_id / i_tot, 2), current_A=round(i_tot, 3),
                      loss_mW=round(p_mw, 3), loss_ideal_mW=round(p_ideal, 3),
                      r_actual_mohm=round(p_mw / i_tot ** 2, 3), r_ideal_mohm=round(p_ideal / i_tot ** 2, 3),
                      drop_mV=round((ir.get('max_drop_mV') or {}).get(n) or 0.0, 2),
                      element_location=(ir.get('J_max_location') or {}).get(n), totes_copper=_tot(ir, n, j_star, _pads_without_current(B, K, n, source)))
    if not out: return dict(net_list={}, efficiency=None, efficiency_median=None, element_max=None, copper_mm2=0.0, skipped=skipped)
    ws = sorted(v['efficiency'] for v in out.values())
    return dict(net_list=out, efficiency=round(sum(v['raw_data']['loss_ideal_mW'] for v in out.values()) / sum(v['raw_data']['loss_mW'] for v in out.values()), 4),
                efficiency_median=ws[len(ws) // 2], element_max=max((v['element'] or 0) for v in out.values()),
                copper_mm2=round(sum(v['copper_mm2'] for v in out.values()), 1),
                totes_copper=round(sum((v['totes_copper'] or 0) * v['copper_mm2'] for v in out.values()) /
                                   max(1e-9, sum(v['copper_mm2'] for v in out.values() if v['totes_copper'] is not None)), 3),
                skipped=skipped)


def main():
    B, R, K = (json.load(open(x)) for x in sys.argv[1:4])
    E = link_chain(B, R, K)
    print('Chain measure: efficiency %s (loss-weighted), median %s, weakest link %s, copper %s mm², dead copper %s' % (
        E['efficiency'], E['efficiency_median'], E['element_max'], E['copper_mm2'], E.get('totes_copper')))
    for n, v in sorted(E['net_list'].items(), key=lambda kv: kv[1]['efficiency']):
        print('  %-14s η %5.1f %%  link %5s  copper %7.1f mm²  L %6.1f mm  I %5.2f A  R %7.2f mΩ (ideal %6.2f)  drop %6.1f mV' % (
            n, 100 * v['efficiency'], v['element'], v['copper_mm2'], v['l_eff_mm'], v['current_A'], v['r_actual_mohm'], v['r_ideal_mohm'], v['drop_mV'])
            + ('  tot %.0f %%' % (100 * v['totes_copper']) if v.get('totes_copper') is not None else '')
            + ('  Link at %s' % (v['element_location'],) if v.get('element_location') else ''))


if __name__ == '__main__':
    main()
