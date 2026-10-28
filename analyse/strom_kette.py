""
import json, math, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

RHO = 1.72e-5
T_AUSSEN, T_INNEN = 0.035, 0.0175


def _pads(B):
    return {'%s:%s' % (p['ref'], p.get('num')): p for p in B['pads']}


def _lasten(K, n, i_tot):
    ""
    fl = (K['netze'].get(n) or {}).get('fluss') or {}
    last = {p: float(q) for p, q in (fl.get('last') or {}).items()}
    s = sum(last.values())
    if s <= 0: return {}, fl.get('quelle') or []
    return {p: q * i_tot / s for p, q in last.items() if q > 0}, fl.get('quelle') or []


def _flaechenvorbereitung(B, R):
    from gemeinsam.zonen import vereinheitlichen, boardgeometrie, viageometrie
    from analyse.kupfermodell import geometrie
    R = viageometrie(vereinheitlichen(R))
    B = boardgeometrie(B, R)
    d_via = float(str((R.get('optionen') or {}).get('via') or '0.45/0.20').split('/')[0])
    lagen = ['F'] + ['In%d' % i for i in range(1, int(B.get('lagen') or 2) - 1)] + ['B']
    return B, R, {L: geometrie(dict(B, pads=[]), R, L, via_d=d_via) for L in lagen}


def kupfer_flaeche(B, R, n, *, _vorbereitet=None):
    ""
    from shapely.geometry import Point
    from shapely.ops import unary_union
    from lernen.kupferbalken import pad_polygon
    B, R, geometrien = _vorbereitet if _vorbereitet is not None else _flaechenvorbereitung(B, R)
    via_spec = str((R.get('optionen') or {}).get('via') or '0.45/0.20').split('/')
    d_bohr = float(via_spec[1]) if len(via_spec) > 1 else 0.2
    lagen = ['F'] + ['In%d' % i for i in range(1, int(B.get('lagen') or 2) - 1)] + ['B']
    je, eq = {}, 0.0
    for L in lagen:
        g = [q for net, q in geometrien[L] if net == n]
        if not g: continue
        pads = [pad_polygon(B, p, q) for p in B['pads'] if p['net'] == n for q in (p.get('polys') or {}).get(L, []) if len(q) >= 3]
        a = unary_union(g).difference(unary_union(pads)).area if pads else unary_union(g).area

        bohr = [Point(v['x'], v['y']).buffer(float(v.get('drill') or d_bohr) / 2, 32) for v in R.get('vias', [])
                if v.get('net') == n and L in (v.get('layers') or lagen)]
        if bohr: a -= unary_union(bohr).intersection(unary_union(g)).area
        if a <= 0: continue
        t = (B.get('stackup') or {}).get(L, {}).get('copper_um')
        t = float(t) / 1000 if t is not None else (T_AUSSEN if L in ('F', 'B') else T_INNEN)
        je[L] = round(a, 3); eq += a * t / T_AUSSEN
    return eq, je


def _tot(ir, n, j_stern, pad_tot=0.0, anteil=0.1):
    ""
    h = (ir.get('J_hist') or {}).get(n)
    if not h: return None
    tot = sum(f for f, oben in zip(h['flaeche_mm2'], h['kanten'][1:]) if oben <= anteil * j_stern)
    ges = sum(h['flaeche_mm2'])
    tot, ges = max(0.0, tot - pad_tot), ges - pad_tot
    return round(tot / ges, 3) if ges > 0 else None

def _pads_ohne_strom(B, K, n, quelle):
    ""
    from shapely.geometry import Polygon
    last = ((K['netze'].get(n) or {}).get('fluss') or {}).get('last') or {}
    a = 0.0
    for p in B['pads']:
        k = '%s:%s' % (p['ref'], p.get('num'))
        if p['net'] != n or k in quelle or float(last.get(k, 0.0)) > 0: continue
        a += sum(Polygon(q).buffer(0).area for qs in (p.get('polys') or {}).values() for q in qs if len(q) >= 3)
    return a


def kette(B, R, K, ir=None):
    ""
    if ir is None or 'J_max_A_mm2' not in ir:
        from analyse.kupfermodell import ir_werte
        from analyse.minimax import lastfehler

        kk = dict(K, netze={n:k for n,k in K['netze'].items() if not lastfehler(k)})
        ir = ir_werte(B, R, kk) if kk['netze'] else {}
    P = _pads(B); rq = RHO / T_AUSSEN; out = {}; ausgelassen = {}; vorbereitet = None
    for n, p_mw in sorted((ir.get('verlust_je_netz_mW') or {}).items()):
        i_tot = (ir.get('strom_je_netz_A') or {}).get(n) or 0.0
        if i_tot <= 1e-6 or p_mw <= 1e-9: continue
        last, quelle = _lasten(K, n, i_tot)
        qs = [P[q] for q in quelle if q in P]
        if not last or not qs or any(p not in P for p in last) or any(q not in P for q in quelle):

            ausgelassen[n] = 'keine Last' if not last else 'keine Quelle' if not qs else 'Pad fehlt im Board'
            continue
        s_id = 0.0
        for p, i in last.items():
            if p not in P: continue
            d = min(math.hypot(P[p]['x'] - q['x'], P[p]['y'] - q['y']) for q in qs)
            s_id += i * d
        if vorbereitet is None: vorbereitet = _flaechenvorbereitung(B, R)
        a_eq, je = kupfer_flaeche(B, R, n, _vorbereitet=vorbereitet)
        if a_eq <= 1e-6 or s_id <= 1e-9: continue
        p_ideal = rq * s_id ** 2 / a_eq * 1000
        j_stern = s_id / (a_eq * T_AUSSEN)
        jmax = (ir.get('J_max_A_mm2') or {}).get(n)
        out[n] = dict(roh=dict(wirkungsgrad=min(1.0, p_ideal / p_mw), glied=jmax / j_stern if jmax is not None else None,
                               kupfer_mm2=a_eq, l_eff_mm=s_id / i_tot, strom_A=i_tot,
                               verlust_mW=p_mw, verlust_ideal_mW=p_ideal), wirkungsgrad=round(min(1.0, p_ideal / p_mw), 4), glied=round(jmax / j_stern, 2) if jmax else None, j_stern=round(j_stern, 3),
                      kupfer_mm2=round(a_eq, 2), kupfer_je_lage=je, l_eff_mm=round(s_id / i_tot, 2), strom_A=round(i_tot, 3),
                      verlust_mW=round(p_mw, 3), verlust_ideal_mW=round(p_ideal, 3),
                      r_ist_mohm=round(p_mw / i_tot ** 2, 3), r_ideal_mohm=round(p_ideal / i_tot ** 2, 3),
                      abfall_mV=round((ir.get('max_abfall_mV') or {}).get(n) or 0.0, 2),
                      glied_ort=(ir.get('J_max_ort') or {}).get(n), totes_kupfer=_tot(ir, n, j_stern, _pads_ohne_strom(B, K, n, quelle)))
    if not out: return dict(netze={}, wirkungsgrad=None, wirkungsgrad_median=None, glied_max=None, kupfer_mm2=0.0, ausgelassen=ausgelassen)
    ws = sorted(v['wirkungsgrad'] for v in out.values())
    return dict(netze=out, wirkungsgrad=round(sum(v['roh']['verlust_ideal_mW'] for v in out.values()) / sum(v['roh']['verlust_mW'] for v in out.values()), 4),
                wirkungsgrad_median=ws[len(ws) // 2], glied_max=max((v['glied'] or 0) for v in out.values()),
                kupfer_mm2=round(sum(v['kupfer_mm2'] for v in out.values()), 1),
                totes_kupfer=round(sum((v['totes_kupfer'] or 0) * v['kupfer_mm2'] for v in out.values()) /
                                   max(1e-9, sum(v['kupfer_mm2'] for v in out.values() if v['totes_kupfer'] is not None)), 3),
                ausgelassen=ausgelassen)


def main():
    B, R, K = (json.load(open(x)) for x in sys.argv[1:4])
    E = kette(B, R, K)
    print('Kettenmaß: Wirkungsgrad %s (verlustgewichtet), Median %s, schwächstes Glied %s, Kupfer %s mm², totes Kupfer %s' % (
        E['wirkungsgrad'], E['wirkungsgrad_median'], E['glied_max'], E['kupfer_mm2'], E.get('totes_kupfer')))
    for n, v in sorted(E['netze'].items(), key=lambda kv: kv[1]['wirkungsgrad']):
        print('  %-14s η %5.1f %%  Glied %5s  Kupfer %7.1f mm²  L %6.1f mm  I %5.2f A  R %7.2f mΩ (ideal %6.2f)  Abfall %6.1f mV' % (
            n, 100 * v['wirkungsgrad'], v['glied'], v['kupfer_mm2'], v['l_eff_mm'], v['strom_A'], v['r_ist_mohm'], v['r_ideal_mohm'], v['abfall_mV'])
            + ('  tot %.0f %%' % (100 * v['totes_kupfer']) if v.get('totes_kupfer') is not None else '')
            + ('  Glied bei %s' % (v['glied_ort'],) if v.get('glied_ort') else ''))


if __name__ == '__main__':
    main()
