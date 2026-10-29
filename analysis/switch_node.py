import math
import re


def an(R):
    return str(R.get('sw_compact', (R.get('options') or {}).get('sw_compact', '0'))).lower() in ('1', 'true')


def correct(B, K):
    pads = B['pads']; nets = K['net_list']; cons = K.get('specs') or {}
    fps = {f['ref']: f for f in B.get('fps', [])}
    byref = {r: {p['net'] for p in pads if p['ref'] == r} for r in sorted(fps)}
    def value(n, field):
        x = nets.get(n, {}).get(field)
        return float(x) if isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) and x > 0 else None
    def power(n):
        return nets.get(n, {}).get('category') in ('current_val', 'supply') and value(n, 'current_A') and value(n, 'voltage_V')
    changed = []
    for n, k in sorted(nets.items()):
        if k.get('category') != 'sw' or k.get('sw_design'): continue
        refs = sorted({p['ref'] for p in pads if p['net'] == n})
        inds = [r for r in refs if re.match(r'^(L|T)\d|^(L|T)_', r)]
        if not inds: continue
        fl = k.get('flow') or {}
        if fl.get('unit_source') == 'spec' or k.get('current_A_source') == 'spec': continue
        if any(r in (cons.get('current_val') or {}) or any(f"{p['ref']}:{p['num']}" in (cons.get('current_val') or {}) for p in pads if p['ref'] == r and p['net'] == n) for r in refs): continue
        candidates = []
        for r in inds:
            if r.startswith('L'):
                outputs = sorted(m for m in byref[r] - {n} if nets.get(m, {}).get('category') in ('supply', 'current_val') and value(m, 'current_A'))
                for m in outputs:

                    w = k.get('converter') or {}
                    if w.get('category') == 'boost':
                        a = value(n, 'current_A')
                        if a: candidates.append((a * 1.3, 'sw_boost_InductorMeanCurrent_x1.3_RippleAssumption'))
                    else:
                        candidates.append((value(m, 'current_A') * 1.3, f'sw_buck_OutPort({m})_x1.3_RippleAssumption'))
            else:

                inside = sorted(m for m in byref[r] - {n} if nets.get(m, {}).get('category') in ('current_val', 'supply') and value(m, 'voltage_V'))
                outputs = set()
                for d, ns in byref.items():
                    if not re.match(r'^D\d|^D_', d) or not (ns & (byref[r] - {n} - set(inside))): continue
                    outputs.update(m for m in ns - byref[r] if power(m))
                primary = {p['net'] for p in pads if p['ref'] == r and re.match(r'^P[12]$', str(p.get('pinfunction', p.get('pinfunc', p.get('func', '')))))}
                if primary and n not in primary:
                    for d, ns in byref.items():
                        if re.match(r'^D\d|^D_', d) and n in ns:
                            for m in sorted(ns - byref[r]):
                                if value(m, 'current_A') and nets.get(m, {}).get('category') in ('current_val', 'supply'):
                                    candidates.append((value(m, 'current_A') * 1.3, f'sw_flyback_Secondary({m})_x1.3_PulseAssumption'))
                elif len(inside) == 1 and outputs:
                    vin = inside[0]
                    a = sum(value(m, 'voltage_V') * value(m, 'current_A') for m in sorted(outputs)) / (value(vin, 'voltage_V') * .8)
                    candidates.append((3 * a, f'sw_flyback_Paus({",".join(sorted(outputs))})/Vin({vin})/eta0.8_x3_PulseAssumption'))

            match = re.search(r'(?<![\w.])(\d+(?:\.\d+)?)\s*A\b', fps[r].get('value', ''), re.I)
            if match and float(match[1]) > 0: candidates.append((float(match[1]), f'sw_RatedCurrent_Value({r})'))
        a, source = max(candidates, key=lambda q: (q[0], q[1])) if candidates else (2., 'sw_Assumption_2A_ConverterData_unknown')

        if (value(n, 'current_A') or 0) > a:
            a, source = value(n, 'current_A'), 'sw_previous_larger_Estimate:' + str(k.get('current_A_source'))
        k['sw_design'] = dict(current_prev_A=k.get('current_A'), source_prev=k.get('current_A_source'),
                                model='Design assumption; RMS/peak/duty cycle and Isat not demonstrated')
        k.update(current_A=a, current_A_source=source)
        if fl.get('unit') == 'A' and sum((fl.get('load') or {}).values()) > 0:
            total = sum(fl['load'].values())
            fl['load'] = {p: x * a / total for p, x in fl['load'].items()}
            fl.update(total=a, unit_source='estimate', provenance={p: source for p in fl['load']})
        changed.append(n)
    if changed:
        from analysis.netclasses import width_after_current
        by_net = {n: [i for i, p in enumerate(pads) if p['net'] == n] for n in nets}
        widths = {n: dict(k) for n, k in nets.items()}
        width_after_current(B, widths, cons, by_net)
        for n in changed: nets[n].update(widths[n])
    return changed


def compact(B, R, K, n, cu_layers, vacant):
    from shapely.geometry import Polygon, LineString
    from shapely.ops import unary_union
    from learning import current_zones as SZ
    terminals = [Polygon(q).buffer(0) for p in B['pads'] if p['net'] == n and re.match(r'^(U|IC|Q|L|T|D)(?:\d|_)', p['ref'])
                 for qs in (p.get('polys') or {}).values() for q in qs if len(q) >= 3]
    if len(terminals) < 2: return {}, dict(reason='SW power pads not unambiguous', model='sw_compact')
    out = {}; k = K['net_list'][n]
    from analysis.current_width import ipc2221, copper_thickness
    from learning.current_zones_minimax import segment_currents
    currents, total = segment_currents(B, R, K, n)
    cons = K.get('specs') or {}
    hull = unary_union(terminals).convex_hull
    for L in cu_layers:
        inner = L.startswith('In')


        fixed = unary_union([g for m, g in SZ._parts(B, dict(R, zone_list=[]), L) if m == n])
        paths = []
        for ti,t in enumerate(R.get('tracks', [])):
            if t.get('net') != n or t['layer'] != L: continue
            for si,(a,b) in enumerate(zip(t['pts'][:-1],t['pts'][1:])):
                current = currents.get((ti,si), total)
                if current <= 0: continue
                width = max(float(k.get('wmin') or .25),
                            ipc2221(current, float(cons.get('heating_K') or 10.), copper_thickness(cons, inner), inner=inner),
                            float(k.get('w_budget') or 0.) * (current / total if total else 1.) * copper_thickness(cons) / copper_thickness(cons, inner))
                domain = hull.buffer(width / 2, join_style=2)
                paths.append(LineString([a,b]).buffer(width / 2, cap_style=2, join_style=2).intersection(domain))
        if not paths: continue
        growth = unary_union(paths).intersection(vacant(n, L))
        out[L] = fixed.union(growth).buffer(0)
    return out, dict(model='sw_compact', source=k.get('current_A_source'), current_A=k.get('current_A'))
