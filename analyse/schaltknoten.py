""
import math
import re


def an(R):
    return str(R.get('sw_kompakt', (R.get('optionen') or {}).get('sw_kompakt', '0'))).lower() in ('1', 'true')


def korrigiere(B, K):
    ""
    pads = B['pads']; nets = K['netze']; vg = K.get('vorgaben') or {}
    fps = {f['ref']: f for f in B.get('fps', [])}
    byref = {r: {p['net'] for p in pads if p['ref'] == r} for r in sorted(fps)}
    def value(n, field):
        x = nets.get(n, {}).get(field)
        return float(x) if isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) and x > 0 else None
    def power(n):
        return nets.get(n, {}).get('art') in ('strom', 'versorgung') and value(n, 'strom_A') and value(n, 'spannung_V')
    changed = []
    for n, k in sorted(nets.items()):
        if k.get('art') != 'sw' or k.get('sw_auslegung'): continue
        refs = sorted({p['ref'] for p in pads if p['net'] == n})
        inds = [r for r in refs if re.match(r'^(L|T)\d|^(L|T)_', r)]
        if not inds: continue
        fl = k.get('fluss') or {}
        if fl.get('einheit_quelle') == 'vorgabe' or k.get('strom_A_quelle') == 'vorgabe': continue
        if any(r in (vg.get('strom') or {}) or any(f"{p['ref']}:{p['num']}" in (vg.get('strom') or {}) for p in pads if p['ref'] == r and p['net'] == n) for r in refs): continue
        candidates = []
        for r in inds:
            if r.startswith('L'):
                outputs = sorted(m for m in byref[r] - {n} if nets.get(m, {}).get('art') in ('versorgung', 'strom') and value(m, 'strom_A'))
                for m in outputs:

                    w = k.get('wandler') or {}
                    if w.get('art') == 'boost':
                        a = value(n, 'strom_A')
                        if a: candidates.append((a * 1.3, 'sw_boost_Induktormittelstrom_x1.3_Rippleannahme'))
                    else:
                        candidates.append((value(m, 'strom_A') * 1.3, f'sw_buck_Ausgang({m})_x1.3_Rippleannahme'))
            else:

                ins = sorted(m for m in byref[r] - {n} if nets.get(m, {}).get('art') in ('strom', 'versorgung') and value(m, 'spannung_V'))
                outputs = set()
                for d, ns in byref.items():
                    if not re.match(r'^D\d|^D_', d) or not (ns & (byref[r] - {n} - set(ins))): continue
                    outputs.update(m for m in ns - byref[r] if power(m))
                primary = {p['net'] for p in pads if p['ref'] == r and re.match(r'^P[12]$', str(p.get('pinfunction', p.get('pinfunc', p.get('func', '')))))}
                if primary and n not in primary:
                    for d, ns in byref.items():
                        if re.match(r'^D\d|^D_', d) and n in ns:
                            for m in sorted(ns - byref[r]):
                                if value(m, 'strom_A') and nets.get(m, {}).get('art') in ('strom', 'versorgung'):
                                    candidates.append((value(m, 'strom_A') * 1.3, f'sw_flyback_Sekundaer({m})_x1.3_Pulsannahme'))
                elif len(ins) == 1 and outputs:
                    vin = ins[0]
                    a = sum(value(m, 'spannung_V') * value(m, 'strom_A') for m in sorted(outputs)) / (value(vin, 'spannung_V') * .8)
                    candidates.append((3 * a, f'sw_flyback_Paus({",".join(sorted(outputs))})/Vin({vin})/eta0.8_x3_Pulsannahme'))

            match = re.search(r'(?<![\w.])(\d+(?:\.\d+)?)\s*A\b', fps[r].get('value', ''), re.I)
            if match and float(match[1]) > 0: candidates.append((float(match[1]), f'sw_Nennstrom_Wert({r})'))
        a, source = max(candidates, key=lambda q: (q[0], q[1])) if candidates else (2., 'sw_Annahme_2A_Wandlerdaten_unbekannt')

        if (value(n, 'strom_A') or 0) > a:
            a, source = value(n, 'strom_A'), 'sw_bisherige_groessere_Schaetzung:' + str(k.get('strom_A_quelle'))
        k['sw_auslegung'] = dict(strom_alt_A=k.get('strom_A'), quelle_alt=k.get('strom_A_quelle'),
                                modell='Auslegungsannahme; RMS/Peak/Tastgrad und Isat nicht nachgewiesen')
        k.update(strom_A=a, strom_A_quelle=source)
        if fl.get('einheit') == 'A' and sum((fl.get('last') or {}).values()) > 0:
            total = sum(fl['last'].values())
            fl['last'] = {p: x * a / total for p, x in fl['last'].items()}
            fl.update(gesamt=a, einheit_quelle='schaetzung', herkunft={p: source for p in fl['last']})
        changed.append(n)
    if changed:
        from analyse.klassen import breite_nach_strom
        bynet = {n: [i for i, p in enumerate(pads) if p['net'] == n] for n in nets}
        widths = {n: dict(k) for n, k in nets.items()}
        breite_nach_strom(B, widths, vg, bynet)
        for n in changed: nets[n].update(widths[n])
    return changed


def kompakt(B, R, K, n, lagen, frei):
    ""
    from shapely.geometry import Polygon, LineString
    from shapely.ops import unary_union
    from lernen import stromzonen as SZ
    terminals = [Polygon(q).buffer(0) for p in B['pads'] if p['net'] == n and re.match(r'^(U|IC|Q|L|T|D)(?:\d|_)', p['ref'])
                 for qs in (p.get('polys') or {}).values() for q in qs if len(q) >= 3]
    if len(terminals) < 2: return {}, dict(grund='SW-Leistungspads nicht eindeutig', modell='sw_kompakt')
    out = {}; k = K['netze'][n]
    from analyse.strombreite import ipc2221, kupferdicke
    from lernen.stromzonen_minimax import segmentstroeme
    currents, total = segmentstroeme(B, R, K, n)
    vg = K.get('vorgaben') or {}
    hull = unary_union(terminals).convex_hull
    for L in lagen:
        innen = L.startswith('In')


        fixed = unary_union([g for m, g in SZ._teile(B, dict(R, zonen=[]), L) if m == n])
        paths = []
        for ti,t in enumerate(R.get('tracks', [])):
            if t.get('net') != n or t['layer'] != L: continue
            for si,(a,b) in enumerate(zip(t['pts'][:-1],t['pts'][1:])):
                current = currents.get((ti,si), total)
                if current <= 0: continue
                width = max(float(k.get('wmin') or .25),
                            ipc2221(current, float(vg.get('erwaermung_K') or 10.), kupferdicke(vg, innen), innen=innen),
                            float(k.get('w_budget') or 0.) * (current / total if total else 1.) * kupferdicke(vg) / kupferdicke(vg, innen))
                domain = hull.buffer(width / 2, join_style=2)
                paths.append(LineString([a,b]).buffer(width / 2, cap_style=2, join_style=2).intersection(domain))
        if not paths: continue
        growth = unary_union(paths).intersection(frei(n, L))
        out[L] = fixed.union(growth).buffer(0)
    return out, dict(modell='sw_kompakt', quelle=k.get('strom_A_quelle'), strom_A=k.get('strom_A'))
