""
import json, math, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from gemeinsam.breitenprofil import zaehlen as _breiten_zaehlen


EXAKT = 0.001
NAH = 0.1


def _abst(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]; l2 = dx * dx + dy * dy
    t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def stoesse(R, B=None):
    ""
    je = {}
    for ti, t in enumerate(R['tracks']):
        for a, b in zip(t['pts'][:-1], t['pts'][1:]): je.setdefault((t['net'], t['layer']), []).append((ti, a, b))
    vias = {}
    for v in R['vias']: vias.setdefault(v['net'], []).append((v['x'], v['y']))
    enden = exakt = fast = 0; stellen = []
    for ti, t in enumerate(R['tracks']):
        if len(t['pts']) < 2: continue
        for e in (t['pts'][0], t['pts'][-1]):
            enden += 1
            d = [_abst(e, a, b) for tj, a, b in je.get((t['net'], t['layer']), []) if tj != ti
                 and abs((a[0] + b[0]) / 2 - e[0]) <= abs(b[0] - a[0]) / 2 + NAH and abs((a[1] + b[1]) / 2 - e[1]) <= abs(b[1] - a[1]) / 2 + NAH]
            d += [math.hypot(e[0] - x, e[1] - y) for x, y in vias.get(t['net'], []) if abs(e[0] - x) <= NAH and abs(e[1] - y) <= NAH]
            if not d: continue
            m = min(d)
            if m <= EXAKT: exakt += 1
            elif m < NAH: fast += 1; stellen.append(dict(netz=t['net'], lage=t['layer'], x=e[0], y=e[1], abstand=round(m, 4)))
    vfast = 0; vstellen = []
    for v in R['vias']:
        for (n_, L), segs in je.items():
            if n_ != v['net']: continue
            e = (v['x'], v['y'])
            d = [_abst(e, a, b) for _, a, b in segs
                 if abs((a[0] + b[0]) / 2 - e[0]) <= abs(b[0] - a[0]) / 2 + NAH and abs((a[1] + b[1]) / 2 - e[1]) <= abs(b[1] - a[1]) / 2 + NAH]
            if d and EXAKT < min(d) < NAH: vfast += 1; vstellen.append(dict(netz=v['net'], lage=L, x=v['x'], y=v['y'], abstand=round(min(d), 4)))
    out = dict(enden=enden, enden_exakt=exakt, fast_stoesse=fast, fast_stoesse_stellen=sorted(stellen, key=lambda s: -s['abstand'])[:40],
               vias_fast=vfast, vias_fast_stellen=sorted(vstellen, key=lambda s: -s['abstand'])[:40])
    if B is not None:
        from shapely.geometry import Point
        from gemeinsam.fluss import padgeo
        pads = {}
        for p in B['pads']:
            if p.get('net'): pads.setdefault(p['net'], []).append((p, padgeo(p)))
        im = mitte = loch = 0; neben = []
        for t in R['tracks']:
            if len(t['pts']) < 2: continue
            for e in (t['pts'][0], t['pts'][-1]):
                tr = [(math.hypot(e[0] - p['x'], e[1] - p['y']), bool(p.get('drill')), p) for p, g in pads.get(t['net'], [])
                      if t['layer'] in g and p['bbox'][0] - 0.03 <= e[0] <= p['bbox'][2] + 0.03 and p['bbox'][1] - 0.03 <= e[1] <= p['bbox'][3] + 0.03
                      and g[t['layer']].distance(Point(e)) <= 0.03]
                if not tr: continue
                d, dr, p = min(tr, key=lambda x: x[:2])
                im += 1; loch += dr
                if d <= EXAKT: mitte += 1
                elif not dr: neben.append(dict(netz=t['net'], lage=t['layer'], pad='%s:%s' % (p['ref'], p['num']), x=e[0], y=e[1], abstand=round(d, 4)))
        out.update(enden_im_pad=im, enden_padmitte=mitte, enden_im_pad_loch=loch, anteil_padmitte=round(100.0 * mitte / im, 1) if im else 0.0,
                   enden_neben_padmitte=sorted(neben, key=lambda s: -s['abstand'])[:40])
    return out


def kennzahlen(R, B=None):
    seg = kurz = 0; L = okt = 0.0
    bw = bw1 = bw3 = hoe = 0
    knoten = {}
    for t in R['tracks']:
        _c, _h = _breiten_zaehlen([math.dist(a, b) for a, b in zip(t['pts'][:-1], t['pts'][1:])], t['w'])
        bw += _c; bw1 += _c >= 1; bw3 += _c >= 3; hoe += _h
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            d = math.dist(a, b)
            if d < 1e-9: continue
            seg += 1; L += d; kurz += d < 0.1
            w = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
            r = (w + 360.0) % 45.0
            if r < 0.5 or r > 44.5: okt += d
            knoten.setdefault((t['net'], t['layer'], round(a[0], 3), round(a[1], 3)), []).append(w)
            knoten.setdefault((t['net'], t['layer'], round(b[0], 3), round(b[1], 3)), []).append(w + 180.0)


    kn = sum(1 for w in knoten.values() if len(w) == 2 and abs((w[0] - w[1]) % 360.0 - 180.0) > 1.0)
    fl = set(R.get('flaechennetze') or [])
    if B is not None:
        from gemeinsam.laenge import aussen
        La = sum(sum(s) for s in aussen(R, B))
    else: La = L
    return dict(stoesse(R, B), laenge_aussen=round(La, 1), laenge_im_pad=round(L - La, 1), bahnen=len(R['tracks']), segmente=seg, knicke=kn, mini=kurz, laenge=round(L, 1), anteil_45=round(100 * okt / L, 1) if L else 0.0,
                vias=len(R['vias']), signal_vias=sum(1 for v in R['vias'] if v['net'] not in fl and not v.get('strom_via')),
                konflikte=len(R.get('conflicts') or {}), offen=len(R.get('failed') or []),
                breitenwechsel=bw, bahnen_wechsel=bw1, bahnen_wechsel3=bw3, hoecker=hoe)

if __name__ == '__main__':
    dateien = [a for a in sys.argv[1:] if '=' not in a]
    _b = [a.split('=', 1)[1] for a in sys.argv[1:] if a.startswith('board=')]
    if not _b and len(dateien) == 2 and 'pads' in json.load(open(dateien[1])): _b = [dateien.pop()]
    B = json.load(open(_b[0])) if _b else None
    for f in dateien:
        k = kennzahlen(json.load(open(f)), B)
        if 'json=1' in sys.argv: print(json.dumps(dict(datei=f, **k)))
        else: print('%-30s Bahnen %4d  Segmente %5d  Knicke %5d  Mini %4d  Laenge %7.1f mm  0/45/90 %5.1f %%  Vias %4d (Signal %4d)  Konflikte %d  offen %d'
                    '  Breitenwechsel %4d (Bahnen %3d, ab 3: %3d)  Hoecker %3d' % (
            f.split('/')[-1][:30], k['bahnen'], k['segmente'], k['knicke'], k['mini'], k['laenge'], k['anteil_45'], k['vias'], k['signal_vias'], k['konflikte'], k['offen'],
            k['breitenwechsel'], k['bahnen_wechsel'], k['bahnen_wechsel3'], k['hoecker'])
            + '  Bahnenden %4d (exakt auf eigenem Netz %4d, Fast-Stoesse %3d, Vias neben eigener Bahn %3d)' % (k['enden'], k['enden_exakt'], k['fast_stoesse'], k['vias_fast'])
            + ('  im Pad %4d, in der Padmitte %4d (%5.1f %%), in Durchsteck-Pads %3d' % (k['enden_im_pad'], k['enden_padmitte'], k['anteil_padmitte'], k['enden_im_pad_loch']) if B else '')
            + ('  Laenge ausserhalb der eigenen Pads %7.1f mm (im Pad %5.1f)' % (k['laenge_aussen'], k['laenge_im_pad']) if B else ''))
