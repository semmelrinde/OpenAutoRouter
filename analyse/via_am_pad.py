""
import heapq, json, math, sys
import shapely
from shapely.geometry import Point, Polygon


def _klasse(ref, KL):
    t = (KL or {}).get('teile', {}).get(ref) or {}
    if t.get('entkopplung'): return 'entkopplung'
    if t.get('rolle') == 'chip': return 'chip'
    return 'rest'


def auswerten(D, R, KL=None, via_d=None):
    fl = set(R.get('flaechennetze') or (KL or {}).get('gnd') or [])
    opt = R.get('optionen') or {}
    if via_d is None: via_d = float(str(opt.get('via', '0.45/0.20')).split('/')[0])
    clr = float(opt.get('clr', 0.13))
    gndl = set(R.get('gndlagen') or [])
    vr = via_d / 2; kreis_a = Point(0, 0).buffer(vr, 32).area
    pads = []
    for p in D['pads']:
        if p['net'] not in fl or any(L in gndl for L in p['layers']): continue
        L = p['layers'][0]
        pads.append(dict(p=p, lage=L, poly=shapely.union_all([Polygon(q) for q in p['polys'][L] if len(q) >= 3])))
    zeilen = []
    for net in sorted(fl):
        vias = [(v['x'], v['y']) for v in R['vias'] if v['net'] == net]
        trs = [t for t in R['tracks'] if t['net'] == net]
        kn = {}
        pos = []; adj = []
        def knoten(L, x, y):
            k = (L, round(x, 2), round(y, 2))
            if k not in kn: kn[k] = len(pos); pos.append((L, x, y)); adj.append([])
            return kn[k]
        for t in trs:
            ids = [knoten(t['layer'], x, y) for x, y in t['pts']]
            for a, b, pa, pb in zip(ids[:-1], ids[1:], t['pts'][:-1], t['pts'][1:]):
                d = math.dist(pa, pb); adj[a].append((b, d)); adj[b].append((a, d))
        via_an = {}
        for i, (L, x, y) in enumerate(pos):
            for j, (vx, vy) in enumerate(vias):
                if math.hypot(x - vx, y - vy) <= 0.03: via_an[i] = j; break
        for q in pads:
            p = q['p']
            if p['net'] != net: continue
            g = q['poly']; gb = g.buffer(0.02)
            dist = {}; heap = []
            for i, (L, x, y) in enumerate(pos):
                if L == q['lage'] and gb.contains(Point(x, y)): dist[i] = 0.0; heap.append((0.0, i))
            heapq.heapify(heap)
            best = None
            for j, (vx, vy) in enumerate(vias):
                if gb.contains(Point(vx, vy)):
                    c = (0.0, math.hypot(vx - p['x'], vy - p['y']), j)
                    if best is None or c < best: best = c
            fertig = set()
            while heap:
                d, i = heapq.heappop(heap)
                if i in fertig: continue
                fertig.add(i)
                if best is not None and d > best[0]: break
                if i in via_an:
                    vx, vy = vias[via_an[i]]; c = (d, math.hypot(vx - p['x'], vy - p['y']), via_an[i])
                    if best is None or c < best: best = c
                for m, w in adj[i]:
                    if d + w < dist.get(m, 1e18): dist[m] = d + w; heapq.heappush(heap, (d + w, m))
            z = dict(ref=p['ref'], num=p['num'], netz=net, klasse=_klasse(p['ref'], KL), lage=q['lage'],
                     pad_mm2=round(g.area, 3))
            if best is None: z.update(via=None)
            else:
                vx, vy = vias[best[2]]

                z.update(via=[vx, vy], stich=round(best[0], 3), abstand=round(best[1], 3),
                         anteil=round(Point(vx, vy).buffer(vr, 32).intersection(g).area / min(kreis_a, g.area), 3))
            zeilen.append(z)


    def klein(p): return min(p['bbox'][2] - p['bbox'][0], p['bbox'][3] - p['bbox'][1]) < 1.0
    smd = {}
    for p in D['pads']:
        if not p.get('drill'): smd.setdefault(p['ref'], []).append(p)
    je = {}
    for q in pads:
        p = q['p']; P = smd.get(p['ref'], [])
        if p.get('drill') or len(P) < 8 or klein(p) or len(p['layers']) != 1 or p['layers'][0] not in ('F', 'B'): continue
        xs = [o['x'] for o in P if klein(o)]; ys = [o['y'] for o in P if klein(o)]
        if len(xs) < 6 or not (min(xs) < p['x'] < max(xs) and min(ys) < p['y'] < max(ys)): continue
        je.setdefault((p['ref'], p['num'], p['net']), []).append(q)
    mp = []
    for (ref, num, net), qs in sorted(je.items()):
        g = shapely.union_all([q['poly'] for q in qs])
        vs = [(v['x'], v['y']) for v in R['vias'] if v['net'] == net and g.buffer(0.02).contains(Point(v['x'], v['y']))]
        dmin = min((math.dist(a, b_) for i, a in enumerate(vs) for b_ in vs[i + 1:]), default=None)
        mp.append(dict(ref=ref, num=num, netz=net, pad_mm2=round(g.area, 2), felder=len(qs), vias=len(vs),
                       via_abstand_min=None if dmin is None else round(dmin, 3),
                       freistellungen_getrennt=None if dmin is None else bool(dmin >= via_d + 2 * clr + 0.2 - 1e-6)))
    return zeilen, mp


def zusammenfassen(zeilen):
    out = {}
    for kl in ('entkopplung', 'chip', 'rest', 'alle'):
        zs = [z for z in zeilen if kl == 'alle' or z['klasse'] == kl]
        mit = [z for z in zs if z.get('via')]
        if not zs: continue
        ab = sorted(z['abstand'] for z in mit); st = sorted(z['stich'] for z in mit); n = len(mit)
        def med(v): return None if not v else round(v[len(v) // 2] if len(v) % 2 else (v[len(v) // 2 - 1] + v[len(v) // 2]) / 2, 3)
        out[kl] = dict(pads=len(zs), ohne_via=len(zs) - n,
                       ganz_im_pad=sum(1 for z in mit if z['anteil'] >= 0.99), ab_haelfte=sum(1 for z in mit if z['anteil'] >= 0.5 - 1e-6),
                       ab_drittel=sum(1 for z in mit if z['anteil'] >= 1 / 3 - 1e-6), daneben=sum(1 for z in mit if z['anteil'] < 1 / 3 - 1e-6),
                       abstand_median=med(ab), abstand_mittel=round(sum(ab) / n, 3) if n else None, abstand_max=ab[-1] if n else None,
                       abstand_ueber_05=sum(1 for a in ab if a > 0.5), abstand_ueber_1=sum(1 for a in ab if a > 1.0),
                       stich_mittel=round(sum(st) / n, 3) if n else None, stich_max=st[-1] if n else None, stich_summe=round(sum(st), 2))
    return out


if __name__ == '__main__':
    if len(sys.argv) < 3: sys.exit(__doc__)
    args = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a)
    D = json.load(open(sys.argv[1])); R = json.load(open(sys.argv[2]))
    KL = json.load(open(args['klassen'])) if 'klassen' in args else None
    zeilen, mp = auswerten(D, R, KL, float(args['via']) if 'via' in args else None)
    S = zusammenfassen(zeilen)
    print('Vias der Flaechennetze am Pad (R9: im Pad = mindestens ein Drittel, besser die Haelfte der Via-Flaeche)')
    print('%-12s %5s %5s | %5s %7s %8s %8s | %7s %7s %6s %6s %5s | %7s %6s' % (
        'Klasse', 'Pads', 'ohne', 'ganz', '>=1/2', '>=1/3', 'daneben', 'Median', 'Mittel', 'Max', '>0,5', '>1,0', 'Stich', 'Max'))
    for kl, s in S.items():
        print('%-12s %5d %5d | %5d %7d %8d %8d | %7s %7s %6s %6d %5d | %7s %6s' % (
            kl, s['pads'], s['ohne_via'], s['ganz_im_pad'], s['ab_haelfte'], s['ab_drittel'], s['daneben'], s['abstand_median'],
            s['abstand_mittel'], s['abstand_max'], s['abstand_ueber_05'], s['abstand_ueber_1'], s['stich_mittel'], s['stich_max']))
    print('   (Abstand = Pad-Mitte bis Via-Mitte in mm, Stich = Bahnlaenge vom Pad zum Via in mm; >=1/2 enthaelt ganz, >=1/3 enthaelt >=1/2)')
    if mp:
        print('Massepads unter Chips (R16; Soll mindestens 4, ab etwa 9 mm2 neun):')
        for m in mp:
            print('   %-10s Pad %-4s %-8s %6.2f mm2 in %d Feldern: %d Vias, kleinster Abstand %s mm, Freistellungen getrennt: %s' % (
                m['ref'], m['num'], m['netz'], m['pad_mm2'], m['felder'], m['vias'], m['via_abstand_min'],
                {None: '-', True: 'ja', False: 'nein'}[m['freistellungen_getrennt']]))
    if 'json' in args: json.dump(dict(zusammenfassung=S, massepads=mp, pads=zeilen), open(args['json'], 'w'), indent=1)
