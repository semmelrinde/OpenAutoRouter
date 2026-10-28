""
import json, math, sys, collections
import shapely
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import nearest_points

PAR_GRAD, PAR_D, PAR_L = 15.0, 0.6, 0.8
SPITZ_GRAD = 59.0
RAND_PAD, RAND_STAMM = 0.3, 0.6
NEBEN_D = 0.8
ARM_R = 0.15


def _segmente(R):
    d = collections.defaultdict(list)
    for ti, t in enumerate(R['tracks']):
        for si, (a, b) in enumerate(zip(t['pts'][:-1], t['pts'][1:])):
            if math.dist(a, b) > 1e-6: d[(t['net'], t['layer'])].append((ti, tuple(a), tuple(b), t['w'][si] if si < len(t['w']) else t['w'][-1], bool(t.get('sig'))))
    return d


def _pads(B):
    ""
    P = collections.defaultdict(list)
    if not B: return P
    for p in B['pads']:
        if not p.get('net'): continue
        for L, polys in p['polys'].items():
            g = shapely.union_all([Polygon(q) for q in polys if len(q) >= 3])
            if not g.is_empty: P[(p['net'], L.replace('.Cu', ''))].append(g)
    return P


def _fuss(p, a, b):
    ""
    dx, dy = b[0] - a[0], b[1] - a[1]; ll = dx * dx + dy * dy
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / ll)) if ll > 0 else 0.0
    return (a[0] + t * dx, a[1] + t * dy)


def _abst_punkt_seg(p, a, b):
    return math.dist(p, _fuss(p, a, b))


def _arm(pts, i, schritt, p, r=ARM_R):
    ""
    q = None; k = i
    while 0 <= k < len(pts):
        q = pts[k]
        if math.dist(q, p) >= r: break
        k += schritt
    if q is None or math.dist(q, p) < 0.02: return None
    return math.degrees(math.atan2(q[1] - p[1], q[0] - p[0])) % 360.0



VERSION = 'topologie-v1'
NATIVE_VERSION = 'topologie-v4-symmetrisch'
STAMM_ALT_VERSION = 'topologie-v5-stamm'
STAMM_V6_VERSION = 'topologie-v6-stamm'
STAMM_V7_VERSION = 'topologie-v7-stamm'
STAMM_VERSION = 'topologie-v8-stamm'


def topologie(R, B=None, bsp_max=8, version=VERSION):
    if version in (STAMM_ALT_VERSION, STAMM_V6_VERSION, STAMM_V7_VERSION, STAMM_VERSION):
        from pathlib import Path
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from analyse.topologie_native import topologie as native
        if version == STAMM_ALT_VERSION:
            from analyse.topologie_stamm_v5 import messen
        elif version == STAMM_V6_VERSION:
            from analyse.topologie_stamm_v6 import messen
        elif version == STAMM_V7_VERSION:
            from analyse.topologie_stamm_v7 import messen
        else:
            from analyse.topologie_stamm import messen
        out = native(R, B, bsp_max)
        stamm = messen(R, B, bsp_max)
        out.update({k: v for k, v in stamm.items() if k != 'wert'})

        out['beispiele_knoten'] = out.pop('bsp_knoten')
        out['beispiele_parallel'] = out.pop('bsp_parallel')


        out.pop('abzweig_mitte', None)
        return out
    if version == NATIVE_VERSION:
        from pathlib import Path
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from analyse.topologie_native import topologie as native
        return native(R, B, bsp_max)
    if version != VERSION:
        raise ValueError('unbekannte Topologieversion: '+str(version))
    D = _segmente(R); PADS = _pads(B)
    clr = float(R.get('clr') or 0.13)
    try: via_d = float(str((R.get('optionen') or {}).get('via', '0.45/0.20')).split('/')[0])
    except ValueError: via_d = 0.45
    vias = collections.defaultdict(list)
    for v in R['vias']: vias[v['net']].append((v['x'], v['y']))
    par = dict(abgriff=collections.Counter(), kette=collections.Counter(), rest=collections.Counter()); bsp_par = []
    kn = collections.Counter(); bsp_kn = []
    abg = []
    via_im_pad = 0; via_gezaehlt = set()
    for (net, L), S in sorted(D.items()):

        for i in range(len(S)):
            ti, a, b, wa_, sa = S[i]; la = LineString([a, b]); wa = math.atan2(b[1] - a[1], b[0] - a[0]); puf = None
            for j in range(i + 1, len(S)):
                tj, c, d, wb_, sb = S[j]
                if tj == ti: continue
                if min(c[0], d[0]) - PAR_D > max(a[0], b[0]) or max(c[0], d[0]) + PAR_D < min(a[0], b[0]) or \
                   min(c[1], d[1]) - PAR_D > max(a[1], b[1]) or max(c[1], d[1]) + PAR_D < min(a[1], b[1]): continue
                wb = math.atan2(d[1] - c[1], d[0] - c[0]); dw = abs((wa - wb + math.pi / 2) % math.pi - math.pi / 2)
                if dw > math.radians(PAR_GRAD): continue
                if puf is None: puf = la.buffer(PAR_D, cap_style='flat')
                lb = LineString([c, d]); nah = puf.intersection(lb).length
                if nah > PAR_L and la.distance(lb) > 0.02:
                    art = 'abgriff' if (sa or sb) else ('kette' if (R['tracks'][ti].get('kette') or R['tracks'][tj].get('kette')) else 'rest')
                    par[art][net] += nah
                    bsp_par.append((round(nah, 2), art, net, L, [round(v, 2) for v in a + b], [round(v, 2) for v in c + d], round(la.distance(lb), 3)))

        pk = PADS.get((net, L), [])
        zuege = [[tuple(q) for q in t['pts']] for t in R['tracks'] if t['net'] == net and t['layer'] == L]
        kand = sorted({q for P in zuege for q in P}); fertig = []
        for p in kand:
            if any(math.dist(p, q) < 0.05 for q in fertig): continue
            roh = []; mitte = False; enden = 0
            for P in zuege:
                for k, (a, b) in enumerate(zip(P[:-1], P[1:])):
                    if min(a[0], b[0]) - 0.04 > p[0] or max(a[0], b[0]) + 0.04 < p[0] or min(a[1], b[1]) - 0.04 > p[1] or max(a[1], b[1]) + 0.04 < p[1]: continue
                    f = _fuss(p, a, b)
                    if math.dist(p, f) > 0.03: continue
                    roh += [_arm(P, k + 1, 1, f), _arm(P, k, -1, f)]
                if math.dist(P[0], p) <= 0.03 or math.dist(P[-1], p) <= 0.03: enden += 1
                elif any(_abst_punkt_seg(p, a, b) <= 0.03 for a, b in zip(P[:-1], P[1:])) and all(math.dist(q, p) > 0.03 for q in P): mitte = True
            arme = []
            for x in sorted(x for x in roh if x is not None):
                if not arme or (x - arme[-1]) > 3.0: arme.append(x)
            if len(arme) > 1 and (arme[0] + 360.0 - arme[-1]) <= 3.0: arme.pop()
            if len(arme) < 3: continue
            fertig.append(p)
            lu = [(arme[(k + 1) % len(arme)] - arme[k]) % 360 for k in range(len(arme))]
            im_pad = any(g.buffer(0.03).contains(Point(p)) for g in pk) if pk else False
            kn['knoten'] += 1; kn['knoten_pad'] += im_pad; kn['abzweig_mitte'] += mitte
            if len(arme) > 3: kn['stern'] += 1; kn['stern_pad'] += im_pad
            if min(lu) < SPITZ_GRAD: kn['spitz'] += 1; kn['spitz_pad'] += im_pad
            if len(arme) > 3 or min(lu) < SPITZ_GRAD:
                bsp_kn.append((len(arme), round(min(lu), 1), net, L, (round(p[0], 2), round(p[1], 2)), 'im Pad' if im_pad else 'frei'))

        sig_ti = sorted({s[0] for s in S if s[4]})
        if not sig_ti: continue
        nsig = [s for s in S if not s[4]]
        sig_enden = [tuple(q) for t in R['tracks'] if t['net'] == net and t.get('sig') for q in (t['pts'][0], t['pts'][-1])]
        eig_vias = []; abg_vias = []
        for v in vias[net]:
            (abg_vias if any(math.dist(v, e) < 0.05 for e in sig_enden) else eig_vias).append(v)
        netz_pads = [g for (n2, L2), gs in PADS.items() if n2 == net for g in gs]
        end_pads = [g for g in netz_pads if any(g.buffer(0.03).contains(Point(e)) for e in sig_enden)]
        for v in abg_vias:
            if v not in via_gezaehlt and any(g.contains(Point(v)) for g in netz_pads) and not any(g.contains(Point(v)) for g in end_pads):
                via_gezaehlt.add(v); via_im_pad += 1
        kupfer = [LineString([a, b]).buffer(w / 2) for _, a, b, w, _ in nsig] + [Point(v).buffer(via_d / 2) for v in eig_vias]
        for ti in sig_ti:
            t = R['tracks'][ti]; pts = [tuple(q) for q in t['pts']]; anker = []; art = 'kelvin'; eigene_pads = []
            for e in (pts[0], pts[-1]):
                hit = [g for g in pk if g.buffer(0.03).contains(Point(e))]
                if hit: anker += [g.buffer(RAND_PAD) for g in hit]; eigene_pads += hit; continue
                if any(math.dist(e, v) < 0.05 for v in abg_vias): continue
                if not t.get('kelvin') and any(_abst_punkt_seg(e, a, b) < w / 2 + 0.03 for _, a, b, w, _ in nsig):
                    art = 'abzweig'; anker.append(Point(e).buffer(RAND_STAMM))

            A = shapely.union_all(anker) if anker else None
            O = shapely.union_all(kupfer + [g for g in pk if not any(g is h for h in eigene_pads)]) if (kupfer or pk) else None
            ws = t['w']; teile = [LineString([a, b]).buffer((ws[i] if i < len(ws) else ws[-1]) / 2) for i, (a, b) in enumerate(zip(pts[:-1], pts[1:])) if math.dist(a, b) > 1e-6]
            if not teile: continue
            T = shapely.union_all(teile); lin = LineString(pts)
            if A is not None: T = T.difference(A); lin = lin.difference(A)
            if O is not None and A is not None: O = O.difference(A)
            lg = lin.length if not lin.is_empty else 0.0
            if O is None or O.is_empty or T.is_empty:
                abg.append(dict(net=net, lage=L, art=art, abstand=None, nah_mm=0.0, ort=None, laenge=lg, neben_mm=0.0)); continue
            Ks = shapely.union_all([LineString([a, b]).buffer(w / 2) for _, a, b, w, _ in nsig]) if nsig else None
            if Ks is not None and A is not None: Ks = Ks.difference(A)
            neben = lin.intersection(Ks.buffer(NEBEN_D)).length if (Ks is not None and not Ks.is_empty and not lin.is_empty) else 0.0
            d = T.distance(O); nah = lin.intersection(O.buffer(clr + min(ws) / 2 - 1e-6)).length if not lin.is_empty else 0.0
            ort = None
            if d < clr:
                q = nearest_points(T, O)[0]; ort = (round(q.x, 2), round(q.y, 2))
            abg.append(dict(net=net, lage=L, art=art, abstand=round(d, 3), nah_mm=round(nah, 2), ort=ort, laenge=lg, neben_mm=neben, flag=bool(t.get('kelvin'))))
    kel = [a for a in abg if a['art'] == 'kelvin' and a['abstand'] is not None]
    kfl = [a for a in kel if a.get('flag')]
    alle = par['abgriff'] + par['kette'] + par['rest']
    erg = dict(parallel_abgriff_mm=round(sum(par['abgriff'].values()), 1), parallel_kette_mm=round(sum(par['kette'].values()), 1),
               parallel_rest_mm=round(sum(par['rest'].values()), 1), parallel_netze=dict(sorted((n, round(v, 1)) for n, v in alle.items())),
               knoten=kn['knoten'], knoten_pad=kn['knoten_pad'], abzweig_mitte=kn['abzweig_mitte'],
               stern=kn['stern'], stern_pad=kn['stern_pad'], spitz=kn['spitz'], spitz_pad=kn['spitz_pad'],
               abgriffe=len(abg), abgriffe_kelvin=sum(1 for a in abg if a['art'] == 'kelvin'), abgriffe_abzweig=sum(1 for a in abg if a['art'] == 'abzweig'),
               abgriff_abstand_min=min((a['abstand'] for a in kel), default=None),
               abgriff_nah_mm=round(sum(a['nah_mm'] for a in kel), 1), abgriff_nah=sum(1 for a in kel if a['abstand'] < clr - 1e-6),
               abgriff_laenge_mm=round(sum(a['laenge'] for a in abg if a['art'] == 'kelvin'), 1),
               abgriff_neben_strom_mm=round(sum(a['neben_mm'] for a in abg if a['art'] == 'kelvin'), 1),
               kelvin_gekennzeichnet=len(kfl), kelvin_abstand_min=min((a['abstand'] for a in kfl), default=None),
               kelvin_nah=sum(1 for a in kfl if a['abstand'] < clr - 1e-6), kelvin_nah_mm=round(sum(a['nah_mm'] for a in kfl), 1),
               kelvin_laenge_mm=round(sum(a['laenge'] for a in kfl), 1), kelvin_neben_strom_mm=round(sum(a['neben_mm'] for a in kfl), 1),
               abgriff_via_im_pad=via_im_pad, mit_board=bool(B), clr=clr)
    erg['beispiele_parallel'] = sorted(bsp_par, reverse=True)[:bsp_max]
    erg['beispiele_knoten'] = sorted(bsp_kn, key=lambda k: (-k[0], k[1]))[:bsp_max]
    erg['beispiele_abgriff'] = sorted((a for a in kel if a['abstand'] < clr - 1e-6), key=lambda a: a['abstand'])[:bsp_max]
    return erg


if __name__ == '__main__':
    opt = dict(a.split('=', 1) for a in sys.argv[1:] if '=' in a)
    spitz_grenze = 60.0 if opt.get('version') in (NATIVE_VERSION, STAMM_ALT_VERSION, STAMM_V6_VERSION, STAMM_V7_VERSION, STAMM_VERSION) else SPITZ_GRAD
    B = json.load(open(opt['board'])) if 'board' in opt else None
    for f in [a for a in sys.argv[1:] if '=' not in a]:
        e = topologie(json.load(open(f)), B, version=opt.get('version', VERSION))
        if opt.get('json') == '1': print(json.dumps(dict(datei=f, **e))); continue
        print('%-22s parallel im eigenen Netz: Abgriff %5.1f mm, Kette %4.1f mm, Rest %5.1f mm | Knoten %3d (mitten auf einem Segment %s, im Pad %d), Stern (>3 Arme) %d (im Pad %d), '
              'Winkel < %g Grad %d (im Pad %d)' % (f.split('/')[-1][:22], e['parallel_abgriff_mm'], e['parallel_kette_mm'], e['parallel_rest_mm'], e['knoten'], e.get('abzweig_mitte', 'n/a'),
                                                  e['knoten_pad'], e['stern'], e['stern_pad'], spitz_grenze, e['spitz'], e['spitz_pad']))
        print('    Abgriffe %d (Kelvin %d, Abzweig vom Strompfad %d): kleinster Abstand Kelvin-Abgriff zu eigenem Kupfer %s mm (Regel %g), naeher als die Regel: %d Abgriffe auf %.1f mm; '
              'Abgriff-Vias im Zwischen-Pad %d; Kelvin-Abgriffe %.1f mm lang, davon %.1f mm auf der Lage des eigenen Strompfads in weniger als %g mm Abstand%s' % (
                  e['abgriffe'], e['abgriffe_kelvin'], e['abgriffe_abzweig'], e['abgriff_abstand_min'], e['clr'], e['abgriff_nah'],
                  e['abgriff_nah_mm'], e['abgriff_via_im_pad'], e['abgriff_laenge_mm'], e['abgriff_neben_strom_mm'], NEBEN_D, '' if B else '  (ohne board=: Pads nicht beruecksichtigt)'))
        if e['kelvin_gekennzeichnet']:
            print('    davon vom Router als Kelvin gekennzeichnet %d: kleinster Abstand %s mm, naeher als die Regel %d auf %.1f mm, %.1f mm lang, davon %.1f mm neben dem Strompfad' % (
                e['kelvin_gekennzeichnet'], e['kelvin_abstand_min'], e['kelvin_nah'], e['kelvin_nah_mm'], e['kelvin_laenge_mm'], e['kelvin_neben_strom_mm']))
        print('   ', [(n[-14:], v) for n, v in sorted(e['parallel_netze'].items(), key=lambda kv: -kv[1])[:8]])
        if 'parallel_r27_mm' in e:
            print('    R27 inklusive Ketten/normaler sig-Aeste, nur Kelvin ausgenommen: %.1f mm parallel' % e['parallel_r27_mm'])
        if opt.get('details') == '1':
            for b in e['beispiele_parallel']: print('    parallel', b)
            for b in e['beispiele_knoten']: print('    Knoten (Arme, kleinster Winkel, Netz, Lage, Ort)', b)
            for b in e['beispiele_abgriff']: print('    Abgriff nah', b)
