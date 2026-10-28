""
import glob, json, math, os, shutil, statistics, sys, time
from concurrent.futures import ProcessPoolExecutor

R_ = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); sys.path.insert(0, R_)
ORDNER = os.path.expanduser(os.environ.get('AUTOROUTER_ZONEN_WERKBANK', '~/Library/Caches/Autorouter/zonen_werkbank'))
SELBST = os.path.expanduser('~/Library/Caches/Autorouter/selbstspiel')
MISCHUNG = {2: 3, 4: 8, 6: 7, 8: 6}


def faelle():
    return sorted(d for d in glob.glob(os.path.join(ORDNER, 'faelle', '*')) if os.path.exists(os.path.join(d, 'routes.json')))


def sammeln(n=24):
    ""
    from lernen.stromzonen import stromnetze
    kand = sorted(glob.glob(os.path.join(SELBST, 'aufgabe_*', 'k*', 'routes_stromzonen', 'eingabe.json')), key=os.path.getmtime, reverse=True)
    je = {k: 0 for k in MISCHUNG}; ziel = {k: round(v * n / 24) for k, v in MISCHUNG.items()}; neu = 0; je_aufgabe = {}
    os.makedirs(os.path.join(ORDNER, 'faelle'), exist_ok=True)
    for e in kand:
        k = os.path.dirname(os.path.dirname(e)); a = os.path.dirname(k)
        try: B = json.load(open(os.path.join(k, 'board.json'))); K = json.load(open(os.path.join(k, 'klassen.json')))
        except (OSError, ValueError): continue
        L = int(B.get('lagen') or 2)
        if L not in je or je[L] >= ziel[L] or je_aufgabe.get(a, 0) >= 2 or not stromnetze(K, set(K.get('gnd') or ['GND'])): continue
        je_aufgabe[a] = je_aufgabe.get(a, 0) + 1
        name = '%s_%s' % (os.path.basename(a).replace('aufgabe_', 'a'), os.path.basename(k))
        d = os.path.join(ORDNER, 'faelle', name)
        if os.path.exists(d): je[L] += 1; continue
        os.makedirs(d)
        for f in ('board.json', 'klassen.json'): shutil.copyfile(os.path.join(k, f), os.path.join(d, f))
        shutil.copyfile(e, os.path.join(d, 'routes.json'))
        if os.path.exists(os.path.join(a, 'regeln.json')): shutil.copyfile(os.path.join(a, 'regeln.json'), os.path.join(d, 'regeln.json'))
        json.dump(dict(quelle=k, lagen=L, zeit=time.strftime('%Y-%m-%d %H:%M')), open(os.path.join(d, 'meta.json'), 'w'))
        je[L] += 1; neu += 1
        if all(je[x] >= ziel[x] for x in je): break
    print('Fälle: %d neu, je Lagenzahl %s -> %s' % (neu, je, os.path.join(ORDNER, 'faelle')))


def _winkel(poly):
    ""
    spitz = kerbe = 0
    for ring, aussen in [(poly.exterior, True)] + [(i, False) for i in poly.interiors]:
        pts = list(ring.simplify(0.02).coords)[:-1]
        if len(pts) < 3: continue
        flaeche = sum(pts[i][0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * pts[i][1] for i in range(len(pts)))
        ccw = flaeche > 0
        for i in range(len(pts)):
            a, b, c = pts[i - 1], pts[i], pts[(i + 1) % len(pts)]
            v1, v2 = (a[0] - b[0], a[1] - b[1]), (c[0] - b[0], c[1] - b[1])
            l1, l2 = math.hypot(*v1), math.hypot(*v2)
            if l1 < 0.05 or l2 < 0.05: continue
            w = math.degrees(math.acos(max(-1, min(1, (v1[0] * v2[0] + v1[1] * v2[1]) / (l1 * l2)))))
            kreuz = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
            konvex = (kreuz > 0) == ccw if aussen else (kreuz > 0) != ccw
            if w < 80: spitz += konvex; kerbe += not konvex
    return spitz, kerbe


def keile(B, R, K, clr=0.15, w_spitz=45.0, w_eng=80.0, n_orte=10):
    ""
    from shapely.geometry import Polygon, Point
    from shapely.ops import unary_union
    from lernen import stromzonen as SZ
    umriss = unary_union([Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3])
    zs = [z for z in R.get('zonen', []) if z.get('quelle') == 'stromzonen']
    n_spitz = n_eng = 0; orte = []
    for L in sorted({z['layer'] for z in zs}):
        zl = unary_union([SZ._geo(z) for z in zs if z['layer'] == L])
        teile = SZ._teile(B, R, L)
        je = {}
        for m, g in teile: je.setdefault(m, []).append(g)
        kupfer = unary_union([unary_union(v).buffer(clr / 2 + .01, 8) for _, v in sorted(je.items())])
        weiss = umriss.difference(kupfer)

        router = unary_union([g for _, g in SZ._teile(B, dict(R, zonen=[z for z in R.get('zonen', []) if z.get('quelle') != 'stromzonen']), L)])
        nah = zl.difference(router.buffer(.01, 8)).buffer(clr / 2 + .04, 8)
        for poly in getattr(weiss, 'geoms', [weiss]):
            if poly.geom_type != 'Polygon' or poly.area < 1e-3: continue
            for ring, aussen in [(poly.exterior, True)] + [(i, False) for i in poly.interiors]:
                pts = list(ring.simplify(0.01).coords)[:-1]
                if len(pts) < 3: continue
                fl = sum(pts[i][0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * pts[i][1] for i in range(len(pts)))
                for i in range(len(pts)):
                    a, b, c = pts[i - 1], pts[i], pts[(i + 1) % len(pts)]
                    v1, v2 = (a[0] - b[0], a[1] - b[1]), (c[0] - b[0], c[1] - b[1])
                    l1, l2 = math.hypot(*v1), math.hypot(*v2)


                    if l1 < 0.1 or l2 < 0.1: continue
                    w = math.degrees(math.acos(max(-1, min(1, (v1[0] * v2[0] + v1[1] * v2[1]) / (l1 * l2)))))
                    kreuz = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
                    konvex = (kreuz > 0) == (fl > 0) if aussen else (kreuz > 0) != (fl > 0)
                    if not konvex or w >= w_eng or not nah.contains(Point(b)): continue
                    n_eng += 1; n_spitz += w < w_spitz; orte.append((round(w, 1), L, round(b[0], 2), round(b[1], 2)))
    return n_spitz, n_eng, sorted(orte)[:n_orte]


def messen(B, R, K, gndlagen, regeln=None, paar_modell=None, *, strom_kette=None, engstellen=None, nur_form=False):
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    from lernen import stromzonen as SZ
    umriss = unary_union([Polygon(o).buffer(0) for o in B['outline'] if len(o) >= 3])
    zs = [z for z in R.get('zonen', []) if z.get('quelle') == 'stromzonen']
    pads = [(p['net'], L, Polygon(q).buffer(0)) for p in B['pads'] for L, qs in (p.get('polys') or {}).items() for q in qs if len(q) >= 3]
    stuecke, je_nl, spitz, kerbe, aussen_flaeche, kanten_45, kanten = [], {}, 0, 0, 0.0, 0.0, 0.0
    for z in zs:
        g = SZ._geo(z)
        for p in getattr(g, 'geoms', [g]):
            if p.geom_type != 'Polygon' or p.area < 1e-4: continue
            npads = sum(1 for n, L, q in pads if n == z['net'] and L == z['layer'] and p.intersects(q))
            stuecke.append(dict(netz=z['net'], lage=z['layer'], flaeche=p.area, pads=npads))
            je_nl[(z['net'], z['layer'])] = je_nl.get((z['net'], z['layer']), 0) + 1
            s, k = _winkel(p); spitz += s; kerbe += k
            if z['layer'] in ('F', 'B'): aussen_flaeche += p.area
            for a, b in zip(list(p.exterior.coords)[:-1], list(p.exterior.coords)[1:]):
                l = math.dist(a, b); kanten += l
                w = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 45
                if min(w, 45 - w) < 1.0: kanten_45 += l
    fl = sorted(s['flaeche'] for s in stuecke)
    q = lambda xs, p: xs[min(len(xs) - 1, int(p * len(xs)))] if xs else None
    try: eng = engstellen if engstellen is not None else __import__('analyse.strom_engstellen', fromlist=['engstellen']).engstellen(B, R, K, gndlagen)
    except Exception as e: eng = dict(unnoetig_n=None, unnoetig_mm2=None, fehler=repr(e)[:120])
    try: k_spitz, k_eng, k_orte = keile(B, R, K)
    except Exception as e: k_spitz = k_eng = None; k_orte = repr(e)[:120]
    from lernen.zonen_geometrie import messen as geometrie_messen
    from lernen.zonen_harte_pruefung import pruefen as hart_pruefen
    t_pruef = time.process_time()
    try:
        geometrie = geometrie_messen(B,R,K)
    except Exception as e:
        geometrie = dict(fehler=repr(e), zaehler={})
    harte = None
    harte_details = dict(status='nicht_gemessen')
    if not nur_form:
        try:
            harte_details = hart_pruefen(B,R,K,gndlagen,regeln=regeln)
            harte = len(harte_details['fehler'])
        except Exception as e:
            harte_details = dict(bestanden=False, fehler_pruefung=repr(e))
    pruefer_cpu = time.process_time()-t_pruef
    try:
        from analyse.strom_kette import kette
        KE = strom_kette if strom_kette is not None else kette(B, R, K)
    except Exception as e: KE = dict(fehler=repr(e)[:120])
    from lernen.form_merkmale import kette_ok
    result = dict(geometrie=geometrie, harte_pruefung=harte_details, pruefer_cpu_s=pruefer_cpu,
                nebenstreifen=geometrie['zaehler'].get('nebenstreifen'),
                eigennetz_schlitze=geometrie['zaehler'].get('eigennetz_schlitz'),
                via_bisse=geometrie['zaehler'].get('via_biss'), stuecke=len(stuecke), z2_aussen_pct=round(100 * aussen_flaeche / max(1e-6, 2 * umriss.area), 2),
                z2_median_mm2=round(q(fl, 0.5), 2) if fl else None, z2_p90_mm2=round(q(fl, 0.9), 2) if fl else None,
                z2_pads_median=statistics.median([s['pads'] for s in stuecke]) if stuecke else None,
                z2_ohne_pad_pct=round(100 * sum(1 for s in stuecke if s['pads'] == 0) / max(1, len(stuecke)), 1),
                z3_kanten45_pct=round(100 * kanten_45 / max(1e-6, kanten), 1),
                z4_je_netz_lage=round(statistics.mean(je_nl.values()), 2) if je_nl else None,
                z4_einteilig_pct=round(100 * sum(1 for v in je_nl.values() if v == 1) / max(1, len(je_nl)), 1),
                z4_splitter=sum(1 for s in stuecke if s['flaeche'] < 0.5),
                spitze_ecken=spitz, kerben=kerbe, keile_45=k_spitz, keile_80=k_eng, keile_orte=k_orte, r39_unnoetig_n=eng.get('unnoetig_n'), r39_unnoetig_mm2=eng.get('unnoetig_mm2'),
                harte_verstoesse=harte, strom_kette=KE,
                kette_abdeckung=dict(vollstaendig=kette_ok(KE),
                                     ausgelassen=KE.get('ausgelassen'), netze=len(KE.get('netze') or {}),
                                     **({'fehler': KE['fehler']} if 'fehler' in KE else {})),
                kette_eta=KE.get('wirkungsgrad'), kette_eta_median=KE.get('wirkungsgrad_median'),
                kette_glied_max=KE.get('glied_max'), kupfer_strom_mm2=KE.get('kupfer_mm2'), kette_netze=KE.get('netze'))
    if nur_form: return result
    from lernen.zonen_richter import diagnose
    zr = diagnose(B, R)
    result.update(zonen_richter_note=zr['zonen_richter_note'], zonen_richter=zr)
    from lernen.richter_paar import Dienst, komponenten, komponentenbeleg, messsignatur
    judge = Dienst(modell=paar_modell, ziel='strom_note') if paar_modell else Dienst(ziel='strom_note')

    if (judge.modell/'strom_note_meister.json').is_file():
        from analyse.bewertung import bewerte
        try:
            E = bewerte(B, R, K, v3=True, minimax=False)
            z = dict(E, gueltig=bool(E['gueltig']) and harte == 0, regel=E.get('regeln') or {},
                     strom_kette=KE, v3_komponenten=komponenten(E['metriken']), v3_komponenten_beleg=komponentenbeleg(E['metriken']), paar_messversion=messsignatur())
            judge.anwenden(z)
            result['strom_paar'] = z
        except Exception as e: result['strom_paar'] = dict(paar_status='fehler', fehler=repr(e)[:160])
    else: result['strom_paar'] = dict(paar_status='keine_noten', paar_freigegeben=False)
    return result


def _regeln_fall(d):
    p = os.path.join(d,'regeln.json')
    return json.load(open(p)) if os.path.exists(p) else None


def _ein_fall(arg):
    d, variante, params, gnd = arg
    try: os.nice(10)
    except OSError: pass
    from lernen import stromzonen as SZ
    t0 = time.process_time()
    B, K, R = (json.load(open(os.path.join(d, f))) for f in ('board.json', 'klassen.json', 'routes.json'))
    gl = R.get('gndlagen')
    R['zonen'] = [z for z in R.get('zonen', []) if z.get('quelle') not in ('stromzonen', 'gnd_fuellung')]
    R['vias'] = [v for v in R.get('vias', []) if v.get('quelle') != 'gnd_fuellung']
    try:
        params = dict(params); modul = params.pop('modul', None); paar_modell = params.pop('paar_modell', None)
        diagnose=params.pop('zonen_richter_diagnose', '0')
        R['optionen']=dict(R.get('optionen') or {}, zonen_richter_diagnose=diagnose)
        richter_modell=params.pop('zonen_richter_modell', None)
        if richter_modell:R['optionen']=dict(R.get('optionen') or {}, zonen_richter_modell=richter_modell)
        if modul:
            zs = __import__('lernen.stromzonen_' + modul, fromlist=['zonen']).zonen(B, R, K, gl, **params)
        else: zs = SZ.zonen(B, R, K, gl, **params)
        from lernen.stromzonen_topo import skelett_anwenden
        skelett_anwenden(R, zs)
        R['zonen'] += zs
        if gnd: gz, gv = SZ.gnd_fuellung(B, R, K, gl); R['zonen'] += gz; R['vias'] += gv
        cpu = time.process_time() - t0
        m = messen(B, R, K, gl, regeln=_regeln_fall(d), paar_modell=paar_modell); m.update(cpu_s=round(cpu, 1), lagen=B.get("lagen"))
    except Exception as e:
        m = dict(fehler=repr(e)[:300])
    json.dump(R, open(os.path.join(d, 'routes_%s.json' % variante), 'w'))
    json.dump(m, open(os.path.join(d, 'mass_%s.json' % variante), 'w'), indent=1)
    return os.path.basename(d), m


def _nachmessen(arg):
    d, variante = arg
    try: os.nice(10)
    except OSError: pass
    p = os.path.join(d, 'routes_%s.json' % variante)
    if not os.path.exists(p): return os.path.basename(d), dict(fehler='keine Route')
    B, K, R = json.load(open(os.path.join(d, 'board.json'))), json.load(open(os.path.join(d, 'klassen.json'))), json.load(open(p))
    alt = json.load(open(os.path.join(d, 'mass_%s.json' % variante))) if os.path.exists(os.path.join(d, 'mass_%s.json' % variante)) else {}
    t0 = time.process_time()
    try:
        m = messen(B, R, K, R.get('gndlagen'), regeln=_regeln_fall(d))
        m.update(cpu_s=alt.get('cpu_s'), messen_cpu_s=time.process_time()-t0, lagen=B.get('lagen'))
    except Exception as e: m = dict(fehler=repr(e)[:300])
    json.dump(m, open(os.path.join(d, 'mass_%s.json' % variante), 'w'), indent=1)
    return os.path.basename(d), m


def nachmessen(variante, par=3):
    erg = {}
    with ProcessPoolExecutor(par) as ex:
        for name, m in ex.map(_nachmessen, [(d, variante) for d in faelle()]): erg[name] = m
    p = os.path.join(ORDNER, 'variante_%s.json' % variante)
    V = json.load(open(p)) if os.path.exists(p) else dict(variante=variante)
    V.update(faelle=erg, nachgemessen=time.strftime('%Y-%m-%d %H:%M')); json.dump(V, open(p, 'w'), indent=1)
    zusammenfassen(variante, erg)


def _wert(v):
    if v in ('0', '1') : return v == '1'
    try: return float(v) if '.' in v else int(v)
    except ValueError: return v


def rechnen(variante, par=3, gnd=False, **params):
    params = {k: _wert(v) for k, v in params.items()}
    F = faelle(); erg = {}
    with ProcessPoolExecutor(par) as ex:
        for name, m in ex.map(_ein_fall, [(d, variante, params, gnd) for d in F]): erg[name] = m
    json.dump(dict(variante=variante, params=params, gnd=gnd, zeit=time.strftime('%Y-%m-%d %H:%M'), faelle=erg),
              open(os.path.join(ORDNER, 'variante_%s.json' % variante), 'w'), indent=1)
    zusammenfassen(variante, erg)


SCHLUESSEL = ('kette_eta', 'kette_eta_median', 'kette_glied_max', 'kupfer_strom_mm2', 'stuecke', 'z2_aussen_pct', 'z2_median_mm2', 'z2_p90_mm2', 'z2_ohne_pad_pct', 'z3_kanten45_pct', 'z4_je_netz_lage',
              'z4_einteilig_pct', 'z4_splitter', 'spitze_ecken', 'kerben', 'keile_45', 'keile_80', 'r39_unnoetig_n', 'harte_verstoesse', 'nebenstreifen', 'eigennetz_schlitze', 'via_bisse', 'pruefer_cpu_s', 'cpu_s', 'zonen_richter_note')


def zusammenfassen(variante, erg):
    counts = dict(faelle=len(erg), fallfehler=0, geometrie_fehler=0, geometrie_unbekannt=0,
                  harte_pruefungsfehler=0, ungueltig=0, status_unbekannt=0, bestanden=0,
                  vollstaendig_bestanden=0)
    for m in erg.values():
        geo = m.get('geometrie') or {}
        hard = m.get('harte_pruefung') or {}
        error = 'fehler_pruefung' in hard
        valid = hard.get('bestanden')
        counts['fallfehler'] += 'fehler' in m
        counts['geometrie_fehler'] += 'fehler' in geo
        counts['geometrie_unbekannt'] += not geo
        counts['harte_pruefungsfehler'] += error
        counts['ungueltig'] += valid is False and not error
        counts['status_unbekannt'] += valid is not True and valid is not False and not error
        counts['bestanden'] += valid is True and not error
        counts['vollstaendig_bestanden'] += (valid is True and not error and bool(geo)
                                            and 'fehler' not in geo and 'fehler' not in m)
    print('Variante %s: %d Fälle, %d vollständig bestanden' %
          (variante, len(erg), counts['vollstaendig_bestanden']))
    print('  Status (Prüfarten können sich überschneiden): ' +
          ', '.join('%s=%d'%(k,v) for k,v in counts.items() if k not in ('faelle','vollstaendig_bestanden')))
    metrics = {}
    for k in SCHLUESSEL:
        xs = [m[k] for m in erg.values() if 'fehler' not in m and type(m.get(k)) in (int,float)
              and math.isfinite(m[k])]
        metrics[k] = dict(n=len(xs), nenner=len(erg), fehlend=len(erg)-len(xs),
                          median=statistics.median(xs) if xs else None, summe=sum(xs) if xs else None)
        values = 'Median %8.2f  Summe %9.1f'%(statistics.median(xs),sum(xs)) if xs else 'Median --  Summe --'
        print('  %-20s %s  n=%d/%d  fehlend=%d'%(k,values,len(xs),len(erg),len(erg)-len(xs)))
    return dict(status=counts, kennzahlen=metrics)


def bild(d, varianten, ziel, breite=700):
    ""
    from PIL import Image, ImageDraw
    B = json.load(open(os.path.join(d, 'board.json')))
    xs = [p[0] for o in B['outline'] for p in o]; ys = [p[1] for o in B['outline'] for p in o]
    x0, y0, x1, y1 = min(xs) - .5, min(ys) - .5, max(xs) + .5, max(ys) + .5
    S = breite / (x1 - x0); W, H = int((x1 - x0) * S), int((y1 - y0) * S)
    tr = lambda p: ((p[0] - x0) * S, (p[1] - y0) * S)
    PAL = [(230, 90, 40), (40, 150, 220), (150, 80, 200), (60, 170, 90), (220, 170, 30), (200, 60, 140), (90, 120, 120), (170, 110, 60)]
    zeilen = []
    for v in varianten:
        p = os.path.join(d, 'routes_%s.json' % v)
        if not os.path.exists(p): continue
        R = json.load(open(p)); netze = sorted({z['net'] for z in R.get('zonen', []) if z.get('quelle') == 'stromzonen'})
        farbe = {n: PAL[i % len(PAL)] for i, n in enumerate(netze)}
        bilder = []
        for L in ('F', 'B'):
            im = Image.new('RGB', (W, H + 18), (255, 255, 255)); dr = ImageDraw.Draw(im, 'RGBA')
            off = lambda p: (tr(p)[0], tr(p)[1] + 18)
            for o in B['outline']: dr.polygon([off(p) for p in o], fill=(246, 246, 241), outline=(0, 0, 0))
            for z in R.get('zonen', []):
                if z['layer'] != L: continue
                c = (205, 228, 205) if z.get('quelle') == 'gnd_fuellung' else farbe.get(z['net'], (200, 200, 200))
                for q in z['polys']:
                    m = Image.new('L', im.size, 0); dm = ImageDraw.Draw(m); dm.polygon([off(p) for p in q['aussen']], fill=255)
                    for h in q.get('innen') or []: dm.polygon([off(p) for p in h], fill=0)
                    im.paste(c, (0, 0), m)
            dr = ImageDraw.Draw(im, 'RGBA')
            for pd in B['pads']:
                for q in (pd.get('polys') or {}).get(L, []): dr.polygon([off(x) for x in q], fill=(120, 120, 120, 200))
            for t in R.get('tracks', []):
                if t['layer'] != L: continue
                c = farbe.get(t['net'], (60, 60, 60)); c = tuple(int(x * .6) for x in c) + (255,)
                for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']): dr.line([off(a), off(b)], fill=c, width=max(1, int(w * S)))
            for via in R.get('vias', []):
                x, y = off((via['x'], via['y'])); r = max(1.5, .225 * S); dr.ellipse([x - r, y - r, x + r, y + r], fill=(20, 20, 20))
            m = json.load(open(os.path.join(d, 'mass_%s.json' % v))) if os.path.exists(os.path.join(d, 'mass_%s.json' % v)) else {}
            for n, kv in (m.get('kette_netze') or {}).items():
                o = kv.get('glied_ort')
                if not o: continue
                x, y = off((o[0], o[1])); r = 9
                dr.ellipse([x - r, y - r, x + r, y + r], outline=(255, 0, 0), width=3)
                dr.text((x + r + 2, y - 6), '%s %.0f%% x%s %s' % (n, 100 * kv['wirkungsgrad'], kv['glied'], o[2] if o[2] != L else ''), fill=(200, 0, 0))
            dr.text((4, 3), '%s  %s  | Kette eta %s, Glied x%s, Kupfer %s mm2 | Stücke %s, aussen %s %%, Kerben %s, R39 %s' % (v, L,
                    m.get('kette_eta'), m.get('kette_glied_max'), m.get('kupfer_strom_mm2'), m.get('stuecke'), m.get('z2_aussen_pct'),
                    m.get('kerben'), m.get('r39_unnoetig_n')), fill=(0, 0, 0))
            bilder.append(im)
        zeile = Image.new('RGB', (2 * W + 8, H + 18), (255, 255, 255)); zeile.paste(bilder[0], (0, 0)); zeile.paste(bilder[1], (W + 8, 0))
        zeilen.append(zeile)
    if not zeilen: return None
    out = Image.new('RGB', (zeilen[0].width, sum(z.height + 6 for z in zeilen)), (255, 255, 255)); y = 0
    for z in zeilen: out.paste(z, (0, y)); y += z.height + 6
    out.save(ziel); return ziel


def vergleich(a, b, mit_bild=True, nur=None, paar_modell=None):
    for v in (a, b):
        p = os.path.join(ORDNER, 'variante_%s.json' % v)
        if os.path.exists(p): zusammenfassen(v, json.load(open(p))['faelle'])
    from lernen.richter_paar import Dienst
    judge = Dienst(modell=paar_modell,ziel='strom_note') if paar_modell else Dienst(ziel='strom_note')
    for d in faelle():
        if nur and os.path.basename(d) not in nur: continue
        try:
            rows = [json.load(open(os.path.join(d, 'mass_%s.json' % v))).get('strom_paar') or {} for v in (a,b)]
            decision = judge.vergleich(*rows)
            print(os.path.basename(d), 'Strom-Paar:', decision)
        except (OSError, ValueError): pass
    if not mit_bild: return
    os.makedirs(os.path.join(ORDNER, 'bilder'), exist_ok=True)
    for d in faelle():
        if nur and os.path.basename(d) not in nur: continue
        z = bild(d, [a, b], os.path.join(ORDNER, 'bilder', '%s_%s_%s.png' % (os.path.basename(d), a, b)))
        if z: print(z)


def main():
    a = sys.argv[1:]; kw = dict(x.split('=', 1) for x in a if '=' in x); pos = [x for x in a if '=' not in x]
    if not pos: print(__doc__); return
    if pos[0] == 'sammeln': sammeln(int(kw.get('n', '24')))
    elif pos[0] == 'rechnen':
        par = int(kw.pop('parallel', '3')); gnd = kw.pop('gnd', '0') == '1'
        rechnen(pos[1], par, gnd, **kw)
    elif pos[0] == 'messen': nachmessen(pos[1], int(kw.get('parallel', '3')))
    elif pos[0] == 'vergleich':
        vergleich(pos[1], pos[2], kw.get('bild', '1') == '1', set(kw['faelle'].split(',')) if kw.get('faelle') else None, kw.get('paar_modell'))


if __name__ == '__main__':
    main()
