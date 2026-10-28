""
import hashlib, json, os, sys, time, random, glob, threading, subprocess
from concurrent.futures import ThreadPoolExecutor
R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, R)
from analyse import korpus as KO
from lernen.merkmale import merkmale



RAUM = dict(kreuz=[1, 2, 4], trenn=[3.0], via_kosten=[0.6, 1.2], fein=['1'], via_f=[1.0, 1.5])
LOCK = threading.Lock()
V2 = dict(an=False, code='')
V3 = dict(an=True)
EBENEN = dict(an=False, lagen68=False, nativ=False)
DICHTE = False
PLACEMENT_SCHUTZ = False
R11_IR = False
R36_ZONEN = False
R36_GLATT = False
STROM_MAX = 20.0


def code_stand():
    ""
    h = hashlib.sha256()
    for folder in ('lernen', 'router', 'platzierung', 'analyse', 'gemeinsam', 'kicad', 'nachbearbeitung'):
        for p in sorted(glob.glob(os.path.join(R, folder, '*.py')) + glob.glob(os.path.join(R, folder, '*.json')) +
                        (glob.glob(os.path.join(R, 'lernen/ki', '*.py')) + glob.glob(os.path.join(R, 'lernen/ki', '*.json')) if folder == 'lernen' else []) +
                        (glob.glob(os.path.join(R, 'analyse/k20', '*.py')) if folder == 'analyse' else [])):
            h.update(os.path.relpath(p, R).encode())
            with open(p, 'rb') as f: h.update(f.read())
    return h.hexdigest()

def k19_ersatz(B, K, o, L):
    ""
    import math
    gn = K.get('gnd_haupt') or (K.get('gnd') or [None])[0]
    mp = os.path.join(o, 'mensch.json')
    if L < 4 or not gn or not os.path.exists(mp): return [], []
    R = json.load(open(mp)); lang = {}
    for t in R.get('tracks', []):
        lang[t['layer']] = lang.get(t['layer'], 0.0) + sum(math.dist(a, b) for a, b in zip(t['pts'][:-1], t['pts'][1:]))
    ges = sum(lang.values()) or 1.0
    innen = ['In%d' % i for i in range(1, L - 1)]
    if sum(lang.get(x, 0.0) for x in innen) / ges >= 0.05: return [], []
    lagen = ['In1'] if L < 6 else ['In1', 'In%d' % (L - 2)]
    zonen = [dict(net=gn, layer=x, pts=[list(p) for p in q], holes=[], prio=0, full=False, k19_ersatz=True)
             for x in lagen for q in B['outline'] if len(q) >= 3]
    with open(os.path.join(o, 'ebenen_export.log'), 'a') as f:
        f.write('K19-Ersatz: keine Originalflaechen, Innenlagen beim Menschen %.1f %% der Bahnlaenge -> %s = %s\n' % (
            100 * sum(lang.get(x, 0.0) for x in innen) / ges, ','.join(lagen), gn))
    return lagen, zonen


def stromnachlauf(bj, kj, rj, rg, gndlagen, zonen=False, glatt=False, ir=False):
    ""
    if not (zonen or glatt or ir) or not os.path.exists(rj): return {}
    if glatt:
        with open(rj) as f: route = json.load(f)
        route['r36_glatt'] = True
        with open(rj, 'w') as f: json.dump(route, f)
    out = {}
    if ir:






        from lernen.stromzonen import anwenden
        out['stromzonen'] = anwenden(bj, kj, rj, gndlagen, regeln=rg)
    elif zonen or glatt:
        from lernen.zonenplaner import abschluss
        out['stromzonen'] = abschluss(bj, kj, rj, rg, gndlagen, modus='legacy')
    if ir:
        from lernen.r11_nachlauf import anwenden
        out['r11_ir'] = anwenden(bj, kj, rj, rg)
    return out


def regelmessung(B, routes, K, gndlagen, dicht=False, fluss_cache=None, kette=False, *, v3_bewertung=None):
    ""
    from analyse.bewertung import bewerte
    from analyse.topologie import topologie
    from analyse.kopplung import kopplung

    KK, begrenzt = strom_begrenzt(K)
    RR = dict(routes, gndlagen=list(gndlagen))
    from analyse.standard import ziel
    minimax = ziel(K.get('vorgaben'), explizit=V3.get('ziel')) == 'minimax'
    if v3_bewertung is not None and not minimax and not begrenzt:
        import copy
        E = copy.deepcopy(v3_bewertung)
    else:
        E = bewerte(B, RR, K if minimax else KK, v3=True, minimax=minimax, fluss_cache=fluss_cache)
    diagnose = bewerte(B, RR, KK, v3=True, minimax=False, fluss_cache=fluss_cache) if minimax and begrenzt else E
    if dicht and 'k20' not in diagnose['regeln']:


        from analyse.k20.kennzahl import diagnose as k20_diagnose
        k20_diagnose(diagnose['regeln'], B, RR, K)
    T = topologie(routes, B)
    from analyse.regelkennzahlen import torfelder
    from analyse.strom_engstellen import lernwerte
    from analyse.signalintegritaet import kurz
    rt = {k: E['regeln'].get(k) for k in torfelder(E['regeln'])}
    if E['note_v3_version'] == 'v3.2': rt['note_v3_version'] = 'v3.2'
    zusatz = dict(regeltor=rt, **{k: E[k] for k in ('minimax', 'minimax_version', 'minimax_profil', 'minimax_rang', 'minimax_summe', 'minimax_strompfade',
                                    'minimax_vollstaendig', 'minimax_boardvertrag', 'minimax_quellen', 'minimax_gueltig', 'minimax_status', 'engpass', 'sicherheit', 'bewertungsziel')}) if minimax else {}


    _N = ((diagnose.get('metriken') or {}).get('netze') or {})
    strom_x = max([max(v['x'].get('dc', 0.0), v['x'].get('j', 0.0)) for v in _N.values() if v.get('klasse') in ('strom', 'versorgung', 'sw')] or [0.0])
    from lernen.richter_paar import komponenten, komponentenbeleg, messsignatur
    from lernen.paar_vertrag import boardkennung
    chain = (E.get('metriken') or {}).get('strom_kette')
    if kette and chain is None and E['gueltig']:
        from analyse.strom_kette import kette as messen_kette
        try: chain = messen_kette(B, RR, K)
        except Exception as e: chain = dict(status='unbekannt', fehler=repr(e)[:160])
    return dict(board_kennung=boardkennung(B), note_v3_version=diagnose['note_v3_version'], paar_messversion=messsignatur(), v3_komponenten=komponenten(diagnose['metriken']), v3_komponenten_beleg=komponentenbeleg(diagnose['metriken']), strom_kette=chain or dict(status='unbekannt'), **(dict(tor=E['tor'], messversion=E['tor'].get('messversion')) if 'tor' in E else {}), **({'_physik': dict(diagnose['metriken'], unverbunden=E['unverbunden'], flaechennetze=routes.get('flaechennetze', []), strom_begrenzt=begrenzt, note_v3_voll=diagnose['note_v3'])} if dicht else {}), si=kurz(diagnose['metriken']), regeln=diagnose['regeln'], **zusatz, strom_x_max=round(strom_x, 2),
                gueltig=E['gueltig'], note_v3=diagnose['note_v3_netze'], version=diagnose['note_v3_version'],
                topologie={k: T[k] for k in ('parallel_rest_mm', 'abzweig_mitte', 'spitz', 'abgriff_nah_mm')},
                r27=E['regeln'].get('r27'), kopplung=kopplung(routes, KK)['summe'],
                strom_engstellen=lernwerte(diagnose['regeln'].get('strom_engstellen')),
                **({'k20': diagnose['regeln']['k20']} if 'k20' in diagnose['regeln'] else {}),
                r39_abdeckung=diagnose['regeln'].get('r39_abdeckung'),
                strom_engstellen_mm2=diagnose['regeln'].get('strom_engstellen_mm2'))

def strom_begrenzt(K, i_max=STROM_MAX):
    ""
    import copy
    from analyse.metriken import _strom
    N = K.get('netze') or {}
    zu = sorted(n for n in N if _strom(n, K)[0] > i_max and not N[n].get('strom_verlangt') and (N[n].get('fluss') or {}).get('einheit_quelle') != 'vorgabe' and not any('vorgabe' in str(h) for h in (N[n].get('fluss') or {}).get('herkunft', {}).values()))
    if not zu: return K, []
    KK = dict(K, netze=dict(N))
    for n in zu:
        k = copy.deepcopy(N[n]); alt, _ = _strom(n, K); faktor = i_max / alt
        k['strom_A'] = i_max
        k['strom_A_quelle'] = str(k.get('strom_A_quelle') or 'schaetzung') + ' (lernbegrenzt)'
        fl = k.get('fluss') or {}
        if fl.get('einheit') == 'A':
            fl['gesamt'] = i_max
            fl['last'] = {p: float(i) * faktor for p, i in (fl.get('last') or {}).items()}
            fl['lernbegrenzung'] = dict(original_A=alt, faktor=faktor, grenze_A=i_max)
        KK['netze'][n] = k
    return KK, zu


def note_v3(B, R, K, gndlagen=None):
    ""
    t0 = time.time()
    try:
        from analyse.bewertung import bewerte
        K, _zu = strom_begrenzt(K)
        E = bewerte(B, dict(R, gndlagen=list(gndlagen or [])), K, v3=True)
        M = E['metriken']
        top = []
        for n, v in sorted(M['netze'].items(), key=lambda kv: (-kv[1]['beitrag'], kv[0]))[:3]:
            gr = sorted(v['teile'].items(), key=lambda kv: (-kv[1], kv[0]))[:2]
            top.append(dict(netz=n, klasse=v['klasse'], beitrag=v['beitrag'],
                            grund=', '.join('%s %.2f (x %.2f)' % (k, w, v['x'].get(k, 0.0)) for k, w in gr)))
        return M['note_v3'], top, round(time.time() - t0, 3)
    except Exception as e:
        return None, [dict(fehler=repr(e)[:200])], round(time.time() - t0, 3)


def v3_rel(note, mensch):
    ""
    try:
        return round(float(note) / float(mensch), 4) if note is not None and mensch is not None and float(mensch) > 1e-6 else None
    except (TypeError, ValueError):
        return None


def zustand_von(rc, log):
    ""
    if rc in ('zeitlimit', 'wandlimit'): return rc, False
    if rc != 0: return 'fehler', False
    try: txt = open(log, errors='replace').read()
    except OSError: txt = ''
    return ('zeitlimit', True) if 'Rechenzeit-Budget' in txt else ('ok', False)


def konflikt_verlauf(log):
    ""
    import re
    try: return [[int(a), int(b)] for a, b in re.findall(r'Iteration \d+: Konflikte (\d+), unvollstaendig (\d+)', open(log, errors='replace').read())]
    except OSError: return []


def zufall_cfg(rnd):
    return dict(seed=rnd.randint(1, 10 ** 6), **{k: rnd.choice(v) for k, v in RAUM.items()})


def nativ(cmd, log, cpu=300):
    from gemeinsam import plaetze as PL
    cmd = list(cmd)
    if cmd[0] == KO.KPY: cmd[0] = '/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3'
    r = PL.lauf(cmd, cpu_s=cpu, log=log)
    if r['zustand'] != 'ok': raise RuntimeError('KiCad: ' + repr(r))
    return r


def vorbereiten(d, o):
    ""
    os.makedirs(o, exist_ok=True)
    pcb = os.path.join(d, 'processed.kicad_pcb')
    if not os.path.exists(pcb): return None
    for cmd in ([KO.KPY, os.path.join(R, 'kicad/export_board.py'), pcb, os.path.join(o, 'board.json')],
                [KO.KPY, os.path.join(R, 'kicad/routes_export.py'), pcb, os.path.join(o, 'mensch.json')]):
        nativ(cmd, os.path.join(o, os.path.basename(cmd[1]) + '.log'))
    if not os.path.exists(os.path.join(o, 'board.json')): return None
    B = json.load(open(os.path.join(o, 'board.json'))); L = B.get('lagen') or 2
    RG = KO.regeln_aus(pcb, L, B); json.dump(RG, open(os.path.join(o, 'regeln.json'), 'w'))
    vj = os.path.join(o, 'vorgaben.json')
    vg_original = os.path.join(d, 'vorgaben.json')
    VG = json.load(open(vg_original)) if os.path.exists(vg_original) else {}
    VG.setdefault('breite', RG.get('breite') or {})
    json.dump(VG, open(vj, 'w'))
    K = KO.klassen_lauf(o, os.path.join(o, 'board.json'), os.path.join(o, 'klassen.json'), vj)
    gndlagen, gndzonen = [], []
    mit_ebenen = (EBENEN['an'] and L == 4) or (EBENEN.get('lagen68') and L in (6, 8))
    if mit_ebenen:
        from lernen.ebenen import aus_original
        source = os.path.join(EBENEN.get('originale') or d, os.path.basename(d), 'original.kicad_pcb') if EBENEN.get('originale') else pcb
        source = source if os.path.exists(source) else pcb
        gndlagen, gndzonen = aus_original(source, B, K, o, nativ=EBENEN.get('nativ', False))
        if not gndlagen and EBENEN.get('k19_ersatz', True):
            gndlagen, gndzonen = k19_ersatz(B, K, o, L)
    json.dump(KO.verwuerfeln(B, 1), open(os.path.join(o, 'board_wuerfel.json'), 'w'))
    KW = KO.klassen_lauf(o, os.path.join(o, 'board_wuerfel.json'), os.path.join(o, 'klassen_wuerfel.json'), vj)
    beweglich = [r for r, t in KW['teile'].items() if not t['fest'] and t['rolle'] in ('klein', 'chip')]
    if len(beweglich) < 3: return None
    M = KO.bewerte_wer(B, K, os.path.join(o, 'mensch.json'))
    if M and EBENEN.get('nativ'):
        from lernen.ebenen import original_drc
        M['drc'] = original_drc(pcb, os.path.join(o, 'mensch.kicad_pcb'), KO)
    elif M: M['drc'] = KO.drc_pruef(pcb, None, os.path.join(o, 'mensch.kicad_pcb'), RG['router']['via'], L)
    seiten = ''.join(sorted({f['side'] for f in B['fps'] if f.get('n_pads')}, reverse=True)) or 'F'
    kzm = None
    if V2['an']:
        try:
            from analyse.kennzahlen import kennzahlen
            kzm = kennzahlen(json.load(open(os.path.join(o, 'mensch.json'))))
        except Exception: kzm = None
    v3m = dict(note=None, top=None, s=None)
    if V3['an']:
        try: v3m['note'], v3m['top'], v3m['s'] = note_v3(B, json.load(open(os.path.join(o, 'mensch.json'))), K, gndlagen=gndlagen)
        except Exception as e: v3m['top'] = [dict(fehler=repr(e)[:200])]
    from analyse.dichte import messen
    from analyse.standard import ziel
    return dict(**(dict(dichte_original=messen(B, RG)) if DICHTE else {}), profil=VG.get('profil', 'ausgewogen'), bewertungsziel=ziel(VG, explizit=V3.get('ziel')), pcb=pcb, lagen=L, seiten=seiten, mensch=M, merkmale_mensch=merkmale(B, K, RG, gndlagen=gndlagen), beweglich=len(beweglich),
                via=RG['router']['via'], kennzahlen_mensch=kzm, v3_mensch=v3m, gndlagen=gndlagen, gndzonen=gndzonen, gnd_quelle=source if mit_ebenen else None,
                gndzonen_sha256=hashlib.sha256(json.dumps(gndzonen, sort_keys=True).encode()).hexdigest())


def variante_eng(d, o, rnd):
    ""
    import shutil as _sh
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    with open(os.path.join(o, 'board.json')) as f: B = json.load(f)
    with open(os.path.join(o, 'klassen_wuerfel.json')) as f: KW = json.load(f)
    if (KW.get('vorgaben') or {}).get('umriss_fest'): return None, None
    if len(B['outline']) != 1: return None, None
    if any(B.get('outline_holes') or []): return None, None
    native_p = os.path.join(o, 'umriss_native.json')
    native_python = os.path.expanduser('~/.local/bin/codex-kicad-python')
    proc = subprocess.run([native_python, os.path.join(R, 'analyse/dichte_native.py'), os.path.join(d, 'processed.kicad_pcb'), native_p],
                          capture_output=True, text=True, timeout=300)
    if proc.returncode != 0: raise RuntimeError('Native Umrisspruefung fehlgeschlagen: ' + proc.stderr[-300:])
    with open(native_p) as f: native = json.load(f)
    if not native['eng_erlaubt']: return None, None
    U = unary_union([Polygon(q) for q in B['outline'] if len(q) >= 3]); x0, y0, x1, y1 = U.bounds
    hx, hy = (x1 - x0) / 2, (y1 - y0) / 2

    fest = {r for r, t in KW['teile'].items() if t['fest']}
    pts = [pt for c in B.get('courtyards', []) if c['ref'] in fest for q in c['polys'] for pt in q]
    from gemeinsam.teile import lade_teile
    TT = lade_teile(B)
    fp = {f['ref']: f for f in B['fps']}
    pts += [pt for r in sorted(fest & set(TT)) for g in TT[r].geo(fp[r]['x'], fp[r]['y'], 0, fp[r]['side']).values()
            for pt in ((g.bounds[0], g.bounds[1]), (g.bounds[2], g.bounds[3]))]

    pts += [pt for k in B.get('keepouts', []) if not k.get('ref') for poly in k['polys'] for pt in poly]
    pts += [pt for k in B.get('kupfer_grafik', []) for poly in k['polys'] for pt in poly]
    statisch = [Polygon(poly).buffer(0) for key in ('keepouts', 'kupfer_grafik')
                for k in B.get(key, []) if key == 'kupfer_grafik' or not k.get('ref')
                for poly in k['polys'] if len(poly) >= 3]

    frei = dict(l=min([px - x0 for px, py in pts] + [2 * hx]), r=min([x1 - px for px, py in pts] + [2 * hx]),
                o=min([py - y0 for px, py in pts] + [2 * hy]), u=min([y1 - py for px, py in pts] + [2 * hy]))
    zug = {k: max(0.0, min(v - 1.0, 0.2 * (2 * (hx if k in 'lr' else hy)))) * rnd.uniform(0.6, 1.0) for k, v in frei.items()}
    X0, X1, Y0, Y1 = x0 + zug['l'], x1 - zug['r'], y0 + zug['o'], y1 - zug['u']
    f = (X1 - X0) * (Y1 - Y0) / ((x1 - x0) * (y1 - y0))
    if f > 0.92: return None, None



    from shapely.affinity import affine_transform
    fx, fy = (X1 - X0) / (x1 - x0), (Y1 - Y0) / (y1 - y0)
    eng_u = affine_transform(U, [fx, 0, 0, fy, X0 - fx * x0, Y0 - fy * y0])
    if any(g.difference(eng_u).area > g.difference(U).area + 1e-6 for g in statisch): return None, None
    for r in sorted(fest & set(TT)):
        for g in TT[r].geo(fp[r]['x'], fp[r]['y'], 0, fp[r]['side']).values():
            if g.difference(eng_u).area > g.difference(U).area + 1e-6: return None, None
    q = os.path.join(os.path.dirname(o), '_quellen', os.path.basename(o) + '_eng'); os.makedirs(q, exist_ok=True)
    target = os.path.join(q, 'processed.kicad_pcb')
    from kicad.projekt_kopie import regeln_sichern, regeln_ziel_leeren, regeln_kopieren
    projektregeln = regeln_sichern(os.path.join(d, 'processed.kicad_pcb'))
    regeln_ziel_leeren(target)
    if os.path.exists(target): os.remove(target)
    proc = subprocess.run([KO.KPY, os.path.join(R, 'kicad/umriss_skalieren.py'), os.path.join(d, 'processed.kicad_pcb'), target,
                    'box'] + ['%.4f' % v for v in (x0, y0, x1, y1, X0, Y0, X1, Y1)],
                   capture_output=True, text=True, timeout=300)
    if proc.returncode != 0: return None, None
    regeln_kopieren(projektregeln, target, erzeugtes_pro=True)


    probe = os.path.join(q, 'umriss_pruef.json')
    if os.path.exists(probe): os.remove(probe)
    proc = subprocess.run([native_python, '-c',
                           "import wx,runpy,sys; app=wx.GetApp() or wx.App(False); lz=wx.LogStderr(); wx.Log.SetActiveTarget(lz); sys.argv=sys.argv[1:]; runpy.run_path(sys.argv[0],run_name='__main__')",
                           os.path.join(R, 'kicad/export_board.py'), target, probe],
                          capture_output=True, text=True, timeout=300)
    if proc.returncode != 0 or not os.path.exists(probe):
        raise RuntimeError('Eng-Export fehlgeschlagen: rc=%s %s' % (proc.returncode, proc.stderr[-500:]))
    with open(probe) as h: BN = json.load(h)
    UN = unary_union([Polygon(poly) for poly in BN['outline'] if len(poly) >= 3]).buffer(0)
    if any(g.difference(UN).area > g.difference(U).area + 1e-6 for g in statisch): return None, None
    fn = {p['ref']: p for p in BN['fps']}
    for r in sorted(fest):
        if r not in fn or any(fn[r].get(k) != fp[r].get(k) for k in ('x', 'y', 'side', 'rot')): return None, None
        if r in TT:
            for g in TT[r].geo(fp[r]['x'], fp[r]['y'], 0, fp[r]['side']).values():
                if g.difference(UN).area > g.difference(U).area + 1e-6: return None, None
    f = UN.area / U.area
    if not 0 < f <= 0.92: return None, None
    if os.path.exists(os.path.join(d, 'vorgaben.json')): _sh.copy(os.path.join(d, 'vorgaben.json'), q)
    if os.path.exists(os.path.join(d, 'metadata.json')): _sh.copy(os.path.join(d, 'metadata.json'), q)
    return (q, round(f, 3)) if os.path.exists(os.path.join(q, 'processed.kicad_pcb')) else (None, None)


def eng_referenz(info, original, source, faktor):
    ""
    for key in ('mensch', 'v3_mensch', 'kennzahlen_mensch', 'dichte_original'):
        info[key] = original.get(key)
    info['familie'] = os.path.basename(source)
    info['variante'] = dict(art='eng', faktor=faktor, von=os.path.basename(source))
    return info


def _kandidat(d, o, info, k, cfg, it, zeit, drc, nur_platzieren=False, vorbereitet=False):
    ""
    c = os.path.join(o, 'k%d' % k); os.makedirs(c, exist_ok=True); t0 = time.time()
    zeile = dict(lagen=info.get('lagen'), gndlagen=info.get('gndlagen', []), gnd_quelle=info.get('gnd_quelle'), gndzonen_sha256=info.get('gndzonen_sha256'), board=os.path.basename(o), kandidat=k, cfg=cfg, beweglich=info['beweglich'], seiten=info['seiten'],
                 mensch=info['mensch'], merkmale_mensch=info['merkmale_mensch'], variante=info.get('variante'))
    if V3['an']:
        v3m = info.get('v3_mensch') or {}
        zeile.update(note_v3=None, note_v3_mensch=v3m.get('note'), v3_rel=None, v3_schlechteste=None,
                     v3_schlechteste_mensch=v3m.get('top'), v3_s=None, v3_s_mensch=v3m.get('s'))
    from lernen.familien import board_familie
    zeile['familie'] = info.get('familie') or board_familie(d)
    if V2['an']:
        zeile.update(format='v2', familie=zeile['familie'], code=V2['code'], code_sha256=V2['code'] if len(V2['code']) == 64 else None,
                     router_sha256=hashlib.sha256(open(os.path.join(R, 'router/route_pf.py'), 'rb').read()).hexdigest(),
                     klassen_sha256=hashlib.sha256(open(os.path.join(R, 'analyse/klassen.py'), 'rb').read()).hexdigest(),
                     last=os.getloadavg()[0], budget=dict(cpu_s=zeit, iter=it),
                     kennzahlen_mensch=info.get('kennzahlen_mensch'), cfg_quelle=info.get('cfg_quelle', {}).get(k, 'zufall'))
    try:
        if not vorbereitet:
            args = ['seed=%d' % cfg['seed'], 'kreuz=%g' % cfg['kreuz'], 'trenn=%g' % cfg['trenn'], 'via_kosten=%g' % cfg['via_kosten'],
                    'fein=' + cfg['fein'], 'seiten=' + info['seiten'], 'seiten_original=1']
            if cfg.get('dicht'): args += ['dicht=%g' % cfg['dicht']]
            pcmd = [KO.VPY, os.path.join(R, 'platzierung/placer.py'), os.path.join(o, 'board.json'), os.path.join(o, 'klassen_wuerfel.json'),
                    os.path.join(c, 'placement.json'), 'regeln=' + os.path.join(o, 'regeln.json'), 'modus=neu', 'zufall=1'] + args

            from gemeinsam import plaetze as PL
            rp = PL.lauf(pcmd, cpu_s=zeit, log=os.path.join(c, 'placer.log'), env=dict(os.environ, PYTHONHASHSEED='0'))
            zeile['placer'] = dict(zustand=rp['zustand'], cpu_s=rp['cpu_s'], warte_s=rp['warte_s'])
            zeile['placer_versuche'] = [dict(seed=cfg['seed'], zustand=rp['zustand'], cpu_s=rp['cpu_s'])]
            if rp['zustand'] != 'ok':
                zeile['zustand'] = 'placer_' + rp['zustand']; raise RuntimeError('Placer: ' + rp['zustand'])


            def kollisionen():
                try: return (json.load(open(os.path.join(c, 'placement.json'))).get('kennzahlen') or {}).get('nachher', {}).get('kollisionen')
                except (OSError, ValueError): return None
            for retry in range(2 if PLACEMENT_SCHUTZ else 0):
                if kollisionen() == 0: break
                cmd = [x if not x.startswith('seed=') else 'seed=%d' % (cfg['seed'] + 7919 * (retry + 1)) for x in pcmd]
                rp = PL.lauf(cmd, cpu_s=zeit, log=os.path.join(c, 'placer_wdh%d.log' % (retry + 1)), env=dict(os.environ, PYTHONHASHSEED='0'))
                zeile['placer']['cpu_s'] += rp['cpu_s']
                zeile['placer']['zustand'] = rp['zustand']
                zeile['placer_versuche'].append(dict(seed=cfg['seed'] + 7919 * (retry + 1), zustand=rp['zustand'], cpu_s=rp['cpu_s']))
                zeile['placer_wdh'] = retry + 1
                if rp['zustand'] != 'ok': break
            if PLACEMENT_SCHUTZ and (kollisionen() != 0 or rp['zustand'] != 'ok'):
                zeile.update(gueltig=False, zustand='placer_' + rp['zustand'] if rp['zustand'] != 'ok' else 'placer_kollision',
                    ungueltig_grund='Platzierung kollidiert oder Kollisionsmessung fehlt')
                raise RuntimeError(zeile['ungueltig_grund'])
            ap = os.path.join(c, 'auto.kicad_pcb')
            nativ([KO.KPY, os.path.join(R, 'platzierung/apply_place.py'), info['pcb'], os.path.join(c, 'placement.json'), ap], os.path.join(c, 'apply.log'))
            nativ([KO.KPY, os.path.join(R, 'kicad/export_board.py'), ap, os.path.join(c, 'board.json'), 'placement=' + os.path.join(c, 'placement.json')], os.path.join(c, 'export.log'))
            BA = json.load(open(os.path.join(c, 'board.json')))
            KA = KO.klassen_lauf(o, os.path.join(c, 'board.json'), os.path.join(c, 'klassen.json'), os.path.join(o, 'vorgaben.json'))
            vf = float(cfg.get('via_f', 1.0))
            if vf != 1.0:
                for k_ in KA['netze'].values():
                    if 'via' in k_: k_['via'] = round(k_['via'] * vf, 3)
                json.dump(KA, open(os.path.join(c, 'klassen.json'), 'w'))
            RG = json.load(open(os.path.join(o, 'regeln.json')))
            zeile['place_s'] = round(time.time() - t0, 1)
            zeile['merkmale'] = dict(merkmale(BA, KA, RG, gndlagen=info.get('gndlagen', [])), via_f=vf)
            zeile['placer_kennzahlen'] = (json.load(open(os.path.join(c, 'placement.json'))).get('kennzahlen') or {}).get('nachher')
            zeile['s'] = round(time.time() - t0, 1)
            json.dump(zeile, open(os.path.join(c, 'vorbereitet.json'), 'w'))
        else:
            zeile = json.load(open(os.path.join(c, 'vorbereitet.json')))
            t0 -= zeile.get('s', 0)
            BA = json.load(open(os.path.join(c, 'board.json')))
            KA = json.load(open(os.path.join(c, 'klassen.json')))
            RG = json.load(open(os.path.join(o, 'regeln.json')))
            ap = os.path.join(c, 'auto.kicad_pcb')
        if DICHTE or cfg.get('dicht') or info.get('dichte_original'):
            from analyse.dichte import gegen
            zeile['dichte'] = gegen(BA, info.get('dichte_original') or json.load(open(os.path.join(o, 'board.json'))), RG)
        if nur_platzieren: return zeile
        from lernen.ki.daten import route_kontext
        rc, ts = KO.route_lauf(route_kontext(o, c), os.path.join(c, 'board.json'), os.path.join(c, 'klassen.json'), os.path.join(c, 'routes.json'), it, zeit,
                               os.path.join(c, 'route.log'), **({'gndlagen': info.get('gndlagen', [])} if EBENEN.get('nativ') else {}))
        zeile.update(router_rc=rc, route_s=ts, router=dict(cpu_s=ts))
        zeile['zustand'], zeile['budget_ende'] = zustand_von(rc, os.path.join(c, 'route.log'))
        zeile.update(stromnachlauf(*[os.path.join(c, f) for f in ('board.json', 'klassen.json', 'routes.json')],
                                   os.path.join(o, 'regeln.json'), info.get('gndlagen', []),
                                   zonen=R36_ZONEN, glatt=R36_GLATT, ir=R11_IR))
        if cfg.get('k20_zonen') and os.path.exists(os.path.join(c, 'routes.json')):
            from lernen.k20_nachlauf import anwenden as k20_anwenden
            zeile['k20_zonen'] = k20_anwenden(os.path.join(c, 'board.json'), os.path.join(c, 'klassen.json'),
                                           os.path.join(c, 'routes.json'), os.path.join(o, 'regeln.json'),
                                           info.get('gndlagen', []), cpu_s=zeit)
        if cfg.get('r19_nachlauf') and os.path.exists(os.path.join(c, 'routes.json')):
            from lernen.r19_nachlauf import anwenden as r19_anwenden
            zeile['r19_nachlauf'] = r19_anwenden(os.path.join(c, 'board.json'), os.path.join(c, 'klassen.json'),
                                               os.path.join(c, 'routes.json'), os.path.join(o, 'regeln.json'),
                                               info.get('gndlagen', []))
        from nachbearbeitung.kurzschluss_schutz import kandidat_sichern_messung
        r44_grund, zeile['r44'] = kandidat_sichern_messung(os.path.join(c, 'board.json'), os.path.join(c, 'routes.json'))
        if r44_grund:
            zeile.update(gueltig=False, zustand='r44_ungeprueft', ungueltig_grund=r44_grund[:300], ergebnis=None,
                         s=round(time.time() - t0, 1))
            return zeile
        E = KO.bewerte_wer(BA, KA, os.path.join(c, 'routes.json'))
        if V2['an']:
            zeile['zustand'], zeile['budget_ende'] = zustand_von(rc, os.path.join(c, 'route.log'))
            zeile['konflikt_verlauf'] = konflikt_verlauf(os.path.join(c, 'route.log'))
            if E:
                try:
                    from analyse.kennzahlen import kennzahlen
                    zeile['kennzahlen'] = kennzahlen(json.load(open(os.path.join(c, 'routes.json'))))
                except Exception as e_: zeile['kennzahlen_fehler'] = repr(e_)[:200]
        if V3['an']:
            n3, top3, s3 = None, None, 0.0
            if E:
                try: n3, top3, s3 = note_v3(BA, json.load(open(os.path.join(c, 'routes.json'))), KA, gndlagen=info.get('gndlagen', []))
                except Exception as e_: top3 = [dict(fehler=repr(e_)[:200])]
            zeile.update(note_v3=n3, v3_rel=v3_rel(n3, zeile.get('note_v3_mensch')), v3_schlechteste=top3, v3_s=s3,
                         v3_strom_begrenzt=strom_begrenzt(KA)[1][:10] or None)
        fest = [r for r, t in json.load(open(os.path.join(o, 'klassen_wuerfel.json')))['teile'].items() if t['fest']]
        if E and info.get('gndlagen'):
            from analyse.bewertung import bewerte
            E['gueltig'] = bewerte(BA, json.load(open(os.path.join(c, 'routes.json'))), KA, minimax=False)['gueltig']
            E['unverbunden'] = len(__import__('analyse.bewertung', fromlist=['unverbunden']).unverbunden(BA, json.load(open(os.path.join(c, 'routes.json'))), set(KA['gnd'])))
        native_routes = None
        if E and drc:
            if EBENEN.get('nativ') or info.get('gndlagen'):
                from lernen.ebenen import drc as ebenen_drc
                E['drc'] = ebenen_drc(ap, os.path.join(c, 'routes.json'), os.path.join(c, 'fertig.kicad_pcb'), info, RG, fest, KO, nativ=EBENEN.get('nativ', False))
            else:
                rv = json.load(open(os.path.join(c, 'routes.json'))).get('optionen', {}).get('via', info['via'])
                E['drc'] = KO.drc_pruef(ap, os.path.join(c, 'routes.json'), os.path.join(c, 'fertig.kicad_pcb'), rv, info['lagen'], fest)
            E['drc']['platz_netto'] = KO.platz_netto(E['drc'], (info.get('mensch') or {}).get('drc'))
            if EBENEN.get('nativ') and info['lagen'] in (6, 8):
                from analyse.bewertung import native_flaechen, bewerte
                RR = dict(json.load(open(os.path.join(c, 'routes.json'))), drc=E['drc'])
                RR['native_flaechen'] = dict(version='native-flaechen-v1', netze=sorted({n for q in info.get('gndlagen', []) if '=' in q for n in q.split('=', 1)[1].split('|')}))
                if native_flaechen(RR, KA):
                    native_routes = RR
                    checked = bewerte(BA, RR, KA, minimax=False)
                    E['unverbunden'], E['gueltig'] = len(checked['unverbunden']), checked['gueltig']
        zeile.update(router_rc=rc, route_s=ts, router=dict(cpu_s=ts), ergebnis=E, gueltig=KO.gueltig_voll(E, True, nativ=EBENEN.get('nativ', False)) if E else False)
        if E:
            zeile['kosten'] = KO.kosten(E)
            if V3['an']:
                try:
                    zeile['regel'] = regelmessung(BA, native_routes or dict(json.load(open(os.path.join(c, 'routes.json'))), **({'drc': E.get('drc')} if drc else {})), KA, info.get('gndlagen', []), dicht=True)
                    if 'tor' in zeile['regel']:
                        zeile['tor'] = zeile['regel']['tor']
                    zeile['bewertung_version'] = zeile['regel']['minimax_version'] if zeile['regel'].get('bewertungsziel', 'v3') == 'minimax' else zeile['regel']['version']
                    if zeile['regel'].get('bewertungsziel', 'v3') == 'minimax':
                        zeile['gueltig_routing'] = zeile['gueltig']
                        zeile['gueltig'] = (zeile['gueltig'] if EBENEN.get('nativ') else E['gueltig']) and zeile['regel']['minimax_gueltig']


                    if EBENEN.get('nativ'): zeile['regel']['gueltig'] = zeile['gueltig']
                except Exception as e_:
                    zeile['regel_fehler'] = repr(e_)[:300]
                    if EBENEN.get('nativ'): zeile['gueltig'] = False
    except Exception as e:
        zeile['fehler'] = repr(e)[:300]
        if V2['an']: zeile.setdefault('zustand', 'fehler')
    if R36_ZONEN or R36_GLATT:
        from lernen.zonenplaner import abnahmetor
        zeile.setdefault('gueltig', False)
        abnahmetor(zeile, zeile.get('stromzonen'), 'legacy')
    zeile['s'] = round(time.time() - t0, 1)
    return zeile


def kandidat(d, o, info, k, cfg, it, zeit, drc, nur_platzieren=False, vorbereitet=False, vor_aufnahme=None):
    tc = time.thread_time()
    vorher = code_stand()
    z = _kandidat(d, o, info, k, cfg, it, zeit, drc, nur_platzieren, vorbereitet)
    if vor_aufnahme is not None: z = vor_aufnahme(z)
    nachher = code_stand()
    z.setdefault('code_sha256', vorher)
    if vorher != nachher or (V2['an'] and V2['code'] != nachher) or z.get('code_sha256') != nachher:
        z.update(code_geaendert=True, code_nachher=nachher)
    from lernen.lerneffekt import ergaenzen, kinder_cpu_s
    ergaenzen(z, os.path.join(o, 'k%d' % k))
    try:
        from lernen.ki.daten import aufnehmen
        metrics = (z.get('regel') or {}).pop('_physik', None)
        z.setdefault('familie', info.get('familie') or os.path.basename(o).removesuffix('_eng'))
        z['datensatz'] = aufnehmen(os.path.join(o, 'k%d' % k), z, os.path.join(o, 'board.json'),
                                  herkunft=info.get('etiketten_herkunft', 'dauerlauf'), metrics=metrics)
    except Exception as e: z['datensatz'] = dict(zustand='fehler', fehler=repr(e)[:200])
    z['cpu_total'] = kinder_cpu_s(z) + time.thread_time() - tc
    z['cpu_messung'] = 'teilweise'
    return z


def main():
    tr, out = sys.argv[1], sys.argv[2]
    kw = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a)
    K_ = int(kw.get('kandidaten', '4')); P = int(kw.get('parallel', '8')); zeit = int(kw.get('zeit', '240')); it = int(kw.get('iter', '40'))
    maxp = int(kw.get('max_pads', '220')); nb = int(kw.get('boards', '60')); drc = kw.get('drc', '1') == '1'
    eng = kw.get('eng', '0') == '1'
    global DICHTE; DICHTE = eng or kw.get('dichte_messen') == '1'
    global PLACEMENT_SCHUTZ; PLACEMENT_SCHUTZ = kw.get('placement_schutz', '0') == '1'
    global R11_IR; R11_IR = kw.get('r11_ir', '0') == '1'
    global R36_ZONEN, R36_GLATT
    R36_ZONEN = kw.get('r36_zonen', '0') == '1'
    R36_GLATT = kw.get('r36_glatt', '0') == '1'
    daten = kw.get('daten', os.path.join(out, 'versuche.jsonl')); rnd = random.Random(int(kw.get('seed', '1')))
    V2['an'] = kw.get('v2', '0') == '1'; V3['an'] = kw.get('v3', '1') == '1'
    if V2['an']:
        KO.ROUTER_EXTRA = tuple(KO.ROUTER_EXTRA) + ('cpu_max=%d' % zeit,)
        try: V2['code'] = code_stand()
        except Exception: V2['code'] = ''
    for k_ in list(RAUM):
        if k_ in kw: RAUM[k_] = [kw[k_] if k_ == 'fein' else float(kw[k_])]
    cfg_alt = {}
    if kw.get('cfg_von'):
        for z_ in open(kw['cfg_von']):
            try: j_ = json.loads(z_); cfg_alt[(j_['board'], j_['kandidat'])] = j_['cfg']
            except Exception: pass
    ende = time.time() + float(kw['dauer']) if kw.get('dauer') else None; begonnen = set()
    os.makedirs(out, exist_ok=True); os.makedirs(os.path.dirname(os.path.abspath(daten)), exist_ok=True)
    dirs = sorted(d for d in glob.glob(os.path.join(tr, '*')) if os.path.isdir(d))
    rnd.shuffle(dirs)
    fertig = set()
    if os.path.exists(daten):
        for z in open(daten):
            try: j = json.loads(z); fertig.add((j['board'], j['kandidat']))
            except Exception: pass
    t0 = time.time(); n_ok = 0

    def board(d):
        o = os.path.join(out, os.path.basename(d))
        try:
            meta = json.load(open(os.path.join(d, 'metadata.json')))
            if int(meta.get('layers') or 2) != 2: return []
            txt = open(os.path.join(d, 'processed.kicad_pcb'), errors='replace').read()
            if txt.count('(pad ') > maxp * 1.15: return []
            info = vorbereiten(d, o)
        except Exception:
            return []
        if not info or info['merkmale_mensch']['n_pads'] > maxp: return []
        from lernen.familien import board_familie
        info['familie'] = board_familie(d)
        jobs = [(d, o, info, k) for k in range(K_)]
        if eng:
            try:
                q, f = variante_eng(d, o, random.Random(__import__("zlib").crc32(os.path.basename(d).encode())))
                if q:
                    o2 = os.path.join(out, os.path.basename(q)); info2 = vorbereiten(q, o2)
                    if info2:
                        eng_referenz(info2, info, d, f)
                        jobs += [(q, o2, info2, k) for k in range(K_)]
            except Exception as e:
                print('Variante eng', os.path.basename(d), repr(e)[:120], flush=True)
        return jobs

    with ThreadPoolExecutor(P) as ex:
        jobs = [j for js in ex.map(board, dirs[:nb]) for j in js]
    print('Aufgaben: %d Boards, %d Kandidaten (%.0fs Vorbereitung)' % (len({j[0] for j in jobs}), len(jobs), time.time() - t0), flush=True)
    cfgs = {(d, k): zufall_cfg(rnd) for d, o, info, k in jobs}
    for d, o, info, k in jobs:
        if (os.path.basename(d), k) in cfg_alt:
            cfgs[(d, k)] = dict(cfg_alt[(os.path.basename(d), k)]); info.setdefault('cfg_quelle', {})[k] = 'cfg_von'
        if kw.get('k20_zonen', '0') == '1': cfgs[(d, k)]['k20_zonen'] = True
        if kw.get('r19_nachlauf', '0') == '1': cfgs[(d, k)]['r19_nachlauf'] = True

    def lauf(j):
        d, o, info, k = j
        if (os.path.basename(d), k) in fertig: return None
        with LOCK:
            if ende and time.time() > ende and o not in begonnen: return None
            begonnen.add(o)
        z = kandidat(d, o, info, k, cfgs[(d, k)], it, zeit, drc)
        with LOCK:
            open(daten, 'a').write(json.dumps(z) + '\n')
        if kw.get('aufraeumen', '1') == '1':
            for f in ('auto.kicad_pcb', 'fertig.kicad_pcb', 'fertig.kicad_pro', 'auto.kicad_pro'):
                try: os.remove(os.path.join(o, 'k%d' % k, f))
                except OSError: pass
        return z
    with ThreadPoolExecutor(P) as ex:
        for z in ex.map(lauf, jobs):
            if z is None: continue
            n_ok += bool(z.get('gueltig'))
            print('%-40s k%d %s kosten %s  %4.0fs  (gueltig bisher %d)' % (z['board'][:40], z['kandidat'], 'ja  ' if z.get('gueltig') else 'nein',
                  z.get('kosten'), z['s'], n_ok), flush=True)
    print('fertig in %.0fs -> %s' % (time.time() - t0, daten))


if __name__ == '__main__':
    main()
