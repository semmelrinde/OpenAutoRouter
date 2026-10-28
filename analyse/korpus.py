""
import json, os, re, sys, subprocess, time, glob, shutil, random, zlib
from concurrent.futures import ThreadPoolExecutor
R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, R)
from gemeinsam.prozessschutz import bootstrap
if __name__ == "__main__": bootstrap(heavy=False)
from analyse.bewertung import bewerte, unverbunden
from analyse import dichte as DM

from kicad.python_aufruf import KPY
VPY = os.path.join(R, '.venv', 'bin', 'python')


def netzklassen_aus(pcb):
    ""
    txt = open(pcb, encoding='utf-8', errors='replace').read(); out = []
    for m in re.finditer(r'\(net_class (\S+)(.*?)\n  \)', txt, re.S):
        b = m.group(2); w = {}
        for k in ('clearance', 'trace_width', 'via_dia', 'via_drill'):
            q = re.search(r'\(' + k + r' ([\d.]+)\)', b)
            if q: w[k] = float(q.group(1))
        nets = [n.strip('"') for n in re.findall(r'\(add_net ("[^"]*"|[^)\s]+)\)', b)]
        out.append((m.group(1).strip('"'), w, nets))
    return out


def regeln_aus(pcb, lagen, B=None, sigmin_einheitlich=False):
    ""
    NKB = (B or {}).get('netzklassen') or {}
    if NKB:
        benutzt = set((B.get('netclass') or {}).values()) | {'Default'}
        NK = [(n, dict(clearance=k.get('clr'), trace_width=k.get('w'), via_dia=k.get('via_d'), via_drill=k.get('via_b')),
               [nt for nt, c in (B.get('netclass') or {}).items() if c == n] if n != 'Default' else ['*'])
              for n, k in NKB.items() if n in benutzt]
    else:
        NK = netzklassen_aus(pcb)
    D = next((w for n, w, _ in NK if n == 'Default'), {})
    def wert(k, d): return D.get(k) or d
    clr, w, vd, vb = wert('clearance', 0.2), wert('trace_width', 0.25), wert('via_dia', 0.6), wert('via_drill', 0.4)
    clr = max([clr] + [k.get('clearance') or 0 for n, k, nets in NK if nets])
    breite = {nt: k['trace_width'] for n, k, nets in NK if n != 'Default' and (k.get('trace_width') or 0) > w for nt in nets}
    ds = (B or {}).get('design') or {}
    from analyse.regeln import dru_global, dru_rules, randregeln, rand_strichbreite
    dru, dru_bedingt = dru_global(os.path.splitext(pcb)[0] + '.kicad_dru')
    rand_dru=dru_rules(os.path.splitext(pcb)[0] + '.kicad_dru')
    randvertrag = randregeln(ds.get('rand') or 0.0, rand_dru,rand_strichbreite(pcb,rand_dru))
    ds = dict(ds)
    for kind, key in (('clearance', 'min_abstand'), ('track_width', 'min_breite'),
                      ('edge_clearance', 'rand'), ('hole_clearance', 'loch_abstand'), ('hole_to_hole', 'loch_loch'),
                      ('hole_size', 'bohr_min'), ('via_diameter', 'via_min'), ('annular_width', 'ring_min')):
        if kind in dru: ds[key] = max(ds.get(key) or 0.0, dru[kind])
    bmin = ds.get('min_breite') or 0.0; clr = max(clr, ds.get('min_abstand') or 0.0)
    w = max(w, bmin); vd = max(vd, ds.get('via_min') or 0.0); vb = max(vb, ds.get('bohr_min') or 0.0)



    vd = max(vd, vb + 2 * (ds.get('ring_min') or 0.0), vb + 2 * ((ds.get('loch_abstand') or 0.0) - clr))
    rand = max(0.25, clr, randvertrag['edge_mm'] + 0.05)
    maske = 2 * (ds.get('maske') or 0.0) + (ds.get('maske_steg') or 0.0)
    padabst = round(max(0.15, maske + 0.02 if maske else 0.0), 3)
    padabst_drc = round(max(0.15, clr + 0.005, maske + 0.02 if maske else 0.0), 4)
    from analyse.abstaende import vertrag
    from analyse.regeln import dru_rules
    contract_board = B or dict(netzklassen={n:dict(clr=k.get('clearance')) for n,k,_ in NK},
                              netclass={nt:n for n,_,nets in NK for nt in nets if nt!='*'})
    contract = vertrag(contract_board, dru=dru_rules(os.path.splitext(pcb)[0]+'.kicad_dru'))
    return dict(abstaende=contract, router=dict(lagen=lagen, drc=clr, drcpad=clr, clr=round(clr + 0.005, 4), edge=rand, h2h=max(0.25, ds.get('loch_loch') or 0.0),
                            via='%s/%s' % (vd, vb), sigw=w, sigmin=w if sigmin_einheitlich else min(w, max(bmin, 0.15)), bahn_min=bmin, loch_clr=ds.get('loch_abstand') or 0.0, edge_exakt='1',
                            winkel45='1', vip='0'),
                placer=dict(padabst=padabst, padabst_drc=padabst_drc, rand=round(max(0.2, rand - padabst / 2 + 0.02), 3)),
                breite=breite, dru_global=dru, dru_bedingt=dru_bedingt, randregeln=randvertrag,
                quelle='Board-Setup, Netzklassen und globale .kicad_dru von ' + os.path.basename(pcb))


VIA_F = 1.0
FEST_VON = ''
DRC_PFLICHT = True
ROUTER_EXTRA = ()
SIGMIN_EINHEITLICH = False
DICHTE = False
ROUTEN = 1
ROUTEN_NUR_BEI_FEHLER = False
REPARATUR = 0
KANDIDATEN = 1
AUSWAHL = 'faust'
PAAR_MODELL = None
EINSTELLUNGEN = ''
LENKUNG = ''
PLACER_STANDARD = ()
_MOD = []
def rangfolge(kands):
    ""
    sys.path.insert(0, R)
    from lernen import modell as LM
    import numpy as np
    if AUSWAHL == 'erster': return list(kands)
    if AUSWAHL == 'faust':
        return sorted(kands, key=lambda k: k['merkmale']['mst'] * (1 + 0.02 * k['merkmale']['kreuzungen']) * (1 + max(0.0, k['merkmale']['rudy_max'] - 1)))
    if not _MOD:
        import joblib; _MOD.append(joblib.load(os.path.join(R, 'lernen/modell/auswahl.joblib')))
    P, C = LM.vorhersage(_MOD[0], [k['merkmale'] for k in kands]); med = float(np.median(C))
    E = P * C + (1 - P) * 2 * med
    return [kands[i] for i in np.argsort(E, kind='stable')]


def auswahl(kands):
    ""
    sys.path.insert(0, R)
    from lernen import modell as LM
    if AUSWAHL == 'erster': return kands[0]
    if AUSWAHL == 'faust': return LM.wahl_faust(kands)
    if not _MOD:
        import joblib; _MOD.append(joblib.load(os.path.join(R, 'lernen/modell/auswahl.joblib')))
    return LM.wahl_modell(_MOD[0])(kands)
KC = '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'

PLATZ_TYPEN = {'courtyards_overlap', 'copper_edge_clearance', 'npth_inside_courtyard', 'pth_inside_courtyard'}


def drc_pruef(pcb_src, routes, out_pcb, via, lagen, fest=None, zonen=None):
    ""
    import shutil
    try:
        from nachbearbeitung.kurzschluss_nativ import NativeBudget
        native = NativeBudget(300)
        if routes is None:
            shutil.copy(pcb_src, out_pcb)
            from kicad.projekt_kopie import regeln_sichern, regeln_kopieren
            regeln_kopieren(regeln_sichern(pcb_src), out_pcb)
        else:
            zj = out_pcb + '.zonen.json'; json.dump(list(zonen or []), open(zj, 'w'))
            imp = native.run([KPY, os.path.join(R, 'kicad/import_routes.py'), pcb_src, routes, zj, out_pcb, 'lagen=%d' % lagen, 'via=' + via])
            if imp.returncode: raise RuntimeError('KiCad-Import fehlgeschlagen: ' + imp.stderr[-400:])
        if routes is not None:
            from nachbearbeitung.kurzschluss_schutz import schutz_an
            try: r44_an = schutz_an(json.load(open(routes)).get('optionen'))
            except Exception: r44_an = True
        if routes is not None and r44_an:
            from nachbearbeitung.kurzschluss_nativ import sichern
            guard = sichern(out_pcb, out_pcb, out_pcb + '.r44.json', cpu_s=max(1,300-native.spent))
            native.spent += guard['cpu_s']
            if not guard.get('geprueft'): return dict(fehler='R44: '+guard.get('grund','ungeprueft'), r44=guard)
        js = out_pcb + '.drc.json'
        if os.path.exists(js): os.remove(js)
        native.run([KC, 'pcb', 'drc', '--format', 'json', '--severity-error', '-o', js, out_pcb])
        geo = native.run([KPY, os.path.join(R, 'kicad/drc_geometrie.py'), out_pcb, js])
        if geo.returncode: raise RuntimeError('DRC-Geometrie fehlgeschlagen: ' + geo.stderr[-400:])
        D = json.load(open(js))
        if routes is not None:
            import hashlib
            checked = out_pcb + '.routes_geprueft.json'
            native.run([KPY,os.path.join(R,'kicad/routes_export.py'),out_pcb,checked,'voll=1','einheiten=1'])
            with open(routes, 'rb') as fh: D['autorouter_eingang_routes_sha256'] = hashlib.sha256(fh.read()).hexdigest()
            with open(checked, 'rb') as fh: D['autorouter_routes_sha256'] = hashlib.sha256(fh.read()).hexdigest()
            with open(out_pcb, 'rb') as fh: D['autorouter_pcb_sha256'] = hashlib.sha256(fh.read()).hexdigest()
            D['autorouter_routes_datei'] = os.path.basename(checked)
            D['autorouter_routes_vertrag'] = 'native-abschlusskupfer-v2'
            D['autorouter_native_cpu_s'] = native.spent
            with open(js, 'w') as fh: json.dump(D, fh, indent=2)
    except Exception as e:
        return dict(fehler=repr(e)[:200])
    return drc_auswerten(D, fest)


def drc_auswerten(D, fest=None, nativ=False):
    ""
    import collections
    typen = collections.Counter(v['type'] for v in D['violations'])
    def routing(v): return any(re.match(r'(Track|Via|Arc) \[', i['description']) for i in v['items'])
    def teile(v): return {m.group(1) for i in v['items'] for m in [re.search(r'(?: of |^Footprint )(\S+?)(?: on \S+)?$', i['description'])] if m}
    def platz(v):
        if routing(v): return False
        t = teile(v)
        if fest is not None and (not t or t <= set(fest)): return False
        if v['type'] in PLATZ_TYPEN: return True
        if v['type'] in ('clearance', 'shorting_items', 'hole_clearance') and t: return True
        return v['type'] == 'solder_mask_bridge' and len(t) > 1
    pt = collections.Counter(v['type'] for v in D['violations'] if platz(v))

    ev = sorted({'%s|%s' % (v['type'], ','.join(sorted(teile(v)))) for v in D['violations'] if platz(v) or (fest is None and not routing(v) and teile(v))})



    evidence = D.get('autorouter_geometrie') or {}
    complete = evidence.get('version') == 2 and 'umgebung' in evidence and 'teile' in evidence
    geom = []
    for v in D['violations']:
        if not platz(v): continue
        refs = teile(v)

        proven = complete and bool(refs) and refs <= evidence['teile'].keys()
        geom.append(json.dumps([v['type'], v.get('description', ''), sorted(
            (i['description'], round(i['pos']['x'], 6), round(i['pos']['y'], 6)) for i in v['items']),
            [evidence['umgebung'], sorted((ref, evidence['teile'][ref]) for ref in refs)] if proven else None], ensure_ascii=False))
    geom.sort()


    schema = dict(version='korpus-drc-v1') if isinstance(D.get('unconnected_items'), list) else {}
    zonen_kupfer = sum(1 for v in D['violations'] if v['type'].startswith('zone') or
                       any(i['description'].startswith('Zone ') for i in v['items']))
    return dict(**schema, **({'zonen_kupfer': zonen_kupfer} if nativ else {}), routing=sum(1 for v in D['violations'] if routing(v)), platz=sum(pt.values()), platz_typen=dict(pt), platz_ereignisse=ev, platz_geometrie=geom,
                platz_geometrie_version=2 if complete else 0,
                unverbunden=len(D.get('unconnected_items', [])), alle=len(D['violations']), typen=dict(typen))


def platz_netto(d, basis):
    ""
    if not d: return 0
    if d.get('platz_geometrie_version') != 2 or (basis or {}).get('platz_geometrie_version') != 2:
        return d.get('platz', 0)

    from collections import Counter
    b = Counter((basis or {}).get('platz_geometrie') or [])
    return sum((Counter(e for e in d['platz_geometrie'] if json.loads(e)[-1] is not None) - b).values()) + sum(
        json.loads(e)[-1] is None for e in d['platz_geometrie'])


def verwuerfeln(B, seed):
    ""
    import random, copy
    rnd = random.Random(seed); B = copy.deepcopy(B)
    xs = [p['x'] for p in B['pads']]; ys = [p['y'] for p in B['pads']]
    d = {}
    for f in B['fps']:
        d[f['ref']] = (rnd.uniform(min(xs), max(xs)) - f['x'], rnd.uniform(min(ys), max(ys)) - f['y'])
        f['x'] += d[f['ref']][0]; f['y'] += d[f['ref']][1]
    for p in B['pads']:
        dx, dy = d.get(p['ref'], (0, 0)); p['x'] += dx; p['y'] += dy
    return B

def form(R):
    ""
    import math as _m
    spitz = 0; L = L45 = 0.0
    for t in R.get('tracks', []):
        P = t['pts']
        for a, b in zip(P[:-1], P[1:]):
            l = _m.dist(a, b); L += l
            w = _m.degrees(_m.atan2(b[1] - a[1], b[0] - a[0])) % 45.0
            if min(w, 45.0 - w) < 1.0: L45 += l
        for a, b, c in zip(P[:-2], P[1:-1], P[2:]):
            u = (a[0] - b[0], a[1] - b[1]); v = (c[0] - b[0], c[1] - b[1]); nu, nv = _m.hypot(*u), _m.hypot(*v)
            if nu < 1e-6 or nv < 1e-6: continue
            if (u[0] * v[0] + u[1] * v[1]) / (nu * nv) > 1e-6: spitz += 1
    return dict(spitze_knicke=spitz, anteil_45=round(L45 / L, 3) if L else None)


def normalisierte_route(B, K, Rr, gndlagen=None):
    ""
    from analyse.metriken import review_aktiv
    if gndlagen is not None and review_aktiv(K):
        from analyse.kupfermodell import flaechen
        gl = [x for x in gndlagen.split(',') if x] if isinstance(gndlagen, str) else list(gndlagen)
        planes, _, _ = flaechen(B, Rr, K, gndlagen=gl)
        if (Rr.get('gndlagen') == gl and (Rr.get('optionen') or {}).get('gndlagen') == gl
                and Rr.get('flaechennetze') == sorted(set(planes.values()))): return Rr


        Rr = dict(Rr, gndlagen=gl, optionen=dict(Rr.get('optionen') or {}, gndlagen=gl),
                  flaechennetze=sorted(set(planes.values())))
    return Rr


def bewerte_wer(B, K, f, gndlagen=None, fluss_cache=None, routes=None):
    if routes is None and not os.path.exists(f): return None
    RR = routes if routes is not None else json.load(open(f))
    from gemeinsam.zonen import boardgeometrie
    B = boardgeometrie(B, RR)
    Rr = normalisierte_route(B, K, RR, gndlagen)
    E = bewerte(B, Rr, K, fluss_cache=fluss_cache, minimax=False)
    from analyse.metriken import review_aktiv
    if review_aktiv(K):
        from analyse.kupfermodell import masse, masse_unverbunden
        planes, _ = masse(B, Rr, K)
        unv = len(set(unverbunden(B, Rr, set(planes.values()), fluss_cache=fluss_cache, K=K)) | set(masse_unverbunden(B, Rr, K)))
        ok = E['gueltig']
    else:
        unv = len(unverbunden(B, Rr, set(), fluss_cache=fluss_cache, K=K))
        ok = E['diagnose'] and E['konflikte'] == 0 and E['offen'] == 0
    return dict(gueltig=bool(ok and not unv), unverbunden=unv,
                konflikte=E['konflikte'], offen=E['offen'], vias=E['vias'], note=E['note'], gewichtet=E['gewichtet'],
                laenge_mm=round(sum(E['laenge'].values()), 1), laenge_gesamt_mm=E.get('laenge_gesamt'), umweg_mm=E['umweg_summe'], **form(Rr))


def klassen_lauf(o, bj, kj, vj, via_f=None):
    ""
    vf = VIA_F if via_f is None else float(via_f)
    subprocess.run([VPY, os.path.join(R, 'analyse/klassen.py'), bj, kj, 'regeln=' + os.path.join(o, 'regeln.json'), 'vorgaben=' + vj],
                   capture_output=True, text=True, timeout=300, env=dict(os.environ, PYTHONHASHSEED='0'))
    K = json.load(open(kj))
    if vf != 1.0:
        for k in K['netze'].values():
            if 'via' in k: k['via'] = round(k['via'] * vf, 3)
        json.dump(K, open(kj, 'w'))
    return K


def route_lauf(o, bj, kj, rj, it, zeit, log, stil=None, gndlagen=None):
    ""
    from gemeinsam import plaetze as PL
    from nachbearbeitung.kurzschluss_schutz import schutz_an, schutz_option
    extra = ['etiketten=' + os.path.splitext(rj)[0] + '_etiketten.npz'] + list(ROUTER_EXTRA) + (['stil=' + stil] if stil and stil != 'standard' else [])



    opt = dict(x.split('=', 1) for x in extra if '=' in x)
    if opt.get('etiketten_budget') == '1' and 'cpu_max' not in opt:
        extra += ['cpu_max=%d' % zeit]; opt['cpu_max'] = str(zeit)
    reserve = 60 if float(opt.get('cpu_max', '0')) > 0 else 0
    r = PL.lauf([VPY, os.path.join(R, 'router/route_pf.py'), bj, rj, 'regeln=' + os.path.join(o, 'regeln.json'), 'klassen=' + kj, 'gndlagen=' + ','.join(gndlagen or []),
                 'iter=%d' % it, 'pf0=1', 'pfmul=1.25', 'hinc=1.0', 'ziel_ab=20', 'kurz=1'] + extra + schutz_option(schutz_an(extra)), cpu_s=zeit + reserve, log=log,
                env=dict(os.environ, PYTHONHASHSEED='0'))
    rc = r['rc'] if r['zustand'] in ('ok', 'fehler') else r['zustand']
    return rc, r['cpu_s']


def placer_befehl(bj, kj, pj, rg, seed, seiten, gelernt=(), ausdruecklich=()):
    ""
    return [VPY, os.path.join(R, 'platzierung/placer.py'), bj, kj, pj, 'regeln=' + rg, 'modus=neu', 'zufall=1', 'seed=%d' % seed,
            'seiten=' + seiten, 'seiten_original=1'] + list(PLACER_STANDARD) + list(gelernt) + list(ausdruecklich)


def placer_lauf(cmd, zeit, log):
    ""
    from gemeinsam import plaetze as PL
    return PL.lauf(cmd, cpu_s=zeit, log=log, env=dict(os.environ, PYTHONHASHSEED='0'))


def regeln_messen(B, K, f, gndlagen=None, flaechennetze=None):
    ""
    if not f or not os.path.exists(f): return None
    try:
        from analyse import regelkennzahlen as RK
        from lernen.richter_paar import messsignatur
        RR = json.load(open(f))
        from gemeinsam.zonen import boardgeometrie
        B = boardgeometrie(B, RR)
        result = RK.messen(B, RR, K, gndlagen=gndlagen, flaechennetze=flaechennetze, minimax=False)
        result['paar_messversion'] = messsignatur()
        return result
    except Exception as e:
        return dict(fehler=dict(alles=repr(e)[:200]))


def gelernte_einstellungen(B, K, RG, L, placer_args=()):
    ""
    from lernen.merkmale import merkmale
    from lernen import einstellungen as EI
    fest = {}
    for a in placer_args:
        k_, _, v_ = a.partition('=')
        if k_ in EI.PLACER_DIM: fest[k_] = v_
    for a in ROUTER_EXTRA:
        k_, _, v_ = a.partition('=')
        if k_ == 'stil': fest['stil'] = v_
        elif k_ == 'winkel45': fest['stil'] = 'standard' if v_ == '1' else 'organisch'
    if VIA_F != 1.0: fest['via_f'] = VIA_F
    return EI.einstellungen(merkmale(B, K, RG), L, pfad=LENKUNG or EI.PFAD, vorgabe='pruefstand', fest=fest)


def placer_optionen_gelernt(EG):
    if not EG: return []
    from lernen import einstellungen as EI
    return EI.placer_optionen(EG)


def einstellungen_kurz(EG):
    from lernen import einstellungen as EI
    return EI.kurz(EG)


def ein_board(d, out, it, zeit, drc=True, auto=False, seed=1, placer_args=(), original_ref=None):
    name = os.path.basename(d.rstrip('/')); o = os.path.join(out, name); os.makedirs(o, exist_ok=True)
    pcb = os.path.join(d, 'processed.kicad_pcb')
    if not os.path.exists(pcb): pcb = sorted(glob.glob(os.path.join(d, '*.kicad_pcb')))[0]
    res = dict(board=name); t0 = time.time()
    try:
        for cmd in ([KPY, os.path.join(R, 'kicad/export_board.py'), pcb, os.path.join(o, 'board.json')],
                    [KPY, os.path.join(R, 'kicad/routes_export.py'), pcb, os.path.join(o, 'mensch.json')]):
            subprocess.run(cmd, capture_output=True, text=True, timeout=300)
        B = json.load(open(os.path.join(o, 'board.json'))); L = B.get('lagen') or 2
        RG = regeln_aus(pcb, L, B, sigmin_einheitlich=SIGMIN_EINHEITLICH); json.dump(RG, open(os.path.join(o, 'regeln.json'), 'w'))
        vj = os.path.join(o, 'vorgaben.json')
        p26 = DICHTE or original_ref or any(x.startswith('dicht=') and float(x.split('=',1)[1]) for x in placer_args)
        vorgaben = json.load(open(os.path.join(d, 'vorgaben.json'))) if p26 and os.path.exists(os.path.join(d, 'vorgaben.json')) else {}
        json.dump(dict(vorgaben, breite=RG.get('breite') or {}), open(vj, 'w'))
        K = klassen_lauf(o, os.path.join(o, 'board.json'), os.path.join(o, 'klassen.json'), vj)
        EG = gelernte_einstellungen(B, K, RG, L, placer_args) if EINSTELLUNGEN == 'gelernt' else None
        stil = EG['werte']['stil'] if EG and EG['herkunft']['stil'] == 'gelernt' else None
        via_f = EG['werte']['via_f'] if EG and EG['herkunft']['via_f'] == 'gelernt' else None
        pl_gelernt = placer_optionen_gelernt(EG)
        ST = dict(stil=stil) if stil else {}
        if EG:
            res['einstellungen'] = dict(klasse=EG['klasse'], werte=EG['werte'], herkunft=EG['herkunft'], grund=EG['grund'],
                                        lenkung=EG['lenkung'], lenkung_zeit=EG['lenkung_zeit'], n_min=EG['n_min'])
            print('%s einstellungen %s' % (name[:40], einstellungen_kurz(EG)), flush=True)
        ref = os.path.join(original_ref, 'ergebnis.json') if original_ref else (os.path.join(FEST_VON, name, 'ergebnis.json') if FEST_VON else '')
        uebernommen = bool(ref) and os.path.exists(ref)
        if uebernommen:
            R0 = json.load(open(ref))
            for k_ in ('router_rc', 'route_s', 'mensch', 'unser'): res[k_] = R0.get(k_)
            refordner = os.path.dirname(ref)
            res['fest_von'] = original_ref or FEST_VON
            if original_ref:
                res['fest_original_nur_bezug'] = True
                if os.path.exists(os.path.join(refordner, 'board.json')):
                    shutil.copy(os.path.join(refordner, 'board.json'), os.path.join(o, 'board_original.json'))
            for f_ in ('mensch.kicad_pcb.drc.json', 'unser.kicad_pcb.drc.json', 'unser.json'):
                if os.path.exists(os.path.join(refordner, f_)): shutil.copy(os.path.join(refordner, f_), os.path.join(o, f_))
        else:
            res['router_rc'], res['route_s'] = route_lauf(o, os.path.join(o, 'board.json'), os.path.join(o, 'klassen.json'),
                                                          os.path.join(o, 'unser.json'), it, zeit, os.path.join(o, 'route.log'), **ST)
            for wer in ('mensch', 'unser'):
                res[wer] = bewerte_wer(B, K, os.path.join(o, wer + '.json'))
        for wer in ('mensch', 'unser'):
            if res.get(wer) and not res[wer].get('regeln'):
                res[wer]['regeln'] = regeln_messen(B, K, os.path.join(o, wer + '.json'))
        if p26:
            D0 = ((res.get('mensch') or {}).get('dichte') if original_ref else None) or DM.messen(B, RG)
            for wer in ('mensch', 'unser'):
                if res.get(wer): res[wer]['dichte'] = dict(D0)
        res['netze'] = sum(1 for n in set(p['net'] for p in B['pads'] if p['net']) if sum(1 for p in B['pads'] if p['net'] == n) > 1)
        res['pads'] = len(B['pads']); res['lagen'] = L; res['teile'] = len(B['fps'])
        if drc and not uebernommen:
            via = RG['router']['via']
            if res['mensch']: res['mensch']['drc'] = drc_pruef(pcb, None, os.path.join(o, 'mensch.kicad_pcb'), via, L)
            if res['unser']: res['unser']['drc'] = drc_pruef(pcb, os.path.join(o, 'unser.json'), os.path.join(o, 'unser.kicad_pcb'), via, L)
        if auto:
            ta = time.time()
            json.dump(verwuerfeln(B, seed), open(os.path.join(o, 'board_wuerfel.json'), 'w'))

            seiten = ''.join(sorted({f['side'] for f in B['fps'] if f.get('n_pads')}, reverse=True)) or 'F'
            res['seiten'] = seiten
            klassen_lauf(o, os.path.join(o, 'board_wuerfel.json'), os.path.join(o, 'klassen_wuerfel.json'), vj)

            kands = []; erzeugt = []
            for k in range(KANDIDATEN):
                tag = '' if k == 0 else '_k%d' % k
                pj = os.path.join(o, 'auto_placement%s.json' % tag); bjk = os.path.join(o, 'auto_board%s.json' % tag)
                for alt in (pj, bjk):
                    if os.path.exists(alt): os.remove(alt)
                pl = placer_lauf(placer_befehl(os.path.join(o, 'board.json'), os.path.join(o, 'klassen_wuerfel.json'), pj,
                                               os.path.join(o, 'regeln.json'), seed + 1000 * k, seiten, pl_gelernt, placer_args),
                                 zeit, os.path.join(o, 'auto_placer%s.log' % tag))
                res.setdefault('auto_placer', []).append(dict(kandidat=tag or 'k0', zustand=pl['zustand'], cpu_s=pl['cpu_s'], wand_s=pl['wand_s'], warte_s=pl['warte_s']))
                if pl['zustand'] != 'ok' or not os.path.exists(pj): continue
                apk = os.path.join(o, 'auto%s.kicad_pcb' % tag)
                subprocess.run([KPY, os.path.join(R, 'platzierung/apply_place.py'), pcb, pj, apk],
                               capture_output=True, text=True, timeout=300)
                subprocess.run([KPY, os.path.join(R, 'kicad/export_board.py'), apk, bjk, 'placement=' + pj],
                               capture_output=True, text=True, timeout=300)
                if not os.path.exists(bjk): continue
                erzeugt.append(tag)
                if KANDIDATEN > 1:
                    from lernen.merkmale import merkmale
                    BK = json.load(open(bjk))
                    KK = klassen_lauf(o, bjk, os.path.join(o, 'auto_klassen%s.json' % tag), vj, via_f=via_f)
                    kands.append(dict(tag=tag, merkmale=merkmale(BK, KK, RG), kandidat=k))
            fest = [r for r, t in json.load(open(os.path.join(o, 'klassen_wuerfel.json')))['teile'].items() if t['fest']]
            order = [k['tag'] for k in rangfolge(kands)] if KANDIDATEN > 1 else erzeugt
            if KANDIDATEN > 1: res['auto_kandidaten'] = [dict(k, rang=order.index(k['tag'])) for k in kands]
            res['auto_place_s'] = round(time.time() - ta, 1)
            versuche = []
            for tag in order[:max(1, ROUTEN)]:
                ap = os.path.join(o, 'auto%s.kicad_pcb' % tag); bj = os.path.join(o, 'auto_board%s.json' % tag)
                kj = os.path.join(o, 'auto_klassen%s.json' % tag); rj = os.path.join(o, 'auto%s.json' % tag)
                BA = json.load(open(bj)); KA = klassen_lauf(o, bj, kj, vj, via_f=via_f)
                rc, ts = route_lauf(o, bj, kj, rj, it, zeit, os.path.join(o, 'auto_route%s.log' % tag), **ST)
                E = bewerte_wer(BA, KA, rj); BE = BA
                if REPARATUR and E and not E['gueltig']:

                    rep = os.path.join(o, 'rep%s' % tag); os.makedirs(rep, exist_ok=True); tr_ = time.time()
                    for a_, b_ in ((bj, 'board.json'), (rj, 'routes.json'), (kj, 'klassen.json'), (os.path.join(o, 'regeln.json'), 'regeln.json')):
                        shutil.copy(a_, os.path.join(rep, b_))
                    try:
                        subprocess.run([VPY, os.path.join(R, 'nachbearbeitung/nachbessern.py'), rep, 'runden=%d' % REPARATUR, 'parallel=3', 'ir=0'],
                                       capture_output=True, text=True, timeout=zeit, env=dict(os.environ, PYTHONHASHSEED='0'))
                    except subprocess.TimeoutExpired:
                        pass
                    rn = os.path.join(rep, 'routes_nach.json')
                    if os.path.exists(rn):
                        BN = json.load(open(os.path.join(rep, 'board_nach.json'))); EN = bewerte_wer(BN, KA, rn)
                        if EN and (EN['gueltig'] or EN['konflikte'] + EN['offen'] < E['konflikte'] + E['offen']):
                            ap2 = os.path.join(o, 'auto_rep%s.kicad_pcb' % tag)
                            subprocess.run([KPY, os.path.join(R, 'platzierung/apply_place.py'), ap, os.path.join(rep, 'placement_nach.json'), ap2],
                                           capture_output=True, text=True, timeout=300)
                            if os.path.exists(ap2):
                                shutil.copy(rn, rj); ap = ap2; EN['repariert'] = True; E = EN; BE = BN
                                if p26: json.dump(BN, open(bj, 'w'))
                    res['auto_rep_s'] = round(res.get('auto_rep_s', 0) + time.time() - tr_, 1)
                if E:
                    E['router_rc'] = rc; E['route_s'] = ts; E['kandidat'] = tag or 'k0'
                    E['regeln'] = regeln_messen(BE, KA, rj)
                    if p26:
                        E['dichte'] = DM.gegen(BE, D0, RG)
                        E['dichte_board'] = os.path.basename(bj)
                    if drc:
                        E['drc'] = drc_pruef(ap, rj, os.path.join(o, 'auto_fertig%s.kicad_pcb' % tag), RG['router']['via'], L, fest)
                        E['drc']['platz_netto'] = platz_netto(E['drc'], (res.get('mensch') or {}).get('drc'))
                versuche.append((tag, E))
                if E and gueltig_voll(E, True) and len(versuche) >= 1 and ROUTEN_NUR_BEI_FEHLER: break
            ok = [(t_, e_) for t_, e_ in versuche if e_]
            if ok:
                tag, E = ok[0]
                for tk, ek in ok[1:]:
                    if kandidat_besser(E, ek): tag, E = tk, ek
                res['auto'] = E; res['auto_geroutet'] = [t_ or 'k0' for t_, _ in versuche]
                if tag:
                    for x in ('auto_board', 'auto_placement', 'auto_klassen'):
                        if os.path.exists(os.path.join(o, '%s%s.json' % (x, tag))): shutil.copy(os.path.join(o, '%s%s.json' % (x, tag)), os.path.join(o, x + '.json'))
                    shutil.copy(os.path.join(o, 'auto%s.json' % tag), os.path.join(o, 'auto.json'))
                    if os.path.exists(os.path.join(o, 'auto_fertig%s.kicad_pcb.drc.json' % tag)):
                        shutil.copy(os.path.join(o, 'auto_fertig%s.kicad_pcb.drc.json' % tag), os.path.join(o, 'auto_fertig.kicad_pcb.drc.json'))
            else:
                res['auto'] = None
    except Exception as e:
        import traceback; res['fehler'] = repr(e)[:300]; open(os.path.join(o, 'fehler.txt'), 'w').write(traceback.format_exc())
    try:
        from lernen.ki.daten import pruefstand_aufnehmen
        res['_etiketten_seed'] = seed
        res['_etiketten_auto'] = [e for _, e in versuche if e] if auto and 'versuche' in locals() else []
        res['datensatz'] = pruefstand_aufnehmen(o, res)
    except Exception as _e: res['datensatz'] = [dict(zustand='fehler', fehler=repr(_e)[:200])]
    finally:
        res.pop('_etiketten_seed', None); res.pop('_etiketten_auto', None)
    res['gesamt_s'] = round(time.time() - t0, 1)
    json.dump(res, open(os.path.join(o, 'ergebnis.json'), 'w'), indent=1)
    return res


def gueltig_voll(x, platz=False, nativ=False):
    ""
    if not x or not x.get('gueltig'): return False
    d = x.get('drc')
    if d is None: return not DRC_PFLICHT
    if 'fehler' in d: return False
    if nativ and (d.get('version') != 'korpus-drc-v1' or not all(type(d.get(k)) is int and d[k] >= 0 for k in ('alle', 'platz', 'routing', 'unverbunden', 'zonen_kupfer'))): return False
    return d['routing'] == 0 and d['unverbunden'] == 0 and (not nativ or d.get('zonen_kupfer') == 0) and (not platz or d.get('platz_netto', d['platz']) == 0)


def kosten(x): return x['laenge_mm'] + 3.0 * x['vias']


def note_v3(x):
    ""
    v = ((x or {}).get('regeln') or {}).get('note_v3')
    return float(v) if isinstance(v, (int, float)) else None


def kandidat_besser(a, b):
    ""
    from analyse import regelkennzahlen as RK
    ga, gb = gueltig_voll(a, True), gueltig_voll(b, True)
    if ga != gb:
        if not any((e or {}).get('tor') or ((e or {}).get('regeln') or {}).get('messversion') for e in (a,b)):return gb
        from analyse import vertrag as VT
        tor=VT.vergleich(dict(a,geometrisch_gueltig=ga,regeln=a.get('regeln') or {}),
                         dict(b,geometrisch_gueltig=gb,regeln=b.get('regeln') or {}))
        return gb and tor['nicht_schlechter']
    decision = None
    if ga and gb:
        from lernen.richter_paar import Dienst
        za = dict(a, gueltig=True, regel=a.get('regeln') or {})
        zb = dict(b, gueltig=True, regel=b.get('regeln') or {})
        judge = Dienst(modell=PAAR_MODELL) if PAAR_MODELL else Dienst()
        try:
            judge.laden()
            if judge.anwendbar(za) and judge.anwendbar(zb):
                decision = judge.vergleich(za,zb)
                if not decision['nicht_schlechter']: return False
        except (OSError,ValueError,KeyError): pass
    if RK.annahme(a.get('regeln'), b.get('regeln'))['besser']: return True
    if decision is not None and decision['paar']: return decision['besser']
    return False


def wahl_schluessel(E):
    ""
    v = note_v3(E)
    return (not gueltig_voll(E, True), v is None, v if v is not None else 0.0, kosten(E))


def board_wert(r, wer, platz=False):
    ""
    x, m = r.get(wer), r.get('mensch')
    if not gueltig_voll(x, platz): return 2.0
    if not m: return 1.0
    return round(min(2.0, kosten(x) / max(1e-6, kosten(m))), 4)


def regel_wert(r, wer, platz=False):
    ""
    x = r.get(wer)
    if not gueltig_voll(x, platz): return 2.0
    v = note_v3(x)
    if v is None: return 2.0
    m = note_v3(r.get('mensch'))
    if m is None: return 1.0
    return round(min(2.0, v / max(1.0, m)), 4)


def laden(liste, ziel):
    ""
    import urllib.request, urllib.parse
    L = json.load(open(liste)); fehlt = []
    def eins(b):
        d = os.path.join(ziel, b['name']); os.makedirs(d, exist_ok=True); n = 0
        for f in L['dateien']:
            url = 'https://raw.githubusercontent.com/PCBench/PCBench/%s/PCBs/%s/%s' % (L['sha'], urllib.parse.quote(b['pcbench']), f)
            try:
                data = urllib.request.urlopen(url, timeout=60).read(); open(os.path.join(d, f), 'wb').write(data); n += len(data)
            except Exception as e:
                fehlt.append((b['name'], f, repr(e)[:60]))
        return n
    with ThreadPoolExecutor(8) as ex: n = sum(ex.map(eins, L['boards']))
    print('geladen: %d Boards, %.1f MB nach %s, %d Dateien fehlen' % (len(L['boards']), n / 1e6, ziel, len(fehlt)))
    for x in fehlt[:10]: print('  fehlt', x)


REGEL_DATEIEN = dict(mensch=('board.json', 'klassen.json', 'mensch.json'), unser=('board.json', 'klassen.json', 'unser.json'),
                     auto=('auto_board.json', 'auto_klassen.json', 'auto.json'))


def regeln_nachzaehlen(r, o):
    ""
    for wer, (bj, kj, rj) in REGEL_DATEIEN.items():
        if r.get('fest_original_nur_bezug') and wer in ('mensch', 'unser'): continue
        x = r.get(wer)
        if not x or x.get('repariert') or not all(os.path.exists(os.path.join(o, f)) for f in (bj, kj, rj)): continue
        with open(os.path.join(o, bj)) as f: B = json.load(f)
        with open(os.path.join(o, kj)) as f: K = json.load(f)
        x['regeln'] = regeln_messen(B, K, os.path.join(o, rj))


def dichte_nachzaehlen(r, o):
    ""
    bj = os.path.join(o, 'board.json'); rg = os.path.join(o, 'regeln.json')
    if not os.path.exists(bj): return
    B = json.load(open(bj)); RG = json.load(open(rg)) if os.path.exists(rg) else None
    D0 = ((r.get('mensch') or {}).get('dichte') if r.get('fest_original_nur_bezug') else None) or DM.messen(B, RG)
    if r.get('fest_original_nur_bezug'):


        sources = [os.path.join(o, 'board_original.json'), os.path.join(r.get('fest_von') or o, 'board.json')]
        for source in sources:
            if source == bj: continue
            if os.path.exists(source):
                D0 = DM.messen(json.load(open(source)), RG); break
    for wer in ('mensch', 'unser'):
        if r.get(wer): r[wer]['dichte'] = dict(D0)
    ap = os.path.join(o, 'auto_board.json')
    if (r.get('auto') or {}).get('dichte_board'):
        ap = os.path.join(o, r['auto']['dichte_board'])
    elif (r.get('auto') or {}).get('repariert'):
        tag = r['auto'].get('kandidat', 'k0')

        end = os.path.join(o, 'rep' + ('' if tag == 'k0' else tag), 'board_nach.json')
        if os.path.exists(end): ap = end
        else: return
    if r.get('auto') and os.path.exists(ap):
        r['auto']['dichte'] = DM.gegen(json.load(open(ap)), D0, RG)


def enge_aufgabe(d, out, it, zeit, seed, pa, drc):
    ""
    from lernen import sammeln as S
    name = os.path.basename(d); o = os.path.join(out, name)
    try:
        q, faktor = S.variante_eng(d, o, random.Random(zlib.crc32((name + ':' + str(seed)).encode())))
        if q is None: return dict(board=name, eng_ausgelassen='kein freier Rand')
        r = ein_board(q, os.path.join(out, 'eng'), it, zeit, drc, True, seed, pa, original_ref=o)
        r['variante'] = dict(art='eng', von=name, faktor=faktor)
        json.dump(r, open(os.path.join(out, 'eng', r['board'], 'ergebnis.json'), 'w'), indent=1)
        return r
    except Exception as e:
        return dict(board=name, eng_fehler=repr(e)[:300])
    finally:

        shutil.rmtree(os.path.join(out, '_quellen', name + '_eng'), ignore_errors=True)


def auswerten(out, regeln=True):
    ""
    res = []
    for f in sorted(glob.glob(os.path.join(out, '*', 'ergebnis.json'))):
        r = json.load(open(f)); o = os.path.dirname(f)
        for wer, datei, fj in (('mensch', 'mensch.kicad_pcb.drc.json', None), ('unser', 'unser.kicad_pcb.drc.json', None),
                               ('auto', 'auto_fertig.kicad_pcb.drc.json', 'klassen_wuerfel.json')):
            if r.get(wer) and os.path.exists(os.path.join(o, datei)):
                fest = [x for x, t in json.load(open(os.path.join(o, fj)))['teile'].items() if t['fest']] if fj else None
                r[wer]['drc'] = drc_auswerten(json.load(open(os.path.join(o, datei))), fest)
                if wer == 'auto': r[wer]['drc']['platz_netto'] = platz_netto(r[wer]['drc'], (r.get('mensch') or {}).get('drc'))
        if regeln: regeln_nachzaehlen(r, o)
        if DICHTE or any((r.get(w) or {}).get('dichte') for w in ('mensch','unser','auto')):
            dichte_nachzaehlen(r, o)
        json.dump(r, open(f, 'w'), indent=1); res.append(r)
    return res


def main():
    if sys.argv[1] == 'laden': return laden(sys.argv[2], sys.argv[3])
    if sys.argv[1] == 'tabelle':
        res = [json.load(open(f)) for f in sorted(glob.glob(os.path.join(sys.argv[2], '*', 'ergebnis.json')))]
        tabelle(res); print(json.dumps(zusammenfassung(res, any(r.get('auto') for r in res)))); return
    if sys.argv[1] == 'auswerten':
        kw = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a)
        res = auswerten(sys.argv[2], regeln=kw.get('regeln', '1') == '1'); z = zusammenfassung(res, any(r.get('auto') for r in res))
        tabelle(res)
        json.dump(z, open(os.path.join(sys.argv[2], 'pruefstand.json'), 'w'), indent=1); print(json.dumps(z, indent=1)); return
    if sys.argv[1] == 'vergleich':
        kw = dict(a.split('=', 1) for a in sys.argv[4:] if '=' in a)
        A, Bv = (lauf_laden(x) for x in sys.argv[2:4])
        T = tor(A, Bv, float(kw['tol']) if kw.get('tol') else None)
        print('\n'.join(tor_text(T, sys.argv[2], sys.argv[3])))
        if kw.get('json'): json.dump(T, open(kw['json'], 'w'), indent=1)
        return
    korpus, out = sys.argv[1], sys.argv[2]
    kw = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a)
    nur = set(kw['nur'].split(',')) if kw.get('nur') else None
    global VIA_F, KANDIDATEN, AUSWAHL; VIA_F = float(kw.get('via_faktor', '1'))
    KANDIDATEN = int(kw.get('kandidaten', '1')); AUSWAHL = kw.get('auswahl', 'faust')
    global PAAR_MODELL; PAAR_MODELL = kw.get('paar_modell')
    global FEST_VON, REPARATUR, ROUTEN, ROUTEN_NUR_BEI_FEHLER; FEST_VON = kw.get('fest_von', ''); REPARATUR = int(kw.get('reparatur', '0'))
    global EINSTELLUNGEN, LENKUNG; EINSTELLUNGEN = kw.get('einstellungen', ''); LENKUNG = kw.get('lenkung', '')
    if EINSTELLUNGEN not in ('', 'aus', 'gelernt'): sys.exit('einstellungen=%s unbekannt (gelernt oder aus)' % EINSTELLUNGEN)
    global ROUTER_EXTRA; ROUTER_EXTRA = tuple(x for x in kw.get('router', '').split(',') if x)
    global SIGMIN_EINHEITLICH; SIGMIN_EINHEITLICH = kw.get('sigmin_einheitlich', '0') == '1'
    ROUTEN = int(kw.get('routen', '1')); ROUTEN_NUR_BEI_FEHLER = kw.get('routen_sparsam', '0') == '1'
    auto = kw.get('auto', '0') == '1'; drc = kw.get('drc', '1') == '1'
    global DRC_PFLICHT; DRC_PFLICHT = drc
    pa = tuple(x for x in kw.get('placer', '').split(',') if x)
    global DICHTE; DICHTE = kw.get('eng') == '1' or kw.get('dichte_messen') == '1'
    dirs = sorted(d for d in glob.glob(os.path.join(korpus, '*')) if os.path.isdir(d) and (not nur or os.path.basename(d) in nur))
    os.makedirs(out, exist_ok=True)
    json.dump(dict(boards=[os.path.basename(d) for d in dirs], teile=['unser', 'auto'] if auto else ['unser']),
              open(os.path.join(out, 'manifest.json'), 'w'), indent=1)
    with ThreadPoolExecutor(int(kw.get('parallel', '4'))) as ex:
        res = list(ex.map(lambda d: ein_board(d, out, int(kw.get('iter', '60')), int(kw.get('zeit', '600')), drc, auto,
                                              int(kw.get('seed', '1')), pa), dirs))
    json.dump(res, open(os.path.join(out, 'korpus_ergebnis.json'), 'w'), indent=1)
    tabelle(res)
    zus = zusammenfassung(res, auto)
    json.dump(zus, open(os.path.join(out, 'pruefstand.json'), 'w'), indent=1)
    print(json.dumps(zus, indent=1))
    if kw.get('eng', '0') == '1':
        enur = set(kw['eng_nur'].split(',')) if kw.get('eng_nur') else None
        edirs = [d for d in dirs if not enur or os.path.basename(d) in enur]
        with ThreadPoolExecutor(min(4, int(kw.get('parallel', '4')))) as ex:
            er = list(ex.map(lambda d: enge_aufgabe(d, out, int(kw.get('iter', '60')), int(kw.get('zeit', '600')),
                                                  int(kw.get('seed', '1')), pa, drc), edirs))
        eo = os.path.join(out, 'eng'); os.makedirs(eo, exist_ok=True)
        json.dump(er, open(os.path.join(eo, 'korpus_ergebnis.json'), 'w'), indent=1)
        json.dump(dict(aufgaben=len(er), erzeugt=sum('variante' in r for r in er),
                       auto_gueltig=sum(gueltig_voll(r.get('auto'), True) for r in er),
                       regeln=regel_block(er, 'auto', True), dichte=dichte_block(er, 'auto', True)),
                  open(os.path.join(eo, 'pruefstand.json'), 'w'), indent=1)


def tabelle(res):
    def f(x, platz=False):
        if not x: return '-'
        dd = x.get('drc') or {}; rg = x.get('regeln') or {}
        v = rg.get('note_v3')
        return '%s %4d %6.0f d%s v3 %s T%s' % ('ja  ' if gueltig_voll(x, platz) else 'NEIN', x['vias'], x['laenge_mm'],
                                              '%d/%d' % (dd.get('routing', -1), dd.get('unverbunden', -1)) + ('/p%d' % dd.get('platz_netto', dd.get('platz', -1)) if platz else ''),
                                              '-' if v is None else '%.1f' % v, '-' if rg.get('topologie') is None else '%g' % rg['topologie'])
    print('%-34s %5s | %-40s | %-40s | %-44s' % ('Board', 'Netze', 'Mensch ok/Vias/mm dDRC v3 Topologie', 'Unser (fest)', 'Unser (selbststaendig)'))
    for r in res:
        print('%-34s %5s | %-40s | %-40s | %-44s' % (r['board'][:34], r.get('netze'), f(r.get('mensch')), f(r.get('unser')), f(r.get('auto'), True)))


TEILE = dict(fest=('unser', False), auto=('auto', True))


def regel_block(res, wer, platz):
    ""
    from analyse import regelkennzahlen as RK
    n = len(res); ok = [r for r in res if gueltig_voll(r.get(wer), platz)]
    mit = [r for r in ok if (r[wer].get('regeln') or {}).get('note_v3') is not None]
    mm = [r for r in mit if note_v3(r.get('mensch')) is not None]
    S = RK.summen([r[wer]['regeln'] for r in mit]); SM = RK.summen([r['mensch']['regeln'] for r in mm])
    SW = RK.summen([r[wer]['regeln'] for r in mm])
    rel = {k: (round(SW[k] / SM[k], 3) if SW.get(k) is not None and SM.get(k) else None) for k in RK.TOR + RK.NEBEN}
    return dict(wert=round(sum(regel_wert(r, wer, platz) for r in res) / max(1, n), 4), gueltig_gemessen=len(mit),
                ohne_messung=len(ok) - len(mit), summen=S, mit_mensch=len(mm), summen_mensch=SM, summen_wir_mit_mensch=SW, rel_mensch=rel)


def dichte_block(res, wer, platz=False):
    rows = [r[wer]['dichte'] for r in res if r.get(wer) and r[wer].get('dichte')]
    valid = [r[wer]['dichte'] for r in res if gueltig_voll(r.get(wer), platz) and r[wer].get('dichte')]
    def block(rows):
        return dict(gemessen=len(rows), summen={k: round(sum(x[k] for x in rows), 6) for k in ('platine_mm2', 'belegt_mm2', 'huelle_mm2')},
                    mittel={k: round(sum(x[k] for x in rows if x.get(k) is not None) / max(1, sum(x.get(k) is not None for x in rows)), 6)
                            if any(x.get(k) is not None for x in rows) else None
                            for k in ('dichte', 'dichte_rel_original', 'platine_mm2_rel_original', 'huelle_mm2_rel_original')})
    return dict(alle=block(rows), gueltige=block(valid))


def zusammenfassung(res, auto):
    n = len(res); z = dict(boards=n)
    z['mensch_gueltig'] = sum(gueltig_voll(r.get('mensch')) for r in res)
    z['fest_gueltig'] = sum(gueltig_voll(r.get('unser')) for r in res)
    z['fest_wert'] = round(sum(board_wert(r, 'unser') for r in res) / max(1, n), 4)
    ok = [r for r in res if gueltig_voll(r.get('unser')) and r.get('mensch')]
    if ok:
        z['fest_vias_rel'] = round(sum(r['unser']['vias'] for r in ok) / max(1, sum(r['mensch']['vias'] for r in ok)), 3)
        z['fest_laenge_rel'] = round(sum(r['unser']['laenge_mm'] for r in ok) / max(1e-6, sum(r['mensch']['laenge_mm'] for r in ok)), 3)
    z['fest_zeit_median_s'] = sorted(r.get('route_s') or 0 for r in res)[n // 2] if n else 0
    if auto:
        z['auto_gueltig'] = sum(gueltig_voll(r.get('auto'), True) for r in res)
        z['auto_wert'] = round(sum(board_wert(r, 'auto', True) for r in res) / max(1, n), 4)
        oka = [r for r in res if gueltig_voll(r.get('auto'), True) and r.get('mensch')]
        if oka:
            z['auto_vias_rel'] = round(sum(r['auto']['vias'] for r in oka) / max(1, sum(r['mensch']['vias'] for r in oka)), 3)
            z['auto_laenge_rel'] = round(sum(r['auto']['laenge_mm'] for r in oka) / max(1e-6, sum(r['mensch']['laenge_mm'] for r in oka)), 3)
        z['gesamt'] = round(z['fest_wert'] + z['auto_wert'], 4)

    z['regeln'] = {t: regel_block(res, wer, platz) for t, (wer, platz) in TEILE.items() if t == 'fest' or auto}
    z['fest_regel_wert'] = z['regeln']['fest']['wert']
    if auto:
        z['auto_regel_wert'] = z['regeln']['auto']['wert']
        z['regel_gesamt'] = round(z['fest_regel_wert'] + z['auto_regel_wert'], 4)
    if DICHTE or any((r.get(w) or {}).get('dichte') for r in res for w in ('mensch','unser','auto')):
        z['dichte'] = {t: dichte_block(res, wer, platz) for t, (wer, platz) in TEILE.items() if t == 'fest' or auto}


    z['rang'] = [-(z['fest_gueltig'] + z.get('auto_gueltig', 0)), z.get('regel_gesamt', z['fest_regel_wert']), z.get('gesamt', z['fest_wert'])]
    return z


def lauf_laden(o):
    ""
    daten = {json.load(open(f))['board']: json.load(open(f)) for f in sorted(glob.glob(os.path.join(o, '*', 'ergebnis.json')))}
    m = os.path.join(o, 'manifest.json')
    if os.path.exists(m):
        manifest = json.load(open(m))
        for n in manifest['boards']:
            daten.setdefault(n, dict(board=n, _fehlt=True))
            daten[n]['_teile_erwartet'] = manifest['teile']
    return daten


def tor(A, Bv, tol_rel=None):
    ""
    from analyse import regelkennzahlen as RK
    namen = sorted(set(A) & set(Bv))
    a = [A[n] for n in namen]; b = [Bv[n] for n in namen]
    auto = any(r.get('auto') for r in a + b)
    out = dict(boards=len(namen), nur_bezug=sorted(set(A) - set(Bv)), nur_variante=sorted(set(Bv) - set(A)), teile={})
    fehler = []
    if set(A) != set(Bv) or not namen: fehler.append('Boardmenge verschieden oder leer')
    erwartet = {'unser', 'auto'} if auto else {'unser'}
    for seite, daten in (('bezug', A), ('neu', Bv)):
        for n, r in sorted(daten.items()):
            for wer in erwartet | set(r.get('_teile_erwartet') or []):
                if not r.get(wer): fehler.append('%s %s: Ergebnis %s fehlt' % (seite, n, wer)); continue
                if gueltig_voll(r[wer], wer == 'auto'):
                    for k in RK.torfelder(r[wer].get('regeln')):
                        if RK.wert(r[wer].get('regeln'), k) is None: fehler.append('%s %s %s: %s fehlt/nicht endlich' % (seite, n, wer, k))
    ga = gb = 0; schlecht = []; gut = []
    for t, (wer, platz) in TEILE.items():
        if t == 'auto' and not auto: continue
        va = [n for n in namen if gueltig_voll(A[n].get(wer), platz)]; vb = [n for n in namen if gueltig_voll(Bv[n].get(wer), platz)]
        ga += len(va); gb += len(vb)
        beide = sorted(set(va) & set(vb))
        V = vergleich_geo(RK, {n: A[n][wer].get('regeln') for n in beide}, {n: Bv[n][wer].get('regeln') for n in beide}, tol_rel)
        wa = round(sum(board_wert(A[n], wer, platz) for n in namen) / max(1, len(namen)), 4)
        wb = round(sum(board_wert(Bv[n], wer, platz) for n in namen) / max(1, len(namen)), 4)
        ra = round(sum(regel_wert(A[n], wer, platz) for n in namen) / max(1, len(namen)), 4)
        rb = round(sum(regel_wert(Bv[n], wer, platz) for n in namen) / max(1, len(namen)), 4)
        neben = {}
        for k_, f_ in (('laenge_mm', lambda x: x['laenge_mm']), ('vias', lambda x: x['vias'])):
            sa = sum(f_(A[n][wer]) for n in beide); sb = sum(f_(Bv[n][wer]) for n in beide)
            neben[k_] = dict(bezug=round(sa, 1), neu=round(sb, 1), rel=round((sb - sa) / sa, 4) if sa else None)
        out['teile'][t] = dict(gueltig_bezug=len(va), gueltig_neu=len(vb), verloren=sorted(set(va) - set(vb)), gewonnen=sorted(set(vb) - set(va)),
                               beide=len(beide), regeln=V, regel_wert=dict(bezug=ra, neu=rb), alter_wert=dict(bezug=wa, neu=wb), neben=neben)
        for k, v in V['je'].items():
            if not v or not v['tor']: continue
            if v['urteil'] > 0: schlecht.append('%s %s' % (t, k))
            elif v['urteil'] < 0: gut.append('%s %s' % (t, k))
    out.update(gueltig_bezug=ga, gueltig_neu=gb, schlechter_in=schlecht, besser_in=gut)
    out['vollstaendig'] = not fehler; out['fehler'] = fehler
    if fehler: out['urteil'] = 'unvollstaendig'
    elif gb < ga or schlecht: out['urteil'] = 'schlechter'
    elif gb > ga or gut: out['urteil'] = 'besser'
    else: out['urteil'] = 'gleich'
    return out


def vergleich_geo(RK, A, Bv, tol_rel=None):
    ""
    return RK.vergleich(A, Bv, RK.TOL_REL if tol_rel is None else tol_rel)


def tor_text(T, na='Bezug', nb='Variante'):
    o = ['Tor (H6): %s -> %s, %d gemeinsame Boards: gueltig %d -> %d; Urteil: %s' % (os.path.basename(na.rstrip('/')), os.path.basename(nb.rstrip('/')),
                                                                                      T['boards'], T['gueltig_bezug'], T['gueltig_neu'], T['urteil'].upper())]
    if T.get('fehler'): o.append('  unvollstaendig: ' + '; '.join(T['fehler']))
    if T['schlechter_in']: o.append('  schlechter: ' + ', '.join(T['schlechter_in']))
    if T['besser_in']: o.append('  besser: ' + ', '.join(T['besser_in']))
    for t, x in T['teile'].items():
        o.append('%s: gueltig %d -> %d (verloren %s, gewonnen %s), in beiden gueltig %d; Regelwert %.4f -> %.4f, alter Wert %.4f -> %.4f' % (
            t, x['gueltig_bezug'], x['gueltig_neu'], ','.join(n[:4] for n in x['verloren']) or '-', ','.join(n[:4] for n in x['gewonnen']) or '-',
            x['beide'], x['regel_wert']['bezug'], x['regel_wert']['neu'], x['alter_wert']['bezug'], x['alter_wert']['neu']))
        o.append('  %-14s %4s %10s %10s %8s %7s  %s' % ('Groesse', 'n', 'Bezug', 'neu', 'rel', 'geo', 'besser/schlechter/gleich je Board, p, Urteil'))
        for k, v in x['regeln']['je'].items():
            if not v: continue
            o.append('  %-14s %4d %10.2f %10.2f %8s %7s  %d/%d/%d p %.2f/%.2f %s%s' % (k, v['n'], v['bezug'], v['neu'], '-' if v['rel'] is None else '%+.1f%%' % (100 * v['rel']),
                     '-' if 'geo' not in v else '%.3f' % v['geo'], v['besser'], v['schlechter'], v['gleich'], v['p_besser'], v['p_schlechter'],
                     {1: 'SCHLECHTER', -1: 'besser', 0: 'gleich'}[v['urteil']], '' if v['tor'] else ' (Nebenmass)'))
        for k, v in x['neben'].items():
            o.append('  %-14s %4d %10.1f %10.1f %8s          (Nebenmass)' % (k, x['beide'], v['bezug'], v['neu'], '-' if v['rel'] is None else '%+.1f%%' % (100 * v['rel'])))
    return o


if __name__ == '__main__':
    main()
