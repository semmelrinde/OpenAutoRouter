""
import glob, hashlib, json, os, shutil, subprocess, sys, threading, time, uuid
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

R = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); sys.path.insert(0, R)
from gemeinsam.prozessschutz import bootstrap
if __name__ == "__main__": bootstrap(heavy=False)
ORDNER = os.environ.get('LERNKURVE_ORDNER') or os.path.expanduser('~/Library/Caches/Autorouter/lernkurve')
DATEN = os.environ.get('LERNKURVE_DATEN') or os.path.join(R, 'lernen', 'daten', 'lernkurve.jsonl')
HAUPT = os.environ.get('LERNKURVE_HAUPT') or R
STUFEN = (0, 2, 4, 6, 8); SEEDS = (900001, 900002, 900003)

SAETZE = dict(lernkurve=(SEEDS, 'aufgaben', DATEN),
              abnahme=((950001, 950002, 950003, 950004), 'abnahme', os.path.join(HAUPT, 'lernen', 'daten', 'abnahme.jsonl')))
ABNAHME_TAGE = 7
REF = dict(kreuz=2, trenn=2, via_kosten=0.6, fein='1', pfmul=1.25, form='45r', seed=1)


def aufgaben(satz='lernkurve'):
    ""
    from lernen import aufgaben as AU
    out = []; seeds, unter, _ = SAETZE[satz]
    for st in STUFEN:
        for s in seeds:
            d = os.path.join(ORDNER, unter, 'v' + AU.VERSION, 's%d_%d' % (st, s))
            if not os.path.exists(os.path.join(d, 'aufgabe.json')): AU.erzeugen(d, s * 10 + st, stufe=st)
            out.append(d)
    return out


def beste_cfg(zeilen, kl, ziel=None):
    ""
    from lernen import selbstspiel as SS
    from lernen import aufgaben as AU
    zeilen = [z for z in zeilen if str(z.get('version', '')).split('+')[0] == AU.VERSION]
    zeilen = [x for x in zeilen if x.get('zustand') != 'code_geaendert' and not x.get('nicht_geroutet')]
    from analyse.standard import ziel as standardziel
    ziel = standardziel(explizit=ziel, kontext='lernkurve')
    zeilen = [z for z in zeilen if SS.bewertungsziel(z) == ziel]
    if ziel == 'minimax': zeilen = [z for z in zeilen if SS.belohnung(z) is not None]
    rew = [SS.belohnung(z) for z in zeilen] if ziel == 'minimax' else SS.belohnungen(zeilen)
    cfg = {}
    for k, werte in sorted(SS.ARME.items()):
        best, bm = werte[0], 9e9
        for w in werte:
            r = [b for x, b in zip(zeilen, rew) if SS.arm(x, k) == w and x.get('klasse') == kl] or \
                [b for x, b in zip(zeilen, rew) if SS.arm(x, k) == w]
            if len(r) >= 3 and sum(r) / len(r) < bm: best, bm = w, sum(r) / len(r)
        cfg[k] = best
    cfg['seed'] = 1
    return cfg


def mess_cfg(cfg, budget, arbeit_max, notbremse, router):
    ""
    if budget not in ('arbeit', 'cpu'): raise ValueError('budget=arbeit|cpu')
    if arbeit_max <= 0 or notbremse <= 0: raise ValueError('Arbeitsmenge/Notbremse muessen positiv sein')
    result = dict(cfg, datensatz='0', lerndaten='0', budget=budget)
    if budget == 'arbeit': result.update(arbeit_max=arbeit_max, arbeit_cpu_notbremse=notbremse)
    if router:
        from lernen.selbstspiel import ROUTER_VARIANTEN
        old = result.get('router', 'alt')
        result['router'] = list(ROUTER_VARIANTEN[old] if isinstance(old, str) else old) + list(router)
    return result


def lernzeilen(path):
    ""
    with open(path, 'rb') as f:
        data = f.read(os.fstat(f.fileno()).st_size)
    data = data[:data.rfind(b'\n') + 1]
    return [json.loads(line) for line in data.splitlines() if line.strip()]


def punkt_unvollstaendig(row):
    ""
    if row.get('unbekannt') or row.get('unbekannt_ref'): return True
    for result in row.get('ergebnisse') or []:
        for part in (result, result.get('ref') or {}):
            if 'gueltig' in part and part['gueltig'] is None: return True
    return False


def messvertrag(budget, zeit, it, arbeit_max, notbremse, router):
    ""
    return dict(budget=budget, arbeit_max=arbeit_max if budget == 'arbeit' else None,
                cpu_notbremse=notbremse if budget == 'arbeit' else None,
                zeit=zeit if budget == 'cpu' else None,
                cpu_skalierung='int(zeit * min(5.0, max(1.0, pads / 300.0)))' if budget == 'cpu' else None,
                legal_lp_budget='60 CPU-s' if budget == 'cpu' else 'arbeit: feste Knotenzahl; CPU-Notbremse',
                iter=it, router=list(router),gueltigkeit='modell_v3_mit_kosten_nativ-v2')


def messen(par=3, zeit=300, it=40, satz='lernkurve', ziel=None, budget='cpu',
           arbeit_max=59147, notbremse=60000, router=(), behalten=False, lenkung=None):
    from lernen import selbstspiel as SS
    from analyse.standard import ziel as standardziel
    cli_ziel = ziel
    ziel = standardziel(explizit=cli_ziel, kontext='lernkurve')
    p = os.path.join(HAUPT, 'lernen', 'daten', 'selbstspiel.jsonl')
    eingefrorene_wahl = json.loads(Path(lenkung).read_text()) if lenkung else None
    if eingefrorene_wahl and eingefrorene_wahl['bewertungsziel'] != ziel: raise ValueError('Lernwahl hat anderes Bewertungsziel')
    zeilen = lernzeilen(p) if not lenkung and os.path.exists(p) else []
    mess_cfg({}, budget, arbeit_max, notbremse, router)
    kopf = os.environ.get('LERNKURVE_KOPF') or subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=R, text=True).strip()
    lauf = time.strftime('%Y%m%d_%H%M%S') + '_' + satz + '_' + uuid.uuid4().hex[:8]; ergebnisse = []
    laufordner = os.path.join(ORDNER, 'laeufe', lauf)

    def eins(d):
        A = json.load(open(os.path.join(d, 'aufgabe.json')))
        w = os.path.join(ORDNER, 'laeufe', lauf, os.path.basename(d)); shutil.copytree(d, w)
        vgp = os.path.join(w, 'vorgaben.json'); vg = json.load(open(vgp))
        board_ziel = standardziel(vg, explizit=cli_ziel, kontext='lernkurve')
        vg['bewertung'] = board_ziel
        A['bewertungsziel'] = board_ziel
        if board_ziel == 'minimax':
            vg.setdefault('profil', 'ausgewogen'); A['profil'] = vg['profil']
        json.dump(vg, open(vgp, 'w'), indent=1)
        from analyse import korpus as KO
        KO.klassen_lauf(w, os.path.join(w, 'board.json'), os.path.join(w, 'klassen.json'), os.path.join(w, 'vorgaben.json'))
        out = {}



        gelernt = eingefrorene_wahl['cfg'][SS.klasse(A, board_ziel)] if eingefrorene_wahl else beste_cfg(zeilen, SS.klasse(A, board_ziel), board_ziel)
        if not eingefrorene_wahl:
            from lernen import meister as ME
            md = ME.lesen()
            _, me = ME.auswahl(md, ME.klasse(A), exakt=True) if md['klassen'] else (None, None)
            if me: gelernt = dict(REF)
        for art, cfg in (('ref', dict(REF, legal_lp=True)), ('gelernt', dict(gelernt, legal_lp=True))):
            cfg = mess_cfg(cfg, budget, arbeit_max, notbremse, router)
            wa = w + '_' + art
            if not os.path.exists(wa): shutil.copytree(w, wa)
            z = SS.versuch(wa, A, cfg, 0, zeit, it)
            json.dump(z, open(os.path.join(wa, 'messung.json'), 'w'), indent=1)
            cpu = z.get('cpu_total') or (((z.get('placer') or {}).get('cpu_s') or 0) + ((z.get('router') or {}).get('cpu_s') or 0))
            rp = Path(wa, 'k0', 'routes.json')
            route = json.loads(rp.read_text()) if rp.exists() else {}
            pp = Path(wa, 'k0', 'placement.json')
            placement_sha = hashlib.sha256(pp.read_bytes()).hexdigest() if pp.exists() else None
            kupfer_sha = hashlib.sha256(json.dumps({k: route.get(k) for k in ('tracks', 'vias', 'zones')}, sort_keys=True).encode()).hexdigest() if route else None
            out[art] = dict(regel=z.get('regel'), bewertungsziel=board_ziel, gueltig_routing=z.get('gueltig_routing', z['gueltig']), gueltig=z['gueltig'], note_v3=z.get('note_v3'), cpu_s=round(cpu, 1), vias=(z.get('kennzahlen') or {}).get('vias'),
                            restkonflikte=(z.get('kennzahlen') or {}).get('konflikte'), offen=(z.get('kennzahlen') or {}).get('offen'),
                            grund=z.get('ungueltig_grund'), zustand=z.get('zustand'), fehler=z.get('fehler'), cfg=cfg,
                            kupfer_sha=kupfer_sha, arbeit=route.get('arbeitsbudget'), placement_sha=placement_sha,
                            placer=z.get('placer'), placer_kennzahlen=z.get('placer_kennzahlen'),
                            meister=route.get('meister'),kosten=(z.get('cfg') or {}).get('kosten'),
                            native_meister=z.get('native_meister'))
        g = out['gelernt']
        return dict(generatorversion=A['version'], regel=g['regel'], bewertungsziel=board_ziel, gueltig_routing=g['gueltig_routing'], aufgabe=os.path.basename(d), stufe=A.get('stufe'), gueltig=g['gueltig'], note_v3=g['note_v3'], cpu_s=g['cpu_s'],
                    vias=g['vias'], restkonflikte=g.get('restkonflikte'), offen=g.get('offen'), grund=g['grund'], zustand=g['zustand'], fehler=g['fehler'], cfg=g['cfg'], kupfer_sha=g['kupfer_sha'], arbeit=g['arbeit'],
                    placement_sha=g['placement_sha'], placer=g['placer'], placer_kennzahlen=g['placer_kennzahlen'],
                    meister=g.get('meister'),kosten=g.get('kosten'),native_meister=g.get('native_meister'),ref=out['ref'])
    with ThreadPoolExecutor(par) as ex: ergebnisse = list(ex.map(eins, aufgaben(satz)))
    n_g = sum(e['gueltig'] is True for e in ergebnisse); cpu = sum(e['cpu_s'] for e in ergebnisse)
    h = hashlib.sha256()
    for d in aufgaben(satz):
        for f in ('board.json', 'regeln.json', 'vorgaben.json', 'aufgabe.json'):
            if os.path.exists(os.path.join(d, f)): h.update(open(os.path.join(d, f), 'rb').read())
    from lernen import aufgaben as AU
    zeile = dict(generatorversion=AU.VERSION, bewertungsziel=ziel, unbekannt=sum(e['gueltig'] is None for e in ergebnisse),
                 unbekannt_ref=sum(e['ref']['gueltig'] is None for e in ergebnisse), zeit=time.strftime('%Y-%m-%d %H:%M'), satz=satz, code=kopf, aufgaben=len(ergebnisse), gueltig=n_g, aufgaben_sha=h.hexdigest()[:16],
                 messvertrag=messvertrag(budget, zeit, it, arbeit_max, notbremse, router),
                 quell_sha={p: hashlib.sha256(Path(R, p).read_bytes()).hexdigest() for p in ('router/route_pf.py', 'analyse/klassen.py')},
                 lenkung_sha=hashlib.sha256(Path(lenkung).read_bytes()).hexdigest() if lenkung else hashlib.sha256(json.dumps(zeilen, sort_keys=True).encode()).hexdigest(),
                 lenkung_quelle=eingefrorene_wahl.get('quelle') if eingefrorene_wahl else p,
                 cpu_bilanz='Selbstspiel-Teilbilanz', laufordner=laufordner if behalten else None,
                 gueltig_ref=sum(1 for e in ergebnisse if e['ref']['gueltig']), last=round(os.getloadavg()[0], 1),
                 cpu_je_gueltig=round(cpu / n_g, 1) if n_g else None,
                 gueltig_je_stufe={str(st): sum(e['gueltig'] is True for e in ergebnisse if e['stufe'] == st) for st in STUFEN},
                 ergebnisse=ergebnisse)

    _sum = lambda k: sum(int(e.get(k) or 0) for e in ergebnisse) if all(e.get(k) is not None for e in ergebnisse if e['gueltig'] is not None) else None
    zeile['restkonflikte_summe'] = _sum('restkonflikte'); zeile['offen_summe'] = _sum('offen')
    Path(SAETZE[satz][2]).parent.mkdir(parents=True, exist_ok=True)
    with open(SAETZE[satz][2], 'a') as f: f.write(json.dumps(zeile) + '\n')
    if not behalten: shutil.rmtree(laufordner, ignore_errors=True)
    print('%s %s (%s): %d/%d gueltig, %d unbekannt, Restkonflikte %s, offen %s, CPU je gueltiger Aufgabe %s s, je Stufe %s' % (
        satz.capitalize(), zeile['zeit'], kopf, n_g, len(ergebnisse), zeile['unbekannt'], zeile['restkonflikte_summe'], zeile['offen_summe'], zeile['cpu_je_gueltig'], zeile['gueltig_je_stufe']), flush=True)
    stagnation_pruefen()
    return zeile


def stagnation_pruefen():
    ""
    if Path(DATEN).resolve() != Path(HAUPT, 'lernen/daten/lernkurve.jsonl').resolve():
        print('Diagnosekurve: kein Standard-Stagnationsalarm', flush=True)
        return
    try:
        from lernen.stagnation import pruefen
        result = pruefen(Path(HAUPT))
        print('Stagnationswaechter: ' + result['status'], flush=True)
    except Exception as exc:
        print('Stagnationswaechter unbekannt: ' + repr(exc), flush=True)


def lesen(satz='lernkurve'):
    ""
    from lernen import aufgaben as AU
    p = SAETZE[satz][2]
    rows = [json.loads(l) for l in open(p)] if os.path.exists(p) else []
    return [z for z in rows if z.get('generatorversion') == AU.VERSION]


def abnahme_faellig():
    z = lesen('abnahme')
    if not z: return True
    return time.time() - time.mktime(time.strptime(z[-1]['zeit'], '%Y-%m-%d %H:%M')) > ABNAHME_TAGE * 86400


def eingefroren(par, satz='lernkurve'):
    ""
    kopf = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=R, capture_output=True, text=True).stdout.strip()
    code = os.path.join(ORDNER, 'code', kopf[:12])
    if not os.path.exists(os.path.join(code, 'lernen', 'lernkurve.py')):
        os.makedirs(code, exist_ok=True)
        archive = subprocess.check_output(['git', '-C', R, 'archive', kopf])
        subprocess.run(['tar', '-x', '-C', code], input=archive, check=True)
        os.symlink(os.path.join(R, '.venv'), os.path.join(code, '.venv'))
    alt = sorted(glob.glob(os.path.join(ORDNER, 'code', '*')), key=os.path.getmtime)[:-3]
    for d in alt: shutil.rmtree(d, ignore_errors=True)
    from lernen import meister as ME
    snapshot=ME.einfrieren(Path(ORDNER)/'meister_snapshots'/(uuid.uuid4().hex+'.json'))
    env = dict(os.environ, LERNKURVE_DATEN=DATEN, LERNKURVE_HAUPT=R, LERNKURVE_KOPF=kopf[:7],
               AUTOROUTER_MEISTER=snapshot['pfad'],PYTHONHASHSEED='0')
    subprocess.run([os.path.join(R, '.venv', 'bin', 'python'), os.path.join(code, 'lernen', 'lernkurve.py'), 'messen',
                    'parallel=%s' % par, 'satz=' + satz], cwd=code, env=env)


RELEVANT = ['router/', 'platzierung/', 'analyse/klassen.py', 'lernen/stromzonen.py', 'lernen/stromzonen_topo.py',
            'lernen/form.py', 'lernen/aufgaben.py', 'lernen/bga_belegung.py', 'lernen/zonenplaner.py', 'gemeinsam/']


def alter_h(z):
    ""
    try: return (time.time() - time.mktime(time.strptime(z.get('zeit', ''), '%Y-%m-%d %H:%M'))) / 3600
    except ValueError: return 1e9


def relevant_geaendert(alt, neu):
    ""
    r = subprocess.run(['git', 'diff', '--name-only', alt, neu, '--'] + RELEVANT, cwd=R, capture_output=True, text=True)
    return r.returncode != 0 or bool(r.stdout.strip())


def messung_faellig(rows):
    ""
    return not rows or alter_h(rows[-1]) >= 24


def main():
    a = sys.argv[1:] or ['zeigen']; kw = dict(x.split('=', 1) for x in a if '=' in x)
    if a[0] == 'bilanz':
        return bilanz(kw)
    if a[0] == 'messen':
        messen(int(kw.get('parallel', '3')), zeit=int(kw.get('zeit', '300')), it=int(kw.get('iter', '40')),
               satz=kw.get('satz', 'lernkurve'), ziel=kw.get('ziel'), budget=kw.get('budget', 'cpu'),
               arbeit_max=int(kw.get('arbeit_max', '59147')), notbremse=int(kw.get('notbremse', '60000')),
               router=kw.get('router', '').split(',') if kw.get('router') else (), behalten=kw.get('behalten') == '1', lenkung=kw.get('lenkung'))
        return
    if a[0] == 'dauer':
        stop = os.path.join(R, 'lernen', 'STOP'); iv = int(kw.get('intervall', '10800'))
        if iv < 1 or int(kw.get('zyklen','0')) < 0: raise ValueError('intervall>0, zyklen>=0')


        cycles = 0

        threading.Thread(target=stundenbericht, args=(kw, stop), daemon=True).start()
        while not os.path.exists(stop):


            try:
                from lernen.meister_tor import pruefen as meister_pruefen
                decision = meister_pruefen({k[8:]: v for k, v in kw.items() if k.startswith('meister_')})
                print('Meisterdienst: ' + json.dumps(decision), flush=True)
                if isinstance(decision,dict) and 'tor' in decision: cycles += 1
            except Exception as exc:
                print('Meisterdienst unbekannt: ' + repr(exc), flush=True)

            alt = lesen()
            if kw.get('tagesmessung','1') != '0' and messung_faellig(alt):
                eingefroren(kw.get('parallel', '3'))
            if kw.get('tagesmessung','1') != '0' and abnahme_faellig() and not os.path.exists(stop): eingefroren(kw.get('parallel', '3'), satz='abnahme')
            if int(kw.get('zyklen','0')) and cycles >= int(kw['zyklen']): return

            aktuell = lesen()
            rest = max(60, (24 - alter_h(aktuell[-1])) * 3600) if aktuell else 60
            t0 = time.time()
            while time.time() - t0 < min(iv, rest, 60) and not os.path.exists(stop):
                time.sleep(min(60,iv))
        return
    for z in lesen(): print(z['zeit'], z['code'], '%d/%d gueltig' % (z['gueltig'], z['aufgaben']), 'CPU/gueltig', z['cpu_je_gueltig'], z['gueltig_je_stufe'])

def stundenbericht(kw, stop):
    ""
    while not os.path.exists(stop):
        try:
            bilanz(kw)
        except Exception as e:
            print('Lernbilanz unbekannt: ' + type(e).__name__ + ': ' + str(e), flush=True)
        deadline = time.monotonic() + 3600
        while time.monotonic() < deadline and not os.path.exists(stop): time.sleep(60)


def bilanz(kw):
    ""
    from lernen.lernbilanz import balance, operating_balance, observe
    keys = ('bilanz_satz', 'bilanz_basis', 'bilanz_wirksam')
    if not all(kw.get(k) for k in keys):
        if any(kw.get(k) for k in keys):
            print('Lernbilanz unbekannt: unvollstaendige Qualitaetsbilanz-Argumente', flush=True)
            return 1
        now = time.time()
        try:
            data = kw.get('bilanz_daten', os.path.join(HAUPT, 'lernen/daten'))
            state = kw.get('bilanz_zustand', os.path.expanduser('~/Library/Caches/Autorouter/lernbilanz/eingang.json'))
            report = operating_balance(data, now - 86400, now)
            report['eingang'] = observe(data, state)
        except (OSError, ValueError, KeyError, TypeError, AttributeError) as e:
            report = dict(status='unbekannt', gruende=[str(e)])
        print('Lernbetrieb ' + json.dumps(report, ensure_ascii=False, allow_nan=False), flush=True)
        return 1 if report['status'] == 'unbekannt' else 0
    try:
        if any(os.path.getsize(kw[k]) > 2_000_000 for k in keys):
            raise ValueError('Bilanz braucht kompakte Ergebnisobjekte, keine Rohkorpora')
        report = balance(*(json.load(open(kw[k])) for k in keys))
    except (OSError, ValueError) as e:
        report = dict(status='unbekannt', gruende=[str(e)])
    print('Lernbilanz ' + json.dumps(report, ensure_ascii=False), flush=True)
    return 0 if report['status'] == 'gemessen' else 1


if __name__ == '__main__':
    sys.exit(main())
