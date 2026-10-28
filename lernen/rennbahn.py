""
import copy
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import time

ARME = {
    'via_dichte': ('router', {'via_dichte': '1'}),
    'r27_stamm': ('router', {'r27_stamm': '1'}),
    'abzweig': ('router', {'abzweig': '1'}),
    'knick': ('router', {'knick': '.2'}),
    'knick_strom': ('router', {'knick': '.2', 'knick_strom': '0'}),
    'dicht': ('placer', {'dicht': '2'}),
    'dicht_innen': ('placer', {'dicht_innen': '2'}),
    'bga_fanout': ('router', {'bga_fanout': '1'}),
    'strom_via_preis': ('router', {'strom_via_preis': '1'}),
    'strom_zuerst': ('router', {'strom_zuerst': '1'}),
    'p35': ('placer', {'p35': '1'}),
    'r24': ('cfg', {'r24': True}),
    'r19_form': ('router', {'r19_form': '1'}),
    'paar_suchraum': ('router', {'paar': '1', 'paar_suchraum': '1'}),
    'hist_start': ('hist', {}),
}
ALPHA = .05
MIN_FAMILIEN = 8


def digest(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def codegleich(expected, results, actual):
    ""
    return actual == expected and all(all(z.get(k) == v for k, v in expected.items()) for z in results)


def familie(A):
    import re
    declared = A.get('familie') or (A.get('abstammung') or {}).get('familie')
    if declared: return declared
    skeleton = [re.sub(r'\d+(?:[.,]\d+)?', '#', b) for b in A['bloecke']]
    return 'blockskelett:' + digest(sorted(skeleton))


def anwendbar(arm, A, history=None):
    if arm == 'hist_start': return history is not None and Path(history).exists()
    if arm == 'bga_fanout': return A['lagen'] >= 6 and any('BGA' in b for b in A['bloecke'])
    if arm == 'paar_suchraum': return bool(A.get('laengengruppen'))
    return True


def konfigurieren(cfg, arm, an, history=None):
    c = copy.deepcopy(cfg)
    kind, values = ARME[arm]
    if kind == 'hist':
        c.pop('hist_start', None)
        if an:
            if not history: raise ValueError('Selbstimitation ohne Vorgänger')
            c['hist_start'] = str(Path(history).resolve())
    elif kind == 'router':
        from lernen.selbstspiel import ROUTER_VARIANTEN
        r = c.get('router', 'alt')
        r = list(ROUTER_VARIANTEN[r] if isinstance(r, str) else r)


        if arm == 'knick_strom' and not an:
            c['router'] = r
            return c
        r = [v for v in r if v.split('=', 1)[0] not in values]

        if arm == 'paar_suchraum': r += ['paar=1']
        if arm == 'knick_strom':
            r += ['knick=.2', 'knick_strom=0']
        else:
            r += [f'{k}={v if an else 0}' for k, v in values.items() if k != 'paar']
        c['router'] = r
    else:
        c.update({k: (v if an else (False if isinstance(v, bool) else '0')) for k, v in values.items()})
    return c


def _messung(ad, kid, z):
    ""
    from analyse import bewertung as BW
    from analyse import regelkennzahlen as RK
    from analyse.strom_kette import kette
    c = Path(ad) / f'k{kid}'
    try:
        from lernen.selbstspiel import code_hashes
        before = code_hashes()
        if z.get('code_geaendert') or any(z.get(k) != v for k, v in before.items()):
            raise ValueError('Codewechsel im Versuch')
        B, R, K = [json.loads((c / f'{n}.json').read_text()) for n in ('board', 'routes', 'klassen')]
        R['gndlagen'] = z.get('gndlagen', [])
        E = BW.bewerte(B, R, K, v3=True, minimax=False)
        M = E['regeln']
        if any(RK.wert(M, k) is None for k in RK.torfelder(M)):
            raise ValueError('Regeltor unvollständig')
        chain = kette(B, R, K)

        needs = {n for n, v in K['netze'].items() if any(finite(i) and i > 1e-6 for i in ((v.get('fluss') or {}).get('last') or {}).values()) and
                 v.get('art') in ('strom', 'versorgung', 'sw')}
        complete = not chain.get('ausgelassen') and needs <= set(chain['netze'])
        if chain['netze']:
            complete = complete and all(finite(v.get('glied')) and finite(v.get('wirkungsgrad')) for v in chain['netze'].values())
        elif needs: complete = False
        if code_hashes() != before: raise ValueError('Codewechsel in Messung')
        return dict(abgeschlossen=z.get('zustand') == 'ok', gueltig=z.get('gueltig') is True and E['gueltig'] is True,
                    regeln=M, kette=chain, kette_vollstaendig=bool(complete), version=E['note_v3_version'])
    except Exception as e:
        return dict(gueltig=z.get('gueltig') is True, fehler=repr(e))

def messung(ad, kid, z):
    from gemeinsam import plaetze as PL
    from lernen.selbstspiel import VPY
    c = Path(ad) / f'k{kid}'
    c.mkdir(parents=True, exist_ok=True)
    if any(not (c/f'{name}.json').exists() for name in ('board', 'routes', 'klassen')):
        return dict(gueltig=z.get('gueltig') is True, abgeschlossen=z.get('zustand') == 'placer_kollision',
                    fehler='Versuch ohne vollständige Artefakte', rechenplatz=dict(cpu_s=0., zustand='nicht_gestartet'))
    input_file = c/'rennbahn_eingang.json'
    input_file.write_text(json.dumps(z))
    result = PL.lauf([VPY, str(Path(__file__).resolve()), 'messen', str(ad), str(kid)],
                     cpu_s=180, log=str(c/'rennbahn_messung.log'),
                     env=dict(os.environ, PYTHONHASHSEED='0'))
    output = c/'rennbahn_messung.json'
    m = json.loads(output.read_text()) if result['zustand'] == 'ok' and output.exists() else dict(
        gueltig=z.get('gueltig') is True, fehler='Messung '+result['zustand'])
    m['rechenplatz'] = result
    return m


def finite(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def paarwert(m):
    ""
    n = m['regeln']['note_v3']
    c = m['kette']
    if not c['netze']: return math.log1p(n)
    return math.log1p(n) - math.log(max(1e-12, c['wirkungsgrad'])) + math.log1p(c['glied_max'])


def vergleichen(a, b):
    from analyse import regelkennzahlen as RK
    out = dict(gueltig_delta=int(b['gueltig'])-int(a['gueltig']), tor=False, gemessen=False, delta=None)
    if not a['gueltig'] and not b['gueltig'] and a.get('abgeschlossen') and b.get('abgeschlossen'):
        return dict(out, tor=True, gueltigkeit_gemessen=True, grund='beide_ungueltig')
    if a.get('abgeschlossen') is False or b.get('abgeschlossen') is False:
        return dict(out, grund='zensierter Versuch')
    if not a['gueltig'] and b['gueltig'] and a.get('abgeschlossen'):


        if b.get('fehler') or not b.get('version') or not b.get('kette_vollstaendig') or any(
                RK.wert(b.get('regeln', {}), k) is None for k in RK.torfelder(b.get('regeln', {}))):
            return dict(out, grund='Gültigkeitsgewinn unvollständig gemessen')
        return dict(out, tor=True, gueltigkeit_gemessen=True, grund='gueltigkeitsgewinn')
    if a.get('fehler') or b.get('fehler'): return dict(out, grund='Messfehler')
    if a.get('version') != b.get('version'): return dict(out, grund='Bewertungsvertrag')
    if not a['gueltig'] or not b['gueltig']:
        return dict(out, grund='ungültiges Paar')
    if not a.get('kette_vollstaendig') or not b.get('kette_vollstaendig'):
        return dict(out, grund='Kettenmessung unvollständig')
    gate = RK.annahme(a['regeln'], b['regeln'])
    same = set(a['kette']['netze']) == set(b['kette']['netze'])
    out.update(tor=gate['nicht_schlechter'] and same, gemessen=True, regelvergleich=gate,
               gueltigkeit_gemessen=True,
               delta=paarwert(b)-paarwert(a), grund=gate['grund'] if same else 'andere Kettennetze')
    return out


def statistik(rows, alpha=ALPHA):
    ""
    families = {}
    for r in rows:
        if r.get('evidenz', True) and r['vergleich'].get('gemessen'):
            families.setdefault(r['familie'], r['vergleich']['delta'])
    family_deltas = {}
    for r in rows:
        if r['vergleich'].get('gemessen'):
            family_deltas.setdefault(r['familie'], []).append(r['vergleich']['delta'])
    mean_delta = sum(sum(v)/len(v) for v in family_deltas.values()) / len(family_deltas) if family_deltas else None
    signs = [1 if v < -1e-9 else -1 if v > 1e-9 else 0 for v in families.values()]
    def ewert(signs):
        return sum(math.exp(min(690., sum(math.log1p(lam*s) for s in signs))) for lam in (.1, .25, .5, .75)) / 4
    e = ewert(signs)
    validity = {}
    for r in rows:
        if r.get('evidenz', True) and r['vergleich'].get('gueltigkeit_gemessen'):
            validity.setdefault(r['familie'], r['vergleich']['gueltig_delta'])
    ev = ewert(list(validity.values()))
    bad = any(r['vergleich']['gueltig_delta'] < 0 or
              (r['vergleich'].get('gemessen') and not r['vergleich']['tor']) for r in rows)
    missing = any(not r['vergleich'].get('gemessen') and r['vergleich'].get('grund') not in ('gueltigkeitsgewinn', 'beide_ungueltig') for r in rows)
    threshold = 2 * len(ARME) / alpha
    quality_win = mean_delta is not None and mean_delta < -1e-9 and len(families) >= MIN_FAMILIEN and e >= threshold
    validity_win = (len(validity) >= MIN_FAMILIEN and sum(validity.values()) > 0 and ev >= threshold
                    and (mean_delta is None or mean_delta <= 1e-9))
    return dict(n=len(rows), familien=len(families), delta_familien_mittel=mean_delta, siege=signs.count(1), niederlagen=signs.count(-1), e=e,
                gueltigkeitsfamilien=len(validity), e_gueltigkeit=ev, alpha=alpha, schwelle=threshold,
                tor=not bad, vollstaendig=not missing,
                befoerdern=not bad and not missing and (quality_win or validity_win))


class Rennbahn:
    ""
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def transact(self, fn):
        with open(str(self.path)+'.lock', 'a') as f:
            fcntl.flock(f, fcntl.LOCK_EX)
            d = json.loads(self.path.read_text()) if self.path.exists() else dict(format='rennbahn-v1', kontexte={})
            result = fn(d)
            fd, tmp = tempfile.mkstemp(dir=self.path.parent, suffix='.json')
            try:
                with os.fdopen(fd, 'w') as out: json.dump(d, out, indent=1, allow_nan=False)
                os.replace(tmp, self.path)
            finally:
                if os.path.exists(tmp): os.unlink(tmp)
            return result

    @staticmethod
    def fehlerbudget(d, context, generation):


        trials = d.setdefault('pruefungen', [])
        for ctx, c in sorted(d['kontexte'].items()):
            for gen in range(c['generation'] + 1):
                key = [ctx, gen]
                if key not in trials: trials.append(key)
        key = [context, generation]
        if key not in trials: trials.append(key)
        n = trials.index(key) + 1
        return ALPHA / (n * (n + 1))

    def waehlen(self, context, available, rnd, fest=None):
        def choose(d):
            c = d['kontexte'].setdefault(context, dict(standards=[], generation=0, paare=[], reserviert={}))
            self.fehlerbudget(d, context, c['generation'])
            rows = [r for r in c['paare'] if r['generation'] == c['generation']]
            if fest:
                if fest not in available: raise ValueError('Arm nicht anwendbar: '+fest)
                arm = fest
            else:
                allowed = [a for a in available if a not in c.get('gesperrt', [])]
                if not allowed: return None, list(c['standards']), c['generation']
                def priority(a):
                    st = statistik([r for r in rows if r['arm'] == a])
                    pending = c['reserviert'].get(a, 0)
                    return (st['n'] + pending > 0, rnd.betavariate(1+st['niederlagen'], 1+st['siege']), st['n']+pending, a)
                arm = min(allowed, key=priority)
            c['reserviert'][arm] = c['reserviert'].get(arm, 0)+1
            return arm, list(c['standards']), c['generation']
        return self.transact(choose)

    def melden(self, context, row):
        row = copy.deepcopy(row)
        def update(d):
            c = d['kontexte'][context]
            c['reserviert'][row['arm']] = max(0, c['reserviert'].get(row['arm'], 1)-1)

            if any(r['id'] == row['id'] for r in c['paare']): return dict(aktion='doppelt')


            row['evidenz'] = not any(r['arm'] == row['arm'] and r['familie'] == row['familie']
                and (r['vergleich'].get('gemessen') or r['vergleich'].get('gueltigkeit_gemessen'))
                for prior in d['kontexte'].values() for r in prior['paare'])
            c['paare'].append(row)
            arm = row['arm']; cmp = row['vergleich']



            regression = (cmp['gueltig_delta'] < 0 or
                (not cmp.get('gemessen') and cmp.get('grund') not in ('gueltigkeitsgewinn', 'beide_ungueltig')) or
                (cmp.get('gemessen') and (not cmp['tor'] or cmp['delta'] > 1e-9)))
            if regression:
                betroffen = set(row.get('standards', [])) | {arm}
                c.setdefault('gesperrt', [])[:] = sorted(set(c.get('gesperrt', [])) | betroffen)
                if c['standards']:
                    vorher = list(c['standards'])
                    c['standards'] = []; c['generation'] += 1
                    return dict(aktion='zurueckstufen', entfernt=vorher, standards=[],
                                generation=c['generation'], spaet=row['generation'] != c['generation']-1)
            if row['generation'] != c['generation']:
                return dict(aktion='alte_generation', tor=not regression, standards=list(c['standards']), generation=c['generation'])
            st = statistik([r for r in c['paare'] if r['generation'] == c['generation'] and r['arm'] == arm],
                           self.fehlerbudget(d, context, c['generation']))
            action = 'messen'
            if arm not in c['standards'] and st['befoerdern'] and arm not in c.get('gesperrt', []):
                c['standards'].append(arm); c['standards'].sort(); action = 'befoerdern'
            if action != 'messen': c['generation'] += 1
            return dict(aktion=action, statistik=st, standards=list(c['standards']), generation=c['generation'])
        return self.transact(update)



CFG_OPTIONEN = ('lernbudget', 'lern_cpu_max', 'lernpolicy', 'budget_vor_nachlauf', 'lerndaten',
                'paar_qualitaet', 'r36_glatt', 'r36_korridor', 'strom_form', 'r11_ir',
                'k20_zonen', 'form', 'legal_lp', 'r39', 'r24', 'r27_formtor')
BOOL_OPTIONEN = set(CFG_OPTIONEN) - {'lernbudget', 'lern_cpu_max', 'lernpolicy', 'budget_vor_nachlauf', 'lerndaten', 'form'}


def optionen(kw):
    ""
    for key, default in (('auswahl', 'alle'), ('verdichten', '0'), ('lehrer', '0'),
                         ('potenzial', '0'), ('lerngrenze', '0')):
        if key in kw and str(kw[key]) != default:
            raise ValueError('Rennbahn unterstützt '+key+'='+str(kw[key])+' nicht')

    for key in ('wert_shadow', 'start_grenze', 'leistungsboard', 'bga_achse', 'gesamt_cpu'):
        if key in kw: raise ValueError('Rennbahn unterstützt Option '+key+' nicht')
    allowed = set(CFG_OPTIONEN) | {'auswahl', 'verdichten', 'lehrer', 'potenzial', 'lerngrenze',
        'wert_shadow', 'start_grenze', 'leistungsboard', 'bga_achse', 'gesamt_cpu',
        'rennbahn', 'rennbahn_arm', 'rennbahn_stand', 'ziel', 'zonenplan', 'r36',
        'daten', 'stop', 'zeit', 'iter', 'versuche', 'behalten', 'stufe', 'anker_legal', 'leistungsanteil', 'bloecke',
        'aufgaben', 'parallel', 'seed0', 'eingefroren', 'galerie',
        'wert_trainieren', 'wert_modell', 'wert_neu', 'wert_cpu', 'wert_routen', '_wert_dienst'}
    unknown = set(kw)-allowed
    if unknown: raise ValueError('Rennbahn: unbekannte/unterstützte Optionen fehlen: '+','.join(sorted(unknown)))
    if kw.get('galerie', '0') != '0': raise ValueError('Rennbahn unterstützt galerie=1 nicht')
    for key in BOOL_OPTIONEN & set(kw):
        if str(kw[key]).lower() not in ('1','0','true','false'):
            raise ValueError('Rennbahn: '+key+' muss 0 oder 1 sein')
    return {k: (str(kw[k]).lower() in ('1', 'true') if k in BOOL_OPTIONEN else kw[k])
            for k in CFG_OPTIONEN if k in kw}


def bilanz(*teile):
    ""
    bekannt = all(finite(v) and v >= 0 for v in teile)
    return dict(cpu_s=sum(teile) if bekannt else None, vollstaendig=bekannt,
                komponenten=list(teile))


def mess_cpu(z, measurement):
    ""
    z['cpu_versuch_aufnahme_s'] = z.get('cpu_total')
    charge = measurement.get('rechenplatz', {}).get('cpu_s')
    z['cpu_rennbahn_messung_s'] = charge
    total = bilanz(z.get('cpu_total'), charge)
    z['cpu_total'] = total['cpu_s']
    z['cpu_messung'] = 'voll_wait4' if total['vollstaendig'] else 'unbekannt'


def arbeiter(ad, A, cfg, kid, zeit, it, phase='versuch'):
    ""
    from gemeinsam import plaetze as PL
    from lernen import selbstspiel as SS
    c = Path(ad)/f'k{kid}'; c.mkdir(parents=True, exist_ok=True)
    inp, out = c/(phase+'_eingang.json'), c/(phase+'_ausgang.json')
    inp.write_text(json.dumps(dict(A=A, cfg=cfg, zeit=zeit, it=it)))
    out.unlink(missing_ok=True)
    result = PL.lauf([SS.VPY, str(Path(__file__).resolve()), phase, str(ad), str(kid)],
                     cpu_s=None, log=str(c/(phase+'_arbeiter.log')),
                     env=dict(os.environ, PYTHONHASHSEED='0'))
    if result['zustand'] == 'ok' and out.exists(): z = json.loads(out.read_text())
    else: z = dict(gueltig=False, zustand=result['zustand'], fehler='Rennbahn-Arbeiter ohne Ergebnis')
    z['cpu_total'] = result['cpu_s']; z['cpu_messung'] = 'voll_wait4'
    z['rennbahn_rechenplatz'] = result
    return z


def spielen(out, seed, kw, zeilen, rnd):
    ""
    from lernen import selbstspiel as SS
    SS.optionen_pruefen(kw)
    overrides = optionen(kw)
    task_cpu_start = time.thread_time()
    ziel = SS.standardziel(explizit=kw.get('ziel'), kontext='selbstspiel')
    if ziel != 'v3': raise ValueError('rennbahn=1 unterstützt nur ziel=v3; ziel=minimax benötigt ein eigenes Paar-Tor')
    mode = kw.get('zonenplan', 'aus')
    if mode not in ('aus', 'diagnose', 'lokal'): raise ValueError('zonenplan=aus|diagnose|lokal')
    stufe = SS.stufe_lesen(kw)[1]['stufe']
    ad = Path(out).resolve() / f'aufgabe_{seed:06d}'
    A = SS.aufgabe_erzeugen(str(ad), seed, ziel=ziel, stufe=stufe,
                            anker_legal=kw.get('anker_legal', '0') == '1',
                            leistungsanteil=float(kw.get('leistungsanteil', '0')),
                            **({'bloecke': kw['bloecke']} if 'bloecke' in kw else {}))
    zeit, it = int(kw.get('zeit', '300')), int(kw.get('iter', '40'))
    SS.ZONENPLAN.update(modus=mode, r36=kw.get('r36', 'geometrisch'))

    from gemeinsam import plaetze as PL
    rg = PL.lauf([SS.VPY, str(Path(SS.R)/'analyse/klassen.py'), str(ad/'board.json'), str(ad/'klassen.json'),
                  'regeln='+str(ad/'regeln.json'), 'vorgaben='+str(ad/'vorgaben.json')], cpu_s=60,
                 log=str(ad/'klassen_eingang.log'), env=dict(os.environ, PYTHONHASHSEED='0'))
    if rg['zustand'] != 'ok' or not (ad/'klassen.json').exists():
        raise ValueError('Rennbahn: Eingangsklassifikation '+rg['zustand'])
    hashes = SS.code_hashes()
    context = digest(dict(code=hashes['code_sha256'], klasse=SS.klasse(A), zeit=zeit, iter=it,
                          generator=A['version'], zonen=kw.get('r36', 'geometrisch'), zonenplan=mode, ziel=ziel, form=kw.get('form'),
                          optionen=overrides, protocol='rennbahn-v3',
                          **({'form_arme': kw['form_arme']} if 'form_arme' in kw else {})))
    task = digest({n: hashlib.sha256((ad/n).read_bytes()).hexdigest() for n in ('board.json', 'regeln.json', 'vorgaben.json')})
    track = Rennbahn(kw.get('rennbahn_stand', str(Path(out)/'rennbahn.json')))
    trainer = None
    if kw.get('wert_trainieren', '0') == '1':
        from lernen.wertmodell import Dienst
        trainer = kw.get('_wert_dienst') or Dienst([kw['daten']], kw.get('wert_modell', str(Path(out)/'wert')),
            neu=int(kw.get('wert_neu', '100')), cpu_s=int(kw.get('wert_cpu', '300')), routen=int(kw.get('wert_routen', '4')))
    rows = []; history = None
    if kw.get('rennbahn_arm') == 'hist_start':
        cfg = SS.wahl(zeilen, SS.klasse(A), rnd, ziel='v3', form_arme=kw.get('form_arme'))
        cfg.update(hist_context=task, lerndaten='1', datensatz='0', zonenplan=mode, r36=kw.get('r36', 'geometrisch'))
        cfg.update(overrides)
        warmup = arbeiter(str(ad), A, cfg, -1, zeit, it)
        warmup['rennbahn'] = dict(phase='vorgaenger', context=context)
        warmup['designfamilie'] = familie(A)
        warmup = aufnehmen(str(ad), A, warmup, -1, zeit, it)
        with SS.LOCK:
            with open(kw['daten'], 'a') as f: f.write(json.dumps(warmup)+'\n')
        rows.append(warmup)
        history = ad/'k-1'/'etiketten.npz'
        if not history.exists(): raise ValueError('Selbstimitation: Vorgänger ohne Konfliktkarte')
    for pair in range(int(kw.get('versuche', '3'))):
        if Path(kw['stop']).exists(): break
        available = [a for a in ARME if anwendbar(a, A, history)]
        arm, standards, generation = track.waehlen(context, available, rnd, kw.get('rennbahn_arm'))
        if arm is None: break
        cfg = SS.wahl(zeilen, SS.klasse(A), rnd, ziel='v3', form_arme=kw.get('form_arme'))
        cfg.update(hist_context=task, lerndaten='1', datensatz='0', zonenplan=mode, r36=kw.get('r36', 'geometrisch'))
        cfg.update(overrides)
        for a in standards:
            if a != arm and anwendbar(a, A, history): cfg = konfigurieren(cfg, a, True, history)
        results = {}; measurements = {}

        for on in ([False, True] if (seed+pair)%2 == 0 else [True, False]):
            kid = 2*pair+int(on)
            trial_cfg = konfigurieren(cfg, arm, on, history)
            z = arbeiter(str(ad), A, trial_cfg, kid, zeit, it)
            z.update(designfamilie=familie(A), board=f'erf_{seed:06d}',
                     rennbahn=dict(arm=arm, an=on, paar=pair, context=context, generation=generation))
            measurements[on] = messung(ad, kid, z)
            z['rennbahn']['messung'] = measurements[on]
            results[on] = z
        record = dict(hist_sha256=hashlib.sha256(Path(history).read_bytes()).hexdigest() if history else None, id=digest(dict(task=task, cfg=cfg, arm=arm, generation=generation, paar=pair)),
                      arm=arm, generation=generation, standards=standards, familie=familie(A), task=task,
                      vergleich=vergleichen(measurements[False], measurements[True]),
                      bezug=measurements[False], neu=measurements[True], hashes=hashes,
                      seed=seed, budget=dict(cpu_s=zeit, iter=it), zeit=time.strftime('%Y-%m-%d %H:%M:%S'), last=os.getloadavg(),
                      cpu_s=[results[on].get('cpu_total') for on in (False, True)])
        if not codegleich(hashes, results.values(), SS.code_hashes()):
            record['vergleich'] = dict(gueltig_delta=0, tor=False, gemessen=False, delta=None, grund='Codewechsel im Paar')
        decision = track.melden(context, record)


        for on in (False, True):
            results[on] = aufnehmen(str(ad), A, results[on], 2*pair+int(on), zeit, it)
        for on in (False, True):
            mess_cpu(results[on], measurements[on])
        record['cpu_bilanz'] = bilanz(*(results[on].get('cpu_total') for on in (False, True)))


        track.transact(lambda d: next(r for r in d['kontexte'][context]['paare'] if r['id']==record['id']).update(cpu_bilanz=record['cpu_bilanz']))
        for on in (False, True):
            z = results[on]; z['rennbahn']['entscheidung'] = decision
            z['rennbahn']['cpu_bilanz'] = record['cpu_bilanz']
            with SS.LOCK:
                with open(kw['daten'], 'a') as f: f.write(json.dumps(z)+'\n')
            rows.append(z)
        if trainer is not None: trainer.pruefen()


        last = 2*pair+int((seed+pair)%2 == 0)
        candidate = ad / f'k{last}' / 'etiketten.npz'
        if candidate.exists(): history = candidate
        print(f'Rennbahn {seed} Paar {pair} {arm}: delta={record["vergleich"]["delta"]} Tor={record["vergleich"]["tor"]} {decision["aktion"]}', flush=True)

    complete = len(rows) - int(kw.get('rennbahn_arm') == 'hist_start') == 2*int(kw.get('versuche', '3'))
    completed_rows = [z for z in rows if z.get('rennbahn', {}).get('phase') != 'vorgaenger']
    if complete and completed_rows and all(z['rennbahn']['messung'].get('abgeschlossen') for z in completed_rows):
        SS.stufe_melden(kw, any(z['rennbahn']['messung'].get('gueltig') is True for z in completed_rows), seed)
    if kw.get('behalten') != '1' and completed_rows and all((z.get('datensatz') or {}).get('zustand') == 'ok' for z in rows):
        import shutil
        shutil.rmtree(ad)
    task_bill = bilanz(time.thread_time()-task_cpu_start, rg.get('cpu_s'),
                      *(z.get('cpu_total') for z in rows))
    if trainer is not None:


        task_bill = dict(task_bill, versuchs_cpu_s=task_bill['cpu_s'], cpu_s=None,
                         vollstaendig=False, training_cpu_s=None, grund='asynchrones Werttraining')
    (Path(out)/f'rennbahn_cpu_{seed:06d}.json').write_text(json.dumps(dict(task_bill, seed=seed, last=os.getloadavg()), indent=1))
    return rows


def aufnehmen(ad, A, z, kid, zeit, it):
    c = Path(ad)/f'k{kid}'; c.mkdir(parents=True, exist_ok=True)
    (c/'aufnahme_zeile.json').write_text(json.dumps(z))
    updated = arbeiter(ad, A, z.get('cfg', {}), kid, zeit, it, phase='aufnahme')
    if updated.get('fehler') and 'rennbahn' not in updated:
        updated = dict(z, cpu_total=updated.get('cpu_total'), datensatz=dict(zustand='fehler', fehler=updated['fehler']))
    charge = updated.get('cpu_total')
    total = bilanz(z.get('cpu_total'), charge)
    updated['cpu_aufnahme_s'] = charge
    updated['cpu_total'] = total['cpu_s']
    updated['cpu_messung'] = 'voll_wait4' if total['vollstaendig'] else 'unbekannt'
    return updated

if __name__ == '__main__':
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    if len(sys.argv) != 4 or sys.argv[1] not in ('messen', 'versuch', 'aufnahme'): raise SystemExit('Nur interner Arbeiteraufruf')
    phase, ad, kid = sys.argv[1], sys.argv[2], int(sys.argv[3])
    c = Path(ad)/f'k{kid}'
    if phase == 'messen':
        z = json.loads((c/'rennbahn_eingang.json').read_text())
        result = _messung(ad, kid, z)
        output = c/'rennbahn_messung.json'
    else:
        from gemeinsam import plaetze as PL
        from lernen import selbstspiel as SS
        original = PL.lauf
        def nested(*args, **kw):
            kw['mit_platz'] = False
            return original(*args, **kw)
        PL.lauf = nested
        inp = json.loads((c/(phase+'_eingang.json')).read_text())
        if phase == 'versuch':
            result = SS.versuch(ad, inp['A'], inp['cfg'], kid, inp['zeit'], inp['it'], kandidaten=1, kandidaten_auto=False)
        else:
            result = json.loads((c/'aufnahme_zeile.json').read_text())
            SS.datensatz_aufnehmen(ad, inp['A'], result, kid, kandidaten=1, kandidaten_auto=False)
        output = c/(phase+'_ausgang.json')
    output.write_text(json.dumps(result, allow_nan=False))
