""
import copy
import hashlib
import json
import math
from pathlib import Path
import random
import shutil
import threading

VERSION = 'lehrer-v2'
VERTRAG = 'minimax-vollstaendig-v1'
DATEIEN = ('board.json', 'regeln.json', 'vorgaben.json')
GRENZEN = dict(faktor=(1.05, 3.0), teile=(0, 400), aktive_pins=(0.05, 0.95), strompfade=(0, 12),
               ampere=(0.1, 15.0), lagen=(4, 8), pitch=(0.4, 1.0), bga=(0, 1), takt=(0.25, 2.0),
               paare=(0, 8), laengengruppen=(0, 8), analog=(0, 0.6))
SCHRITTE = dict(faktor=-0.04, teile=2, aktive_pins=0.025, strompfade=1, ampere=0.05,
                lagen=-2, pitch=-0.05, bga=1, takt=0.05, paare=1, laengengruppen=1, analog=0.025)

FAMILIEN = dict(mainboard='train', netzteil='train', funk='kalibrierung', analog='kalibrierung', audio='test')
MERKMALE = tuple(GRENZEN) + ('flaeche_mm2', 'courtyard_mm2', 'teile_ist', 'pads', 'netze', 'aktive_ist',
                             'strompfade_ist', 'strom_A_max', 'analog_ist', 'pitch_ist', 'bga_ist',
                             'takt_Hz', 'paare_ist', 'gruppen_ist')


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def identitaet(d):
    return hashlib.sha256(b''.join(Path(d, f).read_bytes() for f in DATEIEN)).hexdigest()


def familie_id(art):
    if art not in FAMILIEN: raise ValueError('Lehrer braucht vorab festgelegte Blockfamilie, kein auto/Seed-Split')
    return 'generator-v8:' + art


def zufallsrezept(rnd, art=None, seed=None):
    ""
    art = art or rnd.choice([k for k, v in FAMILIEN.items() if v == 'train'])
    return dict(seed=seed if seed is not None else rnd.randrange(100000, 800000), bloecke=art, stufe=0., familie=familie_id(art),
                achsen=dict(faktor=rnd.choice([1.4, 1.6, 1.8, 2.0]), lagen=rnd.choice([4, 6, 8]),
                            aktive_pins=rnd.choice([.3, .4, .5]), ampere=rnd.choice([.1, .3, .5]),
                            teile=rnd.choice([0, 60, 100]), strompfade=rnd.choice([0, 4, 5]), takt=rnd.choice([.5, 1.0]),
                            paare=rnd.choice([0, 0, 1]), laengengruppen=rnd.choice([0, 0, 1]), analog=rnd.choice([0., .15, .25])))


def rolle(familie):
    return FAMILIEN.get(familie.removeprefix('generator-v8:'), 'unbekannt')


def achsen_pruefen(a):
    for k, v in a.items():
        if k not in GRENZEN: raise ValueError('Unbekannte Achse: ' + k)
        lo, hi = GRENZEN[k]
        if not isinstance(v, (int, float)) or not math.isfinite(v) or not lo <= v <= hi:
            raise ValueError('Achse %s ausserhalb [%s,%s]' % (k, lo, hi))
        if k in ('teile', 'strompfade', 'paare', 'laengengruppen', 'bga') and int(v) != v:
            raise ValueError('Ganzzahl erwartet: ' + k)
    if a.get('lagen', 4) not in (4, 6, 8): raise ValueError('K19: nur 4/6/8 Lagen')


def aufgabenmerkmale(A, B):
    ps = [p for p in B['pads'] if p.get('net')]
    eig = A['netze_eig']
    pitch = []; mcu_pitch = None
    import re
    for f in B['fps']:
        m = re.search(r'_P([\d.]+)mm', f['fp'])
        if m:
            pitch.append(float(m[1]))
            if f['ref'] == A.get('mcu_ref'): mcu_pitch = float(m[1])
    return dict(flaeche_mm2=A['flaeche_mm2'], courtyard_mm2=A['courtyard_mm2'], teile_ist=len(B['fps']),
                pads=len(B['pads']), netze=len(B['nets']), aktive_ist=sum(p.get('pintype') in ('input', 'output', 'bidirectional') for p in ps),
                strompfade_ist=len(A['schienen_A']), strom_A_max=max(A['schienen_A'].values(), default=0),
                analog_ist=sum(e.get('art') == 'analog' for e in eig.values()) / max(1, len(eig)),
                pitch_ist=min(pitch, default=1.0), mcu_pitch_ist=mcu_pitch, bga_ist=int(any('BGA' in f['fp'] for f in B['fps'])),
                takt_Hz=max((e.get('f_Hz', 0) for e in eig.values()), default=0),
                paare_ist=sum('z_diff_ohm' in e for e in eig.values()) / 2, gruppen_ist=len(A.get('laengengruppen', {})))


def plausibel(A, B):
    ""
    fehler = []
    from shapely.geometry import Polygon
    fl = sum(Polygon(p).area for p in B['outline'])
    crt = sum(Polygon(p).area for c in B['courtyards'] for p in c['polys'])
    if crt > fl + 1e-8: fehler.append('Courtyard-Summe > Flaeche')
    L = A['lagen']; gnd = set(A['gndlagen'])
    erlaubt = {'In%d' % i for i in range(1, L - 1)}
    if not gnd <= erlaubt or len(gnd) < {4: 1, 6: 2, 8: 3}.get(L, 99): fehler.append('K19: GND-Lagen fehlen')
    by = {}
    for p in B['pads']: by.setdefault(p['net'], []).append(p)
    fps = {f['ref']: f for f in B['fps']}
    from analyse.klassen import REGLER_A, tabelle
    for n, amp in A['schienen_A'].items():
        pp = by.get(n, [])
        quellen = [p for p in pp if p.get('pintype') == 'power_out' or p['ref'].startswith(('L', 'D'))]
        lasten = [p for p in pp if p.get('pintype') == 'power_in' and p['ref'].startswith(('U', 'J'))]
        if not quellen or not lasten: fehler.append(n + ': Quelle/Verbraucher fehlt')

        for p in quellen:
            if not p['ref'].startswith('L'): continue
            sw = {q['net'] for q in B['pads'] if q['ref'] == p['ref'] and q['net'] != n}
            regs = {q['ref'] for q in B['pads'] if q['net'] in sw and q['ref'].startswith('U')}
            caps = [tabelle(REGLER_A, fps[r]['value']) for r in regs]
            if caps and all(c is not None and c + 1e-6 < amp for c in caps): fehler.append(n + ': Regler-Nennstrom zu klein')
        if not math.isfinite(amp) or amp <= 0: fehler.append(n + ': Strom unplausibel')
    return fehler


def kontext(cpu_s, it, solver=None):
    from lernen.selbstspiel import code_hashes
    return dict(solver=solver or code_hashes()['code_sha256'], cpu_s=float(cpu_s), iter=int(it), vertrag=VERTRAG)


def kontext_id(c):
    return hashlib.sha256(json.dumps(c, sort_keys=True).encode()).hexdigest()


def zensiert(z):
    ""
    from lernen.fruehabbruch import abgebrochen
    return bool(z.get('cpu_zeitlimit') or z.get('code_geaendert')
                or abgebrochen(z)
                or (z.get('lerndaten') or {}).get('zensiert')
                or any((z.get(k) or {}).get('zustand') in ('zeitlimit', 'wandlimit', 'fehler', 'fruehabbruch')
                       for k in ('placer', 'router', 'klassen_eingang', 'klassen_placement')))


def labels(z):
    ""
    if z.get('code_geaendert') or z.get('lehrer_kontext_abweichung'):
        return dict(platzierung=None, routing=None, gueltig=None)
    kol = (z.get('placer_kennzahlen') or {}).get('kollisionen')
    if kol is not None: pl = float(kol == 0)
    else: pl = None
    if (z.get('lehrer_pruefung') or {}).get('phase') == 'platzierung': pl = 0.0
    routing = None
    if pl == 1 and z.get('router', {}).get('zustand') == 'ok' and z.get('ergebnis') is not None:
        e = z['ergebnis']
        if all(isinstance(e.get(k), (int, float)) for k in ('konflikte', 'offen', 'unverbunden')):
            routing = float(e['konflikte'] == 0 and e['offen'] == 0 and e['unverbunden'] == 0)
    if zensiert(z): routing = None
    status = None if zensiert(z) else z.get('lehrer_status')
    return dict(platzierung=pl, routing=routing, gueltig=1.0 if status == 'erfuellt' else 0.0 if status == 'verletzt' else None)


def pruefen(d, A, z):
    ""
    kol = (z.get('placer_kennzahlen') or {}).get('kollisionen')
    if kol is not None and kol > 0: return dict(status='verletzt', phase='platzierung', grund='Platzierungskollision')
    if kol != 0: return dict(status='unbekannt', grund='Kollisionsmessung fehlt')
    for f in ('board.json', 'klassen.json', 'routes.json'):
        if not Path(d, f).exists(): return dict(status='unbekannt', grund=f + ' fehlt')
    from analyse.bewertung import bewerte
    B, K, rr = [json.loads(Path(d, f).read_text()) for f in ('board.json', 'klassen.json', 'routes.json')]
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    umriss = unary_union([Polygon(p) for p in B['outline']])
    crts = [(c['ref'], c.get('side', 'F'), unary_union([Polygon(p) for p in c['polys']])) for c in B['courtyards']]
    for i, (ref, side, g) in enumerate(crts):
        if not umriss.buffer(1e-5).covers(g): return dict(status='verletzt', phase='platzierung', grund=ref + ': ausserhalb Umriss')
        if any(side == s and g.distance(h) < .00485 for _, s, h in crts[:i]):
            return dict(status='verletzt', phase='platzierung', grund=ref + ': Courtyard-Abstand verletzt')
        for ko in B.get('keepouts', []):
            if ko.get('ref') != ref and ko.get('verbote', {}).get('teile') and any(g.intersects(Polygon(p)) for p in ko['polys']):
                return dict(status='verletzt', phase='platzierung', grund=ref + ': Bauteilsperre verletzt')
    rr['gndlagen'] = A['gndlagen']
    E = bewerte(B, rr, K, v3=True, minimax=True)
    status = E['minimax_status']

    felder = ('minimax', 'minimax_version', 'minimax_profil', 'minimax_rang', 'minimax_summe',
              'minimax_strompfade', 'minimax_vollstaendig', 'minimax_quellen', 'minimax_gueltig',
              'minimax_status', 'engpass', 'sicherheit', 'bewertungsziel')
    netze = (E.get('metriken') or {}).get('netze') or {}
    strom_x = max([max(v['x'].get('dc', 0), v['x'].get('j', 0)) for v in netze.values()
                   if v.get('klasse') in ('strom', 'versorgung', 'sw')] or [0.0])
    regel = dict(z.get('regel') or {}, **{k: E[k] for k in felder}, regeln=E['regeln'],
                 note_v3=E.get('note_v3_netze'), strom_x_max=strom_x, gueltig=E['gueltig'])

    regel['bewertungsziel'] = A.get('bewertungsziel', 'v3')
    return dict(status={'gueltig': 'erfuellt', 'ungueltig': 'verletzt', 'unbekannt': 'unbekannt'}[status],
                note_v3=E.get('note_v3_netze'), regel=E['regeln'], schueler_regel=regel,
                sicherheit=E['sicherheit'], vertrag=VERTRAG)


def vektor(A):
    v = dict(A.get('achsen') or {}, **(A.get('merkmale_aufgabe') or {}))
    return [math.log1p(max(0.0, float(v.get(k, 0)))) for k in MERKMALE]

def zuverlaessigkeit(y, p):
    if not y: return dict(n=0, brier=None, kurve=[])
    kurve = []
    for i in range(5):
        ix = [j for j, q in enumerate(p) if i / 5 <= q < (i + 1) / 5 or (i == 4 and q == 1)]
        kurve.append(dict(von=i / 5, bis=(i + 1) / 5, n=len(ix), prognose=sum(p[j] for j in ix) / len(ix) if ix else None,
                          erfolg=sum(y[j] for j in ix) / len(ix) if ix else None))
    return dict(n=len(y), brier=sum((a-b)**2 for a, b in zip(y, p)) / len(y), kurve=kurve)


class Modell:
    ""
    def __init__(self):
        self.koepfe = {}

    def fit(self, zeilen):
        import numpy as np
        from scipy.optimize import minimize
        from scipy.special import expit
        self.koepfe = {}
        for ctx in sorted({kontext_id(z['kontext']) for z in zeilen}):
            for ziel in ('platzierung', 'routing', 'gueltig'):
                ds = [z for z in zeilen if kontext_id(z['kontext']) == ctx and labels(z)[ziel] is not None]
                tr = [z for z in ds if rolle(z['A']['familie']) == 'train']
                ca = [z for z in ds if rolle(z['A']['familie']) == 'kalibrierung']
                y = np.array([labels(z)[ziel] for z in tr]); prior = (sum(y) + 1) / (len(y) + 2)
                h = dict(n=len(tr), n_kalibrierung=len(ca), prior=float(prior), kalibriert=False)
                if len(tr) >= 8 and len(set(y)) == 2:
                    x = np.array([vektor(z['A']) for z in tr]); mu = x.mean(0); sd = np.maximum(x.std(0), 0.1)
                    x = np.column_stack((np.ones(len(x)), (x-mu)/sd))
                    def loss(w):
                        t = x @ w
                        return float(np.logaddexp(0, t).sum() - y @ t + 0.5 * np.dot(w[1:], w[1:])), x.T @ (expit(t)-y) + np.r_[0, w[1:]]
                    w = minimize(loss, np.zeros(x.shape[1]), jac=True, method='L-BFGS-B').x
                    h.update(mu=mu.tolist(), sd=sd.tolist(), w=w.tolist(), kalib=[1.0, 0.0])
                    if len(ca) >= 8 and len({labels(z)[ziel] for z in ca}) == 2:
                        xc = (np.array([vektor(z['A']) for z in ca])-mu)/sd
                        yc = np.array([labels(z)[ziel] for z in ca]); t = w[0] + xc @ w[1:]
                        def closs(ab):
                            s = ab[0]*t+ab[1]
                            return float(np.logaddexp(0, s).sum()-yc @ s + .1 * np.dot(ab, ab))
                        ab = minimize(closs, [1., 0.], bounds=[(0, 10), (-10, 10)]).x
                        h.update(kalib=ab.tolist(), kalibriert=True)
                self.koepfe[ctx + ':' + ziel] = h
        return self

    def predict(self, A, c):
        from scipy.special import expit
        import numpy as np
        out = {}
        for ziel in ('platzierung', 'routing', 'gueltig'):
            h = self.koepfe.get(kontext_id(c) + ':' + ziel, {})
            p = h.get('prior', 0.5)
            if 'w' in h:
                w = np.array(h['w']); t = w[0] + ((np.array(vektor(A))-h['mu'])/h['sd']) @ w[1:]
                p = float(expit(h['kalib'][0] * t + h['kalib'][1]))
            out[ziel] = float(p)
            out[ziel + '_kalibriert'] = h.get('kalibriert', False)
        return out

    def pruefbericht(self, zeilen):
        out = {}
        for fam in sorted({z['A']['familie'] for z in zeilen}):
            for ctx in sorted({kontext_id(z['kontext']) for z in zeilen if z['A']['familie'] == fam}):
                for ziel in ('platzierung', 'routing', 'gueltig'):
                    ds = [z for z in zeilen if z['A']['familie'] == fam and kontext_id(z['kontext']) == ctx and labels(z)[ziel] is not None]
                    out[fam + ':' + ctx[:12] + ':' + ziel] = dict(rolle=rolle(fam), **zuverlaessigkeit(
                        [labels(z)[ziel] for z in ds], [self.predict(z['A'], z['kontext'])[ziel] for z in ds]))
        return out


class Lehrer:
    def __init__(self, ordner, kontext, seed=1, neu=0.25, anker=0.15, mutation=0.35, kapazitaet=128,
                 lernfenster=512, kalibrierfenster=256, fit_intervall=16):
        self.root = Path(ordner); self.root.mkdir(parents=True, exist_ok=True)
        self.kontext = kontext; self.rnd = random.Random(seed); self.neu = neu; self.anker = anker
        self.mutation = mutation; self.kapazitaet = kapazitaet; self.lock = threading.RLock()
        if min(kapazitaet, lernfenster, kalibrierfenster, fit_intervall) < 1:
            raise ValueError('Puffer/Fenster/Fitintervall müssen positiv sein')
        self.lernfenster = lernfenster; self.kalibrierfenster = kalibrierfenster
        self.fit_intervall = fit_intervall; self.meldungen = 0
        self.modell = Modell()
        self.tick = 0; self.puffer = {}; self.zeilen = []; self.fenster = {}
        p = self.root / 'zustand.json'
        if p.exists():
            s = json.loads(p.read_text())
            if s['kontext'] != kontext: raise ValueError('Lehrerzustand hat anderen Solver/Budget/Vertrag')
            self.tick = s['tick']; self.puffer = s['puffer']; self.zeilen = s['zeilen']; self.fenster = s['fenster']
            self.meldungen = s.get('meldungen', len(self.zeilen))
            self.zeilen = [self.lernzeile(z) for z in self.zeilen]
            self.begrenzen()

            def tup(x): return tuple(tup(v) if isinstance(v, list) else v for v in x)
            self.rnd.setstate(tup(s['zufall']))
            if s.get('fit_config') == self.fit_config() and 'koepfe' in s:
                self.modell.koepfe = s['koepfe']
            else: self.modell.fit(self.zeilen)

    def fit_config(self):
        return [self.lernfenster, self.kalibrierfenster, self.fit_intervall]

    @staticmethod
    def lernzeile(z):
        ""
        keys = ('kontext', 'placer_kennzahlen', 'router', 'ergebnis', 'placer', 'lehrer_status',
                'lehrer_pruefung', 'code_geaendert', 'lehrer_kontext_abweichung', 'cpu_zeitlimit',
                'zustand', 'lerndaten', 'fruehabbruch')
        row = {k: z[k] for k in keys if k in z}
        if 'lerndaten' in row: row['lerndaten'] = dict(zensiert=zensiert(z))
        row['A'] = {k: z['A'][k] for k in ('familie', 'achsen', 'merkmale_aufgabe') if k in z['A']}
        row['lehrer_pruefung'] = {k: (z.get('lehrer_pruefung') or {})[k] for k in ('status', 'phase')
                                if k in (z.get('lehrer_pruefung') or {})}
        row['ergebnis'] = {k: (z.get('ergebnis') or {})[k] for k in ('konflikte', 'offen', 'unverbunden')
                          if k in (z.get('ergebnis') or {})}
        return row

    def begrenzen(self):
        counts = {}; rows = []
        for z in reversed(self.zeilen):
            r = rolle(z['A']['familie']); n = counts.get(r, 0)
            limit = self.lernfenster if r == 'train' else self.kalibrierfenster
            if r in ('train', 'kalibrierung') and n < limit:
                rows.append(z); counts[r] = n + 1
        self.zeilen = list(reversed(rows))

    def zeuge_pruefen(self, e):
        w = e.get('zeuge')
        if not w: return False
        try:
            d = Path(w['pfad'])
            ok = w['kontext'] == self.kontext and all(sha(d / f) == h for f, h in w['hashes'].items())
            ok = ok and pruefen(d, e['A'], e['zeugenzeile'])['status'] == 'erfuellt'
        except (OSError, ValueError, KeyError): ok = False
        if not ok: e['zeuge'] = None
        return ok

    def speichern(self):
        p = self.root / 'zustand.json'; tmp = p.with_suffix('.neu')
        tmp.write_text(json.dumps(dict(version=VERSION, kontext=self.kontext, tick=self.tick, puffer=self.puffer,
                                      zeilen=self.zeilen, fenster=self.fenster, zufall=self.rnd.getstate(),
                                      meldungen=self.meldungen, fit_config=self.fit_config(), koepfe=self.modell.koepfe)))
        tmp.replace(p)

    def prioritaet(self, e):
        p = self.modell.predict(e['A'], self.kontext)['gueltig']

        band = 4 * p * (1-p)
        alter = min(1., (self.tick-e['tick'])/20)
        return (.1 + band + e.get('fehler', 0) + e.get('abstand_best', 0) + .3*alter) / (1+.2*e['wdh'])

    def verwerfen(self, rezept, grund):
        ""
        with self.lock:
            with (self.root / 'verworfen.jsonl').open('a') as f:
                f.write(json.dumps(dict(tick=self.tick, rezept=rezept, grund=grund, kontext=self.kontext,
                                        status='verworfen', machbarkeit='unbekannt')) + '\n')
            self.speichern()

    def vorschlag(self):
        with self.lock:
            self.tick += 1
            es = list(self.puffer.values()); geloest = [e for e in es if self.zeuge_pruefen(e)]
            u = self.rnd.random()
            if not es or u < self.neu:
                return dict(zufallsrezept(self.rnd), art='neu')
            if geloest and u < self.neu + self.anker:

                e = max(geloest, key=lambda x: (x['A']['achsen'].get('faktor', 0),
                                                -x['A'].get('merkmale_aufgabe', {}).get('aktive_ist', 0)))
                e['wdh'] += 1
                return dict(copy.deepcopy(e['rezept']), art='anker', eltern=e['id'])
            e = self.rnd.choices(es, [self.prioritaet(e) for e in es])[0]; e['wdh'] += 1
            if geloest and self.rnd.random() < self.mutation:
                e = self.rnd.choices(geloest, [self.prioritaet(e) for e in geloest])[0]
                r = copy.deepcopy(e['rezept']); ax = dict(r['achsen'])
                nutzbar = [k for k in SCHRITTE if k not in ('aktive_pins', 'pitch', 'bga') or
                           any(b.startswith('Prozessor') for b in e['A'].get('bloecke', []))]
                if e['A'].get('merkmale_aufgabe', {}).get('mcu_pitch_ist') is None:
                    nutzbar = [k for k in nutzbar if k != 'pitch']
                k = self.rnd.choice(nutzbar); f = self.fenster.get(k, [])[-8:]
                richtung = -1 if len(f) >= 4 and sum(f)/len(f) < .3 else 1
                altwert = ax.get(k, e['A']['achsen'].get(k, .8 if k == 'pitch' else e['A'].get('merkmale_aufgabe', {}).get('bga_ist', 0)))


                istfeld = dict(teile='teile_ist', strompfade='strompfade_ist', analog='analog_ist').get(k)
                if istfeld:
                    altwert = max(altwert, e['A'].get('merkmale_aufgabe', {}).get(istfeld, 0))
                v = altwert + richtung * SCHRITTE[k]
                lo, hi = GRENZEN[k]; ax[k] = round(min(hi, max(lo, v)), 4)
                if k == 'pitch':
                    pitches = [0.4, 0.5, 0.65, 0.8, 1.0]

                    alt = e['A']['merkmale_aufgabe']['mcu_pitch_ist']
                    cand = [p for p in pitches if p < alt] if richtung == 1 else [p for p in pitches if p > alt]
                    ax[k] = max(cand) if richtung == 1 and cand else min(cand) if cand else alt
                r.update(achsen=ax, art='mutation', achse=k, eltern=e['id'], richtung=richtung)

                return r
            return dict(copy.deepcopy(e['rezept']), art='replay', eltern=e['id'])

    def kalibrieren(self, rezept, ad, A, z, kandidat, cpu_budget=None):
        ""
        return self.melden(rezept, ad, A, z, kandidat, cpu_budget=cpu_budget, kalibrierung=True)

    def melden(self, rezept, ad, A, z, kandidat, cpu_budget=None, kalibrierung=False):
        with self.lock:
            erwartet = 'kalibrierung' if kalibrierung else 'train'
            if rolle(A['familie']) != erwartet: raise ValueError('Falsche Familie für ' + erwartet)
            task = identitaet(ad); pred = self.modell.predict(A, self.kontext)
            e = None if kalibrierung else self.puffer.get(task)
            if e is None:
                dest = self.root / ('kalibrieraufgaben' if kalibrierung else 'aufgaben') / task
                dest.mkdir(parents=True, exist_ok=True)
                for f in DATEIEN + ('aufgabe.json',): shutil.copyfile(Path(ad, f), dest / f)
                e = dict(id=task, A=A, rezept={k: copy.deepcopy(rezept[k]) for k in ('seed', 'bloecke', 'stufe', 'achsen', 'familie')},
                         abstammung=dict(ausgangsschaltung=A['seed'], familie=A['familie'], eltern_task=rezept.get('eltern'),
                                         achse=rezept.get('achse'), richtung=rezept.get('richtung')),
                         wdh=0, tick=self.tick, zeuge=None)
                if not kalibrierung: self.puffer[task] = e
            feedback = {k: copy.deepcopy(e[k]) for k in ('fehler', 'abstand_best', 'note_best') if k in e}
            achse = rezept.get('achse'); altes_fenster = copy.deepcopy(self.fenster.get(achse))
            abweichung = bool(z.get('code_geaendert') or z.get('code_sha256') != self.kontext['solver'] or
                             z.get('kontext') != self.kontext)
            if cpu_budget: cpu_budget.protokoll(z)
            if abweichung or zensiert(z):
                check = dict(status='unbekannt', grund='Solver/Budget/Vertrag fehlt/geaendert oder CPU-Budget erschoepft')
            else: check = pruefen(kandidat, A, z)
            zz = dict(z, lehrer_status=check['status'], lehrer_pruefung=check, kontext=z.get('kontext'), A=A, task=task,
                      abstammung=e['abstammung'], rezept=rezept, prognose=pred, lehrer_kontext_abweichung=abweichung)
            if 'schueler_regel' in check:
                zz['regel'] = check['schueler_regel']; zz['note_v3'] = check['note_v3']
            zz['gueltig'] = None if check['status'] == 'unbekannt' else check['status'] == 'erfuellt'
            if cpu_budget: cpu_budget.protokoll(zz)
            if zz['lehrer_status'] == 'erfuellt' and not kalibrierung:
                dest = self.root / 'zeugen' / task
                if dest.exists(): shutil.rmtree(dest)
                shutil.copytree(kandidat, dest)

                chk = pruefen(dest, A, z)
                if chk['status'] == 'erfuellt' and (cpu_budget is None or cpu_budget.rest > 0):
                    e['zeugenzeile'] = dict(placer_kennzahlen=z['placer_kennzahlen'])
                    e['zeuge'] = dict(pfad=str(dest), kontext=self.kontext, aufgabe=task,
                                      hashes={str(p.relative_to(dest)): sha(p) for p in sorted(dest.rglob('*.json'))}, pruefung=chk)
                else:
                    e['zeuge'] = None
                    zz.update(lehrer_status=chk['status'], lehrer_pruefung=chk, gueltig=False)
            e['tick'] = self.tick

            while len(self.puffer) > self.kapazitaet:
                candidates = [q for q in self.puffer.values() if not q.get('zeuge')] or list(self.puffer.values())
                q = min(candidates, key=self.prioritaet); del self.puffer[q['id']]
            self.meldungen += 1

            if self.meldungen % self.fit_intervall == 0:
                self.modell.fit(self.zeilen)
            if cpu_budget: cpu_budget.protokoll(zz)
            y = labels(zz)['gueltig']; e['fehler'] = abs(y-pred['gueltig']) if y is not None else 0.
            n = zz['lehrer_pruefung'].get('note_v3')
            if isinstance(n, (int, float)) and math.isfinite(n):
                best = e.get('note_best')
                e['abstand_best'] = min(1., max(0., (n-best)/max(1., abs(best)))) if best is not None else 0.
                if zz['lehrer_status'] == 'erfuellt': e['note_best'] = min(n, best if best is not None else n)
            if not kalibrierung and rezept.get('achse') and y is not None:
                k = rezept['achse']; self.fenster[k] = (self.fenster.get(k, []) + [y])[-8:]
            self.zeilen.append(self.lernzeile(zz)); self.begrenzen()
            self.speichern()
            if cpu_budget:
                cpu_budget.protokoll(zz)
                self.zeilen[-1] = self.lernzeile(zz)
                if zz.get('cpu_zeitlimit'):
                    e['zeuge'] = None
                    for k in ('fehler', 'abstand_best', 'note_best'):
                        if k in feedback: e[k] = feedback[k]
                        else: e.pop(k, None)
                    if achse:
                        if altes_fenster is None: self.fenster.pop(achse, None)
                        else: self.fenster[achse] = altes_fenster
                    self.speichern()
                    cpu_budget.protokoll(zz)
            if zz.get('lehrer_status') == 'unbekannt': zz['gueltig'] = None
            return zz
