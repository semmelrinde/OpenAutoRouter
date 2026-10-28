""
import hashlib
import json
import math
from pathlib import Path

MODELL = Path(__file__).with_name('fruehabbruch.json')
Q1_MODELL = Path(__file__).with_name('fruehabbruch_q1.json')
VERSION = 'fruehabbruch-v1'
RUNDEN = (8, 12, 16, 20, 24)
REST_MAX = 40
MIN_RUNDE = 12
MIN_KONFLIKTE = 80
MIN_SCHWELLE = .95
MERKMALE = ('runde', 'log_konflikte', 'trend4', 'restanteil', 'konflikte_netze',
            'konflikte_pads', 'log_pads', 'lagen4', 'lagen6', 'lagen8',
            'pad_dichte', 'faktor', 'bga', 'stufe', 'cpu_anteil')


def aktiv(wert):
    return str(wert) in ('1', 'q1')


def modellpfad(cfg):

    if str(cfg.get('fruehabbruch')) == 'q1':
        if cfg.get('fruehabbruch_modell'):
            raise ValueError('q1 hat ein festes Modell; Austauschmodell nur mit fruehabbruch=1')
        return Q1_MODELL
    return cfg.get('fruehabbruch_modell') or MODELL


def abgebrochen(z):
    info = z.get('lerndaten') or {}
    return bool(z.get('zustand') == 'fruehabbruch' or z.get('fruehabbruch')
                or info.get('zustand') == 'fruehabbruch' or info.get('ursache') == 'fruehabbruch')


def kontext_pruefen(kontext):
    if not isinstance(kontext, dict):
        raise ValueError('Fruehabbruch-Kontext muss ein Objekt sein')
    keys = ('pads', 'netze', 'flaeche_mm2', 'lagen', 'faktor', 'stufe', 'bga')
    if any(type(kontext.get(k)) not in (int, float) or not math.isfinite(kontext[k]) for k in keys):
        raise ValueError('unvollstaendiger/nicht endlicher Fruehabbruch-Kontext')
    if (min(kontext[k] for k in ('pads', 'netze', 'flaeche_mm2', 'faktor')) <= 0
            or kontext['lagen'] not in (4, 6, 8) or kontext['stufe'] < 0
            or kontext['bga'] not in (0, 1)):
        raise ValueError('ungueltige Fruehabbruch-Geometrie')
    return kontext


def merkmale(rows, kontext, budget):
    if not isinstance(rows, list) or any(not isinstance(r, dict) for r in rows): return None
    rr = [r for r in rows if r.get('phase') == 'verhandlung']
    if not rr or rr[-1].get('runde') not in RUNDEN or len(rr) < 4:
        return None
    try:

        for r in rr:
            if any(type(r.get(k)) not in (int, float) or not math.isfinite(r[k])
                   or r[k] < 0 for k in ('runde', 'konflikte', 'cpu_s')):
                return None
            if r['runde'] != int(r['runde']): return None
        kontext_pruefen(kontext)
        pads, nets = float(kontext['pads']), float(kontext['netze'])
        area = float(kontext['flaeche_mm2'])
        layers = int(kontext['lagen'])
        faktor = float(kontext['faktor'])
        stufe = float(kontext['stufe'])
        bga = float(kontext['bga'])
        cpu = float(rr[-1]['cpu_s'])
        if (min(pads, nets, area, budget) <= 0 or not math.isfinite(budget)
                or layers not in (4, 6, 8) or faktor <= 0 or stufe < 0 or bga not in (0., 1.)):
            return None
        vals = [float(r['konflikte']) for r in rr]
        c = vals[-1]
        x = [rr[-1]['runde'], math.log1p(c), (c - vals[-4]) / max(1., vals[-4]),
             c / max(1., vals[0]), c / nets, c / pads, math.log1p(pads),
             float(layers == 4), float(layers == 6), float(layers == 8),
             pads / area, faktor, bga, stufe, cpu / budget]
        return x if all(math.isfinite(v) for v in x) and cpu > 0 else None
    except (KeyError, ValueError, TypeError, OverflowError): return None


def modell_laden(path=MODELL):
    data = Path(path).read_bytes()
    m = json.loads(data)
    vergleich = (isinstance(m, dict) and m.get('version') == 'fruehabbruch-v2'
                 and m.get('verwendung') == 'verlustvergleich'
                 and type(m.get('verlustquote_prozent')) is int
                 and m['verlustquote_prozent'] in (0, 1, 2, 3)
                 and m.get('min_runde') == 8 and m.get('min_konflikte') == MIN_KONFLIKTE)
    if (not isinstance(m, dict) or m.get('version') not in (VERSION, 'fruehabbruch-v2')
            or (m.get('version') == 'fruehabbruch-v2' and not vergleich) or m.get('merkmale') != list(MERKMALE)
            or m.get('status') != 'experimentell' or m.get('runden') != list(RUNDEN)
            or m.get('rest_max') != REST_MAX or m.get('min_runde') != (8 if vergleich else MIN_RUNDE)
            or m.get('min_konflikte') != MIN_KONFLIKTE):
        raise ValueError('ungueltiger Fruehabbruch-Modellvertrag')
    for k in ('mittel', 'skala', 'gewichte'):
        if (not isinstance(m.get(k), list) or len(m[k]) != len(MERKMALE)
                or not all(type(v) in (int, float) and math.isfinite(v) for v in m[k])):
            raise ValueError('ungueltige Modellkoeffizienten')
    if (any(v <= 0 for v in m['skala'])
            or type(m.get('schwelle')) not in (int, float)
            or not (0. if vergleich else MIN_SCHWELLE) <= m['schwelle'] <= 1
            or type(m.get('bias')) not in (int, float) or not math.isfinite(m['bias'])):
        raise ValueError('ungueltige Modellskala/Schwelle')
    m = dict(m, sha256=hashlib.sha256(data).hexdigest())
    return m


def entscheidung(rows, kontext, budget, modell):
    x = merkmale(rows, kontext, budget)
    if x is None: return None
    rr = [r for r in rows if r.get('phase') == 'verhandlung']
    if modell.get('vergleichsregel') == 'Runde 8, >80 Konflikte' and rr[-1]['runde'] != 8:
        return None

    if rr[-1]['konflikte'] <= REST_MAX: return None
    if rr[-1]['runde'] < modell['min_runde'] or rr[-1]['konflikte'] <= modell['min_konflikte']:
        return None

    if any(b['runde'] <= a['runde'] or b['cpu_s'] < a['cpu_s'] for a, b in zip(rr, rr[1:])):
        return None
    logit = modell['bias'] + sum(w * (v - mu) / s for w, v, mu, s in
                               zip(modell['gewichte'], x, modell['mittel'], modell['skala']))
    score = 1 / (1 + math.exp(-max(-700., min(700., logit))))
    if score < modell['schwelle']: return None
    return dict(version=modell['version'], modell_sha256=modell['sha256'], score=score,
                schwelle=modell['schwelle'], runde=rr[-1]['runde'],
                konflikte=rr[-1]['konflikte'], cpu_s=rr[-1]['cpu_s'],
                cpu_budget_s=budget, merkmale=dict(zip(MERKMALE, x)),
                ziel='kein gueltiges Ergebnis innerhalb dieses Budgets', zensiert=True)


def kontext_aufgabe(a):
    return dict(pads=a['pads'], netze=a['netze'], flaeche_mm2=a['flaeche_mm2'],
                lagen=a['lagen'], faktor=a['faktor'], stufe=a.get('stufe', 0),
                bga=int(any('BGA' in b for b in a['bloecke'])))


def generator_board_pruefen(board):
    src = board.get('src') if isinstance(board, dict) else None
    if not isinstance(src, str) or not src.startswith('aufgabe seed=') or '(lernen/aufgaben.py v' not in src:
        raise ValueError('native Boards: Vorhersager bis zu eigenen Paarmessungen gesperrt')


def kontext_board(board, lagen):

    raise ValueError('native Boards: Vorhersager bis zu eigenen Paarmessungen gesperrt')


def optionen(cfg, kontext=None):
    ""
    if not aktiv(cfg.get('fruehabbruch', '0')): return []
    modellpfad(cfg)
    out = ['fruehabbruch=' + str(cfg['fruehabbruch'])]
    if cfg.get('fruehabbruch_modell'): out.append('fruehabbruch_modell=' + str(cfg['fruehabbruch_modell']))
    if kontext is not None:
        out.append('fruehabbruch_kontext=' + json.dumps(kontext, sort_keys=True, separators=(',', ':')))
    return out
