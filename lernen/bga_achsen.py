""
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import random
import tempfile

VERSION = 4
QUOTENFENSTER = 100
VERLAUFSFENSTER = 100
BGA_MESSREIHE_FREI = False


def art(z):
    if (z.get('lern_achse') or {}).get('gehaeuse') in ('bga', 'qfn'):
        return z['lern_achse']['gehaeuse']
    return 'bga' if any('BGA' in b.upper() for b in z.get('bloecke', [])) else 'qfn'


def schluessel(gehaeuse, lagen):
    return '%s_L%d' % (gehaeuse, lagen)


def pfad(kw):
    return Path(kw['daten']).with_suffix('.achsen.json')


@contextmanager
def zustand(kw):
    p = pfad(kw); p.parent.mkdir(parents=True, exist_ok=True)
    with open(str(p) + '.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        try:
            S = json.loads(p.read_text()) if p.exists() else dict(version=VERSION, achsen={}, aufgaben={}, bga_fenster=[], leistung={}, reserviert_n=0, reserviert_bga=0, archiv_bis=-1)
            if S.get('version') in (1, 2):



                for cell in S['achsen'].values():
                    cell['fenster'] = []
                    cell['generation'] = cell.get('generation', -1) + 1
                for row in S['aufgaben'].values(): row['generation'] = -1
                S['bga_fenster'] = []
                if S['version'] == 1:
                    S['quotenfenster'] = [S['aufgaben'][k]['gehaeuse']
                        for k in sorted(S['aufgaben'], key=int)][-QUOTENFENSTER:]
                S['version'] = VERSION
            if S.get('version') == 3:



                S['bga_fenster'] = []
                S['version'] = VERSION
            if S.get('version') != VERSION:
                raise ValueError('Unbekannte Achsen-Version')
            for cell in S['achsen'].values():
                history = cell.get('verlauf', [])
                cell['verlauf_n'] = max(cell.get('verlauf_n', 0), len(history))
                cell['verlauf'] = history[-VERLAUFSFENSTER:]
                cell['verlauf_archiviert'] = cell['verlauf_n'] - len(cell['verlauf'])
            yield S
            fd, tmp = tempfile.mkstemp(dir=p.parent, prefix=p.name + '.')
            try:
                with os.fdopen(fd, 'w') as f:
                    json.dump(S, f, indent=1); f.flush(); os.fsync(f.fileno())
                os.replace(tmp, p)
            finally:
                if os.path.exists(tmp): os.unlink(tmp)
        finally:
            fcntl.flock(lock, fcntl.LOCK_UN)


def reservieren(kw, seed, stufe_wahl=None):
    ""
    with zustand(kw) as S:
        key = str(seed)
        if key in S['aufgaben']: return dict(S['aufgaben'][key])
        if seed <= S.get('archiv_bis', -1):
            raise ValueError('Aufgabennummer bereits archiviert; neue seed0 verwenden')
        fen = S['bga_fenster'][-20:]
        frei = len(fen) >= 8 and sum(fen) / len(fen) >= .25
        cap = (.5 if frei else .35) if BGA_MESSREIHE_FREI else .10
        total = S.get('reserviert_n', len(S['aufgaben']))
        bga_n = S.get('reserviert_bga', sum(x['gehaeuse'] == 'bga' for x in S['aufgaben'].values()))
        quote = S.setdefault('quotenfenster', [])[-(QUOTENFENSTER - 1):]
        gehaeuse = 'bga' if (quote.count('bga') + 1) * 100 <= (len(quote) + 1) * int(cap * 100) else 'qfn'
        S['quotenfenster'] = quote + [gehaeuse]
        rnd = random.Random(seed * 7919 + 29)

        layers = int(kw.get('lagen', '0')) or rnd.choice([4, 6, 8])
        if layers not in (4, 6, 8): raise ValueError('Achsen: lagen=4|6|8')
        axis = schluessel(gehaeuse, layers)
        init = float(kw.get('stufe', '0'))
        cell = S['achsen'].setdefault(axis, dict(stufe=init, fenster=[], verlauf=[], generation=0))
        st = cell['stufe'] if stufe_wahl is None else stufe_wahl(cell['stufe'], dict(gehaeuse=gehaeuse, lagen=layers))
        row = dict(gehaeuse=gehaeuse, lagen=layers, achse=axis, stufe=st, bga_cap=cap,
                   bga_quote=sum(fen) / len(fen) if fen else None, zustand='reserviert', generation=cell['generation'])
        S['aufgaben'][key] = row
        S['reserviert_n'] = total + 1
        S['reserviert_bga'] = bga_n + (gehaeuse == 'bga')


        for old in sorted(S['aufgaben'], key=int):
            if len(S['aufgaben']) <= 1000: break
            if S['aufgaben'][old]['zustand'] not in ('fertig', 'abgebrochen'): continue
            S['archiv_bis'] = max(S.get('archiv_bis', -1), int(old))
            del S['aufgaben'][old]
        return dict(row)


def melden(kw, seed, geschafft, cpu_s=None, gueltige=0, generatorfehler=False, abgebrochen=False):
    ""
    with zustand(kw) as S:
        if str(seed) not in S['aufgaben'] and seed <= S.get('archiv_bis', -1): return
        row = S['aufgaben'][str(seed)]
        if row['zustand'] in ('fertig', 'abgebrochen'): return
        if abgebrochen and geschafft is not True: geschafft = None
        row.update(zensiert=geschafft is None, zustand='abgebrochen' if abgebrochen else 'fertig', geschafft=geschafft, cpu_s=cpu_s, gueltige=gueltige, generatorfehler=generatorfehler)
        perf = S.setdefault('leistung', {}).setdefault(row['achse'],
                    dict(aufgaben=0, geschafft=0, gueltige=0, cpu_s=0., cpu_unbekannt=0))
        perf['aufgaben'] += 1
        if isinstance(cpu_s, (int, float)) and cpu_s > 0:
            perf['cpu_s'] += cpu_s
            perf['geschafft'] += geschafft is True
            perf['gueltige'] += gueltige
        else: perf['cpu_unbekannt'] += 1
        perf['gueltige_pro_cpu_h'] = perf['gueltige'] * 3600 / perf['cpu_s'] if perf['cpu_s'] else None
        if geschafft is None:
            perf['zensiert'] = perf.get('zensiert', 0) + 1
            return
        cell = S['achsen'][row['achse']]

        if row['generation'] != cell['generation'] or row['stufe'] != cell['stufe']:
            cell['veraltete'] = cell.get('veraltete', 0) + 1
            return
        if row['gehaeuse'] == 'bga': S['bga_fenster'] = (S['bga_fenster'] + [bool(geschafft)])[-20:]
        cell['fenster'].append(bool(geschafft))
        if len(cell['fenster']) == 10:
            n = sum(cell['fenster']); old = cell['stufe']
            cell['stufe'] = round(max(0., old + (.5 if n > 6 else -.5 if n < 4 else 0.)), 2)
            cell['verlauf'].append(dict(seed=seed, erfolge=n, n=10, von=old, stufe=cell['stufe']))
            cell['verlauf_n'] += 1
            cell['verlauf'] = cell['verlauf'][-VERLAUFSFENSTER:]
            cell['verlauf_archiviert'] = cell['verlauf_n'] - len(cell['verlauf'])
            cell['fenster'] = []
            cell['generation'] += 1


def effizienz(zeilen):
    ""
    out = {}
    for z in zeilen:
        for key in (art(z), 'L%d' % z['lagen'], schluessel(art(z), z['lagen'])):
            s = out.setdefault(key, dict(n=0, gueltig=0, cpu_s=0., cpu_unbekannt=0, etiketten=0))
            s['n'] += 1
            cpu = z.get('cpu_total')
            if isinstance(cpu, (int, float)) and cpu > 0:
                s['cpu_s'] += cpu; s['gueltig'] += z.get('gueltig') is True
            else: s['cpu_unbekannt'] += 1
            s['etiketten'] += (z.get('datensatz') or {}).get('etiketten_n', 0)
    for s in out.values():
        s['gueltige_pro_cpu_h'] = s['gueltig'] * 3600 / s['cpu_s'] if s['cpu_s'] else None
    return out
