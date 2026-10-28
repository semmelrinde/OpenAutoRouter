""
import hashlib
import json
import math
from pathlib import Path
import re
import sys


def lesen(path):
    with open(path) as f:
        for line in f:
            try: yield json.loads(line)
            except ValueError: continue


def cpu_s(z):
    if 'cpu_total' in z and (not isinstance(z['cpu_total'], (int, float)) or
            isinstance(z['cpu_total'], bool) or not math.isfinite(z['cpu_total']) or z['cpu_total'] < 0):
        return None
    if isinstance(z.get('cpu_total'), (int, float)):
        total = z['cpu_total']
    else:
        total = kinder_cpu_s(z) + sum((z.get(k) or {}).get('cpu_s', 0) or 0
                                     for k in ('datensatz', 'etiketten_kandidaten'))
        total += (z.get('router') or {}).get('cpu_eigen_s', 0) or 0
        total += (z.get('r44') or {}).get('cpu_s', 0) or 0



    if 'cpu_rennbahn_messung_s' not in z:
        extra = ((z.get('rennbahn') or {}).get('messung') or {}).get('rechenplatz', {}).get('cpu_s')
        if isinstance(extra, (int, float)) and not isinstance(extra, bool) and math.isfinite(extra) and extra >= 0:
            total += extra
    return total


def kinder_cpu_s(z):
    ""
    enthalten = set((z.get('router') or {}).get('cpu_enthaelt', []))
    return sum(((z.get(k) or {}).get('cpu_s', 0) or 0) - ((z.get(k) or {}).get('cpu_eigen_s', 0) or 0) for k in
               ('placer', 'router', 'form', 'klassen_eingang', 'klassen_placement') if k not in enthalten) + sum(
                    (z.get(k) or {}).get('rechenplatz', {}).get('cpu_s', 0) or 0
                    for k in ('zonenplan', 'stromzonen', 'r11_ir', 'r24') if k not in enthalten)


def cpu_summe(rows):
    charges = [cpu_s(z) for z in rows]
    return round(sum(charges), 3) if all(c is not None for c in charges) else None


def zusammenfassung(rows):
    out = {}
    for lagen in (2, 4, 6, 8, 'alle'):
        rr = [z for z in rows if lagen == 'alle' or z.get('lagen') == lagen]
        charges = [cpu_s(z) for z in rr]
        unknown = sum(c is None for c in charges)
        cpu = sum(c for c in charges if c is not None)
        usable = [z for z in rr if not z.get('code_geaendert') and not z.get('generatorfehler')]
        labels = sum((z.get('datensatz') or {}).get('etiketten_n', 0) or 0 for z in usable
                     if (z.get('datensatz') or {}).get('zustand') == 'ok')
        seen = set(); new_labels = 0; known = 0
        for z in usable:
            ds = z.get('datensatz') or {}; h = ds.get('ziel_sha256')
            identity = (ds.get('ziel_hash_version', 1), ds.get('router_beobachterversion', 1), h)
            if ds.get('zustand') == 'ok' and h:
                known += 1
                if identity not in seen: new_labels += ds.get('etiketten_n', 0); seen.add(identity)
        valid = sum(_grenz_ergebnis(z) is True for z in usable if not z.get('nicht_geroutet'))
        dense = sum((z.get('datensatz') or {}).get('zustand') == 'ok' for z in usable)
        full = sum((z.get('lerndaten') or {}).get('vollstaendig', False) for z in usable)
        best = {}; gains = 0
        for z in usable:
            key = (z.get('aufgabe', z.get('board')), z.get('faktor'), z.get('bewertung_version'))
            n = z.get('note_v3')
            if _grenz_ergebnis(z) is True and isinstance(n, (int, float)) and math.isfinite(n):
                if key in best and n < best[key]: gains += 1
                best[key] = min(best.get(key, float('inf')), n)
        rate = lambda n: n * 3600 / cpu if cpu and not unknown else None
        out[str(lagen)] = dict(versuche=len(rr), cpu_s=round(cpu, 3), gueltig=valid,
            gueltig_je_cpu_h=rate(valid), shards=dense, shards_je_cpu_h=rate(dense),
            tabellenziele=labels, tabellenziele_je_cpu_h=rate(labels),
            neue_tabellenziele=new_labels if known else None, neue_tabellenziele_je_cpu_h=rate(new_labels) if known else None,
            neuheit_shards_gemessen=known,
            lernzeilen_voll=full, lernzeilen_je_cpu_h=rate(full),
            eigene_bestnote_gewinne=gains, eigene_bestnote_gewinne_je_cpu_h=rate(gains),
            optionen_entschieden=None, modellgewinn=None,
            cpu_unbekannt=unknown,
            cpu_vollstaendig=sum(z.get('cpu_messung') in ('voll', 'voll_wait4') for z in rr))
    return dict(version=1, mass='Datenertrag; Modellgewinn und unabhaengige Information nicht gemessen', lagen=out)


def ergaenzen(z, c):
    ""
    c = Path(c); rows = list(lesen(c / 'verlauf.jsonl')) if (c / 'verlauf.jsonl').exists() else []
    if not rows:
        try: txt = (c / 'route.log').read_text(errors='replace')
        except OSError: txt = ''
        rows = [dict(runde=int(n) - 1, konflikte=int(k), offen=int(f), cpu_s=None)
                for n, k, f in re.findall(r'Iteration (\d+): Konflikte (\d+), unvollstaendig (\d+)', txt)]
    r = {}
    try: r = json.loads((c / 'routes.json').read_text())
    except (OSError, ValueError): pass
    z.setdefault('gueltig', False)
    limits = ('SIGXCPU', 'cpu_budget', 'fruehabbruch', 'budget_vor_nachlauf', 'zeitlimit', 'wandlimit', 'zeit')
    state = z.get('zustand', 'ok')
    attempt_limit = (z.get('cpu_zeitlimit') or z.get('budget_ende') or
                     (z.get('cpu_budget') or {}).get('ueberlauf', 0) > 0 or
                     any(state == q or state.endswith('_' + q) for q in limits))
    child_limit = any((z.get(q) or {}).get('zustand') in limits or
                      (z.get(q) or {}).get('rechenplatz', {}).get('zustand') in limits or
                      (z.get(q) or {}).get('grund') in limits
                      for q in ('placer', 'router', 'form', 'zonenplan', 'stromzonen', 'r11_ir',
                                'klassen_eingang', 'klassen_placement'))
    router_limit = r.get('abschluss') in limits
    if r.get('abschluss') == 'fruehabbruch':

        z.update(gueltig=None, fruehabbruch=r.get('fruehabbruch') or dict(zensiert=True))
        if state != 'r44_ungeprueft' and not z.get('code_geaendert'):
            z.update(zustand='fruehabbruch', ungueltig_grund='Vorhersage: kein Erfolg im CPU-Budget (zensiert)')
            state = 'fruehabbruch'
    limited = bool(attempt_limit or child_limit or router_limit)
    incomplete = bool(z.get('fehler') or z.get('regel_fehler') or
                      any((z.get(q) or {}).get('fehler') for q in ('form', 'zonenplan', 'stromzonen', 'r11_ir')) or
                      ((z.get('form') or {}).get('angewendet') is False and (z.get('form') or {}).get('grund') not in (None, '', 'aus', 'Empfaengertor', 'Regeltor', 'Form waere ungueltig')) or state not in ('ok', 'nur_ersatzmasse') or
                      any((z.get(q) or {}).get('zustand') == 'fehler' or
                          (z.get(q) or {}).get('rechenplatz', {}).get('zustand') == 'fehler'
                          for q in ('router', 'form', 'zonenplan', 'stromzonen', 'r11_ir')))
    reason = ('Gesamtbudget' if z.get('cpu_zeitlimit') or (z.get('cpu_budget') or {}).get('ueberlauf', 0) > 0
              else state if attempt_limit or state not in ('ok', 'nur_ersatzmasse')
              else 'Bewertungsabbruch' if z.get('regel_fehler')
              else 'Nachlaufzeitlimit' if child_limit
              else 'Nachlaufunvollstaendig' if incomplete else
              z.get('ungueltig_grund') or r.get('abschluss') or state)
    if limited:
        z['budget_ende'] = True
        if state == 'ok' and not z.get('code_geaendert'):
            z['zustand'] = 'fruehabbruch' if r.get('abschluss') == 'fruehabbruch' else 'zeitlimit'
    z['konflikt_verlauf'] = [[q['konflikte'], q['offen']] for q in rows]
    z['lerndaten'] = dict(version=1, zustand=z.get('zustand', 'fehler'), ursache=reason,
        fehler=z.get('fehler') or z.get('regel_fehler'),
        rechenlaeufe={q: z[q] for q in ('placer', 'router', 'klassen_eingang', 'klassen_placement') if q in z},
        vollstaendig=True, verlauf=rows, teilergebnis=bool(r),
        zensiert=limited or incomplete or not r, loesbarkeit_bekannt=not (limited or incomplete or not r) and z.get('gueltig') is True,
        etiketten_vorhanden=(c / 'etiketten.npz').exists() or (c / 'routes_etiketten.npz').exists(),
        konfliktorte=r.get('lernstand', {}).get('paare', []) or r.get('kurzschluss_schutz', {}).get('paare_vor_bereinigung', []),
        ausgabe_schutz=r.get('kurzschluss_schutz'), budget=r.get('lernbudget'),
        placer_verlauf=z.get('placer_versuche', []))
    return z


def entscheidung(rows, pads, policy):
    ""
    n = int(policy.get('runde', 4))
    rr = [q for q in rows if q.get('phase') == 'verhandlung']
    if len(rr) < n: return False
    tail = rr[-n:]; first = max(1, tail[0]['konflikte'] + tail[0]['offen'])
    current = tail[-1]['konflikte'] + tail[-1]['offen']
    return (current > 8 and current / max(1, pads) >= float(policy['konflikte_pro_pad']) and
            current / first >= float(policy.get('restanteil', .95)))


def vielversprechend(rows):
    rr = [q for q in rows if q.get('phase') == 'verhandlung'][-4:]
    if len(rr) < 3: return False
    v = [q['konflikte'] + q['offen'] for q in rr]
    return 0 < v[-1] < .8 * max(1, v[0])


def _grenz_ergebnis(z):
    ""
    if z.get('code_geaendert') or z.get('generatorfehler'): return None
    if type(z.get('gueltig')) is not bool: return None
    if ((z.get('lerndaten') or {}).get('zensiert') or z.get('cpu_zeitlimit') or
            z.get('budget_ende') or z.get('fehler') or z.get('regel_fehler') or
            (z.get('cpu_budget') or {}).get('ueberlauf', 0) > 0): return None
    if z.get('zustand', 'ok') not in ('ok', 'nur_ersatzmasse', 'placer_kollision'): return None
    form = z.get('form') or {}
    if form.get('angewendet') is False and form.get('grund') not in (
            None, '', 'aus', 'Empfaengertor', 'Regeltor', 'Form waere ungueltig'): return None
    for key in ('placer', 'router', 'form', 'zonenplan', 'stromzonen', 'r11_ir',
                'klassen_eingang', 'klassen_placement'):
        child = z.get(key) or {}
        if child.get('fehler'): return None
        for state in (child.get('zustand'), (child.get('rechenplatz') or {}).get('zustand'), child.get('grund')):
            if state in ('SIGXCPU', 'cpu_budget', 'fruehabbruch', 'budget_vor_nachlauf',
                         'zeitlimit', 'wandlimit', 'zeit', 'fehler'): return None
    return z['gueltig']


def grenze_faktor(rows, klasse, fallback):
    ""
    groups = {}
    for z in rows[-3000:]:
        if z.get('klasse') != klasse: continue
        result = _grenz_ergebnis(z)
        if result is None: continue
        if not isinstance(z.get('faktor'), (float, int)): continue
        key = (z.get('aufgabe'), round(z['faktor'], 1))
        groups[key] = groups.get(key, False) or result
    if len({k[0] for k in groups}) < 12: return fallback
    bins = {}
    for (_, f), g in groups.items(): bins.setdefault(f, []).append(g)
    eligible = [(abs((sum(gs) + 1) / (len(gs) + 2) - .5), f) for f, gs in bins.items() if len(gs) >= 6]
    return max(1.1, min(2.5, min(eligible)[1])) if eligible else fallback


def aufgaben_reihenfolge(tasks, rows, lagen, rnd):
    ""
    rates = {}
    for z in rows[-20000:]:
        result = _grenz_ergebnis(z)
        if result is None: continue
        rates.setdefault(z.get('board'), []).append(result)
    decorated = []
    for task in tasks:
        rr = rates.get(Path(task).name, [])[-40:]
        p = (sum(rr) + 1) / (len(rr) + 2)
        explore = rnd.random() < .2 or len(rr) < 6
        decorated.append(((0 if lagen.get(Path(task).name) in (4, 6, 8) else 1,
                          0 if explore else 1, rnd.random() if explore else abs(p - .5)), task))
    return [task for _, task in sorted(decorated)]


def kalibrieren(rows):
    ""
    result = {}
    for layer in (4, 6, 8):
        rr = [z for z in rows if z.get('lagen') == layer and z.get('konflikt_verlauf') and not z.get('code_geaendert')
              and (z.get('lerndaten') or {}).get('ursache') != 'fruehabbruch']
        def holdout(z):
            family = str(z.get('familie', z.get('designfamilie', z.get('aufgabe', z.get('board')))))
            return int(hashlib.sha256(family.encode()).hexdigest(), 16) % 3 == 0
        train = [z for z in rr if not holdout(z)]; test = [z for z in rr if holdout(z)]
        def evaluate(data, policy):
            hits = []; savings = 0.; known = 0
            for z in data:
                n = policy['runde']
                seq = [dict(phase='verhandlung', konflikte=k, offen=f) for k, f in z['konflikt_verlauf'][:n]]
                pads = z.get('pads') or (z.get('merkmale') or {}).get('n_pads')
                if not pads or not entscheidung(seq, pads, policy): continue
                hits.append(z)
                v = [q for q in (z.get('lerndaten') or {}).get('verlauf', []) if q.get('phase') == 'verhandlung']
                cp = (z.get('router') or {}).get('cpu_s')
                if z.get('gueltig') is False and len(v) >= n and cp and isinstance(v[n-1].get('cpu_s'), (float, int)):
                    known += 1; savings += max(0, cp - v[n-1]['cpu_s'])
            invalid_cpu = sum((z.get('router') or {}).get('cpu_s', 0) or 0 for z in data if z.get('gueltig') is False)
            return dict(n=len(data), valid=sum(z.get('gueltig') is True for z in data), hit=len(hits),
                        lost=sum(z.get('gueltig') is True for z in hits), cpu_sparanteil=savings / invalid_cpu if known and invalid_cpu else None,
                        timing_hits=known)
        candidates = []
        for n in (3, 4, 5, 6):
            for threshold in (.1, .2, .3, .5):
                for rest in (.8, .9, .95, 1.):
                    p = dict(version=1, status='experimentell', runde=n, konflikte_pro_pad=threshold, restanteil=rest)
                    e = evaluate(train, p)
                    if e['hit'] and e['lost'] == 0: candidates.append((e['hit'], p, e))
        if not candidates:
            result[str(layer)] = dict(status='unbekannt', grund='kein trainierbarer Kandidat'); continue
        _, p, e = sorted(candidates, key=lambda q: (-q[0], q[1]['runde'], -q[1]['konflikte_pro_pad']))[0]
        h = evaluate(test, p)
        result[str(layer)] = dict(policy=p, train=e, holdout=h, freigegeben=False,
            empirisch_tor_erfuellt=bool(h['valid'] and h['lost'] / h['valid'] <= .02 and h['cpu_sparanteil'] is not None and h['cpu_sparanteil'] >= .5),
            grund='Replay ist keine prospektive Freigabe; CPU- und Gueltigkeitstor gepaart nachweisen')
    return dict(version=1, split='sha256(familie)%3=0 ist Holdout', lagen=result)


def main():
    paths = [x for x in sys.argv[1:] if '=' not in x and x != 'kalibrieren']
    kw = dict(x.split('=', 1) for x in sys.argv[1:] if '=' in x)
    rows = [z for p in paths for z in lesen(p)]
    result = kalibrieren(rows) if 'kalibrieren' in sys.argv[1:] else zusammenfassung(rows)
    if 'kalibrieren' not in sys.argv[1:]:
        window = max(1, int(kw.get('fenster', '200')))
        result['verlauf'] = [dict(zeilen_von=i, zeilen_bis=min(i + window, len(rows)),
            cpu_kumulativ_s=cpu_summe(rows[:i + window]),
            lagen=zusammenfassung(rows[i:i + window])['lagen']) for i in range(0, len(rows), window)]
    result['quellen'] = [dict(pfad=str(Path(p).resolve()), sha256=hashlib.sha256(Path(p).read_bytes()).hexdigest()) for p in paths]
    s = json.dumps(result, indent=2, ensure_ascii=False)
    if kw.get('out'): Path(kw['out']).write_text(s + '\n')
    else: print(s)


if __name__ == '__main__': main()
