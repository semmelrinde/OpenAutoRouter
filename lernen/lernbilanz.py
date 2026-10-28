""
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import time
from datetime import datetime

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
VERSION = 'lernbilanz-v1'
SHA = re.compile('[0-9a-f]{64}')


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def number(x):
    try:
        return type(x) in (int, float) and math.isfinite(x) and x >= 0
    except OverflowError:
        return False


def seal(boards, train_families, created):
    ""
    if len(boards) != 20 or len({b['id'] for b in boards}) != 20:
        raise ValueError('Genau 20 eindeutige Boards erforderlich')
    if len({b['familie'] for b in boards}) != 20:
        raise ValueError('20 verschiedene Designfamilien erforderlich')
    if not train_families:
        raise ValueError('Trainings-Familieninventar fehlt')
    if any(b['familie'] in train_families or not SHA.fullmatch(b.get('sha256', '')) for b in boards):
        raise ValueError('Trainingsueberlappung oder fehlender Boardhash')
    body = dict(version='lernpruefsatz-v1', boards=sorted(boards, key=lambda b: b['id']),
                training_familien_sha=digest(sorted(train_families)), erstellt=created,
                ungesehen=True)
    return dict(body, sha256=digest(body))


def check_set(satz):
    body = {k: v for k, v in satz.items() if k != 'sha256'}
    boards = satz.get('boards', [])
    if (satz.get('version') != 'lernpruefsatz-v1' or satz.get('ungesehen') is not True or
        satz.get('sha256') != digest(body) or len(boards) != 20 or
        len({b['id'] for b in boards}) != 20 or len({b['familie'] for b in boards}) != 20 or
        not SHA.fullmatch(satz.get('training_familien_sha', '')) or
        any(not SHA.fullmatch(b.get('sha256', '')) for b in boards)):
        raise ValueError('Pruefsatz fehlt, ist nicht versiegelt oder nicht familiengetrennt')
    return {b['id']: b for b in boards}


def snapshot_errors(z, satz):
    errors = []
    if z.get('version') != VERSION or z.get('satz_sha') != satz['sha256']:
        errors.append('falsche Bilanz-/Pruefsatzversion')
    if not all(SHA.fullmatch(z.get(k, '')) for k in ('code_sha', 'modell_sha', 'lenkung_sha', 'evaluator_sha', 'budget_sha')):
        errors.append('vollstaendige Code-/Modell-/Lenkungs-/Evaluator-/Budgetbindung fehlt')
    if z.get('wirksam') is not True:
        errors.append('nur vorbereitet, nicht wirksam')
    rows = z.get('boards', [])
    ids = [r.get('id') for r in rows]
    expected = {b['id']: b for b in satz['boards']}
    if len(ids) != 20 or len(set(ids)) != 20 or set(ids) != set(expected):
        errors.append('leere/unvollstaendige/duplizierte Matrix')
    for r in rows:
        if r.get('board_sha') != expected.get(r.get('id'), {}).get('sha256'):
            errors.append('Boardrevision fehlt/abweichend')
        if not SHA.fullmatch(r.get('kupfer_sha', '')) or not SHA.fullmatch(r.get('drc_sha', '')):
            errors.append('Kupfer-/DRC-Beleg fehlt')
        d = r.get('drc') or {}
        schemas = {'drc-v1': ('fehler', 'offen', 'routing', 'unverbunden'),
                   'korpus-drc-v1': ('alle', 'platz', 'routing', 'unverbunden')}
        keys = schemas.get(d.get('version'), ())
        if not keys or any(type(d.get(k)) is not int or d[k] < 0 for k in keys):
            errors.append('native DRC unbekannt')
        e = r.get('bewertung') or {}
        if type(e.get('gueltig')) is not bool:
            errors.append('Verbindungstor unbekannt')

        if e.get('gueltig') is True and (e.get('minimax_vollstaendig') is not True or
                                        (e.get('sicherheit') or {}).get('sicher') is not True):
            errors.append('Pflichtnachweise unbekannt')
        if e.get('gueltig') is True and not current_minimax(e):
            errors.append('veralteter Minimax-Vertrag')
        if e.get('gueltig') is True:
            errors.extend(measurement_errors(r))
    return sorted(set(errors))

def decision(row):
    return dict(row['bewertung'], regeln=row.get('regeln') or {})


def measurement_errors(row):
    ""
    from analyse import minimax as MM
    from analyse import regelkennzahlen as RK
    e = row['bewertung']
    errors = []
    if not MM.vollstaendig(e):
        errors.append('Minimax-Messgroessen fehlen')
    if not RK.annahme(row.get('regeln') or {}, row.get('regeln') or {})['nicht_schlechter']:
        errors.append('Regelkennzahlen fehlen')
    if any(not number((e.get('minimax_strompfade') or {}).get(k)) for k in ('umweg_mm', 'verlust_mW')):
        errors.append('Minimax-Strompfadschutz fehlt')
    safe = e.get('sicherheit')
    if (not isinstance(safe, dict) or safe.get('sicher') is not True or safe.get('status') != 'sicher'
            or safe.get('fehler') != [] or safe.get('fehlend') != []):
        errors.append('Sicherheitsnachweis fehlt/widerspruechlich')
    if e.get('minimax_gueltig', True) is not True:
        errors.append('Sicherheitstor widerspruechlich')
    if any(k in e and e[k] != row.get('regeln') for k in ('regeln', 'regeltor')):
        errors.append('widerspruechliche Regelkennzahlen')
    return errors


def current_minimax(e):
    ""
    from analyse import minimax as MM
    g = json.loads((ROOT / 'analyse/metriken_gewichte.json').read_text())
    if e.get('note_v3_version', e.get('version')) == 'v3.2':
        si = json.loads((ROOT / 'analyse/metriken_gewichte_si.json').read_text())
        for key in ('minimax', 'profile'):
            si.setdefault(key, g[key])
        g = si
    p = e.get('minimax_profil')
    return p in g['profile'] and e.get('minimax_version') == MM.versionsvertrag(g, p, g.get('version', 'v3.1'), e.get('minimax_boardvertrag'))


def hard_valid(row):
    d = row['drc']
    keys = ('alle', 'platz', 'routing', 'unverbunden') if d['version'] == 'korpus-drc-v1' else ('fehler', 'offen', 'routing', 'unverbunden')
    e = row['bewertung']
    return (e['gueltig'] is True and all(d[k] == 0 for k in keys) and
            e.get('minimax_vollstaendig') is True and (e.get('sicherheit') or {}).get('sicher') is True and
            current_minimax(e) and not measurement_errors(row))


def balance(satz, old, new, now=None):
    t0 = time.process_time()
    report = dict(version=VERSION, status='unbekannt', cpu_s=None,
                  gueltige_je_stunde=None, minimax_prozent_je_stunde=None)
    try:
        check_set(satz)
        errors = snapshot_errors(old, satz) + snapshot_errors(new, satz)
        for k in ('evaluator_sha', 'budget_sha'):
            if old.get(k) != new.get(k):
                errors.append('verschiedener ' + k)
        times = [old.get('entwicklung_start'), old.get('aktiviert'), new.get('entwicklung_start'), new.get('fertig'), new.get('aktiviert')]
        if not all(number(t) for t in times) or not (times[0] <= times[1] <= times[2] <= times[3] <= times[4] <= (now or time.time())):
            errors.append('Lebenszykluszeiten fehlen/ungueltig')
        elif satz['erstellt'] > times[0]:
            errors.append('Satz erst nach Arbeitsbeginn versiegelt')
        if errors:
            report['gruende'] = sorted(set(errors))
            return report
        hours = (new['aktiviert'] - old['entwicklung_start']) / 3600
        if hours <= 0:
            report['gruende'] = ['keine vergangene Zeit']
            return report
        a = {r['id']: r for r in old['boards']}
        b = {r['id']: r for r in new['boards']}
        ga, gb = {k for k in a if hard_valid(a[k])}, {k for k in b if hard_valid(b[k])}
        from analyse import minimax as MM
        improvements, regressions, missing = [], [], []
        lost, gained = sorted(ga - gb), sorted(gb - ga)
        details = [dict(id=k, art='gueltigkeit', grund='hart gueltiges Board verloren') for k in lost]
        for k in sorted(ga & gb):
            ea, eb = a[k]['bewertung'], b[k]['bewertung']
            cmp = MM.annahme(decision(a[k]), decision(b[k]))
            if cmp['delta'] is None or not cmp['nicht_schlechter']:
                (missing if cmp['delta'] is None or 'fehlt' in cmp['grund'] or 'verschiedene' in cmp['grund'] else regressions).append(k)
                if k in missing:
                    continue
                details.append(dict(id=k, art='kennzahlen', grund=cmp['grund'], delta=cmp['delta']))


            x, y = ea['minimax'], eb['minimax']
            if x > 0:
                improvements.append(100 * (x - y) / x)
            elif y == 0:
                improvements.append(0.0)
        if missing:
            report['gruende'] = ['Minimax-/Regelvertrag fehlt auf gemeinsam gueltigen Boards']
            return report
        report.update(status='regression' if lost or regressions else 'gemessen',
                      satz_sha=satz['sha256'], basis_code=old['code_sha'], wirksam_code=new['code_sha'],
                      stunden=hours, gueltig_basis=len(ga), gueltig_wirksam=len(gb),
                      zusaetzlich_gueltig=len(gb) - len(ga), gemeinsam_gueltig=len(ga & gb),
                      gueltige_je_stunde=(len(gb) - len(ga)) / hours,

                      minimax_prozent_je_stunde=min(improvements) / hours if improvements else None,
                      regressionen=len(regressions) + len(lost), regression_details=details,
                      verloren_gueltig=lost, neu_gueltig=gained)
        return report
    except (ValueError, KeyError, TypeError, AttributeError) as e:
        report['gruende'] = [str(e)]
        return report
    finally:
        report['cpu_s'] = time.process_time() - t0

def main():
    if sys.argv[1:2] == ['betrieb']:
        import argparse
        p = argparse.ArgumentParser(description='Lesende Datenertragsbilanz; kein Qualitaetsnachweis')
        p.add_argument('betrieb')
        p.add_argument('daten', type=Path)
        p.add_argument('--start', type=float)
        p.add_argument('--ende', type=float, default=None)
        p.add_argument('--zustand', type=Path)
        args = p.parse_args()
        end = args.ende if args.ende is not None else time.time()
        report = operating_balance(args.daten, args.start if args.start is not None else end - 86400, end)
        if args.zustand:
            report['eingang'] = observe(args.daten, args.zustand, end)
        print(json.dumps(report, ensure_ascii=False, allow_nan=False))
        return 1 if report['status'] == 'unbekannt' else 0
    if len(sys.argv) != 4:
        print(__doc__)
        return 2
    s, a, b = [json.loads(Path(p).read_text()) for p in sys.argv[1:]]
    report = balance(s, a, b)
    print(json.dumps(report, ensure_ascii=False, allow_nan=False))
    return 0 if report['status'] == 'gemessen' else 1


SOURCES = ('dauer.jsonl', 'selbstspiel.jsonl')
STAGES = ('placer', 'router', 'form', 'klassen_eingang', 'klassen_placement',
          'zonenplan', 'stromzonen', 'r11_ir', 'r24', 'r44')
ABORTS = {'zeitlimit', 'wandlimit', 'fehler', 'infrastruktur', 'SIGXCPU',
          'cpu_budget', 'fruehabbruch', 'budget_vor_nachlauf', 'code_geaendert', 'zeit'}


def rate_interval(n, hours):
    ""
    if hours <= 0:
        raise ValueError('positive Exposition erforderlich')

    def quantile(k, target):
        def cdf(mu):
            if mu == 0:
                return 1.0
            return sum(math.exp(-mu + i * math.log(mu) - math.lgamma(i + 1)) for i in range(k + 1))
        lo, hi = 0.0, max(1.0, k + 1.0)
        while cdf(hi) > target:
            hi *= 2
        for _ in range(60):
            mid = (lo + hi) / 2
            if cdf(mid) > target:
                lo = mid
            else:
                hi = mid
        return (lo + hi) / 2
    return [quantile(n - 1, .975) / hours if n else 0.0,
            quantile(n, .025) / hours]


def completed_at(z):
    ""
    value = z.get('zeit')
    if not isinstance(value, str) or not number(z.get('s')):
        return None
    try:
        return datetime.strptime(value, '%Y-%m-%d %H:%M:%S').timestamp() + z['s']
    except ValueError:
        return None


def row_measure(z):
    validate_row(z)
    from lernen.lerneffekt import cpu_s, _grenz_ergebnis
    children = [z.get(k) or {} for k in STAGES]
    stages = [s.get('rechenplatz') or s for s in children]
    states = [z.get('zustand', 'ok')] + [s.get(k) for s in children + stages for k in ('zustand', 'grund')]
    waits = [s['warte_s'] for s in stages if number(s.get('warte_s'))]


    normalized = dict(z)
    for key in STAGES:
        child = dict(z.get(key) or {})
        if child.get('cpu_s') is None and number((child.get('rechenplatz') or {}).get('cpu_s')):
            child['cpu_s'] = child['rechenplatz']['cpu_s']
        if key in ('zonenplan', 'stromzonen', 'r11_ir', 'r24') and not number((child.get('rechenplatz') or {}).get('cpu_s')) and number(child.get('cpu_s')):
            child['rechenplatz'] = dict(child.get('rechenplatz') or {}, cpu_s=child['cpu_s'])
        normalized[key] = child
    cpu_sources = [normalized[k] for k in STAGES] + [z.get(k) or {} for k in
                    ('datensatz', 'etiketten_kandidaten')]
    rb = ((z.get('rennbahn') or {}).get('messung') or {}).get('rechenplatz') or {}
    has_cpu = (number(z.get('cpu_total')) or any(number(c.get('cpu_s')) for c in cpu_sources) or
               number(normalized['router'].get('cpu_eigen_s')) or
               ('cpu_rennbahn_messung_s' not in z and number(rb.get('cpu_s'))))
    cpu = cpu_s(normalized) if has_cpu else None
    aborted = bool(z.get('code_geaendert') or z.get('cpu_zeitlimit') or z.get('budget_ende') or
                   (z.get('lerndaten') or {}).get('zensiert') or
                   (z.get('cpu_budget') or {}).get('ueberlauf', 0) or
                   z.get('fehler') or z.get('regel_fehler') or
                   any(s in ABORTS or any(s.endswith('_' + a) for a in ABORTS)
                       for s in states if s) or any(s.get('fehler') for s in children))
    return dict(gueltig=int(not aborted and _grenz_ergebnis(z) is True),
                abbruch=int(aborted),
                wandlimit=int(any(s == 'wandlimit' or s.endswith('_wandlimit') for s in states if s)),
                cpu_s=cpu if has_cpu else None,
                cpu_voll=int(z.get('cpu_messung') in ('voll', 'voll_wait4')),
                warte_s=sum(waits), warte_belege=len(waits),
                last=z.get('last') if number(z.get('last')) else None)


def validate_row(z):
    ""
    def obj(value, label):
        if value is None:
            return {}
        if not isinstance(value, dict):
            raise ValueError('Zeilenschema: ' + label + ' muss Objekt sein')
        return value

    def fields(value, label):
        for key in ('cpu_s', 'cpu_eigen_s', 'warte_s', 'ueberlauf'):
            if key in value and (key == 'ueberlauf' or value[key] is not None) and not number(value[key]):
                raise ValueError('Zeilenschema: ' + label + '.' + key)
        for key in ('zustand', 'grund'):
            if value.get(key) is not None and not isinstance(value[key], str):
                raise ValueError('Zeilenschema: ' + label + '.' + key)
        if 'cpu_enthaelt' in value and (not isinstance(value['cpu_enthaelt'], list) or
                                        any(not isinstance(k, str) for k in value['cpu_enthaelt'])):
            raise ValueError('Zeilenschema: ' + label + '.cpu_enthaelt')
        if 'rechenplatz' in value:
            if not isinstance(value['rechenplatz'], dict):
                raise ValueError('Zeilenschema: ' + label + '.rechenplatz')
            fields(obj(value['rechenplatz'], label + '.rechenplatz'), label + '.rechenplatz')

    obj(z, 'Zeile')
    if not isinstance(z, dict):
        raise ValueError('Zeilenschema: Zeile muss Objekt sein')
    fields(z, 'Zeile')
    for key in ('cpu_total', 'cpu_rennbahn_messung_s', 's', 'last'):
        if key in z and z[key] is not None and not number(z[key]):
            raise ValueError('Zeilenschema: ' + key)
    for key in ('gueltig', 'cpu_zeitlimit', 'budget_ende'):
        if z.get(key) is not None and type(z[key]) is not bool:
            raise ValueError('Zeilenschema: ' + key)
    if z.get('zeit') is not None and not isinstance(z['zeit'], str):
        raise ValueError('Zeilenschema: zeit')
    for key in STAGES + ('datensatz', 'etiketten_kandidaten', 'lerndaten', 'cpu_budget'):
        fields(obj(z.get(key), key), key)
    form = z.get('form') or {}
    if 'angewendet' in form and type(form['angewendet']) is not bool:
        raise ValueError('Zeilenschema: form.angewendet')
    ld = z.get('lerndaten') or {}
    if ld.get('zensiert') is not None and type(ld['zensiert']) is not bool:
        raise ValueError('Zeilenschema: lerndaten.zensiert')
    rb = obj(z.get('rennbahn'), 'rennbahn')
    fields(obj(rb.get('messung'), 'rennbahn.messung'), 'rennbahn.messung')


def empty_counts():
    return dict(zeilen=0, gueltig=0, abbruch=0, wandlimit=0, cpu_s=0.0,
                cpu_unbekannt=0, cpu_voll=0, warte_s=0.0, warte_belege=0,
                last_n=0, last_summe=0.0, last_ueber_kerne_n=0)


def add_row(counts, z, cores):
    m = row_measure(z)
    counts['zeilen'] += 1
    for k in ('gueltig', 'abbruch', 'wandlimit', 'cpu_voll', 'warte_s', 'warte_belege'):
        counts[k] += m[k]
    if m['cpu_s'] is None:
        counts['cpu_unbekannt'] += 1
    else:
        counts['cpu_s'] += m['cpu_s']
    if m['last'] is not None:
        counts['last_n'] += 1
        counts['last_summe'] += m['last']
        counts['last_ueber_kerne_n'] += int(m['last'] > cores)


def finish_counts(counts, hours):
    result = dict(counts)
    result.update(gueltige_je_stunde=counts['gueltig'] / hours,
                  gueltige_je_stunde_ci95=rate_interval(counts['gueltig'], hours),
                  cpu_je_gueltig_untergrenze_s=counts['cpu_s'] / counts['gueltig'] if counts['gueltig'] else None,
                  abbruch_anteil=counts['abbruch'] / counts['zeilen'] if counts['zeilen'] else None,
                  last_mittel=counts['last_summe'] / counts['last_n'] if counts['last_n'] else None,
                  last_ueber_kerne_anteil=counts['last_ueber_kerne_n'] / counts['last_n'] if counts['last_n'] else None)
    return result


def complete_lines(path, offset=0):
    ""
    with path.open('rb') as f:
        yield from descriptor_lines(f, os.fstat(f.fileno()).st_size, offset)


def descriptor_lines(f, end, offset=0, consumed=None):
    f.seek(offset)
    while f.tell() < end:
        line = f.readline(end - f.tell())
        if not line.endswith(b'\n'):
            break
        if consumed is not None:
            consumed.update(line)
        try:
            z = json.loads(line)
            validate_row(z)
        except (ValueError, UnicodeDecodeError, RecursionError):
            z = None
        yield f.tell(), z


def prefix_sha(f, end, accumulator=None):
    ""
    f.seek(0)
    h = accumulator if accumulator is not None else hashlib.sha256()
    remaining = end
    while remaining:
        chunk = f.read(min(remaining, 1024 * 1024))
        if not chunk:
            return None
        h.update(chunk)
        remaining -= len(chunk)
    return h.hexdigest()


def operating_balance(data, start, end, cores=None):
    ""
    import os
    if not number(start) or not number(end) or end <= start:
        raise ValueError('ungueltiges Messfenster')
    hours = (end - start) / 3600
    sources = {}
    for name in SOURCES:
        counts = empty_counts()
        hourly = [0] * math.ceil(hours)
        untimed = total = malformed = offset = 0
        path = Path(data) / name
        try:
            rows = complete_lines(path)

            rows = iter(rows)
            first = next(rows, None)
        except FileNotFoundError:
            sources[name] = dict(status='fehlt', gueltige_je_stunde=None)
            continue
        from itertools import chain
        for offset, z in chain(() if first is None else (first,), rows):
            if z is None:
                malformed += 1
                continue
            total += 1
            t = completed_at(z) if name == 'selbstspiel.jsonl' else None
            if t is None:
                untimed += 1
            elif start <= t < end:
                add_row(counts, z, cores or os.cpu_count() or 1)
                hourly[int((t - start) // 3600)] += row_measure(z)['gueltig']
        sources[name] = dict(finish_counts(counts, hours),
                             status='teilweise' if untimed or malformed else 'gemessen',
                             zeilen_datei=total, ohne_zeit=untimed, defekt=malformed,
                             prefix_bytes=offset, zeitbasis='Startzeit+s; vor Galerie/Append' if name.startswith('selbst') else 'kein historischer Zeitbeleg')


        if hours >= 6 and hours == int(hours) and name == 'selbstspiel.jsonl':
            import random
            rnd = random.Random(14010)
            draws = sorted(sum(rnd.choice(hourly) for _ in hourly) / hours for _ in range(2000))
            sources[name]['stundenblock_ci95'] = [draws[49], draws[1949]]
            sources[name]['stunden_gueltig'] = hourly
        if name == 'dauer.jsonl' and untimed:
            for key in ('gueltige_je_stunde', 'gueltige_je_stunde_ci95'):
                sources[name][key] = None
    missing = [name for name in SOURCES if sources[name]['status'] == 'fehlt']
    return dict(version='lernbetrieb-v1', status='unbekannt' if len(missing) == len(SOURCES) else 'Datenertrag',
                fehlende_quellen=missing, messgrundlage='fehlt' if len(missing) == len(SOURCES) else
                'teilweise' if missing or any(s['status'] == 'teilweise' for s in sources.values()) else 'vollstaendig',
                start=start, ende=end,
                stunden=hours, quellen=sources, qualitaetsgewinn=None,
                band_annahme='95% Poisson, unabhaengige Ereignisse; Kandidaten sind korreliert. Kein kausales Wirkungsband.',
                grenzen=['CPU ist historische Untergrenze; nicht gespeicherte/aktive Versuche fehlen.',
                         'Last ist ereignisgewichtete Systemstichprobe, keine Zeitreihe oder Doppelbelegungszahl.',
                         'Wartezeit nur explizite Felder; Zulassungs-/Reservewartezeit und verworfene Kinder fehlen.'])


def observe(data, state_path, now=None, cores=None):
    ""
    import os
    import fcntl
    now = time.time() if now is None else now
    data, state_path = Path(data).resolve(), Path(state_path).resolve()
    if state_path == data or data in state_path.parents:
        raise ValueError('Beobachterzustand darf nicht in Lerndaten liegen')
    state_path.parent.mkdir(parents=True, exist_ok=True)
    with Path(str(state_path) + '.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        old = json.loads(state_path.read_text()) if state_path.exists() else None
        if old and (old.get('version') not in ('lerneingang-v1', 'lerneingang-v2') or
                    old.get('daten') != str(data) or now <= old['epoch']):
            raise ValueError('Beobachterstand/Quelle/Zeit stimmt nicht')
        cursors, sources = {}, {}
        for name in SOURCES:
            path = data / name
            try:
                f = path.open('rb')
            except FileNotFoundError:
                sources[name] = dict(status='fehlt')
                continue
            with f:
                st = os.fstat(f.fileno())
                prev = (old or {}).get('cursors', {}).get(name)
                identity = [st.st_dev, st.st_ino]
                reset = (not prev or prev['identity'] != identity or st.st_size < prev['offset'] or
                         not prev.get('prefix_sha256') or prefix_sha(f, prev['offset']) != prev['prefix_sha256'])
                counts = empty_counts()
                offset = st.st_size if reset else prev['offset']
                malformed = 0
                if reset:

                    while offset > 0:
                        f.seek(offset - 1)
                        if f.read(1) == b'\n': break
                        offset -= 1
                else:
                    consumed = hashlib.sha256()
                    prefix_sha(f, offset, consumed)
                    for offset, z in descriptor_lines(f, st.st_size, offset, consumed):
                        if z is None: malformed += 1
                        else: add_row(counts, z, cores or os.cpu_count() or 1)
                binding = prefix_sha(f, offset)


                if not reset and (binding != consumed.hexdigest() or
                                  prefix_sha(f, prev['offset']) != prev['prefix_sha256']):
                    reset = True
                    binding = None
            cursors[name] = dict(identity=identity, offset=offset, prefix_sha256=binding)
            sources[name] = (dict(status='ausgangsstand' if not old else 'luecke_dateiwechsel') if reset else
                             dict(finish_counts(counts, (now - old['epoch']) / 3600),
                                  status='teilweise' if malformed else 'gemessen', defekt=malformed))
        state = dict(version='lerneingang-v2', daten=str(data), epoch=now, cursors=cursors)
        temp = Path(str(state_path) + '.tmp')
        temp.write_text(json.dumps(state, allow_nan=False))
        temp.replace(state_path)
        return dict(version='lerneingang-v2', start=old['epoch'] if old else None, ende=now,
                    zeitbasis='neu angehaengte abgeschlossene Zeilen seit Ausgangsaufnahme', quellen=sources)


if __name__ == '__main__':
    sys.exit(main())
