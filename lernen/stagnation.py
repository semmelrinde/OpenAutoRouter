""
import argparse
from datetime import datetime, timezone
import fcntl
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time

FENSTER_S = 48 * 3600
REST_SENKUNG = .10
NETZ_SENKUNG = .05
GUELTIG_ZUWACHS = 1
MESSABSTAND_S = 24 * 3600
VORWOCHE_S = 7 * 86400
ROOT = Path(__file__).resolve().parents[1]


def epoch(value):
    if type(value) in (int, float) and math.isfinite(value):
        return float(value)
    dt = datetime.fromisoformat(value)
    return dt.timestamp()


def zahl(value):
    return type(value) in (int, float) and math.isfinite(value) and value >= 0


def lesen(path, fehler):
    if not path.exists():
        return []
    try:
        if path.suffix == '.jsonl':
            out = []
            for n, line in enumerate(path.read_text().splitlines(), 1):
                if not line.strip():
                    continue
                try:
                    out.append(json.loads(line))
                except ValueError:
                    fehler.append(f'{path}:{n}: unlesbare Zeile')
            return out
        return [json.loads(path.read_text())]
    except (OSError, ValueError) as exc:
        fehler.append(f'{path}: {exc}')
        return []


def verlauf(root, now):
    ""
    fehler = []
    serien = dict(lernkurve=[], bga=[], v12=[])

    def add(name, row, quelle):
        try:
            t = epoch(row['zeit'])
            g = row['gueltig']
            if not zahl(g) or int(g) != g or t > now:
                raise ValueError('Zeit/Gueltigkeit ungueltig')
            from lernen.lernkurve import punkt_unvollstaendig
            if punkt_unvollstaendig(row):
                raise ValueError('Gueltigkeit teilweise unbekannt')
            n = row.get('aufgaben')
            if n is not None and (not zahl(n) or n < g):
                raise ValueError('Aufgabenzahl ungueltig')
            if name == 'lernkurve' and n != 15:
                raise ValueError('keine feste 15er-Reihe')
            if name == 'bga' and n != 42:
                raise ValueError('keine volle 42er-Reihe')
            for k in ('restkonflikte', 'schlechtestes_netz'):
                if row.get(k) is not None and not zahl(row[k]):
                    raise ValueError(k + ' ungueltig')

            if row.get('cpu_je_gueltig') is not None and not zahl(row['cpu_je_gueltig']):
                fehler.append(f'{quelle}: cpu_je_gueltig ungueltig (nur Nebenzahl)')
                row = dict(row, cpu_je_gueltig=None)
            serien[name].append(dict(row, epoch=t, quelle=str(quelle)))
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            fehler.append(f'{quelle}: {exc}')

    path = root / 'lernen/daten/lernkurve.jsonl'
    for row in lesen(path, fehler):
        if not isinstance(row, dict):
            fehler.append(f'{path}: kein Objekt')
            continue
        if row.get('satz', 'lernkurve') != 'lernkurve':
            continue
        row = dict(row, vertrag=json.dumps([row.get(k) for k in
                   ('generatorversion', 'aufgaben_sha', 'bewertungsziel', 'messvertrag')]))
        add('lernkurve', row, path)

    bga_root = Path(os.environ.get('STAGNATION_BGA_ERGEBNISSE', root / 'codex/bga_fanout/ergebnisse'))
    for path in sorted(bga_root.glob('**/bilanz.json')):
        try:
            c = json.loads(path.with_name('vertrag.json').read_text())
            b = json.loads(path.read_text())['gesamt']

            if c.get('format') != 'bga-fanout-v1' or c.get('arm') != 'basis':
                continue
            if sorted(c['indices']) != list(range(42)):
                continue
            contract = json.dumps({k: c.get(k) for k in
                ('format', 'indices', 'aufgaben', 'cpu_phase', 'iter', 'cfg', 'arm')}, sort_keys=True)
            add('bga', dict(zeit=path.stat().st_mtime, gueltig=b['gueltig'],
                aufgaben=b['n'], unbekannt=b['unbekannt'], code=c.get('commit'), vertrag=contract), path)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            fehler.append(f'{path}: {exc}')
    path = root / 'codex/v12/verlauf.jsonl'
    for row in lesen(path, fehler):
        if not isinstance(row, dict) or not row.get('vertrag'):
            fehler.append(f'{path}: Referenzboard-Vertrag fehlt')
            continue
        add('v12', row, path)
    for name, rows in serien.items():
        rows.sort(key=lambda r: r['epoch'])

        if rows:
            contract = rows[-1]['vertrag']
            serien[name] = [r for r in rows if r['vertrag'] == contract]
    return serien, fehler


def entscheidung(serien, now):
    ""
    events = []
    start = []
    details = {}
    for name, rows in serien.items():
        if not rows:
            details[name] = dict(status='fehlt')
            continue
        start.append(rows[0]['epoch'])
        best_g = rows[0]['gueltig']

        anchors = {}
        for index, row in enumerate(rows):
            g = row['gueltig']
            reasons = []
            if g >= best_g + GUELTIG_ZUWACHS:
                reasons.append('+1 gueltig')
            if g >= best_g:
                metrics = ([('restkonflikte', REST_SENKUNG),
                            ('schlechtestes_netz', NETZ_SENKUNG)] if name == 'v12' else [])
                for key, fraction in metrics:
                    value = row.get(key)
                    if value is None:
                        continue
                    anchor = anchors.get((g, key))
                    recent = [r.get(key) for r in rows[:index]
                              if row['epoch'] - FENSTER_S <= r['epoch'] < row['epoch']
                              and r['gueltig'] == g and r.get(key) is not None]
                    window_gain = any(v > 0 and value <= v * (1 - fraction) for v in recent)
                    if anchor is not None and anchor > 0 and value <= anchor * (1 - fraction) and window_gain:
                        reasons.append(key)
                        anchors[g, key] = value
                    elif anchor is None:
                        anchors[g, key] = value
            best_g = max(best_g, g)
            if reasons:
                events.append(dict(epoch=row['epoch'], reihe=name, gruende=reasons))
        details[name] = dict(status='aktuell' if now - rows[-1]['epoch'] <= MESSABSTAND_S else 'veraltet',
                             aktuell=rows[-1], messungen=len(rows))
    last = max((e['epoch'] for e in events), default=min(start) if start else None)
    fresh = any(d['status'] == 'aktuell' for d in details.values())
    covered = any(rows and now - rows[0]['epoch'] >= FENSTER_S
                  and now - rows[-1]['epoch'] <= MESSABSTAND_S for rows in serien.values())
    status = ('unbekannt' if last is None or not fresh else
              'verbesserung' if any(now - e['epoch'] < FENSTER_S for e in events) else
              'stillstand' if covered and now - last >= FENSTER_S else 'beobachtung')
    basis_rows = [r for rows in serien.values() for r in rows if r['epoch'] == last]
    code = next((r.get('code') for r in basis_rows if r.get('code')), None)
    return dict(code_bezug=code, status=status, letzte_verbesserung_epoch=max((e['epoch'] for e in events), default=None),
                bezug_epoch=last, reihen=details, verbesserungen=events)


def bericht(serien, now):
    def current(name):
        return serien[name][-1] if serien[name] else {}

    def week(name):
        return next((r for r in reversed(serien[name]) if r['epoch'] <= now - VORWOCHE_S), {})

    def fmt(row, key):
        x = row.get(key)
        return 'unbekannt' if x is None else f'{x:g}'

    a, w, b, v, vw = current('lernkurve'), week('lernkurve'), current('bga'), current('v12'), week('v12')
    line = (f"Kurve: 15er {fmt(a, 'gueltig')}/15 (Vorwoche {fmt(w, 'gueltig')}/15), "
            f"CPU je gültigem {fmt(a, 'cpu_je_gueltig')} s (Vorwoche {fmt(w, 'cpu_je_gueltig')} s); "
            f"v12-BGA {fmt(b, 'gueltig')}/42; Referenzboard Restkonflikte {fmt(v, 'restkonflikte')} "
            f"(Vorwoche {fmt(vw, 'restkonflikte')})")
    stale = [name for name in serien if serien[name] and now - serien[name][-1]['epoch'] > MESSABSTAND_S]
    return line + ('; veraltet: ' + ', '.join(stale) if stale else '')


def atomic(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + '.tmp')
    temp.write_text(text)
    temp.replace(path)


def vorziehen(queue):
    ""
    source = queue / 'zurueckgestellt'
    target = queue / 'warten'
    if not source.exists():
        return None
    target.mkdir(parents=True, exist_ok=True)
    for p in sorted(source.iterdir()):
        if not p.is_file() or p.is_symlink() or not re.fullmatch(r'0[0-9x]_strategie_.*\.txt', p.name):
            continue
        dest = target / ('00_' + p.name[3:])
        try:
            os.link(p, dest)
        except FileExistsError:
            continue
        p.unlink()
        return str(dest)
    return None


def pruefen(root=ROOT, queue=None, now=None):
    now = time.time() if now is None else now
    queue = Path(queue or Path.home() / 'Library/Caches/Autorouter/codex_auftraege')
    data = root / 'lernen/daten'
    data.mkdir(parents=True, exist_ok=True)
    with (data / '.stagnation.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        serien, fehler = verlauf(root, now)
        result = entscheidung(serien, now)
        result.update(zeit=datetime.fromtimestamp(now, timezone.utc).isoformat(), lesefehler=fehler)
        state = data / 'stagnation.json'
        old = lesen(state, fehler)
        if result['status'] != 'stillstand':
            return result

        if old and isinstance(old[0], dict) and old[0].get('bezug_epoch') == result['bezug_epoch']:
            return result
        since = datetime.fromtimestamp(result['bezug_epoch'], timezone.utc).isoformat()
        code = result.get('code_bezug')
        revision = None
        if isinstance(code, str) and re.fullmatch(r'[0-9a-f]{7,40}', code):
            resolved = subprocess.run(['git', 'rev-parse', '--verify', code + '^{commit}'],
                                      cwd=root, capture_output=True, text=True)
            if resolved.returncode == 0:
                revision = code + '..HEAD'
        result['commit_zuordnung'] = 'revision' if revision else 'zeitlich'
        cmd = ['git', 'log', '--format=%H %s'] + ([revision] if revision else ['--since=' + since, 'HEAD'])
        git = subprocess.run(cmd, cwd=root, capture_output=True, text=True)
        result['code_commits_seit_verbesserung'] = git.stdout.splitlines() if git.returncode == 0 else None
        result['git_fehler'] = git.stderr.strip() if git.returncode else None

        atomic(state, json.dumps(result, indent=2, ensure_ascii=False) + '\n')
        result['vorgezogen'] = vorziehen(queue)
        atomic(state, json.dumps(result, indent=2, ensure_ascii=False) + '\n')
        day = datetime.fromtimestamp(now).strftime('%Y-%m-%d')
        text = (f'# Stagnation {day}\n\nSeit 48 h kein Schwellenfortschritt in den vergleichbaren Messungen.\n\n'
                + bericht(serien, now) + '\n\nReihenstatus: '
                + ', '.join(f"{k}: {v['status']}" for k, v in result['reihen'].items())
                + '\n\nCode seit letzter Verbesserung (sonst seit erster Messung):\n'
                + ('\n'.join(result['code_commits_seit_verbesserung'] or []) or 'Keine ermittelt; siehe JSON.')
                + '\n\nVorgezogen: ' + str(result['vorgezogen'])
                + '\n\nLesefehler: ' + json.dumps(fehler, ensure_ascii=False) + '\n')
        atomic(root / f'wissen/stagnation_{day}.md', text)
        return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('aktion', nargs='?', choices=('pruefen', 'bericht'), default='pruefen')
    p.add_argument('--root', type=Path, default=Path(os.environ.get('LERNKURVE_HAUPT', ROOT)))
    p.add_argument('--queue', type=Path)
    a = p.parse_args()
    if a.aktion == 'bericht':
        serien, fehler = verlauf(a.root, time.time())
        print(bericht(serien, time.time()) + ('; Lesefehler: ' + str(len(fehler)) if fehler else ''))
    else:
        print(json.dumps(pruefen(a.root, a.queue), ensure_ascii=False))


if __name__ == '__main__':
    main()
