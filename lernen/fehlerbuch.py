""
import collections, json, os, sys, time
from pathlib import Path

R = Path(__file__).resolve().parents[1]
Q = Path.home() / 'Library/Caches/Autorouter/codex_auftraege'


def ursache(r):
    g = r.get('ungueltig_grund') or ''
    z = r.get('zustand') or ''
    if r.get('gueltig'): return 'gueltig'
    if g.startswith('Platzierung') or z == 'placer_kollision': return 'platzierung_kollision'
    if 'zeitlimit' in z and 'placer' in z: return 'placer_zeitlimit'
    if 'Geometrie' in g: return 'zonen_geometrie'
    if z in ('arbeitsmessung_unvollstaendig', 'native_basis_unvollstaendig') or 'unvollstaendig' in z: return 'messung_unvollstaendig'
    if z == 'fehler' or 'ohne Teilergebnis' in g: return 'router_fehler'
    k = r.get('kennzahlen') or (r.get('ergebnis') or {})
    if (k.get('konflikte') or 0) > 0: return 'router_konflikte'
    if (k.get('offen') or 0) > 0: return 'router_offen'

    nm = r.get('native_meister') or {}
    if nm.get('fehler') or nm.get('offen'):
        arten = nm.get('arten') or {}
        if arten: return 'nativ_' + max(sorted(arten), key=lambda a: arten[a])
        return 'nativ_offen' if not nm.get('fehler') else 'nativ_unbekannt'
    return 'router_sonstig' if g else (z or 'unbekannt')


def auswerten(zeilen):
    out = collections.OrderedDict()
    n = len(zeilen) or 1
    cpu_ges = sum(float(r.get('cpu_total') or 0) for r in zeilen) or 1.0
    grp = collections.defaultdict(list)
    for r in zeilen: grp[ursache(r)].append(r)
    for u, rs in sorted(grp.items(), key=lambda x: -len(x[1])):
        cpu = sum(float(r.get('cpu_total') or 0) for r in rs)
        out[u] = dict(anzahl=len(rs), anteil=round(len(rs) / n, 3), cpu_anteil=round(cpu / cpu_ges, 3),
                      klassen=collections.Counter(r.get('klasse') for r in rs).most_common(5))

    arme = collections.defaultdict(lambda: [0, 0])
    for r in zeilen:
        for k, v in sorted((r.get('cfg') or {}).items()):
            if k in ('seed',) or isinstance(v, (dict, list)): continue
            a = arme['%s=%s' % (k, v)]; a[0] += 1; a[1] += 0 if r.get('gueltig') else 1
    arm = {k: dict(n=a[0], ausfall=round(a[1] / a[0], 3)) for k, a in sorted(arme.items()) if a[0] >= 20}
    return out, arm


VORLAGEN = {
    'platzierung_kollision': ('placer_kollisionen', 'Placer/Legalisierung: Im Selbstspiel enden {anteil:.0%} der Laeufe mit Restkollisionen '
        'nach Placer-Wiederholungen und legal_lp (typisch 1-4 Kollisionen, auch auf leichten Stufen; Klassen {klassen}). '
        'Ziel: Restkollisionen allgemein beseitigen (z. B. gezieltes Verschieben/Drehen der kollidierenden Teile in freien Raum, '
        'MILP-Fenster, groesserer Wanderradius nur fuer kollidierende Teile), ohne Qualitaetsverlust. Messen: Selbstspiel-Stichprobe '
        'und feste 15er (v10/v11, budget=arbeit): Anteil kollisionsfreier Platzierungen, gueltige Boards, kein Rueckschritt.'),
    'placer_zeitlimit': ('placer_zeit', 'Placer-Zeitlimit: {anteil:.0%} der Laeufe enden im Placer-Zeitlimit (Klassen {klassen}). '
        'Ursache finden (welcher Schritt), beschleunigen oder Budget sinnvoll staffeln; kein Rueckschritt.'),
    'router_konflikte': ('router_konflikte', 'Router-Restkonflikte: {anteil:.0%} der Laeufe enden mit Konflikten (Klassen {klassen}). '
        'Haeufigste Konfliktpaare/-orte aus den Laeufen auswerten und die groesste Gruppe allgemein loesen.'),
    'router_offen': ('router_offen', 'Router offene Verbindungen: {anteil:.0%} der Laeufe enden mit unerreichbaren Pins (Klassen {klassen}). '
        'Pinzugang/Escape fuer die haeufigsten Bauformen allgemein loesen.'),
    'router_fehler': ('router_fehler', 'Router-Abbrueche: {anteil:.0%} der Laeufe enden ohne Teilergebnis. Ursache je Abbruchart beheben.'),
    'nativ_unbekannt': ('native_luecke', 'Native Luecke: {anteil:.0%} der Laeufe sind intern gueltig, aber KiCad meldet Fehler/Offenes '
        '(Klassen {klassen}). Fehlerarten je Lauf stehen erst ab E96 in native_meister.arten; haeufigste Art finden und im '
        'Router-/Exportvertrag allgemein beheben, nicht im Pruefer lockern.'),
    'zonen_geometrie': ('zonen_geometrie', 'Zonen-Geometriefehler: {anteil:.0%} der Laeufe. Ursache beheben.'),
}


def auftrag_anlegen(u, info, fenster):
    if u not in VORLAGEN: return None
    marke, text = VORLAGEN[u]
    for ordner in ('warten', 'laufen'):
        if any(marke in p.name for p in (Q / ordner).glob('*.txt')): return 'schon eingereiht'

    if any(marke in p.name and time.time() - p.stat().st_mtime < 2 * 86400 for p in (Q / 'fertig').glob('*')):
        return 'kuerzlich bearbeitet'
    kl = ', '.join('%s %d' % (k, n) for k, n in info['klassen'])
    body = ('aufwand: high\nTEIL %s · NEU (automatisch aus lernen/fehlerbuch.py, %s; Fenster %d Selbstspiel-Laeufe). '
            'Neue Arbeitskopie von main. Kein Push. Bericht wissen/codex_%s.md.\n'
            'DATENSCHUTZ (hart): Technische Sperre aktiv (macOS-Sandbox). Erlaubt sind nur Repository, erzeugte Aufgaben, '
            'H6/PCBWorld und Testboard.\n%s\nUebernahmeregel (Nutzer): mehr gueltige Boards ohne Verlust gewinnt; '
            'Ergebnis als Option liefern, Claude uebernimmt nach Vorprobe.\n<= 4 Plaetze.\n') % (
        marke, time.strftime('%d.%m. %H:%M'), fenster, marke, text.format(anteil=info['anteil'], klassen=kl))
    ziel = Q / 'warten' / ('03_%s.txt' % marke)
    ziel.write_text(body)
    return str(ziel)


def main():
    kw = dict(a.split('=', 1) for a in sys.argv[1:] if '=' in a)
    fenster = int(kw.get('fenster', '600'))
    alle = [json.loads(l) for l in open(R / 'lernen/daten/selbstspiel.jsonl') if l.strip()]
    jetzt, vorher = alle[-fenster:], alle[-2 * fenster:-fenster]
    u_jetzt, arme = auswerten(jetzt)
    u_vorher, _ = auswerten(vorher)
    for u, info in u_jetzt.items():
        info['trend'] = round(info['anteil'] - u_vorher.get(u, {}).get('anteil', 0.0), 3)
    kosten = [(info['anteil'] * max(info['cpu_anteil'], 0.01), u) for u, info in u_jetzt.items() if u != 'gueltig']
    top = max(kosten)[1] if kosten else None
    erg = dict(zeit=time.strftime('%Y-%m-%d %H:%M'), fenster=len(jetzt), ursachen=u_jetzt,
               verschwendete_arme={k: v for k, v in arme.items() if v['ausfall'] >= 0.5}, top=top)
    if kw.get('auftrag', '0') == '1' and top:
        erg['auftrag'] = auftrag_anlegen(top, u_jetzt[top], len(jetzt))
    ziel = Path(kw.get('ziel', R / 'lernen/daten/fehlerbuch.json'))
    ziel.write_text(json.dumps(erg, ensure_ascii=False, indent=1))
    for u, info in u_jetzt.items():
        print('%-24s %4d  %5.1f %%  CPU %5.1f %%  Trend %+5.1f %%  %s' % (u, info['anzahl'], 100 * info['anteil'], 100 * info['cpu_anteil'],
              100 * info['trend'], ', '.join('%s %d' % k for k in info['klassen'][:3])))
    print('Verschwendete Arme (Ausfall >= 50 %):', ', '.join('%s %.0f %%' % (k, 100 * v['ausfall']) for k, v in erg['verschwendete_arme'].items()) or '-')
    print('Top-Ursache:', top, '| Auftrag:', erg.get('auftrag'))


if __name__ == '__main__':
    main()
