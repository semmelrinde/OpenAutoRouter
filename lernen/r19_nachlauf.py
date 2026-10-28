""
import copy
import json
import os
from pathlib import Path
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def messen(B, R, K, gl):
    from analyse.bewertung import bewerte
    from lernen.form import gueltig
    from lernen.sammeln import regelmessung
    from analyse.wackel import messen as wackel
    e = bewerte(B, dict(R, gndlagen=list(gl)), K, v3=True, minimax=False)
    good, _ = gueltig(B, K, None, gl, routes=R, minimax=False)
    return dict(gueltig=good and e['gueltig'], regeln=e['regeln'],
                lernregeln=regelmessung(B, R, K, gl)['regeln'],
                wackel=wackel(B, R, gnd=K.get('gnd') or []))


def tor(a, b):
    from analyse.regelkennzahlen import annahme
    gates = [annahme(a[key], b[key]) for key in ('regeln', 'lernregeln')]
    form = b['wackel']['zuege'] <= a['wackel']['zuege'] and b['wackel']['laenge_mm'] <= a['wackel']['laenge_mm']
    return dict(bestanden=bool(a['gueltig'] and b['gueltig'] and form and all(g['nicht_schlechter'] for g in gates)),
                regeln=[dict(bestanden=g['nicht_schlechter'], grund=g['grund']) for g in gates], form=form)


def optimieren(B, R, K, rules, gl):
    from analyse.standard import ziel
    if ziel(K.get('vorgaben')) != 'v3':
        return R, dict(angewendet=False, grund='Nur fuer ausdruecklichen v3-Vertrag')
    from nachbearbeitung.r19 import vorschlag
    candidate, info = vorschlag(B, R, K, rules)
    info.update(angewendet=False, versuche=[])
    if not info['aenderungen']:
        info['grund'] = 'Keine freie Abkuerzung'
        return R, info
    a = messen(B, R, K, gl); info['vorher'] = a
    if not a['gueltig']:
        info['grund'] = 'Ungueltige Basis'
        return R, info
    def pruefen(trial, net):
        from analyse.laengenimpedanz import laengen, empfaenger_schutz, empfaenger_klassen
        lk = empfaenger_klassen(K)
        if not empfaenger_schutz(laengen(B, R, lk), laengen(B, trial, lk)):
            return trial, dict(netz=net, bestanden=False, grund='Empfaengertor'), None
        b = messen(B, trial, K, gl); result = dict(tor(a, b), netz=net)
        return trial, result, b
    result, gate, b = pruefen(candidate, '*')
    info['versuche'].append(gate)
    if gate['bestanden']:
        info.update(angewendet=True, nachher=b, grund='Regel- und Formtor bestanden')
        return result, info


    gains = {}
    for c in info['aenderungen']:
        gains[c['net']] = gains.get(c['net'], 0.)+c['laenge_vorher']-c['laenge_nachher']
    current = R
    for net in sorted(gains, key=lambda n: (-gains[n], n))[:4]:
        trial = copy.deepcopy(current)
        for i, t in enumerate(candidate['tracks']):
            if t['net'] == net: trial['tracks'][i] = copy.deepcopy(t)
        result, gate, b = pruefen(trial, net); info['versuche'].append(gate)
        if gate['bestanden']:
            current = result; a = b
            info.update(angewendet=True, nachher=b)
    info['grund'] = 'Einzelnetze angenommen' if info['angewendet'] else 'Regeltor sperrt Abkuerzungen'
    return current, info


def anwenden(bj, kj, rj, rules, gl, cpu_s=180):
    ""
    from gemeinsam import plaetze as PL
    original = Path(rj).read_bytes()
    with tempfile.TemporaryDirectory(prefix='r19_nachlauf_') as folder:
        out, meta = Path(folder)/'routes.json', Path(folder)/'info.json'
        run = PL.lauf([str(ROOT/'.venv/bin/python'), str(Path(__file__)), 'worker',
                       str(bj), str(kj), str(rj), str(rules), json.dumps(gl), str(out), str(meta)],
                      cpu_s=cpu_s, log=str(Path(folder)/'worker.log'), cwd=str(ROOT),
                      env=dict(os.environ, PYTHONHASHSEED='0'))
        info = json.loads(meta.read_text()) if run['zustand'] == 'ok' and meta.exists() else dict(
            angewendet=False, grund='Worker '+run['zustand'])
        info['rechenplatz'] = run
        if info['angewendet']:
            if Path(rj).read_bytes() != original:
                info.update(angewendet=False, grund='Route waehrend Nachlauf geaendert')
                return info
            Path(rj).with_name(Path(rj).stem+'_vor_r19.json').write_bytes(original)
            with tempfile.NamedTemporaryFile(dir=Path(rj).parent, delete=False) as f:
                tmp = f.name; f.write(out.read_bytes())
            os.replace(tmp, rj)
        return info


if __name__ == '__main__':
    _, bj, kj, rj, rules, gl, out, meta = sys.argv[1:]
    t0 = time.process_time()
    B, K, R, rg = [json.loads(Path(p).read_text()) for p in (bj, kj, rj, rules)]
    result, info = optimieren(B, R, K, rg, json.loads(gl))
    info['cpu_s'] = time.process_time()-t0
    Path(out).write_text(json.dumps(result)); Path(meta).write_text(json.dumps(info))
