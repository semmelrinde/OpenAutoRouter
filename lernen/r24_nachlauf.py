""
import json
import math
import os
import shutil
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def anwenden(bj, kj, rj, regeln, *, cpu_s=180):
    from gemeinsam import plaetze as PL
    original = Path(rj).read_bytes()
    artifact=Path(rj[:-5]+'_r24_pruefung')
    artifact.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='autorouter-r24-') as td:
        candidate, result = Path(td)/'routes.json', Path(td)/'info.json'
        candidate.write_bytes(original)
        run = PL.lauf([str(ROOT/'.venv/bin/python'), str(Path(__file__)), bj, kj, str(candidate),
                       regeln, str(result)], cpu_s=cpu_s, cwd=str(ROOT),
                      log=str(Path(td)/'worker.log'), env=dict(os.environ, PYTHONHASHSEED='0'))
        info = json.loads(result.read_text()) if run['zustand']=='ok' and result.exists() else dict(
            angewendet=False, grund='R24-Worker: '+run['zustand'])
        info['rechenplatz'] = run
        info['artefakt'] = str(artifact)
        shutil.copytree(td, artifact, dirs_exist_ok=True)
        if info.get('angewendet'):
            if Path(rj).read_bytes() != original:
                return dict(info, angewendet=False, grund='Route waehrend Nachlauf geaendert')
            with tempfile.NamedTemporaryFile(dir=Path(rj).parent, prefix='.r24_', delete=False) as f:
                tmp = Path(f.name); f.write(candidate.read_bytes())
            try:
                Path(rj[:-5]+'_vor_r24.json').write_bytes(original)
                os.replace(tmp, rj)
            finally: tmp.unlink(missing_ok=True)
        return info


def r24_erhalten(a, b):
    ""
    keys=('tote_mm', 'gratis_mm', 'sichtbare_knicke', 'haarnadeln', 'zickzack')
    return bool(a and b and all(isinstance(m.get(k),(int,float)) and not isinstance(m[k],bool)
        and math.isfinite(m[k]) for m in (a,b) for k in keys) and all(b[k]<=a[k] for k in keys))


def modelltor(B, old, after, K):
    ""
    from lernen.sammeln import regelmessung
    from analyse.regelkennzahlen import annahme
    gndlagen = old.get('gndlagen') or []
    m0, m1 = [regelmessung(B, q, K, gndlagen, dicht=True) for q in (old, after)]
    gate = annahme(m0.get('regeltor') or m0['regeln'], m1.get('regeltor') or m1['regeln'])

    notes = [m.get('note_v3') for m in (m0, m1)]
    known = all(isinstance(n, (int, float)) and not isinstance(n, bool) and math.isfinite(n) for n in notes)
    note_ok = known and notes[1] <= notes[0] + 1e-9
    gate.update(gueltig_vorher=m0['gueltig'], gueltig_nachher=m1['gueltig'],
                lernnote_vorher=notes[0], lernnote_nachher=notes[1], lernnote_erhalten=note_ok)
    gate['nicht_schlechter'] = gate['nicht_schlechter'] and m1['gueltig'] and note_ok
    if not m1['gueltig']: gate['grund'] = 'Lernmodell ungueltig'
    elif not known: gate['grund'] = 'Lernnote unbekannt'
    elif not note_ok: gate['grund'] = 'Lernnote verschlechtert'
    from lernen.rangmodell import gueltig
    model_valid = gueltig(dict(gueltig=m1['gueltig'], regel=m1))
    gate['nicht_schlechter'] = bool(gate['nicht_schlechter'] and model_valid)
    if not model_valid: gate['grund'] = 'Lernmodell/Zielvertrag ungueltig'
    return gate


def _anwenden(bj, kj, rj, regeln, folder):
    from nachbearbeitung.r24 import vorschlag, messen
    from lernen.kupfer_nativ import bibliotheksgeometrie, pruefen, run, KC
    from analyse.bewertung import bewerte
    from analyse.regelkennzahlen import annahme
    B, K, old, RG = [json.loads(Path(p).read_text()) for p in (bj, kj, rj, regeln)]
    model_board = B

    B, binfo = bibliotheksgeometrie(Path(folder)/'bibliothek', B, old, RG)
    new, info = vorschlag(B, old, K)
    info.update(angewendet=False, bibliothek=binfo)
    if not info['tote_stuecke'] and not info['abkuerzbare_ketten']:
        info['grund'] = 'Kein sicherer geometrischer Vorschlag'; return info
    if new.get('vias') != old.get('vias'):
        raise ValueError('R24 veraendert Vias')
    before, after, native = pruefen(Path(folder)/'nativ', B, old, new, RG)
    info['nativ'] = native


    drc = Path(folder)/'lose.json'
    run([KC, 'pcb', 'drc', '--format', 'json', '--severity-all', '--all-track-errors',
         '-o', str(drc), str(Path(folder)/'nativ/vorschlag.kicad_pcb')], Path(folder)/'lose.log')
    all_drc = json.loads(drc.read_text())
    loose = sum(v['type'] in ('track_dangling','via_dangling') for v in all_drc['violations'])
    info['lose'] = loose
    if not native['vorschlag']['gueltig'] or loose:
        info['grund'] = 'Native DRC sperrt (Fehler/offen/lose)'; return info

    e0, e1 = [bewerte(B, q, K, v3=True) for q in (before, after)]
    gate = annahme(e0['regeln'], e1['regeln'])
    info['regeltor'] = gate



    model_gate = modelltor(model_board, old, after, K)
    info['modelltor'] = model_gate
    info['gueltig_vorher'], info['gueltig_nachher'] = e0['gueltig'], e1['gueltig']
    from analyse.laengenimpedanz import laengen, empfaenger_schutz, empfaenger_klassen
    lk = empfaenger_klassen(K)
    info['empfaengertor'] = empfaenger_schutz(laengen(B,before,lk), laengen(B,after,lk))

    r0, r1 = (messen(B, q, K, suchen=True) for q in (before, after))
    good_r24 = r24_erhalten(r0, r1)
    info['r24_erhalten'] = good_r24
    if not (e1['gueltig'] and gate['nicht_schlechter'] and model_gate['nicht_schlechter'] and info['empfaengertor'] and good_r24):
        info['grund'] = 'Regel-/Empfaenger-/R24-Tor sperrt'; return info
    info.update(angewendet=True, grund='Native DRC 0/0/0 und Regeltor bestanden')
    after['r24_nachlauf'] = {k:v for k,v in info.items() if k not in ('regeltor', 'modelltor')}
    Path(rj).write_text(json.dumps(after))
    return info

if __name__ == '__main__':
    bj, kj, rj, rg, result = sys.argv[1:]
    try:
        td=Path(result).parent/'native'
        td.mkdir(exist_ok=True)
        info = _anwenden(bj,kj,rj,rg,str(td))
    except Exception as exc:
        info = dict(angewendet=False, grund='R24-Pruefung fehlgeschlagen', fehler=repr(exc))
    Path(result).write_text(json.dumps(info, allow_nan=False))
