""
import json
import hashlib
import fcntl
from contextlib import contextmanager
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def _erhalten(a,b,keys):
    return all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)
               for m in (a,b) for k in keys for v in [m.get(k)]) and all(b[k]<=a[k]+1e-9 for k in keys)


def tor(B, old, new, K, gndlagen):
    from lernen.sammeln import regelmessung
    from analyse.regelkennzahlen import torfelder, wert
    from analyse.r18 import messen
    from analyse.wackel import messen as wackel
    from analyse.laengenimpedanz import laengen, empfaenger_schutz, empfaenger_klassen
    ms = [regelmessung(B,q,K,gndlagen,dicht=True) for q in (old,new)]
    contract = bewertungsvertrag(*ms)
    a,b = [m.get('regeltor') or m['regeln'] for m in ms]
    keys = tuple(dict.fromkeys(torfelder(a)+torfelder(b)))
    missing = [k for k in keys if wert(a,k) is None or wert(b,k) is None]
    worse = [k for k in keys if k not in missing and wert(b,k)>wert(a,k)+1e-9]
    note_ok = _erhalten(ms[0],ms[1],('note_v3',))
    form = [messen(B,q,K) for q in (old,new)]
    form_ok = all(_erhalten(form[0][s],form[1][s],('knicke','haarnadeln','zickzack','knick_ueberschuss','umweg_octi_mm')) for s in ('alle','signal'))
    wa = [wackel(B,q,K.get('gnd') or []) for q in (old,new)]
    wackel_ok = _erhalten(wa[0],wa[1],('zuege','zuege_je_10mm','laenge_mm'))
    lk = empfaenger_klassen(K)
    receiver_ok = empfaenger_schutz(laengen(B,old,lk),laengen(B,new,lk))
    improved = any(form[1][s][k]<form[0][s][k] for s in ('alle','signal') for k in ('knicke','haarnadeln','zickzack','knick_ueberschuss','umweg_octi_mm'))
    accepted = bool(ms[1]['gueltig'] and contract['nicht_schlechter'] and not missing and not worse and note_ok and form_ok and wackel_ok and receiver_ok and improved)
    return dict(angenommen=accepted, vertrag=contract, gueltig=[m['gueltig'] for m in ms], fehlend=missing, schlechter=worse,
                lernnote=[m.get('note_v3') for m in ms], lernnote_erhalten=note_ok,
                r18=form, r18_erhalten=form_ok, wackel=wa, wackel_erhalten=wackel_ok,
                empfaengertor=receiver_ok, verbessert=improved)



def bewertungsvertrag(a, b):
    ""
    gates = {}
    if a.get('tor') or b.get('tor'):
        from analyse.vertrag import vergleich
        if not a.get('tor') or not b.get('tor'):
            return dict(nicht_schlechter=False, grund='Pflichtvertrag fehlt')
        gates['tor'] = vergleich(a['tor'], b['tor'])
    if any('minimax_version' in e or 'minimax_gueltig' in e or e.get('bewertungsziel') == 'minimax' for e in (a,b)):
        from analyse.minimax import annahme
        gates['minimax'] = annahme(*[dict(e, regeln=e.get('regeltor') or e.get('regeln')) for e in (a,b)])
    return dict(nicht_schlechter=all(g['nicht_schlechter'] for g in gates.values()), tore=gates)


@contextmanager
def dateisperre(rj):
    with Path(str(Path(rj).resolve())+'.r18.lock').open('a+b') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        try: yield
        finally: fcntl.flock(lock, fcntl.LOCK_UN)



@contextmanager
def belegter_platz():
    ""
    from gemeinsam import plaetze as PL
    lauf = PL.lauf
    def child(*args, **kw):
        kw['mit_platz'] = False
        return lauf(*args, **kw)
    PL.lauf = child
    try: yield
    finally: PL.lauf = lauf

def austauschen(rj, original, candidate):
    ""
    with dateisperre(rj):
        if Path(rj).read_bytes() != original: return False
        with tempfile.NamedTemporaryFile(dir=Path(rj).parent, prefix='.r18_', delete=False) as f:
            tmp = Path(f.name); f.write(candidate)
        try:
            Path(rj[:-5]+'_vor_r18.json').write_bytes(original)
            os.replace(tmp, rj)
        finally: tmp.unlink(missing_ok=True)
    return True


def verweise_einfrieren(value, artifact, base, gelesen=None, abbilder=None):
    ""
    gelesen = {} if gelesen is None else gelesen
    abbilder = {} if abbilder is None else abbilder
    def lesen(src):
        key = str(src.resolve())
        if key not in gelesen: gelesen[key] = src.read_bytes()
        return gelesen[key]
    if isinstance(value, list):
        return [verweise_einfrieren(v, artifact, base, gelesen, abbilder) for v in value]
    if not isinstance(value, dict): return value
    out = {}
    for key, v in value.items():
        if key == 'board_datei' and isinstance(v, str):
            src=Path(v).expanduser()
            if not src.is_absolute(): src=base/src
            dest=artifact/'quellen'/hashlib.sha256(str(src.resolve()).encode()).hexdigest()[:16]/src.name
            if str(dest) not in abbilder:
                obj=verweise_einfrieren(json.loads(lesen(src)),artifact,src.parent,gelesen,abbilder)
                dest.parent.mkdir(parents=True,exist_ok=True)
                dest.write_text(json.dumps(obj)); abbilder[str(dest)]=obj
            out[key]=str(dest)
        elif key in ('src', 'quelle') and isinstance(v, str) and Path(v).suffix in ('.kicad_pcb', '.kicad_pro', '.kicad_dru'):
            src = Path(v).expanduser()
            if not src.is_absolute(): src = base / src
            if str(src.resolve()) not in gelesen and not src.is_file():
                raise ValueError('Native Snapshot-Quelle fehlt: ' + str(src))
            dest = artifact / 'quellen' / hashlib.sha256(str(src.resolve()).encode()).hexdigest()[:16] / src.name
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(lesen(src))
            for ext in ('.kicad_pro', '.kicad_dru'):
                companion = src.with_suffix(ext)
                if str(companion.resolve()) in gelesen or companion.is_file():
                    dest.with_suffix(ext).write_bytes(lesen(companion))
            out[key] = str(dest)
        else: out[key] = verweise_einfrieren(v, artifact, base, gelesen, abbilder)
    return out


def speichern(path, value):
    ""
    path=Path(path)
    with tempfile.NamedTemporaryFile(dir=path.parent,prefix='.r18_',delete=False,mode='w') as f:
        tmp=Path(f.name); json.dump(value,f)
    try: os.replace(tmp,path)
    finally: tmp.unlink(missing_ok=True)


class _Abgelehnt(Exception):
    ""


def anwenden(bj,kj,rj,regeln,gndlagen,cpu_s=180,roh_sha256=None,glatt_sha256=None,abschluss=None,minimax=None):
    from gemeinsam import plaetze as PL
    rj = str(Path(rj).resolve())
    artifact = None
    try:
        with dateisperre(rj): original = Path(rj).read_bytes()
        raw = Path(rj[:-5]+'_glatt.json'); roh = Path(rj[:-5]+'_roh.json')

        if not roh_sha256 or not glatt_sha256 or not raw.exists() or not roh.exists():
            raise _Abgelehnt('Aktuelle gestraffte Basis fehlt')
        raw_bytes = raw.read_bytes()
        if (hashlib.sha256(roh.read_bytes()).hexdigest()!=roh_sha256 or
            hashlib.sha256(raw_bytes).hexdigest()!=glatt_sha256 or
            json.loads(raw_bytes).get('r18_roh_sha256')!=roh_sha256):
            raise _Abgelehnt('Zwischenartefakte gehoeren nicht zur aktuellen Rohroute')
        inputs=[json.loads(Path(p).read_text()) for p in (bj,kj,regeln)]
        original_plan=None
        plan=Path(bj).parent/'zonenplan/zonenplan.json'

        gelesen={}
        if (abschluss or {}).get('mode') == 'lokal':


            from lernen.zonenplaner import pruefe_eingaben, sha
            try:
                original_plan=json.loads(plan.read_text())
                route=json.loads(original); glatt=json.loads(raw_bytes)
                if not original_plan.get('board_datei'):
                    raise ValueError('Zonenplan ohne native Boarddatei')




                native=Path(original_plan['board_datei']).expanduser()
                if not native.is_absolute(): native=plan.parent/native
                native_bytes=native.read_bytes()
                pruefe_eingaben(dict(original_plan,board_datei=None),*inputs,
                    optionen=dict(route.get('optionen') or {},gndlagen=','.join(gndlagen)))
                if sha(json.loads(native_bytes)) != original_plan['eingabehashes']['board']:
                    raise ValueError('Zonenplan: native Padgeometrie geaendert')
                if any(q.get('zonenplan_id') != original_plan['id'] for q in (route,glatt)):
                    raise ValueError('Zonenplan-ID der aktuellen Route/Basis fehlt oder ist fremd')
                gelesen[str(native.resolve())]=native_bytes
            except Exception as exc:
                raise _Abgelehnt('Zonenplan-Originalvertrag: '+str(exc))
        parent = Path(rj[:-5]+'_r18'); parent.mkdir(exist_ok=True)
        artifact = Path(tempfile.mkdtemp(prefix='lauf_',dir=parent))
        source=artifact/'eingabe.json'; raw_source=artifact/'glatt.json'

        frozen=[]; frozen_objs=[]; abbilder={}
        for (path,name),value in zip(((bj,'board.json'),(kj,'klassen.json'),(regeln,'regeln.json')),inputs):
            dest=artifact/name
            snapshot=verweise_einfrieren(value, artifact, Path(path).resolve().parent, gelesen, abbilder)
            dest.write_text(json.dumps(snapshot)); frozen.append(str(dest)); frozen_objs.append(snapshot)
        plan_ids = None
        source_bytes, raw_source_bytes = original, raw_bytes
        if (abschluss or {}).get('mode') == 'lokal':
            target=artifact/'zonenplan/zonenplan.json'; target.parent.mkdir()
            snapshot=verweise_einfrieren(original_plan,artifact,plan.parent,gelesen,abbilder)
            old_id=snapshot['id']



            snapshot['routing_board_sha256']=sha(frozen_objs[0])
            snapshot['eingabehashes']=dict(snapshot['eingabehashes'],klassen=sha(frozen_objs[1]),
                regeln=sha(frozen_objs[2]),board=sha(abbilder[snapshot['board_datei']]))
            snapshot['id']=sha({k:v for k,v in snapshot.items() if k not in ('id','board_datei')})
            plan_ids=(old_id,snapshot['id'])
            target.write_text(json.dumps(snapshot))
            def binden(data):
                q=json.loads(data)
                if q.get('zonenplan_id')!=old_id: return data
                q['zonenplan_id']=snapshot['id']; return json.dumps(q).encode()
            source_bytes, raw_source_bytes = binden(original), binden(raw_bytes)
        source.write_bytes(source_bytes); raw_source.write_bytes(raw_source_bytes)
        (artifact/'snapshot.json').write_text(json.dumps({str(p.relative_to(artifact)):
            hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(artifact.rglob('*')) if p.is_file()},indent=1))
    except _Abgelehnt as exc:
        return dict(angewendet=False,abgeschlossen=False,grund=str(exc),cpu_s=0.)
    except Exception as exc:
        if artifact is not None: shutil.rmtree(artifact, ignore_errors=True)
        return dict(angewendet=False,abgeschlossen=False,grund='R18-Snapshot: '+repr(exc)[:300],cpu_s=0.)
    bj,kj,regeln=frozen
    result=artifact/'info.json'
    run=PL.lauf([str(ROOT/'.venv/bin/python'), str(Path(__file__)), bj,kj,str(source),str(raw_source),regeln,
        json.dumps(list(gndlagen)),str(artifact),json.dumps(abschluss),str(cpu_s),json.dumps(minimax)],
        cpu_s=cpu_s, cwd=str(ROOT),log=str(artifact/'worker.log'),env=dict(os.environ,PYTHONHASHSEED='0'))
    checkpoint=json.loads(result.read_text()) if result.exists() else {}
    info=checkpoint if run['zustand']=='ok' or checkpoint.get('abgeschlossen') else dict(
        angewendet=False,grund='R18-Worker: '+run['zustand'])
    if run['zustand'] != 'ok':
        info.update(angewendet=False,grund='R18-Vorschlag: '+run['zustand'])
        if info.get('referenz_abschluss') is not None: info['abschluss']=info['referenz_abschluss']
    info.update(rechenplatz=run,cpu_s=run['cpu_s'],arbeitsordner=str(artifact))
    if info.get('angewendet') or info.get('abgeschlossen'):
        selected=artifact/'routes.json' if run['zustand']=='ok' else artifact/'referenz/routes.json'
        output=selected.read_bytes()
        if plan_ids:
            q=json.loads(output)
            if q.get('zonenplan_id')==plan_ids[1]:
                q['zonenplan_id']=plan_ids[0];output=json.dumps(q).encode()
        if not austauschen(rj,original,output):
            info.update(angewendet=False,abgeschlossen=False,grund='Route inzwischen geaendert')
    return info


def worker(bj,kj,rj,raw,regeln,gnd,folder,abschluss='null',cpu_s='180',minimax='null'):
    from nachbearbeitung.r18 import vorschlag
    from nachbearbeitung.pulltight_routes import optimize
    from nachbearbeitung.glaetten45 import via_mitte
    from analyse.bewertung import unverbunden
    from lernen.form import abschliessen
    from lernen.cpubudget import Budget
    from lernen.zonenplaner import abnahmetor
    budget=Budget(float(cpu_s)); gnd=json.loads(gnd); finish=json.loads(abschluss)
    B,K,old,R0,RG=[json.loads(Path(p).read_text()) for p in (bj,kj,rj,raw,regeln)]

    mm=json.loads(minimax)
    if mm is not None:
        K=dict(K,vorgaben=dict(K.get('vorgaben') or {},bewertung='minimax' if mm else 'v3'))
        kj=str(Path(folder,'klassen.json')); Path(kj).write_text(json.dumps(K))

    reference=Path(folder,'referenz'); reference.mkdir()
    refpath=reference/'routes.json'; refpath.write_text(json.dumps(old))
    def close(path):
        if not finish: return {}
        with belegter_platz():
            return abschliessen(bj,kj,str(path),regeln,gnd,finish['cfg'],finish['mode'],finish['r36'],budget)
    refinfo=close(refpath)
    old=json.loads(refpath.read_text()); selected=old
    info=dict(angewendet=False, abgeschlossen=bool(finish), abschluss=refinfo, referenz_abschluss=refinfo)

    speichern(Path(folder,'routes.json'),selected)
    speichern(Path(folder,'info.json'),info)
    try:
        if budget.rest < 1: raise TimeoutError('R18-Suche: CPU-Budget erschoepft')
        new, proposal=vorschlag(B,R0,K)
        info.update(proposal)
        if info['geaendert']:
            if budget.rest < 1: raise TimeoutError('R18-Rundung: CPU-Budget erschoepft')
            new=optimize(B,new,RG['router'],K,pull=False,rmax=.5,connectivity=unverbunden)
            new['via_mitte']=via_mitte(new)
            Path(folder,'kandidat.json').write_text(json.dumps(new))
            if budget.rest < 1: raise TimeoutError('R18-Formtor: CPU-Budget erschoepft')
            info['tor']=tor(B,old,new,K,gnd)
            if info['tor']['angenommen']:
                candidate=Path(folder,'vorschlag');candidate.mkdir()
                cp=candidate/'routes.json';cp.write_text(json.dumps(new))
                candidate_info=close(cp); new=json.loads(cp.read_text())
                if budget.rest < 1: raise TimeoutError('R18-Abschlusstor: CPU-Budget erschoepft')
                mode=finish['mode'] if finish else 'aus'; r36=finish['r36'] if finish else '0'
                duty='nativ' if mode=='lokal' or r36=='nativ' else ('legacy' if r36=='legacy' else False)
                closure_ok=abnahmetor(dict(gueltig=True),candidate_info.get('stromzonen'),duty)['gueltig']
                info['abschlusstor']=tor(B,old,new,K,gnd)
                if closure_ok and info['abschlusstor']['angenommen']:
                    if budget.rest < 1: raise TimeoutError('R18-Nativ: CPU-Budget erschoepft')
                    from lernen.kupfer_nativ import pruefen
                    own=time.thread_time()
                    _,native_new,native=pruefen(Path(folder,'nativ'),B,old,new,RG,cpu_s=min(120,budget.rest))


                    budget.add(max(0.,native['cpu_s']-(time.thread_time()-own)))
                    info['nativ']=native
                    if budget.rest < 1: raise TimeoutError('R18-Endtor: CPU-Budget erschoepft')
                    info['endtor']=tor(B,old,native_new,K,gnd)
                    if native['vorschlag']['gueltig'] and info['endtor']['angenommen'] and budget.rest>0:
                        selected=native_new
                        info.update(angewendet=True,abschluss=candidate_info)
    except Exception as exc:
        info['abschluss_fehler']=repr(exc)[:500]
    info['grund']='' if info['angewendet'] else 'Kein gewinnender Vorschlag unter allen Form-/Abschluss-/Regeltoren'
    info['cpu_budget']=dict(verwendet=budget.verwendet,kinder=budget.kinder,rest=budget.rest)
    speichern(Path(folder,'routes.json'),selected)
    speichern(Path(folder,'info.json'),info)


if __name__=='__main__': worker(*sys.argv[1:])
