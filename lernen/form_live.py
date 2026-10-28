""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import time

from lernen.form_merkmale import aus_messung, VERSION
from lernen.paar_vertrag import messmanifest

KONFIG = dict(version='form-live-v2', bewertung=dict(v3=True, minimax=False),
              diagnose=dict(r39_diagnose='1', via_dichte_diagnose='1', k20_messen=True),
              werkbank=dict(nur_form=True), regelmessung=dict(dicht=True))


def signatur():
    return hashlib.sha256(json.dumps(dict(quellen=messmanifest(), konfiguration=KONFIG),
                                        sort_keys=True, allow_nan=False).encode()).hexdigest()


def stabil(v):
    if isinstance(v, dict):
        return {k:stabil(x) for k,x in v.items() if k not in ('cpu_s', 'zeit_s', 'zeit_ms', 'pruefer_cpu_s')}
    if isinstance(v, list): return [stabil(x) for x in v]
    return v


def messen(g, h, sig):
    ""
    from analyse.bewertung import bewerte
    from lernen.sammeln import regelmessung
    b,r,k = g['board'],g['routes'],g['klassen']
    e = bewerte(b,r,k,**KONFIG['bewertung'])


    m = e['regeln']
    rm = regelmessung(b,r,k,r.get('gndlagen') or [],dicht=True,v3_bewertung=e)
    rm.pop('_physik',None)
    form,ke,w,cost = form_messen(g,h,sig,m,e.get('metriken') or {})
    return stabil(dict(ergebnis=e, regelkennzahlen=m, regel=rm, strom_kette=ke,
                       zonen_form=w, form_messung=form)), cost


def form_messen(g,h,sig,m,metriken=None):
    ""
    from analyse import regelkennzahlen as RK
    from analyse.strom_kette import kette
    from analyse.minimax import lastfehler
    from lernen.zonen_werkbank import messen as werkbank
    t=time.process_time()
    b,r,k=g['board'],g['routes'],g['klassen']
    metriken=metriken or {}


    import copy
    fm = copy.deepcopy(m)
    rd = dict(r, optionen=dict(r.get('optionen') or {}, **KONFIG['diagnose']))
    if 'strom_engstellen' not in fm and 'r39_fehler' not in fm:
        RK.r39_messen(b,rd,k,r.get('gndlagen') or [],fm,{})
    RK._r40_diagnose(fm,rd,k)
    RK._via_diagnose(fm,b,rd)
    RK._k20_diagnose(fm,b,rd,k)
    RK._k17_diagnose(fm,b,rd,k)
    ke = metriken.get('strom_kette')
    if ke is None:
        try:


            ir=metriken.get('kupfer_ir') if not any(lastfehler(v) for v in k.get('netze',{}).values()) else None
            ke = kette(b,r,k,ir=ir)
        except Exception as exc: ke=dict(fehler=repr(exc))
    w = werkbank(b,r,k,r.get('gndlagen') or [],regeln=g.get('regeln'),
                 strom_kette=ke, engstellen=fm.get('strom_engstellen'), nur_form=True)
    return aus_messung(w,fm,h,sig),ke,w,dict(zusatz_cpu_s=time.process_time()-t)


def nachmessen(g, h, cache=None):
    start = time.process_time()
    sig = signatur()
    folder = Path(cache or os.environ.get('AUTOROUTER_FORM_CACHE',
                    '~/Library/Caches/Autorouter/form_messung')).expanduser()
    folder.mkdir(parents=True,exist_ok=True)
    path = folder/f'{h}_{sig}.json'
    with (folder/f'{h}_{sig}.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        try: old = json.loads(path.read_text())
        except (OSError, ValueError): old = {}
        fm = (old.get('gemessen') or {}).get('form_messung') or {}
        hit = (old.get('geometrie_sha256') == h and old.get('auswerter_sha256') == sig
               and old.get('vollstaendig') is True and 'fehler' not in old
               and fm.get('version') == VERSION and fm.get('geometrie_sha256') == h
               and fm.get('auswerter_sha256') == sig and fm.get('vollstaendig') is True
               and all(isinstance((old.get('gemessen') or {}).get(k),dict)
                       for k in ('ergebnis','regelkennzahlen','regel','strom_kette','zonen_form')))
        if not hit:
            t = time.process_time()
            result = messen(g,h,sig)

            if isinstance(result,tuple): result,teile=result
            else: teile={}
            if signatur() != sig: raise ValueError('Auswerter waehrend Nachmessung geaendert')
            old = dict(geometrie_sha256=h, auswerter_sha256=sig, vollstaendig=True,
                       gemessen=result, cpu_s=time.process_time()-t, teile_cpu_s=teile)
            from lernen.richter import atomar
            atomar(path,old)
        if hit and signatur() != sig: raise ValueError('Auswerter waehrend Cachezugriff geaendert')
    return old['gemessen'], dict(cache=hit, cpu_s=time.process_time()-start,
                                 messkern_cpu_s=0.0 if hit else old['cpu_s'],
                                 original_cpu_s=old['cpu_s'], teile_cpu_s=old.get('teile_cpu_s') or {}, last=os.getloadavg()[0])
