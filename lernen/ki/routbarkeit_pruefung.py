""
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from scipy.stats import spearmanr
from sklearn.metrics import roc_auc_score

BASELINES = ('rudy', 'hpwl', 'kollisionen', 'auswahl', 'faust')
METRIKEN = ('auc', 'spearman', 'paartreffer')
REGELN = ('note_v3', 'dc', 'j', 'rueck', 'xt', 'abgriff_nah_mm', 'abzweig_mitte',
          'parallel_rest_mm', 'spitz', 'kopplung')


def quellfamilie(f):
    if not isinstance(f,str) or not f.strip():raise ValueError('Quellfamilie fehlt')
    f=f.strip().lower()
    return 'generator:aufgaben-v9' if f.startswith('erf_') else f


def prueffamilie(f,herkunft):
    f=quellfamilie(f)
    if f.startswith('generator:') or herkunft!='dauerlauf':
        raise ValueError('Prüfung nur auf fremden Dauerlauf-Familien, kein Selbstspiel')
    return f


def _zahlen(rows, name):
    ys=[]; ps=[]; ns=[]; qs=[]; hits=[]
    for r in rows:
        pred=r['prognosen'][name]
        if r.get('kandidat_id') is not None:
            ys.append(int(r['gueltig']));ps.append(pred['p_gueltig'])
            ns.append(r['note_v3']);qs.append(pred['note_v3'])
        elif r.get('besser') in ('a','b'):
            hits.append(float(r['besser']==pred['besser']))
    return dict(auc=float(roc_auc_score(ys,ps)) if len(set(ys))==2 else None,
                spearman=float(spearmanr(ns,qs).statistic) if len(ns)>2 and len(set(ns))>1 and len(set(qs))>1 else None,
                paartreffer=float(np.mean(hits)) if hits else None)


def _finite(x): return isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x)


def _vergleichsvertrag(rows):
    ""
    candidates={};pairs={}
    for r in rows:
        pid=r.get('paar_id');kid=r.get('kandidat_id');family=prueffamilie(r['familie'],r.get('herkunft'))
        if not isinstance(pid,str) or not pid:raise ValueError('Paar-ID fehlt')
        if kid is not None:
            if 'besser' in r or 'kandidat_ids' in r:raise ValueError('vermischte Kandidaten-/Paarzeile')
            if not isinstance(kid,str) or not kid or kid in candidates:raise ValueError('Kandidaten-ID fehlt/doppelt')
            candidates[kid]=(pid,family)
            if not isinstance(r.get('gueltig'),bool) or not _finite(r.get('note_v3')):raise ValueError('Kandidatenmessung fehlt')
        else:
            if 'gueltig' in r or 'note_v3' in r:raise ValueError('vermischte Kandidaten-/Paarzeile')
            ids=r.get('kandidat_ids')
            if pid in pairs or not isinstance(ids,list) or len(ids)!=2 or len(set(ids))!=2 or r.get('besser') not in ('a','b','gleich'):
                raise ValueError('Paarmessung/IDs fehlen oder doppelt')
            pairs[pid]=(ids,family)
        for name in ('modell',)+BASELINES:
            pred=r.get('prognosen',{}).get(name)
            if not isinstance(pred,dict) or pred.get('paar_id')!=pid:raise ValueError('Prognose/Paar-ID fehlt: '+name)
            if kid is not None:
                if pred.get('kandidat_id')!=kid or not _finite(pred.get('p_gueltig')) or not 0<=pred['p_gueltig']<=1 or not _finite(pred.get('note_v3')):
                    raise ValueError('Prognose/Kandidaten-ID fehlt: '+name)
            elif pred.get('kandidat_ids')!=ids or pred.get('besser') not in ('a','b'):
                raise ValueError('Prognose/Paarinhalt fehlt: '+name)
    for pid,(ids,family) in pairs.items():
        if any(candidates.get(kid)!=(pid,family) for kid in ids):raise ValueError('Paar ohne gemeinsame Kandidaten')
    expected={kid for ids,_ in pairs.values() for kid in ids}
    if expected!=set(candidates):raise ValueError('Kandidat ohne vollständiges Paar')


def vergleichen(rows, wiederholungen=2000, seed=20261008, min_familien=10):
    if not rows: return dict(freigegeben=False,gruende=['keine Testdaten'],familien=0)
    fs=sorted({quellfamilie(r['familie']) for r in rows}); groups={f:[r for r in rows if quellfamilie(r['familie'])==f] for f in fs}
    if any(r.get('split')!='test' for r in rows): raise ValueError('Freigabevergleich nur test')
    _vergleichsvertrag(rows)
    names=('modell',)+BASELINES; punkt={n:_zahlen(rows,n) for n in names}; rng=np.random.default_rng(seed)
    deltas={m:[] for m in METRIKEN}
    for _ in range(wiederholungen):
        rr=[r for f in rng.choice(fs,len(fs),replace=True) for r in groups[f]]
        values={n:_zahlen(rr,n) for n in names}
        for m in METRIKEN:
            v=values['modell'][m]; bs=[values[n][m] for n in BASELINES]
            if _finite(v) and all(_finite(b) for b in bs): deltas[m].append(v-max(bs))
    ci={m:np.quantile(a,[.025,.975]).tolist() if len(a)>=.95*wiederholungen else None for m,a in deltas.items()}
    reasons=[]
    if len(fs)<min_familien: reasons.append('zu wenige unabhängige Testfamilien')
    if any(any(not _finite(punkt[n][m]) for m in METRIKEN) for n in names): reasons.append('Metrik oder starke Grundlinie fehlt')
    if any(ci[m] is None or ci[m][0]<=0 for m in METRIKEN): reasons.append('Konfidenzbereich gegen beste Grundlinie nicht über null')
    return dict(freigegeben=not reasons,gruende=reasons,familien=len(fs),zeilen=len(rows),punkt=punkt,delta_ci95=ci,
                bootstrap=wiederholungen,seed=seed,min_familien=min_familien,einheit='Quellfamilie, nicht Pixel/Seed')


def _native_pruefen(messung,root):
    ""
    n=messung['native_nachweis'];root=Path(root or '.')
    board=root/n['board'];report=root/n['bericht']
    if board.suffix!='.kicad_pcb' or sha(board)!=n['board_sha256'] or sha(report)!=n['bericht_sha256']:
        raise ValueError('nativer Artefakthash')
    raw=json.loads(report.read_text())
    if Path(raw['source']).name!=board.name:raise ValueError('native Prüfquelle')
    if not isinstance(raw['violations'],list) or not isinstance(raw['unconnected_items'],list):raise ValueError('nativer Bericht unvollständig')
    if any(not isinstance(v,dict) or v.get('severity') not in ('error','warning','ignore') for v in raw['violations']):raise ValueError('native Fehlerschwere fehlt')
    native_ok=not raw['unconnected_items'] and not any(v['severity']=='error' for v in raw['violations'])
    if messung['gueltig'] and not native_ok:raise ValueError('native DRC widerspricht Gültigkeit')


def nutzentor(paare,einsatz='auswahl',root=None):
    ""
    if not paare: return dict(freigegeben=False,gruende=['keine unabhängigen Endprüfungen'])
    if einsatz not in ('auswahl','placer','richter'):raise ValueError('Einsatz')
    reasons=[]; cpu=[];ids=set();nutzen=False
    for r in paare:
        a,b=r.get('ohne',{}),r.get('mit',{})
        if any(r.get(k) is not True for k in ('gleiche_aufgabe','gleiches_budget','native_endpruefung','vorab_festgelegt')):
            reasons.append('Paarvertrag/Endprüfung fehlt');continue
        if r.get('einsatz')!=einsatz:reasons.append('Endmessung gehört nicht zum Einsatz')
        if b.get('routbarkeit_einsatz')!=einsatz or a.get('routbarkeit_einsatz') is not None:
            reasons.append('Aktiver Einsatz in Mit/Ohne-Endmessung fehlt')
        if not isinstance(r.get('id'),str) or not r['id'] or r['id'] in ids:reasons.append('Endpaar-ID fehlt/doppelt')
        else:ids.add(r['id'])
        try:prueffamilie(r['familie'],r.get('herkunft'))
        except (KeyError,ValueError):reasons.append('Endprüfung ohne fremde Dauerlauf-Familie')
        if not isinstance(a.get('gueltig'),bool) or not isinstance(b.get('gueltig'),bool): reasons.append('Gültigkeit unbekannt'); continue
        for m in (a,b):
            try:_native_pruefen(m,root)
            except (OSError,ValueError,KeyError,TypeError):reasons.append('Nativer Prüfbericht fehlt/ungültig')
        if a['gueltig'] and not b['gueltig']: reasons.append('weniger gültig')
        if b['gueltig'] and not a['gueltig']:nutzen=True
        for k in REGELN:
            x,y=a.get(k),b.get(k)
            if not _finite(x) or not _finite(y): reasons.append('Regel fehlt: '+k)
            elif y>x+1e-6: reasons.append('Regel schlechter: '+k)
            elif a['gueltig'] and b['gueltig'] and y<x-1e-6:nutzen=True
        if any(not _finite(m.get('cpu_s')) or m['cpu_s']<0 for m in (a,b)): reasons.append('CPU-Messung fehlt/ungültig')
        else: cpu.append(b['cpu_s']-a['cpu_s'])
    if cpu and float(np.mean(cpu))<0 and all(r.get('ohne',{}).get('gueltig') is True and r.get('mit',{}).get('gueltig') is True for r in paare):nutzen=True
    if not nutzen:reasons.append('Kein gemessener Nutzen für Einsatz')
    return dict(freigegeben=not reasons,gruende=sorted(set(reasons)),einsatz=einsatz,paare=len(paare),cpu_delta_mittel=float(np.mean(cpu)) if cpu else None)


def freigabetor(statistik, nutzung, kalibrierung, latency_ms=None,einsatz='placer'):
    reasons=list(statistik.get('gruende') or [])
    if statistik.get('freigegeben') is not True: reasons.append('Statistiktor gesperrt')
    from lernen.ki.routbarkeit_kalibrieren import nachweis_pruefen
    reasons+=nachweis_pruefen(kalibrierung)
    if nutzung.get('freigegeben') is not True: reasons+=nutzung.get('gruende') or ['Nutzentor gesperrt']
    if nutzung.get('einsatz')!=einsatz:reasons.append('Nutzennachweis für anderen Einsatz')
    if einsatz not in ('auswahl','richter','placer'): raise ValueError('Einsatz')
    if einsatz=='placer' and (not _finite(latency_ms) or not 0<=latency_ms<20): reasons.append('Ende-zu-Ende-Abfrage nicht unter 20 ms')
    return dict(freigegeben=not reasons,gruende=sorted(set(reasons)),statistik=statistik,nutzung=nutzung,
                kalibrierung=kalibrierung,latency_ms=latency_ms,einsatz=einsatz)


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def atomar(path, value):
    import os,tempfile
    p=Path(path);p.parent.mkdir(parents=True,exist_ok=True)
    fd,tmp=tempfile.mkstemp(dir=p.parent,prefix='.'+p.name)
    try:
        with os.fdopen(fd,'w') as f:
            json.dump(value,f,indent=2,sort_keys=True,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
        os.replace(tmp,p)
    finally: Path(tmp).unlink(missing_ok=True)
