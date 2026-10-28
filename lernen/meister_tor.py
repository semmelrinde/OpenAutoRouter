""
import contextlib
import copy
import fcntl
import hashlib
import importlib.metadata
import json
import os
import platform
import shutil
import subprocess
import sys
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R))
from lernen import meister as M
from gemeinsam import plaetze as PL


def atomar(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.' + uuid.uuid4().hex + '.neu')
    with tmp.open('w') as f:
        json.dump(value, f, indent=1, sort_keys=True, allow_nan=False); f.write('\n')
        f.flush(); os.fsync(f.fileno())
    os.replace(tmp, path)


def jsonl(path):
    if not Path(path).exists(): return []
    from lernen.lernkurve import lernzeilen
    return lernzeilen(path)


def tasks(root):
    ""
    from lernen import aufgaben as A
    out = []
    for v in ('10', '11'):
        for st in (0, 2, 4, 6, 8):
            for seed in (900001, 900002, 900003):
                folder = Path(root) / ('v'+v) / ('s%d_%d' % (st, seed))
                if not (folder/'aufgabe.json').exists():
                    A.erzeugen(str(folder), seed*10+st, stufe=st, generatorversion=v)
                a = json.loads((folder/'aufgabe.json').read_text())
                if a['version'] != v or a['seed'] != seed*10+st or a['stufe'] != st:
                    raise ValueError('Torsatz veraendert: '+str(folder))
                hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(folder.iterdir()) if p.is_file()}
                if v=='11':
                    reference=json.loads((R/'codex/generator_echt/lernkurve_v11.json').read_text())
                    expected=next(x['sha256'] for x in reference['aufgaben'] if x['id']==folder.name)
                    if any(hashes.get(f)!=digest for f,digest in expected.items()):
                        raise ValueError('V11 weicht vom versionierten Torsatz ab: '+folder.name)
                else:
                    from lernen.lernkurve import ORDNER
                    canonical=Path(ORDNER)/'aufgaben/v10'/folder.name
                    for f,digest in hashes.items():
                        if (canonical/f).exists() and hashlib.sha256((canonical/f).read_bytes()).hexdigest()!=digest:
                            raise ValueError('V10 weicht vom eingefrorenen Lernkurvensatz ab: '+folder.name+'/'+f)
                out.append(dict(id='v'+v+'/'+folder.name, pfad=str(folder), klasse=M.klasse(a), sha=hashes))
    manifest = Path(root)/'satz.json'
    if manifest.exists() and json.loads(manifest.read_text()) != out:
        raise ValueError('Torsatzinhalt gegen eingefrorenes Manifest veraendert')
    atomar(manifest, out)
    return out


def kopf():
    ""
    r = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=R, capture_output=True, text=True)
    if r.returncode == 0 and r.stdout.strip(): return r.stdout.strip()
    from lernen.sammeln import code_stand
    return 'ohne_git:' + code_stand()


def laufzeit():
    ""
    cli=Path('/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli')
    packages={}
    for name in ('numpy','scipy','shapely','numba'):
        try:packages[name]=importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:packages[name]=None
    return dict(python=sys.version,plattform=platform.platform(),pakete=packages,
        kicad=subprocess.check_output([str(cli),'version'],text=True).strip(),
        kicad_cli_sha=hashlib.sha256(cli.read_bytes()).hexdigest())


def effektive_cfg(cfg):
    ""
    cfg=copy.deepcopy(cfg);kosten=M.parameter(cfg.get('kosten') or {})
    if 'form' in kosten:cfg['form']=kosten['form']
    if 'placer_stil' in kosten and isinstance(cfg.get('placer','alt'),str):cfg['placer']=kosten['placer_stil']
    if 'router_stil' in kosten and isinstance(cfg.get('router','alt'),str):cfg['router']=kosten['router_stil']
    return cfg


def placement_cfg(cfg):
    ""
    cfg = effektive_cfg(cfg)
    cfg['kosten'] = {k:v for k,v in (cfg.get('kosten') or {}).items() if k in M.PLACER}
    for key in M.ROUTER | {'form','router'}:
        cfg.pop(key, None)
    return cfg


def wiederverwenden(task, cfg, source_sha, reference, laufzeit_sha=None):
    ""
    if not reference:return None
    z, directory=reference
    directory=Path(directory);p=directory/'messung.json'
    if not p.exists():return None
    raw=json.loads(p.read_text())
    measured={k:v for k,v in z.items() if k!='wiederverwendung'}
    if M.sha(raw)!=M.sha(measured):return None
    if z.get('wiederverwendung') and hashlib.sha256(p.read_bytes()).hexdigest()!=z['wiederverwendung']['messung_sha']:
        raise ValueError('Wiederverwendeter Originalbeleg veraendert')
    meta=raw.get('meister_messung') or {}
    if laufzeit_sha is not None and meta.get('laufzeit_sha')!=laufzeit_sha:return None
    cfg=effektive_cfg(cfg)
    placement_only = (raw.get('zustand') == 'placer_kollision' and
                      placement_cfg(raw.get('cfg') or {}) == placement_cfg(cfg))
    if (raw.get('code_sha256')!=source_sha or (raw.get('cfg')!=cfg and not placement_only) or
        meta.get('eingang')!=task['sha'] or meta.get('klasse')!=task['klasse'] or
        meta.get('budget')!='arbeit' or meta.get('arbeit')!=cfg.get('arbeit_max') or
        meta.get('seed')!=cfg.get('seed') or type(raw.get('gueltig')) is not bool):
        return None
    if raw.get('zustand') not in ('ok','placer_kollision'):return None
    if raw['zustand']=='ok' and not raw.get('native_meister'):return None
    artifacts=meta.get('artefakte')
    if not isinstance(artifacts,dict) or not artifacts:return None
    for name,digest in artifacts.items():
        artifact=(directory/name).resolve()
        if not artifact.is_relative_to(directory.resolve()):raise ValueError('Artefakt ausserhalb des Torfalls')
        if not artifact.exists() or hashlib.sha256(artifact.read_bytes()).hexdigest()!=digest:
            raise ValueError('Wiederverwendungsbeleg veraendert: '+str(artifact))
    raw['wiederverwendung']=dict(ordner=str(directory),messung_sha=hashlib.sha256(p.read_bytes()).hexdigest(),
        grund='Identische Quelle/Eingabe/vollstaendige Konfiguration/Seed/Arbeitsbudget/native Artefakte',
        erneut_geroutet=False)
    if placement_only:
        raw['wiederverwendung'].update(grund='Identisches Placement vor Routing; Routerkosten wurden nicht ausgefuehrt',
                                       angefragte_cfg=cfg, nur_placement=True)
    return raw


def letzte_referenzen(root):
    ""
    root=Path(root).resolve();pointer=root/'letzte_messung.json'
    if not pointer.exists():return {}
    item=json.loads(pointer.read_text());p=Path(item['bericht']).resolve()
    if not p.is_relative_to(root):raise ValueError('Torcache ausserhalb seiner Messwurzel')
    if not p.exists():return {}
    if hashlib.sha256(p.read_bytes()).hexdigest()!=item['sha256']:raise ValueError('Torcache-Bericht geaendert')
    report=json.loads(p.read_text());arm=item['arm'];data=p.parent/arm/'ergebnisse.json'
    if not data.exists():return {}
    rows=json.loads(data.read_text())
    if M.sha(rows)!=report['ergebnisse_sha'][arm]:raise ValueError('Torcache-Einzelbelege geaendert')
    return {k:(z,report['beleg_ordner'][arm][k]) for k,z in rows.items()}


def konfiguration(task, doc, arbeit):
    from lernen.lernkurve import REF, mess_cfg
    _, entry = M.auswahl(doc, task['klasse'], exakt=True)
    cfg = dict(REF, legal_lp=True, meister='0', kosten=entry['parameter'] if entry else {}, router=['versuche=0'])
    return effektive_cfg(mess_cfg(cfg, 'arbeit', arbeit, 60000, ()))


def messen(satz, doc, folder, parallel=3, arbeit=59147, it=40, referenzen=None):
    from lernen import selbstspiel as SS
    from lernen.lernkurve import REF, mess_cfg
    from analyse import korpus as KO
    folder = Path(folder); folder.mkdir(parents=True, exist_ok=True)
    runtime=laufzeit();runtime_sha=M.sha(runtime)
    def eins(task):
        cfg = konfiguration(task, doc, arbeit)
        reused=wiederverwenden(task,cfg,SS.code_hashes()['code_sha256'],(referenzen or {}).get(task['id']),laufzeit_sha=runtime_sha)
        if reused is not None:
            reused['wiederverwendung']['angefragtes_modell_sha']=M.sha(doc)
            print('Meistertor wiederverwendet '+task['id']+' '+str(reused['gueltig']),flush=True)
            return task['id'],reused
        w = folder/task['id']; shutil.copytree(task['pfad'], w)
        A = json.loads((w/'aufgabe.json').read_text())
        KO.klassen_lauf(str(w), str(w/'board.json'), str(w/'klassen.json'), str(w/'vorgaben.json'))
        z = SS.versuch(str(w), A, cfg, 0, 300, it)
        if z.get('zustand') == 'ok' and not z.get('native_meister'):
            try:
                nativ(w, A, z)
            except Exception as exc:
                z.update(gueltig=None, zustand='native_tormessung_unvollstaendig', fehler=repr(exc))

        artifacts = {}
        for p in sorted((w/'k0').rglob('*')):
            if p.is_file() and p.suffix in ('.json', '.kicad_pcb', '.kicad_dru', '.kicad_pro'):
                artifacts[str(p.relative_to(w))] = hashlib.sha256(p.read_bytes()).hexdigest()
        z['meister_messung'] = dict(klasse=task['klasse'], modell_sha=M.sha(doc), eingang=task['sha'],
                                    artefakte=artifacts, seed=1, budget='arbeit', arbeit=arbeit,
                                    laufzeit=runtime,laufzeit_sha=runtime_sha,
                                    last=os.getloadavg(), quell_sha={p:hashlib.sha256((R/p).read_bytes()).hexdigest()
                                        for p in ('router/route_pf.py', 'analyse/klassen.py')})
        placement = w/'k0/placement.json'
        if placement.exists():

            z['meister_messung']['placement_sha'] = M.sha(json.loads(placement.read_text())['parts'])
        atomar(w/'messung.json', z)
        print('Meistertor '+task['id']+' '+str(z.get('gueltig'))+' '+str(M.zahlen(z)), flush=True)
        return task['id'], z
    with ThreadPoolExecutor(parallel) as pool:
        result = dict(pool.map(eins, satz))
    atomar(folder/'ergebnisse.json', result)
    return result


def reihenfolge(satz, referenzen, old=None, new=None, arbeit=59147):
    ""
    def key(task):
        row = referenzen.get(task['id'], ({}, None))[0]
        cpu = row.get('cpu_total')
        if type(cpu) not in (int, float): cpu = float('inf')
        unchanged = (old is not None and new is not None and
                     konfiguration(task,old,arbeit) == konfiguration(task,new,arbeit))
        return (row.get('gueltig') is not True, unchanged, cpu, int(task['id'].split('/s')[1].split('_')[0])
                if '/s' in task['id'] else 0, task['id'])
    return sorted(satz, key=key)


def sequentiell(satz, old, new, out, parallel, arbeit, it, referenzen=None):
    ""
    before, after = {}, {}
    from gemeinsam.cpu_baum import CpuTree
    monitor = CpuTree(subprocess.Popen)
    cpu_start = monitor.sample(os.getpid())
    load_start = os.getloadavg()
    references = referenzen or {}
    planned = reihenfolge(satz, references, old, new, arbeit)
    stop = None
    for index, task in enumerate(planned):
        name = task['id']
        a = messen([task], old, out/'meister', 1, arbeit, it, referenzen=references)[name]
        origin = (a.get('wiederverwendung') or {}).get('ordner', str(out/'meister'/name))
        b = messen([task], new, out/'herausforderer', 1, arbeit, it, referenzen={name:(a,origin)})[name]
        before[name], after[name] = a,b
        if a.get('gueltig') is True and b.get('gueltig') is not True:
            stop = 'Gueltiges Board verloren: '+name
        elif a.get('gueltig') is True and b.get('gueltig') is True and M.qualitaet(a,b):
            stop = 'Regelregression: '+name
        elif any(z.get('gueltig') is None or z.get('zustand') not in ('ok','placer_kollision') for z in (a,b)):
            stop = 'Unbekannter Abschluss: '+name
        else:
            remaining = planned[index+1:]




            unchanged = all(konfiguration(t,old,arbeit) == konfiguration(t,new,arbeit) for t in remaining)
            gained = any(after[k].get('gueltig') is True and before[k].get('gueltig') is not True for k in before)
            ds = [M.gesamtrest(before[k])-M.gesamtrest(after[k]) for k in before
                  if M.gesamtrest(before[k]) is not None and M.gesamtrest(after[k]) is not None]
            optimistic = M.signifikanz(ds+[0]*len(remaining))
            if remaining and unchanged and not gained and (
                    optimistic['p'] > optimistic['alpha'] or sum(ds) <= 0):
                stop = 'Vorzeichentest/Gesamtrest nicht erreichbar; verbleibende Konfigurationen identisch'
        if stop: break
    for arm, rows in (('meister',before),('herausforderer',after)):
        atomar(out/arm/'ergebnisse.json', rows)
    return before, after, dict(vollstaendig=len(before)==len(satz), gemessen=len(before), geplant=len(satz),
                              reihenfolge=[t['id'] for t in planned], frueh_abgelehnt=stop,
                              cpu_s=monitor.sample(os.getpid())-cpu_start,
                              cpu_messung='libproc-Prozessbaum, Stichprobe vor/nach Sequenz',
                              last_vorher=load_start,last_nachher=os.getloadavg())

def nativ(w, A, z, cpu_budget=None):
    ""
    from kicad.python_aufruf import python_befehl
    from codex.bga_fanout.polygonringe import kupfer
    w=Path(w);c = w/('k%d' % z.get('kandidat',0)); source=c/'native_basis/base.kicad_pcb'; out=c/'meister_nativ';out.mkdir()
    runs=[]
    def call(cmd,log):
        limit=min(900,cpu_budget.rest) if cpu_budget is not None else 900
        if limit<1:raise RuntimeError('Kein CPU-Rest fuer native Bewertung')
        run=PL.lauf(list(map(str,cmd)),cpu_s=limit,log=str(log),cwd=R,
                    env=dict(os.environ,PYTHONHASHSEED='0'))
        runs.append(run)
        if '_cpu_kind_s' in z:z['_cpu_kind_s']+=run['cpu_s']
        else:z['cpu_total']=(z.get('cpu_total') or 0)+run['cpu_s']
        if cpu_budget is not None:cpu_budget.add(run['cpu_s'])
        if run['zustand']!='ok':raise RuntimeError('Native Tormessung: '+str(run))
    if not source.exists():
        board=json.loads((c/'board.json').read_text())
        if not str(board.get('src','')).startswith('aufgabe seed='):
            raise ValueError('Native Selbstspielbewertung braucht erzeugte Aufgabe')
        source.parent.mkdir(exist_ok=True)
        v11=str(A.get('version','')).split('+')[0]=='11'
        command=(python_befehl(R/'codex/generator_echt/nativ.py',c/'board.json',source,w) if v11 else
                 python_befehl(R/'kicad/strom_board.py',c/'board.json',source))
        call(command,out/'aufbau.log')
        if not v11:
            from lernen.kupfer_nativ import regelvertrag
            project,dru=regelvertrag(board,dict(optionen={}),json.loads((w/'regeln.json').read_text()))
            atomar(source.with_suffix('.kicad_pro'),project);source.with_suffix('.kicad_dru').write_text(dru)
        zones=(json.loads((w/'zones.json').read_text()) if v11 else
               [dict(net='GND',layer=layer,pts=poly,full=True,prio=0) for layer in A['gndlagen'] for poly in board['outline']])
        atomar(source.parent/'zones.json',zones)
    routes=json.loads((c/'routes.json').read_text())
    zones=routes.get('zones') or json.loads((c/'native_basis/zones.json').read_text())
    atomar(out/'zones.json', zones)
    rules=json.loads((w/'regeln.json').read_text())
    commands=[python_befehl(R/'kicad/import_routes.py', source, c/'routes.json', out/'zones.json',
                          out/'filled.kicad_pcb', 'projekt=1', 'lagen='+str(A['lagen']), 'via='+rules['router']['via']),
              ['/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli', 'pcb','drc','--format','json',
               '--severity-all','-o',str(out/'drc.json'),str(out/'filled.kicad_pcb')],
              python_befehl(R/'kicad/kupfer_export.py',out/'filled.kicad_pcb',out/'cu_roh.json','polygone_exakt=1')]
    for i, cmd in enumerate(commands):
        call(cmd,out/(str(i)+'.log'))
    atomar(out/'cu.json', kupfer(json.loads((out/'cu_roh.json').read_text())))
    d=json.loads((out/'drc.json').read_text())
    if not all(k in d for k in ('violations','unconnected_items','kicad_version')):raise ValueError('DRC unvollstaendig')
    errors=sum(v['severity']=='error' for v in d['violations']);opened=len(d['unconnected_items'])

    arten={};[arten.__setitem__(v['type'],arten.get(v['type'],0)+1) for v in d['violations'] if v['severity']=='error']
    z['native_meister']=dict(fehler=errors,offen=opened,arten=dict(sorted(arten.items())),version=d['kicad_version'],laeufe=runs)
    z['gueltig_modell']=z['gueltig'];z['gueltig']=z['gueltig'] and not errors and not opened
    z.setdefault('ergebnis',{})['drc_fehler']=errors


def vorprobe(snapshot, satz, out, parallel):

    v10 = next(Path(t['pfad']).parent for t in satz if t['id'].startswith('v10/'))
    env = dict(os.environ, AUTOROUTER_MEISTER=str(snapshot), VORPROBE_AUFGABEN=str(v10),
               PYTHONHASHSEED='0')
    cmd = [sys.executable, str(R/'tests/vorprobe.py'), 'parallel='+str(parallel)]

    cmd = [sys.executable, '-S', str(R/'gemeinsam/prozessschutz.py'), 'koord=1', '--'] + cmd
    p = subprocess.run(cmd, cwd=R, env=env, stdout=(Path(out)/'vorprobe.log').open('w'), stderr=subprocess.STDOUT)
    return dict(argv=cmd, exit=p.returncode, log_sha=hashlib.sha256((Path(out)/'vorprobe.log').read_bytes()).hexdigest())

def uebernehmen(path, old, new, report, history, kurve=None):
    ""
    if not report['tor']['angenommen'] or report['vorprobe']['exit'] != 0:
        raise ValueError('Meisteruebernahme ohne bestandenes Tor')
    if report.get('vorher_sha') != M.sha(old) or report.get('nachher_sha') != M.sha(new):
        raise ValueError('Tormessung passt nicht zu Meister/Herausforderer')
    bindings=report.get('ergebnisse_sha') or {}
    if set(bindings)!={'meister','herausforderer'} or any(len(v)!=64 for v in bindings.values()):
        raise ValueError('Tormessung ohne gebundene Einzelbelege')
    if M.sha(M.lesen(path)) != M.sha(old): raise ValueError('Meister inzwischen geaendert')
    report_sha = M.sha(report)
    generation = old['generation'] + 1
    changed = []
    for kl, entry in new['klassen'].items():
        if entry['parameter'] != old['klassen'].get(kl, {}).get('parameter', {}):
            entry.update(generation=generation, beleg_sha=report_sha); changed.append(kl)
    event = dict(version=M.VERSION, generation=generation, zeit=time.strftime('%Y-%m-%d %H:%M:%S'),
                 klassen=changed, tor=report['tor'], beleg_sha=report_sha, beleg=report['pfad'],
                 vorher_sha=M.sha(old), code=report['code'], messvertrag=report['messvertrag'],
                 kurvenpunkte=report.get('kurvenpunkte', []))
    new['generation'] = generation
    new['verlauf'] = old.get('verlauf', []) + [event]


    atomar(path, new)
    verlauf_sync(new, history)
    if kurve: kurve_sync(new, kurve)
    return event


def verlauf_sync(doc, history):
    rows = jsonl(history)
    known = {x.get('beleg_sha') for x in rows}
    Path(history).parent.mkdir(parents=True, exist_ok=True)
    with Path(history).open('a') as f:
        for event in doc.get('verlauf', []):
            if event['beleg_sha'] not in known: f.write(json.dumps(event, sort_keys=True)+'\n')
        f.flush(); os.fsync(f.fileno())

def kurve_sync(doc, path):
    known={r.get('meister_punkt_id') for r in jsonl(path)}
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with Path(path).open('a') as f:
        for event in doc.get('verlauf', []):
            for point in event.get('kurvenpunkte', []):
                ident=event['beleg_sha']+':'+point['generatorversion']
                if ident in known: continue
                f.write(json.dumps(dict(point,meister_punkt_id=ident,meister_generation=event['generation'],
                    zeit=event['zeit'][:16],code=event['code'],meister_beleg=event['beleg_sha']))+'\n')
        f.flush();os.fsync(f.fileno())


def kurvenpunkte(before, after, report):
    points=[]
    for v in ('10','11'):
        keys=[k for k in sorted(after) if k.startswith('v'+v+'/')]
        def part(z):
            r=M.zahlen(z)
            return dict(gueltig=z.get('gueltig'),note_v3=z.get('note_v3'),restkonflikte=r[0] if r else None,
                        native_drc_fehler=M.native_fehler(z),gesamtrest=M.gesamtrest(z),
                        offen=r[1] if r else None,zustand=z.get('zustand'))
        results=[dict(part(after[k]),aufgabe=k,stufe=after[k].get('stufe_aufgabe'),ref=part(before[k])) for k in keys]
        points.append(dict(generatorversion=v,satz='meistertor_v'+v,aufgaben=len(keys),
            gueltig=sum(after[k].get('gueltig') is True for k in keys),gueltig_ref=sum(before[k].get('gueltig') is True for k in keys),
            unbekannt=sum(after[k].get('gueltig') is None for k in keys),bewertungsziel='v3',
            aufgaben_sha=report['satz_sha'],messvertrag=report['messvertrag'],ergebnisse=results,
            cpu_je_gueltig=None,gueltig_je_stufe={str(s):sum(r['gueltig'] is True for r in results if r['stufe']==s) for s in (0,2,4,6,8)}))
    return points


def zielklassen(satz, kl, pool='auto'):
    ""
    if pool not in ('auto','familie'): raise ValueError('pool=auto|familie')
    local = [t for t in satz if M.familie(t['klasse']) == M.familie(kl)]
    selected = local
    level = 'lagen_gehaeuse'
    if pool == 'auto' and len(local) < 5:
        package = M.familie(kl).split('_',1)[1]
        selected = [t for t in satz if M.familie(t['klasse']).split('_',1)[1] == package]
        level = 'gemeinsames_gehaeuse'
    return sorted({t['klasse'] for t in selected}), dict(ebene=level,
        familien_paare=len(local),geplante_paare=len(selected),
        kleinstes_p_ohne_bindungen=2.**(-len(selected)),alpha=.05)


def pruefen(kw):
    ""
    from lernen.lernkurve import HAUPT
    root = Path(kw.get('wurzel') or Path(HAUPT)/'codex/selbstlernen/ergebnisse/meister')
    root.mkdir(parents=True, exist_ok=True)
    model = Path(kw.get('modell') or M.datei())
    data = Path(kw.get('daten') or Path(HAUPT)/'lernen/daten/selbstspiel.jsonl')
    history = Path(kw.get('verlauf') or model.parent.parent/'daten/meister_verlauf.jsonl')
    curve = Path(kw.get('kurve') or model.parent.parent/'daten/lernkurve.jsonl')
    with (root/'dienst.lock').open('a') as lock:
        try: fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError: return dict(status='anderes Tor laeuft')
        old = M.lesen(model); verlauf_sync(old, history); kurve_sync(old, curve)
        rows = jsonl(data)
        statepath = root/'stand.json'; state = json.loads(statepath.read_text()) if statepath.exists() else {}
        code = kopf()
        from lernen.sammeln import code_stand
        source_sha=code_stand()
        satz=tasks(root/'torsatz')
        covered={t['klasse'] for t in satz};uncovered=[]
        choices = []
        for kl in sorted({M.zeilenklasse(r) for r in rows} - {None}):
            if not kl.startswith(('v10:','v11:')):continue
            if kl not in covered:
                uncovered.append(kl);continue
            if kw.get('klasse') and kl != kw['klasse']: continue
            rs = [r for r in rows if M.zeilenklasse(r) == kl and M.belohnung(r) is not None and r.get('cfg', {}).get('kosten')]
            context=M.sha(dict(code=source_sha,meister=M.sha(old)))
            previous=state.get(kl,{})
            blocked=list(previous.get('pruefungen',{}).get(context,[]))
            if kl in old['klassen']:blocked.append(M.sha(old['klassen'][kl]['parameter']))
            candidate = M.herausforderer(rows, kl, ausgeschlossen=blocked)
            if len(rs) - state.get(kl, {}).get('n', 0) < int(kw.get('neu', '30')) or not candidate: continue
            if candidate == old['klassen'].get(kl, {}).get('parameter'): continue
            rewards=[M.belohnung(r) for r in rs if r['cfg']['kosten']==candidate]
            score=tuple(sum(b[i] for b in rewards)/len(rewards) for i in range(5))
            choices.append((score, kl, candidate, len(rs)))
        if not choices: return dict(status='nicht faellig',ohne_torsatz=uncovered)
        _, kl, candidate, n = min(choices,key=lambda x:(x[0],x[1]))
        out = root/('tor_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]); out.mkdir()
        from gemeinsam.cpu_baum import CpuTree
        tor_monitor = CpuTree(subprocess.Popen)
        tor_cpu_start = tor_monitor.sample(os.getpid())
        tor_last_start = os.getloadavg()
        new = json.loads(json.dumps(old))


        targets, pooling = zielklassen(satz, kl, kw.get('pool','auto'))
        for target in targets:
            new['klassen'][target] = dict(parameter=candidate, generation=old['generation']+1, beleg_sha='herausforderer')
        snapshot = out/'herausforderer.json'; atomar(snapshot, new)
        par = int(kw.get('parallel', '3')); arbeit = int(kw.get('arbeit', '59147')); it = int(kw.get('iter', '40'))
        runtime=laufzeit()
        report = dict(version='meister-tor-v2', klasse=kl, zielklassen=targets, pooling=pooling, code=code, pfad=str(out/'tor.json'),
                      vorprobe=dict(exit=None,status='Nur vor einer tatsaechlichen Uebernahme ausfuehren'),
                      messvertrag=dict(budget='arbeit', arbeit=arbeit, iter=it, seed=1, saetze=('v10', 'v11'),
                          gueltigkeit='bewertung+nativ-v1',rest='konflikte+max(modell_offen,nativ_offen)+native_fehler',belohnung=M.BELOHNUNG_VERSION,
                          statistik='einseitiger exakter Vorzeichentest alpha=0.05, Gesamtrest muss sinken',
                          wiederverwendung='Nur identische Quelle/Eingabe/cfg/Seed/Budget mit unveraenderten nativen Originalbelegen; kein neuer Lauf'),
                      kandidat=candidate, trainingszeilen=n, quell_sha=source_sha, satz_sha=M.sha(satz), vorher_sha=M.sha(old), nachher_sha=M.sha(new),laufzeit=runtime)


        before, after, sequence = sequentiell(satz, old, new, out, par, arbeit, it,
                                              referenzen=letzte_referenzen(root))
        report['sequenz'] = sequence
        retryable=any(z.get('gueltig') is None or z.get('zustand') not in ('ok','placer_kollision')
                      for arm in (before,after) for z in arm.values())
        report['tor'] = M.tor(before, after)
        if not sequence['vollstaendig']:
            report['tor']['angenommen'] = False
            report['tor']['gruende'].append('Satz unvollstaendig: '+str(sequence['frueh_abgelehnt']))
        report['ergebnisse_sha']=dict(meister=M.sha(before),herausforderer=M.sha(after))
        report['beleg_ordner']={arm:{k:(z.get('wiederverwendung') or {}).get('ordner',str(out/arm/k))
                                    for k,z in rows_.items()} for arm,rows_ in (('meister',before),('herausforderer',after))}
        report['wiederverwendet']=sum(bool(z.get('wiederverwendung')) for z in after.values())
        report['wiederverwendet_vorher']=sum(bool(z.get('wiederverwendung')) for z in before.values())
        report['kurvenpunkte'] = kurvenpunkte(before, after, report)
        if report['tor']['angenommen']:
            report['vorprobe'] = vorprobe(snapshot, satz, out, par)
            if report['vorprobe']['exit'] != 0:
                retryable=True
                report['tor']['angenommen']=False
                report['tor']['gruende'].append('Vorprobe fehlgeschlagen')
        if code_stand()!=source_sha or any(z.get('code_sha256')!=source_sha for arm in (before,after) for z in arm.values()):
            report['tor']['angenommen']=False
            report['tor']['gruende'].append('Code waehrend Tormessung geaendert')
        if laufzeit()!=runtime or any(z['meister_messung'].get('laufzeit_sha')!=M.sha(runtime) for arm in (before,after) for z in arm.values()):
            report['tor']['angenommen']=False
            report['tor']['gruende'].append('Laufzeitumgebung waehrend Tormessung geaendert')
        report['cpu_s'] = tor_monitor.sample(os.getpid())-tor_cpu_start
        report['cpu_messung'] = 'libproc-Prozessbaum vor/nach Tor, einschliesslich Vorprobe'
        report['last_vorher'] = tor_last_start
        report['last_nachher'] = os.getloadavg()
        atomar(out/'tor.json', report)
        if report.get('ergebnisse_sha'):


            atomar(root/'letzte_messung.json',dict(bericht=str(out.resolve()/'tor.json'),
                sha256=hashlib.sha256((out/'tor.json').read_bytes()).hexdigest(),
                arm='herausforderer' if report['tor']['angenommen'] else 'meister'))
        if report['tor']['angenommen']:
            event = uebernehmen(model, old, new, report, history, curve)
            atomar(out/'uebernahme.json', event)
        context=M.sha(dict(code=source_sha,meister=M.sha(old)))
        tried=state.get(kl,{}).get('pruefungen',{})
        if not retryable:tried.setdefault(context,[]).append(M.sha(candidate))
        state[kl] = dict(n=n, code=code, kandidat_sha=M.sha(candidate), tor=str(out/'tor.json'),pruefungen=tried,
                        status='unbekannt_erneut_nach_neuen_daten' if retryable else 'entschieden')
        atomar(statepath, state)
        with (root/'zyklen.jsonl').open('a') as f:
            f.write(json.dumps(dict(zeit=time.time(),bericht=report['pfad'],tor=report['tor'],
                generation=M.lesen(model)['generation'],sequenz=report.get('sequenz')))+'\n')
            f.flush();os.fsync(f.fileno())
        return report


if __name__ == '__main__':
    from gemeinsam.prozessschutz import bootstrap
    bootstrap(heavy=False)
    print(json.dumps(pruefen(dict(a.split('=', 1) for a in sys.argv[1:] if '=' in a)), indent=1))
