""
import json, math, os, shutil, subprocess, sys, tempfile, time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from gemeinsam import plaetze as PL
KP = '/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3'
KC = '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'


class NativeBudget:
    def __init__(self, cpu_s=300):
        self.limit = max(1., float(cpu_s)); self.spent = 0.; self.start = time.thread_time(); self.runs = []

    def run(self, cmd, **kwargs):
        rest = self.limit-self.spent-(time.thread_time()-self.start)
        if rest <= 0: raise TimeoutError('R44: natives CPU-Budget erreicht')
        with tempfile.TemporaryDirectory(prefix='r44-cpu-') as tmp:
            log = Path(tmp)/'lauf.log'
            run = PL.lauf(cmd, cpu_s=max(1,math.floor(rest)), log=str(log), mit_platz=False)
            self.runs.append(run); self.spent += run['cpu_s']
            output = log.read_text(errors='replace')
        if run['zustand'] != 'ok': raise RuntimeError('R44 native '+run['zustand']+': '+output[-400:])
        return subprocess.CompletedProcess(cmd,run['rc'],stdout=output,stderr='')


def native_lauf(cmd, cpu_s=300, **kwargs):
    return NativeBudget(cpu_s).run(cmd, **kwargs)


def sichern(source, target, report=None, cpu_s=300):
    source, target = Path(source), Path(target)
    removed = []
    budget = NativeBudget(max(1,float(cpu_s)*.9))
    result = dict(version=2, entfernte_kupferteile=removed, geprueft=False)
    try:
        with tempfile.TemporaryDirectory(prefix='autorouter-r44-native-') as tmp:
            tmp = Path(tmp); current = tmp/'board.kicad_pcb'
            shutil.copyfile(source,current)
            for ext in ('.kicad_pro','.kicad_dru'):
                if source.with_suffix(ext).exists(): shutil.copyfile(source.with_suffix(ext),current.with_suffix(ext))
            try:
                _pruefen(source,target,budget,result,current,tmp,removed)
            except Exception as exc:

                result.update(geprueft=False,grund=str(exc)[:400],restkontakte=None,native_kurzschluesse=None)

                reserve = NativeBudget(max(1,float(cpu_s)-budget.spent-(time.thread_time()-budget.start)))
                empty = tmp/'offen.kicad_pcb'
                try:
                    reserve.run([KP,str(ROOT/'kicad/kurzschluss_oeffnen.py'),str(current),str(empty),'alle'])
                    shutil.copyfile(empty,target)
                    result['ausgabe_offen'] = True
                except Exception as fallback: result['oeffnen_fehler'] = str(fallback)[:300]
                budget.spent += reserve.spent; budget.runs.extend(reserve.runs)
    except Exception as exc:
        result.update(geprueft=False,grund=str(exc)[:400],restkontakte=None,native_kurzschluesse=None)
    result.update(cpu_s=round(budget.spent+time.thread_time()-budget.start,3),unterlaeufe=budget.runs)
    if report: Path(report).write_text(json.dumps(result,indent=2))
    return result


def _pruefen(source,target,budget,result,current,tmp,removed):
    def export_check():
        from nachbearbeitung.kurzschluss_schutz import ausgabe_kontakte
        for script,output,opts in [('export_board.py','board.json',['ausgabekupfer=1']),('routes_export.py','routes.json',['voll=1','einheiten=1'])]:
            budget.run([KP,str(ROOT/'kicad'/script),str(current),str(tmp/output)]+opts)
        b,r = [json.loads((tmp/f).read_text()) for f in ('board.json','routes.json')]
        for p in b['pads']:
            if p.get('board_copper') and not r.get('board_copper_vollstaendig'):
                r.setdefault('zones',[]).extend(dict(net=p['net'],layer=L,pts=q,filled=True,einheit=p['native_uuid']) for L,qs in p['polys'].items() for q in qs)
        b['pads'] = [p for p in b['pads'] if not p.get('board_copper')]
        return ausgabe_kontakte(b,r)
    for iteration in range(1000):
        contacts,inherited = export_check()
        drc = tmp/'drc.json'
        budget.run([KC,'pcb','drc','--format','json','--severity-all','-o',str(drc),str(current)])
        data = json.loads(drc.read_text())
        shorts = [e for e in data['violations'] if e['type'] in ('shorting_items','tracks_crossing')]
        inv = budget.run([KP,str(ROOT/'kicad/kurzschluss_oeffnen.py'),str(current),'-','inventar'])
        mutable = set(json.loads(inv.stdout.strip().splitlines()[-1]))
        edges = [set(i['uuid'] for i in e['items']) & mutable for e in shorts]
        fixed_short = [e for e,s in zip(shorts,edges) if not s]
        edges = [e for e in edges if e]
        edges += [{u for u,art in zip(e['einheiten'],e['arten']) if art!='pad'} & mutable for e in contacts]
        if any(not e for e in edges): raise ValueError('R44: Kontakt nicht sicher zuordenbar')
        if not edges:


            result.update(geprueft=True,restkontakte=0,eingang_pad_kontakte=inherited,eingang_native_kurzschluesse=fixed_short,
                          native_kurzschluesse=len(fixed_short),native_kurzschluesse_beweglich=0,
                          native_kurzschluesse_fest=len(fixed_short),geometrische_kontakte_fest=len(inherited))
            break
        chosen = set()
        while edges:
            count = {u:sum(u in e for e in edges) for u in sorted(set.union(*edges))}
            u = min(count,key=lambda u:(-count[u],u)); chosen.add(u)
            edges = [e for e in edges if u not in e]
        selection = tmp/'remove.json'; selection.write_text(json.dumps(sorted(chosen)))
        next_board = tmp/'next.kicad_pcb'
        budget.run([KP,str(ROOT/'kicad/kurzschluss_oeffnen.py'),str(current),str(next_board),str(selection)])
        os.replace(next_board,current); removed.extend(sorted(chosen))
    else: raise ValueError('R44: keine sichere Konvergenz')
    if removed or source != target:
        fd,stage = tempfile.mkstemp(dir=target.parent,suffix='.kicad_pcb'); os.close(fd)
        try: shutil.copyfile(current,stage); os.replace(stage,target)
        finally:
            if os.path.exists(stage): os.unlink(stage)
    return result


if __name__ == '__main__':
    sys.path.insert(0,str(ROOT))
    r = sichern(*sys.argv[1:])


    sys.exit(0 if r.get('geprueft') is True else 1)
