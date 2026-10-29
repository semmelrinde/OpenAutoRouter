import json, math, os, shutil, subprocess, sys, tempfile, time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from common import slots as PL
KP = '/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3'
KC = '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'


class NativeBudget:
    def __init__(self, cpu_s=300):
        self.limit = max(1., float(cpu_s)); self.spent = 0.; self.start = time.thread_time(); self.runs = []

    def run(self, cmd, **kwargs):
        rest = self.limit-self.spent-(time.thread_time()-self.start)
        if rest <= 0: raise TimeoutError('R44: native CPU budget reached')
        with tempfile.TemporaryDirectory(prefix='r44-cpu-') as tmp:
            log = Path(tmp)/'run.log'
            run = PL.job(cmd, cpu_s=max(1,math.floor(rest)), log=str(log), with_slot=False)
            self.runs.append(run); self.spent += run['cpu_s']
            output = log.read_text(errors='replace')
        if run['cur_state'] != 'ok': raise RuntimeError('R44 native '+run['cur_state']+': '+output[-400:])
        return subprocess.CompletedProcess(cmd,run['rc'],stdout=output,stderr='')


def native_run(cmd, cpu_s=300, **kwargs):
    return NativeBudget(cpu_s).run(cmd, **kwargs)


def save(source, target, report=None, cpu_s=300):
    source, target = Path(source), Path(target)
    removed = []
    budget = NativeBudget(max(1,float(cpu_s)*.9))
    result = dict(version=2, removed_copper_parts=removed, checked=False)
    try:
        with tempfile.TemporaryDirectory(prefix='autorouter-r44-native-') as tmp:
            tmp = Path(tmp); current = tmp/'board.kicad_pcb'
            shutil.copyfile(source,current)
            for ext in ('.kicad_pro','.kicad_dru'):
                if source.with_suffix(ext).exists(): shutil.copyfile(source.with_suffix(ext),current.with_suffix(ext))
            try:
                _verify(source,target,budget,result,current,tmp,removed)
            except Exception as exc:

                result.update(checked=False,reason=str(exc)[:400],remaining_contacts=None,native_short_circuits=None)

                reserve = NativeBudget(max(1,float(cpu_s)-budget.spent-(time.thread_time()-budget.start)))
                empty = tmp/'open_items.kicad_pcb'
                try:
                    reserve.run([KP,str(ROOT/'kicad/short_circuit_open.py'),str(current),str(empty),'all_items'])
                    shutil.copyfile(empty,target)
                    result['export_open'] = True
                except Exception as fallback: result['open_error'] = str(fallback)[:300]
                budget.spent += reserve.spent; budget.runs.extend(reserve.runs)
    except Exception as exc:
        result.update(checked=False,reason=str(exc)[:400],remaining_contacts=None,native_short_circuits=None)
    result.update(cpu_s=round(budget.spent+time.thread_time()-budget.start,3),underflows=budget.runs)
    if report: Path(report).write_text(json.dumps(result,indent=2))
    return result


def _verify(source,target,budget,result,current,tmp,removed):
    def export_check():
        from postprocessing.short_circuit_protection import export_contacts
        for script,output,opts in [('export_board.py','board.json',['output_copper=1']),('routes_export.py','routes.json',['entire=1','unit_list=1'])]:
            budget.run([KP,str(ROOT/'kicad'/script),str(current),str(tmp/output)]+opts)
        b,r = [json.loads((tmp/f).read_text()) for f in ('board.json','routes.json')]
        for p in b['pads']:
            if p.get('board_copper') and not r.get('board_copper_complete'):
                r.setdefault('zones',[]).extend(dict(net=p['net'],layer=L,pts=q,filled=True,unit=p['native_uuid']) for L,qs in p['polys'].items() for q in qs)
        b['pads'] = [p for p in b['pads'] if not p.get('board_copper')]
        return export_contacts(b,r)
    for iteration in range(1000):
        contacts,inherited = export_check()
        drc = tmp/'drc.json'
        budget.run([KC,'pcb','drc','--format','json','--severity-all','-o',str(drc),str(current)])
        data = json.loads(drc.read_text())
        shorts = [e for e in data['violations'] if e['type'] in ('shorting_items','tracks_crossing')]
        inv = budget.run([KP,str(ROOT/'kicad/short_circuit_open.py'),str(current),'-','inventory'])
        mutable = set(json.loads(inv.stdout.strip().splitlines()[-1]))
        edges = [set(i['uuid'] for i in e['items']) & mutable for e in shorts]
        fixed_short = [e for e,s in zip(shorts,edges) if not s]
        edges = [e for e in edges if e]
        edges += [{u for u,category in zip(e['unit_list'],e['kinds']) if category!='pad'} & mutable for e in contacts]
        if any(not e for e in edges): raise ValueError('R44: contact cannot be assigned reliably')
        if not edges:


            result.update(checked=True,remaining_contacts=0,input_pad_contacts=inherited,input_native_short_circuits=fixed_short,
                          native_short_circuits=len(fixed_short),native_short_circuits_movable=0,
                          native_short_circuits_fixed=len(fixed_short),geometric_contacts_fixed=len(inherited))
            break
        chosen = set()
        while edges:
            count = {u:sum(u in e for e in edges) for u in sorted(set.union(*edges))}
            u = min(count,key=lambda u:(-count[u],u)); chosen.add(u)
            edges = [e for e in edges if u not in e]
        selection = tmp/'remove.json'; selection.write_text(json.dumps(sorted(chosen)))
        next_board = tmp/'next.kicad_pcb'
        budget.run([KP,str(ROOT/'kicad/short_circuit_open.py'),str(current),str(next_board),str(selection)])
        os.replace(next_board,current); removed.extend(sorted(chosen))
    else: raise ValueError('R44: no reliable convergence')
    if removed or source != target:
        fd,stage = tempfile.mkstemp(dir=target.parent,suffix='.kicad_pcb'); os.close(fd)
        try: shutil.copyfile(current,stage); os.replace(stage,target)
        finally:
            if os.path.exists(stage): os.unlink(stage)
    return result


if __name__ == '__main__':
    sys.path.insert(0,str(ROOT))
    r = save(*sys.argv[1:])


    sys.exit(0 if r.get('checked') is True else 1)
