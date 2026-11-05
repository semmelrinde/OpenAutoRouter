import json
from pathlib import Path
import re
import shutil

from collections import Counter

from postprocessing.thermal_island import KC, ROOT, faults


def open_per_net(drc):
    c = Counter()
    for u in drc['unconnected_items']:
        if u.get('excluded'): continue
        c[tuple(sorted({m.group(1) for i in u.get('items', []) for m in [re.search(r'\[([^\]]*)\]', i.get('description', ''))] if m}))] += 1
    return c


def no_regression(before, after):
    return not (faults(after)[0] - faults(before)[0]) and not (open_per_net(after) - open_per_net(before))


def active(routes):
    value = str((routes.get('options') or {}).get('thermal_spokes', '1'))
    if value not in ('0', '1'):
        raise ValueError('thermal_spokes must be 0 or 1')
    return value == '1'


def findings(drc):
    if not all(k in drc for k in ('violations', 'unconnected_items')):
        raise ValueError('Incomplete native DRC report')
    return [v for v in drc['violations'] if v.get('type') == 'starved_thermal' and v.get('severity') == 'error'
            and not v.get('excluded') and not re.search(r'island|Island', v.get('description', ''))]


def finish(pcb, routes, call, attempts=3):
    if not active(routes): return None
    from kicad.python_call import python_command
    from kicad.project_copy import design_rules_save, design_rules_copy
    pcb = Path(pcb).resolve()
    rules = design_rules_save(pcb)
    folder = pcb.parent / (pcb.stem + '_thermal_spokes')
    folder.mkdir(exist_ok=False)
    report = dict(version='thermal-spokes-v1', before=None, after=None, taken=0, pieces=[], attempts=[])
    def drc(path, tag):
        dst = folder / (tag + '.drc.json')
        call([KC, 'pcb', 'drc', '--refill-zones', '--save-board', '--format', 'json', '--severity-all', '-o', str(dst), str(path)],
             folder / (tag + '.drc.log'))
        design_rules_copy(rules, path, generated_pro=True)
        return json.loads(dst.read_text())
    working = folder / 'working.kicad_pcb'
    shutil.copy2(pcb, working); design_rules_copy(rules, working)
    current = drc(working, 'before'); report['before'] = len(findings(current))
    exclusion = []
    try:
        for i in range(attempts):
            if not findings(current): break
            cand = folder / ('candidate%d.kicad_pcb' % i); before = folder / ('suggestion%d.json' % i); ex = folder / ('exclusion%d.json' % i)
            ex.write_text(json.dumps(exclusion) + '\n')
            call(python_command(ROOT / 'kicad/spokes_set.py', working, folder / ('%s.drc.json' % ('before' if i == 0 else 'state%d' % (i - 1))),
                               cand, before, 'exclusion=' + str(ex)), folder / ('suggestion%d.log' % i))
            pieces = json.loads(before.read_text())
            if not pieces: break
            design_rules_copy(rules, cand, generated_pro=True)
            after = drc(cand, 'candidate%d' % i)
            ok = len(findings(after)) < len(findings(current)) and no_regression(current, after)
            report['attempts'].append(dict(pieces=len(pieces), taken=ok, starved=len(findings(after))))
            if ok:
                shutil.copy2(cand, working); design_rules_copy(rules, working, generated_pro=True)
                current = after; report['taken'] += len(pieces); report['pieces'] += pieces
                shutil.copy2(folder / ('candidate%d.drc.json' % i), folder / ('state%d.drc.json' % i))
            else:

                exclusion += [[s['pad'], s['layer_name'], s['direction']] for s in pieces]
                shutil.copy2(folder / ('%s.drc.json' % ('before' if i == 0 else 'state%d' % (i - 1))), folder / ('state%d.drc.json' % i))
        report['after'] = len(findings(current))
        report['open_items'] = [dict(description=v['description'], items=v['items']) for v in findings(current)]
        if report['taken']:
            shutil.copy2(working, pcb); design_rules_copy(rules, pcb, generated_pro=True)
        (folder / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        return report
    except Exception as exc:
        report['faults'] = repr(exc); (folder / 'report.json').write_text(json.dumps(report, indent=2) + '\n'); raise


def main():
    import subprocess, sys
    def call(argv, log):
        with open(log, 'w') as f:
            r = subprocess.run([str(a) for a in argv], stdout=f, stderr=subprocess.STDOUT)
        if r.returncode not in (0, 5): raise RuntimeError('Call failed (%d): %s' % (r.returncode, log))
    print(json.dumps(finish(sys.argv[1], {}, call), indent=1, ensure_ascii=False))


if __name__ == '__main__':
    main()
