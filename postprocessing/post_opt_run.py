import subprocess, json, sys, shutil, collections, os, re
KPY = os.environ.get('POST_OPT_KPY') or '/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3'
VPY = 'python3'
if not os.path.exists(VPY): VPY = sys.executable
A = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, A)
from common.drc_events import new_events, count
cur, PRO, DRU, NETS, PFX = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4].split(';'), sys.argv[5]
OPT = dict(a.split('=', 1) for a in sys.argv[6:] if '=' in a)
WIN = (OPT.get('window') or os.environ['WINDOW']).split(',')
OFF = (OPT.get('excerpt') or os.environ['EXCERPT']).split(',')
LAYERS = OPT.get('cu_layers') or os.environ.get('LAYERS', '')
EXPENSIVE = os.environ.get('EXPENSIVE', '')
KN = os.environ.get('BEND', '0.6'); VIA = os.environ.get('VIA', '2.0')
EXTRA = os.environ.get('POST_OPT_OPT', '').split()


def hole_spacing():
    value_list = [0.25]
    try: value_list.append(float(json.load(open(PRO))['board']['design_settings']['rules']['min_hole_clearance']))
    except (OSError, KeyError, ValueError, TypeError): pass
    try:
        txt = '\n'.join(l for l in open(DRU).read().splitlines() if not l.lstrip().startswith('#'))
        value_list += [float(x) for x in re.findall(r'\(constraint\s+hole_clearance\s+\(min\s+([0-9.]+)\s*mm\)', txt)]
    except OSError: pass
    return max(value_list)


HOLE = OPT.get('hole_size') or '%.3f' % hole_spacing()


def hole_spacing_via():
    try: return max(0.25, float(json.load(open(PRO))['board']['design_settings']['rules']['min_hole_clearance']))
    except (OSError, KeyError, ValueError, TypeError): return 0.25


HOLE_VIA = OPT.get('hole_via') or '%.3f' % hole_spacing_via()


CLI = os.environ.get('POST_OPT_CLI') or shutil.which('kicad-cli') or '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
LAST = {}


def drc(pcb):
    base = pcb[:-10]; shutil.copy(PRO, base + '.kicad_pro'); shutil.copy(DRU, base + '.kicad_dru')
    if os.path.exists(base + '_drc.json'): os.remove(base + '_drc.json')
    r = subprocess.run([CLI, 'pcb', 'drc', '--format', 'json', '--severity-all', '-o', base + '_drc.json', pcb], capture_output=True, text=True)
    LAST.clear()
    if r.returncode != 0 or not os.path.exists(base + '_drc.json'): return 10 ** 6, 10 ** 6, 10 ** 6, 10 ** 6
    d = json.load(open(base + '_drc.json'))
    if 'violations' not in d or 'unconnected_items' not in d: return 10 ** 6, 10 ** 6, 10 ** 6, 10 ** 6
    LAST.update(d=d, ev=count(d))
    c = collections.Counter((v['severity'], v['type']) for v in d['violations'])
    warn = sum(v for k, v in c.items() if k[0] == 'warning')
    return sum(v for k, v in c.items() if k[0] == 'error'), len(d['unconnected_items']), c.get(('warning', 'track_dangling'), 0) + c.get(('warning', 'via_dangling'), 0), warn


def score(L, V, S): return L + float(VIA) * V + float(KN) * S


shutil.copy(cur, PFX + '_start.kicad_pcb')
_e0, _u0, _d0, W0 = drc(PFX + '_start.kicad_pcb')
if not LAST: raise SystemExit('DRC of the start board failed')
REF = dict(LAST)
print('Start: DRC errors %d, open %d, loose %d, warnings %d; drill spacing %s mm%s' % (_e0, _u0, _d0, W0, HOLE, (', layers ' + LAYERS) if LAYERS else ''), flush=True)
k = 0
for net in NETS:
    for f in ('h_cur.json', 'o_cur.json'):
        if os.path.exists(f): os.remove(f)
    if subprocess.run([KPY, A + '/kicad/obst_export.py', cur] + OFF + ['h_cur.json'], capture_output=True).returncode != 0 or not os.path.exists('h_cur.json'):
        print('   obst_export failed', flush=True); continue
    r = subprocess.run([VPY, A + '/postprocessing/post_opt.py', 'h_cur.json', net] + WIN + (['expensive=' + EXPENSIVE] if EXPENSIVE else [])
                       + (['cu_layers=' + LAYERS] if LAYERS else []) + ['bend=' + KN, 'via=' + VIA, 'hole_size=' + HOLE, 'hole_via=' + HOLE_VIA, 'w=auto', 'via_size=auto'] + EXTRA
                       + ['o_cur.json'], capture_output=True, text=True)
    print(r.stdout.strip() or r.stderr.strip()[-300:], flush=True)
    if r.returncode != 0: continue
    o = json.load(open('o_cur.json'))
    if not o['tracks']: continue
    H = json.load(open('h_cur.json')); rip = set(o['ripped'])
    old_segs = sum(1 for t in H['tracks'] if t['uuid'] in rip)
    new_segs = sum(len(t['pts']) - 1 for t in o['tracks'])
    so, sn = score(o['old_len'], o['old_vias'], old_segs), score(o['new_len'], len(o['vias']), new_segs)
    if len(o['vias']) > o['old_vias']: print('   rejected (more vias: %d -> %d, R20)' % (o['old_vias'], len(o['vias']))); continue
    if sn > so - 0.3: print('   rejected (evaluation %.1f -> %.1f, segments %d -> %d)' % (so, sn, old_segs, new_segs)); continue
    k += 1; nxt = '%s_%02d.kicad_pcb' % (PFX, k)
    if os.path.exists(nxt): os.remove(nxt)
    ra = subprocess.run([KPY, A + '/kicad/apply_route.py', cur, 'o_cur.json', ','.join(o['ripped']), nxt, 'fill'], capture_output=True, text=True)
    if ra.returncode != 0 or not os.path.exists(nxt): print('   apply_route failed -> rejected', ra.stderr[-300:], flush=True); continue
    e, u, dg, wn = drc(nxt)
    new = new_events(LAST, REF) if LAST else ['DRC failed']
    if LAST and any(isinstance(v, dict) and v.get('type') in ('track_dangling', 'via_dangling') for v in new):

        subprocess.run([KPY, A + '/kicad/loose_shorten.py', nxt, nxt[:-10] + '_drc.json', nxt, net], capture_output=True)
        e, u, dg, wn = drc(nxt)
        new = new_events(LAST, REF) if LAST else ['DRC failed']


    if LAST and not new:
        print('   ACCEPTED (evaluation %.1f -> %.1f, segments %d -> %d) -> %s' % (so, sn, old_segs, new_segs, nxt), flush=True)
        cur = nxt; REF = dict(LAST)
    else:
        kinds = collections.Counter(v['type'] if isinstance(v, dict) else v for v in new)
        print('   DRC problem (errors %d, open %d, loose %d, new reports %d %s) -> rejected' % (e, u, dg, len(new), dict(kinds)), flush=True)
print('END', cur)
