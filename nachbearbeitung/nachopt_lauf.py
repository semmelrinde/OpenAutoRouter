""
import subprocess, json, sys, shutil, collections, os, re
KPY = os.environ.get('NACHOPT_KPY') or '/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3'
VPY = 'python3'
if not os.path.exists(VPY): VPY = sys.executable
A = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, A)
from gemeinsam.drc_ereignisse import neue_ereignisse, zaehlen
cur, PRO, DRU, NETS, PFX = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4].split(';'), sys.argv[5]
OPT = dict(a.split('=', 1) for a in sys.argv[6:] if '=' in a)
FEN = (OPT.get('fenster') or os.environ.get('FENSTER', '117,88.5,130.655,102.5')).split(',')
AUS = (OPT.get('ausschnitt') or os.environ.get('AUSSCHNITT', '114,86,133,112')).split(',')
LAGEN = OPT.get('lagen') or os.environ.get('LAGEN', '')
TEUER = os.environ.get('TEUER', '')
KN = os.environ.get('KNICK', '0.6'); VIA = os.environ.get('VIA', '2.0')
EXTRA = os.environ.get('NACHOPT_OPT', '').split()


def lochabstand():
    ""
    werte = [0.25]
    try: werte.append(float(json.load(open(PRO))['board']['design_settings']['rules']['min_hole_clearance']))
    except (OSError, KeyError, ValueError, TypeError): pass
    try:
        txt = '\n'.join(l for l in open(DRU).read().splitlines() if not l.lstrip().startswith('#'))
        werte += [float(x) for x in re.findall(r'\(constraint\s+hole_clearance\s+\(min\s+([0-9.]+)\s*mm\)', txt)]
    except OSError: pass
    return max(werte)


LOCH = OPT.get('loch') or '%.3f' % lochabstand()


def lochabstand_via():
    ""
    try: return max(0.25, float(json.load(open(PRO))['board']['design_settings']['rules']['min_hole_clearance']))
    except (OSError, KeyError, ValueError, TypeError): return 0.25


LOCH_VIA = OPT.get('lochvia') or '%.3f' % lochabstand_via()


CLI = os.environ.get('NACHOPT_CLI') or shutil.which('kicad-cli') or '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
LETZTER = {}


def drc(pcb):
    base = pcb[:-10]; shutil.copy(PRO, base + '.kicad_pro'); shutil.copy(DRU, base + '.kicad_dru')
    if os.path.exists(base + '_drc.json'): os.remove(base + '_drc.json')
    r = subprocess.run([CLI, 'pcb', 'drc', '--format', 'json', '--severity-all', '-o', base + '_drc.json', pcb], capture_output=True, text=True)
    LETZTER.clear()
    if r.returncode != 0 or not os.path.exists(base + '_drc.json'): return 10 ** 6, 10 ** 6, 10 ** 6, 10 ** 6
    d = json.load(open(base + '_drc.json'))
    if 'violations' not in d or 'unconnected_items' not in d: return 10 ** 6, 10 ** 6, 10 ** 6, 10 ** 6
    LETZTER.update(d=d, ev=zaehlen(d))
    c = collections.Counter((v['severity'], v['type']) for v in d['violations'])
    warn = sum(v for k, v in c.items() if k[0] == 'warning')
    return sum(v for k, v in c.items() if k[0] == 'error'), len(d['unconnected_items']), c.get(('warning', 'track_dangling'), 0) + c.get(('warning', 'via_dangling'), 0), warn


def score(L, V, S): return L + float(VIA) * V + float(KN) * S


shutil.copy(cur, PFX + '_start.kicad_pcb')
_e0, _u0, _d0, W0 = drc(PFX + '_start.kicad_pcb')
if not LETZTER: raise SystemExit('DRC des Startboards fehlgeschlagen')
REF = dict(LETZTER)
print('Start: DRC Fehler %d, offen %d, lose %d, Warnungen %d; Bohrungsabstand %s mm%s' % (_e0, _u0, _d0, W0, LOCH, (', Lagen ' + LAGEN) if LAGEN else ''), flush=True)
k = 0
for net in NETS:
    for f in ('h_cur.json', 'o_cur.json'):
        if os.path.exists(f): os.remove(f)
    if subprocess.run([KPY, A + '/kicad/hind_export.py', cur] + AUS + ['h_cur.json'], capture_output=True).returncode != 0 or not os.path.exists('h_cur.json'):
        print('   hind_export fehlgeschlagen', flush=True); continue
    r = subprocess.run([VPY, A + '/nachbearbeitung/nachopt.py', 'h_cur.json', net] + FEN + (['teuer=' + TEUER] if TEUER else [])
                       + (['lagen=' + LAGEN] if LAGEN else []) + ['knick=' + KN, 'via=' + VIA, 'loch=' + LOCH, 'lochvia=' + LOCH_VIA, 'w=auto', 'viamass=auto'] + EXTRA
                       + ['o_cur.json'], capture_output=True, text=True)
    print(r.stdout.strip() or r.stderr.strip()[-300:], flush=True)
    if r.returncode != 0: continue
    o = json.load(open('o_cur.json'))
    if not o['tracks']: continue
    H = json.load(open('h_cur.json')); rip = set(o['ripped'])
    old_segs = sum(1 for t in H['tracks'] if t['uuid'] in rip)
    new_segs = sum(len(t['pts']) - 1 for t in o['tracks'])
    so, sn = score(o['old_len'], o['old_vias'], old_segs), score(o['new_len'], len(o['vias']), new_segs)
    if len(o['vias']) > o['old_vias']: print('   verworfen (mehr Vias: %d -> %d, R20)' % (o['old_vias'], len(o['vias']))); continue
    if sn > so - 0.3: print('   verworfen (Bewertung %.1f -> %.1f, Segmente %d -> %d)' % (so, sn, old_segs, new_segs)); continue
    k += 1; nxt = '%s_%02d.kicad_pcb' % (PFX, k)
    if os.path.exists(nxt): os.remove(nxt)
    ra = subprocess.run([KPY, A + '/kicad/apply_route.py', cur, 'o_cur.json', ','.join(o['ripped']), nxt, 'fill'], capture_output=True, text=True)
    if ra.returncode != 0 or not os.path.exists(nxt): print('   apply_route fehlgeschlagen -> verworfen', ra.stderr[-300:], flush=True); continue
    e, u, dg, wn = drc(nxt)
    neu = neue_ereignisse(LETZTER, REF) if LETZTER else ['DRC fehlgeschlagen']
    if LETZTER and any(isinstance(v, dict) and v.get('type') in ('track_dangling', 'via_dangling') for v in neu):

        subprocess.run([KPY, A + '/kicad/lose_kuerzen.py', nxt, nxt[:-10] + '_drc.json', nxt, net], capture_output=True)
        e, u, dg, wn = drc(nxt)
        neu = neue_ereignisse(LETZTER, REF) if LETZTER else ['DRC fehlgeschlagen']


    if LETZTER and not neu:
        print('   UEBERNOMMEN (Bewertung %.1f -> %.1f, Segmente %d -> %d) -> %s' % (so, sn, old_segs, new_segs, nxt), flush=True)
        cur = nxt; REF = dict(LETZTER)
    else:
        arten = collections.Counter(v['type'] if isinstance(v, dict) else v for v in neu)
        print('   DRC-Problem (Fehler %d, offen %d, lose %d, neue Meldungen %d %s) -> verworfen' % (e, u, dg, len(neu), dict(arten)), flush=True)
print('ENDE', cur)
