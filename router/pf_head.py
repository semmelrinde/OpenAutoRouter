"""PathFinder grid router for KiCad boards (negotiated congestion, rip-up and reroute).

The router is assembled from three parts: router/pf_head.py (options, board model, cost maps),
router/pf_body.py (path search, clearance checks, exact geometry) and router/pf_main.py (negotiation loop,
end phase, repair moves, output). router/build.sh concatenates them into router/route_pf.py; never edit the
generated file.

How it routes:
- Every net is split into routing units (two-pin connections or small trees). Each pass routes every unit with A* on a
  multi-layer grid (numba accelerated, a pure Python search gives bit-identical results), then raises the history and
  present-congestion costs on shared cells, PathFinder style, until no cell is used by two nets.
- With netclasses=<file> (output of analysis/netclasses.py) nets are routed in class order: switch nodes, fast
  signals by rank, current paths (as wide as the free space allows), analog nets (kept away from fast and switching
  nets), GPIO. Ground pads get short stubs and vias to the ground layers named in gnd_layers.
- Clearances are checked against the real pad, track and via geometry (not only the grid), including KiCad design rules
  from design_rules=<file> (output of analysis/design_rules.py).
- When only a few conflicts or unrouted units are left, move=1 enables repair moves (see move_loop).

usage: python3 router/route_pf.py <board.json> <routes.json> [design_rules=design_rules.json] [netclasses=netclasses.json]
       [iter=60] [pf0=1] [pfmul=1.25] [hinc=1.0] [cu_layers=N] [via=0.45/0.20] [gnd_layers=In1] [move=1]
       [move_policy=policy.json] [png=<prefix>] [seed=1] ...
<board.json> comes from kicad/export_board.py or the task generator learning/tasks.py. <routes.json> holds
tracks, vias, remaining conflicts and statistics; kicad/import_routes.py writes them back into a KiCad board.
Same input and seed give the same output file.
"""
import json, math, heapq, sys, time, os, copy
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common.process_guard import bootstrap


_bootstrap_options = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a)
try: _bootstrap_portfolio = int(_bootstrap_options.get('attempts', '0'))
except ValueError: _bootstrap_portfolio = 0
bootstrap(heavy=_bootstrap_portfolio <= 0)
from pathlib import Path
import numpy as np
from scipy import ndimage
try:
    from edt_exact import distance_transform_edt as _edt
except ImportError:

    from scipy.ndimage import distance_transform_edt as _edt
from PIL import Image, ImageDraw
from shapely.geometry import Polygon, Point

def border_polygon(board):
    import shapely
    holes = board.get('outline_holes') or []
    if len(holes) > len(board['outline']):
        raise ValueError('outline_holes without assigned outer ring')
    polygons = [Polygon(q, holes[i] if i < len(holes) else [])
                for i, q in enumerate(board['outline'])]
    if not polygons or any(p.is_empty or not p.is_valid for p in polygons):
        raise ValueError('Invalid Edge.Cuts contours; check native outline')
    result = shapely.union_all(polygons)
    return result


t_start = time.time()
args = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a)


CHAMPION_EVIDENCE = None
if 'netclasses' in args:
    from learning.champion import apply as _champion_apply, check_cli as _champion_check_cli
    CHAMPION_EVIDENCE = _champion_apply(args, json.loads(Path(sys.argv[1]).read_text()), 'router')
    _champion_check_cli(args)



if args.get('critique', '0') not in ('0', '') or any(k in args for k in ('critique_min', 'critique_max', 'critique_until')):
    raise ValueError('critique/critique_min/critique_max/critique_until were removed; see knowledge/codex_options_tidy_up.md')
_ARGS_EXPLICIT = set(args)
STD_ON = 'netclasses' in args and args.get('std', '1') == '1'
if STD_ON:
    for _k, _v in (('pin_access', 'exact'), ('pin_access_rest', '1'), ('pin_access_minimax', '1'),
                   ('short_circuit_protection', '1'), ('netclass_spacing', '1')):
        args.setdefault(_k, _v)
    def _native_basis_present():
        try: _r = json.loads(Path(args['design_rules']).read_text()) if args.get('design_rules') else {}
        except (OSError, ValueError): _r = {}
        _b = Path(args.get('kicad_native_basis') or _r.get('circuit_board') or '')
        return _b.suffix == '.kicad_pcb' and all(_b.with_suffix(x).is_file() for x in ('.kicad_pcb', '.kicad_pro', '.kicad_dru'))
    if _native_basis_present():
        args.setdefault('skew', '1')
        if args.get('budget') == 'work':
            args.setdefault('attempts', '3')
    elif 'skew' not in args or 'attempts' not in args:
        print('Default: skew/attempts off (no native KiCad base)', flush=True)


if args.get('budget', 'cpu') not in ('0', 'cpu', 'work'):
    raise ValueError('budget must be 0, cpu or work')
BUDGET_WORK = args.get('budget') == 'work'
WORK_MEASURE = BUDGET_WORK or args.get('work_measurement') == '1'


WORK_MAX = int(args.get('work_max', '59147'))
if WORK_MAX <= 0: raise ValueError('work_max must be a positive integer')
WORK_SEED = int(args.get('seed', '0')) if BUDGET_WORK else 0
if not 0 <= WORK_SEED < 2**32: raise ValueError('seed must be between 0 and 2**32-1')
if BUDGET_WORK:

    for _name in ('loop', 'agent_loop', 'further', 'remaining_conflicts', 'rest_repair',
                  'learning_data', 'early_abort', 'learning_budget'):
        if args.get(_name, '0') not in ('0', ''):
            raise ValueError('budget=work does not support %s yet' % _name)
_WORK = dict(unit=0, astar=0, checking=0)
_WORK_MARKS = []


def work_step(category):
    work_emergency_stop()
    if WORK_MEASURE: _WORK[category] += 1

def work_emergency_stop():
    if BUDGET_WORK and globals().get('CPU_MAX', 0) > 0 and time.process_time() >= CPU_MAX:


        raise SystemExit('budget=work: CPU emergency brake; no reproducible measurement result')


def work_total():
    return sum(_WORK.values())


def budget_clock():
    return work_total() if BUDGET_WORK else time.process_time()

def work_mark(phase):
    if WORK_MEASURE:
        _WORK_MARKS.append(dict(phase=phase, work=work_total(), counter=dict(_WORK),
                                  cpu_s=time.process_time()))


def work_evidence():
    return dict(version='routing_orders-v1', mode='work' if BUDGET_WORK else 'cpu_measurement',
                maximum=WORK_MAX, consumed=work_total(), counter=dict(_WORK),
                marks=list(_WORK_MARKS), emergency_stop=False)


def budget_rng(salt):


    return np.random.RandomState((int(salt) + 1000003 * WORK_SEED) % 2**32)



def native_route_verify(board, routes, options, folder, export=False):
    import shutil
    from common import slots as PL
    from kicad.python_call import python_command
    root = Path(__file__).resolve().parents[1]
    folder = Path(folder); folder.mkdir(parents=True, exist_ok=True)
    rules = json.loads(Path(options['design_rules']).read_text()) if options.get('design_rules') else {}
    base = Path(options.get('kicad_native_basis') or rules.get('circuit_board') or '')
    if not base.is_file() or base.suffix != '.kicad_pcb':
        raise ValueError('Native acceptance needs kicad_native_basis= or design_rules.circuit_board (.kicad_pcb)')
    for suffix in ('.kicad_pcb', '.kicad_pro', '.kicad_dru'):
        src = base.with_suffix(suffix)
        if not src.is_file(): raise ValueError('Native input missing: ' + str(src))
        shutil.copy2(src, folder / ('base' + suffix))
    rp = folder / 'routes.json'; rp.write_text(json.dumps(routes))
    zones = json.loads(Path(options['kicad_native_zones']).read_text()) if options.get('kicad_native_zones') else routes.get('zones', board.get('zones', []))
    (folder/'zones.json').write_text(json.dumps(zones))
    runs = []
    def call(tag, cmd):
        r = PL.job(list(map(str, cmd)), cpu_s=900, log=str(folder/(tag+'.log')),
                    cwd=root, env=dict(os.environ, PYTHONHASHSEED='0'))
        runs.append(dict(tag=tag, argv=list(map(str,cmd)), **r))
        (folder/'runs.json').write_text(json.dumps(runs, indent=2))
        if r['cur_state'] != 'ok': raise RuntimeError('Native acceptance '+tag+': '+str(r))
    pcb = folder/'board.kicad_pcb'
    call('import', python_command(root/'kicad/import_routes.py', folder/'base.kicad_pcb', rp,
         folder/'zones.json', pcb, 'project=1', 'cu_layers='+str(board.get('cu_layers',4)),
         'via='+options.get('via', (rules.get('router') or {}).get('via','0.45/0.20'))))
    cli = '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
    call('drc', [cli,'pcb','drc','--format','json','--severity-all','-o',folder/'drc.json',pcb])
    d = json.loads((folder/'drc.json').read_text())
    if not all(k in d for k in ('violations','unconnected_items','kicad_version')):
        raise ValueError('Incomplete native DRC report')
    if export:
        call('export', python_command(root/'kicad/routes_export.py',pcb,folder/'native.routes.json','entire=1'))


        (folder/'input_.board.json').write_text(json.dumps(board))
        call('effective', python_command(root/'codex/refboard/native_effective_pads.py',pcb,
                                      folder/'input_.board.json',folder/'native.board.json'))
    return d


def native_skew_error(drc):
    import re
    out = []
    for v in drc['violations']:
        if v['type'] != 'skew_out_of_range' or v['severity'] != 'error': continue
        text = v['description']
        m = re.search(r"rule '(.+)' max skew ([0-9.]+) mm; actual (-?[0-9.]+) mm; target net length ([0-9.]+) mm \(from (.+)\); actual ([0-9.]+) mm", text)
        item = re.search(r'\[([^]]+)\]', v['items'][0]['description'])
        if not m or not item: raise ValueError('Unknown native skew measurement: '+text)
        name,tol,skew,target,partner,length = m.groups()
        if float(skew) >= 0: raise ValueError('Skew post-pass can only lengthen shorter cores')
        out.append(dict(rule=name, net=item[1], partner=partner, tolerance_mm=float(tol),
                        delta_mm=-float(skew), before_mm=float(length), goal_mm=float(target)))
    return sorted(out, key=lambda q:(q['rule'],q['net']))


def multi_attempts(board_path, out, options, count):
    from common import slots as PL
    from analysis.evaluation import rate
    if not options.get('netclasses'): raise ValueError('attempts=N needs netclasses= for score v3')
    if options.get('budget','work')!='work': raise ValueError('attempts=N needs budget=work')
    seed=int(options.get('seed','0'))
    if seed<0 or seed+count>2**32: raise ValueError('attempts: seed range outside uint32')
    folder=Path(out).with_suffix('.attempts');folder.mkdir(parents=True,exist_ok=False)
    board=json.loads(Path(board_path).read_text());classes=json.loads(Path(options['netclasses']).read_text())
    entries=[]
    for offset in range(count):
        current_seed=seed+offset;dest=folder/('seed_%d'%current_seed);dest.mkdir()


        child={k:v for k,v in options.items() if k!='attempts' and (k in _ARGS_EXPLICIT or not STD_ON)}
        child['attempts']='0'
        child.update(seed=str(current_seed),budget='work')
        for name in ('pre','png','labels','history'):
            if name in child: child[name]=str(dest/name)
        argv=[sys.executable,__file__,str(board_path),str(dest/'routes.json')]+[k+'='+str(v) for k,v in child.items()]
        run=PL.job(argv,cpu_s=float(options.get('attempts_cpu_emergency_stop','12000')),log=str(dest/'route.log'),
                    env=dict(os.environ,PYTHONHASHSEED='0'))
        (dest/'run.json').write_text(json.dumps(dict(argv=argv,**run),indent=2))
        if run['cur_state']!='ok':raise RuntimeError('attempts: incomplete seed '+str(current_seed)+': '+str(run))
        route=json.loads((dest/'routes.json').read_text())


        with PL.slot():
            d=native_route_verify(board,route,child,dest/'kicad_native',export=True)
            nr=json.loads((dest/'kicad_native/native.routes.json').read_text())
            nr.update({k:route[k] for k in ('options','gnd_layers','area_nets') if k in route})
            evaluation=rate(json.loads((dest/'kicad_native/native.board.json').read_text()),nr,classes,v3=True)
            (dest/'evaluation.json').write_text(json.dumps(evaluation))
        errors=sum(v['severity']=='error' for v in d['violations']);opens=len(d['unconnected_items'])
        valid=errors==0 and opens==0 and evaluation['is_valid']
        rating=evaluation.get('score_v3')
        if rating is None or not math.isfinite(rating):raise ValueError('attempts: score v3 missing')
        entry=dict(seed=current_seed,is_valid=valid,faults=errors,open_pairs=opens,score_v3=rating,
                   model_valid=evaluation['is_valid'],model_unconnected=evaluation['not_connected'],
                   rank=[not valid,opens,rating,offset],route=str(dest/'routes.json'),job=run)
        entries.append(entry)
        (folder/'interim_state.json').write_text(json.dumps(entries,indent=2))
    winner=min(entries,key=lambda q:q['rank'])
    result=json.loads(Path(winner['route']).read_text())
    result['multi_attempts']=dict(version='seed-portfolio-v1',count=count,start_seed=seed,
         chosen=winner['seed'],attempts=entries,contract_spec='valid native+evaluation, open pairs, score v3, seed sequence',
         cpu_router_s=sum(q['job']['cpu_s'] for q in entries))
    Path(out).write_text(json.dumps(result))
    print('Multiple attempts: %d, chosen seed %d, valid %s, open %d, score %.3f'%
          (count,winner['seed'],winner['is_valid'],winner['open_pairs'],winner['score_v3']),flush=True)


if args.get('skew', '0') not in ('0', '1'):
    raise ValueError('skew must be 0 or 1')
if 'attempts' in args:
    _attempts = int(args['attempts'])
    if _attempts < 0: raise ValueError('attempts must be a nonnegative integer')
    if _attempts:
        multi_attempts(sys.argv[1], sys.argv[2], args, _attempts)
        sys.exit(0)


_AGENT_SAVED = None
if args.get('agent_loop', '0') != '0' or args.get('further'):
    from codex.agent_loop.cur_state import prepare as _agent_prepare, source_list as _agent_sources, sha as _agent_sha
    _AGENT_ARGV = dict(args)
    _AGENT_SOURCES = _agent_sources(__file__, sys.argv, args)
    _AGENT_BOARD_SHA = _agent_sha(sys.argv[1])
    _AGENT_SAVED = _agent_prepare(__file__, sys.argv, args)



if args.get('remaining_conflicts') == '1': args.setdefault('short_circuit_protection', '1')
if args.get('loop') == '1' or args.get('agent_loop') == '1':
    args.setdefault('short_circuit_protection', '1')
    args.setdefault('conflict_site', 'exact')
if args.get('rest_repair') == '1' and 'start' not in args:
    raise ValueError('rest_repair=1 needs start= with saved copper')
D = json.load(open(sys.argv[1]))
_REST_START_SOURCE = None
if 'start' in args:
    _REST_START_SOURCE = json.load(open(args['start']))
    if args.get('rest_repair') == '1':
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from postprocessing.remaining_conflicts import repair_keepout
        _rest_block = repair_keepout(_REST_START_SOURCE)
        if _rest_block:
            _REST_START_SOURCE['remaining_conflicts'] = dict(accepted=0, attempts=[], cpu_s=0,
                reason=_rest_block)
            if args.get('short_circuit_protection', '0') == '1':


                import tempfile
                from postprocessing.short_circuit_protection import file_save, R44Error
                _r44_bp = sys.argv[1]
                with tempfile.TemporaryDirectory(prefix='autorouter_r45_keepout_') as _r44_td:
                    if _REST_START_SOURCE.get('rest_board'):
                        from postprocessing.remaining_conflicts import effective_board
                        _r44_bp = os.path.join(_r44_td, 'board.json')
                        Path(_r44_bp).write_text(json.dumps(effective_board(D, _REST_START_SOURCE)))
                    try: file_save(_r44_bp, args['start'], sys.argv[2], data=_REST_START_SOURCE)
                    except R44Error as _r44_exc:
                        print(_r44_exc, file=sys.stderr); sys.exit(1)
            else:
                Path(sys.argv[2]).write_text(json.dumps(_REST_START_SOURCE))
            sys.exit(0)
    if _REST_START_SOURCE.get('rest_board') and not args.get('new'):
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from postprocessing.remaining_conflicts import effective_board
        D = effective_board(D, _REST_START_SOURCE)
_REST_BOARD_SOURCE = copy.deepcopy(D) if (_REST_START_SOURCE or {}).get('rest_board') else None
if _AGENT_SAVED is not None: D = _AGENT_SAVED['board']
ZONE_PLAN = None
if args.get('zone_plan'):
    from pathlib import Path as _ZPPath
    sys.path.insert(0, str(_ZPPath(__file__).resolve().parents[1]))
    from learning.zone_planner import check_inputs as _zp_check
    ZONE_PLAN = json.load(open(args['zone_plan']))
    _zp_check(ZONE_PLAN, D,
               json.load(open(args['netclasses'])) if args.get('netclasses') else None,
               json.load(open(args['design_rules'])) if args.get('design_rules') else None, args)
OUT = sys.argv[2]


_EARLY_ON = args.get('early_abort', '0') in ('1', 'q1')
_LEARN_ON = args.get('learning_data', '0') == '1' or _EARLY_ON
if _EARLY_ON:
    args.setdefault('labels', os.path.splitext(OUT)[0] + '_labels.npz')
    args.setdefault('short_circuit_protection', '1')
_LEARN_SNAPSHOT = None
_LEARN_LABELS = None
_LEARN_SIGNAL = False


def learn_atomic(path, payload):
    import tempfile
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(os.path.abspath(path)), suffix='.tmp')
    try:
        with os.fdopen(fd, 'w') as f:
            f.write(payload); f.flush(); os.fsync(f.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp): os.unlink(tmp)


def learn_npz_atomic(path, data):
    import tempfile
    dest = os.path.abspath(path); os.makedirs(os.path.dirname(dest), exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(dest), suffix='.npz'); os.close(fd)
    try:
        np.savez_compressed(tmp, **data); os.replace(tmp, dest)
    finally:
        if os.path.exists(tmp): os.unlink(tmp)


def learn_sigxcpu(signum, frame):
    global _LEARN_SIGNAL
    import signal, resource
    _LEARN_SIGNAL = True


    soft, hard = resource.getrlimit(resource.RLIMIT_CPU)
    reserve = max(soft + 30, int(time.process_time()) + 30)
    reserve = reserve if hard == resource.RLIM_INFINITY else min(reserve, hard)
    resource.setrlimit(resource.RLIMIT_CPU, (reserve, reserve))
    signal.signal(signal.SIGXCPU, signal.SIG_IGN)
    if _LEARN_SNAPSHOT is None:
        data = dict(tracks=[], vias=[], failed=sorted({p['net'] for p in D['pads'] if p.get('net')}),
                    conflicts={'#initialization': 1}, options=args,
                    under_wmin=[], learning_state=dict(phase='initialization', pairs=[], censored=True))


        plan = globals().get('ZONE_PLAN')
        if plan:
            if args.get('short_circuit_protection', '0') == '1':
                data.update(zone_plan_id=plan['id'], zone_list=[], zone_plan_export='unchecked_withheld')
            else:
                data.update(zone_plan_id=plan['id'], zone_list=plan['core_set'], native_vias=True,
                    vias=[v for f in plan['via_candidates'] if f['taken'] for v in f['vias']])
    else:
        data = json.loads(_LEARN_SNAPSHOT)
    if args.get('short_circuit_protection', '0') == '1' and data.get('failed'):


        names = data.get('short_circuit_units', {})
        if 'export_ratsnest' in globals():
            data['ratsnest'] = export_ratsnest(D, {names.get(n, n) for n in data['failed']})
    data['final'] = 'SIGXCPU'
    data['learning_state']['censored'] = True
    learn_atomic(OUT, json.dumps(data))
    if _LEARN_LABELS is not None:
        learn_npz_atomic(args['labels'], _LEARN_LABELS)
    print('Compute time budget: SIGXCPU, load safe state saved', flush=True)
    sys.exit(0)


if _LEARN_ON:
    import signal
    signal.signal(signal.SIGXCPU, learn_sigxcpu)
KEEPOUT_N = 0
_LOOP_SOURCE_PAD_COUNT = len(D['pads'])
_VIA_KEEP = []


def _hole_free(g):
    from shapely.geometry import LineString as _Ls, Polygon as _Pg2
    from shapely.ops import split as _split
    out, st = [], [g]
    while st:
        q = st.pop()
        if not q.interiors: out.append(q); continue
        c = _Pg2(q.interiors[0]).representative_point(); b = q.bounds
        st.extend(sorted((h for h in _split(q, _Ls([(c.x, b[1] - 1), (c.x, b[3] + 1)])).geoms if h.geom_type == 'Polygon' and h.area > 1e-9),
                         key=lambda h: h.bounds))
    return out


DESIGN_RULES_STRUCTURE = json.load(open(args['design_rules'])).get('structure') if 'design_rules' in args else None
if 'design_rules' in args:
    for _k, _v in json.load(open(args['design_rules']))['router'].items(): args.setdefault(_k, str(_v))

if args.get('keepout_list', '1') == '1' and D.get('keepouts'):






    from shapely.geometry import Polygon as _Pg, Point as _Pt
    from shapely.ops import unary_union as _uu
    _brd = border_polygon(D)
    _nl = int(D.get('cu_layers') or args.get('cu_layers', '2'))
    _all = ['F'] + ['In%d' % i for i in range(1, _nl - 1)] + ['B']
    _clr0 = float(args.get('clr', '0.13'))
    for _ki, _k in enumerate(list(D['keepouts'])):
        _lo = _k.get('hole_list') or []
        _g = _uu([_Pg(q, [h for h in (_lo[i] if i < len(_lo) else []) if len(h) >= 3]).buffer(0)
                  for i, q in enumerate(_k.get('polys', [])) if len(q) >= 3]).intersection(_brd)
        if not (_k.get('prohibitions') or {}).get('track_list', False):
            if (_k.get('prohibitions') or {}).get('vias', False) and args.get('keepouts_vias', '1' if 'netclasses' in args else '0') == '1' and not _g.is_empty:
                _VIA_KEEP.append(_g)
            continue
        _lag = [L_ for L_ in (_k.get('cu_layers') or _all) if L_ in _all]
        for _L in _lag:
            _mine = [_Pg(q).buffer(_clr0) for p_ in D['pads'] if _k.get('ref') and p_['ref'] == _k['ref']
                    and _L in p_['layers'] for q in p_['polys'].get(_L, [])]
            _gl = _g.difference(_uu(_mine)) if _mine else _g
            _ps = [g_ for g_ in getattr(_gl, 'geoms', [_gl]) if g_.geom_type == 'Polygon' and g_.area > 1e-3]
            _ps = [h_ for g_ in _ps for h_ in (_hole_free(g_) if g_.interiors else [g_]) if h_.area > 1e-6]
            if not _ps: continue
            _q = [[[round(x, 5), round(y, 5)] for x, y in g_.exterior.coords] for g_ in _ps]
            _bb = _uu(_ps).bounds; _c = _uu(_ps).representative_point()
            D['pads'].append(dict(ref='KEEPOUT_%d_%s' % (_ki, _L), num='1', net='#KEEPOUT_%d' % _ki, layers=[_L], polys={_L: _q},
                                  x=_c.x, y=_c.y, bbox=list(_bb), drill=0.0, fp_side='F', pinfunc='', pintype='', mask={}))
            KEEPOUT_N += 1
    if KEEPOUT_N: print('Keepout areas: %d as obstacle' % KEEPOUT_N, flush=True)
    if _VIA_KEEP: print('Keepout areas for vias only: %d' % len(_VIA_KEEP), flush=True)
if args.get('style_per_net_kind') == '1': args['style'] = 'per_net_kind'
if 'style' in args:
    args['angle45'] = '1' if args['style'] in ('45', 'angle45', 'per_net_kind') else '0'
NC = json.load(open(args['netclasses'])) if 'netclasses' in args else None
if args.get('sw_compact', '0') == '1':
    if NC is None: raise ValueError('sw_compact=1 needs netclasses=')
    from analysis.switch_node import correct as _sw_correct
    _sw_correct(D, NC)
PIN_ACCESS = args.get('pin_access', '0') == 'exact'
if args.get('pin_access', '0') not in ('0', 'exact'):
    raise ValueError('pin_access must be 0 or exact')
if PIN_ACCESS and NC is None:
    raise ValueError('pin_access=exact needs netclasses= and board rules')
PZ_REST = args.get('pin_access_rest', '0') == '1'
PZ_MINI = args.get('pin_access_minimax', '0') == '1'
if (PZ_REST or PZ_MINI) and (not PIN_ACCESS or args.get('short_circuit_protection') != '1'):
    raise ValueError('pin_access_rest/minimax need pin_access=exact and short_circuit_protection=1')
PZ_WIDTHS = {}
PZ_LOCAL = {}
PZ_BORDER_NM = 1000
PZ_RADIUS_NM = 1600000


def _pz_active(net):
    return bool(globals().get('PIN_ACCESS', False) and net in PZ_LOCAL)


def _pz_near(net,x,y):
    if not _pz_active(net):return False
    x,y=_pz_nm(x),_pz_nm(y)
    for p in D['pads']:
        if id(p) not in PZ_LOCAL[net]:continue
        a,b,c,d=map(_pz_nm,p['bbox'])
        if a-PZ_RADIUS_NM<=x<=c+PZ_RADIUS_NM and b-PZ_RADIUS_NM<=y<=d+PZ_RADIUS_NM:return True
    return False


def _pz_nm(value):
    from decimal import Decimal, ROUND_HALF_EVEN
    return int((Decimal(str(value)) * 1000000).to_integral_value(rounding=ROUND_HALF_EVEN))


def _pz_mm(nm):

    value = int(nm) / 1000000
    if int(value * 1000000) != nm:
        value = math.nextafter(value, math.inf if nm > 0 else -math.inf)
    assert int(value * 1000000) == nm
    return value


def _pz_snap(point):
    return [_pz_mm(_pz_nm(v)) for v in point]



def _pz_widths(D, NC, design_rules):
    import re
    raw_data = design_rules.get('raw_data') or {}
    minimum = max(float((D.get('design') or {}).get('min_width') or 0),
                  float(raw_data.get('min_track_width') or 0))
    if not math.isfinite(minimum) or minimum < 0:
        raise ValueError('pin_access: no occupied valid board minimum width')
    if minimum == 0 and not (D.get('net_classes') or raw_data.get('netclass_track')):
        raise ValueError('pin_access: no occupied positive board minimum width/netclass')

    source = design_rules.get('circuit_board') or (D.get('src') if str(D.get('src','')).endswith('.kicad_pcb') else None)
    if not source and design_rules.get('source'):

        raise ValueError('pin_access: rule source is another board, DRU not read')
    rules = []
    if source:
        base = Path(source).with_suffix('')
        pro = base.with_suffix('.kicad_pro'); dru = base.with_suffix('.kicad_dru')
        if pro.exists():
            actual = json.loads(pro.read_text())['board']['design_settings']['rules']['min_track_width']
            minimum = max(minimum, float(actual))
        if dru.exists():
            from analysis.design_rules import dru_rules
            rules = dru_rules(str(dru))
        elif raw_data.get('dru_conditional') or design_rules.get('dru_conditional'):
            raise ValueError('pin_access: conditional DRU rules without source file')
    elif raw_data.get('dru_conditional') or design_rules.get('dru_conditional'):
        raise ValueError('pin_access: conditional DRU rules without source')
    from common.dru_condition import fits as dru_fits
    net_list = {p['net'] for p in D['pads']}
    out = {}
    for net, k in sorted(NC['net_list'].items()):
        if k.get('category') not in ('signal', 'analog', 'fast'): continue

        if k.get('z_ohm') or k.get('z_diff_ohm') or k.get('pair'):
            continue
        nc = (D.get('netclass') or {}).get(net, 'Default')
        nominal = float(((D.get('net_classes') or {}).get(nc) or {}).get('w') or raw_data.get('netclass_track') or 0)
        w = minimum if minimum > 0 else nominal
        if not math.isfinite(w) or w <= 0:raise ValueError('pin_access: no positive rule width for '+net)
        maximum = float('inf'); names = []; dru_min = minimum
        for r in rules:
            c = r['constraints'].get('track_width')
            if not c: continue

            v = None if r.get('layer') else dru_fits(r['cond'], net, nc, None, net_list)
            if v is False: continue
            if c.get('min') is None:
                raise ValueError('pin_access: width rule without evaluable minimum: '+r['name'])
            if v is None:
                dru_min = max(dru_min, float(c['min'])); w = max(w, dru_min); names.append(r['name']); continue

            dru_min = max(minimum, float(c['min']))
            w = dru_min
            if w <= 0:w=nominal
            maximum = float(c['max']) if c.get('max') is not None else float('inf'); names = [r['name']]
            if c.get('opt') is not None: w = max(w, float(c['opt']))
        if not math.isfinite(w) or not 0 < w <= maximum:
            raise ValueError('pin_access: contradictory width rules for '+net)
        out[net] = dict(w=w, min_board=minimum, netclass=nc, design_rules=names,
                        source=source or 'board.design.min_width')
        if names:out[net]['dru_min']=dru_min
    return out


if PIN_ACCESS:
    try:
        PZ_WIDTHS = _pz_widths(D, NC, json.load(open(args['design_rules'])) if 'design_rules' in args else {})
    except ValueError as _e:
        if 'pin_access' in _ARGS_EXPLICIT or not str(_e).startswith('pin_access'): raise

        print('Pin access exact (default) off: %s' % _e, flush=True)
        PIN_ACCESS = PZ_REST = PZ_MINI = False; PZ_WIDTHS = {}
        args['pin_access'] = '0'; args['pin_access_rest'] = '0'; args['pin_access_minimax'] = '0'
if PIN_ACCESS:
    print('Pin access exact: %d uniform signal widths from board rules' % len(PZ_WIDTHS), flush=True)
DRCL = NC is not None and args.get('drc_model', '0') == '1'
if DRCL: D['pads'].extend(D.get('footprint_copper') or [])
if NC and args.get('mask_graphic', '0') == '1':
    D['pads'].extend(D.get('mask_graphics') or [])
GNDS = set(NC['gnd']) if NC and NC.get('gnd') else {'/GND'}
GND_MAIN = NC['gnd_main'] if NC and NC.get('gnd_main') else '/GND'


CURRENT_FIRST = args.get('current_first', '0') == '1'
CURRENT_J = float(args.get('current_J', '40')) if CURRENT_FIRST else 40.0
CURRENT_DETOUR = float(args.get('current_detour', '1')) if CURRENT_FIRST else 1.0
CURRENT_POT = {}
if CURRENT_FIRST:
    if NC is None: raise ValueError('current_first needs netclasses= and an occupied flow tree')
    if not math.isfinite(CURRENT_J) or CURRENT_J <= 0 or not math.isfinite(CURRENT_DETOUR) or CURRENT_DETOUR < 0:
        raise ValueError('current_J must be positive, current_detour finite and nonnegative')
    _sp_names = {p['net'] for p in D['pads']}
    _sp_ground = []
    for _spec in args.get('gnd_layers', 'In1').split(','):
        _layer, _, _names = _spec.partition('=')
        _nn = {n if n in _sp_names else '/' + n if '/' + n in _sp_names else n.lstrip('/') for n in _names.split('|') if n}
        if _nn and not _nn <= GNDS:
            if _nn & GNDS: raise ValueError('current_first: mixed GND/potential layer not supported')
            CURRENT_POT[_layer] = sorted(_nn)
        elif _spec: _sp_ground.append(_spec)

    args['gnd_layers'] = ','.join(_sp_ground)

RES = 0.05
TRACK_MIN = float(args.get('track_min', '0'))
HOLE_CLR = float(args.get('hole_clr', '0'))
CLR = float(args.get('clr', '0.13'))
QM = 0.036
CLR_EFF = CLR + QM
EDGE = float(args.get('edge', '0.30')) + 0.036
VIA_D, VIA_DR = [float(v) for v in args.get('via', '0.45/0.20').split('/')]
DRCC = float(args.get('drc', '0.127'))
DRCP = float(args.get('drcpad', args.get('drc', '0.127')))

NETCLASS_SPACING = args.get('netclass_spacing', '0') == '1'
if args.get('netclass_spacing', '0') not in ('0', '1'):
    raise ValueError('netclass_spacing must be 0 or 1')
KA = None
KA_CONTRACT = None
KA_LEGACY_COVERS = False
try:
    if NETCLASS_SPACING:
        if NC is None: raise ValueError('netclass_spacing=1 needs netclasses=')
        if args.get('loop','0') not in ('0','') or args.get('agent_loop','0') not in ('0',''):
            raise ValueError('netclass_spacing: placement transactions not covered yet')
        from analysis.spacings import contract_spec as _ka_contract, Spacing as _Spacing
        _ka_rg = json.load(open(args['design_rules'])) if args.get('design_rules') else {}
        if not _ka_rg.get('spacings') and NC.get('spacings'):
            _ka_rg = dict(_ka_rg, spacings=NC['spacings'])
        _ka_c = _ka_contract(D, _ka_rg)
        if not (D.get('net_classes') or _ka_c['netclasses'].get('Default') is not None):
            raise ValueError('netclass_spacing: no occupied netclass/board contract')

        if not _ka_rg.get('spacings') and not NC.get('spacings') and ((_ka_rg.get('raw_data') or {}).get('dru_conditional') or _ka_rg.get('dru_conditional')):
            raise ValueError('netclass_spacing: DRU contract missing; regenerate rules/netclasses')
        KA = _Spacing(_ka_c)
        for _ka_net in sorted({p['net'] for p in D['pads'] if p.get('net')}):
            KA.pair(_ka_net,'')
        print('Net pair spacings: %d netclasses, %d DRU rules, max %.6g mm' %
              (len(_ka_c['netclasses']), len(_ka_c['dru']), KA.maximum), flush=True)
        KA_CONTRACT = _ka_c
        _ka_L = int(args.get('cu_layers','4'))
        KA_LEGACY_COVERS = KA.legacy_covers({p['net'] for p in D['pads'] if p.get('net')},
                             ['F']+['In%d'%i for i in range(1,_ka_L-1)]+['B'],DRCC,DRCP)
        if KA_LEGACY_COVERS:
            print('Net pair contract fully covered by the previous spacing; identical old maps',flush=True)
            KA = None


except ValueError as _e:
    if 'netclass_spacing' in _ARGS_EXPLICIT: raise

    print('Netclass spacing (default) off: %s' % _e, flush=True)
    NETCLASS_SPACING = False; KA = None; KA_CONTRACT = None; KA_LEGACY_COVERS = False; args['netclass_spacing'] = '0'
def _ka_clr(net, other, li=0, type_='Track', type2='Track', pad='', pad2=''):
    if KA is None: return DRCP if 'Pad' in (type_,type2) else DRCC


    if 'MaskKeepout' in (type_,type2) or 'HoleKeepout' in (type_,type2):return 0.
    if any(str(n).startswith(('#KEEPOUT','#MASK','#NTBAR')) for n in (net,other)):
        return DRCP if 'Pad' in (type_,type2) else DRCC

    net = '' if str(net).startswith('#') else net
    other = '' if str(other).startswith('#') else other
    return KA.pair(net, other, LAYERS[li], type_, type2, pad, pad2)[0]


def _ka_margin(net, other, li=0, type_='Track', type2='Track', pad='', pad2=''):
    if KA is None: return _pz_clearance(net,other,'Pad' in (type_,type2))
    return _ka_clr(net,other,li,type_,type2,pad,pad2) + (PZ_BORDER_NM/1e6 if _pz_active(net) or _pz_active(other) else .004)
VIP = args.get('vip') == '1'
BGA_HOLE = NC is not None and int(args.get('cu_layers', '4')) >= 6 and args.get('bga_via_exact', '0') == '1'
if NC is not None and int(args.get('cu_layers', '4')) >= 6 and args.get('bga_via_min', '0') == '1':


    _bd = D.get('design') or {}
    _dr = float(_bd.get('bore_min') or VIA_DR)
    _dia = max(float(_bd.get('via_min') or VIA_D), _dr + 2 * float(_bd.get('ring_min') or 0.1),
               0.0 if BGA_HOLE else _dr + 2 * max(0.0, HOLE_CLR - min(DRCP, DRCC)))
    VIA_D, VIA_DR = round(_dia, 4), round(_dr, 4)
    args['via'] = '%g/%g' % (VIA_D, VIA_DR)
    print('BGA small vias (option, whole board):', args['via'], flush=True)
VIA_R = VIA_D / 2


VIA_KEEP_R = max(VIA_R, VIA_DR / 2 + HOLE_CLR - min(DRCP, DRCC)) if BGA_HOLE else VIA_R
VIA_OCC_R = VIA_R if BGA_HOLE and args.get('bga_occupancy', '') == 'real' else VIA_KEEP_R
H2H = float(args.get('h2h', '0.25'))
HOLE_C = H2H + VIA_DR + 0.02
NLAY = int(args.get('cu_layers', '4'))
BGA_ESCAPE = NC is not None and NLAY >= 4 and args.get('bga_escape', '0') == '1'
BGA_FANOUT = NC is not None and ((NLAY >= 6 and args.get('bga_fanout', '0') == '1') or BGA_ESCAPE)
BGA_PLAN = BGA_FANOUT and (args.get('bga_plan', '0') == '1' or BGA_ESCAPE)
if BGA_ESCAPE: args.setdefault('bga_budget', '1')
BGA_TIGHT = BGA_PLAN and args.get('bga_lanes', '0') == '1'
PAD_EXTRA = 0.000001 if BGA_TIGHT else 0.006

POTENTIAL_PLAN = None
LAYER_STACK = False
if NC is not None:
    from pathlib import Path as _FlPath
    sys.path.insert(0, str(_FlPath(__file__).resolve().parents[1]))
    from analysis.areas import active as _fl_active, plan as _fl_plan
    LAYER_STACK = _fl_active(dict(options=args), NC)
    if LAYER_STACK and not (ZONE_PLAN and ZONE_PLAN.get('audit',{}).get('stage') == 3):
        _fl_map, _fl_fallback = _fl_plan(dict(D, cu_layers=NLAY), dict(options=args), NC)
        if _fl_fallback:
            POTENTIAL_PLAN = dict(requested=args.get('gnd_layers', 'In1'), track_tree=_fl_fallback)
            args['gnd_layers'] = ','.join(L + '=' + '|'.join(nn) for L, nn in _fl_map.items())
            print('Potential plan: track tree instead of separate area:', _fl_fallback, flush=True)
GND_LAYERS = [q.split('=')[0] for q in args.get('gnd_layers', 'In1').split(',') if q]


PLANE_SIGNALS = set()
if PIN_ACCESS and D.get('plane_domains'):
    PLANE_SIGNALS = set(json.load(open(D['plane_domains'])).get('signal_layers', []))
    if PLANE_SIGNALS - set(GND_LAYERS): raise ValueError('signal_layers needs assigned area layers')
STYLE_PER_NET_KIND = args.get('style') == 'per_net_kind'
from common.net_style import organic as _net_organic
OCT = args.get('angle45') == '1'
LAYERS = ['F'] + ['In%d' % k for k in range(1, NLAY - 1) if 'In%d' % k not in GND_LAYERS or 'In%d' % k in PLANE_SIGNALS] + ['B']




_all_nets = {p['net'] for p in D['pads'] if p['net']}
def _net_name(n):
    return n if n in _all_nets else ('/' + n if '/' + n in _all_nets else (n.lstrip('/') if n.lstrip('/') in _all_nets else n))
_FL_EXPLICIT = '=' in args.get('gnd_layers', 'In1')
if LAYER_STACK and _FL_EXPLICIT:
    from pathlib import Path as _FlPath
    sys.path.insert(0, str(_FlPath(__file__).resolve().parents[1]))
    from analysis.areas import assignment as _fl_assignment
    FL_ASSIGNMENT = _fl_assignment(dict(D, cu_layers=NLAY), dict(options=args), dict(gnd=sorted(GNDS)))
else:

    FL_ASSIGNMENT = {q.split('=')[0]: sorted({_net_name(n) for n in q.split('=', 1)[1].split('|') if n}) if '=' in q else ([] if _FL_EXPLICIT else sorted(GNDS))
                    for q in args.get('gnd_layers', 'In1').split(',') if q and q.split('=')[0] in ['F'] + ['In%d' % i for i in range(1, NLAY - 1)] + ['B']}
AREA_NETS = {n for nn in FL_ASSIGNMENT.values() for n in nn}
PLANE_MIXED_NETS = {n for L in PLANE_SIGNALS for n in FL_ASSIGNMENT.get(L,[])}
if not LAYER_STACK:
    _fl = [n for q in args.get('gnd_layers', 'In1').split(',') if '=' in q for n in q.split('=', 1)[1].split('|') if n]
    AREA_NETS = {_net_name(n) for n in _fl} if _fl else (set(GNDS) if GND_LAYERS else set())
LI = {L: i for i, L in enumerate(LAYERS)}
NL = len(LAYERS)
CURRENT_T = {}
if CURRENT_FIRST:
    _st_all = ['F'] + ['In%d' % i for i in range(1, NLAY - 1)] + ['B']
    if set(CURRENT_POT) - set(LAYERS): raise ValueError('current_first: invalid potential layer')
    _st_cu = (DESIGN_RULES_STRUCTURE or D.get('structure') or {}).get('copper_mm')
    if _st_cu is not None and (len(_st_cu) != NLAY or not all(math.isfinite(float(t)) and float(t) > 0 for t in _st_cu)):
        raise ValueError('current_first: invalid copper thicknesses')
    CURRENT_T = dict(zip(_st_all, map(float, _st_cu))) if _st_cu is not None else {L: .035 if L in ('F', 'B') else .0175 for L in _st_all}


xs = [p[0] for o in D['outline'] for p in o]; ys = [p[1] for o in D['outline'] for p in o]
X0, Y0 = min(xs) - 0.3, min(ys) - 0.3
NX = int(math.ceil((max(xs) + 0.3 - X0) / RES)); NY = int(math.ceil((max(ys) + 0.3 - Y0) / RES))
NN = NX * NY
print('Raster', NX, 'x', NY, '=', NN, 'Cells per layer')



VIA_DENSITY = args.get('via_density', '0') == '1'
VD_AFTER = args.get('via_density_after', '0') == '1' and NC is not None
VD_COUNTER = VIA_DENSITY or VD_AFTER
VD_WINDOW = float(args.get('via_density_window', '2.0'))
VD_THRESHOLD = float(args.get('via_density_threshold', '1.25'))
VD_MM = float(args.get('via_density_mm', '0.5'))
VD_CURRENT = float(args.get('via_density_current', '0.5'))
if VD_COUNTER and (not all(math.isfinite(v) for v in (VD_WINDOW, VD_THRESHOLD, VD_MM, VD_CURRENT)) or
                   min(VD_WINDOW, VD_THRESHOLD) <= 0 or min(VD_MM, VD_CURRENT) < 0):
    raise ValueError('Invalid via_density options')
_VD_COUNT = np.zeros((NY, NX), dtype=np.int32) if VD_COUNTER else None
_VD_VIAS = {}
_VD_VERSION = [0]
_VD_CACHE = [None]


def to_px(x, y): return ((x - X0) / RES - 0.5, (y - Y0) / RES - 0.5)
def cell_xy(ix, iy): return (X0 + (ix + 0.5) * RES, Y0 + (iy + 0.5) * RES)


def raster_polys(polys, buf=0.0):
    img = Image.new('1', (NX, NY), 0); dr = ImageDraw.Draw(img)
    for pts in polys:
        P = Polygon(pts)
        if buf: P = P.buffer(buf, join_style=2)
        geoms = getattr(P, 'geoms', [P])
        for g in geoms:
            if g.is_empty: continue
            dr.polygon([to_px(x, y) for x, y in g.exterior.coords], fill=1, outline=1)
    return np.array(img, dtype=bool)


inside = raster_polys(D['outline'])
if D.get('outline_holes'):
    for _hs in D['outline_holes']:
        for _h in _hs: inside &= ~raster_polys([_h])
edge_d = _edt(inside) * RES
if args.get('edge_exact') == '1' or any(D.get('outline_holes') or []):


    import shapely as _edge_shapely
    _edge_board = border_polygon(D)
    _edge_x = X0 + (np.arange(NX) + .5) * RES
    for _y0 in range(0, NY, 64):
        _yy = Y0 + (np.arange(_y0, min(NY, _y0 + 64)) + .5) * RES
        _gx, _gy = np.meshgrid(_edge_x, _yy)
        _points = _edge_shapely.points(_gx.ravel(), _gy.ravel())
        _within = _edge_shapely.covers(_edge_board, _points)
        _dist = _edge_shapely.distance(_edge_board.boundary, _points)
        edge_d[_y0:_y0 + len(_yy)] = np.where(_within, _dist, 0.).reshape(_gx.shape)
        inside[_y0:_y0 + len(_yy)] = _within.reshape(_gx.shape)
nets = [''] + sorted({p['net'] for p in D['pads']})
NID = {n: i for i, n in enumerate(nets)}
occ = np.zeros((NL, NY, NX), dtype=np.int16)
padany = np.zeros((NL, NY, NX), dtype=bool)
drills = np.zeros((NY, NX), dtype=bool)
pad_cells = []
for k, p in enumerate(D['pads']):
    p.setdefault('cells', {}); p.setdefault('bcells', {})
    for L in p['layers']:
        if L not in LI: continue
        li = LI[L]
        m_buf = raster_polys(p['polys'][L], QM)
        m_core = raster_polys(p['polys'][L], -0.05)
        if not m_core.any():
            cx, cy = to_px(p['x'], p['y'])
            if 0 <= int(round(cy)) < NY and 0 <= int(round(cx)) < NX: m_core[int(round(cy)), int(round(cx))] = True
        nid = NID[p['net']] if p['net'] and not p['net'].startswith('unconnected') else -1
        occ[li][m_buf] = np.where(occ[li][m_buf] == 0, nid if nid > 0 else 32000, occ[li][m_buf])
        padany[li] |= m_buf
        p.setdefault('cells', {})[L] = np.flatnonzero(m_core)
        p.setdefault('bcells', {})[L] = np.flatnonzero(m_buf)
def pad_hull(p, L, spacing, model):
    poly = shapely.union_all([Polygon(q) for q in p['polys'][L]])
    mk = (p.get('mask') or {}).get(L, 0.0)
    if model and mk > spacing:
        poly = poly.buffer(mk - spacing + 0.01, join_style=2)
    if model and p.get('drill', 0) > 0:
        hole = shapely.union_all([Polygon(q) for q in p['hole_polys']]) if p.get('hole_polys') else Point(p['x'], p['y']).buffer(p['drill'] / 2, 32)
        hole = hole.buffer(max(0.0, HOLE_CLR - spacing), 32)
        poly = shapely.union_all([poly, hole])
    elif HOLE_CLR > spacing and p.get('drill', 0) > 0:
        hole = Point(p['x'], p['y']).buffer(p['drill'] / 2 + HOLE_CLR - spacing, 32)
        poly = shapely.union_all([poly, hole])
    if not model and mk > spacing:
        poly = poly.buffer(mk - spacing + 0.01, join_style=2)
    return poly



import shapely


PAD_HOLE_GEOMS = []
PAD_HOLE_CIRCLES = []
PAD_HOLE_DIST = None






PAD_HOLE_RESERVE = 0.004
if NC is not None:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from common.drills import pad_drill_hole
    for p in D['pads']:
        try:
            hole = pad_drill_hole(p)
        except (ValueError, TypeError, IndexError) as exc:
            raise ValueError('Pad %s:%s: %s' % (p['ref'],p['num'],exc)) from exc
        if hole is None: continue
        PAD_HOLE_GEOMS.append(hole['geo'])
        PAD_HOLE_CIRCLES.append((hole['x'],hole['y'],hole['drill']/2) if hole['circle'] else None)
    if PAD_HOLE_GEOMS:
        PAD_HOLE_DIST = np.full((NY,NX),9.0,dtype=np.float32)
        margin = VIA_DR/2+H2H+.05
        for hole, circle in zip(PAD_HOLE_GEOMS,PAD_HOLE_CIRCLES):
            x0,y0,x1,y1 = hole.bounds
            ix0=max(0,int((x0-margin-X0)/RES));ix1=min(NX,int((x1+margin-X0)/RES)+1)
            iy0=max(0,int((y0-margin-Y0)/RES));iy1=min(NY,int((y1+margin-Y0)/RES)+1)
            if ix1<=ix0 or iy1<=iy0: continue
            gx,gy=np.meshgrid(X0+(np.arange(ix0,ix1)+.5)*RES,Y0+(np.arange(iy0,iy1)+.5)*RES)
            distances=(np.maximum(0.,np.hypot(gx-circle[0],gy-circle[1])-circle[2])
                       if circle is not None else shapely.distance(hole,shapely.points(gx,gy))).astype(np.float32)
            PAD_HOLE_DIST[iy0:iy1,ix0:ix1]=np.minimum(PAD_HOLE_DIST[iy0:iy1,ix0:ix1],distances)
PD1 = np.full((NL, NY, NX), 9.0, dtype=np.float32); PN1 = np.zeros((NL, NY, NX), dtype=np.int32); PD2 = np.full((NL, NY, NX), 9.0, dtype=np.float32)
GXc = X0 + (np.arange(NX) + 0.5) * RES; GYc = Y0 + (np.arange(NY) + 0.5) * RES
RW = 1.6
for k, p in enumerate(D['pads']):
    pn = NID[p['net']] if p['net'] and not p['net'].startswith('unconnected') else 30000 + k
    for L in p['layers']:
        if L not in LI: continue
        li = LI[L]
        poly = pad_hull(p, L, CLR, DRCL)
        mk = (p.get('mask') or {}).get(L, 0.0)
        bx = p['bbox']
        if mk > CLR or HOLE_CLR > CLR:
            g = poly.bounds; bx = [min(bx[0], g[0]), min(bx[1], g[1]), max(bx[2], g[2]), max(bx[3], g[3])]
        ixa = max(0, int((bx[0] - RW - X0) / RES)); ixb = min(NX, int((bx[2] + RW - X0) / RES) + 1)
        iya = max(0, int((bx[1] - RW - Y0) / RES)); iyb = min(NY, int((bx[3] + RW - Y0) / RES) + 1)
        GX, GY = np.meshgrid(GXc[ixa:ixb], GYc[iya:iyb])
        d = shapely.distance(poly, shapely.points(GX.ravel(), GY.ravel())).reshape(GX.shape).astype(np.float32)
        d1 = PD1[li, iya:iyb, ixa:ixb]; n1 = PN1[li, iya:iyb, ixa:ixb]; d2 = PD2[li, iya:iyb, ixa:ixb]
        same = n1 == pn
        d1[same] = np.minimum(d1[same], d[same])
        newbest = (~same) & (d < d1)
        d2[newbest] = d1[newbest]; d1[newbest] = d[newbest]; n1[newbest] = pn
        other = (~same) & (~newbest) & (d < d2)
        d2[other] = d[other]

VSP = np.full((NY, NX), 9.0, dtype=np.float32)
for vs in D.get('via_keepout', []):
    poly = shapely.union_all([Polygon(pts) for pts in vs['polys']]); bx = poly.bounds
    ixa = max(0, int((bx[0] - RW - X0) / RES)); ixb = min(NX, int((bx[2] + RW - X0) / RES) + 1)
    iya = max(0, int((bx[1] - RW - Y0) / RES)); iyb = min(NY, int((bx[3] + RW - Y0) / RES) + 1)
    GX, GY = np.meshgrid(GXc[ixa:ixb], GYc[iya:iyb])
    d = shapely.distance(poly, shapely.points(GX.ravel(), GY.ravel())).reshape(GX.shape).astype(np.float32)
    VSP[iya:iyb, ixa:ixb] = np.minimum(VSP[iya:iyb, ixa:ixb], d)
    for p in D['pads']:
        if p['ref'] == vs['ref']:
            for L, cells in p['cells'].items(): VSP.flat[cells] = 9.0

VIA_BAN = None
if _VIA_KEEP:
    VIA_BAN = np.zeros((NY, NX), dtype=bool)
    for _g in _VIA_KEEP:
        bx = _g.bounds
        ixa = max(0, int((bx[0] - RW - X0) / RES)); ixb = min(NX, int((bx[2] + RW - X0) / RES) + 1)
        iya = max(0, int((bx[1] - RW - Y0) / RES)); iyb = min(NY, int((bx[3] + RW - Y0) / RES) + 1)
        GX, GY = np.meshgrid(GXc[ixa:ixb], GYc[iya:iyb])
        d = shapely.distance(_g, shapely.points(GX.ravel(), GY.ravel())).reshape(GX.shape)
        VIA_BAN[iya:iyb, ixa:ixb] |= d < VIA_R + 0.01
occ[:] = 0

under_esp = np.zeros((NY, NX), dtype=bool)
for c in D['courtyards']:
    if ((NC and c['ref'] in NC.get('expensive_under', []))) and c['side'] == 'F':
        under_esp |= raster_polys(c['polys'])
padd = [PD1[0]] + [np.full((NY, NX), 9.0, dtype=np.float32) for _ in range(NL - 2)] + [PD1[NL - 1]]


WIDE = {0: (0.6, 0.3), 1: (0.35, 0.2), 2: (1.0, 0.3), 3: (0.8, 0.3), 4: (0.5, 0.2), 5: (0.4, 0.15), 6: (0.35, 0.15)}
SENS = ('FB_', 'VSENSE', 'SENSE', 'NTC', 'VREF')




DRU_MIN = {}
if NC is not None and args.get('dru_width', '1') == '1':
    _dq = None
    try: _rg = json.load(open(args['design_rules'])) if 'design_rules' in args else {}
    except (OSError, ValueError): _rg = {}
    if _rg.get('circuit_board'): _dq = Path(_rg['circuit_board']).with_suffix('.kicad_dru')
    elif str(D.get('src', '')).endswith('.kicad_pcb'): _dq = Path(D['src']).with_suffix('.kicad_dru')
    else: _dq = Path(sys.argv[1]).with_suffix('.kicad_dru')
    if _dq is not None and _dq.exists():
        from analysis.design_rules import dru_rules as _dru_rules
        from common.dru_condition import min_width as _dru_mb
        _dr = [r for r in _dru_rules(str(_dq)) if (r['constraints'].get('track_width') or {}).get('min') is not None]
        _dn = {p['net'] for p in D['pads']}
        _dl = ['F.Cu'] + ['In%d.Cu' % k for k in range(1, NLAY - 1)] + ['B.Cu']
        for _n in sorted(NC['net_list']):
            _m, _nm = _dru_mb(_dr, _n, (D.get('netclass') or {}).get(_n, 'Default'), _dl, _dn, TRACK_MIN)
            if _nm and max(_m.values()) > TRACK_MIN: DRU_MIN[_n] = max(_m.values())
        if DRU_MIN: print('DRU minimum widths (E95): %d nets, %s' % (len(DRU_MIN), sorted(set(DRU_MIN.values()))), flush=True)

def cls(net):
    C = _cls(net)
    if NC:
        _kind = (NC['net_list'].get(net) or {}).get('category', 'signal')
        _via = float(args.get('cost_via_' + _kind, '1'))
        _inner = float(args.get('cost_inner_' + _kind, '1'))
        if _via != 1: C = dict(C, via=C['via'] * _via)
        if _inner != 1:
            C = dict(C, inner=C['inner'] * _inner)
            if C.get('lf'):
                C['lf'] = tuple(v * (_inner if 0 < li < NL-1 else 1.) for li, v in enumerate(C['lf']))
    if 'hist_weight_sig' in args:
        C = dict(C, hist_weight=float(args['hist_weight_sig']) if C['prio'] >= 8 else float(args.get('hist_weight_pw', '0.5')))
    if args.get('wide') == '1' and NC is None and C['prio'] in WIDE:
        C = dict(C, w=WIDE[C['prio']][0], wmin=WIDE[C['prio']][1])
    if C['prio'] == 2 and 'vin_wmin' in args:
        C = dict(C, wmin=float(args['vin_wmin']), w=max(C['w'], float(args.get('vin_w', '2.0'))), pen=0.3)
    if args.get('narrow') == '1':
        C = dict(C, wmin=min(C['wmin'], {0: 0.25, 1: 0.15, 2: 0.2, 3: 0.2, 4: 0.15, 5: 0.12, 6: 0.12}.get(C['prio'], C['wmin'])))
    if NC and args.get('current_nominal', '0') == '1':
        from common.current_track import nominal_width
        k = NC['net_list'].get(net, {})
        if k.get('category') in ('current_val', 'supply', 'sw'):
            C = dict(C, w=nominal_width(k))
    if _pz_active(net) and net in PZ_WIDTHS:
        C = dict(C, w=PZ_WIDTHS[net]['w'], wmin=PZ_WIDTHS[net]['w'])
    if net in DRU_MIN:
        m = DRU_MIN[net]
        C = dict(C, w=max(C['w'], m), wmin=max(C['wmin'], m), dru_min=m)
    return C












FAST_FROM = int(args.get('fast_from', '3'))
ORDER = int(args.get('order', '1')) if NC else 1
FAST_LAYER = float(args.get('fast_layer', '0.5')) if NC else 0.0
FAST_INNER = float(args.get('fast_inner', '0')) if NC else 0.0
FAST_VIA = float(args.get('fast_via', '1')) if NC else 1.0
WEIGHTED = int(args.get('weighted', '0')) if NC else 0
DISTURB_FLANK = args.get('disturb_flank', '0') == '1' and NC is not None
DISTURB_LIST = None
if DISTURB_FLANK:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from analysis.aggressor import read_in as _aggressor_read
    DISTURB_LIST = _aggressor_read(NC)


def actual_fast(net):
    k = NC['net_list'].get(net) if NC else None
    if DISTURB_FLANK and k:
        tr = (DISTURB_LIST['net_list'].get(net) or {}).get('tr_ns')
        if k.get('category') not in ('signal', 'fast'): return False
        if tr is not None and 0 < tr <= 5.0: return True
        lk = k.get('l_crit_mm')
        if lk is None or float(lk) <= 0:
            if tr is None or tr <= 0: return False
            lk = 1000 * tr / (6 * 6.0)
        ps = [(p['x'], p['y']) for p in D['pads'] if p['net'] == net]
        span = max((math.dist(a, b) for a in ps for b in ps), default=0.0)
        return span >= float(lk)
    return bool(k) and k.get('category') == 'fast' and k.get('rank', 0) >= FAST_FROM


def _layer_baseline():
    stack = ['F'] + ['In%d' % k for k in range(1, NLAY - 1)] + ['B']
    category = {L: (2 if (not _FL_EXPLICIT or set(nn) & GNDS) else 1)
           for L, nn in FL_ASSIGNMENT.items()}
    out = []
    for L in LAYERS:
        i = stack.index(L)
        out.append(max([category.get(stack[j], 0) for j in (i - 1, i + 1) if 0 <= j < len(stack)] or [0]))
    return out


LAYER_BASELINE = _layer_baseline()
_LF = {}







_DIEL_JLC = {2: [1.5], 4: [0.0994, 1.265, 0.0994], 6: [0.0994, 0.55, 0.109, 0.55, 0.0994]}


def _dielectrics(n):
    cons = ((NC or {}).get('specs') or {}).get('structure') or {}
    if cons.get('dielectrics') and len(cons['dielectrics']) == n - 1: return [float(d) for d in cons['dielectrics']]
    if n in _DIEL_JLC: return list(_DIEL_JLC[n])
    k = n - 1; pp = [i for i in range(k) if i % 2 == 0]; core_set = [i for i in range(k) if i % 2 == 1]
    rest = 1.6 - 2 * 0.035 - (n - 2) * 0.0175 - 0.0994 * len(pp)
    return [0.0994 if i % 2 == 0 else max(0.1, rest / max(1, len(core_set))) for i in range(k)]


def _structure():
    stack = ['F'] + ['In%d' % k for k in range(1, NLAY - 1)] + ['B']
    z = [0.0]
    for d in _dielectrics(NLAY): z.append(z[-1] + d)
    plane = {}
    for L, nn in FL_ASSIGNMENT.items():
        ns = set(nn); gn = sorted(ns & GNDS)
        plane[L] = (gn[0] if len(gn) == 1 and len(ns) == 1 else None) if _FL_EXPLICIT else (GND_MAIN if GND_MAIN in AREA_NETS or not AREA_NETS else sorted(AREA_NETS)[0])
    out = []
    for L in LAYERS:
        i = stack.index(L)
        ob = [j for j in range(i) if stack[j] in plane]; un = [j for j in range(i + 1, len(stack)) if stack[j] in plane]
        h1 = z[i] - z[ob[-1]] if ob else None; h2 = z[un[0]] - z[i] if un else None
        if h1 is None and h2 is None: ref = h = H = None
        else:
            ref = stack[ob[-1]] if (h2 is None or (h1 is not None and h1 <= h2)) else stack[un[0]]
            h = min(x for x in (h1, h2) if x is not None)
            H = h1 * h2 / (h1 + h2) if (h1 is not None and h2 is not None) else h
        out.append(dict(L=L, i=i, z=z[i], ref=ref, ref_net=plane.get(ref) if ref else None, h=h, H=H))
    return out, stack


STRUCTURE, STACK = _structure()


def lf_fast(inner):
    if inner not in _LF:
        prior = [1.0] + [inner] * (NL - 2) + [1.0]
        lf = list(prior)
        Hs = [a['H'] for a in STRUCTURE if a['H']]
        if FAST_LAYER > 0 and Hs:
            Hb = min(Hs)
            for li, a in enumerate(STRUCTURE):
                q = 1.0 if not a['H'] else 1.0 - (Hb / a['H']) ** 2
                if a['H'] and a['ref_net'] not in GNDS: q = max(q, 0.5)
                lf[li] *= 1.0 + FAST_LAYER * q
        best = max(LAYER_BASELINE) if LAYER_BASELINE else 0
        if FAST_INNER > 0 and best > 0 and any(LAYER_BASELINE[li] == best for li in range(1, NL - 1)):
            for li in (0, NL - 1): lf[li] *= 1.0 + FAST_INNER
        _LF[inner] = None if lf == prior else tuple(lf)
    return _LF[inner]







NEIGHBOR_LAYER = args.get('neighbor_layer', '1') == '1' and NC is not None


def neighbor_spacing(li, dist):
    if not NEIGHBOR_LAYER: return []
    a = STRUCTURE[li]; out = []
    if not a['H']: return out
    for lj, b in enumerate(STRUCTURE):
        if lj == li or abs(b['i'] - a['i']) != 1 or not b['H']: continue
        dz = abs(b['z'] - a['z']); s2 = dist * dist * b['H'] / a['H'] - dz * dz
        if s2 > 0: out.append((lj, min(math.sqrt(s2), 2.0 * dist)))
    return out








IMPEDANCE = args.get('impedance', '0') == '1' and NC is not None
SI_FIELDS = NC is not None and (any(k.get('z_ohm') or k.get('z_diff_ohm') or k.get('length_group') for k in NC['net_list'].values())
                              or bool((NC.get('specs') or {}).get('length_groups')))
PAIR = args.get('pair', '0') == '1' and NC is not None
PAIR_SEARCH_SPACE = args.get('pair_search_space', '0') == '1'
if PAIR_SEARCH_SPACE and not PAIR: raise ValueError('pair_search_space=1 needs pair=1 and netclasses=')
PAIR_PRESET = args.get('pair_preset', 'so_far')
if PAIR_PRESET not in ('so_far', 'locker'): raise ValueError('pair_preset=so_far|locker')
PAIR_LAYER = float(args.get('pair_layer', '0.0' if PAIR_PRESET == 'locker' else '3.0')); PAIR_VIA = float(args.get('pair_via', '1.0' if PAIR_PRESET == 'locker' else '3.0')); PAIR_GAP = float(args.get('pair_gap', '0.2'))
PAIR_MOVE = float(args.get('pair_move', '1.0')); PAIR_DISTURB = float(args.get('pair_disturb', '1.0' if PAIR_PRESET == 'locker' else '3.0')); PAIR_FREE = float(args.get('pair_free', '0.3'))
PAIR_CHANNEL = float(args.get('pair_channel', '1.0'))
PAIR_NETS = {}
if PAIR:
    _ne = {p['net'] for p in D['pads'] if p['net']}
    for _n, _k in sorted(NC['net_list'].items()):
        if _k.get('pair') and _k['pair'] in NC['net_list'] and _n in _ne and _k['pair'] in _ne: PAIR_NETS[_n] = _k['pair']


def pair_fast(net):
    return net in PAIR_NETS and NC['net_list'][net].get('pair_kind', NC['net_list'][net].get('category')) == 'fast'


def pair_lf(inner):
    good = [li for li, a in enumerate(STRUCTURE) if li in (0, NL - 1) and a['ref_net'] in GNDS]
    if not good: return None
    return tuple((1.0 if li in good else 1.0 + PAIR_LAYER) * (1.0 if li in (0, NL - 1) else inner) for li in range(NL))
def pair_free_mm(net):
    hs = [a['h'] for li, a in enumerate(STRUCTURE) if li in (0, NL - 1) and a['ref_net'] in GNDS and a['h']]
    return max(3 * min(hs), CLR + 0.05) if hs else PAIR_FREE


IMP_TAB = {}
if IMPEDANCE:
    import os as _os
    sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    from analysis.impedance_table import table as _z_table
    from analysis.structure import reference_layers as _z_refs
    _za = ((NC.get('specs') or {}).get('structure') or NC.get('structure') or DESIGN_RULES_STRUCTURE)
    _zr = _z_refs(dict(options={'gnd_layers': args.get('gnd_layers', 'In1')}), GNDS, _all_nets)
    for _zn, _zk in sorted(NC['net_list'].items()):
        _zd = bool(_zk.get('z_diff_ohm') and _zk.get('pair'))
        _goal = _zk.get('z_diff_ohm') if _zd else _zk.get('z_ohm')
        if _goal:
            IMP_TAB[_zn] = _z_table(_za, NLAY, _zr, float(_goal), _zd, max(TRACK_MIN, float(args.get('sigmin', '0.09'))),
                                     max(DRCC + (.006 if PAIR_SEARCH_SPACE and _zd else 0.), float(args.get('impedance_gap', '0.13'))))
    print('Impedance (option): %d nets, %d layer values' % (len(IMP_TAB), sum(map(len, IMP_TAB.values()))), flush=True)


def z_width(net, li, fallback):
    return IMP_TAB.get(net, {}).get(LAYERS[li], {}).get('w', fallback)


def z_gap(net, li):
    return IMP_TAB.get(net, {}).get(LAYERS[li], {}).get('gap_width', PAIR_GAP)

SI = args.get('si', '0') == '1' and NC is not None
SI_VIA = float(args.get('si_via', '1.5'))
SI_STUB = args.get('si_stub', '1') == '1' and SI


def _cls(net):
    if NC is not None:
        k = NC['net_list'].get(net)
        if k:
            C = dict(w=k['w'], wmin=k['wmin'], pen=k['pen'], via=k['via'], inner=k['inner'], prio=k['prio'])
            if globals().get('KA') is not None: C['spacing_contract'] = KA.c['version']
            if (FAST_LAYER > 0 or FAST_INNER > 0) and actual_fast(net):
                _lf = lf_fast(k['inner'])
                if _lf is not None: C['lf'] = _lf
            if SI and k.get('category') == 'fast' and k.get('rank', 0) >= 5:
                C['via'] *= 1.0 + SI_VIA
            if net in PAIR_NETS:
                _pl = pair_lf(k['inner'])
                if _pl is not None: C['lf'] = tuple(a * b for a, b in zip(C['lf'], _pl)) if C.get('lf') else _pl
                C['via'] = C['via'] * PAIR_VIA
            if net in IMP_TAB and IMP_TAB[net]:
                C['zw'] = tuple(z_width(net, li, C['w']) for li in range(NL))
                C['w'] = max(C['zw']); C['wmin'] = min(C['zw'])

                C['lf'] = tuple((C.get('lf') or [1.0] * NL)[li] *
                                (min(2.0, 1 + 0.25 * max(0.0, min(IMP_TAB[net][LAYERS[li]]['profile_data']['h'],
                                 IMP_TAB[net][LAYERS[li]]['profile_data'].get('upper', float('inf'))) / .1 - 1))
                                 if LAYERS[li] in IMP_TAB[net] else 8.0) for li in range(NL))
            return C
        sw_ = float(args.get('sigw', NC.get('sigw', 0.15)))
        return dict(w=sw_, wmin=sw_, pen=0.0, via=0.9, inner=0.9, prio=8)
    n = net.split('/')[-1]
    return dict(w=float(args.get('sigw', '0.15')), wmin=float(args.get('sigmin', '0.12')), pen=0.0, via=0.9, inner=0.9, prio=8)


def loop_pad_patch(B):
    global D, PD1, PD2, PN1, padany, padd
    old = D['pads']
    if len(old) != len(B['pads']): raise ValueError('Pad count changed')
    clean = lambda p: {k:v for k,v in p.items() if k not in ('cells','bcells')}
    changed = [k for k,p in enumerate(B['pads']) if clean(old[k]) != clean(p)]
    refs = {old[k]['ref'] for k in changed}
    if not changed or len(refs) != 1 or len(changed) > 2: raise ValueError('no local SMD two-terminal part')
    if any(p.get('drill') for k in changed for p in (old[k],B['pads'][k])): raise ValueError('moved drill')
    if any(v['ref'] in refs for v in B.get('via_keepout',[])): raise ValueError('moved net tie keepout')
    if any(v.get('ref') in refs for v in B.get('keepouts',[])): raise ValueError('moved part affects keepout geometry; loop_maps=entire')
    if LANES or any(r in (NC.get('expensive_under') or []) for r in refs): raise ValueError('coupled special map needs loop_maps=entire')

    D = dict(B, pads=[p if k in changed else old[k] for k,p in enumerate(B['pads'])])
    boxes = []
    def window(p, L):
        poly = pad_hull(p,L,CLR,DRCL); bx=p['bbox']
        if (p.get('mask') or {}).get(L,0.) > CLR or HOLE_CLR > CLR:
            g=poly.bounds;bx=[min(bx[0],g[0]),min(bx[1],g[1]),max(bx[2],g[2]),max(bx[3],g[3])]
        return (max(0,int((bx[0]-RW-X0)/RES)),min(NX,int((bx[2]+RW-X0)/RES)+1),
                max(0,int((bx[1]-RW-Y0)/RES)),min(NY,int((bx[3]+RW-Y0)/RES)+1),poly)
    for k in changed:
        for p in (old[k],D['pads'][k]):
            for L in p['layers']:
                if L in LI: boxes.append(window(p,L)[:4])
    xa=min(b[0] for b in boxes);xb=max(b[1] for b in boxes);ya=min(b[2] for b in boxes);yb=max(b[3] for b in boxes)
    PD1=PD1.copy();PD2=PD2.copy();PN1=PN1.copy();padany=padany.copy()
    PD1[:,ya:yb,xa:xb]=9.;PD2[:,ya:yb,xa:xb]=9.;PN1[:,ya:yb,xa:xb]=0;padany[:,ya:yb,xa:xb]=False
    for k in changed:
        p=D['pads'][k];p['cells']={};p['bcells']={}
        for L in p['layers']:
            if L not in LI: continue
            core=raster_polys(p['polys'][L],-.05)
            if not core.any():
                cx,cy=to_px(p['x'],p['y'])
                if 0<=int(round(cy))<NY and 0<=int(round(cx))<NX:core[int(round(cy)),int(round(cx))]=True
            p['cells'][L]=np.flatnonzero(core)
            p['bcells'][L]=np.flatnonzero(raster_polys(p['polys'][L],QM))
    for k,p in enumerate(D['pads']):
        pn=NID[p['net']] if p['net'] and not p['net'].startswith('unconnected') else 30000+k
        for L in p['layers']:
            if L not in LI: continue
            li=LI[L];cells=p['bcells'][L];cy,cx=cells//NX,cells%NX
            mask=(cx>=xa)&(cx<xb)&(cy>=ya)&(cy<yb);padany[li,cy[mask],cx[mask]]=True

            bx=p['bbox'];extra=max((p.get('mask') or {}).get(L,0.),HOLE_CLR)+.02


            if (p.get('mask') or {}).get(L,0.)<=CLR and HOLE_CLR<=CLR:
                if bx[2]+RW+extra<X0+xa*RES or bx[0]-RW-extra>X0+xb*RES or bx[3]+RW+extra<Y0+ya*RES or bx[1]-RW-extra>Y0+yb*RES: continue
            x0,x1,y0,y1,poly=window(p,L);x0=max(xa,x0);x1=min(xb,x1);y0=max(ya,y0);y1=min(yb,y1)
            if x1<=x0 or y1<=y0:continue
            gx,gy=np.meshgrid(GXc[x0:x1],GYc[y0:y1]);d=shapely.distance(poly,shapely.points(gx.ravel(),gy.ravel())).reshape(gx.shape).astype(np.float32)
            d1=PD1[li,y0:y1,x0:x1];n1=PN1[li,y0:y1,x0:x1];d2=PD2[li,y0:y1,x0:x1]
            same=n1==pn;d1[same]=np.minimum(d1[same],d[same]);newbest=(~same)&(d<d1)
            d2[newbest]=d1[newbest];d1[newbest]=d[newbest];n1[newbest]=pn
            other=(~same)&(~newbest)&(d<d2);d2[other]=d[other]
    padd=[PD1[0]]+padd[1:-1]+[PD1[NL-1]]
    return changed,(xa,xb,ya,yb)
