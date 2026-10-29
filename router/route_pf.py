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
from shapely.geometry import LineString as _LS, Point as _PT
import shapely

_ZP_GEO = {}; _ZP_CACHE = {}; _ZP_VIA_MASK = {}; _ZP_VIAS = []; _ZP_DRILL = None
_ZP_POT_NETS = set()
if ZONE_PLAN:
    from learning.current_zones import _geo as _zp_geo
    from shapely.ops import unary_union as _zp_union
    if ZONE_PLAN.get('audit', {}).get('stage') == 3:
        for _label in ZONE_PLAN['reference_protection']['gnd_layers']:
            _layer, *_names = _label.split('=', 1)
            if _names and _layer in ZONE_PLAN.get('potential_layers', []):
                _ZP_POT_NETS.update(n for n in _names[0].split('|')
                                    if ZONE_PLAN['net_list'].get(n, {}).get('chosen'))
    for _z in ZONE_PLAN['core_set']:
        _ZP_GEO.setdefault((_z['net'], _z['layer']), []).append(_zp_geo(_z))
    _ZP_VIAS = [v for f in ZONE_PLAN['via_candidates'] if f['taken'] for v in f['vias']]
    for _v in _ZP_VIAS:
        for _L in _v['layers']:
            _ZP_GEO.setdefault((_v['net'], _L), []).append(_PT(_v['x'], _v['y']).buffer(_v['d']/2))
    _ZP_GEO = {key:_zp_union(gs) for key,gs in _ZP_GEO.items()}
    if _ZP_VIAS:
        _zx, _zy = np.meshgrid(X0+np.arange(NX)*RES, Y0+np.arange(NY)*RES)
        _ZP_DRILL = np.full((NY,NX), 9.0, dtype=np.float32)
        for _v in _ZP_VIAS:
            _ZP_DRILL = np.minimum(_ZP_DRILL, np.hypot(_zx-_v['x'],_zy-_v['y'])-_v['drill']/2)


def _zp_maps(net):
    if net not in _ZP_CACHE:
        if len(_ZP_CACHE)>=4: del _ZP_CACHE[next(iter(_ZP_CACHE))]
        xx,yy = np.meshgrid(X0+np.arange(NX)*RES, Y0+np.arange(NY)*RES)
        hard=[]; soft=[]
        for L in LAYERS:
            g = _zp_union([g for (n,l),g in _ZP_GEO.items() if n!=net and l==L])
            mask = shapely.intersects_xy(g.buffer(QM),xx,yy)
            hard.append(((_edt(~mask)*RES-2*QM) if mask.any() else np.full((NY,NX),9.0)).astype(np.float32))
            growth = _zp_union([_zp_geo(z) for z in ZONE_PLAN['growth'] if z['net']!=net and z['layer']==L])
            soft.append(np.where(shapely.intersects_xy(growth,xx,yy),2.,1.).astype(np.float32))
        _ZP_CACHE[net] = (hard,np.concatenate([s.ravel() for s in soft]))
    return _ZP_CACHE[net]


def _zp_hard(net, ch, li, ys=slice(None), xs=slice(None)):
    if not ZONE_PLAN: return ch
    spec=ZONE_PLAN['net_list'].get(net,{})
    if spec.get('local_without_layer_change') and LAYERS[li]!=spec.get('local_layer'):
        return np.full(ch.shape,-1.,dtype=np.float32)
    return np.minimum(ch,2*(_zp_maps(net)[0][li][ys,xs]-CLR-.005))


def _zp_viah(net, v, ys=slice(None), xs=slice(None)):
    if not ZONE_PLAN: return v
    if ZONE_PLAN['net_list'].get(net,{}).get('local_without_layer_change'):
        return np.zeros(v.shape,dtype=bool)
    if ZONE_PLAN.get('corridors') or net in _ZP_POT_NETS:
        if net not in _ZP_VIA_MASK:
            xx,yy=np.meshgrid(X0+np.arange(NX)*RES,Y0+np.arange(NY)*RES)
            legal=np.ones((NY,NX),dtype=bool)
            for corridor in ZONE_PLAN['corridors']:
                if corridor['net']==net:continue
                area=_ZP_GEO[(corridor['net'],corridor['layer'])]
                forbidden=shapely.intersects_xy(area.buffer(VIA_D/2+CLR+QM),xx,yy)
                allowed=np.zeros((NY,NX),dtype=bool)
                for slot in ZONE_PLAN['via_window']:
                    if slot['net']==corridor['net'] and slot['layer']==corridor['layer'] and VIA_D<=slot['max_via_d']+1e-9:
                        allowed |= np.hypot(xx-slot['x'],yy-slot['y'])<=slot['radius_mm']
                legal &= ~forbidden | allowed
            if net in _ZP_POT_NETS:
                area=_zp_union([_zp_geo(z) for z in ZONE_PLAN['core_set']
                                if z['net']==net and z.get('corridor')
                                and z['layer'] in ZONE_PLAN['potential_layers']])
                legal &= shapely.intersects_xy(area.buffer(-VIA_D/2-QM-.002),xx,yy)
            if len(_ZP_VIA_MASK)>=4:del _ZP_VIA_MASK[next(iter(_ZP_VIA_MASK))]
            _ZP_VIA_MASK[net]=legal
        v = v & _ZP_VIA_MASK[net][ys,xs]
    if _ZP_DRILL is not None:
        v = v & (_ZP_DRILL[ys,xs] >= VIA_DR/2+H2H+.004)
    return v

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
NB = None
if args.get('numba', '1') == '1':
    try:
        import os as _os
        sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
        from router.astar_nb import Search as _Search
        from geometry_nb import line_cap_min_kernel as _linecap_nb
        NB = _Search(NL, NN, RES)
    except ImportError:
        NB = None
CLR_PAD = DRCP + 0.012
use = np.zeros(NL * NN, dtype=np.int16)
drill_use = np.zeros(NN, dtype=np.int16)
hist = np.zeros(NL * NN, dtype=np.float32)
if args.get('hist_start'):
    from learning.self_imitation import load_data as _hist_load
    hist[:] = _hist_load(args['hist_start'], args.get('hist_context', ''), LAYERS, [X0, Y0], NX, NY, RES)
    print('Self-imitation: conflict map loaded, max hist', float(hist.max()))
U = {}
CURRENT_AUDIT = {}
_SZ_VERSION = [0]; _SZ_RESERVE = [None]; _SZ_FLOOR = {}


def current_model(net):
    if not CURRENT_FIRST: return None
    k = NC['net_list'].get(net) or {}; f = k.get('flow') or {}
    if k.get('category') not in ('current_val', 'supply', 'sw'): return None
    if f.get('unit') != 'A' or not f.get('source') or not f.get('provenance'): return None
    ls = f.get('load') or {}
    if not ls or not all(math.isfinite(float(v)) and float(v) >= 0 for v in ls.values()): return None
    if sum(map(float, ls.values())) <= 0: return None
    return f


def current_plan(net, pads, trunk, KT):
    fl = current_model(net)
    if not fl: return None
    pk = {i: ('PAD', p['ref'], p['num']) for i, p in enumerate(pads)}
    names = {'%s:%s' % (p['ref'], p['num']): pk[i] for i, p in enumerate(pads)}
    if set(fl['source']) - set(names) or set(fl['load']) - set(names):
        raise ValueError('current_first: flow pad missing at ' + net)
    source = set(fl['source'])
    if any(float(fl['load'].get(q, 0)) > 0 for q in source):
        raise ValueError('current_first: source is also load at ' + net)


    planned = set(trunk) | {i for c in KT or [] for i in c['pins'] + c['caps']}
    required = set(fl['load']) | source
    if required - {'%s:%s' % (pads[i]['ref'], pads[i]['num']) for i in planned}:
        raise ValueError('current_first: load/source outside the current tree ' + net)
    starts = [i for i, p in enumerate(pads) if '%s:%s' % (p['ref'], p['num']) in source]
    first = starts[0]
    if first not in trunk:
        first = next(c['caps'][-1] for c in KT or [] if first in c['pins'] + c['caps'])
    order = []; parent = {}; tree = [first]; rest = [i for i in trunk if i != first]
    while rest:
        _, j, a = min((math.dist((pads[i]['x'], pads[i]['y']), (pads[a]['x'], pads[a]['y'])), i, a) for i in rest for a in tree)
        parent[j] = a; order.append(j); tree.append(j); rest.remove(j)
    adj = {q: [] for q in pk.values()}; edges = []; keys = {}
    def edge(a, b, key):
        ti = len(edges); kid = ('s', ti, 0, 0., 1.)
        adj[pk[a]].append((pk[b], kid)); adj[pk[b]].append((pk[a], kid))
        edges.append(dict(layer='F', pts=[[pads[a]['x'], pads[a]['y']], [pads[b]['x'], pads[b]['y']]], w=[1.]))
        keys[key] = ti
    for j in order: edge(parent[j], j, ('trunk', j))
    for c in KT or []:
        cs, ps = c['caps'], c['pins']
        edge(cs[0], ps[0], ('link_chain', c['nr'], 0, cs[0]))
        for j in ps[1:]: edge(ps[0], j, ('link_chain', c['nr'], 0, j))
        for s, (a, b) in enumerate(zip(cs[:-1], cs[1:])): edge(b, a, ('link_chain', c['nr'], s + 1, b))
    from common.flow import analysis as _sz_flow
    reached = {names[q] for q in source}; open_items = list(reached)
    while open_items:
        for q, _ in adj[open_items.pop()]:
            if q not in reached: reached.add(q); open_items.append(q)
    if any(names[q] not in reached for q in required):
        raise ValueError('current_first: unreachable load/source in current tree ' + net)
    info = _sz_flow(edges, [], pads, fl, graph=dict(adj=adj, pad_pts=names, cu_layers=['F']))
    if info is None or any(pk[i] not in reached for i in planned):
        raise ValueError('current_first: incomplete planned flow tree ' + net)
    amp = {key: info['total'] * info['share'].get((ti, 0), 0.) for key, ti in keys.items()}
    CURRENT_AUDIT[net] = dict(current_A=info['total'], planned_branches=len(edges), source=sorted(source),
                           J=CURRENT_J, copper_mm=CURRENT_T, detour=CURRENT_DETOUR)
    return first, order, parent, amp


def current_net_class(C, I, a, b):
    widths = tuple(max(TRACK_MIN, C.get('dru_min', 0.), min(C['wmin'], NECK_W), I / (CURRENT_J * CURRENT_T[L])) for L in LAYERS)
    return dict(C, w=max(widths), wmin=max(widths), current_w=widths, current_I=I,
                current_clearance=(a['x'], a['y'], b['x'], b['y']), hist_weight=float(args.get('hist_weight_pw', '.3')),
                via=C['via'] * max(2, math.ceil(4 * I)))


def current_segment_width(C, capacity):


    return max(TRACK_MIN, min(NECK_W,C['w']), min(C['w'],capacity))
def current_floor(net, C):
    key = (net, C['current_w'])
    if key not in _SZ_FLOOR:
        while len(_SZ_FLOOR) >= 4: del _SZ_FLOOR[next(iter(_SZ_FLOOR))]
        wf = neck_field_map(net, C, max(C['current_w']))
        base = np.repeat(np.asarray(C['current_w'], np.float32), NN)
        if wf is not None: base = np.minimum(base, wf)
        _SZ_FLOOR[key] = base
    return _SZ_FLOOR[key]


def current_reserve(net, capH, viaH):
    if not CURRENT_FIRST: return capH, viaH
    pot = [li for li, L in enumerate(LAYERS) if L in CURRENT_POT and net not in CURRENT_POT[L]]
    reserve = current_model(net) is None and net not in GNDS and not pair_fast(net)
    if not reserve and not pot: return capH, viaH


    hard = _Maps(); hard.planar = np.concatenate([c.ravel() for c in capH])
    hard.extend(hard.planar[li*NN:(li+1)*NN].reshape(NY,NX) for li in range(NL))
    vh = viaH.copy()
    for li in pot: hard[li][:] = -1.
    if reserve:
        if _SZ_RESERVE[0] is None or _SZ_RESERVE[0][0] != _SZ_VERSION[0]:
            mask = np.zeros(NL * NN, bool); drills = np.zeros(NN, bool)
            for r in U.values():
                if current_model(r['net']): mask[r['cells']] = True; drills[r['dcells']] = True
            ds = [((_edt(~mask[li*NN:(li+1)*NN].reshape(NY,NX))*RES - QM - CLR - .005)*2).astype(np.float32)
                  if mask[li*NN:(li+1)*NN].any() else np.full((NY,NX),1e6,np.float32) for li in range(NL)]
            vd = (_edt(~drills.reshape(NY, NX))*RES - QM) if drills.any() else np.full((NY,NX), 1e6)
            _SZ_RESERVE[0] = (_SZ_VERSION[0], ds, vd)
        _, ds, vd = _SZ_RESERVE[0]
        for c, d in zip(hard, ds): np.minimum(c, d, out=c)
        vh &= vd >= HOLE_C
        for d in ds: vh &= d / 2 >= VIA_KEEP_R
    return hard, vh


def current_field_offsets(n):
    pitch = max(.5, HOLE_C, VIA_D + max(DRCC, DRCP) + .02)
    step = math.ceil(pitch / RES); r = math.ceil(math.sqrt(n))
    return sorted(((x * step, y * step) for y in range(-r, r + 1) for x in range(-r, r + 1)),
                  key=lambda q: (q[0]**2 + q[1]**2, q[1], q[0]))[:n]


def current_web_width(C, n, L):
    return max(TRACK_MIN, VIA_D, C.get('current_I', 0.) / n / (CURRENT_J * CURRENT_T[L]))


def current_field_mask(viaH, n, C, capH):
    offsets = current_field_offsets(n); legal = viaH.copy()
    for x,y in C.get('current_vias', ()):
        ix,iy=int((x-X0)/RES),int((y-Y0)/RES); rr=math.ceil(HOLE_C/RES)+1
        x0,x1=max(0,ix-rr),min(NX,ix+rr+1);y0,y1=max(0,iy-rr),min(NY,iy+rr+1)
        gx,gy=np.meshgrid(GXc[x0:x1],GYc[y0:y1])
        legal[y0:y1,x0:x1] &= np.hypot(gx-x,gy-y) >= HOLE_C
    good = legal.copy()


    r = max(math.hypot(x, y) for x, y in offsets)
    for li,c in enumerate(capH):

        if LAYERS[li] in CURRENT_POT and NET_CUR[0] not in CURRENT_POT[LAYERS[li]]: continue
        good &= c >= 2 * r * RES + current_web_width(C, n, LAYERS[li])
    for dx, dy in offsets:
        shifted = np.zeros_like(viaH)
        y0,y1=max(0,-dy),min(NY,NY-dy); x0,x1=max(0,-dx),min(NX,NX-dx)
        shifted[y0:y1,x0:x1] = legal[y0+dy:y1+dy,x0+dx:x1+dx]
        good &= shifted
    return good


def current_field_set(res, vs, C):
    bridges, extra = [], []; n = max(2, math.ceil(4 * C['current_I']-1e-8), C.get('current_n',0))
    for v, (a, b) in zip(vs, [(a//NN,b//NN) for a,b in zip(res[:-1],res[1:]) if a//NN != b//NN]):
        v['n'] = n; v['current_A'] = C['current_I']; v['current_layers'] = [LAYERS[a], LAYERS[b]]
        for dx, dy in current_field_offsets(n)[1:]:
            x,y = round(v['x'] + dx*RES,4),round(v['y'] + dy*RES,4)
            extra.append(dict(x=x,y=y,gr=1,too=[v['x'],v['y']],current_A=C['current_I']))
            for li in (a,b):

                p=(v['x'],v['y']); q=(x,y); mid=(p[0]+math.copysign(min(abs(q[0]-p[0]),abs(q[1]-p[1])),q[0]-p[0]),
                                                p[1]+math.copysign(min(abs(q[0]-p[0]),abs(q[1]-p[1])),q[1]-p[1]))
                pts=[list(p)]+([list(mid)] if mid!=p and mid!=q else [])+[list(q)]
                w=current_web_width(C,n,LAYERS[li])
                bridges.append(dict(layer=LAYERS[li],pts=pts,w=[w]*(len(pts)-1),web=True))
    return bridges, extra


def current_fields_free(res, vs, C, capH=None):
    ts, extra = current_field_set(res, copy.deepcopy(vs), C)
    points = [(v['x'],v['y']) for v in vs+extra] + list(C.get('current_vias',()))
    if not all(math.dist(a,b) >= HOLE_C - 1e-6 for i,a in enumerate(points) for b in points[i+1:]): return False


    return capH is None or all(line_cap_min(capH[LI[t['layer']]], a, b) >= w
                              for t in ts for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']))

def current_actual(tracks, vias, pads, fl):
    from common.flow import net_graph, pad_geo
    from shapely.geometry import Point, LineString
    from shapely.ops import nearest_points
    from shapely import STRtree
    from scipy.sparse import coo_matrix
    from scipy.sparse.linalg import spsolve
    from collections import defaultdict
    tol = 1e-7
    def net_copper(L, p): return (L, round(p[0]/tol)*tol, round(p[1]/tol)*tol)
    segs = []
    for ti,t in enumerate(tracks):
        for si,(a,b,w) in enumerate(zip(t['pts'][:-1],t['pts'][1:],t['w'])):
            if math.dist(a,b)>tol: segs.append((ti,si,t['layer'],LineString([a,b]),w))
    cuts = [{0.,s[3].length} for s in segs]; left = []; via_left = []
    by_layer = defaultdict(list)
    for i,s in enumerate(segs): by_layer[s[2]].append(i)
    for L,ids in sorted(by_layer.items()):
        tree = STRtree([segs[i][3].buffer(segs[i][4]/2+tol) for i in ids])
        for ii,i in enumerate(ids):
            a=segs[i][3]; aw=segs[i][4]
            for jj in sorted(tree.query(a.buffer(aw/2+tol))):
                j=ids[int(jj)]
                if j<=i: continue
                b=segs[j][3]
                if a.distance(b)> (aw+segs[j][4])/2+tol: continue
                inter=a.intersection(b)
                if inter.is_empty: pairs=[nearest_points(a,b)]
                elif inter.geom_type=='Point': pairs=[(inter,inter)]
                else:


                    gs=list(inter.geoms) if hasattr(inter,'geoms') else [inter]
                    pairs=[(Point(p),Point(p)) for g in gs for p in (list(g.coords) if hasattr(g,'coords') else [])]
                for p,q in pairs:
                    pa,pb=a.project(p),b.project(q); cuts[i].add(pa);cuts[j].add(pb)
                    left.append((net_copper(L,a.interpolate(pa).coords[0]),net_copper(L,b.interpolate(pb).coords[0])))
            for vi,v in enumerate(vias):
                p=Point(v['x'],v['y'])
                if a.distance(p)<=aw/2+float(v.get('d',VIA_D))/2+tol:
                    d=a.project(p); cuts[i].add(d)
                    via_left.append((net_copper(L,a.interpolate(d).coords[0]),net_copper(L,(v['x'],v['y']))))
            for p in pads:
                geo=pad_geo(p).get(L)
                if geo is not None and a.distance(geo)<=aw/2+tol:
                    q,_=nearest_points(a,geo);cuts[i].add(a.project(q))
    split=[]; orig=[]
    for i,(ti,si,L,line,w) in enumerate(segs):
        ds=sorted(cuts[i])
        for a,b in zip(ds[:-1],ds[1:]):
            if b-a<=tol:continue
            split.append(dict(layer=L,pts=[list(line.interpolate(a).coords[0]),list(line.interpolate(b).coords[0])],w=[w]))
            orig.append((ti,si))
    g=net_graph(split,[],pads,tol=tol);adj=g['adj'];pad_pts=g['pad_pts']
    for vi,v in enumerate(vias):
        ks=[net_copper(L,(v['x'],v['y'])) for L in LAYERS]
        for a,b in zip(ks[:-1],ks[1:]):
            adj[a].append((b,('v',vi)));adj[b].append((a,('v',vi)))
        for p in pads:
            for L,geo in pad_geo(p).items():
                if L in LAYERS and geo.distance(Point(v['x'],v['y']))<=float(v.get('d',VIA_D))/2+tol:
                    left.append((('PAD',p['ref'],p['num']),net_copper(L,(v['x'],v['y']))))
    for a,b in left+via_left:
        adj[a].append((b,None));adj[b].append((a,None))

    parent={q:q for q in adj}
    def root(q):
        while parent[q]!=q: parent[q]=parent[parent[q]];q=parent[q]
        return q
    for a,qs in adj.items():
        for b,kid in qs:
            if kid is None: parent[root(b)]=root(a)
    sources={root(pad_pts[q]) for q in fl['source'] if q in pad_pts and pad_pts[q] in parent}
    resistors={}; members=defaultdict(list)
    for a,qs in adj.items():
        for b,kid in qs:
            if kid is None or a>b:continue
            u,v=root(a),root(b)
            if u==v:continue
            if kid[0]=='s':
                t=split[kid[1]]
                resistance=1.72e-5*math.dist(t['pts'][0],t['pts'][1])*(kid[4]-kid[3])/(t['w'][0]*CURRENT_T[t['layer']])
                tag=('s',*orig[kid[1]])
            else:
                resistance=.0024/max(1,NL-1); tag=('v',kid[1],a[0],b[0])
            if resistance<=1e-15:continue
            key=('s',a,b) if kid[0]=='s' else ('v',kid[1],a,b)


            old=resistors.get(key)
            resistors[key]=(u,v,max(old[2] if old else 0.,1/resistance));members[key].append(tag)
    edges=defaultdict(float)
    for u,v,conductance in resistors.values():edges[tuple(sorted((u,v)))]+=conductance
    conn=defaultdict(list)
    for u,v in edges:conn[u].append(v);conn[v].append(u)
    reached=set(sources);todo=list(sources)
    while todo:
        for v in conn[todo.pop()]:
            if v not in reached:reached.add(v);todo.append(v)
    missing=sorted(q for q in set(fl['load'])|set(fl['source']) if q not in pad_pts or pad_pts[q] not in parent or root(pad_pts[q]) not in reached)
    if missing:return dict(absent=missing,seg={},via={},total=sum(map(float,fl['load'].values())))
    nodes=sorted(reached-sources);idx={q:i for i,q in enumerate(nodes)};rhs=np.zeros(len(nodes));row=[];col=[];val=[]
    for q,I in fl['load'].items():
        k=root(pad_pts[q])
        if k in idx:rhs[idx[k]]+=float(I)
    for (u,v),conductance in edges.items():
        if u not in reached:continue
        for a,b in ((u,v),(v,u)):
            if a in idx:
                row.append(idx[a]);col.append(idx[a]);val.append(conductance)
                if b in idx:row.append(idx[a]);col.append(idx[b]);val.append(-conductance)
    voltage=spsolve(coo_matrix((val,(row,col)),shape=(len(nodes),len(nodes))).tocsr(),rhs) if nodes else np.zeros(0)
    if not np.all(np.isfinite(voltage)):raise ValueError('current_first: singular final graph')
    amps={};viamps={}
    for key,(u,v,conductance) in resistors.items():
        if u not in reached:continue
        I=abs((voltage[idx[u]] if u in idx else 0.)-(voltage[idx[v]] if v in idx else 0.))*conductance
        for tag in members[key]:
            if tag[0]=='s':amps[tag[1:]]=max(amps.get(tag[1:],0.),I)
            else:viamps[tag[1:]]=max(viamps.get(tag[1:],0.),I)
    return dict(absent=[],seg=amps,via=viamps,total=sum(map(float,fl['load'].values())))


def current_correct(u, C):
    r=U[u];net=r['net'];fl=current_model(net)
    info=current_actual(r['tracks'],r['vias'],[D['pads'][i] for i in by_net[net]],fl)
    if info['absent']:return info
    goal={(ti,si):max(TRACK_MIN,DRU_MIN.get(net,0.),min(NECK_W,C['wmin']),I/(CURRENT_J*CURRENT_T[r['tracks'][ti]['layer']]))
          for (ti,si),I in info['seg'].items()}
    FLOW_W[u]=goal


    points=[p for t in r['tracks'] for p in t['pts']]+[[v['x'],v['y']] for v in r['vias']]
    xs=[p[0] for p in points];ys=[p[1] for p in points]
    limit=max([C.get('w',0.)]+list(goal.values())+[w for t in r['tracks'] for w in t['w']])
    capH,*_=maps(net,(min(xs)-2.,min(ys)-2.,max(xs)+2.,max(ys)+2.),limit)
    sb=_disturb_width(net)
    X=_cand_idx_build(u);tr=copy.deepcopy(r['tracks']);vs=copy.deepcopy(r['vias'])
    metric=_metric_prepare(r,net) if CHAIN and any('metric' in t for t in tr) else None
    ab=_tap_prepare(r,net) if TAP_ON and any(t.get('tap') for t in tr) else None
    widths=[list(t['w']) for t in tr]
    for ti,t in enumerate(tr):
        li=LI[t['layer']]
        for si,(a,b,w) in enumerate(zip(t['pts'][:-1],t['pts'][1:],t['w'])):
            target=max(w,goal.get((ti,si),w));limit=_r32_clearance(X,u,li,a,b,net,target)
            limit=min(limit,line_cap_min(capH[li],a,b),2*pad_room(li,a,b,net))
            if metric is not None:limit=min(limit,_metric_width(metric,r,widths,ti,a,b,li))
            if ab is not None:limit=min(limit,_tap_width(ab,a,b,li))
            for dk,sp in sb:
                limit=min(limit,2*(line_cap_min(dk[li],a,b)-sp))
                for lj,s in neighbor_spacing(li,sp):limit=min(limit,2*(line_cap_min(dk[lj],a,b)-s))
            t['w'][si]=max(w,min(target,limit))
        widths[ti]=t['w']


    for vi,v in enumerate(vs):
        v['current_actual_A']=max([I for (j,_,_),I in info['via'].items() if j==vi]+[0.])
        if not v.get('n'):continue
        ids={j for j,w in enumerate(vs) if j==vi or w.get('too')==[v['x'],v['y']]}
        sums={}
        for (j,a,b),I in info['via'].items():
            if j in ids:sums[a,b]=sums.get((a,b),0.)+I
        I=max(sums.values(),default=0.);v['current_A']=I
        imax=max([a for (j,_,_),a in info['via'].items() if j in ids]+[0.])
        v['n']=max(2,math.ceil(4*I-1e-8),math.ceil(len(ids)*imax/.25-1e-8))
        if v['n']<=len(ids):continue
        layers=v.get('current_layers')
        if not layers:continue
        bridges,extras=current_field_set([LI[layers[0]]*NN,LI[layers[1]]*NN],[dict(x=v['x'],y=v['y'])],dict(current_I=I,current_n=v['n']))
        template=next((t for t in tr if t.get('web') and t.get('link_chain')==v.get('link_chain')), {})
        for t in bridges:t.update({k:template[k] for k in ('link_chain','kp') if k in template})
        new=[w for w in extras if all(math.dist((w['x'],w['y']),(q['x'],q['y']))>.02 for q in vs)]
        if not new:continue
        if any(math.dist((w['x'],w['y']),(q['x'],q['y']))<HOLE_C-1e-6 for w in new for q in vs):continue
        good=True
        for w in new:
            ix,iy=int((w['x']-X0)/RES),int((w['y']-Y0)/RES);q=(w['x'],w['y'])
            good &= 0<=ix<NX and 0<=iy<NY
            if not good:break
            good &= via_hard(net,np.array([iy*NX+ix]))
            good &= all(_r32_clearance(X,u,li,q,q,net,VIA_D)>=VIA_D-1e-9 for li in range(NL))
            good &= all(math.dist(q,(v2['x'],v2['y']))>=HOLE_C-1e-6
                        for uu,rr in U.items() if uu!=u for v2 in rr['vias'])
        if not good:continue
        for t in bridges:
            li=LI[t['layer']]
            for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']):
                good &= _r32_clearance(X,u,li,a,b,net,w)>=w-1e-9 and 2*pad_room(li,a,b,net)>=w-1e-9
                good &= line_cap_min(capH[li],a,b)>=w-1e-9
                if metric is not None:
                    rr=dict(r,tracks=tr+[t]);ww=widths+[t['w']]
                    good &= _metric_width(metric,rr,ww,len(tr),a,b,li)>=w-1e-9
                if ab is not None:good &= _tap_width(ab,a,b,li)>=w-1e-9
        if good:tr+=bridges;vs+=new
    del_unit(u);add_unit(u,net,tr,vs)
    return info


def current_final(u, C):
    for _ in range(8):
        before=copy.deepcopy((U[u]['tracks'],U[u]['vias']))
        info=current_correct(u,C)
        if info['absent'] or before==(U[u]['tracks'],U[u]['vias']):break
    r=U[u];info=current_actual(r['tracks'],r['vias'],[D['pads'][i] for i in by_net[r['net']]],current_model(r['net']))
    deficit=[]
    for (ti,si),I in info['seg'].items():
        t=r['tracks'][ti];target=I/(CURRENT_J*CURRENT_T[t['layer']])
        if target>t['w'][si]+1e-6:deficit.append(dict(track=ti,segment=si,current_A=I,actual_mm=t['w'][si],nominal_mm=target))
    via_deficit=[]
    for vi,v in enumerate(r['vias']):
        I=max([I for (j,_,_),I in info['via'].items() if j==vi]+[0.]);v['current_actual_A']=I
        if I>.25+1e-6:via_deficit.append(dict(via=vi,current_A=I,nominal_A=.25))


    for vi,v in enumerate(r['vias']):
        if not v.get('n'):continue
        ids={j for j,w in enumerate(r['vias']) if j==vi or w.get('too')==[v['x'],v['y']]};sums={}
        for (j,a,b),I in info['via'].items():
            if j in ids:sums[a,b]=sums.get((a,b),0.)+I
        v['current_A']=max(sums.values(),default=0.)
        imax=max([a for (j,_,_),a in info['via'].items() if j in ids]+[0.])
        v['n']=max(2,math.ceil(4*v['current_A']-1e-8),math.ceil(len(ids)*imax/.25-1e-8))
    audit=CURRENT_AUDIT.setdefault(r['net'],{})
    audit['final_check']=dict(model='Resistance graph/Kirchhoff, ideal contacts',current_A=info['total'],
                             missing_pads=info['absent'],width_deficits=deficit,via_deficits=via_deficit,
                             via_field_missing=cons_missing(u))
    return not (info['absent'] or deficit or via_deficit or cons_missing(u))
IMP_REF_MASK = {}; IMP_REF_GEOM = {}
if IMP_TAB and D.get('plane_domains'):
    from analysis.structure import reference_planes as _z_areas
    _z_polys = _z_areas(D, GNDS, dict(options={'gnd_layers': args.get('gnd_layers', 'In1')}), _all_nets)
    _z_x, _z_y = np.meshgrid(GXc, GYc)
    for _net, _tab in sorted(IMP_TAB.items()):
        for _layer, _q in sorted(_tab.items()):
            _key = (_net, _layer)
            _mask = np.ones((NY, NX), dtype=bool)
            _geom = None
            for _ref in _q['profile_data']['ref']:
                _p = _z_polys.get(_ref, Polygon())
                _geom = _p if _geom is None else _geom.intersection(_p)
            IMP_REF_GEOM[_key] = _geom

            _mask &= shapely.contains_xy(_geom.buffer(-_q['w']/2 - RES), _z_x, _z_y)
            IMP_REF_MASK[_key] = _mask
    del _z_x, _z_y


def z_ref_free(C, li, a, b, net=None):
    if not C.get('zw'): return True
    g = IMP_REF_GEOM.get((NET_CUR[0] if net is None else net, LAYERS[li]))
    return g is None or g.covers(_LS([a, b]).buffer(C['zw'][li]/2))

PLANE_MASK = {}; PLANE_AREA = {}; PLANE_INFO = None
PLANE_COPPER = {}; PLANE_TRACE = {}; PLANE_LAYER = {}; PLANE_REF = {}
PLANE_FEEDERS = False
if (BGA_PLAN or PIN_ACCESS) and D.get('plane_domains'):
    PLANE_LAYER = {}; PLANE_COPPER = {}


    import hashlib as _plane_hash
    _raw = open(D['plane_domains'], 'rb').read(); _domains = json.loads(_raw)
    PLANE_FEEDERS = PIN_ACCESS and bool(_domains.get('group_feeders',False))
    if PIN_ACCESS and _domains.get('outline_targets') and not _domains.get('reference_domains'):
        raise ValueError('outline_targets needs native reference_domains')
    PLANE_REF={}
    if PIN_ACCESS:
        for _z in _domains.get('reference_domains',_domains['zones']):
            PLANE_REF.setdefault((_z['net'],_z['layer']),[]).append(Polygon(_z['pts'],_z.get('holes',[])).buffer(0))
        PLANE_REF={key:shapely.union_all(gs) for key,gs in PLANE_REF.items()}
    _plane_nets = {}
    if PIN_ACCESS and not BGA_PLAN:

        _plane_nets = {n: [] for n in sorted(AREA_NETS)}
    for _z in _domains['zones']:
        if PIN_ACCESS:PLANE_LAYER.setdefault((_z['net'],_z['layer']),[]).append(Polygon(_z['pts'],_z.get('holes',[])).buffer(0))
        if _z.get('layer') in globals().get('PLANE_SIGNALS', set()):
            PLANE_COPPER.setdefault((_z['net'],_z['layer']),[]).append(Polygon(_z['pts'],_z.get('holes',[])).buffer(0))
        if _z['net'] in AREA_NETS:
            if PIN_ACCESS and not BGA_PLAN and _z.get('layer') not in [L for L,nn in FL_ASSIGNMENT.items() if _z['net'] in nn]: continue
            _plane_nets.setdefault(_z['net'], []).append(Polygon(_z['pts'], _z.get('holes', [])).buffer(0))
    for _n, _gs in sorted(_plane_nets.items()):
        _area = shapely.union_all(_gs).buffer(-VIA_R - 0.01)
        if PIN_ACCESS and _domains.get('outline_targets') and _n in globals().get('PLANE_MIXED_NETS',()) and _n not in GNDS:

            _original=[g for (n,L),g in PLANE_REF.items() if n==_n and L in PLANE_SIGNALS and n in FL_ASSIGNMENT.get(L,[])]
            _area=shapely.union_all(_original).buffer(-VIA_R-.01) if _original else Polygon()
        PLANE_AREA[_n] = _area
        _mask = np.zeros((NY, NX), dtype=bool)
        if not _area.is_empty:
            _x0,_y0,_x1,_y1 = _area.bounds
            _ix0,_ix1 = max(0,int((_x0-X0)/RES)),min(NX,int((_x1-X0)/RES)+2)
            _iy0,_iy1 = max(0,int((_y0-Y0)/RES)),min(NY,int((_y1-Y0)/RES)+2)
            _xx,_yy = np.meshgrid(GXc[_ix0:_ix1], GYc[_iy0:_iy1])
            _mask[_iy0:_iy1,_ix0:_ix1] = shapely.contains_xy(_area,_xx,_yy)
        PLANE_MASK[_n] = _mask
    PLANE_INFO = dict(sha256=_plane_hash.sha256(_raw).hexdigest(),net_list=sorted(PLANE_AREA),cu_layers=_domains.get('gnd_layers'))
    PLANE_COPPER = {key:shapely.union_all(gs) for key,gs in PLANE_COPPER.items()}
    PLANE_LAYER = {key:shapely.union_all(gs) for key,gs in PLANE_LAYER.items()}


def plane_trace_maps(net, capH, rng=None):
    if not PLANE_COPPER: return
    ix0,ix1,iy0,iy1 = rng if rng is not None else (0,NX,0,NY)
    ys,xs=slice(iy0,iy1),slice(ix0,ix1)
    for (n,L),g in PLANE_COPPER.items():
        if n==net:continue
        key=(n,L)
        if key not in PLANE_TRACE:
            xx,yy=np.meshgrid(GXc,GYc)
            blocked=shapely.intersects_xy(g,xx,yy)
            PLANE_TRACE[key]=(_edt(~blocked)*RES-QM-CLR-.005).astype(np.float32)*2
            PLANE_TRACE[key][blocked]=-1.
        np.minimum(capH[LI[L]][ys,xs],PLANE_TRACE[key][ys,xs],out=capH[LI[L]][ys,xs])


def plane_feeder_targets(net, p, li, viaH, viaBad, rng):
    x0,x1,y0,y1=rng
    mask=viaH[y0:y1,x0:x1] & ~viaBad[y0:y1,x0:x1] & PLANE_MASK[net][y0:y1,x0:x1]
    yy,xx=np.nonzero(mask);xx=xx+x0;yy=yy+y0
    if not len(xx):return set()
    d=(GXc[xx]-p['x'])**2+(GYc[yy]-p['y'])**2
    order=np.lexsort((xx,yy,d))[:32]
    return {li*NN+int(yy[i])*NX+int(xx[i]) for i in order}
PF = [float(args.get('pf0', '0.6'))]
HARD_FOREIGN = [False]
SOURCE_FREE = [args.get('source_free', '0') == '1']
NET_CUR = ['']


def oct_net(net=None):

    return not _net_organic(NET_CUR[0] if net is None else net, NC) if STYLE_PER_NET_KIND else OCT

DISTURB_FACTOR = [1.0]
VIA_MASK = [None]
VIA_FACTOR = [1.0]
VIAF = VIA_FACTOR
ASTAR_COST = [None]
ASTAR_HINT = [None]
_LOOP_DEADLINE = None


def _loop_cpu_check():
    work_emergency_stop()
    if _LOOP_DEADLINE is not None and budget_clock() >= _LOOP_DEADLINE:
        raise TimeoutError('Loop CPU budget in search')

TAPS = args.get('taps', '1' if NC and NC.get('taps') else '0') == '1'



TAP_NC = TAPS and bool(NC) and 'taps' in NC and args.get('taps_source', 'netclasses') == 'netclasses'
WINDOW = float(args.get('window', '0'))
CONFLICT_SITE_EXACT = args.get('conflict_site', '') == 'exact'
NECK = float(args.get('neck', '0')); NECK_CACHE = {}
_NECK_MAX = max(4, int(float(args.get('neck_mb', '600')) * 1e6 / (NL * NN * 4)))


HFAC = float(args.get('astar', '1'))


BEND = float(args.get('bend', '0'))
if not math.isfinite(BEND) or BEND < 0: raise ValueError('bend must be finite and >= 0')


BEND_CURRENT = float(args.get('bend_current', '1'))
if not math.isfinite(BEND_CURRENT) or BEND_CURRENT < 0: raise ValueError('bend_current must be finite and >= 0')
VIA_TOO = [None]




VIA_WEB = float(args.get('via_web', '0.03'))
NECK_W = float(args.get('neck_w', args.get('sigw', '0.15') if NC else '0.25'))

NECK_FIELD = args.get('neck_field', '1' if NC else '0') == '1'
DENSE = []
if NECK_FIELD:
    from scipy.spatial import cKDTree as _KD
    _per = {}
    for _p in D['pads']: _per.setdefault(_p['ref'], []).append(_p)
    for _r in sorted(_per):
        _P = _per[_r]
        if len(_P) < 16: continue
        _xy = np.array([(q['x'], q['y']) for q in _P]); _pit = float(np.median(_KD(_xy).query(_xy, k=2)[0][:, 1]))
        if _pit > 0.85: continue
        _lg = sorted({LI[L] for q in _P for L in q['layers'] if L in LI})
        DENSE.append((_lg, max(0, int((_xy[:, 0].min() - _pit - X0) / RES)), min(NX, int((_xy[:, 0].max() + _pit - X0) / RES) + 1),
                      max(0, int((_xy[:, 1].min() - _pit - Y0) / RES)), min(NY, int((_xy[:, 1].max() + _pit - Y0) / RES) + 1),
                      {q['net'] for q in _P if q['net']}))
    if DENSE: print('dense fields (BGA, fine grid):', len(DENSE))
by_net_g = {}
for _k, _p in enumerate(D['pads']):
    if _p['net']: by_net_g.setdefault(_p['net'], []).append(_k)
_EMPTY = np.zeros(0, dtype=np.float32)
_MULT = np.empty(NL * NN, dtype=np.float32)


class _Maps(list):
    planar = None





CLOSE = args.get('near_map', '1') == '1'
CLOSE_W = float(args.get('close_w', '0.6'))




AREA = float(args.get('surface', '0'))
FL_GAP = float(args.get('area_gap', '0.6'))
FL_WMAX = float(args.get('area_wmax', '0.3'))



FL_PRICE = float(args.get('area_price', '0'))
if AREA > 0 or FL_PRICE > 0: CLOSE_W = max(CLOSE_W, 2 * (FL_GAP + FL_WMAX / 2 - CLR - RES))
D_CLOSE = max(CLOSE_W, VIA_D) / 2 + CLR + 2 * RES; PCN = int(math.ceil(D_CLOSE / RES)) + 2
CAP_CLOSE = np.float32(2 * (D_CLOSE - CLR))
D_BORE = HOLE_C + 2 * RES; PCB = int(math.ceil(D_BORE / RES)) + 2
SN = {}
UOWN = {}
NETU = {}
DDN = np.full(NN, D_BORE, dtype=np.float32)
SN_OPEN = {}
_TMP = np.zeros(NL * NN, dtype=bool)
_NVER = {}
_GEO = {}
_TRUNK = {}
_TRUNK_V = {}
if CLOSE:
    SN[''] = np.full(NL * NN, CAP_CLOSE, dtype=np.float32)
    for _g in sorted(AREA_NETS):
        SN[_g] = np.full(NL * NN, CAP_CLOSE, dtype=np.float32); UOWN[_g] = np.zeros(NL * NN, dtype=np.int16); SN_OPEN[_g] = []
FIXD = [None, None]
FIXNETS = set()
OFFS = [(1, 0, RES), (-1, 0, RES), (0, 1, RES), (0, -1, RES),
        (1, 1, RES * 1.4142), (1, -1, RES * 1.4142), (-1, 1, RES * 1.4142), (-1, -1, RES * 1.4142)]
GXc = X0 + (np.arange(NX) + 0.5) * RES; GYc = Y0 + (np.arange(NY) + 0.5) * RES


def seg_cells(li, a, b, w):
    (x0, y0), (x1, y1) = a, b
    r = w / 2 + QM
    ixa = max(0, int((min(x0, x1) - r - X0) / RES) - 1); ixb = min(NX, int((max(x0, x1) + r - X0) / RES) + 2)
    iya = max(0, int((min(y0, y1) - r - Y0) / RES) - 1); iyb = min(NY, int((max(y0, y1) + r - Y0) / RES) + 2)
    GX, GY = np.meshgrid(GXc[ixa:ixb], GYc[iya:iyb])
    dx, dy = x1 - x0, y1 - y0; ll = dx * dx + dy * dy
    t = np.clip(((GX - x0) * dx + (GY - y0) * dy) / ll, 0, 1) if ll > 0 else 0
    m = np.hypot(GX - (x0 + t * dx), GY - (y0 + t * dy)) <= r
    yy, xx = np.nonzero(m)
    return li * NN + (yy + iya) * NX + (xx + ixa)


def drill_cells(x, y):
    r = VIA_DR / 2
    ix, iy = int((x - X0) / RES), int((y - Y0) / RES); rr = int(r / RES) + 1
    out = []
    for jy in range(iy - rr, iy + rr + 1):
        for jx in range(ix - rr, ix + rr + 1):
            if 0 <= jx < NX and 0 <= jy < NY and math.hypot(GXc[jx] - x, GYc[jy] - y) <= r + QM: out.append(jy * NX + jx)
    return np.array(out, dtype=np.int64)


def _vd_slice(x, y):
    r = VD_WINDOW / 2
    ax = max(0, int(math.ceil((x - r - X0) / RES - 0.5 - 1e-9)))
    bx = min(NX, int(math.floor((x + r - X0) / RES - 0.5 + 1e-9)) + 1)
    ay = max(0, int(math.ceil((y - r - Y0) / RES - 0.5 - 1e-9)))
    by = min(NY, int(math.floor((y + r - Y0) / RES - 0.5 + 1e-9)) + 1)
    return slice(ay, by), slice(ax, bx)


def _vd_update(net, vias, delta):
    for v in vias:
        key = (round(v['x'], 6), round(v['y'], 6), v.get('net') or net)
        before = _VD_VIAS.get(key, 0); after = before + delta
        if after < 0: raise RuntimeError('Via density reference counter negative')
        if bool(before) != bool(after):
            _VD_COUNT[_vd_slice(v['x'], v['y'])] += 1 if after else -1
            _VD_VERSION[0] += 1
        if after: _VD_VIAS[key] = after
        else: _VD_VIAS.pop(key, None)


def _vd_window_max(z):
    n = 2 * int(math.floor(VD_WINDOW / (2 * RES) + 1e-9)) + 1
    return ndimage.maximum_filter1d(ndimage.maximum_filter1d(z, n, axis=0, mode='constant', cval=0),
                                    n, axis=1, mode='constant', cval=0)


def _vd_cost(local=()):
    c = _VD_CACHE[0]
    if not local and c is not None and c[0] == _VD_VERSION[0]: return c[1]
    z = _VD_COUNT.astype(np.float32)
    seen = set()
    for v in local:
        key = (round(v['x'], 6), round(v['y'], 6))
        present = (*key, v.get('net') or NET_CUR[0]) in _VD_VIAS
        if key not in seen and not present:
            z[_vd_slice(*key)] += 1; seen.add(key)


    z = np.square(np.maximum(np.float32(0), _vd_window_max(z) + np.float32(1 - VD_THRESHOLD * VD_WINDOW ** 2)))
    if not local: _VD_CACHE[0] = (_VD_VERSION[0], z)
    return z


def add_unit(u, net, tracks, vias):
    if globals().get('KA') is not None: _KA_CELLS.pop(u,None)
    if globals().get('PIN_ACCESS', False) or globals().get('KA') is not None:
        _PZ_VERSION[1]+=1
        _PZ_UNITS.pop(u,None)
        if u.startswith('FIX#'):
            _PZ_VERSION[0]+=1
            if '_pad_room_cached' in globals():_pad_room_cached.cache_clear()
    if current_model(net): _SZ_VERSION[0] += 1
    vv = []
    for v in vias:
        if all(math.hypot(v['x'] - w['x'], v['y'] - w['y']) > 0.02 for w in vv): vv.append(v)
    vias = vv
    cl = []
    for t in tracks:
        li = LI[t['layer']]
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']): cl.append(seg_cells(li, a, b, w))
    dl = []
    for v in vias:


        for li in range(NL): cl.append(seg_cells(li, (v['x'], v['y']), (v['x'], v['y']), 2 * VIA_OCC_R))
        dl.append(drill_cells(v['x'], v['y']))
    cells = np.unique(np.concatenate(cl)) if cl else np.zeros(0, dtype=np.int64)
    dcells = np.unique(np.concatenate(dl)) if dl else np.zeros(0, dtype=np.int64)
    use[cells] += 1; drill_use[dcells] += 1
    _TRUNK.clear(); _TRUNK_V.clear()
    bb = _sub_boxes(cl, dl) if CLOSE else None
    U[u] = dict(net=net, tracks=tracks, vias=vias, cells=cells, dcells=dcells, bb=bb)
    if VD_COUNTER: _vd_update(net, vias, 1)
    _NVER[net] = _NVER.get(net, 0) + 1; _GEO.pop(u, None)
    if CLOSE:
        NETU[net] = NETU.get(net, 0) + 1
        if net in UOWN: UOWN[net][cells] += 1
        _close_update(bb, net, cells)


def del_unit(u):
    if globals().get('KA') is not None: _KA_CELLS.pop(u,None)
    r = U.pop(u, None)
    _TRUNK.clear(); _TRUNK_V.clear()
    if r is None: return
    if globals().get('PIN_ACCESS', False) or globals().get('KA') is not None:
        _PZ_VERSION[1]+=1
        _PZ_UNITS.pop(u,None)
        if u.startswith('FIX#'):
            _PZ_VERSION[0]+=1
            if '_pad_room_cached' in globals():_pad_room_cached.cache_clear()
    if current_model(r['net']): _SZ_VERSION[0] += 1
    if VD_COUNTER: _vd_update(r['net'], r['vias'], -1)
    if VIA_ANCHOR > 0: VIA_PREV[u] = [(v['x'], v['y']) for v in r['vias'] if not v.get('rv') and not v.get('gr')]
    use[r['cells']] -= 1; drill_use[r['dcells']] -= 1
    _NVER[r['net']] = _NVER.get(r['net'], 0) + 1; _GEO.pop(u, None)
    if CLOSE:
        NETU[r['net']] -= 1
        if r['net'] in UOWN: UOWN[r['net']][r['cells']] -= 1
        _close_update(r['bb'], r['net'])


def _merge(B, border):
    B = sorted(set(B)); fl = lambda b: (b[1] - b[0] + 2 * border) * (b[3] - b[2] + 2 * border) + 6000
    i = 0
    while i < len(B):
        j = i + 1; new = False
        while j < len(B):
            a, b = B[i], B[j]; v = (min(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), max(a[3], b[3]))
            if fl(v) <= fl(a) + fl(b): B[i] = v; del B[j]; new = True
            else: j += 1
        if not new: i += 1
    return B


def _sub_boxes(cl, dl):
    per = {}
    for a in cl:
        if not len(a): continue
        li = int(a[0]) // NN; idx = a - li * NN; iy = idx // NX; ix = idx - iy * NX
        per.setdefault(li, []).append((int(iy.min()), int(iy.max()) + 1, int(ix.min()), int(ix.max()) + 1))
    bo = []
    for a in dl:
        if not len(a): continue
        iy = a // NX; ix = a - iy * NX
        bo.append((int(iy.min()), int(iy.max()) + 1, int(ix.min()), int(ix.max()) + 1))
    return dict(copper_map=[(li, b) for li in sorted(per) for b in _merge(per[li], 2 * PCN)], bore=_merge(bo, 2 * PCB))


def _close_new(S, key, li, y0, y1, x0, x1):
    ay0, ay1, ax0, ax1 = max(0, y0 - PCN), min(NY, y1 + PCN), max(0, x0 - PCN), min(NX, x1 + PCN)
    wy0, wy1, wx0, wx1 = max(0, ay0 - PCN), min(NY, ay1 + PCN), max(0, ax0 - PCN), min(NX, ax1 + PCN)
    sl = slice(li * NN, (li + 1) * NN)
    ul = use[sl].reshape(NY, NX)[wy0:wy1, wx0:wx1]
    f = (ul > 0) if key == '' else ((ul - UOWN[key][sl].reshape(NY, NX)[wy0:wy1, wx0:wx1]) > 0)
    goal = S[sl].reshape(NY, NX)
    if f.any():
        d = (_edt(~f) * RES).astype(np.float32)[ay0 - wy0:ay1 - wy0, ax0 - wx0:ax1 - wx0]
        goal[ay0:ay1, ax0:ax1] = np.minimum(2 * (d - CLR), CAP_CLOSE)
    else:
        goal[ay0:ay1, ax0:ax1] = CAP_CLOSE


def _close_update(bb, net, cells=None):
    S = SN['']
    if cells is not None: _TMP[cells] = True
    for li, (y0, y1, x0, x1) in bb['copper_map']:
        if cells is None: _close_new(S, '', li, y0, y1, x0, x1)
        else:
            ay0, ay1, ax0, ax1 = max(0, y0 - PCN), min(NY, y1 + PCN), max(0, x0 - PCN), min(NX, x1 + PCN)
            sl = slice(li * NN, (li + 1) * NN)
            d = (_edt(~_TMP[sl].reshape(NY, NX)[ay0:ay1, ax0:ax1]) * RES).astype(np.float32)
            z = S[sl].reshape(NY, NX)[ay0:ay1, ax0:ax1]
            np.minimum(z, np.minimum(2 * (d - CLR), CAP_CLOSE), out=z)
    if cells is not None: _TMP[cells] = False
    for key, off in SN_OPEN.items():
        if key == net or off is None: continue
        off.extend((li, b) for li, b in bb['copper_map'])
        if len(off) > 400: SN_OPEN[key] = None
    D2 = DDN.reshape(NY, NX); du = drill_use.reshape(NY, NX)
    for y0, y1, x0, x1 in bb['bore']:
        ay0, ay1, ax0, ax1 = max(0, y0 - PCB), min(NY, y1 + PCB), max(0, x0 - PCB), min(NX, x1 + PCB)
        wy0, wy1, wx0, wx1 = max(0, ay0 - PCB), min(NY, ay1 + PCB), max(0, ax0 - PCB), min(NX, ax1 + PCB)
        fd = du[wy0:wy1, wx0:wx1] > 0
        if fd.any():
            dd = (_edt(~fd) * RES).astype(np.float32)[ay0 - wy0:ay1 - wy0, ax0 - wx0:ax1 - wx0]
            D2[ay0:ay1, ax0:ax1] = np.minimum(dd, D_BORE)
        else:
            D2[ay0:ay1, ax0:ax1] = D_BORE


def _close_flush(key):
    off = SN_OPEN[key]
    if off is None:
        S = SN[key]
        for li in range(NL):
            sl = slice(li * NN, (li + 1) * NN)
            f = ((use[sl] - UOWN[key][sl]) > 0).reshape(NY, NX)
            if f.any(): S[sl] = np.minimum(2 * ((_edt(~f) * RES).astype(np.float32) - CLR), CAP_CLOSE).ravel()
            else: S[sl] = CAP_CLOSE
    else:
        per = {}
        for li, b in off: per.setdefault(li, []).append(b)
        for li in sorted(per):
            for b in _merge(per[li], 2 * PCN): _close_new(SN[key], key, li, *b)
    SN_OPEN[key] = []


def own_counts(net):
    own = np.zeros(NL * NN, dtype=np.int16); od = np.zeros(NN, dtype=np.int16)
    for r in U.values():
        if r['net'] == net:
            own[r['cells']] += 1; od[r['dcells']] += 1
    return own, od


def _box_idx(box):
    if box is None: return (0, NX, 0, NY)
    ix0, ix1 = max(0, int((box[0] - X0) / RES) - 1), min(NX, int((box[2] - X0) / RES) + 2)
    iy0, iy1 = max(0, int((box[1] - Y0) / RES) - 1), min(NY, int((box[3] - Y0) / RES) + 2)
    if ix1 <= ix0 or iy1 <= iy0: return (0, NX, 0, NY)
    return (ix0, ix1, iy0, iy1)




_KA_META = {}
_KA_HARD = {}
_KA_CELLS = {}


def _ka_kind(g, pad=False):
    return _KA_META.get(id(g), ('Pad' if pad else 'Track', ''))


def _ka_pad_kind(p):
    if p.get('graphic') or p.get('board_copper') or p.get('ref','').startswith('CU_'):
        return ('Graphic','')
    return ('Pad',p.get('dru_pad_type') or ('NPTH, mechanical' if p.get('pad_type')=='npth' else 'Through-hole' if p.get('drill') else 'SMD'))


def _ka_pad_keeps(p,L):
    mk=(p.get('mask') or {}).get(L,0.)
    if mk>0:
        g=shapely.union_all([Polygon(q) for q in p['polys'][L]])
        yield g.buffer(mk+.01,join_style=2),'MaskKeepout'
    if p.get('drill',0)>0 and HOLE_CLR>0:
        g=(shapely.union_all([Polygon(q) for q in p['hole_polys']])
           if DRCL and p.get('hole_polys') else _PT(p['x'],p['y']).buffer(p['drill']/2,32))
        yield g.buffer(HOLE_CLR,32),'HoleKeepout'


def _ka_geo_limit(net, other, li, g=None, g2=None, pad=False):
    if globals().get('KA') is None: return _pz_clearance(net,other,pad)
    t,p = _ka_kind(g) if g is not None else ('Track','')
    t2,p2 = _ka_kind(g2,pad)
    return _ka_margin(net,other,li,t,t2,p,p2)
def _ka_obstacles(net, li, fixed=True, without=None, pads=True):
    if pads:
        for n,g in _PADG[li]:
            if n != net and not g.is_empty:
                type_,pad = _ka_kind(g,True)
                yield n,g,type_,pad
        for n,g,type_ in globals().get('_KA_PAD_KEEP',{}).get(li,[]):
            if n!=net and not g.is_empty:yield n,g,type_,''
    for u,r in U.items():
        if u == without or (fixed and not u.startswith('FIX#')): continue
        for l,g,_,n in _unit_geo(u,r):
            if l==li and n!=net and not g.is_empty:
                type_,pad = _ka_kind(g)
                yield n,g,type_,pad
    for source in (_ZP_GEO, PLANE_COPPER):
        for (n,L),g in source.items():
            if n!=net and L==LAYERS[li] and not g.is_empty:yield n,g,'Zone',''


def _ka_field(net, li, points, obstacles, type_='Track', margin=.012):
    groups={}
    for n,g,t,p in obstacles:
        c=_ka_clr(net,n,li,type_,t,'',p)+margin
        groups.setdefault(c,[]).append(g)
    out=np.full(len(points),9.,dtype=np.float64)
    for c,gs in sorted(groups.items()):

        d=_pz_distance(shapely.STRtree(gs),points)
        out=np.minimum(out,d-c)
    return out


def _ka_raster(net,li,rng,obstacles,type_='Track',margin=.012,raw=False):


    x0,x1,y0,y1=rng
    cap=max(1.,cls(net)['w']*max(1.,WIDTH_INNER)/2+.5,VIA_KEEP_R+.5)
    out=np.full((y1-y0,x1-x0),cap,dtype=np.float32)
    for n,g,t,p in obstacles:
        if g.is_empty:continue
        c=0. if raw else _ka_clr(net,n,li,type_,t,'',p)+margin
        bx=g.bounds;r=cap+c+RES
        xa=max(x0,int(math.floor((bx[0]-r-X0)/RES)));xb=min(x1,int(math.ceil((bx[2]+r-X0)/RES)))
        ya=max(y0,int(math.floor((bx[1]-r-Y0)/RES)));yb=min(y1,int(math.ceil((bx[3]+r-Y0)/RES)))
        if xa>=xb or ya>=yb:continue
        xx,yy=np.meshgrid(GXc[xa:xb],GYc[ya:yb])
        d=shapely.distance(shapely.points(xx,yy),g)-c
        v=out[ya-y0:yb-y0,xa-x0:xb-x0]
        np.minimum(v,d,out=v)
    return out


def _ka_hard_maps(net,rng=None):
    rng=tuple(rng) if LARGE and rng is not None else _WHOLE
    key=(net,rng,_PZ_VERSION[0],tuple(sorted(PZ_LOCAL.get(net,()))))
    if key in _KA_HARD:return _KA_HARD[key]
    x0,x1,y0,y1=rng;ys=slice(y0,y1);xs=slice(x0,x1)
    margin=PZ_BORDER_NM/1e6 if _pz_active(net) else .012
    capH=_Maps();capH.planar=np.full(NL*NN,-1.,dtype=np.float32)
    via=(edge_d[ys,xs]-EDGE>=VIA_R)&(VSP[ys,xs]>=H2H+VIA_DR/2+.01)
    if VIA_BAN is not None:via &= ~VIA_BAN[ys,xs]
    fx0,fx1=_fix_map(net)
    if fx1 is not None:via &= fx1[ys,xs]-QM>=HOLE_C
    for li in range(NL):
        obs=list(_ka_obstacles(net,li))
        track=_ka_raster(net,li,rng,obs,margin=margin)
        vroom=_ka_raster(net,li,rng,obs,'Via',margin)
        layer=capH.planar[li*NN:(li+1)*NN].reshape(NY,NX)
        layer[ys,xs]=2*np.minimum(track,edge_d[ys,xs]-EDGE)
        capH.append(layer)
        via &= vroom>=VIA_R
        if HOLE_CLR>0:
            raw=_ka_raster(net,li,rng,[o for o in obs if o[2] not in ('MaskKeepout','HoleKeepout')],raw=True)
            via &= raw>=VIA_DR/2+HOLE_CLR+margin
        if li in (0,NL-1) and not VIP:via &= padd[li][ys,xs]>=VIA_R+VIA_WEB
    viaH=np.zeros((NY,NX),dtype=bool);viaH[ys,xs]=via
    if net in PLANE_MASK and net not in globals().get('PLANE_MIXED_NETS',()):viaH[ys,xs] &= PLANE_MASK[net][ys,xs]
    viaH[ys,xs]=_zp_viah(net,viaH[ys,xs],ys,xs)
    if PAD_HOLE_DIST is not None:viaH[ys,xs] &= _pad_hole_free(PAD_HOLE_DIST[ys,xs])
    if VP:_vp_one(net,viaH,rng)

    allowed=viaH.copy()
    if EPZ:_ep_one(net,viaH,capH,rng)
    viaH &= allowed
    plane_trace_maps(net,capH,rng)
    while len(_KA_HARD)>=_HM_MAX:_KA_HARD.pop(next(iter(_KA_HARD)))
    _KA_HARD[key]=(capH,viaH)
    return capH,viaH


def _ka_raw_distance(points,geoms):
    return _pz_distance(shapely.STRtree(geoms),points)


def _ka_cells(u,r):
    if u in _KA_CELLS:return _KA_CELLS[u]
    groups={}
    for t in r['tracks']:
        li=LI[t['layer']];n=t.get('net') or r['net']
        for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']):
            groups.setdefault((li,n,'Track'),[]).append(seg_cells(li,a,b,w)-li*NN)
    for v in r['vias']:
        n=v.get('net') or r['net']
        for L in v.get('layers',LAYERS):
            if L not in LI:continue
            li=LI[L]
            groups.setdefault((li,n,'Via'),[]).append(seg_cells(li,(v['x'],v['y']),(v['x'],v['y']),float(v.get('d',VIA_D)))-li*NN)
    result={k:np.unique(np.concatenate(v)) for k,v in groups.items()}
    _KA_CELLS[u]=result
    return result


def _ka_soft_maps(net,box=None):



    rng=_box_idx(box);x0,x1,y0,y1=rng
    pad=int(math.ceil((max(KA.maximum,CLR)+1.5)/RES))+2
    jx0,jx1=max(0,x0-pad),min(NX,x1+pad);jy0,jy1=max(0,y0-pad),min(NY,y1+pad)
    cap=[]
    for li in range(NL):
        groups={}
        for u,r in U.items():
            for (l,n,t),cells in _ka_cells(u,r).items():
                if l!=li or n==net:continue
                c=max(_ka_clr(net,n,li,'Track',t),_ka_clr(net,n,li,'Via',t))+.005
                mask=groups.setdefault(c,np.zeros((jy1-jy0,jx1-jx0),dtype=bool))
                yy=cells//NX;xx=cells%NX
                inside=(yy>=jy0)&(yy<jy1)&(xx>=jx0)&(xx<jx1)
                mask[yy[inside]-jy0,xx[inside]-jx0]=True
        room=np.full((jy1-jy0,jx1-jx0),9.,dtype=np.float32)
        for c,mask in sorted(groups.items()):
            if mask.any():room=np.minimum(room,_edt(~mask)*RES-c)
        a=np.full((NY,NX),-1.,dtype=np.float32)
        a[y0:y1,x0:x1]=2*room[y0-jy0:y1-jy0,x0-jx0:x1-jx0];cap.append(a)
    fd=(drill_use>0).reshape(NY,NX)
    dd=(_edt(~fd)*RES).astype(np.float32) if fd.any() else np.full(fd.shape,9.,dtype=np.float32)
    return cap,dd,rng


def _ka_room(net,li,a,b,fixed=True,without=None):
    seg=_LS([a,b]) if a!=b else _PT(a)
    margin=PZ_BORDER_NM/1e6 if _pz_active(net) else PAD_EXTRA
    room=min((seg.distance(g)-_ka_clr(net,n,li,'Track',t,'',p)-margin
              for n,g,t,p in _ka_obstacles(net,li,fixed,without)),default=9.)
    board=globals().get('_edge_board')
    if board is not None:
        if not board.covers(seg):return -1.
        room=min(room,seg.distance(board.boundary)-float(args.get('edge','.30'))-margin)
    return room

def _ka_via_ok(net,x,y,fixed=True,without=None):
    pt=_PT(x,y);margin=PZ_BORDER_NM/1e6
    board=globals().get('_edge_board')
    if board is not None and (not board.covers(pt) or pt.distance(board.boundary)<VIA_R+float(args.get('edge','.30'))+margin):return False
    if any(pt.distance(g)<VIA_R+margin for g in _VIA_KEEP):return False
    for li in range(NL):
        for n,g,t,p in _ka_obstacles(net,li,fixed,without):
            limit=VIA_R if t in ('MaskKeepout','HoleKeepout') else max(VIA_R+_ka_clr(net,n,li,'Via',t,'',p),VIA_DR/2+HOLE_CLR)
            if pt.distance(g)<limit+margin:return False
        if li in (0,NL-1) and not VIP:
            if any(n==net and pt.distance(g)<VIA_R+VIA_WEB for n,g in _PADG[li]):return False

    return _ka_legacy_via_ok(net,x,y,fixed,without)


def soft_maps(net, box=None, wmax=None):
    if globals().get('KA') is not None: return _ka_soft_maps(net,box)
    if CLOSE and wmax is not None and wmax <= CLOSE_W:
        key = '' if NETU.get(net, 0) == 0 else (net if net in SN else None)
        if key is not None:
            if key and (SN_OPEN[key] is None or SN_OPEN[key]): _close_flush(key)
            capS = _Maps(SN[key][li * NN:(li + 1) * NN].reshape(NY, NX) for li in range(NL)); capS.planar = SN[key]
            return capS, DDN.reshape(NY, NX), _box_idx(None)
    own = None if (CLOSE and NETU.get(net, 0) == 0) else own_counts(net)[0]
    ix0, ix1, iy0, iy1 = _box_idx(box)
    P = int(1.5 / RES) + 2
    jx0, jx1, jy0, jy1 = max(0, ix0 - P), min(NX, ix1 + P), max(0, iy0 - P), min(NY, iy1 + P)
    part = box is not None and (jx0, jx1, jy0, jy1) != (0, NX, 0, NY)
    capS = []
    for li in range(NL):
        f = use[li * NN:(li + 1) * NN].reshape(NY, NX)[jy0:jy1, jx0:jx1]
        f = (f > 0) if own is None else ((f - own[li * NN:(li + 1) * NN].reshape(NY, NX)[jy0:jy1, jx0:jx1]) > 0)
        if f.any(): d = (_edt(~f) * RES).astype(np.float32)
        else: d = np.full(f.shape, 9.0, dtype=np.float32)
        if not part: capS.append(2 * (d - CLR)); continue
        full = np.full((NY, NX), -1.0, dtype=np.float32)
        full[iy0:iy1, ix0:ix1] = 2 * (d[iy0 - jy0:iy1 - jy0, ix0 - jx0:ix1 - jx0] - CLR)
        capS.append(full)
    fd = (drill_use > 0).reshape(NY, NX)[jy0:jy1, jx0:jx1]
    dd = (_edt(~fd) * RES).astype(np.float32) if fd.any() else np.full(fd.shape, 9.0, dtype=np.float32)
    if part:
        full = np.zeros((NY, NX), dtype=np.float32); full[iy0:iy1, ix0:ix1] = dd[iy0 - jy0:iy1 - jy0, ix0 - jx0:ix1 - jx0]; dd = full
    return capS, dd, _box_idx(box)


def _in_window(win, rng):
    ixa, ixb = max(1, int((win[0] - X0) / RES)), min(NX - 2, int((win[2] - X0) / RES))
    iya, iyb = max(1, int((win[1] - Y0) / RES)), min(NY - 2, int((win[3] - Y0) / RES))
    return rng[0] <= ixa and ixb < rng[1] and rng[2] <= iya and iyb < rng[3]


REACH = args.get('reachable', '1') == '1'
_HM = {}
_HM_MAX = max(4, int(float(args.get('hm_mb', '1500')) * 1e6 / (NL * NN * 4 + NN)))


LARGE = args.get('large', 'auto') == '1' or (args.get('large', 'auto') == 'auto' and len(nets) > _HM_MAX)
_WHOLE = (0, NX, 0, NY)
if LARGE:
    _B_CAPH = np.empty(NL * NN, dtype=np.float32); _B_CAPC = np.empty(NL * NN, dtype=np.float32)
    _B_VIAH = np.zeros(NN, dtype=bool); _B_VBAD = np.zeros(NN, dtype=bool)
    print('large board: maps per unit only in the window (%d nets, room for %d full maps)' % (len(nets), _HM_MAX))


_FIX_MAPS = {}


def _fix_map(net):
    if FIXD[0] is None or net not in FIXNETS: return FIXD
    if net not in _FIX_MAPS:
        fo = np.zeros(NL * NN, dtype=bool)
        for u, r in U.items():
            if u.startswith('FIX#') and r['net'] != net: fo[r['cells']] = True
        _FIX_MAPS[net] = [[(_edt(~fo[i * NN:(i + 1) * NN].reshape(NY, NX)) * RES).astype(np.float32) for i in range(NL)], FIXD[1]]
    return _FIX_MAPS[net]


def _pad_hole_free(dist):
    return dist >= VIA_DR/2+H2H+PAD_HOLE_RESERVE




_PZ_FIXED = {}; _PZ_HARD = {}; _PZ_DYNAMIC = {}; _PZ_UNITS = {}; _PZ_VERSION = [0,0]
_PZ_BUDGET_ACTIVE = [False]
_PZ_PARTIAL = [None]
_PZ_BASE = [False]


class _PZBudget(RuntimeError):
    pass


def _pz_check():
    if globals().get('_LOOP_DEADLINE') is not None: _loop_cpu_check()
    if _PZ_BUDGET_ACTIVE[0] and time_tight(.95):raise _PZBudget('Pin access post-pass: CPU budget')


def _pz_index(net, fixed=True, without=None):
    from shapely.strtree import STRtree
    key=(net,_PZ_VERSION[0 if fixed else 1],without)
    cache=_PZ_FIXED if fixed else _PZ_DYNAMIC
    if key in cache:return cache[key]
    units = [(u,r) for u,r in U.items() if u != without and (not fixed or u.startswith('FIX#'))]
    layers = [[] for _ in LAYERS]; holes = []
    for u,r in units:
        _pz_check()
        if u not in _PZ_UNITS:
            copper=[]; drills=[]
            for t in r['tracks']:
                if t['layer'] not in LI:continue
                for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']):
                    center=_LS([a,b]) if a!=b else _PT(a)
                    copper.append((r['net'],LI[t['layer']],center.buffer(w/2/math.cos(math.pi/256),64)))
            for v in r['vias']:
                radius=float(v.get('d',VIA_D))/2;drill=float(v.get('drill',VIA_DR))/2
                point=_PT(v['x'],v['y']);n=v.get('net') or r['net']
                drills.append((point,drill,n,radius))
                geom=point.buffer(max(radius,drill+HOLE_CLR-DRCC)/math.cos(math.pi/256),64)
                for L in v.get('layers',LAYERS):
                    if L in LI:copper.append((n,LI[L],geom))
            _PZ_UNITS[u]=(copper,drills)
        copper,drills=_PZ_UNITS[u];holes.extend(drills)
        for n,li,g in copper:
            if n!=net:layers[li].append(g)
    trees = [STRtree(g) for g in layers]
    hole_tree=STRtree([pt.buffer(drill/math.cos(math.pi/256),64) for pt,drill,_,_ in holes])
    result = (layers, trees, holes,hole_tree)
    if len(cache)>=4:cache.pop(next(iter(cache)))
    cache[key]=result
    return result


def _pz_distance(tree, points):
    d = np.full(len(points),9.,dtype=np.float64)
    if len(tree.geometries):
        for offset in range(0,len(points),16384):
            _pz_check()
            ij, ds = tree.query_nearest(points[offset:offset+16384], return_distance=True, all_matches=False)
            d[offset+ij[0]] = ds
    return d


def _pz_hard_maps(net, rng):
    rng = tuple(rng) if LARGE and rng is not None else _WHOLE
    key = (net,rng,_PZ_VERSION[0])
    if key in _PZ_HARD: return _PZ_HARD[key]
    _PZ_BASE[0] = True
    try: base, viaH = hard_maps(net,rng)
    finally: _PZ_BASE[0] = False
    capH = _Maps(); capH.planar = np.concatenate([a.ravel().copy() for a in base])
    capH.extend([capH.planar[li*NN:(li+1)*NN].reshape(NY,NX) for li in range(NL)])
    viaH = viaH.copy()
    _, trees, holes, hole_tree = _pz_index(net)
    margin = PZ_BORDER_NM / 1e6
    for p in D['pads']:
        if id(p) not in PZ_LOCAL[net]: continue
        bx=p['bbox'];r=PZ_RADIUS_NM/1e6
        x0=max(rng[0],int((bx[0]-r-X0)/RES));x1=min(rng[1],int((bx[2]+r-X0)/RES)+1)
        y0=max(rng[2],int((bx[1]-r-Y0)/RES));y1=min(rng[3],int((bx[3]+r-Y0)/RES)+1)
        if x0>=x1 or y0>=y1:continue
        ys=slice(y0,y1);xs=slice(x0,x1)
        xx,yy=np.meshgrid(GXc[x0:x1],GYc[y0:y1]);pts=shapely.points(xx.ravel(),yy.ravel())
        via=(edge_d[ys,xs]-EDGE>=VIA_R+margin)&(VSP[ys,xs]>=H2H+VIA_DR/2+.01)
        if VIA_BAN is not None:via &= ~VIA_BAN[ys,xs]
        via &= _pz_distance(hole_tree,pts).reshape(xx.shape)>=VIA_DR/2+H2H+margin
        for li in range(NL):
            dp=np.where(PN1[li][ys,xs]==NID[net],PD2[li][ys,xs],PD1[li][ys,xs])
            fixed=_pz_distance(trees[li],pts).reshape(xx.shape)
            ch=np.minimum(2*(dp-DRCP-margin),2*(edge_d[ys,xs]-EDGE-margin))
            ch=np.minimum(ch,2*(fixed-DRCC-margin))
            capH[li][ys,xs]=_zp_hard(net,ch,li,ys,xs)
            via &= (dp>=max(VIA_R+DRCP,VIA_DR/2+HOLE_CLR)+margin)&(fixed>=VIA_R+DRCC+margin)
            if li in (0,NL-1) and not VIP:via &= padd[li][ys,xs]>=VIA_R+VIA_WEB
        if net in PLANE_MASK and net not in globals().get('PLANE_MIXED_NETS',()):via &= PLANE_MASK[net][ys,xs]
        via=_zp_viah(net,via,ys,xs)
        if PAD_HOLE_DIST is not None:via &= _pad_hole_free(PAD_HOLE_DIST[ys,xs])
        viaH[ys,xs]=via
    if VP:_vp_one(net,viaH,rng)
    plane_trace_maps(net,capH,rng)
    while len(_PZ_HARD)>=max(1,min(3,_HM_MAX)):_PZ_HARD.pop(next(iter(_PZ_HARD)))
    _PZ_HARD[key]=(capH,viaH)
    return capH,viaH


def _pz_room(net,li,a,b,fixed=True,without=None):
    if globals().get('KA') is not None: return _ka_room(net,li,a,b,fixed,without)
    seg=_LS([a,b]) if a!=b else _PT(a)
    _, trees, _, _ = _pz_index(net,fixed,without)
    best=9.
    tree=trees[li]
    if len(tree.geometries):
        j=tree.nearest(seg)
        best=min(best,seg.distance(tree.geometries[j])-DRCC-(PZ_BORDER_NM/1e6))

    board = globals().get('_edge_board')
    if board is not None:
        if not board.covers(seg): return -1.
        best=min(best,seg.distance(board.boundary)-float(args.get('edge','.30'))-(PZ_BORDER_NM/1e6))
    return best


def _ka_legacy_via_ok(net,x,y,fixed=True,without=None):
    pt=_PT(x,y); _,trees,holes,hole_tree=_pz_index(net,fixed,without)
    board=globals().get('_edge_board')
    if board is not None and (not board.covers(pt) or pt.distance(board.boundary)<VIA_R+float(args.get('edge','.30'))+(PZ_BORDER_NM/1e6)):return False
    if any(pt.distance(g)<VIA_R+(PZ_BORDER_NM/1e6) for g in _VIA_KEEP):return False
    for li in range(NL):
        for j in _PADT[li].query(pt.buffer(VIA_R+max(DRCP,HOLE_CLR)+.01)):
            n,g=_PADG[li][int(j)];d=pt.distance(g)
            if globals().get('KA') is None and n!=net and d<max(VIA_R+DRCP,VIA_DR/2+HOLE_CLR)+(PZ_BORDER_NM/1e6):return False
            if n==net and li in (0,NL-1) and not VIP and d<VIA_R+VIA_WEB:return False
        for j in ([] if globals().get('KA') is not None else trees[li].query(pt.buffer(max(VIA_R+DRCC,VIA_DR/2+HOLE_CLR)+.01))):
            if pt.distance(trees[li].geometries[j])<max(VIA_R+DRCC,VIA_DR/2+HOLE_CLR)+(PZ_BORDER_NM/1e6):return False
        for (n,L),g in _ZP_GEO.items():
            if globals().get('KA') is None and n!=net and L==LAYERS[li] and pt.distance(g)<VIA_R+CLR+.005:return False
    if any(pt.distance(g)<VIA_DR/2+H2H+(PZ_BORDER_NM/1e6) for g in PAD_HOLE_GEOMS):return False
    for j in hole_tree.query(pt.buffer(VIA_DR/2+H2H+(PZ_BORDER_NM/1e6))):
        p,drill,_,_=holes[int(j)]
        if pt.distance(p)<VIA_DR/2+drill+H2H+(PZ_BORDER_NM/1e6):return False
    for vs in D.get('via_keepout',[]):
        g=shapely.union_all([Polygon(q) for q in vs['polys']])
        own=any(p['ref']==vs['ref'] and p['net']==net and any(Polygon(q).covers(pt) for pp in p['polys'].values() for q in pp) for p in D['pads'])
        if not own and pt.distance(g)<VIA_DR/2+H2H+(PZ_BORDER_NM/1e6):return False
    if net in PLANE_MASK and net not in globals().get('PLANE_MIXED_NETS',()):
        ix=int((x-X0)/RES);iy=int((y-Y0)/RES)
        if not (0<=ix<NX and 0<=iy<NY and PLANE_MASK[net][iy,ix]):return False
        if PIN_ACCESS and not PLANE_AREA[net].covers(pt):return False
    return True


def _pz_via_ok(net,x,y,fixed=True,without=None):
    if globals().get('KA') is not None: return _ka_via_ok(net,x,y,fixed,without)
    return _ka_legacy_via_ok(net,x,y,fixed,without)


def _pz_path_points(path,net):
    pts=[cell_xy((n%NN)%NX,(n%NN)//NX) for n in path]
    for i,(a,b) in enumerate(zip(path[:-1],path[1:])):
        if a//NN!=b//NN:
            p=_vp_xy(net,a%NN) if VP else None
            if p:pts[i]=pts[i+1]=p
    return [_pz_snap(p) for p in pts]


def _pz_grid_geometry(path,net,w):
    points=_pz_path_points(path,net);tr=[];vi=[];run=[points[0]];li=path[0]//NN
    def finish():
        if len(run)>1:

            ps=[run[0]]
            for j,p in enumerate(run[1:]):
                if len(ps)>1 and abs((ps[-1][0]-ps[-2][0])*(p[1]-ps[-1][1])-(ps[-1][1]-ps[-2][1])*(p[0]-ps[-1][0]))<1e-12:ps[-1]=p
                else:ps.append(p)
            tr.append(dict(layer=LAYERS[li],pts=[list(p) for p in ps],w=[w]*(len(ps)-1)))
    for i,n in enumerate(path[1:],1):
        if n//NN!=li:
            finish();vi.append(dict(x=points[i][0],y=points[i][1]));li=n//NN;run=[points[i]]
        else:run.append(points[i])
    finish();return tr,vi


def _pz_edge_retry(res,src,tgt,bx,capH,capS,viaH,viaBad,C,win,extra):
    net=NET_CUR[0];w=PZ_WIDTHS[net]['w'];protected=set(src)|set(tgt)
    ch=_Maps();ch.planar=np.concatenate([a.ravel() for a in capH])
    ch.extend([ch.planar[li*NN:(li+1)*NN].reshape(NY,NX) for li in range(NL)])
    vh=viaH.copy()
    for trial in range(5):
        _pz_check()
        if not res:return None
        tr,vi=_pz_grid_geometry(res,net,w)
        if _pz_geometry_ok(net,tr,vi,not HARD_FOREIGN[0]):
            PZ_EDGE_STAT['taken']+=1
            return res,tr,vi
        if trial==4 or ('time_tight' in globals() and time_tight(.95)):break
        points=_pz_path_points(res,net);blocked=False
        for i,(a,b) in enumerate(zip(res[:-1],res[1:])):
            p,q=points[i],points[i+1]
            if a//NN!=b//NN:
                if not _pz_via_ok(net,*p,not HARD_FOREIGN[0]):vh.flat[a%NN]=False;blocked=True;break
            elif min(pad_room(a//NN,p,q,net),_pz_room(net,a//NN,p,q,not HARD_FOREIGN[0]))<w/2-1e-9:
                candidates=[n for n in (b,a) if n not in protected]
                if candidates:
                    n=min(candidates,key=lambda n:float(ch.planar[n]));ch.planar[n]=-1.;blocked=True;break
        if not blocked:

            for i,v in enumerate(vi):
                if any(math.dist((v['x'],v['y']),(q['x'],q['y']))<VIA_DR+H2H+(PZ_BORDER_NM/1e6) for q in vi[:i]):
                    c=int((v['y']-Y0)/RES)*NX+int((v['x']-X0)/RES);vh.flat[c]=False;blocked=True;break
        if not blocked:break
        PZ_EDGE_STAT['repetitions']+=1
        res=astar_pf(src,tgt,bx,ch,capS,vh,viaBad,dict(C,w=w,wmin=w),win,extra=extra)
    PZ_EDGE_STAT['rejected']+=1;return None


PZ_EDGE_STAT = dict(taken=0,repetitions=0,rejected=0)


def _pz_geometry_ok(net,tracks,vias,fixed=True,without=None):


    if globals().get('PIN_ACCESS', False) or globals().get('KA') is not None:
        for t in tracks:
            t['pts']=[_pz_snap(p) for p in t['pts']]
            t['w']=[_pz_mm(_pz_nm(w)) for w in t['w']]
        for v in vias:
            v['x'],v['y']=_pz_snap((v['x'],v['y']))
    minimum=(globals().get('PZ_WIDTHS',{}).get(net) or {}).get('dru_min')
    if minimum is not None and any(w<minimum-1e-9 for t in tracks for w in t['w']):return False
    for t in tracks:
        li=LI[t['layer']]
        for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']):
            if min(pad_room(li,a,b,net),_pz_room(net,li,a,b,fixed,without))<w/2-1e-9:return False
    for i,v in enumerate(vias):
        if any(.02<math.dist((v['x'],v['y']),(q['x'],q['y']))<VIA_DR+H2H+(PZ_BORDER_NM/1e6) for q in vias[:i]):return False
        if not _pz_via_ok(v.get('net') or net,v['x'],v['y'],fixed,without):return False
    return True


def hard_maps(net, rng=None):
    if globals().get('KA') is not None: return _ka_hard_maps(net,rng)
    if _pz_active(net) and not _PZ_BASE[0]: return _pz_hard_maps(net, rng)
    if net in _HM:
        r = _HM.pop(net); _HM[net] = r; return r
    nid = NID[net]
    fx0, fx1 = _fix_map(net)
    if LARGE and rng is not None and tuple(rng) != _WHOLE:
        ix0, ix1, iy0, iy1 = rng; ys = slice(iy0, iy1); xs = slice(ix0, ix1)
        capH = _Maps(); capH.planar = _B_CAPH
        v = (edge_d[ys, xs] - EDGE >= VIA_R) & (VSP[ys, xs] >= H2H + VIA_DR / 2 + 0.01)
        if VIA_BAN is not None: v &= ~VIA_BAN[ys, xs]
        if fx1 is not None: v &= fx1[ys, xs] - QM >= HOLE_C
        for li in range(NL):
            dp = np.where(PN1[li][ys, xs] == nid, PD2[li][ys, xs], PD1[li][ys, xs])
            ch = np.minimum(2 * (dp - CLR_PAD), 2 * (edge_d[ys, xs] - EDGE))
            if fx0 is not None: ch = np.minimum(ch, 2 * (fx0[li][ys, xs] - QM - CLR - 0.005))
            ch = _zp_hard(net,ch,li,ys,xs)
            ch = ch.astype(np.float32)
            layer_name = _B_CAPH[li * NN:(li + 1) * NN].reshape(NY, NX); layer_name[ys, xs] = ch; capH.append(layer_name)
            v &= ch / 2 >= VIA_KEEP_R
            if li in (0, NL - 1) and not VIP: v &= padd[li][ys, xs] >= VIA_R + VIA_WEB
        viaH = _B_VIAH.reshape(NY, NX); viaH[ys, xs] = v
        if VP: _vp_one(net, viaH, rng)
        if EPZ: _ep_one(net, viaH, capH, rng)
        if GS_N: _lanes_one(net, capH, rng)
        if net in PLANE_MASK and net not in globals().get('PLANE_MIXED_NETS',()): viaH[ys,xs] &= PLANE_MASK[net][ys,xs]
        if ZONE_PLAN: viaH[ys,xs] = _zp_viah(net,viaH[ys,xs],ys,xs)
        if PAD_HOLE_DIST is not None: viaH[ys,xs] &= _pad_hole_free(PAD_HOLE_DIST[ys,xs])
        plane_trace_maps(net,capH,rng)
        return capH, viaH
    capH = _Maps(); capH.planar = np.empty(NL * NN, dtype=np.float32)
    for li in range(NL):
        dp = np.where(PN1[li] == nid, PD2[li], PD1[li])
        ch = np.minimum(2 * (dp - CLR_PAD), 2 * (edge_d - EDGE))
        if fx0 is not None: ch = np.minimum(ch, 2 * (fx0[li] - QM - CLR - 0.005))
        ch = _zp_hard(net,ch,li)
        capH.planar[li * NN:(li + 1) * NN] = ch.astype(np.float32).ravel()
        capH.append(capH.planar[li * NN:(li + 1) * NN].reshape(NY, NX))
    viaH = (edge_d - EDGE >= VIA_R) & (VSP >= H2H + VIA_DR / 2 + 0.01)
    if VIA_BAN is not None: viaH &= ~VIA_BAN
    if fx1 is not None: viaH &= fx1 - QM >= HOLE_C
    for li in range(NL):
        viaH &= capH[li] / 2 >= VIA_KEEP_R
        if li in (0, NL - 1) and not VIP: viaH &= padd[li] >= VIA_R + VIA_WEB
    if VP: _vp_one(net, viaH, None)
    if EPZ: _ep_one(net, viaH, capH, None)
    if GS_N: _lanes_one(net, capH, None)
    if net in PLANE_MASK and net not in globals().get('PLANE_MIXED_NETS',()): viaH &= PLANE_MASK[net]
    viaH = _zp_viah(net,viaH)
    if PAD_HOLE_DIST is not None: viaH &= _pad_hole_free(PAD_HOLE_DIST)
    plane_trace_maps(net,capH)
    while len(_HM) >= _HM_MAX: del _HM[next(iter(_HM))]
    _HM[net] = (capH, viaH)
    return capH, viaH


def _pz_soft_maps(net,rng):
    x0,x1,y0,y1=rng;ys=slice(y0,y1);xs=slice(x0,x1)
    xx,yy=np.meshgrid(GXc[x0:x1],GYc[y0:y1]);pts=shapely.points(xx.ravel(),yy.ravel())
    _,trees,holes,hole_tree=_pz_index(net,False)
    cap=[]
    for tree in trees:
        a=np.full((NY,NX),-1.,dtype=np.float32)
        a[ys,xs]=2*(_pz_distance(tree,pts).reshape(xx.shape)-DRCC-1e-6)
        cap.append(a)
    dd=np.full((NY,NX),9.,dtype=np.float32)

    dd[ys,xs]=_pz_distance(hole_tree,pts).reshape(xx.shape)+VIA_DR/2+.02-1e-6
    return cap,dd


def maps(net, box, wmax):
    capS, dd, rng = soft_maps(net, box, wmax)
    if LARGE and box is not None and tuple(rng) == _WHOLE: rng = _box_idx(box)
    capH, viaH = hard_maps(net, rng)
    if globals().get('KA') is None and _pz_active(net) and (FIXD[0] is not None or HARD_FOREIGN[0]):
        if all(u.startswith('FIX#') or r['net']==net for u,r in U.items()):
            capS=capH


            dd=np.full((NY,NX),9.,dtype=np.float32)
        else:capS,dd=_pz_soft_maps(net,rng)
    capH, viaH = current_reserve(net, capH, viaH)
    if not LARGE or tuple(rng) == _WHOLE:
        viaBad = (dd < HOLE_C)
        for li in range(NL): viaBad |= capS[li] / 2 < VIA_R
        capC = [np.minimum(capH[i], capS[i]) for i in range(NL)]
        capCf = np.concatenate([c.ravel() for c in capC])
        if HARD_FOREIGN[0]:
            capH = _Maps(); capH.planar = capCf
            capH.extend([capCf[i * NN:(i + 1) * NN].reshape(NY, NX) for i in range(NL)])
            viaH = viaH & ~viaBad
        return capH, viaH, capS, dd, rng, viaBad, capC, capCf, viaBad.ravel()
    ix0, ix1, iy0, iy1 = rng; ys = slice(iy0, iy1); xs = slice(ix0, ix1)
    v = dd[ys, xs] < HOLE_C
    capC = _Maps(); capC.planar = _B_CAPC
    for li in range(NL):
        v |= capS[li][ys, xs] / 2 < VIA_R
        layer_name = _B_CAPC[li * NN:(li + 1) * NN].reshape(NY, NX); layer_name[ys, xs] = np.minimum(capH[li][ys, xs], capS[li][ys, xs]); capC.append(layer_name)
    viaBad = _B_VBAD.reshape(NY, NX); viaBad[ys, xs] = v
    return (capC if HARD_FOREIGN[0] else capH), (viaH & ~viaBad if HARD_FOREIGN[0] else viaH), capS, dd, rng, viaBad, capC, _B_CAPC, _B_VBAD


def maps_larger(net, box, win, wmax):
    if not LARGE: return maps(net, None, None), None
    b2 = (min(box[0], win[0]) - 1.0, min(box[1], win[1]) - 1.0, max(box[2], win[2]) + 1.0, max(box[3], win[3]) + 1.0)
    return maps(net, b2, wmax), b2


_ERR = {}
_ERR_MAX = max(2, int(float(args.get('err_mb', '600')) * 1e6 / (NL * NN * 4)))
UNREACHABLE = {}
_ERR_NETS = set()


def _components(net, wmin):
    key = (net, wmin)
    if key in _ERR:
        K = _ERR.pop(key); _ERR[key] = K; return K
    capH, viaH = hard_maps(net)
    from scipy.ndimage import label as _label
    eight = np.ones((3, 3), dtype=bool)
    K = np.zeros(NL * NN, dtype=np.int32); n_ges = 0
    for li in range(NL):
        lab, n = _label((capH[li] >= np.float64(wmin)) | viaH, structure=eight)
        lab = lab.astype(np.int32); lab[lab > 0] += n_ges; n_ges += n
        K[li * NN:(li + 1) * NN] = lab.ravel()
    parent = list(range(n_ges + 1))
    def root(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]; a = parent[a]
        return a
    vidx = np.flatnonzero(viaH.ravel())
    if len(vidx):
        k0 = K[vidx]
        for li in range(1, NL):
            nc = K[li * NN + vidx]
            new_ = np.ones(len(vidx), dtype=bool); new_[1:] = (k0[1:] != k0[:-1]) | (nc[1:] != nc[:-1])
            for a, b in np.unique(np.stack([k0[new_], nc[new_]], axis=1), axis=0).tolist():
                ra, rb = root(a), root(b)
                if ra != rb: parent[max(ra, rb)] = min(ra, rb)
    K = np.array([root(i) for i in range(n_ges + 1)], dtype=np.int32)[K]
    while len(_ERR) >= _ERR_MAX: del _ERR[next(iter(_ERR))]
    _ERR[key] = K
    return K


def _unreachable(K, src, tgt):
    def surroundings(a):
        li = a // NN; idx = a - li * NN; iy = idx // NX; ix = idx - iy * NX
        out = [a]
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0: continue
                jy = iy + dy; jx = ix + dx; m = (jy >= 0) & (jy < NY) & (jx >= 0) & (jx < NX)
                out.append((li * NN + jy * NX + jx)[m])
        return np.concatenate(out)
    s = surroundings(np.asarray(src, dtype=np.int64)); t = np.fromiter(tgt, dtype=np.int64, count=len(tgt))
    if np.intersect1d(s, t).size: return False
    ks = np.unique(K[s]); kt = np.unique(K[surroundings(t)])
    return not np.intersect1d(ks[ks > 0], kt[kt > 0]).size


def neck_active(net, C, wmin, stub=False):
    if wmin <= max(NECK_W, DRU_MIN.get(net, 0.)) or net not in by_net_g: return False
    fld = NECK_FIELD and any(net in f[5] for f in DENSE)
    if C.get('prio', 8) < 8: return NECK > 0 or fld
    return (stub or BGA_TIGHT) and fld


def neck_field_map(net, C, wmin, stub=False):
    if not neck_active(net, C, wmin, stub): return None
    if (net, wmin) in NECK_CACHE:
        WMINF = NECK_CACHE.pop((net, wmin)); NECK_CACHE[(net, wmin)] = WMINF; return WMINF

    while len(NECK_CACHE) >= _NECK_MAX: del NECK_CACHE[next(iter(NECK_CACHE))]
    WMINF = np.full(NL * NN, wmin, dtype=np.float32); NECK_CACHE[(net, wmin)] = WMINF
    hw = max(NECK_W, DRU_MIN.get(net, 0.))
    if NECK > 0 and C.get('prio', 8) < 8:
        m = np.zeros(NL * NN, dtype=bool)
        for k in by_net_g[net]:
            for L, cells in D['pads'][k]['cells'].items(): m[LI[L] * NN + cells] = True
        for li in range(NL):
            sl = slice(li * NN, (li + 1) * NN)
            if m[sl].any():
                near = _edt(~m[sl].reshape(NY, NX)) * RES <= NECK
                WMINF[sl][near.ravel()] = hw
    if NECK_FIELD:
        for lg, ix0, ix1, iy0, iy1, net_list in DENSE:
            if net not in net_list: continue
            for li in lg: WMINF[li * NN:(li + 1) * NN].reshape(NY, NX)[iy0:iy1, ix0:ix1] = hw
    return WMINF



def _bend_h(n, nn, nx, x0, y0, res, tx0, ty0, tx1, ty1, bend=0.):


    idx = n % nn
    x = x0 + (idx % nx + .5) * res; y = y0 + (idx // nx + .5) * res
    dx = max(tx0-x, 0., x-tx1); dy = max(ty0-y, 0., y-ty1)
    return max(dx, dy) + (1.4142 - 1.) * min(dx, dy)

def bend_core(src, tgt, tbox, capHf, mult, viaf, vbad, hist, wmin, VC, PFv, via_target, win,
               NX, NN, NL, X0, Y0, RES, g, par, gst, cst, tst, run, ofx, ofy, ofs, wminf, hfac, info, viaz, vzk,
               bend, via_extra, vpr, via_anchor, vtbox):
    ixa, ixb, iya, iyb = win[0], win[1], win[2], win[3]
    tx0, ty0, tx1, ty1 = tbox[0], tbox[1], tbox[2], tbox[3]
    hfac = min(1.0, hfac) * (1.4142 / math.sqrt(2.0))
    for t in tgt: tst[t] = run
    cap_k = max(65536, 2 * src.shape[0] + 64)
    hf = np.empty(cap_k, dtype=np.float64); hn = np.empty(cap_k, dtype=np.int64); hs = 0
    for s0 in src:
        s = s0 * 9 + 8
        gst[s] = run; g[s] = 0.0; par[s] = -1
        f0 = hfac * _bend_h(s // 9, NN, NX, X0, Y0, RES, tx0, ty0, tx1, ty1, bend)
        hs = _hpush(hf, hn, hs, f0, np.int64(s))
    while hs > 0:
        n = hn[0]
        hs = _hpop(hf, hn, hs)
        if hs + NL + 12 > cap_k:
            cap_k *= 2
            hf2 = np.empty(cap_k, dtype=np.float64); hn2 = np.empty(cap_k, dtype=np.int64)
            hf2[:hs] = hf[:hs]; hn2[:hs] = hn[:hs]; hf = hf2; hn = hn2
        if n < 0:
            n = -n - 1
            L = 0; q = n
            while q != -1: L += 1; q = par[q]
            out = np.empty(L, dtype=np.int64); q = n; k = L - 1
            while q != -1: out[k] = q // 9; k -= 1; q = par[q]
            return out
        if cst[n] == run: continue
        if info.shape[0] > 1: info[1] += 1
        node = n // 9; direction = n % 9


        dominated = False
        for d in range(9):
            other = node * 9 + d
            if other != n and gst[other] == run and (g[other] + (0.0 if d == 8 else bend) < g[n]):
                dominated = True; break
        if dominated:
            if info.shape[0] > 2: info[2] += 1
            continue
        cst[n] = run
        idx = node % NN
        li = node // NN
        if via_target:
            if tst[node] == run:
                L = 0; q = n
                while q != -1: L += 1; q = par[q]
                out = np.empty(L, dtype=np.int64); q = n; k = L - 1
                while q != -1: out[k] = q // 9; k -= 1; q = par[q]
                return out
            if viaf[idx] and (li == 0 or li == NL - 1):
                idx0 = node % NN
                xx = X0 + (idx0 % NX + 0.5) * RES; yy = Y0 + (idx0 // NX + 0.5) * RES
                dx0 = max(vtbox[0]-xx, 0.0, xx-vtbox[2]); dy0 = max(vtbox[1]-yy, 0.0, yy-vtbox[3])
                hv = math.sqrt(dx0*dx0+dy0*dy0)

                hs = _hpush(hf, hn, hs, g[n] + 0.3 + 2.0 * hv + (3.0 * PFv if vbad[idx] else 0.0)
                            + (vzk * np.float64(viaz[idx]) if viaz.shape[0] > 0 else 0.0), np.int64(-n - 1))
        elif tst[node] == run:
            L = 0; q = n
            while q != -1: L += 1; q = par[q]
            out = np.empty(L, dtype=np.int64); q = n; k = L - 1
            while q != -1: out[k] = q // 9; k -= 1; q = par[q]
            return out
        iy = idx // NX; ix = idx - iy * NX
        gn = g[n]

        for k in range(8):
            jx = ix + ofx[k]; jy = iy + ofy[k]
            if jx < ixa or jx > ixb or jy < iya or jy > iyb:
                info[0] += 1
                continue
            nextnode = li * NN + jy * NX + jx
            m = nextnode * 9 + k
            wm = wmin if wminf.shape[0] == 0 else wminf[nextnode]
            if capHf[nextnode] < wm and tst[nextnode] != run: continue
            ng = gn + ofs[k] * np.float64(mult[nextnode])
            if direction != 8 and direction != k: ng += bend
            if gst[m] != run or ng < g[m]:
                gst[m] = run; g[m] = ng; par[m] = n; cst[m] = 0
                hm = hfac * _bend_h(nextnode, NN, NX, X0, Y0, RES, tx0, ty0, tx1, ty1, bend)
                hs = _hpush(hf, hn, hs, ng + hm, np.int64(m))
        if viaf[idx] and not via_target:
            vc = VC * (1.0 + 3.0 * PFv * vbad[idx]) + 0.2 * np.float64(hist[node])
            if via_anchor.shape[0] > 0: vc = vc + np.float64(via_anchor[idx])
            for lj in range(NL):
                if lj == li: continue
                nextnode = lj * NN + idx
                m = nextnode * 9 + 8
                vcl = vc
                if via_extra.shape[0] > 0: vcl = vc + vpr[li * NL + lj] * np.float64(via_extra[idx])
                ng = gn + vcl
                if gst[m] != run or ng < g[m]:
                    gst[m] = run; g[m] = ng; par[m] = n; cst[m] = 0
                    hs = _hpush(hf, hn, hs, ng + hfac * _bend_h(nextnode, NN, NX, X0, Y0, RES, tx0, ty0, tx1, ty1, bend), np.int64(m))
    return np.empty(0, dtype=np.int64)



_BEND_SEARCH = None
_BEND_CPU = 0.; _BEND_CALLS = 0; _BEND_POP = 0; _BEND_DOM = 0
if NB is not None:
    from router.astar_nb import _hpush, _hpop
    import numba as _bend_numba
    _bend_h = _bend_numba.njit(cache=True, nogil=True)(_bend_h)
    bend_core = _bend_numba.njit(cache=True, nogil=True)(bend_core)

def bend_search(src, tgt, tbox, cap, mult, via, vbad, hist, wmin, vc, pf, via_target,
                 win, nx, nn, nl, x0, y0, res, wminf, hfac=1., viaz=None, vzk=0.,
                 bend=0., via_extra=None, vpr=None, via_anchor=None):
    global _BEND_SEARCH, _BEND_CPU, _BEND_CALLS, _BEND_POP, _BEND_DOM
    if _BEND_SEARCH is None:
        _BEND_SEARCH = _Search(nl, nn, res, direction=True)
        _BEND_SEARCH.info = np.zeros(3, np.int64)
    search = _BEND_SEARCH
    xa, xb, ya, yb = win
    vtbox = tuple(tbox)
    hfac = min(float(hfac), max(0., float(mult.reshape(nl, nn//nx, nx)[:,ya:yb+1,xa:xb+1].min())))
    if tgt:
        points = [(x0+(t % nn % nx+.5)*res, y0+(t % nn // nx+.5)*res) for t in tgt]
        tbox = (min(tbox[0], min(x for x,y in points)), min(tbox[1], min(y for x,y in points)),
                max(tbox[2], max(x for x,y in points)), max(tbox[3], max(y for x,y in points)))
    search.run += 1; search.info[:] = 0
    if search.run >= 2**31-2:
        search.gst[:] = 0; search.cst[:] = 0; search.tst[:] = 0; search.run = 1
    from router.astar_nb import OFX, OFY
    start = time.process_time()
    path = bend_core(np.asarray(src, np.int64), np.asarray(sorted(tgt), np.int64), np.asarray(tbox, np.float64),
        cap, mult, via, vbad, hist, float(wmin), float(vc), float(pf), bool(via_target), np.asarray(win,np.int64),
        nx,nn,nl,float(x0),float(y0),float(res),search.g,search.par,search.gst,search.cst,search.tst,np.int32(search.run),
        OFX,OFY,search.ofs,np.asarray(wminf,np.float32),float(hfac),search.info,
        _EMPTY if viaz is None else viaz,float(vzk),float(bend),_EMPTY if via_extra is None else via_extra,
        np.zeros(1,np.float64) if vpr is None else vpr,_EMPTY if via_anchor is None else via_anchor,np.asarray(vtbox,np.float64))
    _BEND_CPU += time.process_time()-start; _BEND_CALLS += 1
    _BEND_POP += int(search.info[1]); _BEND_DOM += int(search.info[2])
    NB.info[0] = search.info[0]
    return path.tolist() if len(path) else None

def astar_pf(src, tgt_set, tgt_box, capH, capS, viaH, viaBad, C, window, via_target=False, extra=None, viaz=None, vzk=0.0):
    work_step('astar')
    if globals().get('_LOOP_DEADLINE') is not None: _loop_cpu_check()
    x0, y0, x1, y1 = window
    ixa, ixb = max(1, int((x0 - X0) / RES)), min(NX - 2, int((x1 - X0) / RES))
    iya, iyb = max(1, int((y0 - Y0) / RES)), min(NY - 2, int((y1 - Y0) / RES))
    w, wmin = C['w'], C['wmin']

    category = (NC.get('net_list', {}).get(NET_CUR[0], {}).get('category') if NC is not None else None)
    turn = BEND * (BEND_CURRENT if category in ('current_val','supply','sw','gnd') else 1.)
    WMINF = neck_field_map(NET_CUR[0], C, wmin, via_target)
    if C.get('current_w'): WMINF = current_floor(NET_CUR[0], C)
    if C.get('zw'):
        WMINF = np.repeat(np.asarray(C['zw'], dtype=np.float32), NN)
    capHf = capH.planar if getattr(capH, 'planar', None) is not None else np.concatenate([c.ravel() for c in capH])
    if HARD_FOREIGN[0] or (CURRENT_FIRST and not current_model(NET_CUR[0])):


        def vacant(n): return capHf[n] >= (wmin if WMINF is None else WMINF[n])
        src = [n for n in src if vacant(n)]
        tgt_set = {n for n in tgt_set if vacant(n)}
        if not src or (not via_target and not tgt_set): return None
    elif SOURCE_FREE[0]:
        def vacant(n):
            wm = wmin if WMINF is None else WMINF[n]
            return min(capHf[n], capS[n // NN].ravel()[n % NN]) >= wm
        free_src = [n for n in src if vacant(n)]
        free_tgt = {n for n in tgt_set if vacant(n)}

        if free_src: src = free_src
        if free_tgt: tgt_set = free_tgt
    hw = C.get('hist_weight', 1.0)


    mult = _MULT
    _key = (id(capH), id(capS), id(extra), w, wmin, C['inner'], C['pen'], hw, PF[0], C.get('lf'), C.get('zw'), C.get('current_w'), C.get('current_clearance'))
    _c = ASTAR_COST[0]
    if not (_c is not None and _c[0] == _key and _c[1][0] <= iya and iyb <= _c[1][1] and _c[1][2] <= ixa and ixb <= _c[1][3]):
        fy0, fy1, fx0, fx1 = iya, iyb, ixa, ixb
        for _f in (ASTAR_HINT[0], _c[1] if _c is not None and _c[0] == _key else None):
            if _f is not None: fy0, fy1, fx0, fx1 = min(fy0, _f[0]), max(fy1, _f[1]), min(fx0, _f[2]), max(fx1, _f[3])
        lf = C.get('lf') or ([1.0] + [C['inner']] * (NL - 2) + [1.0])
        ys = slice(fy0, fy1 + 1); xs = slice(fx0, fx1 + 1)
        for li in range(NL):
            if C.get('zw'): w = wmin = C['zw'][li]
            if C.get('current_w'): w = wmin = C['current_w'][li]
            sl = slice(li * NN, (li + 1) * NN)
            c = np.minimum(capH[li][ys, xs], capS[li][ys, xs])
            m = (1.0 + C['pen'] * np.clip((w - c) / w, 0, 1)) * lf[li]


            _tight = float(args.get('cost_tight', '0')) if NC is not None else 0.
            if _tight:
                m *= 1. + _tight * np.clip(1. - c / (2*w + .2), 0., 1.)
            if C.get('zw') and IMP_REF_MASK:
                _zp = IMP_TAB.get(NET_CUR[0], {}).get(LAYERS[li])
                if _zp:
                    _mask = IMP_REF_MASK[NET_CUR[0], LAYERS[li]][ys, xs]
                    m = np.where(_mask, m, m * 8.0)
            if FL_PRICE > 0 and w <= FL_WMAX:
                e = 0.5 * c + CLR - 0.5 * w
                m = np.where((e > CLR + RES) & (e < FL_GAP), m * (1.0 + FL_PRICE * (e - CLR)), m)
            elif AREA > 0 and w <= FL_WMAX:
                e = 0.5 * c + CLR - 0.5 * w
                m = np.where((e > CLR + RES) & (e < FL_GAP), m * (1.0 + AREA), m)
            if li == 0: m = np.where(under_esp[ys, xs], m * 1.8, m)
            m = (m * (1.0 + PF[0] * (c < wmin))).astype(np.float32)
            m *= (1.0 + hw * hist[sl].reshape(NY, NX)[ys, xs])
            if C.get('current_clearance') and CURRENT_DETOUR:
                ax,ay,bx,by = C['current_clearance']; gx,gy = np.meshgrid(GXc[xs],GYc[ys])
                clearance = math.hypot(bx-ax,by-ay)
                more = np.maximum(0.,np.hypot(gx-ax,gy-ay)+np.hypot(gx-bx,gy-by)-clearance)
                m *= (1. + CURRENT_DETOUR * more / max(1.,clearance)).astype(np.float32)
            if extra is not None: m *= extra[sl].reshape(NY, NX)[ys, xs]
            mult[sl].reshape(NY, NX)[ys, xs] = m

        ASTAR_COST[0] = (_key, (fy0, fy1, fx0, fx1), capH, capS, extra)
    viaf = viaH.ravel(); vbad = viaBad.ravel()
    if C.get('current_w'):
        viaf = current_field_mask(viaH, max(2, math.ceil(4*C['current_I'])), C, capH).ravel()
    if VIA_MASK[0] is not None: viaf = viaf & VIA_MASK[0]
    VC = C['via'] * VIA_FACTOR[0]
    _vz = VIA_TOO[0] if not via_target else None
    via_extra, vpr, via_anchor = _vz if _vz is not None else (None, None, None)

    bounded_loop = globals().get('_LOOP_DEADLINE') is not None and not BUDGET_WORK
    if NB is not None and not bounded_loop and turn > 0:
        return bend_search(src, tgt_set, tgt_box, capHf, mult, viaf, vbad, hist, wmin, VC, PF[0], via_target,
            (ixa,ixb,iya,iyb),NX,NN,NL,X0,Y0,RES,_EMPTY if WMINF is None else WMINF,HFAC,viaz,vzk,turn,via_extra,vpr,via_anchor)
    if NB is not None and not bounded_loop:
        return NB(src, tgt_set, tgt_box, capHf, mult, viaf, vbad, hist, wmin, VC, PF[0], via_target,
                  (ixa, ixb, iya, iyb), NX, NN, NL, X0, Y0, RES, _EMPTY if WMINF is None else WMINF, HFAC, viaz, vzk,
                  turn, via_extra, vpr, via_anchor)
    if turn > 0:

        return astar_direction_python(src, tgt_set, tgt_box, capHf, mult, viaf, vbad, hist, wmin, VC, PF[0], via_target,
                                (ixa, ixb, iya, iyb), NX, NN, NL, X0, Y0, RES, _EMPTY if WMINF is None else WMINF,
                                HFAC, viaz, vzk, turn, via_extra, vpr, via_anchor)
    tx0, ty0, tx1, ty1 = tgt_box
    def h(n):
        idx = n % NN; ix = idx % NX; iy = idx // NX
        x, y = X0 + (ix + 0.5) * RES, Y0 + (iy + 0.5) * RES
        dx = max(tx0 - x, 0, x - tx1); dy = max(ty0 - y, 0, y - ty1)
        return math.sqrt(dx * dx + dy * dy) if turn > 0.0 else math.hypot(dx, dy)
    g = {}; parent = {}; openh = []
    for n in src:
        g[n] = 0.0; parent[n] = -1; heapq.heappush(openh, (0.0 if via_target else HFAC * h(n), n))
    closed = set()
    loop_poll = 0
    while openh:
        if bounded_loop:
            loop_poll += 1
            if loop_poll % 128 == 0: _loop_cpu_check()
        f, n = heapq.heappop(openh)
        if n < 0:
            n = -n - 1; path = []
            while n != -1: path.append(n); n = parent[n]
            return path[::-1]
        if n in closed: continue
        closed.add(n)
        idx = n % NN
        if via_target:
            if n in tgt_set:
                path = []
                while n != -1: path.append(n); n = parent[n]
                return path[::-1]
            if viaf[idx] and n // NN in (0, NL - 1):
                heapq.heappush(openh, (g[n] + 0.3 + 2.0 * h(n) + (3.0 * PF[0] if vbad[idx] else 0.0)
                                       + (vzk * float(viaz[idx]) if viaz is not None else 0.0), -n - 1))
        elif n in tgt_set:
            path = []
            while n != -1: path.append(n); n = parent[n]
            return path[::-1]
        li = n // NN; iy, ix = divmod(idx, NX)
        gn = g[n]
        pdx = 0; pdy = 0; has_direction = False
        if turn > 0.0 and parent[n] >= 0 and parent[n] // NN == li:
            pq = parent[n] % NN
            pdx = ix - pq % NX; pdy = iy - pq // NX; has_direction = True
        for dx, dy, st in OFFS:
            jx, jy = ix + dx, iy + dy
            if jx < ixa or jx > ixb or jy < iya or jy > iyb: continue
            m = li * NN + jy * NX + jx
            if capHf[m] < (wmin if WMINF is None else WMINF[m]) and m not in tgt_set: continue
            ng = gn + st * float(mult[m])
            if has_direction and (dx != pdx or dy != pdy): ng += turn
            if ng < g.get(m, 1e18):
                g[m] = ng; parent[m] = n
                heapq.heappush(openh, (ng + (0.0 if via_target else HFAC * h(m)), m))
        if viaf[idx] and not via_target:
            vc = VC * (1.0 + 3.0 * PF[0] * vbad[idx]) + 0.2 * float(hist[n])
            if via_anchor is not None: vc = vc + float(via_anchor[idx])
            for lj in range(NL):
                if lj == li: continue
                m = lj * NN + idx
                vcl = vc
                if via_extra is not None: vcl = vc + float(vpr[li * NL + lj]) * float(via_extra[idx])
                ng = gn + vcl
                if ng < g.get(m, 1e18):
                    g[m] = ng; parent[m] = n
                    heapq.heappush(openh, (ng + HFAC * h(m), m))
    return None


def astar_direction_python(src, tgt, tbox, cap, mult, via, vbad, hist, wmin, vc, pf, via_target,
                          win, nx, nn, nl, x0, y0, res, wminf, hfac=1., viaz=None, vzk=0.,
                          bend=0., via_extra=None, vpr=None, via_anchor=None):
    ny = nn // nx
    xa, xb, ya, yb = win
    tx0, ty0, tx1, ty1 = tbox
    vx0, vy0, vx1, vy1 = tbox
    if tgt:
        points = [(x0+(t % nn % nx+.5)*res, y0+(t % nn // nx+.5)*res) for t in tgt]
        tx0 = min(tx0, min(x for x, y in points)); tx1 = max(tx1, max(x for x, y in points))
        ty0 = min(ty0, min(y for x, y in points)); ty1 = max(ty1, max(y for x, y in points))
    hfac = min(1., float(hfac), max(0., float(mult.reshape(nl, ny, nx)[:, ya:yb+1, xa:xb+1].min()))) * (1.4142 / math.sqrt(2.))
    def h(node):
        idx = node % nn
        x, y = x0+(idx % nx+.5)*res, y0+(idx // nx+.5)*res
        dx, dy = max(tx0-x, 0., x-tx1), max(ty0-y, 0., y-ty1)
        return max(dx,dy)+(1.4142-1.)*min(dx,dy)
    directions = ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1))
    g, parent, closed, queue = {}, {}, set(), []
    for node in src:
        s = node*9+8
        g[s] = 0.; parent[s] = -1
        heapq.heappush(queue, (hfac*h(node), s))
    def relax(s, cost, previous, heuristic):
        if cost < g.get(s, float('inf')):
            g[s] = cost; parent[s] = previous; closed.discard(s)
            heapq.heappush(queue, (cost+heuristic, s))
    def path(s):
        result = []
        while s != -1:
            result.append(s//9); s = parent[s]
        return result[::-1]
    bounded_loop = globals().get('_LOOP_DEADLINE') is not None
    loop_poll = 0
    while queue:
        if bounded_loop:
            loop_poll += 1
            if loop_poll % 128 == 0: _loop_cpu_check()
        _, s = heapq.heappop(queue)
        if s < 0: return path(-s-1)
        if s in closed: continue
        node, direction = divmod(s, 9)
        if any(other != s and g.get(other, float('inf')) + (0. if d == 8 else bend) < g[s]
               for d in range(9) for other in (node*9+d,)): continue
        closed.add(s)
        layer, idx = divmod(node, nn)
        if node in tgt: return path(s)
        if via_target and via[idx] and layer in (0, nl-1):
            extra = vzk*float(viaz[idx]) if viaz is not None and len(viaz) else 0.
            heapq.heappush(queue, (g[s]+.3+2.*math.sqrt(max(vx0-(x0+(idx % nx+.5)*res),0.,(x0+(idx % nx+.5)*res)-vx1)**2+
                max(vy0-(y0+(idx // nx+.5)*res),0.,(y0+(idx // nx+.5)*res)-vy1)**2)+(3.*pf if vbad[idx] else 0.)+extra, -s-1))
        y, x = divmod(idx, nx)
        for k, (dx, dy) in enumerate(directions):
            xx, yy = x+dx, y+dy
            if not (xa <= xx <= xb and ya <= yy <= yb): continue
            target = layer*nn+yy*nx+xx
            wm = wmin if not len(wminf) else wminf[target]
            if cap[target] < wm and target not in tgt: continue
            cost = g[s]+(res*1.4142 if k >= 4 else res)*float(mult[target])
            if direction != 8 and direction != k: cost += bend
            relax(target*9+k, cost, s, hfac*h(target))
        if via[idx] and not via_target:
            extra = float(via_anchor[idx]) if via_anchor is not None and len(via_anchor) else 0.
            cost = vc*(1.+3.*pf*vbad[idx])+.2*float(hist[node])+extra
            for ll in range(nl):
                if ll == layer: continue
                target = ll*nn+idx
                change = cost
                if via_extra is not None and len(via_extra): change += vpr[layer*nl+ll]*float(via_extra[idx])
                relax(target*9+8, g[s]+change, s, hfac*h(target))
    return None



_PADG = {li: [] for li in range(NL)}
_KA_PAD_KEEP = {li: [] for li in range(NL)}
_PADXY = {li: [] for li in range(NL)}
for _p in D['pads']:
    for _L in _p['layers']:
        if _L in LI:
            _PADXY[LI[_L]].append((_p['x'], _p['y']) if not _p.get('drill') else None)
            _PADG[LI[_L]].append((_p['net'] if _p['net'] and not _p['net'].startswith('unconnected') else '#%d' % id(_p),
                                  shapely.union_all([Polygon(q) for q in _p['polys'][_L]])))
            if globals().get('KA') is not None:
                _KA_META[id(_PADG[LI[_L]][-1][1])] = _ka_pad_kind(_p)
                for _g,_kind in _ka_pad_keeps(_p,_L):
                    _KA_META[id(_g)]=(_kind,'')
                    _KA_PAD_KEEP[LI[_L]].append((_PADG[LI[_L]][-1][0],_g,_kind))
_PADT = {li: shapely.STRtree([g for _, g in _PADG[li]]) for li in range(NL)}





VPL = args.get('via_slots', '1' if NC else '0') == '1'
VP = {}
if VPL:
    from scipy.spatial import cKDTree as _KD2
    _jr = {}
    for _p in D['pads']:
        if not _p.get('drill') and any(L in LI for L in _p['layers']): _jr.setdefault(_p['ref'], []).append(_p)
    _rn = VIA_KEEP_R + max(DRCP, 0.03) + 0.03
    for _r in sorted(_jr):
        _P = _jr[_r]
        if len(_P) < 16 or any(max(q['bbox'][2] - q['bbox'][0], q['bbox'][3] - q['bbox'][1]) > 1.3 * min(q['bbox'][2] - q['bbox'][0], q['bbox'][3] - q['bbox'][1]) for q in _P): continue
        _xy = np.array([(q['x'], q['y']) for q in _P]); _d, _i = _KD2(_xy).query(_xy, k=2); _pit = float(np.median(_d[:, 1]))
        if not 0.3 <= _pit <= 1.05: continue
        _v = _xy[_i[:, 1]] - _xy; _w = np.arctan2(_v[:, 1], _v[:, 0])
        _th = float(np.angle(np.mean(np.exp(4j * _w)))) / 4
        _e1 = (math.cos(_th) * _pit / 2, math.sin(_th) * _pit / 2); _e2 = (-_e1[1], _e1[0])
        _places = sorted({(round(x + a * _e1[0] + b * _e2[0], 3), round(y + a * _e1[1] + b * _e2[1], 3)) for x, y in _xy.tolist() for a in (-1, 1) for b in (-1, 1)})
        for _x, _y in _places:
            _ix, _iy = int((_x - X0) / RES), int((_y - Y0) / RES)
            if not (0 < _ix < NX - 1 and 0 < _iy < NY - 1): continue
            _pt = shapely.Point(_x, _y); _kr = _pt.buffer(_rn); _close = []
            for _li in range(NL):
                for _j in _PADT[_li].query(_kr):
                    _n2, _g2 = _PADG[_li][int(_j)]; _dd = float(_pt.distance(_g2))
                    if _dd < _rn: _close.append((_n2, round(_dd, 4), _li in (0, NL - 1)))
            VP[_iy * NX + _ix] = (_x, _y, tuple(sorted(_close)))
    if VP: print('exact via slots in the ball grid:', len(VP), flush=True)



BGA = {}
if BGA_FANOUT:
    from scipy.spatial import cKDTree as _BKD
    _bf = {f['ref']: f for f in D.get('fps', []) if 'BGA' in (f.get('fp', '') + f.get('lib', '')).upper()}
    for _ref in sorted(_bf):
        _bp = [p for p in D['pads'] if p['ref'] == _ref and not p.get('drill') and len(p['layers']) == 1 and p['layers'][0] in LI]
        if len(_bp) < 16: continue
        _xy = np.array([(p['x'], p['y']) for p in _bp]); _bd, _bi = _BKD(_xy).query(_xy, k=2)
        _pitch = float(np.median(_bd[:, 1]))
        if not 0.3 <= _pitch <= 1.05: continue
        _v = _xy[_bi[:, 1]] - _xy; _th = float(np.angle(np.mean(np.exp(4j * np.arctan2(_v[:, 1], _v[:, 0]))))) / 4
        _rot = np.array([[math.cos(_th), -math.sin(_th)], [math.sin(_th), math.cos(_th)]])
        _local = _xy @ _rot; _origin = _local.min(axis=0)
        _ij = np.rint((_local - _origin) / _pitch).astype(int)
        if np.max(np.abs(_local - _origin - _ij * _pitch)) > 0.02: continue
        _mx, _my = _ij.max(axis=0); _mid = (_local.min(axis=0) + _local.max(axis=0)) / 2
        for _p, (_i, _j), _lp in zip(_bp, _ij.tolist(), _local):
            _ring = min(_i, _j, int(_mx) - _i, int(_my) - _j)
            _sx, _sy = (1 if _lp[0] >= _mid[0] else -1), (1 if _lp[1] >= _mid[1] else -1)
            _signs = [(_sx, _sy), (_sx, -_sy), (-_sx, _sy), (-_sx, -_sy)]
            _dog = [tuple(np.round(np.array([_p['x'], _p['y']]) + (np.array([a, b]) * _pitch / 2) @ _rot.T, 5)) for a, b in _signs]
            _vip = [(_p['x'], _p['y'])] if VIP else []
            _pts = (_vip if _pitch <= 0.5 + 1e-6 else _dog + _vip)
            BGA[id(_p)] = dict(ref=_ref, pad=_p['num'], ring=_ring, pitch=round(_pitch, 5), layer=_p['layers'][0], pts=[])
            for _x, _y in _pts:
                _ix, _iy = int((_x - X0) / RES), int((_y - Y0) / RES)
                if not (0 < _ix < NX - 1 and 0 < _iy < NY - 1): continue
                _c = _iy * NX + _ix
                if _c in VP and math.dist((_x, _y), VP[_c][:2]) > 1e-5: continue
                _pt = shapely.Point(_x, _y); _close = []; _rn = VIA_KEEP_R + max(DRCP, 0.03) + 0.03
                for _li in range(NL):
                    for _k in _PADT[_li].query(_pt.buffer(_rn)):
                        _n2, _g2 = _PADG[_li][int(_k)]; _dist = float(_pt.distance(_g2))
                        if _dist < _rn: _close.append((_n2, round(_dist, 4), _li in (0, NL - 1)))
                VP[_c] = (_x, _y, tuple(sorted(_close)))
                BGA[id(_p)]['pts'].append(_c)
    print('BGA fanout (option): %d balls, %d inner from ring 2' % (len(BGA), sum(v['ring'] >= 2 for v in BGA.values())), flush=True)
_VP_C = np.array(sorted(VP), dtype=np.int64); _VP_IY, _VP_IX = _VP_C // NX, _VP_C % NX







LANES = (args.get('lanes', '0') == '1' or BGA_TIGHT) and NC is not None
GS_PT = {}
GS_N = 0
if LANES:
    from shapely.ops import nearest_points as _np_pts
    _wg = float(args.get('lanes_w', '%g' % cls('/__signal__')['w'])); _rg = DRCP + PAD_EXTRA
    _gmin = _wg + 2 * _rg; _gmax = _wg + 2 * CLR_PAD + 2 * QM
    _nid = lambda n_: NID[n_] if n_ in NID else -2
    for _li in range(NL):
        _G = _PADG[_li]; _T = _PADT[_li]; _best = {}
        for _i, (_n1, _g1) in enumerate(_G):
            for _j in sorted(int(q) for q in _T.query(_g1.buffer(_gmax))):
                if _j <= _i: continue
                _n2, _g2 = _G[_j]
                if _n1 == _n2: continue
                _d = float(_g1.distance(_g2))
                if not (_gmin <= _d < _gmax): continue
                _pa, _pb = _np_pts(_g1, _g2); _ux, _uy = (_pb.x - _pa.x) / _d, (_pb.y - _pa.y) / _d
                _mx, _my = (_pa.x + _pb.x) / 2, (_pa.y + _pb.y) / 2; _vx, _vy = -_uy, _ux
                _b1, _b2 = _g1.bounds, _g2.bounds
                _L = max(_b1[2] - _b1[0], _b1[3] - _b1[1], _b2[2] - _b2[0], _b2[3] - _b2[1]) / 2 + 0.3
                _k = int(_L / (RES / 4))
                for _t in range(-_k, _k + 1):
                    _x, _y = _mx + _vx * _t * RES / 4, _my + _vy * _t * RES / 4
                    _ix, _iy = int((_x - X0) / RES), int((_y - Y0) / RES)
                    if not (0 < _ix < NX - 1 and 0 < _iy < NY - 1): continue
                    _c = _iy * NX + _ix
                    if 2 * (float(PD1[_li].ravel()[_c]) - CLR_PAD) >= _wg: continue
                    _pt = shapely.Point(_x, _y)
                    _dm = min(float(_G[int(q)][1].distance(_pt)) for q in _T.query(_pt.buffer(_gmax)))
                    _cap = 2 * (_dm - _rg)
                    if _cap < _wg - 1e-9: continue
                    _o = (abs(_x - cell_xy(_ix, _iy)[0]) + abs(_y - cell_xy(_ix, _iy)[1]), -_cap)
                    if _c not in _best or _o < _best[_c][0]: _best[_c] = (_o, _x, _y, _cap, _nid(_n1), _nid(_n2))
        for _c, (_o, _x, _y, _cap, _a, _b) in sorted(_best.items()): GS_PT[_li * NN + _c] = (round(_x, 4), round(_y, 4), _a, _b, _cap)
    GS_N = len(GS_PT)
    _GS_I = np.array(sorted(GS_PT), dtype=np.int64)
    _GS_CAP = np.array([GS_PT[c][4] for c in _GS_I.tolist()], dtype=np.float32)
    _GS_A = np.array([GS_PT[c][2] for c in _GS_I.tolist()], dtype=np.int64); _GS_B = np.array([GS_PT[c][3] for c in _GS_I.tolist()], dtype=np.int64)
    print('narrow lanes (R15, open 29): %d cells walkable on the exact centerline (track %.3f mm)' % (GS_N, _wg), flush=True)


def _lanes_one(net, capH, rng):
    nid = NID[net]; m = (_GS_A != nid) & (_GS_B != nid)
    idx = _GS_I[m]; cap = _GS_CAP[m]; ic = idx % NN; li = idx // NN
    if rng is not None:
        ix, iy = ic % NX, ic // NX; w = (ix >= rng[0]) & (ix < rng[1]) & (iy >= rng[2]) & (iy < rng[3]); idx, cap, ic, li = idx[w], cap[w], ic[w], li[w]
    if not len(idx): return
    cap = np.minimum(cap, 2 * (edge_d.ravel()[ic] - EDGE)).astype(np.float32)
    if FIXD[0] is not None and net not in FIXNETS:
        cap = np.minimum(cap, np.array([2 * (FIXD[0][l].ravel()[c] - QM - CLR - 0.005) for l, c in zip(li.tolist(), ic.tolist())], dtype=np.float32))
    capH.planar[idx] = np.maximum(capH.planar[idx], cap)

VIP_EP = (not VIP) and args.get('vip_ep', '1' if NC else '0') == '1'
EPZ = {}



EP_VIAS = bool(AREA_NETS) and args.get('ep_vias', '1' if NC else '0') == '1'
EP_DIST = max(float(args.get('ep_spacing', '%g' % (VIA_D + 0.3))), HOLE_C)
EP_DIST_K12 = max(EP_DIST, VIA_D + 2 * CLR + 0.2)
EPM = {}
EPG = []
_ep = {}; _epk = {}
if VIP_EP or EP_VIAS:
    _jr3 = {}; _jk3 = {}
    for _k, _p in enumerate(D['pads']):
        if not _p.get('drill'): _jr3.setdefault(_p['ref'], []).append(_p); _jk3[id(_p)] = _k
    for _r in sorted(_jr3):
        _P = _jr3[_r]
        if len(_P) < 8: continue
        for _p in _P:
            _lg = [L for L in _p['layers'] if L in LI and LI[L] in (0, NL - 1)]; _b = _p['bbox']
            if not _p['net'] or _p['net'].startswith('unconnected') or len(_lg) != 1 or min(_b[2] - _b[0], _b[3] - _b[1]) < 1.0: continue
            _xs = [q['x'] for q in _P if min(q['bbox'][2] - q['bbox'][0], q['bbox'][3] - q['bbox'][1]) < 1.0]
            _ys = [q['y'] for q in _P if min(q['bbox'][2] - q['bbox'][0], q['bbox'][3] - q['bbox'][1]) < 1.0]
            if len(_xs) < 6 or not (min(_xs) < _p['x'] < max(_xs) and min(_ys) < _p['y'] < max(_ys)): continue
            _ep.setdefault((_r, _p['net'], LI[_lg[0]]), []).extend(Polygon(q) for q in _p['polys'][_lg[0]] if len(q) >= 3)
            _epk.setdefault((_r, _p['net'], LI[_lg[0]], _p['num']), []).append(_jk3[id(_p)])
if VIP_EP:
    for (_r, _n, _li), _pl in sorted(_ep.items()):
        _g = shapely.union_all(_pl).buffer(-VIA_R)
        if _g.is_empty: continue
        _b = _g.bounds
        _ixa, _ixb = max(1, int((_b[0] - X0) / RES)), min(NX - 1, int((_b[2] - X0) / RES) + 1)
        _iya, _iyb = max(1, int((_b[1] - Y0) / RES)), min(NY - 1, int((_b[3] - Y0) / RES) + 1)
        if _ixb <= _ixa or _iyb <= _iya: continue
        _GX, _GY = np.meshgrid(np.arange(_ixa, _ixb), np.arange(_iya, _iyb))
        _in = shapely.contains_xy(_g, X0 + (_GX.ravel() + 0.5) * RES, Y0 + (_GY.ravel() + 0.5) * RES)
        if _in.any(): EPZ.setdefault(_n, []).append((_li, _GY.ravel()[_in].astype(np.int64), _GX.ravel()[_in].astype(np.int64)))
    if EPZ: print('Thermal pads with allowed via in pad:', sum(len(v) for v in EPZ.values()), flush=True)


def _ep_axis(W):
    return max(1, int((W - VIA_D + 1e-9) / EP_DIST) + 1) if W >= VIA_D else 1


def _ep_split(W, n):
    return 0.0 if n < 2 else min((W - VIA_D) / (n - 1), max(W / n, EP_DIST_K12))


if EP_VIAS:
    for (_r, _n, _li, _num), _ks in sorted(_epk.items()):
        if _n not in AREA_NETS: continue
        _L = LAYERS[_li]
        _gf = [shapely.union_all([Polygon(q) for q in D['pads'][_k]['polys'][_L] if len(q) >= 3]) for _k in _ks]
        _g = shapely.union_all(_gf); _A = float(_g.area)
        _nominal = 4 if _A < 9.0 else (9 if _A < 25.0 else 16)
        _per = -(-_nominal // len(_ks))
        EPG.append(dict(ref=_r, num=_num, net_name=_n, surface=round(_A, 2), nominal=_nominal, fields=len(_ks), poly=_g.buffer(0.02), pads=list(_ks)))
        if _per < 2: continue
        for _k, _gk in zip(_ks, _gf):
            _b = _gk.bounds; _W, _H = _b[2] - _b[0], _b[3] - _b[1]; _cx, _cy = (_b[0] + _b[2]) / 2, (_b[1] + _b[3]) / 2
            _best = None
            for _nx in range(1, _ep_axis(_W) + 1):
                for _ny in range(1, _ep_axis(_H) + 1):
                    _q = (max(0, _per - _nx * _ny), max(0, _nx * _ny - _per), abs(_ep_split(_W, _nx) - _ep_split(_H, _ny)), _nx, _ny)
                    if _best is None or _q < _best: _best = _q
            _nx, _ny = _best[3], _best[4]; _px, _py = _ep_split(_W, _nx), _ep_split(_H, _ny)
            _gi = _gk.buffer(-VIA_R + 0.002)
            if _gi.is_empty: continue
            _places = []
            for _j in range(_ny):
                for _i in range(_nx):
                    _ix, _iy = int((_cx + (_i - (_nx - 1) / 2) * _px - X0) / RES), int((_cy + (_j - (_ny - 1) / 2) * _py - Y0) / RES)
                    if 0 < _ix < NX - 1 and 0 < _iy < NY - 1 and _iy * NX + _ix not in _places: _places.append(_iy * NX + _ix)
            _bi = _gi.bounds
            _ixa, _ixb = max(1, int((_bi[0] - X0) / RES)), min(NX - 1, int((_bi[2] - X0) / RES) + 1)
            _iya, _iyb = max(1, int((_bi[1] - Y0) / RES)), min(NY - 1, int((_bi[3] - Y0) / RES) + 1)
            if _ixb <= _ixa or _iyb <= _iya: continue
            _GX, _GY = np.meshgrid(np.arange(_ixa, _ixb), np.arange(_iya, _iyb))
            _in = shapely.contains_xy(_gi, X0 + (_GX.ravel() + 0.5) * RES, Y0 + (_GY.ravel() + 0.5) * RES)
            EPM[_k] = dict(nominal=_nx * _ny, wish=_per, places=_places, grid_cells=(_GY.ravel()[_in] * NX + _GX.ravel()[_in]).astype(np.int64),
                           split=(round(_px, 3), round(_py, 3)))
    if EPG: print('Ground pads under chips (R16):', ', '.join('%s:%s %.1f mm2 target %d' % (e['ref'], e['num'], e['surface'], e['nominal']) for e in EPG), flush=True)



EP_RES = {}
for _k, _E in sorted(EPM.items()):
    _m = EP_RES.setdefault(D['pads'][_k]['net'], np.zeros((NY, NX), dtype=bool)); _rr = int(EP_DIST / RES) + 1
    for _c in _E['places']:
        _iy, _ix = divmod(_c, NX)
        _ya, _yb, _xa, _xb = max(0, _iy - _rr), min(NY, _iy + _rr + 1), max(0, _ix - _rr), min(NX, _ix + _rr + 1)
        _GX, _GY = np.meshgrid(np.arange(_xa, _xb), np.arange(_ya, _yb))
        _m[_ya:_yb, _xa:_xb] |= np.hypot(_GX - _ix, _GY - _iy) * RES < EP_DIST - 1e-6


def via_hard(net, cells):
    if not len(cells): return False
    if not LARGE: return bool(hard_maps(net)[1].ravel()[cells].any())
    iy, ix = cells // NX, cells % NX
    rng = (max(0, int(ix.min()) - 2), min(NX, int(ix.max()) + 3), max(0, int(iy.min()) - 2), min(NY, int(iy.max()) + 3))
    return bool(hard_maps(net, rng)[1].ravel()[cells].any())


def ep_set(k, viaH, vbadf, rng):
    E = EPM[k]; viaf = viaH.ravel(); Z = E['grid_cells']
    def applies(c): return rng[0] < c % NX < rng[1] - 1 and rng[2] < c // NX < rng[3] - 1
    zx = X0 + (Z % NX + 0.5) * RES; zy = Y0 + (Z // NX + 0.5) * RES
    vacant = viaf[Z] & ~vbadf[Z] & (Z % NX > rng[0]) & (Z % NX < rng[1] - 1) & (Z // NX > rng[2]) & (Z // NX < rng[3] - 1) if len(Z) else None
    def attempt(ab):
        assigned = []; open_items = []
        def wide(c):
            x, y = cell_xy(c % NX, c // NX)
            return all(math.hypot(x - a, y - b) >= EP_DIST - 1e-6 for a, b in assigned)
        for c in E['places']:
            if applies(c) and viaf[c] and not vbadf[c] and wide(c): assigned.append(cell_xy(c % NX, c // NX))
            else: open_items.append(c)
        if open_items and vacant is not None:
            rest = []
            for c in open_items:
                x, y = cell_xy(c % NX, c // NX); m = vacant.copy()
                for a, b in assigned: m &= np.hypot(zx - a, zy - b) >= ab - 1e-6
                if not m.any(): rest.append(c); continue
                d = np.hypot(zx - x, zy - y); d[~m] = 1e9; j = int(np.argmin(d))
                assigned.append((float(zx[j]), float(zy[j])))
            open_items = rest
        if CONS_SOFT[0]:
            for c in open_items:
                if applies(c) and viaf[c] and wide(c): assigned.append(cell_xy(c % NX, c // NX))
        return assigned
    assigned = attempt(EP_DIST)
    if EP_DIST_K12 > EP_DIST + 1e-9:
        g2 = attempt(EP_DIST_K12)
        if len(g2) >= len(assigned): assigned = g2
    return [dict(x=round(x, 4), y=round(y, 4)) for x, y in assigned]







VAP = VIP and bool(AREA_NETS) and args.get('via_at_pad', '1' if NC else '0') == '1'
VAP_K = 3.0 * float(args.get('via_at_pad_mm', '1.5'))
VAP_THIRD = float(args.get('via_at_pad_third', '0.25'))
VAP_WF = float(args.get('via_at_pad_important', '2.0'))
VAP_IMPORTANT = set()
_VZ = np.ones(NN, dtype=np.float32) if VAP else None
_VAPZ = {}
if VAP and NC:
    _wr = {r for r, t in NC.get('part_list', {}).items() if t.get('decoupling')}
    _wr |= {p['ref'] for p in D['pads'] if (NC['net_list'].get(p['net']) or {}).get('category') == 'sw'}
    VAP_IMPORTANT = {k for k, p in enumerate(D['pads']) if p['net'] in AREA_NETS and p['ref'] in _wr}
if VAP: print('Via at pad (R9): a via next to the pad costs %.1f (like %.1f mm stub), %d of %d pads of the area nets count %gfold (decoupling, switching node)' % (
    VAP_K, VAP_K / 3.0, len(VAP_IMPORTANT), sum(len(by_net_g.get(n, ())) for n in AREA_NETS), VAP_WF), flush=True)


def vap_share(g, xs, ys):
    if not len(xs): return np.zeros(0)
    kr = shapely.buffer(shapely.points(xs, ys), VIA_R, quad_segs=8)
    return shapely.area(shapely.intersection(kr, g)) / min(float(shapely.area(kr[0])), float(g.area))


def _vap_cells(k):
    r = _VAPZ.get(k)
    if r is None:
        p = D['pads'][k]; L = next(x for x in p['layers'] if x in LI)
        g = shapely.union_all([Polygon(q) for q in p['polys'][L] if len(q) >= 3]); b = g.bounds
        ixa, ixb = max(0, int((b[0] - VIA_R - X0) / RES)), min(NX, int((b[2] + VIA_R - X0) / RES) + 1)
        iya, iyb = max(0, int((b[1] - VIA_R - Y0) / RES)), min(NY, int((b[3] + VIA_R - Y0) / RES) + 1)
        GX, GY = np.meshgrid(np.arange(ixa, ixb), np.arange(iya, iyb)); gx = GX.ravel(); gy = GY.ravel()
        x = X0 + (gx + 0.5) * RES; y = Y0 + (gy + 0.5) * RES
        close = shapely.distance(g, shapely.points(x, y)) < VIA_R
        a = vap_share(g, x[close], y[close]); m = a >= 1 / 3 - 0.01
        r = _VAPZ[k] = ((gy[close][m] * NX + gx[close][m]).astype(np.int64), np.where(a[m] >= 0.49, 0.0, VAP_THIRD).astype(np.float32))
    return r


def _ep_one(net, viaH, capH, rng):
    for li_e, cy_, cx_ in EPZ.get(net, ()):
        m = ~viaH[cy_, cx_]
        if rng is not None: m &= (cx_ >= rng[0]) & (cx_ < rng[1]) & (cy_ >= rng[2]) & (cy_ < rng[3])
        if not m.any(): continue
        cy2, cx2 = cy_[m], cx_[m]
        ok = (edge_d[cy2, cx2] - EDGE >= VIA_R) & (VSP[cy2, cx2] >= H2H + VIA_DR / 2 + 0.01)
        if VIA_BAN is not None: ok &= ~VIA_BAN[cy2, cx2]
        if FIXD[1] is not None and net not in FIXNETS: ok &= FIXD[1][cy2, cx2] - QM >= HOLE_C
        for li in range(NL):
            ok &= capH[li][cy2, cx2] / 2 >= VIA_KEEP_R
            if li in (0, NL - 1) and li != li_e: ok &= padd[li][cy2, cx2] >= VIA_R + VIA_WEB
        viaH[cy2[ok], cx2[ok]] = True


def _vp_ok(net, close):
    for n2, d, outer in close:
        if n2 != net:
            if d < VIA_R + DRCP + 0.006: return False
            if BGA_HOLE and d < VIA_DR / 2 + HOLE_CLR + 0.000001: return False
        elif outer and not VIP and d < VIA_R + VIA_WEB: return False
    return True
def _vp_xy(net, c):
    if globals().get('KA') is not None:
        v=VP.get(c)
        return (v[0],v[1]) if v is not None and _ka_via_ok(net,v[0],v[1]) else None
    if PIN_ACCESS and net in PLANE_AREA and net not in globals().get('PLANE_MIXED_NETS',()) and c in VP and not PLANE_AREA[net].covers(_PT(*VP[c][:2])): return None
    if _pz_active(net) and c in VP and _pz_near(net,*VP[c][:2]):
        v=VP.get(c)
        return (v[0],v[1]) if v is not None and _pz_via_ok(net,v[0],v[1]) else None
    if FIXD[0] is not None: return None
    v = VP.get(c)
    return (v[0], v[1]) if v is not None and _vp_ok(net, v[2]) else None


def _vp_one(net, viaH, rng):
    if globals().get('KA') is not None:
        for c in _VP_C.tolist():
            ix,iy=c%NX,c//NX
            if rng is not None and not (rng[0]<=ix<rng[1] and rng[2]<=iy<rng[3]):continue
            if _ka_via_ok(net,*VP[c][:2]):viaH[iy,ix]=True
        return
    if _pz_active(net) and not _PZ_BASE[0]:
        for c in _VP_C.tolist():
            ix,iy=c%NX,c//NX
            if rng is not None and not (rng[0]<=ix<rng[1] and rng[2]<=iy<rng[3]):continue
            v=VP[c]
            if not _pz_near(net,v[0],v[1]):continue
            if _pz_via_ok(net,v[0],v[1]): viaH[iy,ix]=True
        return
    if not len(_VP_C) or FIXD[0] is not None: return
    m = ~viaH[_VP_IY, _VP_IX] & (edge_d[_VP_IY, _VP_IX] - EDGE >= VIA_R) & (VSP[_VP_IY, _VP_IX] >= H2H + VIA_DR / 2 + 0.01)
    if VIA_BAN is not None: m &= ~VIA_BAN[_VP_IY, _VP_IX]
    if rng is not None: m &= (_VP_IX >= rng[0]) & (_VP_IX < rng[1]) & (_VP_IY >= rng[2]) & (_VP_IY < rng[3])
    if FIXD[1] is not None and net not in FIXNETS: m &= FIXD[1][_VP_IY, _VP_IX] - QM >= HOLE_C
    for c in _VP_C[m].tolist():
        if PIN_ACCESS and net in PLANE_MASK and net not in globals().get('PLANE_MIXED_NETS',()):
            if not PLANE_MASK[net][c // NX, c % NX] or not PLANE_AREA[net].covers(_PT(*VP[c][:2])): continue
        if _vp_ok(net, VP[c][2]): viaH[c // NX, c % NX] = True


from functools import lru_cache as _lru_cache
PAD_ROOM_CACHE = args.get('pad_room_cache', '1') == '1'


def pad_room(li, a, b, net):

    a, b = tuple(a), tuple(b)
    return _pad_room_cached(li, a, b, net) if PAD_ROOM_CACHE else _pad_room_exact(li, a, b, net)


def _pad_room_exact(li, a, b, net):
    if globals().get('KA') is not None: return _ka_room(net,li,a,b)
    seg = _LS([a, b]) if (abs(a[0] - b[0]) > 1e-9 or abs(a[1] - b[1]) > 1e-9) else _PT(a)
    best = 9.0
    for j in _PADT[li].query(seg.buffer(1.0)):
        n2, g2 = _PADG[li][int(j)]
        if n2 == net: continue
        best = min(best, seg.distance(g2))
    room = best - (DRCP + PAD_EXTRA)
    if _pz_active(net): room = min(best - DRCP - PZ_BORDER_NM / 1e6, _pz_room(net,li,a,b))
    if ZONE_PLAN:
        room = min(room, min((seg.distance(g)-CLR-.005 for (n,L),g in _ZP_GEO.items() if n!=net and L==LAYERS[li]), default=9.0))
    if PLANE_COPPER:
        room=min(room,min((seg.distance(g)-CLR-.005 for (n,L),g in PLANE_COPPER.items() if n!=net and L==LAYERS[li]),default=9.))
    return room


_pad_room_cached = _lru_cache(maxsize=100000)(_pad_room_exact)


def _pad_room_lower_loop(d1, d2, pn, nid, ax, ay, bx, by, x0, y0, res, nx, ny, count):
    best = 9.0
    for k in range(count):
        t = k / (count - 1)
        ix = int(math.floor((ax + t * (bx - ax) - x0) / res))
        iy = int(math.floor((ay + t * (by - ay) - y0) / res))
        if ix < 0 or ix >= nx or iy < 0 or iy >= ny: return -999.0
        d = d2[iy, ix] if pn[iy, ix] == nid else d1[iy, ix]
        if d < best: best = d
    return best


_pad_room_lower_nb = None
if NB is not None:
    from numba import njit as _pad_njit
    _pad_room_lower_nb = _pad_njit(cache=True)(_pad_room_lower_loop)


@_lru_cache(maxsize=100000)
def _pad_room_lower(li, a, b, net):
    L = math.dist(a, b); count = max(2, int(L / (RES * 0.5)) + 1)
    f = _pad_room_lower_nb if _pad_room_lower_nb is not None else _pad_room_lower_loop
    dist = f(PD1[li], PD2[li], PN1[li], NID.get(net, -1), a[0], a[1], b[0], b[1], X0, Y0, RES, NX, NY, count)
    return min(float(dist), RW - 2 * RES) - RES / math.sqrt(2) - L / (2 * (count - 1)) - 1e-5 - DRCP - PAD_EXTRA


def pad_room_min(li, a, b, net, demand):
    if globals().get('KA') is None and PAD_ROOM_CACHE and not ZONE_PLAN and not _pz_active(net):
        if _pad_room_lower(li, tuple(a), tuple(b), net) >= demand: return demand
    return min(demand, pad_room(li, a, b, net))


def line_cap_min(capL, a, b):
    (x0, y0), (x1, y1) = a, b
    L = math.hypot(x1 - x0, y1 - y0); n = max(2, int(L / (RES * 0.5)) + 1)
    if NB is not None:
        return float(_linecap_nb(capL, x0, y0, x1, y1, n, X0, Y0, RES))
    t = np.linspace(0, 1, n)
    ix = ((x0 + t * (x1 - x0) - X0) / RES).astype(int); iy = ((y0 + t * (y1 - y0) - Y0) / RES).astype(int)
    return float(capL[iy, ix].min())


def oct_(a, b):
    dx, dy = abs(b[0] - a[0]), abs(b[1] - a[1])
    return dx < 1e-6 or dy < 1e-6 or abs(dx - dy) < 1e-6






HYPO = OCT and args.get('hypo', '1') != '0'



HYPO_LATE = HYPO and args.get('hypo', 'end') == 'end' and ('start' not in args or args.get('hypo') == 'end')
_HYPO_ON = [HYPO and not HYPO_LATE]


def _corners(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]; m = min(abs(dx), abs(dy))
    if m < 1e-9 or abs(abs(dx) - abs(dy)) < 1e-9: return ()
    sx, sy = (1.0 if dx > 0 else -1.0), (1.0 if dy > 0 else -1.0)
    return ((b[0] - sx * m, b[1] - sy * m), (a[0] + sx * m, a[1] + sy * m))










PAD_CENTER = args.get('pad_center', '1' if NC else '0') == '1'
PAD_CENTER_MAX = float(args.get('pad_center_max', '4.0'))


def _pad_center(li, pt):
    q = _PT(pt)
    for j in _PADT[li].query(q.buffer(0.03)):
        n2, g2 = _PADG[li][int(j)]
        if n2 == NET_CUR[0] and g2.distance(q) <= 0.03:
            c = _PADXY[li][int(j)]
            if c is not None and g2.contains(_PT(c)): return (c, g2)
    return None


def _pad_connection(P, li, capL, C):
    r = _pad_center(li, P[-1])
    if r is None: return P
    c, g = r; e = P[-1]; d = math.dist(e, c)
    if d < 1e-6 or d > PAD_CENTER_MAX: return P
    req = C['wmin']
    def vacant(a, b): return line_cap_min(capL, a, b) >= req - 1e-6 and pad_room(li, a, b, NET_CUR[0]) >= req / 2 and (not IMP_REF_MASK or z_ref_free(C, li, a, b))
    def in_pad(a, b): return g.buffer(0.03).contains(_LS([a, b]))
    def paths(a, b):
        if not oct_net() or oct_(a, b): yield []
        if oct_net():
            for ck in reversed(_corners(a, b)): yield [ck]
    if len(P) >= 3:
        q, a = P[-3], P[-2]; d1 = (a[0] - q[0], a[1] - q[1]); d2 = (e[0] - a[0], e[1] - a[1])
        det = -d1[0] * d2[1] + d1[1] * d2[0]
        if abs(det) > 1e-9:
            t = (-(c[0] - q[0]) * d2[1] + (c[1] - q[1]) * d2[0]) / det
            a2 = (q[0] + t * d1[0], q[1] + t * d1[1])
            if t > 1e-6 and math.dist(a2, a) <= 0.25 and vacant(q, a2) and vacant(a2, c): return P[:-2] + [a2, c]
    if len(P) >= 2:
        a = P[-2]
        for zw in paths(a, c):
            pk = [a] + zw + [c]
            if all(vacant(x, y) for x, y in zip(pk[:-1], pk[1:])): return P[:-1] + zw + [c]
    for zw in paths(e, c):
        pk = [e] + zw + [c]
        if all(in_pad(x, y) and vacant(x, y) for x, y in zip(pk[:-1], pk[1:])): return P + zw + [c]
    return P












R27_TRUNK = PAD_CENTER and args.get('r27_trunk', '0') == '1'
BRANCH_EXACT = args.get('branch', '0') == '1' or (R27_TRUNK and args.get('branch_exact', '1') == '1')
OFFSET = PAD_CENTER and args.get('offset', '1') == '1'
JOINT_STAT = dict(branch_exact=0, branch_pushed=0, branch_corner=0, branch_skewed=0, branch_open=0, via_start=0, via_start_beside=0,
                  via_beside_track=0, offset=0, offset_end=0)
def _cut(p, d1, q, d2):
    det = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(det) < 1e-9 * max(1e-9, math.hypot(*d1) * math.hypot(*d2)): return None
    t = ((q[0] - p[0]) * d2[1] - (q[1] - p[1]) * d2[0]) / det
    return (p[0] + t * d1[0], p[1] + t * d1[1])


def _on_anchor(P, A, vacant):
    if math.dist(P[0], A) < 1e-7: return [A] + P[1:]
    if not oct_net(): return [A] + P[1:] if vacant(A, P[1]) else None
    if len(P) < 3: return None
    d1 = (P[1][0] - P[0][0], P[1][1] - P[0][1]); d2 = (P[2][0] - P[1][0], P[2][1] - P[1][1])
    a2 = _cut(A, d1, P[1], d2)
    if a2 is None or math.dist(a2, P[1]) > 0.25: return None
    if (a2[0] - A[0]) * d1[0] + (a2[1] - A[1]) * d1[1] <= 1e-9 or (P[2][0] - a2[0]) * d2[0] + (P[2][1] - a2[1]) * d2[1] <= 1e-9: return None
    return [A, a2] + P[2:] if vacant(A, a2) and vacant(a2, P[2]) else None


def _trunk_point(t, i, X):
    t['pts'] = list(t['pts'][:i + 1]) + [[X[0], X[1]]] + list(t['pts'][i + 1:])
    t['w'] = list(t['w'][:i + 1]) + [t['w'][i]] + list(t['w'][i + 1:])


def _branch(P, li, node, capL, C):
    S = P[0]
    if len(P) < 2: return P
    v = _TRUNK_V.get(node); st = [t for t in _TRUNK.get(node, ()) if LI[t['layer']] == li]
    if v is None and not st: return P
    c0 = min(C['wmin'], line_cap_min(capL, S, P[1])); r0 = min(C['wmin'] / 2, pad_room(li, S, P[1], NET_CUR[0]))



    def vacant(a, b): return pad_room(li, a, b, NET_CUR[0]) >= r0 - 1e-9 and (not oct_net() or line_cap_min(capL, a, b) >= c0 - 1e-6)
    if v is not None:
        if math.dist(S, v) < 1e-7: return P
        Q = _on_anchor(P, v, vacant); JOINT_STAT['via_start' if Q else 'via_start_beside'] += 1
        return Q or P
    cand = []; corner = []
    d1 = (P[1][0] - S[0], P[1][1] - S[1])
    for t in st:
        pts = t['pts']
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]; dx, dy = b[0] - a[0], b[1] - a[1]; l2 = dx * dx + dy * dy
            if l2 < 1e-12: continue
            u = max(0.0, min(1.0, ((S[0] - a[0]) * dx + (S[1] - a[1]) * dy) / l2)); L_ = (a[0] + u * dx, a[1] + u * dy); d = math.dist(S, L_)
            if d > 0.045: continue
            corner.append((d, 1, len(corner), i, L_, t))
            for j_, e_ in ((i, a), (i + 1, b)):
                if math.dist(S, e_) <= 0.08: corner.append((math.dist(S, e_), 0, len(corner), j_, (e_[0], e_[1]), t))
            if d < 1e-7: cand.append((0.0, len(cand), t, i, S)); continue
            X = _cut(S, d1, a, (dx, dy))
            if X is None: continue
            ux = ((X[0] - a[0]) * dx + (X[1] - a[1]) * dy) / l2
            if ux < -1e-9 or ux > 1 + 1e-9 or math.dist(X, S) > 0.08: continue
            if (P[1][0] - X[0]) * d1[0] + (P[1][1] - X[1]) * d1[1] <= 1e-9: continue
            cand.append((math.dist(X, S), len(cand), t, i, X))
    for d, _, t, i, X in sorted(cand, key=lambda k_: k_[:2]):
        a, b = t['pts'][i], t['pts'][i + 1]
        if math.dist(X, a) < 2e-4: X = (a[0], a[1]); new = False
        elif math.dist(X, b) < 2e-4: X = (b[0], b[1]); new = False
        else: X = (round(X[0], 4), round(X[1], 4)); new = True


        if new: _trunk_point(t, i, X)
        JOINT_STAT['branch_exact' if d <= 1e-7 else 'branch_pushed'] += 1
        return [X] + P[1:]
    for d, lot, _, i, A, t in sorted(corner, key=lambda k_: k_[:3]):
        A = (round(A[0], 4), round(A[1], 4))
        Q = _on_anchor(P, A, vacant); category = 'branch_corner'
        if Q is None and vacant(A, P[1]): Q = [A] + P[1:]; category = 'branch_skewed'
        if Q is None: continue
        if lot and math.dist(A, t['pts'][i]) > 2e-4 and math.dist(A, t['pts'][i + 1]) > 2e-4: _trunk_point(t, i, A)
        JOINT_STAT[category] += 1
        return Q
    if cand or corner: JOINT_STAT['branch_open'] += 1
    return P


def _via_on_trunk(li, node, S):
    if node in _TRUNK_V: return
    best = None
    for t in _TRUNK.get(node, ()):
        if LI[t['layer']] != li: continue
        pts = t['pts']
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]; dx, dy = b[0] - a[0], b[1] - a[1]; l2 = dx * dx + dy * dy
            if l2 < 1e-12: continue
            u = max(0.0, min(1.0, ((S[0] - a[0]) * dx + (S[1] - a[1]) * dy) / l2)); d = math.hypot(S[0] - a[0] - u * dx, S[1] - a[1] - u * dy)
            if best is None or d < best[0]: best = (d, t, i)
    if best is None: return
    d, t, i = best
    if d > 1e-7: JOINT_STAT['via_beside_track'] += 1; return
    X = (round(S[0], 4), round(S[1], 4))
    if math.dist(X, t['pts'][i]) > 2e-4 and math.dist(X, t['pts'][i + 1]) > 2e-4: _trunk_point(t, i, X)


def joints_repair():
    n = 0; per = {}; prior = {}
    for u, r in U.items():
        if u.startswith(('FIX#', 'TACK#')): continue
        V = [(v['x'], v['y']) for v in r['vias'] if (v.get('net') or r['net']) == r['net']]; ch = False
        prior[u] = [(t, list(t['pts']), list(t['w'])) for t in r['tracks']]
        for t in r['tracks']:
            if len(t['pts']) < 2: continue
            li = LI[t['layer']]
            for end in (0, -1):
                E = t['pts'][end]; N = t['pts'][1] if end == 0 else t['pts'][-2]
                if any(math.dist(E, v) < 1e-6 for v in V): continue
                _pe = _PT(E)
                if any(_PADG[li][int(j)][0] == r['net'] and _PADG[li][int(j)][1].covers(_pe) for j in _PADT[li].query(_pe)): continue
                def direction_preserving(X):
                    if not oct_net(r['net']): return True
                    for t3 in r['tracks']:
                        if t3['layer'] != t['layer'] or len(t3['pts']) < 2: continue
                        for e3, n3 in ((0, 1), (-1, -2)):
                            if math.dist(t3['pts'][e3], E) >= 1e-6: continue
                            q = t3['pts'][n3]; dx, dy = E[0] - q[0], E[1] - q[1]
                            vx, vy = X[0] - q[0], X[1] - q[1]
                            if not oct_(q, X) or abs(dx * vy - dy * vx) > 1e-7 * max(1.0, math.hypot(dx, dy)) or dx * vx + dy * vy <= 0:
                                return False
                    return True
                best = None; exact = False
                for t2 in r['tracks']:
                    if t2 is t or t2['layer'] != t['layer']: continue
                    P2 = t2['pts']
                    for i in range(len(P2) - 1):
                        a, b = P2[i], P2[i + 1]; dx, dy = b[0] - a[0], b[1] - a[1]; l2 = dx * dx + dy * dy
                        if l2 < 1e-12: continue
                        uu = max(0.0, min(1.0, ((E[0] - a[0]) * dx + (E[1] - a[1]) * dy) / l2))
                        d = math.hypot(E[0] - a[0] - uu * dx, E[1] - a[1] - uu * dy)
                        if d <= 1e-7: exact = True; break
                        if d > 0.045: continue
                        X = _cut(E, (N[0] - E[0], N[1] - E[1]), a, (dx, dy))
                        if X is not None:
                            ux = ((X[0] - a[0]) * dx + (X[1] - a[1]) * dy) / l2
                            if -1e-9 <= ux <= 1 + 1e-9 and math.dist(X, E) <= 0.08 and (N[0] - X[0]) * (N[0] - E[0]) + (N[1] - X[1]) * (N[1] - E[1]) > 1e-9:
                                k_ = (math.dist(X, E), 0, t2, i, X)
                                if direction_preserving(X) and (best is None or k_[:2] < best[:2]): best = k_
                                continue
                        for e_ in (a, b):
                            if math.dist(E, e_) <= 0.08:
                                k_ = (math.dist(E, e_), 1, t2, i, (e_[0], e_[1]))
                                if direction_preserving(e_) and (best is None or k_[:2] < best[:2]): best = k_
                    if exact: break
                if exact or best is None: continue
                _, _, t2, i, X = best
                a, b = t2['pts'][i], t2['pts'][i + 1]
                if math.dist(X, a) < 2e-4: X = (a[0], a[1])
                elif math.dist(X, b) < 2e-4: X = (b[0], b[1])
                else: X = (round(X[0], 4), round(X[1], 4))
                if not direction_preserving(X): continue
                if math.dist(X, a) >= 2e-4 and math.dist(X, b) >= 2e-4: _trunk_point(t2, i, X)
                for t3 in r['tracks']:
                    if t3['layer'] != t['layer']: continue
                    for e3 in (0, -1):
                        if math.dist(t3['pts'][e3], E) < 1e-6:
                            if e3 == 0: t3['pts'] = [[X[0], X[1]]] + list(t3['pts'][1:])
                            else: t3['pts'] = list(t3['pts'][:-1]) + [[X[0], X[1]]]
                n += 1; ch = True; per[u] = per.get(u, 0) + 1
        if ch: _GEO.pop(u, None)
    if per:
        post = exact_conflicts()
        for u in sorted(per):
            if u not in post: continue
            for t, p_, w_ in prior[u]: t['pts'] = p_; t['w'] = w_
            _GEO.pop(u, None); n -= per[u]; JOINT_STAT['joint_back'] = JOINT_STAT.get('joint_back', 0) + per[u]
    return n


def collinear_together():
    kn = {}
    for u, r in U.items():
        s = kn.setdefault(r['net'], set())
        for t in r['tracks']:
            for q in (t['pts'][0], t['pts'][-1]): s.add((t['layer'], round(q[0], 4), round(q[1], 4)))
        for v in r['vias']:
            if (v.get('net') or r['net']) != r['net']: continue
            for L in LAYERS: s.add((L, round(v['x'], 4), round(v['y'], 4)))
    n = 0
    for u, r in U.items():
        if u.startswith(('FIX#', 'TACK#')): continue
        s = kn[r['net']]; ch = False
        for t in r['tracks']:
            P, W = t['pts'], t['w']
            if len(P) < 3: continue
            np_, nw = [P[0]], []
            for i in range(1, len(P)):
                if nw and abs(nw[-1] - W[i - 1]) < 1e-6 and len(np_) >= 2 and (t['layer'], round(np_[-1][0], 4), round(np_[-1][1], 4)) not in s:
                    a, b, c = np_[-2], np_[-1], P[i]; dx, dy = c[0] - a[0], c[1] - a[1]; l_ = math.hypot(dx, dy)
                    if l_ > 1e-9 and abs((b[0] - a[0]) * dy - (b[1] - a[1]) * dx) / l_ <= 2e-4 and (b[0] - a[0]) * dx + (b[1] - a[1]) * dy > 0 \
                            and (c[0] - b[0]) * dx + (c[1] - b[1]) * dy > 0:
                        np_[-1] = P[i]; n += 1; ch = True; continue
                np_.append(P[i]); nw.append(W[i - 1])
            if len(np_) != len(P): t['pts'] = np_; t['w'] = nw
        if ch: _GEO.pop(u, None)
    return n


MINI = 0.1


def _offset_way(P, li, capL, C):
    if len(P) < 4: return P
    def cap(a, b): return line_cap_min(capL, a, b)
    def vacant(a, b, req): return cap(a, b) >= req - 1e-6 and pad_room(li, a, b, NET_CUR[0]) >= C['wmin'] / 2 and (not IMP_REF_MASK or z_ref_free(C, li, a, b))
    def middle(Q, i):
        if i < 1 or i + 3 > len(Q) - 1: return None
        u = (Q[i][0] - Q[i - 1][0], Q[i][1] - Q[i - 1][1]); v = (Q[i + 2][0] - Q[i + 1][0], Q[i + 2][1] - Q[i + 1][1])
        lu, lv = math.hypot(*u), math.hypot(*v)
        if lu < 1e-9 or lv < 1e-9 or abs(u[0] * v[1] - u[1] * v[0]) > 1e-6 * lu * lv or u[0] * v[0] + u[1] * v[1] <= 0: return None
        w = (Q[i + 3][0] - Q[i + 2][0], Q[i + 3][1] - Q[i + 2][1])
        X = _cut(Q[i - 1], u, Q[i + 2], w)
        if X is None or math.dist(X, Q[i + 2]) > 0.2: return None
        if (X[0] - Q[i - 1][0]) * u[0] + (X[1] - Q[i - 1][1]) * u[1] <= 1e-9 or (Q[i + 3][0] - X[0]) * w[0] + (Q[i + 3][1] - X[1]) * w[1] <= 1e-9: return None
        req = min([C['wmin']] + [cap(Q[j], Q[j + 1]) for j in range(i - 1, i + 3)])
        return Q[:i] + [X] + Q[i + 3:] if vacant(Q[i - 1], X, req) and vacant(X, Q[i + 3], req) else None
    def end(Q):
        if len(Q) < 4: return None
        q, a, e, c = Q[-4], Q[-3], Q[-2], Q[-1]
        d1 = (a[0] - q[0], a[1] - q[1]); d2 = (e[0] - a[0], e[1] - a[1])
        a2 = _cut(q, d1, c, d2)
        if a2 is None or math.dist(a2, a) > 0.2: return None
        if (a2[0] - q[0]) * d1[0] + (a2[1] - q[1]) * d1[1] <= 1e-9 or (c[0] - a2[0]) * d2[0] + (c[1] - a2[1]) * d2[1] <= 1e-9: return None
        req = min(C['wmin'], cap(q, a), cap(a, e), cap(e, c))
        return Q[:-3] + [a2, c] if vacant(q, a2, req) and vacant(a2, c, req) else None
    for _ in range(40):
        new = None
        for i in range(len(P) - 1):
            if not (1e-9 < math.dist(P[i], P[i + 1]) < MINI + 1e-9): continue
            new = middle(P, i)
            if new is None:
                new = middle(P[::-1], len(P) - 2 - i); new = new[::-1] if new else None
            if new is not None: JOINT_STAT['offset'] += 1; break
            if i == len(P) - 2: new = end(P)
            elif i == 0: new = end(P[::-1]); new = new[::-1] if new else None
            if new is not None: JOINT_STAT['offset_end'] += 1; break
        if new is None: break
        P = new
    return P


def _in_own_pad(li, pt):
    q = _PT(pt)
    for j in _PADT[li].query(q):
        n2, g2 = _PADG[li][int(j)]
        if n2 == NET_CUR[0] and g2.contains(q) and g2.boundary.distance(q) >= 0.02: return True
    return False


def _surcharge(li, a, b, ex):
    L = math.hypot(b[0] - a[0], b[1] - a[1]); n = max(2, int(L / (RES * 0.5)) + 1); t = np.linspace(0, 1, n)
    ix = ((a[0] + t * (b[0] - a[0]) - X0) / RES).astype(int); iy = ((a[1] + t * (b[1] - a[1]) - Y0) / RES).astype(int)
    f = np.ones(n, dtype=np.float32) if ex is None else ex[li * NN + iy * NX + ix].astype(np.float32)
    if li == 0: f = np.where(under_esp[iy, ix], f * 1.8, f)
    if IMP_REF_MASK:
        _m = IMP_REF_MASK.get((NET_CUR[0], LAYERS[li]))
        if _m is not None: f = np.where(_m[iy, ix], f, f * 8.0)
    return float((f - 1.0).sum()) * L / n


def path_to_geometry(path, capC, C):
    runs, cur = [], [path[0]]
    for n in path[1:]:
        if n // NN != cur[-1] // NN: runs.append(cur); cur = [n]
        else: cur.append(n)
    runs.append(cur)
    out_tr, out_v = [], []
    for k, run in enumerate(runs):
        li = run[0] // NN
        if C.get('zw'): C = dict(C, w=C['zw'][li], wmin=C['zw'][li])
        if C.get('current_w'): C = dict(C, w=C['current_w'][li], wmin=C['current_w'][li])
        pts = [cell_xy((n % NN) % NX, (n % NN) // NX) for n in run]
        if GS_N:
            _nc = NID.get(NET_CUR[0], -3)
            for _q, _nq in enumerate(run):
                _g = GS_PT.get(_nq)
                if _g is not None and _g[2] != _nc and _g[3] != _nc: pts[_q] = (_g[0], _g[1])
        if VP:
            if k > 0:
                _s = _vp_xy(NET_CUR[0], run[0] % NN)
                if _s: pts[0] = _s
            if k < len(runs) - 1:
                _s = _vp_xy(NET_CUR[0], run[-1] % NN)
                if _s: pts[-1] = _s
        capL = capC[li]
        pc = [float(capL.ravel()[n % NN]) for n in run]


        _r41_pre = None
        if R41_PARAM is not None and C.get('extra') is not None:
            _r41_pre = [0.0]
            for _a, _b in zip(pts[:-1], pts[1:]):
                _r41_pre.append(_r41_pre[-1] + _surcharge(li, _a, _b, C['extra']))
        def _r41_shortcut(i, j):
            return _r41_pre is None or _surcharge(li, pts[i], pts[j], C['extra']) <= _r41_pre[j] - _r41_pre[i] + RES
        if BRANCH_EXACT and k == 0 and len(run) == 1 and len(runs) > 1: _via_on_trunk(li, run[0], pts[0])
        if _HYPO_ON[0] and oct_net():
            _ex = C.get('extra'); _pre = [0.0]
            for _a, _b in zip(pts[:-1], pts[1:]): _pre.append(_pre[-1] + _surcharge(li, _a, _b, _ex))
            def _free(a, b, req):
                return line_cap_min(capL, a, b) >= req - 1e-6 and pad_room_min(li, a, b, NET_CUR[0], max(req, C['wmin']) / 2) >= max(req, C['wmin']) / 2 and (not IMP_REF_MASK or z_ref_free(C, li, a, b))
            def _connect(i, j):
                if j == i + 1: return []
                a, b = pts[i], pts[j]; mn = min(pc[i:j + 1]); req = max(min(C['wmin'], mn), min(C['w'], mn))
                if oct_(a, b): return [] if _free(a, b, req) and _r41_shortcut(i, j) else None
                best = None
                for c in _corners(a, b):
                    if _free(a, c, req) and _free(c, b, req):
                        kk = _surcharge(li, a, c, _ex) + _surcharge(li, c, b, _ex)
                        q = min(line_cap_min(capL, a, c), line_cap_min(capL, c, b))
                        if best is None or kk < best[0] - 1e-9 or (abs(kk - best[0]) <= 1e-9 and q > best[1] + 1e-9): best = (kk, q, c)
                if best is None or best[0] > _pre[j] - _pre[i] + 0.05: return None
                return [best[2]]
            P, i, n_ = [pts[0]], 0, len(pts)
            while i < n_ - 1:
                j, zw = n_ - 1, None
                zw = _connect(i, j)
                if zw is None:
                    lo, hi, zw = i + 1, j, []
                    while hi - lo > 1:
                        mid = (lo + hi) // 2; r = _connect(i, mid)
                        if r is None: hi = mid
                        else: lo, zw = mid, r
                    j = lo
                P.extend(zw); P.append(pts[j]); i = j


            if k == 0 and len(P) >= 3 and math.dist(P[0], P[1]) < 0.1 + 1e-9 and _in_own_pad(li, P[1]): P = P[1:]
            if k == len(runs) - 1 and len(P) >= 3 and math.dist(P[-1], P[-2]) < 0.1 + 1e-9 and _in_own_pad(li, P[-2]): P = P[:-1]
        else:
            simp, i = [0], 0
            while i < len(pts) - 1:
                j_best = i + 1
                for j in range(len(pts) - 1, i + 1, -1):
                    if oct_net() and not oct_(pts[i], pts[j]): continue
                    req = max(min(C['wmin'], min(pc[i:j + 1])), min(C['w'], min(pc[i:j + 1])))
                    if SOURCE_FREE[0]: req = max(req, max(TRACK_MIN, min(C['wmin'], NECK_W)))
                    if line_cap_min(capL, pts[i], pts[j]) >= req - 1e-6 and (j == i + 1 or pad_room_min(li, pts[i], pts[j], NET_CUR[0], max(req, C['wmin']) / 2) >= max(req, C['wmin']) / 2) and (j == i + 1 or not IMP_REF_MASK or z_ref_free(C, li, pts[i], pts[j])):
                        if j > i + 1 and not _r41_shortcut(i, j): continue
                        j_best = j; break
                simp.append(j_best); i = j_best
            P = [pts[i] for i in simp]
        if PAD_CENTER and C.get('anchor', True):
            if k == len(runs) - 1: P = _pad_connection(P, li, capL, C)
            if k == 0: P = _pad_connection(P[::-1], li, capL, C)[::-1]
        if BRANCH_EXACT and k == 0 and P[0] == pts[0]: P = _branch(P, li, run[0], capL, C)
        if OFFSET and (_HYPO_ON[0] or not oct_net()): P = _offset_way(P, li, capL, C)
        seg_pts, seg_w = [P[0]], []
        for a, b in zip(P[:-1], P[1:]):
            L = math.hypot(b[0] - a[0], b[1] - a[1]); m = max(1, int(math.ceil(L / 0.8)))
            for q in range(m):
                pa = (a[0] + (b[0] - a[0]) * q / m, a[1] + (b[1] - a[1]) * q / m)
                pb = (a[0] + (b[0] - a[0]) * (q + 1) / m, a[1] + (b[1] - a[1]) * (q + 1) / m)
                wv = min(C['w'], line_cap_min(capL, pa, pb), 2 * pad_room_min(li, pa, pb, NET_CUR[0], C['w'] / 2))
                _bo = min(C['wmin'], 0.15) if C.get('prio', 8) < 8 else min(C['wmin'], 0.1)
                if C.get('ground_map') is not None:
                    _bk = float(C['ground_map'][li * NN + int(((pa[1] + pb[1]) / 2 - Y0) / RES) * NX + int(((pa[0] + pb[0]) / 2 - X0) / RES)])
                    if _bk < _bo: _bo = _bk
                wv = max(TRACK_MIN, _bo, math.floor(wv * 40) / 40)
                if C.get('current_w'): wv = current_segment_width(C,min(line_cap_min(capL,pa,pb), 2*pad_room_min(li,pa,pb,NET_CUR[0],C['w']/2)))
                if C.get('zw'): wv = C['zw'][li]
                if _pz_active(NET_CUR[0]) and NET_CUR[0] in PZ_WIDTHS:wv=PZ_WIDTHS[NET_CUR[0]]['w']
                seg_pts.append(pb); seg_w.append(wv)
        mp, mw = [seg_pts[0]], []
        for i2, wv in enumerate(seg_w):
            pb = seg_pts[i2 + 1]
            if mw and abs(mw[-1] - wv) < 1e-6 and len(mp) >= 2:
                a, b = mp[-2], mp[-1]
                if abs((b[0] - a[0]) * (pb[1] - b[1]) - (b[1] - a[1]) * (pb[0] - b[0])) < 1e-6: mp[-1] = pb; continue
            mp.append(pb); mw.append(wv)
        if len(mp) >= 2:
            out_tr.append(dict(layer=LAYERS[li], pts=[[round(x, 4), round(y, 4)] for x, y in mp], w=mw))
        if k < len(runs) - 1:
            vx, vy = pts[-1]; out_v.append(dict(x=round(vx, 4), y=round(vy, 4)))
    return out_tr, out_v


SW_DIST = float(args.get('sw_dist', '0.8' if NC else '0'))
_SWN = [n for n in nets if n.split('/')[-1].startswith('SW_')]
SEPARATE = float(args.get('separate', '0.5'))
if NC:
    AGG_SW = {n for n, k in NC['net_list'].items() if k['category'] == 'sw'}
    AGG_FAST = {n for n, k in NC['net_list'].items() if k['category'] == 'fast' and k['rank'] >= 3}
    VICT_SW = {n for n, k in NC['net_list'].items() if k.get('sensitive_sw')}
    VICT_A = {n for n, k in NC['net_list'].items() if k['category'] == 'analog' and k['rank'] >= 2}
    if DISTURB_FLANK:
        AGG_SW = set(DISTURB_LIST['sw'] + DISTURB_LIST['current_val'])
        AGG_FAST = set(DISTURB_LIST['digital'])
        VICT_SW = VICT_A = set(DISTURB_LIST['victim'])
_COILS_M = None
if DISTURB_FLANK:
    _COILS_M = np.zeros(NL * NN, dtype=bool)
    for c in D.get('courtyards', []):
        if c['ref'] in DISTURB_LIST['coils'] and c.get('side') in LI:
            li = LI[c['side']]; _COILS_M[li * NN:(li + 1) * NN] |= raster_polys(c['polys']).ravel()


def _edges_groups():
    out = {}
    for n in sorted(AGG_FAST):
        f = DISTURB_LIST['net_list'][n]['spacing_factor']
        out.setdefault(f, set()).add(n)
    return sorted(out.items())


def _separate_mm(net):
    return SEPARATE * DISTURB_LIST['net_list'][net]['spacing_factor'] if DISTURB_FLANK and net in AGG_FAST else SEPARATE


def _mask_arr(nets):
    m = np.zeros(NL * NN, dtype=bool)
    for p in D['pads']:
        if p['net'] in nets:
            for L, c in p['cells'].items(): m[LI[L] * NN + c] = True
    for u, r in U.items():
        if r['net'] in nets: m[r['cells']] = True
    return m


def _nearby(m, dist, fac):
    ex = np.ones(NL * NN, dtype=np.float32)
    if NEIGHBOR_LAYER:
        dl = {}
        for li in range(NL):
            sl = slice(li * NN, (li + 1) * NN)
            if m[sl].any(): dl[li] = (_edt(~m[sl].reshape(NY, NX)) * RES).ravel()
        for li in range(NL):
            close = dl[li] < dist if li in dl else None
            for lj, s in neighbor_spacing(li, dist):
                if lj in dl: close = (dl[lj] < s) if close is None else (close | (dl[lj] < s))
            if close is not None: ex[li * NN:(li + 1) * NN] = np.where(close, fac, 1.0)
        return ex
    for li in range(NL):
        sl = slice(li * NN, (li + 1) * NN)
        if not m[sl].any(): continue
        d = _edt(~m[sl].reshape(NY, NX)) * RES
        ex[sl] = np.where(d.ravel() < dist, fac, 1.0)
    return ex


_NEARBY_C = {}


R41_GAP = float(args.get('r41_gap', '0'))
if not math.isfinite(R41_GAP) or R41_GAP < 0: raise ValueError('r41_gap must be finite and >= 0')
R41_FAST, R41_NON_DC, R41_PARAM = set(), set(), None
if R41_GAP and NC is not None and NLAY >= 4:
    from analysis.r41 import net_list as _r41_nets, parameter as _r41_parameter
    R41_FAST, R41_NON_DC = _r41_nets(NC)
    R41_PARAM = _r41_parameter(NC, dict(args, via='%g/%g' % (VIA_D, VIA_DR)))
    print('R41 via room (option):', R41_PARAM, 'Price', R41_GAP, flush=True)
_R41_C = {}


def r41_extra(net):
    if R41_PARAM is None or net not in R41_NON_DC: return None
    nets = (R41_NON_DC if net in R41_FAST else R41_FAST) - {net}
    if net in PAIR_NETS: nets = nets - {PAIR_NETS[net]}
    sig = tuple((n, _NVER.get(n, 0)) for n in sorted(nets))
    key = (net, sig)
    if key in _R41_C: return _R41_C[key]

    if len(_R41_C) >= 4: _R41_C.pop(next(iter(_R41_C)))
    ex = np.ones(NL * NN, dtype=np.float32)
    geometries = [[] for _ in range(NL)]
    for unit in U.values():
        if unit['net'] not in nets: continue
        for t in unit['tracks']:
            li = LI[t['layer']]
            for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                if a != b: geometries[li].append(_LS([a,b]).buffer(w/2))
    from shapely.ops import unary_union as _r41_union
    for li, gg in enumerate(geometries):
        if not gg: continue
        w = z_width(net, li, cls(net)['w']) if net in IMP_TAB else cls(net)['w']


        band = _r41_union([g.buffer(w/2 + R41_PARAM['via_slot']).difference(
            g.buffer(w/2 + R41_PARAM['gap_from'])) for g in gg])

        x0,y0,x1,y1 = band.bounds if not band.is_empty else (0,0,0,0)
        xa,xb = max(0,int((x0-X0)/RES)),min(NX,int((x1-X0)/RES)+2)
        ya,yb = max(0,int((y0-Y0)/RES)),min(NY,int((y1-Y0)/RES)+2)
        if xa >= xb or ya >= yb: continue
        xx,yy = np.meshgrid(GXc[xa:xb], GYc[ya:yb])
        mask = shapely.contains_xy(band,xx,yy)
        ex[li*NN:(li+1)*NN].reshape(NY,NX)[ya:yb,xa:xb][mask] = 1.0 + R41_GAP
    _R41_C[key] = ex
    return ex


COPPER_RESERVE = max(0.0, float(args.get('copper_reserve', '0'))) if NC else 0.0
_COPPER_RESERVE_C = {}


def copper_reserve_extra(net):
    if not COPPER_RESERVE:
        return None
    ex = np.ones(NL * NN, dtype=np.float32)
    for n, k in sorted(NC['net_list'].items()):
        if n == net or k.get('category') not in ('current_val', 'supply', 'sw'):
            continue
        version = _NVER.get(n, 0)
        cached = _COPPER_RESERVE_C.get(n)
        if cached is None or cached[0] != version:
            groups = {}
            for r in U.values():
                if r['net'] != n:
                    continue
                for t in r['tracks']:
                    if t.get('sig') or t.get('kelvin'):
                        continue
                    li = LI[t['layer']]
                    goal = cls(n)['w'] * f_inner(n, li)
                    for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                        if goal <= w + 1e-6:
                            continue
                        band = math.ceil(((goal - w) / 2 + CLR) / RES) * RES
                        groups.setdefault((li, band), []).append(seg_cells(li, a, b, w))
            maps = []
            for (li, band), cells in sorted(groups.items()):
                idx = np.unique(np.concatenate(cells)) - li * NN
                if not len(idx):
                    continue
                xx, yy = idx % NX, idx // NX
                pad = math.ceil(band / RES) + 2
                xa, xb = max(0, int(xx.min()) - pad), min(NX, int(xx.max()) + pad + 1)
                ya, yb = max(0, int(yy.min()) - pad), min(NY, int(yy.max()) + pad + 1)
                mask = np.zeros((yb - ya, xb - xa), dtype=bool)
                mask[yy - ya, xx - xa] = True
                d = _edt(~mask) * RES
                cost = (1 + COPPER_RESERVE * np.clip(1 - d / band, 0, 1)).astype(np.float32)
                maps.append((li, xa, xb, ya, yb, cost))
            cached = (version, maps)
            _COPPER_RESERVE_C[n] = cached
        for li, xa, xb, ya, yb, cost in cached[1]:
            target = ex[li * NN:(li + 1) * NN].reshape(NY, NX)[ya:yb, xa:xb]
            np.maximum(target, cost, out=target)
    return ex


def _nearby_c(name, nets, dist, fac, body=False):
    key = (name, frozenset(nets), dist, fac, body) if DISTURB_FLANK else name
    sig = tuple((n, _NVER.get(n, 0)) for n in sorted(nets)) if DISTURB_FLANK else sum(_NVER.get(n, 0) for n in nets)
    c = _NEARBY_C.get(key)
    if c is None or c[0] != sig:
        m = _mask_arr(nets)
        if body and _COILS_M is not None: m |= _COILS_M
        c = (sig, _nearby(m, dist, fac)); _NEARBY_C[key] = c
    return c[1]

def si_spacing(victim, aggressor):


    from analysis.signal_integrity import spec, count
    budget = count(spec(NC, victim, 'noise_budget_mV'), 1.0, 1e-9) / 1000
    swing = count(spec(NC, aggressor, 'hub_V'), 12.0 if aggressor in AGG_SW else 3.3)
    hs = [a['H'] for a in STRUCTURE if a['H']]
    h = max(hs) if hs else 1.5
    return min(float(args.get('si_spacing_max', '3.0')), h * math.sqrt(max(0.0, swing / budget - 1)))


def si_disturb_extra(net, ex):
    if not SI: return ex
    victim = {n for n, k in NC['net_list'].items() if k.get('category') in ('analog', 'kelvin') or k.get('kelvin')}
    agg = AGG_FAST | AGG_SW
    if net in victim:
        group_list = {}
        for ag in sorted(agg):
            d = round(si_spacing(net, ag), 3)
            group_list.setdefault(d, set()).add(ag)
    elif net in agg:
        group_list = {}
        for v in sorted(victim):
            d = round(si_spacing(v, net), 3)
            group_list.setdefault(d, set()).add(v)
    else: return ex
    for d, ns in sorted(group_list.items()):
        if d <= 0: continue
        e = _nearby_c(('si', tuple(sorted(ns)), d), ns, d, 4.0)
        ex = e if ex is None else np.maximum(ex, e)
    return ex


def disturb_extra(net):
    if NC is None: return sw_extra(net)
    ex = None
    if net in VICT_SW and SW_DIST > 0: ex = _nearby_c('sw', AGG_SW - {net} if DISTURB_FLANK else AGG_SW, SW_DIST, 6.0, body=DISTURB_FLANK)
    if net in VICT_A and SEPARATE > 0:
        if DISTURB_FLANK:
            e2 = np.ones(NL * NN, dtype=np.float32)
            for f, ns in _edges_groups(): e2 = np.maximum(e2, _nearby_c(('flank', f), ns - {net}, SEPARATE * f, 3.0))
        else: e2 = _nearby_c('fast', AGG_FAST, SEPARATE, 3.0)
        ex = e2 if ex is None else ex * e2
    if net in AGG_FAST and SEPARATE > 0:
        e2 = _nearby_c(('analog', _separate_mm(net)), VICT_A - {net} if DISTURB_FLANK else VICT_A, _separate_mm(net), 3.0); ex = e2 if ex is None else ex * e2
    if PAIR_NETS and PAIR_DISTURB > 1 and net not in PAIR_NETS:
        e2 = _nearby_c('pair', set(PAIR_NETS), pair_free_mm(net), PAIR_DISTURB); ex = e2 if ex is None else ex * e2
    if COPPER_RESERVE:
        e2 = copper_reserve_extra(net)
        ex = e2 if ex is None else ex * e2
    ex = si_disturb_extra(net, ex)
    if ZONE_PLAN:
        zex = _zp_maps(net)[1]
        ex = zex if ex is None else ex * zex
    if R41_PARAM is not None:
        e2 = r41_extra(net)
        if e2 is not None: ex = e2 if ex is None else ex * e2
    if ex is not None and DISTURB_FACTOR[0] != 1.0:
        ex = np.where(ex > 1.0, ex * np.float32(DISTURB_FACTOR[0]), ex).astype(ex.dtype)
    return ex



def pair_unit(u):
    return PAIR_SEARCH_SPACE and pair_fast(u)


def _pair_tolerances(k):
    if not PAIR_SEARCH_SPACE: return k
    nets=k['net_list'];seen=set()
    explicit=(k.get('specs') or {}).get('length_groups') or {}
    for n,x in nets.items():
        p=x.get('pair')
        if not p or p not in nets or n in seen or not pair_fast(n):continue
        seen.update((n,p));name='pair:'+'|'.join(sorted((n,p)))
        ts=[nets[q].get('pair_tolerance_mm',None if nets[q].get('length_group') else nets[q].get('tolerance_mm')) for q in (n,p)]
        ts=[float(t) for t in ts if t is not None]
        tol=explicit.get(name,{}).get('tolerance_mm')
        tol=float(tol) if tol is not None else min(ts) if ts else .15
        if not math.isfinite(tol) or tol<=0:raise ValueError('Pair skew tolerance must be finite and positive')
        for q in (n,p):nets[q]['pair_tolerance_mm']=tol
    return k


def _pair_fixed_verify(u):
    fixed = [n for n in (u, PAIR_NETS[u]) if 'FIX#' + n in FIXED]
    if fixed:
        raise ValueError('pair_search_space=1: fixed pair conductors %s; route both partners freely or supply the pair completely with fix=' % ', '.join(sorted(fixed)))


def pair_nets(series):
    out = set(series)
    out.update(PAIR_NETS[u] for u in series if pair_unit(u))
    return out


def pair_series(series):
    if not PAIR_SEARCH_SPACE: return list(series)
    out = []; seen = set()
    for u in series:
        if pair_unit(u): _pair_fixed_verify(u)
        key = min(u, PAIR_NETS[u]) if pair_unit(u) else u
        if key not in seen: seen.add(key); out.append(key)
    return sorted(out, key=lambda u: (0 if pair_unit(u) else 1, out.index(u)))


def _pair_ground(a, b, li):
    wa, wb = z_width(a, li, cls(a)['w']), z_width(b, li, cls(b)['w'])
    gap = max(z_gap(a, li), z_gap(b, li))
    if not all(math.isfinite(v) and v > 0 for v in (wa, wb, gap)):
        raise ValueError('Pair geometry must be finite and positive')
    return wa, wb, gap, (wa + wb) / 2 + gap, wa + wb + gap


def _pair_pad_groups(a, b):
    from scipy.optimize import linear_sum_assignment
    aa = [D['pads'][k] for k in by_net.get(a, [])]
    bb = [D['pads'][k] for k in by_net.get(b, [])]
    groups = []
    for ref in sorted({p['ref'] for p in aa + bb}):
        ap = sorted([p for p in aa if p['ref'] == ref], key=lambda p: str(p['num']))
        bp = sorted([p for p in bb if p['ref'] == ref], key=lambda p: str(p['num']))
        if len(ap) != len(bp): return None
        if not ap: continue
        costs = np.array([[math.dist((p['x'],p['y']),(q['x'],q['y'])) for q in bp] for p in ap])
        ii,jj = linear_sum_assignment(costs)
        for i,j in zip(ii,jj):
            p,q = ap[i],bp[j]
            layers = sorted(set(p['layers']) & set(q['layers']) & set(LAYERS), key=LI.get)
            if not layers: return None
            groups.append((p,q,layers))
    return groups if len(groups) >= 2 else None


def _pair_zp_ok(net, li, geom):
    if not ZONE_PLAN: return True
    spec = ZONE_PLAN['net_list'].get(net, {})
    if spec.get('local_without_layer_change') and LAYERS[li] != spec.get('local_layer'): return False
    return all(geom.distance(g) >= (_ka_clr(net,n,li,'Track','Zone')+.005 if globals().get('KA') is not None else CLR+.005) - 1e-9 for (n, L), g in _ZP_GEO.items() if n != net and L == LAYERS[li])


def _pair_zp_via_ok(net, x, y):
    if not ZONE_PLAN: return True
    ix, iy = math.floor((x - X0) / RES), math.floor((y - Y0) / RES)
    if not (0 <= ix < NX and 0 <= iy < NY): return False
    return bool(_zp_viah(net, np.ones((1, 1), dtype=bool), slice(iy, iy + 1), slice(ix, ix + 1))[0, 0])


def _pair_hard(a, b):
    maps = []; soft = []
    cnt = use.copy()
    for r in U.values():
        if r['net'] in (a,b): cnt[r['cells']] -= 1
    for li in range(NL):
        heights = [IMP_TAB.get(n,{}).get(LAYERS[li],{}).get('profile_data',{}).get('h') for n in (a,b)]
        heights += [STRUCTURE[li]['h'] if STRUCTURE[li]['ref_net'] in GNDS else None]
        outside = max(KA.maximum if globals().get('KA') is not None else CLR, 3*max(h for h in heights if h) if any(heights) else PAIR_FREE)
        mask = np.zeros((NY,NX), dtype=bool)
        for p in D['pads']:
            if p['net'] not in (a,b) and LAYERS[li] in p['cells']:
                mask.ravel()[p['cells'][LAYERS[li]]] = True
        for u,r in U.items():
            if u.startswith(('FIX#','TACK#')) and r['net'] not in (a,b):
                cells = r['cells']; cells = cells[cells//NN == li] % NN
                mask.ravel()[cells] = True
        dist = _edt(~mask)*RES if mask.any() else np.full((NY,NX), 1e6)
        h = np.minimum(2*(dist-outside-QM), 2*(edge_d-EDGE))

        for net in (a,b):
            ref = IMP_REF_MASK.get((net,LAYERS[li]))
            if ref is not None:
                rd = _edt(ref)*RES
                h = np.minimum(h,2*(rd-QM))

        if ZONE_PLAN: h = _zp_hard(b, _zp_hard(a, h, li), li)
        maps.append(h.astype(np.float32))
        occupied = cnt[li*NN:(li+1)*NN].reshape(NY,NX)>0
        d = _edt(~occupied)*RES if occupied.any() else np.full((NY,NX),1e6)
        soft.append((2*(d-outside-QM)).astype(np.float32))
    if HARD_FOREIGN[0]: maps = [np.minimum(h,s) for h,s in zip(maps,soft)]
    return maps, soft


def _pair_line_ok(net, li, pts, w, other=None):
    if globals().get('KA') is not None:
        if any(_ka_room(net,li,a,b,True)<w/2 for a,b in zip(pts[:-1],pts[1:])):return False
        if other is not None and _LS(pts).buffer(w/2).distance(other)<_ka_margin(net,PAIR_NETS.get(net,net),li):return False
    line = _LS(pts)
    if not line.is_simple: return False
    copper = line.buffer(w/2)
    board = border_polygon(D).buffer(-float(args.get('edge','0.3')))
    if not board.covers(copper): return False
    for nn,g in _PADG[li]:
        if nn != net and copper.distance(g) < (_ka_geo_limit(net,nn,li,g2=g,pad=True) if globals().get('KA') is not None else DRC_PADC)-1e-6: return False
    for u,r in U.items():
        if r['net'] == net: continue
        if not u.startswith(('FIX#','TACK#')) and not HARD_FOREIGN[0]: continue
        for t in r['tracks']:
            if LI[t['layer']] != li: continue
            for x,y,ww in zip(t['pts'][:-1],t['pts'][1:],t['w']):
                if copper.distance(_LS([x,y]).buffer(ww/2)) < (_ka_margin(net,r['net'],li) if globals().get('KA') is not None else DRC_CLR)-1e-6: return False
        for v in r['vias']:
            if copper.distance(Point(v['x'],v['y']).buffer(VIA_R)) < (_ka_margin(net,v.get('net') or r['net'],li,type2='Via') if globals().get('KA') is not None else DRC_CLR)-1e-6: return False
    if net in IMP_TAB and any(not z_ref_free(cls(net),li,x,y,net) for x,y in zip(pts[:-1],pts[1:])): return False
    if globals().get('KA') is None and other is not None and copper.distance(other) < DRC_CLR-1e-6: return False
    if not _pair_zp_ok(net, li, copper): return False
    return True


def _pair_pad_escape(net,pad,end,li,w,direction,other=None):
    if time_tight():return None
    sx,sy=pad['x'],pad['y']
    xa=max(1,int((min(sx,end[0])-.8-X0)/RES));xb=min(NX-2,int((max(sx,end[0])+.8-X0)/RES))
    ya=max(1,int((min(sy,end[1])-.8-Y0)/RES));yb=min(NY-2,int((max(sy,end[1])+.8-Y0)/RES))
    nx,ny=xb-xa+1,yb-ya+1
    if nx<2 or ny<2:return None
    cap=hard_maps(net)[0][li][ya:yb+1,xa:xb+1].copy()
    yy,xx=np.indices((ny,nx));px=X0+(xx+xa+.5)*RES;py=Y0+(yy+ya+.5)*RES
    foreign=shapely.union_all([g for nn,g in _PADG[li] if nn!=net])
    if not foreign.is_empty:
        exact=shapely.distance(shapely.points(px,py),foreign)
        cap=np.minimum(cap,2*(exact-DRC_PADC-QM))
    if other is not None and not other.is_empty:
        distance=shapely.distance(shapely.points(px,py),other)
        cap=np.minimum(cap,2*(distance-DRC_CLR-.005))
    ix,iy=int((sx-X0)/RES)-xa,int((sy-Y0)/RES)-ya
    tx,ty=int((end[0]-X0)/RES)-xa,int((end[1]-Y0)/RES)-ya
    if not (0<=ix<nx and 0<=iy<ny and 0<=tx<nx and 0<=ty<ny):return None
    dirs=np.array(((1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1)),np.int64)
    goals=np.zeros(nx*ny,np.int64)
    for j,(dx,dy) in enumerate(dirs):
        if (dx*direction[0]+dy*direction[1])/math.hypot(dx,dy)>=math.sqrt(.5)-1e-7:goals[ty*nx+tx]|=1<<j
    rest=(RES*np.hypot(xx-tx,yy-ty)).ravel()
    path,count=_PAIR_CORE(np.full(8,iy*nx+ix,np.int64),np.arange(8,dtype=np.int64),np.zeros(8),goals,
        cap.reshape(1,-1),np.ones((1,nx*ny)),np.zeros((8,nx*ny),np.bool_),np.zeros(nx*ny,np.bool_),np.array([w]),dirs,
        nx,ny,RES,0,0,nx-1,ny-1,0,np.ones(8,np.int64),12.,15000,rest,np.ones((1,8,nx*ny),np.bool_))
    if not len(path):return None
    raster=[(X0+(xa+n%nx+.5)*RES,Y0+(ya+n//nx+.5)*RES) for n in path]
    own=shapely.union_all([Polygon(poly) for poly in pad.get('polys',{}).get(LAYERS[li],[])])
    while raster and not own.is_empty and own.covers(Point(*raster[0])):raster.pop(0)
    pts=[(sx,sy)]+raster+[end]
    pts=_oct_move(pts)

    out=[pts[0]]
    for j in range(1,len(pts)-1):
        a,b,c=pts[j-1:j+2]
        if abs((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))>1e-8:out.append(b)
    out.append(pts[-1]);out=[list(p) for j,p in enumerate(out) if j==0 or math.dist(p,out[j-1])>1e-8]
    if len(out)<2:return None
    if not _pair_line_ok(net,li,out,w,other):return None
    return out


def _pair_connection(p, q, li, center, direction, side, a, b, copper=None):
    wa,wb,gap,d,total = _pair_ground(a,b,li)
    norm = (-direction[1],direction[0])
    foreign=[None,None] if copper is None else [copper[1][li],copper[0][li]]
    ends = [(center[0]+side*(wb+gap)/2*norm[0],center[1]+side*(wb+gap)/2*norm[1]),
            (center[0]-side*(wa+gap)/2*norm[0],center[1]-side*(wa+gap)/2*norm[1])]
    def paths(pad,end):
        start = (pad['x'],pad['y'])
        choices = [[start,end]] if oct_(start,end) else [[start,k,end] for k in _corners(start,end)]

        choices=[pp for pp in choices if math.dist(pp[-2],pp[-1])<1e-8 or (pp[-1][0]-pp[-2][0])*direction[0]+(pp[-1][1]-pp[-2][1])*direction[1]>=-1e-8]
        return [[list(x) for i,x in enumerate(pp) if i == 0 or math.dist(x,pp[i-1])>1e-8] for pp in choices]
    for pa in paths(p,ends[0]):
        if len(pa)<2: pa=[pa[0],pa[0]]
        if not _pair_line_ok(a,li,pa,wa,foreign[0]): continue
        for pb in paths(q,ends[1]):
            if len(pb)<2: pb=[pb[0],pb[0]]
            if _pair_line_ok(b,li,pb,wb,shapely.union_all([_LS(pa).buffer(wa/2)]+([foreign[1]] if foreign[1] is not None else []))):
                return [dict(layer=LAYERS[li],pts=pa,w=[wa]*(len(pa)-1)),
                        dict(layer=LAYERS[li],pts=pb,w=[wb]*(len(pb)-1))]


    for first in (0,1):
        pads=(p,q);nets=(a,b);widths=(wa,wb);second=1-first
        paths=[None,None]
        paths[first]=_pair_pad_escape(nets[first],pads[first],ends[first],li,widths[first],direction,foreign[first])
        if paths[first] is None:continue
        paths[second]=_pair_pad_escape(nets[second],pads[second],ends[second],li,widths[second],direction,shapely.union_all([_LS(paths[first]).buffer(widths[first]/2)]+([foreign[second]] if foreign[second] is not None else [])))
        if paths[second] is not None:
            return [dict(layer=LAYERS[li],pts=pp,w=[widths[k]]*(len(pp)-1)) for k,pp in enumerate(paths)]
    return None


def _pair_ports(group, a, b, hard, side, copper=None):
    p,q,layers = group
    middle = ((p['x']+q['x'])/2,(p['y']+q['y'])/2)
    ports = {}
    for layer in layers:
        li = LI[layer]; _,_,_,_,width = _pair_ground(a,b,li)
        for dx,dy,_ in OFFS:
            length = math.hypot(dx,dy); direction = (dx/length,dy/length)

            for step in (0.,.2,.4,.6,.8,1.0,1.4,2.0,2.6,3.2,4.0):
                c = (middle[0]+step*direction[0],middle[1]+step*direction[1])
                ix,iy = int((c[0]-X0)/RES),int((c[1]-Y0)/RES)
                if not (1<=ix<NX-1 and 1<=iy<NY-1) or hard[li][iy,ix]<width: continue
                center = cell_xy(ix,iy)
                left = _pair_connection(p,q,li,center,direction,side,a,b,copper)
                if left is not None:
                    node = li*NN+iy*NX+ix
                    cost = sum(math.dist(x,y) for t in left for x,y in zip(t['pts'][:-1],t['pts'][1:]))
                    ports.setdefault(node,[]).append((cost,left,direction))
    if not ports: return ports
    shortest = min(v[0] for vv in ports.values() for v in vv)
    return {n:[v for v in vv if v[0]<=shortest+.3] for n,vv in ports.items() if any(v[0]<=shortest+.3 for v in vv)}



def _pair_states(a,b,side,tracks,vias):
    yy,xx=np.indices((NY,NX));px=(X0+(xx+.5)*RES).ravel();py=(Y0+(yy+.5)*RES).ravel()
    copper=[[None]*NL for _ in range(2)];allowed=np.ones((NL,8,NN),np.bool_)
    for k,net in enumerate((a,b)):
        for li in range(NL):
            gg=[_LS(t['pts']).buffer(t['w'][0]/2) for t in tracks[k] if LI[t['layer']]==li]
            gg += [Point(v['x'],v['y']).buffer(VIA_R) for v in vias[k]]
            gg += [g for nn,g in _PADG[li] if nn==net]
            copper[k][li]=shapely.union_all(gg)
    for li in range(NL):
        wa,wb,gap,_,_=_pair_ground(a,b,li)
        for k,sign,offset,w in ((0,side,(wb+gap)/2,wa),(1,-side,(wa+gap)/2,wb)):
            other=copper[1-k][li]
            if other.is_empty:continue


            near=np.flatnonzero(shapely.contains_xy(other.buffer(offset+w/2+DRC_CLR+.01),px,py))
            for j,(dx,dy) in enumerate(((1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1))):
                norm=(-dy/math.hypot(dx,dy),dx/math.hypot(dx,dy))
                positions=shapely.points(px[near]+sign*offset*norm[0],py[near]+sign*offset*norm[1])
                allowed[li,j,near] &= shapely.distance(positions,other)>=w/2+DRC_CLR-1e-7
    return allowed,copper


def _pair_tree_free(tracks,vias):
    for li in range(NL):
        gg=[]
        for k in range(2):
            shapes=[_LS(t['pts']).buffer(t['w'][0]/2) for t in tracks[k] if LI[t['layer']]==li]
            shapes += [Point(v['x'],v['y']).buffer(VIA_R) for v in vias[k]]
            gg.append(shapely.union_all(shapes))
        if gg[0].distance(gg[1])<(_ka_margin(a,b,li) if globals().get('KA') is not None else DRC_CLR)-1e-6:return False
        for k in range(2):
            for v in vias[k]:
                if Point(v['x'], v['y']).distance(gg[1-k]) < VIA_DR/2 + HOLE_CLR - 1e-6:
                    return False
    allvias = [(k, v) for k in range(2) for v in vias[k]]
    for i, (k, v) in enumerate(allvias):
        for qk, q in allvias[i+1:]:
            dist=math.dist((v['x'],v['y']),(q['x'],q['y']))
            if k == qk and dist <= 1e-8: continue
            if dist < VIA_DR+H2H+.012: return False
    return True


def _pair_via_mask(a,b,hard,side=1,copper=None):
    distance=max(VIA_D+DRCC+.012,VIA_DR+H2H+.012,VIA_R+VIA_DR/2+HOLE_CLR+.012,max(_pair_ground(a,b,li)[3] for li in range(NL)))
    yy,xx=np.indices((NY,NX));out=np.zeros((8,NN),np.bool_)

    zp=[_zp_viah(n,np.ones((NY,NX),np.bool_)) for n in (a,b)] if ZONE_PLAN else None
    for j,(dx,dy) in enumerate(((1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1))):
        norm=(-dy/math.hypot(dx,dy),dx/math.hypot(dx,dy));ok=np.ones((NY,NX),np.bool_)
        for sign in (-1,1):
            ix=np.rint(xx+sign*distance/2/RES*norm[0]).astype(int);iy=np.rint(yy+sign*distance/2/RES*norm[1]).astype(int)
            inside=(ix>=0)&(ix<NX)&(iy>=0)&(iy<NY);ix=np.clip(ix,0,NX-1);iy=np.clip(iy,0,NY-1)
            ok &= inside & (VSP[iy,ix]>=H2H+VIA_DR/2+.01)
            if VIA_BAN is not None:ok &= ~VIA_BAN[iy,ix]
            for h in hard:ok &= h[iy,ix]>=2*VIA_KEEP_R
            k=0 if sign==side else 1
            if zp is not None:ok &= zp[k][iy,ix]
            if copper is not None:
                px=X0+(xx+.5)*RES;py=Y0+(yy+.5)*RES
                for li in range(NL):
                    other=copper[1-k][li]
                    if other.is_empty:continue
                    near=shapely.contains_xy(other.buffer(distance/2+VIA_R+max(DRC_CLR,DRC_PADC)+.01),px,py)&ok
                    rows,cols=np.nonzero(near)
                    points=shapely.points(px[rows,cols]+sign*distance/2*norm[0],py[rows,cols]+sign*distance/2*norm[1])
                    ok[rows,cols] &= shapely.distance(points,other)>=VIA_R+max(DRC_CLR,DRC_PADC)
        out[j]=ok.ravel()
    return out


def _pair_remaining_cost(tgt,hard,widths,mult,viah,nx,ny,res,xmin,ymin,xmax,ymax,via_cost):
    nl=hard.shape[0];nn=nx*ny
    out=np.full(nl*nn,np.inf);queue=[(0.,np.int64(0))];queue.pop()
    for n in range(len(tgt)):
        if tgt[n]:out[n]=0.;heapq.heappush(queue,(0.,np.int64(n)))
    dxs=(1,1,0,-1,-1,-1,0,1);dys=(0,1,1,1,0,-1,-1,-1)
    while queue:
        cost,raw=heapq.heappop(queue);n=int(raw)
        if cost>out[n]+1e-10:continue
        li=n//nn;idx=n%nn;iy=idx//nx;ix=idx%nx
        for j in range(8):
            x=ix+dxs[j];y=iy+dys[j]
            if x<xmin or x>xmax or y<ymin or y>ymax:continue
            ii=y*nx+x;m=li*nn+ii
            if hard[li,ii]<widths[li]:continue
            val=cost+res*math.hypot(dxs[j],dys[j])*mult[li,idx]
            if val<out[m]:out[m]=val;heapq.heappush(queue,(val,np.int64(m)))


        if np.any(viah[:,idx]):
            for lj in range(nl):
                if lj==li or hard[lj,idx]<widths[lj]:continue
                m=lj*nn+idx;val=cost+via_cost
                if val<out[m]:out[m]=val;heapq.heappush(queue,(val,np.int64(m)))
    return out


def _pair_search_core(src_nodes, src_dirs, src_cost, tgt_dirs, hard, mult, viah, viabad, widths,
                      offsets, nx, ny, res, xmin, ymin, xmax, ymax, leadcap, via_steps, via_cost, maxexpand, rest, allowed, require_via=False):
    layers=hard.shape[0];nn=nx*ny;wx=xmax-xmin+1;wy=ymax-ymin+1;wn=wx*wy
    base_stride=8*(leadcap+1);stride=base_stride*(2 if require_via else 1)


    g={np.int64(-1):np.float64(np.inf)}
    parent={np.int64(-1):np.int64(-2)};vcenter={np.int64(-1):np.int64(-1)}
    closed={np.int64(-1):np.int64(0)}
    queue=[(0.0,np.int64(0))];queue.pop()
    if not np.any(tgt_dirs):return np.empty(0,np.int64),0
    for i in range(len(src_nodes)):
        n=src_nodes[i];li=n//nn;iy=n%nn//nx;ix=n%nn%nx
        if ix<xmin or ix>xmax or iy<ymin or iy>ymax:continue
        if not allowed[li,src_dirs[i],n%nn]:continue
        local=li*wn+(iy-ymin)*wx+ix-xmin
        state=local*stride+src_dirs[i]*(leadcap+1)
        if state not in g or src_cost[i]<g[state]:
            g[state]=src_cost[i];parent[state]=-1
            heapq.heappush(queue,(src_cost[i]+rest[n],state))
    expanded=0;goal=-1
    while queue:
        _,rawstate=heapq.heappop(queue);state=int(rawstate)
        if state in closed:continue
        closed[state]=1;expanded+=1
        if expanded>maxexpand:break
        local=state//stride;phase=(state%stride)//base_stride;direction=(state%base_stride)//(leadcap+1);lead=state%(leadcap+1)
        li=local//wn;idx=local%wn;iy=idx//wx+ymin;ix=idx%wx+xmin;n=li*nn+iy*nx+ix
        if (not require_via or phase==1) and lead>=leadcap and tgt_dirs[n] & (1<<direction):goal=state;break
        for turn in (-1,0,1):
            if turn and lead<leadcap:continue
            nd=(direction+turn)%8;dx=int(offsets[nd,0]);dy=int(offsets[nd,1]);jx,jy=ix+dx,iy+dy
            if jx<xmin or jx>xmax or jy<ymin or jy>ymax:continue
            nnidx=jy*nx+jx
            if hard[li,nnidx]<widths[li] or not allowed[li,nd,n%nn] or not allowed[li,nd,nnidx]:continue
            nl=min(leadcap,lead+1) if turn==0 else 0
            dest=(li*wn+(jy-ymin)*wx+jx-xmin)*stride+phase*base_stride+nd*(leadcap+1)+nl
            cost=g[state]+res*math.hypot(dx,dy)*mult[li,nnidx]+(.025 if turn else 0.)
            if dest in g and cost>=g[dest]:continue
            g[dest]=cost;parent[dest]=state;vcenter[dest]=-1
            heapq.heappush(queue,(cost+rest[li*nn+nnidx],dest))
        if lead<leadcap:continue
        dx=int(offsets[direction,0]);dy=int(offsets[direction,1]);vs=int(via_steps[direction])
        cx,cy=ix+vs*dx,iy+vs*dy;jx,jy=ix+2*vs*dx,iy+2*vs*dy
        if jx<xmin or jx>xmax or jy<ymin or jy>ymax:continue
        ci=cy*nx+cx;end=jy*nx+jx
        if not viah[direction,ci]:continue
        for lj in range(layers):
            if lj==li:continue
            free=True;cost=g[state]+via_cost*(1.+3.*viabad[ci])
            for step in range(1,2*vs+1):
                ii=(iy+step*dy)*nx+ix+step*dx;ll=li if step<=vs else lj
                if hard[ll,ii]<widths[ll] or not allowed[ll,direction,ii]:free=False;break
                cost+=res*math.hypot(dx,dy)*mult[ll,ii]
            if not free:continue
            dest=(lj*wn+(jy-ymin)*wx+jx-xmin)*stride+(base_stride if require_via else 0)+direction*(leadcap+1)+leadcap
            if dest in g and cost>=g[dest]:continue
            g[dest]=cost;parent[dest]=state;vcenter[dest]=ci
            heapq.heappush(queue,(cost+rest[lj*nn+end],dest))
    if goal<0:return np.empty(0,np.int64),expanded
    reverse=[];state=goal
    while state>=0:
        local=state//stride;li=local//wn;idx=local%wn;iy=idx//wx+ymin;ix=idx%wx+xmin
        reverse.append(li*nn+iy*nx+ix)
        prev=parent[state]
        if state in vcenter and vcenter[state]>=0 and prev>=0:
            pli=prev//stride//wn
            reverse.append(li*nn+vcenter[state]);reverse.append(pli*nn+vcenter[state])
        state=prev
    return np.asarray(reverse[::-1],dtype=np.int64),expanded


_PAIR_REST = _pair_remaining_cost
_PAIR_CORE = _pair_search_core
if NB is not None:
    from numba import njit as _pair_njit
    _PAIR_REST = _pair_njit(cache=True)(_pair_remaining_cost)
    _PAIR_CORE = _pair_njit(cache=True)(_pair_search_core)


def _pair_search(src,tgt,hard,soft,viaH,viaBad,C):
    directions=np.array([(1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1)],dtype=np.int64)
    def heading(d):return min(range(8),key=lambda i: abs(d[0]-directions[i,0]/math.hypot(*directions[i]))+abs(d[1]-directions[i,1]/math.hypot(*directions[i])))
    entries=[(n,v) for n in sorted(src) for v in src[n]]
    sn=np.array([n for n,v in entries],dtype=np.int64);sd=np.array([heading(v[2]) for n,v in entries],dtype=np.int64)
    sc=np.array([v[0]/2 for n,v in entries],dtype=np.float64)
    td=np.zeros(NL*NN,dtype=np.int64)
    for n,vv in tgt.items():
        for v in vv: td[n] |= 1<<((heading(v[2])+4)%8)
    h=np.asarray([v.ravel() for v in hard]);mult=[]
    for li in range(NL):
        c=np.minimum(hard[li],soft[li]);width=C['zw'][li]
        m=(1.+PF[0]*(c<width))*(1.+hist[li*NN:(li+1)*NN].reshape(NY,NX))*C['lf'][li]
        if C.get('pair_tree') is not None: m=np.where(C['pair_tree'][li*NN:(li+1)*NN].reshape(NY,NX),m*.2,m)
        mult.append(m.ravel())
    mult=np.asarray(mult,dtype=np.float64)
    step=max(VIA_D+DRCC+.012,VIA_DR+H2H+.012,VIA_R+VIA_DR/2+HOLE_CLR+.012,max(_pair_ground(NET_CUR[0],PAIR_NETS[NET_CUR[0]],li)[3] for li in range(NL)),.6)
    vs=np.array([math.ceil((step+RES)/RES/math.hypot(*v)) for v in directions],dtype=np.int64)
    lead=max(2,math.ceil(max(C['zw'])*math.tan(math.pi/8)/RES)+1)
    xs=[n%NN%NX for n in list(src)+list(tgt)];ys=[n%NN//NX for n in list(src)+list(tgt)]
    for margin in (3.,8.,20.):
        if time_tight():return None
        pad=math.ceil(margin/RES)
        xa,ya,xb,yb=max(1,min(xs)-pad),max(1,min(ys)-pad),min(NX-2,max(xs)+pad),min(NY-2,max(ys)+pad)
        states=NL*(xb-xa+1)*(yb-ya+1)*8*(lead+1)*(2 if C.get('pair_via_forced',False) else 1)
        state_limit=int(args.get('pair_states_max','0'))
        if state_limit>0 and states>state_limit:continue
        rest=_PAIR_REST(td,h,np.array(C['zw']),mult,viaH,NX,NY,RES,xa,ya,xb,yb,C['via'])
        if time_tight():return None
        if not any(math.isfinite(rest[n]) for n in sn):continue
        path,count=_PAIR_CORE(sn,sd,sc,td,h,mult,viaH,viaBad.ravel(),np.array(C['zw']),directions,NX,NY,RES,
            xa,ya,xb,yb,lead,vs,C['via'],int(args.get('pair_expansions','1000000')),rest,C['pair_permissible'],C.get('pair_via_forced',False))
        if len(path):return [int(n) for n in path]
    return None


def _pair_center(path, hard, widths):
    runs = []; cur = [path[0]]
    for n in path[1:]:
        if n//NN != cur[-1]//NN: runs.append(cur); cur=[n]
        else: cur.append(n)
    runs.append(cur); result=[]
    for run in runs:
        li=run[0]//NN
        pts=[cell_xy(n%NN%NX,n%NN//NX) for n in run]


        out=[pts[0]]
        for j in range(1,len(pts)-1):
            x,y,z=pts[j-1:j+2]
            if abs((y[0]-x[0])*(z[1]-y[1])-(y[1]-x[1])*(z[0]-y[0]))>1e-8 or (y[0]-x[0])*(z[0]-y[0])+(y[1]-x[1])*(z[1]-y[1])<=0:
                out.append(y)
        out.append(pts[-1])
        result.append((li,out))
    return result


def _pair_via_pattern(a,b,li,lj,center,direction,side):
    wa,wb,gap0,d0,_=_pair_ground(a,b,li); w1,w2,gap1,d1,_=_pair_ground(a,b,lj)
    distance=max(d0,d1,VIA_D+DRCC+.012,VIA_DR+H2H+.012,VIA_R+VIA_DR/2+HOLE_CLR+.012)
    normal=(-direction[1],direction[0]); reach=max(distance, .6)
    pair=[]
    for net,sign,w0,wnew,offset0,offset1 in ((a,side,wa,w1,(wb+gap0)/2,(w2+gap1)/2),(b,-side,wb,w2,(wa+gap0)/2,(w1+gap1)/2)):
        v=(center[0]+sign*distance/2*normal[0],center[1]+sign*distance/2*normal[1])
        if not _pair_vias_allowed([dict(x=v[0],y=v[1])]): return None
        if not _pair_zp_via_ok(net,*v) or not all(_pair_zp_ok(net,ll,Point(*v).buffer(VIA_R)) for ll in range(NL)): return None
        board=border_polygon(D).buffer(-float(args.get('edge','0.3')))
        if not board.covers(Point(*v).buffer(VIA_R)): return None
        for ll in range(NL):
            for nn,g in _PADG[ll]:
                if nn!=net and Point(*v).distance(g)<max(VIA_R+DRC_PADC,VIA_DR/2+HOLE_CLR)+.006: return None
        start=(center[0]-reach*direction[0]+sign*offset0*normal[0],center[1]-reach*direction[1]+sign*offset0*normal[1])
        end=(center[0]+reach*direction[0]+sign*offset1*normal[0],center[1]+reach*direction[1]+sign*offset1*normal[1])
        pp=_oct_move([start,v]); qq=_oct_move([v,end])
        if not _pair_line_ok(net,li,pp,w0) or not _pair_line_ok(net,lj,qq,wnew): return None
        pair.append(dict(via=dict(x=v[0],y=v[1]),before=pp,post=qq,reach=reach))
    for ll,key in ((li,'before'),(lj,'post')):
        w0,w1=_pair_ground(a,b,ll)[:2]
        g0=_LS(pair[0][key]).buffer(w0/2);g1=_LS(pair[1][key]).buffer(w1/2)
        if g0.distance(g1)<DRC_CLR-1e-6: return None
        for i,g in enumerate((g0,g1)):
            v=pair[1-i]['via']
            if g.distance(Point(v['x'],v['y']).buffer(VIA_R))<DRC_CLR-1e-6: return None
    return pair


_PAIR_SPLIT_REASON = [None]


def _pair_split(runs, a, b, side, start, end, copper=None):
    out=[[],[]]; vias=[[],[]]
    offsets=[]
    _PAIR_SPLIT_REASON[0]='Channel-Split'
    for li,pts in runs:
        if len(pts)<2: _PAIR_SPLIT_REASON[0]='Layer segment too short'; return None
        wa,wb,gap,d,_=_pair_ground(a,b,li)
        lane=[]
        for sign,offset in ((side,(wb+gap)/2),(-side,(wa+gap)/2)):
            line=_LS(pts).offset_curve(sign*offset,join_style=2,mitre_limit=3)



            if line.geom_type=='MultiLineString': line=shapely.line_merge(line,directed=True)
            if line.geom_type!='LineString' or line.is_empty or not line.is_simple:
                _PAIR_SPLIT_REASON[0]='Offset not permitted: '+str((line.geom_type,li,pts));return None
            lane.append([list(p) for p in line.coords])
        offsets.append(lane)
    for i in range(len(runs)-1):
        li,pp=runs[i]; lj,qq=runs[i+1]
        dx,dy=pp[-1][0]-pp[-2][0],pp[-1][1]-pp[-2][1]; ll=math.hypot(dx,dy)
        direction=(dx/ll,dy/ll)

        if math.dist(qq[0],pp[-1])>1e-6: return None
        if (qq[1][0]-qq[0][0])*direction[0]+(qq[1][1]-qq[0][1])*direction[1] < math.dist(qq[0],qq[1])-.00001: return None
        pattern=_pair_via_pattern(a,b,li,lj,pp[-1],direction,side)
        if pattern is None or math.dist(pp[-2],pp[-1])<=pattern[0]['reach'] or math.dist(qq[0],qq[1])<=pattern[0]['reach']:
            _PAIR_SPLIT_REASON[0]='Via pattern or straight before/after: '+str((li,lj,pp,qq));return None
        for k in range(2):
            offsets[i][k]=offsets[i][k][:-1]+[list(p) for p in pattern[k]['before']]
            offsets[i+1][k]=[list(p) for p in pattern[k]['post']]+offsets[i+1][k][1:]
            vias[k].append(pattern[k]['via'])
    for i,(li,_) in enumerate(runs):
        for k,net in enumerate((a,b)):
            pts=offsets[i][k]
            if i==0: pts=start[k]['pts'][:-1]+pts
            if i==len(runs)-1: pts=pts+end[k]['pts'][-2::-1]
            pts=[p for j,p in enumerate(pts) if j==0 or math.dist(p,pts[j-1])>1e-8]
            w=_pair_ground(a,b,li)[k]
            if len(pts)<2 or not all(oct_(x,y) for x,y in zip(pts[:-1],pts[1:])) or not _pair_line_ok(net,li,pts,w,None if copper is None else copper[1-k][li]):
                _PAIR_SPLIT_REASON[0]='Conductor split not permitted: '+str((net,li,pts));return None
            out[k].append(dict(layer=LAYERS[li],pts=pts,w=[w]*(len(pts)-1),pair_search_space=True))
        ga=shapely.union_all([_LS(t['pts']).buffer(t['w'][0]/2) for t in out[0] if LI[t['layer']]==li])
        gb=shapely.union_all([_LS(t['pts']).buffer(t['w'][0]/2) for t in out[1] if LI[t['layer']]==li])
        if ga.distance(gb)<(_ka_margin(a,b,li) if globals().get('KA') is not None else DRC_CLR)-1e-6:
            _PAIR_SPLIT_REASON[0]='Pair spacing at split';return None
    return out,vias


def _pair_tree_geometry(tracks,vias):
    result=[[],[]];vv=[[],[]]
    for k in range(2):
        for layer in LAYERS:
            tt=[t for t in tracks[k] if t['layer']==layer]
            if not tt:continue
            w=tt[0]['w'][0]
            assert all(all(abs(x-w)<1e-9 for x in t['w']) for t in tt)
            geom=shapely.line_merge(shapely.union_all([_LS(t['pts']) for t in tt]),directed=True)
            lines=[geom] if geom.geom_type=='LineString' else list(geom.geoms)
            for ln in lines:
                pts=[list(p) for p in ln.coords]
                if len(pts)>1:result[k].append(dict(layer=layer,pts=pts,w=[w]*(len(pts)-1),pair_search_space=True))
        seen=set()
        for v in vias[k]:
            key=(round(v['x'],8),round(v['y'],8))
            if key not in seen:seen.add(key);vv[k].append(v)
    return result,vv


_PAIR_SEARCH_INFO = {}

def _pair_planar_keepouts(C, path, src, tgt):
    candidates = [i for i in range(1, len(path)-1) if path[i] not in src and path[i] not in tgt
                  and not C['pair_tree'][path[i]]]
    if not candidates: return False
    def rank(i):
        x,y,z = path[i-1:i+2]
        dx,dy = y%NN%NX-x%NN%NX, y%NN//NX-x%NN//NX
        ex,ey = z%NN%NX-y%NN%NX, z%NN//NX-y%NN//NX
        return (0 if dx*ey-dy*ex else 1, abs(i-(len(path)-1)/2), i)
    n = path[min(candidates, key=rank)]
    C['pair_permissible'][n//NN, :, n%NN] = False
    return True


def _pair_edge(a,b,src,tgt,hard,soft,viaH,viaBad,C,widths,side,copper,alltr,allvs,source,target):
    allowed = C['pair_permissible']; details = []; rejected = False
    forced = C.get('pair_via_forced', False)
    try:
        for with_vias in (False, True):
            C['pair_permissible'] = allowed.copy() if not with_vias else allowed
            attempts = 1 if with_vias else max(1, int(args.get('pair_planar_attempts', '3')))
            if not with_vias and not ({n//NN for n in src}&{n//NN for n in tgt}): continue
            for attempt in range(attempts):
                if time_tight(): return None, None, details
                ASTAR_COST[0] = None
                C['pair_via_forced'] = with_vias and rejected
                path = _pair_search(src,tgt,hard,soft,viaH if with_vias else np.zeros_like(viaH),viaBad,C)
                if path is None or len(path)<2:
                    details.append('No path '+str((with_vias,attempt,len(src),len(tgt))))
                    break
                runs = _pair_center(path,hard,widths)
                l0,p0 = runs[0]; l1,p1 = runs[-1]; found = None
                if len(p0)>=2 and len(p1)>=2:
                    def direct(x,y):
                        d=math.dist(x,y); return ((y[0]-x[0])/d,(y[1]-x[1])/d)
                    start = _pair_connection(*source[:2],l0,p0[0],direct(p0[0],p0[1]),side,a,b,copper)
                    end = _pair_connection(*target[:2],l1,p1[-1],direct(p1[-1],p1[-2]),-side,a,b,copper)
                    if start is not None and end is not None:
                        found = _pair_split(runs,a,b,side,start,end,copper)
                        if found is not None and not _pair_tree_free([alltr[k]+found[0][k] for k in range(2)], [allvs[k]+found[1][k] for k in range(2)]):
                            found = None; _PAIR_SPLIT_REASON[0] = 'Pair branch violates existing trunk'
                    else: _PAIR_SPLIT_REASON[0] = 'PadPort-Direction'
                else: _PAIR_SPLIT_REASON[0] = 'Layer segment too short'
                if found is not None:
                    if not with_vias and attempt: C['pair_planar_fallback'] = C.get('pair_planar_fallback', 0)+1
                    return found, path, details
                details.append(('Via' if with_vias else 'Planar')+' '+str(attempt)+': '+str(_PAIR_SPLIT_REASON[0]))
                if with_vias: break
                rejected = True
                C['pair_planar_rejected'] = C.get('pair_planar_rejected', 0)+1
                if attempt+1<attempts and not _pair_planar_keepouts(C,path,src,tgt): break
        return None, None, details
    finally:
        C['pair_permissible'] = allowed
        C['pair_via_forced'] = forced


def route_pair(u):
    _pair_fixed_verify(u)
    a,b=sorted((u,PAIR_NETS[u])); old={n:U.get(n) for n in (a,b)}
    anchors=copy.deepcopy(VIA_PREV); success=False
    for n in (a,b): del_unit(n)
    try:
        groups=_pair_pad_groups(a,b)
        if groups is None: _PAIR_SEARCH_INFO[a]=dict(reason='Pad pair assignment unknown'); return False
        hard,soft=_pair_hard(a,b); widths=tuple(_pair_ground(a,b,li)[4] for li in range(NL))

        C=dict(cls(a),w=max(widths),wmin=min(widths),zw=widths,via=max(12.,2*cls(a)['via']*VIA_FACTOR[0]),inner=1.,pen=3.)
        C['lf']=pair_lf(1.) or tuple(1. if li in (0,NL-1) else 4. for li in range(NL))
        NET_CUR[0]=a; ASTAR_COST[0]=None; ASTAR_HINT[0]=None; VIA_TOO[0]=None

        viaH=_pair_via_mask(a,b,hard)
        viaBad=np.logical_or.reduce([v<VIA_D for v in soft])


        def root_key(i):
            p,q,_=groups[i]; ref=p['ref']
            driver=all(x.get('pintype') in ('output','tri_state') for x in (p,q))
            chip=ref.startswith(('U','IC')) and sum(g[0]['ref']==ref for g in groups)==1 and all(x.get('pintype') in ('output','tri_state','bidirectional') for x in (p,q))
            return (0 if driver else 1 if chip else 2,ref,str(p['num']))
        root=min(range(len(groups)),key=root_key)
        def middle(g):return ((g[0]['x']+g[1]['x'])/2,(g[0]['y']+g[1]['y'])/2)
        edges=[(root,j) for j in sorted((j for j in range(len(groups)) if j!=root),key=lambda j:(math.dist(middle(groups[root]),middle(groups[j])),j))]
        C['pair_tree']=np.zeros(NL*NN,dtype=np.bool_)
        alltr=[[],[]]; allvs=[[],[]]
        for i,j in edges:
            found=None;details=[]
            dx,dy=middle(groups[j])[0]-middle(groups[i])[0],middle(groups[j])[1]-middle(groups[i])[1]
            p=groups[i][0];mid=middle(groups[i])
            preferred=1 if dx*(p['y']-mid[1])-dy*(p['x']-mid[0])>=0 else -1
            for side in (preferred,-preferred):
                if time_tight(): return False
                C['pair_permissible'],copper=_pair_states(a,b,side,alltr,allvs)
                viaH=_pair_via_mask(a,b,hard,side,copper)
                src=_pair_ports(groups[i],a,b,hard,side,copper); tgt=_pair_ports(groups[j],a,b,hard,-side,copper)
                if not src or not tgt: details.append('No pad ports '+str((len(src),len(tgt))));continue
                found,path,attempts = _pair_edge(a,b,src,tgt,hard,soft,viaH,viaBad,C,widths,side,copper,alltr,allvs,groups[i],groups[j])
                details += attempts
                if found is not None: C['pair_tree'][path] = True
                if found is not None: break
            if found is None:
                _PAIR_SEARCH_INFO[a]=dict(reason='No shared channel with legal pad/via split',edges=len(edges),details=details);return False
            for k in range(2): alltr[k]+=found[0][k]; allvs[k]+=found[1][k]
        if time_tight(): return False
        alltr,allvs=_pair_tree_geometry(alltr,allvs)
        if not _pair_tree_free(alltr,allvs):
            _PAIR_SEARCH_INFO[a]=dict(reason='Pair branches/vias violate spacing');return False
        for k,n in enumerate((a,b)): add_unit(n,n,alltr[k],allvs[k])

        if RET>0 and (allvs[0] or allvs[1]) and set(RV_NET.values()).issubset(NID):
            RK=rv_map(a,(min(p[0] for tt in alltr[0]+alltr[1] for p in tt['pts'])-2,min(p[1] for tt in alltr[0]+alltr[1] for p in tt['pts'])-2,max(p[0] for tt in alltr[0]+alltr[1] for p in tt['pts'])+2,max(p[1] for tt in alltr[0]+alltr[1] for p in tt['pts'])+2),C['via'])
            if RK is not None:
                extra=rv_set(a,alltr[0]+alltr[1],allvs[0]+allvs[1],RK)
                del_unit(a);add_unit(a,a,alltr[0],allvs[0]+extra)
        if not time_tight(): lengths_match(only_pairs=True,only_nets={a,b})
        success=not time_tight()
        _PAIR_SEARCH_INFO[a]=dict(reason='Shared channel' if success else 'CPU-Budget',edges=len(edges),tree_begin=groups[root][0]['ref'],vias=[len(v) for v in allvs],planar_rejected=C.get('pair_planar_rejected',0),planar_fallback=C.get('pair_planar_fallback',0))
        return success
    finally:
        if not success:
            for n in (a,b): del_unit(n)
            for n,r in old.items():
                if r is not None: add_unit(n,r['net'],r['tracks'],r['vias'])
                else: add_unit(n,n,[],[])
            VIA_PREV.clear();VIA_PREV.update(anchors)
        _CAND_IDX[0]=None;ASTAR_COST[0]=None


def _pair_distance_costs(d, nominal, strength):
    channel = np.abs(d - nominal) <= .75 * RES
    return np.where(channel, PAIR_CHANNEL, 1.0 + strength * np.minimum(np.abs(d-nominal)/max(nominal,RES), 3.0)).astype(np.float32)


def pair_extra(net, C, capS, ex):
    fr = pair_free_mm(net); e = np.ones(NL * NN, dtype=np.float32) if ex is None else ex.copy()
    pn = PAIR_NETS[net]
    cnt = use.copy()
    if pn in U: cnt[U[pn]['cells']] -= 1
    for li in range(NL) if PAIR_DISTURB > 1 else ():
        sl = slice(li * NN, (li + 1) * NN); f = cnt[sl] > 0
        if not f.any(): continue
        d = (_edt(~f.reshape(NY, NX)) * RES).ravel()
        e[sl] = np.where(d - C['w'] / 2 < fr, e[sl] * np.float32(PAIR_DISTURB), e[sl])
    if pn in U and U[pn]['tracks']:
        strength = PAIR_MOVE if pair_fast(net) else .25 * PAIR_MOVE
        m = np.zeros(NL * NN, dtype=bool)
        for t in U[pn]['tracks']:
            li = LI[t['layer']]
            for a, b in zip(t['pts'][:-1], t['pts'][1:]): m[seg_cells(li, a, b, 0.0)] = True
        nominal = max(C['w'], (NC['net_list'].get(pn) or {}).get('w', C['w'])) / 2 + C['w'] / 2 + PAIR_GAP
        for li in range(NL):
            sl = slice(li * NN, (li + 1) * NN)
            if not m[sl].any():
                e[sl] *= np.float32(1.0 + 3*strength)
                continue
            if net in IMP_TAB:
                nominal = z_width(net, li, C['w']) / 2 + z_width(pn, li, C['w']) / 2 + z_gap(net, li)
            d = (_edt(~m[sl].reshape(NY, NX)) * RES).ravel()
            e[sl] *= _pair_distance_costs(d,nominal,strength)
    return e


def _oct_move(pts):
    out = [pts[0]]
    for q in pts[1:]:
        c = _corners(out[-1], q)
        if c: out.append(c[1])
        out.append(q)
    return out


def _pair_path_groups(net):
    from analysis.length_impedance import length_list, length_groups
    k = _pair_tolerances(copy.deepcopy(NC))
    groups = length_groups(k)
    selected = {g:q for g,q in groups.items() if net in q.get('net_list', [])}
    k['specs'] = dict(k.get('specs') or {},length_groups=selected)
    for n, entry in k['net_list'].items():
        entry.pop('length_group',None)
        if n in (net,PAIR_NETS[net]) and pair_fast(n): entry['z_diff_ohm'] = entry.get('z_diff_ohm') or 1
        else: entry.pop('z_diff_ohm',None)
    tracks = [dict(t,net=r['net']) for r in U.values() for t in r['tracks']]
    vias = [dict(v,net=v.get('net',r['net'])) for r in U.values() for v in r['vias']]
    return length_list(D,dict(tracks=tracks,vias=vias),k)


def _pair_vias_allowed(vias):
    for v in vias:
        ix, iy = math.floor((v['x']-X0)/RES), math.floor((v['y']-Y0)/RES)
        if not (0 <= ix < NX and 0 <= iy < NY): return False
        if VSP[iy, ix] < H2H + VIA_DR/2 + .01: return False
        if VIA_BAN is not None and VIA_BAN[iy, ix]: return False
    return True


def _pair_candidate(net, tracks, vias, verify, gates=True):
    if gates and (time_tight() or not _pair_vias_allowed(vias)): return False
    old = U[net]; accepted = False
    anchor_present = net in VIA_PREV; anchor = copy.deepcopy(VIA_PREV.get(net))
    del_unit(net)
    try:
        add_unit(net, old['net'], tracks, vias)
        accepted = (not gates or not time_tight()) and unit_conflict_free(net) and verify() and (not gates or not time_tight())
        if accepted: cand_idx_changed(net)
        return accepted
    finally:
        if not accepted:
            del_unit(net); add_unit(net, old['net'], old['tracks'], old['vias']); cand_idx_back(net)
            if gates:
                if anchor_present: VIA_PREV[net] = anchor
                else: VIA_PREV.pop(net, None)


def _pair_shift(a, b):
    from analysis.length_impedance import path_lengths
    from shapely.geometry import LineString, Point
    if time_tight(): return False
    rp, old = U[a], U[b]
    if any(v.get('rv') or v.get('gr') for r in (rp,old) for v in r['vias']): return False
    if not rp['tracks']: return False
    base = max(rp['tracks'], key=lambda t: sum(math.dist(p, q) for p, q in zip(t['pts'][:-1], t['pts'][1:])))
    dx, dy = base['pts'][-1][0]-base['pts'][0][0], base['pts'][-1][1]-base['pts'][0][1]

    directions = [(0, 1), (1, 0)] if abs(dx) >= abs(dy) else [(1, 0), (0, 1)]
    if dx and dy: directions.insert(0, (-math.copysign(1/math.sqrt(2), dy), math.copysign(1/math.sqrt(2), dx)))
    w = cls(b)['w']
    board = border_polygon(D).buffer(-float(args.get('edge', '0.3')))
    before = next((p for p in pairs_report(only={a,b},prev_gate=not PAIR_SEARCH_SPACE) if p['net_list'] == sorted((a, b))), None)
    if before is None: return False
    before_groups = _pair_path_groups(b)
    for nx, ny in directions:
        if time_tight(): return False
        for side in (1, -1):
            if time_tight(): return False
            shifts = {LI[t['layer']]: (side*(_pair_ground(a,b,LI[t['layer']])[3] if PAIR_SEARCH_SPACE else (cls(a)['w']+w)/2+PAIR_GAP)*nx,
                                      side*(_pair_ground(a,b,LI[t['layer']])[3] if PAIR_SEARCH_SPACE else (cls(a)['w']+w)/2+PAIR_GAP)*ny) for t in rp['tracks']}
            if rp['vias'] and len(set(shifts.values())) > 1: continue
            shift = next(iter(shifts.values()))
            tracks = [dict(layer=t['layer'], pts=[[round(p[0]+shifts[LI[t['layer']]][0],4),round(p[1]+shifts[LI[t['layer']]][1],4)] for p in t['pts']],
                           w=[z_width(b, LI[t['layer']], w)]*len(t['w'])) for t in rp['tracks']]
            vias = [dict(x=round(v['x']+shift[0],4),y=round(v['y']+shift[1],4)) for v in rp['vias']]
            valid = True
            for ai in by_net[b]:
                if time_tight(): return False
                pad = D['pads'][ai]
                lines = [(LineString(t['pts']), t) for t in tracks if t['layer'] in pad['layers']]
                if not lines: valid = False; break
                point = Point(pad['x'], pad['y']); line, t = min(lines, key=lambda v: v[0].distance(point))
                q = line.interpolate(line.project(point)); pts = _oct_move([(pad['x'],pad['y']),(q.x,q.y)])
                pts = [p for i,p in enumerate(pts) if i == 0 or math.dist(p,pts[i-1]) > 1e-6]
                if len(pts)>1: tracks.append(dict(layer=t['layer'],pts=[list(p) for p in pts],w=[w]*(len(pts)-1)))
            if not valid or any(not board.covers(LineString(t['pts']).buffer(max(t['w'])/2)) for t in tracks): continue
            if any(not board.covers(Point(v['x'],v['y']).buffer(VIA_R)) for v in vias): continue
            if IMP_REF_MASK and any(not z_ref_free(cls(b), LI[t['layer']], p, q, b) for t in tracks for p,q in zip(t['pts'][:-1],t['pts'][1:])): continue
            if time_tight(): return False
            def verify():
                paths = path_lengths(D,dict(tracks=[dict(t,net=b) for t in U[b]['tracks']],
                    vias=[dict(v,net=b) for v in U[b]['vias']]),b,DESIGN_RULES_STRUCTURE,all_pads=True)
                after = next(p for p in pairs_report(only={a,b},prev_gate=not PAIR_SEARCH_SPACE) if p['net_list'] == sorted((a,b)))
                good = bool(paths) and all(p['length_mm'] is not None for p in paths)
                good = good and after['coupled'] > before['coupled'] + .01 and after['skewed_mm'] <= before['skewed_mm'] + (1e-6 if PAIR_SEARCH_SPACE else .15)
                good = good and _receiver_not_worse(before_groups,_pair_path_groups(b),b)
                return good
            if _pair_candidate(b, tracks, vias, verify): return True
    return False


def pair_couple(only=None):
    from shapely.geometry import LineString as _L2
    n_ok = 0; _CAND_IDX[0] = None
    for P in sorted(PAIR_NETS):
        if time_tight(): break
        N = PAIR_NETS[P]
        if P > N or P not in U or N not in U or pair_unit(P): continue
        if only is not None and P not in only: continue
        if any(v.get('rv') or v.get('gr') for q in (P,N) for v in U[q]['vias']): continue
        if any(n_sv(U[q]) or len(U[q]['tracks']) != 1 or len(by_net.get(q, ())) != 2 for q in (P,N)):
            if _pair_shift(P,N) or _pair_shift(N,P): n_ok += 1
            continue
        for a_, b_ in ((P, N), (N, P)):
            if time_tight(): break
            rp, rn = U[a_], U[b_]
            if n_sv(rp) or len(rp['tracks']) != 1 or len(by_net.get(a_, ())) != 2 or len(by_net.get(b_, ())) != 2: continue
            tp = rp['tracks'][0]; li = LI[tp['layer']]; Pp = [tuple(q) for q in tp['pts']]
            if len(Pp) < 2: continue
            pn = [D['pads'][k] for k in by_net[b_]]
            if not all(tp['layer'] in q['layers'] for q in pn): continue
            Cn = cls(b_); w = z_width(b_, li, Cn['w'])
            nominal = (_pair_ground(a_,b_,li)[3] if PAIR_SEARCH_SPACE else
                    ((z_width(a_, li, cls(a_)['w'])/2 + w/2 + z_gap(b_, li)) if b_ in IMP_TAB else
                     max(Cn['w'], cls(a_)['w'])/2 + w/2 + PAIR_GAP))
            pa = min(pn, key=lambda q: math.dist((q['x'], q['y']), Pp[0])); pb = [q for q in pn if q is not pa][0]
            dx, dy = Pp[1][0] - Pp[0][0], Pp[1][1] - Pp[0][1]
            board_side = 1.0 if (dx * (pa['y'] - Pp[0][1]) - dy * (pa['x'] - Pp[0][0])) > 0 else -1.0
            try: o = _L2(Pp).offset_curve(board_side * nominal, join_style=2, mitre_limit=3.0)
            except Exception: continue
            if o.geom_type != 'LineString' or o.is_empty: continue
            oq = [(round(x, 4), round(y, 4)) for x, y in o.coords]
            if math.dist(oq[0], (pa['x'], pa['y'])) > math.dist(oq[-1], (pa['x'], pa['y'])): oq = oq[::-1]
            move = [(pa['x'], pa['y'])] + oq + [(pb['x'], pb['y'])]
            if oct_net(b_): move = _oct_move(move)
            move = [q for i, q in enumerate(move) if i == 0 or math.dist(q, move[i - 1]) > 1e-6]
            new_t = [dict(net=b_, layer=tp['layer'], pts=[[x, y] for x, y in move], w=[w] * (len(move) - 1))]
            if IMP_REF_MASK and not all(z_ref_free(Cn, li, a, b, b_) for a, b in zip(move[:-1], move[1:])): continue
            Lp = sum(math.dist(x, y) for x, y in zip(Pp[:-1], Pp[1:])); Ln = sum(math.dist(x, y) for x, y in zip(move[:-1], move[1:]))
            L0 = sum(math.dist(x, y) for t in rn['tracks'] for x, y in zip(t['pts'][:-1], t['pts'][1:]))


            if abs(Ln - Lp) > abs(L0 - Lp) + (1e-6 if PAIR_SEARCH_SPACE else .15) and n_sv(rn) == 0: continue
            if not _L2(move).is_simple: continue
            board = border_polygon(D).buffer(-float(args.get('edge', '0.3')))
            if not board.covers(_L2(move).buffer(w/2)): continue
            before_groups = _pair_path_groups(b_)
            if _pair_candidate(b_, new_t, [], lambda: _receiver_not_worse(before_groups,_pair_path_groups(b_),b_)):
                n_ok += 1; break
    _CAND_IDX[0] = None
    return n_ok


def pairs_report(receiver=False, only=None, prev_gate=False):
    out = []
    for n in sorted(PAIR_NETS):
        pn = PAIR_NETS[n]
        if n > pn: continue
        if only is not None and n not in only: continue
        category = NC['net_list'][n].get('pair_kind', NC['net_list'][n].get('category'))
        if n not in U or pn not in U:
            out.append(dict(net_list=[n,pn], category=category, total_length=[None,None], skewed_mm=None,
                vias=[n_sv(U[q]) if q in U else None for q in (n,pn)], coupled=0.0, incomplete=True))
            continue
        L = {q: sum(math.dist(a,b) for t in U[q]['tracks'] for a,b in zip(t['pts'][:-1],t['pts'][1:])) for q in (n,pn)}
        nominal = cls(n)['w']/2 + cls(pn)['w']/2 + PAIR_GAP
        fractions = []; spacings = {}
        for q, partner in ((n,pn),(pn,n)):
            shortened = 0.0
            for t in U[q]['tracks']:
                li = LI[t['layer']]
                distance = (_pair_ground(q,partner,li)[3] if PAIR_SEARCH_SPACE else
                            ((z_width(q,li,cls(q)['w'])/2 + z_width(partner,li,cls(partner)['w'])/2 + z_gap(q,li)) if q in IMP_TAB else nominal))
                spacings[t['layer']] = round(distance,4)
                for a,b in zip(t['pts'][:-1],t['pts'][1:]):
                    length = math.dist(a,b)
                    if length < 1e-8: continue
                    k = max(1, int(math.ceil(length/.05)))
                    parallel = [(c,d) for t2 in U[partner]['tracks'] if t2['layer'] == t['layer'] for c,d in zip(t2['pts'][:-1],t2['pts'][1:])
                                if math.dist(c,d)>1e-8 and abs((b[0]-a[0])*(d[0]-c[0])+(b[1]-a[1])*(d[1]-c[1]))/(length*math.dist(c,d)) > .995]
                    for i in range(k):
                        pt=(a[0]+(b[0]-a[0])*(i+.5)/k,a[1]+(b[1]-a[1])*(i+.5)/k)

                        dm=min((_seg_dist(pt,c,d) for c,d in parallel if prev_gate or
                                0 <= (pt[0]-c[0])*(d[0]-c[0])+(pt[1]-c[1])*(d[1]-c[1]) <= (d[0]-c[0])**2+(d[1]-c[1])**2),default=math.inf)
                        if abs(dm-distance)<=.05: shortened += length/k
            fractions.append(shortened/max(L[q],1e-9))
        out.append(dict(net_list=[n,pn], category=category, total_length=[round(L[n],3),round(L[pn],3)],
            skewed_mm=round(abs(L[n]-L[pn]),3), vias=[n_sv(U[n]),n_sv(U[pn])],
            coupled=round(min(fractions),3), coupled_per_net=[round(f,3) for f in fractions],
            spacing_nominal=round(nominal,3), spacing_per_layer=spacings))
    if receiver:
        from analysis.length_impedance import length_list
        k = _pair_tolerances(copy.deepcopy(NC))
        for n in PAIR_NETS:
            if pair_fast(n): k['net_list'][n]['z_diff_ohm'] = k['net_list'][n].get('z_diff_ohm') or 1
        tr,vi = export_geometry(); groups=length_list(D,dict(tracks=tr,vias=vi),k)
        for p in out:
            g=groups.get('pair:'+'|'.join(p['net_list']))
            p.update(skew_paths_mm=g['error_mm'] if g else None, tolerance_mm=g['tolerance_mm'] if g else None,
                     skew_in_tolerance=bool(g and g['complete'] and g['over_mm']<=1e-6),
                     receiver=g['receiver'] if g else {})
    return out


def _receiver_not_worse(before, after, net):
    from analysis.length_impedance import receiver_protection
    return receiver_protection(before, after, {net}, strict=PAIR_SEARCH_SPACE)


def lengths_match(only_pairs=False, only_nets=None):
    from analysis.length_impedance import length_list
    from shapely.geometry import LineString, Point
    board = border_polygon(D).buffer(-float(args.get('edge', '0.3')))
    structure = (NC.get('specs') or {}).get('structure') or NC.get('structure') or DESIGN_RULES_STRUCTURE
    def routes():
        tr, vi = export_geometry()
        return dict(tracks=tr, vias=vi)
    length_nc = _pair_tolerances(copy.deepcopy(NC))
    if only_pairs:
        length_nc['specs'] = dict(length_nc.get('specs') or {}, length_groups={})
        for k in length_nc['net_list'].values(): k.pop('length_group', None); k.pop('z_diff_ohm', None)


    for n in PAIR_NETS:
        if pair_fast(n): length_nc['net_list'][n]['z_diff_ohm'] = length_nc['net_list'][n].get('z_diff_ohm') or 1
    groups = length_list(D, routes(), length_nc)
    lengths = {n: v for g in groups.values() for n, v in g['net_list'].items()}
    targets = {n: v for n, v in lengths.items() if v is not None}

    for _ in range(len(groups) + 1):
        changed = False
        for g in groups.values():
            if not g['complete']: continue
            target = max(targets[n] for n in g['net_list'])
            for n in g['net_list']:
                if targets[n] < target: targets[n] = target; changed = True
        if not changed: break
    accepted = []; tried = 0; _CAND_IDX[0] = None
    for net in sorted(targets):
        if only_nets is not None and net not in only_nets: continue
        if net not in U or net.startswith(('FIX#', 'TACK#', 'GND#')): continue


        full_groups = _pair_path_groups(net) if only_pairs and net in PAIR_NETS else None
        tol = min((g['tolerance_mm'] for g in groups.values() if net in g['net_list']), default=0.15)
        for _ in range(int(args.get('lengths_moves', '12'))):
            if time_tight(0.95) or targets[net] - lengths[net] <= tol * 0.8: break
            old = U[net]; delta = targets[net] - lengths[net]; success = False
            segments = sorted([(math.dist(a, b), ti, si, a, b, w) for ti, t in enumerate(old['tracks'])
                               for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w']))], reverse=True)
            for L, ti, si, a, b, w in segments:
                if not oct_(a, b): continue
                layer = old['tracks'][ti]['layer']; c = max(3 * w, 0.15); top = 4 * w + 0.02
                span = 4 * c + top; lead = (L - span) / 2
                compact = net in PAIR_NETS and (PAIR_SEARCH_SPACE or delta <= 0.6)
                if compact:



                    if PAIR_SEARCH_SPACE: delta=max(0.,targets[net]-lengths[net]-.75*tol)
                    height = delta / (2 * (math.sqrt(2) - 1)); span = 2*height; lead = (L-span)/2
                if lead < 4 * w + 0.02: continue
                e = ((b[0] - a[0]) / L, (b[1] - a[1]) / L); normal = (-e[1], e[0])

                for fraction in (1.0, 0.5, 0.25, 0.125):
                    add = delta * fraction
                    height = add / (2 * (math.sqrt(2) - 1)) if compact else add / 2 + 2 * c * (2 - math.sqrt(2))
                    if not compact and height < 2 * c + 0.02: continue
                    if compact:
                        span = 2*height; lead = (L-span)/2
                        if lead < 4*w+.02: continue
                    if PAIR_SEARCH_SPACE and net in PAIR_NETS:
                        receivers=[]
                        for g in groups.values():
                            if net not in g['net_list'] or not g['complete'] or g.get('error_mm',float('inf'))<=g['tolerance_mm']: continue
                            for path in g.get('paths',{}).get(net,[]):
                                ref,num=(path.get('receiver') or ':').rsplit(':',1)
                                receivers += [(p['x'],p['y']) for p in D.get('pads',[]) if p['net']==net and p['ref']==ref and str(p['num'])==num]
                        local_span=2*height if compact else span
                        available=L-local_span
                        margin=4*w+.02
                        if available>=2*margin and receivers:
                            da=min(math.dist(a,p) for p in receivers);db=min(math.dist(b,p) for p in receivers)
                            lead=margin if da<db else available-margin

                    positions=[lead]
                    if compact and PAIR_SEARCH_SPACE:
                        available=L-span
                        direction=1. if lead<available/2 else -1.
                        margin=4*w+.02
                        positions += [min(available-margin,max(margin,lead+direction*d)) for d in (.5,1.)]
                        positions += [available/2]
                    for lead in dict.fromkeys(positions):

                        foreign = [LineString([pa, pb]).buffer(ww/2) for r2 in U.values() if r2['net'] != net for tt in r2['tracks'] if tt['layer'] == layer
                                   for pa, pb, ww in zip(tt['pts'][:-1], tt['pts'][1:], tt['w'])]
                        foreign += [g for nn, g in _PADG[LI[layer]] if nn != net]
                        def clearance(side):
                            mid = (Point(a[0]+(lead+(height if compact else span/2))*e[0] + side*height*normal[0], a[1]+(lead+(height if compact else span/2))*e[1] + side*height*normal[1])
                                   if PAIR_SEARCH_SPACE else Point((a[0]+b[0])/2 + side*height*normal[0], (a[1]+b[1])/2 + side*height*normal[1]))
                            return min((mid.distance(g) for g in foreign), default=1e9)
                        for side in sorted((1, -1), key=lambda s: (-clearance(s), -s)):
                            if time_tight(0.95): break
                            tried += 1
                            q = [(0, 0), (lead, 0), (lead + c, c), (lead + c, height - c),
                                 (lead + 2*c, height), (lead + 2*c + top, height),
                                 (lead + 3*c + top, height - c), (lead + 3*c + top, c),
                                 (lead + span, 0), (L, 0)]
                            if compact:
                                if not PAIR_SEARCH_SPACE: lead = (L - 2*height)/2
                                q = [(0,0),(lead,0),(lead+height,height),(lead+2*height,0),(L,0)]
                            pts = [[round(a[0] + x*e[0] + side*y*normal[0], 4),
                                    round(a[1] + x*e[1] + side*y*normal[1], 4)] for x, y in q]
                            pts[0], pts[-1] = list(a), list(b)
                            line = LineString(pts); copper = line.buffer(w/2, cap_style=1)
                            if not line.is_simple or not board.covers(copper): continue
                            if IMP_REF_MASK and not all(z_ref_free(cls(net), LI[layer], a, b, net) for a, b in zip(pts[:-1], pts[1:])): continue

                            pocket = LineString(pts[1:-1])
                            own_units = [(u2, r2) for u2, r2 in U.items() if r2['net'] == net]
                            own = [LineString([pa, pb]).buffer(ww/2) for u2, r2 in own_units for tj, t in enumerate(r2['tracks']) if t['layer'] == layer
                                   for sj, (pa, pb, ww) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])) if (u2, tj, sj) != (net, ti, si)]
                            own += [Point(v['x'], v['y']).buffer(VIA_R) for _, r2 in own_units for v in r2['vias']]
                            own += [g for n, g in _PADG[LI[layer]] if n == net]
                            if any(pocket.distance(g) < 3*w + w/2 - 1e-5 for g in own): continue
                            new = copy.deepcopy(old['tracks']); t = new[ti]
                            t['pts'] = t['pts'][:si] + pts + t['pts'][si+2:]
                            t['w'] = t['w'][:si] + [w]*(len(pts)-1) + t['w'][si+1:]
                            result = {}
                            def verify():
                                after = length_list(D, routes(), length_nc)
                                observed = [g['net_list'][net] for g in after.values() if net in g['net_list']]
                                length = max(observed) if observed and all(v is not None for v in observed) else None
                                good = length is not None and lengths[net] + 1e-4 < length <= targets[net] + 0.001
                                good = good and _receiver_not_worse(groups, after, net)
                                after_full = _pair_path_groups(net) if good and full_groups is not None else None
                                if full_groups is not None:
                                    good = good and _receiver_not_worse(full_groups, after_full, net)
                                result.update(after=after, length=length, full=after_full)
                                return good
                            if _pair_candidate(net, new, old['vias'], verify, gates=PAIR_SEARCH_SPACE):
                                length = result['length']
                                accepted.append(dict(net=net, layer=layer, before_mm=lengths[net], after_mm=length, w=w))
                                if full_groups is not None: full_groups = result['full']
                                lengths[net] = length; groups = result['after']; success = True; break
                        if success: break
                    if success: break
                if success: break
            if not success: break
    _CAND_IDX[0] = None
    return dict(move_records=accepted, attempts=tried, group_list=length_list(D, routes(), length_nc))


def sw_extra(net):
    if SW_DIST <= 0 or not any(k in net for k in SENS): return None
    sig = sum(_NVER.get(n, 0) for n in _SWN)
    c = _NEARBY_C.get('sw_prev')
    if c is not None and c[0] == sig: return c[1]
    m = np.zeros(NL * NN, dtype=bool)
    for p in D['pads']:
        if p['net'].split('/')[-1].startswith('SW_'):
            for L, c in p['cells'].items(): m[LI[L] * NN + c] = True
    for u, r in U.items():
        if r['net'].split('/')[-1].startswith('SW_'): m[r['cells']] = True
    ex = np.ones(NL * NN, dtype=np.float32)
    for li in range(NL):
        sl = slice(li * NN, (li + 1) * NN)
        if not m[sl].any(): continue
        d = _edt(~m[sl].reshape(NY, NX)) * RES
        ex[sl] = np.where(d.ravel() < SW_DIST, 6.0, 1.0)
    _NEARBY_C['sw_prev'] = (sig, ex)
    return ex


def geo_nodes(trs, vs):
    out = set()
    for t in trs:
        li = LI[t['layer']]
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            L = math.hypot(b[0] - a[0], b[1] - a[1]); n = max(2, int(L / (RES * 0.25)) + 1)
            for k in range(n):
                x = a[0] + (b[0] - a[0]) * k / (n - 1); y = a[1] + (b[1] - a[1]) * k / (n - 1)
                ix, iy = int((x - X0) / RES), int((y - Y0) / RES)
                if math.hypot(X0 + (ix + 0.5) * RES - x, Y0 + (iy + 0.5) * RES - y) < 0.03:
                    out.add(li * NN + iy * NX + ix)
                    if BRANCH_EXACT:
                        _l = _TRUNK.setdefault(li * NN + iy * NX + ix, [])
                        if not any(_t is t for _t in _l): _l.append(t)
    for v in vs:
        ix, iy = int((v['x'] - X0) / RES), int((v['y'] - Y0) / RES)
        for li in range(NL):
            out.add(li * NN + iy * NX + ix)
            if BRANCH_EXACT: _TRUNK_V[li * NN + iy * NX + ix] = (v['x'], v['y'])
    return sorted(out)







CONS = args.get('via_group', '1' if NC else '0') == '1'
CONS_DIST = max(float(args.get('cons_spacing', '0.7')), HOLE_C)
CONS_IVIA = float(args.get('cons_current03', '1.0')) * VIA_DR / 0.3
CURRENT_VIA_PRICE = args.get('current_via_price', '0') == '1' and NC is not None
CONS_RES = float(args.get('cons_reserve', '1.3')); CONS_MIN = float(args.get('cons_minimum', '0.5')); CONS_MAX = int(args.get('cons_max', '0' if CURRENT_VIA_PRICE else '4'))
CONS_REL = args.get('cons_relative', '0') == '1'
CONS_SOFT = [True]


def cons_count(net, pads, tree, j, with_=()):
    k = NC['net_list'].get(net) or {}; fl = k.get('flow') or {}
    if k.get('category') not in ('current_val', 'supply', 'sw') or not (fl.get('unit') == 'A' or CONS_REL): return 1
    load = fl.get('load') or {}; source = set(fl.get('source') or [])
    def nm(i): return '%s:%s' % (pads[i]['ref'], pads[i]['num'])
    lt = sum(float(load.get(nm(i), 0.0)) for i in tree if nm(i) not in source)
    lj = float(load.get(nm(j), 0.0))
    if with_: lj += sum(float(load.get(nm(i), 0.0)) for i in with_ if nm(i) not in source)
    if nm(j) in source or any(nm(i) in source for i in with_): current_val = lt
    elif any(nm(i) in source for i in tree): current_val = lj
    else: current_val = min(lj, lt)
    if current_val < CONS_MIN: return 1
    nominal = max(2, int(math.ceil(CONS_RES * current_val / CONS_IVIA - 1e-9)))
    return min(CONS_MAX, nominal) if CONS_MAX > 0 else nominal


def cons_set(net, res, trs, vs, n_nominal, capH, viaH, capC, vbadf, Cb, already, rng):
    viaf = viaH.ravel(); w_st = max(TRACK_MIN, min(Cb['wmin'], VIA_D))
    change = [(a // NN, b // NN) for a, b in zip(res[:-1], res[1:]) if a // NN != b // NN]
    own = [(v['x'], v['y']) for v in already] + [(v['x'], v['y']) for v in vs]
    bridges, new_v = [], []
    for v, (la, lb) in zip(vs, change):
        v['n'] = n_nominal; vx, vy = v['x'], v['y']
        cand = []
        for t in trs:
            li = LI[t['layer']]
            if li not in (la, lb): continue
            for pts in (t['pts'], t['pts'][::-1]):
                if abs(pts[0][0] - vx) > 1e-6 or abs(pts[0][1] - vy) > 1e-6: continue
                way = 0.0; k = 1
                for a, b in zip(pts[:-1], pts[1:]):
                    L = math.hypot(b[0] - a[0], b[1] - a[1])
                    while k < n_nominal and L > 0 and way + L >= k * CONS_DIST - 1e-9:
                        f = (k * CONS_DIST - way) / L
                        cand.append((k, li, a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))); k += 1
                    way += L
                    if k >= n_nominal: break
        rings = [(10, CONS_DIST), (11, 1.5 * CONS_DIST)]
        if CURRENT_VIA_PRICE:
            rings += [(12 + k, (2 + k) * CONS_DIST) for k in range(max(1, int(math.ceil(math.sqrt(n_nominal))))) ]
        for ring, r in rings:
            nq = 8 if ring < 12 else max(8, int(2 * math.pi * r / CONS_DIST))
            for q in range(nq):
                cand.append((ring, -1, vx + r * math.cos(q * 2 * math.pi / nq), vy + r * math.sin(q * 2 * math.pi / nq)))
        missing = n_nominal - 1
        for soft in ((False, True) if CONS_SOFT[0] else (False,)):
            grid_map = capH if soft else capC
            for rank, lt, x, y in sorted(cand):
                if missing <= 0: break
                ix, iy = int((x - X0) / RES), int((y - Y0) / RES)
                if not (rng[0] < ix < rng[1] - 1 and rng[2] < iy < rng[3] - 1): continue
                c = iy * NX + ix; x, y = cell_xy(ix, iy)
                if not viaf[c] or (not soft and vbadf[c]): continue
                if any(math.hypot(x - ex, y - ey) < CONS_DIST - 0.06 for ex, ey in own): continue
                st = []
                for lo in (la, lb):
                    if lo == lt: continue
                    if line_cap_min(grid_map[lo], (x, y), (vx, vy)) < w_st - 1e-6 or pad_room(lo, (x, y), (vx, vy), net) < w_st / 2: st = None; break
                    st.append(dict(layer=LAYERS[lo], pts=[[round(x, 4), round(y, 4)], [vx, vy]], w=[w_st], web=True))
                if st is None: continue
                bridges += st; new_v.append(dict(x=round(x, 4), y=round(y, 4), gr=1, too=[vx, vy])); own.append((x, y)); missing -= 1
    return bridges, new_v








RET = float(args.get('ret', '1.27')) if NC is not None else 0.0
RET_R3 = float(args.get('ret_r3', '2.5'))
RET_PRESENT = float(args.get('ret_present', '0.25')); RET_MISSING = float(args.get('ret_missing', '1.0'))
RV_PAIR = np.zeros(NL * NL, dtype=np.float64)
RV_NET = {}
if RET > 0:



    for _li, _a in enumerate(STRUCTURE):
        for _lj, _b in enumerate(STRUCTURE):
            if _li == _lj or not _a['ref'] or not _b['ref'] or _a['ref'] == _b['ref']: continue
            _gn = sorted({x for x in (_a['ref_net'], _b['ref_net']) if x in GNDS})
            if not _gn and GND_MAIN in AREA_NETS: _gn = [GND_MAIN]
            if len(_gn) == 1: RV_PAIR[_li * NL + _lj] = 1.0; RV_NET[(_li, _lj)] = _gn[0]
    if not RV_NET: RET = 0.0
    else: print('Return vias (Q2): %d layer pairs with different ground reference areas, radius %.2f mm (rank <= 3: %.2f mm)' % (
        len(RV_NET) // 2, RET, RET_R3), flush=True)
RV_STAT = dict(assigned=0, already=0, no_slot=0, post_run_new=0, post_run_way=0)


def rv_radius(net):
    if RET <= 0: return 0.0
    k = NC['net_list'].get(net) or {}
    if k.get('category') != 'fast': return 0.0
    return RET if k.get('rank', 0) >= 4 else RET_R3


def _rv_gnd(net_name):
    xy = [(v['x'], v['y']) for r in U.values() for v in r['vias'] if (v.get('net') or r['net']) == net_name]
    xy += [(D['pads'][k]['x'], D['pads'][k]['y']) for k in by_net_g.get(net_name, ()) if D['pads'][k].get('drill')]
    return np.array(xy, dtype=np.float64).reshape(-1, 2)


def rv_map(net, box, VC):
    R = rv_radius(net)
    if R <= 0: return None
    if PLANE_LAYER:
        local={g:_rv_map_net(g,box,VC,R) for g in sorted(GNDS & PLANE_AREA.keys())}
        if not local:return None

        via_extra=np.full(NN,np.float32(RET_MISSING*VC),dtype=np.float32)
        for g,rk in local.items():
            mask=PLANE_MASK[g].ravel();via_extra[mask]=np.minimum(via_extra[mask],rk['via_extra'][mask])
        return dict(local=local,via_extra=via_extra,R=R)
    net_list = sorted(set(RV_NET.values()))
    if len(net_list) != 1: return None
    g = net_list[0]
    return _rv_map_net(g,box,VC,R)


def _rv_map_net(g, box, VC, R):
    bxR = (box[0] - R, box[1] - R, box[2] + R, box[3] + R)
    ix0, ix1, iy0, iy1 = _box_idx(bxR)
    ys, xs = slice(iy0, iy1), slice(ix0, ix1)
    rng = (ix0, ix1, iy0, iy1)
    _, vH = hard_maps(g, rng if LARGE else None)
    vacant = np.zeros((NY, NX), dtype=bool); f = np.array(vH[ys, xs], dtype=bool, copy=True)
    capS, dd, _r2 = soft_maps(g, bxR, VIA_D)
    f &= dd[ys, xs] >= HOLE_C
    for li in range(NL): f &= capS[li][ys, xs] / 2 >= VIA_R
    if PLANE_LAYER:
        xx,yy=np.meshgrid(GXc[ix0:ix1],GYc[iy0:iy1])
        for L,nn in FL_ASSIGNMENT.items():
            if g in nn:
                area=PLANE_LAYER.get((g,L),Polygon()).buffer(-VIA_R-.01)
                f &= shapely.contains_xy(area,xx,yy)
    vacant[ys, xs] = f
    via_extra = np.full(NN, np.float32(RET_MISSING * VC), dtype=np.float32); V = via_extra.reshape(NY, NX)
    if f.any(): V[ys, xs] = np.where(_edt(~f) * RES <= R, np.float32(RET_PRESENT * VC), V[ys, xs])
    gv = _rv_gnd(g)
    if len(gv):
        m = np.zeros((iy1 - iy0, ix1 - ix0), dtype=bool)
        jx = ((gv[:, 0] - X0) / RES).astype(int) - ix0; jy = ((gv[:, 1] - Y0) / RES).astype(int) - iy0
        ok = (jx >= 0) & (jx < ix1 - ix0) & (jy >= 0) & (jy < iy1 - iy0)
        m[jy[ok], jx[ok]] = True
        if m.any(): V[ys, xs] = np.where(_edt(~m) * RES <= R, np.float32(0.0), V[ys, xs])
    return dict(net_name=g, vacant=vacant, via_extra=via_extra, R=R)


def _seg_dist(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]; l2 = dx * dx + dy * dy
    t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def _via_layers(net, v, trs):
    gen = set()
    for t in trs:
        if t.get('net', net) != net: continue
        for e in (t['pts'][0], t['pts'][-1]):
            if abs(e[0] - v['x']) < VIA_R + 0.05 and abs(e[1] - v['y']) < VIA_R + 0.05: gen.add(LI[t['layer']])
    for k in by_net_g.get(net, ()):
        p = D['pads'][k]; bb = p['bbox']
        if bb[0] - 0.01 <= v['x'] <= bb[2] + 0.01 and bb[1] - 0.01 <= v['y'] <= bb[3] + 0.01: gen |= {LI[L] for L in p['layers'] if L in LI}
    return gen


def rv_demand(net, v, trs):
    gen = sorted(_via_layers(net, v, trs))
    for i, a in enumerate(gen):
        for b in gen[i + 1:]:
            if RV_PAIR[a * NL + b] > 0:
                if PLANE_LAYER:
                    refs=[STRUCTURE[i]['ref'] for i in (a,b)]
                    mass=[L for L in refs if set(FL_ASSIGNMENT.get(L,[])) & GNDS]
                    pt=_PT(v['x'],v['y'])
                    domains=globals().get('PLANE_REF',PLANE_LAYER)
                    candidates=[g for g in sorted(GNDS) if mass and all(domains.get((g,L),Polygon()).covers(pt) for L in mass)]
                    return candidates[0] if len(candidates)==1 else None
                return RV_NET[(a, b)]
    return None


def rv_set(net, trs, vs, RK, owned=None):
    if 'local' in RK:
        new=[]
        for g,rk in sorted(RK['local'].items()):
            subset=[v for v in vs if rv_demand(net,v,trs)==g]
            new+=rv_set(net,trs,subset,rk,owned=(vs if owned is None else owned)+new)
        return new
    R = RK['R']; g = RK['net_name']; vacant = RK['vacant']; new = []
    gv = _rv_gnd(g)
    mine_v = [(v['x'], v['y']) for v in (vs if owned is None else owned)]
    r0 = VIA_D + DRCC + 0.02
    rings = [r0] + [r for r in (0.7, 0.85, 1.0, 1.15, 1.3, 1.5, 1.75, 2.0, 2.25, 2.5) if r0 < r < R - 0.02] + [R - 0.02]
    for v in vs:
        if v.get('rv') or v.get('gr'): continue
        if rv_demand(net, v, trs) != g: continue
        all_items = [tuple(q) for q in gv.tolist()] + [(w['x'], w['y']) for w in new]
        if any(math.hypot(v['x'] - a, v['y'] - b) <= R for a, b in all_items): RV_STAT['already'] += 1; continue
        ok = False
        for r in rings:
            for q in range(16):
                ix = int((v['x'] + r * math.cos(q * math.pi / 8) - X0) / RES); iy = int((v['y'] + r * math.sin(q * math.pi / 8) - Y0) / RES)
                if not (0 <= ix < NX and 0 <= iy < NY) or not vacant[iy, ix]: continue
                x, y = cell_xy(ix, iy)
                if math.hypot(x - v['x'], y - v['y']) > R: continue
                if any(math.hypot(x - a, y - b) < max(VIA_D + DRC_CLR, DRC_HOLE) + 0.01 for a, b in mine_v + [(w['x'], w['y']) for w in new]): continue
                if any(_seg_dist((x, y), a, b) < VIA_R + w_ / 2 + DRC_CLR + 0.01 for t in trs for a, b, w_ in zip(t['pts'][:-1], t['pts'][1:], t['w'])): continue
                new.append(dict(x=round(x, 4), y=round(y, 4), net=g, rv=True)); ok = True; break
            if ok: break
        RV_STAT['assigned' if ok else 'no_slot'] += 1
    return new


def rv_post_run():
    if RET <= 0: return 0, 0
    new_n = way_n = 0; _CAND_IDX[0] = None
    for u in sorted(U):
        if u.startswith(('GND#', 'FIX#', 'TACK#')): continue
        r = U[u]; net = r['net']; R = rv_radius(net)
        if R <= 0 or not r['vias']: continue
        sv = [v for v in r['vias'] if not v.get('rv')]; rv = [v for v in r['vias'] if v.get('rv')]
        demand = [(v, rv_demand(net, v, r['tracks'])) for v in sv]; demand = [(v, g) for v, g in demand if g]
        if not demand and not rv: continue
        legacy_g = demand[0][1] if demand else rv[0]['net']
        foreign_nets={}
        for g in sorted({g for _,g in demand}|{v['net'] for v in rv}):
            source_g=g if PLANE_LAYER else legacy_g
            foreign_nets[g]=[(v['x'],v['y']) for w,r2 in U.items() if w!=u for v in r2['vias'] if (v.get('net') or r2['net'])==source_g]
            foreign_nets[g]+=[(D['pads'][k]['x'],D['pads'][k]['y']) for k in by_net_g.get(source_g,()) if D['pads'][k].get('drill')]
        keep = []
        for w in rv:
            required = any((not PLANE_LAYER or w['net']==g) and math.hypot(v['x'] - w['x'], v['y'] - w['y']) <= R and not any(math.hypot(v['x'] - a, v['y'] - b) <= R for a, b in foreign_nets[g])
                         for v, g in demand)
            if required: keep.append(w)
            else: way_n += 1
        open_items = [v for v,g in demand if not any(math.hypot(v['x'] - a, v['y'] - b) <= R for a,b in foreign_nets[g]+[(w['x'],w['y']) for w in keep if not PLANE_LAYER or w['net']==g])]
        if len(keep) != len(rv):
            del_unit(u); add_unit(u, net, r['tracks'], sv + keep); r = U[u]; cand_idx_changed(u)
        if open_items:
            pads = [D['pads'][k] for k in by_net[net]]
            bx = (min(p['bbox'][0] for p in pads) - 3.0, min(p['bbox'][1] for p in pads) - 3.0, max(p['bbox'][2] for p in pads) + 3.0, max(p['bbox'][3] for p in pads) + 3.0)
            bx = (min([bx[0]] + [v['x'] - 1 for v in open_items]), min([bx[1]] + [v['y'] - 1 for v in open_items]), max([bx[2]] + [v['x'] + 1 for v in open_items]), max([bx[3]] + [v['y'] + 1 for v in open_items]))
            import copy as _cp
            save = _cp.deepcopy(U[u]); del_unit(u)
            RK = rv_map(net, bx, 0.0)
            extra = rv_set(net, save['tracks'], open_items, RK, owned=save['vias']) if RK else []
            add_unit(u, net, save['tracks'], save['vias'] + extra)
            if extra and not unit_conflict_free(u):
                del_unit(u); add_unit(u, net, save['tracks'], save['vias']); extra = []
            cand_idx_changed(u); new_n += len(extra)
    RV_STAT['post_run_new'] += new_n; RV_STAT['post_run_way'] += way_n; _CAND_IDX[0] = None
    return new_n, way_n








VIA_ANCHOR = float(args.get('via_anchor', '0')) if NC is not None else 0.0
VIA_PREV = {}
VIA_ANCHOR_ON = [False]
VIA_PRECEDENCE = args.get('via_precedence', '0') == '1' and NC is not None
PAIRS_RAW = []



K12 = float(args.get('k12', '0')) if (NC is not None and AREA_NETS) else 0.0
K12_DIST = float(args.get('k12_spacing', '%g' % (VIA_D + 2 * 0.15 + 0.15)))


def via_surcharge(u, net, C, box):
    k = (NC['net_list'].get(net) or {}) if NC else {}
    anchor = VIA_ANCHOR > 0 and VIA_ANCHOR_ON[0] and k.get('category') in ('current_val', 'supply', 'sw') and VIA_PREV.get(u)
    k12 = K12 > 0 and DENSE and net not in AREA_NETS
    if not (anchor or k12): return None
    V = None
    ix0, ix1, iy0, iy1 = _box_idx(box); ys, xs = slice(iy0, iy1), slice(ix0, ix1)
    if anchor:
        V = np.full((NY, NX), np.float32(VIA_ANCHOR), dtype=np.float32)
        m = np.zeros((iy1 - iy0, ix1 - ix0), dtype=bool)
        for x, y in VIA_PREV[u]:
            jx, jy = int((x - X0) / RES) - ix0, int((y - Y0) / RES) - iy0
            if 0 <= jx < ix1 - ix0 and 0 <= jy < iy1 - iy0: m[jy, jx] = True
        if m.any():
            d = _edt(~m) * RES
            V[ys, xs] = (VIA_ANCHOR * np.clip((d - 1.0) / 4.0, 0.0, 1.0)).astype(np.float32)
    if k12:
        vs = [(v['x'], v['y']) for w, r in U.items() for v in r['vias'] if (v.get('net') or r['net']) not in AREA_NETS]
        for lg, dx0, dx1, dy0, dy1, _nz in DENSE:
            ax0, ax1, ay0, ay1 = dx0, dx1, dy0, dy1
            if ax1 <= ax0 or ay1 <= ay0: continue
            pz = int(K12_DIST / RES) + 2
            m2 = np.zeros((ay1 - ay0 + 2 * pz, ax1 - ax0 + 2 * pz), dtype=bool)
            for x, y in vs:
                jx, jy = int((x - X0) / RES) - ax0 + pz, int((y - Y0) / RES) - ay0 + pz
                if 0 <= jx < m2.shape[1] and 0 <= jy < m2.shape[0]: m2[jy, jx] = True
            if not m2.any(): continue
            d = (_edt(~m2) * RES)[pz:-pz, pz:-pz]
            if V is None: V = np.zeros((NY, NX), dtype=np.float32)
            V[ay0:ay1, ax0:ax1] += np.where(d < K12_DIST, np.float32(K12 * C['via'] * VIA_FACTOR[0]), np.float32(0.0))
    return None if V is None else V.ravel()


def precedence_filter(conf):
    def level1(w): return not w.startswith(('GND#', 'FIX#', 'TACK#')) and w in U and (NC['net_list'].get(U[w]['net']) or {}).get('category') in ('signal', 'analog')
    new = {}
    def rating(w, x, y, li):
        if w == 'PAD': return
        ix, iy = int((x - X0) / RES), int((y - Y0) / RES); lst = new.setdefault(w, [])
        for jy in range(max(0, iy - 3), min(NY, iy + 4)):
            for jx in range(max(0, ix - 3), min(NX, ix + 4)): lst.append(li * NN + jy * NX + jx)
    n = 0
    for li, x, y, w1, w2 in PAIRS_RAW:
        for a, b in ((w1, w2), (w2, w1)):
            if a.startswith('GND#') and level1(b) and li >= 0: break
        else: a = None
        lis = range(NL) if li < 0 else (li,)
        for l_ in lis:
            if a is None: rating(w1, x, y, l_); rating(w2, x, y, l_)
            else: rating(b, x, y, l_); n += 1
    return new, n


def rv_report():
    if RET <= 0: return None
    n = ok = owned = 0; missing = []
    for u in sorted(U):
        r = U[u]; net = r['net']; R = rv_radius(net)
        if R <= 0 or u.startswith(('GND#', 'FIX#', 'TACK#')): continue
        for v in r['vias']:
            if v.get('rv'): continue
            g = rv_demand(net, v, r['tracks'])
            if not g: continue
            n += 1; gv = _rv_gnd(g)
            d = float(np.min(np.hypot(gv[:, 0] - v['x'], gv[:, 1] - v['y']))) if len(gv) else 99.0
            if d <= R + 1e-6:
                ok += 1; owned += any(w.get('rv') and math.hypot(w['x'] - v['x'], w['y'] - v['y']) <= R + 1e-6 for w in r['vias'])
            else: missing.append(dict(net_name=net, x=v['x'], y=v['y'], spacing=round(d, 2)))
    return dict(change=n, with_ret=ok, own_ret_via=owned, without=missing, stat=dict(RV_STAT))







ESCAPE = args.get('escape', '1' if NC else '0') == '1'
_FL = {}


def _pz_ports(net,p,w,capH,rng):
    if p.get('drill') or max(p['bbox'][2]-p['bbox'][0],p['bbox'][3]-p['bbox'][1])>1.2:return ()
    out={};a=(p['x'],p['y'])
    for L in p['layers']:
        if L not in LI:continue
        li=LI[L]
        own=shapely.union_all([Polygon(q) for q in p['polys'][L]])
        if not own.covers(_PT(a)):continue
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            seen=set()
            for step in range(1,33):
                d=step*.025
                ix=int((a[0]+dx*d-X0)/RES);iy=int((a[1]+dy*d-Y0)/RES)
                if (ix,iy) in seen:continue
                seen.add((ix,iy))
                if not (rng[0]<=ix<rng[1] and rng[2]<=iy<rng[3]):continue
                if capH[li][iy,ix]<w:continue
                b=cell_xy(ix,iy)
                if own.covers(_PT(b)):continue
                paths=[[a,b]] if oct_(a,b) else [[a,c,b] for c in _corners(a,b)]
                good=[]
                for pts in paths:
                    pts=[_pz_snap(p) for p in pts]
                    if all(pad_room(li,x,y,net)>=w/2 and (not HARD_FOREIGN[0] or _pz_room(net,li,x,y,False)>=w/2) for x,y in zip(pts[:-1],pts[1:])):
                        good.append(pts)
                if good:
                    pts=min(good,key=lambda ps:sum(math.dist(x,y) for x,y in zip(ps[:-1],ps[1:])))
                    c=li*NN+iy*NX+ix
                    if c not in out:out[c]=(c,L,[list(pt) for pt in pts])
                    break
    return tuple(out[c] for c in sorted(out))


def escape(net, p, wk, capH, rng):
    if _pz_active(net) and id(p) in PZ_LOCAL[net]:return _pz_ports(net,p,wk,capH,rng)
    if not ESCAPE or p.get('drill'): return ()
    key = (id(p), wk)
    if key in _FL: return _FL[key]
    lag = [x for x in p['layers'] if x in LI]
    bx = p['bbox']; bw, bh = bx[2] - bx[0], bx[3] - bx[1]
    if len(lag) != 1 or min(bw, bh) > 0.45 or max(bw, bh) < 1.3 * min(bw, bh): _FL[key] = (); return ()
    L = lag[0]; li = LI[L]; cap = capH[li]; cx, cy = (bx[0] + bx[2]) / 2, (bx[1] + bx[3]) / 2
    out = []; remember = True
    for sgn in (-1.0, 1.0):
        if bw < bh: ex, ey, dx, dy = cx, (bx[3] if sgn > 0 else bx[1]), 0.0, sgn
        else: ex, ey, dx, dy = (bx[2] if sgn > 0 else bx[0]), cy, sgn, 0.0
        grid_cells = []; open_items = False
        for q in range(1, 13):
            ix, iy = int(math.floor((ex + dx * q * RES - X0) / RES)), int(math.floor((ey + dy * q * RES - Y0) / RES))


            if not (0 <= ix < NX and 0 <= iy < NY): break
            if not (rng[0] <= ix < rng[1] and rng[2] <= iy < rng[3]): open_items = True; break
            grid_cells.append((q * RES, ix, iy, bool(cap[iy, ix] >= wk)))
        if len(grid_cells) >= 2 and grid_cells[0][3] and grid_cells[1][3]: continue
        if open_items and len(grid_cells) < 2: remember = False; continue
        for d, ix, iy, ok in grid_cells:
            if not ok: continue
            zx, zy = cell_xy(ix, iy); da = max(0.0, d - 0.15); A = (ex + dx * da, ey + dy * da)
            if pad_room(li, (cx, cy), A, net) >= wk / 2 and pad_room(li, A, (zx, zy), net) >= wk / 2:
                out.append((li * NN + iy * NX + ix, L, [[round(cx, 4), round(cy, 4)], [round(A[0], 4), round(A[1], 4)], [round(zx, 4), round(zy, 4)]]))
            break
        else:
            if open_items: remember = False
    if remember: _FL[key] = tuple(out)
    return tuple(out)

def bga_own_hole_map(viaH, vias):
    if not vias: return viaH
    out = viaH.copy()
    for x, y in sorted({(float(v['x']), float(v['y'])) for v in vias}):
        ix, iy = int((x - X0) / RES), int((y - Y0) / RES)
        rad = int(math.ceil(HOLE_C / RES)) + 2
        x1, x2 = max(0, ix - rad), min(NX, ix + rad + 1)
        y1, y2 = max(0, iy - rad), min(NY, iy + rad + 1)
        if x1 >= x2 or y1 >= y2: continue
        yy, xx = np.mgrid[y1:y2, x1:x2]
        px = X0 + (xx + .5) * RES; py = Y0 + (yy + .5) * RES
        for vy, vx in zip(*np.nonzero(out[y1:y2, x1:x2])):
            c = (vy + y1) * NX + vx + x1
            if c in VP: px[vy, vx], py[vy, vx] = VP[c][:2]
        d2 = (px - x) ** 2 + (py - y) ** 2
        out[y1:y2, x1:x2] &= (d2 >= HOLE_C ** 2) | (d2 <= 1e-12)
    return out


def bga_escape(net, p, wk, capH, viaH, vbadf, rng, surface=False, soft=None, inner_w=None):
    b = BGA.get(id(p))
    ring_min = int(args.get('bga_ring_min', '2' if BGA_ESCAPE else '0')) if BGA_PLAN else 2
    if b is None: return None
    if BGA_ESCAPE and b['ring'] == 1 and 'bga_ring_min' not in args:


        bb = p['bbox']
        if b['pitch'] - max(bb[2]-bb[0],bb[3]-bb[1]) - 2*DRCP < wk - 1e-9:
            ring_min = 1


    supply_escape = BGA_ESCAPE and p.get('pintype') == 'power_in'
    if b['ring'] < ring_min and not surface and not supply_escape: return None
    inner = [i for i, L in enumerate(LAYERS) if L.startswith('In')]
    if not inner and not surface: return None
    if inner:
        q = max(0, b['ring'] - 2) % len(inner)
        inner = inner[q:] + inner[:q]
    candidates = b['pts']
    _vd = _vd_cost().ravel() if VIA_DENSITY else None
    if BGA_PLAN:


        def price(c):
            iy, ix = divmod(c, NX)



            via_narrow = (sum(max(0.0, VIA_D - float(soft[il][iy, ix])) / VIA_D for il in range(NL))
                        if BGA_ESCAPE and soft is not None else 0.0)
            return ((float(vbadf[c]) + via_narrow) * PF[0] + sum(float(hist[il * NN + c]) for il in range(NL)) +
                    (VD_MM * float(_vd[c]) if _vd is not None else 0.0),
                    math.dist((p['x'], p['y']), VP[c][:2]), c)
        candidates = sorted((c for c in candidates if rng[0] <= c % NX < rng[1] and rng[2] <= c // NX < rng[3]), key=price)
    if VIA_DENSITY and not BGA_PLAN and _vd.any():
        candidates = sorted(candidates, key=lambda c: (VD_MM * float(_vd[c]) + float(vbadf[c]) * PF[0],
                                                       math.dist((p['x'], p['y']), VP[c][:2]), c))
    best = None; best_key = None
    def sample_price(tracks):



        cells = {}
        for tr in tracks:
            il = LI[tr['layer']]
            for a_, b_, width in zip(tr['pts'][:-1], tr['pts'][1:], tr['w']):
                count = max(2, int(math.ceil(math.dist(a_, b_) / RES)) + 1)
                for x_, y_ in np.linspace(a_, b_, count):
                    ix_, iy_ = int((x_ - X0) / RES), int((y_ - Y0) / RES)
                    if not (0 <= ix_ < NX and 0 <= iy_ < len(capH[il])): return math.inf
                    key = (il, iy_ * NX + ix_)
                    cells[key] = max(cells.get(key, 0.0), width)
        cost = 0.0
        for (il, cell), width in sorted(cells.items()):
            iy_, ix_ = divmod(cell, NX)
            if soft is not None:
                cost += PF[0] * max(0.0, width - float(soft[il][iy_, ix_])) / max(width, 1e-6)
            cost += float(hist[il * NN + cell])
        return cost

    for c in candidates:
        iy, ix = divmod(c, NX)
        if not (rng[0] <= ix < rng[1] and rng[2] <= iy < rng[3]): continue
        if not viaH[iy, ix] or (vbadf[c] and not BGA_PLAN) or not _vp_ok(net, VP[c][2]): continue
        x, y = VP[c][:2]; a = (p['x'], p['y']); v = (x, y)
        if net in PLANE_AREA and not PLANE_AREA[net].covers(_PT(x,y)): continue
        pts = _oct_move([a, v]) if oct_net(net) else [a, v]
        pts = [q for i, q in enumerate(pts) if i == 0 or math.dist(q, pts[i - 1]) > 1e-7]
        li = LI[b['layer']]
        if any(pad_room(li, a_, b_, net) < wk / 2 for a_, b_ in zip(pts[:-1], pts[1:])): continue
        trs = [dict(layer=b['layer'], pts=[list(q) for q in pts], w=[wk]*(len(pts)-1))] if len(pts) > 1 else []
        vs = [dict(x=x, y=y)]
        if surface:
            result = (None, trs, vs)
            if not BGA_ESCAPE: return result
            score = (price(c)[0] if BGA_PLAN else 0.0) + sample_price(trs)
            key = (score, math.dist(a, v), c)
            if math.isfinite(score) and (best_key is None or key < best_key): best_key, best = key, result
            continue
        z = cell_xy(ix, iy)
        order = inner
        if BGA_PLAN and soft is not None:
            order = sorted(inner, key=lambda il: (PF[0] * max(0.0, wk - float(soft[il][iy, ix])) / max(wk, 1e-6) + float(hist[il * NN + c]), inner.index(il)))
        for il in order:




            iw = wk if inner_w is None else max(wk, inner_w)
            if capH[il][iy, ix] < iw: continue
            ip = _oct_move([v, z]) if oct_net(net) else [v, z]
            ip = [q for i, q in enumerate(ip) if i == 0 or math.dist(q, ip[i - 1]) > 1e-7]
            if any(pad_room(il, a_, b_, net) < iw/2 for a_, b_ in zip(ip[:-1], ip[1:])): continue
            it = [dict(layer=LAYERS[il], pts=[list(q) for q in ip], w=[iw]*(len(ip)-1))] if len(ip) > 1 else []
            result = (il*NN+c, trs+it, vs)
            if not BGA_ESCAPE: return result
            score = (price(c)[0] if BGA_PLAN else 0.0) + sample_price(trs + it)
            key = (score, math.dist(a, v), c, inner.index(il))
            if math.isfinite(score) and (best_key is None or key < best_key): best_key, best = key, result
    return best














CHAIN = NC is not None and args.get('link_chain', '1' if NLAY >= 4 else '0') == '1'
CHAIN_VIA = float(args.get('chain_via', '10'))
CHAIN_DIST = float(args.get('chain_spacing', '0.15'))
CHAIN_MAX = float(args.get('chain_max', '6.0'))
CHAIN_DETOUR = float(args.get('chain_detour', '2.0')); CHAIN_DETOUR_MM = float(args.get('chain_detour_mm', '3.0'))
CHAIN_PIN = float(args.get('chain_pin', '4.0'))
CHAINS = {}
KF_AFTER = {}
KF_BEFORE = {}
_METRIC_OFF = set()
CHAIN_STAT = dict(element=0, without_zone=0, back=0, detour=0, rest_free=0)
CHAIN_REST = CHAIN and args.get('chain_rest', '1') == '1'
CHAIN_REST_ANT = float(args.get('chain_rest_share', '0.2')); CHAIN_REST_MM = float(args.get('chain_rest_mm', '5.0'))


def _clearance_min(src, tgt):
    s = np.unique(np.asarray(src, dtype=np.int64) % NN); t = np.unique(np.fromiter(tgt, dtype=np.int64, count=len(tgt)) % NN)
    sx, sy = (s % NX) * RES, (s // NX) * RES; tx, ty = (t % NX) * RES, (t // NX) * RES
    best = np.inf
    for i in range(0, len(s), 512):
        d2 = (sx[i:i + 512, None] - tx[None, :]) ** 2 + (sy[i:i + 512, None] - ty[None, :]) ** 2
        best = min(best, float(d2.min()))
    return math.sqrt(best) * (1.4142 / math.sqrt(2.0))


def _plen(res):
    L = 0.0
    for a, b in zip(res[:-1], res[1:]):
        if a // NN != b // NN: L += 0.6; continue
        ia, ib = a % NN, b % NN
        L += RES * (1.4142 if (ia % NX != ib % NX and ia // NX != ib // NX) else 1.0)
    return L


def _pad_dist(p, q):
    a, b = p['bbox'], q['bbox']
    return math.hypot(max(a[0] - b[2], b[0] - a[2], 0.0), max(a[1] - b[3], b[1] - a[3], 0.0))


def _chains_build():
    per = {}
    for r in sorted(NC.get('part_list', {})):
        t = NC['part_list'][r]; st = t.get('supports') or {}
        if not str(t.get('function', '')).startswith('decoupling_') or st.get('over_coil') or not t.get('C'): continue
        n, chip, pin = st.get('net_name'), st.get('ref'), st.get('pin')
        if not n or chip is None or pin is None or n in GNDS or n not in by_net_g: continue
        ck = [k for k in by_net_g[n] if D['pads'][k]['ref'] == r]
        if len(ck) != 1 or not any(L in LI for L in D['pads'][ck[0]]['layers']): continue


        group = tuple(sorted(str(x) for x in (st.get('pins') or [pin]))) if st.get('p3_group') else ()
        if group:


            instances = {}
            for k in by_net_g[n]:
                p = D['pads'][k]
                if p['ref'] == chip and str(p['num']) in group:
                    instances.setdefault(str(p['num']), []).append(k)
            candidates = [ks[0] for ks in instances.values() if len(ks) == 1
                          and any(L in LI for L in D['pads'][ks[0]]['layers'])]
            if not candidates: continue
            pk = [min(candidates, key=lambda k: (_pad_dist(D['pads'][ck[0]], D['pads'][k]), k))]
        else:
            pk = [k for k in by_net_g[n] if D['pads'][k]['ref'] == chip and str(D['pads'][k]['num']) == str(pin)]
            if len(pk) != 1 or not any(L in LI for L in D['pads'][pk[0]]['layers']): continue
        if _pad_dist(D['pads'][ck[0]], D['pads'][pk[0]]) > CHAIN_MAX: continue
        g = per.setdefault((chip, n, group), dict(chain_list={}, pins=set()))
        if group:
            g.setdefault('anchor', []).append((float(t['C']), r, pk[0]))
            pk = [group]
        g['chain_list'].setdefault(pk[0], []).append((float(t['C']), r, ck[0]))
        g['pins'].update(str(x) for x in (st.get('pins') or [pin]))
    for (chip, n, group), g in sorted(per.items()):
        if group:
            anchor = min(g['anchor'])[2]
            if not any(L in LI for L in D['pads'][anchor]['layers']): continue
            ks = [dict(pins=[anchor], caps=[c for _, _, c in sorted(cs)]) for cs in g['chain_list'].values()]
        else:
            ks = [dict(pins=[pk], caps=[c for _, _, c in sorted(cs)]) for pk, cs in sorted(g['chain_list'].items())]
        occupied = {k for K_ in ks for k in K_['pins'] + K_['caps']}
        for k in by_net_g[n]:
            p = D['pads'][k]
            if p['ref'] != chip or k in occupied or str(p['num']) not in g['pins'] or not any(L in LI for L in p['layers']): continue
            if group and sum(D['pads'][j]['ref'] == chip and str(D['pads'][j]['num']) == str(p['num']) for j in by_net_g[n]) != 1: continue
            d, i = min((_pad_dist(p, D['pads'][K_['caps'][0]]), i) for i, K_ in enumerate(ks))
            if d <= CHAIN_PIN: ks[i]['pins'].append(k)
        CHAINS.setdefault(n, []).extend(ks)
    for n in sorted(CHAINS):
        if n not in AREA_NETS: continue
        for K_ in CHAINS[n]:
            for a, b in [(k, K_['caps'][0]) for k in K_['pins']] + list(zip(K_['caps'][:-1], K_['caps'][1:])):
                KF_AFTER[a] = b; KF_BEFORE.setdefault(b, []).append(a)
    if CHAINS: print('Supply chains (R22): %d chains in %d nets, of which %d nets with area' % (
        sum(len(v) for v in CHAINS.values()), len(CHAINS), sum(1 for n in CHAINS if n in AREA_NETS)), flush=True)

if CHAIN: _chains_build()

def _rectangle_cells(bx, r):
    ixa = max(0, int((bx[0] - r - X0) / RES)); ixb = min(NX, int((bx[2] + r - X0) / RES) + 1)
    iya = max(0, int((bx[1] - r - Y0) / RES)); iyb = min(NY, int((bx[3] + r - Y0) / RES) + 1)
    if ixb <= ixa or iyb <= iya: return np.zeros(0, dtype=np.int64)
    GX, GY = np.meshgrid(GXc[ixa:ixb], GYc[iya:iyb])
    m = (np.maximum(bx[0] - GX, GX - bx[2]) <= r) & (np.maximum(bx[1] - GY, GY - bx[3]) <= r)
    yy, xx = np.nonzero(m)
    return (yy + iya) * NX + (xx + ixa)

def _metric(inner, traces, vias, vacant, wmin, dist=None):
    if dist is None: dist = CHAIN_DIST
    rb = wmin / 2 + dist; rv = VIA_R + dist; zb, zv = [], []
    for p in inner:
        for L in p['layers']:
            if L in LI: zb.append(LI[L] * NN + _rectangle_cells(p['bbox'], rb))
        zv.append(_rectangle_cells(p['bbox'], rv))
    for t in traces:
        li = LI[t['layer']]
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            zb.append(seg_cells(li, a, b, w + 2 * rb)); zv.append(seg_cells(0, a, b, w + 2 * rv))
    for v in vias:
        q = (v['x'], v['y'])
        for li in range(NL): zb.append(seg_cells(li, q, q, VIA_D + 2 * rb))
        zv.append(seg_cells(0, q, q, VIA_D + 2 * rv))
    zb = np.concatenate(zb) if zb else np.zeros(0, dtype=np.int64)
    zv = np.concatenate(zv) if zv else np.zeros(0, dtype=np.int64)
    for p in vacant:
        zb = zb[~np.isin(zb % NN, _rectangle_cells(p['bbox'], rb + 0.05))]
        zv = zv[~np.isin(zv, _rectangle_cells(p['bbox'], 0.0))]
    return zb, zv


def _metric_maps(capH, viaH, zb, zv):
    c2 = _Maps(); c2.planar = capH.planar.copy(); c2.planar[zb] = -1.0
    for li in range(NL): c2.append(c2.planar[li * NN:(li + 1) * NN].reshape(NY, NX))
    v2 = np.array(viaH, dtype=bool, copy=True); v2.reshape(-1)[zv] = False
    return c2, v2


def _metric_prepare(r, net):
    inner = set()
    for t in r['tracks']:
        if 'metric' in t: inner.update(t['metric'][1:])
    owned = {0: {k for k in by_net_g[net] if k not in inner}}
    for t in r['tracks']:
        if t.get('link_chain'): owned.setdefault(t['link_chain'], set()).update(t.get('kp') or ())
    geo = {k: {LI[L]: shapely.union_all([Polygon(q) for q in D['pads'][k]['polys'][L]]) for L in D['pads'][k]['layers'] if L in LI} for k in by_net_g[net]}
    G = {}
    for g in sorted(owned):
        own = owned[g]; e = {}; pf = {}; fr = {}
        for li in range(NL):
            ow = [geo[k][li] for k in sorted(own) if li in geo[k]]
            e[li] = shapely.union_all(ow) if ow else None
            pf[li] = e[li].buffer(CHAIN_DIST + 0.1) if ow else None
            fr[li] = [geo[k][li] for k in by_net_g[net] if k not in own and li in geo[k]]
        G[g] = dict(owned=e, buf_dist=pf, foreign=fr)
    xs = [q[0] for t in r['tracks'] if t.get('link_chain') for q in t['pts']] + [D['pads'][k]['bbox'][i] for k in inner for i in (0, 2)]
    ys = [q[1] for t in r['tracks'] if t.get('link_chain') for q in t['pts']] + [D['pads'][k]['bbox'][i] for k in inner for i in (1, 3)]
    box = (min(xs) - 2.5, min(ys) - 2.5, max(xs) + 2.5, max(ys) + 2.5) if xs else (0.0, 0.0, 0.0, 0.0)
    return dict(G=G, box=box)


def _metric_width(metric, r, width_list, ti, a, b, li):
    g = r['tracks'][ti].get('link_chain') or 0; bx = metric['box']
    if g == 0 and (max(a[0], b[0]) < bx[0] or min(a[0], b[0]) > bx[2] or max(a[1], b[1]) < bx[1] or min(a[1], b[1]) > bx[3]): return 9.0
    G = metric['G'][g]; seg = _LS([a, b]) if a != b else _PT(a)
    if G['owned'][li] is not None: seg = seg.difference(G['owned'][li])
    if seg.is_empty: return 9.0
    d = 9.0
    for q in G['foreign'][li]: d = min(d, seg.distance(q))
    pf = G['buf_dist'][li]
    for tj, t2 in enumerate(r['tracks']):
        if (t2.get('link_chain') or 0) == g or LI[t2['layer']] != li: continue
        for a2, b2, w2 in zip(t2['pts'][:-1], t2['pts'][1:], width_list[tj]):
            if min(a2[0], b2[0]) - 2.0 > max(a[0], b[0]) or max(a2[0], b2[0]) + 2.0 < min(a[0], b[0]) or \
               min(a2[1], b2[1]) - 2.0 > max(a[1], b[1]) or max(a2[1], b2[1]) + 2.0 < min(a[1], b[1]): continue
            s2 = _LS([a2, b2]) if a2 != b2 else _PT(a2)
            if pf is not None: s2 = s2.difference(pf)
            if not s2.is_empty: d = min(d, seg.distance(s2) - w2 / 2)
    for v in r['vias']:
        if (v.get('link_chain') or 0) == g: continue
        q = _PT(v['x'], v['y'])
        if pf is None or not pf.contains(q): d = min(d, seg.distance(q) - VIA_R)
    return 2 * (d - CHAIN_DIST)


def _kf_zone(k, p, wmin):
    before, st = [], [k]
    while st:
        for a in KF_BEFORE.get(st.pop(), ()): before.append(a); st.append(a)
    if not before: return None
    sp = [t for a in before if 'GND#%d' % a in U for t in U['GND#%d' % a]['tracks']]
    vv = [v for a in before if 'GND#%d' % a in U for v in U['GND#%d' % a]['vias']]
    return _metric([D['pads'][a] for a in before], sp, vv, [p], wmin)


def _kf_element(u, k, p, net, C, src, fst, wk, maps_):
    capH, viaH, capS, dd, rng, viaBad, capC, capCf, vbadf = maps_
    q = D['pads'][KF_AFTER[k]]; bx, qb = p['bbox'], q['bbox']
    Cb = dict(C, via=3.0 * CHAIN_VIA if set(p['layers']) & set(q['layers']) else 0.0)
    win = (min(bx[0], qb[0]) - 3.0, min(bx[1], qb[1]) - 3.0, max(bx[2], qb[2]) + 3.0, max(bx[3], qb[3]) + 3.0)
    if not _in_window(win, rng): return False
    tgt = set()
    for L, cells in q['cells'].items(): tgt.update((LI[L] * NN + cells).tolist())
    escape_target = {c: (l_, pts) for c, l_, pts in escape(net, q, wk, capH, rng)}; tgt.update(escape_target)
    src = sorted({c for c in src if c not in tgt})
    if not src or not tgt: return False
    res = None; zn = _kf_zone(k, p, Cb['wmin'])
    if zn is not None:
        cH, vH = _metric_maps(capH, viaH, *zn)
        res = astar_pf(src, tgt, qb, cH, capS, vH, viaBad, Cb, win)
        if not res: CHAIN_STAT['without_zone'] += 1
    if not res: res = astar_pf(src, tgt, qb, capH, capS, viaH, viaBad, Cb, win)
    if not res: return False
    trs, vs = path_to_geometry(res, capC, dict(Cb, w=Cb['wmin'])) if len(res) > 1 else ([], [])
    if sum(math.dist(a_, b_) for t in trs for a_, b_ in zip(t['pts'][:-1], t['pts'][1:])) > CHAIN_DETOUR * _pad_dist(p, q) + CHAIN_DETOUR_MM:
        CHAIN_STAT['detour'] += 1; return None
    for c_, f_ in ((res[0], fst), (res[-1], escape_target)):
        if c_ in f_: trs.append(dict(layer=f_[c_][0], pts=f_[c_][1], w=[wk, wk]))
    for t in trs: t['link_chain'] = True
    add_unit(u, net, trs, vs); CHAIN_STAT['element'] += 1
    return True


def n_sv(r):
    return sum(1 for v in r['vias'] if not v.get('rv') and not v.get('gr'))
def cons_missing(u):
    vs = U[u]['vias']
    return sum(max(0, v['n'] - 1 - sum(1 for w in vs if w.get('too') == [v['x'], v['y']])) for v in vs if v.get('n'))

















TAP_ON = TAPS and args.get('tap_spacing', '1' if NC else '0') == '1'
TAP_CLR = float(args.get('tap_clr', '%g' % CLR))
TAP_TREE = TAPS and args.get('tap_tree', '1' if NC else '0') == '1'
TAP_LAYER = float(args.get('tap_layer', '0')) if TAPS else 0.0


TAP_VIA = float(args.get('tap_via', '1')) if TAPS else 1.0
TAP_LAYER_D = float(args.get('tap_layer_d', '0.8'))
BRANCH = args.get('branch', '0') == '1' or R27_TRUNK
BRANCH_D = float(args.get('branch_d', '0.6')); BRANCH_L = float(args.get('branch_l', '1.0')); BRANCH_DEG = float(args.get('branch_deg', '60'))
BRANCH_FACTOR = float(args.get('branch_factor', '3.0')); BRANCH_MORE = float(args.get('branch_more', '1.5')); STAR_R = float(args.get('star_r', '0.3'))
TOPO_STAT = dict(taps=0, tap_without_zone=0, branch_checked=0, branch_shortage=0, branch_better=0, star_blocked=0)


def _arm_direction(pts, i, step, p, r=0.15):
    q = None; k = i
    while 0 <= k < len(pts):
        q = pts[k]
        if math.dist(q, p) >= r: break
        k += step
    if q is None or math.dist(q, p) < 0.02: return None
    return math.degrees(math.atan2(q[1] - p[1], q[0] - p[0])) % 360.0


def _arms_at(traces, li, p):
    raw_data = []
    for t in traces:
        if LI[t['layer']] != li: continue
        P = t['pts']
        for k, (a, b) in enumerate(zip(P[:-1], P[1:])):
            if min(a[0], b[0]) - 0.04 > p[0] or max(a[0], b[0]) + 0.04 < p[0] or min(a[1], b[1]) - 0.04 > p[1] or max(a[1], b[1]) + 0.04 < p[1]: continue
            dx, dy = b[0] - a[0], b[1] - a[1]; ll = dx * dx + dy * dy
            f = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / ll)) if ll > 0 else 0.0
            pf = (a[0] + f * dx, a[1] + f * dy)
            if math.dist(p, pf) > 0.036: continue
            raw_data += [_arm_direction(P, k + 1, 1, pf), _arm_direction(P, k, -1, pf)]
    arms = []
    for w in sorted(x for x in raw_data if x is not None):
        if not arms or w - arms[-1] > 3.0: arms.append(w)
    if len(arms) > 1 and arms[0] + 360.0 - arms[-1] <= 3.0: arms.pop()
    return arms


def _parallel_mm(trs, traces):
    ges = 0.0
    for t in trs:
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            L = math.dist(a, b)
            if L < 1e-6: continue
            ux, uy = (b[0] - a[0]) / L, (b[1] - a[1]) / L
            for t2 in traces:
                if t2['layer'] != t['layer']: continue
                for c, d in zip(t2['pts'][:-1], t2['pts'][1:]):
                    L2 = math.dist(c, d)
                    if L2 < 1e-6 or abs(ux * (d[1] - c[1]) - uy * (d[0] - c[0])) / L2 > 0.2588: continue
                    sc = (c[0] - a[0]) * ux + (c[1] - a[1]) * uy; sd = (d[0] - a[0]) * ux + (d[1] - a[1]) * uy
                    lo, hi = max(0.0, min(sc, sd)), min(L, max(sc, sd))
                    if hi - lo <= 1e-6 or abs(sd - sc) < 1e-9: continue
                    nc = -(c[0] - a[0]) * uy + (c[1] - a[1]) * ux; nd = -(d[0] - a[0]) * uy + (d[1] - a[1]) * ux
                    e0 = nc + (nd - nc) * (lo - sc) / (sd - sc); e1 = nc + (nd - nc) * (hi - sc) / (sd - sc)
                    if max(abs(e0), abs(e1)) < BRANCH_D and min(abs(e0), abs(e1)) > 0.02: ges += hi - lo
    return ges


def _branch_defects(trs, traces, li, p, at_pad):
    arms = [] if at_pad else _arms_at(traces, li, p)
    wn = None
    for t in trs:
        if LI[t['layer']] == li and math.dist(t['pts'][0], p) < 0.08: wn = _arm_direction(t['pts'], 1, 1, p); break
    wk = min((abs((wn - a + 180.0) % 360.0 - 180.0) for a in arms), default=180.0) if wn is not None else 180.0
    par = _parallel_mm(trs, traces)
    return (len(arms) >= 3, par if par > BRANCH_L else 0.0, wk < BRANCH_DEG - (1e-6 if R27_TRUNK else 1.0))


def _bands(ex, traces, wide, factor):
    m = np.zeros(NL * NN, dtype=bool)
    for t in traces:
        li = LI[t['layer']]
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']): m[seg_cells(li, a, b, w + 2 * wide)] = True
    e = np.ones(NL * NN, dtype=np.float32) if ex is None else ex.copy()
    e[m] *= np.float32(factor)
    return e


def _tap_prepare(r, net):
    lane = {li: [] for li in range(NL)}; ends = []
    for t in r['tracks']:
        if not t.get('tap'): continue
        ends += [t['pts'][0], t['pts'][-1]]
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            if a != b: lane[LI[t['layer']]].append((_LS([a, b]), w / 2))
    vv = [_PT(v['x'], v['y']) for v in r['vias'] if v.get('tap')]
    vacant = {}
    for li in range(NL):
        g = [g2 for n2, g2 in _PADG[li] if n2 == net and any(g2.distance(_PT(e)) <= 0.05 for e in ends)]
        vacant[li] = shapely.union_all(g).buffer(TAP_CLR + 0.1) if g else None
    xs = [e[0] for e in ends] + [q.x for q in vv]; ys = [e[1] for e in ends] + [q.y for q in vv]
    for li in range(NL):
        for s2, _ in lane[li]: b_ = s2.bounds; xs += [b_[0], b_[2]]; ys += [b_[1], b_[3]]
    return dict(lane=lane, vias=vv, vacant=vacant, box=(min(xs) - 2.5, min(ys) - 2.5, max(xs) + 2.5, max(ys) + 2.5))


def _tap_width(ab, a, b, li):
    bx = ab['box']
    if max(a[0], b[0]) < bx[0] or min(a[0], b[0]) > bx[2] or max(a[1], b[1]) < bx[1] or min(a[1], b[1]) > bx[3]: return 9.0
    seg = _LS([a, b]) if a != b else _PT(a); fr = ab['vacant'][li]
    if fr is not None: seg = seg.difference(fr)
    if seg.is_empty: return 9.0
    d = 9.0
    for s2, h2 in ab['lane'][li]:
        if fr is not None: s2 = s2.difference(fr)
        if not s2.is_empty: d = min(d, seg.distance(s2) - h2)
    for q in ab['vias']:
        if fr is None or not fr.contains(q): d = min(d, seg.distance(q) - VIA_R)
    return 2 * (d - TAP_CLR)


def route_unit(u):
    work_step('unit')
    ok = _route_unit_impl(u)
    net = U.get(u,{}).get('net',u)
    if not ok and PIN_ACCESS and not _pz_active(net) and net in PZ_WIDTHS and not u.startswith(('GND#','FIX#')):
        C=cls(net);capH,viaH=hard_maps(net)
        blocked=[]
        for k in by_net[net]:
            p=D['pads'][k]
            cells=[(LI[L],cs) for L,cs in p['cells'].items() if L in LI]
            free=any(np.any(capH[li].ravel()[cs]>=C['wmin']) for li,cs in cells)



            cut = '%s:%s' % (p['ref'],p['num']) in UNREACHABLE.get(net,[])
            if cut or (not free and not escape(net,p,C['wmin'],capH,_WHOLE)):blocked.append(id(p))
        if blocked:
            PZ_LOCAL[net]=set(blocked);_pad_room_cached.cache_clear()
            for key in list(_ERR):
                if key[0]==net:_ERR.pop(key)
            _ERR_NETS.difference_update(key for key in list(_ERR_NETS) if key[0]==net)
            print('Pin access local:',net,'cut-off pads',len(blocked),flush=True)
            del_unit(u)
            ok=_route_unit_impl(u)
    return ok


_PLANE_GROUP_ACTIVE = [None]


def route_plane_group(net):
    old=_PLANE_GROUP_ACTIVE[0];_PLANE_GROUP_ACTIVE[0]=net
    tr=[];vs=[];complete=True;del_unit(net)
    try:
        for k in PLANE_GROUPS[net]:
            child='GND#%d'%k
            try:
                ok=_route_unit_impl(child);r=U.get(child)
                if r:tr.extend(r['tracks']);vs.extend(r['vias'])
                complete &= bool(ok)
            finally:del_unit(child)
            del_unit(net);add_unit(net,net,tr,vs)
            vs=list(U[net]['vias'])
        return complete
    finally:_PLANE_GROUP_ACTIVE[0]=old


def _route_unit_impl(u):
    if globals().get('_LOOP_DEADLINE') is not None: _loop_cpu_check()
    if pair_unit(u): return route_pair(u)
    if u in PLANE_GROUPS: return route_plane_group(u)
    ASTAR_COST[0] = None; ASTAR_HINT[0] = None; VIA_TOO[0] = None
    NET_CUR[0] =D['pads'][int(u[4:])]['net'] if u.startswith('GND#') else u
    if u.startswith('GND#'):
        k = int(u[4:]); p = D['pads'][k]; net = p['net']
        _direct_plane = any(net in FL_ASSIGNMENT.get(L,[]) for L in PLANE_SIGNALS)
        if _direct_plane and (net not in PLANE_AREA or PLANE_AREA[net].is_empty):return False
        C = dict(w=max(0.35, TRACK_MIN), wmin=max(0.15, TRACK_MIN), pen=0.5, via=0.0, inner=99.0)
        bx = p['bbox']
        box = (bx[0] - 6.1, bx[1] - 6.1, bx[2] + 6.1, bx[3] + 6.1)
        capH, viaH, capS, dd, rng, viaBad, capC, capCf, vbadf = maps(net, box, C['w'])
        L = next(x for x in p['layers'] if x in LI)
        src = (LI[L] * NN + p['cells'][L]).tolist()
        _wk = min(C['wmin'], NECK_W) if neck_active(net, C, C['wmin'], True) else C['wmin']
        _fst = {c: (l_, pts) for c, l_, pts in escape(net, p, _wk, capH, rng)}
        src += [c for c in sorted(_fst) if c not in src]
        if CHAIN and not _direct_plane and k in KF_AFTER and (net, k) not in _METRIC_OFF:
            _cgfx = _kf_element(u, k, p, net, C, src, _fst, _wk, (capH, viaH, capS, dd, rng, viaBad, capC, capCf, vbadf))
            if _cgfx: return True
            if _cgfx is False: _METRIC_OFF.add((net, k))
            CHAIN_STAT['back'] += 1
        if BGA_FANOUT:
            _bf = bga_escape(net, p, _wk, capH, viaH, vbadf, rng, surface=True, soft=capS)
            if _bf: add_unit(u, net, _bf[1], _bf[2]); return True
        gnd_tgt = set()
        _plane_distance = PLANE_AREA[net].distance(_PT(p['x'],p['y'])) if _direct_plane and net in PLANE_AREA else 0.
        if _plane_distance > 6.:



            for owner,r in U.items():
                if _PLANE_GROUP_ACTIVE[0]==net and owner!=net:continue
                if r['net']!=net or not any(PLANE_AREA[net].covers(_PT(v['x'],v['y'])) for v in r['vias']):continue
                for t in r['tracks']:
                    for q in t['pts']:
                        ix,iy=int((q[0]-X0)/RES),int((q[1]-Y0)/RES)
                        if 0<=ix<NX and 0<=iy<NY:gnd_tgt.add(LI[t['layer']]*NN+iy*NX+ix)
        if k in EPM:
            _ev = ep_set(k, viaH, vbadf, rng)
            if _ev: add_unit(u, net, [], _ev); return True
        _epr = EP_RES.get(net) if k not in EPM else None
        if _epr is not None: viaH = viaH & ~_epr
        _vz = None; _via_cost = 0.0
        if VAP and not _direct_plane:
            _vzc, _vzw = _vap_cells(k); _VZ[_vzc] = _vzw; _vz = _VZ
            _via_cost = VAP_K * (VAP_WF if k in VAP_IMPORTANT else 1.0)
        if VIA_DENSITY:
            _dv = _vd_cost().ravel() * np.float32(VD_MM)
            if _dv.any():


                _vz = _dv if _vz is None else _via_cost * _vz.astype(np.float64) + _dv
                _via_cost = 1.0

        _zs = _kf_zone(k, p, C['wmin']) if CHAIN and not _direct_plane and k in KF_BEFORE and (net, k, 2) not in _METRIC_OFF else None
        res=None
        for _zn in ([_zs] if _zs is not None else []) + [None]:
            _windows = (1.8,3.5,6.0) if _plane_distance <= 6. else (max(6.,_plane_distance+2.),)
            for ml in _windows:
                win = (bx[0] - ml, bx[1] - ml, bx[2] + ml, bx[3] + ml)
                if not _in_window(win, rng):
                    (capH, viaH, capS, dd, rng, viaBad, capC, capCf, vbadf), _b2 = maps_larger(net, box, win, C['w'])
                    if _b2 is not None: box = _b2
                    if _epr is not None: viaH = viaH & ~_epr
                _cH, _vH = (capH, viaH) if _zn is None else _metric_maps(capH, viaH, *_zn)
                if _direct_plane and _plane_distance<=6.:
                    _vH=_vH & PLANE_MASK[net]
                targets=gnd_tgt;target_box=(p['x'],p['y'],p['x'],p['y']);via_target=True
                if _plane_distance>6.:
                    targets=gnd_tgt | plane_feeder_targets(net,p,LI[L],_vH,viaBad,rng)
                    if not targets:continue
                    xy=[cell_xy((c%NN)%NX,(c%NN)//NX) for c in sorted(targets)]
                    target_box=(min(q[0] for q in xy),min(q[1] for q in xy),max(q[0] for q in xy),max(q[1] for q in xy))
                    via_target=False
                res = astar_pf(src, targets, target_box, _cH, capS, _vH, viaBad, C, win, via_target=via_target, viaz=_vz, vzk=_via_cost)
                if res:
                    conf = any(capCf[q] < C['wmin'] for q in res) or (res[-1] not in gnd_tgt and bool(vbadf[res[-1] % NN]))
                    if not conf or ml >= 6.0: break
                    if NB is not None and NB.info[0] == 0: break
            if res or _zn is None: break
            _METRIC_OFF.add((net, k, 2)); CHAIN_STAT['without_zone'] += 1
        if VAP and not _direct_plane and _vz is not None: _VZ[_vzc] = 1.0
        if not res: return False
        trs, _mid_vias = path_to_geometry(res, capC, dict(C, w=C['wmin'])) if len(res) > 1 else ([], [])
        if res[0] in _fst: trs.append(dict(layer=_fst[res[0]][0], pts=_fst[res[0]][1], w=[_wk] * (len(_fst[res[0]][1])-1)))
        n = res[-1]; vx, vy = cell_xy((n % NN) % NX, (n % NN) // NX)
        _s = _vp_xy(net, n % NN) if VP and n not in gnd_tgt else None
        if _s and _direct_plane and not PLANE_AREA[net].covers(_PT(*_s)):_s=None
        if _s:
            vx, vy = _s
            for t in trs:
                if LI[t['layer']] == n // NN and math.hypot(t['pts'][-1][0] - vx, t['pts'][-1][1] - vy) < 0.05: t['pts'][-1] = [round(vx, 4), round(vy, 4)]
        add_unit(u, net, trs, _mid_vias+([] if n in gnd_tgt else [dict(x=round(vx, 4), y=round(vy, 4))]))
        return True
    net = u; C = cls(net)
    pads = [D['pads'][k] for k in by_net[net]]
    pbx = (min(p['bbox'][0] for p in pads), min(p['bbox'][1] for p in pads), max(p['bbox'][2] for p in pads), max(p['bbox'][3] for p in pads))
    box = (pbx[0] - 8.5, pbx[1] - 8.5, pbx[2] + 8.5, pbx[3] + 8.5)
    _m0 = WINDOW if WINDOW else 3.0
    ASTAR_HINT[0] = (max(1, int((pbx[1] - _m0 - Y0) / RES)), min(NY - 2, int((pbx[3] + _m0 - Y0) / RES)),
                     max(1, int((pbx[0] - _m0 - X0) / RES)), min(NX - 2, int((pbx[2] + _m0 - X0) / RES)))
    Cs = cls('/__signal__'); _wmax = max(C['w'], Cs['w'])
    _sz_fl = current_model(net)
    if _sz_fl: _wmax = max(_wmax, sum(map(float,_sz_fl['load'].values())) / (CURRENT_J * min(CURRENT_T.values())))
    RK = rv_map(net, box, C['via'] * VIA_FACTOR[0]) if RET > 0 else None
    _via_anchor = via_surcharge(u, net, C, box) if (VIA_ANCHOR > 0 or K12 > 0) else None
    _via_anchor_prev = _via_anchor
    if VIA_DENSITY:
        _dv = _vd_cost().ravel() * np.float32(VD_MM)
        if _dv.any(): _via_anchor = _dv if _via_anchor is None else _via_anchor + _dv
    if RK is not None or _via_anchor is not None: VIA_TOO[0] = (RK['via_extra'] if RK else None, RV_PAIR if RK else None, _via_anchor)
    capH, viaH, capS, dd, rng, viaBad, capC, capCf, vbadf = maps(net, box, _wmax)

    k = net.split('/')[-1]; taps = []; kel = {}
    if TAP_NC:
        _ak = {'%s:%s' % (p['ref'], p['num']): i for i, p in enumerate(pads)}
        _ae = [e for e in NC['taps'].get(net, []) if e['pad'] in _ak]
        taps = sorted(_ak[e['pad']] for e in _ae)
        for e in _ae:
            if e.get('start') in _ak: kel[_ak[e['pad']]] = [_ak[e['start']]]

    power = [i for i in range(len(pads)) if i not in taps]
    EXTRA = disturb_extra(net)
    if VIA_DENSITY and NC and (NC['net_list'].get(net) or {}).get('category') in ('current_val', 'supply') and VD_CURRENT > 0:


        _dc = np.square(np.maximum(np.float32(0), _vd_window_max(_VD_COUNT.astype(np.float32)) - np.float32(VD_THRESHOLD * VD_WINDOW ** 2)))
        if _dc.any():
            _de = np.tile(1.0 + np.float32(VD_CURRENT) * _dc.ravel(), NL)
            EXTRA = _de if EXTRA is None else EXTRA * _de
    if net in PAIR_NETS: EXTRA = pair_extra(net, C, capS, EXTRA)
    _tap_sp, _tap_vi, _tap_kn, _tap_pads = [], [], [], []
    _star = set(); _pn_set = set()

    KT = None; _inner = set(); trunk = power
    if CHAIN and net in CHAINS and not taps:
        _local = {k_: i for i, k_ in enumerate(by_net[net])}
        KT = [dict(nr=_n, pins=[_local[k_] for k_ in K_['pins']], caps=[_local[k_] for k_ in K_['caps']], pin_ok=[], end=-1, traces=[], vias=[], kn=[], stop=False)
              for _n, K_ in enumerate(CHAINS[net])]
        _inner = {i for K_ in KT for i in K_['pins'] + K_['caps'][:-1]}
        trunk = [i for i in power if i not in _inner]
    BF = {}; BF_NODES = {}; _bf_done = set()
    _bga_own_holes = BGA_PLAN and (args.get('bga_own_via', '0') == '1' or BGA_ESCAPE)
    if BGA_FANOUT and (KT is None or BGA_PLAN) and not taps:
        _bf_viaH = viaH
        _bw = min(C['wmin'], NECK_W) if neck_active(net, C, C['wmin']) else C['wmin']
        for i in sorted(power, key=lambda i: (BGA.get(id(pads[i]), {}).get('ring', -1), pads[i]['ref'], pads[i]['num'])):
            _bf = bga_escape(net, pads[i], _bw, capH, _bf_viaH, vbadf, rng, soft=capS,
                             inner_w=C['wmin'] if args.get('bga_inner_width', '0') == '1' or BGA_ESCAPE else None)
            if _bf:
                BF[i] = _bf; _old = BF_NODES.setdefault(_bf[0], ([], []))
                _old[0].extend(_bf[1]); _old[1].extend(_bf[2])


                if _bga_own_holes: _bf_viaH = bga_own_hole_map(_bf_viaH, _bf[2])
    _bga_reserved_vias = [v for bf in BF.values() for v in bf[2]] if _bga_own_holes else []
    if _sz_fl and not trunk:
        CURRENT_AUDIT[net]=dict(declined='current_first: source/load only as taps, no current trunk ' + net)
        add_unit(u,net,[],[]);return False
    cx = sum(pads[i]['x'] for i in trunk) / len(trunk); cy = sum(pads[i]['y'] for i in trunk) / len(trunk)
    first = min(trunk, key=lambda i: math.hypot(pads[i]['x'] - cx, pads[i]['y'] - cy))
    FL = NC['net_list'].get(net, {}).get('flow') if NC else None
    if FL and FL.get('source') and args.get('flow_start') == '1':
        qs = [i for i in power if '%s:%s' % (pads[i]['ref'], pads[i]['num']) in FL['source']]
        if qs: first = qs[0]
        if first in _inner: first = next(K_['caps'][-1] for K_ in KT if first in K_['pins'] + K_['caps'])
    try: SP = current_plan(net, pads, trunk, KT)
    except ValueError as e:


        CURRENT_AUDIT[net]=dict(declined=str(e))
        add_unit(u,net,[],[]);return False
    if SP: first = SP[0]
    tree = [first]; done_xy = [(pads[first]['x'], pads[first]['y'])]
    rest = [i for i in trunk if i != first]
    all_tr, all_v, pathnodes, ok = [], [], [], True
    if globals().get('PZ_REST') and HARD_FOREIGN[0] and globals().get('_LOOP_DEADLINE') is not None:
        _PZ_PARTIAL[0] = (u, net, all_tr, all_v)
    _stubs = set()
    order = []
    while rest:
        j = min(rest, key=lambda i: min(math.hypot(pads[i]['x'] - x, pads[i]['y'] - y) for x, y in done_xy))
        rest.remove(j); order.append(j); done_xy.append((pads[j]['x'], pads[j]['y']))
    if SP: order = SP[1]
    order += taps
    done_xy = [(pads[first]['x'], pads[first]['y'])]
    jobs = [(j, None) for j in order]
    _kzz = None; _kzr = None; _metric_k = None; tree_cons = tree
    if KT is not None:
        _gl = []
        for K_ in KT:
            _gl.append((K_['caps'][0], (K_, 0, [K_['pins'][0]])))
            _gl += [(q_, (K_, 0, None)) for q_ in K_['pins'][1:]]
            _gl += [(b_, (K_, s_ + 1, [a_])) for s_, (a_, b_) in enumerate(zip(K_['caps'][:-1], K_['caps'][1:]))]
        jobs = _gl + jobs; tree_cons = list(tree)
    _hint0 = ASTAR_HINT[0]
    for j, gl in jobs:
        if globals().get('_LOOP_DEADLINE') is not None: _loop_cpu_check()
        if VIA_DENSITY and all_v:
            _dv = _vd_cost(all_v).ravel() * np.float32(VD_MM)
            _via_anchor = _dv if _via_anchor_prev is None else _via_anchor_prev + _dv
            VIA_TOO[0] = (RK['via_extra'] if RK else None, RV_PAIR if RK else None, _via_anchor)
        p = pads[j]; is_tap = j in taps; Cb = (dict(Cs, via=Cs['via'] * TAP_VIA) if TAP_VIA != 1.0 else Cs) if is_tap else C
        _sz_parent = SP[2].get(j, first) if SP else None
        if SP and not is_tap:
            _sz_key = ('trunk',j) if gl is None else ('link_chain',gl[0]['nr'],gl[1],j)
            _sz_I = SP[3].get(_sz_key, sum(map(float,_sz_fl['load'].values())))
            Cb = current_net_class(Cb,_sz_I,pads[_sz_parent] if gl is None else pads[(gl[2] or gl[0]['pins'])[0]],p)
            Cb['current_vias'] = tuple((v['x'],v['y']) for v in all_v)
        ASTAR_HINT[0] = _hint0 if gl is None else None
        _zone = None
        _tap_z = False; _ex = EXTRA
        if gl is not None:
            K_, _level, _qp = gl
            if K_['stop'] or (_level > 0 and K_['end'] != _level - 1): continue
            if _qp is None: _qp = K_['pin_ok'] + K_['caps'][:1]
            src = list(K_['kn']) if gl[2] is None else []
            for i in _qp:
                if i in BF: src.append(BF[i][0])
                else:
                    for L, cells in pads[i]['cells'].items(): src.extend((LI[L] * NN + cells).tolist())
            if any(set(pads[i]['layers']) & set(p['layers']) for i in _qp) and not (BGA_PLAN and (j in BF or any(i in BF for i in _qp))):
                Cb = dict(Cb if SP else C, via=Cb['via'] * CHAIN_VIA)
            if (net, by_net[net][j]) not in _METRIC_OFF:
                _mine = (set(K_['pins']) | {K_['caps'][0]}) if _level == 0 else (set(_qp) | {j})
                _zone = _metric([pads[i] for i in power if i not in _mine], K_['traces'] if _level > 0 else [], K_['vias'] if _level > 0 else [],
                            [pads[i] for i in sorted(_mine)], Cb['wmin'])
        elif j in kel:
            src = list(_tap_kn) if TAP_TREE else []
            for i in kel[j] + (_tap_pads if TAP_TREE else []):
                for L, cells in pads[i]['cells'].items(): src.extend((LI[L] * NN + cells).tolist())
            _current = [t for t in all_tr if not t.get('tap')]
            if TAP_ON:
                _cgfx = set(kel[j]) | {t_ for t_ in kel if kel[t_] == kel[j]}
                _zone = _metric([pads[i] for i in range(len(pads)) if i not in _cgfx], _current, [v for v in all_v if not v.get('tap')],
                            [pads[i] for i in sorted(_cgfx)], Cb['wmin'], TAP_CLR + QM)
                _tap_z = True
                if not (len(_zone[0]) or len(_zone[1])): _zone = None
            if TAP_LAYER > 0 and _current:
                _ex = _bands(EXTRA, _current, TAP_LAYER_D, 1.0 + 2.0 * Cb['via'] * VIA_FACTOR[0] / TAP_LAYER)
        else:
            _si_chain = SI_STUB and actual_fast(net) and gl is None and not is_tap
            _sz_sources = [_sz_parent] if SP and not is_tap and _sz_parent in tree else ([] if SP and not is_tap else None)
            src = [] if _si_chain or _sz_sources is not None else ([q for q in pathnodes if q not in _star] if _star else list(pathnodes))
            for i in (_sz_sources if _sz_sources is not None else (tree[-1:] if _si_chain else tree)):
                if i in BF: src.append(BF[i][0])
                else:
                    for L, cells in pads[i]['cells'].items(): src.extend((LI[L] * NN + cells).tolist())
            if KT is not None and not is_tap:
                if _kzz is None:
                    _zi = [pads[i] for K_ in KT for i in K_['pin_ok'] + K_['caps'][:max(K_['end'], 0)]]
                    _zf = [pads[K_['caps'][K_['end']]] for K_ in KT if K_['end'] >= 0]
                    _kzz = _metric(_zi, [t for K_ in KT for t in K_['traces']], [v for K_ in KT for v in K_['vias']], _zf, C['wmin'])
                    _kzr = _metric(_zi, [], [], _zf, C['wmin'])
                    tree_cons += [i for K_ in KT if K_['end'] >= 0 and K_['caps'][K_['end']] == first for i in K_['pin_ok'] + K_['caps'][:K_['end']]]
                if len(_kzz[0]) and (net, by_net[net][j]) not in _METRIC_OFF: _zone = _kzz
                elif len(_kzr[0]) and (net, by_net[net][j], 1) not in _METRIC_OFF: _zone = _kzr
            if TAP_ON and _tap_sp and _zone is None:
                _zone = _metric([], _tap_sp, _tap_vi, [pads[i] for i in sorted({i_ for t_ in kel for i_ in kel[t_]} | set(_tap_pads))], Cb['wmin'], TAP_CLR + QM)
                _tap_z = True
                if not (len(_zone[0]) or len(_zone[1])): _zone = None
        tgt = set()
        if j in BF: tgt.add(BF[j][0])
        else:
            for L, cells in p['cells'].items(): tgt.update((LI[L] * NN + cells).tolist())
        if j in BF and BF[j][0] in src:


            if BGA_PLAN and gl is None:
                _c = BF[j][0]
                for _i in tree + [j]:
                    if _i in BF and BF[_i][0] == _c and _i not in _bf_done:
                        _bf_done.add(_i); all_tr += BF[_i][1]; all_v += BF[_i][2]
                _stubs.add(_c)
                tree.append(j); continue
            if not BGA_PLAN:
                all_tr += BF[j][1]; all_v += BF[j][2]; tree.append(j); continue
        if gl is None and not is_tap and j not in kel:
            tcells = set()
            for i in tree:
                for L, cells in pads[i]['cells'].items(): tcells.update((LI[L] * NN + cells).tolist())
            if tgt & tcells: tree.append(j); continue
        _wk = min(Cb['wmin'], NECK_W) if neck_active(net, Cb, Cb['wmin']) else Cb['wmin']
        _fst, _escape_target = {}, {}
        if ESCAPE:
            for i in (_qp if gl is not None else (kel[j] if j in kel else (_sz_sources if _sz_sources is not None else (tree[-1:] if SI_STUB and actual_fast(net) and not is_tap else tree)))):
                for c, l_, pts in escape(net, pads[i], _wk, capH, rng): _fst[c] = (l_, pts)
            for c, l_, pts in escape(net, p, _wk, capH, rng): _escape_target[c] = (l_, pts)
            _gem = sorted(c for c in _escape_target if c in _fst)
            if _gem:


                _c = _gem[0]; trs = [dict(layer=_escape_target[_c][0], pts=_escape_target[_c][1], w=[_wk] * (len(_escape_target[_c][1])-1))]
                if _c not in _stubs: trs.append(dict(layer=_fst[_c][0], pts=_fst[_c][1], w=[_wk] * (len(_fst[_c][1])-1)))
                _stubs.add(_c)
                if is_tap:
                    for t in trs: t['sig'] = True
                    if j in kel and (TAP_ON or TAP_TREE):
                        for t in trs: t['tap'] = True
                        _tap_sp += trs; _tap_pads.append(j)
                else: _kn = geo_nodes(trs, []); pathnodes.extend(_kn); _pn_set.update(_kn)
                all_tr += trs; tree.append(j); continue
            src.extend(_fst); tgt.update(_escape_target)
        _bf_shared = sorted(set(src) & tgt) if BGA_PLAN and gl is not None and j in BF else []
        src = sorted(set(src)) if _bf_shared else sorted({q for q in src if q not in tgt})
        if j in kel:
            src_ok = [q for q in src if capCf[q] >= Cb['wmin']]
            if src_ok: src = src_ok
        if CURRENT_VIA_PRICE and CONS and not is_tap and not SP:
            _with_cons = [i for K_ in (KT or []) if K_['end'] >= 0 and K_['caps'][K_['end']] == j for i in K_['pin_ok'] + K_['caps'][:K_['end']]]
            _via_n = cons_count(net, pads, tree_cons, j, _with_cons)
            if _via_n > 1: Cb = dict(Cb, via=Cb['via'] * _via_n)
        bx = p['bbox']; res = None
        if not src or not tgt: ok = False; continue
        sxy = [cell_xy((q % NN) % NX, (q % NN) // NX) for q in src]
        if len(src) > 4000:
            o = sorted(range(len(src)), key=lambda q: math.hypot(sxy[q][0] - p['x'], sxy[q][1] - p['y']))[:4000]
            src_s = [src[q] for q in o]
        else: src_s = src


        _hard = REACH and (net, _wk) in _ERR_NETS and _unreachable(_components(net, _wk), src_s, tgt)
        _edges = () if _hard else ((WINDOW, 40.0) if WINDOW else (3.0, 8.0, 40.0))
        _zl = [] if _zone is None else ([_zone, _kzr] if _zone is _kzz and len(_kzr[0]) and (net, by_net[net][j], 1) not in _METRIC_OFF else [_zone])
        for _zone in _zl + [None]:
            _mz = _zone is not None
            _cH, _vH, _cC = capH, viaH, capC
            if _mz:
                if _zone is not _kzz: _cH, _vH = _metric_maps(capH, viaH, *_zone)
                else:
                    if _metric_k is None or _metric_k[0] is not capH: _metric_k = (capH,) + _metric_maps(capH, viaH, *_zone)
                    _cH, _vH = _metric_k[1], _metric_k[2]
                _cC = [np.minimum(_cH[i], capS[i]) for i in range(NL)]
            for _mi, margin in enumerate(_edges):
                win = (min(min(x for x, _ in sxy), bx[0]) - margin, min(min(y for _, y in sxy), bx[1]) - margin,
                       max(max(x for x, _ in sxy), bx[2]) + margin, max(max(y for _, y in sxy), bx[3]) + margin)
                if not _in_window(win, rng):
                    (capH, viaH, capS, dd, rng, viaBad, capC, capCf, vbadf), _b2 = maps_larger(net, box, win, _wmax)
                    if _b2 is not None: box = _b2
                    _cH, _vH, _cC = capH, viaH, capC
                    if _mz: _cH, _vH = _metric_maps(capH, viaH, *_zone); _cC = [np.minimum(_cH[i], capS[i]) for i in range(NL)]
                if _bga_own_holes:



                    viaH = bga_own_hole_map(viaH, _bga_reserved_vias + all_v)
                    _vH = bga_own_hole_map(_vH, _bga_reserved_vias + all_v)
                res = astar_pf(src_s, tgt, bx, _cH, capS, _vH, viaBad, Cb, win, extra=_ex)


                if res is None and REACH and _mi == len(_edges) - 2 and _unreachable(_components(net, _wk), src_s, tgt):
                    _hard = True; _ERR_NETS.add((net, _wk)); break
                if res and WINDOW: break
                if res:
                    conf = any(capCf[q] < Cb['wmin'] for q in res) or any(
                        a // NN != b2 // NN and vbadf[a % NN] for a, b2 in zip(res[:-1], res[1:]))
                    if not conf or margin >= 40.0: break
                    if NB is not None and NB.info[0] == 0: break
            if _mz and res and gl is not None and Cb is not C and any(a // NN != b2 // NN for a, b2 in zip(res[:-1], res[1:])): res = None
            if res and _mz and CHAIN_REST and gl is None and not is_tap and KT is not None and (_zone is _kzz or _zone is _kzr):




                _lb = _clearance_min(src_s, tgt)
                res_f = astar_pf(src_s, tgt, bx, capH, capS, viaH, viaBad, Cb, win, extra=_ex) if _plen(res) > _lb + max(CHAIN_REST_ANT * _lb, CHAIN_REST_MM) else None
                if res_f and _plen(res) > _plen(res_f) + max(CHAIN_REST_ANT * _plen(res_f), CHAIN_REST_MM):
                    res = res_f; _mz = False; _cC = capC; CHAIN_STAT['rest_free'] += 1
                    _METRIC_OFF.add((net, by_net[net][j])); _METRIC_OFF.add((net, by_net[net][j], 1))
            if res or _hard or not _mz: break
            if _tap_z: TOPO_STAT['tap_without_zone'] += 1; continue
            _METRIC_OFF.add((net, by_net[net][j], 1) if _zone is _kzr else (net, by_net[net][j])); CHAIN_STAT['without_zone'] += 1
        trs = vs = None
        if res:
            if _mz:
                for q_ in res: _cC[q_ // NN].reshape(-1)[q_ % NN] = capCf[q_]
            trs, vs = path_to_geometry(res, _cC, dict(Cb, w=Cb['wmin'], ground_map=neck_field_map(net, Cb, Cb['wmin']), extra=_ex, anchor=j not in kel))
            if _pz_active(net) and net in PZ_WIDTHS and not taps and gl is None and not BF and not _pz_geometry_ok(net,trs,vs,not HARD_FOREIGN[0]):
                repair=_pz_edge_retry(res,src_s,tgt,bx,_cH,capS,_vH,viaBad,Cb,win,_ex)
                if repair:res,trs,vs=repair
                else:res=None
            if SP and not is_tap and vs and not current_fields_free(res,vs,Cb,_cH): res = None
            if gl is not None and sum(math.dist(a_, b_) for t in trs for a_, b_ in zip(t['pts'][:-1], t['pts'][1:])) > \
               CHAIN_DETOUR * min(_pad_dist(pads[i], p) for i in _qp) + CHAIN_DETOUR_MM:
                res = None; CHAIN_STAT['detour'] += 1
        if not res and gl is not None:
            K_ = gl[0]; CHAIN_STAT['back'] += 1
            if gl[2] is None: jobs.append((j, None)); continue
            K_['stop'] = True
            _free = [i for i in K_['pins'] + K_['caps'][:-1] if i not in K_['pin_ok'] and i not in K_['caps'][:max(K_['end'], 0)]]
            jobs.extend((i, None) for i in _free); continue
        if not res:
            if _hard:
                _pk = '%s:%s' % (p['ref'], p['num'])
                if _pk not in UNREACHABLE.setdefault(net, []): UNREACHABLE[net].append(_pk)
            ok = False; continue
        _sp0 = (_tap_sp if j in kel else all_tr) if BRANCH and gl is None and (j not in kel or TAP_TREE) else None
        if _sp0 and not SP:
            TOPO_STAT['branch_checked'] += 1; _kset = set(_tap_kn) if j in kel else _pn_set
            _p0 = cell_xy((res[0] % NN) % NX, (res[0] % NN) // NX); _l0 = res[0] // NN
            _m0 = _branch_defects(trs, _sp0, _l0, _p0, res[0] not in _kset)
            if any(_m0):
                TOPO_STAT['branch_shortage'] += 1
                _ex2 = _bands(_ex, _sp0, BRANCH_D, BRANCH_FACTOR); _s2 = src_s
                if _m0[0] or _m0[2]:
                    _r = int(STAR_R / RES) + 1; _iy0, _ix0 = divmod(res[0] % NN, NX)
                    _way = {_l0 * NN + (_iy0 + dy_) * NX + _ix0 + dx_ for dy_ in range(-_r, _r + 1) for dx_ in range(-_r, _r + 1) if dx_ * dx_ + dy_ * dy_ <= _r * _r}
                    _s2 = [q for q in src_s if q not in _way or q not in _kset]
                res2 = astar_pf(_s2, tgt, bx, _cH, capS, _vH, viaBad, Cb, win, extra=_ex2) if _s2 else None
                if res2:



                    _prev_trunk = [(t_, t_['pts'], t_['w']) for t_ in _sp0] if R27_TRUNK else []
                    if _mz:
                        for q_ in res2: _cC[q_ // NN].reshape(-1)[q_ % NN] = capCf[q_]
                    trs2, vs2 = path_to_geometry(res2, _cC, dict(Cb, w=Cb['wmin'], ground_map=neck_field_map(net, Cb, Cb['wmin']), extra=_ex2, anchor=j not in kel))
                    _p2 = cell_xy((res2[0] % NN) % NX, (res2[0] % NN) // NX)
                    _m2 = _branch_defects(trs2, _sp0, res2[0] // NN, _p2, res2[0] not in _kset)
                    _lg = lambda T_: sum(math.dist(a_, b_) for t in T_ for a_, b_ in zip(t['pts'][:-1], t['pts'][1:]))
                    _kf = lambda R_: any(capCf[q] < Cb['wmin'] for q in R_) or any(a // NN != b2 // NN and vbadf[a % NN] for a, b2 in zip(R_[:-1], R_[1:]))


                    _topo_better = (all(a_ <= b_ for a_, b_ in zip(_m2, _m0)) and _m2 != _m0) if R27_TRUNK else _m2 < _m0
                    if _topo_better and len(vs2) <= len(vs) and (not _kf(res2) or _kf(res)) and _lg(trs2) <= BRANCH_MORE * _lg(trs) + 1.0:
                        res, trs, vs = res2, trs2, vs2; _p0 = _p2; _l0 = res[0] // NN; TOPO_STAT['branch_better'] += 1
                    else:
                        for t_, p_, w_ in _prev_trunk: t_['pts'], t_['w'] = p_, w_
            if res[0] in _kset and len(_arms_at(_sp0, _l0, _p0)) >= 2:
                _r = int(STAR_R / RES) + 1; _iy0, _ix0 = divmod(res[0] % NN, NX); TOPO_STAT['star_blocked'] += 1
                _star.update(_l0 * NN + (_iy0 + dy_) * NX + _ix0 + dx_ for dy_ in range(-_r, _r + 1) for dx_ in range(-_r, _r + 1) if dx_ * dx_ + dy_ * dy_ <= _r * _r)


        if res[0] in _fst and res[0] not in _stubs:
            _stubs.add(res[0]); trs.append(dict(layer=_fst[res[0]][0], pts=_fst[res[0]][1], w=[_wk] * (len(_fst[res[0]][1])-1)))
        if res[-1] in _escape_target:
            _stubs.add(res[-1]); trs.append(dict(layer=_escape_target[res[-1]][0], pts=_escape_target[res[-1]][1], w=[_wk] * (len(_escape_target[res[-1]][1])-1)))
        if BF_NODES:
            if BGA_PLAN:


                for _i in (_qp if gl is not None else tree) + [j]:
                    if _i in BF and BF[_i][0] in (res[0], res[-1]) and _i not in _bf_done:
                        _bf_done.add(_i); trs += BF[_i][1]; vs += BF[_i][2]
            else:
                for _c in (res[0], res[-1]):
                    if _c in BF_NODES and _c not in _stubs:
                        _stubs.add(_c); trs += BF_NODES[_c][0]; vs += BF_NODES[_c][1]
        if is_tap:
            for t in trs: t['sig'] = True
            if j in kel and (TAP_ON or TAP_TREE):
                for t in trs: t['tap'] = True
                for v in vs: v['tap'] = True
                _kn = geo_nodes(trs, vs); _tap_sp += trs; _tap_vi += vs; _tap_pads.append(j); TOPO_STAT['taps'] += 1
                _tap_kn.extend(q for q in _kn if q not in _star)
        if gl is not None:
            if SP and vs:
                _sg,_sv = current_field_set(res,vs,Cb); trs += _sg; vs += _sv
            K_ = gl[0]; CHAIN_STAT['element'] += 1
            _g = 1 + 16 * K_['nr'] + gl[1]
            _kp = [by_net[net][i] for i in (K_['pins'] + K_['caps'][:1] if gl[1] == 0 else _qp + [j])]
            for t in trs: t['link_chain'] = _g; t['kp'] = _kp
            for v in vs: v['link_chain'] = _g
            K_['traces'] += trs; K_['vias'] += vs; all_tr += trs; all_v += vs
            if gl[1] == 0: K_['kn'].extend(geo_nodes(trs, vs))
            if gl[2] is None: K_['pin_ok'].append(j)
            elif gl[1] == 0: K_['pin_ok'].append(gl[2][0]); K_['end'] = 0
            else: K_['end'] = gl[1]
            continue
        _cons = ([], [])
        if SP and vs and not is_tap and j not in kel:
            _cons = current_field_set(res,vs,Cb)
        elif CONS and NC is not None and vs and not is_tap and j not in kel:
            _with = [i for K_ in KT if K_['end'] >= 0 and K_['caps'][K_['end']] == j for i in K_['pin_ok'] + K_['caps'][:K_['end']]] if KT is not None else []
            _n = cons_count(net, pads, tree_cons, j, _with)
            if _n > 1: _cons = cons_set(net, res, trs, vs, _n, capH, viaH, capC, vbadf, Cb, all_v, rng)
        all_tr += trs + _cons[0]; all_v += vs + _cons[1]
        if not is_tap: _kn = geo_nodes(trs, vs); pathnodes.extend(_kn); _pn_set.update(_kn)
        tree.append(j)
        if KT is not None:
            tree_cons.append(j); tree_cons += [i for K_ in KT if K_['end'] >= 0 and K_['caps'][K_['end']] == j for i in K_['pin_ok'] + K_['caps'][:K_['end']]]
    if KT is not None:
        for K_ in KT:
            if K_['end'] >= 0 and K_['traces']:
                K_['traces'][0]['metric'] = [by_net[net][K_['caps'][K_['end']]]] + [by_net[net][i] for i in K_['pin_ok'] + K_['caps'][:K_['end']]]
    if RK is not None and all_v: all_v = all_v + rv_set(net, all_tr, all_v, RK)
    if _pz_active(net) and not _pz_geometry_ok(net,all_tr,all_v):
        all_tr,all_v,ok=[],[],False
    add_unit(u, net, [dict(net=net, **t) for t in all_tr], all_v)
    return ok


def unit_conflicts(u, capS, dd):
    r = U[u]; bad = []
    for t in r['tracks']:
        li = LI[t['layer']]; capL = capS[li]
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            L = math.hypot(b[0] - a[0], b[1] - a[1]); n = max(2, int(L / (RES * 0.5)) + 1)
            tt = np.linspace(0, 1, n)
            ix = ((a[0] + tt * (b[0] - a[0]) - X0) / RES).astype(int); iy = ((a[1] + tt * (b[1] - a[1]) - Y0) / RES).astype(int)
            m = capL[iy, ix] < w - 0.005
            if m.any(): bad.extend((li * NN + iy[m] * NX + ix[m]).tolist())
    for v in r['vias']:
        ix, iy = int((v['x'] - X0) / RES), int((v['y'] - Y0) / RES)
        if dd[iy, ix] < HOLE_C or any(capS[li][iy, ix] / 2 < VIA_R - 0.003 for li in range(NL)):
            bad.extend([li * NN + iy * NX + ix for li in range(NL)])
    return bad


from shapely.geometry import LineString as _LS, Point as _PT
from shapely.strtree import STRtree as _STR
DRC_CLR = DRCC + 0.004
DRC_HOLE = H2H + VIA_DR + 0.004
DRC_PADC = DRCP + (PAD_EXTRA if BGA_TIGHT else 0.004)


def _pz_clearance(net,other,pad=False):
    if globals().get('KA') is not None:
        return _ka_margin(net,other,type2='Pad' if pad else 'Track')
    if _pz_active(net) or _pz_active(other):
        return (DRCP if pad else DRCC) + PZ_BORDER_NM / 1e6
    return DRC_PADC if pad else DRC_CLR


PAIRS = []
_DMAX = max(DRC_CLR, DRC_PADC)
if globals().get('KA') is not None: _DMAX = max(_DMAX,KA.maximum+.012)
_PADB = {li: [(pn, pg, pg.buffer(_DMAX)) for pn, pg in _PADG[li]] for li in range(NL)}
if DRCL:


    _PADB = {li: [] for li in range(NL)}
    for _p in D['pads']:
        _pn = _p['net'] if _p['net'] and not _p['net'].startswith('unconnected') else '#%d' % id(_p)
        for _L in _p['layers']:
            if _L not in LI: continue
            _g = (shapely.union_all([Polygon(q) for q in _p['polys'][_L]])
                  if globals().get('KA') is not None else pad_hull(_p, _L, DRCP, True))
            if globals().get('KA') is not None: _KA_META[id(_g)] = _ka_pad_kind(_p)
            _PADB[LI[_L]].append((_pn, _g, _g.buffer(_DMAX)))
if globals().get('KA') is not None:
    for _li,_keeps in _KA_PAD_KEEP.items():
        _PADB[_li].extend((_pn,_g,_g.buffer(_DMAX)) for _pn,_g,_kind in _keeps)


def _unit_geo(u, r):
    g = _GEO.get(u)
    if g is None:
        g = []; n0 = r['net']
        for t in r['tracks']:
            li = LI[t['layer']]
            for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                radius=w/2/math.cos(math.pi/256) if globals().get('KA') is not None or n0 in globals().get('PZ_LOCAL',{}) else w/2
                k = _LS([a, b]).buffer(radius, 64 if globals().get('KA') is not None or n0 in globals().get('PZ_LOCAL',{}) else 8) if a != b else _PT(a).buffer(radius, 64 if globals().get('KA') is not None or n0 in globals().get('PZ_LOCAL',{}) else 8)
                g.append((li, k, k.buffer(_DMAX), n0))
                if globals().get('KA') is not None: _KA_META[id(k)] = ('Track','')
        for v in r['vias']:
            k = _PT(v['x'], v['y']).buffer(VIA_R/math.cos(math.pi/256) if globals().get('KA') is not None or n0 in globals().get('PZ_LOCAL',{}) else VIA_R, 64 if globals().get('KA') is not None or n0 in globals().get('PZ_LOCAL',{}) else 8); kb = k.buffer(_DMAX); nv = v.get('net') or n0
            for li in range(NL):
                if globals().get('KA') is not None and LAYERS[li] not in v.get('layers',LAYERS):continue
                g.append((li, k, kb, nv))
            if globals().get('KA') is not None: _KA_META[id(k)] = ('Via','')
        _GEO[u] = g
    return g


FIX_PAIRS = []


def _fixed_pair(a, b):


    return a.startswith('FIX#') and b.startswith('FIX#') and (globals().get('args') or {}).get('rest_checking') != '1'


def _conflict_points(a, b, spacing):
    if spacing == 0:
        c = shapely.point_on_surface(shapely.intersection(a, b))
        return c, c
    line = shapely.shortest_line(a, b)
    return shapely.get_point(line, 0), shapely.get_point(line, -1)


def _pad_drill_conflicts(vias, rating, report=True):
    if not PAD_HOLE_GEOMS or not vias: return
    holes=np.array(PAD_HOLE_GEOMS,dtype=object)
    points=np.array([_PT(x,y) for _,_,x,y in vias],dtype=object)
    a,b=_STR(holes).query(shapely.envelope(shapely.buffer(points,VIA_DR/2+H2H+.0011)))
    distance=shapely.distance(points[a],holes[b])-VIA_DR/2
    for k,j in enumerate(b.tolist()):
        circle=PAD_HOLE_CIRCLES[j]
        if circle is not None:
            _,_,x,y=vias[int(a[k])]
            distance[k]=math.hypot(x-circle[0],y-circle[1])-circle[2]-VIA_DR/2
    for i,j,d in zip(a.tolist(),b.tolist(),distance.tolist()):
        if d >= H2H-1e-9: continue
        u,_,x,y=vias[i]
        for li in range(NL): rating(u,x,y,li)
        if report:
            PAIRS.append(('*',round(d,4),u,'PAD',round(x,2),round(y,2)))
            if VIA_PRECEDENCE: PAIRS_RAW.append((-1,x,y,u,'PAD'))


def _in_window(bounds, F):
    return bounds[0] <= F[2] and bounds[2] >= F[0] and bounds[1] <= F[3] and bounds[3] >= F[1]
def exact_conflicts(without=None, window=None):
    PAIRS.clear(); PAIRS_RAW.clear(); FIX_PAIRS.clear()
    geoms = {li: [] for li in range(NL)}
    drills_l = []
    for u, r in U.items():
        if without is not None and u in without: continue
        for li, k, kb, ne in _unit_geo(u, r):
            if window is None or _in_window(k.bounds, window): geoms[li].append((u, ne, k, kb))
        for v in r['vias']:


            if window is not None and not _in_window((v['x'], v['y'], v['x'], v['y']), window): continue
            nv = (v.get('net') or r['net']) if BGA_HOLE or bool(globals().get('PZ_LOCAL')) else r['net']
            drills_l.append((u, nv, v['x'], v['y']))
    for li in range(NL):
        for pn, pg, pb in _PADB[li]:
            if window is None or _in_window(pg.bounds, window): geoms[li].append(('PAD', pn, pg, pb))
    conf = {}
    def rating(u, x, y, li):
        if u == 'PAD': return
        ix, iy = int((x - X0) / RES), int((y - Y0) / RES)
        rr = 3
        lst = conf.setdefault(u, [])
        for jy in range(max(0, iy - rr), min(NY, iy + rr + 1)):
            for jx in range(max(0, ix - rr), min(NX, ix + rr + 1)):
                lst.append(li * NN + jy * NX + jx)
    for li in range(NL):
        G = geoms[li]
        if not G: continue
        ga = np.array([g for _, _, g, _ in G], dtype=object)
        tree = _STR(ga)
        net_name = np.array([n for _, n, _, _ in G], dtype=object)
        if (BGA_HOLE or bool(globals().get('PZ_LOCAL')) or globals().get('KA') is not None) and HOLE_CLR > 0 and drills_l:


            hp = np.array([_PT(x, y) for _, _, x, y in drills_l], dtype=object)
            hi, hj = tree.query(shapely.buffer(hp, VIA_DR / 2 + HOLE_CLR + 0.00001))
            hn = np.array([n for _, n, _, _ in drills_l], dtype=object)
            hd = shapely.distance(hp[hi], ga[hj]) - VIA_DR / 2
            hm = (hn[hi] != net_name[hj]) & (hd < HOLE_CLR - 0.000001)
            for i, j, dij in zip(hi[hm].tolist(), hj[hm].tolist(), hd[hm].tolist()):
                u, _, x, y = drills_l[i]; u2 = G[j][0]
                if _fixed_pair(u, u2): continue
                c2 = shapely.get_point(shapely.shortest_line(hp[i], ga[j]), -1)



                rating(u, x, y, li); rating(u2, c2.x, c2.y, li)
                cx, cy = (x+c2.x)/2, (y+c2.y)/2
                PAIRS.append((LAYERS[li], round(dij, 4), u, u2, round(cx, 2), round(cy, 2)))
                if VIA_PRECEDENCE: PAIRS_RAW.append((li, cx, cy, u, u2))
        qi, qj = tree.query(np.array([gb for _, _, _, gb in G], dtype=object))
        m = qj > qi
        qi, qj = qi[m], qj[m]
        if not len(qi): continue
        net_name = np.array([n for _, n, _, _ in G], dtype=object); pad = np.array([u == 'PAD' for u, _, _, _ in G])
        m = (net_name[qi] != net_name[qj]) & ~(pad[qi] & pad[qj])
        qi, qj = qi[m], qj[m]
        if not len(qi): continue
        dist = shapely.distance(ga[qi], ga[qj])
        limits = np.where(pad[qi] | pad[qj], DRC_PADC, DRC_CLR)
        if globals().get('PZ_LOCAL'):
            limits = np.array([_pz_clearance(net_name[i],net_name[j],bool(pad[i] or pad[j])) for i,j in zip(qi,qj)])
        if globals().get('KA') is not None:
            limits = np.array([_ka_geo_limit(net_name[i],net_name[j],li,ga[i],ga[j],bool(pad[j])) for i,j in zip(qi,qj)])
        m = dist < limits
        for i, j, dij in zip(qi[m].tolist(), qj[m].tolist(), dist[m].tolist()):
            u, net, g, _ = G[i]; u2, net2, g2, _ = G[j]
            if _fixed_pair(u, u2):
                FIX_PAIRS.append((LAYERS[li], round(dij, 4), u, u2)); continue
            if BGA_PLAN or CONFLICT_SITE_EXACT or globals().get('_REST_LOCAL_CONFLICT', False):


                c, c2 = _conflict_points(g, g2, dij)
                rating(u, c.x, c.y, li); rating(u2, c2.x, c2.y, li)
                c = _PT((c.x+c2.x)/2,(c.y+c2.y)/2)
            else:
                c = g.centroid if dij == 0 else g.boundary.interpolate(g.boundary.project(g2.centroid))
                rating(u, c.x, c.y, li); rating(u2, c.x, c.y, li)
            PAIRS.append((LAYERS[li], round(dij, 3), u, u2, round(c.x, 2), round(c.y, 2)))
            if VIA_PRECEDENCE: PAIRS_RAW.append((li, c.x, c.y, u, u2))
    _pad_drill_conflicts(drills_l,rating)
    if len(drills_l) > 1:
        from scipy.spatial import cKDTree
        P = np.array([(x, y) for _, _, x, y in drills_l])
        _vhclear = max(DRC_HOLE, VIA_DR / 2 + VIA_R + HOLE_CLR) if BGA_HOLE or bool(globals().get('PZ_LOCAL')) else DRC_HOLE
        for i, j in sorted(cKDTree(P).query_pairs(_vhclear * (1 + 1e-9) + 1e-9)):
            u, n1, x1, y1 = drills_l[i]; u2, n2, x2, y2 = drills_l[j]
            if math.hypot(x1 - x2, y1 - y2) < DRC_HOLE or ((BGA_HOLE or bool(globals().get('PZ_LOCAL'))) and n1 != n2 and math.hypot(x1 - x2, y1 - y2) < VIA_DR / 2 + VIA_R + HOLE_CLR - 0.000001):
                if _fixed_pair(u, u2):
                    FIX_PAIRS.append(('*', round(math.hypot(x1 - x2, y1 - y2), 4), u, u2)); continue
                for li in range(NL): rating(u, x1, y1, li); rating(u2, x2, y2, li)
                if VIA_PRECEDENCE: PAIRS_RAW.append((-1, x1, y1, u, 'PAD')); PAIRS_RAW.append((-1, x2, y2, u2, 'PAD'))


                PAIRS.append(('*', round(math.hypot(x1 - x2, y1 - y2), 3), u, u2, round((x1 + x2) / 2, 2), round((y1 + y2) / 2, 2)))
    return conf


def _narrow_raw():
    out = []; Cs = cls('/__signal__')
    for u, r in U.items():
        if u.startswith(('FIX#', 'TACK#')): continue
        stub = u.startswith('GND#') or u in PLANE_GROUPS
        C = dict(wmin=max(0.15, TRACK_MIN)) if stub else cls(r['net'])
        WF = neck_field_map(r['net'], C, C['wmin'], stub)
        for t in r['tracks']:
            if t.get('web'): continue
            nominal = Cs['wmin'] if t.get('sig') else C['wmin']; li = LI[t['layer']]
            for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                if w >= nominal - 1e-6: continue
                ix, iy = int(((a[0] + b[0]) / 2 - X0) / RES), int(((a[1] + b[1]) / 2 - Y0) / RES)
                permitted = WF is not None and not t.get('sig') and 0 <= ix < NX and 0 <= iy < NY and float(WF[li * NN + iy * NX + ix]) < nominal
                out.append((u, r['net'], li, a, b, w, nominal, bool(permitted)))
    return out
def under_wmin():
    out = []; before = None
    for u, net, li, a, b, w, nominal, permitted in _narrow_raw():
        L = math.dist(a, b)
        if before is not None and before[0] == (u, li, permitted, nominal) and before[1] == tuple(a):
            e = out[-1]; e['_p'].append((a, b)); e['total_length'] += L; e['width'] = min(e['width'], w)
        else:
            out.append(dict(net_name=net, layer_name=LAYERS[li], x=0.0, y=0.0, width=w, nominal=nominal, total_length=L, allowed=permitted, _p=[(a, b)]))
        before = ((u, li, permitted, nominal), tuple(b))
    for e in out:
        half = e['total_length'] / 2; p = e.pop('_p')
        for a, b in p:
            L = math.dist(a, b)
            if half <= L or (a, b) == p[-1]:
                f = min(1.0, half / L) if L > 0 else 0.0
                e['x'] = round(a[0] + f * (b[0] - a[0]), 3); e['y'] = round(a[1] + f * (b[1] - a[1]), 3); break
            half -= L
        e['total_length'] = round(e['total_length'], 3)
    return out


WMIN_BEFORE = args.get('wmin_precedence', '1') == '1'


def _precedence(net):
    if not NC: return 1
    return 2 if NC['net_list'].get(net, {}).get('category') in ('current_val', 'supply', 'sw', 'gnd') else 1


def tight_conflicts():
    raw_data = [s for s in _narrow_raw() if not s[7]]
    if WMIN_BEFORE and NC:
        raw_data = [s for s in raw_data if not s[0].startswith('GND#') and _precedence(s[1]) >= 2]
    conf = {}
    if not raw_data: return conf
    def rating(u, x, y, li):
        ix, iy = int((x - X0) / RES), int((y - Y0) / RES)
        lst = conf.setdefault(u, [])
        for jy in range(max(0, iy - 3), min(NY, iy + 4)):
            for jx in range(max(0, ix - 3), min(NX, ix + 4)): lst.append(li * NN + jy * NX + jx)
    for li in sorted({s[2] for s in raw_data}):
        G = [(u, ne, k) for u, r in U.items() for l2, k, _kb, ne in _unit_geo(u, r) if l2 == li]
        ga = np.array([g for _, _, g in G], dtype=object); tree = _STR(ga)
        for u, net, _li, a, b, w, nominal, _e in raw_data:
            if _li != li: continue
            k = _LS([a, b]).buffer(nominal / 2, 8) if a != b else _PT(a).buffer(nominal / 2, 8)
            mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
            opponent = sorted({G[j][0] for j in tree.query(k.buffer(DRC_CLR)).tolist() if G[j][1] != net and k.distance(G[j][2]) < DRC_CLR})



            soft = [u2 for u2 in opponent if _precedence(U[u2]['net']) < _precedence(net)] if WMIN_BEFORE else []
            if not soft: rating(u, mx, my, li)
            for u2 in (soft or opponent): rating(u2, mx, my, li)
            for u2 in opponent or ['PAD']: PAIRS.append((LAYERS[li], round(w - nominal, 3), u, u2, round(mx, 2), round(my, 2)))
    return conf


_CAND_IDX = [None]


def _cand_idx_build(without):
    cu_layers = {}
    per = {li: [] for li in range(NL)}
    for u, r in U.items():
        if u == without: continue
        for li, k, kb, ne in _unit_geo(u, r): per[li].append((u, ne, k, False))
    for li in range(NL):
        for pn, pg, pb in _PADB[li]: per[li].append(('PAD', pn, pg, True))
        if per[li]:
            ga = np.array([g for _, _, g, _ in per[li]], dtype=object)
            cu_layers[li] = (_STR(ga), [o for o, _, _, _ in per[li]], [n for _, n, _, _ in per[li]], ga, [q for _, _, _, q in per[li]])
    return dict(cu_layers=cu_layers, prior=set(), new={}, missing={without})


def cand_idx_changed(u):
    X = _CAND_IDX[0]
    if X is not None: X['prior'].add(u); X['new'][u] = _unit_geo(u, U[u]); X['missing'].discard(u)


def cand_idx_back(u):
    X = _CAND_IDX[0]
    if X is not None and u in X['missing']: cand_idx_changed(u)


def unit_conflict_free(u):
    if BGA_HOLE or globals().get('PZ_LOCAL') or globals().get('KA') is not None: return u not in exact_conflicts()
    X = _CAND_IDX[0]
    if X is None or len(X['prior']) > 48: X = _CAND_IDX[0] = _cand_idx_build(u)
    r = U[u]; net = r['net']; geo = _unit_geo(u, r)
    for li in range(NL):
        mine = [(k, kb, ne) for l2, k, kb, ne in geo if l2 == li]
        if not mine: continue
        T = X['cu_layers'].get(li)
        if T is not None:
            tree, own, net_name, ga, pad = T
            qi, qj = tree.query(np.array([kb for _, kb, _ in mine], dtype=object))
            for i, j in zip(qi.tolist(), qj.tolist()):
                if own[j] == u or own[j] in X['prior'] or net_name[j] == mine[i][2] or _fixed_pair(u, own[j]): continue
                if mine[i][0].distance(ga[j]) < (_ka_geo_limit(net,net_name[j],li,g2=ga[j],pad=pad[j]) if globals().get('KA') is not None else (DRC_PADC if pad[j] else DRC_CLR)): return False
        for u2, g2 in X['new'].items():
            if u2 == u or u2 not in U or _fixed_pair(u, u2): continue
            for l2, k2, _, n2 in g2:
                if l2 != li: continue
                for k, _, ne in mine:
                    if n2 != ne and k.distance(k2) < DRC_CLR: return False
        if any(ne != net for _, _, ne in mine):
            for i, (k, _, ne) in enumerate(mine):
                for k2, _, n2 in mine[i + 1:]:
                    if n2 != ne and k.distance(k2) < DRC_CLR: return False
    if r['vias']:
        pad_conflict = []
        _pad_drill_conflicts([(u, net, v['x'], v['y']) for v in r['vias']],
                          lambda *q: pad_conflict.append(q), report=False)
        if pad_conflict: return False
        all_items = [(v['x'], v['y'], u2) for u2, r2 in U.items() for v in r2['vias']]
        for i, v in enumerate(r['vias']):
            for x, y, u2 in all_items:
                if (u2 == u and x == v['x'] and y == v['y']) or _fixed_pair(u, u2): continue
                if math.hypot(v['x'] - x, v['y'] - y) < DRC_HOLE: return False
    return True


def groups_index(group):
    g = set(group); per = {li: [] for li in range(NL)}; bore = []
    for u, r in U.items():
        if u in g: continue
        for li, k, kb, ne in _unit_geo(u, r): per[li].append((u, ne, k))
        for v in r['vias']: bore.append((v['x'], v['y'], u))
    cu_layers = {}
    for li in range(NL):
        for pn, pg, pb in _PADB[li]: per[li].append(('PAD', pn, pg))
        if per[li]:
            ga = np.array([q for _, _, q in per[li]], dtype=object)
            cu_layers[li] = (_STR(ga), [o for o, _, _ in per[li]], [n for _, n, _ in per[li]], ga)
    return dict(cu_layers=cu_layers, bore=bore, bxy=np.array([(x, y) for x, y, _ in bore], dtype=np.float64).reshape(-1, 2))


def conflicts_group(group, X, without=None):
    if BGA_HOLE or globals().get('PZ_LOCAL') or globals().get('KA') is not None:


        conf = exact_conflicts(without)
        g = set(group)
        with_ = {u for pair in PAIRS if pair[2] in g or pair[3] in g for u in pair[2:4]}
        return {u: cs for u, cs in conf.items() if u in with_}
    conf = {}
    def rating(u, x, y, li):
        if u == 'PAD': return
        ix, iy = int((x - X0) / RES), int((y - Y0) / RES)
        lst = conf.setdefault(u, [])
        for jy in range(max(0, iy - 3), min(NY, iy + 4)):
            for jx in range(max(0, ix - 3), min(NX, ix + 4)):
                lst.append(li * NN + jy * NX + jx)
    with_ = [u for u in group if u in U]
    for li in range(NL):
        mine = [(u, ne, k, kb) for u in with_ for l2, k, kb, ne in _unit_geo(u, U[u]) if l2 == li]
        if not mine: continue
        search = np.array([kb for _, _, _, kb in mine], dtype=object)
        T = X['cu_layers'].get(li)
        if T is not None:
            tree, own, net_name, ga = T
            qi, qj = tree.query(search)
            for i, j in zip(qi.tolist(), qj.tolist()):
                u, net, k, _ = mine[i]
                if net_name[j] == net or _fixed_pair(u, own[j]) or (without is not None and own[j] in without): continue
                d = k.distance(ga[j])
                if d < (DRC_PADC if own[j] == 'PAD' else DRC_CLR):
                    if CONFLICT_SITE_EXACT:
                        c, c2 = _conflict_points(k, ga[j], d)
                        rating(u, c.x, c.y, li); rating(own[j], c2.x, c2.y, li)
                    else:
                        c = k.centroid if d == 0 else k.boundary.interpolate(k.boundary.project(ga[j].centroid))
                        rating(u, c.x, c.y, li); rating(own[j], c.x, c.y, li)
        gm = np.array([k for _, _, k, _ in mine], dtype=object)
        qi, qj = _STR(gm).query(search)
        for i, j in zip(qi.tolist(), qj.tolist()):
            if j <= i or mine[i][1] == mine[j][1] or _fixed_pair(mine[i][0], mine[j][0]): continue
            d = mine[i][2].distance(mine[j][2])
            if d < DRC_CLR:
                k = mine[i][2]
                if CONFLICT_SITE_EXACT:
                    c, c2 = _conflict_points(k, mine[j][2], d)
                    rating(mine[i][0], c.x, c.y, li); rating(mine[j][0], c2.x, c2.y, li)
                else:
                    c = k.centroid if d == 0 else k.boundary.interpolate(k.boundary.project(mine[j][2].centroid))
                    rating(mine[i][0], c.x, c.y, li); rating(mine[j][0], c.x, c.y, li)
    own_set = [(v['x'], v['y'], u) for u in with_ for v in U[u]['vias']]
    _pad_drill_conflicts([(u,None,x,y) for x,y,u in own_set],rating,report=False)
    for i, (x, y, u) in enumerate(own_set):
        if len(X['bxy']):
            for j in np.flatnonzero(np.hypot(X['bxy'][:, 0] - x, X['bxy'][:, 1] - y) < DRC_HOLE).tolist():
                if _fixed_pair(u, X['bore'][j][2]) or (without is not None and X['bore'][j][2] in without): continue
                for li in range(NL): rating(u, x, y, li); rating(X['bore'][j][2], X['bore'][j][0], X['bore'][j][1], li)
        for x2, y2, u2 in own_set[i + 1:]:
            if math.hypot(x - x2, y - y2) < DRC_HOLE:
                if _fixed_pair(u, u2): continue
                for li in range(NL): rating(u, x, y, li); rating(u2, x2, y2, li)
    return conf


FLOW_W = {}






WIDE_DISTURB = args.get('wide_disturb', '1' if NC else '0') == '1'
R32 = args.get('r32', '0') == '1' and NC is not None
_DIST_C = {}


def _dist_c(name, nets, body=False):
    key = (name, frozenset(nets), body) if DISTURB_FLANK else name
    sig = tuple((n, _NVER.get(n, 0)) for n in sorted(nets)) if DISTURB_FLANK else sum(_NVER.get(n, 0) for n in nets)
    c = _DIST_C.get(key)
    if c is None or c[0] != sig:
        m = _mask_arr(nets)
        if body and _COILS_M is not None: m |= _COILS_M
        d = np.full(NL * NN, 99.0, dtype=np.float32)
        for li in range(NL):
            sl = slice(li * NN, (li + 1) * NN)
            if m[sl].any(): d[sl] = (_edt(~m[sl].reshape(NY, NX)) * RES).ravel()
        c = (sig, d.reshape(NL, NY, NX)); _DIST_C[key] = c
    return c[1]


def _disturb_width(net):
    if not (WIDE_DISTURB and NC): return []
    pairs = []
    if net in AGG_SW and SW_DIST > 0: pairs.append(('b_vsw', VICT_SW, SW_DIST))
    if net in AGG_FAST and SEPARATE > 0: pairs.append(('b_va', VICT_A, _separate_mm(net)))
    if net in VICT_SW and SW_DIST > 0: pairs.append(('b_asw', AGG_SW, SW_DIST))
    if net in VICT_A and SEPARATE > 0:
        if DISTURB_FLANK:
            for f, ns in _edges_groups(): pairs.append((('b_af', f), ns, SEPARATE * f))
        else: pairs.append(('b_af', AGG_FAST, SEPARATE))
    if SI:
        victim = {n for n, k in NC['net_list'].items() if k.get('category') in ('analog', 'kelvin') or k.get('kelvin')}
        for other in sorted((AGG_SW | AGG_FAST) if net in victim else victim if net in AGG_SW | AGG_FAST else []):
            d = si_spacing(net, other) if net in victim else si_spacing(other, net)
            if d > 0: pairs.append((('b_si', other), {other}, d))
    return [(_dist_c(n_, frozenset(m_ - {net}), body=DISTURB_FLANK and n_ == 'b_asw'), sp) for n_, m_, sp in pairs if m_ - {net} or (DISTURB_FLANK and n_ == 'b_asw')]





WIDTH_INNER = float(args.get('width_inner', '2.3')) if NC is not None else 0.0


def f_inner(net, li):
    if WIDTH_INNER <= 0 or li in (0, NL - 1): return 1.0
    k = NC['net_list'].get(net) or {}
    if k.get('category') not in ('current_val', 'supply', 'sw'): return 1.0
    if args.get('current_nominal') == '1':
        from common.current_track import nominal_width
        return nominal_width(k, inner=True) / nominal_width(k)
    if k.get('w_inner') and k.get('w'): return max(1.0, float(k['w_inner']) / float(k['w']))
    return WIDTH_INNER


def _r32_clearance(X, u, li, a, b, net, limit):
    T = X['cu_layers'].get(li)
    if T is None or limit <= 0: return limit
    seg = _LS([a, b]) if a != b else _PT(a)
    tree, own, net_name, ga, pad = T
    for j in tree.query(seg.buffer(limit / 2 + _DMAX + 1e-9)):
        j = int(j)
        if net_name[j] == net or _fixed_pair(u, own[j]): continue
        limit = min(limit, 2 * (seg.distance(ga[j]) - (_ka_geo_limit(net,net_name[j],li,g2=ga[j],pad=pad[j]) if globals().get('KA') is not None else (DRC_PADC if pad[j] else DRC_CLR))))
    return limit


def _widen_unit_prev(u, C):
    if pair_unit(u) or U[u]['net'] in IMP_TAB: return
    if current_model(U[u]['net']):
        current_correct(u,C);return
    r = U[u]; net = r['net']
    _sb = _disturb_width(net)
    del_unit(u)
    uniform = R32 and (NC['net_list'].get(net) or {}).get('category') in ('signal', 'fast', 'analog')
    if all(t.get('sig') for t in r['tracks']) and not uniform:
        add_unit(u, net, list(r['tracks']), r['vias']); return
    rx = _cand_idx_build(u) if uniform else None
    xs = [q[0] for t in r['tracks'] for q in t['pts']] or [0.0]; ys = [q[1] for t in r['tracks'] for q in t['pts']] or [0.0]
    goal = FLOW_W.get(u, {})
    _fmax = max([f_inner(net, LI[t['layer']]) for t in r['tracks']] + [1.0])
    capH, _, capS, dd, _, _, capC, _, _ = maps(net, (min(xs) - 1.0, min(ys) - 1.0, max(xs) + 1.0, max(ys) + 1.0), max([C['w']] + list(goal.values())) * _fmax)
    new_tr = list(r['tracks']); series = range(len(new_tr)); metric = None
    if CHAIN and any('metric' in t for t in new_tr):
        metric = _metric_prepare(r, net); width_list = [list(t['w']) for t in new_tr]
        series = sorted(series, key=lambda ti: (not new_tr[ti].get('link_chain'), new_tr[ti].get('link_chain') or 0, ti))
    ab = _tap_prepare(r, net) if TAP_ON and any(t.get('tap') for t in new_tr) else None
    for ti in series:
        t = new_tr[ti]
        if t.get('sig') and not uniform: continue
        li = LI[t['layer']]; pts = t['pts']; ws = []
        for si, (a, b, w0) in enumerate(zip(pts[:-1], pts[1:], t['w'])):
            wv = min(goal.get((ti, si), C['w']) * f_inner(net, li), line_cap_min(capC[li], a, b), 2 * pad_room(li, a, b, net))
            if metric is not None: wv = min(wv, _metric_width(metric, r, width_list, ti, a, b, li))
            if ab is not None: wv = min(wv, _tap_width(ab, a, b, li))
            for _dk, _sp in _sb:
                wv = min(wv, 2 * (line_cap_min(_dk[li], a, b) - _sp))
                for _lj, _s in neighbor_spacing(li, _sp):
                    wv = min(wv, 2 * (line_cap_min(_dk[_lj], a, b) - _s))
            if 'w_step' in C: wv = min(wv, w0 + C['w_step'])
            if uniform:
                ws.append(_r32_clearance(rx, u, li, a, b, net, min(C['w'], wv)))
            else: ws.append(max(w0, math.floor(wv * 40) / 40))
        new_tr[ti] = dict(t, w=ws)
        if metric is not None: width_list[ti] = ws
    if uniform:

        widths = [w for t in new_tr for w in t['w']]
        if widths:


            minimum = TRACK_MIN if TRACK_MIN > 0 else C['wmin']
            target = max(minimum, C['w']) if min(widths) >= max(minimum, C['w']) - 1e-9 else minimum
            new_tr = [dict(t, w=[target] * len(t['w'])) for t in new_tr]
    add_unit(u, net, new_tr, r['vias'])


WIDE_DENSE = args.get('wide_dense', '0') == '1'


def _wide_dense_cap(X, u, li, a, b, net, limit):
    seg = _LS([a, b]) if a != b else _PT(a)
    T = X['cu_layers'].get(li)
    if T is not None:
        tree, own, net_name, ga, pad = T

        for j in tree.query(seg.buffer(limit / 2 + _DMAX + 1e-7).envelope):
            j = int(j)
            if own[j] == u or own[j] in X['prior'] or net_name[j] == net or _fixed_pair(u, own[j]): continue
            limit = min(limit, 2 * (seg.distance(ga[j]) - (_ka_geo_limit(net,net_name[j],li,g2=ga[j],pad=pad[j]) if globals().get('KA') is not None else (DRC_PADC if pad[j] else DRC_CLR)) - 1e-7))
    for other, geo in X['new'].items():
        if other == u or other not in U or _fixed_pair(u, other): continue
        for l2, g, _, n2 in geo:
            if l2 == li and n2 != net:
                limit = min(limit, 2 * (seg.distance(g) - (_ka_geo_limit(net,n2,li,g2=g) if globals().get('KA') is not None else DRC_CLR) - 1e-7))

    for l2, g, _, n2 in _unit_geo(u, U[u]):
        if l2 == li and n2 != net:
            limit = min(limit, 2 * (seg.distance(g) - (_ka_geo_limit(net,n2,li,g2=g) if globals().get('KA') is not None else DRC_CLR) - 1e-7))
    if (BGA_HOLE or bool(globals().get('PZ_LOCAL')) or globals().get('KA') is not None) and HOLE_CLR > 0:
        for other, r in U.items():
            if _fixed_pair(u, other): continue
            for v in r['vias']:
                if (v.get('net') or r['net']) != net:
                    limit = min(limit, 2 * (seg.distance(_PT(v['x'], v['y'])) - VIA_DR / 2 - HOLE_CLR - 1e-7))
    return limit
def widen_unit(u, C):
    if not WIDE_DENSE:
        return _widen_unit_prev(u, C)
    old = copy.deepcopy(U[u])
    X = _CAND_IDX[0]
    if X is None or len(X['prior']) > 48:
        X = _CAND_IDX[0] = _cand_idx_build(u)
    _widen_unit_prev(u, C)
    r = U[u]


    shape = lambda q: ([(t['layer'], t['pts']) for t in q['tracks']],
                       [(v['x'], v['y'], v.get('net')) for v in q['vias']])
    if shape(r) != shape(old):
        del_unit(u); add_unit(u, old['net'], old['tracks'], old['vias'])
        cand_idx_changed(u)
        return
    tr = []
    for t, t0 in zip(r['tracks'], old['tracks']):
        ws = []
        for a, b, w, w0 in zip(t['pts'][:-1], t['pts'][1:], t['w'], t0['w']):
            if w > w0:
                cap = _wide_dense_cap(X, u, LI[t['layer']], a, b, r['net'], w)

                w = max(w0, min(w, math.floor(max(0., cap) * 40) / 40))
            ws.append(w)
        tr.append(dict(t, w=ws))
    del_unit(u); add_unit(u, r['net'], tr, r['vias'])
    cand_idx_changed(u)









QUIET = args.get('width_quiet', '1' if NC else '0') == '1'
QUIET_R = float(args.get('quiet_r', '0.03')); QUIET_MM = float(args.get('quiet_mm', '0.35')); QUIET_SHORT = float(args.get('quiet_short', '1.0'))
FLOW_A = {}
if QUIET:
    import os as _os
    sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    from common.width_profile import calm as _bp_quiet, count as _bp_count


def width_calm(u, C):
    if pair_unit(u) or U[u]['net'] in IMP_TAB: return (0, 0, 0)
    r = U[u]; goal = FLOW_W.get(u); ant = FLOW_A.get(u); before = post = n = 0; new_tr = []
    for ti, t in enumerate(r['tracks']):
        if t.get('sig') or t.get('web') or len(t['w']) < 2: new_tr.append(t); continue
        l = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(t['pts'][:-1], t['pts'][1:])]
        _fi = f_inner(r['net'], LI[t['layer']])
        z = [goal.get((ti, si), C['w']) * _fi for si in range(len(l))] if goal else None
        a_ = max(ant.get((ti, si), 1.0) for si in range(len(l))) if ant else 1.0
        ws = _bp_quiet(l, t['w'], goal=z, floor=C['wmin'], lam=QUIET_MM, short=QUIET_SHORT, r_max=QUIET_R / max(a_, 0.2))
        before += _bp_count(l, t['w'])[0]; post += _bp_count(l, ws)[0]
        if ws != list(t['w']): n += 1; t = dict(t, w=ws)
        new_tr.append(t)
    if n: del_unit(u); add_unit(u, r['net'], new_tr, r['vias'])
    else: U[u] = U.pop(u)
    return before, post, n
def skew_match(board, routes, options, target_folder):
    import collections, hashlib, re
    from shapely.geometry import LineString, Point, Polygon
    from shapely.ops import unary_union
    folder = Path(target_folder); folder.mkdir(parents=True, exist_ok=True)
    def sha(v): return hashlib.sha256(json.dumps(v,sort_keys=True).encode()).hexdigest()
    def signature(d):
        return collections.Counter((v['type'],v['description'],json.dumps([(i['description'],i['pos']) for i in v['items']],sort_keys=True))
            for v in d['violations'] if v['severity']=='error' and v['type']!='skew_out_of_range')
    def opens(d):
        return collections.Counter(re.search(r'\[([^]]+)\]',v['items'][0]['description'])[1] for v in d['unconnected_items'])
    def allowed(old,new):
        return not(signature(new)-signature(old)) and not(opens(new)-opens(old))
    current = copy.deepcopy(routes)
    drc = native_route_verify(board,current,options,folder/'begin')
    native_folder=folder/'begin'
    report = dict(version='native-skew-v1', move_records=[], tried_kicad_native=0, before=native_skew_error(drc),
                  input_sha=sha(routes), kicad_native_before=dict(faults=sum(v['severity']=='error' for v in drc['violations']),open_items=len(drc['unconnected_items'])))
    boundary=border_polygon(board).buffer(-float(options.get('edge',.3)))
    clearance=max(float(options.get('drc',.09)),float(options.get('drcpad',.09)))+.002
    vd=float(options.get('via','0.45/0.20').split('/')[0])
    maxmoves=int(options.get('skew_moves','24'));maxnative=int(options.get('skew_candidates','80'))
    for iteration in range(maxmoves):
        errors=native_skew_error(drc)
        if not errors: break
        moved=False
        for error in errors:
            net=error['net'];partner=error['partner'];delta=error['delta_mm']-.35*error['tolerance_mm']

            foreign={}; own={}; partners={}
            for t in current['tracks']:
                L=t['layer']
                for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']):
                    if math.dist(a,b)<1e-7:continue
                    g=LineString([a,b]).buffer(w/2)
                    if t['net']!=net:foreign.setdefault(L,[]).append(g)
                    if t['net']==partner:partners.setdefault(L,[]).append(g)
            for v in current['vias']:
                for L in v.get('layers') or ['F']+['In%d'%i for i in range(1,board.get('cu_layers',4)-1)]+['B']:
                    (foreign if v['net']!=net else own).setdefault(L,[]).append(Point(v['x'],v['y']).buffer(v.get('d',vd)/2))
            for p in board['pads']:
                for L,polys in p.get('polys',{}).items():
                    (foreign if p['net']!=net else own).setdefault(L,[]).extend(Polygon(q) for q in polys)
            obstacles={L:unary_union(gg) for L,gg in foreign.items()}


            spans=[]
            for ti,t in enumerate(current['tracks']):
                if t['net']!=net or t.get('items'):continue
                pts=t['pts'];ws=t['w']
                for i in range(len(ws)):
                    for j in range(i+1,len(pts)):
                        widths=ws[i:j]
                        if max(widths)-min(widths)>1e-7:break
                        a,b=pts[i],pts[j];L=math.dist(a,b)
                        if L<.25:continue
                        dx,dy=abs(a[0]-b[0]),abs(a[1]-b[1])
                        oldlen=sum(math.dist(x,y) for x,y in zip(pts[i:j],pts[i+1:j+1]))

                        if min(dx,dy,abs(dx-dy))<=2e-5:
                            spans.append((-L,ti,i,j,oldlen,(tuple(a),tuple(b)),0))
                        else:



                            diag=min(dx,dy);sx=1 if b[0]>a[0] else -1;sy=1 if b[1]>a[1] else -1
                            for mid in ((a[0]+sx*diag,a[1]+sy*diag),(b[0]-sx*diag,b[1]-sy*diag)):
                                chain=(tuple(a),mid,tuple(b))
                                for ai in range(2):
                                    length=math.dist(chain[ai],chain[ai+1])
                                    if length>=.25:spans.append((-length,ti,i,j,oldlen,chain,ai))
            spans.sort(key=lambda q:(len(q[5])>2,q))
            candidates=[]
            for _,ti,i,j,oldlen,chain,ai in spans:
                t=current['tracks'][ti];a,b=chain[ai],chain[ai+1];w=t['w'][i];L=math.dist(a,b);layer=t['layer']
                baselen=sum(math.dist(x,y) for x,y in zip(chain,chain[1:]))
                e=((b[0]-a[0])/L,(b[1]-a[1])/L);normal=(-e[1],e[0])
                rest=[g for g in own.get(layer,[]) if g.distance(Point(a))>w and g.distance(Point(b))>w]
                for tj,tt in enumerate(current['tracks']):
                    if tt['net']!=net or tt['layer']!=layer:continue
                    rest.extend(LineString([pa,pb]).buffer(ww/2) for si,(pa,pb,ww) in enumerate(zip(tt['pts'][:-1],tt['pts'][1:],tt['w'])) if tj!=ti or not max(0,i-1)<=si<min(len(tt['w']),j+1))
                selfgeo=unary_union(rest)
                for fraction in (1.,.5,.25,.125,.0625):
                    added=delta*fraction
                    if added < .01:continue
                    extra=added+oldlen-baselen
                    if extra<=.005:continue
                    c=max(w/2,.04);pitch=max(2*w+clearance,w+float(options.get('pair_gap',.2)));top=pitch-2*c
                    from common.meander import crests_shapes as _ma_shapes, dome_track as _ma_track
                    for fi_,(n_,s_,r_,A_,F_) in enumerate(_ma_shapes(extra,L,w,clearance) if options.get('skew_shape','rounded')=='rounded' else []):
                        leadspace=L-F_

                        for lead in [x for x in dict.fromkeys([leadspace/2,.05,leadspace-.05,leadspace*.25,leadspace*.75]) if x>=.05-1e-9 and leadspace-x>=.05-1e-9]:
                            for side in (1,-1):
                                points,sp_items=_ma_track(chain,ai,n_,s_,r_,A_,lead,side,w)
                                line=LineString(points);copper=line.buffer(w/2)
                                full=t['pts'][:i]+points+t['pts'][j+1:]
                                if not LineString(full).is_simple or not line.is_simple or not boundary.covers(copper):continue
                                obs=obstacles.get(layer)
                                if obs is not None and copper.distance(obs)<clearance:continue
                                pocket=LineString(points[1:-1]).buffer(w/2)

                                if not selfgeo.is_empty and pocket.distance(selfgeo)<max(clearance,w):continue
                                coupling=min((line.distance(g) for g in partners.get(layer,[])),default=1e3)

                                rank=(-added,0,fi_,coupling,j-i,ti,i,j,side,lead,'dome',chain,ai)
                                candidates.append((rank,ti,i,j,points,added,w,layer,'dome',sp_items))
                    for shape in ('trombone','triangle'):
                        height=(extra+4*c*(2-math.sqrt(2)))/2 if shape=='trombone' else extra/(2*(math.sqrt(2)-1))
                        span=4*c+top if shape=='trombone' else 2*height
                        if shape=='trombone' and height<2*c:continue
                        leadspace=L-span
                        if leadspace<.04:continue
                        for lead in dict.fromkeys([leadspace/2,.02,leadspace-.02,leadspace*.25,leadspace*.75]):
                            for side in (1,-1):
                                if shape=='trombone':
                                    q=[(0,0),(lead,0),(lead+c,c),(lead+c,height-c),(lead+2*c,height),(lead+2*c+top,height),(lead+3*c+top,height-c),(lead+3*c+top,c),(lead+span,0),(L,0)]
                                else:q=[(0,0),(lead,0),(lead+height,height),(lead+span,0),(L,0)]
                                points=[[round(a[0]+x*e[0]+side*y*normal[0],6),round(a[1]+x*e[1]+side*y*normal[1],6)] for x,y in q]
                                points[0]=list(a);points[-1]=list(b)
                                points=[list(p) for p in chain[:ai]]+points+[list(p) for p in chain[ai+2:]]
                                line=LineString(points);copper=line.buffer(w/2)
                                full=t['pts'][:i]+points+t['pts'][j+1:]
                                if not LineString(full).is_simple:continue
                                if not line.is_simple or not boundary.covers(copper):continue
                                obs=obstacles.get(layer)
                                if obs is not None and copper.distance(obs)<clearance:continue

                                pocket=LineString(points[1:-1]).buffer(w/2)
                                if not selfgeo.is_empty and pocket.distance(selfgeo)<max(clearance,2*w):continue
                                coupling=min((line.distance(g) for g in partners.get(layer,[])),default=1e3)

                                rank=(-added,1,height,coupling,j-i,ti,i,j,side,lead,shape,chain,ai)
                                candidates.append((rank,ti,i,j,points,added,w,layer,shape,None))
                if len(candidates)>100:break
            candidates.sort(key=lambda q:q[0])
            seen=set()
            for _,ti,i,j,points,added,w,layer,shape,sp_items in candidates:
                key=(ti,i,j,tuple(map(tuple,points)))
                if key in seen:continue
                seen.add(key)
                if report['tried_kicad_native']>=maxnative:break
                candidate=copy.deepcopy(current);t=candidate['tracks'][ti]
                if sp_items is not None:
                    before=[['L',list(p),list(q),ww] for p,q,ww in zip(t['pts'][:i],t['pts'][1:i+1],t['w'][:i]) if math.dist(p,q)>1e-9]
                    post=[['L',list(p),list(q),ww] for p,q,ww in zip(t['pts'][j:],t['pts'][j+1:],t['w'][j:]) if math.dist(p,q)>1e-9]
                    t['items']=before+sp_items+post
                t['pts']=t['pts'][:i]+points+t['pts'][j+1:]
                t['w']=t['w'][:i]+[w]*(len(points)-1)+t['w'][j:]
                for name in ('nm_pts','nm_w'):t.pop(name,None)
                if sp_items is not None:
                    from common.track_items import validate as _bi_validate
                    try:_bi_validate(t)
                    except ValueError:continue
                report['tried_kicad_native']+=1
                nd=native_route_verify(board,candidate,options,folder/('candidate_%03d'%report['tried_kicad_native']))
                ne=native_skew_error(nd)
                oldscore=sum(max(0,q['delta_mm']-q['tolerance_mm']) for q in errors)
                newscore=sum(max(0,q['delta_mm']-q['tolerance_mm']) for q in ne)

                oldmap={(q['rule'],q['net']):q['delta_mm'] for q in errors}
                safe=all(q['delta_mm']<=oldmap.get((q['rule'],q['net']),q['tolerance_mm'])+1e-6 for q in ne)
                if allowed(drc,nd) and safe and newscore<oldscore-.005:
                    report['move_records'].append(dict(net_name=net,rule=error['rule'],layer=layer,shaping=shape,geometric_plus_mm=added,
                       before_skew_mm=error['delta_mm'],rest=[q for q in ne if q['rule']==error['rule']],evidence=str(folder/('candidate_%03d'%report['tried_kicad_native']))))
                    current=candidate;drc=nd;moved=True;native_folder=folder/('candidate_%03d'%report['tried_kicad_native'])
                    print('Skew: %s +%.4f mm, remainder %s'%(net,added,[q['delta_mm'] for q in ne if q['rule']==error['rule']]),flush=True)
                    break
            if moved:break
        if not moved:break
    report.update(post=native_skew_error(drc),out_port_sha=sha(current),
                  native_folder=str(native_folder),
                  drc_sha256=hashlib.sha256((native_folder/'drc.json').read_bytes()).hexdigest() if (native_folder/'drc.json').exists() else None,
                  pcb_sha256=hashlib.sha256((native_folder/'board.kicad_pcb').read_bytes()).hexdigest() if (native_folder/'board.kicad_pcb').exists() else None,
                  kicad_native_after=dict(faults=sum(v['severity']=='error' for v in drc['violations']),open_items=len(drc['unconnected_items'])))
    current['skew_matching']=report
    (folder/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    return current


class _Labels:
    def __init__(self):
        self.cpu = 0.0
        self.an = bool(args.get('labels'))
        self.rounds_n = []; self.conflict_list = []; self.signatures = {}; self.change = {}
        self.names = sorted(n for n in by_net if not n.startswith('#'))
        self.nid = {n: i for i, n in enumerate(self.names)}
        self.nx = (NX + 4) // 5; self.ny = (NY + 4) // 5
        if self.an:
            self.hmax = np.zeros((NL, self.ny, self.nx), np.float32)
            self.kmap = np.zeros_like(self.hmax, dtype=np.uint32)
            self.maps = []

    def coarse(self, a, medium=False):
        a = np.asarray(a).reshape(NL, NY, NX)
        p = np.pad(a, ((0, 0), (0, self.ny * 5 - NY), (0, self.nx * 5 - NX)))
        p = p.reshape(NL, self.ny, 5, self.nx, 5)
        return p.mean((2, 4)) if medium else p.max((2, 4))

    def grid_cells(self, c):
        c = np.asarray(c, dtype=np.int64)
        c = c[(c >= 0) & (c < NL * NN)]
        return (c // NN) * self.nx * self.ny + ((c % NN) // NX // 5) * self.nx + (c % NX) // 5

    def net_name(self, u):
        if u in U: return U[u]['net']
        if u.startswith('GND#'): return D['pads'][int(u[4:])]['net']
        return u[4:] if u.startswith('FIX#') else u

    def conflict_copy(self, conf):
        if not self.an: return None
        t = time.process_time()
        try: return {u: list(c) for u, c in conf.items()}
        except Exception as e:
            self.an = False; print('Labels: conflict copy skipped:', repr(e)[:180], flush=True)
        finally: self.cpu += time.process_time() - t

    def complete(self, conf, tight):
        if not self.an: return
        t = time.process_time()
        try:
            for u, c in tight.items(): conf.setdefault(u, []).extend(c)
        except Exception as e:
            self.an = False; print('Labels: conflict supplement skipped:', repr(e)[:180], flush=True)
        finally: self.cpu += time.process_time() - t

    def observe(self, conf, pass_, phase='negotiation', geometry=True):
        if not self.an or phase == 'initial_build': return
        t = time.process_time()
        try:
            counts = np.zeros(len(self.names), np.uint32); cells = []
            for u, c in conf.items():
                n = self.net_name(u)
                if n in self.nid: counts[self.nid[n]] += len(c)
                if len(c): cells.append(self.grid_cells(c))
            if cells:
                self.kmap.ravel()[np.unique(np.concatenate(cells))] += 1
            np.maximum(self.hmax, self.coarse(hist), out=self.hmax)
            self.rounds_n.append((pass_, phase)); self.conflict_list.append(counts)
            if geometry:
                by = {}
                for u in sorted(U):
                    r = U[u]; by.setdefault(r['net'], []).append((r['tracks'], r['vias']))
                for n, g in by.items():
                    sig = json.dumps(g, sort_keys=True)
                    if self.signatures.get(n) != sig:
                        self.signatures[n] = sig; self.change[n] = pass_
            if args.get('labels_rounds') == '1': self.maps.append(self.coarse(hist).astype(np.float32))
        except Exception as e:
            self.an = False; print('Labels: observation skipped:', repr(e)[:180], flush=True)
        finally: self.cpu += time.process_time() - t

    def output_assignment(self, tracks, vias, zone_list):
        from postprocessing.short_circuit_protection import output_assignment
        return output_assignment(dict(tracks=tracks, vias=vias, zone_list=list(zone_list), options=args),
                               LAYERS, np.array([X0,Y0]), (self.ny,self.nx))

    def write_out(self, tracks, vias, only_cache=False, absent=(), zone_list=()):
        global _LEARN_LABELS
        if not self.an: return
        t = time.process_time()
        try:
            import os, tempfile
            nr = len(self.names); a = np.zeros((nr, 7), np.float32)
            failed = {self.net_name(u) for u in set(fail) | set(absent)}
            need = {self.net_name(u) for u in units}
            if globals().get('_LEARN_ON'): failed |= {self.net_name(u) for u in set(units) - set(U) - set(FIXED)}
            present = {r['net'] for r in U.values()}
            _r45_label = (globals().get('_safe') or {}) if not only_cache else {}
            _r45_board = _r45_label.get('rest_board')
            if _r45_board: present = {q['net'] for q in tracks+vias}
            lm = np.zeros((nr, NL), bool); vmap = np.zeros_like(self.kmap)
            for n, i in self.nid.items():
                ps = ([p for p in _r45_board['pads'] if p['net'] == n] if _r45_board else [D['pads'][k] for k in by_net[n]])
                hp = max(p['x'] for p in ps) - min(p['x'] for p in ps) + max(p['y'] for p in ps) - min(p['y'] for p in ps)
                a[i, 0] = n not in failed and (n not in need or n in present)
                a[i, 2] = hp; a[i, 6] = self.change.get(n, 0)
            for tr in tracks:
                i = self.nid.get(tr['net'])
                if i is None: continue
                a[i, 1] += sum(math.dist(x, y) for x, y in zip(tr['pts'][:-1], tr['pts'][1:]))
                lm[i, LI[tr['layer']]] = True
            for v in vias:
                i = self.nid.get(v['net'])
                if i is not None: a[i, 4] += 1; lm[i, :] = True
                x = int((v['x'] - X0) / .25); y = int((v['y'] - Y0) / .25)
                if 0 <= x < self.nx and 0 <= y < self.ny: vmap[:, y, x] += 1
            a[:, 3] = np.divide(a[:, 1], a[:, 2], out=np.zeros(nr, np.float32), where=a[:, 2] > 0)
            a[:, 5] = lm.sum(1)
            search_occ = self.coarse(use > 0, medium=True).astype(np.float16)
            protected = args.get('short_circuit_protection', '0') == '1'
            occ = self.output_assignment(tracks, vias, zone_list) if protected else search_occ
            data = dict(hist_context=np.asarray(args.get('hist_context', '')), label_version=np.asarray(2 if protected else 1, np.int16), observer_version=np.asarray(4 if protected else 3, np.int16), origin_mm=np.array([X0, Y0]), raster_mm=np.asarray(.25),
                        cu_layers=np.asarray(LAYERS), history_max=self.hmax, history_final=self.coarse(hist),
                        occupancy=occ, occupancy_search=search_occ, occupancy_contract=np.asarray('output_copper_cell_contact' if protected else 'search_occupancy_share'),
                        conflict_rounds=self.kmap, vias_raster=vmap,
                        net_list=np.asarray(self.names), net_columns=np.asarray(['routed', 'length_mm', 'hpwl_mm', 'detour_hpwl', 'vias', 'layers_n', 'last_change']),
                        net_values=a, net_layers=lm, net_conflicts=np.asarray(self.conflict_list, np.uint32).reshape(-1, nr),
                        passes=np.asarray([r[0] for r in self.rounds_n], np.int32), phases=np.asarray([r[1] for r in self.rounds_n]))
            if not protected:
                del data['occupancy_search']; del data['occupancy_contract']
            if _r45_board:
                data['r45_placement_json'] = np.asarray(json.dumps(_r45_label['rest_placement'], sort_keys=True))
                data['r45_target_pad_json'] = np.asarray(json.dumps(_r45_board['pads'], sort_keys=True))
            if self.maps: data['history_rounds'] = np.asarray(self.maps)
            if only_cache:
                _LEARN_LABELS = {k: v.copy() for k, v in data.items()}
                return
            dest = os.path.abspath(args['labels']); os.makedirs(os.path.dirname(dest), exist_ok=True)
            fd, tmp = tempfile.mkstemp(dir=os.path.dirname(dest), suffix='.npz'); os.close(fd)
            try:
                np.savez_compressed(tmp, **data); os.replace(tmp, dest)
            finally:
                if os.path.exists(tmp): os.unlink(tmp)
            print('Labels: %s (%d nets, %d observations, %.3f CPU-s)' % (dest, nr, len(self.rounds_n), self.cpu + time.process_time() - t), flush=True)
        except Exception as e: print('Labels: writing skipped:', repr(e)[:180], flush=True)
        finally: self.cpu += time.process_time() - t


_ET = None



by_net = {}
for k, p in enumerate(D['pads']):
    if p['net'] and not p['net'].startswith('unconnected'):
        by_net.setdefault(p['net'], []).append(k)
def span(n):
    pads = [D['pads'][k] for k in by_net[n]]
    return max(p['x'] for p in pads) - min(p['x'] for p in pads) + max(p['y'] for p in pads) - min(p['y'] for p in pads)


def learn_series_value(n, value):
    if NC is not None and n in by_net:
        category=(NC['net_list'].get(n) or {}).get('category','signal')
        factor=float(args.get('cost_rank_'+category,'1'))
        if factor!=1.:return value-(factor-1.)*math.log1p(span(n))/5.
    return value


def learn_startspan(n, power=False):
    value=span(n)
    if NC is not None:
        category=(NC['net_list'].get(n) or {}).get('category','signal')
        factor=float(args.get('cost_rank_'+category,'1'))
        if factor!=1.:return value*(4*factor-3 if power else 3-2*factor)
    return value
NETTIES = set(NC.get('net_ties', [])) if NC else set()
def grp(n):
    if n in PLANE_GROUPS:return (4.8,0)
    if pair_fast(n):
        k = NC['net_list'][n]
        return (-2, -k.get('pair_rank', k.get('rank', 0)), min(n, PAIR_NETS[n]), n)
    if current_model(n): return (-1.5, -sum(map(float,current_model(n)['load'].values())), n)
    if n.startswith('GND#'):
        r_ = D['pads'][int(n[4:])]['ref']
        if r_.startswith('NT_') or r_ in NETTIES: return (-1, 0) if NC else (0, 0)
        if int(n[4:]) in VAP_IMPORTANT: return (4.8, 0)
        return (5, 0) if NC else (4, 0)
    if NC is not None and n in NC['net_list']:
        k = NC['net_list'][n]; g = k.get('group', 4)
        _rank_f = float(args.get('cost_rank_' + k.get('category', 'signal'), '1'))
        rank = k.get('rank', 0) * float(args.get('cost_rank_' + k.get('category', 'signal'), '1'))
        if g == 2: return (2 - .1 * (_rank_f-1), -learn_startspan(n,power=True))
        if ORDER >= 2:
            if g == 1 and k.get('rank', 0) < FAST_FROM: g = 3.5
            elif g == 4 and k.get('rank', 0) <= 0: g = 4.5
        return (g - .1 * (_rank_f-1), -rank, learn_startspan(n))
    pr = cls(n)['prio']
    if pr <= 1: return (0, span(n))
    if pr in (2, 3, 5): return (1, -span(n))
    if pr < 8: return (2, span(n))
    return (3, span(n))
units = [n for n in by_net if n not in GNDS and n not in AREA_NETS and len(by_net[n]) > 1 and not n.startswith('#')]
if args.get('only') == 'power':
    units = [n for n in units if cls(n)['prio'] < 8]
esp61 = 0
WITHOUT = {}
for spec in args.get('gnd_without_via', '').split(';'):
    if ':' in spec: r_, nums = spec.split(':'); WITHOUT[r_] = set(nums.split(','))
for _g in sorted(AREA_NETS):
    for k in by_net.get(_g, []):
        if D['pads'][k]['num'] in WITHOUT.get(D['pads'][k]['ref'], ()): continue
        if any(L in GND_LAYERS for L in D['pads'][k]['layers']): continue
        units.append('GND#%d' % k)

units += [g for g in sorted(GNDS - AREA_NETS) if len(by_net.get(g, [])) > 1]
PLANE_GROUPS = {}
if PLANE_FEEDERS:
    for _n in sorted(PLANE_MIXED_NETS):
        _far=[k for k in by_net.get(_n,[]) if PLANE_AREA[_n].distance(_PT(D['pads'][k]['x'],D['pads'][k]['y']))>6.]
        if not _far:continue
        PLANE_GROUPS[_n]=sorted(_far,key=lambda k:(PLANE_AREA[_n].distance(_PT(D['pads'][k]['x'],D['pads'][k]['y'])),k))
        _remove={'GND#%d'%k for k in _far};units=[u for u in units if u not in _remove];units.append(_n)
FIXED_GND = {'FIX#' + g for g in AREA_NETS}
FIXED = []
if 'fix' in args:
    FX = json.load(open(args['fix']))
    if PIN_ACCESS and any(abs(float(v.get('d',VIA_D))-VIA_D)>1e-9 or abs(float(v.get('drill',VIA_DR))-VIA_DR)>1e-9 for v in FX['vias']):
        raise ValueError('pin_access: fix= with mixed via ground masses not supported in the global router model')
    byn = {}
    for t in FX['tracks']: byn.setdefault(t['net'], ([], []))[0].append(dict(layer=t['layer'], pts=t['pts'], w=t['w']))
    for v in FX['vias']: byn.setdefault(v['net'], ([], []))[1].append(dict(x=v['x'], y=v['y']))
    for n, (tr, vs) in byn.items():

        add_unit('FIX#' + n, n, tr, vs); FIXED.append('FIX#' + n)
    fixnets = set(byn)
    if args.get('fixed_hard', '1') == '1':
        fo = np.zeros(NL * NN, dtype=bool); fdr = np.zeros(NN, dtype=bool)
        for u in FIXED:
            fo[U[u]['cells']] = True; fdr[U[u]['dcells']] = True
        FIXD[0] = [(_edt(~fo[li * NN:(li + 1) * NN].reshape(NY, NX)) * RES).astype(np.float32) for li in range(NL)]
        FIXD[1] = (_edt(~fdr.reshape(NY, NX)) * RES).astype(np.float32)
        FIXNETS.update(fixnets); _HM.clear(); _ERR.clear(); _FIX_MAPS.clear()
        print('fixed copper as hard obstacles')


    def fix_pad_joined(k):
        p = D['pads'][k]; net = p['net']; tr, vs = byn.get(net, ([], []))
        for L in p['layers']:
            g = shapely.union_all([Polygon(q) for q in p['polys'].get(L, [])])
            copper_map = [g] + [_LS((a, b)).buffer(w / 2) for t in tr if t['layer'] == L for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w'])]
            seen = {0}; open_items = [0]
            while open_items:
                j = open_items.pop()
                if any(copper_map[j].distance(Point(v['x'], v['y'])) <= VIA_R + 1e-7 for v in vs): return True
                for q, h in enumerate(copper_map):
                    if q not in seen and copper_map[j].distance(h) <= 1e-7: seen.add(q); open_items.append(q)
        return False
    units = [u for u in units if (not fix_pad_joined(int(u[4:])) if u.startswith('GND#') else u not in fixnets)]
    print('pinned:', len(FIXED), 'Nets from', args['fix'])
units.sort(key=grp)
if BGA_FANOUT:

    def bga_ring(u):
        ks = [int(u[4:])] if u.startswith('GND#') else by_net.get(u, [])
        return min((BGA[id(D['pads'][k])]['ring'] for k in ks if id(D['pads'][k]) in BGA), default=-1)
    units.sort(key=lambda u: (grp(u)[:2], bga_ring(u), grp(u)[2:], u))
if PAIR_NETS:
    _positions = {u:i for i,u in enumerate(units)}
    units.sort(key=lambda u: (grp(u)[0], -(NC['net_list'].get(u) or {}).get('pair_rank',0),
                             min(u,PAIR_NETS[u]) if u in PAIR_NETS else '', u if u in PAIR_NETS else _positions[u]))
print('Units', len(units), '(Nets', sum(1 for u in units if not u.startswith('GND#')), ')')
if NC is not None and (ORDER >= 2 or FAST_LAYER > 0 or FAST_INNER > 0 or FAST_VIA != 1 or WEIGHTED):
    _sn = [u for u in units if actual_fast(u)]; _lfs = sorted({cls(u).get('lf') for u in _sn if cls(u).get('lf')})
    print('Rules R1/R14/R21: order=%d, fast_from=%d (%d fast, %d medium-fast nets), fast_via=%g, weighted=%d; layers %s' % (
        ORDER, FAST_FROM, len(_sn), sum(1 for u in units if u in NC['net_list'] and NC['net_list'][u].get('category') == 'fast' and not actual_fast(u)),
        FAST_VIA, WEIGHTED, ', '.join('%s %s' % (L, {2: 'next to ground', 1: 'next to area', 0: 'without area'}[LAYER_BASELINE[i]]) for i, L in enumerate(LAYERS))) +
        ('; cost factor of fast nets: %s' % ' / '.join(', '.join('%s x%.2f' % (L, f) for L, f in zip(LAYERS, lf)) for lf in _lfs) if _lfs else '; layer choice without effect (all layers equally good or fast_layer=0)'), flush=True)
MAXIT = int(args.get('iter', '40')); HINC = float(args.get('hinc', '0.4'))
PFMAX = float(args['pfmax']) if 'pfmax' in args else None
if PFMAX is not None and (not math.isfinite(PFMAX) or PFMAX <= 0): raise ValueError('pfmax must be finite and > 0')
if PFMAX is not None: PF[0] = min(PF[0], PFMAX)
t0 = time.time()
fail = set()



CPU_MAX = float(args.get('cpu_max', '0'))
if _LEARN_ON:
    args.setdefault('history', os.path.join(os.path.dirname(OUT), 'history.jsonl'))
_HISTORY = open(args['history'], 'w') if args.get('history') else None
_HISTORY_CPU = [time.process_time()]
_LEARN_ROWS = []
_LEARN_STOP = None
_LEARN_BASIS = CPU_MAX
_LEARN_MAX = float(args.get('learn_cpu_max', str(2 * CPU_MAX))) if args.get('learning_budget') == 'adaptive' else CPU_MAX
_LEARN_POLICY = None
_EARLY_MODEL = None
_EARLY_INFO = None
if _EARLY_ON:
    from learning import early_abort as _EARLY
    try:
        _EARLY.generator_board_verify(D)
        _EARLY_CONTEXT = _EARLY.context_verify(
            json.loads(args['early_abort_context']) if args.get('early_abort_context')
            else _EARLY.context_board(D, NL))
        _EARLY_MODEL = _EARLY.model_load(_EARLY.model_path(args))
    except (OSError, ValueError, KeyError, TypeError) as _e:
        print('Early abort blocked: model/context error %s' % repr(_e)[:180], flush=True)
if _LEARN_ON and args.get('learning_policy'):
    _LEARN_POLICY = json.load(open(args['learning_policy']))
    if _LEARN_POLICY.get('version') != 1 or _LEARN_POLICY.get('status') != 'experimental':
        raise ValueError('learning_policy: version=1, status=experimental required')


def export_geometry():
    tracks = []; vias = []
    for u, r in U.items():
        if args.get('rest_repair') == '1':
            tracks.extend(dict(t,net=r['net'],**({'current_web': True} if t.get('web') else {})) for t in r['tracks'])
            vias.extend(dict(v,net=r['net'],**({'current_via': True} if v.get('gr') else {})) for v in r['vias'])
            continue
        for t in r['tracks']: tracks.append(dict(net=r['net'], layer=t['layer'], pts=t['pts'], w=t['w'], sig=bool(t.get('sig')), **({'current_web': True} if t.get('web') else {}), **({'link_chain': True} if t.get('link_chain') else {})))
        for v in r['vias']: vias.append(dict(net=r['net'], x=v['x'], y=v['y'], current_via=True) if v.get('gr') else
                                        (dict(net=v['net'], x=v['x'], y=v['y'], ret_via=True) if v.get('rv') else dict(net=r['net'], x=v['x'], y=v['y'])))
    if args.get('short_circuit_protection', '0') == '1':
        for q, u in zip(tracks, [u for u,r in U.items() for t in r['tracks']]): q['unit'] = u if u.startswith('GND#') else q['net']
        for q, u in zip(vias, [u for u,r in U.items() for v in r['vias']]): q['unit'] = u if u.startswith('GND#') else q['net']
    for _t, _q in zip(tracks, [t for r in U.values() for t in r['tracks']]):
        if _q.get('tap'): _t['kelvin'] = True
    if globals().get('ZONE_PLAN'): vias += _ZP_VIAS
    for t in tracks:
        if _pz_active(t['net']):
            t['pts']=[_pz_snap(p) for p in t['pts']]
            t['w']=[_pz_mm(_pz_nm(w)) for w in t['w']]
            t['nm_pts']=[[_pz_nm(v) for v in p] for p in t['pts']]
            t['nm_w']=[_pz_nm(w) for w in t['w']]
    for v in vias:
        if _pz_active(v['net']):
            v['x'],v['y']=_pz_snap((v['x'],v['y']))
            v['nm_xy']=[_pz_nm(v['x']),_pz_nm(v['y'])]
    return tracks, vias


def export_contacts(*pos, **kw):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from postprocessing.short_circuit_protection import export_contacts as impl
    return impl(*pos, **kw)


def export_selection(*pos, **kw):
    from postprocessing.short_circuit_protection import export_selection as impl
    return impl(*pos, **kw)


def export_ratsnest(*pos, **kw):
    from postprocessing.short_circuit_protection import export_ratsnest as impl
    return impl(*pos, **kw)


def export_safe(*pos, **kw):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from postprocessing.short_circuit_protection import export_safe as impl
    return impl(*pos, **kw)



def export_data(tracks, vias, conf, absent=None, checkpoint=False, raw_data=False, **extra):
    data = dict(tracks=tracks, vias=vias, failed=sorted(fail if absent is None else absent),
        conflicts=conf, res=RES, clr=CLR, options=args,
        unreachable={n:v for n,v in sorted(UNREACHABLE.items())},
        area_nets=sorted(AREA_NETS), gnd_layers=GND_LAYERS, under_wmin=under_wmin())
    if WORK_MEASURE: data['work_budget'] = work_evidence()
    if KA_CONTRACT is not None:
        data['spacings'] = KA_CONTRACT
        data['spacing_legacy_covers'] = KA_LEGACY_COVERS
    if PIN_ACCESS:
        data['pin_access_exact']=dict(width_list=PZ_WIDTHS,minimax=globals().get('PZ_MINIMAX'),edges=PZ_EDGE_STAT,local={n:len(ps) for n,ps in sorted(PZ_LOCAL.items())},border_nm=PZ_BORDER_NM)
    if globals().get('WIDE_DENSE_REPORT') is not None: data['wide_dense'] = WIDE_DENSE_REPORT
    if globals().get('CURRENT_FIRST'):
        data['current_first'] = dict(net_list=CURRENT_AUDIT, potential_layers=CURRENT_POT, J=CURRENT_J,
            without_model=sorted(n for n,k in NC['net_list'].items() if k.get('category') in ('current_val','supply','sw') and not current_model(n)))
    if globals().get('ZONE_PLAN'): data.update(zone_plan_id=ZONE_PLAN['id'], zone_list=ZONE_PLAN['core_set'], native_vias=True)
    if PLANE_INFO: data['plane_domains'] = PLANE_INFO
    if PLANE_GROUPS:data['plane_groups']={n:['%s:%s'%(D['pads'][k]['ref'],D['pads'][k]['num']) for k in ks] for n,ks in sorted(PLANE_GROUPS.items())}
    if POTENTIAL_PLAN: data['potential_plan'] = POTENTIAL_PLAN
    if IMP_TAB: data['impedance_table'] = IMP_TAB
    if FIX_PAIRS: data['fix_spacings'] = FIX_PAIRS
    if args.get('fix'):
        data['fix_nets'] = sorted({r['net'] for u, r in U.items() if u.startswith('FIX#')})
    group_list = []
    for r in U.values():
        for v in r['vias']:
            if v.get('n'):
                group_list.append(dict(net_name=r['net'], x=v['x'], y=v['y'], nominal=v['n'],
                    actual=1 + sum(1 for w in r['vias'] if w.get('too') == [v['x'], v['y']])))
    if group_list: data['via_groups'] = group_list
    for key, name, enabled in (
        ('r19_short', 'R19_SHORT_STAT', globals().get('R19_SHORT')),
        ('r19_shape', 'R19_SHAPE_STAT', globals().get('R19_SHAPE')),
        ('length_matching', 'LENGTHS_REPORT', globals().get('LENGTHS_REPORT') is not None),
        ('ground_pad_vias', 'MPV', True), ('via_beside_pad', 'VAPB', True),
        ('ret_vias', 'RVB', True), ('pairs', 'PAB', True),
        ('vias_save', 'VS_LOG', args.get('vias_save') == '1'),
        ('via_fine_shift', 'FINE_SHIFT_LOG', args.get('via_fine_shift') == '1'),
        ('via_density_after', 'VD_AFTER_LOG', globals().get('VD_AFTER')),
        ('bga_fanout', 'BGB', globals().get('BGA_FANOUT'))):
        if enabled and name in globals() and (globals()[name] or key in ('length_matching', 'vias_save', 'via_fine_shift', 'bga_fanout')):
            data[key] = globals()[name]
    if globals().get('ST') and ST.get('rest_board'):
        if args.get('new'):
            from postprocessing.remaining_conflicts import placement_marker
            data.update(placement_marker(ST['rest_board'],copy.deepcopy(_REST_BOARD_SOURCE)))
        else:
            for _field in ('rest_board','rest_source','rest_moves','rest_placement','rest_native_sources'):
                if ST.get(_field) is not None: data[_field] = ST[_field]
    if args.get('remaining_conflicts') == '1' and globals().get('ST'):
        for _field in ('zones','zone_list','r41_fill_layers'):
            if ST.get(_field): data[_field] = ST[_field]
    if globals().get('REST_REPORT') is not None: data['remaining_conflicts'] = REST_REPORT
    if globals().get('_RL') is not None:
        data['routing_loop'] = _RL.summary()
        if _RL.board(D) != _RL.initial_board:
            data.update(_RL.rest.placement_marker(_RL.initial_board, _RL.board(D)))
    data.update(extra)
    if globals().get('_EARLY_INFO') is not None: data['early_abort'] = _EARLY_INFO




    if raw_data or args.get('short_circuit_protection', '0') != '1': return data
    return r44_export(data, checkpoint=checkpoint)
def r44_export(data, checkpoint=False, board=None, pairs=None, units_extra=None):
    board = D if board is None else board
    unit_list = {u:r['net'] for u,r in U.items()}
    for u, n in (units_extra or {}).items(): unit_list.setdefault(u, n)
    output_nets = {q['net'] for k in ('tracks', 'vias', 'zone_list', 'zones') for q in data.get(k, [])}
    important = {n: max(1, 20 - int(cls(n).get('prio', 10))) for n in output_nets}
    important.update({u:important.get(n, 1) for u,n in unit_list.items() if u.startswith('GND#')})
    result = export_safe(board, data, unit_list, important, list(PAIRS) if pairs is None else list(pairs), checkpoint=checkpoint)
    if checkpoint:
        result = dict(result)
        result['short_circuit_units'] = {u: (board['pads'][int(u[4:])]['net'] if u.startswith('GND#') else unit_list.get(u, u[4:] if u.startswith('FIX#') else u)) for u in result['failed']}
    return result


def rest_final(result):
    pairs = result.pop('rest_check_pairs', None)
    if args.get('short_circuit_protection', '0') != '1': return result
    if result.get('remaining_conflicts') is not None:
        result['remaining_conflicts']['before_r44'] = dict(conflict_list=len(result['conflicts']), open_items=len(result['failed']))
    board = result.get('rest_board') or D

    extra = {}
    for u in list(result['conflicts']) + [q for p in (pairs or ()) for q in p[2:4]]:
        if u in extra or u == 'PAD': continue
        if u.startswith('FIX#'): extra[u] = u[4:]
        elif u.startswith('GND#'): extra[u] = board['pads'][int(u[4:])]['net']
        else: extra[u] = u
    return r44_export(result, board=board, pairs=pairs, units_extra=extra)


_LEARN_CPU = 0.0
_LEARN_DEPTH = 0


def observer_cpu():
    return _LEARN_CPU + (_ET.cpu if _ET else 0.0)


def learn_observer(fn):
    def measure(*pos, **kw):
        global _LEARN_CPU, _LEARN_DEPTH
        if not _LEARN_ON and _HISTORY is None: return fn(*pos, **kw)
        outer = _LEARN_DEPTH == 0
        start = time.process_time(); mark = _ET.cpu if _ET else 0.0
        _LEARN_DEPTH += 1
        try: return fn(*pos, **kw)
        finally:
            _LEARN_DEPTH -= 1
            if outer:
                _LEARN_CPU += max(0.0, time.process_time() - start - ((_ET.cpu if _ET else 0.0) - mark))
    return measure


@learn_observer
def learn_snapshot(round_no, conf, phase):
    global _LEARN_SNAPSHOT


    tracks, vias = export_geometry()
    data = export_data(tracks, vias, {u: len(c) for u,c in conf.items()}, checkpoint=True,
        absent=set(fail) | (set(units) - set(U) - set(FIXED)),
        learning_state=dict(round_no=round_no, phase=phase, cpu_s=time.process_time(), pairs=list(PAIRS), censored=True),
        learning_budget=dict(basis_cpu_s=_LEARN_BASIS, current_cpu_s=CPU_MAX, max_cpu_s=_LEARN_MAX))


    import signal
    oldmask = signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGXCPU})
    try:
        if _ET:
            _ET.observe(conf, round_no, phase)
            _ET.write_out(data['tracks'], data['vias'], only_cache=True, absent=data['failed'], zone_list=data.get('zone_list', []) + data.get('zones', []))
        _LEARN_SNAPSHOT = json.dumps(data)
    finally: signal.pthread_sigmask(signal.SIG_SETMASK, oldmask)


def learn_decision():
    global CPU_MAX, _LEARN_STOP, _EARLY_INFO
    if not _LEARN_ON: return
    if globals().get('_EARLY_MODEL') is not None and args.get('learning_budget') != 'adaptive':
        _info = _EARLY.decision(_LEARN_ROWS, _EARLY_CONTEXT, CPU_MAX, _EARLY_MODEL)
        if _info:
            _EARLY_INFO = _info
            _LEARN_STOP = 'early_abort'
            print('Early abort: round %d, %d conflicts, score %.6f >= %.6f, %.1f CPU-s' %
                  (_info['round_no'], _info['conflict_list'], _info['score'], _info['threshold'], _info['cpu_s']), flush=True)
    if _LEARN_POLICY or args.get('learning_budget') == 'adaptive':
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from learning.learning_effect import decision, promising
        if _LEARN_POLICY and decision(_LEARN_ROWS, len(D['pads']), _LEARN_POLICY):
            _LEARN_STOP = 'early_abort'
        elif CPU_MAX > 0 and time.process_time() >= .65 * CPU_MAX and CPU_MAX < _LEARN_MAX and promising(_LEARN_ROWS):
            CPU_MAX = min(_LEARN_MAX, max(CPU_MAX * 1.5, time.process_time() / .6))
            print('Learning budget: falling conflicts, extended to %.1f CPU-s' % CPU_MAX, flush=True)


@learn_observer
def negotiation_measure(round_no, conf, phase='negotiation'):
    if _HISTORY is None: return
    cpu = time.process_time()
    cells = np.unique(np.concatenate([np.asarray(c, dtype=np.int64) for c in conf.values()])) if conf else np.empty(0, dtype=np.int64)
    h = hist[cells]
    row = dict(round_no=round_no, phase=phase, cpu_s=cpu, cpu_round_s=cpu - _HISTORY_CPU[0], pf=float(PF[0]),
               conflict_list=len(conf), open_items=len(fail), pairs=list(PAIRS),
               hist_max=float(hist.max()), hist_conflict_max=float(h.max()) if len(h) else 0.0,
               hist_conflict_medium=float(h.mean()) if len(h) else 0.0,
               hist_conflict_null=float(np.mean(h == 0)) if len(h) else 0.0,
               net_list=sorted(conf), vias=sum(len(r['vias']) for r in U.values()))
    _HISTORY.write(json.dumps(row) + '\n'); _HISTORY.flush(); _HISTORY_CPU[0] = time.process_time()
    if _LEARN_ON:
        _LEARN_ROWS.append(row)
        learn_snapshot(round_no, conf, phase)
        if phase == 'negotiation': learn_decision()

if args.get('labels'):
    _mark_c0 = time.process_time()
    try:
        _ET = _Labels(); _ET.cpu += time.process_time() - _mark_c0
    except Exception as _e: print('Labels: initialization skipped:', repr(_e)[:180], flush=True)


@learn_observer
def initial_build_checkpoint():

    negotiation_measure(0, exact_conflicts(), 'initial_build')


def search_cpu():


    return time.process_time() + (getattr(globals().get('_AL'), 'cpu_offset', 0.))


def budget_message():
    if BUDGET_WORK:
        return 'Work budget (work_max=%d, consumed=%d jobs)' % (WORK_MAX, work_total())
    return 'Compute time budget (cpu_max=%.0f s)' % CPU_MAX

def time_tight(share=0.8):
    if BUDGET_WORK:

        work_emergency_stop()
        return work_total() >= share * WORK_MAX
    if _LEARN_ON and _LEARN_STOP: return True
    if _LEARN_ON and (args.get('learning_budget') == 'adaptive' or _LEARN_POLICY):
        return CPU_MAX > 0 and time.process_time() > share * CPU_MAX


    return CPU_MAX > 0 and (search_cpu() - observer_cpu()) > share * CPU_MAX


todo = None
if _LEARN_ON: learn_snapshot(0, {}, 'before_routing')
if _AGENT_SAVED is not None:
    pass
elif 'start' in args:

    ST = _REST_START_SOURCE if _REST_START_SOURCE is not None else json.load(open(args['start']))
    byn = {}
    for t in ST['tracks']: byn.setdefault(t['net'], ([], []))[0].append(t)
    for v in ST['vias']:
        _sv = dict(v) if args.get('rest_repair') == '1' else dict(x=v['x'], y=v['y'])
        if v.get('current_via'): _sv['gr'] = 1
        byn.setdefault(v['net'], ([], []))[1].append(_sv)


    NEW = {r for r in args.get('new', '').split(',') if r}
    new_nets = {D['pads'][k]['net'] for k in range(len(D['pads'])) if D['pads'][k]['ref'] in NEW} - AREA_NETS - {''}
    new_nets |= {n for n in args.get('new_nets', '').split(',') if n}
    new_nets = {n for n in new_nets if n in by_net and len(by_net[n]) > 1}
    zone = [(p['x'], p['y']) for p in D['pads'] if p['ref'] in NEW]
    gnd_new = [k for k, p in enumerate(D['pads']) if p['net'] in AREA_NETS and
               (p['ref'] in NEW or any(math.hypot(p['x'] - a, p['y'] - b_) < 1.5 for a, b_ in zone))]


    fail |= {n for n in (ST.get('failed') or []) if n in units and n not in new_nets} - {'GND#%d' % k for k in gnd_new}
    def in_pad(x, y, ks):
        for k in ks:
            bb = D['pads'][k]['bbox']
            if bb[0] - 0.05 <= x <= bb[2] + 0.05 and bb[1] - 0.05 <= y <= bb[3] + 0.05: return True
        return False
    for n, (tr, vs) in byn.items():
        if n in new_nets: continue
        if n in AREA_NETS:
            way_v = set()
            tr_keep = []
            for t in tr:
                ends = [tuple(t['pts'][0]), tuple(t['pts'][-1])]
                if any(in_pad(x, y, gnd_new) for x, y in ends):
                    for x, y in ends: way_v.add((round(x, 3), round(y, 3)))
                    continue
                tr_keep.append(t)
            vs_keep = [v for v in vs if (round(v['x'], 3), round(v['y'], 3)) not in way_v and not in_pad(v['x'], v['y'], gnd_new)]
            add_unit('FIX#' + n, n, [dict(t) if args.get('rest_repair') == '1' else dict(layer=t['layer'], pts=t['pts'], w=t['w']) for t in tr_keep], vs_keep); FIXED.append('FIX#' + n)
            continue
        C = cls(n); Cs = cls('/__signal__')


        _wk = lambda t: (Cs if t.get('sig') else C)['wmin']
        tr2 = [dict(t) if args.get('rest_repair') == '1' else dict(net=n, layer=t['layer'], pts=t['pts'], w=[(wi if args.get('rest_checking') == '1' else min(wi, _wk(t))) for wi in t['w']], sig=bool(t.get('sig')), **({'web': True} if t.get('current_web') else {})) for t in tr]
        add_unit(n, n, tr2, vs)
        for _t2, _t in zip(tr2, tr):
            if _t.get('kelvin'): _t2['tap'] = True
    if args.get('rest_check') == '1':
        _REST_LOCAL_CONFLICT = True
        _rc = exact_conflicts(); _rt, _rv = export_geometry()
        json.dump(export_data(_rt, _rv, {u: len(v) for u,v in _rc.items()}, raw_data=True, rest_check_pairs=list(PAIRS)), open(OUT, 'w'))
        sys.exit(0)
    if args.get('rest_repair') == '1':
        from postprocessing.remaining_conflicts import repair_export
        repair_export(globals())
        sys.exit(0)
    if NEW or new_nets:

        new_units = sorted(new_nets, key=lambda n: (grp(n), n)) + ['GND#%d' % k for k in gnd_new]
        for u in pair_series(new_units):
            if route_unit(u): fail.difference_update(pair_nets([u]))
            else:
                if pair_unit(u): fail.update((u,PAIR_NETS[u]))
                else: fail.add(u)
        if args.get('rest_part_local') == '1':


            _REST_LOCAL_CONFLICT = True
            _rc=exact_conflicts();_rt,_rv=export_geometry()
            json.dump(export_data(_rt,_rv,{u:len(v) for u,v in _rc.items()},raw_data=True,rest_check_pairs=list(PAIRS)),open(OUT,'w'))
            sys.exit(0)
        units = [u for u in units if not u.startswith('GND#')] + ['GND#%d' % k for k in gnd_new]
        args['all_items'] = '0'
        print('Start off', args['start'], ': rerouted', len(new_units), 'Units for', sorted(NEW), flush=True)
    else:
        units = [u for u in units if not u.startswith('GND#')]
        MAXIT = (MAXIT if args.get('loop') == '1' or args.get('agent_loop') == '1' else (2 if PAIR_SEARCH_SPACE else 1)); conf = exact_conflicts(); todo = (set(conf) | fail | ({u for u in units if pair_unit(u)} if PAIR_SEARCH_SPACE else set())) - set(FIXED)
        print('Start off', args['start'], ':', len(U), 'Units loaded', flush=True)
else:
    _learn_cp_cpu = time.process_time()
    _series = pair_series(units)
    for _i, u in enumerate(_series):
        if (_LEARN_ON and args.get('learning_budget') == 'adaptive' and CPU_MAX > 0
                and CPU_MAX < _LEARN_MAX and _i >= max(1, len(_series) // 2)
                and time.process_time() >= .65 * CPU_MAX):



            CPU_MAX = min(_LEARN_MAX, max(CPU_MAX, time.process_time() * len(_series) / _i / .65))
            print('Learning budget: initial build progress, %.1f CPU-s' % CPU_MAX, flush=True)
        if time_tight():
            rest = pair_nets(_series[_i:]); fail.update(rest); print('%s used up in the first pass: %d units not routed' % (budget_message(), len(rest)), flush=True); break
        if not route_unit(u):
            if pair_unit(u): fail.update((u,PAIR_NETS[u]))
            else: fail.add(u)
        if u in PAIR_NETS and u > PAIR_NETS[u] and u not in fail and PAIR_NETS[u] not in fail:
            pair_couple(only={u, PAIR_NETS[u]})
        if _LEARN_ON and time.process_time() - _learn_cp_cpu >= 10:
            initial_build_checkpoint()
            _learn_cp_cpu = time.process_time()
print('Iteration 1: %.0fs, incomplete %d' % (time.time() - t0, len(fail)))
class _RoutingLoop:
    VERSION = 'routing-loop-v4-local-cpu'

    def __init__(self, G):
        import hashlib
        from postprocessing import remaining_conflicts as rest
        from analysis.evaluation import rate
        self.G, self.rest, self.rate = G, rest, rate
        self.initial_board = self.board(G['D'])
        a = G['args']
        G['_LOOP_START_CPU'] = time.process_time()
        code=Path(G['__file__']).resolve();root=code.parents[1]
        self.source_list={name:hashlib.sha256(path.read_bytes()).hexdigest() for name,path in
                      [('router',code),('netclasses_code',root/'analysis/netclasses.py')]+
                      [(name,Path(a[name])) for name in ('netclasses','design_rules') if a.get(name)]}
        self.source_list['board']=hashlib.sha256(json.dumps(self.initial_board,sort_keys=True).encode()).hexdigest()
        self.fraction = self.number(a, 'loop_share', .10, 0., .5)
        self.explicit_budget = 'loop_cpu' in a
        self.limit = self.number(a, 'loop_cpu', self.budget(G, self.fraction), 0., 86400.)
        self.move_cpu = self.number(a, 'loop_move_cpu', 5., .01, 300.)
        self.large = len(G['units']) >= 400
        self.threshold = int(self.number(a, 'loop_threshold', 80, 1, 1000000))
        if a.get('loop_maps','local') not in ('local','entire'):
            raise ValueError('loop_maps=local|entire')
        if a.get('loop_search','limited') not in ('limited','numba'):
            raise ValueError('loop_search=limited|numba')
        self.counts = []
        self.prefiltered = {}; self.rejected = {}
        self.k = int(self.number(a, 'loop_k', 4, 1, 100))
        self.n = int(self.number(a, 'loop_n', 4, 1, 100))
        self.cap = int(self.number(a, 'loop_cand', 24, 0, 256))
        self.radius = self.number(a, 'loop_radius', .8, .2, 3.)
        self.used = 0.; self.previous = None; self.stale = 0; self.cells = {}
        self.seen = set(); self.rounds = 0; self.accepted = {}; self.trials = 0
        self.accepted_moves = []
        self.disabled = None
        self.log = Path(a.get('loop_log', G['OUT'].replace('.json', '') + '_loop.jsonl'))
        self.exchange = Path(a.get('loop_state', G['OUT'].replace('.json', '') + '_bottlenecks'))
        if not G['NC']: self.disabled = 'loop needs netclasses='
        source = dict(G.get('ST') or {}, options=a)
        self.disabled = self.disabled or rest.repair_keepout(source)
        if any(source.get(k) for k in ('zones','zone_list','r41_fill_layers')):
            self.disabled = 'native areas need a coordinated copper state'
        if G.get('PAIR_NETS') or G.get('ZONE_PLAN'):
            self.disabled = 'Pair nets/zone plan need a coordinated group move'
        self.input_path = a.get('loop_moves')
        self.external = []
        if self.input_path:
            payload = json.loads(Path(self.input_path).read_text())
            if not isinstance(payload, list) or len(payload) > 256:
                raise ValueError('loop_moves: JSON list with at most 256 moves expected')
            self.external = [self.validate_move(m) for m in payload]
        self.emit(dict(event='start', standard_off=True, reason=self.disabled,
                       budget_cpu_s=self.limit, argv=list(sys.argv)))

    @staticmethod
    def budget(G, fraction):


        total = G.get('CPU_MAX', 0)
        if total > 0: return min(86400., fraction * total)
        return min(86400., max(20., .05 * len(G['units']),
                              fraction * max(0., time.process_time() - G.get('_LOOP_START_CPU', time.process_time()))))

    @staticmethod
    def board(B):
        result = copy.deepcopy({k:v for k,v in B.items() if k != 'pads'})
        result['pads'] = [copy.deepcopy({k:v for k,v in p.items() if k not in ('cells','bcells')}) for p in B['pads']]
        return result

    @staticmethod
    def number(a, key, default, lo, hi):
        v = float(a.get(key, default))
        if not math.isfinite(v) or not lo <= v <= hi:
            raise ValueError('%s must be between %s and %s' % (key, lo, hi))
        return v

    @staticmethod
    def validate_move(m):
        if isinstance(m,dict) and m.get('category') in ('window_layer','ball_via','priority'):
            from codex.agent_loop.moves import check
            return check(m)
        if not isinstance(m, dict) or m.get('category') not in ('via', 'part', 'layer_name', 'relay'):
            raise ValueError('Unknown loop move type')
        fields = dict(via={'category','unit','via','goal','cur_state'},
                      part={'category','ref','delta','cur_state'},
                      layer_name={'category','unit','track','segment','layer_name','cur_state'},
                      relay={'category','unit','cur_state'})
        if set(m) != fields[m['category']]: raise ValueError('Missing/unknown move fields')
        if not isinstance(m['cur_state'], str) or len(m['cur_state']) != 64 or any(x not in '0123456789abcdef' for x in m['cur_state']):
            raise ValueError('Move needs SHA-256 of the bottleneck state')
        if m['category'] == 'part':
            v = m['delta']
            if not isinstance(m['ref'],str) or not isinstance(v,list) or len(v)!=3:
                raise ValueError('Part move needs ref and delta=[dx,dy,rotation]')
            if any(type(x) not in (int,float) or not math.isfinite(x) for x in v): raise ValueError('Invalid delta')
            if math.hypot(*v[:2]) > .500001 or v[2] not in (0,90,180,270): raise ValueError('Part move outside 0.5 mm/90 degrees')
        else:
            if not isinstance(m['unit'],str): raise ValueError('unit missing')
            for key in ('via','track','segment'):
                if key in m and (type(m[key]) is not int or m[key]<0): raise ValueError('Invalid index')
            if m['category']=='via' and (not isinstance(m['goal'],list) or len(m['goal'])!=2 or
                    any(type(x) not in (int,float) or not math.isfinite(x) for x in m['goal'])): raise ValueError('Invalid via target')
            if m['category']=='layer_name' and not isinstance(m['layer_name'],str): raise ValueError('Invalid layer')
        return copy.deepcopy(m)

    def emit(self, row):
        def finite(v):
            if isinstance(v,float) and not math.isfinite(v):return None
            if isinstance(v,dict):return {k:finite(x) for k,x in v.items()}
            if isinstance(v,(list,tuple)):return [finite(x) for x in v]
            return v
        row = dict(version=self.VERSION, source_list=self.source_list, **row)
        self.log.parent.mkdir(parents=True, exist_ok=True)
        with self.log.open('a') as f:
            f.write(json.dumps(finite(row), sort_keys=True, allow_nan=False) + '\n'); f.flush()

    def summary(self):
        return dict(version=self.VERSION, cpu_s=round(self.used,5), attempts=self.trials,
                    accepted=dict(self.accepted), reason=self.disabled,
                    declined=dict(self.rejected), prefiltered=dict(self.prefiltered),
                    budget_cpu_s=self.limit, move_budget_cpu_s=self.move_cpu,
                    logbook=str(self.log), bottlenecks=str(self.exchange),source_list=self.source_list)

    def out(self):
        G=self.G; c=G['exact_conflicts'](); ts,vs=G['export_geometry']()
        return G['export_data'](ts,vs,{u:len(v) for u,v in c.items()},raw_data=True), list(G['PAIRS'])

    def digest(self):
        import hashlib
        G=self.G
        io={'loop_moves','loop_log','loop_state','labels','png','history','netclasses','design_rules','start','fix','further','interim_state','agent_halt','agent_round','agent_packages','agent_moves'}
        state=dict(board=self.board(G['D']), unit_list={u:{k:r[k] for k in ('net','tracks','vias')} for u,r in sorted(G['U'].items())}, failed=sorted(G['fail']),
                   source_list=self.source_list,cu_layers=G['LAYERS'],res=G['RES'],pinned=sorted(G['FIXED']),options={k:v for k,v in G['args'].items() if k not in io})
        if G['args'].get('agent_loop') == '1':
            state['negotiation']=dict(hist_sha256=hashlib.sha256(G['hist'].tobytes()).hexdigest(),
                presence=G['PF'],units_order=list(G['U']),
                specs=G.get('_AGENT_SPECS',[]),priority=G.get('_AGENT_PRIORITY',[]))
        return hashlib.sha256(json.dumps(state,sort_keys=True,allow_nan=False).encode()).hexdigest()

    def clusters(self, pairs):


        from scipy.spatial import cKDTree
        if not pairs: return []
        xy=np.asarray([(p[4],p[5]) for p in pairs]);tree=cKDTree(xy)
        balls=tree.query_ball_point(xy,self.radius);remaining=set(range(len(pairs)));groups={}



        order=sorted(remaining,key=lambda i:(-len(balls[i]),pairs[i][4],pairs[i][5],pairs[i][0],pairs[i][2],pairs[i][3],pairs[i][1]))
        for i in order:
            if i not in remaining:continue
            members=sorted(set(balls[i])&remaining,key=lambda j:tuple(pairs[j]))
            remaining.difference_update(members);groups[i]=[pairs[j] for j in members]
        G=self.G; result=[]
        for ps in groups.values():
            points=sorted({(p[4],p[5]) for p in ps}); us=sorted({u for p in ps for u in p[2:4] if u in G['U']})
            xs,ys=zip(*points); bounds=[min(xs)-self.radius,min(ys)-self.radius,max(xs)+self.radius,max(ys)+self.radius]
            pads=[i for i,p in enumerate(G['D']['pads']) if any(math.dist((p['x'],p['y']),q)<=self.radius for q in points)]
            vias=[dict(unit=u,via=i,net_name=v.get('net') or r['net'],x=v['x'],y=v['y'],involved=u in us,
                       group_protection=any(v.get(k) for k in ('n','too','gr','rv','current_via','ret_via')))
                  for u,r in sorted(G['U'].items()) for i,v in enumerate(r.get('vias',[]))
                  if any(math.dist((v['x'],v['y']),q)<=self.radius for q in points)]
            history=[]
            for p in ps:
                ix=int((p[4]-G['X0'])/G['RES']);iy=int((p[5]-G['Y0'])/G['RES'])
                if 0<=ix<G['NX'] and 0<=iy<G['NY']:
                    history.append(float(G['hist'].reshape(G['NL'],G['NY'],G['NX'])[:,iy,ix].max()))
            result.append(dict(pairs=ps, points=points, unit_list=us, bbox=bounds,
                               net_list=sorted({G['U'][u]['net'] for u in us}), pads=pads,
                               components=sorted({G['D']['pads'][i]['ref'] for i in pads}),vias=vias,
                               history_log=max(history,default=0)))
        return sorted(result,key=lambda c:(-len(c['pairs']),-c['history_log'],c['bbox']))

    def movable(self,u,via=False):
        G=self.G
        if u not in G['U'] or u.startswith('TACK#'): return False
        if u in G['FIXED'] or u.startswith('FIX#'):

            return via and u.startswith('FIX#') and 'start' in G['args'] and not G['args'].get('fix') and G['U'][u]['net'] in G['AREA_NETS']
        return True

    def neighbours(self,c):
        G=self.G
        if 'neighbors' in c:
            return [u for u in c['neighbors'] if u in G['U'] and self.movable(u)]
        return [u for u,r in G['U'].items() if self.movable(u) and G['_close_on'](r,c['points'],self.radius)]

    def part_units(self,ref):
        G=self.G
        nets={p['net'] for p in G['D']['pads'] if p['ref']==ref}-G['AREA_NETS']
        touched={u for u,r in G['U'].items() if r['net'] in nets and self.movable(u)}
        if G.get('_AL') is not None:
            touched.update(u for u in G['units'] if u in nets and u not in G['FIXED'])


        touched.update('GND#%d'%i for i,p in enumerate(G['D']['pads'])
                       if p['ref']==ref and p['net'] in G['AREA_NETS']
                       and 'GND#%d'%i in G['units'] and self.movable('GND#%d'%i))
        return touched

    def part_moves(self,c):
        import re
        from common import travel_range as WW
        G=self.G;B=G['D'];W=WW.table(G['NC']);fps={f['ref']:f for f in B['fps']}
        raw=B.get('placement_groups') or []
        groups=raw.values() if isinstance(raw,dict) else raw
        grouped={r for g in groups if isinstance(g,dict) for r in g.get('refs',[])}
        if not c['points']:return
        for ref in sorted(fps,key=lambda r:(min((math.dist((fps[r]['x'],fps[r]['y']),p) for p in c['points']),default=math.inf),r)):
            f=fps[ref];t=G['NC']['part_list'].get(ref,{})
            if (not re.search(r'(?:^|[_:])[CR]_(0201|0402|0603|0805)(?:_|$)',f.get('fp',''))
                    or ref in grouped or f.get('locked') or t.get('pinned') or t.get('role')!='small'
                    or ref not in W or W[ref]['max']<=0):continue
            nearest=min(c['points'],key=lambda p:math.dist((f['x'],f['y']),p))
            ax,ay=f['x']-nearest[0],f['y']-nearest[1]
            if math.hypot(ax,ay)>1.5:continue
            direction=(0,1 if ay>=0 else -1) if abs(ay)>=abs(ax) else (1 if ax>=0 else -1,0)
            preferred=[(round(direction[0]*d,5),round(direction[1]*d,5),0) for d in (.1,.2,.3,.4,.5)]
            rotations=[(0,0,th) for th in (90,180,270)] if W[ref]['rotate'] else []
            origin=(B.get('wander_basis') or {}).get(ref) or f
            if isinstance(origin,(list,tuple)):origin=dict(x=origin[0],y=origin[1])
            for dx,dy,th in preferred+rotations+[(x,y,0) for x,y in self.rest.offsets(.5) if (x,y,0) not in preferred]:
                if math.dist((f['x']+dx,f['y']+dy),(origin['x'],origin['y']))>W[ref]['max']+1e-8:continue
                yield dict(ref=ref,moves={ref:(dx,dy,th)})

    def external_local(self,m,c):
        G=self.G
        if m['category'] in ('window_layer','ball_via','priority'):
            from codex.agent_loop.moves import local
            return local(G,m,c,self.radius)
        if m['category']=='part':
            f=next((f for f in G['D']['fps'] if f['ref']==m['ref']),None)
            return f is not None and any(math.dist((f['x'],f['y']),p)<=1.5 for p in c['points'])
        r=G['U'].get(m['unit'])
        if r is None:return False
        try:
            if m['category']=='via':
                v=r['vias'][m['via']]
                return any(math.dist((v['x'],v['y']),p)<=self.radius for p in c['points'])
            if m['category']=='layer_name':
                from shapely.geometry import LineString
                t=r['tracks'][m['track']];j=m['segment'];line=LineString(t['pts'][j:j+2])
                return any(line.distance(Point(p))<=self.radius for p in c['points'])
        except (IndexError,ValueError):return False
        return G['_close_on'](r,c['points'],self.radius)

    def part_board(self,m,c):
        import re
        from common import travel_range as WW
        from common.parts import load_parts,circuit_board,Occupancy
        from common.shift import relocate
        from postprocessing.move_rules import keepout
        G=self.G;B=self.board(G['D']);r=m['ref'];dx,dy,th=m['delta']
        fps={f['ref']:f for f in B['fps']};f=fps.get(r);t=G['NC']['part_list'].get(r,{})
        groups=B.get('placement_groups') or []
        if isinstance(groups,dict):groups=groups.values()
        grouped={ref for group in groups if isinstance(group,dict) for ref in group.get('refs',[])}
        W=WW.table(G['NC'])
        chip=bool(G.get('_AL') is not None and f and re.search(r'(?:^|[_:])(?:SOT|SOIC)[-_]',f.get('fp',''),re.I))
        small=bool(f and re.search(r'(?:^|[_:])[CR]_(0201|0402|0603|0805)(?:_|$)',f.get('fp','')))
        if f is None or not (small or chip) or r in grouped or f.get('locked') or t.get('pinned') or t.get('role') not in (('small','chip') if chip else ('small',)) or r not in W:
            raise ValueError('Part is not a free C/R or released SOT/SOIC')
        if chip and not any(q['ref']==r for q in B.get('courtyards',[])):raise ValueError('SOT/SOIC needs exported courtyard')
        if not any(math.dist((f['x'],f['y']),p)<=1.5 for p in c['points']):raise ValueError('Part not local')
        if math.hypot(dx,dy)>.500001 or th not in (0,90,180,270) or (th and not (W[r]['rotate'] or chip)):raise ValueError('Part move outside movement contract')
        origin=(B.get('wander_basis') or {}).get(r) or next(q for q in self.initial_board['fps'] if q['ref']==r)
        if isinstance(origin,(tuple,list)):origin=dict(x=origin[0],y=origin[1])
        if W[r]['max']<=0 or math.dist((f['x']+dx,f['y']+dy),(origin['x'],origin['y']))>W[r]['max']+1e-8:raise ValueError('Travel distance exceeded')
        B1,_=relocate(B,{r:(dx,dy,th)});T=load_parts(B1,.15);fps={f['ref']:f for f in B1['fps']};bel=Occupancy()
        for ref in T:
            q=fps[ref]
            if ref!=r:bel.add(ref,T[ref].geo(q['x'],q['y'],0,q['side']))
        f=fps[r];geo=T[r].geo(f['x'],f['y'],0,f['side']);_,_,inside,_=circuit_board(B,.2)
        if bel.collision(geo) or not all(inside.contains(g) for g in geo.values()) or keepout(B1,geo,(r,)):raise ValueError('Courtyard/pad spacing/edge/keepout violated')
        return B1

    def via_move(self,owner,index,target):
        G=self.G;r=copy.deepcopy(G['U'][owner]);v=r['vias'][index];q=[v['x'],v['y']]
        for t in r['tracks']:
            ps=t['pts'];ws=t['w']
            if len(ps)<2:continue
            if math.dist(ps[0],q)<1e-6 and math.dist(q,ps[1])>.55:
                d=math.dist(q,ps[1]);a=[round(q[k]+.55/d*(ps[1][k]-q[k]),5) for k in (0,1)]
                t['pts']=[ps[0],a]+ps[1:];t['w']=[ws[0]]+ws
            elif math.dist(ps[-1],q)<1e-6 and math.dist(q,ps[-2])>.55:
                d=math.dist(q,ps[-2]);a=[round(q[k]+.55/d*(ps[-2][k]-q[k]),5) for k in (0,1)]
                t['pts']=ps[:-1]+[a,ps[-1]];t['w']=ws+[ws[-1]]
        return self.rest.via_shift(G['D'],r,index,target,G['VIA_D'],G['VIP'],owner.startswith('GND#') and int(owner[4:]) in G['EPM'],G['cls']('/__signal__')['wmin'])

    def candidates(self,c,digest):
        G=self.G; limit=max(1,self.cap//4); count=0


        vv=[]
        for u,r in sorted(G['U'].items()):
            if not self.movable(u,True): continue
            for i,v in enumerate(r['vias']):
                d=min(math.dist((v['x'],v['y']),p) for p in c['points'])
                if d<=self.radius: vv.append((u not in c['unit_list'],d,u,i))
        for _,_,u,i in sorted(vv):
            v=G['U'][u]['vias'][i]; targets=[]
            for step in (.1,.2,.3,.4,.5):
                for dx,dy in ((0,step),(0,-step),(step,0),(-step,0),(step,step),(-step,step),(step,-step),(-step,-step)):
                    if math.hypot(dx,dy)<=.500001:
                        x,y=v['x']+dx,v['y']+dy
                        targets.append((round(x,5),round(y,5)))
                        x=G['X0']+round((x-G['X0'])/G['RES'])*G['RES']; y=G['Y0']+round((y-G['Y0'])/G['RES'])*G['RES']
                        if .099999<=math.dist((v['x'],v['y']),(x,y))<=.500001: targets.append((round(x,5),round(y,5)))
            targets=sorted(set(targets),key=lambda p:(max(0., G['VIA_D']/2 + G['DRC_CLR'] - min(math.dist(p,q) for q in c['points'])),math.dist(p,(v['x'],v['y'])),p))
            for target in targets[:limit]:
                yield dict(category='via',unit=u,via=i,goal=list(target),cur_state=digest)
                count+=1
                if count>=limit: break
            if count>=limit: break
        for i,p in enumerate(self.part_moves(c)):
            if i>=limit: break
            yield dict(category='part',ref=p['ref'],delta=list(p['moves'][p['ref']]),cur_state=digest)
        count=0
        for u in c['unit_list']:
            if not self.movable(u): continue
            for i,t in enumerate(G['U'][u]['tracks']):
                if any(t.get(k) for k in ('web','current_web','kelvin','tap','rv')): continue
                for j,(a,b) in enumerate(zip(t['pts'][:-1],t['pts'][1:])):
                    from shapely.geometry import LineString
                    if not any(LineString([a,b]).distance(Point(p))<=self.radius for p in c['points']): continue
                    for layer in G['LAYERS']:
                        if layer==t['layer']: continue
                        yield dict(category='layer_name',unit=u,track=i,segment=j,layer_name=layer,cur_state=digest)
                        count+=1
                        if count>=limit: break
                    if count>=limit: break
                if count>=limit: break
            if count>=limit: break
        neighbours=sorted((u for u in G['U'] if self.movable(u) and G['_close_on'](G['U'][u],c['points'],self.radius)),key=lambda u:(u in c['unit_list'],u))
        for u in neighbours[:limit]: yield dict(category='relay',unit=u,cur_state=digest)

    @staticmethod
    def pair_key(p): return (p[0],*sorted(p[2:4]),round(p[4],2),round(p[5],2))

    def pair_gate(self,p0,p1,c):
        from collections import Counter
        b=c['bbox']
        inside=lambda p:b[0]<=p[4]<=b[2] and b[1]<=p[5]<=b[3]
        before=[p for p in p0 if inside(p)]; after=[p for p in p1 if inside(p)]
        if len(after)>=len(before): return 'Pair count in cluster does not drop'
        old=Counter(self.pair_key(p) for p in p0 if not inside(p))
        new=Counter(self.pair_key(p) for p in p1 if not inside(p))
        if new-old: return 'new conflict outside the cluster'

        distances={};after_distances={}
        for p in p0:
            if not inside(p):distances.setdefault(self.pair_key(p),[]).append(p[1])
        for p in p1:
            if not inside(p):after_distances.setdefault(self.pair_key(p),[]).append(p[1])
        for key,ds in after_distances.items():
            if any(n<o-1e-6 for o,n in zip(sorted(distances[key]),sorted(ds))):return 'Conflict outside gets worse'
        return None

    def evaluate_gate(self,B0,R0,E0,R1,p0,p1,c):
        G=self.G; K=G['NC']; B1=G['D']
        reason=self.pair_gate(p0,p1,c)
        if reason: return reason,None
        if not set(R1['failed'])<=set(R0['failed']): return 'new open unit',None
        for check in (self.rest.hard_vias(B1,R0,R1),self.rest.hard_segments(B1,R0,R1,diagnose=True),self.rest.part_clearance(B0,B1,R1)):
            if not check['ok']: return check['gate_check'],None
        from collections import Counter
        old_vias=Counter((v['net'],v['x'],v['y']) for v in R0['vias'])
        for v in R1['vias']:
            key=(v['net'],v['x'],v['y'])
            if old_vias[key]:old_vias[key]-=1;continue
            ix=int((v['x']-G['X0'])/G['RES']);iy=int((v['y']-G['Y0'])/G['RES'])
            if not (0<=ix<G['NX'] and 0<=iy<G['NY']) or not G['via_hard'](v['net'],np.asarray([iy*G['NX']+ix],dtype=np.int64)):
                return 'Via violates static grid map (including VIP)',None
        E1=self.rate(B1,R1,K,v3=True,minimax=False)
        if E1['open_items']>E0['open_items'] or not set(E1['not_connected'])<=set(E0['not_connected']): return 'Connections worse',E1
        from analysis import rule_metrics as RK
        from postprocessing.move_rules import rule_values, TOPO
        rg=RK.assumption(E0['design_rules'],E1['design_rules'])
        if not rg['not_worse']: return rg['reason'],E1
        if not math.isfinite(E1['score_v3']) or E1['score_v3']>E0['score_v3']+1e-6: return 'Score v3 worse',E1
        a=rule_values(B0,R0,K);b=rule_values(B1,R1,K)
        if any(b['topology'][k]>a['topology'][k]+1e-6 for k in TOPO) or b['coupling']>a['coupling']+1e-6: return 'Topology/coupling worse',E1
        protected=set(K['gnd'])|{n for n,k in K['net_list'].items() if k.get('category') in ('current_val','supply','sw')}
        a=Counter(v['net'] for v in R0['vias'] if v['net'] in protected);b=Counter(v['net'] for v in R1['vias'])
        if a-b: return 'protected via count decreases',E1
        a=self.rest.ground_pad_numbers(B0,R0,K);b=self.rest.ground_pad_numbers(B1,R1,K)
        if any(b.get(p,0)<n for p,n in a.items()): return 'R16 ground pad via missing',E1


        from analysis.evaluation import current_paths,current_loss_equal
        s0=current_paths(B0,R0,K,E0,dc=None);s1=current_paths(B1,R1,K,E1,dc=None)
        p0w,p1w=current_loss_equal(E0,E1,B0,R0,B1,R1,K)
        if (s1.get('detour_mm') or 0)-(s0.get('detour_mm') or 0)>1. and p1w>p0w*1.02+1e-9: return 'Current paths worse',E1
        return None,E1

    def static_preflight(self,R0,R1):
        v=self.rest.hard_vias(self.G['D'],R0,R1)
        if not v['ok'] and v['gate_check']!='Via-DrillHole-DrillHole':return v['gate_check']
        s=self.rest.hard_segments(self.G['D'],R0,R1,diagnose=True)
        return None if s['ok'] else s['gate_check']

    def prefilter(self,m,c):
        G=self.G
        if m['category']=='part':
            projected=self.part_units(m['ref'])|set(self.neighbours(c))
            limit=min(int(G['args'].get('loop_group','24')),int(G['args'].get('loop_new_max','4' if self.large else '24')))
            return 'Part group already too large before placement check' if len(projected)>limit else None
        if m['category'] != 'via': return None
        u=m['unit']
        if not self.movable(u,True): return 'fixed/missing via'
        if m['via'] >= len(G['U'][u]['vias']): return 'missing via'
        x,y=m['goal'];v=G['U'][u]['vias'][m['via']]
        if not .099999<=math.hypot(x-v['x'],y-v['y'])<=.500001: return 'Via path outside contract'
        ix=int((x-G['X0'])/G['RES']);iy=int((y-G['Y0'])/G['RES'])
        if not (0<=ix<G['NX'] and 0<=iy<G['NY']): return 'Via target outside grid'

        if not G['via_hard'](G['U'][u]['net'],np.asarray([iy*G['NX']+ix],dtype=np.int64)):
            return 'Via target statically blocked'
        if self.via_move(u,m['via'],m['goal']) is None: return 'Via-Connection/GroupProtection'
        return None

    def rebuild(self,B):
        if self.G.get('_AL') is not None:
            changed=[i for i,(a,b) in enumerate(zip(self.board(self.G['D'])['pads'],B['pads'])) if a!=b]
            if len(changed)>2:return self.rebuild_full(B)
        if self.G['args'].get('loop_maps','local') != 'entire':
            return self.rebuild_local(B)
        return self.rebuild_full(B)

    def rebuild_local(self,B):
        G=self.G;original=G.copy()
        changed,window=G['loop_pad_patch'](B)


        changed=set(changed);idx={li:0 for li in range(G['NL'])}
        for key in ('_PADG','_PADXY','_PADB'):G[key]={li:list(rows) for li,rows in G[key].items()}
        for k,p in enumerate(G['D']['pads']):
            for L in p['layers']:
                if L not in G['LI']:continue
                li=G['LI'][L];j=idx[li];idx[li]+=1
                if k not in changed:continue
                pg=G['shapely'].union_all([G['Polygon'](q) for q in p['polys'][L]])
                pn=p['net'] if p['net'] and not p['net'].startswith('unconnected') else '#%d'%id(p)
                G['_PADG'][li][j]=(pn,pg);G['_PADXY'][li][j]=(p['x'],p['y'])
                if G['DRCL']:pg=G['pad_hull'](p,L,G['DRCP'],True)
                G['_PADB'][li][j]=(pn,pg,pg.buffer(G['_DMAX']))
        G['_PADT']={li:G['shapely'].STRtree([g for _,g in rows]) for li,rows in G['_PADG'].items()}

        rn=G['VIA_KEEP_R']+max(G['DRCP'],.03)+.03
        VP=G['VP'].copy()
        for cell,(x,y,near) in VP.items():
            if not any(p['bbox'][0]-rn<=x<=p['bbox'][2]+rn and p['bbox'][1]-rn<=y<=p['bbox'][3]+rn
                       for k in changed for p in (original['D']['pads'][k],G['D']['pads'][k])):continue
            pt=Point(x,y);near=[]
            for li in range(G['NL']):
                for j in G['_PADT'][li].query(pt.buffer(rn)):
                    n,g=G['_PADG'][li][int(j)];d=float(pt.distance(g))
                    if d<rn:near.append((n,round(d,4),li in (0,G['NL']-1)))
            VP[cell]=(x,y,tuple(sorted(near)))
        G['VP']=VP
        for key in ('_HM','_ERR','NECK_CACHE','_VAPZ','_NEARBY_C','_DIST_C'):
            if key in G and isinstance(G[key],dict):G[key]={}
        G['_pad_room_cached'].cache_clear()
        G['CHAINS']={};G['KF_AFTER']={};G['KF_BEFORE']={}
        if G['CHAIN']:G['_chains_build']()
        G['_CAND_IDX']=[None];G['ASTAR_COST']=[None]
        self.last_window=list(window)
        return original

    def rebuild_full(self,B):



        import tempfile,contextlib,io
        G=self.G; original=G.copy(); argv=list(sys.argv)
        code=Path(G['__file__']).read_text().split('# Density labels are a passive observer;',1)[0]


        code=code.replace('\nbootstrap(heavy=True)\n','\n# existing process guard is kept\n',1)
        with tempfile.TemporaryDirectory(prefix='routing_loop_') as td:
            payload=self.board(B)


            payload['pads']=payload['pads'][:original['_LOOP_SOURCE_PAD_COUNT']]
            path=Path(td)/'board.json';path.write_text(json.dumps(payload))
            opt={k:v for k,v in G['args'].items() if k not in ('start','new','new_nets','hist_start','hist_context','agent_loop','further','agent_moves')}
            try:
                sys.argv=[argv[0],str(path),original['OUT']]+[k+'='+str(v) for k,v in opt.items()]
                with contextlib.redirect_stdout(io.StringIO()): exec(compile(code,G['__file__'],'exec'),G)
                for u,r in original['U'].items(): G['add_unit'](u,r['net'],copy.deepcopy(r['tracks']),copy.deepcopy(r['vias']))

                for key in ('args','t_start','PF','VIA_PREV','CONS_SOFT','VIA_ANCHOR_ON','SOURCE_FREE','HARD_FOREIGN','_REST_BOARD_SOURCE','_REST_START_SOURCE'):
                    if key in original: G[key]=copy.deepcopy(original[key])
                G['hist'][:]=original['hist']; G['fail']=set(original['fail']); G['_CAND_IDX'][0]=None
                for key in original:
                    if key.startswith('_AGENT_'):G[key]=original[key]
                if G.get('_AL') is not None:G['_AL'].install_router()
            finally: sys.argv=argv
        return original
    def apply(self,m,c):
        G=self.G; category=m['category']; owner=m.get('unit'); touched=[]
        if category in ('window_layer','ball_via','priority'):
            from codex.agent_loop.moves import apply
            return apply(self,m,c)
        if category=='part':
            marker=self.rest.placement_marker(self.initial_board,self.part_board(m,c))
            self.rebuild(marker['rest_board']); G=self.G
            ref=m['ref']; nets={p['net'] for p in G['D']['pads'] if p['ref']==ref}
            touched=list(self.part_units(ref))


            for u,r in list(G['U'].items()):
                if not u.startswith('FIX#') or r['net'] not in nets: continue
                if G['_close_on'](r,c['points'],self.radius): raise ValueError('anonymous area stub needs a joint move')
        elif category=='via':
            if not self.movable(owner,True): raise ValueError('fixed/missing via')
            r=G['U'][owner];v=r['vias'][m['via']]
            if not .099999<=math.dist((v['x'],v['y']),m['goal'])<=.500001: raise ValueError('Via path outside 0.1 to 0.5 mm')
            if not any(math.dist((v['x'],v['y']),p)<=self.radius for p in c['points']): raise ValueError('Via not local')
            r1=self.via_move(owner,m['via'],m['goal'])
            if r1 is None: raise ValueError('Via-Connection/GroupProtection')
            G['del_unit'](owner);G['add_unit'](owner,r1['net'],r1['tracks'],r1['vias'])
        elif category=='layer_name':
            if not self.movable(owner) or m['layer_name'] not in G['LAYERS']: raise ValueError('blocked unit/layer')
            r=copy.deepcopy(G['U'][owner]);i=m['track'];j=m['segment'];t=r['tracks'][i]
            if any(t.get(k) for k in ('web','current_web','kelvin','tap','rv')): raise ValueError('protected track')
            a,b=t['pts'][j:j+2];w=t['w'][j]
            from shapely.geometry import LineString
            if t['layer']==m['layer_name'] or not any(LineString([a,b]).distance(Point(p))<=self.radius for p in c['points']): raise ValueError('Layer change not local')
            if math.dist(a,b)<G['DRC_HOLE']: raise ValueError('Via pair too tight')
            replacement=[]
            if j: replacement.append(dict(t,pts=t['pts'][:j+1],w=t['w'][:j]))
            replacement.append(dict(t,layer=m['layer_name'],pts=[a,b],w=[w]))
            if j+1<len(t['w']): replacement.append(dict(t,pts=t['pts'][j+1:],w=t['w'][j+1:]))
            r['tracks'][i:i+1]=replacement
            for x,y in (a,b):
                if not any(math.dist((v['x'],v['y']),(x,y))<1e-6 for v in r['vias']):r['vias'].append(dict(x=x,y=y))
            G['del_unit'](owner);G['add_unit'](owner,r['net'],r['tracks'],r['vias'])
        else:
            if not self.movable(owner) or not G['_close_on'](G['U'][owner],c['points'],self.radius): raise ValueError('Blocker not movable/local')
            touched=[owner]

        touched+=[u for u in self.neighbours(c) if u!=owner]
        touched=sorted(set(touched),key=lambda u:(u!=m.get('unit'),G['grp'](u),u))
        if len(touched)>int(G['args'].get('loop_group','24')): raise ValueError('local group too large')
        return touched

    def trial(self,m,c,R0,p0,E0,deadline):
        start=time.process_time()
        G=self.G; original=G.copy(); B0=G['D']; order=list(G['U'])


        saved=G['U'].copy()
        hs=G['hist'].copy();vs=copy.deepcopy(G['VIA_PREV']);fails=set(G['fail'])
        flags={k:copy.deepcopy(G[k]) for k in ('PF','CONS_SOFT','HARD_FOREIGN','SOURCE_FREE','VIA_ANCHOR_ON')}
        semantic={k:copy.deepcopy(G[k]) for k in ('_METRIC_OFF','_ERR_NETS','UNREACHABLE','JOINT_STAT','CHAIN_STAT','RV_STAT','TOPO_STAT','CURRENT_AUDIT','_AGENT_SPECS','_AGENT_PRIORITY') if k in G}
        ok=False;E1=None;R1=None;p1=None;reason=None;phases={'save':time.process_time()-start}
        old_deadline=G.get('_LOOP_DEADLINE')
        stamp=[time.process_time()]
        def phase(name):
            now=time.process_time();phases[name]=now-stamp[0];stamp[0]=now
        try:
            touched=self.apply(m,c)
            phase('apply')
            if G['args'].get('loop_search','limited')=='limited':
                G['_LOOP_DEADLINE']=deadline-min(.5,max(0.,deadline-start)*.2)
            G['CONS_SOFT'][0]=False;G['HARD_FOREIGN'][0]=True
            R1,p1=self.out()
            phase('conflict_list')
            if m['category'] in ('via','layer_name','window_layer','ball_via'):
                blocked=self.static_preflight(R0,R1)
                if blocked:raise ValueError('static pre-check: '+blocked)
            phase('static')
            pair_reason=self.pair_gate(p0,p1,c)
            if m['category'] in ('via','layer_name','window_layer','ball_via') and pair_reason and 'outside' in pair_reason:
                raise ValueError(pair_reason)

            if m['category'] in ('part','priority') or pair_reason is not None:
                if len(touched)>int(G['args'].get('loop_new_max','4' if self.large else '24')):
                    raise ValueError('too many units for a cheap rerouting move')
                for u in touched:G['del_unit'](u)
                for u in touched:
                    if time.process_time()>=deadline or G['time_tight'](): raise TimeoutError('Loop-CPU-Budget')
                    if G['route_unit'](u):G['fail'].discard(u)
                    else:G['fail'].add(u)
                R1,p1=self.out()
            phase('reroute')
            if time.process_time()>=deadline or G['time_tight'](): raise TimeoutError('Loop-CPU-Budget')
            reason,E1=self.evaluate_gate(B0,R0,E0,R1,p0,p1,c)
            phase('gates')
            ok=reason is None
            if time.process_time()>=deadline:ok=False;reason='Loop CPU budget after check'
        except Exception as e:
            reason=type(e).__name__+': '+str(e)[:220]
        finally:
            phase('work_until_abort_or_final')
            G['_LOOP_DEADLINE']=old_deadline
            if not ok:
                for u in list(G['U']):
                    if G['U'][u] is not saved.get(u):G['del_unit'](u)
                for u in order:
                    if u not in G['U']:
                        r=saved[u];G['add_unit'](u,r['net'],r['tracks'],r['vias'])
                if G['D'] is not B0:


                    dynamic={k:G[k] for k in ('U','use','drill_use','SN','SN_OPEN','UOWN','NETU','DDN','_NVER','_GEO')}
                    G.clear();G.update(original);G.update(dynamic)
                G['fail'].clear();G['fail'].update(fails)
                G['VIA_PREV'].clear();G['VIA_PREV'].update(vs)
                for k,v in semantic.items():G[k]=v
            G['hist'][:]=hs
            for k,v in flags.items():G[k][:]=v
            G['_CAND_IDX'][0]=None

            current=G['U'].copy();G['U'].clear()
            for u in order:
                if u in current:G['U'][u]=current.pop(u)
            for u in sorted(current):G['U'][u]=current[u]
            G['exact_conflicts']()
            G['_LOOP_DEADLINE']=old_deadline
            phase('rollback_or_final')
        self.trials+=1
        if not ok:self.rejected[m['category']]=self.rejected.get(m['category'],0)+1
        self.emit(dict(event='move',round_no=self.rounds,bottleneck=c,move=m,taken=ok,
                       reason=reason or 'Cluster improved, full gates held',cpu_s=round(time.process_time()-start,6),
                       cpu_phases_s=phases, move_budget_cpu_s=max(0.,deadline-start),
                       pairs_before=p0,pairs_after=p1,
                       before={k:E0.get(k) for k in ('is_valid','score_v3','conflict_list','open_items','design_rules')},
                       after=None if E1 is None else {k:E1.get(k) for k in ('is_valid','score_v3','conflict_list','open_items','design_rules')}))
        if ok:
            self.accepted[m['category']]=self.accepted.get(m['category'],0)+1
            self.accepted_moves.append(copy.deepcopy(m))
        return ok

    def export(self,c,digest,quick=False):
        d=self.exchange/('%04d_%s'%(self.rounds,digest[:12]));d.mkdir(parents=True,exist_ok=True)
        G=self.G;b=c['bbox'];margin=.5;b=[b[0]-margin,b[1]-margin,b[2]+margin,b[3]+margin]
        scale=min(160,1000/max(b[2]-b[0],b[3]-b[1]));size=(max(1,int((b[2]-b[0])*scale)),max(1,int((b[3]-b[1])*scale)))
        in_=Image.new('RGB',size,'white');draw=ImageDraw.Draw(in_)
        def p(q):return ((q[0]-b[0])*scale,(q[1]-b[1])*scale)
        for u,r in sorted(G['U'].items()):
            for t in r['tracks']:draw.line([p(q) for q in t['pts']],fill=('#315e8e' if u in c['unit_list'] else '#b3bdc7'),width=max(1,int(min(t['w'],default=.1)*scale)))
            for v in r['vias']:
                x,y=p((v['x'],v['y']));rr=G['VIA_D']/2*scale;draw.ellipse((x-rr,y-rr,x+rr,y+rr),outline='#245c40',width=2)
        for i in c['pads']:
            for qs in G['D']['pads'][i]['polys'].values():
                for q in qs:draw.polygon([p(v) for v in q],outline='#9b7300')
        for q in c['points']:
            x,y=p(q);draw.ellipse((x-6,y-6,x+6,y+6),outline='red',width=3)
        in_.save(d/'conflict_list.png')


        places={}; previous=G['HARD_FOREIGN'][0];G['HARD_FOREIGN'][0]=True
        try:
            ix0=max(0,int((b[0]-G['X0'])/G['RES']));ix1=min(G['NX'],int((b[2]-G['X0'])/G['RES']))
            iy0=max(0,int((b[1]-G['Y0'])/G['RES']));iy1=min(G['NY'],int((b[3]-G['Y0'])/G['RES']))
            step=max(1,int(.2/G['RES']));cells=np.asarray([iy*G['NX']+ix for iy in range(iy0,iy1,step) for ix in range(ix0,ix1,step)],dtype=np.int64)
            for net in (() if quick else c['net_list']):

                allowed=G['hard_maps'](net)[1].ravel()
                probe=[k for k in cells[:256] if allowed[k]]
                points=[(G['X0']+(int(k)%G['NX']+.5)*G['RES'],G['Y0']+(int(k)//G['NX']+.5)*G['RES']) for k in probe]
                foreign=[geo for u,r in G['U'].items() for li,geo,_,ne in G['_unit_geo'](u,r) if ne!=net]
                holes=[Point(v['x'],v['y']) for r in G['U'].values() for v in r['vias']]
                copper=G['shapely'].union_all(foreign);drills=G['shapely'].union_all(holes)
                places[net]=[[round(x,5),round(y,5)] for x,y in points if (copper.is_empty or Point(x,y).distance(copper)>=G['VIA_D']/2+G['DRC_CLR']) and (drills.is_empty or Point(x,y).distance(drills)>=G['DRC_HOLE'])]
        finally:G['HARD_FOREIGN'][0]=previous
        clean=self.board(G['D'])
        state=dict(version=self.VERSION,cur_state=digest,bottleneck=c,cu_layers=G['LAYERS'],
                   raster=dict(res=G['RES'],origin=[G['X0'],G['Y0']]),free_via_grid_points=places,
                   free_via_grid_points_checked=not quick,
                   components=[f for f in G['D']['fps'] if f['ref'] in c['components']],pads=[clean['pads'][i] for i in c['pads']],
                   previous_moves=copy.deepcopy(self.accepted_moves),
                   unit_list={u:{k:G['U'][u][k] for k in ('net','tracks','vias')} for u in sorted(set(c['unit_list'])|set(c.get('neighbors',[]))|{v['unit'] for v in c.get('vias',[])})},
                   move_format='List: category via/part/layer_name/relay; cur_state binds exactly this snapshot')
        (d/'cur_state.json').write_text(json.dumps(state,sort_keys=True,indent=1))
        try:
            with (d/'moves.json').open('x') as f:
                f.write(json.dumps(self.accepted_moves,sort_keys=True,indent=1)+'\n')
        except FileExistsError:
            pass
        self.emit(dict(event='unsolved',round_no=self.rounds,cur_state=digest,folder=str(d)))

    def trigger(self,pairs):
        count=len(pairs);self.stale=self.stale+1 if count==self.previous else 1;self.previous=count
        self.counts=(self.counts+[count])[-self.k:]
        cells={(p[0],round(p[4]/self.G['RES']),round(p[5]/self.G['RES'])) for p in pairs}
        self.cells={k:self.cells.get(k,0)+1 for k in cells}


        if self.large and (count>self.threshold or len(self.counts)<self.k or min(self.counts)<self.counts[0]):return False
        return bool(count and (self.stale>=self.k or max(self.cells.values(),default=0)>=self.n))

    def tick(self,round_number,pairs=None):
        if not self.explicit_budget:self.limit=self.budget(self.G,self.fraction)
        if self.disabled or self.used>=self.limit or self.trials>=self.cap or self.G['time_tight']():return False
        t=time.process_time();G=self.G;self.rounds=round_number;phases={};triggered=False
        def timed(name,start):phases[name]=phases.get(name,0.)+time.process_time()-start
        try:
            start=time.process_time()
            if pairs is None:
                G['exact_conflicts']();pairs=list(G['PAIRS'])
            p0=pairs
            if not self.trigger(p0):return False
            triggered=True;timed('observe',start)
            deadline=t+max(0,self.limit-self.used)
            clusters=[];skipped=None;start=time.process_time()
            for c in self.clusters(p0):
                neighbours=[u for u,r in G['U'].items() if self.movable(u) and G['_close_on'](r,c['points'],self.radius)]
                c['neighbors']=sorted(neighbours)
                if len(neighbours)>int(G['args'].get('loop_group','24'))+1:
                    self.emit(dict(event='cluster_skipped',round_no=round_number,reason='local group already too large before move',group_n=len(neighbours),bottleneck=c));skipped=skipped or c;continue
                clusters.append(c)
                if len(clusters)>=3:break
            timed('cluster',start)
            if not clusters:
                if skipped:self.export(skipped,self.digest(),quick=True)
                return False
            start=time.process_time();R0,p0=self.out();timed('outputs',start)
            start=time.process_time();E0=self.rate(G['D'],R0,G['NC'],v3=True,minimax=False);timed('initial_evaluation',start)
            if not math.isfinite(E0.get('score_v3',math.inf)):
                self.disabled='incomplete initial evaluation';self.emit(dict(event='blocked',reason=self.disabled));return False
            start=time.process_time();digest=self.digest();timed('state_hash',start)
            for c in clusters:
                candidates=[m for m in self.external if m['cur_state']==digest] if self.input_path else self.candidates(c,digest)
                iterator=iter(candidates)
                while time.process_time()<deadline and self.trials<self.cap:


                    start=time.process_time()
                    try:m=next(iterator)
                    except StopIteration:
                        timed('proposal_build',start);break
                    timed('proposal_build',start)
                    if time.process_time()>=deadline:break
                    if self.input_path and not self.external_local(m,c):continue
                    key=json.dumps(m,sort_keys=True)
                    if key in self.seen:continue
                    self.seen.add(key)
                    if m['cur_state']!=digest:
                        self.emit(dict(event='declined',reason='outdated state',move=m));continue
                    start=time.process_time();blocked=self.prefilter(m,c);timed('prefilter',start)
                    if blocked:
                        self.prefiltered[m['category']]=self.prefiltered.get(m['category'],0)+1
                        self.emit(dict(event='prefiltered',move=m,reason=blocked));continue
                    start=time.process_time();ok=self.trial(m,c,R0,p0,E0,min(deadline,time.process_time()+self.move_cpu));timed('move_records',start)
                    if ok:
                        self.stale=0;self.cells.clear();return True
                start=time.process_time();self.export(c,digest,quick=time.process_time()>=deadline or G['time_tight']());timed('interface',start)
            for m in self.external:
                if json.dumps(m,sort_keys=True) not in self.seen:
                    self.emit(dict(event='waiting',reason='State not reached yet or no local target in checked clusters/budget',move=m))
            return False
        finally:
            elapsed=time.process_time()-t;self.used+=elapsed
            self.emit(dict(event='loop_round',round_no=round_number,triggered=triggered,cpu_s=elapsed,cpu_phases_s=phases,budget_cpu_s=self.limit))


_RL = _RoutingLoop(globals()) if args.get('loop') == '1' or args.get('agent_loop') == '1' else None





ENDGAME = args.get('endgame', '1' if NC else '0') == '1'



WMIN_CONF = args.get('wmin_conf', '1' if NC else '0') == '1'
WMIN_UNTIL = int(args.get('wmin_conf_until', str(2 * MAXIT // 3)))
ALL_CLOSE = float(args.get('all_close', '0'))
CONVERGENCE_LOCAL = args.get('convergence', '') == 'local'
_local_best = [len(units), 0]
END_MAX = int(args.get('endgame_max', '8'))
_es = dict(n=0, before=None, level=0, ab=0, ran=False, cpu=0.0, off=False)
END_TIME = float(args.get('endgame_time', '1.0'))


def local_search_mask(pairs, open_, radius):
    m = np.zeros((NY, NX), dtype=bool)
    def region(x1, y1, x2, y2, border):
        xa = max(0, min(NX, int(math.floor((x1 - border - X0) / RES))))
        xb = max(0, min(NX, int(math.ceil((x2 + border - X0) / RES)) + 1))
        ya = max(0, min(NY, int(math.floor((y1 - border - Y0) / RES))))
        yb = max(0, min(NY, int(math.ceil((y2 + border - Y0) / RES)) + 1))
        m[ya:yb, xa:xb] = True
    for (_l, _d, _a, _b, x, y) in pairs: region(x, y, x, y, radius)
    for u in sorted(open_):
        ks = [int(u[4:])] if u.startswith('GND#') else by_net.get(u, [])
        boxes = [D['pads'][k]['bbox'] for k in ks]
        if not boxes: continue

        region(min(b[0] for b in boxes), min(b[1] for b in boxes),
               max(b[2] for b in boxes), max(b[3] for b in boxes),
               6.1 + radius if u.startswith('GND#') else (WINDOW or 3.0) + radius)
    return m


def _close_on(r, sites, R):
    for v in r['vias']:
        if any(math.hypot(v['x'] - x, v['y'] - y) <= R for x, y in sites): return True
    for t in r['tracks']:
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]; ll = dx * dx + dy * dy
            for x, y in sites:
                s = 0.0 if ll == 0 else min(1.0, max(0.0, ((x - a[0]) * dx + (y - a[1]) * dy) / ll))
                if math.hypot(a[0] + s * dx - x, a[1] + s * dy - y) <= R: return True
    return False


def endgame(conf, pairs, level, seed, repair=False):
    import copy
    R = (1.0, 2.0, 3.5, 5.0)[level % 4]
    sites = [(x, y) for (_l, _d, _a, _b, x, y) in pairs]
    core_set = sorted(u for u in conf if u in U and not u.startswith(('FIX#', 'TACK#')))
    group = list(core_set)
    for w, r in U.items():
        if w in core_set or w.startswith(('FIX#', 'TACK#')): continue
        if _close_on(r, sites, R): group.append(w)
    if PAIR_NETS: group = sorted(set(group) | {PAIR_NETS[w] for w in group if w in PAIR_NETS and PAIR_NETS[w] in U and PAIR_NETS[w] not in FIXED})
    if len(group) == len(core_set) or len(group) > int(args.get('endgame_group', '40')):
        print('   Endgame: radius %.1f mm, group %d with %d conflict nets (%s)' % (R, len(group), len(core_set), 'nobody in the way' if len(group) == len(core_set) else 'too large'), flush=True)
        return False
    save = {w: copy.deepcopy(U[w]) for w in group}; hsave = hist.copy(); PFs = PF[0]; soft = CONS_SOFT[0]; CONS_SOFT[0] = False
    source_free_before = SOURCE_FREE[0]


    X = groups_index(group)
    vacant = False


    for attempt in range(int(args.get('endgame_attempts', '3'))):
        _end_used = _es.get('work', 0) if BUDGET_WORK else _es['cpu']
        _end_total = work_total() if BUDGET_WORK else search_cpu() - observer_cpu()
        if _end_used > max(30.0, END_TIME * (_end_total - _end_used)) or time_tight():
            if BUDGET_WORK: print('   Endgame: work share used up (%d jobs)' % _end_used, flush=True)
            else: print('   Endgame: time share used up (%.0f s)' % _es['cpu'], flush=True)
            _es['off'] = True; break
        rng_ = budget_rng(1000 + 10 * seed + attempt); _tc = time.process_time()
        _end_work = work_total()
        _mark_cpu_before = observer_cpu()
        SOURCE_FREE[0] = repair or source_free_before
        HARD_FOREIGN[0] = repair and attempt == int(args.get('endgame_attempts', '3')) - 1
        for w in group:
            if w in U: del_unit(w)
        hist[:] = 0; PF[0] = min(1.0, PFMAX) if PFMAX is not None else 1.0
        if attempt == 0: order = core_set + sorted((w for w in group if w not in core_set), key=lambda w: (grp(w)[0], -(NC['net_list'].get(w) or {}).get('rank', 0), rng_.rand()) if NC and args.get('rank_all') == '1' else (grp(w)[0], rng_.rand()))
        else: order = sorted(group, key=lambda w: (grp(w)[0], -(NC['net_list'].get(w) or {}).get('rank', 0), rng_.rand()) if NC and args.get('rank_all') == '1' else (grp(w)[0] if attempt % 2 else 0, rng_.rand()))
        if PAIR_NETS:
            positions = {w:i for i,w in enumerate(order)}
            order.sort(key=lambda w: (grp(w)[0], -(NC['net_list'].get(w) or {}).get('pair_rank',0), min(w,PAIR_NETS[w]) if w in PAIR_NETS else '', w if w in PAIR_NETS else positions[w]))
        ok = True; history = []; _from = False
        for w in pair_series(order):
            if time_tight(): _from = True; break
            if not route_unit(w): ok = False
        for _k in range(int(args.get('endgame_iter', '14'))):
            if not ok or _from: break
            cfg = conflicts_group(group, X); history.append(len(cfg))
            if _ET: _ET.observe(cfg, it if 'it' in globals() else 1, 'endgame', geometry=False)
            if args.get('endgame_chk') == '1':
                _v = exact_conflicts(); assert set(_v) == set(cfg), ('Endgame-Checking', sorted(set(_v) ^ set(cfg)))
            if not cfg: vacant = True; break
            if len(history) >= 5 and min(history[-4:]) >= history[-5]: break
            for w, cells in cfg.items(): hist[np.array(cells, dtype=np.int64)] += 1.0
            PF[0] *= 1.5
            if PFMAX is not None: PF[0] = min(PF[0], PFMAX)
            for w in pair_series(sorted((w for w in order if w in cfg), key=lambda w: (grp(w)[0], -(NC['net_list'].get(w) or {}).get('rank', 0), rng_.rand()) if NC and args.get('rank_all') == '1' else rng_.rand())):
                if time_tight(): _from = True; break
                if not pair_unit(w): del_unit(w)
                if not route_unit(w): ok = False
        print('   Endgame: radius %.1f mm, group %d (of which GND stubs %d), ramp-up %d, conflicts per round %s%s' % (
            R, len(group), sum(1 for w in group if w.startswith('GND#')), attempt + 1, history, '' if ok else ', incomplete'), flush=True)
        HARD_FOREIGN[0] = False
        SOURCE_FREE[0] = source_free_before
        _es['cpu'] += time.process_time() - _tc - (observer_cpu() - _mark_cpu_before)
        if BUDGET_WORK: _es['work'] = _es.get('work', 0) + work_total() - _end_work
        if _from:
            print('   Endgame: Compute time budget used up, ramp-up aborted', flush=True); _es['off'] = True; break
        if vacant: break
    hist[:] = hsave; PF[0] = PFs; CONS_SOFT[0] = soft
    if vacant: return True
    for w in group:
        if w in U: del_unit(w)
    for w in group: add_unit(w, save[w]['net'], save[w]['tracks'], save[w]['vias'])
    return False









MOVE = args.get('move', '0') == '1'
MOVE_MAX = int(args.get('move_max', '80'))
MOVE_SITES = int(args.get('move_sites', '10'))
MOVE_GROUP = int(args.get('move_group', '25'))
MOVE_SHARE = float(args.get('move_share', '0.25'))
MOVE_ITER = int(args.get('move_iter', '10'))



MOVE_STRATEGY = (dict(R=2.0, series='core_set', warm=True), dict(R=3.5, series='core_set', warm=True), dict(R=1.0, series='core_set'),
             dict(R=2.0, series='core_set'), dict(R=2.0, series='chance'), dict(R=3.5, series='core_set'),
             dict(R=2.0, series='core_set', repair=True), dict(R=5.0, series='core_set'))
MOVE_WARM_PF = float(args.get('move_warm_pf', '50'))
_zs = dict(n=0, ok=0, cpu=0.0, work=0, quiet=0, best=None, log=None)

MOVE_POLICY = json.load(open(args['move_policy']))['table'] if args.get('move_policy') else None
def _move_pads2(x, y):
    return sum(1 for p in D['pads'] if abs(p['x'] - x) <= 2.0 and abs(p['y'] - y) <= 2.0)

def _move_series(site):
    if not MOVE_POLICY: return list(enumerate(MOVE_STRATEGY))
    k = len(site['core_set']); p = _move_pads2(site['x'], site['y'])
    layer_name = '%s|k%d|p%d' % (site['type_'], min(k, 3), 0 if p <= 8 else 1 if p <= 20 else 2)
    tab = MOVE_POLICY.get(layer_name) or MOVE_POLICY.get(site['type_']) or {}
    def amount(js):
        s = js[1]; e = tab.get('%s|%s|%d|%d' % (s['R'], s['series'], int(bool(s.get('repair'))), int(bool(s.get('warm')))))
        return (-(e['amount'] if e else -1.0), js[0])
    return sorted(enumerate(MOVE_STRATEGY), key=amount)


def _move_log(row):
    if _zs['log'] is None:
        _zs['log'] = open(args.get('move_log', OUT.replace('.json', '') + '_moves.jsonl'), 'w')
    _zs['log'].write(json.dumps(row, sort_keys=True) + '\n'); _zs['log'].flush()


def _move_sites(pairs, open_):
    st = []
    for (_l, _d, a, b, x, y) in sorted(pairs, key=lambda p: (round(p[4], 3), round(p[5], 3), p[2], p[3])):
        core_set = {w for w in (a, b) if w in U and not w.startswith(('FIX#', 'TACK#'))}
        if not core_set: continue
        for s in st:
            if s['type_'] == 'conflict' and math.hypot(s['x'] - x, s['y'] - y) <= 0.5:
                s['core_set'] |= core_set; break
        else: st.append(dict(type_='conflict', x=x, y=y, core_set=core_set))
    for u in sorted(open_):
        if u in FIXED or u.startswith(('FIX#', 'TACK#')): continue
        ks = [int(u[4:])] if u.startswith('GND#') else by_net.get(u, [])
        seen = []
        for k in ks:
            p = D['pads'][k]
            if any(math.hypot(p['x'] - x, p['y'] - y) <= 1.0 for x, y in seen): continue
            seen.append((p['x'], p['y']))
            st.append(dict(type_='open_items', x=p['x'], y=p['y'], core_set={u}))
            if len(seen) >= 4: break
    return st


def _move_spacing(r, p):
    x, y = p; d = float('inf')
    for v in r['vias']: d = min(d, math.hypot(v['x'] - x, v['y'] - y))
    for t in r['tracks']:
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]; ll = dx * dx + dy * dy
            s_ = 0.0 if ll == 0 else min(1.0, max(0.0, ((x - a[0]) * dx + (y - a[1]) * dy) / ll))
            d = min(d, math.hypot(a[0] + s_ * dx - x, a[1] + s_ * dy - y))
    return d


MOVE_LOCAL = args.get('move_local', '1') == '1'
MOVE_CHK = args.get('move_chk', '0') == '1'


_MOVE_BORDER = _DMAX + max(DRC_HOLE, VIA_DR / 2 + VIA_R + HOLE_CLR, VIA_DR / 2 + HOLE_CLR, VIA_DR / 2 + H2H) + VIA_R + 0.05


def conflicts_local(group):
    g = set(group); b = None
    for w in sorted(g):
        if w not in U: continue
        for _li, k, _kb, _ne in _unit_geo(w, U[w]):
            x0, y0, x1, y1 = k.bounds
            b = [x0, y0, x1, y1] if b is None else [min(b[0], x0), min(b[1], y0), max(b[2], x1), max(b[3], y1)]
    if b is None: return {}
    F = (b[0] - _MOVE_BORDER, b[1] - _MOVE_BORDER, b[2] + _MOVE_BORDER, b[3] + _MOVE_BORDER)
    sv = (PAIRS[:], PAIRS_RAW[:], FIX_PAIRS[:])
    def selection(conf):
        pairs = [p for p in PAIRS if p[2] in g or p[3] in g]
        with_ = {u for p in pairs for u in p[2:4]}
        return {u: cs for u, cs in conf.items() if u in with_}, sorted(pairs)
    res, pairs = selection(exact_conflicts(window=F))
    if MOVE_CHK:
        resv, pairs_v = selection(exact_conflicts())
        equal = (sorted(res) == sorted(resv) and pairs == pairs_v
                  and all(sorted(res[u]) == sorted(resv[u]) for u in res if u in g))
        _zs['chk'] = _zs.get('chk', 0) + 1
        if not equal:
            raise AssertionError('Move window check deviates: %s / %s' % (sorted(set(res) ^ set(resv)), sorted(set(pairs) ^ set(pairs_v))[:6]))
    PAIRS[:], PAIRS_RAW[:], FIX_PAIRS[:] = sv
    return res


def _move_conf(group, X):
    return conflicts_local(group) if MOVE_LOCAL else conflicts_group(group, X)


def _move_value(group, X):
    cfg = _move_conf(group, X)
    return len(cfg) + 2 * sum(1 for w in group if w in fail), cfg


def move_attempt(site, strategy, seed):
    """Try one repair move at a spot with one strategy.

    The move rips up the units at the spot (its core nets, plus neighbours within radius R unless the strategy is 'warm')
    and reroutes this group for a few local iterations, ordered by the strategy. The move is kept if the spot is better
    afterwards (fewer conflicts or fewer unrouted units in the group), otherwise the previous state is restored. Every
    move is logged with its features to <OUT>_move_records.jsonl, the training data for learning/move_choice.py.
    Returns True if the move was accepted.
    """
    import copy
    R = strategy['R']; xy = [(site['x'], site['y'])]
    core_set = sorted(site['core_set'])
    close = [w for w, r in sorted(U.items()) if w not in site['core_set'] and not w.startswith(('FIX#', 'TACK#'))
           and w not in FIXED and _close_on(r, xy, R)]
    capped = len(core_set) + len(close) > MOVE_GROUP
    if capped:
        close = sorted(close, key=lambda w: (_move_spacing(U[w], xy[0]), w))[:max(0, MOVE_GROUP - len(core_set))]
    group = list(core_set) + close
    if PAIR_NETS: group = sorted(set(group) | {PAIR_NETS[w] for w in group if w in PAIR_NETS and PAIR_NETS[w] in units and PAIR_NETS[w] not in FIXED})
    row = dict(move=_zs['n'], type_=site['type_'], x=round(site['x'], 3), y=round(site['y'], 3), R=R, series=strategy['series'], warm=bool(strategy.get('warm')),
                 repair=bool(strategy.get('repair')), core_set=len(core_set), group=len(group),
                 gnd=sum(1 for w in group if w.startswith('GND#')), pair=sum(1 for w in group if w in PAIR_NETS),
                 kinds=sorted({(NC['net_list'].get(w) or {}).get('category', 'signal') for w in core_set}) if NC else [],
                 pads=sum(1 for p in D['pads'] if abs(p['x'] - site['x']) <= R and abs(p['y'] - site['y']) <= R),
                 pads2=_move_pads2(site['x'], site['y']), policy=bool(MOVE_POLICY),
                 conflicts_ges=len(PAIRS), open_ges=len(fail), cu_layers=NL, capped=capped)
    if len(group) > MOVE_GROUP:
        row.update(outcome='group_too_large'); _move_log(row); return False
    _tc = time.process_time(); _ta = work_total() if BUDGET_WORK else 0
    X = None if MOVE_LOCAL else groups_index(group)
    before, _ = _move_value(group, X)
    save = {w: copy.deepcopy(U[w]) for w in group if w in U}; fail_before = {w for w in group if w in fail}
    hsave = hist.copy(); PFs = PF[0]; soft = CONS_SOFT[0]; CONS_SOFT[0] = False; qf = SOURCE_FREE[0]
    SOURCE_FREE[0] = bool(strategy.get('repair')) or qf; HARD_FOREIGN[0] = False
    rng_ = budget_rng(5000 + seed)
    other = [w for w in group if w not in site['core_set']]
    if strategy.get('warm'):
        for w in core_set:
            if w in U: del_unit(w)
        PF[0] = min(PFs, MOVE_WARM_PF)
        order = list(core_set)
    else:
        for w in group:
            if w in U: del_unit(w)
        hist[:] = 0; PF[0] = min(1.0, PFMAX) if PFMAX is not None else 1.0
        if strategy['series'] == 'core_set': order = core_set + sorted(other, key=lambda w: (grp(w)[0], rng_.rand()))
        else: order = sorted(group, key=lambda w: rng_.rand())
    ok = {}; history = []
    for w in pair_series(order): ok[w] = route_unit(w)
    for _k in range(MOVE_ITER):
        cfg = _move_conf(group, X); history.append(len(cfg))
        if not cfg or time_tight(0.95): break
        if len(history) >= 4 and min(history[-3:]) >= history[-4]: break
        if strategy.get('warm') and len(history) >= 3 and min(history[-2:]) >= min(history[:-2]): break
        for w, cells in cfg.items(): hist[np.array(cells, dtype=np.int64)] += 1.0
        PF[0] *= 1.5
        if PFMAX is not None: PF[0] = min(PF[0], PFMAX)
        for w in pair_series(sorted((w for w in group if w in cfg or not ok.get(w, True)), key=lambda w: rng_.rand())):
            if not pair_unit(w): del_unit(w)
            ok[w] = route_unit(w)
    for w in group:
        if ok.get(w, w in U): fail.discard(w)
        else: fail.add(w)
    post, _ = _move_value(group, X)
    hist[:] = hsave; PF[0] = PFs; CONS_SOFT[0] = soft; SOURCE_FREE[0] = qf
    taken = post < before
    if not taken:
        for w in group:
            if w in U: del_unit(w)
        for w, r in sorted(save.items()): add_unit(w, r['net'], r['tracks'], r['vias'])
        for w in group:
            if w in fail_before: fail.add(w)
            else: fail.discard(w)
    cpu = time.process_time() - _tc; _zs['cpu'] += cpu
    if BUDGET_WORK: _zs['work'] += work_total() - _ta
    row.update(outcome='taken' if taken else 'rejected', before=before, after=post, history=history, cpu_s=round(cpu, 3))
    _move_log(row)
    return taken


def move_remember():
    import copy
    amount = len(exact_conflicts()) + len(fail)
    if _zs.get('state') is None or amount < _zs['state'][0]:
        _zs['state'] = (amount, {w: copy.deepcopy(r) for w, r in U.items() if not w.startswith(('FIX#', 'TACK#'))}, set(fail))


def move_best():
    global conf
    if _zs.get('state') is None: return False
    now = len(exact_conflicts()) + len(fail)
    if _zs['state'][0] >= now: return False
    _, prior, prev_fail = _zs['state']
    for w in sorted(set(U) | set(prior)):
        if w.startswith(('FIX#', 'TACK#')): continue
        if w in U: del_unit(w)
    for w, r in sorted(prior.items()): add_unit(w, r['net'], r['tracks'], r['vias'])
    fail.clear(); fail.update(prev_fail)
    conf = exact_conflicts()
    print('   Move loop: best state restored (%d instead of %d conflicts+open)' % (_zs['state'][0], now), flush=True)
    return True


def move_budget_off():
    if _zs['n'] >= MOVE_MAX or time_tight(0.95): return True
    if BUDGET_WORK: return _zs['work'] >= MOVE_SHARE * WORK_MAX
    return CPU_MAX > 0 and _zs['cpu'] >= MOVE_SHARE * CPU_MAX


def move_loop(phase, seed):
    """One pass of repair moves over all remaining problem spots (enabled with move=1).

    A spot is one conflict or one unrouted unit. For each spot the strategies are tried in order (fixed order, or the
    learned order from move_policy=<file>, see learning/move_choice.py) until one move is accepted. Each strategy is
    tried at most twice per spot. The loop only runs when conflicts plus unrouted units are at most move_sites, and
    stops when the move budget (move_max, move_share of the CPU budget) is used up.
    Returns the number of accepted moves in this pass.
    """
    global conf
    conf = exact_conflicts()
    if not (0 < len(conf) + len(fail) <= MOVE_SITES): return 0
    good = 0
    for i, st in enumerate(_move_sites(list(PAIRS), set(fail))):
        if not (st['core_set'] & (set(conf) | fail)): continue
        for j, strategy in _move_series(st):
            if move_budget_off(): break
            tabu = (round(st['x'], 1), round(st['y'], 1), j)
            if _zs.setdefault('tabu', {}).get(tabu, 0) >= 2: continue
            _zs['tabu'][tabu] = _zs['tabu'].get(tabu, 0) + 1
            _zs['n'] += 1
            if move_attempt(st, strategy, 100 * seed + 10 * i + j):
                good += 1; _zs['ok'] += 1; conf = exact_conflicts(); break
    print('   Move loop (%s): %d moves total, %d accepted in this pass, now %d conflicts, %d open (%.0f CPU-s)' % (
        phase, _zs['n'], good, len(conf), len(fail), _zs['cpu']), flush=True)
    return good


if MAXIT < 2 and todo is None:
    conf = exact_conflicts(); todo = (set(conf) | fail) - set(FIXED)
VIA_ANCHOR_ON[0] = True
_AL = None
if args.get('agent_loop') == '1':
    from codex.agent_loop.cur_state import AgentLoop
    _AL = AgentLoop(globals(), _AGENT_SAVED)
for it in range(_AGENT_SAVED['round_no'] if _AGENT_SAVED else 2, MAXIT + 1):
    CONS_SOFT[0] = it <= int(args.get('cons_iter', '10'))

    conf = exact_conflicts(); _es['ran'] = True
    if (_AL.before_round(it, list(PAIRS)) if _AL else (_RL and _RL.tick(it - 1, list(PAIRS)))):
        print('   Routing loop: local move accepted', flush=True)
        conf = exact_conflicts()
    negotiation_measure(it - 1, conf)
    _mark_conf = _ET.conflict_copy(conf) if _ET else None
    if VIA_PRECEDENCE and it <= WMIN_UNTIL:
        conf, _nvr = precedence_filter(conf)
        if _nvr: print('   Via priority (R25): %d spots where only the signal yields' % _nvr, flush=True)
    if WMIN_CONF and it <= WMIN_UNTIL:
        _tight = tight_conflicts()
        for _u, _c in _tight.items(): conf.setdefault(_u, []).extend(_c)
        if _ET: _ET.complete(_mark_conf, _tight)
        if _tight: print('   too narrow (wmin_conf): %d units involved' % len(_tight), flush=True)
    if _ET and not _LEARN_ON: _ET.observe(_mark_conf, it - 1)
    todo = (set(conf) | fail) - set(FIXED)
    print('Iteration %d: conflicts %d, incomplete %d, PF %.2f (%.0fs)' % (it, len(conf), len(fail), PF[0], time.time() - t0), flush=True)
    work_mark('round_%d' % it)
    if it % 4 == 0:
        import collections
        print('   Pairs', len(PAIRS), collections.Counter(tuple(sorted((x[2].split('#')[0][:12], x[3].split('#')[0][:12]))) for x in PAIRS).most_common(12), flush=True)
        print('   Examples', sorted(PAIRS, key=lambda x: x[1])[:6], flush=True)
    if not todo: break
    if time_tight():
        print('%s 80 %% used up: negotiation ends with %d conflicts' % (budget_message(), len(conf)), flush=True); break
    if MOVE and it >= 8:
        _zsum = len(conf) + len(fail)
        if _zs['best'] is None or _zsum < _zs['best']: _zs['best'] = _zsum; _zs['quiet'] = 0
        else: _zs['quiet'] += 1
        if _zs['quiet'] >= 3 and 0 < _zsum <= MOVE_SITES and not move_budget_off():
            _zs['quiet'] = 0; _z_good = 0
            for _zr in range(4):
                if not move_loop('Negotiation', 10 * it + _zr): break
                _z_good += 1; move_remember()
                if not conf and not fail: break
            if _z_good:
                _zs['best'] = len(conf) + len(fail); continue
    if ENDGAME and not fail and len(conf) <= END_MAX and it >= 6:
        if _es.get('ok_before') is not None:
            if set(conf) == _es['ok_before']:
                _es['apparent'] = _es.get('apparent', 0) + 1; _es['level'] += 1; _es['ab'] = it + 3
                print('   Endgame: apparent success %d (same conflict back again)' % _es['apparent'], flush=True)
                if _es['apparent'] >= 2: _es['off'] = True; print('   Endgame: off for this run', flush=True)
            _es['ok_before'] = None
        _es['n'] = _es['n'] + 1 if set(conf) == _es['before'] else 1; _es['before'] = set(conf)
        if _es['n'] >= 4 and it >= _es['ab'] and not _es['off']:
            _ok = endgame(conf, list(PAIRS), _es['level'], it)
            print('   Endgame (ramp-up round %d): %s' % (_es['level'] + 1, 'conflict_free' if _ok else 'without success, old state'), flush=True)
            if _ok: _es['ok_before'] = set(conf); continue
            _es['level'] += 1; _es['ab'] = it + 3
    else:
        _es['n'] = 0; _es['before'] = None
    for u, b in conf.items(): hist[np.array(b, dtype=np.int64)] += HINC
    PF[0] *= float(args.get('pfmul', '1.5'))
    if PFMAX is not None: PF[0] = min(PF[0], PFMAX)
    fail_prev = fail; fail = set(); _next = set()
    all_items = args.get('all_items', '1') == '1' and len(conf) > int(args.get('goal_from', '0'))
    close = ALL_CLOSE
    if CONVERGENCE_LOCAL:
        if len(conf) < _local_best[0]: _local_best[:] = [len(conf), 0]
        else: _local_best[1] += 1
        all_items = it % 4 == 0
        close = ALL_CLOSE or min(6.0, 1.0 + _local_best[1] / 4)
    rng = budget_rng(it)
    def gkey(u):
        if PAIR_NETS:


            p = PAIR_NETS.get(u)
            if p: return (grp(u)[0], -NC['net_list'][u].get('pair_rank', 0), min(u, p), u)
            rank = (NC['net_list'].get(u) or {}).get('rank', 0) if args.get('rank_all', '0') == '1' else 0
            if NC:
                rank *= float(args.get('cost_rank_' + (NC['net_list'].get(u) or {}).get('category', 'signal'), '1'))
            return (grp(u)[0], -rank, '', learn_series_value(u,rng.rand()))
        if NC and args.get('rank_all', '0') == '1':
            _net_copper = NC['net_list'].get(u) or {}
            return (grp(u)[0], -_net_copper.get('rank', 0) * float(args.get('cost_rank_' + _net_copper.get('category', 'signal'), '1')), learn_series_value(u,rng.rand()))
        return (grp(u)[0], learn_series_value(u,rng.rand()))


    if all_items and close > 0:


        if CONVERGENCE_LOCAL:
            _m = local_search_mask(PAIRS, fail_prev, close)
        else:
            _m = np.zeros((NY, NX), dtype=bool); _rr = int(close / RES)
            for (_l, _d, _a, _b, _x, _y) in PAIRS:
                _ix, _iy = int((_x - X0) / RES), int((_y - Y0) / RES)
                _m[max(0, _iy - _rr):_iy + _rr + 1, max(0, _ix - _rr):_ix + _rr + 1] = True
        _mf = _m.ravel()
        series = [u for u in units if u in todo or (u in U and u not in FIXED and len(U[u]['cells']) and bool(_mf[U[u]['cells'] % NN].any()))]
        print('   Radius %.1f mm: %d of %d units new' % (close, len(series), len(units)), flush=True)
    else:
        series = units if all_items else sorted(todo)
    if PAIR_NETS:
        series = sorted(set(series) | {PAIR_NETS[u] for u in series if u in PAIR_NETS and PAIR_NETS[u] in units})
    _negotiation_series = pair_series(sorted(series, key=gkey))
    if _AL:
        from codex.agent_loop.moves import order as _agent_order
        _negotiation_series = _agent_order(_negotiation_series, _AGENT_PRIORITY)
    for u in _negotiation_series:
        if time_tight():
            fail |= fail_prev - _next; break
        if pair_unit(u):
            _next.update((u,PAIR_NETS[u]))
            if route_unit(u): fail.discard(u); fail.discard(PAIR_NETS[u])
            else: fail.update((u,PAIR_NETS[u]))
        else:
            del_unit(u); _next.add(u)
            if not route_unit(u): fail.add(u)
        if u in PAIR_NETS and u > PAIR_NETS[u] and u not in fail and PAIR_NETS[u] not in fail:
            pair_couple(only={u, PAIR_NETS[u]})
if ENDGAME and _es['ran'] and not fail:
    conf = exact_conflicts()
    for _st in range(_es['level'], _es['level'] + 4):
        if not (0 < len(conf) <= END_MAX) or _es['off']: break
        _ok = endgame(conf, list(PAIRS), _st, 100 + _st)
        print('Endgame after the load pass (ramp-up round %d): %s' % (_st + 1, 'conflict_free' if _ok else 'without success'), flush=True)
        conf = exact_conflicts()
    todo = (set(conf) | fail) - set(FIXED)
elif _es['ran']:
    conf = exact_conflicts(); todo = (set(conf) | fail) - set(FIXED)
if ENDGAME and NC and args.get('endgame_repair', '1') == '1' and not fail and not time_tight():



    conf = exact_conflicts()
    if 0 < len(conf) <= END_MAX:
        _es['off'] = False; _es_prev_cpu = _es['cpu']; _es['cpu'] = 0.0
        for _st in range(4):
            if time_tight(): break

            _ok = endgame(conf, list(PAIRS), _st, 24 + _st, repair=True)
            print('Endgame repair: radius %.1f mm, %s' % ((1.0, 2.0, 3.5, 5.0)[_st], 'conflict_free' if _ok else 'old state'), flush=True)
            conf = exact_conflicts()
            if not conf or _es['off']: break
        _es['cpu'] += _es_prev_cpu
        todo = (set(conf) | fail) - set(FIXED)
if MOVE and _es['ran']:
    move_best()
    for _zr in range(4):
        if move_budget_off() or not move_loop('End %d' % (_zr + 1), 900 + _zr): break
    conf = exact_conflicts(); todo = (set(conf) | fail) - set(FIXED)
VIA_ANCHOR_ON[0] = False
CONS_SOFT[0] = False

_cf_checkpoint = exact_conflicts(); PAIRS_LAST = list(PAIRS)
work_mark('before_post_run')
negotiation_measure(it if 'it' in globals() else 1, _cf_checkpoint, 'before_post_run')
if _ET and not _LEARN_ON: _ET.observe(_cf_checkpoint, it if 'it' in globals() else 1)
json.dump(PAIRS_LAST, open(OUT.replace('.json', '_conflicts_before_widen.json'), 'w'))
if ((BGA_PLAN and args.get('bga_budget', '0') == '1') or args.get('budget_before_post_run', '0') == '1' or globals().get('_EARLY_INFO') is not None) and time_tight() and (_cf_checkpoint or fail):

    _ct, _cv = export_geometry()
    _cc = {u:len(cs) for u,cs in _cf_checkpoint.items()}
    _safe = export_data(_ct, _cv, _cc,
        final=(_LEARN_STOP or 'cpu_budget') if _LEARN_ON else 'budget_before_post_run',
        **({'learning_state': json.loads(_LEARN_SNAPSHOT)['learning_state'], 'learning_budget':json.loads(_LEARN_SNAPSHOT)['learning_budget']} if _LEARN_ON else {}))
    json.dump(_safe, open(OUT,'w'))
    json.dump(PAIRS_LAST,open(OUT.replace('.json','_conflicts.json'),'w'))
    print('Compute time budget: invalid state saved, width/shape post-pass skipped',flush=True)
    print('End: conflicts',_cc,'incomplete',sorted(fail),flush=True)
    print('Tracks',len(_ct),'Vias',len(_cv),'Time %.0fs' % (time.time()-t_start),flush=True)
    if _ET: _ET.write_out(_safe['tracks'], _safe['vias'], absent=_safe['failed'], zone_list=_safe.get('zone_list', []) + _safe.get('zones', []))
    sys.exit(0)
import copy
def unit_len(u):
    r = U[u]
    if CONS and any(v.get('gr') for v in r['vias']):
        return sum(math.dist(a, c) for t in r['tracks'] if not t.get('web') for a, c in zip(t['pts'][:-1], t['pts'][1:])) + 0.6 * n_sv(r)
    return sum(math.dist(a, c) for t in r['tracks'] for a, c in zip(t['pts'][:-1], t['pts'][1:])) + 0.6 * n_sv(r)
def grp_key(u): return grp(u)[0]


WT_KIND = None
if WEIGHTED:
    try:
        import os as _os
        sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
        from common.importance import weights as _weights, swap_better as _swap_better
        WT_KIND = _weights()
    except Exception as _e:
        print('weighted=%d: weights from analysis/evaluation.py not loadable (%r), staying with bare length' % (WEIGHTED, _e), flush=True)
        WEIGHTED = 0
def u_wt(u):
    if WT_KIND is None or u not in U: return 1.0
    return WT_KIND.get(NC['net_list'].get(U[u]['net'], {}).get('category', 'signal'), 1.0)
def u_vias(u):
    r = U[u]
    return n_sv(r)
def unit_len_wt(u):
    nv = u_vias(u)
    return u_wt(u) * (unit_len(u) - 0.6 * nv) + 0.6 * nv
def current_resistance(u, reachable=False):
    r = U[u]; net = r['net']; C = cls(net)
    k = (NC['net_list'].get(net) or {}) if NC else {}
    from common.flow import analysis as fa, goal_width as fz
    info = fa(r['tracks'], r['vias'], [D['pads'][i] for i in by_net_g.get(net, [])], k.get('flow') or {})
    share = info['share'] if info and info['total'] > 0 else None
    cap = None
    if reachable and r['tracks']:
        xy = [q for t in r['tracks'] for q in t['pts']]
        box = (min(q[0] for q in xy) - 1, min(q[1] for q in xy) - 1, max(q[0] for q in xy) + 1, max(q[1] for q in xy) + 1)
        cap = maps(net, box, C['w'] * max(f_inner(net, LI[t['layer']]) for t in r['tracks']))[6]
    out = 0.0
    for ti, t in enumerate(r['tracks']):
        if t.get('sig') or t.get('web'): continue
        li = LI[t['layer']]; thick = 0.035 if t['layer'] in ('F', 'B') else 0.0175
        for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
            f = share.get((ti, si), 0.0) if share is not None else 1.0
            if cap is not None:
                goal = fz(f, C['wmin'], C['w'], float(args.get('flow_exp', '0.5'))) * f_inner(net, li)
                w = max(w, math.floor(min(goal, line_cap_min(cap[li], a, b), 2 * pad_room(li, a, b, net)) * 40) / 40)
            out += f * f * 1.72e-5 * math.dist(a, b) / (w * thick)
    out += 0.0024 * sum(1 for v in r['vias'] if not v.get('gr') and not v.get('rv'))
    return out


def routing_mass(u):
    if NC and args.get('current_short') == '1' and (NC['net_list'].get(U[u]['net']) or {}).get('category') in ('current_val', 'supply', 'sw'):

        return current_resistance(u, reachable=True) * cls(U[u]['net'])['w'] * 0.035 / 1.72e-5
    return unit_len(u)


def routing_mass_wt(u):


    if NC and args.get('current_short') == '1' and (NC['net_list'].get(U[u]['net']) or {}).get('category') in ('current_val', 'supply', 'sw'):
        return u_wt(u) * routing_mass(u)
    return unit_len_wt(u)


def short_key(u): return -unit_len(u) * (u_wt(u) if WEIGHTED >= 2 else 1.0)


R19_SHORT = args.get('r19_short', '0') == '1' and NC is not None
R19_SHORT_STAT = dict(attempts=0, more_vias_rejected=0, extra_vias_rejected=0)
def r19_short_ok(prior, new):
    return n_sv(new) <= n_sv(prior)

if args.get('short') == '1' and not todo and not time_tight():
    t2 = time.time(); hist[:] = 0; PFs = PF[0]; PF[0] = 1e3
    _CAND_IDX[0] = None
    for rnd in range(int(args.get('short_rounds', '2'))):
        better = 0; gain = 0.0
        for u in sorted([u for u in units if not u.startswith('GND#') and u in U and not pair_unit(u)], key=short_key):
            if time_tight(0.9): break
            old = copy.deepcopy(U[u]); L0 = routing_mass(u); _f0 = cons_missing(u) if CONS else 0
            del_unit(u)
            ok = route_unit(u)
            if ok and routing_mass(u) < L0 - 0.05 and (not CONS or cons_missing(u) <= _f0):
                if R19_SHORT and (NC['net_list'].get(old['net']) or {}).get('category') in ('signal', 'analog', 'fast'):
                    R19_SHORT_STAT['attempts'] += 1
                    if not r19_short_ok(old, U[u]):
                        R19_SHORT_STAT['more_vias_rejected'] += 1
                        R19_SHORT_STAT['extra_vias_rejected'] += n_sv(U[u]) - n_sv(old)
                        del_unit(u); add_unit(u, old['net'], old['tracks'], old['vias']); cand_idx_back(u); continue
                if unit_conflict_free(u): better += 1; gain += L0 - routing_mass(u); cand_idx_changed(u); continue
            del_unit(u); add_unit(u, old['net'], old['tracks'], old['vias']); cand_idx_back(u)
        print('Shortening round %d: %d nets shorter, %.1f mm together (%.0fs)' % (rnd + 1, better, gain, time.time() - t2), flush=True)
        if not better: break
    PF[0] = PFs
    if R19_SHORT: print('R19 shortening:', R19_SHORT_STAT, flush=True)



SHORT_VIA = float(args.get('short_via', '8' if NC and 'start' not in args else '0'))
if SHORT_VIA > 0 and NC is not None and not todo and not time_tight():
    t2 = time.time(); hist[:] = 0; PFs = PF[0]; PF[0] = 1e3; _CAND_IDX[0] = None; VIA_FACTOR[0] = SHORT_VIA
    _mm = float(args.get('short_via_mm', '3.0')); _saved = 0; _more = 0.0
    def _via_candidate(u):
        if pair_unit(u) or u.startswith(('GND#', 'FIX#', 'TACK#')) or u not in U or not U[u]['vias']: return False
        if NC['net_list'].get(U[u]['net'], {}).get('category') in ('current_val', 'supply', 'sw', 'gnd'): return False
        return not any(v.get('n') or v.get('gr') for v in U[u]['vias'])



    def _via_wt(u): return FAST_VIA if FAST_VIA != 1 and actual_fast(U[u]['net']) else 1.0
    def _via_key(u):
        if WEIGHTED >= 2: return (-n_sv(U[u]) * _via_wt(u) * u_wt(u), u)
        return (-n_sv(U[u]) * _via_wt(u), u)
    for rnd in range(int(args.get('short_via_rounds', '2'))):
        _n = 0
        for u in sorted([u for u in units if _via_candidate(u)], key=_via_key):
            if time_tight(0.9): break
            old = copy.deepcopy(U[u]); _v0 = n_sv(old); L0 = unit_len(u) - 0.6 * _v0; _cons_ = _via_wt(u)
            del_unit(u)
            ok = route_unit(u)
            _v1 = n_sv(U[u]); L1 = unit_len(u) - 0.6 * _v1
            _allowed = _mm * (2.0 if NC['net_list'].get(old['net'], {}).get('category', 'signal') == 'signal' else 1.0) * _cons_
            if ok and _v1 < _v0 and L1 <= L0 + _allowed * (_v0 - _v1) and unit_conflict_free(u):
                _n += 1; _saved += _v0 - _v1; _more += L1 - L0; cand_idx_changed(u); continue
            del_unit(u); add_unit(u, old['net'], old['tracks'], old['vias']); cand_idx_back(u)
        if not _n: break
    VIA_FACTOR[0] = 1.0; PF[0] = PFs
    print('Via round (via price x%g): %d vias saved, tracks %.1f mm longer for it (%.0fs)' % (SHORT_VIA, _saved, _more, time.time() - t2), flush=True)





if (VAP or EPM) and args.get('via_at_pad_clear', '0' if 'start' in args else '1') == '1' and not todo and not time_tight():
    t2 = time.time(); hist[:] = 0; PFs = PF[0]; PF[0] = 1e3; _nr = 0; _nv = 0; _more = 0.0; _nep = 0; _wt = 0.0
    _detour = float(args.get('via_at_pad_detour', '2.0')); _gmin = float(args.get('via_at_pad_gain', '0.6'))
    def _quality(u):
        r = U[u]; _p = D['pads'][int(u[4:])]; _v = r['vias'][0]; _c, _w = _vap_cells(int(u[4:]))
        _j = np.flatnonzero(_c == int((_v['y'] - Y0) / RES) * NX + int((_v['x'] - X0) / RES))
        return (sum(math.dist(a, c) for t in r['tracks'] for a, c in zip(t['pts'][:-1], t['pts'][1:]))
                + 2.0 * math.hypot(_v['x'] - _p['x'], _v['y'] - _p['y']) + VAP_K * (float(_w[_j[0]]) if len(_j) else 1.0))
    for u in [u for u in units if u.startswith('GND#') and u in U and U[u]['vias']]:
        if time_tight(0.9): break
        _k = int(u[4:]); _n0 = len(U[u]['vias'])
        if _k in EPM:
            if _n0 >= EPM[_k]['nominal']: continue
        elif not VAP: continue
        else: _g0 = _quality(u)
        save = {u: copy.deepcopy(U[u])}
        del_unit(u)
        if _k in EPM:
            CONS_SOFT[0] = True; ok = route_unit(u); CONS_SOFT[0] = False
            ok = ok and len(U[u]['vias']) > _n0; _g1 = 0.0; _g0 = 0.0
        else:
            PF[0] = 0.01; ok = route_unit(u); PF[0] = 1e3
            if LAYER_STACK: ok = ok and bool(U[u]['vias'])
            if ok: _g1 = _quality(u); ok = _g1 < _g0 - _gmin
        if ok:
            _nv += 1; _pp = D['pads'][_k]
            _cf = conflicts_group([u], groups_index([u])); in_way = sorted(w for w in _cf if w != u)
            if not _cf:
                _nr += 1; _wt += _g0 - _g1; _nep += len(U[u]['vias']) - _n0 if _k in EPM else 0; continue
            ok = 0 < len(in_way) <= 4 and not any(w.startswith(('GND#', 'FIX#', 'TACK#')) for w in in_way)
            _reason = 'accepted' if ok else ('foreign pad in the way' if not in_way else 'other stubs, fixed units or more than four in the way')
            if ok:
                if PAIR_SEARCH_SPACE: in_way=sorted(set(in_way)|{PAIR_NETS[w] for w in in_way if pair_unit(w)})
                L0 = sum(unit_len(w) - 0.6 * n_sv(U[w]) for w in in_way); _v0 = sum(n_sv(U[w]) for w in in_way)
                _f0 = sum(cons_missing(w) for w in in_way) if CONS else 0
                for w in in_way: save[w]=copy.deepcopy(U[w])
                for w in pair_series(in_way):
                    if not pair_unit(w): del_unit(w)
                    if not route_unit(w): ok = False
                group = [u] + in_way
                if not ok: _reason = 'no way for the units in the way'
                else:
                    L1 = sum(unit_len(w) - 0.6 * n_sv(U[w]) for w in in_way); _dv = sum(n_sv(U[w]) for w in in_way) - _v0
                    if L1 > L0 + _detour * (VAP_WF if (_k in VAP_IMPORTANT or _k in EPM) else 1.0): ok = False; _reason = 'Detour %.1f mm' % (L1 - L0)
                    elif _dv > 0: ok = False; _reason = '%d more vias' % _dv
                    elif CONS and sum(cons_missing(w) for w in in_way) > _f0: ok = False; _reason = 'Via group loses a via'
                    elif conflicts_group(group, groups_index(group)): ok = False; _reason = 'Conflict remains'
            print('   %s:%s %s, in the way %s: %s' % (_pp['ref'], _pp['num'], 'Ground pad %d -> %d vias' % (_n0, len(U[u]['vias'])) if _k in EPM else 'Stub cost %.1f -> %.1f' % (_g0, _g1),
                                               in_way[:5], _reason), flush=True)
            if ok:
                _nr += 1; _more += L1 - L0; _wt += _g0 - _g1; _nep += len(U[u]['vias']) - _n0 if _k in EPM else 0; continue
        for w in save:
            if w in U: del_unit(w)
        for w in save: add_unit(w, save[w]['net'], save[w]['tracks'], save[w]['vias'])
    PF[0] = PFs; _CAND_IDX[0] = None
    print('Room at pad cleared: %d of %d stubs and ground pads improved with better room (stub cost together -%.1f), %d more ground pad vias, other tracks %.1f mm longer for it (%.0fs)' % (
        _nr, _nv, _wt, _nep, _more, time.time() - t2), flush=True)

def net_mst(n):
    pts = [(D['pads'][k]['x'], D['pads'][k]['y']) for k in by_net[n]]
    inT = [pts[0]]; rest = pts[1:]; L = 0
    while rest:
        d, i = min((min(math.dist(r, t) for t in inT), i) for i, r in enumerate(rest)); L += d; inT.append(rest.pop(i))
    return L
def unit_box(u):
    xs, ys = [], []
    for t in U[u]['tracks']:
        for q in t['pts']: xs.append(q[0]); ys.append(q[1])
    for v in U[u]['vias']: xs.append(v['x']); ys.append(v['y'])
    return (min(xs), min(ys), max(xs), max(ys)) if xs else None
if args.get('relayed') == '1' and not todo and not time_tight():
    t3 = time.time(); hist[:] = 0; PFs = PF[0]; PF[0] = 1e3; _ugw = [0, 0]
    for rnd in range(int(args.get('relayed_rounds', '3'))):
        cand = []
        for u in units:
            if pair_unit(u) or u.startswith('GND#') or u not in U: continue
            m = net_mst(u); L = unit_len(u)
            if L - m > float(args.get('relayed_min', '1.5')) and L / max(m, 0.1) > 1.4: cand.append(((L - m) * (u_wt(u) if WEIGHTED else 1.0), u))
        cand.sort(reverse=True); done = 0
        for exc, u in cand:
            if time_tight(0.9): break
            pads = [D['pads'][k] for k in by_net[u]]
            bx = (min(p['x'] for p in pads) - 0.6, min(p['y'] for p in pads) - 0.6, max(p['x'] for p in pads) + 0.6, max(p['y'] for p in pads) + 0.6)
            group = [u]
            for w in list(U):
                if w == u or w.startswith(('FIX#', 'TACK#')): continue
                b2 = unit_box(w)
                if b2 and not (b2[0] > bx[2] or b2[2] < bx[0] or b2[1] > bx[3] or b2[3] < bx[1]): group.append(w)
            if PAIR_SEARCH_SPACE:
                group=sorted(set(group)|{PAIR_NETS[w] for w in group if pair_unit(w)})
            if len(group) > int(args.get('relayed_max', '40')): continue
            save = {w: copy.deepcopy(U[w]) for w in group}
            _ux = groups_index(group) if WEIGHTED else None
            L0 = sum(routing_mass(w) for w in group)
            if WEIGHTED:
                _g0 = {w: (u_wt(w), routing_mass(w), routing_mass_wt(w)) for w in group}
            for w in group: del_unit(w)
            ok = True; hsave = hist.copy(); PF[0] = 1.0
            order = [u] + sorted((w for w in group if w != u), key=grp_key)
            for w in pair_series(order):
                if not route_unit(w): ok = False
            for k_ in range(int(args.get('relayed_iter', '12'))):
                cfg = conflicts_group(group, _ux) if _ux is not None else exact_conflicts()
                if args.get('relayed_chk') == '1': assert set(cfg) == set(exact_conflicts()), 'Relayed-GroupCheck'
                bad = [w for w in cfg if w in group]
                if not bad or not ok: break
                for w, cells in cfg.items():
                    if w in group: hist[np.array(cells, dtype=np.int64)] += 1.0
                PF[0] *= 1.4
                for w in pair_series([w for w in order if w in bad]):
                    if not pair_unit(w): del_unit(w)
                    if not route_unit(w): ok = False
            PF[0] = 1e3; hist[:] = hsave
            L1 = sum(routing_mass(w) for w in group if w in U)
            _better = L1 < L0 - 0.3
            if WEIGHTED and ok:



                _better, _gw0, _gw1, _reversed = _swap_better([(_g0[w][0], _g0[w][1], routing_mass(w), _g0[w][2], routing_mass_wt(w)) for w in group])
                _ugw[0] += 1
                if _better != (L1 < L0 - 0.3):
                    _ugw[1] += 1
                    print('   reroute %-24s weighted %s: bare %.1f -> %.1f mm, weighted %.1f -> %.1f%s' % (
                        u, 'taken' if _better else 'declined', L0, L1, _gw0, _gw1, ', a more important net would get longer' if _reversed else ''), flush=True)
            if ok and _better and not (conflicts_group(group, _ux) if _ux is not None else exact_conflicts()):
                done += 1; print('   rerouted %-24s group %2d: %.1f -> %.1f mm' % (u, len(group), L0, L1), flush=True)
            else:
                for w in group:
                    if w in U: del_unit(w)
                for w in group: add_unit(w, save[w]['net'], save[w]['tracks'], save[w]['vias'])
        print('Rerouting round %d: %d groups improved (%.0fs)' % (rnd + 1, done, time.time() - t3), flush=True)
        if not done: break
    PF[0] = PFs
    if WEIGHTED: print('Weighted rerouting (R21): %d complete attempts, %d of them decided differently than by bare length' % tuple(_ugw), flush=True)







SINGLE_TEST = args.get('single_test', '0') == '1'
ET_LOCATION = args.get('et_location', 'post')
def single_test():
    import os as _os
    sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    from analysis.coupling import coupling as _coupling, sets as _kmeans
    t2 = time.time(); hist[:] = 0; PFs = PF[0]; PF[0] = 1e3; _CAND_IDX[0] = None
    _ET_O, _ET_S = _kmeans(NC)
    _ET_WT = WT_KIND
    if _ET_WT is None:
        try:
            from common.importance import weights as _gw_et
            _ET_WT = _gw_et()
        except Exception: _ET_WT = {}
    _p_via = float(args.get('et_via', args.get('short_via_mm', '3.0'))); _p_narrow = float(args.get('et_narrow', '2.0'))
    _p_coupling = float(args.get('et_coupling', '1.0')); _et_min = float(args.get('et_min', '0.1'))
    _et_vmore = int(args.get('et_vias_more', '0')); _et_var = int(args.get('et_variants', '2')); _et_disturb = float(args.get('et_disturb', '2.0'))
    _HARD = ('current_val', 'supply', 'sw')
    def _et_kind(u): return NC['net_list'].get(U[u]['net'], {}).get('category', 'signal')
    def _et_narrow(u):
        r = U[u]; C = cls(r['net']); Cs = cls('/__signal__'); s = 0.0
        WF = neck_field_map(r['net'], C, C['wmin'], False)
        for t in r['tracks']:
            if t.get('web'): continue
            nominal = Cs['wmin'] if t.get('sig') else C['wmin']; li = LI[t['layer']]
            for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                if w >= nominal - 1e-6: continue
                ix, iy = int(((a[0] + b[0]) / 2 - X0) / RES), int(((a[1] + b[1]) / 2 - Y0) / RES)
                if WF is not None and not t.get('sig') and 0 <= ix < NX and 0 <= iy < NY and float(WF[li * NN + iy * NX + ix]) < nominal: continue
                s += math.dist(a, b)
        return s
    def _et_tr(w): return [dict(net=U[w]['net'], layer=t['layer'], pts=t['pts'], w=t['w'], kelvin=bool(t.get('tap'))) for t in U[w]['tracks']]
    def _et_coupling(u):
        net = U[u]['net']; k = 0.0
        if net in _ET_O or any(t.get('tap') for t in U[u]['tracks']):
            R = dict(tracks=_et_tr(u) + [x for w in U if w != u and U[w]['net'] in _ET_S for x in _et_tr(w)])
            k += _coupling(R, NC, victim={net}, aggressor=_ET_S)['total_sum']
        if net in _ET_S:
            R = dict(tracks=_et_tr(u) + [x for w in U if w != u and U[w]['net'] != net and (U[w]['net'] in _ET_O or any(t.get('tap') for t in U[w]['tracks'])) for x in _et_tr(w)])
            k += _coupling(R, NC, aggressor={net: _ET_S[net]})['total_sum']
        return k
    def _et_mass(u):
        nv = u_vias(u); Lb = unit_len(u) - 0.6 * nv; g = _ET_WT.get(_et_kind(u), 1.0)
        pv = _p_via * (2.0 if actual_fast(U[u]['net']) else 1.0); ps = _p_narrow * (2.5 if _et_kind(u) in _HARD else 1.0)
        Ls = _et_narrow(u); K = _et_coupling(u)
        return dict(rating=g * Lb + pv * nv + ps * Ls + _p_coupling * K, L=Lb, v=nv, s=Ls, k=K, f=cons_missing(u) if CONS else 0)
    def _et_ok(u, m0, m1):
        category = _et_kind(u); hard = category in _HARD
        if m1['rating'] >= m0['rating'] - _et_min: return False
        if m1['L'] > m0['L'] * 1.10 + 1.0: return False
        if m1['v'] > m0['v'] + (0 if hard or actual_fast(U[u]['net']) else _et_vmore): return False
        if m1['s'] > m0['s'] + (1e-6 if hard else 0.3): return False
        if m1['k'] > m0['k'] + 1e-6 or m1['f'] > m0['f']: return False
        return True
    _et_units = sorted([u for u in units if u in U and not pair_unit(u) and not u.startswith(('GND#', 'FIX#', 'TACK#')) and U[u]['tracks']
                        and not any(v.get('gr') or v.get('n') for v in U[u]['vias'])], key=lambda u: (grp(u), u))
    _et_n = 0; _et_var2 = 0; _et_d = dict(rating=0.0, L=0.0, v=0, s=0.0, k=0.0); _et_det = []
    for u in _et_units:
        if time_tight(0.9): break
        old = copy.deepcopy(U[u]); m0 = _et_mass(u); best = None
        variants = [1.0] + ([_et_disturb] if _et_var >= 2 and disturb_extra(U[u]['net']) is not None else [])
        del_unit(u)
        for vi, factor in enumerate(variants):
            DISTURB_FACTOR[0] = factor; ok = route_unit(u); DISTURB_FACTOR[0] = 1.0
            if ok:
                m1 = _et_mass(u)
                if _et_ok(u, m0, m1) and (best is None or m1['rating'] < best[0]['rating']) and unit_conflict_free(u):
                    best = (m1, copy.deepcopy(U[u]), vi)
            del_unit(u)
        if best is not None:
            m1, nu, vi = best; add_unit(u, nu['net'], nu['tracks'], nu['vias']); cand_idx_changed(u)
            _et_n += 1; _et_var2 += vi
            for k_ in _et_d: _et_d[k_] += m1[k_] - m0[k_]
            _et_det.append('%s %.1f->%.1f (L %+.1f, V %+d, narrow %+.2f, coupling %+.2f%s)' % (
                u.split('/')[-1], m0['rating'], m1['rating'], m1['L'] - m0['L'], m1['v'] - m0['v'], m1['s'] - m0['s'], m1['k'] - m0['k'], ', disturbance cost x%g' % _et_disturb if vi else ''))
        else:
            add_unit(u, old['net'], old['tracks'], old['vias']); cand_idx_back(u)
    PF[0] = PFs; _CAND_IDX[0] = None
    print('Single test: %d of %d units better (%d with doubled disturbance cost); metric %+.1f, track %+.1f mm, vias %+d, below minimum width %+.2f mm, coupling %+.2f (%.0fs)' % (
        _et_n, len(_et_units), _et_var2, _et_d['rating'], _et_d['L'], _et_d['v'], _et_d['s'], _et_d['k'], time.time() - t2), flush=True)
    for _z in _et_det: print('   ' + _z, flush=True)
if SINGLE_TEST and ET_LOCATION == 'before' and NC is not None and not todo and not time_tight(): single_test()
def vs_metrics(tracks, vias):
    L = 0.0; Lo = 0.0; bends = 0; short_ = 0; seg = 0
    for t in tracks:
        rv = None; fl = t['net'] in AREA_NETS
        for a, b in zip(t['pts'][:-1], t['pts'][1:]):
            d = math.dist(a, b); L += d; seg += 1
            if not fl: Lo += d
            if d < 0.1: short_ += 1
            if d < 1e-9: continue
            r = ((b[0] - a[0]) / d, (b[1] - a[1]) / d)
            if rv is not None and rv[0] * r[0] + rv[1] * r[1] < 0.99985: bends += 1
            rv = r
    category = {}
    if NC is not None:
        for v in vias:
            a_ = 'gnd' if v['net'] in GNDS else (NC['net_list'].get(v['net']) or {}).get('category', 'signal'); category[a_] = category.get(a_, 0) + 1
    return dict(vias=len(vias), vias_without_area=sum(1 for v in vias if v['net'] not in AREA_NETS),
                current_vias=sum(1 for v in vias if v.get('current_via') or v.get('gr')), track_list=len(tracks), segments=seg,
                bends=bends, seg_under_0_1=short_, total_length=round(L, 2), length_without_area=round(Lo, 2),
                **({'vias_per_kind': dict(sorted(category.items()))} if NC is not None else {}))


def metrics_now():
    return vs_metrics([dict(net=r['net'], pts=t['pts']) for r in U.values() for t in r['tracks']],
                      [dict(v, net=r['net']) for r in U.values() for v in r['vias']])




def local_connection(pts, A, B, capL, li, net, w):
    q = [tuple(x) for x in pts]; q[0] = tuple(A); q[-1] = tuple(B)
    out = [q[0]]
    for a, b in zip(q[:-1], q[1:]):
        if math.dist(a, b) < 1e-8: continue
        def vacant(x, y): return line_cap_min(capL, x, y) >= w - 1e-6 and pad_room(li, x, y, net) >= w / 2 - 1e-6
        if not oct_net(net) or oct_(a, b):
            if not vacant(a, b): return None
            out.append(b)
        else:
            c = next((c for c in _corners(a, b) if vacant(a, c) and vacant(c, b)), None)
            if c is None: return None
            out.extend((c, b))
    return out if len(out) >= 2 else None

def ast_shift(w):
    old = copy.deepcopy(U[w]); net = old['net']; C = cls(net)
    if pair_unit(w) or w.startswith(('FIX#', 'TACK#', 'GND#')) or (NC and (NC['net_list'].get(net) or {}).get('crystal')): return False
    for ti, t in enumerate(old['tracks']):
        if t.get('tap') or t.get('web') or len(t['pts']) < 3: continue
        A, B = t['pts'][0], t['pts'][-1]; li = LI[t['layer']]; ln = _LS(t['pts'])
        inner = ln.difference(_PT(A).buffer(0.03).union(_PT(B).buffer(0.03)))
        rest = [q for j, q in enumerate(old['tracks']) if j != ti]
        contacts = [(_LS((a, b)).buffer(max(t['w']) / 2).intersection(_LS((c, d)).buffer(ww / 2)))
                    for a, b in zip(t['pts'][:-1], t['pts'][1:]) for q in rest if q['layer'] == t['layer'] for c, d, ww in zip(q['pts'][:-1], q['pts'][1:], q['w'])]
        border = max([max(t['w'])] + [max(q['w']) for q in rest]) + 0.03
        ends = _PT(A).buffer(border).union(_PT(B).buffer(border))
        if any(not g.difference(ends).is_empty for g in contacts): continue
        if any(inner.distance(_PT(v['x'], v['y'])) < VIA_D / 2 for v in old['vias']): continue
        box = (min(q[0] for q in t['pts']) - 1, min(q[1] for q in t['pts']) - 1, max(q[0] for q in t['pts']) + 1, max(q[1] for q in t['pts']) + 1)
        del_unit(w); add_unit(w, net, rest, old['vias']); NET_CUR[0] = net
        ch, vh, cs, dd, rng, vb, cc, cf, vbf = maps(net, box, C['w'])
        def node(q): return li * NN + int((q[1] - Y0) / RES) * NX + int((q[0] - X0) / RES)
        src, tgt = node(A), node(B)
        if ch[li].ravel()[src % NN] < min(t['w']) or ch[li].ravel()[tgt % NN] < min(t['w']): path = None
        else: path = astar_pf([src], {tgt}, (B[0], B[1], B[0], B[1]), cc, cs, np.zeros_like(vh), vb, C, box)
        Q = None
        if path:
            tr, vi = path_to_geometry(path, cc, C)
            if len(tr) == 1 and not vi:
                Q = local_connection(tr[0]['pts'], A, B, cc[li], li, net, min(t['w']))
        del_unit(w)
        if Q is not None:
            nt = dict(t, pts=Q, w=[min(t['w'])] * (len(Q) - 1)); new = list(old['tracks']); new[ti] = nt
            add_unit(w, net, new, old['vias'])
            if unit_conflict_free(w): return True
            del_unit(w)
        add_unit(w, net, old['tracks'], old['vias'])
    return False




























VS_LOG = []
if args.get('vias_save') == '1' and not todo and not time_tight():
    t4 = time.time(); _vs_c0 = time.process_time()
    import os as _os
    sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    try: from analysis.evaluation import WT as VS_WT
    except Exception: VS_WT = {'sw': 4.0, 'current_val': 2.5, 'supply': 1.0, 'fast': 3.0, 'analog': 2.0, 'signal': 1.0}
    VS_MM = float(args.get('vs_mm', '3.0'))
    VS_VAR = [(float(x.split(':')[0]), float(x.split(':')[1]) if ':' in x else VS_MM) for x in args.get('vs_var', '1000/3').split('/') if x]
    VS_HARD = float(args.get('vs_hard', '1000')); VS_GR = int(args.get('vs_group', '6')); VS_IT = int(args.get('vs_iter', '4'))
    VS_MIN = float(args.get('vs_min', '1.0')); VS_TOL = float(args.get('vs_tol', '0.1'))
    VS_EPS = float(args.get('vs_gain', '0.5'))
    VS_BEND = float(args.get('vs_bend', '0'))
    VS_PROTECTION = args.get('vs_protection', '1') == '1'
    VS_CURRENT = args.get('vs_current', '0') == '1'
    VS_WIDTH = args.get('vs_width', '1') == '1'
    VS_RET = float(args.get('vs_ret', '1.0'))
    VS_AMPAD = float(args.get('vs_ampad', '0.5'))
    VS_PURE = args.get('vs_pure', '0') == '1'
    VS_PADS = int(args.get('vs_pads', '6'))
    VS_PATIENCE = int(args.get('vs_patience', '1'))
    VS_NEIGHBOR_VIAS = args.get('vs_neighbor_vias', '1') == '1'
    VS_HIGHER = args.get('vs_higher', '1') == '1'
    _vs_p = sum(len(by_net[u]) for u in units if not u.startswith('GND#')) + sum(1 for u in units if u.startswith('GND#'))
    VS_WORK = float(args.get('vs_work', '6')) * _vs_p; _vs_work = [0, 0.0, 0.0, 0.0]
    VS_BUDGET = max(float(args.get('vs_time_min', '30')), float(args.get('vs_time', '1.0')) * _vs_c0)
    VS_CHK = args.get('vs_chk') == '1'
    _vs_prev = {w: (r['net'], r['tracks'], r['vias']) for w, r in U.items()}
    hsave = hist.copy(); hist[:] = 0; PFs = PF[0]; _CAND_IDX[0] = None
    metric0 = metrics_now()
    def vs_vias(w): return sum(1 for v in U[w]['vias'] if not v.get('gr') and not v.get('rv'))
    def u_len(w): return sum(math.dist(a, c) for t in U[w]['tracks'] if not t.get('web') for a, c in zip(t['pts'][:-1], t['pts'][1:]))
    def u_missing(w): return cons_missing(w) if CONS else 0
    def u_bends(w):
        n_ = 0
        for t in U[w]['tracks']:
            rv = None
            for a, b in zip(t['pts'][:-1], t['pts'][1:]):
                d = math.dist(a, b)
                if d < 1e-9: continue
                r_ = ((b[0] - a[0]) / d, (b[1] - a[1]) / d)
                if rv is not None and rv[0] * r_[0] + rv[1] * r_[1] < 0.99985: n_ += 1
                rv = r_
        return n_
    def vs_kind(w):
        if w.startswith('GND#') or w in GNDS: return 'gnd'
        if NC is not None: return (NC['net_list'].get(w) or {}).get('category', 'signal')
        return 'current_val' if cls(w)['prio'] < 8 else 'signal'
    def vs_protection(w): return pair_unit(w) or VS_PROTECTION and vs_kind(w) in ('gnd', 'current_val', 'supply', 'sw')
    def vs_current(w): return vs_kind(w) in ('gnd', 'current_val', 'supply', 'sw')
    def vs_actual_current(w): return VS_WIDTH and not w.startswith('GND#') and vs_current(w)
    def vs_pad_spacing(w):
        bbs = [D['pads'][int(w[4:])]['bbox']] if w.startswith('GND#') else [D['pads'][k]['bbox'] for k in by_net_g.get(U[w]['net'], [])]
        return sorted(min(math.hypot(max(b[0] - v['x'], 0.0, v['x'] - b[2]), max(b[1] - v['y'], 0.0, v['y'] - b[3])) for b in bbs)
                      for v in U[w]['vias'] if not v.get('gr') and not v.get('rv')) if bbs else []
    def vs_pad_ok(a_, n_): return not a_ or not n_ or (max(n_) <= max(a_) + 0.05 and sum(n_) / len(n_) <= sum(a_) / len(a_) + 0.05)
    def vs_leff(net, tracks):
        C = cls(net); nominal = C['w']; s_ = 0.0
        xs_ = [q[0] for t in tracks for q in t['pts']] or [0.0]; ys_ = [q[1] for t in tracks for q in t['pts']] or [0.0]
        _cH, _, _cS, _dd, _, _, capC, _, _ = maps(net, (min(xs_) - 1.0, min(ys_) - 1.0, max(xs_) + 1.0, max(ys_) + 1.0), nominal)
        for t in tracks:
            if t.get('web'): continue
            li = LI[t['layer']]; thick = 1.0 if li in (0, NL - 1) else 2.0
            for a, b, w0 in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                d = math.dist(a, b)
                if d <= 0: continue
                if t.get('sig'): s_ += d; continue
                wv = max(w0, math.floor(min(nominal, line_cap_min(capC[li], a, b), 2 * pad_room(li, a, b, net)) * 40) / 40)
                s_ += d * thick * nominal / wv
        return s_
    def u_mass(w):
        if not vs_actual_current(w): return u_len(w)
        s_ = (U[w]['net'], U[w]['tracks'], U[w]['vias']); del_unit(w); v = vs_leff(s_[0], s_[1]); add_unit(w, *s_)
        return v
    _vs_m = {}
    def u_mass0(w):
        if w not in _vs_m: _vs_m[w] = u_mass(w)
        return _vs_m[w]
    def u_wt(w): return VS_WT.get(vs_kind(w), 1.0)
    def u_via_weight(w): return VS_MM * (2.0 if vs_kind(w) == 'fast' else 1.0)
    def vs_bottom(n):
        gem = None
        for k in by_net[n]:
            s_ = {L for L in D['pads'][k]['layers'] if L in LI}; gem = s_ if gem is None else gem & s_
        return 0 if gem else 1
    def vs_potential(u):
        return u_wt(u) * (u_mass0(u) + u_via_weight(u) * vs_vias(u) - net_mst(u) - u_via_weight(u) * vs_bottom(u))
    def vs_viaf(w, mm):
        if vs_protection(w): return 1.0
        return max(1.0, mm * u_via_weight(w) / VS_MM / max(cls(w)['via'], 0.05))
    def vs_pad_d(net, v):
        return min((math.hypot(max(b[0] - v['x'], 0.0, v['x'] - b[2]), max(b[1] - v['y'], 0.0, v['y'] - b[3]))
                    for b in (D['pads'][k]['bbox'] for k in by_net_g.get(net, []))), default=9.0)
    def vs_mask(net, vias):
        m = np.zeros((NY, NX), dtype=bool)
        for v in vias:
            r_ = 0.0 if vs_pad_d(net, v) <= VS_AMPAD else VS_RET; rr = int(r_ / RES) + 1
            ix, iy = int((v['x'] - X0) / RES), int((v['y'] - Y0) / RES)
            y0, y1, x0, x1 = max(0, iy - rr), min(NY, iy + rr + 1), max(0, ix - rr), min(NX, ix + rr + 1)
            yy, xx = np.ogrid[y0:y1, x0:x1]
            m[y0:y1, x0:x1] |= (xx - ix) ** 2 + (yy - iy) ** 2 <= (r_ / RES) ** 2
        return m.ravel()
    def vs_pad_vias_present(net, prev_v, new_v):
        return all(any(abs(v['x'] - q['x']) < 0.011 and abs(v['y'] - q['y']) < 0.011 for q in new_v) for v in prev_v if vs_pad_d(net, v) <= VS_AMPAD)
    def vs_offset(prev_tr, new_tr):
        m = 0.0
        for t in new_tr:
            old = [_LS(a['pts']) for a in prev_tr if a['layer'] == t['layer'] and len(a['pts']) >= 2]
            if not old: return None
            for q in t['pts']: m = max(m, min(l_.distance(_PT(q[0], q[1])) for l_ in old))
        return round(m, 2)
    _vs_mine = {}
    def vs_hard(net, tracks, vias):
        xs_ = [q[0] for t in tracks for q in t['pts']] + [v['x'] for v in vias]; ys_ = [q[1] for t in tracks for q in t['pts']] + [v['y'] for v in vias]
        if not xs_: return 0.0
        capH, viaH = hard_maps(net, _box_idx((min(xs_) - 1.0, min(ys_) - 1.0, max(xs_) + 1.0, max(ys_) + 1.0)) if LARGE else None)
        s_ = 0.0
        if net not in _vs_mine:
            _vs_mine[net] = {}
            for k in by_net_g.get(net, []):
                for L_, c in D['pads'][k]['bcells'].items(): _vs_mine[net].setdefault(LI[L_], set()).update(c.tolist())
        for t in tracks:
            li = LI[t['layer']]; capL = capH[li]; own_set = _vs_mine[net].get(li, ())
            for a, b, w0 in zip(t['pts'][:-1], t['pts'][1:], t['w']):
                L = math.dist(a, b)
                if L <= 0: continue
                tt = np.linspace(0, 1, max(2, int(L / (RES * 0.5)) + 1))
                ix = ((a[0] + tt * (b[0] - a[0]) - X0) / RES).astype(int); iy = ((a[1] + tt * (b[1] - a[1]) - Y0) / RES).astype(int)
                tight = capL[iy, ix] < w0 - 0.005
                if tight.any(): s_ += L * sum(1 for q in (iy[tight] * NX + ix[tight]).tolist() if q not in own_set) / len(tt)
        for v in vias:
            if not viaH[int((v['y'] - Y0) / RES), int((v['x'] - X0) / RES)]: s_ += VIA_D
        return s_
    def vs_fixed(w, gu=None):
        if w.startswith(('FIX#', 'TACK#')): return True
        if gu is None: return False
        if w.startswith('GND#'): return True
        if NC is not None and (NC['net_list'].get(w) or {}).get('crystal'): return True
        if len(by_net[w]) > VS_PADS and not (args.get('vs_branches') == '1' and not vs_protection(w)): return True
        return gu is not None and not VS_HIGHER and u_wt(w) > gu + 1e-9
    def vs_route(w, who):
        _t = time.process_time(); _vs_work[0] += 1 if w.startswith('GND#') else len(by_net[w])
        ok_ = (ast_shift(w) if who == 2 and w in U and args.get('vs_branches') == '1' and not vs_protection(w) else False)
        if not ok_ and not (who == 2 and args.get('vs_branches') == '1' and len(by_net[w]) > VS_PADS):
            if w in U: del_unit(w)
            ok_ = route_unit(w)
        _vs_work[who] += time.process_time() - _t
        return ok_
    def vs_attempt(u, pfw, mm, X):
        G = [u]; prior = {u: (U[u]['net'], U[u]['tracks'], U[u]['vias'])}; masks = {}
        amount = {u: (vs_vias(u), u_len(u), u_missing(u), u_mass0(u), vs_pad_spacing(u) if vs_protection(u) else None, u_bends(u) if VS_BEND else 0)}; gu = u_wt(u)
        def k0(w): return u_wt(w) * (amount[w][3] + u_via_weight(w) * amount[w][0] + VS_BEND * amount[w][5])
        def k1(w): return u_wt(w) * (u_mass(w) + u_via_weight(w) * vs_vias(w) + (VS_BEND * u_bends(w) if VS_BEND else 0.0))
        hc = []; vacant = False; history = []; reason = 'no gain of its own'
        if VS_RET > 0 and vs_protection(u): masks[u] = vs_mask(prior[u][0], prior[u][2])
        del_unit(u); PF[0] = pfw; VIAF[0] = vs_viaf(u, mm); VIA_MASK[0] = masks.get(u)
        ok = vs_route(u, 1); VIA_MASK[0] = None
        if ok and k1(u) < k0(u) - VS_EPS:
            reason = 'Conflicts remain'
            for round_no in range(VS_IT + 1):
                _t = time.process_time(); cfg = conflicts_group(G, X, set(G)); history.append(len(cfg)); _vs_work[3] += time.process_time() - _t
                if VS_CHK:
                    _v = exact_conflicts(); assert set(_v) == set(cfg), ('vias_save-Checking', sorted(set(_v) ^ set(cfg)))
                if not cfg: vacant = True; break
                if round_no == VS_IT: break
                new = [w for w in sorted(cfg) if w not in prior and not vs_fixed(w, gu)]
                if len(G) + len(new) > VS_GR: reason = 'Group too large'; break
                for w in new:
                    prior[w] = (U[w]['net'], U[w]['tracks'], U[w]['vias']); amount[w] = (vs_vias(w), u_len(w), u_missing(w), (_vs_m[w] if w in _vs_m else u_mass(w)) if vs_actual_current(w) else u_len(w), vs_pad_spacing(w) if vs_protection(w) else None, u_bends(w) if VS_BEND else 0); G.append(w)
                other = sorted((w for w in G[1:] if w in cfg), key=lambda w: vs_kind(w) == 'fast')
                if round_no >= VS_PATIENCE or not other:
                    for w, cells in cfg.items():
                        c = np.array(cells, dtype=np.int64); hist[c] += 1.0; hc.append(c)
                PF[0] = VS_HARD
                for w in other:
                    if VS_RET > 0 and vs_protection(w):
                        if w not in masks: masks[w] = vs_mask(prior[w][0], prior[w][2])
                        VIA_MASK[0] = masks[w]
                    if args.get('vs_branches') != '1' or vs_protection(w): del_unit(w)
                    VIAF[0] = vs_viaf(w, mm)
                    if not vs_route(w, 2): ok = False
                    VIA_MASK[0] = None
                if u in cfg and (round_no >= VS_PATIENCE or not other):
                    PF[0] = VS_HARD if other else pfw
                    del_unit(u); VIAF[0] = vs_viaf(u, mm); VIA_MASK[0] = masks.get(u)
                    if not vs_route(u, 1): ok = False
                    VIA_MASK[0] = None
                    if ok and not k1(u) < k0(u) - VS_EPS: reason = 'no gain of its own'; break
                if not ok: reason = 'incomplete'; break
                if sum(vs_vias(w) for w in G) > sum(amount[w][0] for w in G): reason = 'Vias of the group increase'; break
        elif not ok: reason = 'incomplete'
        def hard_bad(): return [w for w in G if vs_hard(U[w]['net'], U[w]['tracks'], U[w]['vias']) > vs_hard(*prior[w]) + 0.05]
        if vacant and ok:



            new_z = list(G) if VS_PURE else hard_bad()
            if new_z:
                PF[0] = VS_HARD
                for w in new_z:
                    if VS_RET > 0 and vs_protection(w) and w not in masks: masks[w] = vs_mask(prior[w][0], prior[w][2])
                    if args.get('vs_branches') != '1' or vs_protection(w): del_unit(w)
                    VIAF[0] = vs_viaf(w, mm); VIA_MASK[0] = masks.get(w)
                    if not vs_route(w, 1 if w == u else 2): ok = False
                    VIA_MASK[0] = None
                if not ok: reason = 'incomplete'
                else:
                    cfg = conflicts_group(G, X, set(G)); history.append(len(cfg))
                    if cfg: vacant = False; reason = 'Conflicts after final drawing'
                    elif hard_bad(): vacant = False; reason = 'Track would lie in blocked area (solder mask, edge, drill)'
        erg = None
        if vacant and ok:
            V0 = sum(amount[w][0] for w in G); V1 = sum(vs_vias(w) for w in G)
            L0 = sum(amount[w][1] for w in G); L1 = sum(u_len(w) for w in G)
            M1 = {w: u_mass(w) for w in G}
            K1 = {w: u_wt(w) * (M1[w] + u_via_weight(w) * vs_vias(w) + (VS_BEND * u_bends(w) if VS_BEND else 0.0)) for w in G}
            balance = sum(K1[w] - k0(w) for w in G)
            if V1 > V0: reason = 'Vias of the group increase (%d -> %d)' % (V0, V1)
            elif sum(u_missing(w) for w in G) > sum(amount[w][2] for w in G): reason = 'Via group incomplete'
            elif any(vs_protection(w) and vs_vias(w) < amount[w][0] for w in G): reason = 'protected net would lose a via'
            elif any(vs_protection(w) and not (vs_pad_ok(amount[w][4], vs_pad_spacing(w)) and vs_pad_vias_present(prior[w][0], prior[w][2], U[w]['vias'])) for w in G):
                reason = 'Via of a protected net moves away from the pad'
            elif any(vs_protection(w) and M1[w] > 1.05 * amount[w][3] + VS_TOL for w in G[1:]): reason = 'Current net would get more resistance'
            elif any(u_wt(w) > gu + 1e-9 and (vs_vias(w) > amount[w][0] or K1[w] > k0(w) + u_wt(w) * VS_TOL) for w in G[1:]):
                reason = 'more important net would get worse'
            elif not VS_NEIGHBOR_VIAS and any(vs_vias(w) > amount[w][0] for w in G[1:]): reason = 'Neighbor would get more vias'
            elif not balance < -VS_EPS: reason = 'Balance %+.1f' % balance
            else:
                speed = [w for w in G[1:] if U[w]['tracks'] != prior[w][1] or U[w]['vias'] != prior[w][2]]
                new_v = {(v['x'], v['y']) for v in U[u]['vias']}
                erg = dict(net_name=u, pf=pfw, via_mm=mm, group=G[1:], pushed=speed, vias=[V0, V1], total_length=[round(L0, 2), round(L1, 2)],
                           balance=round(balance, 2), ratio=round(-balance / (1 + sum(u_wt(w) for w in speed)), 3), rounds_n=history,
                           way=[[v['x'], v['y']] for v in prior[u][2] if not v.get('gr') and not v.get('rv') and (v['x'], v['y']) not in new_v][:6],
                           per={w: [amount[w][0], vs_vias(w), round(amount[w][1], 2), round(u_len(w), 2), vs_offset(prior[w][1], U[w]['tracks'])] for w in G},
                           _new={w: (U[w]['net'], U[w]['tracks'], U[w]['vias']) for w in G}, _prev=prior)
        for c in hc: hist[c] = 0.0
        for w in G:
            if w in U: del_unit(w)
        for w in G: add_unit(w, *prior[w])
        return erg, len(G), history, reason
    _vs_error = False
    try:
        for w in [w for w in U if vs_actual_current(w) and not w.startswith(('FIX#', 'TACK#'))]: u_mass0(w)
        X = groups_index([]); _vs_off = False; _vs_boxes = None
        for rnd in range(int(args.get('vs_rounds', '2'))):
            cand = [(grp(u)[0], -vs_potential(u), u) for u in units if u in U and not u.startswith('GND#') and not vs_fixed(u)
                    and (not vs_protection(u) or (VS_CURRENT and vs_kind(u) != 'gnd'))]
            cand = [u for _g, p_, u in sorted(cand) if -p_ >= VS_MIN]
            if _vs_boxes is not None:
                def _nearby_box(u):
                    b = unit_box(u); pb = [D['pads'][k]['bbox'] for k in by_net[u]]
                    b = (min([q[0] for q in pb] + ([b[0]] if b else [])), min([q[1] for q in pb] + ([b[1]] if b else [])),
                         max([q[2] for q in pb] + ([b[2]] if b else [])), max([q[3] for q in pb] + ([b[3]] if b else [])))
                    return any(not (b[0] > q[2] + 1.5 or b[2] < q[0] - 1.5 or b[1] > q[3] + 1.5 or b[3] < q[1] - 1.5) for q in _vs_boxes)
                cand = [u for u in cand if _nearby_box(u)]
            boxes = []; n_ok = 0; dv = 0; dl = 0.0; db = 0.0; n_supply = 0
            for u in cand:
                if _vs_work[0] > VS_WORK: _vs_off = 'Effort used up'; break
                if (not BUDGET_WORK and time.process_time() - _vs_c0 > VS_BUDGET) or time_tight(0.9): _vs_off = 'Time share used up (result thus depends on compute time)'; break
                pot = vs_potential(u); best = None
                for pfw, mm in VS_VAR:
                    erg, ng, history, reason = vs_attempt(u, pfw, mm, X); n_supply += 1
                    if args.get('vs_log') == '1' and erg is None and (ng > 1 or reason != 'no gain of its own'):
                        print('   no gain   %-24s PF %g, group %d, conflicts per round %s: %s' % (u, pfw, ng, history, reason), flush=True)
                    if erg is None: continue
                    if best is None or erg['ratio'] > best['ratio']: best = erg
                    if -erg['balance'] >= 0.9 * pot: break
                if best is None: continue
                new_ = best.pop('_new'); prev_ = best.pop('_prev'); _nb = len(boxes)
                for w in new_: del_unit(w)
                for w in new_: add_unit(w, *new_[w])
                best['round_no'] = rnd + 1; best['potential'] = round(pot, 2); VS_LOG.append(best)
                n_ok += 1; dv += best['vias'][0] - best['vias'][1]; dl += best['total_length'][1] - best['total_length'][0]; db += best['balance']
                for w in new_:
                    ta = {(t['layer'], tuple(map(tuple, t['pts']))) for t in prev_[w][1]}; tn = {(t['layer'], tuple(map(tuple, t['pts']))) for t in new_[w][1]}
                    for _l, pts in sorted(ta ^ tn): boxes.append((min(q[0] for q in pts), min(q[1] for q in pts), max(q[0] for q in pts), max(q[1] for q in pts)))
                    va = {(v['x'], v['y']) for v in prev_[w][2]}; vn = {(v['x'], v['y']) for v in new_[w][2]}
                    for x_, y_ in sorted(va ^ vn): boxes.append((x_, y_, x_, y_))
                for w in new_: _vs_m.pop(w, None)
                for w in [w for w in list(_vs_m) if w not in new_]:
                    b = unit_box(w)
                    if b and any(not (b[0] > q[2] + 2.0 or b[2] < q[0] - 2.0 or b[1] > q[3] + 2.0 or b[3] < q[1] - 2.0) for q in boxes[_nb:]): _vs_m.pop(w)
                for w in [w for w in U if vs_actual_current(w) and not w.startswith(('FIX#', 'TACK#'))]: u_mass0(w)
                X = groups_index([])
                print('   %-13s %-24s Vias %d -> %d, length %.1f -> %.1f mm, balance %+.1f (potential %.1f), PF %g, pushed %s, via gone at %s' % (
                    'Via saved' if best['vias'][1] < best['vias'][0] else 'shorter', u, best['vias'][0], best['vias'][1], best['total_length'][0],
                    best['total_length'][1], best['balance'], pot, best['pf'],
                    ['%s (%s mm)' % (w, best['per'][w][4]) for w in best['pushed']], best['way']), flush=True)
            print('Saving vias round %d: %d candidates, %d attempts, %d accepted, %d fewer vias, length %+.1f mm, balance %+.1f (%.0fs)%s' % (
                rnd + 1, len(cand), n_supply, n_ok, dv, dl, db, time.time() - t4, ', ' + _vs_off if _vs_off else ''), flush=True)
            if not n_ok or _vs_off: break
            _vs_boxes = boxes
    except Exception:
        if VS_CHK: raise
        import traceback
        traceback.print_exc(file=sys.stdout); _vs_error = True
    VIAF[0] = 1.0; VIA_MASK[0] = None; PF[0] = PFs; hist[:] = hsave; _CAND_IDX[0] = None
    if _vs_error or exact_conflicts():
        print('Saving vias: %s -> everything back to the previous state' % ('Error in post-pass (see above)' if _vs_error else 'Final check finds conflicts %s' % sorted(set((a, b) for (_l, _d, a, b, _x, _y) in PAIRS))[:6]), flush=True)
        for w in list(U): del_unit(w)
        for w, s_ in _vs_prev.items(): add_unit(w, *s_)
        VS_LOG.clear()



    for w in _vs_prev:
        if w in U: U[w] = U.pop(w)
    metric1 = metrics_now()
    print('Saving vias: vias %d -> %d (without area nets %d -> %d), bends %d -> %d, length %.1f -> %.1f mm, CPU %.0f s (candidate %.0f, partner %.0f, check %.0f), effort %d of %d connections' % (
        metric0['vias'], metric1['vias'], metric0['vias_without_area'], metric1['vias_without_area'], metric0['bends'], metric1['bends'],
        metric0['total_length'], metric1['total_length'], time.process_time() - _vs_c0, _vs_work[1], _vs_work[2], _vs_work[3], _vs_work[0], VS_WORK), flush=True)



R19_SHAPE = args.get('r19_shape', '0') == '1' and NC is not None
R19_SHAPE_STAT = dict(checked=0, rejected=0, examples=[])
def r19_shape_ok(prior, new):
    if not R19_SHAPE or (NC['net_list'].get(prior['net']) or {}).get('category') not in ('signal', 'analog', 'fast'): return True
    from analysis.r19 import shape_ok, shape_size
    R19_SHAPE_STAT['checked'] += 1
    if shape_ok(prior, new): return True
    R19_SHAPE_STAT['rejected'] += 1
    if len(R19_SHAPE_STAT['examples']) < 12:
        R19_SHAPE_STAT['examples'].append(dict(net=prior['net'], prior=shape_size(prior), new=shape_size(new)))
    return False
if HYPO_LATE and not todo and not time_tight():
    _HYPO_ON[0] = True; t2 = time.time(); PFs = PF[0]; PF[0] = 1e3; hist[:] = 0; _CAND_IDX[0] = None; _nu = 0
    _htol = float(args.get('hypo_tol', '0.01'))
    _rest = sorted([u for u in units if u in U and not pair_unit(u) and not (u.startswith('GND#') and int(u[4:]) in EPM)], key=short_key); _why = {}
    for _rd in range(2):
        _remains = []
        for u in _rest:
            if time_tight(0.9): break
            old = copy.deepcopy(U[u]); L0 = unit_len(u); _v0 = n_sv(U[u]); _f0 = cons_missing(u) if CONS else 0
            del_unit(u)
            ok = route_unit(u)
            if ok and unit_len(u) <= L0 * (1 + _htol) + 0.2 and n_sv(U[u]) <= _v0 and (not CONS or cons_missing(u) <= _f0) and unit_conflict_free(u) and r19_shape_ok(old, U[u]):
                _nu += 1; cand_idx_changed(u); continue
            _why[u] = 'incomplete' if not ok else ('longer %.1f instead of %.1f' % (unit_len(u), L0) if unit_len(u) > L0 * (1 + _htol) + 0.2 else
                         ('Vias %d instead of %d' % (n_sv(U[u]), _v0) if n_sv(U[u]) > _v0 else 'Via group or conflict'))
            del_unit(u); add_unit(u, old['net'], old['tracks'], old['vias']); cand_idx_back(u); _remains.append(u)
        _rest = _remains
        if not _rest: break
    PF[0] = PFs
    print('Reshaping: %d units reshaped, %d in old shape (%.0fs)' % (_nu, len(_rest), time.time() - t2), {u: _why[u] for u in _rest if not u.startswith('GND#')}, flush=True)
if SINGLE_TEST and ET_LOCATION != 'before' and NC is not None and not todo and not time_tight(): single_test()
if PAIR_NETS and not todo and not time_tight():
    print('Differential pairs (R29) coupled: %d' % pair_couple(), flush=True)
if RET > 0 and not todo and not time_tight():
    _rvn = rv_post_run(); print('Return vias (Q2) after negotiation: %d added, %d superfluous removed' % _rvn, flush=True)
if NC is not None and args.get('flow', '1') == '1':
    import os as _os
    sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    from common.flow import analysis as _fl_analysis, goal_width as _fl_goal
    _nf = 0
    for u, r in list(U.items()):
        if u.startswith(('GND#', 'FIX#', 'TACK#')): continue
        k = NC['net_list'].get(r['net'], {})
        if k.get('category') not in (('current_val', 'supply', 'sw') if args.get('current_fix') == '1' else ('current_val', 'supply')) or not k.get('flow'): continue
        info = _fl_analysis([t for t in r['tracks']], r['vias'], [D['pads'][i] for i in by_net[r['net']]], k['flow'], endpoints=args.get('current_fix') == '1')
        if not info: continue
        C_ = cls(r['net'])
        FLOW_W[u] = {sid: _fl_goal(a_, C_['wmin'], C_['w'], float(args.get('flow_exp', '0.5'))) for sid, a_ in info['share'].items()}
        if current_model(r['net']):
            FLOW_W[u] = {sid: max(TRACK_MIN,min(NECK_W,C_['wmin']),info['total'] * a_ / (CURRENT_J * CURRENT_T[r['tracks'][sid[0]]['layer']])) for sid,a_ in info['share'].items()}
        FLOW_A[u] = info['share']
        _nf += 1
    print('Flow principle: target widths for %d current nets' % _nf, flush=True)





FINE_SHIFT_LOG = []
def via_fine_shift():
    from analysis.evaluation import rate as _vf_score, not_connected as _vf_unconnected
    from analysis.topology import topology as _vf_topo
    from analysis.coupling import coupling as _vf_coupling
    def snapshot():
        tracks, vias = export_geometry()
        return dict(tracks=tracks, vias=vias, conflicts={}, failed=[], options=args,
                    gnd_layers=GND_LAYERS, area_nets=sorted(AREA_NETS))
    def gates(R):
        e = _vf_score(D, R, NC, v3=True); tp = _vf_topo(R, D)
        is_valid = e['diagnose'] and e['conflict_list'] == 0 and e['open_items'] == 0 and not _vf_unconnected(D, R, set(NC['gnd']) | AREA_NETS)
        return is_valid, e['score_v3'], _vf_coupling(R, NC)['total_sum'], tuple(tp[k] - tp.get(k + '_pad', 0) for k in ('star', 'sharp')), tp['parallel_rest_mm'], tp['kelvin_close_mm']
    def better(a, b): return b[0] and all(y <= x + 1e-7 for x, y in zip((a[1], a[2], *a[3], a[4], a[5]), (b[1], b[2], *b[3], b[4], b[5])))
    start = time.process_time(); attempts = 0; series = list(U); _CAND_IDX[0] = None; shifted = set()
    power = [u for u in U if not u.startswith(('FIX#', 'TACK#', 'GND#')) and (NC['net_list'].get(U[u]['net']) or {}).get('category') in ('current_val', 'supply')]
    for pu in power:
        if time_tight() or (not BUDGET_WORK and time.process_time() - start > float(args.get('vf_cpu', '30'))): break
        C = cls(U[pu]['net'])
        tight = [(LI[t['layer']], _LS((a, b)), max(C['wmin'], FLOW_W.get(pu, {}).get((ti, si), C['w'])))
               for ti, t in enumerate(U[pu]['tracks']) if not t.get('sig') and not t.get('web')
               for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])) if w < C['w'] - 1e-6 and math.dist(a, b) > 0.3]
        for u in list(U):
            r = U[u]; k = NC['net_list'].get(r['net']) or {}
            if u.startswith(('FIX#', 'TACK#', 'GND#')) or r['net'] in GNDS or k.get('crystal') or k.get('category') in ('current_val', 'supply', 'sw', 'fast'): continue
            for vi, v in enumerate(list(r['vias'])):
                if (u, vi) in shifted or v.get('gr') or v.get('rv') or v.get('n'): continue
                A = (v['x'], v['y'])
                if any(_PT(A).distance(_LS(poly + [poly[0]])) < 0.5 or _PT(A).within(Polygon(poly))
                       for pi in by_net_g.get(r['net'], []) for polys in D['pads'][pi].get('polys', {}).values() for poly in polys): continue
                near = [(li, ln, goal) for li, ln, goal in tight if ln.distance(_PT(A)) < goal / 2 + VIA_D / 2 + CLR + 0.3]
                if not near: continue
                inc = [(ti, e) for ti, t in enumerate(r['tracks']) for e in (0, -1) if math.dist(t['pts'][e], A) < 1e-6]
                if len(inc) < 2 or any(_LS((a, b)).distance(_PT(A)) < VIA_D / 2 + w / 2
                                      for tj, t in enumerate(r['tracks']) for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w']))
                                      if ((tj, 0) not in inc and (tj, -1) not in inc) or ((tj, 0) in inc and si > 0 and (tj, -1) not in inc) or ((tj, -1) in inc and si < len(t['w']) - 1 and (tj, 0) not in inc)): continue
                if any(r['tracks'][ti].get('tap') or r['tracks'][ti].get('web') for ti, e in inc): continue
                old = copy.deepcopy(r); pwold = copy.deepcopy(U[pu]); before = gates(snapshot())
                for ti, e in inc:
                    q = old['tracks'][ti]['pts'][1 if e == 0 else -2]; d = math.dist(A, q)
                    for step in (0.05, 0.1, 0.2, 0.3):
                        if step >= d - 0.05 or attempts >= int(args.get('vf_attempts', '60')) or time_tight() or (not BUDGET_WORK and time.process_time() - start > float(args.get('vf_cpu', '30'))): break
                        attempts += 1; B = (A[0] + (q[0] - A[0]) * step / d, A[1] + (q[1] - A[1]) * step / d)
                        del_unit(u); net = old['net']; Cn = cls(net); ch, vh, cs, dd, rng, vb, cc, cf, vbf = maps(net, (B[0]-2,B[1]-2,B[0]+2,B[1]+2), Cn['w'])
                        ix, iy = int((B[0]-X0)/RES), int((B[1]-Y0)/RES); new = copy.deepcopy(old); ok = 0 <= ix < NX and 0 <= iy < NY and vh[iy, ix] and not vb[iy, ix]
                        for tj, end in inc:
                            t = new['tracks'][tj]; P = t['pts'] if end == 0 else t['pts'][::-1]; W = t['w'] if end == 0 else t['w'][::-1]
                            Q = local_connection([B, P[1]], B, P[1], cc[LI[t['layer']]], LI[t['layer']], net, W[0]) if ok else None
                            if Q is None: ok = False; break
                            Q += P[2:]; ww = [W[0]] * (len(Q) - len(P) + 1) + W[1:]
                            t['pts'], t['w'] = (Q, ww) if end == 0 else (Q[::-1], ww[::-1])
                        if ok:
                            new['vias'][vi] = dict(v, x=B[0], y=B[1]); add_unit(u, net, new['tracks'], new['vias']); _CAND_IDX[0] = None; ok = unit_conflict_free(u)
                            if ok:
                                widen_unit(pu, C); _CAND_IDX[0] = None; after = gates(snapshot()); ok = unit_conflict_free(pu) and better(before, after) and current_resistance(pu) < current_resistance_prev(pwold, pu) - 1e-9
                        if ok:
                            shifted.add((u, vi)); FINE_SHIFT_LOG.append(dict(net_name=u, current_val=pu, prior=A, new=B, offset_mm=step)); cand_idx_changed(u); cand_idx_changed(pu); break
                        if u in U: del_unit(u)
                        add_unit(u, old['net'], old['tracks'], old['vias'])
                        del_unit(pu); add_unit(pu, pwold['net'], pwold['tracks'], pwold['vias']); _CAND_IDX[0] = None
                    if FINE_SHIFT_LOG and FINE_SHIFT_LOG[-1]['prior'] == A: break
                r = U[u]
    for w in series: U[w] = U.pop(w)
    _CAND_IDX[0] = None
    print('Via fine shift: %d accepted, %d attempts, CPU %.1f s' % (len(FINE_SHIFT_LOG), attempts, time.process_time()-start), flush=True)

def via_density_post_run():
    global VIA_DENSITY
    from analysis.via_density import measure as density, short, guard_nets, via_guarded
    from analysis.evaluation import rate
    from analysis.rule_metrics import assumption as rule_gate
    from analysis.contract_spec import comparison as contract_gate
    from analysis.minimax import assumption as minimax_gate
    from analysis.wobble import measure as wobble
    start = time.process_time(); series = list(U); cost = VIA_DENSITY; pf = PF[0]
    def tight():
        return time_tight(0.95) or (not BUDGET_WORK and time.process_time()-start >= float(args.get('via_density_after_cpu', '45')))
    def snapshot():
        tr, vi = export_geometry()
        return export_data(tr, vi, {}, absent=[])
    def quality(R):
        return rate(D, R, NC, v3=True, **{k: str(args.get(k, '0')).lower() in ('1', 'true')
                       for k in ('si_score', 'si_measure', 'length_gate')}), wobble(D, R, gnd=NC.get('gnd') or [])
    try:
        if tight():
            VD_AFTER_LOG.update(reason='cpu_budget', cpu_s=round(time.process_time()-start, 3)); return
        base = snapshot(); vd0 = density(D, base, window=VD_WINDOW, threshold=VD_THRESHOLD)
        if not vd0['window_over'] and vd0['sharp_vias_mm2'] <= VD_THRESHOLD:
            VD_AFTER_LOG.update(reason='under_threshold', cpu_s=round(time.process_time()-start, 3)); return
        if tight():
            VD_AFTER_LOG.update(reason='cpu_budget', cpu_s=round(time.process_time()-start, 3)); return
        e0, w0 = quality(base)
        protected = guard_nets(NC, base, IMP_TAB)
        if not e0['is_valid']:
            VD_AFTER_LOG.update(reason='evaluation_invalid', cpu_s=round(time.process_time()-start, 3)); return
    except Exception as exc:
        VD_AFTER_LOG.update(reason='diagnosis_error', faults=[repr(exc)[:160]], cpu_s=round(time.process_time()-start, 3)); return
    cand = []
    for u in sorted(U):
        r = U[u]
        if pair_unit(u) or u.startswith(('FIX#', 'TACK#', 'GND#')): continue
        if not r['vias'] or any(via_guarded(dict(v, net=r['net']), NC, protected) for v in r['vias']): continue
        score = sum(float(_vd_cost()[int((v['y']-Y0)/RES), int((v['x']-X0)/RES)])
                    for v in r['vias'] if 0 <= int((v['x']-X0)/RES) < NX and 0 <= int((v['y']-Y0)/RES) < NY)
        if score: cand.append((-score, u))
    try:
        VIA_DENSITY = True; PF[0] = 1e3; _CAND_IDX[0] = None
        for _, u in sorted(cand)[:int(args.get('via_density_after_max', '8'))]:
            if tight(): break
            old = copy.deepcopy(U[u]); saved_fail = set(fail)
            saved_unreachable = copy.deepcopy(UNREACHABLE)
            VD_AFTER_LOG['attempts'] += 1
            accepted = False
            try:
                del_unit(u)
                if route_unit(u) and not tight() and unit_conflict_free(u) and not exact_conflicts():
                    new = snapshot(); q = short(new, NC, VD_WINDOW, VD_THRESHOLD)
                    if q['sharp_vias_mm2'] <= vd0['sharp_vias_mm2'] and q['vias'] <= vd0['vias']:
                        if tight(): continue
                        vd1 = density(D, new, window=VD_WINDOW, threshold=VD_THRESHOLD)
                        if (vd1['overload_integral_mm2'] < vd0['overload_integral_mm2'] - 1e-7
                                and vd1['window_over'] <= vd0['window_over']):
                            if tight(): continue
                            e1, w1 = quality(new)
                            accepted = e1['is_valid'] and rule_gate(e0.get('design_rules'), e1.get('design_rules'))['not_worse']
                            if accepted and any(e.get('gate_check') or (e.get('design_rules') or {}).get('measurement_version') for e in (e0, e1)):
                                accepted = contract_gate(e0, e1)['not_worse']
                            if accepted and any(e.get('evaluation_target') == 'minimax' for e in (e0, e1)):
                                accepted = minimax_gate(e0, e1)['not_worse']
                            accepted = accepted and w1['move_records'] <= w0['move_records']


                            accepted = accepted and not tight()
                            if accepted:
                                VD_AFTER_LOG['accepted'] += 1
                                VD_AFTER_LOG['move_records'].append(dict(net_name=u, integral=[vd0['overload_integral_mm2'], vd1['overload_integral_mm2']]))
                                vd0, e0, w0 = vd1, e1, w1
            except Exception as exc:
                VD_AFTER_LOG.setdefault('faults', []).append(repr(exc)[:160])
            finally:
                if not accepted:
                    if u in U: del_unit(u)
                    add_unit(u, old['net'], old['tracks'], old['vias'])
                    fail.clear(); fail.update(saved_fail)
                    UNREACHABLE.clear(); UNREACHABLE.update(saved_unreachable)
                _CAND_IDX[0] = None
    finally:
        VIA_DENSITY = cost; PF[0] = pf
        for u in series: U[u] = U.pop(u)
        _CAND_IDX[0] = None
        VD_AFTER_LOG['cpu_s'] = round(time.process_time()-start, 3)
    print('Via-Density-PostRun:', VD_AFTER_LOG, flush=True)


def current_resistance_prev(r, u):
    now = U[u]; U[u] = r
    try: return current_resistance(u)
    finally: U[u] = now

t1 = time.time()
PRE = {u: copy.deepcopy(r) for u, r in U.items()}
WIDE_DENSE_REPORT = None
if WIDE_DENSE:
    _CAND_IDX[0] = None
    _bd_before = exact_conflicts()
    _bd_before_n = len(PAIRS)

if 'pre' in args:
    _t, _v = [], []
    for u, r in U.items():
        for t in r['tracks']: _t.append(dict(net=r['net'], layer=t['layer'], pts=t['pts'], w=t['w']))
        for v in r['vias']: _v.append(dict(net=v.get('net') or r['net'], x=v['x'], y=v['y']))
    if args.get('short_circuit_protection', '0') == '1': _t, _v = export_geometry()
    json.dump(export_data(_t, _v, {}) if args.get('short_circuit_protection', '0') == '1' else dict(tracks=_t, vias=_v), open(args['pre'], 'w')); print('saved before widening:', args['pre'])
RHO = 1.72e-5
def t_cu(L): return 0.035 if L in ('F', 'B') else 0.0175
def chan_R(c):
    R = 0.0
    for u, r in U.items():
        k = r['net'].split('/')[-1]
        if k in ('K_' + c, 'S_' + c, 'M_' + c):
            for t in r['tracks']:
                if t.get('sig'): continue
                for a, b2, w in zip(t['pts'][:-1], t['pts'][1:], t['w']): R += RHO * math.dist(a, b2) / (w * t_cu(t['layer']))
            R += 0.0024 * len(r['vias'])
    return R
CH = sorted({r['net'].split('/')[-1][2:] for r in U.values() if r['net'].split('/')[-1][:2] in ('K_', 'S_', 'M_')})
WMAX = {0: 0.8, 1: 0.35, 2: 1.5, 3: 1.2, 4: 1.0, 5: 0.5, 6: 0.35} if args.get('wider') == '1' else {}
def ucls(u):
    if u.startswith('GND#'): return dict(w=max(0.35, TRACK_MIN), wmin=max(0.15, TRACK_MIN))
    if u in PLANE_GROUPS:return dict(w=max(0.35,TRACK_MIN),wmin=max(0.15,TRACK_MIN),prio=6.5)
    C = cls(u[4:] if u.startswith('FIX#') else u)
    return dict(C, w=max(C['w'], WMAX.get(C['prio'], 0)))
if args.get('fair') in ('1', 'all_items'):

    STEP = 0.05
    pw = [u for u in U if not u.startswith(('GND#', 'FIX#', 'TACK#')) and u not in FIXED_GND and u not in PLANE_GROUPS and
          ((NC['net_list'].get(U[u]['net']) or {}).get('category') in ('current_val', 'supply', 'sw') if NC and (args.get('fair_all') == '1' or args.get('fair') == 'all_items') else ucls(u)['prio'] < 8)]
    saturated = set()
    for rnd in range(60):
        Rc = {c: chan_R(c) for c in CH}
        Rn = {u: current_resistance(u) for u in pw} if NC and (args.get('fair_all') == '1' or args.get('fair') == 'all_items') else None
        def key(u):
            if Rn is not None: return (0, -Rn[u], ucls(u)['prio'])
            k = U[u]['net'].split('/')[-1]
            return (0, -Rc[k[2:]]) if k[:2] in ('K_', 'S_', 'M_') else (1, ucls(u)['prio'])
        ch = 0
        if time_tight(0.95): break
        for u in sorted(pw, key=key):
            if u in saturated:
                U[u] = U.pop(u); continue
            C = ucls(u); before = [list(t['w']) for t in U[u]['tracks']]
            widen_unit(u, dict(C, w_step=STEP))
            if [list(t['w']) for t in U[u]['tracks']] != before: ch += 1
            else: saturated.add(u)
        if not ch: break
    print('Fairly widened in %d rounds' % (rnd + 1), {c: round(chan_R(c) * 1000, 1) for c in CH}, 'mOhm', flush=True)
    rest = [u for u in U if u not in pw and not u.startswith(('FIX#', 'TACK#')) and u not in FIXED_GND]
else:
    rest = [u for u in U if not u.startswith(('FIX#', 'TACK#')) and u not in FIXED_GND]
for u in sorted(rest, key=lambda u: (ucls(u)['prio'] if not u.startswith('GND#') else 6.5)):
    if time_tight(0.95): break
    widen_unit(u, ucls(u))
print('Widened in %.0fs' % (time.time() - t1))

for rnd in range(0 if WIDE_DENSE else 6):
    cf = exact_conflicts()
    if not cf: break
    print('   conflicts after widening', len(PAIRS), sorted(PAIRS, key=lambda x: x[1])[:4], flush=True)
    _last = {}
    for _pi, (lay, dist, ua, ub, x, y) in enumerate(PAIRS):
        if lay == '*': continue
        for _ui, u in enumerate((ua, ub)):
            if pair_unit(u) or u not in U or u not in PRE or u.startswith(('FIX#', 'TACK#')): continue
            r = U[u]; pre = PRE[u]; rad = 0.6 + 0.4 * rnd
            if NC and args.get('current_fix') == '1':
                from common.current_track import correct
                li = LI[lay]
                obst = [(g, DRC_CLR) for other, rr in U.items() if rr['net'] != r['net'] and not _fixed_pair(u, other)
                        for l2, g, _, _ in _unit_geo(other, rr) if l2 == li]
                obst += [(g, DRC_PADC) for pn, g, _ in _PADB[li] if pn != r['net']]
                if correct(r['tracks'], pre['tracks'], lay, obst): _last[u] = (_pi, _ui)
                continue
            chg = False
            for t, t0 in zip(r['tracks'], pre['tracks']):
                if t['layer'] != lay: continue
                ws = list(t['w'])
                for k, (a, b2) in enumerate(zip(t['pts'][:-1], t['pts'][1:])):
                    if _LS([a, b2]).distance(_PT(x, y)) < rad and ws[k] > t0['w'][k]:
                        ws[k] = t0['w'][k] if rnd < 4 else min(t0['w'][k], ws[k])
                        chg = True
                t['w'] = ws
            if chg or rnd >= 4: _last[u] = (_pi, _ui)
    for u in sorted(_last, key=_last.get):
        r = U[u]
        tr = r['tracks'] if rnd < 5 or (NC and args.get('current_fix') == '1') else PRE[u]['tracks']
        del_unit(u); add_unit(u, r['net'], tr, r['vias'])
    if BGA_PLAN and not _last:


        break
if WIDE_DENSE:
    _bd_after = exact_conflicts()
    _bd_under = []
    for u, r in U.items():
        if u.startswith(('FIX#', 'TACK#', 'GND#')): continue
        if NC is None or (NC['net_list'].get(r['net']) or {}).get('category') not in ('current_val', 'supply', 'sw'): continue
        for ti, t in enumerate(r['tracks']):
            if t.get('sig'): continue
            for si, (a, b, w) in enumerate(zip(t['pts'][:-1], t['pts'][1:], t['w'])):
                nominal = FLOW_W.get(u, {}).get((ti, si), ucls(u)['w'])
                if not current_model(r['net']): nominal *= f_inner(r['net'], LI[t['layer']])
                if w + 1e-7 < nominal:
                    _bd_under.append(dict(net_name=r['net'], layer=t['layer'], pts=[a,b], width=w,
                                          nominal=nominal, total_length=math.dist(a,b)))
    WIDE_DENSE_REPORT = dict(phase='direct_after_widen',
        conflict_pairs_before=_bd_before_n, conflict_pairs_after=len(PAIRS),
        conflict_units_before=len(_bd_before), conflict_units_after=len(_bd_after),
        under_goal_width=_bd_under)
    print('Dense widened: pairs %d -> %d, units %d -> %d; %d current pieces below target width' % (
        _bd_before_n, len(PAIRS), len(_bd_before), len(_bd_after), len(_bd_under)), flush=True)
if R32:

    for u in list(U):
        r = U[u]
        if pair_unit(u) or u.startswith(('FIX#', 'TACK#')) or r['net'] in IMP_TAB or (NC['net_list'].get(r['net']) or {}).get('category') not in ('signal', 'fast', 'analog'): continue
        widths = [w for t in r['tracks'] for w in t['w']]
        goal = max(TRACK_MIN, min(widths)) if widths else TRACK_MIN
        if widths and any(abs(w - goal) > 1e-9 for w in widths):

            tr = [dict(t, w=[goal] * len(t['w'])) for t in r['tracks']]
            del_unit(u); add_unit(u, r['net'], tr, r['vias'])

if QUIET:
    _bw = [0, 0, 0]
    for u in list(U):
        if pair_unit(u) or u.startswith(('FIX#', 'TACK#')): U[u] = U.pop(u); continue
        _bw = [a + b for a, b in zip(_bw, width_calm(u, ucls(u)))]
    print('Width profile smoothed (quiet_r=%g): %d tracks narrower, width changes %d -> %d' % (QUIET_R, _bw[2], _bw[0], _bw[1]), flush=True)

if PAD_CENTER:
    _ns = joints_repair() if args.get('joint_fix', '1') == '1' else 0
    if _ns: print('Joints (R24): %d track ends set exactly on the centerline of their own track at the end' % _ns, flush=True)
    _net_copper = collinear_together() if args.get('collinear', '1') == '1' else 0
    print('Joints (R24): %d collinear support points merged; while routing (all passes)' % _net_copper, JOINT_STAT, flush=True)


if args.get('via_fine_shift') == '1' and NC and not fail and not exact_conflicts() and not time_tight(): via_fine_shift()
LENGTHS_REPORT = None
if (args.get('length_list', '0') == '1' and SI_FIELDS or PAIR_NETS) and (PAIR_SEARCH_SPACE or not fail and not exact_conflicts()) and not time_tight(0.95):
    import os as _os
    sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    if PAIR_NETS: pair_couple()
    LENGTHS_REPORT = lengths_match(only_pairs=args.get('length_list', '0') != '1')
    print('Length matching (option): %d meanders, %d attempts' % (len(LENGTHS_REPORT['move_records']), LENGTHS_REPORT['attempts']), flush=True)

VD_AFTER_LOG = dict(attempts=0, accepted=0, move_records=[], cpu_s=0.0)
if VD_AFTER and not fail and not exact_conflicts() and not time_tight(): via_density_post_run()

if CURRENT_FIRST:


    for u,r in list(U.items()):
        if current_model(r['net']) and not u.startswith(('FIX#','TACK#')) and not current_final(u,ucls(u)): fail.add(u)

REST_REPORT = None
if args.get('remaining_conflicts', '0') == '1':
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from postprocessing.remaining_conflicts import solve_vias
    REST_REPORT = solve_vias(globals())
    print('R45: %d via moves accepted, %.2f CPU-s' % (REST_REPORT['accepted'], REST_REPORT['cpu_s']), flush=True)


def pin_access_rule_values(net):
    from analysis.coupling import coupling, sets
    from analysis.topology import topology
    tracks, vias = export_geometry()
    own = [t for t in tracks if t['net'] == net]
    victim, aggressor = sets(NC)
    k = 0.
    if net in victim or any(t.get('kelvin') for t in own):
        r = dict(tracks=own + [t for t in tracks if t['net'] != net and t['net'] in aggressor])
        k += coupling(r, NC, victim={net}, aggressor=aggressor)['total_sum']
    if net in aggressor:
        r = dict(tracks=own + [t for t in tracks if t['net'] != net and (t['net'] in victim or t.get('kelvin'))])
        k += coupling(r, NC, victim=victim - {net}, aggressor={net: aggressor[net]})['total_sum']
    r = dict(tracks=own, vias=[v for v in vias if v['net'] == net], clr=DRCC)
    t = topology(r, dict(D, pads=[p for p in D['pads'] if p['net'] == net]))
    keys = ('parallel_rest_mm', 'star', 'sharp', 'kelvin_close_mm', 'tap_via_in_pad')
    return dict(coupling=k, **{key:t[key] for key in keys},
                star_free=t['star']-t['star_pad'], sharp_free=t['sharp']-t['sharp_pad'])


def pin_access_rule_assumption(before, post):
    return all(math.isfinite(post[k]) and post[k] <= v + 1e-6 for k,v in before.items())

def pin_access_post_run():
    global _LOOP_DEADLINE
    report = dict(repair=[], attempts=[], accepted=0, cpu_s=0., budget_s=0.)
    cpu_start = time.process_time()
    start = budget_clock()
    budget = max(0., start / 9.)
    if BUDGET_WORK: budget = min(budget, max(0., .90 * WORK_MAX - work_total()))
    elif CPU_MAX > 0: budget = min(budget, max(0., .90 * CPU_MAX - search_cpu()))
    report['budget_work' if BUDGET_WORK else 'budget_s'] = budget

    if budget < .1: return report
    old_deadline = _LOOP_DEADLINE
    deadline = start + budget
    old_hist, old_pf, old_hard = hist.copy(), PF[0], HARD_FOREIGN[0]
    old_active = _PZ_BUDGET_ACTIVE[0]
    try:
        hist[:] = 0; PF[0] = 1e3; HARD_FOREIGN[0] = True
        _PZ_BUDGET_ACTIVE[0] = True; _LOOP_DEADLINE = deadline

        removed = []
        if PZ_REST:
            _loop_cpu_check()
            conflicts = exact_conflicts()
            tr, vi = export_geometry()
            safe = r44_export(dict(tracks=tr, vias=vi, failed=sorted(fail), conflicts=conflicts))
            removed = (safe.get('short_circuit_protection') or {}).get('units_removed', [])


            removed = [u for u in removed if u in U and u not in FIXED and not u.startswith(('FIX#','TACK#'))]
            for u in removed:
                del_unit(u); fail.add(u)
            _CAND_IDX[0] = None


            order = sorted(removed,key=lambda u:(-len(by_net.get(u,[])),u))
            if order:
                order = order[:1] + sorted(order[1:],key=lambda u:(u.startswith('GND#'),len(by_net.get(u,[])),cls(U[u]['net'] if u in U else u).get('prio',10),u))
            repair_end = start + budget * .55
            for index, u in enumerate(order):
                if budget_clock() >= repair_end: break
                _LOOP_DEADLINE = min(repair_end,budget_clock()+budget*(.20 if index==0 else .07))
                entry = dict(unit=u, taken=False)
                local_before = {n:set(ps) for n,ps in PZ_LOCAL.items()}
                _PZ_PARTIAL[0] = None
                design_rules0 = None
                try:
                    net = D['pads'][int(u[4:])]['net'] if u.startswith('GND#') else u
                    design_rules0 = pin_access_rule_values(net)
                    ok = route_unit(u); candidate = U.get(u)
                    valid = bool(candidate and (candidate['tracks'] or candidate['vias']))
                    valid = valid and _pz_geometry_ok(candidate['net'], candidate['tracks'], candidate['vias'], False, u)
                    _CAND_IDX[0] = None
                    valid = valid and unit_conflict_free(u)
                    if valid:
                        design_rules1 = pin_access_rule_values(net)
                        entry.update(design_rules_before=design_rules0, design_rules_after=design_rules1)
                        valid = pin_access_rule_assumption(design_rules0, design_rules1)
                        if not valid: entry['reason'] = 'Topology/coupling worse'
                    _loop_cpu_check()
                    if valid:
                        entry.update(taken=True, complete=bool(ok))
                        if ok: fail.discard(u)
                except (TimeoutError, _PZBudget):
                    entry['reason'] = 'CPU-Budget'


                    partial = _PZ_PARTIAL[0]
                    _LOOP_DEADLINE = repair_end
                    try:
                        _loop_cpu_check()
                        if partial and partial[0] == u and partial[2]:
                            add_unit(u,partial[1],partial[2],partial[3])
                            candidate = U[u]; _CAND_IDX[0] = None
                            valid = _pz_geometry_ok(candidate['net'],candidate['tracks'],candidate['vias'],False,u)
                            valid = valid and unit_conflict_free(u)
                            if valid and design_rules0 is not None:
                                design_rules1 = pin_access_rule_values(net)
                                entry.update(design_rules_before=design_rules0, design_rules_after=design_rules1)
                                valid = pin_access_rule_assumption(design_rules0, design_rules1)
                                if not valid: entry['reason'] = 'Topology/coupling worse'
                            else: valid = False
                            _loop_cpu_check()
                            if valid: entry.update(taken=True,complete=False,subtree=True)
                    except Exception as error:
                        entry['checking'] = repr(error)[:180]
                except Exception as error:
                    entry['reason'] = 'Candidate rejected: ' + repr(error)[:180]
                finally:
                    _PZ_PARTIAL[0] = None
                    if not entry['taken']:
                        del_unit(u)
                        PZ_LOCAL.clear(); PZ_LOCAL.update(local_before)
                        _pad_room_cached.cache_clear()
                    report['repair'].append(entry); _CAND_IDX[0] = None
                if budget_clock() >= repair_end: break
        _LOOP_DEADLINE = deadline


        if PZ_MINI and (removed or PZ_LOCAL):
            eligible = [u for u in units if u in U and u not in FIXED and u not in fail
                        and not u.startswith(('GND#','FIX#','TACK#')) and not pair_unit(u)
                        and u not in PAIR_NETS and U[u]['net'] in PZ_WIDTHS and net_mst(U[u]['net']) > .000001]
            for u in sorted(eligible, key=lambda u: (-(unit_len(u)-.6*n_sv(U[u]))/net_mst(U[u]['net']), u))[:5]:
                _loop_cpu_check()
                old = copy.deepcopy(U[u]); l0 = sum(math.dist(a,b) for t in old['tracks'] for a,b in zip(t['pts'][:-1],t['pts'][1:]))
                entry = dict(net_name=old['net'], before_mm=l0, q_before=l0/net_mst(old['net']), taken=False)
                local_before = {n:set(ps) for n,ps in PZ_LOCAL.items()}
                try:
                    design_rules0 = pin_access_rule_values(old['net'])
                    del_unit(u)
                    ok = route_unit(u); candidate = U.get(u)
                    l1 = sum(math.dist(a,b) for t in candidate['tracks'] for a,b in zip(t['pts'][:-1],t['pts'][1:])) if candidate else float('inf')
                    entry['after_mm'] = l1 if math.isfinite(l1) else None
                    ok = ok and l1 < l0-.01 and n_sv(candidate) <= n_sv(old)
                    ok = ok and min((w for t in candidate['tracks'] for w in t['w']),default=TRACK_MIN) >= min((w for t in old['tracks'] for w in t['w']),default=TRACK_MIN)-1e-9
                    ok = ok and _pz_geometry_ok(old['net'],candidate['tracks'],candidate['vias'],False,u)
                    _CAND_IDX[0] = None
                    ok = ok and unit_conflict_free(u)
                    if ok:
                        from analysis.evaluation import not_connected as _pz_unconnected
                        b = dict(D,pads=[p for p in D['pads'] if p['net']==old['net']])
                        r = dict(tracks=[dict(t,net=old['net']) for t in candidate['tracks']],
                                 vias=[dict(v,net=v.get('net') or old['net'],d=VIA_D,drill=VIA_DR,layers=LAYERS) for v in candidate['vias']])
                        ok = not _pz_unconnected(b,r,set())
                    if ok:
                        design_rules1 = pin_access_rule_values(old['net'])
                        entry.update(design_rules_before=design_rules0, design_rules_after=design_rules1)
                        ok = pin_access_rule_assumption(design_rules0, design_rules1)
                        if not ok: entry['reason'] = 'Topology/coupling worse'
                    _loop_cpu_check()
                    if ok: entry['taken']=True; report['accepted']+=1
                except (TimeoutError, _PZBudget):
                    entry['reason']='CPU-Budget'
                except Exception as error:
                    entry['reason']='Candidate rejected: ' + repr(error)[:180]
                finally:
                    if not entry['taken']:
                        del_unit(u); add_unit(u,old['net'],old['tracks'],old['vias'])
                        PZ_LOCAL.clear(); PZ_LOCAL.update(local_before)
                        _pad_room_cached.cache_clear()
                    _CAND_IDX[0]=None; report['attempts'].append(entry)
                if entry.get('reason')=='CPU-Budget': break
    except (TimeoutError, _PZBudget):
        report['abort']='CPU-Budget'
    finally:
        hist[:]=old_hist; PF[0]=old_pf; HARD_FOREIGN[0]=old_hard
        _PZ_BUDGET_ACTIVE[0]=old_active; _LOOP_DEADLINE=old_deadline
        report['cpu_s']=time.process_time()-cpu_start
        if BUDGET_WORK: report['work'] = work_total()-start
        report['search_kernel']='atomic search jobs' if BUDGET_WORK else 'interruptible Python core'
    return report


PZ_MINIMAX = dict(attempts=[], accepted=0)
work_mark('before_pin_access')
if PZ_REST or PZ_MINI:
    PZ_MINIMAX = pin_access_post_run()
    print('Pin access minimax:', PZ_MINIMAX, flush=True)


if PZ_LOCAL:
    for u,r in list(U.items()):
        if u in FIXED or u.startswith(('FIX#','TACK#')) or not _pz_active(r['net']):continue
        if not _pz_geometry_ok(r['net'],r['tracks'],r['vias'],False,u):
            del_unit(u);add_unit(u,r['net'],[],[]);fail.add(u)

_cf_final = exact_conflicts()
if _ET and not _LEARN_ON: _ET.observe(_cf_final, MAXIT + 1, 'post_run')
if _LEARN_ON: negotiation_measure(MAXIT + 1, _cf_final, 'final')
conf = {u: len(v) for u, v in _cf_final.items()}
print('End: conflicts', conf, 'incomplete', sorted(fail))
json.dump(PAIRS, open(OUT.replace('.json', '_conflicts.json'), 'w'))
tracks, vias = export_geometry()
if PAIR_SEARCH_SPACE: print('Shared pair search space:', _PAIR_SEARCH_INFO, flush=True)
if TAP_ON or TAP_TREE or BRANCH: print('Topology (R27, total over all passes):', TOPO_STAT, flush=True)

VGB = []
for u, r in U.items():
    for v in r['vias']:
        if v.get('n'):
            actual = 1 + sum(1 for w in r['vias'] if w.get('too') == [v['x'], v['y']])
            VGB.append(dict(net_name=r['net'], x=v['x'], y=v['y'], nominal=v['n'], actual=actual))
RVB = rv_report()
PAB = pairs_report(receiver=True) if PAIR_NETS else []
if PAB: print('Differential pairs (R29): %s' % ['%s/%s %s/%s mm, skewed %s, vias %s, coupled %.0f %%' % (
    b['net_list'][0].split('/')[-1], b['net_list'][1].split('/')[-1], b['total_length'][0], b['total_length'][1], b['skewed_mm'], b['vias'], 100 * b['coupled']) for b in PAB], flush=True)
if RVB: print('Return vias (Q2): %d layer changes of fast nets with different ground reference area, %d with ground via in reach (%d own return via), %d without %s; %s' % (
    RVB['change'], RVB['with_ret'], RVB['own_ret_via'], len(RVB['without']), [(b['net_name'], b['spacing']) for b in RVB['without'][:6]], RVB['stat']), flush=True)
if VGB: print('Via groups (K10): %d layer changes with current, %d extra vias, too little room at %d spots' % (
    len(VGB), sum(1 for r in U.values() for v in r['vias'] if v.get('gr')), sum(1 for b in VGB if b['actual'] < b['nominal'])), flush=True)
if CHAIN and CHAINS:
    _kb = [t for r in U.values() for t in r['tracks'] if t.get('link_chain')]
    print('Supply chains (R22): %d tracks as links, %d pads without chain or without keepout zone (no way)' % (len(_kb), len(_METRIC_OFF)), sorted(_METRIC_OFF), CHAIN_STAT, flush=True)

UNDER = under_wmin()
if UNDER:
    _uf = [e for e in UNDER if not e['allowed']]
    print('Below minimum width: %d pieces (%.1f mm) in %d nets%s; plus %d allowed in neck or dense field (under_wmin in the output)' % (
        len(_uf), sum(e['total_length'] for e in _uf), len({e['net_name'] for e in _uf}),
        ', narrowest %.3f instead of %.3f mm (%s)' % min((e['width'] - e['nominal'], e['width'], e['nominal'], e['net_name']) for e in _uf)[1:] if _uf else '',
        len(UNDER) - len(_uf)), flush=True)
if UNREACHABLE: print('Unreachable (hard obstacles, pad has no exit):', {n: v for n, v in sorted(UNREACHABLE.items())})

MPV = []
for _e in EPG:
    _actual = sum(1 for r in U.values() if r['net'] == _e['net_name'] for v in r['vias'] if _e['poly'].contains(_PT(v['x'], v['y'])))
    _ges = False
    if any(_k in EPM for _k in _e['pads']):
        _ges = not any(via_hard(_e['net_name'], EPM[_k]['grid_cells']) for _k in _e['pads'] if _k in EPM)
    MPV.append(dict(part=_e['ref'], pad=_e['num'], net_name=_e['net_name'], surface=_e['surface'], fields=_e['fields'], nominal=_e['nominal'], actual=_actual,
                    **({'reason': 'blocked'} if _ges else {})))
if MPV: print('Ground pad vias (R16): %d ground pads, target %d vias, actual %d, at %d pads fewer than target %s' % (
    len(MPV), sum(m['nominal'] for m in MPV), sum(m['actual'] for m in MPV), sum(1 for m in MPV if m['actual'] < m['nominal']),
    ['%s:%s %d/%d%s' % (m['part'], m['pad'], m['actual'], m['nominal'], ' (blocked: no allowed room in pad)' if m.get('reason') else '') for m in MPV]), flush=True)


VAPB = []
if VAP:
    _count = dict(in_pad=0, third=0, beside=0)
    for u, r in U.items():
        if not u.startswith('GND#') or int(u[4:]) in EPM or not r['vias']: continue
        _k = int(u[4:]); _c, _w = _vap_cells(_k); _v = r['vias'][0]
        _vc = int((_v['y'] - Y0) / RES) * NX + int((_v['x'] - X0) / RES); _j = np.flatnonzero(_c == _vc)
        if len(_j): _count['in_pad' if _w[_j[0]] == 0.0 else 'third'] += 1; continue
        _count['beside'] += 1; _p = D['pads'][_k]
        _hard = via_hard(r['net'], _c)
        VAPB.append(dict(part=_p['ref'], pad=_p['num'], net_name=r['net'], spacing=round(math.hypot(_v['x'] - _p['x'], _v['y'] - _p['y']), 3),
                         reason='occupied' if _hard else 'blocked', important=_k in VAP_IMPORTANT))
    print('Via at pad (R9): %d stubs with via from half inside the pad, %d from one third, %d beside it (of which %d blocked, %d occupied) %s' % (
        _count['in_pad'], _count['third'], _count['beside'], sum(1 for b in VAPB if b['reason'] == 'blocked'),
        sum(1 for b in VAPB if b['reason'] == 'occupied'), ['%s:%s %s' % (b['part'], b['pad'], b['reason']) for b in VAPB]), flush=True)
BGB = []
if BGA_FANOUT:
    for p in D['pads']:
        b = BGA.get(id(p))
        if b is None or not p['net'] or p['net'].startswith('unconnected'): continue

        near = [v for v in vias if v['net'] == p['net'] and math.dist((v['x'], v['y']), (p['x'], p['y'])) <= b['pitch'] * 0.72]
        BGB.append(dict(ref=b['ref'], pad=b['pad'], ring=b['ring'], pitch=b['pitch'],
                        via_close=bool(near), via_in_pad=any(math.dist((v['x'], v['y']), (p['x'], p['y'])) < 0.02 for v in near)))


_rest_final = export_data(tracks, vias, conf, raw_data=True,
    **({'pair_search_space_info': _PAIR_SEARCH_INFO} if PAIR_SEARCH_SPACE else {}),
    **({'final': 'early_abort' if _EARLY_INFO is not None else ('cpu_budget' if time_tight() else 'finished'),
        'learning_state': json.loads(_LEARN_SNAPSHOT)['learning_state'], 'learning_budget':json.loads(_LEARN_SNAPSHOT)['learning_budget']} if _LEARN_ON else {}))
if args.get('remaining_conflicts') == '1' and args.get('rest_parts', '1') == '1' and REST_REPORT and NC and args.get('netclasses'):
    import tempfile
    from postprocessing.remaining_conflicts import parts_post_run, repair_keepout
    _rest_budget = max(0, float(args.get('rest_cpu', '30')) - REST_REPORT['cpu_s'])
    if CPU_MAX > 0: _rest_budget = min(_rest_budget, max(0, CPU_MAX-search_cpu()))
    try:
        with tempfile.TemporaryDirectory(prefix='autorouter_r45_final_') as _rest_tmp:
            _rp = Path(_rest_tmp) / 'routes.json'
            _bp = Path(_rest_tmp) / 'board.json'
            _rp.write_text(json.dumps(_rest_final)); _bp.write_text(Path(sys.argv[1]).read_text())
            _rest_block = repair_keepout(globals().get('ST') or {})
            if _rest_block:

                _rest_budget = 0
            _rp.with_name('routes_conflicts.json').write_text(json.dumps(PAIRS))
            _rest_parts = parts_post_run(str(_bp), args['netclasses'], str(_rp), _rest_budget)
            if _rest_block:
                _rest_parts['reason'] = _rest_block
            _rest_final = json.loads(_rp.read_text()); _rest_final['rest_parts'] = _rest_parts
        print('R45 parts: %d moves, %.2f CPU-s' % (_rest_parts['accepted'], _rest_parts['cpu_s']), flush=True)
    except Exception as _rest_error:
        _rest_final['rest_parts'] = dict(accepted=0, reason='Check failed: '+repr(_rest_error)[:180])
_safe = rest_final(_rest_final)
if args.get('skew', '0') == '1':
    _safe = skew_match(D, _safe, args, Path(OUT).with_suffix('.skew'))

work_emergency_stop()
if WORK_MEASURE:
    work_mark('finished')
    _safe['work_budget'] = work_evidence()
if CHAMPION_EVIDENCE is not None: _safe['champion'] = CHAMPION_EVIDENCE
json.dump(_safe, open(OUT, 'w'))
if _safe.get('rest_board'):
    Path(OUT).with_name(Path(OUT).stem+'_rest_board.json').write_text(json.dumps(_safe['rest_board']))
    Path(OUT).with_name(Path(OUT).stem+'_rest_placement.json').write_text(json.dumps(_safe['rest_placement']))
print('Tracks', len(tracks), 'Vias', len(vias), 'Time %.0fs' % (time.time() - t_start))
if _ET: _ET.write_out(_safe['tracks'], _safe['vias'], absent=_safe['failed'], zone_list=_safe.get('zone_list', []) + _safe.get('zones', []))
if _safe.get('short_circuit_protection'):
    print('R44: output %d tracks, %d vias, open %s' % (len(_safe['tracks']), len(_safe['vias']), _safe['short_circuit_protection']['nets_removed']), flush=True)
if 'png' in args:
    _png_export = args.get('short_circuit_protection', '0') == '1' or args.get('skew', '0') == '1'
    for li, L in enumerate(LAYERS):
        img = np.zeros((NY, NX, 3), dtype=np.uint8); img[inside] = (235, 235, 230)
        if li in (0, NL - 1): img[PD1[li] <= 0.0] = (150, 150, 150)
        rng = np.random.RandomState(1); cols = rng.randint(40, 220, size=(len(nets) + 1, 3)); [cols.__setitem__(NID[_g], (60, 60, 60)) for _g in GNDS if _g in NID]
        if not _png_export:
            for u, r in U.items():
                c = r['cells'][(r['cells'] >= li * NN) & (r['cells'] < (li + 1) * NN)] - li * NN
                img.reshape(-1, 3)[c] = cols[NID[r['net']]]
        in_ = Image.fromarray(img).resize((NX * 2, NY * 2), Image.NEAREST)
        dr = ImageDraw.Draw(in_)
        xy = lambda p: ((p[0]-X0)/RES*2, (p[1]-Y0)/RES*2)
        for t in (_safe['tracks'] if _png_export else []):
            if t['layer'] != L: continue
            c = tuple(int(q) for q in cols[NID[t['net']]])
            for a,b,w in zip(t['pts'][:-1],t['pts'][1:],t['w']): dr.line([xy(a),xy(b)], fill=c, width=max(1,round(w/RES*2)))
        for v in (_safe['vias'] if _png_export else []):
            x,y=xy([v['x'],v['y']]); r=VIA_R/RES*2
            dr.ellipse([x-r,y-r,x+r,y+r],fill=(40,40,40))
        for e in _safe.get('ratsnest', []): dr.line([xy(p) for p in e['pts']],fill=(230,120,20),width=1)
        if not _png_export:
            for (lay, dist, ua, ub, x, y) in PAIRS_LAST:
                if lay not in (L, '*'): continue
                cx, cy = (x - X0) / RES * 2, (y - Y0) / RES * 2
                dr.ellipse([cx - 14, cy - 14, cx + 14, cy + 14], outline=(255, 0, 0), width=3)
        in_.save(args['png'] + '_' + L + '.png')

if _safe.get('rest_board') and 'png' in args and NC:
    from analysis.evaluation import image
    image(_safe['rest_board'], _safe, NC, args['png'])
if BEND > 0 and NB is not None:
    print('Bend search: %d calls, %d removals, %d dominated, %.6f CPU-s' %
          (_BEND_CALLS, _BEND_POP, _BEND_DOM, _BEND_CPU), flush=True)
