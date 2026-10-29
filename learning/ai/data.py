import contextlib
import datetime
import fcntl
import functools
import hashlib
import json
import math
import os
import re
from pathlib import Path
import tempfile
import time
import uuid

import numpy as np
from learning.ai.features_raster import features_raster, substitute_ground

VERSION = 2
OBSERVER_VERSION = 3
READ_CONTRACT = 'observer-v3-search_occupancy-v3'
GOAL_HASH_VERSION = 5
ROOT = Path(__file__).resolve().parents[2]
STANDARD = Path.home() / 'Library/Caches/Autorouter/dataset'
X_COLUMNS = ('dc', 'j', 'ret', 'xt_victim', 'xt_disturb', 'total_length', 'detour', 'vias',
             'c_load', 'l_load', 'sw_area', 'hot_L', 'hot_A', 'ground',
             'eye_v', 'eye_h', 'eye_skew', 'disturb', 'power')


def folder():
    return Path(os.environ.get('AUTOROUTER_DATASET', str(STANDARD))).expanduser()


def board_hash(B):
    return hashlib.sha256(json.dumps(B, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


@functools.lru_cache(maxsize=64)
def _check_hash(path, stamp, size):
    b = Path(path).read_bytes()
    return hashlib.sha256(b).hexdigest(), board_hash(json.loads(b))


def excluded(path, B=None):
    p = Path(path); raw = hashlib.sha256(p.read_bytes()).hexdigest()
    canon = board_hash(B if B is not None else json.loads(p.read_bytes()))
    from learning.learning_curve import FOLDER, LEVELS, SEEDS
    hashes = set()
    for st in LEVELS:
        for seed in SEEDS:
            ref = Path(FOLDER) / 'tasks' / ('s%d_%d' % (st, seed)) / 'board.json'
            stat = ref.stat()
            hashes.update(_check_hash(str(ref), stat.st_mtime_ns, stat.st_size))
    return raw in hashes or canon in hashes


def _design_name(f):

    f = re.sub(r'^\d+_', '', str(f)).lower().removesuffix('_tight')
    seed = re.match(r'erf_\d+', f)
    if seed: return seed.group()
    t = f.split('_')
    return '_'.join(t[:2]) if t[0] == 'kitspace' else t[0]

def family(z):
    if z.get('design_family'): return _design_name(z['design_family'])
    if z.get('task') is not None: return 'erf_%06d' % z['task']
    return _design_name(z.get('family') or z['board'])


@functools.lru_cache(maxsize=1)
def test_families():
    p = Path(__file__).with_name('families_v1.json')
    data = json.loads(p.read_text())

    ids = data['test_families']
    return frozenset(ids)


def split_family(f):
    f = _design_name(f)
    if _design_name(f) in {_design_name(t) for t in test_families()}: return 'test'
    h = int(hashlib.sha256(('data_factory-split-v1:' + f).encode()).hexdigest()[:8], 16) % 10
    return 'test' if h == 0 else 'val' if h == 1 else 'train'


@contextlib.contextmanager
def keepout(root, exclusive=True):
    root.mkdir(parents=True, exist_ok=True)
    with open(root / '.index.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH)
        try: yield
        finally: fcntl.flock(lock, fcntl.LOCK_UN)


@functools.lru_cache(maxsize=8)
def _code(stamps):
    h = hashlib.sha256()
    for p, _, _ in stamps:
        h.update(str(Path(p).relative_to(ROOT)).encode()); h.update(Path(p).read_bytes())
    return h.hexdigest()


def code_hash():
    paths = sorted(p for d in ('learning', 'router', 'placement', 'analysis', 'common', 'kicad', 'postprocessing')
                   for p in (ROOT / d).rglob('*.py') if '__pycache__' not in str(p))
    return _code(tuple((str(p), p.stat().st_mtime_ns, p.stat().st_size) for p in paths))


def _half(data):
    data = dict(data)
    for k in ('input_', 'router_history_max', 'router_history_final', 'router_history_rounds'):
        if k not in data: continue
        a = np.asarray(data[k], np.float32)
        mx = float(np.max(np.abs(a))) if a.size else 0.
        if not math.isfinite(mx): raise ValueError('non-finite grid map: ' + k)
        scale = max(1., 2. ** math.ceil(math.log2(max(1., mx / 60000.))))
        data[k] = (a / scale).astype(np.float16); data[k + '_scale'] = np.asarray(scale, np.float32)
    return data


def _index_append(root, row):
    path=root/'index.jsonl'
    with open(path,'a+b') as index:
        index.seek(0,2);size=index.tell()
        if size:
            index.seek(size-1)
            if index.read(1)!=b'\n':
                index.seek(max(0,size-65536));start=index.tell();tail=index.read()
                load=tail.rfind(b'\n');cut=start+load+1 if load>=0 else 0
                rest=tail[load+1:]
                try: json.loads(rest);complete=True
                except (ValueError,UnicodeError): complete=False
                if complete:
                    index.write(b'\n')
                else:
                    with open(root/'index_aborted.jsonl','ab') as bad: bad.write(rest+b'\n')
                    index.truncate(cut)
        index.seek(0,2);start=index.tell()
        try:
            index.write((json.dumps(row,sort_keys=True)+'\n').encode())
            index.flush();os.fsync(index.fileno())
        except Exception:
            index.truncate(start)
            raise
        return start


def _source_observer(data):
    observer = int(data.get('router_observer_version', 1))


    return int(data.get('router_source_observer_version', 1 if observer == 4 else observer))


def target_contract(data):
    return dict(label_version=VERSION, goal_hash_version=GOAL_HASH_VERSION,
                router_occupancy_contract=str(data.get('router_occupancy_contract', 'historical_without_contract')),
                router_source_observer_version=_source_observer(data),
                router_observer_version=int(data.get('router_observer_version', 1)),
                router_label_version=int(data.get('router_label_version', 1)))


def goal_hash(data):
    keys = ('router_history_max', 'router_history_final', 'router_history_rounds',
            'router_occupancy', 'router_occupancy_search', 'router_conflict_rounds', 'router_vias_raster',
            'router_net_values', 'router_net_layers', 'router_net_conflicts', 'router_phases',
            'physics_x', 'physics_mask', 'global_goal', 'global_mask',
            'net_joined', 'net_joined_mask',
            'part_conflicts', 'part_conflicts_mask')

    contract = target_contract(data)
    zh = hashlib.sha256(json.dumps(contract, sort_keys=True).encode())

    keys = set(keys) | {k + '_mask_arr' for k in keys}
    for key in sorted(k for k in keys if k in data):
        a = np.asarray(data[key]); zh.update(key.encode()); zh.update(str((a.dtype.str, a.shape)).encode()); zh.update(a.tobytes())
    return zh.hexdigest()


def write_out(data, meta, root=None, before_publish=None):
    root = Path(root or folder()); meta = dict(meta, label_version=VERSION)
    goal_data = data
    meta.update(target_contract(goal_data))
    meta['goal_sha256'] = goal_hash(goal_data)
    f = _design_name(meta['design_family']); meta['design_family'] = f; meta['split'] = split_family(f)
    if meta['provenance'] not in ('selfplay', 'endurance_run', 'test_bench'): raise ValueError('Provenance')
    day = datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%d')
    meta.update(tag_utc=day, id=uuid.uuid4().hex)
    rel = Path('shards') / day / (meta['id'] + '.npz'); dest = root / rel
    dest.parent.mkdir(parents=True, exist_ok=True)
    data = _half(data); data['meta_json'] = np.asarray(json.dumps(meta, sort_keys=True))
    fd, tmp = tempfile.mkstemp(dir=dest.parent, suffix='.npz'); os.close(fd)
    try:
        np.savez_compressed(tmp, **data)
        with open(tmp, 'rb') as file: os.fsync(file.fileno())
        with keepout(root):
            manifest = root / 'split_v1.json'
            expected = dict(version=1, rule='sha256(data_factory-split-v1:family)%10: 0=test, 1=val, otherwise=train',
                            test_families=sorted(test_families()))
            if manifest.exists():
                if json.loads(manifest.read_text()) != expected: raise ValueError('Family manifest changed; new dataset version required')
            else:
                manifest_tmp = root / ('.split-' + meta['id'])
                manifest_tmp.write_text(json.dumps(expected, indent=1)); os.replace(manifest_tmp, manifest)
            while True:
                checksum = hashlib.sha256(Path(tmp).read_bytes()).hexdigest()
                row = dict(meta, shard=str(rel), bytes=Path(tmp).stat().st_size, sha256=checksum,
                           labels_n=sum(int(data[k].size) for k in ('router_history_max', 'router_occupancy',
                                  'router_conflict_rounds', 'router_vias_raster', 'router_net_values', 'router_net_conflicts') if k in data)
                                  + int(np.count_nonzero(data.get('physics_mask', []))) + int(np.count_nonzero(data.get('part_conflicts_mask', []))))
                changed = before_publish is not None and before_publish(data, meta)
                if not changed:
                    os.replace(tmp, dest)
                    try:
                        index_start = _index_append(root, row)
                    except Exception:
                        dest.unlink(missing_ok=True); raise



                    changed = before_publish is not None and before_publish(data, meta)
                    if not changed: break
                    with open(root / 'index.jsonl', 'r+b') as index:
                        index.truncate(index_start); index.flush(); os.fsync(index.fileno())
                    dest.unlink(missing_ok=True)
                meta['goal_sha256'] = goal_hash(goal_data)
                data['meta_json'] = np.asarray(json.dumps(meta, sort_keys=True))
                np.savez_compressed(tmp, **data)
                with open(tmp, 'rb') as file: os.fsync(file.fileno())
        return row
    finally:
        Path(tmp).unlink(missing_ok=True)


def physics(data, z, metrics):
    names = list(data['net_names']); x = np.full((len(names), len(X_COLUMNS)), np.nan, np.float32)
    nodes = (metrics or {}).get('net_list') or {}
    for i, n in enumerate(names):
        for j, key in enumerate(X_COLUMNS):
            v = (nodes.get(n) or {}).get('x', {}).get(key)
            if isinstance(v, (float, int)) and math.isfinite(v): x[i, j] = v
    E = z.get('outcome') or {}; rule = z.get('rule') or {}
    routed = 'router_net_values' in data
    unconn = (metrics or {}).get('not_connected')
    from analysis.current_bottlenecks import learned_values
    r39 = learned_values(rule.get('current_bottlenecks'))
    r39_complete = bool(r39) and (r39.get('coverage') or {}).get('complete', True) and (r39.get('coverage') or {}).get('components') != 0
    plane_nets = set((metrics or {}).get('area_nets') or [])
    data['net_joined'] = np.asarray([n not in (unconn or []) for n in names], bool)
    data['net_joined_mask'] = np.asarray([unconn is not None and n not in plane_nets and bool(n) and not n.startswith('unconnected') for n in names], bool)
    from learning.learning_effect import _limit_result
    known = _limit_result(z) is not None
    from learning.early_abort import aborted
    data.update(physics_x=x, physics_mask=np.isfinite(x), physics_columns=np.asarray(X_COLUMNS),
                global_goal=np.asarray([np.nan if aborted(z) else float(bool(z.get('is_valid'))), z.get('score_v3') if z.get('score_v3') is not None else np.nan], np.float32),
                global_mask=np.asarray([routed and known,
                                         routed and known and z.get('score_v3') is not None], bool),
                physics_meta_json=np.asarray(json.dumps(dict(minimax_status=rule.get('minimax_status'),
                     minimax_valid=rule.get('minimax_valid'), minimax_complete=rule.get('minimax_complete'),
                     minimax_rank=rule.get('minimax_rank'), safety=rule.get('safety'),
                     score_v3_version=rule.get('version'), score_v3_whole=(metrics or {}).get('score_v3_whole'),
                     native_drc=E.get('drc'),
                     topology=rule.get('topology'), r27=rule.get('r27'), coupling=rule.get('coupling'),
                     **({'k20': rule['k20']} if 'k20' in rule else
                        ({'k20': rule['design_rules']['k20']} if 'k20' in (rule.get('design_rules') or {}) else {})),
                     current_bottlenecks=r39,
                     r39_coverage=rule.get('r39_coverage'),
                     current_bottlenecks_mm2=rule.get('current_bottlenecks_mm2') if r39_complete else None,
                     current_limited=z.get('v3_current_limited') or (metrics or {}).get('current_limited'),
                     not_connected=unconn,
                     model_assumptions='analysis/measures; no hardware measurement',
                     shaping=z.get('shaping')), sort_keys=True)))


def capture(c, z, source_board, provenance='selfplay', metrics=None, input_=None, board=None, netclasses=None, labels=None, drc_path=None, cpu_budget=None):
    t0 = time.thread_time(); c = Path(c); r44_cpu = 0.0
    try:
        if excluded(source_board): return dict(cur_state='excluded_learning_curve')
        bj = Path(board) if board else c / 'board.json'
        if not bj.exists(): bj = Path(source_board)
        B = json.loads(bj.read_text()); kj = Path(netclasses) if netclasses else c / 'netclasses.json'
        if not kj.exists(): kj = Path(source_board).parent / 'netclasses.json'
        K = json.loads(kj.read_text()) if kj.exists() else {}
        if excluded(bj, B): return dict(cur_state='excluded_learning_curve')
        gnd = z.get('gnd_layers') or []
        data = dict(input_ or features_raster(B, K, gnd_layers=gnd))
        ep = Path(labels) if labels else c / 'labels.npz'
        if labels is None and not ep.exists(): ep = c / 'routes_labels.npz'
        if ep.exists():
            rp = c / 'routes.json'
            if rp.exists():
                from postprocessing.short_circuit_protection import labels_save, protection_on
                rr = json.loads(rp.read_text())

                if protection_on(rr.get('options')):
                    start = time.thread_time()
                    try: labels_save(B, rr, rp, ep)
                    finally: r44_cpu += time.thread_time()-start
        if ep.exists():
            with np.load(ep, allow_pickle=False) as r:
                if not np.allclose(r['origin_mm'], data['origin_mm']) or r['occupancy'].shape[-2:] != data['input_'].shape[-2:]:
                    raise ValueError('Input/target grid differ')
                for key in r.files: data['router_' + key] = r[key]

            grid = data['router_conflict_rounds']; ls = list(data['router_layers']); origin = data['origin_mm']
            near = np.zeros(len(data['part_names']), np.uint32)
            from learning.ai.features_raster import geometry, times
            courts = {q['ref']: q for q in B.get('courtyards', [])}
            for i, (x, y, _, side, *_) in enumerate(data['part_values']):
                lay = str(data['cu_layers'][int(side)])
                if lay not in ls: continue
                cy_geo = courts.get(str(data['part_names'][i]))
                if cy_geo:
                    m = times(geometry(cy_geo['polys']).buffer(1), origin, grid.shape[-2:])
                    near[i] = grid[ls.index(lay)][m].sum()
                    continue
                cx, cy = np.floor((np.array([x, y]) - origin) / .25).astype(int)
                yy, xx = np.mgrid[max(0, cy - 4):min(grid.shape[1], cy + 5), max(0, cx - 4):min(grid.shape[2], cx + 5)]
                if xx.size:
                    m = (xx - cx) ** 2 + (yy - cy) ** 2 <= 16
                    near[i] = grid[ls.index(lay), yy[m], xx[m]].sum()
            data['part_conflicts'] = near
            data['part_conflicts_mask'] = np.ones(len(near), bool)
        else:
            data['part_conflicts'] = np.zeros(len(data['part_names']), np.uint32)
            data['part_conflicts_mask'] = np.zeros(len(data['part_names']), bool)
        if cpu_budget is not None:
            from learning.learning_effect import complete
            cpu_budget.logbook(z)
            complete(z, c)
        physics(data, z, metrics)
        if z.get('learning_data'):
            data['learning_data_json'] = np.asarray(json.dumps(z['learning_data'], sort_keys=True))
            data['board_json'] = np.asarray(json.dumps(B, sort_keys=True))
            data['netclasses_json'] = np.asarray(json.dumps(K, sort_keys=True))
            for name in ('routes', 'placement'):
                raw = c / (name + '.json')
                if raw.exists(): data[name + '_json'] = np.asarray(raw.read_text())


            if z['learning_data'].get('censored'):
                data['global_mask'][:] = False
        drc = Path(drc_path) if drc_path else c / 'finished.kicad_pcb.drc.json'
        if drc.exists(): data['native_drc_json'] = np.asarray(drc.read_text())
        rg_path = c / 'design_rules.json'
        if rg_path.exists(): data['design_rules_json'] = np.asarray(rg_path.read_text())
        meta = dict(task=z.get('task', z.get('board')), design_family=family(z), seed=z.get('cfg', {}).get('seed'),
                    candidate=z.get('candidate'), code_sha256=z.get('code_sha256') or code_hash(),
                    router_sha256=hashlib.sha256((ROOT / 'router/route_pf.py').read_bytes()).hexdigest(),
                    netclasses_sha256=hashlib.sha256((ROOT / 'analysis/netclasses.py').read_bytes()).hexdigest(),
                    provenance=provenance, board_sha256=hashlib.sha256(bj.read_bytes()).hexdigest(),
                    source_board_sha256=hashlib.sha256(Path(source_board).read_bytes()).hexdigest(),
                    cfg=z.get('cfg'), cpu_s=sum((z.get(p) or {}).get('cpu_s', 0) or 0 for p in ('placer', 'router')),
                    phase='routing' if ep.exists() else 'placement', cur_state=z.get('cur_state'),
                    code_changed=z.get('code_changed', False), substitute_ground=z.get('substitute_ground'),
                    comparison_group=z.get('comparison_group'), pair_with=z.get('pair_with'),
                    collector_cpu_s=time.thread_time() - t0, r44_cpu_s=r44_cpu)
        def budget_final(data, meta):


            before = bool(z.get('cpu_time_limit'))
            cpu_budget.logbook(z)
            if before == bool(z.get('cpu_time_limit')): return False
            complete(z, c)
            from learning.early_abort import aborted
            data['global_goal'][0] = np.nan if aborted(z) else 0
            data['global_mask'][:] = False
            data['learning_data_json'] = np.asarray(json.dumps(z['learning_data'], sort_keys=True))
            meta['cur_state'] = z['cur_state']
            return True
        row = write_out(data, meta, **({'before_publish': budget_final} if cpu_budget is not None else {}))
        return dict(cur_state='ok', shard=row['shard'], bytes=row['bytes'], labels_n=row['labels_n'], goal_sha256=row.get('goal_sha256'), goal_hash_version=row.get('goal_hash_version', GOAL_HASH_VERSION), router_observer_version=row.get('router_observer_version'),
                    cpu_s=time.thread_time() - t0, r44_cpu_s=r44_cpu)
    except Exception as e:
        return dict(cur_state='faults', faults=repr(e)[:250], cpu_s=time.thread_time() - t0, r44_cpu_s=r44_cpu)


def index_read(root):
    root = Path(root)
    with keepout(root, exclusive=False):
        raw = (root / 'index.jsonl').read_bytes()
    rows = []
    for line in raw.splitlines(keepends=True):
        try: row = json.loads(line)
        except ValueError:
            if not line.endswith(b'\n'): break
            raise
        rows.append(row)
    return rows


def _old_targets(data, row, observer):
    phases = data.get('router_phases')
    prep = phases is None or bool(np.any(phases == 'initial_build'))
    if phases is not None:
        keep = phases != 'initial_build'
        for key in ('router_phases', 'router_passes', 'router_net_conflicts', 'router_history_rounds'):
            if key in data:
                if len(data[key]) != len(keep): raise ValueError('Observer rounds unequal')
                data[key] = data[key][keep]
    if prep:
        if 'router_history_rounds' in data and len(data['router_history_rounds']):
            data['router_history_max'] = data['router_history_rounds'].max(axis=0)
        elif 'router_history_max' in data:
            data['router_history_max_mask'] = np.zeros_like(data['router_history_max'], bool)
        for key in ('router_conflict_rounds', 'part_conflicts'):
            if key in data: data[key + '_mask_arr'] = np.zeros_like(data[key], bool)
    if 'router_net_values' in data:
        mask = np.asarray(data.get('router_net_values_mask', np.isfinite(data['router_net_values'])), bool).copy()
        if observer < 2 or prep: mask[:, 6] = False
        data['router_net_values_mask'] = mask
    data['router_source_observer_version'] = np.asarray(observer)
    data['router_observer_version'] = np.asarray(OBSERVER_VERSION)
    data['read_contract'] = np.asarray(READ_CONTRACT)


def _global_masks(data, row):
    if 'global_mask' not in data: return
    from learning.learning_effect import _limit_result
    z = dict(row)
    for key in ('learning_data_json', 'physics_meta_json'):
        if key in data:
            info = json.loads(str(data[key]))
            if key == 'learning_data_json':
                z['learning_data'] = info
                if info.get('faults'): z['faults'] = info['faults']
                z.update(info.get('compute_runs') or {})
            else: z['shaping'] = info.get('shaping') or {}
    targets = data.get('global_goal')
    if targets is not None:
        from learning.early_abort import aborted
        if aborted(z):
            targets[0] = np.nan
        data['global_mask'] = np.asarray(data['global_mask'], bool) & np.isfinite(targets)
        if targets[0] in (0, 1): z['is_valid'] = bool(targets[0])
    if _limit_result(z) is None: data['global_mask'] = np.zeros_like(data['global_mask'], bool)
    if (z.get('learning_data') or {}).get('censored') and 'router_net_values' in data:
        mask = np.asarray(data.get('router_net_values_mask', np.isfinite(data['router_net_values'])), bool).copy()
        mask[:, 0] = False
        data['router_net_values_mask'] = mask


def _r44_search_occupancy(data):
    data['router_occupancy_export'] = data['router_occupancy']
    prior = data.pop('router_occupancy_mask', None)
    if prior is not None: data['router_occupancy_export_mask'] = prior
    data['router_occupancy'] = data['router_occupancy_search']
    if 'router_occupancy_search_mask' in data: data['router_occupancy_mask'] = data['router_occupancy_search_mask']
    elif prior is not None: data['router_occupancy_mask'] = np.asarray(prior, bool).copy()
    data['router_occupancy_contract'] = np.asarray('search_occupancy_share')


class RasterDataset:
    def __init__(self, root=None, split='train', version=None, phase='routing', check_hash=False, observer_version=OBSERVER_VERSION, rows=None):
        self.root = Path(root or folder()); self.rows = []; self.check_hash = check_hash
        self.observer_version = observer_version


        versions = {1, VERSION} if version is None else {version}
        cohorts = {}
        if split not in ('train', 'val', 'test', 'all_items'): raise ValueError('split')
        if rows is None: rows = index_read(self.root)
        for row in rows:
            if row['split'] != split_family(row['design_family']): raise ValueError('Family leak in the index')
            observer = row.get('router_observer_version') if row['phase'] == 'routing' else None
            selected = (not row.get('code_changed') and row['label_version'] in versions
                        and (not phase or row['phase'] == phase)
                        and (split == 'all_items' or row['split'] == split))
            if selected and row['phase'] == 'routing':
                if observer_version == OBSERVER_VERSION:
                    selected = observer in (None, 1, 2, 4, OBSERVER_VERSION)
                else:
                    if observer is None:
                        with np.load(self.root / row['shard'], allow_pickle=False) as shard:
                            observer = int(shard.get('router_observer_version', 1))
                    selected = observer == observer_version
            key = (row['label_version'], observer, row['phase'], row['split'], bool(row.get('code_changed')))
            counts = cohorts.setdefault(key, [0, 0]); counts[0] += 1; counts[1] += int(selected)
            if selected: self.rows.append(row)
        self.cohorts = [dict(label_version=k[0], router_observer_version=k[1], phase=k[2],
                             split=k[3], code_changed=k[4], total=n[0], selected=n[1])
                         for k, n in sorted(cohorts.items(), key=lambda item: str(item[0]))]

    def __len__(self): return len(self.rows)

    def __getitem__(self, i):
        import torch
        row = self.rows[i]; path = self.root / row['shard']
        if self.check_hash and hashlib.sha256(path.read_bytes()).hexdigest() != row['sha256']: raise ValueError('Shard-Hash')
        with np.load(path, allow_pickle=False) as z: data = {k: z[k] for k in z.files}
        for k in list(data):
            if k.endswith('_scale'):
                target = k.removesuffix('_scale'); data[target] = data[target].astype(np.float32) * data[k]
        observer = int(data.get('router_observer_version', 1))
        if row['phase'] == 'routing':
            if row.get('router_observer_version', observer) != observer:
                raise ValueError('Observer contract differs between index and shard')
            if self.observer_version == OBSERVER_VERSION and observer in (1, 2):
                _old_targets(data, row, observer)
            elif observer == 4 and self.observer_version == OBSERVER_VERSION:
                if str(data.get('router_occupancy_contract', '')) != 'output_copper_cell_contact' or 'router_occupancy_search' not in data:
                    raise ValueError('R44 occupancy semantics missing')
                _r44_search_occupancy(data)
                source = _source_observer(data)
                if source in (1, 2): _old_targets(data, row, source)
                data['router_source_observer_version'] = np.asarray(source)
                data['router_observer_version'] = np.asarray(OBSERVER_VERSION)
                data['read_contract'] = np.asarray(READ_CONTRACT)
            elif observer != self.observer_version:
                raise ValueError('Unknown observer contract')
        _global_masks(data, row)

        if 'router_history_max' in data:
            ls=list(data['cu_layers']); rs=list(data['router_layers']); sh=data['input_'].shape[-2:]
            maps=('router_history_max','router_occupancy','router_conflict_rounds','router_vias_raster')
            target=np.full((len(ls),len(maps),*sh),np.nan,np.float32)
            mask=np.zeros_like(target,dtype=bool)
            for j,l in enumerate(rs):
                if l not in ls: continue
                i=ls.index(l)
                for k,key in enumerate(maps): target[i,k]=data[key][j]
                for k,key in enumerate(maps):
                    mask[i,k]=(data['input_'][i,0]>0) & np.isfinite(data[key][j])
                    if key+'_mask_arr' in data: mask[i,k] &= data[key+'_mask_arr'][j]
            data['goal_raster']=target;data['goal_raster_mask']=mask
            data['goal_raster_columns']=np.asarray(['history_max','occupancy','conflict_rounds','vias'])
            names_n=list(data['net_names']); routed_names=list(data['router_nets'])
            target_n=np.full((len(names_n),data['router_net_values'].shape[1]),np.nan,np.float32)
            mask_n=np.zeros_like(target_n,dtype=bool)
            for j,n in enumerate(routed_names):
                if n in names_n:
                    i=names_n.index(n);target_n[i]=data['router_net_values'][j];mask_n[i]=np.isfinite(target_n[i])
                    if 'router_net_values_mask' in data: mask_n[i] &= data['router_net_values_mask'][j]
                    if observer < 2:
                        mask_n[i,6]=False
            data['goal_net']=target_n;data['goal_net_mask']=mask_n
        tensors = {}; names = {}
        for k, v in data.items():
            if v.dtype.kind in 'bifuc':
                a = v.astype(np.int64) if v.dtype.kind == 'u' else v
                tensors[k] = torch.from_numpy(a.copy())
            else: names[k] = v.tolist()
        return dict(tensors=tensors, names=names, meta=row)

    @staticmethod
    def collate(samples): return samples



Dataset = RasterDataset


def import_(c, z, source_board):
    return capture(c, z, source_board, provenance='test_bench')


def route_context(o, c):
    try:
        c = Path(c); rg = json.loads((Path(o) / 'design_rules.json').read_text())
        rg['router']['labels'] = str((c / 'labels.npz').resolve())
        (c / 'design_rules.json').write_text(json.dumps(rg))
        return str(c)
    except Exception as e:
        print('Labels: rule copy skipped:', repr(e)[:180], flush=True)
        return o


def candidates(c, z, source_board, n=3, automatic=True, cpu_budget=None):
    import random
    from common.shift import relocate
    from learning.ai.features_raster import geometry
    c = Path(c); t0 = time.thread_time(); rows = []
    try:
        if n <= 1 or excluded(source_board): return dict(candidates=0, cpu_s=0.)
        B = json.loads((c / 'board.json').read_text()); K = json.loads((c / 'netclasses.json').read_text())
        rg = json.loads((c / 'design_rules.json').read_text()) if (c / 'design_rules.json').exists() else {}
        base_cpu = sum((z.get(s) or {}).get('cpu_s', 0) or 0 for s in ('placer', 'router'))
        budget = max(0., base_cpu * .09 - (z.get('dataset') or {}).get('cpu_s', 0))

        group = '%s:%s:%s' % (family(z), hashlib.sha256(Path(source_board).read_bytes()).hexdigest(), z.get('candidate'))
        rnd = random.Random((z.get('cfg') or {}).get('seed', 0) + 104729)
        fixed = {f['ref'] for f in B['fps'] if f.get('locked') or (K.get('part_list') or {}).get(f['ref'], {}).get('pinned')}
        refs = {f['ref'] for f in B['fps']}
        groups = []
        covered = set(); blocked = set()
        for owner, isl in sorted((K.get('islands') or {}).items()):
            g = ({owner} | set(isl.get('members') or [])) & refs
            covered |= g
            if g & fixed: blocked |= g
            elif g: groups.append(sorted(g))
        groups = [g for g in groups if not set(g) & blocked]
        groups += [[r] for r in sorted(refs - covered - fixed)]
        if not groups: return dict(candidates=0, cpu_s=time.thread_time() - t0)
        estimate = .5
        for k in range(1, n):
            if cpu_budget is not None and cpu_budget.rest <= 0: break
            if automatic and time.thread_time() - t0 + estimate > budget: break
            tc = time.thread_time(); g = rnd.choice(groups)
            dx, dy = rnd.choice(((-.5, 0), (.5, 0), (0, -.5), (0, .5), (-.25, -.25), (.25, .25)))
            BB, _ = relocate(B, {r: (dx, dy, 0) for r in g})
            m, cheap = substitute_ground(BB, K, rg, z.get('gnd_layers'))
            outline = geometry(BB['outline'])
            cheap['parts_outer'] = sum(1 for cy in BB.get('courtyards', []) if geometry(cy['polys']).difference(outline).area > 1e-8)

            tmp = c / ('labels_candidate_%d' % k); tmp.mkdir(exist_ok=True)
            (tmp / 'board.json').write_text(json.dumps(BB)); (tmp / 'netclasses.json').write_text(json.dumps(K))
            zz = dict(z, candidate=str(z.get('candidate')) + ':fallback%d' % k, comparison_group=group,
                      pair_with=z.get('candidate'), substitute_ground=cheap, is_valid=None, score_v3=None,
                      outcome=None, rule=None, cur_state='only_substitute_ground')
            row = capture(tmp, zz, source_board, metrics=None, input_=m)
            rows.append(row)
            import shutil
            shutil.rmtree(tmp)
            estimate = max(.01, (time.thread_time() - tc) * 1.2)
        return dict(candidates=len(rows), cpu_s=time.thread_time() - t0, comparison_group=group,
                    budget_cpu_s=budget if automatic else None, shards=rows)
    except Exception as e:
        return dict(candidates=len(rows), cpu_s=time.thread_time() - t0, faults=repr(e)[:200])


def _native_valid(E, drc):

    from analysis.corpus import valid_whole
    if drc is None: return False
    try: return valid_whole(dict(E, drc=drc), True)
    except (KeyError, TypeError): return False


def test_bench_capture(o, res):
    from analysis.evaluation import rate
    o = Path(o); rows = []
    for route in sorted([o / 'our.json', *o.glob('auto*.json')]):
        if not (route.stem == 'our' or route.stem == 'auto' or route.stem.startswith('auto_k')): continue
        ep = route.with_name(route.stem + '_labels.npz')
        if not route.exists() or not ep.exists(): continue
        tag = route.stem.removeprefix('auto')
        bj = o / ('board.json' if route.stem == 'our' else 'auto_board%s.json' % tag)
        kj = o / ('netclasses.json' if route.stem == 'our' else 'auto_netclasses%s.json' % tag)
        if not bj.exists() or not kj.exists(): continue
        try:
            B = json.loads(bj.read_text()); K = json.loads(kj.read_text()); rr = json.loads(route.read_text())

            z0 = res.get('our') if route.stem == 'our' else next((e for e in res.get('_labels_auto', []) if e.get('candidate') == (tag or 'k0')), None)
            if z0 and z0.get('repaired'):
                ep = o / '_no_raw_target.npz'
                if route.stem.startswith('auto'):
                    rep_board = o / ('rep%s' % tag) / 'board_after.json'
                    if rep_board.exists(): bj = rep_board
                    else: continue
            if bj != o / ('board.json' if route.stem == 'our' else 'auto_board%s.json' % tag): B = json.loads(bj.read_text())
            E = rate(B, dict(rr, **({'drc': z0['drc']} if z0 and z0.get('drc') else {})), K, v3=True)
            z = dict(board=res['board'], design_family=_design_name(res['board']), candidate=route.stem,
                     cfg=dict(seed=res.get('_labels_seed', 1)), is_valid=_native_valid(E, (z0 or {}).get('drc')), score_v3=E.get('score_v3_nets'),
                     rule={**(E.get('design_rules') or {}), 'minimax_status': E.get('minimax_status')},
                     outcome=dict(drc=(z0 or {}).get('drc')), gnd_layers=rr.get('gnd_layers', []), cur_state='ok')
            metrics = dict(E.get('measures') or {}, score_v3_whole=E.get('score_v3'), not_connected=E['not_connected'], area_nets=rr.get('area_nets', []))
            drc_path = o / ('our.kicad_pcb.drc.json' if route.stem == 'our' else 'auto_finished%s.kicad_pcb.drc.json' % tag)
            rows.append(capture(o, z, o / 'board.json', 'test_bench', metrics, board=bj, netclasses=kj, labels=ep, drc_path=drc_path))
        except Exception as e: rows.append(dict(cur_state='faults', faults=repr(e)[:200]))
    return rows
