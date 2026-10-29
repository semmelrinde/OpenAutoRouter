import functools
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform

ROOT = Path(__file__).resolve().parents[1]


def board_id(B):
    data = dict(cu_layers=B.get('cu_layers'), outline=B.get('outline'),
                pads=sorted((str(p.get('ref')), str(p.get('num')), str(p.get('net'))) for p in B.get('pads', [])),
                part_list=sorted((str(f.get('ref')), str(f.get('value')), str(f.get('fp'))) for f in B.get('fps', [])))
    return 'board-sha256:' + hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()


def source_paths(root=ROOT):
    dirs = ('analysis', 'common', 'postprocessing', 'kicad', 'learning', 'codex/ir_drop', 'codex/zones_ir')
    return sorted({p.relative_to(root).as_posix() for d in dirs for p in (root/d).rglob('*')
                   if p.is_file() and (p.suffix == '.py' or p.suffix == '.json' and d == 'analysis') and
                   not any(x in p.relative_to(root/d).parts for x in ('model', 'data', 'results', '__pycache__'))})


@functools.lru_cache(maxsize=1)
def run_duration():
    versions = dict(python=platform.python_version())
    for name in ('numpy', 'scipy', 'shapely', 'scikit-learn', 'numba'):
        try: versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError: versions[name] = None
    import shapely
    versions['geos'] = shapely.geos_version_string
    return versions


@functools.lru_cache(maxsize=2)
def _manifest(root, stamps):
    root = Path(root)
    return {p: hashlib.sha256((root/p).read_bytes()).hexdigest() for p, *_ in stamps}


def measurement_manifest(root=ROOT):
    root = Path(root)
    stamps = tuple((p, (root/p).stat().st_mtime_ns, (root/p).stat().st_size, (root/p).stat().st_ctime_ns)
                   for p in source_paths(root))
    return dict(version='pair-source_list-v2', files=_manifest(str(root), stamps), run_duration=run_duration())


def measurement_signature(root=ROOT):
    return hashlib.sha256(json.dumps(measurement_manifest(root), sort_keys=True).encode()).hexdigest()
