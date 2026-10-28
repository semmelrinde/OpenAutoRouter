""
import functools
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform

ROOT = Path(__file__).resolve().parents[1]


def boardkennung(B):
    ""
    data = dict(lagen=B.get('lagen'), outline=B.get('outline'),
                pads=sorted((str(p.get('ref')), str(p.get('num')), str(p.get('net'))) for p in B.get('pads', [])),
                teile=sorted((str(f.get('ref')), str(f.get('value')), str(f.get('fp'))) for f in B.get('fps', [])))
    return 'board-sha256:' + hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()


def quellpfade(root=ROOT):
    dirs = ('analyse', 'gemeinsam', 'nachbearbeitung', 'kicad', 'lernen', 'codex/ir_drop', 'codex/zonen_ir')
    return sorted({p.relative_to(root).as_posix() for d in dirs for p in (root/d).rglob('*')
                   if p.is_file() and (p.suffix == '.py' or p.suffix == '.json' and d == 'analyse') and
                   not any(x in p.relative_to(root/d).parts for x in ('modell', 'daten', 'ergebnisse', '__pycache__'))})


@functools.lru_cache(maxsize=1)
def laufzeit():
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


def messmanifest(root=ROOT):
    root = Path(root)
    stamps = tuple((p, (root/p).stat().st_mtime_ns, (root/p).stat().st_size, (root/p).stat().st_ctime_ns)
                   for p in quellpfade(root))
    return dict(version='paar-quellen-v2', dateien=_manifest(str(root), stamps), laufzeit=laufzeit())


def messsignatur(root=ROOT):
    return hashlib.sha256(json.dumps(messmanifest(root), sort_keys=True).encode()).hexdigest()
