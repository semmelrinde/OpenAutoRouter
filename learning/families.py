import json
import re
from functools import lru_cache
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def source(meta):
    return str(meta.get('project') or meta.get('source') or '').lower().removeprefix('https://github.com/').removesuffix('.git').strip('/')


def name_family(name):
    n = re.sub(r'^\d+_', '', str(name).lower().removesuffix('_tight'))
    t = n.split('_')
    return '_'.join(t[:2]) if t[0] == 'kitspace' and len(t) > 1 else t[0]


@lru_cache(maxsize=1)
def aliases():
    out = {}; candidates = {}
    def add(name, source): candidates.setdefault(name, set()).add(source)
    for p in sorted((ROOT / 'examples').glob('corpus*.json')):
        for b in json.loads(p.read_text()).get('boards', []):
            q = source(b)
            if not q: continue
            for n in (b.get('name'), b.get('pcbench')):
                if n:
                    add(str(n).lower(), q)

    cache = Path.home() / 'Library/Caches/Autorouter'
    for root in (cache / 'train', cache / 'multilayer/exercise'):
        for p in sorted(root.glob('*/metadata.json')):
            try: q = source(json.loads(p.read_text()))
            except (OSError, ValueError): continue
            if q:
                add(p.parent.name.lower(), q)

    out.update({n: next(iter(q)) for n, q in candidates.items() if len(q) == 1})
    return out


def family(z):
    q = source(z.get('source_meta') or {})
    if q: return q
    n = str(z.get('family') or (z.get('variant') or {}).get('from_') or z['board']).lower().removesuffix('_tight')
    return aliases().get(n) or (str(z['family']) if z.get('family') else str((z.get('variant') or {}).get('from_') or z['board']).removesuffix('_tight'))


def board_family(d):
    p = Path(d, 'metadata.json')
    meta = json.loads(p.read_text()) if p.exists() else {}
    return family(dict(board=Path(d).name, source_meta=meta))
