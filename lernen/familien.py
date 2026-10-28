""
import json
import re
from functools import lru_cache
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def quelle(meta):
    return str(meta.get('project') or meta.get('source') or '').lower().removeprefix('https://github.com/').removesuffix('.git').strip('/')


def name_familie(name):
    n = re.sub(r'^\d+_', '', str(name).lower().removesuffix('_eng'))
    t = n.split('_')
    return '_'.join(t[:2]) if t[0] == 'kitspace' and len(t) > 1 else t[0]


@lru_cache(maxsize=1)
def aliasse():
    ""
    out = {}; kandidaten = {}
    def add(name, source): kandidaten.setdefault(name, set()).add(source)
    for p in sorted((ROOT / 'beispiele').glob('korpus*.json')):
        for b in json.loads(p.read_text()).get('boards', []):
            q = quelle(b)
            if not q: continue
            for n in (b.get('name'), b.get('pcbench')):
                if n:
                    add(str(n).lower(), q)

    cache = Path.home() / 'Library/Caches/Autorouter'
    for root in (cache / 'train', cache / 'mehrlagen/uebung'):
        for p in sorted(root.glob('*/metadata.json')):
            try: q = quelle(json.loads(p.read_text()))
            except (OSError, ValueError): continue
            if q:
                add(p.parent.name.lower(), q)

    out.update({n: next(iter(q)) for n, q in kandidaten.items() if len(q) == 1})
    return out


def familie(z):
    q = quelle(z.get('quelle_meta') or {})
    if q: return q
    n = str(z.get('familie') or (z.get('variante') or {}).get('von') or z['board']).lower().removesuffix('_eng')
    return aliasse().get(n) or (str(z['familie']) if z.get('familie') else str((z.get('variante') or {}).get('von') or z['board']).removesuffix('_eng'))


def board_familie(d):
    p = Path(d, 'metadata.json')
    meta = json.loads(p.read_text()) if p.exists() else {}
    return familie(dict(board=Path(d).name, quelle_meta=meta))
