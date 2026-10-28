""
import math
from analyse.dichte import bewegliche, inselgruppen
from gemeinsam.teile import lade_teile


def messen(B, K, lage=None, teile=None):
    ""
    meta = dict(version='p26-gruppenwege.v1', status='unbekannt', netze=None,
                hpwl_mm=None, max_hpwl_mm=None, buendel=None)
    try:
        refs = {f['ref'] for f in B['fps']}

        bewegliche(K, refs)
        inselgruppen(dict(K, teile={r: dict(K['teile'][r], fest=False) for r in refs}), refs)
        owner = {r: r for r in refs}
        for a, insel in sorted(K['inseln'].items()):
            for r in {a, *insel['mitglieder']}: owner[r] = a
        T = teile if teile is not None else lade_teile(B, float((K.get('regeln') or {}).get('placer', {}).get('padabst', .15)))
        fps = {f['ref']: f for f in B['fps']}
        pos = {}; groups = {}
        for r in sorted(refs):
            s = (lage or {}).get(r, (fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']))
            for _, n, x, y, _ in T[r].pads_abs(*s):
                if not n or n in K['gnd']: continue
                art = K['netze'][n]['art']
                if art not in ('signal', 'schnell', 'analog'): continue
                if not all(math.isfinite(v) for v in (x, y)): raise ValueError('Nichtendliche Padlage')
                pos.setdefault(n, []).append((x, y)); groups.setdefault(n, set()).add(owner[r])
        spans = []; pairs = {}
        for n, pts in sorted(pos.items()):
            if len(groups[n]) < 2: continue
            spans.append(max(x for x, y in pts) - min(x for x, y in pts)
                         + max(y for x, y in pts) - min(y for x, y in pts))
            gs = sorted(groups[n])
            for i, a in enumerate(gs):
                for b in gs[i + 1:]: pairs[a, b] = pairs.get((a, b), 0) + 1
        return dict(meta, status='ok' if spans else 'nicht_anwendbar', netze=len(spans),
                    hpwl_mm=round(sum(spans), 6) if spans else None,
                    max_hpwl_mm=round(max(spans), 6) if spans else None,
                    buendel=sum(v >= 2 for v in pairs.values()))
    except (KeyError, ValueError, TypeError) as e:
        return dict(meta, fehler=str(e))

