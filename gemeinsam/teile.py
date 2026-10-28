""
import math
import numpy as np
from shapely.geometry import Polygon, box, Point
from shapely.ops import unary_union
from shapely import affinity
from shapely.prepared import prep

GEGEN = {'F': 'B', 'B': 'F'}


def _local(pts, x0, y0): return Polygon([(x - x0, y - y0) for x, y in pts]).buffer(0)


class Teil:
    def __init__(self, B, ref, fp, pad_idx, crts, padabst=0.15):
        self.ref = ref; self.side0 = fp['side']; self.x0, self.y0 = fp['x'], fp['y']
        self.pads = []
        self.pad_halb = []
        own, other = [], []
        x_own, x_oth = [], []
        copper = [p for p in B.get('footprint_copper', []) if p['ref'] == ref] if B.get('drc_modell') else []
        self.hat_kupfergrafik = bool(copper)


        for p, electrical in [(B['pads'][i], True) for i in pad_idx] + [(p, False) for p in copper]:
            tht = p.get('drill', 0) > 0 or len(p['layers']) > 1
            if electrical:
                self.pads.append((p['num'], p['net'], p['x'] - self.x0, p['y'] - self.y0, tht))
                bb = p.get('bbox')
                self.pad_halb.append(((bb[2] - bb[0]) / 2, (bb[3] - bb[1]) / 2) if bb else (0.0, 0.0))
            for L, qs in p['polys'].items():
                mk = (p.get('maske') or {}).get(L, 0.0)
                for q in qs:
                    if len(q) >= 3:
                        (own if L == self.side0 else other).append(_local(q, self.x0, self.y0))
                        if mk + 0.01 > padabst / 2:
                            (x_own if L == self.side0 else x_oth).append(_local(q, self.x0, self.y0).buffer(mk + 0.01, join_style=2))


            hole_extra = max(0.0, ((B.get('design') or {}).get('loch_abstand') or 0.0) - padabst)
            holes = [_local(q, self.x0, self.y0).buffer(hole_extra) for q in p.get('hole_polys', []) if len(q) >= 3] if B.get('drc_modell') else []
            if holes:
                own.extend(holes); other.extend(holes)
        c_own = [_local(q, self.x0, self.y0) for c in crts if c['side'] == self.side0 for q in c['polys'] if len(q) >= 3]
        c_oth = [_local(q, self.x0, self.y0) for c in crts if c['side'] != self.side0 for q in c['polys'] if len(q) >= 3]
        pg = unary_union(own).buffer(padabst / 2, join_style=2) if own else None
        if pg is not None and x_own: pg = unary_union([pg] + x_own)
        if c_own: body = unary_union(c_own + ([pg] if pg is not None else []))
        elif own: body = unary_union(own).envelope.buffer(0.15, join_style=2).union(pg)
        else: body = Point(0, 0).buffer(0.3)
        self.g = {'own': body.buffer(0)}
        og = [g for g in c_oth] + ([unary_union(other).buffer(padabst / 2, join_style=2)] if other else []) + x_oth
        self.g['other'] = unary_union(og).buffer(0) if og else None
        self._cache = {}
        self.flaeche = self.g['own'].area
        self.n_pads = len(self.pads)

    def _rot(self, th, side):
        key = (round(th) % 360, side)
        if key not in self._cache:
            mir = side != self.side0
            res = {}
            for k, g in self.g.items():
                if g is None: continue
                h = affinity.scale(g, -1, 1, origin=(0, 0)) if mir else g
                h = affinity.rotate(h, th, origin=(0, 0))
                res[side if k == 'own' else GEGEN[side]] = h
            c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
            pts = []
            for nr, net, lx, ly, tht in self.pads:
                if mir: lx = -lx
                pts.append((nr, net, c * lx - s * ly, s * lx + c * ly, side if not tht else 'FB'))
            self._cache[key] = (res, pts)
        return self._cache[key]

    def geo(self, x, y, th, side):
        ""
        res, _ = self._rot(th, side)
        return {s: affinity.translate(g, x, y) for s, g in res.items()}

    def pads_abs(self, x, y, th, side):
        ""
        _, pts = self._rot(th, side)
        return [(nr, net, x + px, y + py, s) for nr, net, px, py, s in pts]

    def lokal_bounds(self, th, side):
        res, _ = self._rot(th, side)
        return {s: g.bounds for s, g in res.items()}

    def pads_halb(self, th):
        ""
        if round(th) % 180 == 90: return [(hy, hx) for hx, hy in self.pad_halb]
        return list(self.pad_halb)


class Belegung:
    ""
    def __init__(self):
        self.items = {'F': [], 'B': []}
        self._bb = {'F': None, 'B': None}

    def add(self, ref, geo):
        for s, g in geo.items():
            self.items[s].append((ref, g)); self._bb[s] = None

    def remove(self, ref):
        for s in self.items:
            n0 = len(self.items[s])
            self.items[s] = [(r, g) for r, g in self.items[s] if r != ref]
            if len(self.items[s]) != n0: self._bb[s] = None

    def _bbox(self, s):
        if self._bb[s] is None:
            self._bb[s] = np.array([g.bounds for _, g in self.items[s]]) if self.items[s] else np.zeros((0, 4))
        return self._bb[s]

    def kollision(self, geo, ignore=(), luft=0.005):
        ""
        for s, g in geo.items():
            bb = self._bbox(s)
            if not len(bb): continue
            b = g.bounds
            m = (bb[:, 0] < b[2] + luft) & (bb[:, 2] > b[0] - luft) & (bb[:, 1] < b[3] + luft) & (bb[:, 3] > b[1] - luft)
            idx = np.nonzero(m)[0]
            if not len(idx): continue
            for k in idx:
                r, h = self.items[s][k]
                if r in ignore: continue
                if g.distance(h) < luft: return True
        return False

    def ueberlappung(self, geo, ignore=()):
        ""
        a = 0.0; who = []
        for s, g in geo.items():
            bb = self._bbox(s)
            if not len(bb): continue
            b = g.bounds
            m = (bb[:, 0] < b[2]) & (bb[:, 2] > b[0]) & (bb[:, 1] < b[3]) & (bb[:, 3] > b[1])
            for k in np.nonzero(m)[0]:
                r, h = self.items[s][k]
                if r in ignore: continue
                x = g.intersection(h).area
                if x > 1e-7: a += x; who.append(r)
        return a, who


def lade_teile(B, padabst=0.15, refs=None):
    import collections
    pads_by = collections.defaultdict(list)
    for i, p in enumerate(B['pads']): pads_by[p['ref']].append(i)
    crt = collections.defaultdict(list)
    for c in B['courtyards']: crt[c['ref']].append(c)
    T = {}
    for f in B['fps']:
        r = f['ref']
        if refs is not None and r not in refs: continue
        copper = B.get('drc_modell') and any(p['ref'] == r for p in B.get('footprint_copper', []))
        if not pads_by[r] and not crt[r] and not copper: continue
        T[r] = Teil(B, r, f, pads_by[r], crt[r], padabst)
    return T


def sperrflaechen(B, luft=0.005):
    ""
    out = {'F': [], 'B': []}
    for k in B.get('keepouts') or []:
        if k.get('ref'): continue
        if not (k.get('verbote') or {}).get('teile', True): continue
        lagen = k.get('lagen')
        seiten = [s for s in ('F', 'B') if not lagen or s in lagen]
        for q in k['polys']:
            if len(q) >= 3:
                for s in seiten: out[s].append(Polygon(q).buffer(0))
    return {s: (unary_union(v).buffer(luft) if v else None) for s, v in out.items()}




HOEHE_FP = (('NetTie', 0.05), ('TestPoint', 0.05), ('Fiducial', 0.0), ('MountingHole', 0.0), ('R_0201', 0.25), ('C_0201', 0.3),
            ('R_0402', 0.4), ('C_0402', 0.55), ('L_0402', 0.6), ('R_0603', 0.5), ('C_0603', 0.9), ('L_0603', 0.9), ('LED_0603', 0.8),
            ('R_0805', 0.6), ('C_0805', 1.25), ('L_0805', 1.0), ('LED_0805', 1.0), ('SOD-523', 0.7), ('SOD-323', 1.0), ('SOD-123', 1.15),
            ('SOT-563', 0.6), ('SOT-583', 0.6), ('SOT-323', 1.0), ('SOT-23', 1.15), ('TDFN', 0.8), ('DFN', 0.8), ('QFN', 0.9),
            ('TSSOP', 1.2), ('MSOP', 1.1), ('SOIC', 1.75))


def hoehe(fp, ref=None, vorgaben=None):
    import fnmatch
    for g, h in sorted((vorgaben or {}).items()):
        if ref is not None and fnmatch.fnmatchcase(ref, g): return float(h)
    for k, v in HOEHE_FP:
        if k in (fp or ''): return v
    return 1.5


def platine(B, rand=0.2):
    holes = (B.get('outline_holes') or []) if B.get('drc_modell') else []
    board = unary_union([Polygon(o, holes[i] if i < len(holes) else []) for i, o in enumerate(B['outline'])]).buffer(0)
    inner = board.buffer(-rand)
    kg = [Polygon(q) for g in B.get('kupfer_grafik', []) for q in g['polys'] if len(q) >= 3]
    if kg: inner = inner.difference(unary_union(kg).buffer(max(rand, 0.25)))
    keep = unary_union([Polygon(q) for k in B.get('keepouts', []) if k['ref'] or (k.get('verbote') or {}).get('teile', True) for q in k['polys'] if len(q) >= 3]) if B.get('keepouts') else None
    return board, inner, prep(inner), keep
