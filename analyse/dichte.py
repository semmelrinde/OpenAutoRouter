""
from shapely.geometry import box
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union
from shapely.errors import GEOSException
from gemeinsam.teile import lade_teile, platine


def geometrie_ok(g):
    ""
    return (isinstance(g, dict) and bool(g) and all(
        isinstance(p, BaseGeometry) and not p.is_empty and p.is_valid and p.area > 0 for p in g.values()))


def huellbox(geos):
    bs = [g.bounds for geo in geos.values() for g in geo.values() if not g.is_empty]
    if not bs: return None
    return (min(b[0] for b in bs), min(b[1] for b in bs),
            max(b[2] for b in bs), max(b[3] for b in bs))


def huellflaeche(geos):
    b = huellbox(geos)
    return (b[2] - b[0]) * (b[3] - b[1]) if b else 0.0


class Huellziel:
    ""
    def __init__(self, geos, refs=None):
        self.bounds = {}
        for r, geo in geos.items():
            if refs is None or r in refs: self.setze(r, geo)

    def setze(self, r, geo):
        self.bounds[r] = huellbox({r: geo})

    def flaeche(self, ch=None):
        neu = {r: huellbox({r: g}) for r, g in (ch or {}).items()}
        bs = [neu.get(r, b) for r, b in self.bounds.items() if neu.get(r, b) is not None]
        if not bs: return 0.0
        return (max(b[2] for b in bs) - min(b[0] for b in bs)) * (max(b[3] for b in bs) - min(b[1] for b in bs))


def inselgruppen(K, refs, fest=()):
    ""
    refs = set(refs)
    frei = bewegliche(K, refs, fest)
    inseln = K.get('inseln')
    if not isinstance(inseln, dict): raise ValueError('Klassenvertrag: inseln fehlt')
    gruppen = {}
    owner = {}
    for a, insel in sorted(inseln.items()):
        members = insel.get('mitglieder') if isinstance(insel, dict) else None
        if not isinstance(members, list) or any(not isinstance(r, str) for r in members):
            raise ValueError('Klassenvertrag: Inselmitglieder fehlen/ungueltig: ' + a)
        rs = {a, *members}
        if rs - refs: raise ValueError('Inselteile ohne Footprint: ' + ', '.join(sorted(rs - refs)))



        for r in sorted(rs):
            if r in owner: raise ValueError('Mehrdeutige Inselzuordnung: ' + r)
            owner[r] = a
        if len(rs) < 2 or not rs & frei: continue
        gruppen[a] = sorted(rs)
    return gruppen


class Inselziel:
    ""
    def __init__(self, geos, gruppen):
        self.gruppen = gruppen
        self.von = {r: a for a, rs in gruppen.items() for r in rs}
        self.bounds = {r: huellbox({r: geos[r]}) for r in self.von}
        if any(b is None for b in self.bounds.values()): raise ValueError('Insel ohne Geometrie')

    def setze(self, r, geo):
        if r in self.bounds: self.bounds[r] = huellbox({r: geo})

    def beitrag(self, ch):
        neu = {r: huellbox({r: g}) for r, g in ch.items() if r in self.von}
        c = 0.0
        for a in sorted({self.von[r] for r in neu}):
            bs = [neu.get(r, self.bounds[r]) for r in self.gruppen[a]]
            c += max(b[2] for b in bs) - min(b[0] for b in bs)
            c += max(b[3] for b in bs) - min(b[1] for b in bs)
        return c


def insel_messen(B, K, RG=None, lage=None, teile=None, *, fest=(), geos=None):
    ""
    meta = dict(version='p26-inseln.v1', status='unbekannt', anzahl=None,
                halbumpfang_mm=None, huelle_mm2=None, packungsverlust=None, gruppen=[])
    fps = {f['ref']: f for f in B['fps']}
    try:
        inseln = K.get('inseln')
        if not isinstance(inseln, dict): raise ValueError('Klassenvertrag: inseln fehlt/ungueltig')
        T = teile if teile is not None else (lade_teile(B, float((RG or {}).get('placer', {}).get('padabst', 0.15)))
                                           if geos is None else None)
        gs = {}


        for a, island in inseln.items():
            members = island.get('mitglieder') if isinstance(island, dict) else None
            if not isinstance(members, list) or any(not isinstance(r, str) for r in members):
                raise ValueError('Inselmitglieder fehlen/ungueltig: ' + a)
            rs = {a, *members}
            if rs - fps.keys(): raise ValueError('Inselteile ohne Footprint: ' + ', '.join(sorted(rs - fps.keys())))
            for r in sorted(rs):
                if r in gs: continue
                g = geos.get(r, {}) if geos is not None else T[r].geo(
                    *(lage or {}).get(r, (fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side'])))
                if not geometrie_ok(g):
                    raise ValueError('Inselteil ohne vollstaendige gueltige Geometrie: ' + r)
                gs[r] = g
        gruppen = inselgruppen(K, fps, fest)
        out = {}
        for a, rs in gruppen.items():
            group_geos = {r: gs[r] for r in rs}
            b = huellbox(group_geos)
            h = (b[2] - b[0]) * (b[3] - b[1])
            u = unary_union([g for geo in group_geos.values() for g in geo.values()]).area
            out[a] = dict(refs=rs, huelle_mm2=h, belegt_mm2=u,
                          halbumpfang_mm=b[2] - b[0] + b[3] - b[1])
    except (KeyError, ValueError, GEOSException) as e:
        return dict(meta, fehler=str(e))
    if not out: return dict(meta, status='nicht_anwendbar', anzahl=0)
    h = sum(g['huelle_mm2'] for g in out.values())
    u = sum(g['belegt_mm2'] for g in out.values())


    return dict(meta, status='ok', anzahl=len(out), gruppen=[dict(anker=a, **g) for a, g in out.items()],
                huelle_mm2=round(h, 6), halbumpfang_mm=round(sum(g['halbumpfang_mm'] for g in out.values()), 6),
                packungsverlust=round(max(0.0, 1 - u / h), 6))


def messen(B, RG=None, lage=None, teile=None, *, geos=None):
    T = teile if teile is not None else lade_teile(B, float((RG or {}).get('placer', {}).get('padabst', 0.15)))
    fps = {f['ref']: f for f in B['fps']}
    if geos is None:
        geos = {r: t.geo(*(lage or {}).get(r, (fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']))) for r, t in T.items()}
    U = unary_union([g for geo in geos.values() for g in geo.values()])
    board = platine(dict(B, drc_modell=True), 0)[0]
    A = board.area
    return dict(version='dichte.v1', definition='Courtyard+Padabstand, Projektion F/B',
                platine_mm2=round(A, 6), belegt_mm2=round(U.area, 6),
                huelle_mm2=round(huellflaeche(geos), 6),
                ueberstand_mm2=round(U.difference(board).area, 6),
                dichte=round(U.intersection(board).area / A, 6) if A else None)


def gegen(B, original, RG=None):
    D = messen(B, RG); O = original if isinstance(original, dict) and 'platine_mm2' in original else messen(original, RG)
    for k in ('platine_mm2', 'belegt_mm2', 'huelle_mm2', 'dichte'):
        D[k + '_rel_original'] = round(D[k] / O[k], 6) if O.get(k) else None
    return D


def bewegliche(K, refs, fest=()):
    ""
    refs = set(refs)
    kt = K.get('teile') if isinstance(K, dict) else None
    if not isinstance(kt, dict): raise ValueError('Klassenvertrag: teile fehlt')
    for r in sorted(refs):
        t = kt.get(r)
        if not isinstance(t, dict): raise ValueError('Klassenvertrag: Eintrag fehlt: ' + r)
        if type(t.get('fest')) is not bool: raise ValueError('Klassenvertrag: fest fehlt/ungueltig: ' + r)
        if t.get('rolle') not in ('chip', 'klein', 'stecker', 'mechanik', 'ohne'):
            raise ValueError('Klassenvertrag: rolle fehlt/ungueltig: ' + r)
    return {r for r in refs if not kt[r]['fest'] and r not in fest
            and kt[r]['rolle'] not in ('mechanik', 'ohne')}


def innen(B, K, RG=None, lage=None, teile=None, *, fest=(), geos=None):
    ""
    fps = {f['ref']: f for f in B['fps']}
    meta = dict(version='p26-innen.v2', status='unbekannt', teile=None, refs=None,
                huelle_mm2=None, belegt_mm2=None, packungsverlust=None,
                umriss_fest=bool(((K if isinstance(K, dict) else {}).get('vorgaben') or {}).get('umriss_fest')))
    try:
        frei = bewegliche(K, fps, fest)
    except ValueError as e:
        return dict(meta, fehler=str(e))
    T = teile if teile is not None else lade_teile(B, float((RG or {}).get('placer', {}).get('padabst', 0.15)))
    if frei - T.keys(): return dict(meta, fehler='Freie Teile ohne Geometrie: ' + ', '.join(sorted(frei - T.keys())))
    refs = sorted(frei)
    gs = {r: geos.get(r, {}) if geos is not None else
          T[r].geo(*(lage or {}).get(r, (fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']))) for r in refs}
    for r, geo in gs.items():
        if not geometrie_ok(geo):
            return dict(meta, fehler='Freies Teil ohne vollstaendige gueltige Geometrie: ' + r)
    A = huellflaeche(gs)
    U = unary_union([g for geo in gs.values() for g in geo.values()])
    return dict(meta, status='ok', teile=len(refs), refs=refs, huelle_mm2=round(A, 6),
                belegt_mm2=round(U.area, 6),
                packungsverlust=round(max(0.0, 1 - U.area / A), 6) if A else None)


def diagnose(B, K, RG=None, lage=None, teile=None, *, fest=()):
    ""
    contract = B.get('p26_fest')
    if contract is not None:
        if contract.get('version') != 'p26-fest-v1':
            raise ValueError('P26-Festteilvertrag unbekannt')
        fest = set(fest) | set(contract['refs'])



        try:
            bewegliche(K, (f['ref'] for f in B['fps']))
        except ValueError:


            pass
        else:
            K = dict(K, teile={r: dict(t, fest=r in fest) if isinstance(t, dict) else t
                               for r, t in K['teile'].items()})
    try:
        T = teile if teile is not None else lade_teile(B, float((RG or {}).get('placer', {}).get('padabst', 0.15)))
        fps = {f['ref']: f for f in B['fps']}
        geos = {r: t.geo(*(lage or {}).get(r, (fps[r]['x'], fps[r]['y'], 0.0, fps[r]['side']))) for r, t in T.items()}
        return dict(gesamt=messen(B, RG, lage, T, geos=geos),
                    innen=innen(B, K, RG, lage, T, fest=fest, geos=geos),
                    inseln=insel_messen(B, K, RG, lage, T, fest=fest, geos=geos))
    except GEOSException as e:

        failure = dict(status='unbekannt', fehler=repr(e)[:200], huelle_mm2=None,
                       belegt_mm2=None, packungsverlust=None)
        return dict(gesamt=dict(failure, version='dichte.v1'),
                    innen=dict(failure, version='p26-innen.v2', refs=None, teile=None),
                    inseln=dict(failure, version='p26-inseln.v1', anzahl=None, halbumpfang_mm=None, gruppen=[]))

def enger_umriss(B, geos, rand, fest=False):
    ""
    if fest: return None, 'umriss_fest'
    if any(B.get('outline_holes') or []): return None, 'Ausschnitt'
    if any(f['ref'] not in geos for f in B['fps']): return None, 'Teil ohne Kollisionsgeometrie'
    if B.get('keepouts') or B.get('kupfer_grafik'): return None, 'Sperrflaeche/Grafik'
    board = platine(B, 0)[0]; b = board.bounds
    if len(B['outline']) != 1 or board.symmetric_difference(box(*b)).area > 1e-6:
        return None, 'kein einfaches Rechteck'
    h = huellbox(geos)
    if h is None: return None, 'keine Teile'


    import math
    x0 = max(b[0], math.floor((h[0] - rand - 0.001) * 1000 + 1e-9) / 1000)
    y0 = max(b[1], math.floor((h[1] - rand - 0.001) * 1000 + 1e-9) / 1000)
    x1 = min(b[2], math.ceil((h[2] + rand + 0.001) * 1000 - 1e-9) / 1000)
    y1 = min(b[3], math.ceil((h[3] + rand + 0.001) * 1000 - 1e-9) / 1000)
    if x1 <= x0 or y1 <= y0 or (x1 - x0) * (y1 - y0) >= board.area - 1e-6:
        return None, 'kein freier Rand'
    return [[[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]]], 'getrimmt'


def gruppenwege(B, K, lage=None, teile=None):
    ""
    from analyse.p26_gruppenwege import messen
    return messen(B, K, lage, teile)
