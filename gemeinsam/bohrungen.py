""
import math

from shapely.errors import ShapelyError
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union


def _konturen(contours):
    ""
    if not isinstance(contours, (list, tuple)):
        raise ValueError('Ungültige Lochkontur')
    polys = []
    try:
        for q in contours:
            pts = [(float(x), float(y)) for x, y in q]
            if not all(math.isfinite(v) for pt in pts for v in pt):
                raise ValueError('Ungültige Lochkontur')
            g = Polygon(pts)
            if not g.is_valid or g.is_empty or g.area <= 0:
                raise ValueError('Ungültige Lochkontur')
            polys.append(g)
        geo = unary_union(polys)
    except (ShapelyError, TypeError) as exc:
        raise ValueError('Ungültige Lochkontur') from exc
    except ValueError as exc:
        raise ValueError('Ungültige Lochkontur') from exc
    if not geo.is_valid or geo.is_empty or geo.area <= 0:
        raise ValueError('Ungültige Lochkontur')
    return geo


def pad_bohrung(p):
    ""
    b = float(p.get('drill') or 0)
    size = p.get('drill_size')
    contours = p.get('hole_polys')
    if b == 0 and not contours and (size is None or size == [0, 0] or size == (0, 0)):
        return None
    if size is not None:
        if not isinstance(size, (list, tuple)) or len(size) != 2:
            raise ValueError('Ungültige Bohrmaße')
        size = tuple(float(v) for v in size)
        if not all(math.isfinite(v) and v > 0 for v in size):
            raise ValueError('Ungültige Bohrmaße')
    if b == 0 and size is None:
        b = None
    elif not math.isfinite(b) or b <= 0:
        raise ValueError('Ungültige Bohrung')
    if not contours:
        if size and not math.isclose(*size, rel_tol=0, abs_tol=1e-9):
            raise ValueError('Langloch ohne native Kontur: Richtung unbekannt')
        if size and not math.isclose(size[0], b, rel_tol=0, abs_tol=1e-9):
            raise ValueError('Widersprüchliche runde Bohrmaße')
        return dict(x=float(p['x']), y=float(p['y']), drill=b, kreis=True,
                    geo=Point(p['x'], p['y']).buffer(b / 2, 64))
    geo = _konturen(contours)
    center = geo.centroid
    radii = [math.hypot(x-center.x, y-center.y) for q in contours for x,y in q]



    circle = (b is not None and max(radii)-min(radii) < 1e-5 and abs(sum(radii)/len(radii)-b/2) < .0011)
    if circle:
        if size and (not math.isclose(*size, rel_tol=0, abs_tol=1e-9)
                     or not math.isclose(size[0], b, rel_tol=0, abs_tol=.0011)):
            raise ValueError('Kreiskontur widerspricht Bohrmaßen')
        small = b
    else:
        rect = list(geo.minimum_rotated_rectangle.exterior.coords)
        small = min(math.dist(a,c) for a,c in zip(rect[:-1],rect[1:]))
        if size: small = min(small, *size)
    return dict(x=center.x, y=center.y, drill=small, kreis=circle, geo=geo)
