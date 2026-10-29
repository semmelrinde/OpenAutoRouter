import math

from shapely.errors import ShapelyError
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union


def _contours(contours):
    if not isinstance(contours, (list, tuple)):
        raise ValueError('Invalid hole contour')
    polys = []
    try:
        for q in contours:
            pts = [(float(x), float(y)) for x, y in q]
            if not all(math.isfinite(v) for pt in pts for v in pt):
                raise ValueError('Invalid hole contour')
            g = Polygon(pts)
            if not g.is_valid or g.is_empty or g.area <= 0:
                raise ValueError('Invalid hole contour')
            polys.append(g)
        geo = unary_union(polys)
    except (ShapelyError, TypeError) as exc:
        raise ValueError('Invalid hole contour') from exc
    except ValueError as exc:
        raise ValueError('Invalid hole contour') from exc
    if not geo.is_valid or geo.is_empty or geo.area <= 0:
        raise ValueError('Invalid hole contour')
    return geo


def pad_drill_hole(p):
    b = float(p.get('drill') or 0)
    size = p.get('drill_size')
    contours = p.get('hole_polys')
    if b == 0 and not contours and (size is None or size == [0, 0] or size == (0, 0)):
        return None
    if size is not None:
        if not isinstance(size, (list, tuple)) or len(size) != 2:
            raise ValueError('Invalid drill dimensions')
        size = tuple(float(v) for v in size)
        if not all(math.isfinite(v) and v > 0 for v in size):
            raise ValueError('Invalid drill dimensions')
    if b == 0 and size is None:
        b = None
    elif not math.isfinite(b) or b <= 0:
        raise ValueError('Invalid drill')
    if not contours:
        if size and not math.isclose(*size, rel_tol=0, abs_tol=1e-9):
            raise ValueError('Slot without native contour: direction unknown')
        if size and not math.isclose(size[0], b, rel_tol=0, abs_tol=1e-9):
            raise ValueError('Contradictory round drill dimensions')
        return dict(x=float(p['x']), y=float(p['y']), drill=b, circle=True,
                    geo=Point(p['x'], p['y']).buffer(b / 2, 64))
    geo = _contours(contours)
    center = geo.centroid
    radii = [math.hypot(x-center.x, y-center.y) for q in contours for x,y in q]



    circle = (b is not None and max(radii)-min(radii) < 1e-5 and abs(sum(radii)/len(radii)-b/2) < .0011)
    if circle:
        if size and (not math.isclose(*size, rel_tol=0, abs_tol=1e-9)
                     or not math.isclose(size[0], b, rel_tol=0, abs_tol=.0011)):
            raise ValueError('Circle contour contradicts drill dimensions')
        small = b
    else:
        rect = list(geo.minimum_rotated_rectangle.exterior.coords)
        small = min(math.dist(a,c) for a,c in zip(rect[:-1],rect[1:]))
        if size: small = min(small, *size)
    return dict(x=center.x, y=center.y, drill=small, circle=circle, geo=geo)
