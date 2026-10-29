import math
from shapely.geometry import Point


def via_free(x, y, inner, tree_idx, bore, new, radius, drill, clearance, h2h):
    if not inner.contains(Point(x, y)):
        return False
    pt = Point(x, y)
    for tree, geometries in tree_idx.values():
        for index in tree.query(pt.buffer(radius + clearance)):
            if geometries[int(index)].distance(pt) < radius + clearance:
                return False
    for bx, by, br in list(bore) + [(a, b, drill / 2) for a, b in new]:
        if math.hypot(bx - x, by - y) < br + drill / 2 + h2h:
            return False
    return True
