from shapely.geometry import Polygon

TOLERANCE = 1e-4


def too_close(shaping, foreign_hard, spacing_hard, foreign_zone=None, spacing_zone=None, tolerance=TOLERANCE):
    if foreign_hard is not None and not foreign_hard.is_empty and shaping.distance(foreign_hard) < spacing_hard - tolerance:
        return True
    if foreign_zone is not None and not foreign_zone.is_empty and spacing_zone is not None and shaping.distance(foreign_zone) < spacing_zone - tolerance:
        return True
    return False


def export_shape(pts):
    return Polygon(pts).buffer(0)
