""
from shapely.geometry import Polygon

TOLERANZ = 1e-4


def zu_nah(form, fremd_hart, abstand_hart, fremd_zone=None, abstand_zone=None, toleranz=TOLERANZ):
    ""
    if fremd_hart is not None and not fremd_hart.is_empty and form.distance(fremd_hart) < abstand_hart - toleranz:
        return True
    if fremd_zone is not None and not fremd_zone.is_empty and abstand_zone is not None and form.distance(fremd_zone) < abstand_zone - toleranz:
        return True
    return False


def ausgabe_form(pts):
    return Polygon(pts).buffer(0)
