
from .fill import with_fill
from .solver_fast import ir_metrics


def zones_ir_metrics(B, R, K, *, options=None, res=0.1):
    filled, meta = with_fill(B, R, K, options=options)
    result = ir_metrics(B, filled, K, res=res)
    result["zones_model"] = meta
    result["copper_perimeter"] = (
        "Tracks/pads/vias plus estimated Shapely fill; no KiCad/DRC evidence."
    )
    return result
