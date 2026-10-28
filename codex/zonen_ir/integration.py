""

from .fuellung import mit_fuellung
from .solver_fast import ir_kennzahlen


def zonen_ir_kennzahlen(B, R, K, *, options=None, res=0.1):
    filled, meta = mit_fuellung(B, R, K, options=options)
    result = ir_kennzahlen(B, filled, K, res=res)
    result["zonen_modell"] = meta
    result["kupferumfang"] = (
        "Bahnen/Pads/Vias plus geschätzte Shapely-Füllung; kein KiCad-/DRC-Nachweis."
    )
    return result
