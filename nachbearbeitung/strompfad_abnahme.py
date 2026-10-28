""

import json
import math
import sys
from pathlib import Path


def pruefe(tor, engpass, dc, reference, geometry, spec):
    reasons = []
    if not tor.get("uebernehmen"):
        reasons.append("Regel-Tor nicht bestanden")
    if tor.get("neu", {}).get("drc") != dict(fehler=0, offen=0):
        reasons.append("Native DRC nicht 0/0")
    if tor.get("alt", {}).get("bewertung", {}).get("gueltig") and not tor.get(
        "neu", {}
    ).get("bewertung", {}).get("gueltig"):
        reasons.append("JSON-Verbindungstor schlechter")
    if not spec.get("kanaele") or not spec.get("engpass_mm"):
        reasons.append("Leere Abnahmevorgabe")
    model = dc.get("modell")
    refmodel = reference.get("modell")
    if (
        not model
        or model != refmodel
        or any(
            k not in model
            for k in ("loeser_sha256", "h_mm", "rho_Ohm_m", "dicke_um", "koordinaten")
        )
    ):
        reasons.append("DC-Modelle fehlen oder sind verschieden")
    if not dc.get("nativ") or not reference.get("nativ"):
        reasons.append("DC nicht im nativen gefuellten Kupfer")
    rows = {}
    for name, keys in spec["kanaele"].items():
        try:
            a, b = [sum(float(d["werte"][k]) for k in keys) for d in (dc, reference)]
            if not all(math.isfinite(v) and v > 0 for v in (a, b)):
                raise ValueError("nicht endlich/positiv")
            rows[name] = dict(neu_mOhm=a, referenz_mOhm=b)
            if a > b + 1e-12:
                reasons.append("Widerstand schlechter: " + name)
        except (KeyError, TypeError, ValueError):
            reasons.append("DC-Kontakt fehlt/ungueltig: " + name)
    for name, minimum in spec["engpass_mm"].items():
        value = engpass.get("kanaele", {}).get(name, {}).get("breite_mm")
        if value is None or not math.isfinite(value) or value < float(minimum) - 1e-12:
            reasons.append("Engpass unter Soll/fehlt: " + name)
    for name, maximum in spec.get("sackgassen_mm2", {}).items():
        value = geometry.get("sackgassen_mm2", {}).get(name)
        if value is None or not math.isfinite(value) or value > maximum:
            reasons.append("Sackgasse ueber Soll/fehlt: " + name)
    for name, tolerance in spec.get("fahne_zunahme_mm", {}).items():
        value = geometry.get("fahne", {}).get(name, {}).get("max_zunahme_mm")
        if value is None or not math.isfinite(value) or value > tolerance:
            reasons.append("Fahnenbreite nicht monoton/fehlt: " + name)
    for section, channels in spec.get("querschnitt_mm", {}).items():
        for net, limits in channels.items():
            value = geometry.get("querschnitte_mm", {}).get(section, {}).get(net)
            if (
                value is None
                or not math.isfinite(value)
                or not limits[0] <= value <= limits[1]
            ):
                reasons.append(
                    "Querschnitt ausserhalb Soll/fehlt: " + section + " " + net
                )
    return dict(
        bestanden=not reasons,
        gruende=reasons,
        dc_kanaele=rows,
        grenze="Numerische 2D-/Raster-/Querschnitt-Abnahme, kein Hardware- oder globaler C1-Nachweis.",
    )


def main():
    *files, out = sys.argv[1:8]
    result = pruefe(*[json.loads(Path(f).read_text()) for f in files])
    Path(out).write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print("R33-Abnahme:", result["bestanden"], "; ".join(result["gruende"]))
    return 0 if result["bestanden"] else 1


if __name__ == "__main__":
    sys.exit(main())
