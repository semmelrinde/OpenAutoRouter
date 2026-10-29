
import json
import math
import sys
from pathlib import Path


def check(gate_check, chokepoint, dc, reference, geometry, spec):
    reasons = []
    if not gate_check.get("accept"):
        reasons.append("Rule gate not passed")
    if gate_check.get("new", {}).get("drc") != dict(faults=0, open_items=0):
        reasons.append("Native DRC not 0/0")
    if gate_check.get("prior", {}).get("evaluation", {}).get("is_valid") and not gate_check.get(
        "new", {}
    ).get("evaluation", {}).get("is_valid"):
        reasons.append("JSON connection gate worse")
    if not spec.get("channels") or not spec.get("chokepoint_mm"):
        reasons.append("Empty acceptance constraint")
    model = dc.get("model")
    refmodel = reference.get("model")
    if (
        not model
        or model != refmodel
        or any(
            k not in model
            for k in ("solver_sha256", "h_mm", "rho_Ohm_m", "thickness_um", "coordinates")
        )
    ):
        reasons.append("DC models missing or differ")
    if not dc.get("kicad_native") or not reference.get("kicad_native"):
        reasons.append("DC not in the native filled copper")
    rows = {}
    for name, keys in spec["channels"].items():
        try:
            a, b = [sum(float(d["value_list"][k]) for k in keys) for d in (dc, reference)]
            if not all(math.isfinite(v) and v > 0 for v in (a, b)):
                raise ValueError("not finite/positive")
            rows[name] = dict(new_mOhm=a, reference_mOhm=b)
            if a > b + 1e-12:
                reasons.append("Resistance worse: " + name)
        except (KeyError, TypeError, ValueError):
            reasons.append("DC contact missing/invalid: " + name)
    for name, minimum in spec["chokepoint_mm"].items():
        value = chokepoint.get("channels", {}).get(name, {}).get("width_mm")
        if value is None or not math.isfinite(value) or value < float(minimum) - 1e-12:
            reasons.append("Bottleneck below target/missing: " + name)
    for name, maximum in spec.get("dead_ends_mm2", {}).items():
        value = geometry.get("dead_ends_mm2", {}).get(name)
        if value is None or not math.isfinite(value) or value > maximum:
            reasons.append("Dead end above target/missing: " + name)
    for name, tolerance in spec.get("flag_increase_mm", {}).items():
        value = geometry.get("flag", {}).get(name, {}).get("max_increase_mm")
        if value is None or not math.isfinite(value) or value > tolerance:
            reasons.append("Flag width not monotonic/missing: " + name)
    for section, channels in spec.get("cross_section_mm", {}).items():
        for net, limits in channels.items():
            value = geometry.get("cross_sections_mm", {}).get(section, {}).get(net)
            if (
                value is None
                or not math.isfinite(value)
                or not limits[0] <= value <= limits[1]
            ):
                reasons.append(
                    "Cross-section outside target/missing: " + section + " " + net
                )
    return dict(
        passed=not reasons,
        reasons=reasons,
        dc_channels=rows,
        limit="Numerical 2D/grid/cross-section acceptance, no hardware or global C1 evidence.",
    )


def main():
    *files, out = sys.argv[1:8]
    result = check(*[json.loads(Path(f).read_text()) for f in files])
    Path(out).write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print("R33-Acceptance:", result["passed"], "; ".join(result["reasons"]))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
