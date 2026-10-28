""

import argparse
import json
import math
from pathlib import Path

SCHEMA = Path(__file__).with_name("korridore.schema.json")


def verbindungsfehler(net):
    ""
    errors = []
    nodes = net["knoten"]
    sources = set(net["quellen_knoten"])
    mapping = net["quellen_anschluesse"]
    if set(mapping) != set(net["quellen_pads"]) or sources != {
        i for ids in mapping.values() for i in ids
    } or not sources:
        errors.append("Quellanschluss fehlt/falsch")
    balance = {int(i): 0.0 for i in nodes}
    adj = {int(i): [] for i in nodes}

    def edge(u, v, I):
        if str(u) not in nodes or str(v) not in nodes or u == v:
            errors.append("Kantenknoten fehlt/falsch")
            return False
        adj[u].append(v)
        balance[u] -= I
        balance[v] += I
        return True

    for c in net["korridore"]:
        if edge(c["von"], c["bis"], c["strom_A"]):
            a, b = [nodes[str(i)] for i in (c["von"], c["bis"])]
            if a["lage"] != c["lage"] or b["lage"] != c["lage"] or any(
                math.dist(p, q) > 1e-7 for p, q in
                ((a["xy"], c["mittellinie"][0]), (b["xy"], c["mittellinie"][-1]))
            ):
                errors.append("Korridoranschluss falsch")
    for f in net["via_felder"]:
        if not all(f.get("kontakt_lagen", {}).get(L) is True for L in f["lagen"]):
            errors.append("Via-Kupferkontakt fehlt")
        for w in f["wechsel"]:
            if edge(w["von"], w["bis"], w["strom_A"]):
                a, b = [nodes[str(i)] for i in (w["von"], w["bis"])]
                if a["lage"] == b["lage"] or {a["lage"], b["lage"]} - set(f["lagen"]) or any(
                    math.dist(p["xy"], f["mitte"]) > 1e-7 for p in (a, b)
                ):
                    errors.append("Lagenwechsel falsch")
    reached = set(sources)
    todo = list(sources)
    while todo:
        for v in adj.get(todo.pop(), []):
            if v not in reached:
                reached.add(v)
                todo.append(v)
    demands = {}
    for key, last in net["lasten"].items():
        i = last["knoten"]
        if i not in reached:
            errors.append("Quelle-Last-Verbindung fehlt: " + key)
        demands[i] = demands.get(i, 0.0) + last["strom_A"]
    for i in balance:
        if i not in sources and abs(balance[i] - demands.get(i, 0.0)) > 1e-7:
            errors.append("Flusserhaltung verletzt")
    supplied = sum(-balance.get(i, 0.0) + demands.get(i, 0.0) for i in sources)
    if abs(supplied - sum(demands.values())) > 1e-7:
        errors.append("Quellenfluss passt nicht zur Lastsumme")
    if set(adj) - reached:
        errors.append("Unverbundene Knoten")

    indegree = {i: 0 for i in adj}
    for vs in adj.values():
        for v in vs:
            indegree[v] += 1
    todo = [i for i, degree in indegree.items() if degree == 0]
    count = 0
    while todo:
        count += 1
        for v in adj[todo.pop()]:
            indegree[v] -= 1
            if indegree[v] == 0:
                todo.append(v)
    if count != len(adj):
        errors.append("Zyklus im Flussbaum")
    return errors


def schema_errors(value, schema, root, path="$"):
    if "$ref" in schema:
        target = root
        for key in schema["$ref"].split("/")[1:]:
            target = target[key]
        return schema_errors(value, target, root, path)
    errors = []
    types = dict(
        object=dict,
        array=list,
        string=str,
        number=(int, float),
        integer=int,
        boolean=bool,
    )
    typ = schema.get("type")
    if typ and (
        not isinstance(value, types[typ])
        or typ in ("number", "integer")
        and isinstance(value, bool)
    ):
        return [path + ": falscher Typ " + typ]
    if typ in ("number", "integer") and not math.isfinite(value):
        errors.append(path + ": nicht endlich")
    if "const" in schema and value != schema["const"]:
        errors.append(path + ": falsche Konstante")
    if "enum" in schema and value not in schema["enum"]:
        errors.append(path + ": unbekannter Wert")
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if "minimum" in schema and value < schema["minimum"]:
            errors.append(path + ": zu klein")
        if "exclusiveMinimum" in schema and value <= schema["exclusiveMinimum"]:
            errors.append(path + ": nicht positiv")
    if isinstance(value, dict):
        errors.extend(
            path + ": fehlt " + k for k in schema.get("required", []) if k not in value
        )
        for k, sub in schema.get("properties", {}).items():
            if k in value:
                errors.extend(schema_errors(value[k], sub, root, path + "." + k))
        extra = schema.get("additionalProperties")
        if isinstance(extra, dict):
            for k, v in value.items():
                if k not in schema.get("properties", {}):
                    errors.extend(schema_errors(v, extra, root, path + "." + k))
    if isinstance(value, list):
        if len(value) < schema.get("minItems", 0) or len(value) > schema.get(
            "maxItems", math.inf
        ):
            errors.append(path + ": falsche Länge")
        for i, v in enumerate(value):
            errors.extend(
                schema_errors(v, schema.get("items", {}), root, f"{path}[{i}]")
            )
    return errors


def pruefe(design, strict=False):
    schema = json.loads(SCHEMA.read_text())
    errors = schema_errors(design, schema, schema)
    if errors:
        return errors
    for net in design["netze"]:
        if any(not k.isdecimal() for k in net["knoten"]):
            errors.append(net["netz"] + ": ungueltige Knotenkennung")
    if errors:
        return errors
    for net in design["netze"]:
        n = net["netz"]
        if not net["quellen_pads"]:
            errors.append(n + ": Quelle fehlt")
        if not net["lasten"]:
            errors.append(n + ": Lasten fehlen")
        for key, last in net["lasten"].items():
            value = last.get("strom_A") if isinstance(last, dict) else None
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
                or value <= 0
            ):
                errors.append(n + ": ungueltige Last " + key)
        for c in net["korridore"]:
            if c["netz"] != n:
                errors.append(n + ": Netzzuordnung falsch")
            if strict and c["status"] != "entwurf":
                errors.append(n + ": Zielbreite beschnitten")
            if strict and (c["beschnitten_mm2"] > 1e-8 or c["verbindung_ok"] is not True):
                errors.append(n + ": Endgeometrie verletzt Breite/Verbindung")
            for a, b in zip(c["mittellinie"], c["mittellinie"][1:]):
                dx, dy = abs(b[0] - a[0]), abs(b[1] - a[1])
                if min(dx, dy) > 1e-7 and abs(dx - dy) > 1e-7:
                    errors.append(n + ": Mittellinie nicht 0/45/90")
        for f in net["via_felder"]:
            if f["netz"] != n or f["via_d_mm"] <= f["drill_mm"]:
                errors.append(n + ": Via-Geometrie/Netz falsch")
            if f["status"] == "legal" and len(f["positionen"]) != f["anzahl_soll"]:
                errors.append(n + ": unvollständiges legales Feld")
            if strict and f["status"] != "legal":
                errors.append(n + ": Via-Platz fehlt")
        if strict:
            errors.extend(n + ": " + e for e in verbindungsfehler(net))

    holes = [(f, p) for net in design["netze"] for f in net["via_felder"]
             for p in f["positionen"]]
    for i, (a, p) in enumerate(holes):
        for b, q in holes[i + 1:]:
            if math.dist(p, q) + 1e-7 < (a["drill_mm"] + b["drill_mm"]) / 2 + max(
                a["bohrabstand_mm"], b["bohrabstand_mm"]
            ):
                errors.append(a["netz"] + ": Bohrabstand verletzt")
    if strict:
        actual = {n["netz"] for n in design["netze"]}
        expected = set(design["erwartete_lastnetze"]) | {
            n for n, i in design["diagnose"].items() if i.get("status") != "keine_last"
        }
        errors.extend(n + ": Entwurf fehlt" for n in expected - actual)
        errors.extend(n + ": positive Last als keine_last" for n in expected
                      if design["diagnose"].get(n, {}).get("status") == "keine_last")
        errors.extend(
            n + ": " + i["fehler"]
            for n, i in design["diagnose"].items()
            if i.get("fehler")
        )
    return sorted(set(errors))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("file", type=Path)
    ap.add_argument("--strict", action="store_true")
    a = ap.parse_args()
    errors = pruefe(json.loads(a.file.read_text()), a.strict)
    print(
        json.dumps(
            dict(schema_ok=not errors, freigabe=False, fehler=errors),
            ensure_ascii=False,
            indent=1,
        )
    )
    raise SystemExit(bool(errors))
