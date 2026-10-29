
import argparse
import json
import math
from pathlib import Path

SCHEMA = Path(__file__).with_name("corridors.schema.json")


def connection_error(net):
    errors = []
    nodes = net["nodes"]
    sources = set(net["sources_nodes"])
    mapping = net["sources_connections"]
    if set(mapping) != set(net["sources_pads"]) or sources != {
        i for ids in mapping.values() for i in ids
    } or not sources:
        errors.append("Source connection missing/wrong")
    balance = {int(i): 0.0 for i in nodes}
    adj = {int(i): [] for i in nodes}

    def edge(u, v, I):
        if str(u) not in nodes or str(v) not in nodes or u == v:
            errors.append("Edge node missing/wrong")
            return False
        adj[u].append(v)
        balance[u] -= I
        balance[v] += I
        return True

    for c in net["corridors"]:
        if edge(c["from_"], c["until"], c["current_A"]):
            a, b = [nodes[str(i)] for i in (c["from_"], c["until"])]
            if a["layer_name"] != c["layer_name"] or b["layer_name"] != c["layer_name"] or any(
                math.dist(p, q) > 1e-7 for p, q in
                ((a["xy"], c["centerline"][0]), (b["xy"], c["centerline"][-1]))
            ):
                errors.append("Corridor connection wrong")
    for f in net["via_fields"]:
        if not all(f.get("contact_layers", {}).get(L) is True for L in f["cu_layers"]):
            errors.append("Via copper contact missing")
        for w in f["change"]:
            if edge(w["from_"], w["until"], w["current_A"]):
                a, b = [nodes[str(i)] for i in (w["from_"], w["until"])]
                if a["layer_name"] == b["layer_name"] or {a["layer_name"], b["layer_name"]} - set(f["cu_layers"]) or any(
                    math.dist(p["xy"], f["middle"]) > 1e-7 for p in (a, b)
                ):
                    errors.append("Layer change wrong")
    reached = set(sources)
    todo = list(sources)
    while todo:
        for v in adj.get(todo.pop(), []):
            if v not in reached:
                reached.add(v)
                todo.append(v)
    demands = {}
    for key, load in net["load_list"].items():
        i = load["nodes"]
        if i not in reached:
            errors.append("Source-load connection missing: " + key)
        demands[i] = demands.get(i, 0.0) + load["current_A"]
    for i in balance:
        if i not in sources and abs(balance[i] - demands.get(i, 0.0)) > 1e-7:
            errors.append("Flow conservation violated")
    supplied = sum(-balance.get(i, 0.0) + demands.get(i, 0.0) for i in sources)
    if abs(supplied - sum(demands.values())) > 1e-7:
        errors.append("Source flow does not match the load sum")
    if set(adj) - reached:
        errors.append("Unconnected nodes")

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
        errors.append("Cycle in the flow tree")
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
    type_ = schema.get("type")
    if type_ and (
        not isinstance(value, types[type_])
        or type_ in ("number", "integer")
        and isinstance(value, bool)
    ):
        return [path + ": wrong type " + type_]
    if type_ in ("number", "integer") and not math.isfinite(value):
        errors.append(path + ": not finite")
    if "const" in schema and value != schema["const"]:
        errors.append(path + ": wrong constant")
    if "enum" in schema and value not in schema["enum"]:
        errors.append(path + ": unknown value")
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if "minimum" in schema and value < schema["minimum"]:
            errors.append(path + ": too small")
        if "exclusiveMinimum" in schema and value <= schema["exclusiveMinimum"]:
            errors.append(path + ": not positive")
    if isinstance(value, dict):
        errors.extend(
            path + ": missing " + k for k in schema.get("required", []) if k not in value
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
            errors.append(path + ": wrong length")
        for i, v in enumerate(value):
            errors.extend(
                schema_errors(v, schema.get("items", {}), root, f"{path}[{i}]")
            )
    return errors


def check(design, strict=False):
    schema = json.loads(SCHEMA.read_text())
    errors = schema_errors(design, schema, schema)
    if errors:
        return errors
    for net in design["net_list"]:
        if any(not k.isdecimal() for k in net["nodes"]):
            errors.append(net["net_name"] + ": invalid node identifier")
    if errors:
        return errors
    for net in design["net_list"]:
        n = net["net_name"]
        if not net["sources_pads"]:
            errors.append(n + ": source missing")
        if not net["load_list"]:
            errors.append(n + ": loads missing")
        for key, load in net["load_list"].items():
            value = load.get("current_A") if isinstance(load, dict) else None
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
                or value <= 0
            ):
                errors.append(n + ": invalid load " + key)
        for c in net["corridors"]:
            if c["net_name"] != n:
                errors.append(n + ": net assignment wrong")
            if strict and c["status"] != "draft":
                errors.append(n + ": target width clipped")
            if strict and (c["trimmed_mm2"] > 1e-8 or c["connection_ok"] is not True):
                errors.append(n + ": final geometry violates width/connection")
            for a, b in zip(c["centerline"], c["centerline"][1:]):
                dx, dy = abs(b[0] - a[0]), abs(b[1] - a[1])
                if min(dx, dy) > 1e-7 and abs(dx - dy) > 1e-7:
                    errors.append(n + ": centerline not 0/45/90")
        for f in net["via_fields"]:
            if f["net_name"] != n or f["via_d_mm"] <= f["drill_mm"]:
                errors.append(n + ": via geometry/net wrong")
            if f["status"] == "legal" and len(f["positions"]) != f["count_nominal"]:
                errors.append(n + ": incomplete legal field")
            if strict and f["status"] != "legal":
                errors.append(n + ": via room missing")
        if strict:
            errors.extend(n + ": " + e for e in connection_error(net))

    holes = [(f, p) for net in design["net_list"] for f in net["via_fields"]
             for p in f["positions"]]
    for i, (a, p) in enumerate(holes):
        for b, q in holes[i + 1:]:
            if math.dist(p, q) + 1e-7 < (a["drill_mm"] + b["drill_mm"]) / 2 + max(
                a["drill_spacing_mm"], b["drill_spacing_mm"]
            ):
                errors.append(a["net_name"] + ": drill spacing violated")
    if strict:
        actual = {n["net_name"] for n in design["net_list"]}
        expected = set(design["expected_load_nets"]) | {
            n for n, i in design["diagnose"].items() if i.get("status") != "no_load"
        }
        errors.extend(n + ": draft missing" for n in expected - actual)
        errors.extend(n + ": positive load as no_load" for n in expected
                      if design["diagnose"].get(n, {}).get("status") == "no_load")
        errors.extend(
            n + ": " + i["faults"]
            for n, i in design["diagnose"].items()
            if i.get("faults")
        )
    return sorted(set(errors))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("file", type=Path)
    ap.add_argument("--strict", action="store_true")
    a = ap.parse_args()
    errors = check(json.loads(a.file.read_text()), a.strict)
    print(
        json.dumps(
            dict(schema_ok=not errors, release=False, faults=errors),
            ensure_ascii=False,
            indent=1,
        )
    )
    raise SystemExit(bool(errors))
