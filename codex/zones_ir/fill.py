
from __future__ import annotations
from collections import defaultdict
from time import perf_counter
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import unary_union
from shapely.strtree import STRtree
from .contours import zones_contours


def polygons(g):
    if g.is_empty:
        return []
    if g.geom_type == "Polygon":
        return [g]
    return [p for q in getattr(g, "geoms", []) for p in polygons(q)]


def cu_layers(B, K):
    return B.get("layers") or ["F"] + [
        "In%d" % i for i in range(1, int(B.get("cu_layers") or K.get("cu_layers") or 2) - 1)
    ] + ["B"]


def fill_zones(B, R, K, contours, *, clearance=0.15, min_thickness=0.2):
    t0 = perf_counter()
    if clearance < 0 or min_thickness < 0:
        raise ValueError("Spacing and minimum width must be non-negative")
    layers = cu_layers(B, K)
    nets = {
        n
        for n, k in K["net_list"].items()
        if k.get("category") in ("current_val", "supply", "sw")
    }
    raw = defaultdict(list)
    anchors = defaultdict(list)
    barrels = []
    via_d, via_drill = map(
        float, (R.get("options") or {}).get("via", "0.45/0.20").split("/")
    )
    holes = []
    for p in B["pads"]:
        for L, qs in p["polys"].items():
            for q in qs:
                g = Polygon(q).buffer(0)
                raw[p["net"], L].append(g)
                if p["net"] in nets:
                    anchors[p["net"], L].append(g)
        if p.get("drill", 0) > 0:
            holes.append(
                (p["net"], Point(p["x"], p["y"]).buffer(p["drill"] / 2, quad_segs=12))
            )
            barrels.append((p["net"], Point(p["x"], p["y"]), p["layers"]))
    for t in R.get("tracks", []):
        if "items" in t:
            raise ValueError(
                "Arcs/items need tessellated geometry; fill refused"
            )
        if len(t["w"]) != len(t["pts"]) - 1 or any(w <= 0 for w in t["w"]):
            raise ValueError("Invalid segment widths")
        for a, b, w in zip(t["pts"][:-1], t["pts"][1:], t["w"]):
            raw[t["net"], t["layer"]].append(
                LineString([a, b]).buffer(w / 2, quad_segs=12)
            )
    for v in R.get("vias", []):
        pt = Point(v["x"], v["y"])
        ls = v.get("layers") or layers
        g = pt.buffer(v.get("diameter", v.get("d", via_d)) / 2, quad_segs=12)
        for L in ls:
            raw[v["net"], L].append(g)
        holes.append((v["net"], pt.buffer(v.get("drill", via_drill) / 2, quad_segs=12)))
        barrels.append((v["net"], pt, ls))
    obs = defaultdict(list)
    owners = defaultdict(list)
    for (net, L), gg in raw.items():

        for g in gg:
            obs[L].append(g.buffer(clearance, quad_segs=8))
            owners[L].append(net)
    for net, g in holes:
        for L in layers:
            obs[L].append(g.buffer(clearance, quad_segs=8))
            owners[L].append(net)
    trees = {L: STRtree(gs) for L, gs in obs.items()}
    candidates = []
    removed = 0
    outlines = [Polygon(z["pts"], z.get("holes", [])).buffer(0) for z in contours]
    for zi, z in enumerate(contours):
        if z["net"] not in nets:
            continue
        if not z.get("full", False):
            raise ValueError("Thermal connection not supported")
        L = z["layer"]
        g = outlines[zi]
        foreign = []
        if L in trees:
            ids = trees[L].query(g, predicate="intersects")
            foreign = [obs[L][i] for i in ids if owners[L][i] != z["net"]]
        foreign += [
            outlines[j].buffer(clearance, quad_segs=8)
            for j, q in enumerate(contours)
            if q["layer"] == L
            and q["net"] != z["net"]
            and q.get("prio", 0) > z.get("prio", 0)
            and outlines[j].distance(g) <= clearance
        ]
        foreign += [
            Polygon(q["pts"], q.get("holes", []))
            for q in B.get("zone_keepouts", [])
            if q["layer"] == L
        ]
        if foreign:
            g = g.difference(unary_union(foreign))
        if min_thickness:
            r = min_thickness / 2
            g = g.buffer(-r, quad_segs=8).buffer(r, quad_segs=8).intersection(g)
        for part in polygons(g):
            candidates.append((z, part))
    tgeo = perf_counter()

    by = defaultdict(list)
    for i, (z, g) in enumerate(candidates):
        by[z["net"], z["layer"]].append(g)
    for key in raw:
        if key[0] in nets:
            by.setdefault(key, [])
    comp = {}
    parent = []
    rooted = set()
    trees2 = {}

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for key, gs in by.items():
        pcs = polygons(unary_union(gs + raw[key]))
        ids = []
        for g in pcs:
            i = len(parent)
            parent.append(i)
            ids.append(i)
            if any(g.intersects(a) for a in anchors[key]):
                rooted.add(i)
        comp[key] = (pcs, ids)
        trees2[key] = STRtree(pcs)
    for net, pt, ls in barrels:
        if net not in nets:
            continue
        ids = []
        for L in ls:
            key = net, L
            if key in trees2:
                ids += [
                    comp[key][1][j]
                    for j in trees2[key].query(pt, predicate="intersects")
                ]
        for i in ids[1:]:
            parent[root(i)] = root(ids[0])
    good = {root(i) for i in rooted}
    out = []
    for z, g in candidates:
        key = z["net"], z["layer"]
        ids = trees2[key].query(g, predicate="intersects")
        if not any(root(comp[key][1][j]) in good for j in ids):
            removed += 1
            continue
        out.append(
            dict(
                net=z["net"],
                layer=z["layer"],
                pts=list(map(list, g.exterior.coords)),
                holes=[list(map(list, r.coords)) for r in g.interiors],
                filled=True,
                estimated=True,
            )
        )
    return out, dict(
        fill_s=perf_counter() - t0,
        geometry_s=tgeo - t0,
        islands_removed=removed,
        zone_list=len(out),
        clearance_mm=clearance,
        min_thickness_mm=min_thickness,
    )


def with_fill(B, R, K, *, options=None):
    opt = {"ZONE_EDGE": "0.25", "ZONE_GND_SIGNAL": "1", **(options or {})}
    t = perf_counter()
    gnd = (R.get("options") or {}).get("gnd_layers", "In1")
    z = zones_contours(B, R, K, gnd_layers=gnd, options=opt)
    outlines_s = perf_counter() - t
    z, info = fill_zones(
        B,
        R,
        K,
        z,
        clearance=float(opt.get("ZCLR", "0.15")),
        min_thickness=float(opt.get("ZONE_MIN_THICKNESS", "0.2")),
    )
    info.update(
        contours_s=outlines_s, total_s=perf_counter() - t, model="shapely_estimated_v1"
    )
    return dict(R, zones=z), info
