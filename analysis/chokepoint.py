
import json
import math
import sys
from pathlib import Path

import numpy as np
import shapely
from numba import njit
from scipy import ndimage
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.flow import pad_geo


@njit(cache=True)
def _maximin(order, distance, source, target, height, width):
    size = height * width
    parent = np.full(size, -1, np.int64)
    rank = np.zeros(size, np.uint8)
    flags = np.zeros(size, np.uint8)
    for p in order:
        parent[p] = p
        flags[p] = (1 if source[p] else 0) | (2 if target[p] else 0)
        y, x = p // width, p % width
        for q in (
            p - 1 if x else -1,
            p + 1 if x + 1 < width else -1,
            p - width if y else -1,
            p + width if y + 1 < height else -1,
        ):
            if q < 0 or parent[q] < 0:
                continue
            a, b = p, q
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            while parent[b] != b:
                parent[b] = parent[parent[b]]
                b = parent[b]
            if a != b:
                if rank[a] < rank[b]:
                    a, b = b, a
                parent[b] = a
                flags[a] |= flags[b]
                if rank[a] == rank[b]:
                    rank[a] += 1
        root = p
        while parent[root] != root:
            root = parent[root]
        if flags[root] == 3:
            return 2 * distance[p], p
    return 0.0, -1


def chokepoint(copper, source, target, h=0.01):
    if not math.isfinite(h) or h <= 0 or copper.is_empty:
        raise ValueError("Empty copper or invalid grid")

    pieces = [copper] if copper.geom_type == "Polygon" else list(copper.geoms)
    if not any(
        p.intersection(source).area > 0 and p.intersection(target).area > 0
        for p in pieces
    ):
        raise ValueError("Contacts in exact copper disconnected or missing")
    x0, y0, x1, y1 = copper.bounds
    xs = np.arange(x0 - 2 * h, x1 + 2 * h, h)
    ys = np.arange(y0 - 2 * h, y1 + 2 * h, h)
    if len(xs) * len(ys) > 8_000_000:
        raise ValueError("Bottleneck grid >8 million cells")
    xx, yy = np.meshgrid(xs, ys)
    mask = shapely.contains_xy(copper, xx, yy)
    distance = ndimage.distance_transform_edt(mask, sampling=h)
    src = mask & shapely.contains_xy(source, xx, yy)
    dst = mask & shapely.contains_xy(target, xx, yy)
    if not src.any() or not dst.any():
        raise ValueError("Contact missing in grid")
    indices = np.flatnonzero(mask)

    order = indices[np.argsort(-distance.ravel()[indices], kind="stable")]
    width, p = _maximin(order, distance.ravel(), src.ravel(), dst.ravel(), *mask.shape)
    if p < 0:
        raise ValueError("No grid path")
    return dict(
        width_mm=float(width),
        location_mm=[float(xx.ravel()[p]), float(yy.ravel()[p])],
        h_mm=h,
    )


def copper_map(data, net, layer):
    return unary_union(
        [
            Polygon(outer, holes).buffer(0)
            for outer, holes in data["nets"].get(net, {}).get(layer, [])
        ]
    )


def measure(board, contacts, copper, h=0.01):
    pads = {str(p["ref"]) + ":" + str(p["num"]): p for p in board["pads"]}
    results = {}
    for name, spec in sorted(contacts.items()):
        layer = spec.get("layer", "F")

        def contact(keys):
            if any(pads[k]["net"] != spec["net"] for k in keys):
                raise ValueError("Contact has different net")
            return unary_union([pad_geo(pads[k])[layer] for k in keys])

        try:
            src = contact(spec["source"])
            dst = (
                box(*spec["goal_box"]) if "goal_box" in spec else contact(spec["goal"])
            )
            results[name] = chokepoint(copper_map(copper, spec["net"], layer), src, dst, h)
        except (ValueError, KeyError) as error:
            results[name] = dict(faults=str(error))
    return dict(
        method="EDT/Union-Find, 4-neighbors, maximin, copper bounding box",
        h_mm=h,
        complete=all("faults" not in r for r in results.values()),
        channels=results,
    )


def main():
    b, contacts, copper, out = sys.argv[1:5]
    opts = dict(a.split("=", 1) for a in sys.argv[5:] if "=" in a)
    result = measure(
        *[json.loads(Path(p).read_text()) for p in (b, contacts, copper)],
        h=float(opts.get("h", ".01")),
    )
    Path(out).write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print({n: round(r.get("width_mm", 0), 3) for n, r in result["channels"].items()})
    return 0 if result["complete"] else 2


if __name__ == "__main__":
    sys.exit(main())
