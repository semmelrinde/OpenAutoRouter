""

from __future__ import annotations
import argparse, collections, hashlib, html, json, math, sys, time
from pathlib import Path
import numpy as np
import scipy.sparse as sp
from scipy.sparse.csgraph import connected_components
from scipy.sparse.linalg import splu
import shapely
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import unary_union

RHO = 1.72e-8


class ModelError(ValueError):
    pass


def layers_of(B, R, K):
    declared = B.get("layers")
    if isinstance(declared, list) and all(isinstance(x, str) for x in declared):
        return declared
    number = int(B.get("lagen") or K.get("lagen") or 2)
    return ["F"] + ["In" + str(i) for i in range(1, number - 1)] + ["B"]


def geometry(B, R, net, layers, via_diameter):
    from gemeinsam.zonen import vereinheitlichen
    R = vereinheitlichen(R)
    shapes = collections.defaultdict(list)
    pads = {}
    barrels = []
    warnings = []
    for p in B["pads"]:
        if p["net"] != net:
            continue
        key = p["ref"] + ":" + str(p["num"])
        pads.setdefault(key, []).append(p)
        for layer, polys in p["polys"].items():
            for points in polys:
                if len(points) >= 3:
                    shapes[layer].append(Polygon(points).buffer(0))
        if p.get("drill", 0) > 0:
            barrels.append(
                dict(
                    x=p["x"], y=p["y"], drill=p["drill"], layers=p["layers"], kind="tht"
                )
            )
    widths = []
    for t in R.get("tracks", []):
        if t["net"] != net:
            continue
        if "items" in t:
            raise ModelError(
                "Bogen-/items-Format noch nicht unterstützt; nicht still auslassen."
            )
        if len(t["w"]) != len(t["pts"]) - 1:
            raise ModelError("Breitenzahl passt nicht zu Segmenten")
        for a, b, w in zip(t["pts"][:-1], t["pts"][1:], t["w"]):
            if w <= 0:
                raise ModelError("Nichtpositive Bahnbreite")
            shapes[t["layer"]].append(LineString([a, b]).buffer(w / 2, quad_segs=12))
            widths.append(w)
    for zone in R.get("zones", []):
        if zone.get("net") != net:
            continue
        if not zone.get("filled", False):
            raise ModelError(
                "Zonen nur mit filled=true und finalen Kupferpolygonen auswerten"
            )
        shapes[zone["layer"]].append(
            Polygon(zone["pts"], zone.get("holes", [])).buffer(0)
        )
    for v in R.get("vias", []):
        if v["net"] != net:
            continue
        ls = v.get("layers") or layers
        if any(l not in layers for l in ls):
            raise ModelError("Unbekannte Via-Lage")
        vv = dict(v)
        vv["layers"] = ls
        vv["kind"] = "via"
        barrels.append(vv)
        for l in ls:
            shapes[l].append(
                Point(v["x"], v["y"]).buffer(
                    v.get("diameter", via_diameter) / 2, quad_segs=12
                )
            )
    if not shapes:
        raise ModelError("Kein Kupfer für Netz " + net)
    unknown = set(shapes) - set(layers)
    if unknown:
        raise ModelError("Stackup fehlen Lagen: " + str(unknown))

    if barrels:
        warnings.append(
            "Via-/THT-Anschlussflächen als volle Scheiben/Padpolygone angenähert; Bohrloch lokal nicht ausgeschnitten."
        )
    return (
        {l: unary_union(s).buffer(0) for l, s in shapes.items()},
        pads,
        barrels,
        widths,
        warnings,
    )


class Grid:
    def __init__(
        self,
        polys,
        pads,
        barrels,
        layers,
        res=0.1,
        stackup=None,
        rho=RHO,
        via_drill=0.2,
        plating_um=25,
        max_nodes=1500000,
    ):
        if res <= 0 or rho <= 0 or plating_um <= 0:
            raise ModelError("Raster/Materialparameter müssen positiv sein")
        self.res = res
        self.rho = rho
        self.layers = [l for l in layers if l in polys]
        self.polys = polys
        self.pads = pads
        self.stackup = stackup or {}
        self.assumptions = []
        self.thickness = {}
        self.z = {}
        for i, l in enumerate(layers):
            v = self.stackup.get(l, {})
            if "copper_um" not in v:
                self.assumptions.append(l + ": Kupferdicke angenommen 35 µm")
            if "z_mm" not in v and barrels:
                self.assumptions.append(
                    l + ": z aus gleichmäßigem 1,6-mm-Stackup angenommen"
                )
            self.thickness[l] = float(v.get("copper_um", 35)) * 1e-6
            self.z[l] = float(v.get("z_mm", 1.6 * i / max(1, len(layers) - 1)))
            if self.thickness[l] <= 0:
                raise ModelError("Kupferdicke nichtpositiv")
        bounds = unary_union(list(polys.values())).bounds
        self.x0 = math.floor(bounds[0] / res) * res
        self.y0 = math.floor(bounds[1] / res) * res
        nx = math.ceil((bounds[2] - self.x0) / res)
        ny = math.ceil((bounds[3] - self.y0) / res)
        if nx * ny * len(self.layers) > max_nodes * 4:
            raise ModelError("Raster zu groß; explizites Speicherlimit erreicht")
        xx, yy = np.meshgrid(
            self.x0 + (np.arange(nx) + 0.5) * res, self.y0 + (np.arange(ny) + 0.5) * res
        )
        self.xx = xx
        self.yy = yy
        self.idx = {}
        self.coords = []
        self.node_layers = []
        n = 0
        for l in self.layers:
            m = shapely.intersects_xy(polys[l], xx, yy)
            ix = np.full(xx.shape, -1, int)
            count = int(m.sum())
            ix[m] = np.arange(n, n + count)
            n += count
            self.idx[l] = ix
            self.coords.extend(zip(xx[m], yy[m]))
            self.node_layers.extend([l] * count)
        if n > max_nodes:
            raise ModelError("Zu viele Kupferknoten")
        if n == 0:
            raise ModelError("Kupfer unter Rasterauflösung")
        self.n = n
        self.coords = np.asarray(self.coords)
        a = []
        b = []
        gs = []
        areas = []
        ls = []
        kinds = []


        for l in self.layers:
            ix = self.idx[l]
            copper = polys[l]
            connected_copper = copper.buffer(1e-10)
            shapely.prepare(copper)
            shapely.prepare(connected_copper)
            for axis in (0, 1):
                aa, bb = (
                    (ix[:, :-1], ix[:, 1:]) if axis == 0 else (ix[:-1, :], ix[1:, :])
                )
                good = (aa >= 0) & (bb >= 0)
                an = aa[good]
                bn = bb[good]
                if not len(an):
                    continue
                pa = self.coords[an]
                pb = self.coords[bn]
                mid = (pa + pb) / 2
                offset = np.array([0, res / 2]) if axis == 0 else np.array([res / 2, 0])
                faces = shapely.linestrings(
                    np.stack([mid - offset, mid + offset], axis=1)
                )
                interior = shapely.covers(copper, faces)
                facewidth = np.full(len(faces), res)
                boundary = ~interior
                facewidth[boundary] = shapely.length(shapely.intersection(copper, faces[boundary]))
                connected = shapely.covers(
                    connected_copper,
                    shapely.linestrings(np.stack([pa, pb], axis=1)),
                )
                good = (facewidth > 1e-10) & connected
                an = an[good]
                bn = bn[good]
                width = facewidth[good]
                a.extend(an)
                b.extend(bn)
                gs.extend(self.thickness[l] / rho * width / res)
                areas.extend(width * self.thickness[l] * 1000)
                ls.extend([l] * len(an))
                kinds.extend(["copper"] * len(an))
        for v in barrels:
            chain = []
            for l in sorted(v["layers"], key=lambda l: self.z.get(l, 0)):
                if l not in self.idx:
                    continue
                i = int((v["x"] - self.x0) / res)
                j = int((v["y"] - self.y0) / res)
                ix = self.idx[l]
                candidates = ix[
                    max(0, j - 1) : min(ix.shape[0], j + 2),
                    max(0, i - 1) : min(ix.shape[1], i + 2),
                ].ravel()
                candidates = candidates[candidates >= 0]
                if not len(candidates):
                    raise ModelError("Via/Bohrung nicht im Raster; Auflösung erhöhen")
                node = int(
                    candidates[
                        np.argmin(
                            np.sum(
                                (self.coords[candidates] - [v["x"], v["y"]]) ** 2,
                                axis=1,
                            )
                        )
                    ]
                )
                chain.append((l, node))
            if len(chain) < 2:
                continue
            drill = float(v.get("drill", via_drill))
            plating = float(v.get("plating_um", plating_um)) * 1e-3
            if drill <= 0 or plating <= 0:
                raise ModelError("Ungültiger Via-Querschnitt")
            if "drill" not in v:
                self.assumptions.append("Via-Bohrung angenommen %.3f mm" % drill)
            if "plating_um" not in v:
                self.assumptions.append(
                    "Via-Hülse angenommen %.1f µm" % (plating * 1000)
                )
            area = math.pi * ((drill / 2 + plating) ** 2 - (drill / 2) ** 2)
            for (la, na), (lb, nb) in zip(chain[:-1], chain[1:]):
                length = abs(self.z[lb] - self.z[la])
                if length <= 0:
                    raise ModelError("Via-Lagen haben gleiche Höhe")
                a.append(na)
                b.append(nb)
                gs.append(area * 1e-6 / (rho * length * 1e-3))
                areas.append(area)
                ls.append(la + "-" + lb)
                kinds.append(v["kind"])
        self.a = np.asarray(a, int)
        self.b = np.asarray(b, int)
        self.g = np.asarray(gs, float)
        self.area = np.asarray(areas)
        self.edge_layers = ls
        self.kinds = kinds
        if not len(self.g):
            raise ModelError("Kein aufgelöster leitfähiger Pfad")
        self.pad_nodes = {}
        for key, plist in pads.items():
            nodes = []
            for p in plist:
                for l, qs in p["polys"].items():
                    if l not in self.idx:
                        continue
                    geo = unary_union([Polygon(q) for q in qs])
                    ix = self.idx[l]
                    mask = shapely.intersects_xy(geo, xx, yy) & (ix >= 0)
                    nodes.extend(ix[mask])
            nodes = np.unique(nodes)
            if not len(nodes):
                raise ModelError("Pad " + key + " unter Rasterauflösung")
            self.pad_nodes[key] = nodes

    def laplacian(self, g=None):
        g = self.g if g is None else g
        return sp.coo_matrix(
            (
                np.r_[g, g, -g, -g],
                (
                    np.r_[self.a, self.b, self.a, self.b],
                    np.r_[self.a, self.b, self.b, self.a],
                ),
            ),
            shape=(self.n, self.n),
        ).tocsc()

    def solve(self, sources, loads, g=None):
        g = self.g if g is None else g

        unknown = (set(sources) | set(loads)) - set(self.pad_nodes)
        if unknown:
            raise ModelError("Unbekannte Pads: " + ", ".join(sorted(unknown)))
        if not sources:
            raise ModelError("Kein Quellenpad")
        if any(not np.isfinite(v) or v < 0 for v in loads.values()):
            raise ModelError("Lasten müssen endlich und nichtnegativ sein")
        boundary = np.unique(np.concatenate([self.pad_nodes[k] for k in sources]))
        L = self.laplacian(g)
        _, labels = connected_components(L, directed=False)
        reached = np.isin(labels, labels[boundary])
        lost = [
            k
            for k, I in loads.items()
            if I > 0 and not np.all(reached[self.pad_nodes[k]])
        ]
        if lost:
            raise ModelError("Von Quelle getrennte Verbraucher: " + ", ".join(lost))
        if any(k in sources and I > 0 for k, I in loads.items()):
            raise ModelError("Quellenpad zugleich positive Last")
        free = reached.copy()
        free[boundary] = False
        rhs = np.zeros(self.n)
        for k, I in loads.items():
            rhs[self.pad_nodes[k]] += I / len(self.pad_nodes[k])
        if np.any(rhs[boundary] > 0):
            raise ModelError(
                "Verbraucherkontakt überlappt idealen Quellenkontakt im Raster"
            )
        lu = splu(L[free][:, free])
        u = np.full(self.n, np.nan)
        u[reached] = 0
        u[free] = lu.solve(rhs[free])

        drops = {
            k: float(np.mean(u[self.pad_nodes[k]])) for k, I in loads.items() if I > 0
        }
        residual = L @ np.nan_to_num(u) - rhs
        source_current = -float(residual[boundary].sum())
        err = float(np.max(np.abs(residual[free])))
        edge_current = g * (np.nan_to_num(u[self.b]) - np.nan_to_num(u[self.a]))
        edge_current[~(reached[self.a] & reached[self.b])] = np.nan
        density = np.abs(edge_current) / self.area
        result = dict(
            drop_V=drops,
            total_current_A=float(sum(loads.values())),
            source_current_A=source_current,
            kcl_max_abs_A=err,
            power_W=float(np.nansum(edge_current**2 / g)),
            floating_nodes=int((~reached).sum()),
        )
        return result, dict(
            u=u,
            free=free,
            lu=lu,
            L=L,
            current=edge_current,
            density=density,
            reached=reached,
            g=g,
        )

    def sensitivity(self, state, target):
        q = np.zeros(self.n)
        q[self.pad_nodes[target]] = 1 / len(self.pad_nodes[target])
        lam = np.zeros(self.n)
        lam[state["free"]] = state["lu"].solve(q[state["free"]])
        u = np.nan_to_num(state["u"])
        benefit = (lam[self.a] - lam[self.b]) * (u[self.a] - u[self.b])

        return benefit

def write_maps(grid, state, path):
    data = {
        "x_mm": grid.xx,
        "y_mm": grid.yy,
        "edge_current_A": state["current"],
        "edge_density_A_mm2": state["density"],
        "edge_a": grid.a,
        "edge_b": grid.b,
        "edge_g_S": grid.g,
        "node_xy_mm": grid.coords,
    }
    for l, ix in grid.idx.items():
        density = np.zeros(grid.n)
        np.maximum.at(density, grid.a, np.nan_to_num(state["density"]))
        np.maximum.at(density, grid.b, np.nan_to_num(state["density"]))
        density[~state["reached"]] = np.nan
        values = np.full(ix.shape, np.nan)
        values[ix >= 0] = density[ix[ix >= 0]]
        u = np.full(ix.shape, np.nan)
        u[ix >= 0] = state["u"][ix[ix >= 0]]
        data[l + "_J_A_mm2"] = values
        data[l + "_drop_V"] = u
        finite = values[np.isfinite(values)]
        maxj = float(finite.max()) if len(finite) else 0
        scale = min(8, max(1, 600 / max(ix.shape)))
        w = ix.shape[1] * scale
        h = ix.shape[0] * scale
        svg = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{w + 30}" height="{h + 75}" viewBox="0 0 {w + 30} {h + 75}">',
            '<rect width="100%" height="100%" fill="white"/>',
            f'<text x="10" y="18" font-family="sans-serif" font-size="12">{html.escape(l)}: Stromdichte, max. {maxj:.4g} A/mm²</text>',
            '<text x="10" y="34" font-family="sans-serif" font-size="10">Modell / ggf. normiertes 1-A-Szenario; lineare Farbskala</text>',
        ]
        for j, i in zip(*np.nonzero(np.isfinite(values))):
            t = values[j, i] / maxj if maxj else 0
            color = f"rgb({int(245 * t)},{int(90 * (1 - t))},{int(230 * (1 - t))})"
            svg.append(
                f'<rect x="{10 + i * scale:.2f}" y="{42 + j * scale:.2f}" width="{scale + 0.01:.2f}" height="{scale + 0.01:.2f}" fill="{color}"/>'
            )
        svg.append(
            f'<text x="10" y="{h + 60}" font-family="sans-serif" font-size="10">Blau: 0; Rot: {maxj:.4g}. Raster {grid.res:g} mm. x rechts, y unten.</text></svg>'
        )
        path.with_name(path.name + "_" + l + ".svg").write_text("\n".join(svg))
    np.savez_compressed(path.with_suffix(".npz"), **data)


def _analyse(B, R, K, res=0.1, outdir=None):
    start = time.perf_counter()
    if outdir is not None:
        outdir = Path(outdir)
        outdir.mkdir(parents=True, exist_ok=True)
    results = {}
    layers = layers_of(B, R, K)
    for net, k in K["netze"].items():
        if k.get("art") not in ("versorgung", "strom", "sw"):
            continue
        f = k.get("fluss")
        item = {
            "netzart": k.get("art"),
            "strommodus": None,
            "last_herkunft": f.get("herkunft") if f else None,
            "drop_V": None,
            "max_drop_V": None,
            "power_W": None,
            "kupferflaeche_mm2": None,
            "hinweis": "DC-Kupfermodell; kein AC/Hot-Loop-, thermischer, GND-Rückweg- oder Fertigungsnachweis.",
        }
        results[net] = item
        try:
            if not f:
                raise ModelError("Stromnetz ohne Fluss-/Lastmodell; Abdeckung unvollständig")
            loads = {s: float(v) for s, v in f.get("last", {}).items()}
            if any(not math.isfinite(v) or v < 0 for v in loads.values()):
                raise ModelError("Lasten müssen endlich und nichtnegativ sein")
            mode = "normiert_1A"
            if f.get("einheit") == "A":
                provenance = f.get("herkunft")
                if not provenance or (isinstance(provenance, dict) and any(
                    not provenance.get(s) for s, v in loads.items() if v > 0
                )):
                    raise ModelError("Ampere-Last ohne vollständige Herkunft")
                mode = "Ampere_mit_Herkunft"
            elif f.get("einheit") not in (None, "relativ"):
                raise ModelError("Unbekannte Last-Einheit: " + str(f["einheit"]))
            item["strommodus"] = mode
            total = sum(loads.values())
            if not math.isfinite(total):
                raise ModelError("Lastsumme nicht endlich")
            if mode == "normiert_1A" and total > 0:
                loads = {s: v / total for s, v in loads.items()}

            via_size = (R.get("optionen") or {}).get("via", "0.45/0.20")
            diameter, drill = map(float, via_size.split("/"))
            if not (0 < drill < diameter and math.isfinite(diameter)):
                raise ModelError("Ungültige Via-Maße")
            polys, pads, barrels, widths, warnings = geometry(B, R, net, layers, diameter)
            if R.get('stromzonen_modell') or R.get('zonen'):


                polys = {l:g.simplify(res/100, preserve_topology=True) for l,g in polys.items()}
                item['konturtoleranz_mm'] = res/100
            unknown = (set(loads) | set(f.get("quelle", []))) - set(pads)
            if unknown:
                raise ModelError("Unbekannte Pads: " + ", ".join(sorted(unknown)))
            item["kupferflaeche_je_lage_mm2"] = {l: float(p.area) for l, p in polys.items()}
            item["kupferflaeche_mm2"] = sum(item["kupferflaeche_je_lage_mm2"].values())
            if total == 0:
                item.update(status="keine_last", drop_V={}, max_drop_V=0.0, power_W=0.0,
                            total_current_A=0.0, source_current_A=0.0,
                            verbraucher={}, schlimmster_verbraucher=None,
                            warnungen=warnings, annahmen=[],
                            hinweis="Stationär keine Last: keine DC-Lösung erforderlich; kein Verbindungs- oder AC-Nachweis.")
                continue
            if widths and min(widths) < 2 * res:
                warnings.append(
                    "Kleinste Bahn < 2 Rasterzellen: Verfeinerungsstudie zwingend vor physikalischer Interpretation."
                )
            grid = Grid(polys, pads, barrels, layers, res, B.get("stackup"), via_drill=drill)
            solution, state = grid.solve(f.get("quelle", []), loads)
            item.update(solution)
            try:
                e = int(np.nanargmax(state["density"])); pos = (grid.coords[grid.a[e]] + grid.coords[grid.b[e]]) / 2
                lage = next((l for l in grid.layers if np.isin(grid.a[e], grid.idx[l])), None)
                item["J_max_ort"] = [round(float(pos[0]), 3), round(float(pos[1]), 3), lage]


                dens = np.nan_to_num(np.asarray(state["density"], float))
                jk = np.zeros(len(grid.coords)); np.maximum.at(jk, grid.a, dens); np.maximum.at(jk, grid.b, dens)
                kanten = np.logspace(-3, 4, 71)
                h, _ = np.histogram(np.clip(jk, kanten[0], kanten[-1] * 0.999), bins=kanten)
                item["J_hist"] = dict(kanten=[float(x) for x in kanten], flaeche_mm2=[float(x) * grid.res ** 2 for x in h])
            except Exception:
                pass
            if R.get("kupferbalken"):
                item["J_max_A_mm2"] = float(np.nanmax(state["density"]))
            if R.get('stromzonen_modell') or R.get('zonen'):


                from analyse.metriken import breite_ipc
                total = solution['total_current_A']
                xx = []
                for li, layer in enumerate(grid.layers):
                    nodes = grid.idx[layer]
                    ids = nodes[nodes >= 0]
                    mask = np.isin(grid.a, ids) & np.isin(grid.b, ids)
                    limit = total / (breite_ipc(total, layer not in ('F', 'B')) *
                                     grid.thickness[layer] * 1000)
                    if np.any(mask):
                        xx.append(float(np.nanmax(state['density'][mask])) / limit)
                item['J_x'] = max(xx, default=0.0)
                item['J_max_A_mm2'] = float(np.nanmax(state['density']))
            item.update(
                status="solved",
                nodes=grid.n,
                edges=len(grid.g),
                annahmen=sorted(set(grid.assumptions)),
                warnungen=warnings,
            )
            item["verbraucher"] = {}
            for pad, drop in solution["drop_V"].items():
                ref = pad.rsplit(":", 1)[0]
                c = item["verbraucher"].setdefault(
                    ref, dict(current_A=0.0, max_drop_V=0.0, pads=[])
                )
                c["current_A"] += loads[pad]
                c["max_drop_V"] = max(c["max_drop_V"], drop)
                c["pads"].append(pad)
            targets = sorted(
                solution["drop_V"], key=solution["drop_V"].get, reverse=True
            )
            item["max_drop_V"] = max(solution["drop_V"].values(), default=0.0)
            item["schlimmster_verbraucher"] = targets[0] if targets else None
            if outdir is None:
                continue
            sens = {}
            for target in targets:
                benefit = grid.sensitivity(state, target)
                length_mm = grid.area * 1e-3 / (grid.g * grid.rho)
                volume_mm3 = grid.area * length_mm
                per_volume = benefit * grid.g / volume_mm3
                indices = np.argsort(-per_volume)[:20]
                sens[target] = []
                for e in indices:
                    if benefit[e] <= 0:
                        continue
                    pos = (grid.coords[grid.a[e]] + grid.coords[grid.b[e]]) / 2
                    sens[target].append(
                        dict(
                            edge=int(e),
                            x_mm=float(pos[0]),
                            y_mm=float(pos[1]),
                            layer=grid.edge_layers[e],
                            kind=grid.kinds[e],
                            minus_dV_dG_V_per_S=float(benefit[e]),
                            minus_dV_dCu_V_per_mm3=float(per_volume[e]),
                            estimated_drop_reduction_V_for_10pct_G=float(
                                0.1 * grid.g[e] * benefit[e]
                            ),
                        )
                    )
            item["sensitivitaet"] = sens
            item["schlimmster_verbraucher"] = targets[0] if targets else None
            item["sensitivitaet_deutung"] = (
                "Lokale Leitwertsteigerung an vorhandenen Rasterkanten; lineare Ableitung, keine Zusage für neues Kupfer/DRC. Negative Wirkungen auf andere Lasten sind möglich."
            )
            slug = hashlib.sha256(net.encode()).hexdigest()[:10]
            prefix = outdir / ("netz_" + slug)
            write_maps(grid, state, prefix)
            item["karten_npz"] = prefix.name + ".npz"
            item["karten_svg"] = [prefix.name + "_" + l + ".svg" for l in grid.layers]
        except (ModelError, ValueError, RuntimeError, TypeError, KeyError) as e:
            item.update(status="error", error=str(e), drop_V=None)
    report = {
        "laufzeit_s": time.perf_counter() - start,
        "raster_mm": res,
        "rho_Ohm_m": RHO,
        "material": "Kupfer bei nominaler Raumtemperatur, kein Temperaturmodell",
        "netze": results,
        "status": "solved"
        if results and all(v["status"] in ("solved", "keine_last") for v in results.values())
        else "incomplete",
        "anzahl_geloest": sum(v["status"] == "solved" for v in results.values()),
        "anzahl_keine_last": sum(v["status"] == "keine_last" for v in results.values()),
        "anzahl_fehler": sum(v["status"] == "error" for v in results.values()),
        "kupferumfang": "Bahnen, Padpolygone, Vias; Flächen nur falls als finale filled-Zonen im routes.json enthalten.",
        "flaechendefinition": "Summe der vereinigten Kupfer-Polygonflächen über alle Lagen; inklusive Pads und Via-Scheiben, ohne Hülsenmantel; keine Doppelzählung überlappender Bahnen je Lage.",
    }
    if outdir is not None:
        (outdir / "report.json").write_text(
            json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
        )
    return report


def ir_kennzahlen(B, R, K, *, res=0.1) -> dict:
    ""
    return _analyse(B, R, K, res=res)


def run(B, R, K, outdir, res=0.1):
    ""
    return _analyse(B, R, K, res=res, outdir=outdir)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("board")
    p.add_argument("routes")
    p.add_argument("klassen")
    p.add_argument("outdir")
    p.add_argument("--res", type=float, default=0.1)
    a = p.parse_args()
    B, R, K = [json.loads(Path(q).read_text()) for q in (a.board, a.routes, a.klassen)]
    result = run(B, R, K, a.outdir, a.res)
    print(
        json.dumps(
            {
                "status": result["status"],
                "laufzeit_s": result["laufzeit_s"],
                "netze": len(result["netze"]),
            }
        )
    )
    return 0 if result["status"] == "solved" else 2


if __name__ == "__main__":
    sys.exit(main())
