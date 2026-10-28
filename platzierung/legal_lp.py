""

import copy
import json
import math
import os
from pathlib import Path
import runpy
import sys
import tempfile
import time
import warnings

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, OptimizeResult, linprog, milp
from scipy.sparse import coo_matrix
from shapely import affinity
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gemeinsam import plaetze as PL

GAP = 0.0052


def placer_kontext(board, klassen, regeln, placement):
    ""
    argv = sys.argv
    opts = dict(placement.get("optionen") or {})
    opts.pop("bild", None)
    opts["regeln"] = str(regeln)
    sys.argv = ["placer.py", str(board), str(klassen), "/dev/null"] + [
        f"{k}={v}" for k, v in opts.items()
    ]
    try:
        return runpy.run_path(
            str(ROOT / "platzierung/placer.py"), run_name="legal_lp_adapter"
        )
    finally:
        sys.argv = argv


def rechtecke(g):
    ""
    result = []
    for p in getattr(g, "geoms", [g]):
        if p.is_empty:
            continue
        rings = [p.exterior, *p.interiors]
        coords = [list(r.coords) for r in rings]
        ortho = all(
            abs(a[0] - b[0]) < 1e-8 or abs(a[1] - b[1]) < 1e-8
            for q in coords
            for a, b in zip(q, q[1:])
        )
        if not ortho:
            result.append(p.bounds)
            continue
        xs = sorted({round(x, 9) for q in coords for x, _ in q})
        for a, b in zip(xs, xs[1:]):
            if b - a < 1e-8:
                continue
            slab = p.intersection(box(a, p.bounds[1] - 1, b, p.bounds[3] + 1))
            for h in getattr(slab, "geoms", [slab]):
                if h.area > 1e-10:
                    result.append(h.bounds)
    return result


class Modell:
    def __init__(self):
        self.c, self.lo, self.hi, self.integer = [], [], [], []
        self.rows, self.rhs = [], []

    def var(self, lo=-np.inf, hi=np.inf, cost=0, integer=0):
        i = len(self.c)
        self.c.append(cost)
        self.lo.append(lo)
        self.hi.append(hi)
        self.integer.append(integer)
        return i

    def le(self, row, rhs):
        self.rows.append({k: v for k, v in row.items() if abs(v) > 1e-12})
        self.rhs.append(rhs)

    def eq(self, row, rhs):
        self.le(row, rhs)
        self.le({k: -v for k, v in row.items()}, -rhs)

    def matrix(self):
        rr, cc, vv = [], [], []
        for r, row in enumerate(self.rows):
            for c, v in row.items():
                rr.append(r)
                cc.append(c)
                vv.append(v)
        return coo_matrix((vv, (rr, cc)), shape=(len(self.rows), len(self.c))).tocsc()

    def lp(self):
        if any(a > b for a, b in zip(self.lo, self.hi)):
            return OptimizeResult(
                status=2, x=None, message="Widerspruch der Koordinatenschranken"
            )
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="Unrecognized options detected")
            return linprog(
                self.c,
                A_ub=self.matrix(),
                b_ub=self.rhs,
                bounds=list(zip(self.lo, self.hi)),
                method="highs",
                options={"threads": 1},
            )

    def packed(self):
        return dict(
            c=self.c,
            lo=self.lo,
            hi=self.hi,
            integer=self.integer,
            rows=self.rows,
            rhs=self.rhs,
        )


def plus(a, b, f=1):
    d = dict(a)
    for k, v in b.items():
        d[k] = d.get(k, 0) + f * v
    return d


class Legalisierer:
    def __init__(self, C, P, arbeitsbudget=False):
        self.arbeitsbudget = arbeitsbudget
        self.C, self.P = C, P
        self.T, self.K, self.fps = C["T"], C["K"], C["fps"]
        self.refs = list(self.T)
        self.old = {
            r: (
                float(P["parts"].get(r, {}).get("x", self.fps[r]["x"])),
                float(P["parts"].get(r, {}).get("y", self.fps[r]["y"])),
                float(P["parts"].get(r, {}).get("drehung", 0)),
                P["parts"].get(r, {}).get("side", self.fps[r]["side"]),
            )
            for r in self.refs
        }
        self.W = C["WT"]
        self.fixed = {
            r
            for r in self.refs
            if self.K["teile"][r].get("fest")
            or self.fps[r].get("locked")
            or (self.W[r]["max"] == 0 and not C["STUETZT"].get(r))
        }
        for r in self.refs:
            if C["mates"](r) & self.fixed:
                self.fixed.update(C["mates"](r))
        self.local = {
            r: self.T[r].geo(0, 0, self.old[r][2], self.old[r][3]) for r in self.refs
        }
        self.rect = {
            r: {s: rechtecke(g) for s, g in self.local[r].items()} for r in self.refs
        }
        self.pin = self.pin_ziele()
        self.report = dict(version="legal_lp-3", lp=[], fenster=[])
        self.trenn_schutz = False
        self.allowed = {
            r: sum(
                g.difference(C["inner"]).area
                for g in self.T[r]
                .geo(self.fps[r]["x"], self.fps[r]["y"], 0, self.fps[r]["side"])
                .values()
            )
            for r in self.refs
        }

    def pin_ziele(self):
        out = []

        for r, (a, n, pins) in sorted(self.C["PIN"].items()):
            cp = [
                (i, p)
                for i, p in enumerate(self.T[r].pads_abs(*self.old[r]))
                if p[1] == n
            ]
            ap = [
                (i, p)
                for i, p in enumerate(self.T[a].pads_abs(*self.old[a]))
                if (p[0] in pins if pins is not None else p[1] == n)
            ]
            if not cp or not ap:
                continue
            (ip, p), (iq, q) = min(
                ((p, q) for p in cp for q in ap),
                key=lambda pq: math.dist(pq[0][1][2:4], pq[1][1][2:4]),
            )

            out.append((r, a, ip, iq, math.dist(p[2:4], q[2:4])))
        return out

    def geos(self, state):
        return {r: self.T[r].geo(*state[r]) for r in self.refs}

    def probleme(self, state):
        gs = self.geos(state)
        pairs = []
        for i, r in enumerate(self.refs):
            for q in self.refs[:i]:
                if q in self.C["mates"](r):
                    continue
                if any(
                    g.distance(gs[q][s]) < 0.00485
                    for s, g in gs[r].items()
                    if s in gs[q]
                ):
                    pairs.append((q, r))
        bad = []
        for r in self.refs:
            if r not in self.fixed:
                if (
                    sum(g.difference(self.C["inner"]).area for g in gs[r].values())
                    > self.allowed[r] + 1e-3
                ):
                    bad.append((r, "rand"))
                if any(
                    not self.C["inner_p"].contains(self.C["Point"](p[2], p[3]))
                    for p in self.T[r].pads_abs(*state[r])
                ):
                    bad.append((r, "pad_rand"))
                if self.C["sperr_flaeche"](r, gs[r]) > 1e-7:
                    bad.append((r, "sperr"))
            for a in self.C["KPLK"]:
                if a == r or a not in state:
                    continue
                if any(
                    k.intersects(g)
                    for k in self.C["kp_abs"](a, state[a], klein=True)
                    for g in gs[r].values()
                ):
                    bad.append((r, "antenne"))
        return pairs, sorted(set(bad))

    def hot_geometrie(self, state):
        out = {}

        for a in sorted(
            {
                (t.get("stuetzt") or {}).get("ref")
                for t in self.K["teile"].values()
                if t.get("funktion") == "spule_sw"
            }
            - {None}
        ):
            cs = [
                (t.get("C") or 1, r, t.get("stuetzt") or {})
                for r, t in self.K["teile"].items()
                if str(t.get("funktion", "")).startswith("entkopplung")
                and (t.get("stuetzt") or {}).get("ref") == a
                and not (t.get("stuetzt") or {}).get("ueber_spule")
            ]
            if not cs:
                continue
            _, r, st = min(cs, key=lambda x: (x[0], x[1]))
            cp = self.T[r].pads_abs(*state[r])
            ap = self.T[a].pads_abs(*state[a])
            vp = next((p for p in ap if p[0] == str(st.get("pin"))), None)
            pp = next((p for p in cp if p[1] == st.get("netz")), None)
            pm = next((p for p in cp if p[1] in self.K["gnd"]), None)
            gnd = [p for p in ap if p[1] in self.K["gnd"]]
            if not (vp and pp and pm and gnd):
                continue
            gp = min(gnd, key=lambda p: (math.dist(p[2:4], vp[2:4]), p[0]))
            out[a] = Polygon([pp[2:4], vp[2:4], gp[2:4], pm[2:4]]).area
        return out

    def quality(self, state):
        for r in self.refs:
            if r in self.fixed and state[r] != self.old[r]:
                return False
            if state[r][3] != self.old[r][3]:
                return False
            a = self.C["STUETZT"].get(r)
            dx, dy = (
                state[r][i]
                - self.old[r][i]
                - (state[a][i] - self.old[a][i] if a else 0)
                for i in (0, 1)
            )
            if math.hypot(dx, dy) > self.W[r]["max"] + 0.00015:
                return False
            if not self.W[r]["drehen"] and state[r][2] != self.old[r][2]:
                return False
        for r, a, nr, na, d in self.pin:
            p = self.T[r].pads_abs(*state[r])[nr]
            q = self.T[a].pads_abs(*state[a])[na]
            if math.dist(p[2:4], q[2:4]) > d + 0.00015:
                return False
        h0, h1 = self.hot_geometrie(self.old), self.hot_geometrie(state)
        if any(h1[a] > v + 0.0005 for a, v in h0.items()):
            return False
        for f, (a, _, _) in self.C["FOLGE"].items():
            if any(
                abs((state[f][i] - self.old[f][i]) - (state[a][i] - self.old[a][i]))
                > 0.00015
                for i in (0, 1)
            ):
                return False
            if state[f][2] != self.old[f][2] or state[a][2] != self.old[a][2]:
                return False


        before = self.C["Paare"](self.old, alle=True).gesamt()
        after = self.C["Paare"](state, alle=True).gesamt()
        for k in ("trenn", "gnd_gegen", "gnd_gegen_belegt", "ausfaecher"):
            if after.get(k, 0) > before.get(k, 0) + 1e-6:
                return False
        return True

    def motion_terms(self, r):

        terms, seen = {}, set()
        while r and r not in self.fixed:
            if r in seen:
                return None
            seen.add(r)
            terms[r] = 1
            r = self.C["STUETZT"].get(r)
        return terms

    def motion_radius(self, r, q=None):
        a = self.motion_terms(r)
        b = self.motion_terms(q) if q else {}
        if a is None or b is None:
            return math.inf
        terms = plus(a, b, -1)
        return sum(abs(v) * self.W[k]["max"] for k, v in terms.items() if v)

    def feste_hindernisse_unmoeglich(self, gs):
        proofs = []
        bb = self.C["inner"].bounds
        for r in self.refs:
            if r in self.fixed or self.allowed[r] > 1e-7:
                continue

            if any(
                g.symmetric_difference(g.envelope).area > 1e-9 for g in gs[r].values()
            ):
                continue
            if self.W[r]["drehen"] and any(
                g.symmetric_difference(affinity.rotate(g, a, origin=(0, 0))).area > 1e-9
                for g in self.local[r].values()
                for a in (90, 180, 270)
            ):
                continue
            radius = self.motion_radius(r)
            lower, upper = [-radius, -radius], [radius, radius]
            for g in gs[r].values():
                gb = g.bounds
                for i in (0, 1):
                    lower[i] = max(lower[i], bb[i] - gb[i])
                    upper[i] = min(upper[i], bb[i + 2] - gb[i + 2])
            obstacles, refs = [], set()
            for q in self.fixed - self.C["mates"](r):
                for side in set(gs[r]) & set(gs[q]):
                    g, h = gs[r][side], gs[q][side]
                    if h.symmetric_difference(h.envelope).area > 1e-9:
                        continue
                    a, b = g.bounds, h.bounds
                    obstacles.append(
                        box(b[0] - a[2], b[1] - a[3], b[2] - a[0], b[3] - a[1])
                    )
                    refs.add(q)
            if any(a > b for a, b in zip(lower, upper)):
                empty = True
            else:

                if lower == upper:
                    region = self.C["Point"](*lower)
                elif lower[0] == upper[0]:
                    from shapely.geometry import LineString

                    region = LineString([(lower[0], lower[1]), (upper[0], upper[1])])
                elif lower[1] == upper[1]:
                    from shapely.geometry import LineString

                    region = LineString([(lower[0], lower[1]), (upper[0], upper[1])])
                else:
                    region = box(*lower, *upper)
                empty = region.difference(unary_union(obstacles)).is_empty
            if empty:
                proofs.append(
                    dict(
                        teil=r,
                        feste_nachbarn=sorted(refs),
                        wander_radius_mm=radius if math.isfinite(radius) else None,
                        verschiebebereich=[lower, upper],
                        grund="notwendiger Verschiebebereich komplett verdeckt",
                        rechtecke_exakt=True,
                    )
                )
        return proofs

    def machbarkeit(self):
        gs = self.geos(self.old)
        side = {}
        for s in ("F", "B"):
            obs = [g[s] for r, g in gs.items() if r in self.fixed and s in g]
            sp = self.C["SPERR"].get(s)
            occupied = unary_union(obs + ([sp] if sp is not None else []))
            free = self.C["inner"].difference(occupied).area


            groups = {}
            for r, g in gs.items():
                if r in self.fixed or s not in g:
                    continue
                groups.setdefault(tuple(sorted(self.C["mates"](r))), []).append(g[s])
            area = sum(unary_union(v).area for v in groups.values())
            area -= sum(
                g.difference(self.C["inner"]).area
                for r in self.refs
                if r not in self.fixed
                for side, g in self.T[r]
                .geo(self.fps[r]["x"], self.fps[r]["y"], 0, self.fps[r]["side"])
                .items()
                if side == s
            )
            area = max(0, area)
            side[s] = dict(
                bewegt_mm2=area,
                frei_mm2=free,
                belegung=area / free if free > 0 else (0 if area == 0 else None),
            )
        pairs, bad = self.probleme(self.old)
        fixed_pairs = [p for p in pairs if all(r in self.fixed for r in p)]
        bounded = []
        for r, q in pairs:


            common = set(gs[r]) & set(gs[q])
            for s in sorted(common):
                g, h = gs[r][s], gs[q][s]
                if g.distance(h) >= 0.00485:
                    continue
                if any(v.symmetric_difference(v.envelope).area > 1e-9 for v in (g, h)):
                    continue
                invariant = True
                for x in (r, q):
                    if x not in self.fixed and self.W[x]["drehen"]:
                        base = self.local[x][s]
                        if any(
                            base.symmetric_difference(
                                affinity.rotate(base, a, origin=(0, 0))
                            ).area
                            > 1e-9
                            for a in (90, 180, 270)
                        ):
                            invariant = False
                if not invariant:
                    continue
                radius = self.motion_radius(r, q)
                b, c = g.bounds, h.bounds
                need = (
                    min(b[2] - c[0], c[2] - b[0], b[3] - c[1], c[3] - b[1]) + 0.00485
                    if g.intersects(h)
                    else 0.00485 - g.distance(h)
                )
                if need > radius + 0.00015:
                    bounded.append(
                        dict(
                            teile=[r, q],
                            seite=s,
                            notwendig_mm=need,
                            erlaubt_relativ_mm=radius,
                        )
                    )

        blocked = self.feste_hindernisse_unmoeglich(gs)
        impossible = bool(fixed_pairs or bounded or blocked) or any(
            v["bewegt_mm2"] > v["frei_mm2"] + 1e-6 for v in side.values()
        )
        return dict(
            seiten=side,
            feste_kollisionen=fixed_pairs,
            wanderweiten_konflikte=bounded,
            feste_hindernisse=blocked,
            bewiesen_unmoeglich=impossible,
            pruefung="notwendige Bedingungen, kein Packungsbeweis",
            probleme=bad,
        )

    def aufbauen(self, active=None, rotation=False):
        M = Modell()
        xy = {}
        z = {}
        bounds = self.C["inner"].bounds
        active = set(self.refs) if active is None else set(active)
        pose = getattr(self, "current", self.old) if rotation else self.old
        groups = set(self.C["FOLGE"]) | {x[0] for x in self.C["FOLGE"].values()}
        for r in self.refs:
            x, y, th, side = self.old[r]
            mx = self.W[r]["max"] if r not in self.fixed and r in active else 0

            window = self.motion_radius(r) if r not in self.fixed and r in active else 0
            window = min(window, max(bounds[2] - bounds[0], bounds[3] - bounds[1]))
            if rotation:
                window = min(window, 3.0)
            xy[r] = [
                M.var(v - window, v + window)
                for v in ((x, y) if window else pose[r][:2])
            ]
            angles = (
                (0, 90, 180, 270)
                if rotation
                and r in active
                and r not in groups
                and mx > 0
                and self.W[r]["drehen"]
                else ((pose[r][2] - self.old[r][2],) if rotation else (0,))
            )
            z[r] = (
                [(a, M.var(0, 1, cost=0.002 * (a != 0), integer=1)) for a in angles]
                if len(angles) > 1
                else [(angles[0], None)]
            )
            if len(angles) > 1:
                M.eq({i: 1 for a, i in z[r]}, 1)
            for k, v in enumerate((x, y)):





                cost = max(0.2, self.W[r]["wichtigkeit"])
                u = M.var(0, cost=cost)
                M.le({xy[r][k]: 1, u: -1}, v)
                M.le({xy[r][k]: -1, u: -1}, -v)

        def coord(r, k, offsets):
            row = {xy[r][k]: 1}
            const = 0
            for a, v in z[r]:
                if v is None:
                    const = offsets[a][k]
                else:
                    row[v] = offsets[a][k]
            return row, const

        def delta(r, a, k):
            row = {xy[r][k]: 1}
            v = self.old[r][k]
            if a:
                row[xy[a][k]] = -1
                v -= self.old[a][k]
            return row, v

        def norm(rows, centers, radius):


            for j in range(32):
                co, si = math.cos(j * math.tau / 32), math.sin(j * math.tau / 32)
                M.le(
                    plus(
                        {k: co * v for k, v in rows[0].items()},
                        {k: si * v for k, v in rows[1].items()},
                    ),
                    radius + co * centers[0] + si * centers[1],
                )
        for r in self.refs:
            a = self.C["STUETZT"].get(r)
            if math.isfinite(self.W[r]["max"]):
                d = [delta(r, a, k) for k in (0, 1)]
                norm([x[0] for x in d], [x[1] for x in d], self.W[r]["max"])
        for f, (a, _, _) in self.C["FOLGE"].items():
            for k in (0, 1):
                row, v = delta(f, a, k)
                M.eq(row, v)
        for r, a, nr, na, distance in self.pin:

            def po(ref, num):
                return {
                    d: self.T[ref].pads_abs(
                        0, 0, self.old[ref][2] + d, self.old[ref][3]
                    )[num][2:4]
                    for d, _ in z[ref]
                }

            pr, pa = po(r, nr), po(a, na)
            rows, centers = [], []
            for k in (0, 1):
                rr, rc = coord(r, k, pr)
                ar, ac = coord(a, k, pa)
                rows.append(plus(rr, ar, -1))
                centers.append(ac - rc)
            norm(rows, centers, distance)
        records = []
        for r in self.refs:
            for side, rects in self.rect[r].items():
                for b in rects:
                    rb = {
                        a: affinity.rotate(box(*b), a, origin=(0, 0)).bounds
                        for a, _ in z[r]
                    }
                    edges = [
                        coord(
                            r,
                            k % 2,
                            {
                                a: (bb[k] if k % 2 == 0 else 0, bb[k] if k % 2 else 0)
                                for a, bb in rb.items()
                            },
                        )
                        for k in range(4)
                    ]
                    records.append((r, side, edges, b))
                    if r in self.fixed:
                        continue


                    oldgeo = self.geos_single(r)
                    low = (
                        [min(bounds[k], oldgeo[k]) for k in (0, 1)]
                        if self.allowed[r] > 1e-7
                        else bounds[:2]
                    )
                    high = (
                        [max(bounds[k + 2], oldgeo[k + 2]) for k in (0, 1)]
                        if self.allowed[r] > 1e-7
                        else bounds[2:]
                    )
                    for k in (0, 1):
                        row, c = edges[k]
                        M.le({i: -v for i, v in row.items()}, c - low[k])
                        row, c = edges[k + 2]
                        M.le(row, high[k] - c)
                        if len(z[r]) == 1:
                            M.lo[xy[r][k]] = max(M.lo[xy[r][k]], low[k] - edges[k][1])
                            M.hi[xy[r][k]] = min(
                                M.hi[xy[r][k]], high[k] - edges[k + 2][1]
                            )

        for r in self.refs:
            for k in self.C["kp_abs"](r, self.old[r], klein=True):
                for b in rechtecke(
                    affinity.translate(k, -self.old[r][0], -self.old[r][1])
                ):
                    edges = [({xy[r][j % 2]: 1}, b[j]) for j in range(4)]
                    for s in ("F", "B"):
                        records.append((r, s, edges, b))
            if r not in self.fixed:
                for s, (g, _) in self.C["sperr_von"](r).items():
                    for b in rechtecke(g):
                        records.append(("#" + r, s, [({}, v) for v in b], b))
        if self.trenn_schutz:
            self.trenn_halbraeume(M, xy, z)
        return M, xy, z, records

    def trenn_halbraeume(self, M, xy, z):
        ""
        pads = [
            (r, i, p)
            for r in self.refs
            for i, p in enumerate(self.T[r].pads_abs(*self.old[r]))
        ]
        agg = self.C["AGG"]
        vic = self.C["VIC"]
        n = 0
        for r, i, p in pads:
            if p[1] not in agg:
                continue
            for q, j, s in pads:
                if r == q or s[1] not in vic or (r in self.fixed and q in self.fixed):
                    continue
                vec = [p[k + 2] - s[k + 2] for k in (0, 1)]
                d = math.hypot(*vec)
                if d < 1e-9:
                    continue
                row = {}
                const = 0
                for ref, idx, sign in ((r, i, 1), (q, j, -1)):
                    for k in (0, 1):
                        w = sign * vec[k] / d
                        row[xy[ref][k]] = row.get(xy[ref][k], 0) + w
                        for a, v in z[ref]:
                            offset = self.T[ref].pads_abs(
                                0, 0, self.old[ref][2] + a, self.old[ref][3]
                            )[idx][k + 2]
                            if v is None:
                                const += w * offset
                            else:
                                row[v] = row.get(v, 0) + w * offset
                M.le(
                    {k: -v for k, v in row.items()}, const - min(d, self.C["TRENN_PAD"])
                )
                n += 1
        self.report["trenn_halbraeume"] = n

    def geos_single(self, r):
        return unary_union(list(self.T[r].geo(*self.old[r]).values())).bounds

    def relations(self, rec, active=None, integer=False, pose_state=None):
        out = []
        pose_state = pose_state or self.old
        pose = {
            **{self.xy[x][k]: pose_state[x][k] for x in self.refs for k in (0, 1)},
            **{
                v: float(a == 0)
                for x in self.refs
                for a, v in self.z[x]
                if v is not None
            },
        }

        def span(edge):
            row, c = edge

            lo = hi = c
            for i, v in row.items():
                lo += min(v * self.model.lo[i], v * self.model.hi[i])
                hi += max(v * self.model.lo[i], v * self.model.hi[i])
            return lo, hi

        for i, A in enumerate(rec):
            r, side, ae, _ = A
            for j in range(i):
                q, sd, be, _ = rec[j]
                if side != sd or r == q:
                    continue
                if r.startswith("#") or q.startswith("#"):
                    if r != "#" + q and q != "#" + r:
                        continue
                elif q in self.C["mates"](r):
                    continue
                if active is not None and r not in active and q not in active:
                    continue

                aa, bb = [span(e) for e in ae], [span(e) for e in be]
                if (
                    aa[0][0] >= bb[2][1] + GAP
                    or bb[0][0] >= aa[2][1] + GAP
                    or aa[1][0] >= bb[3][1] + GAP
                    or bb[1][0] >= aa[3][1] + GAP
                ):
                    continue
                edges = []
                for ax, rev in ((0, False), (0, True), (1, False), (1, True)):
                    left, right = (be, ae) if rev else (ae, be)
                    row = plus(left[ax + 2][0], right[ax][0], -1)
                    rhs = right[ax][1] - left[ax + 2][1] - GAP

                    value = sum(v * pose[k] for k, v in row.items()) - rhs
                    edges.append(
                        (value, row, rhs, ax, q if rev else r, r if rev else q)
                    )
                edges.sort(key=lambda x: (x[0], x[3], x[4], x[5]))
                if not integer and edges[0][0] < -0.5:
                    continue
                out.append(((j, i), edges))
        return out

    def graph(self, rel, choices):
        paths = []
        for ax in (0, 1):
            d = {r: self.model.lo[self.xy[r][ax]] for r in self.refs}
            pred = {}
            last = None
            for _ in range(len(self.refs)):
                last = None
                for key, edges in rel:
                    e = edges[choices.get(key, 0)]
                    _, row, rhs, axis, r, q = e
                    if axis != ax or r.startswith("#") or q.startswith("#"):
                        continue

                    if d[q] < d[r] - rhs - 1e-8:
                        d[q] = d[r] - rhs
                        pred[q] = (r, key)
                        last = q
                if last is None:
                    break
            failed = [
                r for r in self.refs if d[r] > self.model.hi[self.xy[r][ax]] + 1e-7
            ]
            node = failed[0] if failed else last
            critical = []
            seen = set()
            while node in pred and node not in seen:
                seen.add(node)
                node, key = pred[node]
                critical.append(key)

            endpoint = max(
                d[r]
                + max(
                    (b[ax + 2] for bs in self.rect[r].values() for b in bs), default=0
                )
                for r in self.refs
            )
            lower, upper = self.C["inner"].bounds[ax], self.C["inner"].bounds[ax + 2]
            for r in self.refs:
                if self.allowed[r] > 1e-7:
                    bounds = self.geos_single(r)
                    lower = min(lower, bounds[ax])
                    upper = max(upper, bounds[ax + 2])
            paths.append(
                dict(
                    achse="xy"[ax],
                    laenge_mm=endpoint - lower,
                    platine_mm=upper - lower,
                    original_ueberstand_beruecksichtigt=True,
                    passt=not failed and last is None,
                    kritisch=critical,
                )
            )
        return paths

    def solve_lp(self):
        self.model, self.xy, self.z, rec = self.aufbauen()
        rel = self.relations(rec)
        choices = {}
        base = len(self.model.rows)
        for flip in range(25):
            gp = self.graph(rel, choices)
            self.report["constraint_graph"] = gp
            if all(p["passt"] for p in gp):
                self.model.rows = self.model.rows[:base]
                self.model.rhs = self.model.rhs[:base]
                for key, edges in rel:
                    _, row, rhs, *_ = edges[choices.get(key, 0)]
                    self.model.le(row, rhs)
                res = self.model.lp()
                self.report["lp"].append(
                    dict(status=int(res.status), kanten=len(rel), kippen=flip)
                )
                if res.x is not None:



                    for lazy in range(8):
                        candidate = self.state(res.x)
                        pairs, bad = self.probleme(candidate)
                        cuts = self.radial_cuts(candidate)
                        if not pairs and not bad and not cuts:
                            return candidate
                        existing = {key for key, _ in rel}
                        new = [
                            (key, edges)
                            for key, edges in self.relations(rec, pose_state=candidate)
                            if key not in existing
                        ]
                        if not new and not cuts:
                            break
                        for key, edges in new:
                            _, row, rhs, *_ = edges[0]
                            self.model.le(row, rhs)
                        rel.extend(new)
                        res = self.model.lp()
                        self.report["lp"].append(
                            dict(status=int(res.status), kanten=len(rel), lazy=lazy + 1)
                        )
                        if res.x is None:
                            break
                    if res.x is not None:
                        return self.state(res.x)
            crit = [k for p in gp if not p["passt"] for k in p["kritisch"]]
            candidates = [
                (e[0] - edges[choices.get(key, 0)][0], key, j)
                for key, edges in rel
                if not crit or key in crit
                for j, e in enumerate(edges)
                if j == choices.get(key, 0) + 1
            ]
            if not candidates:
                break
            _, key, j = min(candidates)
            choices[key] = j
        return None

    def radial_cuts(self, state):
        ""
        cuts = 0
        for r in self.refs:
            a = self.C["STUETZT"].get(r)
            vec = [
                state[r][k]
                - self.old[r][k]
                - (state[a][k] - self.old[a][k] if a else 0)
                for k in (0, 1)
            ]
            d = math.hypot(*vec)
            radius = self.W[r]["max"]
            if d <= radius + 0.0001:
                continue
            row = {}
            rhs = radius
            for k in (0, 1):
                w = vec[k] / d
                row[self.xy[r][k]] = w
                rhs += w * self.old[r][k]
                if a:
                    row[self.xy[a][k]] = -w
                    rhs -= w * self.old[a][k]
            self.model.le(row, rhs)
            cuts += 1
        for r, a, nr, na, radius in self.pin:
            p = self.T[r].pads_abs(*state[r])[nr]
            q = self.T[a].pads_abs(*state[a])[na]
            vec = [p[k + 2] - q[k + 2] for k in (0, 1)]
            d = math.hypot(*vec)
            if d <= radius + 0.0001:
                continue
            row = {}
            rhs = radius
            for k in (0, 1):
                w = vec[k] / d
                row[self.xy[r][k]] = w
                row[self.xy[a][k]] = -w
                rhs -= w * ((p[k + 2] - state[r][k]) - (q[k + 2] - state[a][k]))
            self.model.le(row, rhs)
            cuts += 1
        return cuts

    def state(self, sol):
        st = {}
        for r in self.refs:
            angle = next(
                (a for a, v in self.z[r] if v is not None and sol[v] > 0.5),
                self.z[r][0][0],
            )
            st[r] = (
                round(float(sol[self.xy[r][0]]), 4),
                round(float(sol[self.xy[r][1]]), 4),
                (self.old[r][2] + angle) % 360 if angle else self.old[r][2],
                self.old[r][3],
            )
            if r in self.fixed:
                st[r] = self.old[r]
        return st

    def window_closure(self, active):
        active = set(active) - self.fixed
        while True:
            before = set(active)
            for r in self.refs:
                a = self.C["STUETZT"].get(r)
                if r in active:
                    active.update(self.C["mates"](r) - self.fixed)
                    if a and a not in self.fixed and self.W[r]["max"] == 0:
                        active.add(a)
                if a in active and r not in self.fixed and self.W[r]["max"] == 0:
                    active.add(r)
            if active == before:
                return active

    def solve_window(self, active):
        active = self.window_closure(active)
        if len(active) > 15:
            self.report["fenster"].append(
                dict(teile=sorted(active), status="gruppe_zu_gross")
            )
            return None
        self.model, self.xy, self.z, rec = self.aufbauen(active, rotation=True)
        rel = self.relations(
            rec, active, integer=True, pose_state=getattr(self, "current", self.old)
        )
        for key, edges in rel:
            binary = [self.model.var(0, 1, integer=1) for _ in edges]
            self.model.eq({v: 1 for v in binary}, 1)
            for v, (_, row, rhs, *_) in zip(binary, edges):
                mx = (
                    sum(
                        max(c * self.model.lo[i], c * self.model.hi[i])
                        for i, c in row.items()
                    )
                    - rhs
                )
                mx = max(0, mx) + 1e-6
                self.model.le(plus(row, {v: mx}), rhs + mx)


        with tempfile.TemporaryDirectory(prefix="autorouter_legal_milp_") as td:
            inp = Path(td) / "model.json"
            out = Path(td) / "sol.json"
            payload = self.model.packed()
            if self.arbeitsbudget: payload['node_limit'] = 5000
            inp.write_text(json.dumps(payload))
            result = PL.lauf(
                [sys.executable, str(Path(__file__)), "--milp", str(inp), str(out)],
                cpu_s=12000 if self.arbeitsbudget else 5,
                mit_platz=False,
                log=str(Path(td) / "solver.log"),
            )
            if self.arbeitsbudget and result['zustand'] != 'ok':
                raise RuntimeError('Arbeitsmessung: MILP-Notbremse/Fehler, keine reproduzierbare Platzierung')
            data = (
                json.loads(out.read_text())
                if out.exists() and result["zustand"] == "ok"
                else {}
            )
        self.report["fenster"].append(
            dict(
                teile=sorted(active),
                paare=len(rel),
                lauf=result,
                status=data.get("status"),
            )
        )
        return self.state(data["x"]) if data.get("x") is not None else None

    def run(self):
        t = time.process_time()
        child_cpu = 0
        before = self.C["kennzahlen"](self.old)
        self.report["vorher"] = before
        self.report["machbarkeit"] = self.machbarkeit()
        oldpairs, oldbad = self.probleme(self.old)
        state = self.old
        status = "bereits_legal"
        if self.report["machbarkeit"]["bewiesen_unmoeglich"]:


            if self.refs:
                self.model, self.xy, self.z, rec = self.aufbauen()
                self.report["constraint_graph"] = self.graph(self.relations(rec), {})
            status = "nicht_legalisierbar_unter_grenzen"
        elif oldpairs or oldbad:
            candidate = self.solve_lp()
            if candidate:
                trenn0 = self.C["Paare"](self.old, alle=True).gesamt().get("trenn", 0)
                trenn1 = self.C["Paare"](candidate, alle=True).gesamt().get("trenn", 0)
                if trenn1 > trenn0 + 1e-6:
                    self.trenn_schutz = True
                    self.report["trenn_rueckfall"] = dict(
                        vorher=trenn0, abgelehnt=trenn1
                    )
                    candidate = self.solve_lp()
            if (
                candidate
                and not any(self.probleme(candidate))
                and self.quality(candidate)
            ):
                state = candidate
                status = "legal_lp"
            else:
                status = "keine_gepruefte_loesung"

                todo = set(r for p in oldpairs for r in p if r not in self.fixed) | {
                    r for r, _ in oldbad if r not in self.fixed
                }
                self.current = dict(self.old)
                while todo:
                    seed = min(todo)
                    center = self.current[seed][:2]
                    near = sorted(
                        (r for r in self.refs if r not in self.fixed),
                        key=lambda r: (
                            r not in todo,
                            math.dist(center, self.current[r][:2]),
                            r,
                        ),
                    )
                    active = self.window_closure({seed})
                    for r in near:
                        trial = self.window_closure(active | {r})
                        if len(trial) <= 15:
                            active = trial
                    if len(active) > 15:
                        self.report["fenster"].append(dict(status="gruppe_zu_gross"))
                        break
                    candidate = self.solve_window(active)
                    child_cpu += (
                        self.report["fenster"][-1].get("lauf", {}).get("cpu_s", 0)
                    )
                    if candidate and self.quality(candidate):
                        old_problems = self.probleme(self.current)
                        new_problems = self.probleme(candidate)
                        old_keys = set(old_problems[0]), set(old_problems[1])
                        new_keys = set(new_problems[0]), set(new_problems[1])
                        improves = (
                            new_keys[0] <= old_keys[0]
                            and new_keys[1] <= old_keys[1]
                            and sum(map(len, new_keys)) < sum(map(len, old_keys))
                        )
                        self.report["fenster"][-1]["teilfortschritt"] = improves
                        if not any(new_problems):
                            state = candidate
                            status = "legal_milp"
                            break
                        if improves:
                            self.current = candidate
                            todo = {
                                r
                                for p in new_problems[0]
                                for r in p
                                if r not in self.fixed
                            } | {r for r, _ in new_problems[1] if r not in self.fixed}
                            continue
                    todo -= active
        output = copy.deepcopy(self.P)
        if state is not self.old:
            for r, s in state.items():
                if s == self.old[r]:
                    continue
                p = output["parts"].setdefault(r, {})
                p.update(
                    x=s[0],
                    y=s[1],
                    drehung=s[2],
                    side=s[3],
                    pads={
                        n: [round(x, 4), round(y, 4)]
                        for n, net, x, y, side in self.T[r].pads_abs(*s)
                    },
                )


                p["padinstanzen"] = [
                    dict(num=n, net=net, x=round(x, 4), y=round(y, 4), side=side)
                    for n, net, x, y, side in self.T[r].pads_abs(*s)
                ]
                p["transformation_eindeutig"] = True


        reserves = self.C.get("KUPFERRESERVE") or []
        if reserves:
            from platzierung.kupferreserve import absolut
            output["kupferreserve"] = absolut(reserves, self.T, state, self.fps)
        self.report.update(
            zustand=status,
            nachher=self.C["kennzahlen"](state),
            verschiebung_mm={
                r: round(math.dist(self.old[r][:2], state[r][:2]), 6) for r in self.refs
            },
            verschiebung_relativ_mm={
                r: round(
                    math.hypot(
                        *[
                            state[r][i]
                            - self.old[r][i]
                            - (
                                state[a][i] - self.old[a][i]
                                if (a := self.C["STUETZT"].get(r))
                                else 0
                            )
                            for i in (0, 1)
                        ]
                    ),
                    6,
                )
                for r in self.refs
            },
            pinabstaende_mm=[
                dict(
                    teil=r,
                    ziel=a,
                    teil_padinstanz=nr,
                    ziel_padinstanz=na,
                    vorher=d,
                    nachher=math.dist(
                        self.T[r].pads_abs(*state[r])[nr][2:4],
                        self.T[a].pads_abs(*state[a])[na][2:4],
                    ),
                )
                for r, a, nr, na, d in self.pin
            ],
            hot_loop_flaeche_vorher=self.hot_geometrie(self.old),
            hot_loop_flaeche_nachher=self.hot_geometrie(state),
            cpu_s=round(time.process_time() - t + child_cpu, 4),
            last=os.getloadavg()[0],
            rest=self.probleme(state),
            qualitaet_ok=self.quality(state),
        )
        output.setdefault("kennzahlen", {})["nachher"] = self.report["nachher"]
        output.setdefault("regeln", {})["nachher"] = self.C["regel_kennzahlen"](state)
        output["legal_lp"] = self.report
        return output


def milp_worker(inp, out):
    data = json.loads(Path(inp).read_text())
    M = Modell()
    for name in ("c", "lo", "hi", "integer", "rhs"):
        setattr(M, name, data[name])
    M.rows = [{int(k): v for k, v in row.items()} for row in data["rows"]]
    if any(a > b for a, b in zip(M.lo, M.hi)):
        Path(out).write_text(json.dumps(dict(status=2, x=None)))
        return
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="Unrecognized options detected")
        options = {"threads": 1, "mip_rel_gap": 0.01}
        if 'node_limit' in data: options['node_limit'] = int(data['node_limit'])
        res = milp(
            M.c,
            integrality=M.integer,
            bounds=Bounds(M.lo, M.hi),
            constraints=LinearConstraint(M.matrix(), -np.inf, M.rhs),
            options=options,
        )
    Path(out).write_text(
        json.dumps(
            dict(status=int(res.status), x=None if res.x is None else res.x.tolist())
        )
    )


def main():
    if sys.argv[1] == "--milp":
        milp_worker(*sys.argv[2:4])
        return
    board, klassen, regeln, pfile = sys.argv[1:5]
    out = sys.argv[5] if len(sys.argv) > 5 else pfile
    P = json.loads(Path(pfile).read_text())
    C = placer_kontext(board, klassen, regeln, P)
    result = Legalisierer(C, P, arbeitsbudget='budget=arbeit' in sys.argv[6:]).run()

    target = Path(out)
    tmp = target.with_name(target.name + ".legal_tmp")
    tmp.write_text(json.dumps(result, indent=1))
    tmp.replace(target)
    print(
        result["legal_lp"]["zustand"],
        result["legal_lp"]["vorher"]["kollisionen"],
        "->",
        result["legal_lp"]["nachher"]["kollisionen"],
    )


if __name__ == "__main__":
    main()
