#!/usr/bin/env python3
import sys, re, json, math, os
from collections import defaultdict, Counter
import numpy as np
from scipy.spatial import cKDTree

pcb_path = sys.argv[1]
pro_path = sys.argv[2] if len(sys.argv) > 2 else None
txt = open(pcb_path, encoding="utf-8", errors="replace").read()

NUM = r"(-?[\d.]+(?:e-?\d+)?)"
GND_RE = re.compile(r"(^|/|_)(A|D|P|S)?GND|(^|/)VSS$|(^|/)GND", re.I)
PWR_RE = re.compile(r"(^|/)(\+|-)?\d+V\d*|(^|/)\+?\d+V\d*|V(CC|DD|IN|BUS|BAT|SYS|TT|REF|DDQ|PP|CCINT|CCAUX|CCO|CCBRAM|MGT|CCADC|ADJ)|(^|/|_)P?\d+V\d+|PWR|POWER|VDD|VCC|VIN|VBUS|VBAT|VDDR|VTT", re.I)


def p(*a):
    print(*a)


version = re.search(r"\(version (\d+)\)", txt).group(1)
title = re.search(r'\(title "?([^")\n]*)"?\)', txt)
p(f"### {os.path.basename(pcb_path)}  (file version {version}, title: {title.group(1) if title else '-'})")
nets = {}
for m in re.finditer(r'^\s*\(net (\d+) ("(?:[^"\\]|\\.)*"|[^\s()]+)\)\s*$', txt, re.M):
    nets[int(m.group(1))] = m.group(2).strip('"')
p(f"Nets: {len(nets)}")

def NET(tok):
    tok = tok.strip()
    if tok.startswith('"'):
        return tok[1:-1]
    return nets.get(int(tok), "")

def NN(x):
    return x

def is_gnd(n):
    return bool(GND_RE.search(n or ""))

def is_pwr(n):
    return (not is_gnd(n)) and bool(PWR_RE.search(n or ""))


st = re.search(r"\(stackup(.*?)\n\s*\)\s*\n\s*\(pad_to_mask|\(stackup(.*?)\(copper_finish", txt, re.S)
stack = []
for m in re.finditer(r'\(layer "([^"]+)"\s*\(type "([^"]+)"\)(.*?)(?=\(layer "|\(copper_finish|\Z)', txt[: 200000], re.S):
    name, type_, rest = m.group(1), m.group(2), m.group(3)
    th = re.search(r"\(thickness " + NUM, rest)
    mat = re.search(r'\(material "([^"]+)"', rest)
    er = re.search(r"\(epsilon_r " + NUM, rest)
    if type_ in ("copper", "core", "prepreg"):
        stack.append((name, type_, float(th.group(1)) if th else None, mat.group(1) if mat else "", er.group(1) if er else ""))
if stack:
    p("Layer stackup: " + " | ".join(f"{n}:{t}:{(th*1000 if th else 0):.0f}um{(' '+mat) if mat else ''}{(' er'+er) if er else ''}" for n, t, th, mat, er in stack))
thick = re.search(r"\(general\s*\(thickness " + NUM, txt)
p(f"Thickness: {thick.group(1) if thick else '?'} mm")


segs = []
for m in re.finditer(r'\(segment\b(.*?)\(net (\d+|"(?:[^"\\]|\\.)*")\)', txt, re.S):
    b = m.group(1)
    s = re.search(r"\(start " + NUM + " " + NUM, b); e = re.search(r"\(end " + NUM + " " + NUM, b)
    w = re.search(r"\(width " + NUM, b); l = re.search(r'\(layer "?([^")\s]+)', b)
    if not (s and e and w and l):
        continue
    x1, y1, x2, y2 = map(float, (s.group(1), s.group(2), e.group(1), e.group(2)))
    segs.append((x1, y1, x2, y2, float(w.group(1)), l.group(1), NET(m.group(2)), math.hypot(x2 - x1, y2 - y1), 0))
arcs_len = 0.0
for m in re.finditer(r"\(arc\s*\(start " + NUM + " " + NUM + r"\)\s*\(mid " + NUM + " " + NUM + r"\)\s*\(end " + NUM + " " + NUM + r'\)(.*?)\(net (\d+|"(?:[^"\\]|\\.)*")\)', txt, re.S):
    x1, y1, xm, ym, x2, y2 = map(float, m.groups()[:6])
    b = m.group(7)
    w = re.search(r"\(width " + NUM, b); l = re.search(r'\(layer "?([^")\s]+)', b)
    if not (w and l):
        continue

    ax, ay, bx, by, cx, cy = x1, y1, xm, ym, x2, y2
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        L = math.hypot(x2 - x1, y2 - y1)
    else:
        ux = ((ax*ax+ay*ay)*(by-cy)+(bx*bx+by*by)*(cy-ay)+(cx*cx+cy*cy)*(ay-by))/d
        uy = ((ax*ax+ay*ay)*(cx-bx)+(bx*bx+by*by)*(ax-cx)+(cx*cx+cy*cy)*(bx-ax))/d
        r = math.hypot(ax-ux, ay-uy)
        a1 = math.atan2(ay-uy, ax-ux); a2 = math.atan2(by-uy, bx-ux); a3 = math.atan2(cy-uy, cx-ux)
        def norm(a):
            return a % (2*math.pi)
        s1 = norm(a2 - a1); s2 = norm(a3 - a1)
        ang = s2 if s1 < s2 else 2*math.pi - s2
        L = r * ang
    arcs_len += L
    segs.append((x1, y1, x2, y2, float(w.group(1)), l.group(1), NET(m.group(8)), L, 1))

tot = sum(s[7] for s in segs)
p(f"Tracks: {len(segs)} pieces, {tot/1000:.2f} m total, of which arcs {100*arcs_len/max(tot,1e-9):.1f} %")
layers = sorted({s[5] for s in segs}, key=lambda n: (0 if n == "F.Cu" else 2 if n == "B.Cu" else 1, n))

def whist(ss, top=6):
    c = Counter()
    for s in ss:
        c[round(s[4], 4)] += s[7]
    T = sum(c.values()) or 1
    return ", ".join(f"{w:g}:{100*v/T:.0f}%" for w, v in c.most_common(top))

p("Track width (length-weighted) all: " + whist(segs))
sig = [s for s in segs if not is_gnd(NN(s[6])) and not is_pwr(NN(s[6]))]
pw = [s for s in segs if is_pwr(NN(s[6]))]
gn = [s for s in segs if is_gnd(NN(s[6]))]
p("  Signal: " + whist(sig))
p("  Supply: " + whist(pw) + f"  (length {sum(s[7] for s in pw)/1000:.2f} m)")
p("  GND tracks: " + whist(gn) + f"  (length {sum(s[7] for s in gn)/1000:.2f} m, {100*sum(s[7] for s in gn)/max(tot,1):.1f} % of all tracks)")
for L in layers:
    ls = [s for s in segs if s[5] == L]
    p(f"  Layer {L}: {sum(s[7] for s in ls)/1000:.2f} m; " + whist(ls, 4))

straight = [s for s in segs if not s[8] and s[7] > 0.05]
on45 = sum(s[7] for s in straight if (abs(((math.degrees(math.atan2(s[3]-s[1], s[2]-s[0])) % 45) + 22.5) % 45 - 22.5) < 0.5))
p(f"Angle: {100*on45/max(sum(s[7] for s in straight),1):.1f} % distance-weighted share of straight length on 0/45/90° grid")

vias = []
for m in re.finditer(r'\(via\b(.*?)\(net (\d+|"(?:[^"\\]|\\.)*")\)', txt, re.S):
    b = m.group(1)
    a = re.search(r"\(at " + NUM + " " + NUM, b); sz = re.search(r"\(size " + NUM, b); dr = re.search(r"\(drill " + NUM, b)
    ly = re.search(r"\(layers ([^)]*)\)", b)
    if not (a and sz and dr):
        continue
    type_ = "micro" if re.search(r"\bmicro\b", b) else "blind" if re.search(r"\bblind\b", b) else "through"
    vias.append((float(a.group(1)), float(a.group(2)), float(sz.group(1)), float(dr.group(1)), NET(m.group(2)), type_, ly.group(1).replace('"', '') if ly else ""))
vc = Counter((v[2], v[3], v[5]) for v in vias)
p(f"Vias: {len(vias)}; sizes (diameter/drill/type): " + ", ".join(f"{a:g}/{b:g}/{t}:{n}" for (a, b, t), n in vc.most_common(6)))
gv = [v for v in vias if is_gnd(NN(v[4]))]
pv = [v for v in vias if is_pwr(NN(v[4]))]
p(f"  GND vias {len(gv)} ({100*len(gv)/max(len(vias),1):.0f} %), supply vias {len(pv)} ({100*len(pv)/max(len(vias),1):.0f} %)")


lines = txt.split("\n")
ind = re.match(r"^(\s+)", lines[1]).group(1) if re.match(r"^(\s+)\(", lines[1]) else "  "
blocks = re.split(r"\n" + re.escape(ind) + r"(?=\()", txt)
fps = []
pads = []
for b in blocks:
    if not (b.startswith("(footprint") or b.startswith("(module")):
        continue
    name = re.match(r'\((?:footprint|module)\s+("[^"]*"|\S+)', b).group(1).strip('"')
    lay = re.search(r'\(layer "?([^")\s]+)', b).group(1)
    at = re.search(r"\(at " + NUM + " " + NUM + r"(?: " + NUM + r")?\)", b)
    fx, fy = float(at.group(1)), float(at.group(2)); rot = float(at.group(3) or 0)
    ref = re.search(r'\(property "Reference" "([^"]*)"', b) or re.search(r"\(fp_text reference \"?([^\s\")]+)", b)
    val = re.search(r'\(property "Value" "([^"]*)"', b) or re.search(r"\(fp_text value \"?([^\"]+?)\"? \(", b)
    fi = len(fps)
    fps.append(dict(name=name, layer=lay, x=fx, y=fy, rot=rot, ref=ref.group(1) if ref else "", val=val.group(1) if val else "", pads=[]))
    th = math.radians(rot); c, s_ = math.cos(th), math.sin(th)
    for pm in re.finditer(r"\(pad\s+(\"[^\"]*\"|\S+)\s+(\w+)\s+(\w+)(.*?)(?=\(pad\s|\Z)", b, re.S):
        ptype, shape, pb = pm.group(2), pm.group(3), pm.group(4)
        pa = re.search(r"\(at " + NUM + " " + NUM, pb); ps = re.search(r"\(size " + NUM + " " + NUM, pb)
        if not (pa and ps):
            continue
        px, py = float(pa.group(1)), float(pa.group(2))
        X = fx + px * c + py * s_; Y = fy - px * s_ + py * c
        pn = re.search(r'\(net (\d+|"(?:[^"\\]|\\.)*")', pb)
        pl = re.search(r"\(layers ([^)]*)\)", pb)
        pls = pl.group(1) if pl else ""
        if "Cu" not in pls and ptype not in ("thru_hole",):
            continue
        side = "*" if ("*.Cu" in pls or ptype in ("thru_hole", "np_thru_hole")) else ("F" if "F.Cu" in pls else "B" if "B.Cu" in pls else "?")
        pads.append((X, Y, float(ps.group(1)), float(ps.group(2)), shape, NET(pn.group(1)) if pn else "", side, fi, ptype, px, py))
        fps[fi]["pads"].append(len(pads) - 1)
p(f"Components: {len(fps)}, pads: {len(pads)}")


ex = []
for m in re.finditer(r"\(gr_(?:line|arc|rect|poly|circle)\b(.*?)\(layer \"?Edge\.Cuts", txt, re.S):
    for q in re.finditer(r"\((?:start|end|xy|mid|center) " + NUM + " " + NUM, m.group(1)):
        ex.append((float(q.group(1)), float(q.group(2))))
if ex:
    ex = np.array(ex)
    bw, bh = ex[:, 0].max() - ex[:, 0].min(), ex[:, 1].max() - ex[:, 1].min()
    area = bw * bh
    p(f"Board (bounding box): {bw:.1f} x {bh:.1f} mm = {area/100:.1f} cm²")
else:
    area = None
cx = [q[0] for q in pads] + [s_[0] for s_ in segs] + [s_[2] for s_ in segs]
cy = [q[1] for q in pads] + [s_[1] for s_ in segs] + [s_[3] for s_ in segs]
if cx:
    ca = (max(cx) - min(cx)) * (max(cy) - min(cy))
    if area is None or ca < 0.8 * area:
        p(f"  Edge.Cuts hull does not fit; copper hull {max(cx)-min(cx):.1f} x {max(cy)-min(cy):.1f} mm = {ca/100:.1f} cm² is used")
        area = ca
        ex = np.array([(min(cx), min(cy)), (max(cx), max(cy))])


cu = sorted({l for s in stack for l in [s[0]] if s[1] == "copper"})
nl = len([s for s in stack if s[1] == "copper"]) or len(set(layers) | {"F.Cu", "B.Cu"})
p(f"Copper layers: {nl}")
p(f"Via density: {len(vias)/(area/100) if area else 0:.1f} /cm², GND vias {len(gv)/(area/100) if area else 0:.1f} /cm²")


z_info = defaultdict(lambda: defaultdict(float))
zset = Counter()
for b in blocks:
    if not b.startswith("(zone"):
        continue
    nm = re.search(r'\(net_name "?([^")]*)"?\)', b) or re.search(r'\(net "([^"]*)"\)', b)
    net = nm.group(1) if nm else "?"
    head = b.split("(polygon")[0]
    cl = re.search(r"\(connect_pads\s*(\w+)?\s*\(clearance " + NUM, head)
    mode = cl.group(1) if cl and cl.group(1) else "thermal"
    mt = re.search(r"\(min_thickness " + NUM, head)
    tg = re.search(r"\(thermal_gap " + NUM, head); tb = re.search(r"\(thermal_bridge_width " + NUM, head)
    isl = re.search(r"\(island_removal_mode (\d)", head)
    zset[(net if (is_gnd(net)) else ("PWR" if is_pwr(net) else "SIG"), mode, cl.group(2) if cl else "?", mt.group(1) if mt else "?", tg.group(1) if tg else "?", tb.group(1) if tb else "?", isl.group(1) if isl else "0")] += 1
    for fm in re.finditer(r'\(filled_polygon\s*\(layer "?([^")\s]+)"?\)(.*?)\)\s*\)', b, re.S):
        pts = np.array([(float(a), float(c_)) for a, c_ in re.findall(r"\(xy " + NUM + " " + NUM + r"\)", fm.group(2))])
        if len(pts) < 3:
            continue
        A = 0.5 * abs(np.dot(pts[:, 0], np.roll(pts[:, 1], 1)) - np.dot(pts[:, 1], np.roll(pts[:, 0], 1)))
        z_info[fm.group(1)][net] += A

    if "(filled_polygon\n" in b or re.search(r"\(filled_polygon\s*\(pts", b):
        lay = re.search(r'\(layer "?([^")\s]+)', head)
        for fm in re.finditer(r"\(filled_polygon\s*\(pts(.*?)\)\s*\)", b, re.S):
            pts = np.array([(float(a), float(c_)) for a, c_ in re.findall(r"\(xy " + NUM + " " + NUM + r"\)", fm.group(1))])
            if len(pts) < 3:
                continue
            A = 0.5 * abs(np.dot(pts[:, 0], np.roll(pts[:, 1], 1)) - np.dot(pts[:, 1], np.roll(pts[:, 0], 1)))
            z_info[lay.group(1) if lay else "?"][net] += A
p("Zone settings (net type, pad connection, spacing, min. width, thermal gap, web, island removal): ")
for k, n in zset.most_common(8):
    p(f"  {k}: {n}x")
p("Copper areas per layer (share of bounding box):")
order = sorted(z_info, key=lambda n: (0 if n == "F.Cu" else 2 if n == "B.Cu" else 1, int(re.sub(r"\D", "", n) or 0)))
for L in order:
    items = sorted(z_info[L].items(), key=lambda kv: -kv[1])[:4]
    p(f"  {L}: " + ", ".join(f"{n}:{100*a/area:.0f}%" for n, a in items) if area else "")


pad_xy = np.array([(q[0], q[1]) for q in pads]) if pads else np.zeros((0, 2))
pad_r = np.array([max(q[2], q[3]) / 2 for q in pads]) if pads else np.zeros(0)
ptree = cKDTree(pad_xy) if len(pads) else None
via_xy = np.array([(v[0], v[1]) for v in vias]) if vias else np.zeros((0, 2))
vtree = cKDTree(via_xy) if len(vias) else None
seg_end = np.array([(s[0], s[1]) for s in segs] + [(s[2], s[3]) for s in segs]) if segs else np.zeros((0, 2))
etree = cKDTree(seg_end) if len(segs) else None


vip = 0
vip_gnd = 0
vip_kind = Counter()
for i, v in enumerate(vias):
    for j in ptree.query_ball_point((v[0], v[1]), 1.5):
        q = pads[j]
        if q[8] == "smd" and q[5] == v[4] and abs(v[0]-q[0]) <= q[2]/2 + 1e-3 and abs(v[1]-q[1]) <= q[3]/2 + 1e-3 and max(q[2], q[3]) < 3.0:

            if math.hypot(v[0]-q[0], v[1]-q[1]) <= min(q[2], q[3]) / 2 + 0.05:
                vip += 1
                vip_kind[re.sub(r"[^A-Z].*", "", fps[q[7]]["ref"].upper()) or "?"] += 1
                vip_gnd += is_gnd(NN(v[4]))
                break
p(f"Via-in-pad (SMD, pad < 3 mm): {vip} vias ({100*vip/max(len(vias),1):.0f} %), of which GND {vip_gnd}")
p("  Via-in-pad by component type: " + ", ".join(f"{k}:{n}" for k, n in vip_kind.most_common(6)))

cap_g, cap_p = [], []
for fp in fps:
    if not fp["ref"].startswith("C") or len(fp["pads"]) != 2:
        continue
    for i in fp["pads"]:
        q = pads[i]
        if not q[5] or q[8] != "smd":
            continue
        cand = [j for j in (vtree.query_ball_point((q[0], q[1]), 3.0) if vtree is not None else []) if vias[j][4] == q[5]]
        d = min((math.hypot(vias[j][0]-q[0], vias[j][1]-q[1]) for j in cand), default=99)
        (cap_g if is_gnd(q[5]) else cap_p).append(d)
for nm_, arr in (("GND", cap_g), ("Supply/Signal", cap_p)):
    if arr:
        a_ = np.array(arr)
        p(f"  C pads {nm_}: n={len(a_)}, own via <=1.0 mm: {100*np.mean(a_<=1.0):.0f} %, <=0.6 mm: {100*np.mean(a_<=0.6):.0f} %, median {np.median(a_[a_<99]) if (a_<99).any() else 0:.2f} mm")


stitch = []
for v in gv:
    if etree is not None and etree.query_ball_point((v[0], v[1]), 0.02):
        continue
    near = ptree.query_ball_point((v[0], v[1]), 3.0)
    if any(math.hypot(v[0]-pads[j][0], v[1]-pads[j][1]) - pad_r[j] < 0.5 for j in near):
        continue
    stitch.append((v[0], v[1]))
if len(stitch) > 3:
    S = np.array(stitch); d, _ = cKDTree(S).query(S, k=2)
    nn = d[:, 1]
    p(f"Stitching vias (GND, free, without track): {len(stitch)}, density {len(stitch)/(area/100):.1f}/cm², neighbor spacing median {np.median(nn):.2f} mm (P10 {np.percentile(nn,10):.2f}, P90 {np.percentile(nn,90):.2f})")
    if len(ex):
        xmin, xmax, ymin, ymax = ex[:, 0].min(), ex[:, 0].max(), ex[:, 1].min(), ex[:, 1].max()
        de = np.minimum.reduce([S[:, 0]-xmin, xmax-S[:, 0], S[:, 1]-ymin, ymax-S[:, 1]])
        E = S[de < 2.0]
        if len(E) > 3:
            d2, _ = cKDTree(E).query(E, k=2)
            p(f"  at edge (<2 mm to bounding box): {len(E)}, spacing median {np.median(d2[:,1]):.2f} mm")


if gv:
    gtree = cKDTree(np.array([(v[0], v[1]) for v in gv]))
    sv = [v for v in vias if not is_gnd(NN(v[4])) and not is_pwr(NN(v[4]))]
    if sv:
        d, _ = gtree.query(np.array([(v[0], v[1]) for v in sv]))
        p(f"Signal vias {len(sv)}: spacing to nearest GND via median {np.median(d):.2f} mm, P75 {np.percentile(d,75):.2f}, P90 {np.percentile(d,90):.2f}")


def pair_key(n):
    m = re.match(r"^(.*?)(_?)([PN]|\+|-|_P|_N|P\d*|N\d*)$", n)
    for suf_p, suf_n in (("_P", "_N"), ("+", "-"), ("P", "N"), ("_DP", "_DN"), ("_p", "_n")):
        if n.endswith(suf_p):
            return n[: -len(suf_p)], suf_p, suf_n, "P"
        if n.endswith(suf_n):
            return n[: -len(suf_n)], suf_p, suf_n, "N"
    return None
allnames = {s_[6] for s_ in segs} | set(nets.values())
byname = {v: v for v in allnames}
pairs = []
for n in allnames:
    k = n
    pk = pair_key(n)
    if pk and pk[3] == "P":
        other = pk[0] + pk[2]
        if other in byname:
            pairs.append((k, byname[other], pk[0]))
segs_by_net = defaultdict(list)
for s in segs:
    if not s[8]:
        segs_by_net[s[6]].append(s)

def pt_seg_dist(P, A, B):
    AB = B - A; L2 = (AB**2).sum(1)
    t = np.clip(((P[:, None, :] - A[None]) * AB[None]).sum(2) / np.maximum(L2[None], 1e-12), 0, 1)
    proj = A[None] + t[..., None] * AB[None]
    return np.sqrt(((P[:, None, :] - proj)**2).sum(2))

gaps = []; dpw = []; dplay = Counter()
for a, b_, base in pairs:
    sa, sb = segs_by_net.get(a, []), segs_by_net.get(b_, [])
    if not sa or not sb:
        continue
    for s in sa:
        dplay[s[5]] += s[7]
    for s in sb:
        dplay[s[5]] += s[7]
    for L in {s[5] for s in sa}:
        A_ = [s for s in sa if s[5] == L and s[7] > 0.3]
        B_ = [s for s in sb if s[5] == L]
        if not A_ or not B_:
            continue
        P = np.array([((s[0]+s[2])/2, (s[1]+s[3])/2) for s in A_])
        BA = np.array([(s[0], s[1]) for s in B_]); BB = np.array([(s[2], s[3]) for s in B_])
        d = pt_seg_dist(P, BA, BB).min(1)
        for s, dd in zip(A_, d):
            w = s[4]
            if dd < 6 * w + 1:
                gaps.append((round(w, 3), round(dd - w, 3), s[7]))
                dpw.append(w)
if gaps:
    gc = Counter()
    for w, g, L in gaps:
        gc[(w, round(g, 2))] += L
    p(f"Diff pairs: {len(pairs)} detected; (width, gap) by length: " + ", ".join(f"{w:g}/{g:g}:{v:.0f}mm" for (w, g), v in gc.most_common(6)))
    T = sum(dplay.values())
    p("  Layers of diff pairs: " + ", ".join(f"{L}:{100*v/T:.0f}%" for L, v in dplay.most_common()))


def pkg_of(fp):
    m = re.search(r"(01005|0201|0402|0603|0805|1206|1210|1812|2220)", fp["name"])
    return m.group(1) if m else None
def val_f(v):
    v = re.sub(r"^C_", "", v)
    m = re.match(r"\s*(\d*)(?:[.,](\d+))?\s*([pnuµμm])(\d*)", v)
    if not m or not (m.group(1) or m.group(2) or m.group(4)):
        return None
    ip = m.group(1) or "0"; fr = m.group(2) or m.group(4) or "0"
    return float(ip + "." + fr) * {"p": 1e-12, "n": 1e-9, "u": 1e-6, "µ": 1e-6, "μ": 1e-6, "m": 1e-3}[m.group(3)]

for fi, fp in enumerate(fps):
    pl = [pads[i] for i in fp["pads"] if pads[i][8] == "smd"]
    if len(pl) < 100 or sum(1 for q in pl if q[4] == "circle") < 0.8 * len(pl):
        continue
    loc = np.array([(q[9], q[10]) for q in pl])
    d, _ = cKDTree(loc).query(loc, k=2)
    pitch = float(np.median(d[:, 1]))
    xs = np.round((loc[:, 0] - loc[:, 0].min()) / pitch).astype(int)
    ys = np.round((loc[:, 1] - loc[:, 1].min()) / pitch).astype(int)
    nc, nr = xs.max() + 1, ys.max() + 1
    ring = np.minimum.reduce([xs, ys, nc - 1 - xs, nr - 1 - ys])
    padd = float(np.median([q[2] for q in pl]))
    stats = defaultdict(Counter)
    dog = []
    exitlayer = defaultdict(Counter)
    for q, rg in zip(pl, ring):
        n = q[5]
        if not n:
            stats[min(rg, 6)]["nc"] += 1
            continue
        kind = None
        cand = vtree.query_ball_point((q[0], q[1]), 1.6 * pitch) if vtree is not None else []
        cand = [j for j in cand if vias[j][4] == n]
        best = None
        for j in cand:
            dd = math.hypot(vias[j][0]-q[0], vias[j][1]-q[1])
            if best is None or dd < best[0]:
                best = (dd, j)
        if best and best[0] <= q[2] / 2 + 0.02:
            kind = "vip"
        elif best and best[0] <= 1.1 * pitch:
            kind = "dogbone"; dog.append(best[0])
        else:
            kind = "no_via"
        cat = "GND" if is_gnd(NN(n)) else "PWR" if is_pwr(NN(n)) else "SIG"
        stats[min(rg, 6)][kind] += 1
        stats[min(rg, 6)]["cat_" + cat] += 1

    via_exit = defaultdict(Counter)
    seg_by_pt = defaultdict(list)
    for s in segs:
        seg_by_pt[(round(s[0], 2), round(s[1], 2))].append(s)
        seg_by_pt[(round(s[2], 2), round(s[3], 2))].append(s)
    for q, rg in zip(pl, ring):
        n = q[5]
        if not n or is_gnd(NN(n)) or is_pwr(NN(n)):
            continue
        cand = vtree.query_ball_point((q[0], q[1]), 1.1 * pitch) if vtree is not None else []
        cand = [j for j in cand if vias[j][4] == n]
        if not cand:

            ss = seg_by_pt.get((round(q[0], 2), round(q[1], 2)), [])
            if ss:
                via_exit[min(rg, 6)][ss[0][5] + "(direct)"] += 1
            continue
        j = min(cand, key=lambda j: math.hypot(vias[j][0]-q[0], vias[j][1]-q[1]))
        ss = [s for s in seg_by_pt.get((round(vias[j][0], 2), round(vias[j][1], 2)), []) if s[6] == n]
        lay = Counter(s[5] for s in ss)
        pad_layer = "F.Cu" if q[6] == "F" else "B.Cu"
        others = [L for L in lay if L != pad_layer]
        via_exit[min(rg, 6)][(others[0] if others else (pad_layer + "(via only)")) ] += 1

    xs_a = np.array([q[0] for q in pl]); ys_a = np.array([q[1] for q in pl])
    bx0, bx1, by0, by1 = xs_a.min()-0.3, xs_a.max()+0.3, ys_a.min()-0.3, ys_a.max()+0.3
    under = Counter(); under_same = Counter()
    for g in fps:
        if not g["ref"].startswith("C"):
            continue
        if bx0 <= g["x"] <= bx1 and by0 <= g["y"] <= by1:
            (under if g["layer"] != fp["layer"] else under_same)[pkg_of(g) or "?"] += 1

    inside = [s for s in segs if bx0 <= s[0] <= bx1 and by0 <= s[1] <= by1 and bx0 <= s[2] <= bx1 and by0 <= s[3] <= by1 and not is_gnd(NN(s[6])) and not is_pwr(NN(s[6]))]
    p(f"BGA {fp['ref']} ({fp['name'][:40]}): {len(pl)} balls, grid {pitch:.2f} mm, pad {padd:.3f} mm, {nr}x{nc}")
    for rg in sorted(stats):
        c_ = stats[rg]
        tot_ = c_['vip'] + c_['dogbone'] + c_['no_via']
        p(f"  Ring {rg}{'+' if rg==6 else ''}: {tot_} balls (SIG {c_['cat_SIG']}, PWR {c_['cat_PWR']}, GND {c_['cat_GND']}) -> via-in-pad {c_['vip']}, dogbone {c_['dogbone']}, without via {c_['no_via']}; signal exit: " + ", ".join(f"{L}:{n}" for L, n in via_exit[rg].most_common(4)))
    if dog:
        p(f"  Dogbone spacing ball->via median {np.median(dog):.3f} mm (= {np.median(dog)/pitch:.2f} x grid)")
    p(f"  Capacitors under BGA (opposite side): {dict(under)}; same side: {dict(under_same)}")
    if inside:
        p(f"  Signal track widths in BGA array: " + whist(inside, 4))


ic_pwr_pads = defaultdict(list)
for fi, fp in enumerate(fps):
    if fp["ref"][:1] == "U" and len(fp["pads"]) >= 6:
        for i in fp["pads"]:
            q = pads[i]
            if is_pwr(NN(q[5])):
                ic_pwr_pads[q[5]].append((q[0], q[1], fi))
trees = {n: cKDTree(np.array([(a, b) for a, b, _ in v])) for n, v in ic_pwr_pads.items()}
res = defaultdict(list)
for fi, fp in enumerate(fps):
    if not fp["ref"].startswith("C") or len(fp["pads"]) != 2:
        continue
    a, b_ = [pads[i] for i in fp["pads"]]
    na, nb = NN(a[5]), NN(b_[5])
    if is_gnd(na) and b_[5] in trees:
        pp = b_
    elif is_gnd(nb) and a[5] in trees:
        pp = a
    else:
        continue
    d, k_ = trees[pp[5]].query((pp[0], pp[1]))
    ic_fi = ic_pwr_pads[pp[5]][k_][2]
    opp = fps[ic_fi]["layer"] != fp["layer"]
    v = val_f(fp["val"]) if fp["val"] else None
    if v is None:
        continue
    bucket = "<=10n" if v <= 10.1e-9 else "100n" if v <= 220e-9 else "0.47-1u" if v <= 1.1e-6 else "2.2-4.7u" if v <= 4.8e-6 else "10u" if v <= 10.1e-6 else ">10u"
    res[bucket].append(d); res[bucket + "_opp"].append(opp)
if res:
    p("Decoupling: spacing C pad -> nearest IC supply pin of the same net (median mm, count):")
    for k in ["<=10n", "100n", "0.47-1u", "2.2-4.7u", "10u", ">10u"]:
        if res[k]:
            p(f"  {k}: {np.median(res[k]):.2f} mm (n={len(res[k])}, P25 {np.percentile(res[k],25):.2f}, P75 {np.percentile(res[k],75):.2f}, opposite side {100*np.mean(res[k+'_opp']):.0f} %)")
pk = Counter(pkg_of(fp) for fp in fps if fp["ref"].startswith("C"))
p("Capacitor packages: " + ", ".join(f"{k}:{v}" for k, v in pk.most_common(6)))
pk = Counter(pkg_of(fp) for fp in fps if fp["ref"].startswith("R"))
p("Resistor packages: " + ", ".join(f"{k}:{v}" for k, v in pk.most_common(5)))


if pro_path and os.path.exists(pro_path):
    pro = json.load(open(pro_path))
    r = pro.get("board", {}).get("design_settings", {}).get("rules", {})
    keys = ["min_clearance", "min_track_width", "min_via_diameter", "min_via_annular_width", "min_through_hole_diameter", "min_hole_clearance", "min_hole_to_hole", "min_copper_edge_clearance", "min_microvia_diameter", "min_microvia_drill"]
    p("Rules (kicad_pro): " + ", ".join(f"{k.replace('min_','')}={r.get(k)}" for k in keys if k in r))
    ds = pro.get("board", {}).get("design_settings", {})
    if ds.get("track_widths"):
        p("  Track width constraints: " + ", ".join(f"{x:g}" for x in ds["track_widths"] if x))
    if ds.get("via_dimensions"):
        p("  Via constraints: " + ", ".join(f"{x['diameter']:g}/{x['drill']:g}" for x in ds["via_dimensions"] if x.get("diameter")))
    if ds.get("diff_pair_dimensions"):
        p("  Diff pair constraints: " + ", ".join(f"{x.get('width',0):g}/{x.get('gap',0):g}" for x in ds["diff_pair_dimensions"] if x.get("width")))
    for c in pro.get("net_settings", {}).get("classes", []):
        p(f"  Netclass {c.get('name')}: spacing {c.get('clearance')}, track {c.get('track_width')}, Via {c.get('via_diameter')}/{c.get('via_drill')}, DP {c.get('diff_pair_width')}/{c.get('diff_pair_gap')}")


if os.environ.get("SPACING"):
    pairset = set()
    for a_, b2_, _ in pairs:
        pairset.add((a_, b2_)); pairset.add((b2_, a_))
    for L in layers:
        pts, nid, wid = [], [], []
        for s in segs:
            if s[5] != L or s[8] or s[7] < 0.8:
                continue
            n = max(int((s[7] - 0.6) / 0.1), 1)
            for k in range(n + 1):
                t = (0.3 + k * 0.1) / s[7]
                if t > 1 - 0.3 / s[7]:
                    break
                pts.append((s[0] + t * (s[2]-s[0]), s[1] + t * (s[3]-s[1])))
                nid.append(s[6]); wid.append(s[4])
        if len(pts) < 100:
            continue
        P = np.array(pts); T = cKDTree(P)
        dd, ii = T.query(P, k=12, distance_upper_bound=1.5)
        gaps = []
        for r in range(len(P)):
            n0 = nid[r]
            if is_gnd(n0) or is_pwr(n0):
                continue
            g = None
            for d_, j in zip(dd[r], ii[r]):
                if j >= len(P) or not np.isfinite(d_):
                    break
                if nid[j] != n0 and (n0, nid[j]) not in pairset:
                    g = d_ - wid[r]/2 - wid[j]/2
                    break
            if g is not None and g > 0:
                gaps.append(g / wid[r])
        if len(gaps) > 50:
            G = np.array(gaps)
            p(f"Spacing signal track -> nearest foreign track ({L}, tracks only, without pair partner, <1.5 mm): median {np.median(G):.2f} W, share <1W {100*np.mean(G<1):.0f} %, <2W {100*np.mean(G<2):.0f} %, <3W {100*np.mean(G<3):.0f} %  (n={len(G)})")
