""
import pcbnew, sys, json, math
import wx
_app = wx.GetApp() or wx.App(False)
_log_target = wx.LogStderr()
_log_previous = wx.Log.SetActiveTarget(_log_target)
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), 'kicad'))
from projekt_kopie import regeln_sichern, regeln_kopieren, regeln_ziel_leeren
src, pj, out = sys.argv[1:4]
_regeln = regeln_sichern(src)
b = pcbnew.LoadBoard(src)
PLACEMENT = json.load(open(pj))
P = PLACEMENT['parts']
MM = pcbnew.FromMM


def pads_xy(fp):
    return {p.GetNumber(): (pcbnew.ToMM(p.GetPosition().x), pcbnew.ToMM(p.GetPosition().y)) for p in fp.Pads()}


def padinstanzen(fp):
    return [dict(num=p.GetNumber(), net=p.GetNetname(), x=pcbnew.ToMM(p.GetPosition().x),
                 y=pcbnew.ToMM(p.GetPosition().y), side='FB' if p.GetDrillSize().x > 0 or len(list(p.GetLayerSet().CuStack())) > 1
                 else 'B' if fp.IsFlipped() else 'F') for p in fp.Pads()]


def pruefe_instanzen(actual, expected, ref):

    if len(actual) != len(expected): raise ValueError('Padanzahl stimmt nicht: ' + ref)
    owner = {}
    def match(i, seen):
        for j, q in enumerate(actual):
            e = expected[i]
            if j in seen or any(q[k] != e[k] for k in ('num', 'net', 'side')): continue
            if math.dist((q['x'], q['y']), (e['x'], e['y'])) > 0.01: continue
            seen.add(j)
            if j not in owner or match(owner[j], seen): owner[j] = i; return True
        return False
    if not all(match(i, set()) for i in range(len(expected))):
        raise ValueError('Padinstanzen weichen > 0.01 mm ab: ' + ref)
    return max((math.dist((actual[j]['x'], actual[j]['y']), (expected[i]['x'], expected[i]['y']))
                for j, i in owner.items()), default=0.0)


worst = 0.0
for fp in b.GetFootprints():
    r = fp.GetReference()
    if r not in P: continue
    tgt = P[r]['pads']; side = P[r]['side']
    before = padinstanzen(fp)
    pos0 = fp.GetPosition(); x0, y0 = pcbnew.ToMM(pos0.x), pcbnew.ToMM(pos0.y)
    side0 = 'B' if fp.IsFlipped() else 'F'
    if ('B' if fp.IsFlipped() else 'F') != side:
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)


    if P[r].get('transformation_eindeutig') and all(k in P[r] for k in ('x', 'y', 'drehung')):

        th = math.radians(P[r]['drehung']); c, s = math.cos(th), math.sin(th)
        predicted = []
        for q in before:
            lx, ly = q['x'] - x0, q['y'] - y0
            if side0 != side: lx = -lx
            predicted.append(dict(q, x=P[r]['x'] + c * lx - s * ly, y=P[r]['y'] + s * lx + c * ly,
                                  side='FB' if q['side'] == 'FB' else side))
        fp.Rotate(fp.GetPosition(), pcbnew.EDA_ANGLE(-P[r]['drehung'], pcbnew.DEGREES_T))
        fp.SetPosition(pcbnew.VECTOR2I(MM(P[r]['x']), MM(P[r]['y'])))
        actual = padinstanzen(fp)
        worst = max(worst, pruefe_instanzen(actual, P[r].get('padinstanzen', predicted), r))
        for num, xy in tgt.items():
            err = min((math.dist((q['x'], q['y']), xy) for q in actual if q['num'] == num), default=math.inf)
            if err > 0.01: raise ValueError('Ziel-Padanker weicht > 0.01 mm ab: ' + r + ':' + num)
            worst = max(worst, err)
        continue
    cur = pads_xy(fp)
    keys = [k for k in tgt if k in cur]
    if not keys:
        raise ValueError('Keine Padanker; explizite Transformation erforderlich: ' + r)

    if len(keys) >= 2:
        best = None
        for i in range(len(keys)):
            for j in range(i + 1, len(keys)):
                a, c = keys[i], keys[j]
                d = math.dist(cur[a], cur[c])
                if best is None or d > best[0]: best = (d, a, c)
        _, a, c = best
        ang_c = math.atan2(cur[c][1] - cur[a][1], cur[c][0] - cur[a][0])
        ang_t = math.atan2(tgt[c][1] - tgt[a][1], tgt[c][0] - tgt[a][0])
        dth = math.degrees(ang_t - ang_c)

        fp.Rotate(fp.GetPosition(), pcbnew.EDA_ANGLE(-dth, pcbnew.DEGREES_T))
        cur = pads_xy(fp)
    cx = sum(cur[k][0] for k in keys) / len(keys); cy = sum(cur[k][1] for k in keys) / len(keys)
    tx = sum(tgt[k][0] for k in keys) / len(keys); ty = sum(tgt[k][1] for k in keys) / len(keys)
    fp.Move(pcbnew.VECTOR2I(MM(tx - cx), MM(ty - cy)))
    cur = pads_xy(fp)
    err = max(math.dist(cur[k], tgt[k]) for k in keys)
    if err > 0.01:

        fp.Rotate(fp.GetPosition(), pcbnew.EDA_ANGLE(2 * dth, pcbnew.DEGREES_T))
        cur = pads_xy(fp)
        cx = sum(cur[k][0] for k in keys) / len(keys); cy = sum(cur[k][1] for k in keys) / len(keys)
        fp.Move(pcbnew.VECTOR2I(MM(tx - cx), MM(ty - cy)))
        cur = pads_xy(fp); err = max(math.dist(cur[k], tgt[k]) for k in keys)
    worst = max(worst, err)
    if err > 0.01: print('ABWEICHUNG', r, round(err, 3))
if PLACEMENT.get('outline'):
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from analyse.dichte_native import pruefen
    native = pruefen(b)
    if not native['eng_erlaubt']:
        print('P26: Rand unveraendert (native Ausschnitte/Formen)', native)
        PLACEMENT.pop('outline')
if PLACEMENT.get('outline'):


    def bounds(polys):
        pts = [p for q in polys for p in q]
        return min(p[0] for p in pts), min(p[1] for p in pts), max(p[0] for p in pts), max(p[1] for p in pts)
    old = bounds(PLACEMENT['outline_original']); new = bounds(PLACEMENT['outline'])
    if new[0] < old[0] or new[1] < old[1] or new[2] > old[2] or new[3] > old[3]:
        raise ValueError('P26: Umriss darf nicht wachsen')
    def transform(p):
        x, y = pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)
        return pcbnew.VECTOR2I(MM(new[0] + (x - old[0]) * (new[2] - new[0]) / (old[2] - old[0])),
                              MM(new[1] + (y - old[1]) * (new[3] - new[1]) / (old[3] - old[1])))
    edges = [d for d in b.GetDrawings() if d.GetLayer() == pcbnew.Edge_Cuts and isinstance(d, pcbnew.PCB_SHAPE)]
    for d in edges:
        if d.GetShape() == pcbnew.SHAPE_T_POLY:
            ps = d.GetPolyShape()
            for i in range(ps.OutlineCount()):
                line = ps.Outline(i)
                for k in range(line.PointCount()): line.SetPoint(k, transform(line.CPoint(k)))
        elif d.GetShape() in (pcbnew.SHAPE_T_SEGMENT, pcbnew.SHAPE_T_RECT):
            a, e = d.GetStart(), d.GetEnd(); d.SetStart(transform(a)); d.SetEnd(transform(e))
        else:
            raise ValueError('P26: unerwartete Form im rechteckigen Umriss')
regeln_ziel_leeren(out)
b.Save(out)
regeln_kopieren(_regeln, out, erzeugtes_pro=True)
print('uebernommen', len(P), 'Teile, groesste Padabweichung %.4f mm ->' % worst, out)
