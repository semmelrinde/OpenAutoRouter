import pcbnew, sys, math
from PIL import Image, ImageDraw, ImageFont
src, out, lay = sys.argv[1:4]; x0, y0, x1, y1 = [float(v) for v in sys.argv[4:8]]
S = float(sys.argv[8]) if len(sys.argv) > 8 and '=' not in sys.argv[8] else 40
A = dict(a.split('=', 1) for a in sys.argv[8:] if '=' in a)
HIGHLIGHT = [n for n in A.get('net_list', '/VIN').split(',') if n]; GND = A.get('gnd', '/GND')
COLORS = [(220, 40, 40), (240, 130, 30), (40, 160, 70), (150, 60, 190)]
b = pcbnew.LoadBoard(src); LID = b.GetLayerID(lay)
W, H = int((x1 - x0) * S), int((y1 - y0) * S)
in_ = Image.new('RGB', (W, H), (250, 250, 248)); d = ImageDraw.Draw(in_)
def T(x, y): return ((x - x0) * S, (y - y0) * S)
def col(n):
    if n == GND: return (150, 150, 150)
    if n in HIGHLIGHT[:len(COLORS)]: return COLORS[HIGHLIGHT.index(n)]
    return (70, 110, 200)
def poly(sps, c):
    for i in range(sps.OutlineCount()):
        o = sps.Outline(i); pts = [T(pcbnew.ToMM(o.CPoint(k).x), pcbnew.ToMM(o.CPoint(k).y)) for k in range(o.PointCount())]
        if len(pts) > 2: d.polygon(pts, fill=c)
        for h in range(sps.HoleCount(i)):
            ho = sps.Hole(i, h); hp = [T(pcbnew.ToMM(ho.CPoint(k).x), pcbnew.ToMM(ho.CPoint(k).y)) for k in range(ho.PointCount())]
            if len(hp) > 2: d.polygon(hp, fill=(250, 250, 248))
for z in b.Zones():
    if z.GetLayer() == LID and z.IsFilled(): poly(z.GetFilledPolysList(LID), col(z.GetNetname()))
for g in b.GetDrawings():
    if g.GetClass() == 'PCB_SHAPE' and g.GetLayer() == LID:
        sp = pcbnew.SHAPE_POLY_SET(); g.TransformShapeToPolygon(sp, LID, 0, pcbnew.FromMM(0.005), pcbnew.ERROR_INSIDE); poly(sp, col(g.GetNetname()))
for t in b.GetTracks():
    if t.GetClass() in ('PCB_TRACK', 'PCB_ARC') and t.GetLayer() == LID:
        sp = pcbnew.SHAPE_POLY_SET(); t.TransformShapeToPolygon(sp, LID, 0, pcbnew.FromMM(0.005), pcbnew.ERROR_INSIDE); poly(sp, col(t.GetNetname()))
for f in b.GetFootprints():
    for p in f.Pads():
        if p.IsOnLayer(LID):
            sp = pcbnew.SHAPE_POLY_SET(); p.TransformShapeToPolygon(sp, LID, 0, pcbnew.FromMM(0.005), pcbnew.ERROR_INSIDE); poly(sp, tuple(int(c * 0.8) for c in col(p.GetNetname())))
for t in b.GetTracks():
    if t.GetClass() == 'PCB_VIA':
        x, y = pcbnew.ToMM(t.GetPosition().x), pcbnew.ToMM(t.GetPosition().y); r = pcbnew.ToMM(t.GetWidth(LID)) / 2
        d.ellipse([T(x - r, y - r), T(x + r, y + r)], fill=tuple(int(c * 0.7) for c in col(t.GetNetname())))
        rr = pcbnew.ToMM(t.GetDrillValue()) / 2; d.ellipse([T(x - rr, y - rr), T(x + rr, y + rr)], fill=(255, 255, 255))
edge = pcbnew.SHAPE_POLY_SET(); b.GetBoardPolygonOutlines(edge, True)
for i in range(edge.OutlineCount()):
    o = edge.Outline(i); d.line([T(pcbnew.ToMM(o.CPoint(k).x), pcbnew.ToMM(o.CPoint(k).y)) for k in range(o.PointCount())] + [T(pcbnew.ToMM(o.CPoint(0).x), pcbnew.ToMM(o.CPoint(0).y))], fill=(0, 0, 0), width=2)
f = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', 22)
d.text((8, 6), lay + '   ' + '   '.join('%s: %s' % (fn, n.split('/')[-1]) for fn, n in zip(('red', 'orange', 'green', 'violet'), HIGHLIGHT))
       + '   gray: ' + GND.split('/')[-1], fill=(0, 0, 0), font=f)
in_.save(out); print(out, in_.size)
