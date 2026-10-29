import pcbnew, sys, json
src, out = sys.argv[1:3]
args=sys.argv[3:]
opts=dict(a.split('=',1) for a in args if '=' in a)
NETS=next((a.split(',') for a in args if '=' not in a),None)
ERR=float(opts.get('faults','0.005'))
GRAPHIC=int(opts.get('graphic','0'))
if not 0<ERR<=.005:raise ValueError('Polygon error must be >0 and <=0,005 mm')
b = pcbnew.LoadBoard(src)
ToMM = pcbnew.ToMM
CU = [l for l in b.GetEnabledLayers().CuStack()]
LNAME = {l: pcbnew.BOARD.GetStandardLayerName(l).replace('.Cu', '') for l in CU}


def polyset_out(ps):
    res = []
    for i in range(ps.OutlineCount()):
        o = ps.Outline(i)
        outer = [(ToMM(o.CPoint(k).x), ToMM(o.CPoint(k).y)) for k in range(o.PointCount())]
        holes = []
        for h in range(ps.HoleCount(i)):
            hh = ps.Hole(i, h)
            holes.append([(ToMM(hh.CPoint(k).x), ToMM(hh.CPoint(k).y)) for k in range(hh.PointCount())])
        res.append([outer, holes])
    return res


data = {'layers': [LNAME[l] for l in CU], 'nets': {}, 'vias': [], 'pads': []}
acc = {}
def ps_for(net, L):
    k = (net, L)
    if k not in acc: acc[k] = pcbnew.SHAPE_POLY_SET()
    return acc[k]

for z in b.Zones():
    net = z.GetNetname()
    if NETS and net not in NETS: continue
    for L in CU:
        if not z.IsOnLayer(L): continue
        fp = z.GetFilledPolysList(L)
        if fp.OutlineCount(): ps_for(net, L).Append(fp)
for t in b.GetTracks():
    net = t.GetNetname()
    if NETS and net not in NETS: continue
    if t.Type() == pcbnew.PCB_VIA_T:
        data['vias'].append(dict(net=net, x=ToMM(t.GetPosition().x), y=ToMM(t.GetPosition().y), d=ToMM(t.GetWidth(pcbnew.F_Cu)), drill=ToMM(t.GetDrillValue())))
        for L in CU:
            if t.IsOnLayer(L): t.TransformShapeToPolygon(ps_for(net, L), L, 0, pcbnew.FromMM(ERR), pcbnew.ERROR_INSIDE)
        continue
    L = t.GetLayer()
    t.TransformShapeToPolygon(ps_for(net, L), L, 0, pcbnew.FromMM(ERR), pcbnew.ERROR_INSIDE)
for fp in b.GetFootprints():
    for p in fp.Pads():
        net = p.GetNetname()
        lay = [LNAME[L] for L in CU if p.IsOnLayer(L)]
        data['pads'].append(dict(ref=fp.GetReference(), num=p.GetNumber(), net=net, x=ToMM(p.GetPosition().x), y=ToMM(p.GetPosition().y), layers=lay))
        if NETS and net not in NETS: continue
        for L in CU:
            if p.IsOnLayer(L): p.TransformShapeToPolygon(ps_for(net, L), L, 0, pcbnew.FromMM(ERR), pcbnew.ERROR_INSIDE)
if GRAPHIC:
    for d in b.GetDrawings():
        L=d.GetLayer()
        if L not in CU:continue
        net=d.GetNetname() if hasattr(d,'GetNetName') else ''
        if not net or (NETS and net not in NETS):continue
        d.TransformShapeToPolygon(ps_for(net,L),L,0,pcbnew.FromMM(ERR),pcbnew.ERROR_INSIDE)
for (net, L), ps in acc.items():
    ps.Simplify()
    data['nets'].setdefault(net, {})[LNAME[L]] = polyset_out(ps)
json.dump(data, open(out, 'w'))
print('Nets', len(data['nets']), 'Vias', len(data['vias']), 'Pads', len(data['pads']))
