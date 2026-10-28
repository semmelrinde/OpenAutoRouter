""
import pcbnew, sys, json
b = pcbnew.LoadBoard(sys.argv[1]); mm = pcbnew.ToMM; out = []
zs = [(z, '') for z in b.Zones()] + [(z, fp.GetReference()) for fp in b.GetFootprints() for z in fp.Zones()]
for z, ref in zs:
    if not z.GetIsRuleArea(): continue
    o = z.Outline()
    for i in range(o.OutlineCount()):
        ol = o.Outline(i)
        out.append(dict(ref=ref, lagen=[pcbnew.BOARD.GetStandardLayerName(L).replace('.Cu', '') for L in z.GetLayerSet().CuStack()],
                        fuellen=z.GetDoNotAllowZoneFills(), bahnen=z.GetDoNotAllowTracks(), vias=z.GetDoNotAllowVias(),
                        pts=[(mm(ol.CPoint(k).x), mm(ol.CPoint(k).y)) for k in range(ol.PointCount())]))
json.dump(out, open(sys.argv[2], 'w')); print('Sperrflaechen', len(out))
