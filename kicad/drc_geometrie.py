""
import hashlib
import json
from pathlib import Path
import sys


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


def ring(line):
    points = [(line.CPoint(k).x, line.CPoint(k).y) for k in range(line.PointCount())]
    if points and points[0] == points[-1]: points.pop()
    if not points: return []

    variants = []
    for seq in (points, points[::-1]):
        lowest = min(seq)
        variants.extend(seq[k:] + seq[:k] for k, point in enumerate(seq) if point == lowest)
    return min(variants)


def contours(poly):
    return sorted([ring(poly.Outline(i)), sorted(ring(poly.Hole(i, j)) for j in range(poly.HoleCount(i)))]
                  for i in range(poly.OutlineCount()))


def snapshot(board, pcbnew):
    def shape(item, layer):
        poly = pcbnew.SHAPE_POLY_SET()
        item.TransformShapeToPolygon(poly, layer, 0, pcbnew.FromMM(0.001), pcbnew.ERROR_OUTSIDE)
        return contours(poly)

    parts = {}
    copper = list(board.GetEnabledLayers().CuStack())
    for fp in board.GetFootprints():
        ref = fp.GetReference()
        if ref in parts: raise ValueError('Mehrdeutige Footprint-Referenz: ' + ref)
        pads = []
        for pad in fp.Pads():
            hole = pcbnew.SHAPE_POLY_SET()
            if pad.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH):
                pad.TransformHoleToPolygon(hole, 0, pcbnew.FromMM(0.001), pcbnew.ERROR_OUTSIDE)
            pads.append(dict(num=pad.GetNumber(), net=pad.GetNetname(),
                             angle=pad.GetOrientationDegrees(), type=int(pad.GetAttribute()),
                             shape=int(pad.GetShape()), size=[pad.GetSize().x, pad.GetSize().y],
                             drill=[pad.GetDrillSize().x, pad.GetDrillSize().y],
                             copper={str(layer): shape(pad, layer) for layer in copper if pad.IsOnLayer(layer)},
                             mask={str(layer): shape(pad, layer) for layer in (pcbnew.F_Mask, pcbnew.B_Mask) if pad.IsOnLayer(layer)},
                             mask_expansion={str(layer): pad.GetSolderMaskExpansion(layer) for layer in (pcbnew.F_Mask, pcbnew.B_Mask) if pad.IsOnLayer(layer)},
                             hole=contours(hole)))
        graphics = [dict(layer=g.GetLayer(), shape=shape(g, g.GetLayer())) for g in fp.GraphicalItems()
                    if g.GetLayer() in copper or g.GetLayer() in (pcbnew.F_CrtYd, pcbnew.B_CrtYd, pcbnew.Edge_Cuts, pcbnew.F_Mask, pcbnew.B_Mask)]
        pos = fp.GetPosition()
        parts[ref] = digest(dict(pos=[pos.x, pos.y], orientation=fp.GetOrientationDegrees(), side='B' if fp.IsFlipped() else 'F',
                                 pads=sorted(pads, key=lambda v: json.dumps(v, sort_keys=True)),
                                 graphics=sorted(graphics, key=lambda v: json.dumps(v, sort_keys=True)),
                                 courtyard={str(layer): contours(fp.GetCourtyard(layer)) for layer in (pcbnew.F_CrtYd, pcbnew.B_CrtYd)}))
    drawings = [dict(layer=g.GetLayer(), shape=shape(g, g.GetLayer())) for g in board.GetDrawings()
                if g.GetLayer() in copper or g.GetLayer() in (pcbnew.Edge_Cuts, pcbnew.F_Mask, pcbnew.B_Mask)]
    zones = [dict(layers=list(z.GetLayerSet().Seq()), outline=contours(z.Outline()),
                  rule_area=z.GetIsRuleArea(), net=z.GetNetname(),
                  filled={str(layer): contours(z.GetFilledPolysList(layer))
                          for layer in copper if z.IsOnLayer(layer) and not z.GetIsRuleArea()})
             for z in board.Zones()]

    return dict(version=2, teile=parts, umgebung=digest(dict(drawings=sorted(drawings, key=lambda v: json.dumps(v, sort_keys=True)),
                                                         zones=sorted(zones, key=lambda v: json.dumps(v, sort_keys=True)))))

def main():
    import wx
    import pcbnew
    app = wx.GetApp() or wx.App(False)
    target = wx.LogStderr()
    previous = wx.Log.SetActiveTarget(target)
    board = pcbnew.LoadBoard(sys.argv[1])
    path = Path(sys.argv[2]); data = json.loads(path.read_text())
    data['autorouter_geometrie'] = snapshot(board, pcbnew)
    data['autorouter_pcb_sha256'] = hashlib.sha256(Path(sys.argv[1]).read_bytes()).hexdigest()
    path.write_text(json.dumps(data, indent=2))

    return app, target, previous, board


if __name__ == '__main__':
    _owners = main()
