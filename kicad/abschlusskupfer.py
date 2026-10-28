""


def zeichnungen(board, names, einheiten=False):
    import pcbnew as P
    mm = P.ToMM
    def xy(c):
        return [[mm(c.CPoint(k).x), mm(c.CPoint(k).y)] for k in range(c.PointCount())]
    zones = []
    for i, drawing in enumerate(board.GetDrawings()):
        layer = drawing.GetLayer()
        if layer not in names or not P.IsCopperLayer(layer): continue
        polygons = P.SHAPE_POLY_SET()
        drawing.TransformShapeToPolygon(polygons, layer, 0, P.FromMM(.005), P.ERROR_OUTSIDE)
        net = drawing.GetNetname() if hasattr(drawing, 'GetNetname') else ''
        for j in range(polygons.OutlineCount()):
            zones.append(dict(net=net or '#CU_%d' % i, layer=names[layer], filled=True,
                              pts=xy(polygons.Outline(j)),
                              holes=[xy(polygons.Hole(j, h)) for h in range(polygons.HoleCount(j))],
                              board_copper=True,
                              **({'einheit': str(drawing.m_Uuid.AsString())} if einheiten else {})))
    return zones
