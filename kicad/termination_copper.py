

def drawings(board, names, unit_list=False):
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
        net = drawing.GetNetname() if hasattr(drawing, 'GetNetName') else ''
        for j in range(polygons.OutlineCount()):
            zones.append(dict(net=net or '#CU_%d' % i, layer=names[layer], filled=True,
                              pts=xy(polygons.Outline(j)),
                              holes=[xy(polygons.Hole(j, h)) for h in range(polygons.HoleCount(j))],
                              board_copper=True,
                              **({'unit': str(drawing.m_Uuid.AsString())} if unit_list else {})))
    return zones
