""
def pruefen(board):
    import pcbnew
    poly = pcbnew.SHAPE_POLY_SET()
    board.GetBoardPolygonOutlines(poly, True)
    holes = sum(poly.HoleCount(i) for i in range(poly.OutlineCount()))
    shapes = [d for d in board.GetDrawings() if d.GetLayer() == pcbnew.Edge_Cuts]
    fp_edges = [d for fp in board.GetFootprints() for d in fp.GraphicalItems()
                if d.GetLayer() == pcbnew.Edge_Cuts]
    allowed = (pcbnew.SHAPE_T_SEGMENT, pcbnew.SHAPE_T_RECT, pcbnew.SHAPE_T_POLY)
    supported = all(isinstance(d, pcbnew.PCB_SHAPE) and d.GetShape() in allowed for d in shapes)
    return dict(konturen=poly.OutlineCount(), ausschnitte=holes, formen_unterstuetzt=supported,
                footprint_kanten=len(fp_edges),
                eng_erlaubt=poly.OutlineCount() == 1 and holes == 0 and supported and not fp_edges)

if __name__ == '__main__':
    import sys,json,wx,pcbnew
    app = wx.GetApp() or wx.App(False)
    _log_ziel = wx.LogStderr(); wx.Log.SetActiveTarget(_log_ziel)
    with open(sys.argv[2], 'w') as f: json.dump(pruefen(pcbnew.LoadBoard(sys.argv[1])), f)
