import pcbnew, json, sys
b=pcbnew.LoadBoard(sys.argv[1]); out=[]
def pl(ps):
    r=[]
    for i in range(ps.OutlineCount()):
        o=ps.Outline(i); r.append([(o.CPoint(j).x/1e6,o.CPoint(j).y/1e6) for j in range(o.PointCount())])
    return r
for fp in b.GetFootprints():
    for p in fp.Pads():
        for s,cu,mk in (('F',pcbnew.F_Cu,pcbnew.F_Mask),('B',pcbnew.B_Cu,pcbnew.B_Mask)):
            e=dict(side=s,ref=fp.GetReference(),pad=p.GetNumber(),net=p.GetNetname(),fpn=str(fp.GetFPID().GetLibItemName()),val=fp.GetValue(),th=p.GetAttribute()==pcbnew.PAD_ATTRIB_PTH)
            if p.IsOnLayer(cu):
                ps=pcbnew.SHAPE_POLY_SET(); p.TransformShapeToPolygon(ps,cu,0,1000,pcbnew.ERROR_OUTSIDE); e['cu']=pl(ps)
            if p.IsOnLayer(mk):
                ps=pcbnew.SHAPE_POLY_SET(); p.TransformShapeToPolygon(ps,cu,p.GetSolderMaskExpansion(mk),1000,pcbnew.ERROR_OUTSIDE); e['mk']=pl(ps)
            if 'cu' in e or 'mk' in e: out.append(e)
json.dump(out,open(sys.argv[2],'w'))
