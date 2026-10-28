""
import json
import math
import sys
import pcbnew
import wx
app=wx.GetApp() or wx.App(False)
_log_ziel = wx.LogStderr(); wx.Log.SetActiveTarget(_log_ziel)
b=pcbnew.LoadBoard(sys.argv[1])
cu=list(b.GetEnabledLayers().CuStack())
names={L:pcbnew.BOARD.GetStandardLayerName(L).replace('.Cu','') for L in cu}
def pos(v):return [pcbnew.ToMM(v.x),pcbnew.ToMM(v.y)]
def arc_points(a,m,z):
    ax,ay=a;bx,by=m;cx,cy=z
    den=2*(ax*(by-cy)+bx*(cy-ay)+cx*(ay-by))
    if abs(den)<1e-14:
        raise ValueError('Entarteter nativer Bogen')
    aa=ax*ax+ay*ay;bb=bx*bx+by*by;cc=cx*cx+cy*cy
    ux=(aa*(by-cy)+bb*(cy-ay)+cc*(ay-by))/den
    uy=(aa*(cx-bx)+bb*(ax-cx)+cc*(bx-ax))/den
    radius=math.hypot(ax-ux,ay-uy)
    t0=math.atan2(ay-uy,ax-ux);tm=math.atan2(by-uy,bx-ux);t1=math.atan2(cy-uy,cx-ux)
    sweep=(t1-t0)%(2*math.pi)
    if (tm-t0)%(2*math.pi)>sweep:sweep-=2*math.pi

    step=min(.03/max(radius,1e-9),2*math.acos(max(-1.,1-.001/max(radius,.001))))
    count=max(2,math.ceil(abs(sweep)/step))
    return [a]+[[ux+radius*math.cos(t0+sweep*i/count),uy+radius*math.sin(t0+sweep*i/count)] for i in range(1,count)]+[z]
R=dict(tracks=[],vias=[],drawings=[],conflicts={},failed=[],native_vias=True,quelle='nativ:'+sys.argv[1],optionen={'lagen':len(cu)})
for drawing in b.GetDrawings():
    L=drawing.GetLayer()
    if L not in names or drawing.GetClass()!='PCB_SHAPE':continue
    poly=pcbnew.SHAPE_POLY_SET()
    drawing.TransformShapeToPolygon(poly,L,0,pcbnew.FromMM(.001),pcbnew.ERROR_OUTSIDE)
    shapes=[]
    for i in range(poly.OutlineCount()):
        ring=poly.Outline(i)
        outer=[pos(ring.CPoint(j)) for j in range(ring.PointCount())]
        holes=[]
        for h in range(poly.HoleCount(i)):
            hole=poly.Hole(i,h);holes.append([pos(hole.CPoint(j)) for j in range(hole.PointCount())])
        shapes.append([outer,holes])
    R['drawings'].append(dict(net=drawing.GetNetname(),layer=names[L],polys=shapes))
for t in b.GetTracks():
    if t.GetClass()=='PCB_VIA':
        if not all(t.IsOnLayer(L) for L in cu):
            raise ValueError('Blind/Buried/Micro-Via: nativer Rundlauf nicht unterstuetzt')
        x,y=pos(t.GetPosition())
        R['vias'].append(dict(net=t.GetNetname(),x=x,y=y,d=pcbnew.ToMM(t.GetWidth(pcbnew.F_Cu)),drill=pcbnew.ToMM(t.GetDrillValue()),layers=[names[L] for L in cu if t.IsOnLayer(L)]));continue
    a,z=pos(t.GetStart()),pos(t.GetEnd());w=pcbnew.ToMM(t.GetWidth())
    if t.GetClass()=='PCB_ARC':
        m=pos(t.GetMid());pts=arc_points(a,m,z);items=[['A',a,m,z,w]]
    else:pts=[a,z];items=[['L',a,z,w]]
    R['tracks'].append(dict(net=t.GetNetname(),layer=names[t.GetLayer()],pts=pts,w=[w]*(len(pts)-1),items=items))
Z=[]
for zone in b.Zones():
    if zone.GetIsRuleArea():continue
    poly=zone.Outline()
    for L in cu:
        if not zone.IsOnLayer(L):continue
        for i in range(poly.OutlineCount()):
            ring=poly.Outline(i)
            pts=[pos(ring.CPoint(j)) for j in range(ring.PointCount())]
            holes=[]
            for h in range(poly.HoleCount(i)):
                hh=poly.Hole(i,h);holes.append([pos(hh.CPoint(j)) for j in range(hh.PointCount())])
            clr=zone.GetLocalClearance()
            Z.append(dict(net=zone.GetNetname(),layer=names[L],pts=pts,holes=holes,prio=zone.GetAssignedPriority(),full=zone.GetPadConnection()==pcbnew.ZONE_CONNECTION_FULL,
                          local_clearance=None if clr is None else pcbnew.ToMM(clr),
                          min_thickness=pcbnew.ToMM(zone.GetMinThickness()),pad_connection=int(zone.GetPadConnection()),
                          island_removal=int(zone.GetIslandRemovalMode()),thermal_gap=pcbnew.ToMM(zone.GetThermalReliefGap()),
                          thermal_spoke=pcbnew.ToMM(zone.GetThermalReliefSpokeWidth())))
for f,data in ((sys.argv[2],R),(sys.argv[3],Z)):
    with open(f,'w') as out:json.dump(data,out,sort_keys=True)
print('R33 Export:',len(R['tracks']),'Bahnen,',len(R['vias']),'Vias,',len(Z),'Zonen')
