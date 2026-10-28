""
import math
from shapely import STRtree
from shapely.geometry import Point, Polygon
from gemeinsam.bohrungen import pad_bohrung
from analyse.kupfermodell import via_geo, via_masse, unverbunden_polygone
from lernen import stromzonen as SZ


def pruefen(B, R, K, gndlagen=None, regeln=None):
    rg = regeln if regeln is not None else K.get('regeln') or {}
    sources = [R, R.get('optionen') or {}, rg.get('router',rg), rg.get('roh') or {}, B.get('design') or {}]
    def minimum(default, *keys):
        values = [float(s[k]) for s in sources for k in keys if s.get(k) is not None

                  and not (s is sources[0] and k=='drc' and isinstance(s[k],dict))]
        if any(not math.isfinite(v) or v < 0 for v in values):
            raise ValueError('Ungültige Fertigungsregel: '+str(keys))
        return max(values) if values else default
    limits = dict(via_mm=minimum(.4,'min_via_diameter','via_min'),
        drill_mm=minimum(.2,'min_through_hole_diameter','bohr_min'),
        restring_mm=minimum(.05,'min_via_annular_width','restring_min','ring_min'),
        h2h_mm=minimum(.2,'min_hole_to_hole','h2h','loch_loch'))
    errors = []
    raw_via = (R.get('optionen') or {}).get('via') or rg.get('router',rg).get('via')
    if raw_via:
        try:
            d,b = [float(v) for v in str(raw_via).split('/')]
            if not all(math.isfinite(q) and q > 0 for q in (d,b)) or b > d: raise ValueError('Via')
        except (ValueError,TypeError): errors.append(dict(art='via_standard_geometrie',wert=raw_via))


    kk = dict(K, regeln=rg)
    vd, drill = via_geo(R,kk)
    R = dict(R, optionen=dict(R.get('optionen') or {}, via='%s/%s'%(vd,drill)))

    for i,v in enumerate(R.get('vias', [])):
        if 'd' in v and 'diameter' in v:
            if not math.isclose(float(v['d']),float(v['diameter']),rel_tol=0,abs_tol=1e-9):
                errors.append(dict(art='via_alias_widerspruch',via=i,d=v['d'],diameter=v['diameter']))
    from gemeinsam.zonen import viageometrie, boardgeometrie
    from analyse.kupfermodell import einstellungen
    R = viageometrie(R)
    B = boardgeometrie(B,R)
    clr, rand = einstellungen(kk, regeln=rg)
    rand = max(rand,minimum(0,'min_copper_edge_clearance','rand','edge'))

    router_clr = minimum(0.09,'clr','drc','min_clearance')
    R = dict(R, clr=router_clr)
    native = []
    for z in R.get('zones', []):
        if not z.get('filled'):
            errors.append(dict(art='zone_ungefuellt',netz=z.get('net'),lage=z.get('layer')))
            continue
        geo = Polygon(z['pts'],z.get('holes') or [])
        if not geo.is_valid or geo.is_empty:
            errors.append(dict(art='zone_geometrie',netz=z.get('net'),lage=z.get('layer')))
            continue
        native.append(dict(z, polys=[dict(aussen=z['pts'],innen=z.get('holes') or [])]))


    check = dict(R, zonen=list(R.get('zonen') or [])+native, zones=[])
    errors += [dict(netz=n,lage=l,art='zone_'+a,wert=w) for n,l,a,w in SZ.pruefen(B,check,kk,gndlagen,clr=clr,rand=rand)]
    holes = []
    for i,v in enumerate(R.get('vias',[])):
        d,b = via_masse(v,vd,drill)
        ort = [v['x'],v['y']]
        if not all(math.isfinite(q) and q > 0 for q in (d,b)) or b > d:
            errors.append(dict(art='via_geometrie',via=i,ort=ort)); continue
        for art,value,limit in [('via_durchmesser',d,limits['via_mm']),('via_bohrung',b,limits['drill_mm']),
                                ('via_restring',(d-b)/2,limits['restring_mm'])]:
            if value < limit-1e-9:
                errors.append(dict(art=art,via=i,ort=ort,wert_mm=value,minimum_mm=limit))
        for threshold, diameter in (rg.get('roh') or {}).get('via_min_by_drill',[]):
            if b < float(threshold) and d < float(diameter)-1e-9:
                errors.append(dict(art='via_durchmesser_bohrregel',via=i,ort=ort,wert_mm=d,minimum_mm=diameter))
        holes.append(dict(id='via:%d'%i,x=v['x'],y=v['y'],drill=b,kreis=True,geo=Point(v['x'],v['y']).buffer(b/2,64)))
    for p in B['pads']:
        try:
            hole = pad_bohrung(p)
        except (ValueError, TypeError, IndexError) as exc:
            errors.append(dict(art='pad_bohrung_geometrie',ort=[p['x'],p['y']],grund=str(exc)))
            continue
        if hole is None: continue
        if hole['drill'] < limits['drill_mm']-1e-9:
            errors.append(dict(art='pad_bohrung_min',pad='%s:%s'%(p['ref'],p['num']),
                               ort=[hole['x'],hole['y']],wert_mm=hole['drill'],minimum_mm=limits['drill_mm']))
        holes.append(dict(hole,id='%s:%s'%(p['ref'],p['num'])))
    shapes = [q['geo'] for q in holes]
    tree = STRtree(shapes)
    for i,q in enumerate(holes):


        for j in sorted(tree.query(shapes[i].buffer(limits['h2h_mm']+.0011).envelope)):
            if j <= i: continue
            other = holes[j]

            if q['kreis'] and other['kreis']:
                distance = math.hypot(q['x']-other['x'],q['y']-other['y'])-(q['drill']+other['drill'])/2
            else:
                distance = shapes[i].distance(shapes[j])
            if distance < limits['h2h_mm']-1e-9:
                errors.append(dict(art='bohrungsabstand',paar=[q['id'],other['id']],ort=[q['x'],q['y']],
                    anderer_ort=[other['x'],other['y']],wert_mm=distance,minimum_mm=limits['h2h_mm']))
    edited = {z['net'] for key in ('zonen','zones') for z in R.get(key,[])
              if z.get('quelle') in ('stromzonen','gnd_fuellung') or z.get('r36')}
    edited.update(z['net'] for z in native)
    edited.update(v['net'] for v in R.get('vias',[]) if v.get('quelle') == 'gnd_fuellung')


    ignored = {p['net'] for p in B['pads'] if p['net'] not in edited}
    missing = unverbunden_polygone(B,R,ignored) if edited else []
    errors += [dict(art='unverbunden',netz=n) for n in missing]

    router_valid = (isinstance(R.get('conflicts'),(dict,list)) and not R['conflicts']
                    and isinstance(R.get('failed'),list) and not R['failed'])
    return dict(bestanden=router_valid and not errors, router_gueltig=router_valid, fehler=errors,
                grenzen=limits, zonen_grenzen=dict(zclr_mm=clr,zone_edge_mm=rand), verbindung_gepruefte_netze=sorted(edited),
                hinweis='Polygonprüfung mit nativen Lochkonturen, sonst Kreisexport; native DRC separat.')
