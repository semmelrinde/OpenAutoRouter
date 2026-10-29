import math
from shapely import STRtree
from shapely.geometry import Point, Polygon
from common.drills import pad_drill_hole
from analysis.copper_model import via_geo, via_ground, unconnected_polygons
from learning import current_zones as SZ


def verify(B, R, K, gnd_layers=None, design_rules=None):
    rg = design_rules if design_rules is not None else K.get('design_rules') or {}
    sources = [R, R.get('options') or {}, rg.get('router',rg), rg.get('raw_data') or {}, B.get('design') or {}]
    def minimum(default, *keys):
        values = [float(s[k]) for s in sources for k in keys if s.get(k) is not None

                  and not (s is sources[0] and k=='drc' and isinstance(s[k],dict))]
        if any(not math.isfinite(v) or v < 0 for v in values):
            raise ValueError('Invalid manufacturing rule: '+str(keys))
        return max(values) if values else default
    limits = dict(via_mm=minimum(.4,'min_via_diameter','via_min'),
        drill_mm=minimum(.2,'min_through_hole_diameter','bore_min'),
        restring_mm=minimum(.05,'min_via_annular_width','restring_min','ring_min'),
        h2h_mm=minimum(.2,'min_hole_to_hole','h2h','hole_hole'))
    errors = []
    raw_via = (R.get('options') or {}).get('via') or rg.get('router',rg).get('via')
    if raw_via:
        try:
            d,b = [float(v) for v in str(raw_via).split('/')]
            if not all(math.isfinite(q) and q > 0 for q in (d,b)) or b > d: raise ValueError('Via')
        except (ValueError,TypeError): errors.append(dict(category='via_standard_geometry',amount=raw_via))


    kk = dict(K, design_rules=rg)
    vd, drill = via_geo(R,kk)
    R = dict(R, options=dict(R.get('options') or {}, via='%s/%s'%(vd,drill)))

    for i,v in enumerate(R.get('vias', [])):
        if 'd' in v and 'diameter' in v:
            if not math.isclose(float(v['d']),float(v['diameter']),rel_tol=0,abs_tol=1e-9):
                errors.append(dict(category='via_alias_contradiction',via=i,d=v['d'],diameter=v['diameter']))
    from common.zones import via_geometry, board_geometry
    from analysis.copper_model import settings
    R = via_geometry(R)
    B = board_geometry(B,R)
    clr, border = settings(kk, design_rules=rg)
    border = max(border,minimum(0,'min_copper_edge_clearance','border','edge'))

    router_clr = minimum(0.09,'clr','drc','min_clearance')
    R = dict(R, clr=router_clr)
    native = []
    for z in R.get('zones', []):
        if not z.get('filled'):
            errors.append(dict(category='zone_unfilled',net_name=z.get('net'),layer_name=z.get('layer')))
            continue
        geo = Polygon(z['pts'],z.get('holes') or [])
        if not geo.is_valid or geo.is_empty:
            errors.append(dict(category='zone_geometry',net_name=z.get('net'),layer_name=z.get('layer')))
            continue
        native.append(dict(z, polys=[dict(outer=z['pts'],inner=z.get('holes') or [])]))


    check = dict(R, zone_list=list(R.get('zone_list') or [])+native, zones=[])
    errors += [dict(net_name=n,layer_name=l,category='zone_'+a,amount=w) for n,l,a,w in SZ.verify(B,check,kk,gnd_layers,clr=clr,border=border)]
    holes = []
    for i,v in enumerate(R.get('vias',[])):
        d,b = via_ground(v,vd,drill)
        location = [v['x'],v['y']]
        if not all(math.isfinite(q) and q > 0 for q in (d,b)) or b > d:
            errors.append(dict(category='via_geometry',via=i,location=location)); continue
        for category,value,limit in [('via_diameter',d,limits['via_mm']),('via_drill_hole',b,limits['drill_mm']),
                                ('via_restring',(d-b)/2,limits['restring_mm'])]:
            if value < limit-1e-9:
                errors.append(dict(category=category,via=i,location=location,value_mm=value,minimum_mm=limit))
        for threshold, diameter in (rg.get('raw_data') or {}).get('via_min_by_drill',[]):
            if b < float(threshold) and d < float(diameter)-1e-9:
                errors.append(dict(category='via_diameter_drill_rule',via=i,location=location,value_mm=d,minimum_mm=diameter))
        holes.append(dict(id='via:%d'%i,x=v['x'],y=v['y'],drill=b,circle=True,geo=Point(v['x'],v['y']).buffer(b/2,64)))
    for p in B['pads']:
        try:
            hole = pad_drill_hole(p)
        except (ValueError, TypeError, IndexError) as exc:
            errors.append(dict(category='pad_drill_hole_geometry',location=[p['x'],p['y']],reason=str(exc)))
            continue
        if hole is None: continue
        if hole['drill'] < limits['drill_mm']-1e-9:
            errors.append(dict(category='pad_drill_hole_min',pad='%s:%s'%(p['ref'],p['num']),
                               location=[hole['x'],hole['y']],value_mm=hole['drill'],minimum_mm=limits['drill_mm']))
        holes.append(dict(hole,id='%s:%s'%(p['ref'],p['num'])))
    shapes = [q['geo'] for q in holes]
    tree = STRtree(shapes)
    for i,q in enumerate(holes):


        for j in sorted(tree.query(shapes[i].buffer(limits['h2h_mm']+.0011).envelope)):
            if j <= i: continue
            other = holes[j]

            if q['circle'] and other['circle']:
                distance = math.hypot(q['x']-other['x'],q['y']-other['y'])-(q['drill']+other['drill'])/2
            else:
                distance = shapes[i].distance(shapes[j])
            if distance < limits['h2h_mm']-1e-9:
                errors.append(dict(category='drill_spacing',pair=[q['id'],other['id']],location=[q['x'],q['y']],
                    other_location=[other['x'],other['y']],value_mm=distance,minimum_mm=limits['h2h_mm']))
    edited = {z['net'] for key in ('zone_list','zones') for z in R.get(key,[])
              if z.get('source') in ('current_zones','gnd_fill') or z.get('r36')}
    edited.update(z['net'] for z in native)
    edited.update(v['net'] for v in R.get('vias',[]) if v.get('source') == 'gnd_fill')


    ignored = {p['net'] for p in B['pads'] if p['net'] not in edited}
    missing = unconnected_polygons(B,R,ignored) if edited else []
    errors += [dict(category='not_connected',net_name=n) for n in missing]

    router_valid = (isinstance(R.get('conflicts'),(dict,list)) and not R['conflicts']
                    and isinstance(R.get('failed'),list) and not R['failed'])
    return dict(passed=router_valid and not errors, router_valid=router_valid, faults=errors,
                limits=limits, zones_limits=dict(zclr_mm=clr,zone_edge_mm=border), connection_checked_nets=sorted(edited),
                hint='Polygon check with native hole contours, otherwise circle export; native DRC separately.')
