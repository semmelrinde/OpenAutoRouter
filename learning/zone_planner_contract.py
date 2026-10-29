import math

DEFAULTS = dict(cu_layers=4, clr=.13, drc=.127, edge=.30, h2h=.25, via='0.45/0.20',
                hole_clr=0, vip=0, track_min=0, bga_via_min=0, bga_via_exact=0,
                mask_graphic=0, keepout_list=1, keepouts_vias=1, sigmin=0)


def geometry(design_rules, overrides=None):
    raw = dict((design_rules or {}).get('router') or {}, **(overrides or {}))
    values = dict(DEFAULTS, **{k:v for k,v in raw.items() if k in DEFAULTS})
    values['drcpad'] = raw.get('drcpad', values['drc'])
    for k,v in values.items():
        values[k] = tuple(map(float,str(v).split('/'))) if k=='via' else float(v)
        if not all(math.isfinite(x) and x>=0 for x in (values[k] if k=='via' else (values[k],))):
            raise ValueError('Invalid geometry: '+k)
    values['via'] = list(values['via'])
    return values
