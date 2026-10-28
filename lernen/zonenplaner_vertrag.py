""
import math

DEFAULTS = dict(lagen=4, clr=.13, drc=.127, edge=.30, h2h=.25, via='0.45/0.20',
                loch_clr=0, vip=0, bahn_min=0, bga_via_min=0, bga_via_exakt=0,
                mask_grafik=0, sperren=1, sperren_vias=1, sigmin=0)


def geometrie(regeln, overrides=None):
    raw = dict((regeln or {}).get('router') or {}, **(overrides or {}))
    values = dict(DEFAULTS, **{k:v for k,v in raw.items() if k in DEFAULTS})
    values['drcpad'] = raw.get('drcpad', values['drc'])
    for k,v in values.items():
        values[k] = tuple(map(float,str(v).split('/'))) if k=='via' else float(v)
        if not all(math.isfinite(x) and x>=0 for x in (values[k] if k=='via' else (values[k],))):
            raise ValueError('Ungueltige Geometrie: '+k)
    values['via'] = list(values['via'])
    return values
