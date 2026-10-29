import math


def active(value):
    if value in (True, 1, '1'): return True
    if value in (False, 0, '0', None): return False
    raise ValueError('placer_loosen=0|1')


def request(z):
    if not active((z.get('cfg') or {}).get('placer_loosen')): return False
    if z.get('cur_state') == 'placer_time_limit': return True
    k = (z.get('placer_metrics') or {}).get('collisions')
    return z.get('cur_state') == 'placer_collision' and isinstance(k, (int, float)) and not isinstance(k, bool) and math.isfinite(k) and k > 0


def label(A):
    area, courtyard = A.get('area_mm2'), A.get('courtyard_mm2')
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and v > 0 for v in (area, courtyard)):
        raise ValueError('Relaxation needs measured task areas')
    return dict(relaxed=int(A.get('relaxed', 0)), density=courtyard / area,
                density_definition='Generator-CourtyardArea/OutlineArea',
                relaxation_levels=A.get('relaxation_levels', 0),
                factor_origin=A.get('factor_origin', A['factor']))


def next_factor(A, lines):
    if A.get('relaxation_levels', 0) >= 2: return None
    factor = round(A['factor'] * 1.12, 3)
    if any(z.get('is_valid') is True and z.get('factor', math.inf) <= factor for z in lines): return None
    return factor


def accept(before, after):
    for key in ('seed', 'cu_layers', 'part_list', 'pads', 'net_list', 'blocks', 'courtyard_mm2'):
        if before[key] != after[key]: raise ValueError('Relaxation changes task build plan: ' + key)
    if after['area_mm2'] <= before['area_mm2']: raise ValueError('Relaxation does not enlarge outline')
    after.update(relaxed=1, relaxation_levels=before.get('relaxation_levels', 0) + 1,
                   factor_origin=before.get('factor_origin', before['factor']))
    return after
