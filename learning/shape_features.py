import math
import re

VERSION = 'reviewer-shaping-v3'
ZONES = ('wedges_45', 'wedges_80', 'notches', 'sharp_corners', 'side_strips',
         'own_net_slots', 'via_bites', 'z2_outer_pct', 'z4_splitter',
         'z2_without_pad_pct', 'z4_per_net_layer', 'pieces')
FIELDS = ZONES + ('zones_present', 'chain_eta', 'chain_eta_median', 'chain_element_max',
                   'r39_unnecessary_mm2', 'width_change', 'r18_bend_excess',
                   'r18_signal_detour_mm', 'k20_without_zone_share',
                   'k20_pad_groups_without_zone', 'k20_supply_outer_share',
                   'r40_sharp_vias_mm2', 'r40_window_over',
                   'k17_free_notches_mm2', 'k17_narrow_extra_area_mm2')


def count(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and v >= 0


def failed(data):
    return (not isinstance(data, dict) or 'faults' in data or
            data.get('complete') is False or bool(data.get('not_measurable')) or
            isinstance(data.get('coverage'), dict) and data['coverage'].get('complete') is False)


def group_error(source_list, *group_list):
    for src in source_list:
        if not isinstance(src, dict): continue
        if (src.get('complete') is False or src.get('not_measurable') or
                isinstance(src.get('coverage'), dict) and src['coverage'].get('complete') is False):
            return True
        for key in ('faults', 'diagnose_error'):
            if key in src:
                if not isinstance(src[key], dict): return True
                if any(g in src[key] for g in group_list): return True
        for g in group_list:
            if g+'_error' in src: return True
            obj = src.get(g)
            if isinstance(obj, dict) and ('faults' in obj or group_error([obj])): return True
    return False


def chain_valid(data):
    return (isinstance(data, dict) and 'faults' not in data and
            not group_error([data]) and isinstance(data.get('skipped'), dict)
            and not data['skipped'])


def r39_ok(source_list):
    if group_error(source_list, 'r39', 'current_bottlenecks'): return False
    coverage = []
    for src in source_list:
        if not isinstance(src, dict): continue
        if 'r39_coverage' in src: coverage.append(src['r39_coverage'])
        tight = src.get('current_bottlenecks')
        if isinstance(tight, dict) and 'coverage' in tight: coverage.append(tight['coverage'])
    return bool(coverage) and all(isinstance(c, dict) and 'faults' not in c and
                                c.get('complete') is True and count(c.get('components'))
                                and c['components'] > 0 for c in coverage)


def value_list(z):
    m = z.get('shape_measurement')
    if not isinstance(m, dict) or m.get('version') != VERSION or m.get('complete') is not True or 'faults' in m:
        return {}
    if any(not isinstance(m.get(k), str) or not re.fullmatch('[0-9a-f]{64}', m[k])
           for k in ('geometry_sha256', 'evaluator_sha256')):
        return {}
    if z.get('geometry_sha256') is not None and z['geometry_sha256'] != m['geometry_sha256']:
        return {}
    v, coverage = m.get('value_list'), m.get('coverage')
    if not isinstance(v, dict) or not isinstance(coverage, dict):
        return {}
    chain=m.get('chain_coverage') or {}
    chain_ok=chain.get('complete') is True and chain_valid(chain)
    return {k: float(v[k]) for k in FIELDS if coverage.get(k) is True and count(v.get(k))
            and (not k.startswith('chain_') or chain_ok)}


def off_measurement(w, r, geometry_sha256, evaluator_sha256):
    v = {}
    r_ok = not group_error([r])
    w_ok = not failed(w)
    if w_ok and count(w.get('pieces')):
        v.update(pieces=w['pieces'], zones_present=int(w['pieces'] > 0))
        if w['pieces'] > 0:
            for k in ZONES:
                if k not in ('side_strips', 'own_net_slots', 'via_bites') or not failed(w.get('geometry') or {}):
                    if count(w.get(k)): v[k] = w[k]
    chain_coverage = w.get('chain_coverage') or {}
    if w_ok and chain_coverage.get('complete') is True and chain_valid(chain_coverage):
        for k in ('chain_eta', 'chain_eta_median', 'chain_element_max'):
            if count(w.get(k)): v[k] = w[k]
    tight = r.get('current_bottlenecks') or {}
    if r39_ok([r]):
        if count(tight.get('unnecessary_mm2')): v['r39_unnecessary_mm2'] = tight['unnecessary_mm2']
    if not group_error([r], 'metrics', 'width_change') and count(r.get('width_change')): v['width_change'] = r['width_change']
    r18 = (r.get('r18') or {}) if not group_error([r], 'r18') else {}
    for key, group, field in [('r18_bend_excess', 'all_items', 'bend_excess'),
                              ('r18_signal_detour_mm', 'signal', 'detour_octi_mm')]:
        data = r18.get(group) or {}
        x = data.get(field) if not failed(data) else None
        if count(x): v[key] = x
    k20 = (r.get('k20') or {}) if not group_error([r], 'k20') else {}
    for key, field in [('k20_without_zone_share', 'without_zone_share'),
                       ('k20_pad_groups_without_zone', 'pad_groups_without_common_zone'),
                       ('k20_supply_outer_share', 'supply_outer_share')]:
        if count(k20.get(field)): v[key] = k20[field]
    for key, field in [('r40_sharp_vias_mm2', 'via_density_sharp'), ('r40_window_over', 'via_density_window_over')]:
        if not group_error([r], 'via_density', 'r40') and count(r.get(field)): v[key] = r[field]
    k17 = r.get('k17') or {}
    if (not group_error([r], 'k17') and k17.get('version') == 'k17-free-notches-v2'
            and k17.get('status') == 'measured' and k17.get('complete') is True):
        for field in ('free_notches_mm2', 'narrow_extra_area_mm2'):
            if count(k17.get(field)): v['k17_'+field] = k17[field]
    return dict(version=VERSION, geometry_sha256=geometry_sha256, evaluator_sha256=evaluator_sha256,
                complete=w_ok and r_ok,
                chain_coverage=dict(chain_coverage),
                value_list=v, coverage={k: k in v for k in FIELDS
                                    if not k.startswith('k17_') or k in v})
