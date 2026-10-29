

def notches_fill(g, vacant, radius=.45):
    from shapely.ops import unary_union
    if g.is_empty or radius <= 0:
        return g
    closed = g.buffer(radius, 32).buffer(-radius, 32)
    delta = closed.difference(g).intersection(vacant)
    if delta.is_empty:
        return g


    core_set = g.union(delta).buffer(-radius/3, 32).buffer(radius/3, 32)
    delta = core_set.intersection(delta)
    out = g.union(delta).buffer(0)
    return unary_union([p for p in getattr(out, 'geoms', [out])
                        if p.geom_type == 'Polygon' and p.intersects(g)])

def assumption(a, b, shape_a, shape_b):
    import math
    from analysis.rule_metrics import gate_fields
    missing, worse = [], []
    if not a.get('is_valid') or not b.get('is_valid'):
        missing.append('both finishes hard valid')
    if not a.get('score_v3_version') or a.get('score_v3_version') != b.get('score_v3_version'):
        missing.append('same v3 version')
    ra, rb = a.get('design_rules') or {}, b.get('design_rules') or {}
    fields = sorted(set(gate_fields(ra)) | set(gate_fields(rb)))

    def number(x):
        return isinstance(x, (int,float)) and not isinstance(x, bool) and math.isfinite(x) and x >= 0

    for name in fields:
        x,y = ra.get(name),rb.get(name)
        if not number(x) or not number(y):
            missing.append(name)
        elif y > x+1e-8:
            worse.append(name)
    if (shape_a.get('version') != 'k17-free-notches-v2' or
            shape_b.get('version') != shape_a.get('version') or
            not all(f.get('complete') is True and f.get('status')=='measured' for f in (shape_a, shape_b))):
        missing.append('complete K17 measurement of the same version')
    for key in ('free_notches_mm2','narrow_extra_area_mm2'):
        x,y = shape_a.get(key),shape_b.get(key)
        if not number(x) or not number(y):
            missing.append(key)
        elif y > x+1e-6:
            worse.append(key)
    mm = None
    if a.get('minimax_version') or b.get('minimax_version'):
        from analysis.minimax import assumption as mm_assumption
        mm = mm_assumption(a,b)
        if not mm.get('not_worse'):
            worse.append('minimax')
    return dict(passed=not missing and not worse, missing=missing, worse=worse,
                design_rules_before={k:ra.get(k) for k in fields}, design_rules_after={k:rb.get(k) for k in fields},
                shape_before=shape_a, shape_after=shape_b, minimax=mm)
