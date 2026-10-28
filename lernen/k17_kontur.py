""


def kerben_fuellen(g, frei, radius=.45):
    from shapely.ops import unary_union
    if g.is_empty or radius <= 0:
        return g
    geschlossen = g.buffer(radius, 32).buffer(-radius, 32)
    delta = geschlossen.difference(g).intersection(frei)
    if delta.is_empty:
        return g


    kern = g.union(delta).buffer(-radius/3, 32).buffer(radius/3, 32)
    delta = kern.intersection(delta)
    out = g.union(delta).buffer(0)
    return unary_union([p for p in getattr(out, 'geoms', [out])
                        if p.geom_type == 'Polygon' and p.intersects(g)])

def annahme(a, b, form_a, form_b):
    ""
    import math
    from analyse.regelkennzahlen import torfelder
    missing, worse = [], []
    if not a.get('gueltig') or not b.get('gueltig'):
        missing.append('beide Abschluesse hart gueltig')
    if not a.get('note_v3_version') or a.get('note_v3_version') != b.get('note_v3_version'):
        missing.append('gleiche v3-Version')
    ra, rb = a.get('regeln') or {}, b.get('regeln') or {}
    fields = sorted(set(torfelder(ra)) | set(torfelder(rb)))

    def number(x):
        return isinstance(x, (int,float)) and not isinstance(x, bool) and math.isfinite(x) and x >= 0

    for name in fields:
        x,y = ra.get(name),rb.get(name)
        if not number(x) or not number(y):
            missing.append(name)
        elif y > x+1e-8:
            worse.append(name)
    if (form_a.get('version') != 'k17-freie-kerben-v2' or
            form_b.get('version') != form_a.get('version') or
            not all(f.get('vollstaendig') is True and f.get('status')=='gemessen' for f in (form_a, form_b))):
        missing.append('vollstaendige K17-Messung gleicher Version')
    for key in ('freie_kerben_mm2','schmale_zusatzflaeche_mm2'):
        x,y = form_a.get(key),form_b.get(key)
        if not number(x) or not number(y):
            missing.append(key)
        elif y > x+1e-6:
            worse.append(key)
    mm = None
    if a.get('minimax_version') or b.get('minimax_version'):
        from analyse.minimax import annahme as mm_annahme
        mm = mm_annahme(a,b)
        if not mm.get('nicht_schlechter'):
            worse.append('minimax')
    return dict(bestanden=not missing and not worse, fehlt=missing, schlechter=worse,
                regeln_vorher={k:ra.get(k) for k in fields}, regeln_nachher={k:rb.get(k) for k in fields},
                form_vorher=form_a, form_nachher=form_b, minimax=mm)
