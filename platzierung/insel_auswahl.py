""
from analyse import regelkennzahlen as RK


def waehle(bezug, kandidat, *, gueltig_bezug, gueltig_kandidat):
    out = dict(kandidat=False, grund='Bezug behalten')
    if not isinstance(bezug, dict) or not isinstance(kandidat, dict):
        return dict(out, grund='Ergebnis fehlt')
    if type(gueltig_bezug) is not bool or type(gueltig_kandidat) is not bool:
        return dict(out, grund='Gültigkeit unbekannt')
    if not gueltig_kandidat:
        return dict(out, grund='Kandidat ungültig')
    a, b = bezug.get('regeln'), kandidat.get('regeln')
    if not isinstance(b, dict) or (gueltig_bezug and not isinstance(a, dict)):
        return dict(out, grund='Regelmessung fehlt')
    try:

        own = RK.annahme(b, b)
        if not own['nicht_schlechter']:
            return dict(out, grund=own['grund'])
        if not gueltig_bezug:
            return dict(kandidat=True, grund='Gültigkeitsgewinn')
        tor = RK.annahme(a, b)
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        return dict(out, grund='Regelvertrag: '+str(exc))
    return dict(kandidat=tor['besser'], grund=tor['grund'], tor=tor['vergleich'])
