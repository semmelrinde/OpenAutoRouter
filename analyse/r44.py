""


def messen(B, R):
    import math
    from gemeinsam.zonen import boardgeometrie
    from nachbearbeitung.kurzschluss_schutz import ausgabe_kontakte, bahn_vorschau
    try:
        for t in R['tracks']:
            q, _ = bahn_vorschau(t)
            if len(q['pts']) < 2 or len(q['w']) != len(q['pts']) - 1:
                raise ValueError('R44: unvollstaendige Bahngeometrie')
            if not all(len(p) == 2 and all(math.isfinite(v) for v in p) for p in q['pts']):
                raise ValueError('R44: ungueltige Bahnpunkte')
            if not all(math.isfinite(w) and w > 0 for w in q['w']):
                raise ValueError('R44: ungueltige Bahnbreite')
        default_d = float(str((R.get('optionen') or {}).get('via', '.45/.20')).split('/')[0])
        for v in R['vias']:
            d = float(v.get('d', v.get('diameter', default_d)))
            if not all(math.isfinite(v[k]) for k in ('x','y')) or not math.isfinite(d) or d <= 0:
                raise ValueError('R44: ungueltige Viageometrie')
        kontakte, geerbt = ausgabe_kontakte(boardgeometrie(B, R), R)
        def paare(rows):
            return {(q['lage'], *sorted(q['netze'])) for q in rows}
        beweglich, fest = paare(kontakte), paare(geerbt)
        return dict(version='r44-kontaktpaare-v1', gemessen=True,
                    kontaktpaare=len(beweglich | fest), bewegliche_paare=len(beweglich),
                    geerbte_padpaare=len(fest), kontakte=len(kontakte),
                    geerbte_padkontakte=len(geerbt),
                    beispiele=(kontakte + geerbt)[:8])
    except Exception as exc:

        return dict(version='r44-kontaktpaare-v1', gemessen=False,
                    kontaktpaare=None, fehler=repr(exc)[:200])


def bestanden(m):
    return m.get('gemessen') is True and m.get('kontaktpaare') == 0
