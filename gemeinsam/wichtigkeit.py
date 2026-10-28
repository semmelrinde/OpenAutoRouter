""


def gewichte():
    ""
    from analyse.bewertung import GEW
    return GEW


def beitrag(gewicht, laenge, vias, via_wert=0.6):
    ""
    return gewicht * laenge + via_wert * vias


def tausch_besser(einheiten, schwelle=0.3, tol=0.05):
    ""
    gv = sum(e[3] for e in einheiten); gn = sum(e[4] for e in einheiten)
    stufen = sorted({e[0] for e in einheiten})
    umgekehrt = any(sum(e[2] - e[1] for e in einheiten if e[0] >= s) > tol for s in stufen[1:])
    return (gn < gv - schwelle and not umgekehrt), gv, gn, umgekehrt
