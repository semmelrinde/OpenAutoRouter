""
import math


def aktiv(value):
    if value in (True, 1, '1'): return True
    if value in (False, 0, '0', None): return False
    raise ValueError('placer_lockern=0|1')


def anfordern(z):
    if not aktiv((z.get('cfg') or {}).get('placer_lockern')): return False
    if z.get('zustand') == 'placer_zeitlimit': return True
    k = (z.get('placer_kennzahlen') or {}).get('kollisionen')
    return z.get('zustand') == 'placer_kollision' and isinstance(k, (int, float)) and not isinstance(k, bool) and math.isfinite(k) and k > 0


def etikett(A):
    ""
    area, courtyard = A.get('flaeche_mm2'), A.get('courtyard_mm2')
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and v > 0 for v in (area, courtyard)):
        raise ValueError('Lockerung braucht gemessene Aufgabenflaechen')
    return dict(gelockert=int(A.get('gelockert', 0)), dichte=courtyard / area,
                dichte_definition='Generator-Courtyardflaeche/Outlineflaeche',
                lockerung_stufen=A.get('lockerung_stufen', 0),
                faktor_ursprung=A.get('faktor_ursprung', A['faktor']))


def naechster_faktor(A, zeilen):
    ""
    if A.get('lockerung_stufen', 0) >= 2: return None
    factor = round(A['faktor'] * 1.12, 3)
    if any(z.get('gueltig') is True and z.get('faktor', math.inf) <= factor for z in zeilen): return None
    return factor


def uebernehmen(vorher, nachher):
    ""
    for key in ('seed', 'lagen', 'teile', 'pads', 'netze', 'bloecke', 'courtyard_mm2'):
        if vorher[key] != nachher[key]: raise ValueError('Lockerung aendert Aufgabenbauplan: ' + key)
    if nachher['flaeche_mm2'] <= vorher['flaeche_mm2']: raise ValueError('Lockerung vergroessert Outline nicht')
    nachher.update(gelockert=1, lockerung_stufen=vorher.get('lockerung_stufen', 0) + 1,
                   faktor_ursprung=vorher.get('faktor_ursprung', vorher['faktor']))
    return nachher
