""
import copy

VERSION = 'p3-pin-gruppen-v1'


def board_mit_gruppen(board, placement):
    out = dict(board)
    if 'p3_pin_gruppen' in placement:
        out['p3_pin_gruppen'] = copy.deepcopy(placement['p3_pin_gruppen'])
    if 'p26_fest' in placement:
        out['p26_fest'] = copy.deepcopy(placement['p26_fest'])
    return out


def anwenden(board, klassen):
    ""
    m = board.get('p3_pin_gruppen')
    if m is None: return
    if m.get('version') != VERSION: raise ValueError('Unbekannte P3-Gruppenversion')
    for c, ziel in sorted(m['teile'].items()):
        a, n, pins = ziel['ref'], ziel['netz'], ziel['pins']
        vorhanden = {p['num'] for p in board['pads'] if p['ref'] == a and p['net'] == n}
        cp = [p for p in board['pads'] if p['ref'] == c and p['net'] == n]
        t = klassen['teile'].get(c, {})
        if (not isinstance(pins, list) or not pins or not all(isinstance(p, str) for p in pins)
                or len(pins) != len(set(pins)) or not set(pins) <= vorhanden
                or not cp or a not in klassen['inseln'] or
                not t.get('funktion', '').startswith('entkopplung_')):
            raise ValueError('P3-Pinziel passt nicht zum Board: ' + c)
        from gemeinsam.fluss import padgeo
        cg = list(padgeo(cp[0]).values())
        zp = [(i, p) for i, p in enumerate(board['pads']) if p['ref'] == a and p['net'] == n and p['num'] in pins]
        _, pin = min(zp, key=lambda ip: (min(x.distance(y) for x in cg for y in padgeo(ip[1]).values()), ip[0]))
        st = dict(t.get('stuetzt') or {}, ref=a, netz=n, pins=list(pins), pin=pin['num'],
                  p3_gruppe=True, ueber_spule=False)
        t['stuetzt'] = st
        t['insel'] = a
        for insel in klassen['inseln'].values():
            insel['entkopplung'] = [e for e in insel['entkopplung'] if e['ref'] != c]
            if c in insel['mitglieder']: insel['mitglieder'].remove(c)
        insel = klassen['inseln'][a]
        insel['mitglieder'].append(c)
        insel['entkopplung'].append(dict(ref=c, netz=n, pins=list(pins), C=t['C'], ueber_spule=False))
    klassen['p3_pin_gruppen'] = copy.deepcopy(m)
