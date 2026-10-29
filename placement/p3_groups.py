import copy

VERSION = 'p3-pin-group_list-v1'


def board_with_groups(board, placement):
    """Copy the pin groups and fixed parts that the placer stored in placement.json into the board dict, so that the
    router sees the same groups as the placer.
    """
    out = dict(board)
    if 'p3_pin_groups' in placement:
        out['p3_pin_groups'] = copy.deepcopy(placement['p3_pin_groups'])
    if 'p26_fixed' in placement:
        out['p26_fixed'] = copy.deepcopy(placement['p26_fixed'])
    return out


def apply(board, netclasses):
    m = board.get('p3_pin_groups')
    if m is None: return
    if m.get('version') != VERSION: raise ValueError('Unknown P3 group version')
    for c, goal in sorted(m['part_list'].items()):
        a, n, pins = goal['ref'], goal['net_name'], goal['pins']
        present = {p['num'] for p in board['pads'] if p['ref'] == a and p['net'] == n}
        cp = [p for p in board['pads'] if p['ref'] == c and p['net'] == n]
        t = netclasses['part_list'].get(c, {})
        if (not isinstance(pins, list) or not pins or not all(isinstance(p, str) for p in pins)
                or len(pins) != len(set(pins)) or not set(pins) <= present
                or not cp or a not in netclasses['islands'] or
                not t.get('function', '').startswith('decoupling_')):
            raise ValueError('P3 pin target does not match the board: ' + c)
        from common.flow import pad_geo
        cg = list(pad_geo(cp[0]).values())
        zp = [(i, p) for i, p in enumerate(board['pads']) if p['ref'] == a and p['net'] == n and p['num'] in pins]
        _, pin = min(zp, key=lambda ip: (min(x.distance(y) for x in cg for y in pad_geo(ip[1]).values()), ip[0]))
        st = dict(t.get('supports') or {}, ref=a, net_name=n, pins=list(pins), pin=pin['num'],
                  p3_group=True, over_coil=False)
        t['supports'] = st
        t['island'] = a
        for island in netclasses['islands'].values():
            island['decoupling'] = [e for e in island['decoupling'] if e['ref'] != c]
            if c in island['members']: island['members'].remove(c)
        island = netclasses['islands'][a]
        island['members'].append(c)
        island['decoupling'].append(dict(ref=c, net_name=n, pins=list(pins), C=t['C'], over_coil=False))
    netclasses['p3_pin_groups'] = copy.deepcopy(m)
