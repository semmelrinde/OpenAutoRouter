""
import math
import re


def natuerlich(value):
    return tuple((1, int(x)) if x.isdigit() else (0, x) for x in re.split(r'(\d+)', str(value)) if x)


def anteile(n, gewichte):
    raw = [n * w / sum(gewichte) for w in gewichte]
    counts = [math.floor(v) for v in raw]
    for i in sorted(range(len(raw)), key=lambda i: (-(raw[i] - counts[i]), i))[:n - sum(counts)]:
        counts[i] += 1
    return counts


def raster(pads):
    xs = sorted({round(p['x'], 5) for p in pads})
    ys = sorted({round(p['y'], 5) for p in pads})
    xy = {p['num']: (xs.index(round(p['x'], 5)), ys.index(round(p['y'], 5))) for p in pads}
    rings = {num: min(x, len(xs)-1-x, y, len(ys)-1-y) for num, (x, y) in xy.items()}
    maxring = max(1, (min(len(xs), len(ys))-1)//2)
    norm = {num: min(7, int(r * 7 / maxring + .5)) for num, r in rings.items()}
    return xy, rings, norm


def belegen(pads, signale, schienen, gnd, rnd):
    ""
    pads = sorted(pads, key=lambda p: natuerlich(p['num']))
    xy, rings, norm = raster(pads)
    ns, nv, ng, _ = anteile(len(pads), [65, 39, 32, 88])
    if len(signale) > ns:
        raise ValueError('BGA-Signalbudget ueberschritten: %d > %d' % (len(signale), ns))
    quotas = anteile(len(signale), [36, 9, 10, 7, 3])
    chosen = []
    for ring, count in enumerate(quotas):
        pool = [p for p in pads if norm[p['num']] == ring]


        rnd.shuffle(pool)
        chosen.extend(pool[:count])
    selected = {p['num'] for p in chosen}
    pool = [p for p in pads if p['num'] not in selected and norm[p['num']] < 5]
    rnd.shuffle(pool)
    pool.sort(key=lambda p: norm[p['num']])
    chosen.extend(pool[:len(signale)-len(chosen)])
    if len(chosen) != len(signale):
        raise ValueError('BGA hat nicht genug aeussere Signalplaetze')
    roles = {p['num']: role for p, role in zip(sorted(chosen, key=lambda p: (rings[p['num']], natuerlich(p['num']))), signale)}


    pool = [p for p in pads if p['num'] not in roles]
    weighted = [(rnd.random() ** (1 / (1 + norm[p['num']] ** 2)), p) for p in pool]
    power = [p for _, p in sorted(weighted, key=lambda v: -v[0])[:nv+ng]]
    counts = anteile(nv, [21, 12, 4, 1, 1][:len(schienen)])


    if nv >= len(schienen):
        for i, count in enumerate(counts):
            if count == 0:
                donor = max(range(len(counts)), key=lambda j: counts[j])
                counts[donor] -= 1
                counts[i] = 1
    labels = [net for net, count in zip(schienen, counts) for _ in range(count)] + [gnd] * ng

    rnd.shuffle(power)
    labels.sort(key=lambda n: (n == gnd, schienen.index(n) if n != gnd else 0))
    byxy = {xy[p['num']]: i for i, p in enumerate(power)}
    edges = [(i, byxy[q]) for i, p in enumerate(power) for x, y in [xy[p['num']]]
             for q in ((x+1, y), (x, y+1)) if q in byxy]
    def adjacent():
        return len({v for i, j in edges if labels[i] == labels[j] for v in (i, j)})
    target = round(len(power) * 25 / 71)
    error = abs(adjacent() - target)
    for _ in range(800):
        if error <= 1: break
        i, j = rnd.sample(range(len(labels)), 2)
        labels[i], labels[j] = labels[j], labels[i]
        new = abs(adjacent() - target)
        if new <= error: error = new
        else: labels[i], labels[j] = labels[j], labels[i]
    for p, net in zip(power, labels): roles[p['num']] = (net, 'power_in', 'GND' if net == gnd else 'VDD')
    signale.clear()
    return roles


def zu_zielen(pads):
    ""
    from scipy.optimize import linear_sum_assignment
    refs = sorted({p['ref'] for p in pads if p.get('_bga')})
    for ref in refs:
        slots = sorted([p for p in pads if p['ref'] == ref and p.get('pintype') == 'bidirectional'
                        and p['net'].startswith(('GPIO', 'QSPI_', 'SPI_', 'UART_', 'I2S_'))],
                       key=lambda p: natuerlich(p['num']))
        destinations = []
        for p in slots:
            other = [q for q in pads if q['ref'] != ref and q['net'] == p['net']]
            destinations.append((sum(q['x'] for q in other)/len(other), sum(q['y'] for q in other)/len(other)) if other else (p['x'], p['y']))
        if not slots: continue
        costs = [[math.hypot(p['x']-x, p['y']-y) for x, y in destinations] for p in slots]
        rows, cols = linear_sum_assignment(costs)
        roles = [(p['net'], p['pintype'], p['pinfunc']) for p in slots]
        for i, j in zip(rows, cols):
            slots[i]['net'], slots[i]['pintype'], slots[i]['pinfunc'] = roles[j]
    for p in pads: p.pop('_bga', None)


def statistik(pads):
    ""
    xy, rings, _ = raster(pads)
    power = {p['num']: p['net'] for p in pads if p['net'] and p['pintype'] == 'power_in'}
    adjacent = sum(any(power.get(q['num']) == net and sum(abs(a-b) for a, b in zip(xy[num], xy[q['num']])) == 1
                       for q in pads if q['num'] != num) for num, net in power.items())
    supply = {}; signalrings = {}; freer = {}; groundr = {}; supplyr = {}
    for p in pads:
        ring = str(rings[p['num']])
        if not p['net']: dest = freer
        elif p['pintype'] != 'power_in': dest = signalrings
        elif p['net'] == 'GND': dest = groundr
        else:
            supply[p['net']] = supply.get(p['net'], 0) + 1
            dest = supplyr
        dest[ring] = dest.get(ring, 0) + 1
    return dict(kugeln=len(pads), signale=sum(signalrings.values()), versorgung=sum(supply.values()), gnd=sum(groundr.values()),
                frei=sum(freer.values()), versorgungsnetze=supply, signalringe=signalrings, freiringe=freer,
                gndringe=groundr, versorgungsringe=supplyr, gleichnamige_nachbarn=adjacent)
