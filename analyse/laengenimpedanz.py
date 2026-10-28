""
import collections
import heapq
import math
from gemeinsam.fluss import netzgraph
from analyse.aufbau import modelle, normalisieren, bezugslagen, bezugsflaechen
from analyse.impedanz_tabelle import wert


def weglaengen(B, R, net, aufbau=None, alle_pads=False, quelle_ref=None):
    ""
    tracks = [t for t in R.get('tracks', []) if t['net'] == net]
    vias = [v for v in R.get('vias', []) if v['net'] == net]
    pads = [p for p in B['pads'] if p['net'] == net]
    if len(pads) < 2: return []
    G = netzgraph(tracks, vias, pads, tol=0.0002)
    names = ['F'] + ['In%d' % i for i in range(1, B.get('lagen', 4) - 1)] + ['B']
    a = normalisieren(aufbau, len(names)); heights = {names[0]: 0.0}
    cu = a['kupfer_mm']
    for i, d in enumerate(a['dielektrika']): heights[names[i + 1]] = heights[names[i]] + d + (cu[i] + cu[i+1])/2
    pmap = {('PAD', p['ref'], p['num']): p for p in pads}


    for u in list(G['adj']):
        G['adj'][u] = [(v, kid) for v, kid in G['adj'][u] if not (kid and kid[0] == 'v')]
    for vi, v in enumerate(vias):
        nodes = [(l, round(v['x'] / .0002) * .0002, round(v['y'] / .0002) * .0002) for l in names]
        for u, w in zip(nodes[:-1], nodes[1:]):
            G['adj'][u].append((w, ('v', vi))); G['adj'][w].append((u, ('v', vi)))
    def padweg(p, node):


        dz = min(abs(heights[node[0]] - heights[l]) for l in p['layers'] if l in heights)
        return math.dist((p['x'], p['y']), node[1:]) + dz
    def cost(u, v, kid):
        if kid and kid[0] == 's':
            _, ti, si, f1, f2 = kid
            return math.dist(tracks[ti]['pts'][si], tracks[ti]['pts'][si + 1]) * (f2 - f1)
        if kid and kid[0] == 'v': return abs(heights[u[0]] - heights[v[0]])
        if u[0] == 'PAD' and v[0] != 'PAD': return padweg(pmap[u], v)
        if v[0] == 'PAD' and u[0] != 'PAD': return padweg(pmap[v], u)
        return 0.0
    ends = sorted(set(G['padk'].values()))
    sources = [u for u in ends if pmap[u].get('pintype') in ('output', 'tri_state')]
    if quelle_ref is not None:
        sources = [u for u in ends if u[1] == quelle_ref]
        if not sources: return []

    if len(ends) == 2: sources = (sources or ends)[:1]
    if not sources and (len(ends) == 2 or alle_pads): sources = ends[:1]
    if not sources: return []
    paths = []
    for src in sources:
        dist = {src: 0.0}; queue = [(0.0, src)]
        while queue:
            du, u = heapq.heappop(queue)
            if du != dist[u]: continue
            for v, kid in G['adj'].get(u, []):
                dv = du + cost(u, v, kid)
                if dv < dist.get(v, math.inf): dist[v] = dv; heapq.heappush(queue, (dv, v))
        for dst in ends:
            if dst == src: continue
            paths.append(dict(quelle=':'.join(src[1:]), empfaenger=':'.join(dst[1:]),
                              zuordnung=src[1] + '->' + dst[1], zweipunkt=len(ends) == 2,
                              laenge_mm=dist.get(dst)))
    return paths


def weglaenge(B, R, net, aufbau=None):
    ""
    paths = weglaengen(B, R, net, aufbau)
    return max(p['laenge_mm'] for p in paths) if paths and all(p['laenge_mm'] is not None for p in paths) else None


def laengengruppen(K):
    ""
    groups = {g: dict(v, netze=list(v['netze'])) for g, v in ((K.get('vorgaben') or {}).get('laengengruppen') or {}).items()}
    explicit_tol = {g for g, v in groups.items() if v.get('toleranz_mm') is not None}
    nets = K['netze']
    for n, k in sorted(nets.items()):
        if k.get('laengengruppe'):
            g = groups.setdefault(k['laengengruppe'], dict(netze=[], toleranz_mm=k.get('toleranz_mm', 0.15)))
            g['netze'] = sorted(set(g.get('netze', []) + [n]))
            if k['laengengruppe'] not in explicit_tol and k.get('toleranz_mm') is not None: g['toleranz_mm'] = min(float(g.get('toleranz_mm', 0.15)), float(k['toleranz_mm']))
        if k.get('paar') and k.get('z_diff_ohm'):
            p = sorted([n, k['paar']]); name = 'paar:' + '|'.join(p)
            ts = [nets[q].get('paar_toleranz_mm', nets[q].get('toleranz_mm')) for q in p if q in nets]
            tol = min(float(t) for t in ts if t is not None) if any(t is not None for t in ts) else .15
            groups.setdefault(name, dict(netze=p, toleranz_mm=tol))
        elif k.get('z_diff_ohm'):
            groups['paar_unbekannt:' + n] = dict(netze=[n], toleranz_mm=0.15, paar_unbekannt=True)
    return groups


def laengen(B, R, K):
    A = normalisieren((K.get('vorgaben') or {}).get('aufbau') or K.get('aufbau'), B.get('lagen', 4))
    groups = laengengruppen(K)
    nets = K['netze']
    wanted = sorted({n for g in groups.values() for n in g['netze']})
    pair_sources = {}
    for n in wanted:
        k = nets[n]; partner = k.get('paar')
        if not partner or k.get('paar_art') != 'schnell': continue
        pp = {q: collections.Counter(p['ref'] for p in B['pads'] if p['net'] == q) for q in (n, partner)}
        unique = sorted(r for r in set(pp[n]) & set(pp[partner]) if pp[n][r] == pp[partner][r] == 1)


        drivers = {p['ref'] for p in B['pads'] if p['net'] == n and p.get('pintype') in ('output', 'tri_state')}
        chips = [r for r in unique if r.startswith(('U', 'IC')) and all(
            any(p['net'] == q and p['ref'] == r and p.get('pintype') in ('output', 'tri_state', 'bidirectional')
                for p in B['pads']) for q in (n, partner))]
        explicit = k.get('quelle_ref')
        if explicit is not None: pair_sources[n] = explicit
        elif len(chips) == 1 and (not drivers or drivers == {chips[0]}): pair_sources[n] = chips[0]
    paths = {n: weglaengen(B, R, n, A, quelle_ref=nets[n].get('quelle_ref', pair_sources.get(n))) for n in wanted}
    lengths = {n: max(p['laenge_mm'] for p in pp) if pp and all(p['laenge_mm'] is not None for p in pp) else None for n, pp in paths.items()}
    out = {}
    for g, q in sorted(groups.items()):
        vals = {n: lengths[n] for n in q['netze']}; tol = float(q.get('toleranz_mm', 0.15))
        if not math.isfinite(tol) or tol < 0: raise ValueError('Laengentoleranz muss endlich und >= 0 sein')
        keyed = {n: collections.defaultdict(list) for n in vals}
        for n in vals:
            for p in paths[n]:
                key = 'punktpunkt' if g.startswith('paar:') and p['zweipunkt'] else p['zuordnung']
                keyed[n][key].append(p['laenge_mm'])
        receivers = sorted({e for nn in keyed.values() for e in nn})
        comparisons = {}
        for e in receivers:
            vv = {n: keyed[n].get(e, []) for n in vals}



            usb_c_pins = all({p['empfaenger'].rsplit(':', 1)[1] for p in paths[n] if p['zuordnung'] == e}
                             in ({'A6', 'B6'}, {'A7', 'B7'}) for n in vals)
            if g.startswith('paar:') and all(n in pair_sources for n in vals) and usb_c_pins and len({len(v) for v in vv.values()}) == 1 and len(next(iter(vv.values()))) > 1:
                ordered = {n: sorted((p for p in paths[n] if p['zuordnung'] == e), key=lambda p: p['empfaenger']) for n in vals}
                for i in range(len(next(iter(vv.values())))):
                    lens = {n: ordered[n][i]['laenge_mm'] for n in vals}
                    ok = all(v is not None for v in lens.values())
                    comparisons[e + ':pin%d' % i] = dict(netze={n: [v] for n,v in lens.items()},
                        fehler_mm=max(lens.values())-min(lens.values()) if ok else None, vollstaendig=ok,
                        pins={n: ordered[n][i]['empfaenger'] for n in vals})
                continue

            ok = all(len(v) == 1 and v[0] is not None for v in vv.values())
            err = max(v[0] for v in vv.values()) - min(v[0] for v in vv.values()) if ok else None
            comparisons[e] = dict(netze=vv, fehler_mm=err, vollstaendig=ok)
        known = bool(receivers) and not q.get('paar_unbekannt') and all(v['vollstaendig'] for v in comparisons.values())
        error = max(v['fehler_mm'] for v in comparisons.values()) if known else None
        out[g] = dict(netze=vals, toleranz_mm=tol, fehler_mm=error, vollstaendig=known,
                      wege={n: paths[n] for n in vals}, empfaenger=comparisons,
                      ueber_mm=max(0.0, error - tol) if known else None)
    return out


def empfaenger_schutz(vorher, nachher, netze=None, streng=False):
    ""
    for name, old in vorher.items():
        if netze is not None and not set(old['netze']).intersection(netze): continue
        for receiver, a in old['empfaenger'].items():
            if not a['vollstaendig']: continue
            b = (nachher.get(name) or {}).get('empfaenger', {}).get(receiver)
            if b is None or not b['vollstaendig']: return False
            grenze = a['fehler_mm'] if streng else max(a['fehler_mm'], old['toleranz_mm'])
            if b['fehler_mm'] > grenze + 1e-6: return False
    return True


def empfaenger_klassen(K):
    ""
    import copy
    out = copy.deepcopy(K)
    for k in out['netze'].values():
        if k.get('paar') and k.get('paar_art', k.get('art')) == 'schnell':
            k['z_diff_ohm'] = k.get('z_diff_ohm') or 1
    return out


def messen(B, R, K):
    from shapely.geometry import LineString
    op = R.get('optionen') or {}; refs = bezugslagen(R, K.get('gnd', []), K['netze'])
    profiles = modelle((K.get('vorgaben') or {}).get('aufbau') or K.get('aufbau'), B.get('lagen', int(op.get('lagen', 4))), refs)
    planes = bezugsflaechen(B, K.get('gnd', []), R, K['netze'])
    indexed = collections.defaultdict(list)
    for t in R.get('tracks', []):
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            if math.dist(a, b) > 1e-9: indexed[t['net'], t['layer']].append((LineString([a, b]), w))
    out = {}; sumerr = sumlen = 0.0; unknown = 0.0; incomplete = 0
    for n, k in sorted(K['netze'].items()):
        diff = bool(k.get('z_diff_ohm')); target = k.get('z_diff_ohm') if diff else k.get('z_ohm')
        if not target: continue
        if not math.isfinite(float(target)) or target <= 0: raise ValueError('Sollimpedanz muss endlich und positiv sein')
        err = length = missing = 0.0; worst = 0.0
        for (ne, layer), ss in sorted(indexed.items()):
            if ne != n: continue
            for line, w in ss:
                if layer not in profiles: missing += line.length; continue
                if planes is not None and any(ref not in planes or not planes[ref].covers(line.buffer(w/2)) for ref in profiles[layer]['ref']):
                    missing += line.length; continue
                samples = max(1, int(math.ceil(line.length / 1.0))) if diff else 1
                gaps = []
                if diff:
                    pp = indexed.get((k.get('paar'), layer), [])
                    if not pp: missing += line.length; continue
                    direction = tuple(b-a for a, b in zip(line.coords[0], line.coords[-1]))
                    parallel = [(l, pw) for l, pw in pp if abs(sum(x*(b-a) for x, a, b in zip(direction, l.coords[0], l.coords[-1]))) >= 0.98*line.length*l.length]
                    for i in range(samples):
                        center = line.interpolate((i+.5)/samples, normalized=True)
                        candidates = [(l, pw) for l, pw in parallel if 1e-5 < l.project(center) < l.length - 1e-5]
                        if not candidates: gaps.append(None); continue
                        partner, pw = min(candidates, key=lambda q: q[0].distance(center))

                        gaps.append(round(max(0.001, partner.distance(center) - (w + pw)/2), 3) if abs(pw-w) <= .001 else None)
                else: gaps = [0.0]
                for gap in gaps:
                    if gap is None: missing += line.length / samples; continue
                    if diff and gap > max(2.0, 8*profiles[layer]['h']):
                        actual = 2*wert(profiles[layer], w)
                    else: actual = wert(profiles[layer], w, gap, diff)
                    delta = abs(actual / target - 1.0); err += delta * line.length / samples; length += line.length / samples; worst = max(worst, delta)
        paths = weglaengen(B, R, n, (K.get('vorgaben') or {}).get('aufbau') or K.get('aufbau'), alle_pads=True)
        connected = bool(paths) and all(p['laenge_mm'] is not None for p in paths)
        absent = length + missing <= 1e-9 or not connected
        complete = not absent and missing < 1e-6
        incomplete += not complete
        out[n] = dict(abweichung_rel=err / length if length else None, max_rel=worst if length else None,
                      gemessen_mm=length, unbekannt_mm=missing, soll_ohm=target, vollstaendig=complete,
                      strecke_fehlt=absent, verbunden=connected)
        sumerr += err; sumlen += length; unknown += missing
    groups = laengen(B, R, K)
    penalty = sumerr + unknown + 10*incomplete + sum(q['ueber_mm'] or 0.0 for q in groups.values()) + 10*sum(not q['vollstaendig'] for q in groups.values())
    return dict(version='r35-feld-v2', netze=out, laengengruppen=groups,
                impedanz_rel=sumerr / sumlen if sumlen else None, unbekannt_mm=unknown,
                kontrollierte_netze_unvollstaendig=incomplete, kontrollierte_netze_fehlend=sum(v['strecke_fehlt'] for v in out.values()),
                laengenfehler_max_mm=max([q['fehler_mm'] for q in groups.values() if q['fehler_mm'] is not None], default=None),
                impedanz_vollstaendig=all(v['vollstaendig'] for v in out.values()),
                laengengruppen_unbekannt=sum(not q['vollstaendig'] for q in groups.values()),
                laengentor=all(q['vollstaendig'] and q['ueber_mm'] <= 1e-6 for q in groups.values()),
                note_zusatz=penalty)
