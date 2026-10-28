""
import json, math, os, re, sys, uuid

A = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, A)
from gemeinsam import maeander as M
from gemeinsam.paarnetze import paare as paare_finden


def _r(v):
    return round(float(v), 6)


def paar_daten(H, P, N):
    ""
    out = {}
    for s, net in (('P', P), ('N', N)):
        lagen = {}; boegen = {}
        for t in H['tracks']:
            if t['net'] != net: continue
            if t.get('arc'): boegen.setdefault((t['uuid'], t['layer']), []).append(t); continue
            lagen.setdefault(t['layer'], dict(kanten=[], fest=[]))['kanten'].append(dict(typ='seg', a=t['a'], b=t['b'], w=t['w'], uuid=t['uuid']))
        for (u, L), ch in sorted(boegen.items()):
            if len(ch) != 2: continue
            lagen.setdefault(L, dict(kanten=[], fest=[]))['kanten'].append(dict(typ='arc', a=ch[0]['a'], m=ch[0]['b'], b=ch[1]['b'], w=ch[0]['w'], uuid=u))
        for L, d in lagen.items():
            d['fest'] = sorted({(p['x'], p['y']) for p in H.get('pads', []) if p['net'] == net and p['layer'] == L and 'x' in p}) + \
                        sorted({(v['x'], v['y']) for v in H.get('vias', []) if v['net'] == net and L in (v.get('layers') or [L])})
        out[s] = dict(netz=net, lagen=lagen)
    return out


def skew_regeln(dru_text):
    ""
    out = {}
    for blk in re.split(r'\n\s*\(rule\b', dru_text or ''):
        m = re.search(r"inDiffPair\('([^']*)'\)", blk); s = re.search(r'\(constraint\s+skew\s+\(max\s+([0-9.]+)\s*mm', blk)
        if m and s: out[m.group(1)] = float(s.group(1))
    return out


def toleranz(P, N, regeln, std=0.1):
    import fnmatch
    best = None
    for basis, tol in sorted(regeln.items()):
        if any(fnmatch.fnmatchcase(n, basis + '*') for n in (P, N)): best = tol if best is None else min(best, tol)
    return std if best is None else best


def _uid(*teile):
    return str(uuid.uuid5(uuid.NAMESPACE_URL, 'autorouter-maeander/' + json.dumps(teile, sort_keys=True)))


def generator_block(g):
    ""
    z = lambda v: ('%.6f' % v).rstrip('0').rstrip('.')
    q = lambda s: '"%s"' % s.replace('\\', '\\\\').replace('"', '\\"')
    L = g['layer'] if g['layer'].endswith('.Cu') else ('F.Cu' if g['layer'] == 'F' else 'B.Cu' if g['layer'] == 'B' else g['layer'] + '.Cu')
    (x0, y0), (x1, y1) = g['von'], g['bis']
    zeilen = ['(uuid %s)' % q(g['uuid']), '(type tuning_pattern)', '(name "Tuning Pattern")', '(layer %s)' % q(L),
              '(base_line\n\t\t\t(pts\n\t\t\t\t(xy %s %s) (xy %s %s)\n\t\t\t)\n\t\t)' % (z(x0), z(y0), z(x1), z(y1)),
              '(corner_radius_percent %d)' % round(g.get('eckenradius', 100)), '(end\n\t\t\t(xy %s %s)\n\t\t)' % (z(x1), z(y1)),
              '(initial_side %s)' % q(g.get('seite', 'default')), '(last_diff_pair_gap %s)' % z(g.get('spalt', 0)),
              '(last_netname %s)' % q(g['netz']), '(last_status "tuned")', '(last_track_width %s)' % z(g['w']),
              '(max_amplitude %s)' % z(g.get('amax', 1.0)), '(min_amplitude %s)' % z(g.get('amin', 0.1)), '(min_spacing %s)' % z(g['s']),
              '(origin\n\t\t\t(xy %s %s)\n\t\t)' % (z(x0), z(y0)), '(override_custom_rules no)', '(rounded yes)', '(single_sided yes)',
              '(target_skew 0)', '(target_skew_max %s)' % z(g.get('tol', 0.1)), '(target_skew_min %s)' % z(-g.get('tol', 0.1)),
              '(tuning_mode "diff_pair_skew")', '(members %s)' % ' '.join(q(u) for u in g['members'])]
    return '\n\t(generated\n' + ''.join('\t\t%s\n' % x for x in zeilen) + '\t)'


def maeander_aenderung(H, hin, paare, opt=None, sperren=()):
    ""
    o = dict(opt or {})
    plan_opt = {}
    if o.get('stufen'): plan_opt['stufen'] = tuple(float(x) for x in str(o['stufen']).split(','))
    for k in ('amax', 'rmin', 'lat_max', 'nah_max'):
        if k in o: plan_opt[k] = float(o[k])
    skew_min = float(o.get('skew_min', 0.02)); gen = str(o.get('generator', '0')) == '1'
    D = dict(remove=[], add=[], add_generated=[], stat=dict(paare={}))
    ersetzt = {}
    for P, N, tol in sorted(paare):
        if P in sperren or N in sperren: D['stat']['paare'][P] = 'gesperrt'; continue
        pd = paar_daten(H, P, N)
        if not pd['P']['lagen'] or not pd['N']['lagen']: continue
        umg = M.Umgebung(hin, {P, N}, ersetzt)
        R = M.paar_planen(pd, umg, dict(plan_opt, toleranz=tol))
        info = R['info']

        hat_alt = any(x['boegen'] == 0 for x in info['gerade_gezogen']['P'] + info['gerade_gezogen']['N'])
        if R.get('info', {}).get('fehler') or not R['neu'] or (not hat_alt and abs(info['skew_vorher']) <= skew_min):
            info['angewendet'] = False; D['stat']['paare'][P] = info; continue
        if abs(info['skew_neu']) > abs(info['skew_vorher']) + 1e-6 and abs(info['skew_neu']) > tol:
            info['angewendet'] = False; info['grund'] = 'Skew würde schlechter'; D['stat']['paare'][P] = info; continue
        info['angewendet'] = True; info['toleranz'] = tol; D['stat']['paare'][P] = info
        n_vorher = (len(D['remove']), len(D['add']))
        for net, lagen in sorted(R['neu'].items()):
            for L, kl in sorted(lagen.items()):
                for k in kl:
                    neu = []
                    for p in k['prims']:
                        if p[0] == 'seg':
                            if M.dist(p[1], p[2]) < 1e-6: continue
                            it = dict(net=net, layer=L, a=[_r(p[1][0]), _r(p[1][1])], b=[_r(p[2][0]), _r(p[2][1])], w=k['w'])
                        else:
                            it = dict(net=net, layer=L, a=[_r(p[1][0]), _r(p[1][1])], m=[_r(p[2][0]), _r(p[2][1])], b=[_r(p[3][0]), _r(p[3][1])], w=k['w'])
                        it['uuid'] = _uid(it); neu.append(it)
                    if {it['uuid'] for it in neu} == set(k['uuids']): continue
                    D['remove'] += k['uuids']; D['add'] += neu
                    if gen and net == info['kurz']:
                        spannen = [st for st in info['stellen'] if st['lage'] == L]
                        for st in spannen:
                            if any(x is not st and _in_spanne(st, x) for x in spannen): continue
                            g = _generator(st, neu, net, L, k['w'], tol, P, N, spalt=info.get('paarabstand', {}).get(L, 0.0))
                            if g: D['add_generated'].append(g)
        if (len(D['remove']), len(D['add'])) == n_vorher: info['angewendet'] = False; info['grund'] = 'unverändert'
        ersetzt.update(R['geo'])
    D['remove'] = sorted(set(D['remove']))
    return D


def _in_spanne(st, x):
    ""
    def auf(q):
        a, b = x['von'], x['bis']; L = M.dist(a, b)
        if L < 1e-9: return False
        t = ((q[0] - a[0]) * (b[0] - a[0]) + (q[1] - a[1]) * (b[1] - a[1])) / L
        return -1e-6 <= t <= L + 1e-6 and abs((q[0] - a[0]) * (b[1] - a[1]) - (q[1] - a[1]) * (b[0] - a[0])) / L < 1e-5
    return auf(st['von']) and auf(st['bis']) and (st['von'], st['bis']) != (x['von'], x['bis'])


def _generator(st, neu, net, L, w, tol, P, N, spalt=0.0):
    ""
    i0 = [i for i, it in enumerate(neu) if M.dist(it['a'], st['von']) < 1e-5]
    i1 = [i for i, it in enumerate(neu) if M.dist(it['b'], st['bis']) < 1e-5]
    if not i0 or not i1 or i1[-1] < i0[0]: return None
    glieder = neu[i0[0]:i1[-1] + 1]
    von, bis = st['von'], st['bis']

    ux, uy = bis[0] - von[0], bis[1] - von[1]
    spitze = max((it for it in glieder if 'm' in it), key=lambda it: abs(ux * (it['m'][1] - von[1]) - uy * (it['m'][0] - von[0])), default=None)
    seite = 'default'
    if spitze:
        seite = 'right' if ux * (spitze['m'][1] - von[1]) - uy * (spitze['m'][0] - von[0]) > 0 else 'left'
    return dict(uuid=_uid('gen', net, von, bis), layer=L, von=von, bis=bis, netz=net, w=w, s=st['s'], tol=tol, seite=seite, spalt=spalt,
                members=[it['uuid'] for it in glieder], eckenradius=round(100 * min(1.0, st['r'] / (st['s'] / 2))))


def main():
    a = sys.argv[1:]; o = dict(x.split('=', 1) for x in a[2:] if '=' in x)
    H = json.load(open(a[0]))
    from gemeinsam.kupfer_netz import Hindernisse
    hin = Hindernisse(H, drc=float(o.get('drc', 0.09)), drcpad=float(o.get('drcpad', 0.10)), drcvia=float(o.get('drcvia', o.get('drc', 0.09))),
                      loch=float(o.get('loch', 0.25)), kante=float(o.get('kante', 0.2)))
    regeln = skew_regeln(open(o['dru']).read()) if o.get('dru') else {}
    if o.get('paare'):
        pl = [tuple(x.split(',')) for x in o['paare'].split(';') if x]
    else:
        pl = paare_finden(sorted({t['net'] for t in H['tracks'] if t['net']}), o.get('klassen'))
    D = maeander_aenderung(H, hin, [(p, n, toleranz(p, n, regeln)) for p, n in pl], o)
    json.dump(D, open(a[1], 'w'), indent=1, ensure_ascii=False)
    for p, inf in sorted(D['stat']['paare'].items()):
        if isinstance(inf, dict):
            print(p, 'E %.4f' % inf['E'], 'kurz', inf['kurz'], 'Skew', inf['skew_vorher'], '->', inf['skew_neu'], 'Stellen', len(inf['stellen']),
                  'angewendet' if inf.get('angewendet') else 'nicht angewendet', inf.get('grund', ''))
    print('entfernt %d, neu %d (Bögen %d), Generatoren %d' % (len(D['remove']), len(D['add']), sum(1 for x in D['add'] if 'm' in x), len(D['add_generated'])))


if __name__ == '__main__':
    main()
