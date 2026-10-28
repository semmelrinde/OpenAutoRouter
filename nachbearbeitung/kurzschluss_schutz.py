""
import json, math
import numpy as np


class R44Fehler(RuntimeError):
    ""


def schutz_an(*quellen):
    ""
    wert = '1'
    for q in quellen:
        if isinstance(q, dict):
            if q.get('kurzschluss_schutz') is not None: wert = q['kurzschluss_schutz']
        elif q:
            for a in q:
                if isinstance(a, str) and a.startswith('kurzschluss_schutz='): wert = a.split('=', 1)[1]
    return str(wert).strip().lower() not in ('0', 'false', 'aus', 'nein')


def schutz_option(an):
    ""
    return ['kurzschluss_schutz=%d' % bool(an)]

def bahn_vorschau(t):
    ""
    if 'items' not in t: return t, 0.0
    from gemeinsam.bahnitems import arc_geometry, native_arc_representable, validate
    points, widths = [], []
    for it in t['items']:
        if it[0] == 'A':
            c, r, ang, sweep = arc_geometry(it)
            if not native_arc_representable(it): raise ValueError('entarteter nativer Bogen')
            step = 2 * math.acos(max(-1., min(1., 1 - .00025 / r)))
            count = max(1, math.ceil(abs(sweep) / max(step, 1e-10)))
            if count > 100000: raise ValueError('Bogen nicht sicher abtastbar')
            ps = [[c[0]+r*math.cos(ang+sweep*k/count), c[1]+r*math.sin(ang+sweep*k/count)] for k in range(count+1)]
            ps[0], ps[-1] = it[1], it[3]
        elif it[0] == 'L': ps = it[1:3]
        else: raise ValueError('unbekanntes Bahnitem')
        if points and math.dist(points[-1], ps[0]) > 2e-6: raise ValueError('unterbrochene Item-Kette')
        points.extend(ps if not points else ps[1:]); widths.extend([it[-1]]*(len(ps)-1))
    q = dict(t, pts=points, w=widths)
    validate(q, error=.00025)
    return q, .000252

def _null(a, b):
    ""
    return abs(a[0] - b[0]) < 1e-6 and abs(a[1] - b[1]) < 1e-6


def ausgabe_kontakte(board, data):
    ""
    from shapely.geometry import Polygon, LineString, Point
    from shapely.strtree import STRtree
    stack = ['F'] + ['In%d' % i for i in range(1, int(board.get('lagen') or 2) - 1)] + ['B']
    geoms = {L: [] for L in stack}
    def add(L, net, g, art, ort, einheit=None):
        if not g.is_empty: geoms.setdefault(L, []).append((net, g, art, ort, einheit or net))
    def rund(g, r): return g.buffer(r / math.cos(math.pi / 64) + 1e-9, quad_segs=16)
    tie_groups = {f['ref']: [set(g) for g in f.get('net_tie_pad_groups', [])]
                  for f in board.get('fps', []) if f.get('net_tie')}
    pad_refs = {}
    seen = set()
    for p in list(board['pads']) + list(board.get('footprint_copper') or []):
        if (p not in (board.get('footprint_copper') or []) and not p.get('graphic') and not p.get('pad_type')
            and ((str(p.get('net', '')).startswith('#SPERR_') and str(p.get('ref', '')).startswith('SPERR_'))
                 or (str(p.get('net', '')).startswith('#NTBAR_') and str(p.get('ref', '')).startswith('NTBAR_')))): continue
        net = p.get('net') or '#pad:%s:%s' % (p.get('ref'), p.get('num'))
        polys = dict(p.get('polys') or {})


        if p.get('pad_type') == 'pth' and not p.get('padstack_complete') and p.get('drill') and any(float(q) > 0 for q in (p['drill'] if isinstance(p['drill'], list) else [p['drill']])):
            outer = polys.get('F') or polys.get('B') or []
            for L in stack: polys.setdefault(L, outer)
        for L, qs in polys.items():
            for q in qs:
                g = Polygon(q)
                key = (L, net, p.get('ref'), p.get('num'), g.wkb)
                if key in seen: continue
                seen.add(key); pad_refs[id(g)] = (p.get('ref'), p.get('num'))
                add(L, net, g, 'pad', [p['x'], p['y']])
    for t in data['tracks']:
        t, error = bahn_vorschau(t)
        for a, b, w in zip(t['pts'][:-1], t['pts'][1:], t['w']):
            if _null(a, b): continue
            add(t['layer'], t['net'], rund(LineString([a, b]), w / 2 + error + (2e-6 if error else 0)), 'bahn', a, t.get('einheit'))
    d = float(str(data.get('optionen', {}).get('via', '0.45/0.20')).split('/')[0])
    for v in data['vias']:
        g = rund(Point(v['x'], v['y']), float(v.get('d', v.get('diameter', d))) / 2)


        for L in v.get('layers') or stack: add(L, v['net'], g, 'via', [v['x'], v['y']], v.get('einheit'))
    for z in data.get('zonen', []) + data.get('zones', []):
        if z.get('filled') is True:


            if not z.get('pts'): raise ValueError('R44: Fuellzone ohne pruefbare Kupfergeometrie')
            q = Polygon(z['pts'], z.get('holes', []))
            add(z['layer'], z['net'], q, 'zone', list(q.representative_point().coords)[0], z.get('einheit'))
            continue
        qs = [Polygon(q['aussen'], q.get('innen', [])) for q in z.get('polys', [])]
        qs += [Polygon(a, h) for a, h in z.get('filled', [])] if isinstance(z.get('filled'), list) else []
        if z.get('native_outlines'):
            qs = [Polygon(q['pts'], q.get('holes', [])) for q in z['native_outlines']]
        elif z.get('pts') and not qs: qs = [Polygon(z['pts'], z.get('holes', []))]
        if not qs: raise ValueError('R44: Zone ohne pruefbare Kupfergeometrie')
        for L in z.get('native_layers') or [z['layer']]:
            for q in qs: add(L, z['net'], q, 'zone', list(q.representative_point().coords)[0], z.get('einheit'))
    out = []; geerbt = []
    for L, G in sorted(geoms.items()):
        if not G: continue
        gs = np.asarray([q[1] for q in G], dtype=object)
        ai, bi = STRtree(gs).query(gs, predicate='intersects')
        for i, j in sorted(zip(ai.tolist(), bi.tolist())):
            if i >= j or G[i][0] == G[j][0]: continue
            a, b = G[i], G[j]
            if a[2] == b[2] == 'pad':
                ref, num = pad_refs[id(a[1])]
                other_ref, other_num = pad_refs[id(b[1])]
                if ref == other_ref and any({num, other_num} <= group for group in tie_groups.get(ref, [])): continue
            c = a[1].intersection(b[1]).representative_point()
            e = dict(lage=L, netze=[a[0], b[0]], arten=[a[2], b[2]], einheiten=[a[4], b[4]], ort=[round(c.x, 6), round(c.y, 6)])
            (geerbt if a[2] == b[2] == 'pad' else out).append(e)
    return out, geerbt


def ausgabe_auswahl(kanten, zwang, wichtig, solver=True):
    ""
    import collections
    zwang = set(zwang); kanten = {tuple(sorted(e)) for e in kanten if not set(e) & zwang}
    nodes = sorted({n for e in kanten for n in e})
    chosen = set(zwang); rest = set(kanten)
    while rest:
        degree = collections.Counter(n for e in rest for n in e)
        n = min(degree, key=lambda n: (-degree[n], wichtig[n], n))
        chosen.add(n); rest = {e for e in rest if n not in e}

    for n in sorted(chosen - zwang, key=lambda n: (-wichtig[n], n)):
        if all(set(e) & (chosen - {n}) for e in kanten): chosen.remove(n)
    optimal = not kanten
    if nodes and solver:
        from scipy.optimize import milp, Bounds, LinearConstraint
        from scipy.sparse import lil_matrix
        ix = {n:i for i,n in enumerate(nodes)}; N = len(nodes)
        mx = max(wichtig.values(), default=1); base = mx + 2
        A = lil_matrix((len(kanten) + N, N + 1)); low = []; high = []
        for k, (a, b) in enumerate(sorted(kanten)):
            A[k, ix[a]] = A[k, ix[b]] = 1; low.append(1); high.append(np.inf)
        for i, n in enumerate(nodes):
            A[len(kanten)+i, i] = wichtig[n]; A[len(kanten)+i, N] = -1
            low.append(-np.inf); high.append(0)
        c = np.array([base + wichtig[n] / ((N+1)*(mx+1)) for n in nodes] + [1.0])
        result = milp(c, integrality=np.array([1]*N + [0]),
            bounds=Bounds(np.array([0]*N + [max((wichtig[n] for n in zwang), default=0)]), np.array([1]*N + [mx])),
            constraints=LinearConstraint(A.tocsr(), low, high), options=dict(node_limit=2000))
        def key(s): return (len(s), max((wichtig[n] for n in s), default=0), sum(wichtig[n] for n in s), sorted(s))
        if result.x is not None:
            s = zwang | {n for n,i in ix.items() if result.x[i] > .5}
            if all(set(e) & s for e in kanten) and key(s) <= key(chosen): chosen = s
        optimal = bool(result.success)
    return chosen, optimal


def ausgabe_luftlinien(board, netze):
    ""
    out = []
    for n in sorted(netze):
        ps = sorted({(p['x'], p['y']) for p in board['pads'] if p.get('net') == n})
        if not ps: continue


        best = {b: (math.dist(ps[0], b), ps[0], b) for b in ps[1:]}
        while best:
            _, a, b = min(best.values())
            out.append(dict(net=n, pts=[list(a),list(b)], grund='offen'))
            del best[b]
            for c in best: best[c] = min(best[c], (math.dist(b, c), b, c))
    return out


def _ausgabe_sicher(board, data, einheiten=None, wichtig=None, paare=(), checkpoint=False):
    ""
    import hashlib
    einheiten = einheiten or {}; wichtig = wichtig or {}
    owners = {q.get('einheit') for k in ('tracks', 'vias') for q in data.get(k, []) if q.get('einheit')}
    raw_sha = hashlib.sha256(json.dumps(dict(tracks=data['tracks'], vias=data['vias']), sort_keys=True).encode()).hexdigest()
    kontakte, geerbt = ausgabe_kontakte(board, data)
    kanten = set(); zwang = set(); fehler = list(kontakte)
    for e in kontakte:
        a,b = e.get('einheiten', e['netze']); aa,bb = e['arten']
        if aa == 'pad': zwang.add(b)
        elif bb == 'pad': zwang.add(a)
        else: kanten.add(tuple(sorted((a,b))))


    for L,d,a,b,x,y in paare:
        na,nb = einheiten.get(a),einheiten.get(b)
        if a.startswith('GND#') and na and a in owners: na = a
        if b.startswith('GND#') and nb and b in owners: nb = b
        if a == 'PAD' and nb: zwang.add(nb)
        elif b == 'PAD' and na: zwang.add(na)
        elif na and nb:
            if na == nb: zwang.add(na)
            else: kanten.add(tuple(sorted((na,nb))))
        else: continue
        fehler.append(dict(lage=L, netze=[na or a,nb or b], arten=['regel','regel'], ort=[x,y], abstand=d))
    betroffen = zwang | {n for e in kanten for n in e}
    wichtig = {n:int(wichtig.get(n, 1)) for n in betroffen}
    weg, optimal = ausgabe_auswahl(kanten, zwang, wichtig, solver=not checkpoint)
    if not weg and not geerbt:
        if data.get('failed'):
            result = dict(data)
            if not checkpoint: result['luftlinien'] = ausgabe_luftlinien(board, {einheiten.get(n,n) for n in data['failed']})
            return result
        return data
    result = dict(data)
    for k in ('tracks','vias','zonen','zones'):
        if k in data: result[k] = [q for q in data[k] if q.get('einheit', q['net']) not in weg]
    failed = set(data.get('failed', [])) | weg
    result['failed'] = sorted(failed)
    if not checkpoint: result['luftlinien'] = ausgabe_luftlinien(board, {einheiten.get(n,n) for n in failed})
    rest, _ = ausgabe_kontakte(board, result)
    if rest: raise ValueError('R44: Restkontakte nicht sicher aufloesbar')
    result['kurzschluss_schutz'] = dict(version=1, netze_entfernt=sorted({einheiten.get(u, u) for u in weg}), einheiten_entfernt=sorted(weg), fehler=fehler,
        eingang_pad_kontakte=geerbt, restkontakte=0, minimal_bewiesen=optimal,
        vor_bereinigung=dict(sha=raw_sha, bahnen=len(data['tracks']), vias=len(data['vias'])),
        paare_vor_bereinigung=list(paare))
    return result


def ausgabe_sicher(board, data, einheiten=None, wichtig=None, paare=(), checkpoint=False):
    ""
    units = dict(einheiten or {})
    bad, errors = set(), []
    work = dict(data)
    for key in ('tracks', 'vias', 'zonen', 'zones'):
        if key not in data: continue
        work[key] = []
        for i, obj in enumerate(data[key]):
            owner = obj.get('einheit') or obj.get('net') or '#unbekannt:%s:%d' % (key, i)
            units.setdefault(owner, obj.get('net') or owner)
            try:
                if not obj.get('net'): raise ValueError('Kupfer ohne Netz')
                if key == 'tracks':
                    q, _ = bahn_vorschau(obj)
                    if len(q['pts'])<2 or len(q['w'])!=len(q['pts'])-1 or not q.get('layer'): raise ValueError('ungueltige Bahnvorschau')
                    if not all(len(p)==2 and all(math.isfinite(v) for v in p) for p in q['pts']): raise ValueError('ungueltige Bahnpunkte')
                    if not all(math.isfinite(w) and w>0 for w in q['w']): raise ValueError('ungueltige Bahnbreite')

                    if 'items' in obj:
                        from gemeinsam.bahnitems import validate
                        try: validate(obj, error=.00025); q = obj
                        except ValueError: pass
                else: q = obj
                if key == 'vias':
                    if not all(math.isfinite(q[v]) for v in ('x','y')): raise ValueError('ungueltige Via-Position')
                    diameter = float(q.get('d',q.get('diameter',str(data.get('optionen',{}).get('via','.45/.20')).split('/')[0])))
                    if not math.isfinite(diameter) or diameter<=0: raise ValueError('ungueltiger Via-Durchmesser')
                if key in ('zonen','zones'):
                    ausgabe_kontakte(dict(pads=[], lagen=board.get('lagen',2)), dict(tracks=[],vias=[],**{key:[q]}))
                work[key].append(q)
            except Exception as exc:
                bad.add(owner); errors.append(dict(einheit=owner, grund=str(exc)[:180]))
    for key in ('tracks','vias','zonen','zones'):
        if key in work: work[key] = [q for q in work[key] if (q.get('einheit') or q.get('net')) not in bad]
    work['failed'] = sorted(set(data.get('failed', [])) | bad)
    try:
        result = _ausgabe_sicher(board, work, units, wichtig, paare, checkpoint)
    except Exception as exc:
        errors.append(dict(grund='Gesamtpruefung: '+str(exc)[:180]))
        result = dict(work)
        for key in ('tracks','vias','zonen','zones'):
            for q in result.get(key, []): bad.add(q.get('einheit') or q.get('net'))
            if key in result: result[key] = []
        result['failed'] = sorted(set(result['failed']) | bad)
    if errors:
        result = dict(result)
        info = dict(result.get('kurzschluss_schutz') or {})
        info.update(geometrie_fehler=errors, einheiten_unpruefbar=sorted(bad), restkontakte=0)
        info['netze_entfernt'] = sorted(set(info.get('netze_entfernt',[])) | {units.get(u,u) for u in bad})
        result['kurzschluss_schutz'] = info
    if not checkpoint and result.get('failed'):
        try: result['luftlinien'] = ausgabe_luftlinien(board, {units.get(n,n) for n in result['failed']})
        except Exception: result['luftlinien'] = []

    return data if result == data else result



def ausgabebelegung(data, lagen, origin, shape, raster=.25):
    ""
    from shapely.geometry import LineString, Point, Polygon
    from shapely import box, intersects
    result = np.zeros((len(lagen), *shape), bool)
    forms = []
    for t in data.get('tracks', []):
        t, error = bahn_vorschau(t)
        for a,b,w in zip(t['pts'],t['pts'][1:],t['w']):
            if _null(a, b): continue
            forms.append((t['layer'], LineString([a,b]).buffer(w/2+error)))
    diameter = float(str(data.get('optionen',{}).get('via','.45/.20')).split('/')[0])
    for v in data.get('vias', []):
        g = Point(v['x'],v['y']).buffer(float(v.get('d',v.get('diameter',diameter)))/2)
        forms.extend((L,g) for L in v.get('layers') or lagen)
    for z in data.get('zonen', [])+data.get('zones', []):
        if z.get('filled') is True: qs, ls = [Polygon(z['pts'],z.get('holes',[]))], [z['layer']]
        else:
            qs = [Polygon(q['aussen'],q.get('innen',[])) for q in z.get('polys',[])]
            qs += [Polygon(a,h) for a,h in z.get('filled',[])] if isinstance(z.get('filled'),list) else []
            if z.get('native_outlines'): qs = [Polygon(q['pts'],q.get('holes',[])) for q in z['native_outlines']]
            elif z.get('pts') and not qs: qs = [Polygon(z['pts'],z.get('holes',[]))]
            ls = z.get('native_layers') or [z['layer']]
        forms.extend((L,g) for L in ls for g in qs)
    for L,g in forms:
        if L not in lagen or g.is_empty: continue
        bounds = np.asarray(g.bounds).reshape(2,2)
        lo = np.maximum(0,np.floor((bounds[0]-origin-1e-9)/raster).astype(int))
        hi = np.minimum([shape[1],shape[0]],np.floor((bounds[1]-origin)/raster).astype(int)+1)
        if np.any(hi<=lo): continue
        yy,xx = np.mgrid[lo[1]:hi[1],lo[0]:hi[0]]
        x,y = origin[0]+xx*raster,origin[1]+yy*raster
        result[lagen.index(L),lo[1]:hi[1],lo[0]:hi[0]] |= intersects(g,box(x,y,x+raster,y+raster))
    return result.astype(np.float16)


def etiketten_sichern(board, routes, routes_path, etiketten=None):
    ""
    from pathlib import Path
    import hashlib, os, tempfile
    path = Path(routes_path)
    paths = {Path(etiketten)} if etiketten else {path.with_name(path.stem+'_etiketten.npz'), path.parent/'etiketten.npz'}
    if not etiketten and routes.get('optionen',{}).get('etiketten'):
        q = Path(routes['optionen']['etiketten'])

        if q.resolve().parent == path.resolve().parent: paths.add(q)
    for dest in sorted(paths):
        if not dest.exists(): continue
        try:
            with np.load(dest,allow_pickle=False) as z: d = {k:z[k] for k in z.files}
            layers = list(d['lagen']); origin = d['origin_mm']; shape = d['belegung'].shape[-2:]


            observer = int(d.get('beobachterversion', 1))



            d.setdefault('quell_beobachterversion', np.asarray(1 if observer == 4 else observer, np.int16))
            d.setdefault('belegung_suche', d['belegung'].copy())
            if 'belegung_maske' in d: d.setdefault('belegung_suche_maske', np.asarray(d['belegung_maske'], bool).copy())
            d['belegung'] = ausgabebelegung(routes,layers,origin,shape,float(d['raster_mm']))
            d['belegung_vertrag'] = np.asarray('ausgabekupfer_zellkontakt')
            d['etikettenversion'] = np.asarray(2,np.int16); d['beobachterversion'] = np.asarray(4,np.int16)
            d['ausgabe_routes_sha256'] = np.asarray(hashlib.sha256(path.read_bytes()).hexdigest())
            values = d['netz_werte'].copy(); lm = np.zeros_like(d['netz_lagen']); vm = np.zeros_like(d['vias_raster'])
            names = list(d['netze']); ix = {n:i for i,n in enumerate(names)}
            values[:,1] = 0; values[:,4] = 0
            failed = {q.get('net') or q.get('einheit') for key in ('tracks','vias') for q in routes.get(key,[]) if q.get('einheit') in routes.get('failed',[])} | set(routes.get('failed',[]))
            failed |= set((routes.get('kurzschluss_schutz') or {}).get('netze_entfernt',[]))
            for unit in routes.get('failed',[]):
                if unit.startswith('GND#'):
                    try: failed.add(board['pads'][int(unit[4:])].get('net'))
                    except (ValueError,IndexError): pass
            for n in failed:
                if n in ix: values[ix[n],0] = 0
            for t in routes.get('tracks',[]):
                t,_ = bahn_vorschau(t)
                if t['net'] in ix:
                    i = ix[t['net']]; values[i,1] += sum(math.dist(a,b) for a,b in zip(t['pts'],t['pts'][1:]))
                    if t['layer'] in layers: lm[i,layers.index(t['layer'])] = True
            for v in routes.get('vias',[]):
                if v['net'] in ix: values[ix[v['net']],4] += 1; lm[ix[v['net']],:] = True
                x,y = np.floor((np.array([v['x'],v['y']])-origin)/float(d['raster_mm'])).astype(int)
                if 0<=x<shape[1] and 0<=y<shape[0]: vm[:,y,x] += 1
            values[:,3] = np.divide(values[:,1],values[:,2],out=np.zeros(len(values)),where=values[:,2]>0)
            values[:,5] = lm.sum(1)
            d.update(netz_werte=values,netz_lagen=lm,vias_raster=vm)
            fd,tmp = tempfile.mkstemp(dir=dest.parent,suffix='.npz'); os.close(fd)
            try: np.savez_compressed(tmp,**d); os.replace(tmp,dest)
            finally:
                if os.path.exists(tmp): os.unlink(tmp)
        except Exception as exc:

            print('R44: Etiketten nicht synchronisierbar:',repr(exc)[:180])

            os.replace(dest,dest.with_name(dest.name+'.r44_unpruefbar'))


def _datei_sichern(board_path, routes_path, output_path=None, stand=None, daten=None):
    from pathlib import Path
    import os, tempfile
    stand = {} if stand is None else stand
    b = json.loads(Path(board_path).read_text())
    r = daten if daten is not None else json.loads(Path(routes_path).read_text())
    if not isinstance(r, dict) or not isinstance(b, dict): raise ValueError('Routen/Board kein JSON-Objekt')
    units = {q.get('einheit') or q.get('net'): q.get('net') for key in ('tracks', 'vias', 'zonen', 'zones') for q in r.get(key, []) if q.get('net')}
    units.update({'GND#%d' % i: p.get('net') for i, p in enumerate(b['pads'])})
    safe = ausgabe_sicher(b, r, units)
    dest = Path(output_path or routes_path)
    if safe is r and output_path is None:
        stand['geschrieben'] = True
        etiketten_sichern(b, safe, routes_path)
        return safe
    fd, tmp = tempfile.mkstemp(dir=dest.parent, prefix='.r44_')
    try:
        with os.fdopen(fd, 'w') as f: json.dump(safe, f)
        os.replace(tmp, dest)
    finally:
        if os.path.exists(tmp): os.unlink(tmp)
    stand['geschrieben'] = True
    if output_path is None: etiketten_sichern(b, safe, routes_path)
    return safe


def datei_sichern(board_path, routes_path, output_path=None, daten=None):
    ""
    from pathlib import Path
    import os
    path = Path(routes_path); stand = {}
    try: return _datei_sichern(board_path, routes_path, output_path, stand, daten)
    except Exception as exc:
        grund = 'R44: Ausgabe nicht gesichert: %s: %s' % (type(exc).__name__, str(exc)[:200])
        if output_path is None and not stand.get('geschrieben') and path.exists():
            gesperrt = path.with_name(path.name + '.r44_ungeprueft')
            try:
                os.replace(path, gesperrt)
                try: gesperrt.with_name(gesperrt.name + '.grund').write_text(grund)
                except OSError: pass
            except OSError as sperre: grund += '; Sperren fehlgeschlagen: %s' % str(sperre)[:80]
        raise R44Fehler(grund) from exc


def kandidat_sichern(board_path, routes_path):
    ""
    from pathlib import Path
    path = Path(routes_path)
    if not path.exists():
        gesperrt = path.with_name(path.name + '.r44_ungeprueft')
        if not gesperrt.exists(): return None
        try: return gesperrt.with_name(gesperrt.name + '.grund').read_text()[:300]
        except OSError: return 'R44: Ausgabe ungeprueft gesperrt (%s)' % gesperrt.name
    try: daten = json.loads(path.read_text())
    except Exception: daten = None
    if isinstance(daten, dict) and not schutz_an(daten.get('optionen')): return None
    try: datei_sichern(board_path, routes_path, daten=daten if isinstance(daten, dict) else None)
    except R44Fehler as exc: return str(exc)
    return None


def kandidat_sichern_messung(board_path, routes_path):
    ""
    import time
    start = time.thread_time()
    grund = kandidat_sichern(board_path, routes_path)
    return grund, dict(cpu_s=time.thread_time()-start, cpu_art='eigen_thread')


if __name__ == '__main__':
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    try: datei_sichern(*sys.argv[1:4])
    except R44Fehler as exc:
        print(exc, file=sys.stderr); sys.exit(1)
