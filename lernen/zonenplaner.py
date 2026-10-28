""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import sys
from typing import TypedDict

from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

ROOT = Path(__file__).resolve().parents[1]
SENSE = {'rueckkopplung', 'rueckkopplung_c', 'teiler', 'einstell_r'}


class Plan(TypedDict):
    version: str
    id: str
    netze: dict
    terminale: list
    kern: list
    wachstum: list
    via_kandidaten: list
    audit: dict


def sha(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True, allow_nan=False).encode()).hexdigest()


def pruefe_plan(plan):
    if plan.get('version') != 'zonenplan-lokal-v2' or plan.get('id') != sha(
            {k:v for k,v in plan.items() if k not in ('id','board_datei')}):
        raise ValueError('Zonenplan-Inhalt/Version geaendert')


def pruefe_eingaben(plan, board, klassen, regeln, optionen=None):
    ""
    pruefe_plan(plan)
    if plan.get('routing_board_sha256') != sha(board):
        raise ValueError('Zonenplan passt nicht zum Router-Boardvertrag')
    for name, obj in (('klassen', klassen), ('regeln', regeln)):
        if obj is None or plan.get('eingabehashes', {}).get(name) != sha(obj):
            raise ValueError('Zonenplan: aktueller '+name+'-Vertrag geaendert/fehlt')
    if not plan.get('native_geometrie'):
        raise ValueError('Lokaler Zonenplan braucht native Padgeometrie')
    if plan.get('board_datei'):
        native = json.loads(Path(plan['board_datei']).read_text())
        if sha(native) != plan['eingabehashes']['board']:
            raise ValueError('Zonenplan: native Padgeometrie geaendert')
    if optionen is not None:
        gnd = optionen.get('gndlagen', '').split(',') if optionen.get('gndlagen') else []
        if sorted(gnd) != sorted(plan['referenzschutz']['gndlagen']):
            raise ValueError('Zonenplan: GND-Kontext geaendert')


        from lernen.zonenplaner_vertrag import geometrie
        expected = geometrie(regeln, plan.get('kontext',{}).get('optionen'))
        actual = geometrie(regeln, dict(plan.get('kontext',{}).get('optionen') or {}, **{k:v for k,v in optionen.items() if k!='gndlagen'}))
        if expected != actual: raise ValueError('Zonenplan: Geometrieoptionen geaendert')



def abnahmetor(z, info, pflicht):
    ""
    accepted = (info or {}).get('angewendet') if pflicht == 'legacy' else (info or {}).get('native_abnahme')
    if pflicht and not accepted:
        z.update(gueltig=False, ungueltig_grund='R36: '+str(
            (info or {}).get('grund') or (info or {}).get('fehler') or 'native Abnahme fehlt'))
    if 'gueltig' in (z.get('regel') or {}):
        z['regel']['gueltig'] = z['gueltig']
    return z


def strom(v):
    ""
    f = v.get('fluss') or {}
    x = v.get('strom_A')
    source = v.get('strom_A_quelle')
    if x is None and f.get('einheit') == 'A':
        loads = list((f.get('last') or {}).values())
        if any(not isinstance(a, (int, float)) or not math.isfinite(a) or a < 0 for a in loads):
            return None, 'ungueltige Lasten'
        x, source = sum(loads), f.get('einheit_quelle', 'fluss_A')
    if x is None:
        return None, 'unbekannt'
    if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) or x < 0:
        return None, 'ungueltiger Strom'
    return float(x), source or 'strom_A ohne Herkunft'


def auswahl(K):
    out = {}
    for n, v in sorted(K.get('netze', {}).items()):
        amps, origin = strom(v)
        functional = any(t.get('entkopplung') and (t.get('stuetzt') or {}).get('netz') == n
                         for t in K.get('teile', {}).values())
        power = v.get('art') in ('versorgung', 'strom', 'sw')
        sense = v.get('kelvin') or n in K.get('gnd', []) or v.get('art') == 'gnd'
        selected = not sense and power and (functional or v.get('art') == 'sw' or amps is None or amps >= .01)
        reason = ('GND/Kelvin getrennt' if sense else 'Signal' if not power else
                  'Funktionspflicht' if functional or v.get('art') == 'sw' else
                  'Strom unbekannt, lokale Vorsorge' if amps is None else
                  'R36 Pflicht >=100mA' if amps >= .1 else
                  'R36 Praeferenz >=10mA' if amps >= .01 else 'Kleinstrom, Bahn erlaubt')
        out[n] = dict(gewaehlt=selected, art=v.get('art'), strom_A=amps, strom_quelle=origin,
                      grund=reason, funktionspflicht=functional, lokal_ohne_lagenwechsel=v.get('art') == 'sw')
    return out


def padid(p):
    return '%s:%s' % (p['ref'], p['num'])


def formpruefung(g):
    ""
    corners = []
    for p in getattr(g, 'geoms', [g]):
        if p.geom_type != 'Polygon':
            continue
        for ring in [p.exterior, *p.interiors]:
            xy = list(ring.coords)[:-1]
            for i, b in enumerate(xy):
                a, c = xy[i-1], xy[(i+1) % len(xy)]
                u, v = (b[0]-a[0], b[1]-a[1]), (c[0]-b[0], c[1]-b[1])
                den = math.hypot(*u)*math.hypot(*v)
                if den < 1e-14:
                    continue
                turn = math.degrees(math.acos(max(-1, min(1, (u[0]*v[0]+u[1]*v[1])/den))))
                if turn > 8.01:
                    corners.append(dict(xy=b, richtungswechsel_grad=round(turn, 3)))
    return dict(bestanden=not corners, freie_knicke=corners, sehnenwinkel_max_grad=8.01)


def _runden(g, frei, collar=.06, okt=False):
    from lernen.stromzonen import tangential_runden


    if okt:
        from lernen.zonenplaner_okt import huelle
        convex=huelle(g).buffer(max(.1,collar),quad_segs=48)
    else:convex=g.convex_hull.buffer(collar,quad_segs=48)
    if frei.covers(convex):return convex
    shape = g.buffer(collar, quad_segs=48).intersection(frei)
    for _ in range(3):
        shape = tangential_runden(shape.simplify(.0001, preserve_topology=True), Polygon(), frei, .12)
    return shape


def via_mitte_items(R):
    ""
    from gemeinsam.bahnitems import validate
    via_d=float(str((R.get('optionen') or {}).get('via') or '0.45/0.20').split('/')[0])
    changed=0
    for t in R['tracks']:
        if not t.get('items'):continue
        items=t['items'];pts=t['pts'];widths=t['w']
        first,last=items[0][1],items[-1][2 if items[-1][0]=='L' else 3]
        starts=[i for i,p in enumerate(pts) if math.dist(p,first)<1e-7]
        ends=[i for i,p in enumerate(pts) if math.dist(p,last)<1e-7]
        if not starts or not ends:continue
        lo,hi=starts[0],ends[-1]
        if lo>hi:continue
        additions=[]
        for i in list(range(lo))+list(range(hi,len(pts)-1)):
            from shapely.geometry import LineString
            stub=LineString([pts[i],pts[i+1]]).buffer(widths[i]/2,quad_segs=48)
            rings=[Point(v['x'],v['y']).buffer(v.get('d',v.get('diameter',via_d))/2,quad_segs=48)
                   for v in R['vias'] if v['net']==t['net']]




            endpoint=first if i<lo else last
            olditem=items[0] if i<lo else items[-1]
            endwidth=olditem[3] if olditem[0]=='L' else olditem[4]
            cap=Point(*endpoint).buffer(endwidth/2,quad_segs=48)
            if not any(g.covers(LineString([pts[i],pts[i+1]])) and
                       g.union(cap).buffer(.00025).covers(stub) for g in rings):
                raise ValueError('R37 Item-Erweiterung liegt nicht voll im Via-Kupfer')
            additions.append((i,['L',pts[i],pts[i+1],widths[i]]))
        if additions:
            t['items']=[it for i,it in additions if i<lo]+items+[it for i,it in additions if i>=hi]
            changed+=len(additions)
        validate(t,error=float((R.get('organisch_nachlauf') or {}).get('sehnenfehler_mm',.00025)))
    return changed


def planen(B, K, RG, kontext=None) -> Plan:
    ""
    from lernen import stromzonen as SZ
    ctx = dict(kontext or {})
    op = dict((RG or {}).get('router') or {}, **(ctx.get('optionen') or {}))
    from lernen.zonenplaner_vertrag import geometrie
    normalized = geometrie(RG, op)
    if int(normalized['lagen']) != int(B['lagen']):
        raise ValueError('Plan/Router-Lagen verschieden; lagen explizit in Regeln angeben')
    op = dict(op, **{k:v for k,v in normalized.items() if k!='via'}, via='/'.join(map(str,normalized['via'])))
    layers = SZ._lagen(B)
    clr = max(float(op.get('clr') or .1), float(op.get('drcpad') or 0),
              float((B.get('design') or {}).get('min_abstand') or 0)) + .004
    edge = float(op['edge']) + .036
    drill_gap = max(float(op.get('h2h') or .2), float((B.get('design') or {}).get('loch_loch') or 0))
    vd, drill = SZ._via_masse(dict(optionen=op))
    vd = max(vd, float((B.get('design') or {}).get('via_min') or 0),
             drill + 2*float((B.get('design') or {}).get('ring_min') or 0))
    drill = max(drill, float((B.get('design') or {}).get('bohr_min') or 0))
    vd = max(vd, drill+2*float((B.get('design') or {}).get('ring_min') or 0))
    hole_clr = max(float(op.get('loch_clr') or 0), float((B.get('design') or {}).get('loch_abstand') or 0))
    from analyse.flaechen import umriss
    board = umriss(B).buffer(-edge)
    fps = {f['ref']: f for f in B.get('fps', [])}
    selected = auswahl(K)
    plan = Plan(version='zonenplan-lokal-v2', id='', netze=selected, terminale=[], kern=[],
                wachstum=[], via_kandidaten=[], audit=dict(ablehnungen=[], form=[], nativ_bestaetigt=False))
    plan.update(kontext=ctx, eingabehashes=dict(board=sha(B), klassen=sha(K), regeln=sha(RG), kontext=sha(ctx)),
                padgeometrie=ctx.get('padgeometrie', 'deklarierte Exportpolygone'),
                native_geometrie=bool(ctx.get('native_geometrie')), startabstand_mm=1.5,
                referenzschutz=dict(gndlagen=ctx.get('gndlagen', []), umwidmung=False),
                elektrische_budgets=copy.deepcopy((K.get('vorgaben') or {}).get('budgets', {})))
    occupied = {L: [] for L in layers}
    vias = []
    def rejected(n, L, why, **extra):
        plan['audit']['ablehnungen'].append(dict(net=n, layer=L, grund=why, **extra))
    for n in sorted(selected, key=lambda n: (-(selected[n]['strom_A'] or 0), n)):
        info = selected[n]
        if not info['gewaehlt']:
            continue
        v = K['netze'][n]
        f = v.get('fluss') or {}
        sense = {q['pad'] for q in K.get('abgriffe', {}).get(n, [])}
        pads = sorted([p for p in B['pads'] if p['net'] == n], key=lambda p: (padid(p), p['x'], p['y']))
        sense |= {padid(p) for p in pads if K.get('teile',{}).get(p['ref'],{}).get('funktion') in SENSE
                  or any(x in str(p.get('pinfunc') or '').upper() for x in ('SENSE','FB','VSNS','ISNS'))}
        local_layers = set.intersection(*(set(p.get('layers', [])) for p in pads)) if pads else set()
        if info['lokal_ohne_lagenwechsel']:
            common = sorted(local_layers & {'F', 'B'})
            info['lokale_lage'] = common[0] if common else None
            if not common:
                rejected(n, None, 'SW/BST ohne gemeinsame Aussenlage')
        for L in ('F', 'B'):
            entries = []
            for p in pads:
                if L not in p.get('polys', {}):
                    continue
                ti = K.get('teile', {}).get(p['ref'], {})
                is_sense = padid(p) in sense or ti.get('funktion') in SENSE or any(
                    x in str(p.get('pinfunc') or '').upper() for x in ('SENSE', 'FB', 'VSNS', 'ISNS'))
                bga = 'BGA' in str(fps.get(p['ref'], {}).get('fp', '')).upper()
                support = ti.get('stuetzt') or {}
                group = support.get('ref') if support.get('netz') == n else ti.get('insel') or p['ref']
                if not group:
                    group = p['ref']
                if is_sense:
                    rejected(n, L, 'Sense-Anker bleibt Bahn', pad=padid(p))
                pg = unary_union([Polygon(q).buffer(0) for q in p['polys'][L]])
                terminal = dict(id='%s:%s:%d' % (padid(p), L, len(plan['terminale'])), net=n, layer=L,
                                pad=padid(p), ref=p['ref'], xy=[p['x'], p['y']], gruppe=group,
                                quelle=padid(p) in f.get('quelle', []), last_A=(f.get('last') or {}).get(padid(p)) if f.get('einheit') == 'A' else None,
                                sense=is_sense, bga=bga, polys=SZ._polys(pg))
                plan['terminale'].append(terminal)
                if not is_sense:
                    entries.append((terminal, pg))
            if not entries:
                continue

            obstacles = [Polygon(q).buffer(clr, quad_segs=48) for p in B['pads']
                         if p['net'] != n or padid(p) in sense or K.get('teile', {}).get(p['ref'], {}).get('funktion') in SENSE
                         for q in p.get('polys', {}).get(L, [])]
            obstacles += [Point(p['x'], p['y']).buffer(p['drill']/2+hole_clr+.004, quad_segs=48)
                          for p in B['pads'] if p.get('drill') and p['net'] != n]
            obstacles += [g.buffer(clr, quad_segs=48) for m, g in occupied[L] if m != n]
            obstacles += [SZ.sperrflaeche(B, L, 'flaechen'), SZ.sperrflaeche(B, L, 'bahnen')]
            free = board.difference(unary_union(obstacles))

            normal = [(t, g) for t, g in entries if not t['bga']]

            free=free.difference(unary_union([g.buffer(clr,quad_segs=48) for t,g in entries if t['bga']]))
            for t, _ in entries:
                if t['bga']:
                    rejected(n, L, 'BGA Einzelball/Fanout, keine Pad-Huelle', pad=t['pad'])
            groups = []
            for t, g in normal:
                merged = False
                for members, geometries, bridges in groups:
                    if members[0]['gruppe'] != t['gruppe']:
                        continue
                    choices = sorted([(g.distance(q), j) for j, q in enumerate(geometries)])
                    for dist, j in choices:
                        if dist > 1.5:
                            break
                        pair = g.union(geometries[j]).convex_hull
                        foreign_groups=unary_union([h.buffer(clr) for u,h in normal if u['gruppe']!=t['gruppe']])
                        if free.covers(pair.buffer(.06, quad_segs=48)) and not pair.intersects(foreign_groups):
                            members.append(t); geometries.append(g); bridges.append(pair); merged = True; break
                    if merged:
                        break
                if not merged:
                    groups.append(([t], [g], []))
            for members, geometries, bridges in groups:
                body = unary_union(geometries+bridges)
                group_free=free.difference(unary_union([g.buffer(clr,quad_segs=48) for t,g in normal
                                                       if t['gruppe']!=members[0]['gruppe']]))
                core = _runden(body, group_free, okt=ctx.get('etappe3',False))
                manufacturing = max(float((B.get('design') or {}).get('min_breite') or 0),
                                    float((RG or {}).get('roh', {}).get('min_track_width') or 0), float(op.get('bahn_min') or 0))
                minimum = max(manufacturing, .25 if info['art'] == 'sw' or core.area >= 20 else .1)
                audit = formpruefung(core)
                if (core.is_empty or not core.buffer(1e-6).covers(body) or not audit['bestanden']
                        or len(SZ._polys(core.buffer(-minimum/2))) != 1):
                    rejected(n, L, 'Lokaler Kern: Anschluss/Form/Mindestbreite nicht erfuellbar', pads=[t['pad'] for t in members])
                    continue
                ident = '%s:%s:%d' % (n, L, len(plan['kern']))
                zone = dict(net=n, layer=L, polys=SZ._polys(core), quelle='zonenplaner', terminale=[t['id'] for t in members],
                            kern_id=ident, min_thickness=minimum, form=audit, r36=True)
                occupied[L].append((n, core)); plan['kern'].append(zone)
                plan['wachstum'].append(dict(net=n, layer=L, polys=SZ._polys(core.buffer(.25).intersection(free)), quelle='zonenplaner-wachstum'))
                plan['audit']['form'].append(dict(kern_id=ident, **audit))
                if info['lokal_ohne_lagenwechsel'] or int(B['lagen']) < 4:
                    continue
                amps = info['strom_A'] if any(t['quelle'] for t in members) else sum(t['last_A'] or 0 for t in members)
                if not amps:
                    amps = info['strom_A']
                rule = max(2, math.ceil(4*amps)) if amps is not None else 2
                physics = math.ceil(1.3*amps/(drill/.3)) if amps is not None and drill > 0 else None
                demand = max(rule, physics or 0)
                field = dict(net=n, layer=L, kern_id=ident, strom_A=amps, regel=rule, physikbedarf=physics,
                             soll=demand, vias=[], raster_mm=.5, angenommen=False)
                from shapely.ops import nearest_points


                field_free=free.difference(unary_union([h.buffer(clr) for u,h in normal if u not in members]))
                search=core.buffer(1.5).intersection(field_free)
                x0, y0, x1, y1 = search.bounds
                center = core.representative_point()
                additions=[]
                candidates = [(math.dist((x*.5, y*.5), (center.x, center.y)), x*.5, y*.5)
                              for x in range(math.ceil(x0/.5), math.floor(x1/.5)+1)
                              for y in range(math.ceil(y0/.5), math.floor(y1/.5)+1)]
                hints=[(0,v['x'],v['y']) for f in B.get('kupferreserve',[]) if f['net']==n and any(t['ref']==f['ref'] for t in members) for v in f['vias']]
                for _, x, y in hints+sorted(candidates):
                    pt = Point(x, y); disk = pt.buffer(vd/2, quad_segs=48)
                    if any(math.dist((x,y),(q['x'],q['y'])) < 1e-6 for q in field['vias']):continue
                    if not field_free.covers(disk.buffer(.06)) or any(math.dist((x,y),(q['x'],q['y'])) < (drill+q['drill'])/2+drill_gap+.004 for q in vias+field['vias']):
                        continue
                    a,b=nearest_points(core,disk)
                    from shapely.geometry import LineString
                    bridge=LineString([a,b]).buffer(max(.25,minimum)/2,quad_segs=48) if a.distance(b)>1e-9 else disk
                    extension=disk.union(bridge).buffer(.06,quad_segs=48)
                    if not field_free.covers(extension):continue
                    legal = True
                    for lay in layers:
                        if disk.buffer(clr).intersects(SZ.sperrflaeche(B, lay, 'vias')):
                            legal = False; break
                        for p in B['pads']:
                            ps = unary_union([Polygon(q) for q in p.get('polys', {}).get(lay, [])])
                            if p['net'] != n and disk.distance(ps) < clr:
                                legal = False; break
                            if p.get('drill') and pt.distance(Point(p['x'],p['y'])) < (drill+p['drill'])/2+drill_gap+.004:
                                legal = False; break
                            if float(op.get('vip', 0)) != 1 and lay in ('F','B') and disk.distance(ps) < .1:
                                legal = False; break
                        if any(m != n and disk.distance(g) < clr for m,g in occupied[lay]):
                            legal = False
                        if not legal:
                            break
                    if legal:
                        field['vias'].append(dict(net=n,x=x,y=y,d=vd,drill=drill,layers=layers,quelle='zonenplaner',kern_id=ident))
                        additions.append(extension)
                    if len(field['vias']) == demand:
                        break
                field['ist'] = len(field['vias']); field['angenommen'] = field['ist'] == demand
                if field['angenommen'] and additions:
                    enriched=_runden(unary_union([core]+additions),field_free,collar=.01,okt=ctx.get('etappe3',False))
                    shapeaudit=formpruefung(enriched)
                    if not enriched.buffer(1e-6).covers(body) or not shapeaudit['bestanden']:
                        field['angenommen']=False
                    else:
                        core=enriched;zone.update(polys=SZ._polys(core),form=shapeaudit)
                        occupied[L][-1]=(n,core)
                        plan['wachstum'][-1]['polys']=SZ._polys(core.buffer(.25).intersection(field_free))
                plan['via_kandidaten'].append(field)
                if field['angenommen']:
                    vias += field['vias']
                    for lay in layers:
                        occupied[lay].extend((n, Point(q['x'],q['y']).buffer(vd/2)) for q in field['vias'])
                else:
                    rejected(n, L, 'Via-Feld Platznot', kern_id=ident, soll=demand, ist=field['ist'])
    plan['audit'].update(kerne=len(plan['kern']), terminale=len(plan['terminale']),
                         via_felder=sum(f['angenommen'] for f in plan['via_kandidaten']),
                         strom_unbekannt=[n for n,i in selected.items() if i['gewaehlt'] and i['strom_A'] is None],
                         etappe=2, innenkorridore=False, lastpad_J='im Abschluss getrennt erforderlich')
    if ctx.get('etappe3'):
        from lernen.zonenplaner_korridor import ergaenzen
        ergaenzen(B,K,RG,plan,board,clr)
    plan['id'] = sha({k:v for k,v in plan.items() if k != 'id'})
    return plan


def vorbereiten(bj, kj, regeln, modus, gndlagen, cpu_s=120):
    ""
    from gemeinsam import plaetze as PL
    if modus not in ('diagnose', 'lokal'):
        raise ValueError('zonenplan=diagnose|lokal')
    folder = Path(bj).parent/'zonenplan'; folder.mkdir(parents=True, exist_ok=True)
    out = folder/'zonenplan.json'; out.unlink(missing_ok=True)
    cmd = ['caffeinate','-i','-s',sys.executable,'-m','lernen.zonenplaner','plan',str(Path(bj).resolve()),
           str(Path(kj).resolve()),str(Path(regeln).resolve()),str(out.resolve()),modus,','.join(gndlagen)]
    run = PL.lauf(cmd, cpu_s=cpu_s, log=str(folder/'plan.log'), cwd=str(ROOT), env=dict(os.environ,PYTHONHASHSEED='0'))
    info = dict(modus=modus, rechenplatz=run, datei=str(out), angewendet=run['zustand']=='ok' and out.exists())
    if info['angewendet']:
        plan = json.loads(out.read_text()); info.update(id=plan['id'],audit=plan['audit'])
    return info


def abschluss(bj, kj, rj, regeln, gndlagen, modus='nativ', cpu_s=720):
    ""
    from lernen import stromzonen as SZ
    if modus not in ('legacy', 'nativ', 'lokal', 'geometrisch'): raise ValueError('r36=legacy|nativ|geometrisch|0')
    plan = str(Path(bj).parent/'zonenplan/zonenplan.json') if modus == 'lokal' else None
    if modus == 'geometrisch':


        info = SZ._geometrisch(bj,kj,rj,gndlagen,cpu_s=cpu_s)
        info.update(ersatz='geometrisch', native_abnahme=False, nativ_grund='nicht versucht (r36=geometrisch)')
        modus = 'legacy'
    else:
        info = SZ.anwenden(bj,kj,rj,gndlagen,regeln=regeln,cpu_s=cpu_s,zonenplan=plan)
        info['native_abnahme'] = bool(info.get('angewendet'))
    if modus == 'legacy':


        if not info['native_abnahme'] and info.get('ersatz') != 'geometrisch':
            nat = info

            native_run = nat.get('rechenplatz') or {}
            rest = max(0., cpu_s - native_run.get('cpu_s', 0.))
            if rest < 1:
                return dict(nat, angewendet=False, grund='R36-Ersatz: CPU-Budget erschoepft')
            geo = SZ._geometrisch(bj,kj,rj,gndlagen,cpu_s=rest)
            geo_run = geo.get('rechenplatz') or {}
            info = dict(geo, native_abnahme=False,
                        rechenplatz_nativ=native_run, rechenplatz_geometrisch=geo_run,
                        rechenplatz=dict(geo_run, cpu_s=native_run.get('cpu_s',0.) + geo_run.get('cpu_s',0.)),
                        nativ_grund=str(nat.get('grund') or nat.get('fehler'))[:200])
            if not geo.get('angewendet'): return info
            info['ersatz'] = 'geometrisch'
        if not info.get('angewendet'): return info
        B,K,R = [json.loads(Path(p).read_text()) for p in (bj,kj,rj)]
        failures = SZ.pruefen(B,R,K,gndlagen)
        if failures:
            weg = {(n,L) for n,L,*_ in failures}
            R['zonen'] = [q for q in R.get('zonen',[]) if (q['net'],q['layer']) not in weg]
            if any(n in set(K.get('gnd') or []) for n,_ in weg):
                R['vias'] = [v for v in R['vias'] if v.get('quelle') != 'gnd_fuellung']
                R['zonen'] = [q for q in R['zonen'] if q.get('quelle') != 'gnd_fuellung']
            Path(rj).write_text(json.dumps(R)); info['verworfen'] = failures[:8]
        return info
    if not info['native_abnahme']:

        target = str(Path(rj).with_suffix(''))+'_geometrischer_ersatz.json'
        try:
            native_run = info.get('rechenplatz') or {}
            rest = max(0., cpu_s - native_run.get('cpu_s',0.))
            if rest < 1: return dict(info, ersatz_gesperrt='cpu_budget')
            info['geometrie'] = SZ._geometrisch(bj,kj,rj,gndlagen,aus=target,cpu_s=rest)
            geo_run = info['geometrie'].get('rechenplatz') or {}
            info['rechenplatz_nativ'] = native_run
            info['rechenplatz_geometrisch'] = geo_run
            info['rechenplatz'] = dict(geo_run, cpu_s=native_run.get('cpu_s',0.) + geo_run.get('cpu_s',0.))
            if info['geometrie'].get('angewendet'):
                info['ersatz'] = 'geometrisch'
                info['ersatz_datei'] = target
            else:
                info['ersatz_gesperrt'] = info['geometrie'].get('ersatz_gesperrt')
        except Exception as exc:
            info['ersatz_fehler'] = str(exc)
    return info


def main():
    _, bj,kj,rg,out,mode,gnd = sys.argv[1:]
    from lernen.kupfer_nativ import bibliotheksgeometrie
    B,K,RG = [json.loads(Path(p).read_text()) for p in (bj,kj,rg)]
    routing_hash = sha(B)
    folder = Path(out).parent
    B, geom = bibliotheksgeometrie(folder/'bibliothek',B,dict(optionen=RG.get('router',{})),RG)
    Path(folder/'native_board.json').write_text(json.dumps(B))
    plan = planen(B,K,RG,dict(modus=mode,gndlagen=gnd.split(',') if gnd else [],native_geometrie=True,
                             padgeometrie=geom.get('quelle','nativer Boardexport'),gndnetze=K.get('gnd',[]),etappe3=mode=='lokal' and int(B['lagen'])>=6))
    plan['board_datei'] = str(folder/'native_board.json')
    plan['routing_board_sha256'] = routing_hash
    plan['id'] = sha({k:v for k,v in plan.items() if k not in ('id','board_datei')})
    Path(out).write_text(json.dumps(plan,indent=1,ensure_ascii=False))


if __name__ == '__main__':
    main()
