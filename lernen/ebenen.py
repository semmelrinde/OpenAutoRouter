""
import json
import os
import subprocess
import re
from pathlib import Path
from shapely.geometry import Polygon
from shapely.ops import unary_union
from gemeinsam import plaetze as PL


def lagenzahl(d):
    ""
    p = Path(d)
    if p.is_dir(): p = p / 'processed.kicad_pcb'
    text = p.read_text(errors='replace').split('(setup', 1)[0]
    layers = re.search(r'\(layers\s+(.*?)\n\s*\)', text, re.S)
    if not layers: raise ValueError('Kupferstapel fehlt: ' + str(p))
    n = len(re.findall(r'\(\s*\d+\s+(?:"[^"]+"|[^\s()]+)\s+(?:signal|power|mixed|jumper)\b', layers[1]))
    if n < 2 or n % 2: raise ValueError('Unvollstaendiger Kupferstapel: ' + str(p))
    return n


def erkennen(B, K, data):
    ""
    Z = data['zonen']; U = unary_union([Polygon(q).buffer(0) for q in B['outline'] if len(q) >= 3])
    if U.area <= 0: raise ValueError('Umriss ohne Flaeche')
    layers, details = [], []
    for lay in sorted({z['layer'] for z in Z}, key=lambda l: int(l[2:])):
        zz = [z for z in Z if z['layer'] == lay]
        nets = sorted({z['net'] for z in zz})
        q = unary_union([Polygon(z['pts'], z.get('holes') or []).buffer(0) for z in zz])
        cover = q.intersection(U).area / U.area
        foreign = sorted({t['net'] for t in data.get('tracks', []) if t['layer'] == lay and t['net'] and t['net'] not in nets})
        kind = (K.get('netze', {}).get(nets[0], {}).get('art') if len(nets) == 1 else None)
        gnd = len(nets) == 1 and nets[0] in (K.get('gnd') or []) and not K.get('netze', {}).get(nets[0], {}).get('kelvin')
        reserve = len(nets) == 1 and (gnd or kind in ('versorgung', 'strom')) and cover >= .9 and not foreign
        if reserve: layers.append(lay + '=' + nets[0])
        details.append(dict(layer=lay, nets=nets, outline_fraction=cover, foreign_track_nets=foreign, reserve=reserve,
                            evidence='outline_only', grund='ein Netz, >=90%, keine fremden Bahnen' if reserve else 'Teilflaeche/gemischt/fremde Bahn'))
    return layers, Z, details


def aus_original(pcb, B, K, out, nativ=False):
    if not nativ: return _aus_original_alt(pcb, B, K, out)
    p = os.path.join(out, 'ebenen_original.json')
    Path(p).unlink(missing_ok=True)
    r = PL.lauf(['/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3', os.path.join(os.path.dirname(__file__), 'ebenen_export.py'), pcb, p, 'native_ebenen=1'],
                cpu_s=300, log=os.path.join(out, 'ebenen_export.log'))
    if r['zustand'] != 'ok' or not os.path.exists(p): raise RuntimeError('Ebenenexport: ' + repr(r))
    data = json.load(open(p))

    refs = {f['ref'] for f in B['fps']}
    expected = sorted([p['ref'], p['num'], p['net'], round(p['x'], 3), round(p['y'], 3)] for p in B['pads'] if p['ref'] in refs)
    original = sorted(data['pads'])
    if len(original) != len(expected) or any(a[:3] != b[:3] or max(abs(a[3]-b[3]), abs(a[4]-b[4])) > 0.002001 for a,b in zip(original,expected)):
        raise ValueError('Original passt nicht zum Uebungsboard (Pads/Netze/Positionen, Toleranz 2 um)')
    if data.get('lagen') != B.get('lagen'): raise ValueError('Original hat anderen Kupferstapel')
    if B.get('lagen') in (6, 8):
        layers, zones, details = erkennen(B, K, data)
        json.dump(dict(lagen=B['lagen'], gndlagen=layers, layers=details), open(os.path.join(out, 'ebenen.json'), 'w'), indent=1)
        return layers, zones
    Z = data['zonen']; U = unary_union([Polygon(q) for q in B['outline'] if len(q) >= 3])
    gnd = set(K.get('gnd') or []); layers = []
    for lay in sorted({z['layer'] for z in Z}):
        qs = [Polygon(z['pts'], z.get('holes') or []).buffer(0) for z in Z if z['layer'] == lay and z['net'] in gnd]
        if qs and unary_union(qs).intersection(U).area >= 0.9 * U.area: layers.append(lay)

    layers = [l for l in layers if not any(z['layer'] == l and z['net'] not in gnd for z in Z)]
    return layers, [z for z in Z if z['layer'] in layers and z['net'] in gnd]


def _drc_nativ(pcb, KO, fest=None, neu_fuellen=False):
    ""
    from kicad.python_aufruf import python_befehl
    js = pcb + '.drc.json'; Path(js).unlink(missing_ok=True)


    fill = ['--refill-zones', '--save-board'] if neu_fuellen else []
    from kicad.projekt_kopie import regeln_sichern, regeln_kopieren
    rules = regeln_sichern(pcb)
    r = PL.lauf([KO.KC, 'pcb', 'drc', '--format', 'json', '--severity-error', '-o', js, pcb] + fill,
                cpu_s=300, log=pcb + '.drc.log')
    if r['zustand'] != 'ok' or not Path(js).exists(): return dict(fehler=r)
    if neu_fuellen: regeln_kopieren(rules, pcb, erzeugtes_pro=True)
    geo = PL.lauf(python_befehl(Path(KO.R, 'kicad/drc_geometrie.py'), pcb, js),
                  cpu_s=300, log=pcb + '.geometrie.log')
    if geo['zustand'] != 'ok': return dict(fehler=geo)
    result = KO.drc_auswerten(json.loads(Path(js).read_text()), fest, nativ=True)
    result['ebenen_cpu_s'] = r['cpu_s'] + geo['cpu_s']
    return result


def original_drc(pcb, out, KO):
    ""
    import shutil
    from kicad.projekt_kopie import regeln_sichern, regeln_kopieren
    shutil.copy2(pcb, out)
    regeln_kopieren(regeln_sichern(pcb), out)
    return _drc_nativ(out, KO)


def k12(B, data):
    ""
    from shapely.geometry import box
    out = []
    for f in B.get('fps', []):
        if not any(k in (f.get('fp', '') + f.get('lib', '')).upper() for k in ('BGA', 'CSP')): continue
        pads = [p for p in B['pads'] if p['ref'] == f['ref']]
        if not pads: continue
        field = box(min(p['bbox'][0] for p in pads), min(p['bbox'][1] for p in pads), max(p['bbox'][2] for p in pads), max(p['bbox'][3] for p in pads))
        for layer, net in sorted({(z['layer'], z['net']) for z in data['zonen']}):
            polys = [Polygon(q, h).buffer(0) for z in data['zonen'] if (z['layer'], z['net']) == (layer, net) for q, h in z.get('filled', []) if len(q) >= 3]
            if not polys:
                out.append(dict(ref=f['ref'], layer=layer, net=net, unbekannt='keine gespeicherte Fuellung')); continue
            G = unary_union(polys).intersection(field)
            pieces = [q for q in getattr(G, 'geoms', [G]) if q.geom_type == 'Polygon' and q.area > .0001]
            out.append(dict(ref=f['ref'], layer=layer, net=net, teile=len(pieces), kupfer_mm2=round(G.area, 4), groesster_mm2=round(max((q.area for q in pieces), default=0), 4)))
    return out


def drc(pcb, routes, out, info, RG, fest, KO, nativ=False):
    if not nativ: return _drc_alt(pcb, routes, out, info, RG, fest, KO)
    from kicad.python_aufruf import python_befehl
    zj = out + '.zonen.json'; Path(zj).write_text(json.dumps(info.get('gndzonen') or []))
    via = json.loads(Path(routes).read_text()).get('optionen', {}).get('via', info['via'])
    Path(out).unlink(missing_ok=True)
    cmd = python_befehl(Path(KO.R, 'kicad/import_routes.py'), pcb, routes, zj, out,
                       'lagen=%d' % info['lagen'], 'via=' + via,
                       'zclr=%g' % RG['router'].get('clr', 0.2), 'projekt=1', 'fuellen=0')
    r = PL.lauf(cmd, cpu_s=300, log=out + '.import.log')
    if r['zustand'] != 'ok' or not os.path.exists(out): return dict(fehler='Ebenenimport: ' + repr(r))
    result = _drc_nativ(out, KO, fest, neu_fuellen=True)
    if result.get('fehler'): return result
    from analyse.bewertung import kupfer_fingerabdruck
    result['routes_sha256'] = kupfer_fingerabdruck(json.loads(Path(routes).read_text()))
    result['ebenen_cpu_s'] += r['cpu_s']

    ep = out + '.ebenen.json'; Path(ep).unlink(missing_ok=True)
    rr = PL.lauf(python_befehl(Path(__file__).with_name('ebenen_export.py'), out, ep, 'native_ebenen=1'),
                 cpu_s=300, log=out + '.ebenen.log')
    if rr['zustand'] != 'ok' or not os.path.exists(ep): result['ebenen_fehler'] = rr
    else:
        B = json.loads(Path(routes).with_name('board.json').read_text())
        result['k12'] = k12(B, json.loads(Path(ep).read_text()))
        result['ebenen_cpu_s'] += rr['cpu_s']
    return result



def _aus_original_alt(pcb, B, K, out):
    p = os.path.join(out, 'ebenen_original.json')
    r = subprocess.run(['/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3', os.path.join(os.path.dirname(__file__), 'ebenen_export.py'), pcb, p],
                       capture_output=True, text=True, timeout=300)
    with open(os.path.join(out, 'ebenen_export.log'), 'a') as f:
        f.write(pcb + '\n' + r.stdout + r.stderr + '\n')
    if r.returncode != 0 or not os.path.exists(p): raise RuntimeError('Ebenenexport: ' + r.stderr[-500:])
    data = json.load(open(p))
    expected = sorted([p['ref'], p['num'], p['net'], round(p['x'], 3), round(p['y'], 3)] for p in B['pads'])
    original = sorted(data['pads'])
    if len(original) != len(expected) or any(a[:3] != b[:3] or max(abs(a[3]-b[3]), abs(a[4]-b[4])) > 0.002001 for a,b in zip(original,expected)):
        raise ValueError('Original passt nicht zum Uebungsboard (Pads/Netze/Positionen, Toleranz 2 um)')
    Z = data['zonen']; U = unary_union([Polygon(q) for q in B['outline'] if len(q) >= 3])
    gnd = set(K.get('gnd') or []); layers = []
    for lay in sorted({z['layer'] for z in Z}):
        qs = [Polygon(z['pts'], z.get('holes') or []).buffer(0) for z in Z if z['layer'] == lay and z['net'] in gnd]
        if qs and unary_union(qs).intersection(U).area >= 0.9 * U.area: layers.append(lay)

    layers = [l for l in layers if not any(z['layer'] == l and z['net'] not in gnd for z in Z)]
    return layers, [z for z in Z if z['layer'] in layers and z['net'] in gnd]


def _drc_alt(pcb, routes, out, info, RG, fest, KO):
    zj = out + '.zonen.json'; json.dump(info['gndzonen'], open(zj, 'w'))
    via = json.load(open(routes)).get('optionen', {}).get('via', info['via'])
    cmd = [KO.KPY, os.path.join(KO.R, 'kicad/import_routes.py'), pcb, routes, zj, out,
           'lagen=%d' % info['lagen'], 'via=' + via, 'zclr=%g' % RG['router'].get('clr', 0.2)]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
    if r.returncode: return dict(fehler='Ebenenimport: ' + r.stderr[-300:])
    js = out + '.drc.json'
    r = subprocess.run([KO.KC, 'pcb', 'drc', '--format', 'json', '--severity-error', '-o', js, out], capture_output=True, text=True, timeout=300)
    if not os.path.exists(js): return dict(fehler='Ebenen-DRC: ' + r.stderr[-300:])
    return KO.drc_auswerten(json.load(open(js)), fest)
