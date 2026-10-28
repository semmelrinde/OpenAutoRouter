""
import collections
import gzip
import hashlib
import json
import math
import re
import time
from pathlib import Path

from lernen.gutachten_daten import kanonisch

VERSION = 'blind-geometrie-v3'
ANWEISUNG_V2 = '''Bewerte ausschliesslich die vorgelegten anonymen Platinenbilder nach lernen/gutachten_regeln.md.
Lies keine Kennzahlen, Galerie, Stichproben, bisherigen Noten oder Geometriedateien.
Note 1 bis 10 (hoeher besser): Inseln/Entkopplung P2/P3, Dichte P26, kurze klare Signalwege R19/R24,
Y/T-Abzweige R27, Stromketten R11/R22, kompakte glatte Stromzonen R36/R39/K17/K20.
Zusaetzlich strom_note 1 bis 10, null ohne sichtbare Strompfade. Unbekanntes nicht als gemessen behandeln.
Antworte nur JSON: {"noten":[{"bild":"<Alias>","note":8,"strom_note":7,"regeln":["R19"],"kurz":"Begruendung"}],"schwaechen":[]}.
Bilder:\n'''


ANWEISUNG = '''Bewerte ausschliesslich die vorgelegten anonymen Platinenbilder nach lernen/gutachten_regeln.md.
Lies keine Kennzahlen, Galerie, Stichproben, bisherigen Noten oder Geometriedateien.
Note 1 bis 10 (hoeher besser): Inseln/Entkopplung P2/P3, Dichte P26, kurze klare Signalwege R19/R24,
Y/T-Abzweige R27, Stromketten R11/R22, kompakte glatte Stromzonen R36/R39/K17/K20.
Sich kreuzende oder ueberlappende Bahnen verschiedener Netze auf derselben Lage sind Kurzschluesse (R44): sehr schlecht.
Zusaetzlich strom_note 1 bis 10 nur fuer Strompfade und Stromzonen (kompakt, glatt, ohne Kerben, Kleckse, Stummel,
Spruenge dick/duenn, ohne unnoetige Engstellen), null ohne sichtbare Strompfade. Unbekanntes nicht als gemessen behandeln.
Antworte nur JSON in genau dieser Form:
{"noten":[{"bild":"<Alias>","note":8,"strom_note":7,"regeln":["R19"],"kurz":"Begruendung"}],
 "schwaechen":[{"was":"<haeufige Schwaeche des Algorithmus>","regeln":["R36"],"wie_oft":<Anzahl Bilder>,"vorschlag":"<konkreter Ansatz im Code>"}]}
Bilder:\n'''
ANWEISUNGEN = {'blind-geometrie-v2': ANWEISUNG_V2, VERSION: ANWEISUNG}


def sha(b):
    return hashlib.sha256(b).hexdigest()


def auftrag(aliases, lagen=None, version=VERSION):
    if len(set(aliases)) != len(aliases) or any(not a.startswith('blind_') or not a[6:-4].isdigit() or not a.endswith('.png') for a in aliases):
        raise ValueError('Blindalias ungueltig')
    lagen = lagen or {a:[] for a in aliases}
    if set(lagen)!=set(aliases) or any(not re.fullmatch(r'F|B|In\d+',L) for ls in lagen.values() for L in ls):
        raise ValueError('Lagenliste ungueltig')
    return ANWEISUNGEN[version] + '\n'.join(a + ' (Uebersicht, danach Lagen: '+','.join(lagen[a])+')' for a in aliases)


def geometrie(z):
    p = Path(z.get('geometrie_pfad') or Path(z['bild_pfad']).parent/z['geometrie'])
    raw = p.read_bytes()
    g = json.loads(gzip.decompress(raw))
    if not all(isinstance(g.get(k),dict) for k in ('board','routes','klassen')):
        raise ValueError('Snapshot unvollstaendig')
    return raw,g


def zeichnen(g):
    from lernen.galerie import _zeichnen
    im = _zeichnen(g['board'],g['routes'],720,oben=22)
    return im.crop((0,22,im.width,im.height))


def paarstichprobe(Z, n, rnd):
    from lernen.paar_vertrag import boardkennung
    groups=collections.defaultdict(dict)
    for z in Z:
        if z.get('gueltig') is not True or not z.get('board'):continue
        try: _,g=geometrie(z)
        except (OSError,ValueError,KeyError):continue


        groups[boardkennung(g['board'])][sha(kanonisch(g))]=z
    boards=sorted(b for b,g in groups.items() if len(g)>=2);rnd.shuffle(boards)
    return [z for b in boards[:n//2] for z in rnd.sample(list(groups[b].values()),2)]

def vorbereiten(z, ordner, alias):
    ""
    d = Path(ordner); d.mkdir(parents=True,exist_ok=True)
    raw,g = geometrie(z)
    dst = d/alias; zeichnen(g).save(dst)
    gp = d/(alias[:-4]+'_geo.json.gz'); gp.write_bytes(raw)

    from lernen.galerie import bild_lagen
    names = bild_lagen(g['board'],g['routes'],str(d/alias[:-4]),K=g['klassen'],regeln=g.get('regeln'))
    paths = {L:str(d/name) for L,name in names.items()}
    from lernen.paar_vertrag import boardkennung
    proof = dict(version=VERSION, alias=alias, board_kennung=boardkennung(g['board']), geometrie_sha256=sha(raw),
                 bilder={'uebersicht':sha(dst.read_bytes()), **{L:sha(Path(p).read_bytes()) for L,p in paths.items()}},
                 renderer=renderersignatur())
    return dict(z, bild_pfad=str(dst), geometrie=gp.name, geometrie_pfad=str(gp), lagenbilder_pfade=paths,
                blind=True, blind_version=VERSION, blind_beleg=proof)


def renderersignatur():
    from lernen.paar_vertrag import ROOT
    paths = ['lernen/galerie.py','lernen/kupfer.py','gemeinsam/zonen.py','lernen/paar_blind.py']
    return sha(b''.join(p.encode()+b'\0'+(ROOT/p).read_bytes() for p in paths))


def vorlage(W):
    aliases=[z['blind_beleg']['alias'] for z in W]
    layers={z['blind_beleg']['alias']:sorted(z['lagenbilder_pfade']) for z in W}
    prompt=auftrag(aliases,layers)
    for z in W:
        z['blind_beleg'].update(aliases=aliases, lagen=layers, auftrag=prompt, auftrag_sha256=sha(prompt.encode()))
    return prompt


def antwort(z, x, richter):
    p=z.get('blind_beleg')
    if not p: return {}
    return dict(blind_antwort=dict(alias=p['alias'], note=x['note'], strom_note=x.get('strom_note'),
                                  richter=richter, auftrag_sha256=p['auftrag_sha256'],
                                  bilder=p['bilder']))


def pruefen(z):
    ""
    try:
        if z.get('blind') is not True or z.get('synthetisch') or z.get('_split_konflikt') or z.get('gueltig') is not True:
            return False,'kein gueltiges blindes Realurteil'
        d=Path(z['archiv']); meta=json.loads((d/'kennzahlen.json').read_text())
        proof=meta.get('blind_beleg') or {}
        if proof.get('version') not in ANWEISUNGEN or z.get('blind_beleg')!=proof:
            return False,'Vorlagebeleg fehlt/abweichend'
        if meta.get('board')!=z.get('board') or meta.get('gueltig') is not True:
            return False,'Board/Gueltigkeit passt nicht zum Archiv'
        from lernen.richter_paar import NUM
        fields = set(NUM) | {'gueltig','note_v3','note_v3_version','board_kennung','paar_messversion',
                            'regel','regeln','tor','ergebnis','kennz','kennzahlen','merkmale','strom_kette',
                            'r27','v3_komponenten','v3_komponenten_beleg','form_messung','geometrie_sha256'}


        def measured(row):
            return {k:row[k] for k in fields if k in row and (k != 'regeln' or isinstance(row[k],dict))}
        if kanonisch(measured(z)) != kanonisch(measured(meta)):
            return False,'Kennzahlen passen nicht zum Archiv'
        imraw=(d/'uebersicht.png').read_bytes()
        if sha(imraw)!=z['bild_sha256'] or sha(imraw)!=meta['bild_sha256'] or sha(imraw)!=proof['bilder']['uebersicht']:
            return False,'Bildhash abweichend'
        paths=z.get('lagenbilder_pfade') or {}
        layers=[]
        for L,p in sorted(paths.items()):
            path=Path(p)
            if path.parent.resolve()!=d.resolve():return False,'Lagenbild ausserhalb Archiv'
            raw=path.read_bytes();layers.append((L,raw))
            if sha(raw)!=proof['bilder'].get(L):return False,'Lagenbildhash abweichend'
        if set(proof['bilder']) != {'uebersicht',*paths}: return False,'Bildumfang verschieden'
        m=dict(meta);m.pop('bild_sha256')
        sid=sha(kanonisch(m)+imraw+b''.join(L.encode()+b for L,b in layers))
        if sid!=z['sample_id'] or d.name!=sid:return False,'Archividentitaet abweichend'
        gp=Path(z['geometrie_pfad'])
        if gp.resolve()!= (d/'geometrie.json.gz').resolve():return False,'Geometrie ausserhalb Archiv'
        raw,g=geometrie(z)
        if sha(raw)!=proof['geometrie_sha256'] or sha(raw)!=meta.get('geometrie_sha256'):
            return False,'Geometriehash abweichend'
        from lernen.paar_vertrag import boardkennung
        identity=boardkennung(g['board'])
        if identity!=proof.get('board_kennung'):return False,'Boardkennung passt nicht zum Snapshot'
        for value in (z.get('board_kennung'), (z.get('regel') or {}).get('board_kennung')):
            if value is not None and value!=identity:return False,'Messkennung passt nicht zum Snapshot'
        prompt=auftrag(proof['aliases'],proof['lagen'],proof['version'])
        if proof['alias'] not in proof['aliases'] or prompt!=proof['auftrag'] or sha(prompt.encode())!=proof['auftrag_sha256']:
            return False,'Kennzahlen/Identitaet im Auftrag oder Auftragsbindung fehlt'


        if not isinstance(proof.get('renderer'),str) or not re.fullmatch('[0-9a-f]{64}',proof['renderer']):return False,'Rendererherkunft fehlt'
        from PIL import Image
        import io
        actual=Image.open(io.BytesIO(imraw)).convert('RGB');expected=zeichnen(g).convert('RGB')
        if actual.size!=expected.size or actual.tobytes()!=expected.tobytes():return False,'Bild nicht kennzahlenfrei aus Snapshot'

        import tempfile
        from lernen.galerie import bild_lagen
        with tempfile.TemporaryDirectory(prefix='paar_blind_') as tmp:
            names=bild_lagen(g['board'],g['routes'],str(Path(tmp)/'lage'),K=g['klassen'],regeln=g.get('regeln'))
            if set(names)!=set(paths):return False,'Lagenumfang nicht reproduzierbar'
            for L,name in names.items():
                a=Image.open(paths[L]).convert('RGB');b=Image.open(Path(tmp)/name).convert('RGB')
                if a.size!=b.size or a.tobytes()!=b.tobytes():return False,'Lagenbild nicht aus Snapshot'
        receipt=z.get('blind_antwort') or {}
        if receipt!=dict(alias=proof['alias'],note=z.get('note'),strom_note=z.get('strom_note'),richter=z.get('richter'),
                         auftrag_sha256=proof['auftrag_sha256'],bilder=proof['bilder']):return False,'Urteil nicht an Vorlage gebunden'
        return True,'Archiv/Bilder/Geometrie/Auftrag/Antwort verifiziert'
    except (OSError,ValueError,KeyError,TypeError,AttributeError):
        return False,'Blindartefakte fehlen/defekt'


def ansicht_id(z):
    ""
    from PIL import Image
    paths={'uebersicht':str(Path(z['archiv'])/'uebersicht.png'), **(z.get('lagenbilder_pfade') or {})}
    h=hashlib.sha256()
    for label,path in sorted(paths.items()):
        with Image.open(path) as im:
            im=im.convert('RGB');h.update(kanonisch([label,im.size]));h.update(im.tobytes())
    return h.hexdigest()


def kontrollpaare(P):
    cache={};ids={};out=[];seen=set();reasons=collections.Counter()
    for p in P:
        results=[]
        for z in (p['a'],p['b']):

            key=sha(kanonisch(z))
            if key not in cache:
                cache[key]=pruefen(z)
                if cache[key][0]:ids[key]=ansicht_id(z)
            results.append(cache[key])
        if not all(v[0] for v in results):
            reasons.update(v[1] for v in results if not v[0]);continue
        if p['a']['blind_beleg']['board_kennung']!=p['b']['blind_beleg']['board_kennung']:
            reasons['verschiedene Netzliste/Umriss trotz gleichem Boardnamen']+=1;continue
        hashes=sorted(ids[sha(kanonisch(z))] for z in (p['a'],p['b']))
        key=(p['a']['blind_beleg']['board_kennung'],*hashes)
        if hashes[0]==hashes[1] or key in seen:
            reasons['gleiches/doppeltes Geometriepaar']+=1;continue
        seen.add(key);out.append(p)
    return out,dict(kandidaten=len(cache),paare=len(out),ablehnungen=dict(reasons))


def nachmessen(z):
    ""
    start = time.process_time()
    from lernen.form_live import nachmessen as form_nachmessen
    from lernen.form_live import stabil
    from lernen.richter_paar import komponenten, komponentenbeleg
    from lernen.paar_vertrag import messsignatur, boardkennung
    from analyse import vertrag as VT
    raw,g=geometrie(z)
    h=sha(raw)
    for expected in (z.get('geometrie_sha256'), (z.get('blind_beleg') or {}).get('geometrie_sha256')):
        if expected is not None and expected != h:
            raise ValueError('Snapshot-Hash abweichend')
    b,r,k=g['board'],g['routes'],g['klassen']
    measured,cost=form_nachmessen(g,h)
    e,m,rm=measured['ergebnis'],dict(measured['regelkennzahlen']),measured['regel']



    keep = ('board','aufgabe','designfamilie','familie','quelle','bild','bild_pfad',
            'geometrie','geometrie_pfad','lagenbilder_pfade','blind','blind_version','blind_beleg',
            'note','strom_note','richter','gutachten_id','gutachten_zeit','kurz','blind_antwort',
            '_familie','_familie_aliases','_split_konflikt',



            'merkmale','lagen','dichte','klasse','dauer','kandidat','schwierigkeit','designfamilie')
    out={key:z[key] for key in keep if key in z}
    if isinstance(out.get('merkmale'),dict):
        from lernen.richter_paar import NUM
        out['merkmale']={a:x for a,x in out['merkmale'].items() if a not in NUM}
    if isinstance(z.get('regeln'),list):out['regeln']=z['regeln']
    sig=messsignatur()
    m.update(board_kennung=boardkennung(b),paar_messversion=sig)
    fresh=dict(gueltig=e['gueltig'],regeln=m,note_v3_version=e['note_v3_version'],
               messversion=m.get('messversion') or sig,
               nachweise=VT.pflichtnachweise(e['metriken'],k))
    for key in ('diagnose','konflikte','offen','unverbunden','rang','drc'):
        if key in e:fresh[key]=e[key]
    tor=VT.zustand(fresh)



    laenge=sum(math.hypot(q[0]-p[0],q[1]-p[1]) for t in r.get('tracks',[]) for p,q in zip(t.get('pts') or [],(t.get('pts') or [])[1:]))
    out['kennzahlen']=dict(vias=len(r.get('vias',[])),laenge=m.get('laenge',round(laenge,1)),konflikte=e.get('konflikte'),
                           offen=e.get('offen'),knicke=m.get('knicke'),
                           anteil_45=None if m.get('anteil_45') is None else round(m['anteil_45']/100,3))
    out.update(gueltig=e['gueltig'],note_v3=rm['note_v3'],note_v3_version=e['note_v3_version'],
               board_kennung=boardkennung(b),paar_messversion=sig,regel=rm,regelkennzahlen=m,tor=tor,ergebnis=fresh,
               v3_komponenten=komponenten(e['metriken']),v3_komponenten_beleg=komponentenbeleg(e['metriken']))
    out.update(geometrie_sha256=h, form_messung=measured['form_messung'],
               strom_kette=measured['strom_kette'], form_messung_lauf=dict(cost, zeile_cpu_s=time.process_time()-start))
    return out
