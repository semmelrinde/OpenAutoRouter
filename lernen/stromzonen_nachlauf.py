""
import copy
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def physiktor(a, b):
    ""
    worse = []; missing = []
    if a.get('raster_mm') != b.get('raster_mm') or not a.get('raster_mm'):
        missing.append('Gleiche positive Rasteraufloesung fehlt')
    if a.get('fehler_netze') or b.get('fehler_netze'):
        missing += list(set(a.get('fehler_netze', [])) | set(b.get('fehler_netze', [])))
    for key in ('verlust_je_netz_mW', 'J_max_A_mm2', 'J_x'):
        if key not in a or key not in b:missing.append(key+' fehlt')
        for n in sorted(set(a.get(key, {})) | set(b.get(key, {}))):
            x,y=a.get(key, {}).get(n),b.get(key, {}).get(n)
            if x is None or y is None or not math.isfinite(x) or not math.isfinite(y): missing.append(key+':'+n)
            elif y>x+max(1e-6,abs(x)*1e-6): worse.append(key+':'+n)
    for n,pads in a.get('abfall_je_pad_mV', {}).items():
        for p,x in pads.items():
            y=b.get('abfall_je_pad_mV', {}).get(n, {}).get(p)
            if y is None or not math.isfinite(x) or not math.isfinite(y): missing.append(n+':'+p)
            elif y>x+max(1e-6,abs(x)*1e-6): worse.append(n+':'+p)
    return dict(bestanden=not missing and not worse, schlechter=worse, fehlt=missing,
                relativ_toleranz=1e-6, aufloesung_mm=a.get('raster_mm'))


def physikvergleich(B0, a, B1, b, K, ir0=None, ir1=None, einzelast=False):
    ""
    from analyse.kupfermodell import ir_werte
    attempts=[]
    for res in (.1,.05):
        x=ir0 if res==.1 and ir0 is not None else ir_werte(B0,a,K,res=res)
        y=ir1 if res==.1 and ir1 is not None else ir_werte(B1,b,K,res=res)
        gate=physiktor(x,y)
        if einzelast:
            from lernen.zonenplaner_physik import je_last, tor
            last=tor(je_last(B0,a,K,res),je_last(B1,b,K,res))
            gate['einzelast_J']=last
            gate['bestanden'] &= last['bestanden']
        attempts.append(dict(raster_mm=res,tor=gate,vorher=x,nachher=y))
        if gate['bestanden']:break
    return dict(gate,versuche=attempts)


def verfeinern(B0,a,B,old,K,RG,folder,b,native,physics,gnd=True):
    ""
    from lernen import stromzonen as SZ
    from lernen.kupfer_nativ import pruefen
    history=[];base=k20_basis(old);proposal=None
    for iteration in range(2):
        if physics['bestanden']:break
        ir0=physics['versuche'][-1]['vorher']
        shield=base.setdefault('r36_schutz',{})
        changed=False
        local_free=base.setdefault('r36_frei',{})
        j_nets=set()
        for key in physics['schlechter']:
            net,sep,pad=key.partition(':')
            if net in ('J_max_A_mm2','J_x') and pad in K['netze']:
                if pad not in j_nets:
                    local_free[pad]=max(.0125,local_free.get(pad,.05)/2);j_nets.add(pad)
                changed=True;continue
            if not sep or net not in K['netze'] or ':' not in pad:continue
            value=(ir0.get('abfall_je_pad_mV',{}).get(net) or {}).get(pad)
            maximum=ir0.get('max_abfall_mV',{}).get(net)
            if value is None or not maximum:continue
            shield[net]=min(.4,max(.02,value/maximum*2.5,shield.get(net,0)*1.5))
            changed=True
        if not changed:break
        proposal=copy.deepcopy(base)
        _zs=SZ.zonen(B,proposal,K);SZ.skelett_anwenden(proposal,_zs);proposal['zonen']+=_zs
        if gnd:
            gz,gv=SZ.gnd_fuellung(B,proposal,K)
            proposal['zonen']+=gz;proposal['vias']+=gv
        path=Path(folder)/('physik_'+str(iteration+1));path.mkdir(parents=True,exist_ok=True)
        (path/'vorschlag.json').write_text(json.dumps(proposal))
        _,b,nt=pruefen(path/'nativ',B,base,proposal,RG,cpu_s=240)
        physics=physikvergleich(B0,a,B,b,K)
        history.append(dict(schutz=copy.deepcopy(shield),frei=copy.deepcopy(local_free),nativ=nt,physiktor=physics))
        native=dict(native,vorschlag=nt['vorschlag'],native_vorschlag_gueltig=nt['vorschlag']['gueltig'],
                    nicht_mehr_fehler=nt['vorschlag']['fehler']<=native['vorher']['fehler'],
                    nicht_mehr_offen=nt['vorschlag']['offen']<=native['vorher']['offen'])
    native['physik_nachbesserungen']=history
    return b,native,physics,proposal


def masselagen(B, R, K):
    ""
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    out=copy.deepcopy(R)
    gl=out.get('gndlagen') or (out.get('optionen') or {}).get('gndlagen') or []
    if isinstance(gl,str):gl=gl.split(',')
    gnd=K.get('gnd') or ['GND']
    for label in gl:
        layer=str(label).split('=')[0]
        net=str(label).split('=',1)[1] if '=' in str(label) else gnd[0]
        if net not in gnd:continue
        if any(z['layer']==layer and z['net']==net for z in out.get('zones',[])):continue
        shape=unary_union([Polygon(q) for q in B['outline']]).buffer(-float((R.get('optionen') or {}).get('edge') or .2))
        for p in getattr(shape,'geoms',[shape]):
            out.setdefault('zones',[]).append(dict(net=net,layer=layer,pts=list(p.exterior.coords),
                holes=[list(h.coords) for h in p.interiors],filled=True,full=True,prio=0,
                local_clearance=None,min_thickness=.1))
    return out


def k20_basis(route):
    ""
    out = copy.deepcopy(route)
    if out.get('k20_korridor'):
        for key in ('zones', 'zonen'):
            out[key] = [z for z in out.get(key, []) if not z.get('r36') and
                        z.get('quelle') not in ('stromzonen', 'gnd_fuellung')]
    return out


def ausfuehren(bj, kj, rj, regeln=None, gnd=True, folder=None, zonenplan=None):
    from lernen import stromzonen as SZ
    from lernen.kupfer_nativ import bibliotheksgeometrie, pruefen, native_items
    from analyse.bewertung import bewerte
    from gemeinsam.zonen import vereinheitlichen
    paths=list(map(Path,(bj,kj,rj)))
    B,K,R=[json.loads(p.read_text()) for p in paths]
    folder=Path(folder or (str(paths[2].with_suffix(''))+'_stromzonen'))
    folder.mkdir(parents=True,exist_ok=True)
    rg=regeln or (R.get('optionen') or {}).get('regeln')
    RG=json.loads(Path(rg).read_text()) if rg and Path(rg).exists() else None
    plan=json.loads(Path(zonenplan).read_text()) if zonenplan else None
    if plan:
        from lernen.zonenplaner import sha, pruefe_plan, pruefe_eingaben
        pruefe_plan(plan)
        if plan['routing_board_sha256'] != sha(B) or plan['eingabehashes']['klassen'] != sha(K) or plan['eingabehashes']['regeln'] != sha(RG):
            raise ValueError('Zonenplan-Eingaben geaendert')
        if R.get('zonenplan_id') != plan['id']: raise ValueError('Route gehoert zu anderem/keinem Zonenplan')
        pruefe_eingaben(plan,B,K,RG,R.get('optionen') or {})
        B=json.loads(Path(plan['board_datei']).read_text())
        if sha(B)!=plan['eingabehashes']['board']:
            raise ValueError('Native Plan-Padgeometrie geaendert')
        padinfo=dict(quelle=plan['padgeometrie'])
    else:
        B,padinfo=bibliotheksgeometrie(folder/'bibliothek',B,R,RG)
    if plan:
        from lernen.zonenplaner import via_mitte_items
        r37_items=via_mitte_items(R)
    old=native_items(R)
    old['zonen']=[z for z in old.get('zonen',[]) if z.get('quelle') not in ('stromzonen','gnd_fuellung','zonenplaner')]
    old['vias']=[v for v in old['vias'] if v.get('quelle') not in ('gnd_fuellung','zonenplaner')]
    old=masselagen(B,old,K)
    old['stromzonen_modell']='r36-dc-v1'
    B0,old0=copy.deepcopy(B),copy.deepcopy(old)
    from lernen.stromzonen_reparatur import vorbereiten
    if plan:
        reparatur=dict(angewendet=False,grund='Lokaler Plan: Placement/Geruest unveraendert')
    else:
        B,old,reparatur=vorbereiten(B,old,K,RG,folder/'reparatur')
    new=k20_basis(old)
    if plan:
        import uuid
        from shapely.ops import unary_union
        grouped={}
        for z in plan['kern']:
            grouped.setdefault((z['net'],z['layer']),[]).append(z)
        for (net,layer),zs in sorted(grouped.items()):
            shape=unary_union([SZ._geo(z) for z in zs])
            contours=[dict(pts=p['aussen'],holes=p.get('innen',[])) for p in SZ._polys(shape)]
            ident=net+':'+layer
            for c in contours[:1]:
                new.setdefault('zones',[]).append(dict(net=net,layer=layer,**c,filled=True,full=True,
                    r36=True,strompfad=True,quelle='zonenplaner',kern_id=ident,prio=1,
                    zone_id=str(uuid.uuid5(uuid.NAMESPACE_URL,plan['id']+ident)),native_layers=[layer],
                    native_outlines=contours,min_thickness=max(z['min_thickness'] for z in zs),local_clearance=None))
        from analyse.flaechen import umriss
        for layer in plan.get('potenziallagen',[]):
            ground=(K.get('gnd') or ['GND'])[0]
            shape=umriss(B).buffer(-float((R.get('optionen') or {}).get('edge') or .3))
            for pg in getattr(shape,'geoms',[shape]):
                new.setdefault('zones',[]).append(dict(net=ground,layer=layer,pts=list(pg.exterior.coords),holes=[list(h.coords) for h in pg.interiors],filled=True,full=True,prio=0,min_thickness=.1,quelle='potenzial_rest_gnd'))
        new['vias'] += [v for f in plan['via_kandidaten'] if f['angenommen'] for v in f['vias']]
    else:
        _zs=SZ.zonen(B,new,K);SZ.skelett_anwenden(new,_zs);new['zonen']+=_zs
    if gnd:
        gz,gv=SZ.gnd_fuellung(B,new,K);new['zonen']+=gz;new['vias']+=gv
    if plan:


        new=vereinheitlichen(new)
        from lernen.zonenplaner_form import mindestbreiten
        new=mindestbreiten(B,new,RG,plan)
    (folder/'vorschlag.json').write_text(json.dumps(new))
    a,b,native=pruefen(folder/'nativ',B,old,new,RG,cpu_s=240)
    a['native_padgeometrie']=B['pads'];b['native_padgeometrie']=B['pads']
    if reparatur['angewendet']:
        a,_,native0=pruefen(folder/'ursprung_nativ',B0,old0,old0,RG,cpu_s=180)
        native['ursprung']=native0['vorher']
    from lernen.zonen_richter_auswahl import an as richter_an
    mm=True if richter_an(R) else None
    e0=bewerte(B0,a,K,v3=True,minimax=mm);e1=bewerte(B,b,K,v3=True,minimax=mm)
    physics=physikvergleich(B0,a,B,b,K,e0['ir'],e1['ir'],einzelast=bool(plan) or richter_an(R))
    if plan:
        refined=None
    else:
        b,native,physics,refined=verfeinern(B0,a,B,old,K,RG,folder,b,native,physics,gnd)
    if refined is not None:
        new=refined;b['native_padgeometrie']=B['pads'];e1=bewerte(B,b,K,v3=True,minimax=mm)
    if richter_an(R):


        physics=physikvergleich(B0,a,B,b,K,e0['ir'],e1['ir'],einzelast=True)
    hard=SZ.pruefen(B,new,K)
    from lernen.kupferbalken import pruefe_kanten
    audit=vereinheitlichen(b)
    for z in audit['zones']:
        if z.get('r36') and z['net'] not in K.get('gnd',[]):z['kupferbalken']='R36-Audit'
    kanten=pruefe_kanten(B,audit,K)
    gates=dict(drc=native['vorschlag']['gueltig'],physik=physics['bestanden'],
               abstand=not hard,verbindung=not e1['unverbunden'])
    good=all(gates.values())


    failures=[]
    if not gates['drc']:
        failures.append('native DRC '+str(dict(native['vorschlag']['typen']))+
                        ', offen='+str(native['vorschlag']['offen']))
    if not gates['abstand']:failures.append('Zonenabstand '+str(hard[:3]))
    if not gates['verbindung']:failures.append('Verbindung '+str(e1['unverbunden']))
    if not gates['physik']:failures.append('Physik '+str(physics['schlechter']+physics['fehlt']))
    plan_audit=None
    if plan:
        from lernen.zonenplaner_form import pruefen as native_form, kurz, kupferformen
        from shapely.geometry import Polygon
        from shapely.ops import unary_union
        checks=[]
        for z in plan['kern']:
            filled=unary_union([Polygon(q['pts'],q.get('holes',[])) for q in b['zones']
                               if q.get('quelle')=='zonenplaner' and q['net']==z['net'] and q['layer']==z['layer']])
            checks.append(dict(kern_id=z['kern_id'],form=kurz(native_form(filled)),
                               kern_erhalten=filled.buffer(.006).covers(SZ._geo(z))))
        missing_fields=[f['kern_id'] for f in plan['via_kandidaten'] if not f['angenommen']]
        missing_cores=[q for q in plan['audit']['ablehnungen'] if q['grund'].startswith('Lokaler Kern:')]
        sw_vias=[v for v in b['vias'] if plan['netze'].get(v['net'],{}).get('lokal_ohne_lagenwechsel')]
        native_formen=kupferformen(b)
        from lernen.zonenplaner_korridor import native_pruefung
        potential=native_pruefung(b,plan)
        plan_audit=dict(potenzial=potential,native_formen=native_formen,kerne=checks,fehlende_kerne=missing_cores,fehlende_felder=missing_fields,sw_vias=sw_vias,
                        bestanden=potential['bestanden'] and not plan['audit'].get('potenzial_fehlend') and not missing_cores and not missing_fields and not sw_vias and all(q['bestanden'] for q in native_formen) and all(q['form']['bestanden'] and q['kern_erhalten'] for q in checks))
        from analyse.regelkennzahlen import annahme
        regeltor=annahme(e0['regeln'],e1['regeln'])
        plan_audit['regeltor']=regeltor
        plan_audit['bestanden'] &= regeltor['nicht_schlechter']
        good &= plan_audit['bestanden']
    from lernen.zonen_richter_auswahl import an as richter_an
    richter_audit=None
    if richter_an(R):
        from analyse import minimax as MM
        from analyse.regelkennzahlen import annahme as regeln_annahme
        rules=regeln_annahme(e0.get('regeln'),e1.get('regeln')); h18=MM.annahme(e0,e1)
        note=e1.get('note_v3') is not None and e0.get('note_v3') is not None and math.isfinite(e1['note_v3']) and e1['note_v3']<=e0['note_v3']
        richter_audit=dict(regeln=rules,h18=h18,note_v3_nicht_schlechter=note,
            bestanden=rules['nicht_schlechter'] and h18['nicht_schlechter'] and note)
        gates['zonen_richter']=richter_audit['bestanden'];good &= gates['zonen_richter']
        if not gates['zonen_richter']:failures.append('Zonenrichter: natives Regel-/H18-/v3-Tor nicht bestanden')
    gates['zonenplan']=plan_audit['bestanden'] if plan_audit else True
    if not gates['zonenplan']:failures.append('Lokaler Plan: '+str(plan_audit))
    info=dict(angewendet=good,grund='; '.join(failures),harte_tore=gates,
              geruestreparatur=reparatur,
              nativ=native,physiktor=physics,abstand=hard,padquelle=padinfo,kanten=kanten,
              note_v3_vorher=e0['note_v3_netze'],note_v3_nachher=e1['note_v3_netze'],
              verlust_mW_vorher=e0['ir']['verlust_mW'],verlust_mW_nachher=e1['ir']['verlust_mW'],
              stromzonen=len(new['zonen']),artefakte=str(folder),
              weich_als_tor=False)
    if plan: info.update(zonenplan_id=plan['id'],plan_audit=plan_audit,r37_items=r37_items)
    if richter_audit is not None:
        from lernen.zonen_richter import Dienst
        info['zonen_richter_tor']=richter_audit
        info['zonen_richter_auswahl']=new.get('zonen_richter_auswahl')
        info['zonen_richter']=Dienst((R.get('optionen') or {}).get('zonen_richter_modell')).messen(B,b)
    b['stromzonen_abnahme']=info
    (folder/'gefuellt.json').write_text(json.dumps(b))
    return b,info


def abschluss(bj, kj, rj, rg, gnd, plan=None):
    ""
    route, info = ausfuehren(bj, kj, rj, rg, gnd, zonenplan=plan)
    original = json.loads(Path(rj).read_text())
    if original.get('k20_korridor') and info['angewendet']:
        try:
            from lernen.k20_nachlauf import annahme
            from lernen.form import empfaengertor_an
            from lernen.stromzonen import _empfaenger_erlaubt
            B, K = [json.loads(Path(p).read_text()) for p in (bj, kj)]
            info['k20_tor'] = annahme(B, original, route, K)
            info['empfaengertor'] = _empfaenger_erlaubt(B, K, original, route) if empfaengertor_an(K) else True
            if not info['k20_tor']['angenommen'] or not info['empfaengertor']:
                info.update(angewendet=False, grund='K20: kein abgenommener Gewinn ohne Regelregression')
        except Exception as exc:
            info.update(angewendet=False, grund='K20-Pruefung fehlgeschlagen', fehler=repr(exc))
        route['stromzonen_abnahme'] = info
    return route, info


if __name__=='__main__':
    bj,kj,rj,rg,result,aus,gnd=sys.argv[1:8]
    plan=sys.argv[8] if len(sys.argv)>8 else None
    try:
        route,info=abschluss(bj,kj,rj,rg or None,gnd=='1',plan)
        Path(aus).write_text(json.dumps(route))
    except Exception as exc:
        info=dict(angewendet=False,grund=str(exc))
    Path(result).write_text(json.dumps(info))
