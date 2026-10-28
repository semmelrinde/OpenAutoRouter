""
import copy
import math
from lernen.zonen_richter import Dienst


def an(R,wert=None):
    value=wert if wert is not None else (R.get('optionen') or {}).get('zonen_richter',0)
    if str(value).lower() not in ('0','1','false','true'):raise ValueError('zonen_richter=0|1')
    return str(value).lower() in ('1','true')


def tor(B,a,b,K,gndlagen=None,regeln=None):
    from analyse.bewertung import bewerte
    from analyse import minimax as MM
    from analyse.regelkennzahlen import annahme
    from lernen.zonen_harte_pruefung import pruefen
    from lernen.stromzonen_nachlauf import physikvergleich
    hard=pruefen(B,b,K,gndlagen,regeln)
    e0=bewerte(B,a,K,v3=True,minimax=True);e1=bewerte(B,b,K,v3=True,minimax=True)
    phys=physikvergleich(B,a,B,b,K,e0.get('ir'),e1.get('ir'),einzelast=True)
    rules=annahme(e0.get('regeln'),e1.get('regeln'));h18=MM.annahme(e0,e1)
    note=all(isinstance(e.get('note_v3'),(float,int)) and math.isfinite(e['note_v3']) for e in (e0,e1)) and e1['note_v3']<=e0['note_v3']
    ok=bool(hard.get('bestanden')) and bool(e0.get('gueltig')) and bool(e1.get('gueltig')) and not e1.get('unverbunden') and phys['bestanden'] and rules['nicht_schlechter'] and h18['nicht_schlechter'] and note
    return dict(bestanden=ok,hart=hard,physik=phys,regeln=rules,h18=h18,note_v3_nicht_schlechter=note)


def waehlen(B,R,K,gndlagen=None,**params):
    from lernen import stromzonen as SZ
    from lernen import stromzonen_topo as ST
    opts=R.get('optionen') or {};judge=Dienst(opts.get('zonen_richter_modell'))
    if judge.modell is None:raise ValueError('zonen_richter=1: '+judge.status)
    if SZ.skelett(R):raise ValueError('zonen_richter=1: Skelett-Breitenwechsel noch nicht validiert')
    if R.get('zonenplan_id') or R.get('k20_korridor'):raise ValueError('Zonenrichter mit lokalem Plan/K20 noch nicht validiert')
    base=copy.deepcopy(R);base['optionen']=dict(opts,zonen_richter='0')
    default=SZ.zonen(B,base,K,gndlagen,zonen_richter=False,**params)


    R['vias']=copy.deepcopy(base.get('vias',[]))
    recipes=[('gerade_glatt',dict(form='gerade',r_glatt=.3,r_oeffnen=.1,gruppe=1.0)),
             ('gerade_huelle',dict(form='gerade',r_glatt=.7,r_oeffnen=.15,gruppe=2.0)),
             ('rund_korridor',dict(form='rund',r_glatt=.3,r_oeffnen=.1,gruppe=1.0)),
             ('minimax',dict(form='minimax'))]
    variants=[('standard',default)];errors=[]
    for name,kw in recipes:
        try:
            rr=copy.deepcopy(base)
            if name=='rund_korridor':rr['r36_korridor']=True
            zs=SZ.zonen(B,rr,K,gndlagen,zonen_richter=False,**dict(params,**kw))
            if rr.get('vias')!=base.get('vias'):raise ValueError('Variante veraendert Via-Geruest')
            variants.append((name,zs))
        except (ValueError,RuntimeError) as e:errors.append(dict(variante=name,grund=str(e)))
    for name,kw in [('topo_glatt',dict(kappa=1.,r_glatt=.3,r_huelle=.7,r_rund=.2,keil_max=60.)),
                    ('topo_schmal',dict(kappa=2.,r_glatt=.1,r_huelle=.3,r_rund=.1,keil_max=45.))]:
        try:
            zs=ST.zonen(B,copy.deepcopy(base),K,gndlagen,clr=params.get('clr'),rand=params.get('rand',.25),via_feld=False,**kw)
            if any(z.get('skelett') for z in zs):raise ValueError('Skelett unerwartet')
            variants.append((name,zs))
        except (ValueError,RuntimeError) as e:errors.append(dict(variante=name,grund=str(e)))
    def groups(zs):
        result={}
        for z in zs:
            key=(z['net'],z['layer'])
            if key in result:raise ValueError('Mehrere unverbundene Zonendefinitionen je Netz/Lage; Auswahl gesperrt')
            result[key]=z
        return result
    selected=groups(default);logs=[]
    for key in sorted(selected):
        before=selected[key];score=judge.benoten(B,dict(base,zonen=list(selected.values())),before)['note']
        for name,zs in variants[1:]:
            candidate=groups(zs).get(key)
            if candidate is None:continue
            proposed=dict(selected);proposed[key]=candidate
            a=dict(base,zonen=list(base.get('zonen') or [])+list(selected.values()))
            b=dict(base,zonen=list(base.get('zonen') or [])+list(proposed.values()))
            pred=judge.benoten(B,b,candidate)
            if pred['note']<=score:continue
            try:gate=tor(B,a,b,K,gndlagen,K.get('regeln'))
            except Exception as e:gate=dict(bestanden=False,fehler=repr(e))
            logs.append(dict(net=key[0],layer=key[1],variante=name,note=pred['note'],tor=gate))
            if gate['bestanden']:selected=proposed;score=pred['note']
        selected[key]=dict(selected[key],zonen_richter=dict(note=score,modell_sha=judge.benoten(B,dict(base,zonen=list(selected.values())),selected[key])['modell_sha'],native_abnahme_erforderlich=True))
    R['zonen_richter_auswahl']=dict(status='pilot',freigegeben=False,varianten=len(variants),fehler=errors,entscheidungen=logs)
    return list(selected.values())
