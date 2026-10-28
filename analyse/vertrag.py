""
import math
import re

VERSION = 'tor-v1'
TOR = ('note_v3', 'verlust', 'uebersprechen', 'rueckweg', 'topologie', 'kopplung', 'regelpruefer')


def zahl(v):
    return float(v) if isinstance(v, (float,int)) and not isinstance(v,bool) and math.isfinite(v) else None


def zustand(E, regeln=None, nachweise=None):
    ""
    r = regeln if regeln is not None else E.get('regeln', E.get('regel')) or {}
    r = r.get('regeln') or r
    geo = E.get('geometrisch_gueltig', E.get('gueltig'))
    if E.get('diagnose') is False or E.get('konflikte',0) or E.get('offen',0) or E.get('unverbunden'):
        geo = False
    d=E.get('drc')
    if isinstance(d,dict) and (d.get('fehler') or d.get('routing',0) or d.get('unverbunden',0) or d.get('platz_netto',d.get('platz',0))):geo=False
    nw = dict(nachweise if nachweise is not None else (E.get('nachweise') or (r.get('tor') or {}).get('nachweise') or {}))
    from analyse.regelkennzahlen import torfelder
    keys = torfelder(dict(r, note_v3_version=E.get('note_v3_version') or r.get('note_v3_version') or r.get('version')))
    fehlt = [k for k in keys if zahl(r.get(k)) is None]
    for k in fehlt: nw['regel:'+k] = dict(status='unbekannt', grund='Messung fehlt')
    if not nw: nw['elektrik'] = dict(status='unbekannt', grund='kein Pflichtnachweis')
    unknown = sorted(k for k,v in nw.items() if v.get('status') not in ('erfuellt','verletzt'))
    bad = sorted(k for k,v in nw.items() if v.get('status')=='verletzt')
    elektrisch = 'verletzt' if bad else ('unbekannt' if unknown else 'erfuellt')
    return dict(version=VERSION, geometrisch='erfuellt' if geo is True else ('verletzt' if geo is False else 'unbekannt'),
                elektrisch=elektrisch, unbekannt=unknown, verletzt=bad, nachweise=nw,
                gueltig=geo is True, vollstaendig_gueltig=geo is True and not bad and not unknown,
                regeln={k:zahl(r.get(k)) for k in keys}, rang=E.get('rang'), messversion=E.get('messversion') or r.get('messversion'),note_version=E.get('note_v3_version') or r.get('note_v3_version') or r.get('version'))


def von_zeile(z):
    ""
    e = dict(z.get('ergebnis') or {})
    e['gueltig'] = bool(z.get('gueltig')) and e.get('gueltig',True)
    e['regeln'] = z.get('regel') or e.get('regeln') or {}
    if isinstance(e['regeln'].get('regeln'),dict) or isinstance(e['regeln'].get('regeltor'),dict):
        r=e['regeln'];e['regeln']=dict(r.get('regeln') or r.get('regeltor'))
        for key in TOR:
            if key not in e['regeln'] and zahl(r.get(key)) is not None:e['regeln'][key]=r[key]
        e['regeln'].setdefault('note_v3',r.get('note_v3',z.get('note_v3')))
        e.setdefault('note_v3_version',r.get('version'))
        if r.get('bewertungsziel')=='minimax' and not z.get('tor'):
            status=(r.get('sicherheit') or {}).get('status')
            e['nachweise']={'minimax:sicherheit':dict(status={'sicher':'erfuellt','unsicher':'verletzt'}.get(status,'unbekannt'),version=r.get('minimax_version'))}
    if z.get('tor') or e.get('tor') or (z.get('regel') or {}).get('tor'):
        t = dict(z.get('tor') or e.get('tor') or z['regel']['tor'])

        e.update(geometrisch_gueltig=t.get('geometrisch')=='erfuellt' and e['gueltig'], nachweise=t.get('nachweise') or {}, note_v3_version=t.get('note_version'),messversion=t.get('messversion'))
    return zustand(e)


def vergleich(a,b, strikt=False):
    ""
    def tor(e): return e if e.get('version')==VERSION and 'geometrisch' in e else zustand(e)
    a,b=tor(a),tor(b)
    if any(not isinstance(t.get('messversion'),str) or not t['messversion'].strip() for t in (a,b)):
        return dict(besser=False,nicht_schlechter=False,grund='Auswertersignatur fehlt')
    if a['version'] != b['version'] or a.get('note_version') != b.get('note_version') or a.get('messversion')!=b.get('messversion'):
        return dict(besser=False,nicht_schlechter=False,grund='Vertrags-/Notenversion verschieden')
    keys=tuple(dict.fromkeys((*a['regeln'],*b['regeln'])))
    fehlend=[k for k in keys if a['regeln'].get(k) is None or b['regeln'].get(k) is None]
    if fehlend: return dict(besser=False,nicht_schlechter=False,grund='Messungen unbekannt: '+', '.join(fehlend))
    schlecht=[k for k in keys if b['regeln'][k]>a['regeln'][k]+1e-9]
    if set(a['nachweise'])!=set(b['nachweise']):schlecht.append('Pflichtnachweisumfang verschieden')
    for k,v in a['nachweise'].items():
        bn=b['nachweise'].get(k,{})
        bv=bn.get('status','unbekannt')
        if v.get('status')=='erfuellt' and bv!='erfuellt':schlecht.append(k)
        elif v.get('status')=='verletzt' and bv=='unbekannt':schlecht.append(k)
        if v.get('status')!='verletzt' and bv=='verletzt':schlecht.append(k)
        if v.get('version')!=bn.get('version'):schlecht.append(k+':version')
    if a['gueltig'] and not b['gueltig']:schlecht.append('gueltig')
    if a['elektrisch']!='verletzt' and b['elektrisch']=='verletzt':schlecht.append('elektrisch')
    if strikt and not b['vollstaendig_gueltig']:schlecht.append('Pflichtnachweise unbekannt/verletzt')
    geo_fortschritt=False
    if not a['gueltig'] and not b['gueltig']:
        ra,rb=a.get('rang'),b.get('rang')
        if isinstance(ra,(list,tuple)) and isinstance(rb,(list,tuple)) and len(ra)>=3 and len(rb)>=3 and all(zahl(v) is not None for v in (*ra[:3],*rb[:3])):
            geo_fortschritt=tuple(rb[:3])<tuple(ra[:3])
            if tuple(rb[:3])>tuple(ra[:3]):schlecht.append('geometrischer Rang')
    besser=(not a['gueltig'] and b['gueltig']) or geo_fortschritt or any(b['regeln'][k]<a['regeln'][k]-1e-9 for k in keys)
    besser |= len(b['unbekannt'])<len(a['unbekannt']) or len(b['verletzt'])<len(a['verletzt'])
    return dict(besser=not schlecht and besser,nicht_schlechter=not schlecht,
                grund='Regression: '+', '.join(schlecht) if schlecht else ('Tor besser' if besser else 'Tor gleich'),
                schlechter=schlecht)


def belohnung(z, bezug=None):
    ""
    t=von_zeile(z)
    if not t['gueltig']:return 2.
    if t['elektrisch']=='verletzt':return 2.
    if any(v is None for v in t['regeln'].values()):return None
    if bezug is not None:
        tb=von_zeile(bezug)
        if not vergleich(tb,t)['nicht_schlechter']:return 2.
    vals=list(t['regeln'].values())
    return sum(v/(1+v) for v in vals)/len(vals)


def pflichtnachweise(M,K):
    ""
    nw={}
    VG=K.get('vorgaben') or {}
    names=set(K.get('netze',{})) | set(M.get('netze',{}))
    si_felder=('z_ohm','z_diff_ohm','laengengruppe','auge','stoerbudget_mV')


    for pattern,values in (VG.get('netze') or {}).items():
        try:matched=any(pattern==n or re.fullmatch(pattern,n,re.I) or re.fullmatch(pattern,n.split('/')[-1],re.I) for n in names)
        except re.error:matched=False
        if not matched:
            for field in si_felder:
                if values.get(field) is not None:nw['si:vorgabe:'+pattern+':'+field]=dict(status='unbekannt',grund='Vorgegebenes Netz fehlt in Klassen/Messung')
    for n in sorted(set(K.get('netze',{})) | set(M.get('netze',{}))):
        v=M.get('netze',{}).get(n,{})
        m=v.get('m') or {}; k=K.get('netze',{}).get(n,{})
        roh=m.get('sicherheit_roh') or m
        if k.get('strom_modell')=='gleichricht_Mittelwert_RMS_unbekannt':
            nw['rms:'+n]=dict(status='unbekannt',grund='Gleichrichter-Wellenform/PF fehlt')
        requested_I=zahl(k.get('strom_A'))
        last_I=sum((k.get('fluss') or {}).get('last',{}).values()) if (k.get('fluss') or {}).get('einheit')=='A' else 0.
        power_unknown=k.get('art') in ('strom','versorgung','sw') and requested_I is None
        if (m.get('traegt') and (roh.get('I_A',m.get('I_A')) or 0)>0) or (requested_I or last_I or 0)>0 or power_unknown:
            ampere=(k.get('fluss') or {}).get('einheit')=='A' or zahl(k.get('strom_A')) is not None
            values={field:zahl(roh.get(field)) for field in ('I_A','J_x','J_via_x','dV_mV','s_dc_mV')}
            complete=all(x is not None and x>=0 for x in values.values()) and (values['s_dc_mV'] or 0)>0
            valid=ampere and complete and str(m.get('dV_quelle','')).startswith(('baum','ir')) and not m.get('dc_last_unverbunden')
            checks=[values['J_x'],values['J_via_x']]
            if (values['s_dc_mV'] or 0)>0 and values['dV_mV'] is not None:checks.append(values['dV_mV']/values['s_dc_mV'])
            bad=ampere and any(x is not None and x>1+1e-9 for x in checks)
            nw['dc:'+n]=dict(status='verletzt' if bad else ('erfuellt' if valid else 'unbekannt'), modell='Leiternetz/IPC-Proxy',grund='Ampere-Modell' if ampere else 'Nur normiertes Stromszenario')
        if k.get('art') in ('schnell','analog','kelvin'):
            final=M.get('referenz',{}).get('art')=='endkupfer'
            missing=zahl(roh.get('l_ohne_mm'))
            nw['referenz:'+n]=dict(status=('verletzt' if missing>1e-9 else 'erfuellt') if final and missing is not None else 'unbekannt', grund=M.get('referenz',{}).get('art'))
        soll=dict(k)

        for pattern,values in ((K.get('vorgaben') or {}).get('netze') or {}).items():
            try:
                if pattern==n or re.fullmatch(pattern,n,re.I) or re.fullmatch(pattern,n.rsplit('/',1)[-1],re.I):soll.update(values)
            except re.error:pass
        from analyse.signalintegritaet import vorgabe
        for feld in si_felder:
            if vorgabe(K,n,feld,soll.get(feld)) is not None:nw['si:'+n+':'+feld]=dict(status='unbekannt',grund='SI-Teil liefert Pflichtnachweis')

    for name,gr in M.get('gruppen',{}).items():
        value=gr.get('summe_roh_mV',gr.get('summe_mV'));limit=gr.get('mV')
        nw['kanal:'+name]=dict(status='unbekannt' if zahl(value) is None or not limit else ('verletzt' if value>limit+1e-9 else 'erfuellt'))
    for name,gr in (VG.get('kanal_budget_mV') or {}).items():
        if name not in M.get('gruppen',{}) or any(n not in M.get('netze',{}) for n in gr.get('netze',[])):
            nw['kanal:'+name]=dict(status='unbekannt',grund='Vorgegebene Kanalgruppe/Messnetze fehlen')
    if M.get('sicherheit'):nw['minimax:sicherheit']=dict(status={'sicher':'erfuellt','unsicher':'verletzt'}.get(M['sicherheit'].get('status'),'unbekannt') if M.get('minimax_version') else 'unbekannt',version=M.get('minimax_version'))
    if (M.get('masse') or {}).get('unbekannt'):nw['masse']=dict(status='unbekannt',grund='getrennte Elektrode/Rastergrenze')
    if K.get('net_ties') and not M.get('masse'):nw['masse']=dict(status='unbekannt',grund='kein belastbares Messmassemodell')
    limit=(K.get('vorgaben') or {}).get('entkopplung_L_nH')
    loops={}
    for h in M.get('entkopplung') or []:
        key=(h['chip'],h['netz'],h['pin'])
        if key not in loops or (h['status']!='modell',h['L_nH'])<(loops[key]['status']!='modell',loops[key]['L_nH']):loops[key]=h
    expected={(p['chip'],p['netz'],p['pin']) for p in M.get('entkopplung_pins',[])} | set(loops)
    per_net={}
    for key in sorted(expected):
        per_net.setdefault(key[1],[]).append(key)
    for n,keys in per_net.items():
        states=[]
        for key in keys:
            h=loops.get(key,{})
            known=zahl(limit) is not None and limit>0 and h.get('status')=='modell' and zahl(h.get('L_nH')) is not None and M.get('referenz',{}).get('art')=='endkupfer'
            states.append(('verletzt' if h['L_nH']>limit else 'erfuellt') if known else 'unbekannt')
        nw['entkopplung:'+n]=dict(status='verletzt' if 'verletzt' in states else ('unbekannt' if 'unbekannt' in states else 'erfuellt'),
                                pins=[chip+':'+pin for chip,_,pin in keys],
                                grund='Alle erwarteten Versorgungspins: explizites L-Budget, verbundenes Endkupfer und C erforderlich')
    for key,value in (M.get('nachweise') or {}).items():

        if key in nw and not key.startswith(('si:','minimax:')):continue
        adapter=dict(value)
        if not adapter.get('version'):
            adapter.update(status='unbekannt',grund='Version des Pflichtnachweis-Adapters fehlt')
        nw[key]=adapter
    return nw or dict(elektrik=dict(status='erfuellt',grund='keine elektrischen Anforderungen'))
