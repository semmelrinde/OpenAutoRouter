import math
import re

VERSION = 'gate_check-v1'
GATE = ('score_v3', 'power_loss', 'crosstalk', 'return_path', 'topology', 'coupling', 'rule_checker')


def count(v):
    return float(v) if isinstance(v, (float,int)) and not isinstance(v,bool) and math.isfinite(v) else None


def cur_state(E, design_rules=None, proofs=None):
    r = design_rules if design_rules is not None else E.get('design_rules', E.get('rule')) or {}
    r = r.get('design_rules') or r
    geo = E.get('geometric_valid', E.get('is_valid'))
    if E.get('diagnose') is False or E.get('conflict_list',0) or E.get('open_items',0) or E.get('not_connected'):
        geo = False
    d=E.get('drc')
    if isinstance(d,dict) and (d.get('faults') or d.get('routing',0) or d.get('not_connected',0) or d.get('slot_usable',d.get('slot',0))):geo=False
    nw = dict(proofs if proofs is not None else (E.get('proofs') or (r.get('gate_check') or {}).get('proofs') or {}))
    from analysis.rule_metrics import gate_fields
    keys = gate_fields(dict(r, score_v3_version=E.get('score_v3_version') or r.get('score_v3_version') or r.get('version')))
    missing = [k for k in keys if count(r.get(k)) is None]
    for k in missing: nw['rule:'+k] = dict(status='unknown', reason='Measurement missing')
    if not nw: nw['electrics'] = dict(status='unknown', reason='no mandatory evidence')
    unknown = sorted(k for k,v in nw.items() if v.get('status') not in ('fulfilled','violated'))
    bad = sorted(k for k,v in nw.items() if v.get('status')=='violated')
    electrical = 'violated' if bad else ('unknown' if unknown else 'fulfilled')
    return dict(version=VERSION, geometric='fulfilled' if geo is True else ('violated' if geo is False else 'unknown'),
                electrical=electrical, unknown=unknown, violated=bad, proofs=nw,
                is_valid=geo is True, complete_valid=geo is True and not bad and not unknown,
                design_rules={k:count(r.get(k)) for k in keys}, rank=E.get('rank'), measurement_version=E.get('measurement_version') or r.get('measurement_version'),score_version=E.get('score_v3_version') or r.get('score_v3_version') or r.get('version'))


def from_row(z):
    e = dict(z.get('outcome') or {})
    e['is_valid'] = bool(z.get('is_valid')) and e.get('is_valid',True)
    e['design_rules'] = z.get('rule') or e.get('design_rules') or {}
    if isinstance(e['design_rules'].get('design_rules'),dict) or isinstance(e['design_rules'].get('rule_gate'),dict):
        r=e['design_rules'];e['design_rules']=dict(r.get('design_rules') or r.get('rule_gate'))
        for key in GATE:
            if key not in e['design_rules'] and count(r.get(key)) is not None:e['design_rules'][key]=r[key]
        e['design_rules'].setdefault('score_v3',r.get('score_v3',z.get('score_v3')))
        e.setdefault('score_v3_version',r.get('version'))
        if r.get('evaluation_target')=='minimax' and not z.get('gate_check'):
            status=(r.get('safety') or {}).get('status')
            e['proofs']={'minimax:safety':dict(status={'safe':'fulfilled','uncertain':'violated'}.get(status,'unknown'),version=r.get('minimax_version'))}
    if z.get('gate_check') or e.get('gate_check') or (z.get('rule') or {}).get('gate_check'):
        t = dict(z.get('gate_check') or e.get('gate_check') or z['rule']['gate_check'])

        e.update(geometric_valid=t.get('geometric')=='fulfilled' and e['is_valid'], proofs=t.get('proofs') or {}, score_v3_version=t.get('score_version'),measurement_version=t.get('measurement_version'))
    return cur_state(e)


def comparison(a,b, strict=False):
    def gate_check(e): return e if e.get('version')==VERSION and 'geometric' in e else cur_state(e)
    a,b=gate_check(a),gate_check(b)
    if any(not isinstance(t.get('measurement_version'),str) or not t['measurement_version'].strip() for t in (a,b)):
        return dict(better=False,not_worse=False,reason='Evaluator signature missing')
    if a['version'] != b['version'] or a.get('score_version') != b.get('score_version') or a.get('measurement_version')!=b.get('measurement_version'):
        return dict(better=False,not_worse=False,reason='Contract/score version differs')
    keys=tuple(dict.fromkeys((*a['design_rules'],*b['design_rules'])))
    absent=[k for k in keys if a['design_rules'].get(k) is None or b['design_rules'].get(k) is None]
    if absent: return dict(better=False,not_worse=False,reason='Measurements unknown: '+', '.join(absent))
    bad=[k for k in keys if b['design_rules'][k]>a['design_rules'][k]+1e-9]
    if set(a['proofs'])!=set(b['proofs']):bad.append('Mandatory evidence scope differs')
    for k,v in a['proofs'].items():
        bn=b['proofs'].get(k,{})
        bv=bn.get('status','unknown')
        if v.get('status')=='fulfilled' and bv!='fulfilled':bad.append(k)
        elif v.get('status')=='violated' and bv=='unknown':bad.append(k)
        if v.get('status')!='violated' and bv=='violated':bad.append(k)
        if v.get('version')!=bn.get('version'):bad.append(k+':version')
    if a['is_valid'] and not b['is_valid']:bad.append('is_valid')
    if a['electrical']!='violated' and b['electrical']=='violated':bad.append('electrical')
    if strict and not b['complete_valid']:bad.append('Mandatory evidence unknown/violated')
    geo_progress=False
    if not a['is_valid'] and not b['is_valid']:
        ra,rb=a.get('rank'),b.get('rank')
        if isinstance(ra,(list,tuple)) and isinstance(rb,(list,tuple)) and len(ra)>=3 and len(rb)>=3 and all(count(v) is not None for v in (*ra[:3],*rb[:3])):
            geo_progress=tuple(rb[:3])<tuple(ra[:3])
            if tuple(rb[:3])>tuple(ra[:3]):bad.append('geometric rank')
    better=(not a['is_valid'] and b['is_valid']) or geo_progress or any(b['design_rules'][k]<a['design_rules'][k]-1e-9 for k in keys)
    better |= len(b['unknown'])<len(a['unknown']) or len(b['violated'])<len(a['violated'])
    return dict(better=not bad and better,not_worse=not bad,
                reason='Regression: '+', '.join(bad) if bad else ('Gate better' if better else 'Gate equal'),
                worse=bad)


def reward(z, baseline=None):
    t=from_row(z)
    if not t['is_valid']:return 2.
    if t['electrical']=='violated':return 2.
    if any(v is None for v in t['design_rules'].values()):return None
    if baseline is not None:
        tb=from_row(baseline)
        if not comparison(tb,t)['not_worse']:return 2.
    vals=list(t['design_rules'].values())
    return sum(v/(1+v) for v in vals)/len(vals)


def mandatory_proofs(M,K):
    nw={}
    CONS=K.get('specs') or {}
    names=set(K.get('net_list',{})) | set(M.get('net_list',{}))
    si_fields=('z_ohm','z_diff_ohm','length_group','eye','noise_budget_mV')


    for pattern,values in (CONS.get('net_list') or {}).items():
        try:matched=any(pattern==n or re.fullmatch(pattern,n,re.I) or re.fullmatch(pattern,n.split('/')[-1],re.I) for n in names)
        except re.error:matched=False
        if not matched:
            for field in si_fields:
                if values.get(field) is not None:nw['si:spec:'+pattern+':'+field]=dict(status='unknown',reason='Given net missing in netclasses/measurement')
    for n in sorted(set(K.get('net_list',{})) | set(M.get('net_list',{}))):
        v=M.get('net_list',{}).get(n,{})
        m=v.get('m') or {}; k=K.get('net_list',{}).get(n,{})
        raw_data=m.get('safety_raw') or m
        if k.get('current_model')=='rectifier_Mean_RMS_unknown':
            nw['rms:'+n]=dict(status='unknown',reason='Rectifier waveform/PF missing')
        requested_I=count(k.get('current_A'))
        load_I=sum((k.get('flow') or {}).get('load',{}).values()) if (k.get('flow') or {}).get('unit')=='A' else 0.
        power_unknown=k.get('category') in ('current_val','supply','sw') and requested_I is None
        if (m.get('carries') and (raw_data.get('I_A',m.get('I_A')) or 0)>0) or (requested_I or load_I or 0)>0 or power_unknown:
            ampere=(k.get('flow') or {}).get('unit')=='A' or count(k.get('current_A')) is not None
            values={field:count(raw_data.get(field)) for field in ('I_A','J_x','J_via_x','dV_mV','s_dc_mV')}
            complete=all(x is not None and x>=0 for x in values.values()) and (values['s_dc_mV'] or 0)>0
            valid=ampere and complete and str(m.get('dV_source','')).startswith(('tree_idx','ir')) and not m.get('dc_load_unconnected')
            checks=[values['J_x'],values['J_via_x']]
            if (values['s_dc_mV'] or 0)>0 and values['dV_mV'] is not None:checks.append(values['dV_mV']/values['s_dc_mV'])
            bad=ampere and any(x is not None and x>1+1e-9 for x in checks)
            nw['dc:'+n]=dict(status='violated' if bad else ('fulfilled' if valid else 'unknown'), model='LadderNet/IPC-Proxy',reason='Ampere-Model' if ampere else 'Only normalized current scenario')
        if k.get('category') in ('fast','analog','kelvin'):
            final=M.get('reference',{}).get('category')=='final_copper'
            missing=count(raw_data.get('l_without_mm'))
            nw['reference:'+n]=dict(status=('violated' if missing>1e-9 else 'fulfilled') if final and missing is not None else 'unknown', reason=M.get('reference',{}).get('category'))
        nominal=dict(k)

        for pattern,values in ((K.get('specs') or {}).get('net_list') or {}).items():
            try:
                if pattern==n or re.fullmatch(pattern,n,re.I) or re.fullmatch(pattern,n.rsplit('/',1)[-1],re.I):nominal.update(values)
            except re.error:pass
        from analysis.signal_integrity import spec
        for fld in si_fields:
            if spec(K,n,fld,nominal.get(fld)) is not None:nw['si:'+n+':'+fld]=dict(status='unknown',reason='SI part provides mandatory evidence')

    for name,gr in M.get('group_list',{}).items():
        value=gr.get('total_sum_raw_mV',gr.get('total_sum_mV'));limit=gr.get('mV')
        nw['channel:'+name]=dict(status='unknown' if count(value) is None or not limit else ('violated' if value>limit+1e-9 else 'fulfilled'))
    for name,gr in (CONS.get('channel_budget_mV') or {}).items():
        if name not in M.get('group_list',{}) or any(n not in M.get('net_list',{}) for n in gr.get('net_list',[])):
            nw['channel:'+name]=dict(status='unknown',reason='Given channel group/measurement nets missing')
    if M.get('safety'):nw['minimax:safety']=dict(status={'safe':'fulfilled','uncertain':'violated'}.get(M['safety'].get('status'),'unknown') if M.get('minimax_version') else 'unknown',version=M.get('minimax_version'))
    if (M.get('ground') or {}).get('unknown'):nw['ground']=dict(status='unknown',reason='separate electrode/grid boundary')
    if K.get('net_ties') and not M.get('ground'):nw['ground']=dict(status='unknown',reason='no reliable measurement ground model')
    limit=(K.get('specs') or {}).get('decoupling_L_nH')
    loops={}
    for h in M.get('decoupling') or []:
        key=(h['chip'],h['net_name'],h['pin'])
        if key not in loops or (h['status']!='model',h['L_nH'])<(loops[key]['status']!='model',loops[key]['L_nH']):loops[key]=h
    expected={(p['chip'],p['net_name'],p['pin']) for p in M.get('decoupling_pins',[])} | set(loops)
    per_net={}
    for key in sorted(expected):
        per_net.setdefault(key[1],[]).append(key)
    for n,keys in per_net.items():
        states=[]
        for key in keys:
            h=loops.get(key,{})
            known=count(limit) is not None and limit>0 and h.get('status')=='model' and count(h.get('L_nH')) is not None and M.get('reference',{}).get('category')=='final_copper'
            states.append(('violated' if h['L_nH']>limit else 'fulfilled') if known else 'unknown')
        nw['decoupling:'+n]=dict(status='violated' if 'violated' in states else ('unknown' if 'unknown' in states else 'fulfilled'),
                                pins=[chip+':'+pin for chip,_,pin in keys],
                                reason='All expected supply pins: explicit L budget, connected final copper and C required')
    for key,value in (M.get('proofs') or {}).items():

        if key in nw and not key.startswith(('si:','minimax:')):continue
        adapter=dict(value)
        if not adapter.get('version'):
            adapter.update(status='unknown',reason='Version of the mandatory evidence adapter missing')
        nw[key]=adapter
    return nw or dict(electrics=dict(status='fulfilled',reason='no electrical requirements'))
