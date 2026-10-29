import json
import math
import sys
from pathlib import Path


def table(prior, new, group_list=None):
    def value_list(dc):
        return {(n,p):v['R_mOhm'] for n,a in dc['net_list'].items() for p,v in a['pads'].items()}
    a,b=value_list(prior),value_list(new)
    if group_list is None:
        group_list={n+' '+p:[[n,p]] for n,p in sorted(set(a)|set(b))}


        for n in sorted(set(prior['net_list'])|set(new['net_list'])):
            if not any(k[0]==n for k in set(a)|set(b)):
                group_list[n]=[]
    rows=[]
    for name,keys in group_list.items():
        keys=[tuple(k) for k in keys]
        vals=[sum(m[k] for k in keys) if keys and all(k in m and math.isfinite(m[k]) for k in keys) else None for m in (a,b)]
        x,y=vals
        rows.append(dict(channel=name,contacts=keys,prev_mOhm=x,new_mOhm=y,
                         delta_percent=100*(y/x-1) if x and y is not None else None,
                         status='ok' if x is not None and y is not None else 'not measured in 2D'))
    result=dict(model=prior['model'],raster_prev=prior['h_mm'],raster_new=new['h_mm'],
                copper_thickness_prev=prior['t_um'],copper_thickness_new=new['t_um'],channels=rows,
                error_prev=prior['faults'],error_new=new['faults'])
    for side,index in (('prior',0),('new',1)):
        values=[row[side+'_mOhm'] for row in rows]
        if values and all(v is not None and v>0 for v in values):
            mean=sum(values)/len(values)
            result['equality_'+side]=dict(max_min=max(values)/min(values),
                cv=(sum((v-mean)**2 for v in values)/len(values))**.5/mean)
    return result


def markdown(result):
    fmt=lambda v:'—' if v is None else '%.3f'%v
    lines=['2D direct current in native filled copper (model values, no hardware measurement).',
           '', '| Channel / load pad | old mΩ | new mΩ | Δ % | Status |',
           '|---|---:|---:|---:|---|']
    lines += ['| %s | %s | %s | %s | %s |'%(r['channel'],fmt(r['prev_mOhm']),fmt(r['new_mOhm']),fmt(r['delta_percent']),r['status']) for r in result['channels']]
    for side in ('prior','new'):
        if result['error_'+side]:
            lines+=['',side+': '+ '; '.join(v['net']+(' '+v['pad'] if 'pad' in v else '')+': '+v['reason'] for v in result['error_'+side])]
    return '\n'.join(lines)+'\n'


def main():
    a,b,out,md=sys.argv[1:5]
    opts=dict(v.split('=',1) for v in sys.argv[5:])
    groups=json.loads(Path(opts['channels']).read_text()) if 'channels' in opts else None
    result=table(json.loads(Path(a).read_text()),json.loads(Path(b).read_text()),groups)
    Path(out).write_text(json.dumps(result,indent=2,sort_keys=True,ensure_ascii=False)+'\n')
    Path(md).write_text(markdown(result))
    return 0


if __name__=='__main__':sys.exit(main())
