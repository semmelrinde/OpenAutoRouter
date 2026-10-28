""
import json
import math
import sys
from pathlib import Path


def tabelle(alt, neu, gruppen=None):
    def werte(dc):
        return {(n,p):v['R_mOhm'] for n,a in dc['netze'].items() for p,v in a['pads'].items()}
    a,b=werte(alt),werte(neu)
    if gruppen is None:
        gruppen={n+' '+p:[[n,p]] for n,p in sorted(set(a)|set(b))}


        for n in sorted(set(alt['netze'])|set(neu['netze'])):
            if not any(k[0]==n for k in set(a)|set(b)):
                gruppen[n]=[]
    rows=[]
    for name,keys in gruppen.items():
        keys=[tuple(k) for k in keys]
        vals=[sum(m[k] for k in keys) if keys and all(k in m and math.isfinite(m[k]) for k in keys) else None for m in (a,b)]
        x,y=vals
        rows.append(dict(kanal=name,kontakte=keys,alt_mOhm=x,neu_mOhm=y,
                         delta_prozent=100*(y/x-1) if x and y is not None else None,
                         status='ok' if x is not None and y is not None else 'nicht 2D gemessen'))
    result=dict(modell=alt['modell'],raster_alt=alt['h_mm'],raster_neu=neu['h_mm'],
                kupferdicke_alt=alt['t_um'],kupferdicke_neu=neu['t_um'],kanaele=rows,
                fehler_alt=alt['fehler'],fehler_neu=neu['fehler'])
    for side,index in (('alt',0),('neu',1)):
        values=[row[side+'_mOhm'] for row in rows]
        if values and all(v is not None and v>0 for v in values):
            mean=sum(values)/len(values)
            result['gleichheit_'+side]=dict(max_min=max(values)/min(values),
                cv=(sum((v-mean)**2 for v in values)/len(values))**.5/mean)
    return result


def markdown(result):
    fmt=lambda v:'—' if v is None else '%.3f'%v
    lines=['2D-Gleichstrom im nativen gefuellten Kupfer (Modellwerte, keine Hardwaremessung).',
           '', '| Kanal / Lastpad | alt mΩ | neu mΩ | Δ % | Status |',
           '|---|---:|---:|---:|---|']
    lines += ['| %s | %s | %s | %s | %s |'%(r['kanal'],fmt(r['alt_mOhm']),fmt(r['neu_mOhm']),fmt(r['delta_prozent']),r['status']) for r in result['kanaele']]
    for side in ('alt','neu'):
        if result['fehler_'+side]:
            lines+=['',side+': '+ '; '.join(v['net']+(' '+v['pad'] if 'pad' in v else '')+': '+v['grund'] for v in result['fehler_'+side])]
    return '\n'.join(lines)+'\n'


def main():
    a,b,out,md=sys.argv[1:5]
    opts=dict(v.split('=',1) for v in sys.argv[5:])
    groups=json.loads(Path(opts['kanaele']).read_text()) if 'kanaele' in opts else None
    result=tabelle(json.loads(Path(a).read_text()),json.loads(Path(b).read_text()),groups)
    Path(out).write_text(json.dumps(result,indent=2,sort_keys=True,ensure_ascii=False)+'\n')
    Path(md).write_text(markdown(result))
    return 0


if __name__=='__main__':sys.exit(main())
