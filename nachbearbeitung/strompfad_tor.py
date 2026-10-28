""
import json
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from analyse.bewertung import bewerte
from analyse.topologie import topologie

TOPO=('parallel_rest_mm','stern','spitz','kelvin_nah_mm','abgriff_nah_mm',
      'abgriff_via_im_pad','kelvin_neben_strom_mm')


def pruefe(B,R0,R1,K,D0,D1):
    E0,E1=[bewerte(B,R,K,v3=True) for R in (R0,R1)]
    T0,T1=[topologie(R,B) for R in (R0,R1)]
    def drc(d):
        return dict(fehler=sum(v['severity']=='error' for v in d['violations']),offen=len(d['unconnected_items']))
    C0,C1=drc(D0),drc(D1)
    reasons=[]
    if C1['fehler']>C0['fehler'] or C1['offen']>C0['offen']:
        reasons.append('KiCad-DRC schlechter')
    if E0['gueltig'] and not E1['gueltig']:
        reasons.append('JSON-Verbindungstor schlechter')
    if E1['note_v3']>E0['note_v3']+1e-9:
        reasons.append('Note v3 schlechter')
    for key in TOPO:
        if T1[key]>T0[key]+1e-9:
            reasons.append('Topologie schlechter: '+key)

    for n in sorted(set(E0['metriken']['netze'])|set(E1['metriken']['netze'])):
        for key in ('xt_opfer','xt_stoer'):
            a=(E0['metriken']['netze'].get(n) or {}).get('m',{}).get(key) or 0.
            b=(E1['metriken']['netze'].get(n) or {}).get('m',{}).get(key) or 0.
            if b>a+1e-9:
                reasons.append('Kopplung schlechter: '+n+' '+key)
    return dict(uebernehmen=not reasons,gruende=reasons,alt=dict(drc=C0,bewertung=E0,topologie=T0),
                neu=dict(drc=C1,bewertung=E1,topologie=T1),
                grenze='Note/Topologie/Kopplung benutzen Bahngeruest, keine Flaechenkanten; DC separat im nativen Kupfer.')


def main():
    b,r0,r1,k,d0,d1,out=sys.argv[1:8]
    B,R0,R1,K,D0,D1=[json.loads(Path(f).read_text()) for f in (b,r0,r1,k,d0,d1)]
    result=pruefe(B,R0,R1,K,D0,D1)
    Path(out).write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
    print('Stromflaechen-Tor:', 'uebernehmen' if result['uebernehmen'] else 'verwerfen',
          'Note v3',result['alt']['bewertung']['note_v3'],'->',result['neu']['bewertung']['note_v3'],
          '; '.join(result['gruende'][:4]))
    return 0 if result['uebernehmen'] else 1


if __name__=='__main__':sys.exit(main())
