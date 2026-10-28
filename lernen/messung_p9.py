""
import collections
import glob
import hashlib
import json
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from lernen import dauerlauf as DL
from lernen.lenkung import code_von


def messen(folder):
    paths=sorted(glob.glob(os.path.join(folder,'*.jsonl')))
    Z=DL.zeilen_lesen(paths); DL.ZIEL['ziel']='beide'
    alt=DL.Lenkung(Z,methode='alt')
    out=dict(zeilen=len(Z),code=dict(collections.Counter(code_von(z) or 'unbekannt' for z in Z)),
             dateien={p:hashlib.sha256(open(p,'rb').read()).hexdigest() for p in paths},alt=alt.beste(),pro_code={})
    for code in sorted({code_von(z) for z in Z if code_von(z)}):
        L=DL.Lenkung(Z,code=code)
        report=L.additiv.export();report['beste']=L.beste(); contrasts={}
        for k,best in report['beste'].items():
            contrasts[k]={}
            for dim,(m,a,n) in best.items():
                baseline=DL.ARME[dim][0]
                contrasts[k][dim]=dict(arm=a,n=n,bezug=baseline,kontrast=L.additiv.kontrast(k,dim,a,baseline))
        report['vergleich']=contrasts;out['pro_code'][code]=report
    return out


if __name__=='__main__':
    json.dump(messen(sys.argv[1]),open(sys.argv[2],'w'),indent=1)
