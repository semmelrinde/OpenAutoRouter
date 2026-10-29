import collections
import glob
import hashlib
import json
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from learning import endurance_run as DL
from learning.steering import code_from


def measure(folder):
    paths=sorted(glob.glob(os.path.join(folder,'*.jsonl')))
    Z=DL.lines_read(paths); DL.GOAL['goal']='both'
    prior=DL.Steering(Z,method='prior')
    out=dict(lines=len(Z),code=dict(collections.Counter(code_from(z) or 'unknown' for z in Z)),
             files={p:hashlib.sha256(open(p,'rb').read()).hexdigest() for p in paths},prior=prior.best(),pro_code={})
    for code in sorted({code_from(z) for z in Z if code_from(z)}):
        L=DL.Steering(Z,code=code)
        report=L.additive.export();report['best']=L.best(); contrasts={}
        for k,best in report['best'].items():
            contrasts[k]={}
            for dim,(m,a,n) in best.items():
                baseline=DL.ARMS[dim][0]
                contrasts[k][dim]=dict(arm=a,n=n,baseline=baseline,contrast=L.additive.contrast(k,dim,a,baseline))
        report['comparison']=contrasts;out['pro_code'][code]=report
    return out


if __name__=='__main__':
    json.dump(measure(sys.argv[1]),open(sys.argv[2],'w'),indent=1)
