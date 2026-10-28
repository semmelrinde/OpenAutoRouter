""
import glob
import json
import os
import subprocess
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from lernen import dauerlauf as DL
from lernen.ebenen import aus_original
from analyse import korpus as KO


def inventar(train, originale, out):
    os.makedirs(out,exist_ok=True); result=[]
    for d in sorted(glob.glob(os.path.join(train,'*'))):
        try: m=json.load(open(os.path.join(d,'metadata.json')))
        except (OSError,ValueError):continue
        if int(m.get('layers') or 2)!=4:continue
        ok,grund=DL.board_ok(d,220,nur2=False); n=os.path.basename(d)
        row=dict(board=n,geeignet_vorab=ok,grund=grund,pads=open(os.path.join(d,'processed.kicad_pcb')).read().count('(pad '))
        result.append(row)
        if not ok:continue
        w=os.path.join(out,n);os.makedirs(w,exist_ok=True)
        pcb=os.path.join(d,'processed.kicad_pcb');bj=os.path.join(w,'board.json')
        r=subprocess.run([KO.KPY,os.path.join(KO.R,'kicad/export_board.py'),pcb,bj],capture_output=True,text=True,timeout=300)
        if r.returncode:row['fehler']=r.stderr[-300:];continue
        B=json.load(open(bj)); RG=KO.regeln_aus(pcb,B['lagen'],B);json.dump(RG,open(os.path.join(w,'regeln.json'),'w'))
        vj=os.path.join(w,'vorgaben.json');json.dump({},open(vj,'w'))
        K=KO.klassen_lauf(w,bj,os.path.join(w,'klassen.json'),vj)
        source=os.path.join(originale,n,'original.kicad_pcb')
        for label,p in [('lokal',pcb),('original',source)]:
            if not os.path.exists(p):row[label]=dict(fehlend=True);continue
            try:
                layers,zones=aus_original(p,B,K,w)
                row[label]=dict(gndlagen=layers,zonen=len(zones),pads_identisch=True,quelle=p)
            except Exception as e:row[label]=dict(fehler=str(e))
    return result


if __name__=='__main__':
    json.dump(inventar(*sys.argv[1:4]),open(os.path.join(sys.argv[3],'inventar.json'),'w'),indent=1)
