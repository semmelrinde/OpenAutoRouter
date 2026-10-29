import glob
import json
import os
import subprocess
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from learning import endurance_run as DL
from learning.levels import off_original
from analysis import corpus as KO


def inventory(train, originals, out):
    os.makedirs(out,exist_ok=True); result=[]
    for d in sorted(glob.glob(os.path.join(train,'*'))):
        try: m=json.load(open(os.path.join(d,'metadata.json')))
        except (OSError,ValueError):continue
        if int(m.get('layers') or 2)!=4:continue
        ok,reason=DL.board_ok(d,220,only2=False); n=os.path.basename(d)
        row=dict(board=n,suitable_in_advance=ok,reason=reason,pads=open(os.path.join(d,'processed.kicad_pcb')).read().count('(pad '))
        result.append(row)
        if not ok:continue
        w=os.path.join(out,n);os.makedirs(w,exist_ok=True)
        pcb=os.path.join(d,'processed.kicad_pcb');bj=os.path.join(w,'board.json')
        r=subprocess.run([KO.KPY,os.path.join(KO.R,'kicad/export_board.py'),pcb,bj],capture_output=True,text=True,timeout=300)
        if r.returncode:row['faults']=r.stderr[-300:];continue
        B=json.load(open(bj)); RG=KO.design_rules_off(pcb,B['cu_layers'],B);json.dump(RG,open(os.path.join(w,'design_rules.json'),'w'))
        vj=os.path.join(w,'specs.json');json.dump({},open(vj,'w'))
        K=KO.netclasses_run(w,bj,os.path.join(w,'netclasses.json'),vj)
        source=os.path.join(originals,n,'original.kicad_pcb')
        for label,p in [('local',pcb),('original',source)]:
            if not os.path.exists(p):row[label]=dict(absent=True);continue
            try:
                layers,zones=off_original(p,B,K,w)
                row[label]=dict(gnd_layers=layers,zone_list=len(zones),pads_identical=True,source=p)
            except Exception as e:row[label]=dict(faults=str(e))
    return result


if __name__=='__main__':
    json.dump(inventory(*sys.argv[1:4]),open(os.path.join(sys.argv[3],'inventory.json'),'w'),indent=1)
