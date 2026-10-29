#!/bin/zsh
set -e; setopt PIPE_FAIL
IN=${1:A}; OUT=${2:A}; shift 2
for a in "$@"; do eval "${a%%=*}='${a#*=}'"; done
[ -n "$design_rules" ] && design_rules=${design_rules:A}; [ -n "$netclasses" ] && netclasses=${netclasses:A}
R=${0:A:h}; VPY=$R/.venv/bin/python
KPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
KC=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
clr=${clr:-0.15}; min_width=${min_width:-0.15}
B=${IN%.kicad_pcb}; T=${OUT:h}/zones_tmp; mkdir -p $T; cd $T
if [ -n "$design_rules" ]; then
  eval $($VPY - $design_rules <<'PY'
import json, sys
r = json.load(open(sys.argv[1]))['raw_data']
v = r.get('netclass_via') or [0.45, 0.2]
print('drc=%g h2h=%g edge=%g hole_size=%g via=%g/%g' % (r.get('min_clearance', 0.1), r.get('min_hole_to_hole', 0.25), r.get('min_copper_edge_clearance', 0.3),
      r.get('min_hole_clearance', 0.25), v[0], v[1]))
PY
)
fi
drc=${drc:-0.1}; h2h=${h2h:-0.25}; edge=${edge:-0.3}; hole_size=${hole_size:-0.25}; via=${via:-0.45/0.2}
hole_r=$(python3 -c "print(round($hole_size + 0.015, 3))"); vacant=$(python3 -c "print(round($drc + 0.012, 3))"); clearance=$(python3 -c "print(round($clr + 0.005, 3))")
echo "Zone autorouter: drc $drc, h2h $h2h, edge $edge, drill $hole_size, via $via, zone spacing $clr, minimum width $min_width"
f() { grep -v -E "swig|wxApp|assert" || true }
pro() { cp $B.kicad_pro $1.kicad_pro; [ -f $B.kicad_dru ] && cp $B.kicad_dru $1.kicad_dru || true }
ex() { $KPY $R/kicad/copper_export.py $1.kicad_pcb cu_$1.json >/dev/null 2>&1; $KPY $R/kicad/zones_export.py $1.kicad_pcb zones_$1.json 2>&1 | f >/dev/null
       $KPY $R/kicad/keepout_export.py $1.kicad_pcb keepout_$1.json >/dev/null 2>&1 }
plan() { $KPY $R/kicad/zones_plan_apply.py $1.kicad_pcb $2.kicad_pcb $3 2>&1 | f; pro $2 }
GV="via=$via drc=$drc h2h=$h2h border=$edge"
if [ "${ab:-0}" -le 5 ]; then
cp $IN z0.kicad_pcb; pro z0
if [ "${shorten:-1}" = 1 ]; then
  eval $($KPY - z0.kicad_pcb "$netclasses" $R <<'PY' 2>/dev/null | tail -1
import pcbnew, sys, math
sys.path.insert(0, sys.argv[3]); from common.pair_nets import pair_nets, norm
b = pcbnew.LoadBoard(sys.argv[1]); mm = pcbnew.ToMM; bb = b.GetBoardEdgesBoundingBox()
net_list = sorted({t.GetNetname() for t in b.GetTracks() if t.GetNetname()}); P = pair_nets(net_list, sys.argv[2] or None)
cu_layers = ','.join(pcbnew.BOARD.GetStandardLayerName(l).replace('.Cu', '') for l in b.GetEnabledLayers().CuStack())
print("ANETS='%s' ABB='%d %d %d %d' ALAYERS=%s" % (';'.join(n for n in net_list if norm(n) not in P), math.floor(mm(bb.GetLeft())), math.floor(mm(bb.GetTop())),
      math.ceil(mm(bb.GetRight())), math.ceil(mm(bb.GetBottom())), cu_layers))
PY
)
  AK=$(DEAD=0 SHORTCUT_OPT="drc=$drc border=$edge hole_size=$hole_r hmax=${hmax:-0.15} cu_layers=$ALAYERS" $R/postprocessing/shorten_run.sh z0.kicad_pcb z0b.kicad_pcb $B.kicad_pro $B.kicad_dru "$ANETS" ${=ABB} 2>&1 | f)
  echo "$AK" | grep -E "^Round" | tail -1; echo "$AK" | grep -E "^DRC"
  if echo "$AK" | grep -q "^DRC errors {}"; then mv z0b.kicad_pcb z0.kicad_pcb; pro z0; else echo "Straightening rejected (DRC error)"; fi
  $KPY $R/kicad/obst_export.py z0.kicad_pcb ${=ABB} h_z0.json >/dev/null 2>&1; rm -f stairs.txt
  $VPY $R/analysis/stairs.py h_z0.json net_list=stairs.txt list_=0
  if [ -s stairs.txt ]; then
    AK=$(DEAD=0 SHORTCUT_OPT="drc=$drc border=$edge hole_size=$hole_r hmax=0 cu_layers=$ALAYERS" $R/postprocessing/shorten_run.sh z0.kicad_pcb z0b.kicad_pcb $B.kicad_pro $B.kicad_dru "$(cat stairs.txt)" ${=ABB} 2>&1 | f)
    echo "$AK" | grep -E "^DRC"
    if echo "$AK" | grep -q "^DRC errors {}"; then mv z0b.kicad_pcb z0.kicad_pcb; pro z0; $KPY $R/kicad/obst_export.py z0.kicad_pcb ${=ABB} h_z0.json >/dev/null 2>&1; $VPY $R/analysis/stairs.py h_z0.json list_=3
    else echo "Staircase smoothing rejected (DRC error)"; fi
  fi
fi
$KPY $R/kicad/zones_mode.py z0.kicad_pcb z1.kicad_pcb mode=keep splitter_way=1 min_width=$min_width min_width_all=$min_width 2>&1 | f; pro z1
ex z1; $VPY $R/postprocessing/gnd_via_zone.py cu_z1.json keepout_z1.json vias1.json ${=GV} one=1 | tail -1
$KPY $R/kicad/vias_set.py z1.kicad_pcb vias1.json z2.kicad_pcb via=$via 2>&1 | f; pro z2
$VPY $R/postprocessing/gnd_islands.py z2.kicad_pcb z3.kicad_pcb via=$via ${netclasses:+netclasses=$netclasses} drc=$drc h2h=$h2h edge=$edge splitter=0 2>&1 | f | grep -E "GND islands" || true; pro z3
$KPY $R/kicad/vias_keepout_remove.py z3.kicad_pcb z4.kicad_pcb 2>&1 | f | tail -1; pro z4
$KPY $R/kicad/zones_mode.py z4.kicad_pcb z4.kicad_pcb mode=always 2>&1 | f; pro z4
ex z4; $VPY $R/postprocessing/zones_grow.py cu_z4.json zones_z4.json p5.json keepout_z4.json clr=$clr border=$edge min=0.02 | tail -1; plan z4 z5 p5.json
$KPY $R/kicad/zones_export.py z5.kicad_pcb zones_z5.json 2>&1 | f >/dev/null
$VPY $R/postprocessing/zones_octi.py zones_z5.json p6.json ${OCTI:+octi=1} | tail -1; $VPY $R/postprocessing/zones_octi.py zones_z5.json p6b.json tol=0.12 hmax=0.45 r=0.15 ${OCTI:+octi=1} | tail -1
python3 -c "
import json; a=json.load(open('p6.json')); b=json.load(open('p6b.json'))
[a['zone_list'].setdefault(u,p) for u,p in b['zone_list'].items()]; json.dump(a, open('p6.json','w'))"
plan z5 z6 p6.json
ex z6; $VPY $R/postprocessing/zones_dock.py cu_z6.json zones_z6.json p7.json clr=$clr | tail -1; plan z6 z7 p7.json
ex z7; $VPY $R/postprocessing/pockets_zones.py cu_z7.json keepout_z7.json zones_z7.json p8.json clr=$clr ${netclasses:+netclasses=$netclasses} ${=GV} hole_size=$hole_size | tail -1; plan z7 z8 p8.json
$KPY $R/kicad/zones_export.py z8.kicad_pcb zones_z8.json 2>&1 | f >/dev/null; $VPY $R/postprocessing/zones_merge.py zones_z8.json p9.json | tail -1; plan z8 z9 p9.json
ex z9; $VPY $R/postprocessing/gaps_vias.py cu_z9.json keepout_z9.json vias2.json ${=GV} clr=$clr hole_size=$hole_size | tail -1
$KPY $R/kicad/vias_set.py z9.kicad_pcb vias2.json z10.kicad_pcb via=$via 2>&1 | f; pro z10
ex z10; $VPY $R/postprocessing/zones_dock.py cu_z10.json zones_z10.json p11.json clr=$clr | tail -1; plan z10 z11 p11.json
$KPY $R/kicad/zones_export.py z11.kicad_pcb zones_z11.json 2>&1 | f >/dev/null; $VPY $R/postprocessing/zones_merge.py zones_z11.json p12.json | tail -1; plan z11 z12 p12.json
fi
$KPY $R/kicad/shapes_filter.py z12.kicad_pcb z13.kicad_pcb 2>&1 | f; pro z13
tex() { $KPY $R/kicad/fillet_post_run.py export $1.kicad_pcb k_$1.json 2>&1 | f >/dev/null; $KPY $R/kicad/copper_export.py $1.kicad_pcb cu_$1.json >/dev/null 2>&1
        $KPY $R/kicad/keepout_export.py $1.kicad_pcb keepout_$1.json >/dev/null 2>&1 }
CLASS_OPT=${netclasses:+netclasses=$netclasses}
TOPT="r=0.4 vacant=$clearance free_hard=$clearance border=$edge hole_size=$hole_r $CLASS_OPT"
tex z13; $VPY $R/postprocessing/white_teardrop.py k_z13.json keepout_z13.json t1.json ${=TOPT} net_list=signal zone_list=soft pads=cu_z13.json | tail -1
$KPY $R/kicad/shapes_append.py z13.kicad_pcb t1.json z14.kicad_pcb 2>&1 | f; pro z14
tex z14; $VPY $R/postprocessing/white_teardrop.py k_z14.json keepout_z14.json t2.json ${=TOPT} net_list=surface pads=cu_z14.json | tail -1
$KPY $R/kicad/shapes_append.py z14.kicad_pcb t2.json z15.kicad_pcb 2>&1 | f; pro z15
tex z15; $VPY $R/postprocessing/notches_fill.py k_z15.json keepout_z15.json t3.json free_hard=$vacant vacant=$clearance border=$edge hole_size=$hole_r pads=cu_z15.json ${=CLASS_OPT} | tail -1
$KPY $R/kicad/shapes_append.py z15.kicad_pcb t3.json z16.kicad_pcb 2>&1 | f; pro z16
$KPY $R/kicad/teardrops_loose_remove.py z16.kicad_pcb z14.kicad_pcb 2>&1 | f; pro z14
cp z14.kicad_pcb $OUT; cp $B.kicad_pro ${OUT%.kicad_pcb}.kicad_pro; [ -f $B.kicad_dru ] && cp $B.kicad_dru ${OUT%.kicad_pcb}.kicad_dru || true
$KC pcb drc --format json --severity-all -o drc_z14.json z14.kicad_pcb >/dev/null 2>&1
python3 - drc_z14.json <<'PY'
import json, sys, collections
d = json.load(open(sys.argv[1]))
print('DRC: errors', dict(collections.Counter(v['type'] for v in d['violations'] if v['severity'] == 'error')), 'not_connected', len(d['unconnected_items']),
      'Warnings', sum(1 for v in d['violations'] if v['severity'] == 'warning'))
for v in d['violations'][:10]:
    if v['severity'] == 'error': print('  ', v['type'], v['description'][:70], [(i['description'][:45], i['pos']['x'], i['pos']['y']) for i in v['items']])
PY
ex z14; $VPY $R/analysis/white_strict.py cu_z14.json keepout_z14.json white.png clr=$clr tol=0.01 | tail -3
$KPY $R/kicad/fillet_post_run.py export z14.kicad_pcb k_end.json 2>&1 | f >/dev/null; $VPY $R/analysis/white_notches.py k_end.json keepout_z14.json list_=5
$KPY $R/kicad/zones_export.py z14.kicad_pcb zones_end.json 2>&1 | f >/dev/null; $VPY $R/analysis/white_corners.py k_end.json keepout_z14.json zone_list=zones_end.json ${=CLASS_OPT} pads=cu_z14.json list_=8
echo "-> $OUT"
