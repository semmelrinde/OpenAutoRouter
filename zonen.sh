#!/bin/zsh
set -e; setopt PIPE_FAIL
IN=${1:A}; OUT=${2:A}; shift 2
for a in "$@"; do eval "${a%%=*}='${a#*=}'"; done
[ -n "$regeln" ] && regeln=${regeln:A}; [ -n "$klassen" ] && klassen=${klassen:A}
R=${0:A:h}; VPY=$R/.venv/bin/python
KPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
KC=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
clr=${clr:-0.15}; minw=${minw:-0.15}
B=${IN%.kicad_pcb}; T=${OUT:h}/zonen_tmp; mkdir -p $T; cd $T
if [ -n "$regeln" ]; then
  eval $($VPY - $regeln <<'PY'
import json, sys
r = json.load(open(sys.argv[1]))['roh']
v = r.get('netclass_via') or [0.45, 0.2]
print('drc=%g h2h=%g edge=%g loch=%g via=%g/%g' % (r.get('min_clearance', 0.1), r.get('min_hole_to_hole', 0.25), r.get('min_copper_edge_clearance', 0.3),
      r.get('min_hole_clearance', 0.25), v[0], v[1]))
PY
)
fi
drc=${drc:-0.1}; h2h=${h2h:-0.25}; edge=${edge:-0.3}; loch=${loch:-0.25}; via=${via:-0.45/0.2}
lochr=$(python3 -c "print(round($loch + 0.015, 3))"); frei=$(python3 -c "print(round($drc + 0.012, 3))"); freiz=$(python3 -c "print(round($clr + 0.005, 3))")
echo "Zonen-Autorouter: drc $drc, h2h $h2h, Rand $edge, Bohrung $loch, Via $via, Zonenabstand $clr, Mindestbreite $minw"
f() { grep -v -E "swig|wxApp|assert" || true }
pro() { cp $B.kicad_pro $1.kicad_pro; [ -f $B.kicad_dru ] && cp $B.kicad_dru $1.kicad_dru || true }
ex() { $KPY $R/kicad/kupfer_export.py $1.kicad_pcb cu_$1.json >/dev/null 2>&1; $KPY $R/kicad/zonen_export.py $1.kicad_pcb zonen_$1.json 2>&1 | f >/dev/null
       $KPY $R/kicad/sperr_export.py $1.kicad_pcb sperr_$1.json >/dev/null 2>&1 }
plan() { $KPY $R/kicad/zonen_plan_anwenden.py $1.kicad_pcb $2.kicad_pcb $3 2>&1 | f; pro $2 }
GV="via=$via drc=$drc h2h=$h2h rand=$edge"
if [ "${ab:-0}" -le 5 ]; then                    # ab=6: Schritte 0-5 ueberspringen, auf zonen_tmp/z12 aufsetzen (nur Tropfen/Kerben neu)
cp $IN z0.kicad_pcb; pro z0
if [ "${abkuerzen:-1}" = 1 ]; then
  eval $($KPY - z0.kicad_pcb "$klassen" $R <<'PY' 2>/dev/null | tail -1
import pcbnew, sys, math
sys.path.insert(0, sys.argv[3]); from gemeinsam.paarnetze import paarnetze, norm
b = pcbnew.LoadBoard(sys.argv[1]); mm = pcbnew.ToMM; bb = b.GetBoardEdgesBoundingBox()
netze = sorted({t.GetNetname() for t in b.GetTracks() if t.GetNetname()}); P = paarnetze(netze, sys.argv[2] or None)
lagen = ','.join(pcbnew.BOARD.GetStandardLayerName(l).replace('.Cu', '') for l in b.GetEnabledLayers().CuStack())
print("ANETZE='%s' ABB='%d %d %d %d' ALAGEN=%s" % (';'.join(n for n in netze if norm(n) not in P), math.floor(mm(bb.GetLeft())), math.floor(mm(bb.GetTop())),
      math.ceil(mm(bb.GetRight())), math.ceil(mm(bb.GetBottom())), lagen))
PY
)
  AK=$(TOTE=0 ABK_OPT="drc=$drc rand=$edge loch=$lochr hmax=${hmax:-0.15} lagen=$ALAGEN" $R/nachbearbeitung/abkuerzen_lauf.sh z0.kicad_pcb z0b.kicad_pcb $B.kicad_pro $B.kicad_dru "$ANETZE" ${=ABB} 2>&1 | f)
  echo "$AK" | grep -E "^Runde" | tail -1; echo "$AK" | grep -E "^DRC"
  if echo "$AK" | grep -q "^DRC Fehler {}"; then mv z0b.kicad_pcb z0.kicad_pcb; pro z0; else echo "Begradigen verworfen (DRC-Fehler)"; fi
  $KPY $R/kicad/hind_export.py z0.kicad_pcb ${=ABB} h_z0.json >/dev/null 2>&1; rm -f treppen.txt
  $VPY $R/analyse/treppen.py h_z0.json netze=treppen.txt liste=0
  if [ -s treppen.txt ]; then
    AK=$(TOTE=0 ABK_OPT="drc=$drc rand=$edge loch=$lochr hmax=0 lagen=$ALAGEN" $R/nachbearbeitung/abkuerzen_lauf.sh z0.kicad_pcb z0b.kicad_pcb $B.kicad_pro $B.kicad_dru "$(cat treppen.txt)" ${=ABB} 2>&1 | f)
    echo "$AK" | grep -E "^DRC"
    if echo "$AK" | grep -q "^DRC Fehler {}"; then mv z0b.kicad_pcb z0.kicad_pcb; pro z0; $KPY $R/kicad/hind_export.py z0.kicad_pcb ${=ABB} h_z0.json >/dev/null 2>&1; $VPY $R/analyse/treppen.py h_z0.json liste=3
    else echo "Treppen-Glaetten verworfen (DRC-Fehler)"; fi
  fi
fi
$KPY $R/kicad/zonen_modus.py z0.kicad_pcb z1.kicad_pcb modus=keep splitter_weg=1 minw=$minw minw_alle=$minw 2>&1 | f; pro z1
ex z1; $VPY $R/nachbearbeitung/gnd_via_zone.py cu_z1.json sperr_z1.json vias1.json ${=GV} eins=1 | tail -1
$KPY $R/kicad/vias_setzen.py z1.kicad_pcb vias1.json z2.kicad_pcb via=$via 2>&1 | f; pro z2
$VPY $R/nachbearbeitung/gnd_inseln.py z2.kicad_pcb z3.kicad_pcb via=$via ${klassen:+klassen=$klassen} drc=$drc h2h=$h2h edge=$edge splitter=0 2>&1 | f | grep -E "GND-Inseln" || true; pro z3
$KPY $R/kicad/vias_sperre_weg.py z3.kicad_pcb z4.kicad_pcb 2>&1 | f | tail -1; pro z4
$KPY $R/kicad/zonen_modus.py z4.kicad_pcb z4.kicad_pcb modus=always 2>&1 | f; pro z4
ex z4; $VPY $R/nachbearbeitung/zonen_wachsen.py cu_z4.json zonen_z4.json p5.json sperr_z4.json clr=$clr rand=$edge min=0.02 | tail -1; plan z4 z5 p5.json
$KPY $R/kicad/zonen_export.py z5.kicad_pcb zonen_z5.json 2>&1 | f >/dev/null
$VPY $R/nachbearbeitung/zonen_octi.py zonen_z5.json p6.json ${OCTI:+octi=1} | tail -1; $VPY $R/nachbearbeitung/zonen_octi.py zonen_z5.json p6b.json tol=0.12 hmax=0.45 r=0.15 ${OCTI:+octi=1} | tail -1
python3 -c "
import json; a=json.load(open('p6.json')); b=json.load(open('p6b.json'))
[a['zonen'].setdefault(u,p) for u,p in b['zonen'].items()]; json.dump(a, open('p6.json','w'))"
plan z5 z6 p6.json
ex z6; $VPY $R/nachbearbeitung/zonen_andocken.py cu_z6.json zonen_z6.json p7.json clr=$clr | tail -1; plan z6 z7 p7.json
ex z7; $VPY $R/nachbearbeitung/taschen_zonen.py cu_z7.json sperr_z7.json zonen_z7.json p8.json clr=$clr ${klassen:+klassen=$klassen} ${=GV} loch=$loch | tail -1; plan z7 z8 p8.json
$KPY $R/kicad/zonen_export.py z8.kicad_pcb zonen_z8.json 2>&1 | f >/dev/null; $VPY $R/nachbearbeitung/zonen_merge.py zonen_z8.json p9.json | tail -1; plan z8 z9 p9.json
ex z9; $VPY $R/nachbearbeitung/luecken_vias.py cu_z9.json sperr_z9.json vias2.json ${=GV} clr=$clr loch=$loch | tail -1
$KPY $R/kicad/vias_setzen.py z9.kicad_pcb vias2.json z10.kicad_pcb via=$via 2>&1 | f; pro z10
ex z10; $VPY $R/nachbearbeitung/zonen_andocken.py cu_z10.json zonen_z10.json p11.json clr=$clr | tail -1; plan z10 z11 p11.json
$KPY $R/kicad/zonen_export.py z11.kicad_pcb zonen_z11.json 2>&1 | f >/dev/null; $VPY $R/nachbearbeitung/zonen_merge.py zonen_z11.json p12.json | tail -1; plan z11 z12 p12.json
fi
$KPY $R/kicad/formen_filter.py z12.kicad_pcb z13.kicad_pcb 2>&1 | f; pro z13        # generierte Tropfen weg, Design-Kupfer (>= 0,2 mm²) bleibt
tex() { $KPY $R/kicad/fillet_nachlauf.py export $1.kicad_pcb k_$1.json 2>&1 | f >/dev/null; $KPY $R/kicad/kupfer_export.py $1.kicad_pcb cu_$1.json >/dev/null 2>&1
        $KPY $R/kicad/sperr_export.py $1.kicad_pcb sperr_$1.json >/dev/null 2>&1 }
KOPT=${klassen:+klassen=$klassen}                                   # Differenzpaare aus klassen.json (sonst Namensregel, gemeinsam/paarnetze.py)
TOPT="r=0.4 frei=$freiz frei_hart=$freiz rand=$edge loch=$lochr $KOPT"
tex z13; $VPY $R/nachbearbeitung/weiss_tropfen.py k_z13.json sperr_z13.json t1.json ${=TOPT} netze=signal zonen=weich pads=cu_z13.json | tail -1
$KPY $R/kicad/formen_anhaengen.py z13.kicad_pcb t1.json z14.kicad_pcb 2>&1 | f; pro z14          # erst Signalnetze, neu fuellen ...
tex z14; $VPY $R/nachbearbeitung/weiss_tropfen.py k_z14.json sperr_z14.json t2.json ${=TOPT} netze=flaeche pads=cu_z14.json | tail -1
$KPY $R/kicad/formen_anhaengen.py z14.kicad_pcb t2.json z15.kicad_pcb 2>&1 | f; pro z15          # ... dann Flaechennetze auf der endgueltigen Fuellung
tex z15; $VPY $R/nachbearbeitung/kerben_fuellen.py k_z15.json sperr_z15.json t3.json frei_hart=$frei frei=$freiz rand=$edge loch=$lochr pads=cu_z15.json ${=KOPT} | tail -1
$KPY $R/kicad/formen_anhaengen.py z15.kicad_pcb t3.json z16.kicad_pcb 2>&1 | f; pro z16
$KPY $R/kicad/tropfen_lose_weg.py z16.kicad_pcb z14.kicad_pcb 2>&1 | f; pro z14                    # Ergebnis heisst wieder z14 (Bericht unten)
cp z14.kicad_pcb $OUT; cp $B.kicad_pro ${OUT%.kicad_pcb}.kicad_pro; [ -f $B.kicad_dru ] && cp $B.kicad_dru ${OUT%.kicad_pcb}.kicad_dru || true
$KC pcb drc --format json --severity-all -o drc_z14.json z14.kicad_pcb >/dev/null 2>&1
python3 - drc_z14.json <<'PY'
import json, sys, collections
d = json.load(open(sys.argv[1]))
print('DRC: Fehler', dict(collections.Counter(v['type'] for v in d['violations'] if v['severity'] == 'error')), 'unverbunden', len(d['unconnected_items']),
      'Warnungen', sum(1 for v in d['violations'] if v['severity'] == 'warning'))
for v in d['violations'][:10]:
    if v['severity'] == 'error': print('  ', v['type'], v['description'][:70], [(i['description'][:45], i['pos']['x'], i['pos']['y']) for i in v['items']])
PY
ex z14; $VPY $R/analyse/weiss_streng.py cu_z14.json sperr_z14.json weiss.png clr=$clr tol=0.01 | tail -3
$KPY $R/kicad/fillet_nachlauf.py export z14.kicad_pcb k_end.json 2>&1 | f >/dev/null; $VPY $R/analyse/weiss_kerben.py k_end.json sperr_z14.json liste=5
$KPY $R/kicad/zonen_export.py z14.kicad_pcb zonen_end.json 2>&1 | f >/dev/null; $VPY $R/analyse/weiss_ecken.py k_end.json sperr_z14.json zonen=zonen_end.json ${=KOPT} pads=cu_z14.json liste=8
echo "-> $OUT"
