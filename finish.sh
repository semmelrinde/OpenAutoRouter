#!/bin/zsh
set -e
TMPD=$(mktemp -d)
D=$1; RJ=$2; N=$3; NL=${4:-4}; VIA=${5:-0.45/0.20}; GND_LAYERS=${6:-In1}; BASE=${BASE:-base_${NL}cu_layers}
R=${0:A:h}
VPY=$R/.venv/bin/python
KPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
KC=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
INK=/Applications/Inkscape.app/Contents/MacOS/inkscape
kpy() { ( ulimit -S -t ${NATIVE_CPU_MAX:-300}; $KPY "$@" ); }
kc() { ( ulimit -S -t ${NATIVE_CPU_MAX:-300}; $KC "$@" ); }
cd $D
filt() { grep -v -E 'swig|assert|wxApp|Debug: Adding duplicate' || true }
BJ=${BOARD:-${BJ:-board.json}}
RG=${DESIGN_RULES:-design_rules.json}; [ -f "$RG" ] || RG=""
eval "$(python3 - "${RG:--}" "${ZONE_EDGE:-0.3}" <<'EOF'
import json, sys
r = {}
if sys.argv[1] != '-':
    try: r = json.load(open(sys.argv[1]))['router']
    except Exception: r = {}
f = lambda k, d: float(r.get(k) or d)
drc = f('drc', 0.09)
print('R_DRC=%g R_DRCPAD=%g R_EDGE=%g R_H2H=%g R_HOLE=%g' % (drc, f('drcpad', max(drc, 0.10)), f('edge', 0.2), f('h2h', 0.2), f('hole_clr', 0.2)))
print('R_ZEDGE=%g R_TEARDROP_FREE=%g R_SVCLR=%g' % (max(float(sys.argv[2]), f('edge', 0.2) + 0.02), max(0.11, drc + 0.02), max(0.12, drc + 0.03)))
EOF
)"
if [ "${REMAINING_CONFLICTS:-0}" = "1" ] && [ -n "$NETCLASSES" ]; then
  $VPY -c 'import sys;sys.path.insert(0,sys.argv[1]);from postprocessing.remaining_conflicts import repair_run;repair_run(*sys.argv[1:])' \
    "$R" "$BJ" "$RJ" "routes_${N}_rest.json" "$NETCLASSES" "$RG" "$NL" "$VIA" "$GND_LAYERS" "${REST_CPU:-30}"
  RJ=routes_${N}_rest.json
fi
if [ -n "$NETCLASSES" ]; then
  $VPY -c 'import sys;sys.path.insert(0,sys.argv[1]);from postprocessing.remaining_conflicts import local_board_accept;local_board_accept(sys.argv[2],sys.argv[3])' $R $BJ $RJ
fi
export ZONE_GND_SIGNAL=${ZONE_GND_SIGNAL:-1}
export TEARDROP_FREE=${TEARDROP_FREE:-$R_TEARDROP_FREE}
ZONE_EDGE=$R_ZEDGE
if [ "${SHORT_CIRCUIT_PROTECTION:-1}" != "0" ]; then
  if ! $VPY $R/postprocessing/short_circuit_protection.py $BJ $RJ $TMPD/routes_safe.json || [ ! -s $TMPD/routes_safe.json ]; then
    echo "R44: input route not checked; no final board" >&2
    rm -rf $TMPD; exit 1
  fi
  RJ=$TMPD/routes_safe.json
fi
GNDN=/GND; [ -n "$NETCLASSES" ] && GNDN=$(python3 -c "import json,sys;K=json.load(open(sys.argv[1]));print(K.get('gnd_main') or (K.get('gnd') or ['/GND'])[0])" $NETCLASSES)
ZONE_EDGE=$ZONE_EDGE TONGUE=${TONGUE:-} GND_ALL=${GND_ALL:-0} FILL_NET=${FILL_NET:-} $VPY $R/postprocessing/make_zones.py $BJ $RJ zones_$N.json $GND_LAYERS
if [ "${CURRENT_VIAS:-1}" = "1" ] && [ -n "$NETCLASSES" ]; then
  $VPY $R/postprocessing/current_vias.py $BJ $RJ zones_$N.json $NETCLASSES routes_${N}_sv.json via=$VIA \
    clr=${SV_CLR:-$R_SVCLR} edge=$ZONE_EDGE vip=${VIP:-0} | head -8
  RJ=routes_${N}_sv.json
  ZONE_EDGE=$ZONE_EDGE TONGUE=${TONGUE:-} GND_ALL=${GND_ALL:-0} FILL_NET=${FILL_NET:-} $VPY $R/postprocessing/make_zones.py $BJ $RJ zones_$N.json $GND_LAYERS > /dev/null
fi
kpy $R/kicad/import_routes.py $BASE.kicad_pcb $RJ zones_$N.json $N.kicad_pcb cu_layers=$NL via=$VIA zclr=${ZCLR:-0.2} 2>&1 | filt
proj() { cp $BASE.kicad_pro ${1:r}.kicad_pro; if [ -f $BASE.kicad_dru ]; then cp $BASE.kicad_dru ${1:r}.kicad_dru; fi }
proj $N.kicad_pcb
if [ "${STITCH:-1}" = "1" ]; then
  $VPY $R/postprocessing/gnd_islands.py $N.kicad_pcb $N.kicad_pcb via=$VIA ${NETCLASSES:+netclasses=$NETCLASSES} drc=$R_DRC h2h=$R_H2H edge=$R_EDGE ${=GNDI_OPT} | tail -4
fi
drc_numbers() {
  rm -f "$2"
  if ! kc pcb drc --format json --severity-all -o $2 $1 >/dev/null 2>&1; then
    rm -f "$2"; echo '99999 99999 99999'; return
  fi
  python3 - $2 <<'EOF'
import json, sys
try:
    d = json.load(open(sys.argv[1]))
    v = d['violations']; open_items = d['unconnected_items']
    if not isinstance(v, list) or not isinstance(open_items, list) or any(not isinstance(x, dict) for x in open_items):
        raise ValueError('invalid DRC format')
    if any(not isinstance(x, dict) or x.get('severity') not in ('error', 'warning', 'info') or not isinstance(x.get('type'), str) for x in v):
        raise ValueError('invalid DRC entry')
    numbers = (sum(1 for x in v if x['severity'] == 'error'), len(open_items), sum(1 for x in v if x['type'] in ('track_dangling', 'via_dangling')))
except Exception:
    print(99999, 99999, 99999); sys.exit()
print(*numbers)
EOF
}
STATE=($(drc_numbers $N.kicad_pcb drc_$N.json))
take() {
  local K=$1
  if [ ! -s $K ]; then echo "   $2: no result, state kept"; return 1; fi
  proj $K
  local NEW=($(drc_numbers $K ${K:r}_drc.json))
  if [ ${STATE[1]} -eq 99999 ] || [ ${NEW[1]} -eq 99999 ]; then
    echo "   $2: DRC check failed, state kept"; return 1
  fi
  if [ ${NEW[1]} -le ${STATE[1]} ] && [ ${NEW[2]} -le ${STATE[2]} ] && { [ "$3" != "loose=1" ] || [ ${NEW[3]} -lt ${STATE[3]} ]; }; then
    mv $K $N.kicad_pcb; mv ${K:r}_drc.json drc_$N.json; rm -f ${K:r}.kicad_pro ${K:r}.kicad_dru
    echo "   $2: accepted (DRC errors/open/loose ${STATE[1]}/${STATE[2]}/${STATE[3]} -> ${NEW[1]}/${NEW[2]}/${NEW[3]})"; STATE=($NEW); return 0
  fi
  echo "   $2: rejected (DRC errors/open/loose ${STATE[1]}/${STATE[2]}/${STATE[3]} -> ${NEW[1]}/${NEW[2]}/${NEW[3]})"
  rm -f $K ${K:r}_drc.json ${K:r}.kicad_pro ${K:r}.kicad_dru; return 1
}
if [ "${SHORTEN:-1}" = "1" ]; then
  eval "$($VPY - $BJ $RJ ${NETCLASSES:--} <<'EOF'
import json, sys, math, shlex
B = json.load(open(sys.argv[1])); R = json.load(open(sys.argv[2])); K = json.load(open(sys.argv[3])) if sys.argv[3] != '-' else {}
xs = [p[0] for o in B['outline'] for p in o]; ys = [p[1] for o in B['outline'] for p in o]
gnd = set(K.get('gnd') or ['/GND']); NZ = K.get('net_list') or {}
net_list = sorted({t['net'] for t in R['tracks'] if t['net'] and t['net'] not in gnd and not (NZ.get(t['net']) or {}).get('pair')})
print('ABB=%s ANETS=%s ALAYERS=%s' % (shlex.quote('%d %d %d %d' % (math.floor(min(xs)), math.floor(min(ys)), math.ceil(max(xs)), math.ceil(max(ys)))),
      shlex.quote(';'.join(net_list)), shlex.quote(','.join(sorted({t['layer'] for t in R['tracks']})))))
EOF
)"
  if [ -n "$ANETS" ]; then
    SHORTEN_OPT="${RG:+design_rules=$RG} drc=$R_DRC drcpad=$R_DRCPAD drcvia=$R_DRC border=$R_EDGE hole_size=$R_HOLE hmax=${SHORTCUT_HMAX:-0.15} cu_layers=$ALAYERS"
    DEAD=0 SHORTCUT_OPT="$SHORTEN_OPT" $R/postprocessing/shorten_run.sh $N.kicad_pcb ${N}_ak.kicad_pcb $BASE.kicad_pro $BASE.kicad_dru "$ANETS" ${=ABB} 2>&1 | filt | grep -E "^Round" | tail -1 || true
    take ${N}_ak.kicad_pcb "Shortening (R24)" || true
    rm -f _ak_h.json _ak.json
  fi
fi
if [ "${EMPTY:-1}" = "1" ] && [ "$ZONE_GND_SIGNAL" = "1" ]; then
  kpy $R/postprocessing/empty_fill.py export $N.kicad_pcb $TMPD/cu_empty.json 2>&1 | filt > /dev/null
  if $VPY $R/postprocessing/empty_fill.py compute $TMPD/cu_empty.json $TMPD/plan_empty.json border=$ZONE_EDGE zclr=${ZCLR:-0.2} via=$VIA \
       drc=$R_DRC h2h=$R_H2H gnd=$GNDN ${TONGUE:+tongue=$TONGUE} 2>&1 | tail -2; then
    kpy $R/postprocessing/empty_fill.py insert $N.kicad_pcb $TMPD/plan_empty.json ${N}_empty.kicad_pcb 2>&1 | filt | tail -1 || true
    take ${N}_empty.kicad_pcb "Empty patches (K14)" || true
  fi
fi
if [ "$ZONES" = "1" ]; then
  $R/zones.sh $N.kicad_pcb $N.kicad_pcb ${RG:+design_rules=$RG} ${NETCLASSES:+netclasses=$NETCLASSES} clr=${ZCLR:-0.2} 2>&1 | grep -E 'Zone autorouter|DRC|spots' || true
  proj $N.kicad_pcb; STATE=($(drc_numbers $N.kicad_pcb drc_$N.json))
fi
if [ "${LOOSE:-1}" = "1" ]; then
  for r in 1 2 3; do
    [ ${STATE[3]} -gt 0 ] || break
    kpy $R/kicad/loose_shorten.py $N.kicad_pcb drc_$N.json ${N}_loose.kicad_pcb 2>&1 | filt | tail -1
    [ -s ${N}_loose.kicad_pcb ] && kpy $R/kicad/loose_vias.py ${N}_loose.kicad_pcb drc_$N.json ${N}_loose.kicad_pcb 2>&1 | filt | tail -1
    take ${N}_loose.kicad_pcb "Loose remnants, round $r (R24 d)" loose=1 || break
  done
fi
if [ "${CURRENT_PATH_AREAS:-0}" = "1" ]; then
  if [ -n "$NETCLASSES" ]; then
    if $R/postprocessing/current_path_run.sh $N.kicad_pcb ${N}_sf.kicad_pcb $NETCLASSES ${SF_GAP:-${ZCLR:-0.2}} ${SF_EDGE:-$ZONE_EDGE}; then
      if take ${N}_sf.kicad_pcb "Current-path areas (R33)"; then
        RJ=${N}_sf_new.routes.json
        cp ${N}_sf_zones.json zones_$N.json
      fi
    else
      echo "   Current-path areas: gate failed; final result kept. Candidate/diagnosis: ${N}_sf*"
    fi
  else
    echo "   Current-path areas: NETCLASSES missing; final result kept"
  fi
fi
if [ "${SHORT_CIRCUIT_PROTECTION:-1}" != "0" ]; then
  if ! $VPY $R/postprocessing/short_circuit_kicad_native.py $N.kicad_pcb $N.kicad_pcb r44_$N.json ${NATIVE_CPU_MAX:-300} \
     || ! $VPY -c "import json,sys; sys.exit(0 if json.load(open(sys.argv[1])).get('checked') is True else 1)" r44_$N.json; then
    rm -f $N.kicad_pcb
    echo "R44: final result not checked (check error, see r44_$N.json); no final board" >&2
    rm -rf $TMPD; exit 1
  fi
  STATE=($(drc_numbers $N.kicad_pcb drc_$N.json))
fi
python3 - drc_$N.json <<'EOF'
import json, sys, collections
d = json.load(open(sys.argv[1]))
fe = [v for v in d['violations'] if v['severity'] == 'error']
c = collections.Counter(v['type'] for v in fe); loose = collections.Counter(v['type'] for v in d['violations'] if v['type'] in ('track_dangling', 'via_dangling'))
print('DRC', dict(c), 'not_connected', len(d['unconnected_items']), '| loose tracks %d, loose vias %d | warnings %d' % (
    loose['track_dangling'], loose['via_dangling'], sum(1 for v in d['violations'] if v['severity'] != 'error')))
for v in fe[:12]:
    print('  ', v['type'], v['description'][:70], [(i['description'][:45], i['pos']['x'], i['pos']['y']) for i in v['items']])
for v in d['unconnected_items'][:12]:
    print('   open', [(i['description'][:45], i['pos']['x'], i['pos']['y']) for i in v['items']])
EOF
[ "${IMAGES:-1}" = "1" ] || { rm -rf $TMPD; exit 0; }
TMPD=$TMPD python3 - zones_$N.json <<'EOF'
import json, sys, os
z = json.load(open(sys.argv[1]))
g = set(json.load(open(os.environ['NETCLASSES']))['gnd']) if os.environ.get('NETCLASSES') else {'/GND'}
json.dump([q for q in z if q['net'] in g and not q.get('gnd_signal')], open(os.environ['TMPD'] + '/zones_only_gnd.json', 'w'))
EOF
kpy $R/kicad/import_routes.py $BASE.kicad_pcb $RJ $TMPD/zones_only_gnd.json $TMPD/${N}_tracks.kicad_pcb cu_layers=$NL via=$VIA zclr=${ZCLR:-0.2} 2>&1 | filt
cp $BASE.kicad_pro $TMPD/${N}_tracks.kicad_pro; if [ -f $BASE.kicad_dru ]; then cp $BASE.kicad_dru $TMPD/${N}_tracks.kicad_dru; fi
for P in $N:z $TMPD/${N}_tracks:b; do f=${P%%:*}; t=${P##*:}
  LL=(F.Cu); for k in $(seq 1 $((NL-2))); do LL+=(In$k.Cu); done; LL+=(B.Cu)
  for L in $LL; do
    kc pcb export svg --layers "$L,Edge.Cuts" --mode-single --fit-page-to-board --exclude-drawing-sheet -o $TMPD/zw${t}_$L.svg $f.kicad_pcb >/dev/null 2>&1
    perl -e 'alarm 90; exec @ARGV' $INK $TMPD/zw${t}_$L.svg --export-type=png --export-filename=$TMPD/zw${t}_$L.png --export-width=600 --export-background=white >/dev/null 2>&1 || echo "Inkscape: $L aborted"
  done
done
TMPD=$TMPD $VPY - $N $NL $GND_LAYERS <<'EOF'
import sys, os
from PIL import Image, ImageDraw, ImageFont
N = sys.argv[1]; NL = int(sys.argv[2])
L = ['F.Cu'] + ['In%d.Cu' % k for k in range(1, NL - 1)] + ['B.Cu']
G = sys.argv[3].split(',')
T = ['F (top)'] + ['In%d%s' % (k, ' (GND only)' if 'In%d' % k in G else '') for k in range(1, NL - 1)] + ['B (bottom)']
f = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', 30)
for t, name in (('z', 'Layers_%s.png' % N), ('b', 'Layers_only_Tracks_%s.png' % N)):
    ims = [Image.open(os.environ['TMPD'] + '/zw%s_%s.png' % (t, l)).convert('RGB') for l in L]
    nc = 4; nr = (len(ims) + nc - 1) // nc
    cw = max(i.width for i in ims) + 30; ch = max(i.height for i in ims) + 70
    c = Image.new('RGB', (nc * cw + 30, nr * ch), 'white'); d = ImageDraw.Draw(c)
    for k, (i, tt) in enumerate(zip(ims, T)):
        x = 30 + (k % nc) * cw; y = (k // nc) * ch
        c.paste(i, (x, y + 60)); d.text((x, y + 15), tt, fill=(30, 30, 30), font=f)
    c.save(name); print(name, c.size)
EOF
rm -rf $TMPD
