#!/bin/zsh
set -e
set -o pipefail
export PYTHONHASHSEED=0
D=${1:A}; SRC=${2:A}; NL=${3:-6}; MODE=${4:-new}; GND_LAYERS=${5:-In1}
R=${0:A:h}; VPY=$R/.venv/bin/python
KPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
KC=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
BASE=base_${NL}cu_layers
cd $D
filt() { grep -v -E 'swig|assert|wxApp|Debug: Adding duplicate' || true }
SB=${SRC:r}
if [ "$SRC" != "$D/$BASE.kicad_pcb" ]; then
  cp $SRC $BASE.kicad_pcb; cp $SB.kicad_pro $BASE.kicad_pro; [ -f $SB.kicad_dru ] && cp $SB.kicad_dru $BASE.kicad_dru
fi
PS=$D/.pipeline_project.kicad_pro; cp $BASE.kicad_pro $PS
pro_back() { cp $PS $BASE.kicad_pro }
CONS=""; [ -f specs.json ] && CONS="specs=$D/specs.json"
rm -f design_rules.json
if [ -n "${DESIGN_RULES_SOURCE:-}" ]; then
  echo "== A1 Rules (source $DESIGN_RULES_SOURCE)"
  $VPY $R/analysis/design_rules.py $DESIGN_RULES_SOURCE cu_layers=$NL out=design_rules.json | sed -n 1,2p
else
  echo "== A1 Rules (F1/F4: from the board, the newest board only as template)"
  $VPY $R/analysis/design_rules.py $BASE.kicad_pcb cu_layers=$NL out=design_rules.json fallback=latest template=1 | sed -n 1,2p
  cp $BASE.kicad_pro $PS
fi
[ -s design_rules.json ] || { echo "design_rules.json is missing" >&2; exit 2; }
VIA_RG=$(python3 -c "import json;print(json.load(open('design_rules.json'))['router']['via'])")
echo "== Export";          $KPY $R/kicad/export_board.py $BASE.kicad_pcb board.json 2>&1 | filt
echo "== A2 Netclasses";      $VPY $R/analysis/netclasses.py board.json netclasses.json design_rules=design_rules.json $CONS | head -8
if [ "${LAYER_STACK:-0}" = "1" ]; then
  $VPY $R/analysis/design_rules.py structure board.json netclasses.json "gnd_layers=$GND_LAYERS" out=layer_stack.json
fi
PL_T=(); PL_VIA_COST=(); RT_STYLE=(); NC_ROUTE=netclasses.json
if [ "${SETTINGS:-}" = "learned" ]; then
  echo "== Learned settings (H11, learning/settings.py; explicitly set variables take precedence)"
  eval "$($VPY $R/learning/settings.py board.json netclasses.json design_rules.json cu_layers=$NL spec=pipeline format=sh ${STEERING:+steering=$STEERING} 2>settings.txt)"
  sed 's/^# /   /' settings.txt
  if [ "$MODE" = "new" ]; then
    if [ -z "${CROSS:-}" ] && [ "$E_PROVENANCE_CROSS" = "learned" ]; then CROSS=$E_CROSS; fi
    if [ -z "${SEPARATE:-}" ] && [ "$E_PROVENANCE_SEPARATE" = "learned" ]; then SEPARATE=$E_SEPARATE; fi
    if [ -z "${VIA_COST:-}" ] && [ "$E_PROVENANCE_VIA_COST" = "learned" ]; then VIA_COST=$E_VIA_COST; fi
    if [ -n "${SEPARATE:-}" ]; then PL_T=(separate=$SEPARATE); fi
  fi
  if [ -z "${VIA_F:-}" ] && [ "$E_PROVENANCE_VIA_F" = "learned" ]; then VIA_F=$E_VIA_F; fi
  if [[ "${ROUTER_OPT:-}" == *style=* ]] || [[ "${ROUTER_OPT:-}" == *angle45=* ]]; then STYLE=""
  elif [ "${STYLE:-}" = "learned" ]; then
    if [ "$E_PROVENANCE_STYLE" = "learned" ]; then STYLE=$E_STYLE; else STYLE=""; fi
  elif [ -z "${STYLE:-}" ] && [ "$E_PROVENANCE_STYLE" = "learned" ]; then
    echo "   Style: the learning loop suggests $E_STYLE; keeping the default (R6, accept with STYLE=learned)"
  fi
  echo "   Netclass $E_NET_CLASS, used: cross=${CROSS:-2} separate=${SEPARATE:-3} via_cost=${VIA_COST:-0.6} via_f=${VIA_F:-1} style=${STYLE:-standard}"
fi
if [ -n "${VIA_COST:-}" ]; then PL_VIA_COST=(via_cost=$VIA_COST); fi
if [[ "${ROUTER_OPT:-}" == *style=organic* ]] || [[ "${ROUTER_OPT:-}" == *angle45=0* ]]; then STILE=(organic); STYLE_FIXED=1
elif [[ "${ROUTER_OPT:-}" == *style=* ]] || [[ "${ROUTER_OPT:-}" == *angle45=* ]]; then STILE=(45); STYLE_FIXED=1
else
  STYLE_FIXED=0
  if [ -z "${STYLE:-}" ] || [ "$STYLE" = "learned" ]; then
    STYLE=$(python3 -c "import json,sys;v=(json.load(open(sys.argv[1])) if sys.argv[1] else {}).get('style') or '45';print(','.join(v) if isinstance(v,list) else v)" "${CONS#specs=}")
  fi
  STILE=(${(s:,:)${STYLE//standard/45}})
fi
echo "== Style: ${(j:, :)STILE} (R6; 45 degrees with corners rounded up to ${ROUNDING:-0.5} mm, organically tightened)"
silk() {
  [ "${SILK:-1}" = "1" ] || return 0
  ( cd $1 && $KC pcb drc --format json -o drc_placed.json $BASE.kicad_pcb >/dev/null 2>&1 || true
    $KPY $R/kicad/silk_tidy_up.py $BASE.kicad_pcb drc_placed.json silk.kicad_pcb 2>&1 | filt | tail -1
    [ -f silk.kicad_pcb ] && mv silk.kicad_pcb $BASE.kicad_pcb; rm -f silk.kicad_pro; cp $PS $BASE.kicad_pro )
}
PLS=(.)
ROUTED=0
if [ "${PORTFOLIO:-0}" = "1" ]; then
  if [ "$MODE" != "refine" ]; then
    echo "PORTFOLIO=1 requires mode=refine (cross=2 plus IR refinement)." >&2
    exit 2
  fi
  PD=${PORTFOLIO_DIR:-$D/portfolio}
  PARGS=(board.json design_rules.json "$PD" --source "$R" --pcb "$D/$BASE.kicad_pcb"
    --gnd_layers "$GND_LAYERS" --iter "${ITER:-60}" --sides "${SIDES:-FB}"
    --seed "${PLACER_SEED:-1}" --hashseed "$PYTHONHASHSEED")
  [ -f specs.json ] && PARGS+=(--specs "$D/specs.json")
  for po in ${=ROUTER_OPT:-short=1 fair=1}; do PARGS+=(--router-opt "$po"); done
  caffeinate -i -s "$VPY" "$R/codex/portfolio/run.py" "${PARGS[@]}"
  cp "$PD/selected/base.kicad_pcb" "$BASE.kicad_pcb"
  for f in board.json netclasses.json netclasses.txt routes.json placement.json; do cp "$PD/selected/$f" "$f"; done
  silk .
  ROUTED=1
elif [ "$MODE" != "none" ]; then
  echo "== A3-A5 Placer ($MODE)"
  if [ "$MODE" = "new" ] && [ "${CANDIDATES:-1}" -gt 1 ]; then
    NC=()
    for k in $(seq 1 $CANDIDATES); do
      $VPY $R/common/slots.py cpu=${PLACER_CPU:-7200} -- caffeinate -i -s $VPY $R/placement/placer.py board.json netclasses.json placement_k$k.json design_rules=design_rules.json mode=$MODE sides=${SIDES:-FB} cross=${CROSS:-2} separate=${SEPARATE:-3} $PL_VIA_COST seed=$k | grep -E "Metrics after"
      NC+=(placement_k$k.json)
    done
    $VPY $R/analysis/variants.py ranking board.json netclasses.json design_rules.json $NC selection=${SELECTION:-heuristic} | tee selection.txt
    RANK=($(awk '/^RANK/ {print $3}' selection.txt)); NR=${ROUTE:-2}; [ $NR -gt ${#RANK} ] && NR=${#RANK}
    PLS=()
    for i in $(seq 1 $NR); do
      P=pl$i; rm -rf $P; mkdir -p $P; cp $BASE.kicad_pcb $BASE.kicad_dru $P/ 2>/dev/null || cp $BASE.kicad_pcb $P/; cp $PS $P/$BASE.kicad_pro
      cp ${RANK[$i]} $P/placement.json; cp design_rules.json $P/
      ( cd $P && $KPY $R/placement/apply_place.py $BASE.kicad_pcb placement.json placed.kicad_pcb 2>&1 | filt && mv placed.kicad_pcb $BASE.kicad_pcb && rm -f placed.kicad_pro && cp $PS $BASE.kicad_pro )
      silk $P
      ( cd $P && $KPY $R/kicad/export_board.py $BASE.kicad_pcb board.json placement=placement.json 2>&1 | filt >/dev/null && $VPY $R/analysis/netclasses.py board.json netclasses.json design_rules=design_rules.json $CONS > /dev/null )
      PLS+=($P); echo "   Candidate $i: ${RANK[$i]} -> $P"
    done
  else
    $VPY $R/common/slots.py cpu=${PLACER_CPU:-7200} -- caffeinate -i -s $VPY $R/placement/placer.py board.json netclasses.json placement.json design_rules=design_rules.json mode=$MODE sides=${SIDES:-FB} cross=${CROSS:-2} $PL_T $PL_VIA_COST image=pl | grep -E "Metrics|Legal|finished"
    $KPY $R/placement/apply_place.py $BASE.kicad_pcb placement.json placed.kicad_pcb 2>&1 | filt
    mv placed.kicad_pcb $BASE.kicad_pcb; rm -f placed.kicad_pro; pro_back
    if [ "${SOFT:-0}" = "1" ]; then
      echo "== A5b Soft detail placer (Codex)"
      $KPY $R/kicad/export_board.py $BASE.kicad_pcb board.json placement=placement.json 2>&1 | filt
      $VPY $R/analysis/netclasses.py board.json netclasses.json design_rules=design_rules.json $CONS > /dev/null
      $VPY $R/common/slots.py cpu=${PLACER_CPU:-7200} -- caffeinate -i -s $VPY $R/codex/analytical/placer.py board.json netclasses.json placement_soft.json --mode soft --design_rules design_rules.json | tail -1
      $KPY $R/placement/apply_place.py $BASE.kicad_pcb placement_soft.json placed.kicad_pcb 2>&1 | filt
      mv placed.kicad_pcb $BASE.kicad_pcb; rm -f placed.kicad_pro; pro_back
    fi
    silk .
    $KPY $R/kicad/export_board.py $BASE.kicad_pcb board.json placement=placement.json 2>&1 | filt
    $VPY $R/analysis/netclasses.py board.json netclasses.json design_rules=design_rules.json $CONS > /dev/null
  fi
else
  silk .
  $KPY $R/kicad/export_board.py $BASE.kicad_pcb board.json 2>&1 | filt >/dev/null
fi
NT_ON=$(python3 -c "import json;B=json.load(open('board.json'));K=json.load(open('netclasses.json'));print(1 if B.get('via_keepout') and K.get('net_ties') else 0)")
[ "${NT:-1}" = "1" ] || NT_ON=0
SLOT=(); [ "${CPU_MAX:-7200}" -gt 0 ] && SLOT=(cpu=$(( ${CPU_MAX:-7200} * 5 / 4 + 120 )))
PFS=(${=PFMUL:-1.25 1.24 1.26}); PFS=(${PFS[1,${RUNS:-2}]})
rest_pose() {
  ( cd "$1"
    if $VPY -c 'import json,sys;sys.exit(not bool(json.load(open("routes.json")).get("rest_board")))'; then
      $KPY $R/kicad/rest_placement.py $BASE.kicad_pcb routes.json rest_slot.kicad_pcb 2>&1 | filt
      mv rest_slot.kicad_pcb $BASE.kicad_pcb; rm -f rest_slot.kicad_pro rest_slot.kicad_dru
      cp $PS $BASE.kicad_pro
      $VPY -c 'import sys;sys.path.insert(0,sys.argv[1]);from postprocessing.remaining_conflicts import local_board_accept;local_board_accept("board.json","routes.json")' $R
    fi )
}
shaping() {
  rest_pose "$1"
  local st=$(cat $1/style.txt 2>/dev/null || echo 45)
  if [ "$st" = "organic" ]; then
    $VPY $R/postprocessing/pulltight_routes.py $1/board.json $1/routes.json $1/routes_shape.json design_rules=$1/design_rules.json netclasses=$1/netclasses.json rmax=${RMAX_ORGANIC:-3} > $1/shaping.log 2>&1 \
      && echo "   $1: organically tightened ($(python3 -c "import json,sys;s=json.load(open(sys.argv[1]))['organic_post_run']['statistics'];print('Arcs %s, Vertices way %s' % (s.get('arcs',0), s.get('vertices_removed',0)))" $1/routes_shape.json))" \
      || { echo "   $1: tightening failed ($(tail -1 $1/shaping.log)), router tracks remain"; cp $1/routes.json $1/routes_shape.json; }
  elif [ "${ROUNDING:-0.5}" != "0" ]; then
    $VPY $R/postprocessing/pulltight_routes.py $1/board.json $1/routes.json $1/routes_shape.json design_rules=$1/design_rules.json netclasses=$1/netclasses.json pull=0 rmax=${ROUNDING:-0.5} > $1/shaping.log 2>&1 \
      && echo "   $1: 45 degrees, corners rounded ($(python3 -c "import json,sys;s=json.load(open(sys.argv[1]))['organic_post_run']['statistics'];print('Arcs %s' % s.get('arcs',0))" $1/routes_shape.json))" \
      || { echo "   $1: rounding failed ($(tail -1 $1/shaping.log)), router tracks remain"; cp $1/routes.json $1/routes_shape.json; }
  else
    cp $1/routes.json $1/routes_shape.json
  fi
}
nt_after() {
  rest_pose "$1"
  [ "$NT_ON" = "1" ] || return 0
  ( cd $1
    [ -s star.json ] && $VPY $R/postprocessing/star_vias.py vias routes.json star.json routes.json via=$VIA_RG | tail -2
    eval "$(python3 -c "import json;r=json.load(open('design_rules.json'))['router'];d,b=r['via'].split('/');print('VD=%s VB=%s CL=%g CP=%g H2=%s LO=%s' % (d,b,float(r['drc'])+0.005,float(r.get('drcpad',r['drc']))+0.005,r['h2h'],r.get('hole_clr',0.2)))")"
    GN=$(python3 -c "import json;K=json.load(open('netclasses.json'));print(K.get('gnd_main') or K['gnd'][0])")
    $VPY $R/postprocessing/gnd_via_nt.py board.json routes.json routes.json via=$VD drill=$VB clr=$CL h2h=$H2 gnd=$GN 2>&1 | tail -1
    $VPY $R/postprocessing/via_nt_spacing.py board.json routes.json routes.json hole_cu=$LO drill=$VB via=$VD clr=$CL clrpad=$CP h2h=$H2 2>&1 | tail -1 ) || true
}
if [ "$ROUTED" = "0" ]; then
  echo "== A6 Routing: ${#PLS} placement(s) x ${#PFS} run(s) (pfmul ${(j:, :)PFS}) x ${#STILE} style(s)"
  rm -rf var; mkdir var; VARS=()
  for P in $PLS; do
    for pf in $PFS; do
      for st in $STILE; do
        V=var/${P//[.\/]/}${P:+_}p${pf}_s$st; V=${V//var\/_/var/}; mkdir -p $V
        cp $P/$BASE.kicad_pcb $P/board.json $P/netclasses.json design_rules.json $V/; cp $PS $V/$BASE.kicad_pro; [ -f $BASE.kicad_dru ] && cp $BASE.kicad_dru $V/
        [ -f $P/placement.json ] && cp $P/placement.json $V/
        [ -f specs.json ] && cp specs.json $V/
        echo $st > $V/style.txt; echo $pf > $V/pfmul.txt; VARS+=($V)
      done
    done
  done
  for V in $VARS; do
    ( cd $V
      KLR=netclasses.json
      if [ -n "${VIA_F:-}" ] && [ "$VIA_F" != "1" ]; then
        $VPY -c "import json,sys
K=json.load(open('netclasses.json')); f=float(sys.argv[1])
for k in K['net_list'].values():
    if 'via' in k: k['via']=round(k['via']*f,3)
json.dump(K,open('netclasses_route.json','w'))" $VIA_F
        KLR=netclasses_route.json
      fi
      BRT=board.json; GOV=""
      if [ "$NT_ON" = "1" ]; then
        $VPY $R/postprocessing/star_vias.py candidates board.json star_cand.json via=$VIA_RG > star.log 2>&1 || true
        $KPY $R/postprocessing/star_vias.py verify $BASE.kicad_pcb star_cand.json star.json $BASE.kicad_pro >> star.log 2>&1 || true
        if [ -s star.json ]; then
          GOV=$($VPY $R/postprocessing/star_vias.py pads board.json star.json board_route.json via=$VIA_RG cu_layers=$NL | tail -1) && BRT=board_route.json
        fi
      fi
      ST=(style=$(cat style.txt)); [ "$STYLE_FIXED" = "1" ] && ST=()
      $VPY $R/common/slots.py $SLOT -- caffeinate -i -s $VPY $R/router/route_pf.py $BRT routes.json design_rules=design_rules.json \
        netclasses=$KLR gnd_layers=$GND_LAYERS iter=${ITER:-60} pf0=1 pfmul=$(cat pfmul.txt) hinc=1.0 goal_from=20 ${=ROUTER_OPT:-short=1 fair=1} $ST \
        cpu_max=${CPU_MAX:-7200} ${GOV:+gnd_without_via=$GOV} short_circuit_protection=${SHORT_CIRCUIT_PROTECTION:-1} png=preview > route.log 2>&1 || echo "   $V: router ended with error (route.log)"
      echo "   $V: $(grep -E 'End|Tracks' route.log | filt | tail -1)" ) &
  done
  wait
  for V in $VARS; do [ -s $V/routes.json ] || { echo "   $V: no routing" ; VARS=(${VARS:#$V}); }; done
  [ ${#VARS} -gt 0 ] || { echo "No router run wrote a result" >&2; exit 3; }
  for V in $VARS; do nt_after $V; shaping $V; done
  if [ ${#VARS} -gt 1 ]; then
    echo "== Selection among ${#VARS} variants (finish without images, gate: valid with KiCad DRC, then score v3)"
    for V in $VARS; do
      ( cd $V && NETCLASSES=netclasses.json DESIGN_RULES=design_rules.json IMAGES=0 SHORTEN=0 EMPTY=0 LOOSE=0 BASE=$BASE ZCLR=${ZCLR:-0.15} ZONE_EDGE=${ZONE_EDGE:-0.25} \
        $VPY $R/common/slots.py cpu=${FINISHED_CPU:-14400} -- caffeinate -i -s $R/finish.sh . routes_shape.json finished $NL $VIA_RG $GND_LAYERS > finished.log 2>&1 || echo "   $V: finish ended with error (finished.log)" ) &
    done
    wait
    ZONE_EDGE=${ZONE_EDGE:-0.25} ZCLR=${ZCLR:-0.15} ZONE_GND_SIGNAL=${ZONE_GND_SIGNAL:-1} $VPY $R/analysis/variants.py choose variants.json $VARS | tee selection_variants.txt | grep -v CHOSEN
    W=$(awk '/^CHOSEN/ {print $2}' selection_variants.txt)
  else
    W=${VARS[1]}
  fi
  echo "   chosen: $W"
  for f in $BASE.kicad_pcb board.json netclasses.json routes.json routes_shape.json style.txt; do cp $W/$f $f; done
  for f in placement.json star.json board_route.json netclasses_route.json; do [ -f $W/$f ] && cp $W/$f $f; done
  pro_back
  [ -d $W/preview ] && rm -rf preview && cp -R $W/preview preview || true
  ROUTED=1
else
  echo ${STILE[1]} > style.txt; nt_after .; shaping .
fi
rest_pose .
echo "== A7/A8 Improve automatically (IMPROVE=0 to disable)"
if [ "${IMPROVE:-1}" = "1" ]; then
  ZONE_EDGE=${ZONE_EDGE:-0.25} ZCLR=${ZCLR:-0.15} ZONE_GND_SIGNAL=${ZONE_GND_SIGNAL:-1} \
    $VPY $R/postprocessing/improve.py . rounds_n=${ROUNDS:-3} parallel=${PARALLEL:-8} 2>&1 | grep -E "Base|accepted|additional|Result"
  if [ -s placement_after.json ]; then
    $KPY $R/placement/apply_place.py $BASE.kicad_pcb placement_after.json improved.kicad_pcb 2>&1 | filt
    mv improved.kicad_pcb $BASE.kicad_pcb; rm -f improved.kicad_pro; pro_back
    cp board_after.json board.json; cp routes_after.json routes.json
    shaping .
  fi
fi
echo "== A9/A10 Finish (GND on all layers, stitching, shortening, empty patches, loose remnants, DRC)"
NETCLASSES=netclasses.json DESIGN_RULES=design_rules.json BASE=$BASE ZCLR=${ZCLR:-0.15} ZONE_EDGE=${ZONE_EDGE:-0.25} $VPY $R/common/slots.py cpu=${FINISHED_CPU:-14400} -- caffeinate -i -s $R/finish.sh . routes_shape.json finished $NL \
  $VIA_RG $GND_LAYERS 2>&1 | grep -E "DRC|open|GND islands|Layers_|accepted|rejected|Round" | awk 'NR<=20'
$VPY $R/common/slots.py cpu=${EVALUATION_CPU:-3600} -- $VPY $R/analysis/final_state.py finished.kicad_pcb netclasses.json routes_shape.json
echo "== Evaluation and check images for the AI"
$VPY $R/common/slots.py cpu=${EVALUATION_CPU:-3600} -- $VPY $R/analysis/evaluation.py board_finished.json routes_finished.json netclasses.json ir=1 image=chk json=evaluation.json | awk 'NR<=6'
echo "== Rules report (report.txt; package 4f)"
{
  echo "# Rule report $(date '+%Y-%m-%d %H:%M'), style $(cat style.txt), rules $(python3 -c "import json;r=json.load(open('design_rules.json'));print(r.get('source_kind','?'), r['source'])")"
  [ -f variants.json ] && { echo; echo "## Variants (analysis/variants.py)"; cat selection_variants.txt; }
  echo; echo "## KiCad DRC finished.kicad_pcb"
  python3 - drc_finished.json <<'EOF' || true
import json, sys, collections
d = json.load(open(sys.argv[1])); v = d['violations']
print('Error', dict(collections.Counter(x['type'] for x in v if x['severity'] == 'error')), '| open', len(d['unconnected_items']),
      '| loose', sum(1 for x in v if x['type'] in ('track_dangling', 'via_dangling')), '| Warnings', dict(collections.Counter(x['type'] for x in v if x['severity'] != 'error')))
EOF
  echo; echo "## Evaluation of the final board (routes_finished.json, score v3 with final copper)"
  cat final_state.json; echo
  $VPY -c "import json; e=json.load(open('evaluation.json')); print('VALID' if e['is_valid'] else 'INVALID'); print('rating-v3', e['score_v3']); print(json.dumps(e['design_rules'], indent=1))"
  if [ "${LAYER_STACK:-0}" = "1" ]; then
    echo; echo "## Layer stackup K19 (actual final fill)"
    $VPY $R/analysis/design_rules.py structure board_finished.json netclasses.json routes=routes_finished.json out=layer_stack_finished.json
  fi
  echo; echo "## Rule check (analysis/rule_check.py)"
  $VPY $R/analysis/rule_check.py board_finished.json netclasses.json routes_finished.json json=design_rules_chk.json 2>&1 || true
  echo; echo "## Metrics R19 (final board, arcs as dense chords)"
  $VPY $R/analysis/metrics.py routes_finished.json board=board_finished.json 2>&1 || true
  python3 -c "import json,sys;s=(json.load(open(sys.argv[1])).get('organic_post_run') or {}).get('statistics') or {};print('Style-PostRun: Arcs %s, Vertices way %s, Tracks changed %s, guarded (PowerNets) %s' % (s.get('arcs',0), s.get('vertices_removed',0), s.get('tracks_changed',0), s.get('guarded',0)))" routes_shape.json 2>&1 || true
  echo; echo "## Coupling (analysis/coupling.py)"
  $VPY $R/analysis/coupling.py routes_finished.json netclasses.json 2>&1 || true
  echo; echo "## Topology (final board; dense arc chords can count nodes)"
  $VPY $R/analysis/topology.py routes_finished.json board=board_finished.json 2>&1 || true
} > report.txt
grep -E "^Error|^VALID|^INVALID|^rating-v3|^Coupling" report.txt | head -6 || true
