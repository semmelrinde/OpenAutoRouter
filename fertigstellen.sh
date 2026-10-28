#!/bin/zsh
set -e
TMPD=$(mktemp -d)            # eigener Temp-Ordner je Lauf (parallele Laeufe)
D=$1; RJ=$2; N=$3; NL=${4:-4}; VIA=${5:-0.45/0.20}; GNDL=${6:-In1}; BASE=${BASE:-base_${NL}lagen}
R=${0:A:h}                      # Projektordner (Autorouter)
VPY=$R/.venv/bin/python
KPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
KC=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
INK=/Applications/Inkscape.app/Contents/MacOS/inkscape
kpy() { ( ulimit -S -t ${NATIVE_CPU_MAX:-300}; $KPY "$@" ); }
kc() { ( ulimit -S -t ${NATIVE_CPU_MAX:-300}; $KC "$@" ); }
cd $D
filt() { grep -v -E 'swig|assert|wxApp|Debug: Adding duplicate' || true }
BJ=${BOARD:-${BJ:-board.json}}
RG=${REGELN:-regeln.json}; [ -f "$RG" ] || RG=""
eval "$(python3 - "${RG:--}" "${ZONE_EDGE:-0.3}" <<'EOF'
import json, sys
r = {}
if sys.argv[1] != '-':
    try: r = json.load(open(sys.argv[1]))['router']
    except Exception: r = {}
f = lambda k, d: float(r.get(k) or d)
drc = f('drc', 0.09)
print('R_DRC=%g R_DRCPAD=%g R_EDGE=%g R_H2H=%g R_LOCH=%g' % (drc, f('drcpad', max(drc, 0.10)), f('edge', 0.2), f('h2h', 0.2), f('loch_clr', 0.2)))
print('R_ZEDGE=%g R_TFREI=%g R_SVCLR=%g' % (max(float(sys.argv[2]), f('edge', 0.2) + 0.02), max(0.11, drc + 0.02), max(0.12, drc + 0.03)))
EOF
)"
if [ "${RESTKONFLIKTE:-0}" = "1" ] && [ -n "$KLASSEN" ]; then
  $VPY -c 'import sys;sys.path.insert(0,sys.argv[1]);from nachbearbeitung.restkonflikte import reparatur_lauf;reparatur_lauf(*sys.argv[1:])' \
    "$R" "$BJ" "$RJ" "routes_${N}_rest.json" "$KLASSEN" "$RG" "$NL" "$VIA" "$GNDL" "${REST_CPU:-30}"
  RJ=routes_${N}_rest.json
fi
if [ -n "$KLASSEN" ]; then
  $VPY -c 'import sys;sys.path.insert(0,sys.argv[1]);from nachbearbeitung.restkonflikte import lokales_board_uebernehmen;lokales_board_uebernehmen(sys.argv[2],sys.argv[3])' $R $BJ $RJ
fi
export ZONE_GND_SIGNAL=${ZONE_GND_SIGNAL:-1}
export TROPFEN_FREI=${TROPFEN_FREI:-$R_TFREI}
ZONE_EDGE=$R_ZEDGE
if [ "${KURZSCHLUSS_SCHUTZ:-1}" != "0" ]; then
  if ! $VPY $R/nachbearbeitung/kurzschluss_schutz.py $BJ $RJ $TMPD/routes_sicher.json || [ ! -s $TMPD/routes_sicher.json ]; then
    echo "R44: Eingangsroute nicht geprueft; keine Abschlussplatine" >&2
    rm -rf $TMPD; exit 1
  fi
  RJ=$TMPD/routes_sicher.json
fi
GNDN=/GND; [ -n "$KLASSEN" ] && GNDN=$(python3 -c "import json,sys;K=json.load(open(sys.argv[1]));print(K.get('gnd_haupt') or (K.get('gnd') or ['/GND'])[0])" $KLASSEN)
ZONE_EDGE=$ZONE_EDGE ZUNGE=${ZUNGE:-} GND_ALLE=${GND_ALLE:-0} FILL_NETZ=${FILL_NETZ:-} $VPY $R/nachbearbeitung/make_zones.py $BJ $RJ zones_$N.json $GNDL
if [ "${STROM_VIAS:-1}" = "1" ] && [ -n "$KLASSEN" ]; then
  $VPY $R/nachbearbeitung/strom_vias.py $BJ $RJ zones_$N.json $KLASSEN routes_${N}_sv.json via=$VIA \
    clr=${SV_CLR:-$R_SVCLR} edge=$ZONE_EDGE vip=${VIP:-0} | head -8
  RJ=routes_${N}_sv.json
  ZONE_EDGE=$ZONE_EDGE ZUNGE=${ZUNGE:-} GND_ALLE=${GND_ALLE:-0} FILL_NETZ=${FILL_NETZ:-} $VPY $R/nachbearbeitung/make_zones.py $BJ $RJ zones_$N.json $GNDL > /dev/null
fi
kpy $R/kicad/import_routes.py $BASE.kicad_pcb $RJ zones_$N.json $N.kicad_pcb lagen=$NL via=$VIA zclr=${ZCLR:-0.2} 2>&1 | filt
proj() { cp $BASE.kicad_pro ${1:r}.kicad_pro; if [ -f $BASE.kicad_dru ]; then cp $BASE.kicad_dru ${1:r}.kicad_dru; fi }   # Regeln neben jede Platine
proj $N.kicad_pcb
if [ "${STITCH:-1}" = "1" ]; then
  $VPY $R/nachbearbeitung/gnd_inseln.py $N.kicad_pcb $N.kicad_pcb via=$VIA ${KLASSEN:+klassen=$KLASSEN} drc=$R_DRC h2h=$R_H2H edge=$R_EDGE ${=GNDI_OPT} | tail -4
fi
drc_zahlen() {
  rm -f "$2"                       # kein Bericht eines frueheren Laufs als neue Pruefung
  if ! kc pcb drc --format json --severity-all -o $2 $1 >/dev/null 2>&1; then
    rm -f "$2"; echo '99999 99999 99999'; return
  fi
  python3 - $2 <<'EOF'
import json, sys
try:
    d = json.load(open(sys.argv[1]))
    v = d['violations']; offen = d['unconnected_items']
    if not isinstance(v, list) or not isinstance(offen, list) or any(not isinstance(x, dict) for x in offen):
        raise ValueError('ungueltiges DRC-Format')
    if any(not isinstance(x, dict) or x.get('severity') not in ('error', 'warning', 'info') or not isinstance(x.get('type'), str) for x in v):
        raise ValueError('ungueltiger DRC-Eintrag')
    zahlen = (sum(1 for x in v if x['severity'] == 'error'), len(offen), sum(1 for x in v if x['type'] in ('track_dangling', 'via_dangling')))
except Exception:
    print(99999, 99999, 99999); sys.exit()
print(*zahlen)
EOF
}
STAND=($(drc_zahlen $N.kicad_pcb drc_$N.json))
nimm() {
  local K=$1
  if [ ! -s $K ]; then echo "   $2: kein Ergebnis, Stand bleibt"; return 1; fi
  proj $K
  local NEU=($(drc_zahlen $K ${K:r}_drc.json))
  if [ ${STAND[1]} -eq 99999 ] || [ ${NEU[1]} -eq 99999 ]; then
    echo "   $2: DRC-Pruefung fehlgeschlagen, Stand bleibt"; return 1
  fi
  if [ ${NEU[1]} -le ${STAND[1]} ] && [ ${NEU[2]} -le ${STAND[2]} ] && { [ "$3" != "lose=1" ] || [ ${NEU[3]} -lt ${STAND[3]} ]; }; then
    mv $K $N.kicad_pcb; mv ${K:r}_drc.json drc_$N.json; rm -f ${K:r}.kicad_pro ${K:r}.kicad_dru
    echo "   $2: uebernommen (DRC Fehler/offen/lose ${STAND[1]}/${STAND[2]}/${STAND[3]} -> ${NEU[1]}/${NEU[2]}/${NEU[3]})"; STAND=($NEU); return 0
  fi
  echo "   $2: verworfen (DRC Fehler/offen/lose ${STAND[1]}/${STAND[2]}/${STAND[3]} -> ${NEU[1]}/${NEU[2]}/${NEU[3]})"
  rm -f $K ${K:r}_drc.json ${K:r}.kicad_pro ${K:r}.kicad_dru; return 1
}
if [ "${ABKUERZEN:-1}" = "1" ]; then
  eval "$($VPY - $BJ $RJ ${KLASSEN:--} <<'EOF'
import json, sys, math, shlex
B = json.load(open(sys.argv[1])); R = json.load(open(sys.argv[2])); K = json.load(open(sys.argv[3])) if sys.argv[3] != '-' else {}
xs = [p[0] for o in B['outline'] for p in o]; ys = [p[1] for o in B['outline'] for p in o]
gnd = set(K.get('gnd') or ['/GND']); NZ = K.get('netze') or {}
netze = sorted({t['net'] for t in R['tracks'] if t['net'] and t['net'] not in gnd and not (NZ.get(t['net']) or {}).get('paar')})
print('ABB=%s ANETZE=%s ALAGEN=%s' % (shlex.quote('%d %d %d %d' % (math.floor(min(xs)), math.floor(min(ys)), math.ceil(max(xs)), math.ceil(max(ys)))),
      shlex.quote(';'.join(netze)), shlex.quote(','.join(sorted({t['layer'] for t in R['tracks']})))))
EOF
)"
  if [ -n "$ANETZE" ]; then
    AKOPT="${RG:+regeln=$RG} drc=$R_DRC drcpad=$R_DRCPAD drcvia=$R_DRC rand=$R_EDGE loch=$R_LOCH hmax=${ABK_HMAX:-0.15} lagen=$ALAGEN"
    TOTE=0 ABK_OPT="$AKOPT" $R/nachbearbeitung/abkuerzen_lauf.sh $N.kicad_pcb ${N}_ak.kicad_pcb $BASE.kicad_pro $BASE.kicad_dru "$ANETZE" ${=ABB} 2>&1 | filt | grep -E "^Runde" | tail -1 || true
    nimm ${N}_ak.kicad_pcb "Abkuerzen (R24)" || true
    rm -f _ak_h.json _ak.json
  fi
fi
if [ "${LEER:-1}" = "1" ] && [ "$ZONE_GND_SIGNAL" = "1" ]; then
  kpy $R/nachbearbeitung/leer_fuellen.py export $N.kicad_pcb $TMPD/cu_leer.json 2>&1 | filt > /dev/null
  if $VPY $R/nachbearbeitung/leer_fuellen.py rechnen $TMPD/cu_leer.json $TMPD/plan_leer.json rand=$ZONE_EDGE zclr=${ZCLR:-0.2} via=$VIA \
       drc=$R_DRC h2h=$R_H2H gnd=$GNDN ${ZUNGE:+zunge=$ZUNGE} 2>&1 | tail -2; then
    kpy $R/nachbearbeitung/leer_fuellen.py einfuegen $N.kicad_pcb $TMPD/plan_leer.json ${N}_leer.kicad_pcb 2>&1 | filt | tail -1 || true
    nimm ${N}_leer.kicad_pcb "Leere Flecken (K14)" || true
  fi
fi
if [ "$ZONEN" = "1" ]; then
  $R/zonen.sh $N.kicad_pcb $N.kicad_pcb ${RG:+regeln=$RG} ${KLASSEN:+klassen=$KLASSEN} clr=${ZCLR:-0.2} 2>&1 | grep -E 'Zonen-Autorouter|DRC|Stellen' || true
  proj $N.kicad_pcb; STAND=($(drc_zahlen $N.kicad_pcb drc_$N.json))
fi
if [ "${LOSE:-1}" = "1" ]; then
  for r in 1 2 3; do
    [ ${STAND[3]} -gt 0 ] || break
    kpy $R/kicad/lose_kuerzen.py $N.kicad_pcb drc_$N.json ${N}_lose.kicad_pcb 2>&1 | filt | tail -1
    [ -s ${N}_lose.kicad_pcb ] && kpy $R/kicad/lose_vias.py ${N}_lose.kicad_pcb drc_$N.json ${N}_lose.kicad_pcb 2>&1 | filt | tail -1
    nimm ${N}_lose.kicad_pcb "Lose Reste, Runde $r (R24 d)" lose=1 || break
  done
fi
if [ "${STROMPFAD_FLAECHEN:-0}" = "1" ]; then
  if [ -n "$KLASSEN" ]; then
    if $R/nachbearbeitung/strompfad_lauf.sh $N.kicad_pcb ${N}_sf.kicad_pcb $KLASSEN ${SF_GAP:-${ZCLR:-0.2}} ${SF_EDGE:-$ZONE_EDGE}; then
      if nimm ${N}_sf.kicad_pcb "Strompfad-Flaechen (R33)"; then
        RJ=${N}_sf_neu.routes.json
        cp ${N}_sf_zones.json zones_$N.json
      fi
    else
      echo "   Strompfad-Flaechen: Tor nicht bestanden; Abschluss bleibt. Kandidat/Diagnose: ${N}_sf*"
    fi
  else
    echo "   Strompfad-Flaechen: KLASSEN fehlt; Abschluss bleibt"
  fi
fi
if [ "${KURZSCHLUSS_SCHUTZ:-1}" != "0" ]; then
  if ! $VPY $R/nachbearbeitung/kurzschluss_nativ.py $N.kicad_pcb $N.kicad_pcb r44_$N.json ${NATIVE_CPU_MAX:-300} \
     || ! $VPY -c "import json,sys; sys.exit(0 if json.load(open(sys.argv[1])).get('geprueft') is True else 1)" r44_$N.json; then
    rm -f $N.kicad_pcb
    echo "R44: Abschluss nicht geprueft (Prueffehler, siehe r44_$N.json); keine Abschlussplatine" >&2
    rm -rf $TMPD; exit 1
  fi
  STAND=($(drc_zahlen $N.kicad_pcb drc_$N.json))
fi
python3 - drc_$N.json <<'EOF'
import json, sys, collections
d = json.load(open(sys.argv[1]))
fe = [v for v in d['violations'] if v['severity'] == 'error']
c = collections.Counter(v['type'] for v in fe); lose = collections.Counter(v['type'] for v in d['violations'] if v['type'] in ('track_dangling', 'via_dangling'))
print('DRC', dict(c), 'unverbunden', len(d['unconnected_items']), '| lose Bahnen %d, lose Vias %d | Warnungen %d' % (
    lose['track_dangling'], lose['via_dangling'], sum(1 for v in d['violations'] if v['severity'] != 'error')))
for v in fe[:12]:
    print('  ', v['type'], v['description'][:70], [(i['description'][:45], i['pos']['x'], i['pos']['y']) for i in v['items']])
for v in d['unconnected_items'][:12]:
    print('   offen', [(i['description'][:45], i['pos']['x'], i['pos']['y']) for i in v['items']])
EOF
[ "${BILDER:-1}" = "1" ] || { rm -rf $TMPD; exit 0; }
TMPD=$TMPD python3 - zones_$N.json <<'EOF'
import json, sys, os
z = json.load(open(sys.argv[1]))
g = set(json.load(open(os.environ['KLASSEN']))['gnd']) if os.environ.get('KLASSEN') else {'/GND'}
json.dump([q for q in z if q['net'] in g and not q.get('gnd_signal')], open(os.environ['TMPD'] + '/zones_nur_gnd.json', 'w'))
EOF
kpy $R/kicad/import_routes.py $BASE.kicad_pcb $RJ $TMPD/zones_nur_gnd.json $TMPD/${N}_bahnen.kicad_pcb lagen=$NL via=$VIA zclr=${ZCLR:-0.2} 2>&1 | filt
cp $BASE.kicad_pro $TMPD/${N}_bahnen.kicad_pro; if [ -f $BASE.kicad_dru ]; then cp $BASE.kicad_dru $TMPD/${N}_bahnen.kicad_dru; fi
for P in $N:z $TMPD/${N}_bahnen:b; do f=${P%%:*}; t=${P##*:}
  LL=(F.Cu); for k in $(seq 1 $((NL-2))); do LL+=(In$k.Cu); done; LL+=(B.Cu)
  for L in $LL; do
    kc pcb export svg --layers "$L,Edge.Cuts" --mode-single --fit-page-to-board --exclude-drawing-sheet -o $TMPD/zw${t}_$L.svg $f.kicad_pcb >/dev/null 2>&1
    perl -e 'alarm 90; exec @ARGV' $INK $TMPD/zw${t}_$L.svg --export-type=png --export-filename=$TMPD/zw${t}_$L.png --export-width=600 --export-background=white >/dev/null 2>&1 || echo "Inkscape: $L abgebrochen"
  done
done
TMPD=$TMPD $VPY - $N $NL $GNDL <<'EOF'
import sys, os
from PIL import Image, ImageDraw, ImageFont
N = sys.argv[1]; NL = int(sys.argv[2])
L = ['F.Cu'] + ['In%d.Cu' % k for k in range(1, NL - 1)] + ['B.Cu']
G = sys.argv[3].split(',')
T = ['F (oben)'] + ['In%d%s' % (k, ' (nur GND)' if 'In%d' % k in G else '') for k in range(1, NL - 1)] + ['B (unten)']
f = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', 30)
for t, name in (('z', 'Lagen_%s.png' % N), ('b', 'Lagen_nur_Bahnen_%s.png' % N)):
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
