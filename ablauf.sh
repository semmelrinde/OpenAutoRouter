#!/bin/zsh
set -e
set -o pipefail
export PYTHONHASHSEED=0      # reproduzierbar: Router/Placer durchlaufen Mengen von Netznamen (WISSEN.md E25)
D=${1:A}; SRC=${2:A}; NL=${3:-6}; MODUS=${4:-neu}; GNDL=${5:-In1}
R=${0:A:h}; VPY=$R/.venv/bin/python
KPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
KC=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
BASE=base_${NL}lagen
cd $D
filt() { grep -v -E 'swig|assert|wxApp|Debug: Adding duplicate' || true }
SB=${SRC:r}
if [ "$SRC" != "$D/$BASE.kicad_pcb" ]; then
  cp $SRC $BASE.kicad_pcb; cp $SB.kicad_pro $BASE.kicad_pro; [ -f $SB.kicad_dru ] && cp $SB.kicad_dru $BASE.kicad_dru
fi
PS=$D/.ablauf_projekt.kicad_pro; cp $BASE.kicad_pro $PS
pro_zurueck() { cp $PS $BASE.kicad_pro }
VG=""; [ -f vorgaben.json ] && VG="vorgaben=$D/vorgaben.json"
rm -f regeln.json               # nie mit Regeln eines frueheren Laufs weiterarbeiten
if [ -n "${REGELN_QUELLE:-}" ]; then
  echo "== A1 Regeln (Quelle $REGELN_QUELLE)"
  $VPY $R/analyse/regeln.py $REGELN_QUELLE lagen=$NL out=regeln.json | sed -n 1,2p
else
  echo "== A1 Regeln (F1/F4: aus der Platine, das neueste Board nur als Vorlage)"
  $VPY $R/analyse/regeln.py $BASE.kicad_pcb lagen=$NL out=regeln.json rueckfall=neuestes vorlage=1 | sed -n 1,2p
  cp $BASE.kicad_pro $PS                                  # eine uebernommene Vorlage gehoert jetzt zum Projekt der Arbeitskopie
fi
[ -s regeln.json ] || { echo "regeln.json fehlt" >&2; exit 2; }
VIA_RG=$(python3 -c "import json;print(json.load(open('regeln.json'))['router']['via'])")
echo "== Export";          $KPY $R/kicad/export_board.py $BASE.kicad_pcb board.json 2>&1 | filt
echo "== A2 Klassen";      $VPY $R/analyse/klassen.py board.json klassen.json regeln=regeln.json $VG | head -8
if [ "${LAGENAUFBAU:-0}" = "1" ]; then
  $VPY $R/analyse/regeln.py aufbau board.json klassen.json "gndlagen=$GNDL" out=lagenaufbau.json
fi
PL_T=(); PL_VK=(); RT_STIL=(); KL_ROUTE=klassen.json
if [ "${EINSTELLUNGEN:-}" = "gelernt" ]; then
  echo "== Gelernte Einstellungen (H11, lernen/einstellungen.py; ausdruecklich gesetzte Variablen gehen vor)"
  eval "$($VPY $R/lernen/einstellungen.py board.json klassen.json regeln.json lagen=$NL vorgabe=ablauf format=sh ${LENKUNG:+lenkung=$LENKUNG} 2>einstellungen.txt)"
  sed 's/^# /   /' einstellungen.txt
  if [ "$MODUS" = "neu" ]; then   # gelernt mit modus=neu (Lernschleife); verfeinern bleibt bei den Voreinstellungen
    if [ -z "${KREUZ:-}" ] && [ "$E_HERKUNFT_KREUZ" = "gelernt" ]; then KREUZ=$E_KREUZ; fi
    if [ -z "${TRENN:-}" ] && [ "$E_HERKUNFT_TRENN" = "gelernt" ]; then TRENN=$E_TRENN; fi
    if [ -z "${VIA_KOSTEN:-}" ] && [ "$E_HERKUNFT_VIA_KOSTEN" = "gelernt" ]; then VIA_KOSTEN=$E_VIA_KOSTEN; fi
    if [ -n "${TRENN:-}" ]; then PL_T=(trenn=$TRENN); fi
  fi
  if [ -z "${VIA_F:-}" ] && [ "$E_HERKUNFT_VIA_F" = "gelernt" ]; then VIA_F=$E_VIA_F; fi
  if [[ "${ROUTER_OPT:-}" == *stil=* ]] || [[ "${ROUTER_OPT:-}" == *winkel45=* ]]; then STIL=""   # Stil steht ausdruecklich in ROUTER_OPT
  elif [ "${STIL:-}" = "gelernt" ]; then   # R6: Stil waehlt der Nutzer je Platine; gelernter Stil nur auf ausdrueckliche Bitte
    if [ "$E_HERKUNFT_STIL" = "gelernt" ]; then STIL=$E_STIL; else STIL=""; fi
  elif [ -z "${STIL:-}" ] && [ "$E_HERKUNFT_STIL" = "gelernt" ]; then
    echo "   Stil: die Lernschleife schlaegt $E_STIL vor; bleibt beim Standard (R6, uebernehmen mit STIL=gelernt)"
  fi
  echo "   Klasse $E_KLASSE, genutzt: kreuz=${KREUZ:-2} trenn=${TRENN:-3} via_kosten=${VIA_KOSTEN:-0.6} via_f=${VIA_F:-1} stil=${STIL:-standard}"
fi
if [ -n "${VIA_KOSTEN:-}" ]; then PL_VK=(via_kosten=$VIA_KOSTEN); fi
if [[ "${ROUTER_OPT:-}" == *stil=organisch* ]] || [[ "${ROUTER_OPT:-}" == *winkel45=0* ]]; then STILE=(organisch); STIL_FEST=1
elif [[ "${ROUTER_OPT:-}" == *stil=* ]] || [[ "${ROUTER_OPT:-}" == *winkel45=* ]]; then STILE=(45); STIL_FEST=1
else
  STIL_FEST=0
  if [ -z "${STIL:-}" ] || [ "$STIL" = "gelernt" ]; then
    STIL=$(python3 -c "import json,sys;v=(json.load(open(sys.argv[1])) if sys.argv[1] else {}).get('stil') or '45';print(','.join(v) if isinstance(v,list) else v)" "${VG#vorgaben=}")
  fi
  STILE=(${(s:,:)${STIL//standard/45}})
fi
echo "== Stil: ${(j:, :)STILE} (R6; 45 Grad mit Ecken bis ${RUNDUNG:-0.5} mm gerundet, organisch gestrafft)"
silk() {   # silk <ordner>: Referenzen/Werte verstecken, Silk ueber Kante/Kupfer entfernen (P12), Platine bleibt sonst gleich
  [ "${SILK:-1}" = "1" ] || return 0
  ( cd $1 && $KC pcb drc --format json -o drc_platziert.json $BASE.kicad_pcb >/dev/null 2>&1 || true
    $KPY $R/kicad/silk_aufraeumen.py $BASE.kicad_pcb drc_platziert.json silk.kicad_pcb 2>&1 | filt | tail -1
    [ -f silk.kicad_pcb ] && mv silk.kicad_pcb $BASE.kicad_pcb; rm -f silk.kicad_pro; cp $PS $BASE.kicad_pro )
}
PLS=(.)                       # Ordner mit fertig platzierter Grundplatine, board.json, klassen.json (je Platzierungskandidat)
GEROUTET=0
if [ "${PORTFOLIO:-0}" = "1" ]; then
  if [ "$MODUS" != "verfeinern" ]; then
    echo "PORTFOLIO=1 braucht modus=verfeinern (kreuz=2 plus IR-Feinschliff)." >&2
    exit 2
  fi
  PD=${PORTFOLIO_DIR:-$D/portfolio}
  PARGS=(board.json regeln.json "$PD" --source "$R" --pcb "$D/$BASE.kicad_pcb"
    --gndlagen "$GNDL" --iter "${ITER:-60}" --seiten "${SEITEN:-FB}"
    --seed "${PLACER_SEED:-1}" --hashseed "$PYTHONHASHSEED")
  [ -f vorgaben.json ] && PARGS+=(--vorgaben "$D/vorgaben.json")
  for po in ${=ROUTER_OPT:-kurz=1 fair=1}; do PARGS+=(--router-opt "$po"); done
  caffeinate -i -s "$VPY" "$R/codex/portfolio/run.py" "${PARGS[@]}"
  cp "$PD/selected/base.kicad_pcb" "$BASE.kicad_pcb"
  for f in board.json klassen.json klassen.txt routes.json placement.json; do cp "$PD/selected/$f" "$f"; done
  silk .
  GEROUTET=1
elif [ "$MODUS" != "keiner" ]; then
  echo "== A3-A5 Placer ($MODUS)"
  if [ "$MODUS" = "neu" ] && [ "${KANDIDATEN:-1}" -gt 1 ]; then
    KL=()
    for k in $(seq 1 $KANDIDATEN); do
      $VPY $R/gemeinsam/plaetze.py cpu=${PLACER_CPU:-7200} -- caffeinate -i -s $VPY $R/platzierung/placer.py board.json klassen.json placement_k$k.json regeln=regeln.json modus=$MODUS seiten=${SEITEN:-FB} kreuz=${KREUZ:-2} trenn=${TRENN:-3} $PL_VK seed=$k | grep -E "Kennzahlen nachher"
      KL+=(placement_k$k.json)
    done
    $VPY $R/analyse/varianten.py rangfolge board.json klassen.json regeln.json $KL auswahl=${AUSWAHL:-faust} | tee auswahl.txt
    RANG=($(awk '/^RANG/ {print $3}' auswahl.txt)); NR=${ROUTEN:-2}; [ $NR -gt ${#RANG} ] && NR=${#RANG}
    PLS=()
    for i in $(seq 1 $NR); do
      P=pl$i; rm -rf $P; mkdir -p $P; cp $BASE.kicad_pcb $BASE.kicad_dru $P/ 2>/dev/null || cp $BASE.kicad_pcb $P/; cp $PS $P/$BASE.kicad_pro
      cp ${RANG[$i]} $P/placement.json; cp regeln.json $P/
      ( cd $P && $KPY $R/platzierung/apply_place.py $BASE.kicad_pcb placement.json platziert.kicad_pcb 2>&1 | filt && mv platziert.kicad_pcb $BASE.kicad_pcb && rm -f platziert.kicad_pro && cp $PS $BASE.kicad_pro )
      silk $P
      ( cd $P && $KPY $R/kicad/export_board.py $BASE.kicad_pcb board.json placement=placement.json 2>&1 | filt >/dev/null && $VPY $R/analyse/klassen.py board.json klassen.json regeln=regeln.json $VG > /dev/null )
      PLS+=($P); echo "   Kandidat $i: ${RANG[$i]} -> $P"
    done
  else
    $VPY $R/gemeinsam/plaetze.py cpu=${PLACER_CPU:-7200} -- caffeinate -i -s $VPY $R/platzierung/placer.py board.json klassen.json placement.json regeln=regeln.json modus=$MODUS seiten=${SEITEN:-FB} kreuz=${KREUZ:-2} $PL_T $PL_VK bild=pl | grep -E "Kennzahlen|Legal|fertig"
    $KPY $R/platzierung/apply_place.py $BASE.kicad_pcb placement.json platziert.kicad_pcb 2>&1 | filt
    mv platziert.kicad_pcb $BASE.kicad_pcb; rm -f platziert.kicad_pro; pro_zurueck
    if [ "${WEICH:-0}" = "1" ]; then   # Codex' analytischer Detailplacer (weiche Inseln) als Feinschliff, E30: kuerzer, aber mehr Kupferverlust
      echo "== A5b Weicher Detailplacer (Codex)"
      $KPY $R/kicad/export_board.py $BASE.kicad_pcb board.json placement=placement.json 2>&1 | filt
      $VPY $R/analyse/klassen.py board.json klassen.json regeln=regeln.json $VG > /dev/null
      $VPY $R/gemeinsam/plaetze.py cpu=${PLACER_CPU:-7200} -- caffeinate -i -s $VPY $R/codex/analytisch/placer.py board.json klassen.json placement_weich.json --mode weich --regeln regeln.json | tail -1
      $KPY $R/platzierung/apply_place.py $BASE.kicad_pcb placement_weich.json platziert.kicad_pcb 2>&1 | filt
      mv platziert.kicad_pcb $BASE.kicad_pcb; rm -f platziert.kicad_pro; pro_zurueck
    fi
    silk .
    $KPY $R/kicad/export_board.py $BASE.kicad_pcb board.json placement=placement.json 2>&1 | filt
    $VPY $R/analyse/klassen.py board.json klassen.json regeln=regeln.json $VG > /dev/null
  fi
else
  silk .                       # P12 gilt auch ohne neue Platzierung
  $KPY $R/kicad/export_board.py $BASE.kicad_pcb board.json 2>&1 | filt >/dev/null
fi
NT_AN=$(python3 -c "import json;B=json.load(open('board.json'));K=json.load(open('klassen.json'));print(1 if B.get('via_sperre') and K.get('net_ties') else 0)")
[ "${NT:-1}" = "1" ] || NT_AN=0
PLATZ=(); [ "${CPU_MAX:-7200}" -gt 0 ] && PLATZ=(cpu=$(( ${CPU_MAX:-7200} * 5 / 4 + 120 )))   # harte Grenze ueber dem weichen cpu_max des Routers
PFS=(${=PFMUL:-1.25 1.24 1.26}); PFS=(${PFS[1,${LAEUFE:-2}]})
rest_pose() {   # absolute Zielpose vor JEDEM Geometrie-/Net-Tie-Nachlauf, auch je Variante
  ( cd "$1"
    if $VPY -c 'import json,sys;sys.exit(not bool(json.load(open("routes.json")).get("rest_board")))'; then
      $KPY $R/kicad/rest_platzierung.py $BASE.kicad_pcb routes.json rest_platz.kicad_pcb 2>&1 | filt
      mv rest_platz.kicad_pcb $BASE.kicad_pcb; rm -f rest_platz.kicad_pro rest_platz.kicad_dru
      cp $PS $BASE.kicad_pro
      $VPY -c 'import sys;sys.path.insert(0,sys.argv[1]);from nachbearbeitung.restkonflikte import lokales_board_uebernehmen;lokales_board_uebernehmen("board.json","routes.json")' $R
    fi )
}
form() {   # form <ordner>: routes.json -> routes_form.json nach dem Stil der Variante (R6, R5, R18, R19)
  rest_pose "$1"
  local st=$(cat $1/stil.txt 2>/dev/null || echo 45)
  if [ "$st" = "organisch" ]; then
    $VPY $R/nachbearbeitung/pulltight_routes.py $1/board.json $1/routes.json $1/routes_form.json regeln=$1/regeln.json klassen=$1/klassen.json rmax=${RMAX_ORGANISCH:-3} > $1/form.log 2>&1 \
      && echo "   $1: organisch gestrafft ($(python3 -c "import json,sys;s=json.load(open(sys.argv[1]))['organisch_nachlauf']['statistik'];print('Boegen %s, Eckpunkte weg %s' % (s.get('boegen',0), s.get('eckpunkte_entfernt',0)))" $1/routes_form.json))" \
      || { echo "   $1: Straffen fehlgeschlagen ($(tail -1 $1/form.log)), Router-Bahnen bleiben"; cp $1/routes.json $1/routes_form.json; }
  elif [ "${RUNDUNG:-0.5}" != "0" ]; then
    $VPY $R/nachbearbeitung/pulltight_routes.py $1/board.json $1/routes.json $1/routes_form.json regeln=$1/regeln.json klassen=$1/klassen.json ziehen=0 rmax=${RUNDUNG:-0.5} > $1/form.log 2>&1 \
      && echo "   $1: 45 Grad, Ecken gerundet ($(python3 -c "import json,sys;s=json.load(open(sys.argv[1]))['organisch_nachlauf']['statistik'];print('Boegen %s' % s.get('boegen',0))" $1/routes_form.json))" \
      || { echo "   $1: Runden fehlgeschlagen ($(tail -1 $1/form.log)), Router-Bahnen bleiben"; cp $1/routes.json $1/routes_form.json; }
  else
    cp $1/routes.json $1/routes_form.json
  fi
}
nt_nach() {   # nt_nach <ordner>: Net-Tie-Nacharbeit am Routing (K6, E19, E22, E54), Regeln der Platine
  rest_pose "$1"
  [ "$NT_AN" = "1" ] || return 0
  ( cd $1
    [ -s stern.json ] && $VPY $R/nachbearbeitung/stern_vias.py vias routes.json stern.json routes.json via=$VIA_RG | tail -2
    eval "$(python3 -c "import json;r=json.load(open('regeln.json'))['router'];d,b=r['via'].split('/');print('VD=%s VB=%s CL=%g CP=%g H2=%s LO=%s' % (d,b,float(r['drc'])+0.005,float(r.get('drcpad',r['drc']))+0.005,r['h2h'],r.get('loch_clr',0.2)))")"
    GN=$(python3 -c "import json;K=json.load(open('klassen.json'));print(K.get('gnd_haupt') or K['gnd'][0])")
    $VPY $R/nachbearbeitung/gnd_via_nt.py board.json routes.json routes.json via=$VD drill=$VB clr=$CL h2h=$H2 gnd=$GN 2>&1 | tail -1
    $VPY $R/nachbearbeitung/via_nt_abstand.py board.json routes.json routes.json hole_cu=$LO drill=$VB via=$VD clr=$CL clrpad=$CP h2h=$H2 2>&1 | tail -1 ) || true
}
if [ "$GEROUTET" = "0" ]; then
  echo "== A6 Routen: ${#PLS} Platzierung(en) x ${#PFS} Lauf/Laeufe (pfmul ${(j:, :)PFS}) x ${#STILE} Stil(e)"
  rm -rf var; mkdir var; VARS=()
  for P in $PLS; do
    for pf in $PFS; do
      for st in $STILE; do
        V=var/${P//[.\/]/}${P:+_}p${pf}_s$st; V=${V//var\/_/var/}; mkdir -p $V
        cp $P/$BASE.kicad_pcb $P/board.json $P/klassen.json regeln.json $V/; cp $PS $V/$BASE.kicad_pro; [ -f $BASE.kicad_dru ] && cp $BASE.kicad_dru $V/
        [ -f $P/placement.json ] && cp $P/placement.json $V/
        [ -f vorgaben.json ] && cp vorgaben.json $V/
        echo $st > $V/stil.txt; echo $pf > $V/pfmul.txt; VARS+=($V)
      done
    done
  done
  for V in $VARS; do
    ( cd $V
      KLR=klassen.json
      if [ -n "${VIA_F:-}" ] && [ "$VIA_F" != "1" ]; then   # Via-Faktor nur fuer den Router (klassen.json bleibt fuer Bewertung und Nachbessern)
        $VPY -c "import json,sys
K=json.load(open('klassen.json')); f=float(sys.argv[1])
for k in K['netze'].values():
    if 'via' in k: k['via']=round(k['via']*f,3)
json.dump(K,open('klassen_route.json','w'))" $VIA_F
        KLR=klassen_route.json
      fi
      BRT=board.json; GOV=""
      if [ "$NT_AN" = "1" ]; then   # Sternpunkt-Vias der Kelvin-Messmasse vor dem Routen fest (K6, E54)
        $VPY $R/nachbearbeitung/stern_vias.py kandidaten board.json stern_kand.json via=$VIA_RG > stern.log 2>&1 || true
        $KPY $R/nachbearbeitung/stern_vias.py pruefen $BASE.kicad_pcb stern_kand.json stern.json $BASE.kicad_pro >> stern.log 2>&1 || true
        if [ -s stern.json ]; then
          GOV=$($VPY $R/nachbearbeitung/stern_vias.py pads board.json stern.json board_route.json via=$VIA_RG lagen=$NL | tail -1) && BRT=board_route.json
        fi
      fi
      ST=(stil=$(cat stil.txt)); [ "$STIL_FEST" = "1" ] && ST=()
      $VPY $R/gemeinsam/plaetze.py $PLATZ -- caffeinate -i -s $VPY $R/router/route_pf.py $BRT routes.json regeln=regeln.json \
        klassen=$KLR gndlagen=$GNDL iter=${ITER:-60} pf0=1 pfmul=$(cat pfmul.txt) hinc=1.0 ziel_ab=20 ${=ROUTER_OPT:-kurz=1 fair=1} $ST \
        cpu_max=${CPU_MAX:-7200} ${GOV:+gnd_ohne_via=$GOV} kurzschluss_schutz=${KURZSCHLUSS_SCHUTZ:-1} png=vorschau > route.log 2>&1 || echo "   $V: Router mit Fehler beendet (route.log)"
      echo "   $V: $(grep -E 'Ende|Bahnen' route.log | filt | tail -1)" ) &
  done
  wait
  for V in $VARS; do [ -s $V/routes.json ] || { echo "   $V: kein Routing" ; VARS=(${VARS:#$V}); }; done
  [ ${#VARS} -gt 0 ] || { echo "Kein Routerlauf hat ein Ergebnis geschrieben" >&2; exit 3; }
  for V in $VARS; do nt_nach $V; form $V; done
  if [ ${#VARS} -gt 1 ]; then
    echo "== Auswahl unter ${#VARS} Varianten (fertigstellen ohne Bilder, Tor: gueltig mit KiCad-DRC, dann Note v3)"
    for V in $VARS; do
      ( cd $V && KLASSEN=klassen.json REGELN=regeln.json BILDER=0 ABKUERZEN=0 LEER=0 LOSE=0 BASE=$BASE ZCLR=${ZCLR:-0.15} ZONE_EDGE=${ZONE_EDGE:-0.25} \
        $VPY $R/gemeinsam/plaetze.py cpu=${FERTIG_CPU:-14400} -- caffeinate -i -s $R/fertigstellen.sh . routes_form.json fertig $NL $VIA_RG $GNDL > fertig.log 2>&1 || echo "   $V: fertigstellen mit Fehler (fertig.log)" ) &
    done
    wait
    ZONE_EDGE=${ZONE_EDGE:-0.25} ZCLR=${ZCLR:-0.15} ZONE_GND_SIGNAL=${ZONE_GND_SIGNAL:-1} $VPY $R/analyse/varianten.py waehle varianten.json $VARS | tee auswahl_varianten.txt | grep -v GEWAEHLT
    W=$(awk '/^GEWAEHLT/ {print $2}' auswahl_varianten.txt)
  else
    W=${VARS[1]}
  fi
  echo "   gewaehlt: $W"
  for f in $BASE.kicad_pcb board.json klassen.json routes.json routes_form.json stil.txt; do cp $W/$f $f; done
  for f in placement.json stern.json board_route.json klassen_route.json; do [ -f $W/$f ] && cp $W/$f $f; done
  pro_zurueck
  [ -d $W/vorschau ] && rm -rf vorschau && cp -R $W/vorschau vorschau || true
  GEROUTET=1
else
  echo ${STILE[1]} > stil.txt; nt_nach .; form .
fi
rest_pose .
echo "== A7/A8 Selbststaendig nachbessern (NACHBESSERN=0 zum Abschalten)"
if [ "${NACHBESSERN:-1}" = "1" ]; then
  ZONE_EDGE=${ZONE_EDGE:-0.25} ZCLR=${ZCLR:-0.15} ZONE_GND_SIGNAL=${ZONE_GND_SIGNAL:-1} \
    $VPY $R/nachbearbeitung/nachbessern.py . runden=${RUNDEN:-3} parallel=${PARALLEL:-8} 2>&1 | grep -E "Basis|uebernommen|zusaetzlich|Ergebnis"
  if [ -s placement_nach.json ]; then
    $KPY $R/platzierung/apply_place.py $BASE.kicad_pcb placement_nach.json nachgebessert.kicad_pcb 2>&1 | filt
    mv nachgebessert.kicad_pcb $BASE.kicad_pcb; rm -f nachgebessert.kicad_pro; pro_zurueck
    cp board_nach.json board.json; cp routes_nach.json routes.json
    form .
  fi
fi
echo "== A9/A10 Fertigstellen (GND auf allen Lagen, Stitching, Abkuerzen, leere Flecken, lose Reste, DRC)"
KLASSEN=klassen.json REGELN=regeln.json BASE=$BASE ZCLR=${ZCLR:-0.15} ZONE_EDGE=${ZONE_EDGE:-0.25} $VPY $R/gemeinsam/plaetze.py cpu=${FERTIG_CPU:-14400} -- caffeinate -i -s $R/fertigstellen.sh . routes_form.json fertig $NL \
  $VIA_RG $GNDL 2>&1 | grep -E "DRC|offen|GND-Inseln|Lagen_|uebernommen|verworfen|Runde" | awk 'NR<=20'
$VPY $R/gemeinsam/plaetze.py cpu=${BEWERTUNG_CPU:-3600} -- $VPY $R/analyse/endstand.py fertig.kicad_pcb klassen.json routes_form.json
echo "== Bewertung und Pruefbilder fuer die KI"
$VPY $R/gemeinsam/plaetze.py cpu=${BEWERTUNG_CPU:-3600} -- $VPY $R/analyse/bewertung.py board_fertig.json routes_fertig.json klassen.json ir=1 bild=pruef json=bewertung.json | awk 'NR<=6'
echo "== Regelbericht (bericht.txt; Paket 4f)"
{
  echo "# Regelbericht $(date '+%Y-%m-%d %H:%M'), Stil $(cat stil.txt), Regeln $(python3 -c "import json;r=json.load(open('regeln.json'));print(r.get('quelle_art','?'), r['quelle'])")"
  [ -f varianten.json ] && { echo; echo "## Varianten (analyse/varianten.py)"; cat auswahl_varianten.txt; }
  echo; echo "## KiCad-DRC fertig.kicad_pcb"
  python3 - drc_fertig.json <<'EOF' || true
import json, sys, collections
d = json.load(open(sys.argv[1])); v = d['violations']
print('Fehler', dict(collections.Counter(x['type'] for x in v if x['severity'] == 'error')), '| offen', len(d['unconnected_items']),
      '| lose', sum(1 for x in v if x['type'] in ('track_dangling', 'via_dangling')), '| Warnungen', dict(collections.Counter(x['type'] for x in v if x['severity'] != 'error')))
EOF
  echo; echo "## Bewertung der Endplatine (routes_fertig.json, Note v3 mit finalem Kupfer)"
  cat endstand.json; echo
  $VPY -c "import json; e=json.load(open('bewertung.json')); print('GUELTIG' if e['gueltig'] else 'UNGUELTIG'); print('note-v3', e['note_v3']); print(json.dumps(e['regeln'], indent=1))"
  if [ "${LAGENAUFBAU:-0}" = "1" ]; then
    echo; echo "## Lagenaufbau K19 (tatsächliche Endfüllung)"
    $VPY $R/analyse/regeln.py aufbau board_fertig.json klassen.json routes=routes_fertig.json out=lagenaufbau_fertig.json
  fi
  echo; echo "## Regelpruefung (analyse/regelpruefung.py)"
  $VPY $R/analyse/regelpruefung.py board_fertig.json klassen.json routes_fertig.json json=regeln_pruef.json 2>&1 || true
  echo; echo "## Kennzahlen R19 (Endplatine, Boegen als dichte Sehnen)"
  $VPY $R/analyse/kennzahlen.py routes_fertig.json board=board_fertig.json 2>&1 || true
  python3 -c "import json,sys;s=(json.load(open(sys.argv[1])).get('organisch_nachlauf') or {}).get('statistik') or {};print('Stil-Nachlauf: Boegen %s, Eckpunkte weg %s, Bahnen geaendert %s, geschuetzt (Leistungsnetze) %s' % (s.get('boegen',0), s.get('eckpunkte_entfernt',0), s.get('bahnen_geaendert',0), s.get('geschuetzt',0)))" routes_form.json 2>&1 || true
  echo; echo "## Kopplung (analyse/kopplung.py)"
  $VPY $R/analyse/kopplung.py routes_fertig.json klassen.json 2>&1 || true
  echo; echo "## Topologie (Endplatine; dichte Bogensehnen koennen Knoten zaehlen)"
  $VPY $R/analyse/topologie.py routes_fertig.json board=board_fertig.json 2>&1 || true
} > bericht.txt
grep -E "^Fehler|^GUELTIG|^UNGUELTIG|^note-v3|^Kopplung" bericht.txt | head -6 || true
