# Open Auto Router

An experimental, self-improving placer and autorouter for **KiCad 10** printed circuit boards, written in Python.

The goal: a router that places and routes real multi-layer boards (BGA, buck/boost converters, USB pairs, dense MCUs) the way a good human layouter would, and that **gets better on its own** by playing against generated boards.

> Status: research code. It routes generated boards and many open-hardware boards, but it is not a finished product. Comments and identifiers are mostly in German (see *Language* below). Contributions are very welcome.

## What is in here

| Part | Folder | What it does |
|---|---|---|
| Rules | `analyse/regeln.py` | Reads design rules from a KiCad project (`.kicad_pro`, `.kicad_dru`) |
| Net classes | `analyse/klassen.py` | Classifies nets (power, switch node, fast signal, analog, ground), estimates currents from the schematic, builds component islands |
| Placer | `platzierung/placer.py` | Island-based global placement, refinement and legalisation (`modus=neu` or `modus=verfeinern`) |
| Router | `router/pf_*.py` → `router/route_pf.py` | Grid PathFinder (negotiated congestion) on all layers, numba A*, exact clearance checks against KiCad geometry, pin access, current-aware widths, differential pairs, skew meanders |
| End phase | `router/pf_main.py` (`zug=1`) | **Repair moves**: for the last few conflicts, rip up a local neighbourhood and re-route it with a strategy; keep the move if the spot improves. Every move is logged with features |
| Learning | `lernen/` | Synthetic task generator, self-play, Thompson-sampling bandit over options, learned move selection (`lernen/zugwahl.py`) |
| Post-processing | `nachbearbeitung/` | Shortcuts, teardrops/fillets, zones, stitching vias, short-circuit guard |
| KiCad bridge | `kicad/` | Export a board to JSON, import routes back, DRC via `kicad-cli` |

## Requirements

- macOS or Linux, Python 3.12, KiCad 10 (its bundled Python is used for `pcbnew` steps; set `KICAD_PYTHON=/path/to/kicad/python3` if it is not the macOS default)
- `python3 -m venv .venv && .venv/bin/pip install -r requirements.txt`

## Quick start on a generated board

No board of your own needed: the task generator invents plausible circuits (MCU/BGA, regulators, sensors, connectors) from KiCad library footprints.

```bash
.venv/bin/python lernen/aufgaben.py /tmp/oar seed=1 stufe=0          # writes board.json, regeln.json, vorgaben.json
cd /tmp/oar/aufgabe_*                                                     # (folder name printed by the generator)
../../path/to/.venv/bin/python <repo>/analyse/klassen.py board.json klassen.json regeln=regeln.json vorgaben=vorgaben.json
<repo>/.venv/bin/python <repo>/platzierung/placer.py board.json klassen.json placement.json regeln=regeln.json modus=neu
<repo>/.venv/bin/python <repo>/router/route_pf.py board.json routes.json regeln=regeln.json klassen=klassen.json iter=40 zug=1
```

For your own KiCad board: `kicad/export_board.py` (KiCad Python) → `analyse/regeln.py` / `analyse/klassen.py` → placer → router → `fertigstellen.sh` writes a new `.kicad_pcb` next to (never over) your project. **Always work on a copy of your board.**

## How you can help

The most valuable contributions right now:

1. **Cheaper and smarter repair moves** (`router/pf_main.py`, `zug=1`): new move types (shift a small part by 0.2–0.5 mm, swap layers, merge vias of the same net), better move selection from the logged moves.
2. **Native validity gap**: cases where the internal check says "valid" but KiCad DRC disagrees (edge clearance, via connections). A failing case plus a fix is gold.
3. **Placement**: keep-outs of other footprints, room for current paths and escape routes.
4. **Benchmarks**: run the generator + router on many seeds and report valid boards per CPU hour.

Router changes are made in `router/pf_head.py`, `pf_body.py`, `pf_main.py`; run `router/build.sh` afterwards (it rebuilds `route_pf.py`). Outputs must be reproducible: same input and seed give the same file.

Please **never commit board files** (`.kicad_*`, exported JSON, routes) that you are not allowed to share. Use the generator or clearly open-licensed boards for tests and examples.

## Language

The project grew in German. Most comments, option names (`klassen=`, `zug=`, `modus=neu`) and log messages are German. English issues and pull requests are welcome; translating comments is a welcome contribution too.

*Kurz auf Deutsch:* Experimenteller, selbstlernender Placer und Autorouter für KiCad 10. Kommentare und Optionen sind deutsch. Beiträge willkommen, aber bitte keine Leiterplatten-Dateien einchecken, die nicht frei geteilt werden dürfen.

## License

GPL-3.0-or-later, see `LICENSE`.
