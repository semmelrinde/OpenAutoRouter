# Open Auto Router

An experimental, self-improving placer and autorouter for **KiCad 10** printed circuit boards, written in Python.

The goal: a router that places and routes real multi-layer boards (BGA, buck/boost converters, USB pairs, dense MCUs) the way a good human layouter would, and that **gets better on its own** by playing against generated boards.

> Status: research code. It routes generated boards and many open-hardware boards, but it is not a finished product. Contributions are very welcome.

## What is in here

| Part | Where | What it does |
|---|---|---|
| Rules | `analysis/design_rules.py` | Reads design rules from a KiCad project (`.kicad_pro`, `.kicad_dru`) |
| Net classes | `analysis/netclasses.py` | Classifies nets (power, switch node, fast signal, analog, ground), estimates currents from the circuit, builds component islands |
| Placer | `placement/placer.py` | Island-based global placement, refinement and legalisation (`mode=new` or `mode=refine`) |
| Router | `router/pf_*.py` → `router/route_pf.py` | Grid PathFinder (negotiated congestion) on all layers, numba A*, exact clearance checks against KiCad geometry, pin access, current-aware widths, differential pairs, skew meanders |
| End phase | `router/pf_main.py` (`move=1`) | **Repair moves**: for the last few conflicts, rip up a local neighbourhood and re-route it with a strategy; keep the move if the spot improves. Every move is logged with features |
| Learning | `learning/` | Synthetic task generator (`learning/tasks.py`), self-play (`learning/selfplay.py`), Thompson-sampling bandit over options, learned move selection (`learning/move_choice.py`) |
| Post-processing | `postprocessing/` | Shortcuts, teardrops/fillets, zones, stitching vias, short-circuit guard |
| Shared helpers | `common/` | Moving parts without KiCad, CPU slots for heavy runs, geometry helpers |
| KiCad bridge | `kicad/` | Export a board to JSON, import routes back, DRC via `kicad-cli` |

The main scripts start with a docstring that explains what they do and how to call them.

## Requirements

- macOS or Linux, Python 3.12, KiCad 10 (its bundled Python is used for `pcbnew` steps; set `KICAD_PYTHON=/path/to/kicad/python3` if it is not the macOS default)
- `python3 -m venv .venv && .venv/bin/pip install -r requirements.txt`

## Quick start on a generated board

No board of your own needed: the task generator invents plausible circuits (MCU/BGA, regulators, sensors, connectors) from KiCad library footprints.

```bash
R=$PWD; PY=$R/.venv/bin/python
$PY learning/tasks.py /tmp/oar seed=1 level=0     # writes board.json, design_rules.json, specs.json, task.json
cd /tmp/oar/task_000001
$PY $R/analysis/netclasses.py board.json netclasses.json design_rules=design_rules.json specs=specs.json
$PY $R/placement/placer.py board.json netclasses.json placement.json design_rules=design_rules.json mode=new
$PY - "$R" <<'EOF'                                      # apply the placement to the board without KiCad
import json, sys
sys.path.insert(0, sys.argv[1])
from common.shift import relocate
from placement.p3_groups import board_with_groups
B = json.load(open('board.json')); P = json.load(open('placement.json'))['parts']
fps = {f['ref']: f for f in B['fps']}
moves = {r: (p['x'] - fps[r]['x'], p['y'] - fps[r]['y'], p.get('rotation', 0.0)) for r, p in P.items() if r in fps}
B, _ = relocate(B, moves, sides={r: p['side'] for r, p in P.items() if r in fps})
json.dump(board_with_groups(B, json.load(open('placement.json'))), open('board_placed.json', 'w'))
EOF
$PY $R/router/route_pf.py board_placed.json routes.json design_rules=design_rules.json netclasses=netclasses.json iter=40 move=1
```

This is what `learning/selfplay.py` does for every generated task. A dense 8-layer BGA task takes a while (minutes per negotiation pass on a laptop); lower `iter=` for a first try.

For your own KiCad board: `kicad/export_board.py` (KiCad Python) → `analysis/design_rules.py` / `analysis/netclasses.py` → placer → router → `finish.sh` writes a new `.kicad_pcb` next to (never over) your project. **Always work on a copy of your board.**

## How you can help

The most valuable contributions right now:

1. **Cheaper and smarter repair moves** (`router/pf_main.py`, `move=1`): new move types (shift a small part by 0.2–0.5 mm, swap layers, merge vias of the same net), better move selection from the logged moves.
2. **Native validity gap**: cases where the internal check says "valid" but KiCad DRC disagrees (edge clearance, via connections). A failing case plus a fix is gold.
3. **Placement**: keep-outs of other footprints, room for current paths and escape routes.
4. **Benchmarks**: run the generator + router on many seeds and report valid boards per CPU hour.

Router changes are made in `router/pf_head.py`, `pf_body.py`, `pf_main.py`; run `router/build.sh` afterwards (it rebuilds `route_pf.py`). Outputs must be reproducible: same input and seed give the same file.

Please **never commit board files** (`.kicad_*`, exported JSON, routes) that you are not allowed to share. Use the generator or clearly open-licensed boards for tests and examples.

## Language

The project was originally written in German. This repository is generated from it and translated to English automatically, so names stay consistent between releases. Option names changed with the translation (for example `netclasses=`, `move=`, `mode=new`). Code comments are not included; the main entry points have English docstrings.

## License

GPL-3.0-or-later, see `LICENSE`.
