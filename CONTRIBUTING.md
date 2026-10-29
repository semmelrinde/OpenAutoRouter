# Contributing to Open Auto Router

Thanks for looking! This project wants to become the open, self-learning placer and autorouter that KiCad never had. It is
research code, so there is room for everybody: people who route boards by hand, algorithm people, ML people, students.

You do **not** need your own boards. The task generator invents as many test boards as you like, so every result here can
be reproduced by anybody with a seed number and the same setup.

## Ways to help (pick one)

| You are … | Good first contribution |
|---|---|
| **KiCad user / layouter** | Run the demo, open an issue with the picture and tell us what a human would have done differently ("this cap belongs on the other side", "this trace is too thin for 3 A"). Expert judgement is the scarcest input we have. |
| **Python developer** | Pick an issue labelled [`good first issue`](../../labels/good%20first%20issue): tests, clearer error messages, docs for options. |
| **Algorithm / optimisation person** | New repair moves in the end phase (`router/pf_main.py`, `move=1`), faster A* (`router/astar_nb.py`), better placement cost. |
| **ML / RL person** | The self-play loop (`learning/selfplay.py`) and the move choice (`learning/move_choice.py`) log every attempt with features. Better policies are very welcome. |
| **Student** | A thesis-sized topic: see the "research" label, or open a Discussion and we find one together. |

## Setup

```bash
git clone https://github.com/semmelrinde/OpenAutoRouter.git && cd OpenAutoRouter
python3.12 -m venv .venv && .venv/bin/pip install numpy scipy shapely numba pillow   # core; requirements.txt has the learning extras
.venv/bin/python learning/demo.py /tmp/oar seed=1          # first run: one generated board, placed and routed
.venv/bin/python -m unittest discover -s tests               # unit tests
```

KiCad 10 is only needed for the steps that read or write `.kicad_pcb` files (`kicad/`) and for the native DRC. The
generator reads footprints from the KiCad library folder; set `KICAD_FOOTPRINTS=/path/to/kicad/footprints` if KiCad is not
installed in the default place.

## Rules that keep the project healthy

1. **Reproducible.** Same input and same seed must give the same output file. Never iterate over a `set` or `dict` of
   names without `sorted()`; seed every random generator.
2. **Validity first, quality second.** A change that makes more boards *valid* (no clearance conflicts, nothing
   unrouted, KiCad DRC clean) beats any change that only improves the score of already valid boards. Never compare a
   score of an invalid result with a valid one.
3. **Router source.** Edit `router/pf_head.py`, `router/pf_body.py`, `router/pf_main.py`, then run `router/build.sh`.
   `router/route_pf.py` is generated, never edit it directly.
4. **No private board data.** Do not commit `.kicad_*` files, exported board JSON, routes or images of boards you are not
   allowed to share. Use generated tasks (seed + level) or clearly open-licensed boards and link to their source.
5. **Small pull requests.** One idea per PR, with a before/after number on a few seeds (see below).

## Showing that a change helps

Run the demo on a handful of seeds before and after your change and paste the summary table into the PR:

```bash
.venv/bin/python learning/demo.py /tmp/oar_before seed=1 n=5 cu_layers=4
# apply your change
.venv/bin/python learning/demo.py /tmp/oar_after  seed=1 n=5 cu_layers=4
```

Please include the CPU, the commit, and the options you used. Harder boards: `level=1` or higher, `cu_layers=6`/`8`.

## How pull requests are merged

The code in this repository is generated from the maintainer's main tree (written in German) and translated to English
automatically, so names stay stable between releases. Your pull request is reviewed here as normal; when it is accepted,
the maintainer ports it into the main tree and it comes back in the next release with your authorship credited
(`Co-authored-by`). Keep that in mind for big renames: they are hard to port, small focused changes are easy.

## Communication

- Questions, ideas, show-and-tell: [Discussions](../../discussions)
- Bugs and concrete tasks: [Issues](../../issues)
- Please be kind and constructive, see [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md).
