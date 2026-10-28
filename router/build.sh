#!/bin/zsh
set -e
cd ${0:A:h}
cat pf_head.py pf_body.py pf_main.py > route_pf.py.neu
../.venv/bin/python -c "import ast; ast.parse(open('route_pf.py.neu').read())"
mv route_pf.py.neu route_pf.py && echo 'route_pf.py ok'   # atomar: laufende Versuche lesen nie eine halbe Datei
