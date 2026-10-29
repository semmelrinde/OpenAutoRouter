import os, runpy, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kicad_protection
kicad_protection.activate()
script = os.path.abspath(sys.argv[1])
sys.argv = [script] + sys.argv[2:]
sys.path[0] = os.path.dirname(script)
runpy.run_path(script, run_name='__main__')
