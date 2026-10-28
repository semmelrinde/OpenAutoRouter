""
import os, runpy, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kicad_schutz
kicad_schutz.aktivieren()
skript = os.path.abspath(sys.argv[1])
sys.argv = [skript] + sys.argv[2:]
sys.path[0] = os.path.dirname(skript)
runpy.run_path(skript, run_name='__main__')
