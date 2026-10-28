""
from pathlib import Path
import runpy
import sys

target = Path(sys.argv[1]).resolve()
sys.argv = [str(target)] + sys.argv[2:]
sys.path.insert(0, str(target.parent))
runpy.run_path(str(Path(__file__).resolve().parent / 'startschutz/sitecustomize.py'))
runpy.run_path(str(target), run_name='__main__')
