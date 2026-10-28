""
import os
import sys
import importlib.util
import importlib.machinery
from pathlib import Path
import types
import hashlib
import json

root = os.environ.get('AUTOROUTER_STARTSCHUTZ')
if root:
    try:
        if 'gemeinsam' not in sys.modules:
            local = next((p / 'gemeinsam' for p in Path(sys.argv[0]).absolute().parents if (p / 'gemeinsam').is_dir()), None)
            locations = ([str(local)] if local else []) + [str(Path(p) / 'gemeinsam') for p in sys.path if p and (Path(p) / 'gemeinsam').is_dir()]
            if not locations: locations = [os.path.join(root, 'gemeinsam')]
            package = types.ModuleType('gemeinsam')
            package.__path__ = list(dict.fromkeys(locations))
            package.__spec__ = importlib.machinery.ModuleSpec('gemeinsam', loader=None, is_package=True)
            package.__spec__.submodule_search_locations = package.__path__
            sys.modules['gemeinsam'] = package
        for name, filename in (('gemeinsam.prozessschutz', 'prozessschutz.py'),
                               ('gemeinsam.plaetze', 'plaetze.py')):
            spec = importlib.util.spec_from_file_location(name, os.path.join(root, 'gemeinsam', filename))
            module = importlib.util.module_from_spec(spec)
            sys.modules[name] = module
            spec.loader.exec_module(module)
        sys.modules['gemeinsam.prozessschutz'].bootstrap()
        if os.environ.get('AUTOROUTER_QUELLEN_LOG'):
            sources = {name: dict(pfad=str(Path(sys.modules[name].__file__).resolve()),
                                 sha256=hashlib.sha256(Path(sys.modules[name].__file__).read_bytes()).hexdigest())
                       for name in ('gemeinsam.prozessschutz', 'gemeinsam.plaetze')}
            record = json.dumps(dict(pid=os.getpid(), quellen=sources)) + '\n'
            fd = os.open(os.environ['AUTOROUTER_QUELLEN_LOG'], os.O_CREAT | os.O_WRONLY | os.O_APPEND, 0o600)
            try:
                os.write(fd, record.encode())
            finally:
                os.close(fd)
    except BaseException as error:
        print('Startschutz Infrastrukturfehler: %r' % error, file=sys.stderr, flush=True)
        os._exit(78)
