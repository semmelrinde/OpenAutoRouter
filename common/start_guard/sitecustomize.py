import os
import sys
import importlib.util
import importlib.machinery
from pathlib import Path
import types
import hashlib
import json

root = os.environ.get('AUTOROUTER_START_GUARD')
if root:
    try:
        if 'common' not in sys.modules:
            local = next((p / 'common' for p in Path(sys.argv[0]).absolute().parents if (p / 'common').is_dir()), None)
            locations = ([str(local)] if local else []) + [str(Path(p) / 'common') for p in sys.path if p and (Path(p) / 'common').is_dir()]
            if not locations: locations = [os.path.join(root, 'common')]
            package = types.ModuleType('common')
            package.__path__ = list(dict.fromkeys(locations))
            package.__spec__ = importlib.machinery.ModuleSpec('common', loader=None, is_package=True)
            package.__spec__.submodule_search_locations = package.__path__
            sys.modules['common'] = package
        for name, filename in (('common.process_guard', 'process_guard.py'),
                               ('common.slots', 'slots.py')):
            spec = importlib.util.spec_from_file_location(name, os.path.join(root, 'common', filename))
            module = importlib.util.module_from_spec(spec)
            sys.modules[name] = module
            spec.loader.exec_module(module)
        sys.modules['common.process_guard'].bootstrap()
        if os.environ.get('AUTOROUTER_SOURCES_LOG'):
            sources = {name: dict(fpath=str(Path(sys.modules[name].__file__).resolve()),
                                 sha256=hashlib.sha256(Path(sys.modules[name].__file__).read_bytes()).hexdigest())
                       for name in ('common.process_guard', 'common.slots')}
            record = json.dumps(dict(pid=os.getpid(), source_list=sources)) + '\n'
            fd = os.open(os.environ['AUTOROUTER_SOURCES_LOG'], os.O_CREAT | os.O_WRONLY | os.O_APPEND, 0o600)
            try:
                os.write(fd, record.encode())
            finally:
                os.close(fd)
    except BaseException as error:
        print('Startup guard infrastructure error: %r' % error, file=sys.stderr, flush=True)
        os._exit(78)
