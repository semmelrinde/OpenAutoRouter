import os
import shutil

START = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'kicad_start.py')

KPY = os.environ.get('KICAD_PYTHON', '/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3')


def python_command(*args):
    if args and str(args[0]).endswith('.py') and os.path.abspath(str(args[0])) != START and os.environ.get('KICAD_PROTECTION', '1') != '0':
        args = (START,) + args
    wrapper = os.environ.get('KICAD_PYTHON_WRAPPER', '/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3')
    if wrapper and wrapper != '0':
        executable = shutil.which(os.path.expanduser(wrapper))
        if executable: return [executable, *map(str, args)]
    executable = shutil.which(os.path.expanduser(KPY))
    if not executable: raise FileNotFoundError('KiCad Python not executable: ' + KPY)
    return [executable, *map(str, args)]
