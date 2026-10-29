import json
from pathlib import Path

FILE = Path(__file__).with_name('standard.json')


def goal(specs=None, explicit=None, context='evaluation'):
    if explicit is not None: return explicit
    if (specs or {}).get('evaluation') is not None: return specs['evaluation']
    with FILE.open() as f: z = json.load(f)['evaluation']
    if z not in ('v3', 'minimax'): raise ValueError('standard.evaluation must be v3 or minimax')
    return 'gate_check' if z == 'v3' and context == 'endurance_run' else z
