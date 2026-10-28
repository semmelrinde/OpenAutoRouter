""
import json
from pathlib import Path

DATEI = Path(__file__).with_name('standard.json')


def ziel(vorgaben=None, explizit=None, kontext='bewertung'):
    if explizit is not None: return explizit
    if (vorgaben or {}).get('bewertung') is not None: return vorgaben['bewertung']
    with DATEI.open() as f: z = json.load(f)['bewertung']
    if z not in ('v3', 'minimax'): raise ValueError('standard.bewertung muss v3 oder minimax sein')
    return 'tor' if z == 'v3' and kontext == 'dauerlauf' else z
