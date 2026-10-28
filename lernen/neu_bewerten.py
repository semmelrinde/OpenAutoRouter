""
import json, os, sys
R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, R)
from analyse import korpus as KO


def main():
    outs = sys.argv[1].split(','); kw = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
    daten = kw.get('daten', os.path.join(R, 'lernen/daten/versuche.jsonl'))
    Z = [json.loads(z) for z in open(daten) if z.strip()]
    basis = {}; geaendert = 0
    for z in Z:
        o = next((os.path.join(x, z['board']) for x in outs if os.path.exists(os.path.join(x, z['board'], 'k%d' % z['kandidat']))), os.path.join(outs[0], z['board']))
        c = os.path.join(o, 'k%d' % z['kandidat'])
        E = z.get('ergebnis')
        if not E or not os.path.exists(os.path.join(c, 'fertig.kicad_pcb.drc.json')): continue
        if z['board'] not in basis:
            mj = os.path.join(o, 'mensch.kicad_pcb.drc.json')
            if not os.path.exists(mj):
                rg = json.load(open(os.path.join(o, 'regeln.json')))
                KO.drc_pruef(_pcb(o, z), None, os.path.join(o, 'mensch.kicad_pcb'), rg['router']['via'], 2)
            basis[z['board']] = KO.drc_auswerten(json.load(open(mj))) if os.path.exists(mj) else None
        fest = [r for r, t in json.load(open(os.path.join(o, 'klassen_wuerfel.json')))['teile'].items() if t['fest']]
        d = KO.drc_auswerten(json.load(open(os.path.join(c, 'fertig.kicad_pcb.drc.json'))), fest)
        d['platz_netto'] = KO.platz_netto(d, basis[z['board']])
        E['drc'] = d; g = KO.gueltig_voll(E, True)
        if g != z.get('gueltig'): geaendert += 1
        z['gueltig'] = g
    aus = kw.get('aus', daten)
    with open(aus + '.tmp', 'w') as f:
        for z in Z: f.write(json.dumps(z) + '\n')
    os.replace(aus + '.tmp', aus)
    print('neu bewertet: %d Zeilen, %d mit geaenderter Gueltigkeit, gueltig jetzt %d' % (len(Z), geaendert, sum(bool(z.get('gueltig')) for z in Z)))


def _pcb(o, z):
    ""
    return json.load(open(os.path.join(o, 'board.json')))['src']


if __name__ == '__main__':
    main()
