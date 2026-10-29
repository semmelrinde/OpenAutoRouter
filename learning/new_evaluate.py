import json, os, sys
R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, R)
from analysis import corpus as KO


def main():
    outs = sys.argv[1].split(','); kw = dict(a.split('=', 1) for a in sys.argv[2:] if '=' in a)
    data = kw.get('data', os.path.join(R, 'learning/data/attempts.jsonl'))
    Z = [json.loads(z) for z in open(data) if z.strip()]
    basis = {}; changed = 0
    for z in Z:
        o = next((os.path.join(x, z['board']) for x in outs if os.path.exists(os.path.join(x, z['board'], 'k%d' % z['candidate']))), os.path.join(outs[0], z['board']))
        c = os.path.join(o, 'k%d' % z['candidate'])
        E = z.get('outcome')
        if not E or not os.path.exists(os.path.join(c, 'finished.kicad_pcb.drc.json')): continue
        if z['board'] not in basis:
            mj = os.path.join(o, 'human.kicad_pcb.drc.json')
            if not os.path.exists(mj):
                rg = json.load(open(os.path.join(o, 'design_rules.json')))
                KO.drc_chk(_pcb(o, z), None, os.path.join(o, 'human.kicad_pcb'), rg['router']['via'], 2)
            basis[z['board']] = KO.drc_evaluate(json.load(open(mj))) if os.path.exists(mj) else None
        pinned = [r for r, t in json.load(open(os.path.join(o, 'netclasses_dice.json')))['part_list'].items() if t['pinned']]
        d = KO.drc_evaluate(json.load(open(os.path.join(c, 'finished.kicad_pcb.drc.json'))), pinned)
        d['slot_usable'] = KO.slot_usable(d, basis[z['board']])
        E['drc'] = d; g = KO.valid_whole(E, True)
        if g != z.get('is_valid'): changed += 1
        z['is_valid'] = g
    off = kw.get('off', data)
    with open(off + '.tmp', 'w') as f:
        for z in Z: f.write(json.dumps(z) + '\n')
    os.replace(off + '.tmp', off)
    print('re-evaluated: %d rows, %d with changed validity, valid now %d' % (len(Z), changed, sum(bool(z.get('is_valid')) for z in Z)))


def _pcb(o, z):
    return json.load(open(os.path.join(o, 'board.json')))['src']


if __name__ == '__main__':
    main()
