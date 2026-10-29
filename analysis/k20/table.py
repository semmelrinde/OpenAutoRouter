import json, os, statistics
S = os.path.dirname(os.path.abspath(__file__))
C = os.path.expanduser('~/Library/Caches/Autorouter/selfplay')
BOARDS = [('ModelExample', 'model_example', 'KiCad-Fill', ['In1', 'In6']),
          ('a164_k4', 'test/a164_k4', 'kicad_native', None), ('a166_k0', 'test/a166_k0', 'native ¹', None), ('a159_k4', 'test/a159_k4', 'kicad_native', None),
          ('a163_k2', 'test/a163_k2', 'Model', None), ('a161_k3', 'test/a161_k3', 'Model', None), ('a160_k7', 'test/a160_k7', 'Model', None),
          ('d_1077', 'test/d_1077', 'Duration, model ²', []), ('d_0645', 'test/d_0645', 'Duration, model ²', [])]
SCATTER = ['a164_k0', 'a164_k1', 'a164_k3', 'a164_k4']


def gnd_layers(name):
    if name.startswith('a'):
        a, k = name[1:].split('_')
        return json.load(open(os.path.join(C, 'task_000%s' % a, k, 'routes.json'))).get('gnd_layers') or []
    return []


def load(d):
    E = json.load(open(os.path.join(S, d, 'k20.json')))
    F = json.load(open(os.path.join(S, d, 'k20_shape.json'))) if os.path.exists(os.path.join(S, d, 'k20_shape.json')) else None
    return E, F


def p(x, n=0): return ('%.' + str(n) + 'f %%') % (100 * x)
def f(x, n=1): return ('%.' + str(n) + 'f') % x


lines = []
def value_list(name, d, category, gl):
    E, F = load(d); J = E['per_layer']; G = E['total']; A = E['outer']; I = E['inner']
    gl = gl if gl is not None else gnd_layers(name)
    cu_layers = E['cu_layers']
    fl = lambda Ls: sum(J[L]['area_mm2'] for L in Ls)
    gw = lambda Ls, k: sum(J[L][k] * J[L]['area_mm2'] for L in Ls) / max(1e-9, fl(Ls))
    auss = [L for L in cu_layers if L in ('F', 'B')]; plane = [L for L in cu_layers if L in gl]; rest = [L for L in cu_layers if L not in auss and L not in gl]
    nL = len(cu_layers); flL = fl(cu_layers)
    inside = [J[L]['gnd_islands'] for L in cu_layers]
    v = {}
    v['Source'] = category
    v['Layers'] = str(nL)
    v['BoardArea'] = '%.1f cm²' % (E['area_mm2'] / 100)
    v['GND share all layers'] = p(G['gnd_share'])
    v['GND share F/B'] = ' / '.join(p(J[L]['gnd_share']) for L in auss)
    v['GND share GND planes'] = (' / '.join(p(J[L]['gnd_share']) for L in plane)) if plane else '–'
    v['GND share remaining inner layers'] = (' / '.join(p(J[L]['gnd_share']) for L in rest)) if rest else '–'
    v['Supply inner layers (share of area)'] = p(gw([L for L in cu_layers if L not in ('F', 'B')], 'supply_share'), 1)
    v['GND islands ≥ 1 mm² per layer (mean / max)'] = '%.1f / %d' % (sum(inside) / nL, max(inside))
    v['GND islands per layer and 10 cm²'] = f(sum(inside) / nL / (flL / nL / 1000), 1)
    v['GND splinters < 1 mm² with via/pad · without'] = '%d · %d' % (G['gnd_splitter_with'], G['gnd_splitter_without'])
    v['smallest GND island (share of GND area of the layer)'] = '%.3f %%' % (100 * G['gnd_smallest_island_share'])
    v['White K17 (spacing 0.15) total'] = '%.1f mm² (%d spots)' % (G['white15_mm2'], G['white15_n'])
    v['White K17 per 10 cm² layer area'] = '%.2f mm²' % (G['white15_mm2'] / flL * 1000)
    v['White K17 with own zone spacing'] = '%.1f mm² (%.2f per 10 cm²; spacing %.2f)' % (G['white_own_mm2'], G['white_own_mm2'] / flL * 1000, E['clr_owned'])
    v['Supply net×layer with zone'] = str(G['supply_net_layer'])
    v['Supply zone pieces (per net×layer)'] = '%d (%.1f)' % (G['supply_pieces'], G['supply_pieces'] / max(1, G['supply_net_layer'])) if G['supply_net_layer'] else '–'
    v['Net×layer single-piece (fill · with own copper)'] = ('%s · %s' % (p(G['supply_one_piece'] / G['supply_net_layer']), p(G['supply_one_piece_with_owned'] / G['supply_net_layer']))) if G['supply_net_layer'] else '–'
    v['Supply splinters < 0.5 mm²'] = str(G['supply_splitter']) if G['supply_net_layer'] else '–'
    if F:
        a = F['outer']; i = F['inner']
        v['Outer zones supply: pieces median / p90 / max'] = '%.1f / %.0f / %.0f mm²' % (a['median_mm2'], a['p90_mm2'], a['max_mm2'])
        v['Outer zones: share of F+B area'] = p(a['share_layer_area'], 1)
        v['Outer zones: width 2A/U median'] = '%.2f mm' % a['width_median']
        v['Outer zones: pads per piece (median) · pieces without pad'] = '%g · %d of %d' % (a['pads_median'], a['without_pad'], a['pieces'])
        v['Fill edge supply outer at 0/45/90°'] = p(a['edge45_supply'])
        v['Inner: supply areas > 50 mm² (largest)'] = '%d (%s)' % (sum(1 for g in i['largest'] if g[2] > 50) if i['pieces'] else 0, ', '.join('%s %s %.0f mm²' % (g[0].split('/')[-1], g[1], g[2]) for g in i['largest'][:2] if g[2] > 50) or '–')
    else:
        for k in ('Outer zones supply: pieces median / p90 / max', 'Outer zones: share of F+B area', 'Outer zones: width 2A/U median',
                  'Outer zones: pads per piece (median) · pieces without pad', 'Fill edge supply outer at 0/45/90°', 'Inner: supply areas > 50 mm² (largest)'): v[k] = '–'
    v['Supply as zone (outer · inner)'] = ('%s · %s' % (p(A['zone_share']), p(I['zone_share']) if I['zone_mm2'] + I['track_mm2'] > 0 else '–')) if A['zone_mm2'] + A['track_mm2'] > 0 else '– (no zones)'
    v['Hole share supply areas (outer · inner)'] = ('%s · %s' % (p(A['hole_share'], 1), p(I['hole_share'], 1))) if G['supply_net_layer'] else '–'
    v['sharp copper corners < 90° per 100 mm zone edge (all · supply)'] = '%.2f · %s' % (G['sharp_cu_per100'], f(G['supply_sharp_cu_per100'], 2) if G['supply_border_mm'] else '–')
    v['sharp white corners (> 270°) per 100 mm'] = '%.2f' % G['sharp_white_per100']
    v['Corners with radius ≥ 0.1 mm (all · supply)'] = '%s · %s' % (p(G['round_share']), p(G['supply_round_share']) if G['supply_corners'] else '–')
    R = E['r37']
    v['R37 track connections at via center'] = '%s (%d of %d; %d vias with connection at the ring)' % (p(R['share'], 1), R['middle'], R['connections'], R['vias_skewed'])
    return v


all_items = [(n, value_list(n, d, a, g)) for n, d, a, g in BOARDS]
keys = list(all_items[0][1].keys())
print('| Metric | ' + ' | '.join(n for n, _ in all_items) + ' |')
print('|---|' + '---|' * len(all_items))
for k in keys:
    print('| ' + k + ' | ' + ' | '.join(v.get(k, '') for _, v in all_items) + ' |')
print()
print('Spread of the 4 native candidates of task 164 (same board):')
st = [(n, value_list(n, 'test/' + n, 'kicad_native', None)) for n in SCATTER]
for k in ('GND share all layers', 'GND share F/B', 'White K17 (spacing 0.15) total', 'Outer zones supply: pieces median / p90 / max', 'Outer zones: share of F+B area',
          'sharp copper corners < 90° per 100 mm zone edge (all · supply)', 'Corners with radius ≥ 0.1 mm (all · supply)', 'Fill edge supply outer at 0/45/90°', 'R37 track connections at via center'):
    print('| ' + k + ' | ' + ' | '.join(v[k] for _, v in st) + ' |')
