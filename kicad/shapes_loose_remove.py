import pcbnew, sys, json
b = pcbnew.LoadBoard(sys.argv[1]); D = json.load(open(sys.argv[2]))
U = {i['uuid'] for u in D.get('unconnected_items', []) for i in u['items'] if i['description'].startswith('Polygon')}
way = [dr for dr in b.GetDrawings() if dr.m_Uuid.AsString() in U and dr.IsOnCopperLayer() and dr.GetNetname()]
for dr in way: b.Remove(dr)
n = len(way)
b.Save(sys.argv[3])
print('loose copper shapes removed:', n)
