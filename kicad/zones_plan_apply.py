import pcbnew, sys, json, uuid
b = pcbnew.LoadBoard(sys.argv[1]); MM = pcbnew.FromMM; P = json.load(open(sys.argv[3]))
byu = {z.m_Uuid.AsString(): z for z in b.Zones()}
for u in P['keepouts_way'] + P.get('zones_way', []):
    if u in byu: b.Remove(byu[u])
def contour(z, p):
    ol = z.Outline(); ol.RemoveAllContours(); ol.NewOutline()
    for x, y in p['outer']: ol.Append(MM(x), MM(y))
    for h in p.get('hole_list', []):
        hi = ol.NewHole()
        for x, y in h: ol.Append(MM(x), MM(y), -1, hi)
for u, polys in P['zone_list'].items():
    z = byu[u]
    for i, p in enumerate(polys):
        if i == 0: contour(z, p); continue
        k = pcbnew.ZONE(z); k.SetUuid(pcbnew.KIID(str(uuid.uuid5(uuid.NAMESPACE_URL, '%s:contour%d' % (u, i))))); contour(k, p); b.Add(k)
LAY = {'F': pcbnew.F_Cu, 'B': pcbnew.B_Cu, 'In1': pcbnew.In1_Cu, 'In2': pcbnew.In2_Cu}
for nz in P.get('new', []):
    z = pcbnew.ZONE(b); z.SetLayer(LAY[nz['layer_name']]); z.SetNet(b.FindNet(nz['net'])); z.SetAssignedPriority(nz['prio'])
    z.SetMinThickness(MM(0.15)); z.SetLocalClearance(MM(0.15)); z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL); z.SetThermalReliefGap(MM(0.2)); z.SetThermalReliefSpokeWidth(MM(0.3))
    for i, p in enumerate(nz['polys']):
        k = z if i == 0 else pcbnew.ZONE(z)
        if i: k.SetUuid(pcbnew.KIID(str(uuid.uuid5(uuid.NAMESPACE_URL, '%s:contour%d' % (z.m_Uuid.AsString(), i)))))
        contour(k, p); b.Add(k)
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(sys.argv[2]); print('Plan applied: zones', len(P['zone_list']), 'Keepouts removed', len(P['keepouts_way']), 'new', len(P.get('new', [])))
