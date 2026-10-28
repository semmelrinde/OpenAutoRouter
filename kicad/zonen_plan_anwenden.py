""
import pcbnew, sys, json
b = pcbnew.LoadBoard(sys.argv[1]); MM = pcbnew.FromMM; P = json.load(open(sys.argv[3]))
byu = {z.m_Uuid.AsString(): z for z in b.Zones()}
for u in P['sperren_weg'] + P.get('zonen_weg', []):
    if u in byu: b.Remove(byu[u])
for u, polys in P['zonen'].items():
    z = byu[u]; ol = z.Outline(); ol.RemoveAllContours()
    for p in polys:
        ol.NewOutline()
        for x, y in p['aussen']: ol.Append(MM(x), MM(y))
        for h in p['loecher']:
            hi = ol.NewHole()
            for x, y in h: ol.Append(MM(x), MM(y), -1, hi)
LAY = {'F': pcbnew.F_Cu, 'B': pcbnew.B_Cu, 'In1': pcbnew.In1_Cu, 'In2': pcbnew.In2_Cu}
for nz in P.get('neu', []):
    z = pcbnew.ZONE(b); z.SetLayer(LAY[nz['lage']]); z.SetNet(b.FindNet(nz['net'])); z.SetAssignedPriority(nz['prio'])
    ol = z.Outline()
    for p in nz['polys']:
        ol.NewOutline()
        for x, y in p['aussen']: ol.Append(MM(x), MM(y))
    z.SetMinThickness(MM(0.15)); z.SetLocalClearance(MM(0.15)); z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL); z.SetThermalReliefGap(MM(0.2)); z.SetThermalReliefSpokeWidth(MM(0.3))
    b.Add(z)
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(sys.argv[2]); print('Plan angewendet: Zonen', len(P['zonen']), 'Sperren weg', len(P['sperren_weg']), 'neu', len(P.get('neu', [])))
