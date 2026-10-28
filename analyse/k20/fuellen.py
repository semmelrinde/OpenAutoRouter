""
import pcbnew, sys, json, time
src, dst = sys.argv[1:3]
b = pcbnew.LoadBoard(src)
CU = list(b.GetEnabledLayers().CuStack())
def areas():
    out = {}
    for i, z in enumerate(b.Zones()):
        if z.GetIsRuleArea(): continue
        for l in CU:
            if z.IsOnLayer(l):
                out['%d|%s|%s' % (i, z.GetNetname(), pcbnew.BOARD.GetStandardLayerName(l))] = z.GetFilledPolysList(l).Area() / 1e12
    return out
vor = areas(); t = time.time()
f = pcbnew.ZONE_FILLER(b); ok = f.Fill(b.Zones())
nach = areas()
print('Fuellen ok', ok, '%.1f s' % (time.time() - t))
diff = {k: (round(vor[k], 3), round(nach[k], 3)) for k in vor if abs(vor[k] - nach[k]) > 0.01}
print('Zonen x Lage mit Flaechenaenderung > 0,01 mm2:', len(diff), 'von', len(vor))
for k, v in sorted(diff.items(), key=lambda x: -abs(x[1][0] - x[1][1]))[:15]: print('  ', k, v)
print('Summe vorher %.1f nachher %.1f mm2' % (sum(vor.values()), sum(nach.values())))
json.dump(dict(vor=vor, nach=nach), open(dst + '.fuellung.json', 'w'))
b.Save(dst)
