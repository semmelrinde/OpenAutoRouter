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
before = areas(); t = time.time()
f = pcbnew.ZONE_FILLER(b); ok = f.Fill(b.Zones())
post = areas()
print('Fill ok', ok, '%.1f s' % (time.time() - t))
diff = {k: (round(before[k], 3), round(post[k], 3)) for k in before if abs(before[k] - post[k]) > 0.01}
print('Zones x layer with area change > 0.01 mm2:', len(diff), 'from_', len(before))
for k, v in sorted(diff.items(), key=lambda x: -abs(x[1][0] - x[1][1]))[:15]: print('  ', k, v)
print('Total before %.1f after %.1f mm2' % (sum(before.values()), sum(post.values())))
json.dump(dict(before=before, post=post), open(dst + '.fill.json', 'w'))
b.Save(dst)
