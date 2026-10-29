import pcbnew, sys
src, out = sys.argv[1:3]; opt = dict(a.split('=', 1) for a in sys.argv[3:] if '=' in a)
b = pcbnew.LoadBoard(src)
way = 0
if opt.get('splitter_way') == '1':
    for z in list(b.Zones()):
        if z.GetIsRuleArea() and z.GetDoNotAllowZoneFills() and not z.GetDoNotAllowTracks() and not z.GetDoNotAllowVias() \
           and z.Outline().Area() / 1e12 < 20.0:
            b.Remove(z); way += 1
mode = pcbnew.ISLAND_REMOVAL_MODE_NEVER if opt['mode'] == 'keep' else pcbnew.ISLAND_REMOVAL_MODE_ALWAYS
n = 0
for z in b.Zones():
    if not z.GetIsRuleArea() and z.GetNetname() == 'GND':
        z.SetIslandRemovalMode(mode); n += 1
        if 'min_width' in opt: z.SetMinThickness(pcbnew.FromMM(float(opt['min_width'])))
if opt.get('min_width_all'):
    for z in b.Zones():
        if not z.GetIsRuleArea() and z.GetNetname() != 'GND': z.SetMinThickness(pcbnew.FromMM(float(opt['min_width_all'])))
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(out); print('Sliver keepout areas removed:', way, '| GND zones on', opt['mode'], ':', n)
