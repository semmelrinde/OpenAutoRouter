from learning.selection import board_off_placement
from learning.features import feats
from learning.value_model import Service, expectation, prediction


class CoarseValue:
    def __init__(self, B, K, RG, part_list, args):
        self.B, self.K, self.RG, self.part_list = B, K, RG, part_list
        self.all_items = max(1, int(args.get('value_all', '250')))
        self.weight = float(args.get('value_weight', '1'))
        if args.get('value_pilot', '0') != '1':
            raise ValueError('Coarse-move steering without its own release; only value_pilot=1')
        from learning.value_model import finite, profile_verify
        self.budget = dict(cpu_s=finite(args.get('value_cpu')), iter=finite(args.get('value_iter')))
        if any(v is None or v <= 0 for v in self.budget.values()):
            raise ValueError('Coarse-move pilot needs target router budget value_cpu and value_iter > 0')
        self.profile_data = args.get('value_profile', 'balanced')
        d = Service([], args['value_model'])
        self.M, meta = d.load_data(pilot=True)
        if self.M is None: raise ValueError('no value model present')
        profile_verify(self.M, [dict(profile_data=self.profile_data)])
        self.logbook = dict(id=self.M['id'], application='coarse_pilot', profile_data=self.profile_data, budget=self.budget,
                             all_items=self.all_items, weight=self.weight, calls=0)
        gnd = args.get('gnd_layers')
        self.gnd = gnd.split(',') if gnd is not None else None
        from learning.value_model import CFG
        self.cfg = {k: args[k] for k in CFG if k in args}
        self.cfg['style'] = args.get('value_style', '45')
        self.logbook.update(cfg=self.cfg, gnd_layers=self.gnd)

    def delta(self, prior, new):
        Z = []
        for layer_name in (prior, new):
            parts = {r: dict(x=s[0], y=s[1], side=s[3], pads={nr: [x, y] for nr, _, x, y, _ in self.part_list[r].pads_abs(*s)})
                     for r, s in layer_name.items()}
            B = board_off_placement(self.B, dict(parts=parts))
            Z.append(dict(feats=feats(B, self.K, self.RG, self.gnd), cfg=self.cfg,
                          budget=self.budget, profile_data=self.profile_data))
        p = prediction(self.M, Z); self.logbook['calls'] += 1
        return self.weight * (expectation(p[1]) - expectation(p[0]))
