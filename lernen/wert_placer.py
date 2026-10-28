""
from lernen.auswahl import board_aus_placement
from lernen.merkmale import merkmale
from lernen.wertmodell import Dienst, erwartung, vorhersage


class Grobwert:
    def __init__(self, B, K, RG, teile, args):
        self.B, self.K, self.RG, self.teile = B, K, RG, teile
        self.alle = max(1, int(args.get('wert_alle', '250')))
        self.gewicht = float(args.get('wert_gewicht', '1'))
        if args.get('wert_pilot', '0') != '1':
            raise ValueError('Grobzuglenkung ohne eigene Freigabe; nur wert_pilot=1')
        from lernen.wertmodell import endlich, profil_pruefen
        self.budget = dict(cpu_s=endlich(args.get('wert_cpu')), iter=endlich(args.get('wert_iter')))
        if any(v is None or v <= 0 for v in self.budget.values()):
            raise ValueError('Grobzug-Pilot braucht Ziel-Routerbudget wert_cpu und wert_iter > 0')
        self.profil = args.get('wert_profil', 'ausgewogen')
        d = Dienst([], args['wert_modell'])
        self.M, meta = d.laden(pilot=True)
        if self.M is None: raise ValueError('kein Wertmodell vorhanden')
        profil_pruefen(self.M, [dict(profil=self.profil)])
        self.protokoll = dict(id=self.M['id'], anwendung='grob_pilot', profil=self.profil, budget=self.budget,
                             alle=self.alle, gewicht=self.gewicht, aufrufe=0)
        gnd = args.get('gndlagen')
        self.gnd = gnd.split(',') if gnd is not None else None
        from lernen.wertmodell import CFG
        self.cfg = {k: args[k] for k in CFG if k in args}
        self.cfg['stil'] = args.get('wert_stil', '45')
        self.protokoll.update(cfg=self.cfg, gndlagen=self.gnd)

    def delta(self, alt, neu):
        Z = []
        for lage in (alt, neu):
            parts = {r: dict(x=s[0], y=s[1], side=s[3], pads={nr: [x, y] for nr, _, x, y, _ in self.teile[r].pads_abs(*s)})
                     for r, s in lage.items()}
            B = board_aus_placement(self.B, dict(parts=parts))
            Z.append(dict(merkmale=merkmale(B, self.K, self.RG, self.gnd), cfg=self.cfg,
                          budget=self.budget, profil=self.profil))
        p = vorhersage(self.M, Z); self.protokoll['aufrufe'] += 1
        return self.gewicht * (erwartung(p[1]) - erwartung(p[0]))
