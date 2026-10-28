""
import json, os, sys, tempfile, unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from lernen import zugwahl as Z


def zug(ok, R=2.0, reihe='kern', rep=False, typ='konflikt', kern=1, pads2=5, cpu=1.0):
    return dict(typ=typ, R=R, reihe=reihe, reparatur=rep, kern=kern, pads2=pads2, cpu_s=cpu,
                ergebnis='angenommen' if ok else 'verworfen')


class Zugwahl(unittest.TestCase):
    def test_rate_und_wert(self):
        zeilen = [zug(True)] * 6 + [zug(False)] * 2 + [zug(False, R=5.0, cpu=4.0)] * 8
        tab = Z.lernen(zeilen, min_n=5)
        lage = tab['konflikt|k1|p0']
        self.assertEqual(lage['2.0|kern|0|0']['n'], 8)
        self.assertAlmostEqual(lage['2.0|kern|0|0']['p'], 0.7)
        self.assertGreater(lage['2.0|kern|0|0']['wert'], lage['5.0|kern|0|0']['wert'])
        self.assertIn('konflikt', tab)

    def test_zu_wenig_daten(self):
        self.assertEqual(Z.lernen([zug(True)] * 3, min_n=5), {})

    def test_sammeln_ohne_doppelte(self):
        with tempfile.TemporaryDirectory() as d:
            os.makedirs(os.path.join(d, 'a'))
            with open(os.path.join(d, 'a', 'r_zuege.jsonl'), 'w') as f:
                for z in (zug(True), zug(False)): f.write(json.dumps(z) + '\n')
            ziel = os.path.join(d, 'alle.jsonl')
            Z.main(['sammeln', d, ziel]); Z.main(['sammeln', d, ziel])
            self.assertEqual(len(open(ziel).read().splitlines()), 2)
            Z.main([ziel, os.path.join(d, 'm', 'zugwahl.json'), 'min_n=1'])
            self.assertEqual(json.load(open(os.path.join(d, 'm', 'zugwahl.json')))['angenommen'], 1)


if __name__ == '__main__':
    unittest.main()
