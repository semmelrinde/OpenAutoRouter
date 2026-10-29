import os, sys, unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common.dru_condition import min_width, pair_basis, fits

LAYERS = ['F.Cu', 'In1.Cu', 'B.Cu']
NETS = {'+3V3', 'VREF', 'SIG', 'USB_TEST_P', 'USB_TEST_N', '/USB_IN_P', '/USB_IN_N', 'D+', 'D-'}
V11 = [
    dict(name='Power F', cond="A.NetClass == 'Power' && A.Layer == 'F.Cu'", constraints=dict(track_width=dict(min=0.40))),
    dict(name='Analog', cond="A.NetClass == 'Analog'", constraints=dict(track_width=dict(min=0.14))),
    dict(name='USB F', cond="A.NetName == 'USB_TEST_*' && A.Layer == 'F.Cu'", constraints=dict(track_width=dict(min=0.13, max=0.20))),
    dict(name='Spacing', cond="(A.NetClass == 'Power' && B.NetClass == 'Analog')", constraints=dict(clearance=dict(min=0.30))),
]


class Condition(unittest.TestCase):
    def test_ternary(self):
        c = "A.NetClass == 'Power' && A.Layer == 'F.Cu'"
        self.assertIsNone(fits(c, '+3V3', 'Power', None, NETS))
        self.assertTrue(fits(c, '+3V3', 'Power', 'F.Cu', NETS))
        self.assertFalse(fits(c, '+3V3', 'Power', 'B.Cu', NETS))
        self.assertFalse(fits(c, 'SIG', 'Default', None, NETS))
        self.assertIsNone(fits("A.NetClass == 'Power' && B.NetClass == 'Analog'", '+3V3', 'Power', 'F.Cu', NETS))
        self.assertIsNone(fits("A.foo(1)", 'SIG', 'Default', 'F.Cu', NETS))
        self.assertTrue(fits('', 'SIG', 'Default'))
        self.assertTrue(fits("!(A.NetClass == 'USB') || A.NetName == 'X'", 'SIG', 'Default'))

    def test_pairs(self):
        self.assertEqual(pair_basis('/USB_IN_P', NETS), '/USB_IN')
        self.assertEqual(pair_basis('D-', NETS), 'D')
        self.assertIsNone(pair_basis('SIG', NETS))
        self.assertTrue(fits("A.inDiffPair('/USB_*')", '/USB_IN_N', 'Default', None, NETS))
        self.assertFalse(fits("A.inDiffPair('/USB_*')", 'SIG', 'Default', None, NETS))

    def test_min_width_v11(self):
        m, n = min_width(V11, '+3V3', 'Power', LAYERS, NETS, 0.1)
        self.assertEqual(m, {'F.Cu': 0.40, 'In1.Cu': 0.1, 'B.Cu': 0.1}); self.assertEqual(n, ['Power F'])
        self.assertEqual(min_width(V11, 'VREF', 'Analog', LAYERS, NETS, 0.1)[0]['B.Cu'], 0.14)
        self.assertEqual(min_width(V11, 'USB_TEST_P', 'USB', LAYERS, NETS, 0.1)[0], {'F.Cu': 0.13, 'In1.Cu': 0.1, 'B.Cu': 0.1})
        self.assertEqual(min_width(V11, 'SIG', 'Default', LAYERS, NETS, 0.1), ({'F.Cu': 0.1, 'In1.Cu': 0.1, 'B.Cu': 0.1}, []))

    def test_not_evaluable_cautious(self):
        r = [dict(name='x', cond="A.Unknown == 'y'", constraints=dict(track_width=dict(min=0.3)))]
        self.assertEqual(min_width(r, 'SIG', 'Default', ['F.Cu'], NETS, 0.1)[0], {'F.Cu': 0.3})


if __name__ == '__main__':
    unittest.main()
