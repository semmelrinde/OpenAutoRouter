""
import os, sys, unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from gemeinsam.dru_bedingung import mindestbreite, paar_basis, passt

LAGEN = ['F.Cu', 'In1.Cu', 'B.Cu']
NETZE = {'+3V3', 'VREF', 'SIG', 'USB_TEST_P', 'USB_TEST_N', '/USB_IN_P', '/USB_IN_N', 'D+', 'D-'}
V11 = [
    dict(name='Power F', cond="A.NetClass == 'Power' && A.Layer == 'F.Cu'", constraints=dict(track_width=dict(min=0.40))),
    dict(name='Analog', cond="A.NetClass == 'Analog'", constraints=dict(track_width=dict(min=0.14))),
    dict(name='USB F', cond="A.NetName == 'USB_TEST_*' && A.Layer == 'F.Cu'", constraints=dict(track_width=dict(min=0.13, max=0.20))),
    dict(name='Abstand', cond="(A.NetClass == 'Power' && B.NetClass == 'Analog')", constraints=dict(clearance=dict(min=0.30))),
]


class Bedingung(unittest.TestCase):
    def test_dreiwertig(self):
        c = "A.NetClass == 'Power' && A.Layer == 'F.Cu'"
        self.assertIsNone(passt(c, '+3V3', 'Power', None, NETZE))
        self.assertTrue(passt(c, '+3V3', 'Power', 'F.Cu', NETZE))
        self.assertFalse(passt(c, '+3V3', 'Power', 'B.Cu', NETZE))
        self.assertFalse(passt(c, 'SIG', 'Default', None, NETZE))
        self.assertIsNone(passt("A.NetClass == 'Power' && B.NetClass == 'Analog'", '+3V3', 'Power', 'F.Cu', NETZE))
        self.assertIsNone(passt("A.foo(1)", 'SIG', 'Default', 'F.Cu', NETZE))
        self.assertTrue(passt('', 'SIG', 'Default'))
        self.assertTrue(passt("!(A.NetClass == 'USB') || A.NetName == 'X'", 'SIG', 'Default'))

    def test_paare(self):
        self.assertEqual(paar_basis('/USB_IN_P', NETZE), '/USB_IN')
        self.assertEqual(paar_basis('D-', NETZE), 'D')
        self.assertIsNone(paar_basis('SIG', NETZE))
        self.assertTrue(passt("A.inDiffPair('/USB_*')", '/USB_IN_N', 'Default', None, NETZE))
        self.assertFalse(passt("A.inDiffPair('/USB_*')", 'SIG', 'Default', None, NETZE))

    def test_mindestbreite_v11(self):
        m, n = mindestbreite(V11, '+3V3', 'Power', LAGEN, NETZE, 0.1)
        self.assertEqual(m, {'F.Cu': 0.40, 'In1.Cu': 0.1, 'B.Cu': 0.1}); self.assertEqual(n, ['Power F'])
        self.assertEqual(mindestbreite(V11, 'VREF', 'Analog', LAGEN, NETZE, 0.1)[0]['B.Cu'], 0.14)
        self.assertEqual(mindestbreite(V11, 'USB_TEST_P', 'USB', LAGEN, NETZE, 0.1)[0], {'F.Cu': 0.13, 'In1.Cu': 0.1, 'B.Cu': 0.1})
        self.assertEqual(mindestbreite(V11, 'SIG', 'Default', LAGEN, NETZE, 0.1), ({'F.Cu': 0.1, 'In1.Cu': 0.1, 'B.Cu': 0.1}, []))

    def test_nicht_auswertbar_vorsichtig(self):
        r = [dict(name='x', cond="A.Unbekannt == 'y'", constraints=dict(track_width=dict(min=0.3)))]
        self.assertEqual(mindestbreite(r, 'SIG', 'Default', ['F.Cu'], NETZE, 0.1)[0], {'F.Cu': 0.3})


if __name__ == '__main__':
    unittest.main()
