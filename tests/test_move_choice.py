import json, os, sys, tempfile, unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from learning import move_choice as Z


def move(ok, R=2.0, series='core_set', rep=False, type_='conflict', core_set=1, pads2=5, cpu=1.0):
    return dict(type_=type_, R=R, series=series, repair=rep, core_set=core_set, pads2=pads2, cpu_s=cpu,
                outcome='taken' if ok else 'rejected')


class MoveChoice(unittest.TestCase):
    def test_rate_and_value(self):
        lines = [move(True)] * 6 + [move(False)] * 2 + [move(False, R=5.0, cpu=4.0)] * 8
        tab = Z.learning(lines, min_n=5)
        layer_name = tab['conflict|k1|p0']
        self.assertEqual(layer_name['2.0|core_set|0|0']['n'], 8)
        self.assertAlmostEqual(layer_name['2.0|core_set|0|0']['p'], 0.7)
        self.assertGreater(layer_name['2.0|core_set|0|0']['amount'], layer_name['5.0|core_set|0|0']['amount'])
        self.assertIn('conflict', tab)

    def test_too_little_data(self):
        self.assertEqual(Z.learning([move(True)] * 3, min_n=5), {})

    def test_collect_without_double(self):
        with tempfile.TemporaryDirectory() as d:
            os.makedirs(os.path.join(d, 'a'))
            with open(os.path.join(d, 'a', 'r_moves.jsonl'), 'w') as f:
                for z in (move(True), move(False)): f.write(json.dumps(z) + '\n')
            goal = os.path.join(d, 'all_items.jsonl')
            Z.main(['collect', d, goal]); Z.main(['collect', d, goal])
            self.assertEqual(len(open(goal).read().splitlines()), 2)
            Z.main([goal, os.path.join(d, 'm', 'move_choice.json'), 'min_n=1'])
            self.assertEqual(json.load(open(os.path.join(d, 'm', 'move_choice.json')))['taken'], 1)


if __name__ == '__main__':
    unittest.main()
