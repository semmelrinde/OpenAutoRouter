import math, sys, json

WHOLE = math.inf



TABLE = {


    'chip':                 (0.0, 2.0, 100.0, False, 'P22 main part: moves only a little, expensive (user 06.10. 19:45; price from Codex P3)'),
    'connector':              (0.0, 0.0, 0.0, False, 'P22/P16 mechanics, fixed'),
    'mechanics':             (0.0, 0.0, 0.0, False, 'P22/P16 mechanics, fixed'),

    'loose':                 (0.0, WHOLE, 0.5, True, 'loose in generated task (netclasses.loose_connector), place freely'),
    'crystal':                (0.0, 0.0, 0.0, False, 'P20(3) crystal stays at its chip'),

    ('decoupling_small', 'power'): (0.3, 0.6, 80.0, True, 'P8/P20(2) hot loop of the switching regulator'),
    ('decoupling_small', 'digital'):  (0.5, 1.0, 50.0, True, 'P19/P20(2) 100 nF at the fast chip'),
    ('decoupling_small', 'mixed'): (0.5, 1.0, 50.0, True, 'P19/P20(2) 100 nF at the fast chip'),
    ('decoupling_small', 'analog'):   (1.5, 2.5, 15.0, True, 'P19 user 06.10.: OPV supply C 1-2 mm, more only if routing is much better'),
    'decoupling_small':               (1.0, 2.0, 25.0, True, 'P19/P20(2) small decoupling, medium-speed chip'),
    ('decoupling_medium', 'power'): (0.5, 1.0, 40.0, True, 'P8 input/output C of the switching regulator'),
    'decoupling_medium':              (2.0, 3.0, 8.0, True, 'P20(5) medium C (~1 uF)'),
    ('decoupling_large', 'power'): (0.5, 1.5, 25.0, True, 'P8/P14 input/output C of the switching regulator (loop)'),
    'decoupling_large':               (3.0, 5.0, 4.0, True, 'P19/P20(6) large C (10 uF+) may move'),
    'bootstrap':            (0.3, 0.5, 60.0, True, 'P8 bootstrap C at switching node, loop small'),
    'coil_sw':             (0.3, 0.5, 60.0, False, 'P8/P20 inductor at switching node; do not rotate (winding start)'),
    'crystal_load':           (0.3, 0.5, 60.0, True, 'P20(3) load capacitors, tracks extremely short (ST AN2867)'),
    'series_r':             (0.5, 1.0, 30.0, True, 'P20(4) series resistor on fast line'),

    'feedback':        (1.0, 2.0, 10.0, True, 'FB node sensitive, divider near the pin'),
    'feedback_c':      (1.0, 2.0, 10.0, True, 'FB/compensation C near the pin'),
    'setting_r':           (1.0, 2.0, 10.0, True, 'RT/ISET node sensitive'),
    'shunt':                (1.0, 2.0, 10.0, True, 'K6/R13 sense resistor in the current path (IR protection also checks)'),

    'divider':               (3.0, WHOLE, 1.0, True, 'P20 slow divider, may move'),
    'rc_element':             (3.0, WHOLE, 1.0, True, 'P20 "22 nF in the RC element may not matter"'),
    'misc':              (2.0, WHOLE, 2.0, True, 'unknown function: medium range, moderate price'),
    'pullup':               (WHOLE, WHOLE, 0.0, True, 'P20(7)/P19 user 06.10.: pull-up anywhere, as long as there is room'),
    'pulldown':             (WHOLE, WHOLE, 0.0, True, 'P20(7)/P19 user 06.10.: pull-down anywhere, as long as there is room'),
}
WITHOUT_FUNCTION = (2.0, WHOLE, 2.0, True, 'netclasses.json without function: like misc')


def _island_kind(t, islands):
    s = t.get('supports') or {}
    I = islands.get(s.get('ref')) or islands.get(t.get('island')) or {}
    return I.get('category')


def width(t, islands):
    f = t.get('function') or t.get('role')
    if t.get('pinned') or t.get('locked') or t.get('role') != 'small':
        key = t.get('role') if t.get('role') in TABLE and not t.get('pinned') and not t.get('locked') else 'pinned'
        if t.get('loose') and not t.get('pinned') and not t.get('locked'): key = 'loose'
        g = TABLE.get(key) or (0.0, 0.0, 0.0, False, 'fixed (specs.json, connector, control element or not a small part; P22)')
    else:
        a = _island_kind(t, islands)
        key = (f, a) if (f, a) in TABLE else f
        g = TABLE.get(key) or WITHOUT_FUNCTION
        if key not in TABLE: key = 'without_function'
    reason, mx, w, rotate, rule = g
    return dict(reason=reason, max=mx, importance=w, rotate=rotate, keyname=key if isinstance(key, str) else '%s@%s' % key, rule=rule)


def table(K):
    I = K.get('islands') or {}
    return {r: width(t, I) for r, t in sorted(K['part_list'].items())}


def over(w, way):
    return max(0.0, way - w['reason']) if w['reason'] != WHOLE else 0.0


def price(w, way_prev, way_new):
    return w['importance'] * (over(w, way_new) - over(w, way_prev))


def in_frame(w, way_new, eps=1e-6):
    return w['max'] > 0 and way_new <= w['max'] + eps


def movable(w):
    return w['max'] > 0


if __name__ == '__main__':
    K = json.load(open(sys.argv[1]))
    for r, w in table(K).items():
        f = lambda x: 'whole' if x == WHOLE else '%.1f' % x
        print('%-10s %-30s ground %-5s max %-5s %5.1f/mm %s  %s' % (r, w['keyname'], f(w['reason']), f(w['max']), w['importance'],
                                                                 '' if w['rotate'] else 'do not rotate', w['rule']))
