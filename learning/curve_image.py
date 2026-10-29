import json
import math
from pathlib import Path
import sys
from datetime import datetime
from PIL import Image, ImageDraw, ImageFont

R = Path(__file__).resolve().parents[1]
W, H = 1200, 840
COLORS = ('#c0392b', '#2471a3', '#1e8449', '#7d3c98', '#b9770e')


def read_in(path):
    path = Path(path)
    return [json.loads(l) for l in path.read_text().splitlines() if l.strip()] if path.exists() else []


def series(z):
    return json.dumps([contract_spec(z),z.get('options', {})], sort_keys=True)



def contract_spec(z):
    return tuple(json.dumps(z.get(k), sort_keys=True) for k in
                 ('generator_version', 'tasks_sha', 'evaluation_target', 'measurement_contract'))


def connectable(a, b):

    for z in (a, b):
        c = z.get('measurement_contract') or {}
        if c.get('budget') == 'cpu' and not (c.get('time_s') and c.get('cpu_scaling')):
            return False
    return contract_spec(a) == contract_spec(b)

def draw(generated, real, goal):
    in_ = Image.new('RGB', (W, H), 'white')
    d = ImageDraw.Draw(in_)
    try:
        font = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf', 14)
    except OSError:
        font = ImageFont.load_default()
    def text(xy, s, fill='black'):
        d.text(xy, s, fill=fill, font=font)
    for col, (title, rows) in enumerate((('Generated tasks', generated), ('Real boards · native DRC', real))):
        left, right = col*600+65, col*600+575
        text((left, 16), title)
        if not rows:
            text((left, 72), 'No measurement yet')
            continue
        rows = sorted(rows, key=lambda z: z['time_s'])
        times = [datetime.strptime(z['time_s'], '%Y-%m-%d %H:%M').timestamp() for z in rows]
        x = lambda t: left+(t-min(times))/max(60, max(times)-min(times))*(right-left)
        groups = {}
        for z, t in zip(rows, times):
            groups.setdefault(series(z), []).append((z, t))
        n = max(z['tasks'] if isinstance(z.get('tasks'), int) else len(z.get('tasks') or []) for z in rows) or 1
        y = lambda v: 310-v/n*240
        for k in sorted({0, n//2, n}):
            d.line([(left, y(k)), (right, y(k))], fill='#dddddd')
            text((left-35, y(k)-8), str(k))
        load = rows[-1]
        text((left, 43), 'Valid: %s/%s · unknown: %s' % (load.get('is_valid'), load['tasks'], load.get('unknown', 0)))
        cv = [z.get('cpu_per_valid') for z in rows if z.get('cpu_per_valid') and z['cpu_per_valid'] > 0]
        if col == 1:
            text((left, 355), 'Native residual errors: open pairs (blue), DRC errors (red)')
            rest = [(z, t, [r.get('open_pairs') for r in z.get('results', [])],
                     [r.get('drc_error') for r in z.get('results', [])]) for z, t in zip(rows, times)]
            valid_rest = [(z, t, sum(op), sum(dr)) for z, t, op, dr in rest
                          if op and dr and all(v is not None for v in op+dr)]
            if valid_rest:
                maximum = max(1, max(max(op, dr) for z, t, op, dr in valid_rest))
                yr = lambda v: 550-v/maximum*150
                for v in (0, maximum):
                    d.line([(left, yr(v)), (right, yr(v))], fill='#dddddd')
                    text((left-45, yr(v)-8), str(v))
                for index, color in ((2, '#2471a3'), (3, '#c0392b')):
                    previous = {}
                    for item in valid_rest:
                        z, t = item[:2]
                        pt = (x(t), yr(item[index]))
                        key = series(z)
                        if key in previous:
                            d.line([previous[key], pt], fill=color, width=2)
                        d.ellipse([pt[0]-3, pt[1]-3, pt[0]+3, pt[1]+3], fill=color)
                        previous[key] = pt
                text((left, 570), 'Latest %d open pairs / %d DRC errors (total)' % valid_rest[-1][2:])
            else:
                text((left, 420), 'Native residual errors still unknown')
        else:
            text((left, 400), 'Validity remains the quality number.')
            text((left, 430), 'Different inputs/options shaping separate series.')
        text((left, 625), 'CPU-s per valid board · secondary number (log)')
        if cv:
            lo, hi = math.log10(min(cv)), math.log10(max(cv))
            yc = lambda v: 750-(math.log10(v)-lo)/max(1, hi-lo)*90
            for v in sorted({min(cv), max(cv)}):
                d.line([(left, yc(v)), (right, yc(v))], fill='#dddddd')
                text((left-55, yc(v)-8), '%.0f' % v)
        else:
            text((left, 680), 'No valid CPU reference values')
        for i, data in enumerate(groups.values()):
            color = COLORS[i % len(COLORS)]
            prev = prev_cpu = None
            for z, t in data:
                pt = (x(t), y(z.get('is_valid') or 0))
                if prev and connectable(prev[1],z):
                    d.line([prev[0], (pt[0], prev[0][1]), pt], fill=color, width=2)
                d.ellipse([pt[0]-3, pt[1]-3, pt[0]+3, pt[1]+3], fill=color)
                prev = (pt,z)
                c = z.get('cpu_per_valid')
                if c and c > 0:
                    cp = (x(t), yc(c))
                    if prev_cpu and connectable(prev_cpu[1],z):
                        d.line([prev_cpu[0], cp], fill=color, width=2)
                    d.ellipse([cp[0]-3, cp[1]-3, cp[0]+3, cp[1]+3], fill=color)
                    prev_cpu = (cp,z)
                else:
                    prev_cpu = None
        text((left, 785), rows[0]['time_s']+' → '+rows[-1]['time_s'])
        text((left, 812), '%d separate input/option series' % len(groups))
    goal = Path(goal).expanduser()
    goal.parent.mkdir(parents=True, exist_ok=True)
    in_.save(goal)
    return goal


def main():
    args = sys.argv[1:]
    kw = dict(a.split('=', 1) for a in args if '=' in a)
    goal = next((a for a in args if '=' not in a), '~/Library/Caches/Autorouter/reports/learning_curve.png')
    print(draw(read_in(kw.get('data', R/'learning/data/learning_curve.jsonl')),
                   read_in(kw.get('real', R/'learning/data/real_curve.jsonl')), goal))


if __name__ == '__main__':
    main()
