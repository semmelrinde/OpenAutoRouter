""
import collections, glob, hashlib, json, os, sys

VERSION = 'zugwahl-v1'


def strat_key(z):
    return '%s|%s|%s|%d' % (z['R'], z['reihe'], int(bool(z.get('reparatur'))), int(bool(z.get('warm'))))


def lage_key(z):
    ""
    k = z.get('kern', 1); p = z.get('pads2', 0)
    return '%s|k%d|p%d' % (z['typ'], min(k, 3), 0 if p <= 8 else 1 if p <= 20 else 2)


def lernen(zeilen, min_n=5):
    ""
    st = collections.defaultdict(lambda: [0, 0, 0.0])
    for z in zeilen:
        if z.get('ergebnis') not in ('angenommen', 'verworfen'): continue
        for lk in (lage_key(z), z['typ']):
            s = st[(lk, strat_key(z))]; s[0] += 1; s[1] += z['ergebnis'] == 'angenommen'; s[2] += float(z.get('cpu_s') or 0)
    tab = {}
    for (lk, sk), (n, ok, cpu) in sorted(st.items()):
        if n < min_n: continue
        p = (ok + 1) / (n + 2); c = max(cpu / n, 0.05)
        tab.setdefault(lk, {})[sk] = dict(n=n, ok=ok, p=round(p, 4), cpu=round(c, 3), wert=round(p / c, 6))
    return tab


def main(argv):
    if argv and argv[0] == 'sammeln':
        ordner, ziel = argv[1], argv[2]; n = 0
        alt = set()
        if os.path.exists(ziel):
            alt = {json.loads(l).get('_quelle_zeile') for l in open(ziel)}
        with open(ziel, 'a') as out:
            for f in sorted(glob.glob(os.path.join(ordner, '**', '*_zuege.jsonl'), recursive=True)):
                for i, l in enumerate(open(f)):
                    key = hashlib.sha256(('%s:%d:%s' % (f, i, l)).encode()).hexdigest()[:16]
                    if key in alt: continue
                    z = json.loads(l); z['_quelle_zeile'] = key; out.write(json.dumps(z, sort_keys=True) + '\n'); n += 1
        print('gesammelt', n, 'neue Zuege nach', ziel); return
    quelle, ziel = argv[0], argv[1]
    kw = dict(a.split('=', 1) for a in argv[2:])
    zeilen = [json.loads(l) for l in open(quelle)]
    tab = lernen(zeilen, int(kw.get('min_n', '5')))
    out = dict(version=VERSION, zuege=len(zeilen), angenommen=sum(z.get('ergebnis') == 'angenommen' for z in zeilen),
               quelle_sha=hashlib.sha256(open(quelle, 'rb').read()).hexdigest(), tabelle=tab)
    os.makedirs(os.path.dirname(os.path.abspath(ziel)), exist_ok=True)
    with open(ziel + '.tmp', 'w') as f: json.dump(out, f, indent=1, sort_keys=True)
    os.replace(ziel + '.tmp', ziel)
    print('%d Zuege, %d angenommen, %d Lagen in %s' % (out['zuege'], out['angenommen'], len(tab), ziel))


if __name__ == '__main__':
    main(sys.argv[1:])
