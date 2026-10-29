import json, os, re, sys, uuid

BLOCK = re.compile(r'\n\t\((segment|arc|via)\b[^\n]*\n.*?\n\t\)', re.S)
GEN = re.compile(r'\n\t\(generated\b[^\n]*\n.*?\n\t\)', re.S)
MEMBER = re.compile(r'\(members((?:\s+"[^"]*")*)\s*\)', re.S)
UUID = re.compile(r'\(uuid "([^"]+)"\)')
NET = re.compile(r'\(net "((?:[^"\\]|\\.)*)"\)')


def layer_name(L):
    return L if L.endswith('.Cu') else ('F.Cu' if L == 'F' else 'B.Cu' if L == 'B' else L + '.Cu')


def count(v):
    s = ('%.6f' % round(float(v), 6)).rstrip('0').rstrip('.')
    return '0' if s in ('-0', '') else s


def esc(n):
    return n.replace('\\', '\\\\').replace('"', '\\"')


def _uid(category, content, k):
    return str(uuid.uuid5(uuid.NAMESPACE_URL, 'autorouter-sexpr/%s/%d/%s' % (category, k, json.dumps(content, sort_keys=True))))


def segment(s, k):
    if 'm' in s: return arc(s, k)
    return ('\n\t(segment\n\t\t(start %s %s)\n\t\t(end %s %s)\n\t\t(width %s)\n\t\t(layer "%s")\n\t\t(net "%s")\n\t\t(uuid "%s")\n\t)'
            % (count(s['a'][0]), count(s['a'][1]), count(s['b'][0]), count(s['b'][1]), count(s['w']), layer_name(s['layer']), esc(s['net']),
               s.get('uuid') or _uid('segment', s, k)))


def arc(s, k):
    return ('\n\t(arc\n\t\t(start %s %s)\n\t\t(mid %s %s)\n\t\t(end %s %s)\n\t\t(width %s)\n\t\t(layer "%s")\n\t\t(net "%s")\n\t\t(uuid "%s")\n\t)'
            % (count(s['a'][0]), count(s['a'][1]), count(s['m'][0]), count(s['m'][1]), count(s['b'][0]), count(s['b'][1]), count(s['w']),
               layer_name(s['layer']), esc(s['net']), s.get('uuid') or _uid('arc', s, k)))


def via(v, k):
    Ls = v.get('layers') or ['F', 'B']
    if not ('F' in Ls and 'B' in Ls):
        raise ValueError('only through-hole vias are written: %s' % Ls)
    Ls = ['F', 'B']
    return ('\n\t(via\n\t\t(at %s %s)\n\t\t(size %s)\n\t\t(drill %s)\n\t\t(layers "%s" "%s")\n\t\t(net "%s")\n\t\t(uuid "%s")\n\t)'
            % (count(v['x']), count(v['y']), count(v['d']), count(v['drill']), layer_name(Ls[0]), layer_name(Ls[-1]), esc(v['net']), _uid('via', v, k)))


def swap(text, D, omit=()):
    skip = set(omit); rm = set(D.get('remove', [])); located = set(); part_list = []; pos = 0
    for m in BLOCK.finditer(text):
        blk = m.group(0); u = UUID.search(blk); n = NET.search(blk)
        u = u.group(1) if u else None; n = n.group(1) if n else ''
        if u in rm:
            if n.replace('\\\\', '\x00').replace('\\"', '"').replace('\x00', '\\') in skip: located.add(u); continue
            part_list.append(text[pos:m.start()]); pos = m.end(); located.add(u)
    part_list.append(text[pos:])
    new = ''.join(part_list)
    ng = 0
    if rm & located and '\n\t(generated' in new:
        def gen_way(m):
            nonlocal ng
            mm = MEMBER.search(m.group(0))
            if mm and set(re.findall(r'"([^"]*)"', mm.group(1))) & rm: ng += 1; return ''
            return m.group(0)
        new = GEN.sub(gen_way, new)
    extra = []; k = 0
    for s in D.get('add', []):
        if s['net'] in skip: continue
        if abs(s['a'][0] - s['b'][0]) < 5e-7 and abs(s['a'][1] - s['b'][1]) < 5e-7: continue
        extra.append(segment(s, k)); k += 1
    nb = k; nv = ngen = 0
    for g in D.get('add_generated', []):
        extra.append(g if isinstance(g, str) else _gen_text(g)); ngen += 1
    for v in D.get('add_vias', []):
        if v['net'] in skip: continue
        extra.append(via(v, k)); k += 1; nv += 1

    i = new.find('\n\t(zone')
    if i < 0: i = new.rstrip().rfind(')')
    new = new[:i] + ''.join(extra) + new[i:]
    info = dict(removed=len(located), new_tracks=nb, new_vias=nv, absent=sorted(rm - located))
    if ngen or ng: info.update(new_generators=ngen, generators_removed=ng)
    return new, info


def _gen_text(g):
    from postprocessing.meander import generator_block
    return generator_block(g)


def main():
    a = sys.argv[1:]
    text = open(a[0], encoding='utf-8').read(); D = json.load(open(a[1]))
    new, info = swap(text, D, [n for n in (a[3].split(';') if len(a) > 3 else []) if n])
    if os.path.realpath(a[2]) == os.path.realpath(a[0]) or (os.path.exists(a[2]) and os.path.samefile(a[0], a[2])):
        raise SystemExit('Output must be a new file')
    if info['absent']: raise SystemExit('not found, nothing written: %s' % info['absent'][:10])
    open(a[2], 'w', encoding='utf-8').write(new)
    print(json.dumps(info))


if __name__ == '__main__':
    main()
