import fnmatch, re

_TOK = re.compile(r"\s*(\|\||&&|!=|==|!|\(|\)|'[^']*'|\"[^\"]*\"|[A-Za-z_][\w.]*\('[^']*'\)|[A-Za-z_][\w.]*)")


def pair_basis(net, net_list):
    for a, b in (('P', 'N'), ('N', 'P'), ('+', '-'), ('-', '+')):
        if net.endswith(a) and net[:-len(a)] + b in net_list:
            base = net[:-len(a)]
            return base[:-1] if base.endswith('_') else base
    return None


def _and(a, b):
    if a is False or b is False: return False
    return None if a is None or b is None else True


def _or(a, b):
    if a is True or b is True: return True
    return None if a is None or b is None else False


def _not(a):
    return None if a is None else not a


def _value(text):
    return text[1:-1] if text[:1] in '\'"' else text


def fits(cond, net, netclass, layer=None, net_list=(), type_='Track'):
    cond = (cond or '').strip()
    if not cond: return True
    toks = _TOK.findall(cond)
    if ''.join(toks).replace(' ', '') != cond.replace(' ', ''): return None
    pos = [0]

    def take():
        t = toks[pos[0]] if pos[0] < len(toks) else None
        pos[0] += 1
        return t

    def see():
        return toks[pos[0]] if pos[0] < len(toks) else None

    def atom():
        t = take()
        if t is None: raise ValueError
        if t == '(':
            v = or_();
            if take() != ')': raise ValueError
            return v
        if t == '!': return _not(atom())
        m = re.fullmatch(r"A\.inDiffPair\('([^']*)'\)", t)
        if m:
            b = pair_basis(net, net_list)
            return False if b is None else fnmatch.fnmatchcase(b, m[1])
        if see() in ('==', '!='):
            op = take(); right = take()
            if right is None: raise ValueError
            right = _value(right)
            if t == 'A.NetClass': v = fnmatch.fnmatchcase(netclass or 'Default', right)
            elif t == 'A.NetName': v = fnmatch.fnmatchcase(net, right)
            elif t == 'A.Type': v = right.lower() == type_.lower()
            elif t == 'A.Layer': v = None if layer is None else fnmatch.fnmatchcase(layer, right)
            else: return None
            return _not(v) if op == '!=' else v
        return None

    def and_():
        v = atom()
        while see() == '&&':
            take(); v = _and(v, atom())
        return v

    def or_():
        v = and_()
        while see() == '||':
            take(); v = _or(v, and_())
        return v

    try:
        v = or_()
        return v if pos[0] == len(toks) else None
    except (ValueError, IndexError):
        return None


def min_width(rules, net, netclass, cu_layers, net_list, reason=0.0):
    out = {}; names = set()
    for layer_name in cu_layers:
        w = None; caution = 0.0
        for r in rules:
            c = (r.get('constraints') or {}).get('track_width') or {}
            if c.get('min') is None: continue
            if r.get('layer') and layer_name is not None and not fnmatch.fnmatchcase(layer_name, r['layer'].replace('outer', '?.Cu')):
                continue
            v = fits(r.get('cond', ''), net, netclass, layer_name, net_list)
            if v is True: w = float(c['min']); names.add(r['name'])
            elif v is None: caution = max(caution, float(c['min'])); names.add(r['name'])
        out[layer_name] = max(reason, w or 0.0, caution)
    return out, sorted(names)
