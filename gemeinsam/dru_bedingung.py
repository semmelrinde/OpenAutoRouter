""
import fnmatch, re

_TOK = re.compile(r"\s*(\|\||&&|!=|==|!|\(|\)|'[^']*'|\"[^\"]*\"|[A-Za-z_][\w.]*\('[^']*'\)|[A-Za-z_][\w.]*)")


def paar_basis(net, netze):
    ""
    for a, b in (('P', 'N'), ('N', 'P'), ('+', '-'), ('-', '+')):
        if net.endswith(a) and net[:-len(a)] + b in netze:
            base = net[:-len(a)]
            return base[:-1] if base.endswith('_') else base
    return None


def _und(a, b):
    if a is False or b is False: return False
    return None if a is None or b is None else True


def _oder(a, b):
    if a is True or b is True: return True
    return None if a is None or b is None else False


def _nicht(a):
    return None if a is None else not a


def _wert(text):
    return text[1:-1] if text[:1] in '\'"' else text


def passt(cond, net, netclass, layer=None, netze=(), typ='Track'):
    ""
    cond = (cond or '').strip()
    if not cond: return True
    toks = _TOK.findall(cond)
    if ''.join(toks).replace(' ', '') != cond.replace(' ', ''): return None
    pos = [0]

    def nehmen():
        t = toks[pos[0]] if pos[0] < len(toks) else None
        pos[0] += 1
        return t

    def sehen():
        return toks[pos[0]] if pos[0] < len(toks) else None

    def atom():
        t = nehmen()
        if t is None: raise ValueError
        if t == '(':
            v = oder();
            if nehmen() != ')': raise ValueError
            return v
        if t == '!': return _nicht(atom())
        m = re.fullmatch(r"A\.inDiffPair\('([^']*)'\)", t)
        if m:
            b = paar_basis(net, netze)
            return False if b is None else fnmatch.fnmatchcase(b, m[1])
        if sehen() in ('==', '!='):
            op = nehmen(); rechts = nehmen()
            if rechts is None: raise ValueError
            rechts = _wert(rechts)
            if t == 'A.NetClass': v = fnmatch.fnmatchcase(netclass or 'Default', rechts)
            elif t == 'A.NetName': v = fnmatch.fnmatchcase(net, rechts)
            elif t == 'A.Type': v = rechts.lower() == typ.lower()
            elif t == 'A.Layer': v = None if layer is None else fnmatch.fnmatchcase(layer, rechts)
            else: return None
            return _nicht(v) if op == '!=' else v
        return None

    def und():
        v = atom()
        while sehen() == '&&':
            nehmen(); v = _und(v, atom())
        return v

    def oder():
        v = und()
        while sehen() == '||':
            nehmen(); v = _oder(v, und())
        return v

    try:
        v = oder()
        return v if pos[0] == len(toks) else None
    except (ValueError, IndexError):
        return None


def mindestbreite(rules, net, netclass, lagen, netze, grund=0.0):
    ""
    out = {}; namen = set()
    for lage in lagen:
        w = None; vorsicht = 0.0
        for r in rules:
            c = (r.get('constraints') or {}).get('track_width') or {}
            if c.get('min') is None: continue
            if r.get('layer') and lage is not None and not fnmatch.fnmatchcase(lage, r['layer'].replace('outer', '?.Cu')):
                continue
            v = passt(r.get('cond', ''), net, netclass, lage, netze)
            if v is True: w = float(c['min']); namen.add(r['name'])
            elif v is None: vorsicht = max(vorsicht, float(c['min'])); namen.add(r['name'])
        out[lage] = max(grund, w or 0.0, vorsicht)
    return out, sorted(namen)
