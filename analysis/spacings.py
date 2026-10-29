import fnmatch
import math
import re
from functools import lru_cache

VERSION = 'net_pair-spacing-v1'
TOKEN = re.compile(r"\s*(\&\&|\|\||==|!=|!|\(|\)|[AB]\.(?:hasNetclass|NetName|NetClass|Type|Pad_Type)|'(?:[^'\\]|\\.)*')")


@lru_cache(maxsize=4096)
def _wild_regex(pattern):

    return re.compile(re.escape(pattern.upper()).replace(r'\*','.*').replace(r'\?','.'),re.S)


def _wild(value,pattern):
    return _wild_regex(pattern).fullmatch(value.upper()) is not None


def condition(text):
    if not text.strip(): return ('true',)
    tokens = []; end = 0
    while end < len(text.rstrip()):
        m = TOKEN.match(text, end)
        if not m: raise ValueError('unsupported clearance condition: '+text)
        tokens.append(m[1]); end = m.end()
    pos = [0]
    def take(value=None):
        if pos[0] >= len(tokens): raise ValueError('incomplete clearance condition: '+text)
        t = tokens[pos[0]]
        if value is not None and t != value: raise ValueError('invalid clearance condition: '+text)
        pos[0] += 1; return t
    def atom():
        if pos[0] < len(tokens) and tokens[pos[0]] == '!':
            take(); return ('not', atom())
        if pos[0] < len(tokens) and tokens[pos[0]] == '(':
            take(); a = expr(); take(')'); return a
        prop = take()
        if not re.fullmatch(r'[AB]\.(NetName|NetClass|Type|Pad_Type|hasNetclass)', prop):
            raise ValueError('invalid clearance property: '+text)
        if prop.endswith('hasNetclass'):
            take('('); value = take(); take(')'); op = 'has'
        else:
            op = take()
            if op not in ('==', '!='): raise ValueError('invalid clearance comparison: '+text)
            value = take()
        if not value.startswith("'"):
            if op != 'has' and re.fullmatch(r'[AB]\.(NetName|NetClass|Type|Pad_Type)',value):return ('compare',prop,op,('prop',value))
            raise ValueError('invalid clearance value: '+text)
        return ('compare', prop, op, ('text',value[1:-1].replace("\\'", "'").replace('\\\\','\\')))
    def expr():


        a = atom()
        while pos[0] < len(tokens) and tokens[pos[0]] in ('&&','||'):
            op = take(); a = ('and' if op=='&&' else 'or', a, atom())
        return a
    ast = expr()
    if pos[0] != len(tokens): raise ValueError('Remainder in clearance condition: '+text)
    return ast


def _match(ast, a, b):
    op = ast[0]
    if op == 'true': return True
    if op == 'not': return not _match(ast[1], a, b)
    if op == 'and': return _match(ast[1], a, b) and _match(ast[2], a, b)
    if op == 'or': return _match(ast[1], a, b) or _match(ast[2], a, b)
    prop, cmp, val = ast[1:]
    def get(p): return (a if p[0]=='A' else b).get(p.split('.')[1], '')
    left = get(prop); right = get(val[1]) if val[0]=='prop' else val[1]
    if prop.endswith('.Pad_Type') and (a if prop[0]=='A' else b)['Type']!='Pad':return False
    if val[0]=='prop' and val[1].endswith('.Pad_Type') and (a if val[1][0]=='A' else b)['Type']!='Pad':return False
    if cmp == 'has': return any(_wild(n,right) for n in (a if prop[0]=='A' else b)['classes'])
    if prop.endswith('.NetClass') and val[0]=='text':
        equal = _wild(left,right) or any(_wild(n,right) for n in left.split(','))
    elif val[0]=='prop' and prop.split('.')[1] in ('NetName','NetClass') and prop.split('.')[1]==val[1].split('.')[1]:
        equal = left == right
    else:
        equal = _wild(left,right) if val[0]=='text' else left.upper()==right.upper()
    return equal if cmp=='==' else not equal


def _mass(x):
    x = float(x)
    if not math.isfinite(x) or x < 0: raise ValueError('Clearance must be finite and non-negative')
    return x


def _mass_expression(s):
    m=re.fullmatch(r'\s*([+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?)\s*(mm|mil|th|in)?\s*',s)
    if not m:return None
    value=float(m[1]);return value*.0254 if m[2] in ('mil','th') else value*25.4 if m[2]=='in' else value


def contract_spec(board=None, design_rules=None, project=None, dru=None):
    board = board or {}; design_rules = design_rules or {}; project = project or {}
    old = design_rules.get('spacings') or {}
    if old and old.get('version') != VERSION: raise ValueError('unknown spacing contract version')
    raw_data = design_rules.get('raw_data') or {}
    minimum = max(_mass((board.get('design') or {}).get('min_spacing') or 0),
                  _mass(old.get('board_min', raw_data.get('min_clearance') or 0)))
    classes = dict(old.get('netclasses') or {})
    for c in ((project.get('net_settings') or {}).get('classes') or []):
        classes[c['name']] = c.get('clearance')
    for name, c in (board.get('net_classes') or {}).items(): classes[name] = c.get('clr')
    classes = {k:None if v is None else _mass(v) for k,v in sorted(classes.items())}
    if 'Default' not in classes: classes['Default'] = raw_data.get('netclass_clearance')
    nets = dict(old.get('net_list') or {})
    ns=project.get('net_settings') or {}
    patterns=(ns.get('netclass_patterns') or []) if 'netclass_patterns' in ns else (old.get('match_pattern') or [])
    for n, names in (ns.get('netclass_assignments') or {}).items():
        nets[n]=names if isinstance(names,str) else ','.join(names)

    for c in ((project.get('net_settings') or {}).get('classes') or []):
        for n in (c.get('nets') or []): nets[n] = c['name']
    nets.update(board.get('netclass') or {})
    effective=dict(old.get('nets_values') or {})
    effective.update({n:_mass(v['clr']) for n,v in (board.get('netclass_effective') or {}).items() if v.get('clr') is not None})
    rules = []; unknown = []
    if not old and dru is None and (raw_data.get('dru_conditional') or design_rules.get('dru_conditional') or
            raw_data.get('dru_global') or design_rules.get('dru_global') or
            raw_data.get('pad_track') is not None or raw_data.get('pad_pad_smd') is not None):
        unknown.append(dict(name='DRU content missing',reason='Legacy rules contain only summarized DRU values; re-export'))
    raw_rules = dru if dru is not None else old.get('dru', [])
    for r in raw_rules:
        c = r.get('constraints', {}).get('clearance')
        if c is None: continue
        try:
            expr=((r.get('constraints_expression') or {}).get('clearance') or {}).get('min')
            if expr is not None:
                c=dict(c,min=_mass_expression(expr))
                r=dict(r,constraints=dict(r['constraints'],clearance=c))
            if c.get('min') is None: raise ValueError('Clearance without minimum')
            _mass(c['min']); ast = condition(r.get('cond',''))
            if r.get('severity') == 'ignore':
                raise ValueError('ignored clearance rule needs native resolution')
            if r.get('layer') and r['layer'] not in ('outer','inner','F.Cu','B.Cu','*.Cu') and not re.fullmatch(r'In\d+\.Cu',r['layer']):
                raise ValueError('unsupported clearance layer')
            rules.append(dict(r, ast=ast))
        except ValueError as e: unknown.append(dict(name=r['name'],reason=str(e)))

    unknown.extend(old.get('unknown', []))
    return dict(version=VERSION, board_min=minimum, netclasses=classes,
                net_list=dict(sorted(nets.items())), nets_values=effective,
                match_pattern=patterns, dru=rules, unknown=unknown)


class Spacing:
    def __init__(self, contract):
        if contract.get('version') != VERSION: raise ValueError('unknown spacing contract version')
        if contract.get('unknown'): raise ValueError('Spacing could not be evaluated: '+str(contract['unknown']))
        if not contract['board_min'] and not any(v is not None for v in contract['netclasses'].values()) and not contract.get('nets_values'):
            raise ValueError('no occupied copper spacing; re-export board/rules')
        self.c = contract
        values = [contract['board_min']] + [v for v in contract['netclasses'].values() if v is not None]
        values += [r['constraints']['clearance']['min'] for r in contract['dru']]
        values += list(contract.get('nets_values',{}).values())
        self.maximum = max(map(_mass,values))

    def legacy_covers(self, nets, layers, drc, drcpad):
        if any('NetName' in r.get('cond','') for r in self.c['dru']):return False
        groups={}
        for n in nets:
            o=self.obj(n,'Track','');name=o['NetClass']
            value=self.c.get('nets_values',{}).get(n,self.c['netclasses'].get(name))
            if value is None and name!='Default':return False
            groups[(name,value)]=None
        c=dict(self.c,net_list=dict(self.c['net_list']),nets_values=dict(self.c.get('nets_values',{})))
        reps=[]
        for i,(name,value) in enumerate(sorted(groups,key=lambda v:(v[0],-1. if v[1] is None else v[1]))):
            for j in (0,1):
                n='__spacing_probe_%d_%d'%(i,j);reps.append(n);c['net_list'][n]=name
                if value is not None:c['nets_values'][n]=value
        probe=Spacing(c)
        for a in reps:
            for b in reps:
                if a==b:continue
                for layer in layers:
                    for type_ in ('Track','Via'):
                        for type2,pad in (('Track',''),('Via',''),('Pad','SMD'),('Pad','Through-hole'),('Pad','Edge connector'),('Pad','NPTH, mechanical'),('Zone',''),('Graphic','')):
                            if probe.pair(a,b,layer,type_,type2,'',pad)[0]>(drcpad if type2=='Pad' else drc)+1e-12:return False
        return True

    def obj(self, net, type_, pad):
        name = self.c['net_list'].get(net)
        if name is None:
            names=sorted({m['netclass'] for m in self.c.get('match_pattern',[]) if fnmatch.fnmatchcase(net,m['pattern'])})
            name=','.join(names) if names else 'Default'

        return dict(NetName=net, NetClass=name, Type=type_, Pad_Type=pad,
                    classes=name.split(','))

    @lru_cache(maxsize=262144)
    def pair(self, a, b, layer='F', type_a='Track', type_b='Track', pad_a='', pad_b=''):
        if a == b and a: return (0.0,'same net')
        objs = self.obj(a,type_a,pad_a), self.obj(b,type_b,pad_b)
        classes = self.c['netclasses']; default = classes.get('Default')
        vals = []
        for net,o in zip((a,b),objs):
            value=self.c.get('nets_values',{}).get(net,classes.get(o['NetClass']))
            if value is None and o['NetClass']!='Default':
                raise ValueError('effective KiCad netclass spacing missing for '+net+': '+o['NetClass'])
            vals.append(value)
        vals = [default if v is None else v for v in vals]
        value = max([self.c['board_min']] + [v for v in vals if v is not None]); source = 'Netclass/Board'
        for r in self.c['dru']:
            L = r.get('layer')
            label = layer if layer.endswith('.Cu') else layer+'.Cu'
            if L and not (L == label or L=='*.Cu' or (L=='outer' and label in ('F.Cu','B.Cu')) or (L=='inner' and label.startswith('In'))): continue
            ast = r.get('ast') or condition(r.get('cond',''))
            if _match(ast,*objs) or _match(ast,*reversed(objs)):
                value = max(self.c['board_min'], _mass(r['constraints']['clearance']['min'])); source = 'DRU:'+r['name']
        return value, source
