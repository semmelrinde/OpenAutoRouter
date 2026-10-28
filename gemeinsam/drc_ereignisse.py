""
import collections, re



NETZWEIT = ('skew_out_of_range', 'length_out_of_range', 'diff_pair_uncoupled_length_too_long')


def ereignis(v, art=None):
    ""
    if not art and v.get('type') in NETZWEIT:
        netze = sorted({n for i in v['items'] for n in re.findall(r'\[([^\]]+)\]', i.get('description', ''))})
        regel = re.search(r'["\']([^"\']+)["\']', v.get('description', ''))
        return (v['severity'], v['type'], ('netz',) + tuple(netze) + ((regel.group(1),) if regel else ()))
    return (art or v['severity'], art or v['type'], tuple(sorted(i.get('uuid') or i.get('description', '') for i in v['items'])))


def zaehlen(d):
    return collections.Counter(ereignis(v) for v in d['violations']) + collections.Counter(ereignis(v, 'offen') for v in d['unconnected_items'])


def neue_ereignisse(R, ref):
    ""
    rest = collections.Counter(ref['ev']); neu = []
    for v in R['d']['violations']:
        k = ereignis(v)
        if rest[k] > 0: rest[k] -= 1
        else: neu.append(v)
    for v in R['d']['unconnected_items']:
        k = ereignis(v, 'offen')
        if rest[k] > 0: rest[k] -= 1
        else: neu.append(v)
    return neu
