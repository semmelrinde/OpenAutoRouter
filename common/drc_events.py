import collections, re



NET_WIDE = ('skew_out_of_range', 'length_out_of_range', 'diff_pair_uncoupled_length_too_long')


def event(v, category=None):
    if not category and v.get('type') in NET_WIDE:
        net_list = sorted({n for i in v['items'] for n in re.findall(r'\[([^\]]+)\]', i.get('description', ''))})
        rule = re.search(r'["\']([^"\']+)["\']', v.get('description', ''))
        return (v['severity'], v['type'], ('net_name',) + tuple(net_list) + ((rule.group(1),) if rule else ()))
    return (category or v['severity'], category or v['type'], tuple(sorted(i.get('uuid') or i.get('description', '') for i in v['items'])))


def count(d):
    return collections.Counter(event(v) for v in d['violations']) + collections.Counter(event(v, 'open_items') for v in d['unconnected_items'])


def new_events(R, ref):
    rest = collections.Counter(ref['ev']); new = []
    for v in R['d']['violations']:
        k = event(v)
        if rest[k] > 0: rest[k] -= 1
        else: new.append(v)
    for v in R['d']['unconnected_items']:
        k = event(v, 'open_items')
        if rest[k] > 0: rest[k] -= 1
        else: new.append(v)
    return new
