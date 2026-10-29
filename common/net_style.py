import re

_RF = re.compile(r'(?:^|[/_])(RF|ANT|ANTENNA|LNA)(?:$|[_0-9])', re.I)


def organic(net, classes):
    k = (classes or {}).get('net_list', {}).get(net, {})
    return bool(k.get('category') in ('current_val', 'supply', 'sw', 'rf') or k.get('rf') or
                'RF matching' in k.get('reason', '') or _RF.search(net))
