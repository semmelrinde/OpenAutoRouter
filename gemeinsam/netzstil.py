""
import re

_RF = re.compile(r'(?:^|[/_])(RF|ANT|ANTENNA|LNA)(?:$|[_0-9])', re.I)


def organisch(net, classes):
    k = (classes or {}).get('netze', {}).get(net, {})
    return bool(k.get('art') in ('strom', 'versorgung', 'sw', 'rf') or k.get('rf') or
                'HF-Anpassung' in k.get('grund', '') or _RF.search(net))
