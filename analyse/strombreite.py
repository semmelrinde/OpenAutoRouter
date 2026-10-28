""


def ipc2221(I, dT=10.0, t=0.035, innen=False):
    if not I or I <= 0: return 0.0
    if dT <= 0 or t <= 0: raise ValueError('Kupferdicke und Erwaermung muessen positiv sein')
    k = 0.024 if innen else 0.048
    area = (I / (k * dT ** 0.44)) ** (1 / 0.725)
    return area / (t / 0.0254) * 0.0254


def kupferdicke(vorgaben, innen=False):
    a = (vorgaben or {}).get('aufbau') or {}
    return float(a.get('kupfer_innen_mm' if innen else 'kupfer_aussen_mm') or (0.0175 if innen else 0.035))
