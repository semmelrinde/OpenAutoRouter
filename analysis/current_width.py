

def ipc2221(I, dT=10.0, t=0.035, inner=False):
    if not I or I <= 0: return 0.0
    if dT <= 0 or t <= 0: raise ValueError('Copper thickness and heating must be positive')
    k = 0.024 if inner else 0.048
    area = (I / (k * dT ** 0.44)) ** (1 / 0.725)
    return area / (t / 0.0254) * 0.0254


def copper_thickness(specs, inner=False):
    a = (specs or {}).get('structure') or {}
    return float(a.get('copper_inner_mm' if inner else 'copper_outer_mm') or (0.0175 if inner else 0.035))
