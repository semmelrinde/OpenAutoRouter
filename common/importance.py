

def weights():
    from analysis.evaluation import WT
    return WT


def contribution(weight, total_length, vias, via_value=0.6):
    return weight * total_length + via_value * vias


def swap_better(unit_list, threshold=0.3, tol=0.05):
    gv = sum(e[3] for e in unit_list); gn = sum(e[4] for e in unit_list)
    levels = sorted({e[0] for e in unit_list})
    reversed_ = any(sum(e[2] - e[1] for e in unit_list if e[0] >= s) > tol for s in levels[1:])
    return (gn < gv - threshold and not reversed_), gv, gn, reversed_
