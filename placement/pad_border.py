from collections import defaultdict
from shapely import affinity
from shapely.ops import unary_union
from shapely.prepared import prep


class PadBorder:
    def __init__(self, polys, fps, board, spacing):
        by=defaultdict(list)
        for (r,_),g in polys.items():by[r].append(g)
        self.local={r:unary_union(gs) for r,gs in by.items()}
        self.board_side={r:f['side'] for r,f in fps.items()}
        self.inner=board.buffer(-max(0,spacing)-.001)
        self.test=prep(self.inner);self.cache={}

    def vacant(self,r,st):
        if r not in self.local:return True
        x,y,th,side=st;key=(r,th,side)
        if key not in self.cache:
            g=self.local[r]
            if side!=self.board_side[r]:g=affinity.scale(g,-1,1,origin=(0,0))
            self.cache[key]=affinity.rotate(g,th,origin=(0,0))
        return self.test.covers(affinity.translate(self.cache[key],x,y))
