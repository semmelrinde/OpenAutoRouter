from dataclasses import asdict, dataclass
import math
import torch
from torch import nn
from torch.nn import functional as F
from learning.ai.features_raster import CHANNELS, VERSION as FEATURES_VERSION

VERSION = 'routability-v2'
PHYSICS = ('dc', 'j', 'ret', 'xt')
INPUTS = ('input_', 'origin_mm', 'raster_mm', 'global_values', 'part_values',
             'pad_values', 'pad_layers', 'net_values')

@dataclass(frozen=True)
class Configuration:
    width: int = 32
    dropout: float = .1


def hw_device(name='mps'):
    if name == 'mps' and not torch.backends.mps.is_available():
        raise RuntimeError('MPS not available; no silent CPU fallback')
    if name not in ('mps', 'cpu'): raise ValueError('hw_device=mps|cpu')
    return torch.device(name)


def input_(sample, device='cpu'):
    d = sample.get('tensors', sample)
    if 'names' in sample:
        if tuple(sample['names'].get('channel_names', ())) != CHANNELS:
            raise ValueError('GridChannelContract')
    if 'features_version' in d and int(d['features_version']) != FEATURES_VERSION:
        raise ValueError('FeaturesVersion')
    out = {k: torch.as_tensor(d[k]).float().cpu() for k in INPUTS}
    x = out['input_']
    if x.ndim != 4 or x.shape[1] != len(CHANNELS) or x.shape[0] < 2 or min(x.shape[-2:]) < 1:
        raise ValueError('GridShape')
    if not torch.isfinite(x).all() or (x < 0).any(): raise ValueError('GridValues')
    if not math.isclose(float(out['raster_mm']), .25): raise ValueError('GridSize')
    for k in ('origin_mm', 'global_values', 'part_values', 'pad_values', 'pad_layers'):
        if not torch.isfinite(out[k]).all(): raise ValueError('Graph values: ' + k)
    if out['part_values'].ndim != 2 or out['part_values'].shape[1] != 7 or out['pad_values'].ndim != 2 or out['pad_values'].shape[1] != 5 or out['net_values'].ndim != 2 or out['net_values'].shape[1] != 6:
        raise ValueError('GraphColumnContract')
    nw = out['net_values']
    if torch.isinf(nw).any() or (torch.nan_to_num(nw[:,1:3],nan=0.) < 0).any(): raise ValueError('electrical input values')
    if not torch.isfinite(nw[:, (0, 3, 4, 5)]).all(): raise ValueError('NetTable')
    pads = out['pad_values']; parts = out['part_values']
    if len(pads):
        ix = pads[:, 2:4]
        if (ix != ix.round()).any() or (ix[:, 0] < -1).any() or (ix[:, 0] >= len(parts)).any() or (ix[:, 1] < 0).any() or (ix[:, 1] >= len(nw)).any():
            raise ValueError('GraphIncidence')
    if out['pad_layers'].shape != (len(pads), x.shape[0]): raise ValueError('PadLayers')


    names=sample.get('names',{})
    import json
    rules=json.loads(names.get('design_rules_json','{}'))
    stack=rules.get('structure')
    stack_values=torch.zeros((x.shape[0],6),dtype=torch.float32)
    if stack:
        from analysis.structure import normalize
        if not all(k in stack for k in ('dielectrics','copper_mm')) or not ('er' in stack or 'er_layers' in stack):
            raise ValueError('incomplete explicit stackup')
        a=normalize(stack,x.shape[0])
        for i in range(x.shape[0]):
            stack_values[i]=torch.tensor([a['copper_mm'][i],a['dielectrics'][i-1] if i else 0,
                a['dielectrics'][i] if i<x.shape[0]-1 else 0,a['er_layers'][i-1]/10 if i else 0,
                a['er_layers'][i]/10 if i<x.shape[0]-1 else 0,1.])
    out['stackup']=stack_values

    import numpy as np
    pp = torch.as_tensor(d['pad_values']).cpu().numpy()
    origin = torch.as_tensor(d['origin_mm']).cpu().numpy()
    xy = np.floor((pp[:, :2] - origin) / .25).astype(np.int64)
    ids = pp[:, 3].astype(np.int64)
    bounds = np.zeros((len(nw), 4), np.int64)
    if len(pp):
        a = np.full((len(nw), 2), 2**30, np.int64); b = np.full((len(nw), 2), -1, np.int64)
        np.minimum.at(a, ids, xy); np.maximum.at(b, ids, xy + 1)
        used = b[:, 0] >= 0; bounds[used, :2] = a[used]; bounds[used, 2:] = b[used]
    bounds[:, (0, 2)] = bounds[:, (0, 2)].clip(0, x.shape[-1])
    bounds[:, (1, 3)] = bounds[:, (1, 3)].clip(0, x.shape[-2])
    out['net_boxes'] = torch.as_tensor(bounds)
    return {k:v.to(device) for k,v in out.items()}


class Block(nn.Sequential):
    def __init__(self, ni, no, dropout):
        super().__init__(nn.Conv2d(ni, no, 3, padding=1, bias=False), nn.GroupNorm(8, no), nn.SiLU(),
                         nn.Conv2d(no, no, 3, padding=1, bias=False), nn.GroupNorm(8, no), nn.SiLU(),
                         nn.Dropout2d(dropout))


class Routability(nn.Module):
    def __init__(self, cfg=Configuration()):
        super().__init__(); self.cfg = cfg
        if cfg.width < 8 or cfg.width % 8: raise ValueError('Width must be divisible by 8')
        w = [cfg.width * 2**i for i in range(5)]

        self.down = nn.ModuleList(Block(len(CHANNELS)+13 if i == 0 else w[i-1], w[i], cfg.dropout) for i in range(5))
        self.up = nn.ModuleList(Block(w[i+1]+w[i], w[i], cfg.dropout) for i in reversed(range(4)))
        self.grid_map = nn.Conv2d(w[0], 2, 1)
        self.global_head = nn.Sequential(nn.Linear(3*w[0]+19, 128), nn.SiLU(), nn.Dropout(cfg.dropout), nn.Linear(128, 11))

        self.net_head = nn.Sequential(nn.Linear(2*w[0]+16, 128), nn.SiLU(), nn.Dropout(cfg.dropout), nn.Linear(128, 9))

    def encoder(self, xs):
        x = torch.cat(xs); skips=[]
        for i, block in enumerate(self.down):
            if i: x = F.avg_pool2d(x, 2)
            x = block(x); skips.append(x)
        for block, skip in zip(self.up, reversed(skips[:-1])):
            x = block(torch.cat((F.interpolate(x, size=skip.shape[-2:], mode='bilinear', align_corners=False), skip), 1))
        return x

    def prepare(self, d, hw):
        x = d['input_']; l, _, h, w = x.shape
        z = torch.arange(l, device=x.device, dtype=x.dtype) / max(1, l-1)
        ctx = torch.stack((z, (z==0).float(), (z==1).float(), x[:,7].mean((-1,-2))), 1)
        g = d['global_values'] / 8
        ctx = torch.cat((ctx, g.expand(l,-1), d['stackup']), 1)[:,:,None,None].expand(-1,-1,h,w)
        x = torch.cat((torch.log1p(x),ctx),1)
        return F.pad(x,(0,hw[1]-w,0,hw[0]-h))

    def pool_nets(self, f, d):
        l, c, h, w = f.shape; n = len(d['net_values']); p = d['pad_values']; part = d['part_values']
        mask = d['input_'][:,0:1] * (1-d['input_'][:,7:8])
        ff = (f*mask).sum(0); count = mask.sum(0)
        integ = F.pad(ff.cumsum(-1).cumsum(-2),(1,0,1,0)); ci=F.pad(count.cumsum(-1).cumsum(-2),(1,0,1,0))
        coords = torch.floor((p[:,:2]-d['origin_mm'])/d['raster_mm']).long()
        xx=coords[:,0].clamp(0,w-1); yy=coords[:,1].clamp(0,h-1)
        padfeat = f[:, :, yy, xx].permute(2,0,1)
        pl = d['pad_layers']
        padfeat = (padfeat*pl[:,:,None]).sum(1)/pl.sum(1).clamp_min(1)[:,None]
        pf = torch.zeros((len(p),7),device=f.device)
        good = p[:,2]>=0
        pf[good] = part[p[good,2].long()]
        pf[:,:2]=(pf[:,:2]-d['origin_mm'])/(d['raster_mm']*torch.tensor([w,h],device=f.device))
        pf[:,2:4] /= torch.tensor([360,max(1,l-1)],device=f.device); pf[:,5]=(pf[:,5]>=0).float()
        box=d['net_boxes']; x0,y0,x1,y1=box.unbind(1)
        val=(integ[:,y1,x1]-integ[:,y0,x1]-integ[:,y1,x0]+integ[:,y0,x0]).T
        the=(ci[:,y1,x1]-ci[:,y0,x1]-ci[:,y1,x0]+ci[:,y0,x0]).T.clamp_min(1)
        padnet=torch.cat((padfeat,pf),1)
        sums=f.new_zeros((n,c+7)); counts=f.new_zeros((n,1)); ids=p[:,3].long()
        sums.index_add_(0,ids,padnet); counts.index_add_(0,ids,f.new_ones((len(p),1)))
        pooled=torch.cat((val/the,sums/counts.clamp_min(1)),1)
        nw=d['net_values']; known=torch.isfinite(nw[:,1:3]).float()
        nw=torch.nan_to_num(nw,nan=0.)
        nw=torch.cat((nw[:,:1]/6,torch.log1p(nw[:,1:2].clamp_min(0)),
                      torch.log1p(nw[:,2:3].clamp_min(0)/1e6),(nw[:,3:4]>=0).float(),
                      torch.log1p(nw[:,4:6].clamp_min(0)),known,d['global_values'][2].expand(n,1)/8),1)
        return torch.cat((pooled,nw),1)

    def forward(self, inputs):
        if not isinstance(inputs,list): raise TypeError('Batch as list of explicit inputs')
        if not inputs: return []


        groups={}; feature_rows={}
        for i,d in enumerate(inputs):
            hw=tuple(((v+15)//16)*16 for v in d['input_'].shape[-2:])
            groups.setdefault(hw,[]).append((i,d))
        for hw, group in groups.items():
            features=self.encoder([self.prepare(d,hw) for _,d in group]);off=0
            for i,d in group:
                l,_,h,w=d['input_'].shape
                feature_rows[i]=features[off:off+l,:,:h,:w];off+=l
        out=[]
        for i,d in enumerate(inputs):
            f=feature_rows[i]
            m=d['input_'][:,0:1]; g=(f*m).sum((0,2,3))/m.sum().clamp_min(1)
            graph=self.pool_nets(f,d)
            graph_g=graph.mean(0) if len(graph) else f.new_zeros(2*f.shape[1]+16)
            q=self.global_head(torch.cat((g,d['global_values']/8,graph_g)))
            net=self.net_head(graph); maps=self.grid_map(f)
            out.append(dict(overload_mu=maps[:,0], overload_logvar=maps[:,1].clamp(-8,8),
                            valid_logit=q[0], score_mu=q[1], score_logvar=q[2].clamp(-8,8),
                            physics_mu=q[3:7], physics_logvar=q[7:11].clamp(-8,8),
                            net_logit=net[:,0], net_physics_mu=net[:,1:5], net_physics_logvar=net[:,5:9].clamp(-8,8)))
        return out


def goal_physics(t):
    from learning.ai.data import X_COLUMNS
    ids=[X_COLUMNS.index(k) for k in ('dc','j','ret','xt_victim','xt_disturb')]
    x=t['physics_x'][:,ids].float(); m=t['physics_mask'][:,ids].bool() & torch.isfinite(x) & (x>=0)

    y=torch.cat((x[:,:3],x[:,3:5].amax(1,keepdim=True)),1)
    mask=torch.cat((m[:,:3],m[:,3:5].all(1,keepdim=True)),1)
    return y, mask


def goal_physics_global(t):
    y,m=goal_physics(t);kinds=t['net_values'][:,0].long()
    req=torch.stack(((kinds==2)|(kinds==3),(kinds==2)|(kinds==3),kinds==1,(kinds==1)|(kinds==4)),1)
    valid=req.any(0)&(m|~req).all(0)
    values=torch.where(req&m,y,torch.full_like(y,float('-inf'))).amax(0) if len(y) else torch.full((4,),float('nan'))
    return torch.where(valid,values,torch.full_like(values,float('nan'))),valid


def power_loss(p, sample):
    t=sample['tensors']; dev=p['score_mu'].device; losses={}
    def gaussian(name, mu, lv, y, mask):
        y=y.to(dev).float(); mask=mask.to(dev).bool() & torch.isfinite(y) & (y>=0)
        if mask.any():
            e=mu[mask]-torch.log1p(y[mask]); v=lv[mask]
            losses[name]=(.5*(torch.exp(-v)*e.square()+v)).mean()
    def binary(name,logit,y,mask):
        mask=mask.to(dev).bool();y=y.to(dev).float()
        if mask.any(): losses[name]=F.binary_cross_entropy_with_logits(logit[mask],y[mask])
    if 'goal_raster' in t:
        gaussian('overload',p['overload_mu'],p['overload_logvar'],t['goal_raster'][:,0],t['goal_raster_mask'][:,0])
    binary('is_valid',p['valid_logit'].reshape(1),t['global_goal'][:1],t['global_mask'][:1])
    gaussian('rating',p['score_mu'].reshape(1),p['score_logvar'].reshape(1),t['global_goal'][1:2],t['global_mask'][1:2])
    if 'net_joined' in t:
        binary('net_name',p['net_logit'],t['net_joined'],t['net_joined_mask'])
    if 'physics_x' in t:
        y,m=goal_physics(t); gaussian('net_physics',p['net_physics_mu'],p['net_physics_logvar'],y,m)
        gy,gm=goal_physics_global(t)
        for j in range(4):
            gaussian('physics_'+PHYSICS[j],p['physics_mu'][j:j+1],p['physics_logvar'][j:j+1],gy[j:j+1],gm[j:j+1])
    total=sum(losses.values(),p['score_mu']*0)
    return total, {k:float(v.detach()) for k,v in losses.items()}


def configuration(model): return asdict(model.cfg)
