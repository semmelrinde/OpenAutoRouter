""
from dataclasses import asdict, dataclass
import math
import torch
from torch import nn
from torch.nn import functional as F
from lernen.ki.merkmale_raster import KANALE, VERSION as MERKMALE_VERSION

VERSION = 'routbarkeit-v2'
PHYSIK = ('dc', 'j', 'rueck', 'xt')
EINGAENGE = ('eingang', 'origin_mm', 'raster_mm', 'global_werte', 'teil_werte',
             'pad_werte', 'pad_lagen', 'netz_werte')

@dataclass(frozen=True)
class Konfiguration:
    breite: int = 32
    dropout: float = .1


def geraet(name='mps'):
    if name == 'mps' and not torch.backends.mps.is_available():
        raise RuntimeError('MPS nicht verfügbar; kein stiller CPU-Ersatz')
    if name not in ('mps', 'cpu'): raise ValueError('geraet=mps|cpu')
    return torch.device(name)


def eingang(sample, device='cpu'):
    ""
    d = sample.get('tensoren', sample)
    if 'namen' in sample:
        if tuple(sample['namen'].get('kanal_namen', ())) != KANALE:
            raise ValueError('Rasterkanalvertrag')
    if 'merkmale_version' in d and int(d['merkmale_version']) != MERKMALE_VERSION:
        raise ValueError('Merkmaleversion')
    out = {k: torch.as_tensor(d[k]).float().cpu() for k in EINGAENGE}
    x = out['eingang']
    if x.ndim != 4 or x.shape[1] != len(KANALE) or x.shape[0] < 2 or min(x.shape[-2:]) < 1:
        raise ValueError('Rasterform')
    if not torch.isfinite(x).all() or (x < 0).any(): raise ValueError('Rasterwerte')
    if not math.isclose(float(out['raster_mm']), .25): raise ValueError('Rastermaß')
    for k in ('origin_mm', 'global_werte', 'teil_werte', 'pad_werte', 'pad_lagen'):
        if not torch.isfinite(out[k]).all(): raise ValueError('Graphwerte: ' + k)
    if out['teil_werte'].ndim != 2 or out['teil_werte'].shape[1] != 7 or out['pad_werte'].ndim != 2 or out['pad_werte'].shape[1] != 5 or out['netz_werte'].ndim != 2 or out['netz_werte'].shape[1] != 6:
        raise ValueError('Graphspaltenvertrag')
    nw = out['netz_werte']
    if torch.isinf(nw).any() or (torch.nan_to_num(nw[:,1:3],nan=0.) < 0).any(): raise ValueError('elektrische Eingangswerte')
    if not torch.isfinite(nw[:, (0, 3, 4, 5)]).all(): raise ValueError('Netztabelle')
    pads = out['pad_werte']; parts = out['teil_werte']
    if len(pads):
        ix = pads[:, 2:4]
        if (ix != ix.round()).any() or (ix[:, 0] < -1).any() or (ix[:, 0] >= len(parts)).any() or (ix[:, 1] < 0).any() or (ix[:, 1] >= len(nw)).any():
            raise ValueError('Graphinzidenz')
    if out['pad_lagen'].shape != (len(pads), x.shape[0]): raise ValueError('Padlagen')


    names=sample.get('namen',{})
    import json
    rules=json.loads(names.get('regeln_json','{}'))
    stack=rules.get('aufbau')
    stack_values=torch.zeros((x.shape[0],6),dtype=torch.float32)
    if stack:
        from analyse.aufbau import normalisieren
        if not all(k in stack for k in ('dielektrika','kupfer_mm')) or not ('er' in stack or 'er_lagen' in stack):
            raise ValueError('unvollständiger expliziter Stackup')
        a=normalisieren(stack,x.shape[0])
        for i in range(x.shape[0]):
            stack_values[i]=torch.tensor([a['kupfer_mm'][i],a['dielektrika'][i-1] if i else 0,
                a['dielektrika'][i] if i<x.shape[0]-1 else 0,a['er_lagen'][i-1]/10 if i else 0,
                a['er_lagen'][i]/10 if i<x.shape[0]-1 else 0,1.])
    out['stackup']=stack_values

    import numpy as np
    pp = torch.as_tensor(d['pad_werte']).cpu().numpy()
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
    out['netz_boxen'] = torch.as_tensor(bounds)
    return {k:v.to(device) for k,v in out.items()}


class Block(nn.Sequential):
    def __init__(self, ni, no, dropout):
        super().__init__(nn.Conv2d(ni, no, 3, padding=1, bias=False), nn.GroupNorm(8, no), nn.SiLU(),
                         nn.Conv2d(no, no, 3, padding=1, bias=False), nn.GroupNorm(8, no), nn.SiLU(),
                         nn.Dropout2d(dropout))


class Routbarkeit(nn.Module):
    def __init__(self, cfg=Konfiguration()):
        super().__init__(); self.cfg = cfg
        if cfg.breite < 8 or cfg.breite % 8: raise ValueError('Breite muss durch 8 teilbar sein')
        w = [cfg.breite * 2**i for i in range(5)]

        self.down = nn.ModuleList(Block(len(KANALE)+13 if i == 0 else w[i-1], w[i], cfg.dropout) for i in range(5))
        self.up = nn.ModuleList(Block(w[i+1]+w[i], w[i], cfg.dropout) for i in reversed(range(4)))
        self.karte = nn.Conv2d(w[0], 2, 1)
        self.global_kopf = nn.Sequential(nn.Linear(3*w[0]+19, 128), nn.SiLU(), nn.Dropout(cfg.dropout), nn.Linear(128, 11))

        self.netz_kopf = nn.Sequential(nn.Linear(2*w[0]+16, 128), nn.SiLU(), nn.Dropout(cfg.dropout), nn.Linear(128, 9))

    def encoder(self, xs):
        x = torch.cat(xs); skips=[]
        for i, block in enumerate(self.down):
            if i: x = F.avg_pool2d(x, 2)
            x = block(x); skips.append(x)
        for block, skip in zip(self.up, reversed(skips[:-1])):
            x = block(torch.cat((F.interpolate(x, size=skip.shape[-2:], mode='bilinear', align_corners=False), skip), 1))
        return x

    def vorbereiten(self, d, hw):
        x = d['eingang']; l, _, h, w = x.shape
        z = torch.arange(l, device=x.device, dtype=x.dtype) / max(1, l-1)
        ctx = torch.stack((z, (z==0).float(), (z==1).float(), x[:,7].mean((-1,-2))), 1)
        g = d['global_werte'] / 8
        ctx = torch.cat((ctx, g.expand(l,-1), d['stackup']), 1)[:,:,None,None].expand(-1,-1,h,w)
        x = torch.cat((torch.log1p(x),ctx),1)
        return F.pad(x,(0,hw[1]-w,0,hw[0]-h))

    def pool_netze(self, f, d):
        ""
        l, c, h, w = f.shape; n = len(d['netz_werte']); p = d['pad_werte']; part = d['teil_werte']
        mask = d['eingang'][:,0:1] * (1-d['eingang'][:,7:8])
        ff = (f*mask).sum(0); count = mask.sum(0)
        integ = F.pad(ff.cumsum(-1).cumsum(-2),(1,0,1,0)); ci=F.pad(count.cumsum(-1).cumsum(-2),(1,0,1,0))
        coords = torch.floor((p[:,:2]-d['origin_mm'])/d['raster_mm']).long()
        xx=coords[:,0].clamp(0,w-1); yy=coords[:,1].clamp(0,h-1)
        padfeat = f[:, :, yy, xx].permute(2,0,1)
        pl = d['pad_lagen']
        padfeat = (padfeat*pl[:,:,None]).sum(1)/pl.sum(1).clamp_min(1)[:,None]
        pf = torch.zeros((len(p),7),device=f.device)
        good = p[:,2]>=0
        pf[good] = part[p[good,2].long()]
        pf[:,:2]=(pf[:,:2]-d['origin_mm'])/(d['raster_mm']*torch.tensor([w,h],device=f.device))
        pf[:,2:4] /= torch.tensor([360,max(1,l-1)],device=f.device); pf[:,5]=(pf[:,5]>=0).float()
        box=d['netz_boxen']; x0,y0,x1,y1=box.unbind(1)
        val=(integ[:,y1,x1]-integ[:,y0,x1]-integ[:,y1,x0]+integ[:,y0,x0]).T
        den=(ci[:,y1,x1]-ci[:,y0,x1]-ci[:,y1,x0]+ci[:,y0,x0]).T.clamp_min(1)
        padnet=torch.cat((padfeat,pf),1)
        sums=f.new_zeros((n,c+7)); counts=f.new_zeros((n,1)); ids=p[:,3].long()
        sums.index_add_(0,ids,padnet); counts.index_add_(0,ids,f.new_ones((len(p),1)))
        pooled=torch.cat((val/den,sums/counts.clamp_min(1)),1)
        nw=d['netz_werte']; known=torch.isfinite(nw[:,1:3]).float()
        nw=torch.nan_to_num(nw,nan=0.)
        nw=torch.cat((nw[:,:1]/6,torch.log1p(nw[:,1:2].clamp_min(0)),
                      torch.log1p(nw[:,2:3].clamp_min(0)/1e6),(nw[:,3:4]>=0).float(),
                      torch.log1p(nw[:,4:6].clamp_min(0)),known,d['global_werte'][2].expand(n,1)/8),1)
        return torch.cat((pooled,nw),1)

    def forward(self, inputs):
        if not isinstance(inputs,list): raise TypeError('Batch als Liste expliziter Eingänge')
        if not inputs: return []


        groups={}; feature_rows={}
        for i,d in enumerate(inputs):
            hw=tuple(((v+15)//16)*16 for v in d['eingang'].shape[-2:])
            groups.setdefault(hw,[]).append((i,d))
        for hw, group in groups.items():
            features=self.encoder([self.vorbereiten(d,hw) for _,d in group]);off=0
            for i,d in group:
                l,_,h,w=d['eingang'].shape
                feature_rows[i]=features[off:off+l,:,:h,:w];off+=l
        out=[]
        for i,d in enumerate(inputs):
            f=feature_rows[i]
            m=d['eingang'][:,0:1]; g=(f*m).sum((0,2,3))/m.sum().clamp_min(1)
            graph=self.pool_netze(f,d)
            graph_g=graph.mean(0) if len(graph) else f.new_zeros(2*f.shape[1]+16)
            q=self.global_kopf(torch.cat((g,d['global_werte']/8,graph_g)))
            net=self.netz_kopf(graph); maps=self.karte(f)
            out.append(dict(ueberlast_mu=maps[:,0], ueberlast_logvar=maps[:,1].clamp(-8,8),
                            gueltig_logit=q[0], note_mu=q[1], note_logvar=q[2].clamp(-8,8),
                            physik_mu=q[3:7], physik_logvar=q[7:11].clamp(-8,8),
                            netz_logit=net[:,0], netz_physik_mu=net[:,1:5], netz_physik_logvar=net[:,5:9].clamp(-8,8)))
        return out


def ziel_physik(t):
    from lernen.ki.daten import X_SPALTEN
    ids=[X_SPALTEN.index(k) for k in ('dc','j','rueck','xt_opfer','xt_stoer')]
    x=t['physik_x'][:,ids].float(); m=t['physik_maske'][:,ids].bool() & torch.isfinite(x) & (x>=0)

    y=torch.cat((x[:,:3],x[:,3:5].amax(1,keepdim=True)),1)
    mask=torch.cat((m[:,:3],m[:,3:5].all(1,keepdim=True)),1)
    return y, mask


def ziel_physik_global(t):
    y,m=ziel_physik(t);kinds=t['netz_werte'][:,0].long()
    req=torch.stack(((kinds==2)|(kinds==3),(kinds==2)|(kinds==3),kinds==1,(kinds==1)|(kinds==4)),1)
    valid=req.any(0)&(m|~req).all(0)
    values=torch.where(req&m,y,torch.full_like(y,float('-inf'))).amax(0) if len(y) else torch.full((4,),float('nan'))
    return torch.where(valid,values,torch.full_like(values,float('nan'))),valid


def verlust(p, sample):
    t=sample['tensoren']; dev=p['note_mu'].device; losses={}
    def gaussian(name, mu, lv, y, mask):
        y=y.to(dev).float(); mask=mask.to(dev).bool() & torch.isfinite(y) & (y>=0)
        if mask.any():
            e=mu[mask]-torch.log1p(y[mask]); v=lv[mask]
            losses[name]=(.5*(torch.exp(-v)*e.square()+v)).mean()
    def binary(name,logit,y,mask):
        mask=mask.to(dev).bool();y=y.to(dev).float()
        if mask.any(): losses[name]=F.binary_cross_entropy_with_logits(logit[mask],y[mask])
    if 'ziel_raster' in t:
        gaussian('ueberlast',p['ueberlast_mu'],p['ueberlast_logvar'],t['ziel_raster'][:,0],t['ziel_raster_maske'][:,0])
    binary('gueltig',p['gueltig_logit'].reshape(1),t['global_ziel'][:1],t['global_maske'][:1])
    gaussian('note',p['note_mu'].reshape(1),p['note_logvar'].reshape(1),t['global_ziel'][1:2],t['global_maske'][1:2])
    if 'netz_verbunden' in t:
        binary('netz',p['netz_logit'],t['netz_verbunden'],t['netz_verbunden_maske'])
    if 'physik_x' in t:
        y,m=ziel_physik(t); gaussian('netz_physik',p['netz_physik_mu'],p['netz_physik_logvar'],y,m)
        gy,gm=ziel_physik_global(t)
        for j in range(4):
            gaussian('physik_'+PHYSIK[j],p['physik_mu'][j:j+1],p['physik_logvar'][j:j+1],gy[j:j+1],gm[j:j+1])
    total=sum(losses.values(),p['note_mu']*0)
    return total, {k:float(v.detach()) for k,v in losses.items()}


def konfiguration(model): return asdict(model.cfg)
