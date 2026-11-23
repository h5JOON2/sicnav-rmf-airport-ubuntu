"""Pinned upstream DR-SPAAM network, explicit CPU/MPS inference (no YOLO)."""
import math
import sys
from pathlib import Path
import numpy as np
import torch
from .lidar import prepare_drow_scan,drow_xy_to_ros


class DrSpaamDetector:
    def __init__(self,source,checkpoint,device='mps',confidence=.5,stride=1):
        if device not in ('cpu','mps'):raise ValueError('device must be cpu or mps')
        if device=='mps' and not torch.backends.mps.is_available():raise RuntimeError('MPS unavailable; choose CPU explicitly')
        if not 0<=confidence<=1 or stride<1:raise ValueError('Invalid detector parameters')
        source=Path(source).resolve()
        if not(source/'dr_spaam/model/dr_spaam.py').is_file():raise RuntimeError('DR-SPAAM source missing; run prepare_dr_spaam.py')
        sys.path.insert(0,str(source))
        from dr_spaam.model.dr_spaam import DrSpaam
        from dr_spaam.utils import utils
        self.utils=utils;self.device=device;self.confidence=confidence;self.stride=stride
        self.model=DrSpaam(dropout=.5,num_pts=56,embedding_length=128,alpha=.5,
                           window_size=17,panoramic_scan=True,cls_loss=None,mixup_alpha=0.,mixup_w=0.)
        data=torch.load(checkpoint,map_location='cpu',weights_only=False)
        self.model.load_state_dict(data['model_state'],strict=True)
        self.model.to(device).eval()
        self.signature=None
        self.last_stamp=None
        self.neural_calls=0

    def reset(self):
        self.model.gate.reset()
        self.signature=None;self.last_stamp=None

    def raw(self,scan,fov_degrees):
        signature=(len(scan),round(fov_degrees,7))
        if signature!=self.signature:
            self.model.gate.reset();self.model.gate.neighbor_masks=None
            self.model.gate.neighbor_inds=None;self.signature=signature
        self.model.gate._panoramic_scan=fov_degrees>=359.
        phi=np.linspace(-.5*np.deg2rad(fov_degrees),.5*np.deg2rad(fov_degrees),len(scan),dtype=np.float32)
        cut=self.utils.scans_to_cutout(scan[None,...],phi,stride=self.stride,centered=True,fixed=True,
                                      window_width=1.,window_depth=.5,num_cutout_pts=56,padding_val=29.99,area_mode=True)
        x=torch.from_numpy(cut).float().to(self.device)
        with torch.inference_mode():
            cls,reg,_=self.model(x.unsqueeze(0),inference=True)
            if cls.device.type!=self.device:raise RuntimeError('Unexpected detector device')
            scores=torch.sigmoid(cls[0]).cpu().numpy()
            reg=reg[0].cpu().numpy()
        if self.device=='mps':torch.mps.synchronize()
        self.neural_calls+=1
        return self.utils.nms_predicted_center(scan[::self.stride],phi[::self.stride],scores[:,0],reg)

    def detect(self,ranges,header):
        stamp=float(header['stamp'])
        if self.last_stamp is not None and (stamp<=self.last_stamp or stamp-self.last_stamp>1.):self.reset()
        self.last_stamp=stamp
        scan,fov,center=prepare_drow_scan(ranges,float(header['angle_min']),float(header['angle_increment']),
                                        float(header['range_min']),float(header['range_max']))
        xy,scores,_=self.raw(scan,fov)
        xy=drow_xy_to_ros(xy,center)
        distance=np.linalg.norm(xy,axis=1)
        keep=(np.isfinite(xy).all(axis=1)&np.isfinite(scores)&(scores>=self.confidence)&
              (distance>=header['range_min'])&(distance<=header['range_max']))
        return xy[keep],scores[keep]
