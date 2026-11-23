import math
import numpy as np
from scipy.optimize import linear_sum_assignment


class PersonSlots:
    """Small fixed slot set matches the legacy solver's index-based histories."""
    def __init__(self, count=3, expiry=1.,retain_missing=True):
        self.count, self.expiry = count, expiry
        self.retain_missing=retain_missing
        self.slots = [None]*count
        self.next_id = 1
        self.stamp = None

    def update(self, stamp, points):
        # Repeated simulator timestamps must not allocate a second identity for
        # the same observation (matching below requires strictly positive dt).
        if self.stamp is not None and stamp == self.stamp:
            return [dict(v) if v else None for v in self.slots]
        if self.stamp is not None and stamp < self.stamp:
            self.slots = [None]*self.count
        self.stamp = stamp
        for i, old in enumerate(self.slots):
            if old and stamp-old['stamp'] > self.expiry:
                self.slots[i] = None
        active=[i for i,v in enumerate(self.slots) if v]
        used=set()
        if active and points:
            costs=np.array([[math.dist([self.slots[i]['px']+self.slots[i]['vx']*(stamp-self.slots[i]['stamp']),
                                        self.slots[i]['py']+self.slots[i]['vy']*(stamp-self.slots[i]['stamp'])],p[:2])
                             for p in points] for i in active])
            rr,cc=linear_sum_assignment(costs)
            for r,c in zip(rr,cc):
                i=active[r];old=self.slots[i];dt=stamp-old['stamp']
                if costs[r,c] > .8+min(1.,2*max(0.,dt)) or dt<=0:
                    continue
                vx,vy=(points[c][0]-old['px'])/dt,(points[c][1]-old['py'])/dt
                speed=math.hypot(vx,vy)
                if speed>3.:vx,vy=vx*3/speed,vy*3/speed
                self.slots[i]={**old,'px':points[c][0],'py':points[c][1],
                               'vx':.5*old['vx']+.5*vx,'vy':.5*old['vy']+.5*vy,'stamp':stamp}
                used.add(c)
        if not self.retain_missing:
            self.slots=[v if v and v['stamp']==stamp else None for v in self.slots]
        for c,p in enumerate(points):
            if c in used:continue
            empty=next((i for i,v in enumerate(self.slots) if v is None),None)
            if empty is None:break
            self.slots[empty]={'id':self.next_id,'px':p[0],'py':p[1],'vx':0.,'vy':0.,'radius':.25,'stamp':stamp}
            self.next_id+=1
        return [dict(v) if v else None for v in self.slots]


def observation_compatible(observation, ids):
    if observation is None or observation.get('capacity_exceeded', True):
        return False
    slots = observation.get('slots', [])
    if observation.get('variable_humans'):
        visible=[value for value in slots if value is not None and value['stamp']==observation['stamp']]
        return tuple(value['id'] for value in visible)==tuple(ids)
    return all(value is not None for value in slots) and tuple(value['id'] for value in slots) == tuple(ids)
