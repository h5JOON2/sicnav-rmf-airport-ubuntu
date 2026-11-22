"""Bounded local recovery; independent prediction checks, NOT an MPC solution."""
from dataclasses import dataclass
import math
import numpy as np

@dataclass(frozen=True)
class RecoveryLimits:
    dt: float = .25
    horizon: float = 2.0
    integration_dt: float = .025
    robot_radius: float = .25
    human_clearance: float = .30
    wall_clearance: float = .08
    forward_speed: float = .95
    reverse_speed: float = .95
    acceleration: float = .5
    deceleration: float = 1.5
    angular_speed: float = math.pi / 3
    angular_acceleration: float = 3.0

def velocity_window(v, dt, acceleration, deceleration, forward, reverse):
    """Respect braking to zero before accelerating in the opposite direction."""
    if v >= 0:
        upper = min(forward, v + acceleration * dt)
        lower = v - deceleration * dt if v > deceleration * dt else -min(reverse, acceleration * (dt-v/deceleration))
    else:
        lower = max(-reverse, v - acceleration * dt)
        upper = v + deceleration * dt if -v > deceleration * dt else min(forward, acceleration * (dt+v/deceleration))
    return lower, upper

def _point_segment(p, a, b):
    d = b-a
    t = np.clip(np.sum((p-a)*d,axis=-1)/np.maximum(np.sum(d*d,axis=-1),1e-30),0,1)
    return np.linalg.norm(p-(a+t[...,None]*d),axis=-1)

def _cross(a,b):return a[...,0]*b[...,1]-a[...,1]*b[...,0]

def swept_wall_gap(paths, walls, radius):
    if len(walls)==0:return np.full((len(paths),paths.shape[1]-1,0),np.inf)
    a,b=paths[:,:-1,None,:2],paths[:,1:,None,:2]
    c,d=walls[None,None,:,0,:],walls[None,None,:,1,:]
    distance=np.minimum(np.minimum(_point_segment(a,c,d),_point_segment(b,c,d)),np.minimum(_point_segment(c,a,b),_point_segment(d,a,b)))
    r,s=b-a,d-c;den=_cross(r,s);safe=np.where(abs(den)>1e-15,den,1.)
    t=_cross(c-a,s)/safe;u=_cross(c-a,r)/safe
    intersect=(abs(den)>1e-15)&(t>=0)&(t<=1)&(u>=0)&(u<=1)
    return np.where(intersect,0.,distance)-radius

def swept_human_gap(paths, humans, times, radius):
    if len(humans)==0:return np.full((len(paths),len(times)-1,0),np.inf)
    positions=humans[None,:,:2]+times[:,None,None]*humans[None,:,2:4]
    rel=positions[None,:,:,:]-paths[:,:,None,:2]
    start=rel[:,:-1];delta=np.diff(rel,axis=1)
    fraction=np.clip(-np.sum(start*delta,axis=-1)/np.maximum(np.sum(delta*delta,axis=-1),1e-30),0,1)
    return np.linalg.norm(start+fraction[...,None]*delta,axis=-1)-radius-humans[None,None,:,4]

class RecoveryPlanner:
    def __init__(self,limits=RecoveryLimits()):
        self.limits=limits

    def plan(self,robot,humans,walls):
        """robot: x,y,yaw,signed_v,omega,gx,gy; humans: x,y,vx,vy,radius."""
        lim=self.limits;r=np.asarray(robot,dtype=float);h=np.asarray(humans,dtype=float).reshape(-1,5);walls=np.asarray(walls,dtype=float).reshape(-1,2,2)
        if r.shape!=(7,)or not np.isfinite(r).all()or not np.isfinite(h).all()or not np.isfinite(walls).all():
            raise ValueError('finite robot, human and wall observations required')
        if np.any(h[:,4]<=0):raise ValueError('positive human radii required')
        lower,upper=velocity_window(r[3],lim.dt,lim.acceleration,lim.deceleration,lim.forward_speed,lim.reverse_speed)
        wl=max(-lim.angular_speed,r[4]-lim.angular_acceleration*lim.dt);wu=min(lim.angular_speed,r[4]+lim.angular_acceleration*lim.dt)
        if lower>upper or wl>wu:return self._stop('no_reachable_velocity')
        vs=np.unique(np.r_[np.linspace(lower,upper,7),np.clip(0,lower,upper)])
        ws=np.unique(np.r_[np.linspace(wl,wu,9),np.clip(0,wl,wu)])
        v,w=np.meshgrid(vs,ws,indexing='ij');commands=np.c_[v.ravel(),w.ravel()]
        commands=commands[(abs(commands[:,0])>.005)|(abs(commands[:,1])>.01)]
        if len(commands)==0:return self._stop('no_moving_candidate')
        times=np.linspace(0,lim.horizon,round(lim.horizon/lim.integration_dt)+1)
        paths=np.zeros((len(commands),len(times),3));paths[:,0]=r[:3]
        for i in range(len(times)-1):
            t=(times[i]+times[i+1])*.5;fraction=min(t/lim.dt,1.)
            vm=r[3]+fraction*(commands[:,0]-r[3]);wm=r[4]+fraction*(commands[:,1]-r[4]);dt=times[i+1]-times[i]
            angle=paths[:,i,2]+wm*dt*.5
            paths[:,i+1,0]=paths[:,i,0]+vm*np.cos(angle)*dt;paths[:,i+1,1]=paths[:,i,1]+vm*np.sin(angle)*dt;paths[:,i+1,2]=paths[:,i,2]+wm*dt
        hg=swept_human_gap(paths,h,times,lim.robot_radius);wg=swept_wall_gap(paths,walls,lim.robot_radius)
        safe=np.ones(len(paths),dtype=bool);restoring=np.zeros(len(paths),dtype=bool)
        minima=[]
        for gaps,target in [(hg,lim.human_clearance),(wg,lim.wall_clearance)]:
            if gaps.shape[-1]==0:continue
            if gaps is hg:
                initial=np.linalg.norm(h[:,:2]-r[:2],axis=1)-lim.robot_radius-h[:,4]
            else:
                initial=_point_segment(r[:2],walls[:,0,:],walls[:,1,:])-lim.robot_radius
            # Existing contact cannot be certified collision-free by this predictor.
            if np.any(initial<=0):return self._stop('current_body_overlap')
            safe_at_start=initial>=target-1e-9
            if np.any(safe_at_start):safe &= np.all(gaps[:,:,safe_at_start]>=target-1e-9,axis=(1,2))
            if np.any(~safe_at_start):
                small=gaps[:,:,~safe_at_start]
                # Keep each already-small clearance from decreasing; require recovery.
                safe &= np.all(small>0,axis=(1,2))
                safe &= np.all(small[:,0]>=initial[~safe_at_start]-1e-7,axis=1)
                safe &= np.all(np.diff(small,axis=1)>=-1e-7,axis=(1,2))
                safe &= np.all(small[:,-1]>=np.minimum(target,initial[~safe_at_start]+.01)-1e-7,axis=1)
                restoring[:]=True
            minima.append(np.min(gaps,axis=(1,2)))
        if not np.any(safe):return self._stop('no_prediction_checked_motion',len(commands))
        end=paths[:,-1];goal=np.array(r[5:7]);distance=np.linalg.norm(end[:,:2]-goal,axis=1)
        bearing=np.arctan2(goal[1]-end[:,1],goal[0]-end[:,0]);heading=abs(np.arctan2(np.sin(bearing-end[:,2]),np.cos(bearing-end[:,2])))
        score=distance+.20*heading+.03*abs(commands[:,1])+.02*abs(commands[:,0]-r[3])
        if minima:
            clearance=np.min(np.stack(minima),axis=0)
            score-=np.where(restoring,2.,.05)*np.minimum(clearance,.5)
        score[~safe]=np.inf;best=int(np.argmin(score));v,w=commands[best]
        def value(g):return float(np.min(g[best]))if g.shape[-1]else None
        return {'available':True,'action':[float(v),float(w)],'kind':'clearance_restoration'if restoring[best]else'local_avoidance',
                'score':float(score[best]),'candidate_count':len(commands),'accepted_count':int(safe.sum()),
                'predicted_human_gap_m':value(hg),'predicted_wall_gap_m':value(wg),
                'human_target_m':lim.human_clearance,'wall_target_m':lim.wall_clearance,
                'prediction':'constant_velocity_humans; ramped_unicycle_robot; swept_segments',
                'horizon_s':lim.horizon,'mpc_constraints_validated':False}
    def _stop(self,reason,count=0):
        return {'available':False,'action':[0.,0.],'kind':'emergency_stop','reason':reason,'candidate_count':count,'accepted_count':0,'mpc_constraints_validated':False}
