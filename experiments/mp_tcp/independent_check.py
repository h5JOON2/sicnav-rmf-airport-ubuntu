"""Select a control using original model constraints, independently of solver t/status."""
from dataclasses import dataclass
import math
import numpy as np

@dataclass(frozen=True)
class Check:
    eligible: bool
    max_violation: float
    initial_state: float
    dynamics: float
    inequalities: float
    soft_slack: float
    reason: str = ''

def validate_candidate(data, metadata, stage_h, terminal_h, dynamics, tolerance=0.05):
    """Validate all shooting nodes; data keys: x_i, u_i, sl_i, su_i, p_i, lbx0, ubx0."""
    n = int(metadata['N'])
    try:
        for value in data.values():
            if not np.isfinite(np.asarray(value, dtype=float)).all():
                return Check(False, math.inf, math.inf, math.inf, math.inf, math.inf, 'nonfinite candidate')
        first=np.asarray(data['x_0']).ravel()[metadata['idxbx0']]
        init=max(0., float(np.max(np.asarray(data['lbx0'])-first)), float(np.max(first-np.asarray(data['ubx0']))))
        dv=iv=sv=0.
        for i in range(n+1):
            x=np.asarray(data[f'x_{i}']).ravel();p=np.asarray(data[f'p_{i}']).ravel()
            sl=np.asarray(data[f'sl_{i}']).ravel();su=np.asarray(data[f'su_{i}']).ravel()
            if i<n:
                u=np.asarray(data[f'u_{i}']).ravel();h=np.asarray(stage_h(x,u,p),dtype=float).ravel()
                pred=np.asarray(dynamics(x,u,p),dtype=float).ravel()
                if not np.isfinite(pred).all():raise ValueError('nonfinite model dynamics')
                dv=max(dv,float(np.max(abs(np.asarray(data[f'x_{i+1}']).ravel()-pred))))
                lo=np.asarray(metadata['lh']);hi=np.asarray(metadata['uh']);idx=metadata['idxsh']
            else:
                h=np.asarray(terminal_h(x,p),dtype=float).ravel();lo=np.asarray(metadata['lh_e']);hi=np.asarray(metadata['uh_e']);idx=metadata['idxsh_e']
            if not np.isfinite(h).all():raise ValueError('nonfinite original constraint')
            if len(sl)!=len(idx) or len(su)!=len(idx):raise ValueError('soft slack shape mismatch')
            low=h-lo;high=hi-h
            low=low.copy();high=high.copy();low[idx]+=sl;high[idx]+=su
            iv=max(iv,float(np.max(-low)),float(np.max(-high)))
            if sl.size:sv=max(sv,float(np.max(-sl)),float(np.max(-su)))
        worst=max(init,dv,iv,sv)
        return Check(worst<=tolerance,worst,init,dv,iv,sv)
    except Exception as exc:
        return Check(False,math.inf,math.inf,math.inf,math.inf,math.inf,str(exc))

def select_output(final_check, final_u0, initial_check, initial_u0, dt):
    """Return None when neither trajectory meets the direct feasibility tolerance."""
    for name,check,u in [('final',final_check,final_u0),('initial',initial_check,initial_u0)]:
        if check.eligible:
            u=np.asarray(u,dtype=float).ravel()
            if len(u)>=2 and np.isfinite(u[:2]).all():
                return {'source':name,'action':[float(u[0]),float(u[1]*dt)]}
    return None
