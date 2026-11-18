"""MP clearance30 solver adapter, mechanically ported from its verified worker.
One controller per process. Original MP files are never modified.
"""
import os, sys, json, time, ctypes
from pathlib import Path
from copy import deepcopy
from types import SimpleNamespace
import numpy as np
import casadi as cs
from clearance30_gate import min_body_clearance

ROOT = Path(os.environ.get('SICNAV_MP_ROOT', Path(__file__).resolve().parents[2]))
LIB_DIR = Path(os.environ.get('SICNAV_MP_NATIVE', str(Path(__file__).parent / 'native')))
MODEL_DIR = None
RUN_DIR = None
policy = None
current = {}
solver_info = {}
A = SimpleNamespace(merit=False,init_slacks=False,cold_qp=False,eq_tol=0.)
original_factory = None

def install(target_policy, model_dir, run_dir):
 global policy, MODEL_DIR, RUN_DIR, original_factory
 import sicnav.utils.mpc_utils.sicnav_acados.campc_acados_opt as opt
 from acados_template import AcadosOcpSolver
 policy = target_policy
 MODEL_DIR = Path(model_dir).resolve(); MODEL_DIR.mkdir(parents=True,exist_ok=True)
 RUN_DIR = Path(run_dir).resolve(); RUN_DIR.mkdir(parents=True,exist_ok=True)
 original_factory = AcadosOcpSolver
 opt.AcadosOcpSolver = factory

def factory(ocp,*x,**kw):
   N = int(ocp.solver_options.N_horizon) if hasattr(ocp.solver_options, "N_horizon") else int(ocp.dims.N)
   ocp.solver_options.nlp_solver_max_iter=50;ocp.solver_options.qp_solver_iter_max=15
   import acados_template.acados_ocp_solver as solver_module
   old_loader=solver_module.DllLoader
   def isolated_loader(path,*args,**kwargs):
    if str(path).endswith('/libacados.dylib') and os.environ.get('SWEEP_RANK_LIB'):
     path=os.environ['SWEEP_RANK_LIB']
    return old_loader(path,*args,**kwargs)
   solver_module.DllLoader=isolated_loader
   # h_old = R_old^2 - distance^2; upper_new = -(R_new^2-R_old^2).
   # This enforces a total 0.30m body gap, not the old buffer plus 0.30m.
   person_rows=[];person_deltas=[]
   for names,bound in [(policy.mpc_env.all_state_names,ocp.constraints.uh),(policy.mpc_env.all_term_names,ocp.constraints.uh_e)]:
    for row,name in enumerate(names):
     if name.startswith('coll_const_rob_hum'):
      hi=int(name.split('hum',1)[1].split('_',1)[0]);rh=float(policy.mpc_env.callback_orca.human_radii[hi]);rr=float(policy.mpc_env.radius);old_r=rr+rh+policy.mpc_env.rob_len_buffer+.01;new_r=rr+rh+.30;bound[row]-=new_r**2-old_r**2
      if names is policy.mpc_env.all_state_names:person_rows.append(row);person_deltas.append(new_r**2-old_r**2)
   model_dir=Path(str(MODEL_DIR / 'generated_jmid'));model_dir.mkdir(parents=True,exist_ok=True)
   ocp.model.name='sicnav_arena10_jmid';ocp.code_export_directory=str(model_dir);ocp.solver_options.ext_fun_compile_flags='-O0'
   model_json=str(model_dir.parent/'jmid_solver.json');needs_build=not(model_dir/'libacados_ocp_solver_sicnav_arena10_jmid.dylib').exists()
   solver=original_factory(ocp,json_file=model_json,generate=needs_build,build=needs_build);original_solve=solver.solve;solver.print_statistics=lambda:None
   kw=dict(json_file=model_json,generate=False,build=False);x=()
   NX=int(ocp.model.x.numel());NU=int(ocp.model.u.numel());NH=policy.mpc_env.num_hums;NL=policy.mpc_env.nlambda_hum;NW=policy.mpc_env.num_MID_samples;LW_START=10+6*NH

   native_opts=ctypes.CDLL(os.environ.get('SWEEP_RANK_LIB',str(ROOT/'work/acados/lib/libacados.dylib')))
   native_opts.ocp_nlp_solver_opts_set.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_char_p,ctypes.c_void_p]
   for field,key,default in [('max_iter','SWEEP_SQP','50'),('qp_iter_max','SWEEP_QP','15')]:
    value=ctypes.c_int(int(os.environ.get(key,default)))
    native_opts.ocp_nlp_solver_opts_set(solver.nlp_config,solver.nlp_opts,field.encode(),ctypes.byref(value))
   if os.environ.get('ORCA_LM'):
    lm_value=ctypes.c_double(float(os.environ['ORCA_LM']));native_opts.ocp_nlp_solver_opts_set(solver.nlp_config,solver.nlp_opts,b'levenberg_marquardt',ctypes.byref(lm_value))
   if A.merit:
    solver.options_set('globalization','merit_backtracking');solver.options_set('line_search_use_sufficient_descent',1);solver.options_set('full_step_dual',0)
   if A.init_slacks:solver.options_set('initialize_t_slacks',1)
   if A.cold_qp:solver.options_set('qp_warm_start',0)
   if A.eq_tol>0:
    lo=ocp.constraints.lh.copy();hi=ocp.constraints.uh.copy();eq=np.isclose(lo,hi,rtol=0,atol=0);lo[eq]-=A.eq_tol;hi[eq]+=A.eq_tol
    for stage in range(N):solver.constraints_set(stage,'lh',lo);solver.constraints_set(stage,'uh',hi)
   solver.constraints_set(N,'lh',ocp.constraints.lh_e);solver.constraints_set(N,'uh',ocp.constraints.uh_e)
   native_lib=ctypes.CDLL(os.environ.get('SWEEP_RANK_LIB',str(ROOT/'work/acados/lib/libacados.dylib')))
   native_rank=getattr(native_lib,'sicnav_qp_rank_stats',None)
   if native_rank:native_rank.argtypes=[ctypes.POINTER(ctypes.c_int)];native_rank.restype=None
   lo_original=ocp.constraints.lh.copy();hi_original=ocp.constraints.uh.copy()
   comp_indices=[i for i,name in enumerate(policy.mpc_env.all_state_names) if 'eq_comp_slack' in name]
   grad_indices=[i for i,name in enumerate(policy.mpc_env.all_state_names) if 'eq_grad_Lagr' in name]
   relaxation=os.environ.get('ORCA_RELAX_MODE','exact');mu=float(os.environ.get('ORCA_RELAX_MU','0'))
   lo=lo_original.copy();hi=hi_original.copy()
   targets=comp_indices+grad_indices if relaxation=='optimality_band' else comp_indices
   if relaxation!='exact':
    lo[targets]-=mu
    hi[targets]+=mu if relaxation!='comp_lower' else 1e6
   for stage in range(N):solver.constraints_set(stage,'lh',lo);solver.constraints_set(stage,'uh',hi)
   solver.constraints_set(N,'lh',ocp.constraints.lh_e);solver.constraints_set(N,'uh',ocp.constraints.uh_e)
   # Bound changes only: retain primal/dual feasibility, original h expressions and costs.
   hp_lib=ctypes.CDLL(str(ROOT/'work/acados/lib/libhpipm.dylib'))
   shim=ctypes.CDLL(str(LIB_DIR/'libqp_snapshot.dylib'))
   native_lib.ocp_nlp_get.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_char_p,ctypes.c_void_p]
   shim.snapshot_dense_dims.argtypes=[ctypes.c_void_p,ctypes.c_void_p]
   def dense_check():
    ptr=ctypes.c_void_p();native_lib.ocp_nlp_get(solver.nlp_config,solver.nlp_solver,b'qp_xcond_in',ctypes.byref(ptr))
    dims=np.zeros(5,dtype=np.int32);shim.snapshot_dense_dims(ptr,dims.ctypes.data);nv,ne,nb,ng,ns=map(int,dims)
    data={}
    for field,shape in dict(C=(ng,nv),A=(ne,nv),b=(ne,),lg=(ng,),ug=(ng,),lg_mask=(ng,),ug_mask=(ng,),idxs_rev=(nb+ng,)).items():
     arr=np.zeros(shape,dtype=np.int32 if field=='idxs_rev' else np.float64,order='F');fun=getattr(hp_lib,'d_dense_qp_get_'+field);fun.argtypes=[ctypes.c_void_p,ctypes.c_void_p]
     if arr.size:fun(ptr,arr.ctypes.data)
     data[field]=arr
    G=data['C'];l=data['lg'];u=data['ug'];hard=data['idxs_rev'][nb:]<0;both=(data['lg_mask']>0)&(data['ug_mask']>0);eq=hard&both&(abs(l-u)<1e-10);norm=np.linalg.norm(G,axis=1);zero=norm<1e-12;E=np.vstack([data['A'],G[eq]]);en=np.maximum(np.linalg.norm(E,axis=1),1e-15);sing=np.linalg.svd(E/en[:,None],compute_uv=False);rank=int(sum(sing>1e-9))
    tight=hard&zero&(((data['lg_mask']>0)&(abs(l)<=1e-8))|((data['ug_mask']>0)&(abs(u)<=1e-8)))
    interior=hard&zero&(l< -1e-8)&(u>1e-8)
    result=dict(nv=nv,soft=ns,hard_equalities=int(eq.sum()+ne),equality_rank=rank,equality_deficiency=int(eq.sum()+ne-rank),zero_gradient_equalities=int(sum(eq&zero)),zero_gradient_tight_at_zero_step=int(sum(tight)),zero_gradient_strict_interior_at_zero_step=int(sum(interior)))
    arr=np.zeros((nv,nv),order='F');fun=hp_lib.d_dense_qp_get_H;fun.argtypes=[ctypes.c_void_p,ctypes.c_void_p];fun(ptr,arr.ctypes.data);H=np.tril(arr)+np.tril(arr,-1).T
    _,ss,vh=np.linalg.svd(E/en[:,None],full_matrices=True);rr=int(sum(ss>1e-9));Z=vh[rr:].T
    result.update(hessian_min=float(np.linalg.eigvalsh(H)[0]),reduced_hessian_min=float(np.linalg.eigvalsh(Z.T@H@Z)[0]) if Z.shape[1] else None,nullspace_dimension=Z.shape[1],exact_zero_equality_rows=int(sum(eq&np.all(G==0,axis=1))))
    return result
   def set_budget(value):
    v=ctypes.c_int(value);native_lib.ocp_nlp_solver_opts_set.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_char_p,ctypes.c_void_p]
    native_lib.ocp_nlp_solver_opts_set(solver.nlp_config,solver.nlp_opts,b'max_iter',ctypes.byref(v))
   import casadi as cs
   hf=cs.Function('validate_h',[ocp.model.x,ocp.model.u,ocp.model.p],[ocp.model.con_h_expr])
   he=cs.Function('validate_he',[ocp.model.x,ocp.model.p],[ocp.model.con_h_expr_e])
   jf=cs.Function('validate_jac',[ocp.model.x,ocp.model.u,ocp.model.p],[cs.jacobian(ocp.model.con_h_expr,cs.vertcat(ocp.model.x,ocp.model.u))])
   ff=cs.Function('validate_f',[ocp.model.x,ocp.model.u,ocp.model.p],[ocp.model.disc_dyn_expr])
   def direct_check():
    violations=[];raw=[];dyn=0.;slackmax=0.;slackmin=0.;finite_iterate=True
    for stage in range(N+1):
     xx=solver.get(stage,'x');pp=tracked_p.get(stage,ocp.parameter_values)
     if stage<N:
      uu=solver.get(stage,'u');h=np.array(hf(xx,uu,pp)).ravel();l=lo_original;r=hi_original;idxsh=np.asarray(ocp.constraints.idxsh,dtype=int);names=policy.mpc_env.all_state_names
      dyn=max(dyn,float(np.max(abs(solver.get(stage+1,'x')-np.array(ff(xx,uu,pp)).ravel()))))
     else:
      h=np.array(he(xx,pp)).ravel();l=ocp.constraints.lh_e;r=ocp.constraints.uh_e;idxsh=np.asarray(ocp.constraints.idxsh_e,dtype=int);names=policy.mpc_env.all_term_names
     if not np.isfinite(h).all():raise RuntimeError('nonfinite original constraint')
     if not np.isfinite(solver.get(stage,'x')).all():raise RuntimeError('nonfinite candidate state')
     if stage<N and not np.isfinite(solver.get(stage,'u')).all():raise RuntimeError('nonfinite candidate input')
     sl=solver.get(stage,'sl');su=solver.get(stage,'su');dl=np.zeros(len(h));du=np.zeros(len(h));assert sl.size==idxsh.size;dl[idxsh]=sl;du[idxsh]=su
     if not np.isfinite(np.r_[sl,su]).all():raise RuntimeError('nonfinite soft slack')
     v=np.maximum(l-h-dl,h-r-du);vraw=np.maximum(l-h,h-r)
     if len(h):
      k=int(np.argmax(v));violations.append(dict(stage=stage,row=k,name=names[k],violation=float(max(0,v[k]))));raw.append(float(max(0,np.max(vraw))))
     if sl.size:slackmax=max(slackmax,float(np.max(np.r_[sl,su])));slackmin=min(slackmin,float(np.min(np.r_[sl,su])))
    idx=np.asarray(ocp.constraints.idxbx_0,dtype=int);xx=solver.get(0,'x')[idx];initial_violation=0.
    if (0,'lbx') in tracked_bounds:initial_violation=max(initial_violation,float(np.max(tracked_bounds[(0,'lbx')]-xx)))
    if (0,'ubx') in tracked_bounds:initial_violation=max(initial_violation,float(np.max(xx-tracked_bounds[(0,'ubx')])))
    if not np.isfinite(dyn):raise RuntimeError('nonfinite dynamics')
    worst=max(violations,key=lambda x:x['violation']);return dict(finite=True,initial_state_violation=initial_violation,dynamics_violation=dyn,softened_nonlinear_violation=worst['violation'],raw_nonlinear_violation=max(raw),worst=worst,slack_max=slackmax,slack_min=slackmin)
   def kkt_check():
    compmax=0.;gradmax=0.;zero_comp=0;zero_active=0;relax_violation=0.;hard_eq_total=0;hard_rank_sum=0;pdmax=0.
    pdidx=[i for i,name in enumerate(policy.mpc_env.all_state_names) if 'ineq_con_func' in name]
    hard=np.ones(len(lo),dtype=bool);hard[np.asarray(ocp.constraints.idxsh,dtype=int)]=False
    for stage in range(N):
     xx=solver.get(stage,'x');uu=solver.get(stage,'u');pp=tracked_p.get(stage,ocp.parameter_values);h=np.array(hf(xx,uu,pp)).ravel();J=np.array(jf(xx,uu,pp));cn=np.linalg.norm(J[comp_indices],axis=1);cv=h[comp_indices]
     pdmax=max(pdmax,float(max(0,np.max(lo_original[pdidx]-h[pdidx]),np.max(h[pdidx]-hi_original[pdidx]))))
     compmax=max(compmax,float(np.max(abs(cv))));gradmax=max(gradmax,float(np.max(abs(h[grad_indices]))));z=cn<1e-12;zero_comp+=int(sum(z));active=(abs(cv-lo[comp_indices])<1e-8)|(abs(cv-hi[comp_indices])<1e-8);zero_active+=int(sum(z&active))
     relax_violation=max(relax_violation,float(max(0,np.max(lo[targets]-h[targets]),np.max(h[targets]-hi[targets]))))
     eq=hard&(abs(lo-hi)<1e-10);E=J[eq];norm=np.maximum(np.linalg.norm(E,axis=1),1e-15);rank=int(sum(np.linalg.svd(E/norm[:,None],compute_uv=False)>1e-9));hard_eq_total+=int(sum(eq));hard_rank_sum+=rank
    return dict(primal_dual_violation=pdmax,comp_max=compmax,stationarity_equation_max=gradmax,zero_gradient_comp=zero_comp,zero_gradient_active_comp=zero_active,relaxed_optimality_violation=relax_violation,stage_hard_equalities=hard_eq_total,stage_hard_equality_rank_sum=hard_rank_sum)
   def safe_diagnostic(fn):
    try:return fn()
    except Exception as e:return dict(diagnostic_error=str(e),finite=False)
   diagnostic_index=0
   trace_library=ctypes.CDLL(str(LIB_DIR / 'qp_observer.dylib'))
   trace_library.qp_observer_begin.argtypes=[ctypes.c_void_p,ctypes.c_void_p];trace_library.qp_observer_begin.restype=ctypes.c_int
   trace_library.qp_observer_finish.argtypes=[];trace_library.qp_observer_finish.restype=None
   stats_reader=ctypes.CDLL(str(LIB_DIR / 'hpipm_trace_stats.dylib'))
   stats_reader.hpipm_trace_stats.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_void_p];stats_reader.hpipm_trace_stats.restype=ctypes.c_int
   cf=cs.Function('pure_cost',[ocp.model.x,ocp.model.u,ocp.model.p],[ocp.model.cost_expr_ext_cost])
   ce=cs.Function('pure_terminal_cost',[ocp.model.x,ocp.model.p],[ocp.model.cost_expr_ext_cost_e])
   trace_dir=Path(str(RUN_DIR));trace_dir.mkdir(exist_ok=True)
   for name,fn in [('h',hf),('he',he),('f',ff),('cost',cf),('terminal_cost',ce)]:fn.save(str(trace_dir/(name+'.casadi')))
   meta=dict(N=N,dt=.25,nx=NX,nu=NU,humans=NH,lambda_per_human=NL,body_clearance_m=.30,clearance_gate='candidate and nonlinear rollout swept segments',lh=lo_original.tolist(),uh=hi_original.tolist(),lh_e=ocp.constraints.lh_e.tolist(),uh_e=ocp.constraints.uh_e.tolist(),idxsh=ocp.constraints.idxsh.tolist(),idxsh_e=ocp.constraints.idxsh_e.tolist(),names=policy.mpc_env.all_state_names,terminal_names=policy.mpc_env.all_term_names)
   (trace_dir/'model.json').write_text(json.dumps(meta,indent=2))
   def pure_cost():
    total=0.
    for i in range(N+1):
     xx=solver.get(i,'x');pp=tracked_p.get(i,ocp.parameter_values);sl=solver.get(i,'sl');su=solver.get(i,'su')
     if i<N:
      value=float(cf(xx,solver.get(i,'u'),pp));Zl=ocp.cost.Zl;Zu=ocp.cost.Zu;zl=ocp.cost.zl;zu=ocp.cost.zu;scale=.25
     else:
      value=float(ce(xx,pp));Zl=ocp.cost.Zl_e;Zu=ocp.cost.Zu_e;zl=ocp.cost.zl_e;zu=ocp.cost.zu_e;scale=1.
     value+=float(.5*np.dot(Zl,sl*sl)+.5*np.dot(Zu,su*su)+zl@sl+zu@su)
     total+=scale*value
    return total
   pass # package-local geometry helper
   from clearance30_gate import min_body_clearance
   fallback_models={}
   result_dir=Path(str(RUN_DIR));result_dir.mkdir(exist_ok=True)
   prox_indices=[i for i,name in enumerate(policy.mpc_env.all_state_names)if 'hums_close_to_preds' in name]
   assert len(prox_indices)==NH
   def fallback_model():
    if fallback_models:return fallback_models['ocp'],fallback_models['ff']
    mo=deepcopy(ocp);xx=mo.model.x;uu=mo.model.u;pp=mo.model.p;oldf=mo.model.disc_dyn_expr
    parts=[oldf[k]for k in range(NX)]
    # Xr human velocity entries are unused by the robot-only quadratic cost.
    # Supply observed current vx/vy there, constant over this MPC solve.
    for hi in range(NH):
     off=10+6*hi
     parts[off+4]=oldf[off]+.25*pp[off+2]
     parts[off+5]=oldf[off+1]+.25*pp[off+3]
    for k in range(LW_START,NX):parts[k]=xx[k]
    mo.model.disc_dyn_expr=cs.vertcat(*parts)
    hh=[mo.model.con_h_expr[k]if k not in prox_indices else cs.MX(0)for k in range(mo.model.con_h_expr.numel())]
    mo.model.con_h_expr=cs.vertcat(*hh)
    mo.model.name='sicnav_arena10_orca'
    mo.code_export_directory=str(MODEL_DIR / 'generated_orca')
    # Same dimensions, exact Hessian, project regularizer and installed HPIPM backend.
    mo.solver_options.nlp_solver_max_iter=50;mo.solver_options.qp_solver_iter_max=15
    fmode=cs.Function('orca_observed_f',[xx,uu,pp],[mo.model.disc_dyn_expr])
    # JMID sample parameters have no influence on the fallback dynamics/constraints.
    assert cs.jacobian(mo.model.disc_dyn_expr,pp)[:,3*NX+2*NU:].is_zero()
    assert cs.jacobian(mo.model.con_h_expr,pp)[:,3*NX+2*NU:].is_zero()
    modelpath=Path(str(MODEL_DIR / 'orca_solver.json'))
    needs_build=not(Path(mo.code_export_directory)/'libacados_ocp_solver_sicnav_arena10_orca.dylib').exists()
    ss=original_factory(mo,json_file=str(modelpath),generate=needs_build,build=needs_build)
    del ss
    fmode.save(str(result_dir/'observed_f.casadi'))
    fallback_models.update(ocp=mo,ff=fmode,json=str(modelpath))
    return mo,fmode
   fallback_model() # Prebuild before the timed simulation, not during a control tick.
   def solve():
    nonlocal solver
    if 'job'not in current:return original_solve()
    import hashlib
    seq=current['job']['seq'];base_solver=solver
    fresh={(i,f):solver.get(i,f).copy()for i in range(N+1)for f in ['x','u','sl','su','lam','t','z']+(['pi']if i<N else[])}
    params={i:v.copy()for i,v in tracked_p.items()};bounds={k:v.copy()for k,v in tracked_bounds.items()}
    digest=hashlib.sha256()
    for k,v in sorted(fresh.items()):digest.update(str(k).encode());digest.update(v.tobytes())
    for k,v in sorted(params.items()):digest.update(str(k).encode());digest.update(v.tobytes())
    fresh_hash=digest.hexdigest();obsvel=np.concatenate([fresh[(0,'x')][12+6*h:14+6*h]for h in range(NH)])
    assert all(np.all(v[NX+NU+10:2*NX+NU]==0)and np.all(v[2*NX+NU+2:2*NX+2*NU]==0)and np.all(v[2*NX+2*NU+10:3*NX+2*NU]==0)for v in params.values())
    def append(name,row):
     def clean(v):
      if isinstance(v,float)and not np.isfinite(v):return None
      if isinstance(v,dict):return{k:clean(z)for k,z in v.items()}
      if isinstance(v,list):return[clean(z)for z in v]
      return v
     with(result_dir/name).open('a')as out:out.write(json.dumps(clean(row),allow_nan=False)+'\n')
    def geometric_gate(active_f,ps):
     path=np.stack([solver.get(i,'x')for i in range(N+1)]);roll=[path[0].copy()]
     for i in range(N):roll.append(np.array(active_f(roll[-1],solver.get(i,'u'),ps[i])).ravel())
     radii=np.asarray(policy.mpc_env.callback_orca.human_radii,dtype=float)
     guessed=min_body_clearance(path,policy.mpc_env.radius,radii)
     rolled=min_body_clearance(np.stack(roll),policy.mpc_env.radius,radii)
     return guessed,rolled
    def evaluate(active_f,mode,ps):
     out=dict(finite=True,dynamics=0.,robot_constraints=0.,orca_primal_dual=0.,orca_stationarity=0.,orca_complementarity=0.,proximity=0.,slack_nonnegative=0.,x0=0.,original_jmid_dynamics=0.,slack_max=0.,raw_robot_constraints=0.,max_violation=0.)
     worst=dict(violation=-1.)
     try:
      for i in range(N+1):
       xx=solver.get(i,'x');sl=solver.get(i,'sl');su=solver.get(i,'su');pp=params[i];pm=ps[i]
       if not np.isfinite(np.r_[xx,sl,su]).all():raise ValueError('nonfinite iterate')
       out['slack_nonnegative']=max(out['slack_nonnegative'],float(max(0.,-np.min(np.r_[sl,su]))))
       out['slack_max']=max(out['slack_max'],float(np.max(np.r_[sl,su])))
       if i<N:
        uu=solver.get(i,'u')
        if not np.isfinite(uu).all():raise ValueError('nonfinite input')
        active_next=np.array(active_f(xx,uu,pm)).ravel();h=np.array(hf(xx,uu,pp)).ravel();assert np.isfinite(h).all();assert np.isfinite(active_next).all();dl=np.zeros(len(h));du=dl.copy();idx=np.asarray(ocp.constraints.idxsh,dtype=int);dl[idx]=sl;du[idx]=su;vv=np.maximum(0,np.maximum(lo_original-h-dl,h-hi_original-du));raw=np.maximum(0,np.maximum(lo_original-h,h-hi_original));out['raw_robot_constraints']=max(out['raw_robot_constraints'],float(max(raw[k]for k,n in enumerate(policy.mpc_env.all_state_names)if n.startswith(('coll_const_rob','stat_ob','kin_acc','rob_act_bounds')))))
        out['dynamics']=max(out['dynamics'],float(np.max(abs(solver.get(i+1,'x')-active_next))))
        out['original_jmid_dynamics']=max(out['original_jmid_dynamics'],float(np.max(abs(solver.get(i+1,'x')-(active_next if mode=='baseline' else np.array(ff(xx,uu,pp)).ravel())))))
        for k,v in enumerate(vv):
         name=policy.mpc_env.all_state_names[k]
         group='proximity'if k in prox_indices else'robot_constraints'if name.startswith(('coll_const_rob','stat_ob','kin_acc','rob_act_bounds')) else'orca_complementarity'if'eq_comp_slack'in name else'orca_stationarity'if'eq_grad_Lagr'in name else'orca_primal_dual'
         out[group]=max(out[group],float(v))
         if (group!='proximity'or mode=='baseline')and v>worst['violation']:worst=dict(stage=i,row=k,name=name,violation=float(v))
       else:
        h=np.array(he(xx,pp)).ravel();assert np.isfinite(h).all();dl=np.zeros(len(h));du=dl.copy();idx=np.asarray(ocp.constraints.idxsh_e,dtype=int);dl[idx]=sl;du[idx]=su;vv=np.maximum(0,np.maximum(ocp.constraints.lh_e-h-dl,h-ocp.constraints.uh_e-du));out['robot_constraints']=max(out['robot_constraints'],float(vv.max()))
      x0=solver.get(0,'x');target=mode_initial_x0
      out['x0']=float(np.max(abs(x0-target)))
      groups=['dynamics','robot_constraints','orca_primal_dual','orca_stationarity','orca_complementarity','slack_nonnegative','x0']+(['proximity']if mode=='baseline'else[])
      out['max_violation']=max(out[g]for g in groups);out['algebraic_feasible']=out['max_violation']<=.05;out['worst_nonlinear']=worst
      guessed,rolled=geometric_gate(active_f,ps);out['candidate_swept_gap']=guessed;out['rollout_swept_gap']=rolled;out['predicted_body_gap']=min(guessed,rolled);out['distance_gate_pass']=out['predicted_body_gap']>=.30-1e-9;out['feasible']=out['algebraic_feasible']and out['distance_gate_pass']
      out['original_jmid_max_violation']=max(out['original_jmid_dynamics'],out['proximity'],*[out[g]for g in groups if g not in ['dynamics','x0']])
      # Fallback changes human goals/weights; original mode x0 is reported separately.
      out['original_x0_violation']=float(np.max(abs(x0-fresh[(0,'x')])));out['original_jmid_max_violation']=max(out['original_jmid_max_violation'],out['original_x0_violation'])
     except Exception as exc:out.update(finite=False,feasible=False,max_violation=None,error=str(exc))
     return out
    def mode_cost(ps):
     total=0.
     for i in range(N+1):
      xx=solver.get(i,'x');sl=solver.get(i,'sl');su=solver.get(i,'su');v=float(cf(xx,solver.get(i,'u'),ps[i]))if i<N else float(ce(xx,ps[i]));Zl=ocp.cost.Zl if i<N else ocp.cost.Zl_e;Zu=ocp.cost.Zu if i<N else ocp.cost.Zu_e;zl=ocp.cost.zl if i<N else ocp.cost.zl_e;zu=ocp.cost.zu if i<N else ocp.cost.zu_e;v+=float(.5*(Zl@(sl*sl)+Zu@(su*su))+zl@sl+zu@su);total+=(.25 if i<N else 1.)*v
     return total
    mode_initial_x0=fresh[(0,'x')].copy()
    trigger_metrics=evaluate(ff,'baseline',params)
    selected_mode='orca_observed_reinit'if current['strategy']=='store_switch'and trigger_metrics['proximity']>=.05 else'baseline'
    for mode in [selected_mode]:
     if mode.startswith('orca_observed'):mo,fmode=fallback_model()
     else:mo,fmode=ocp,ff
     control=[]
     for observed in [True]:
      solver=original_factory(mo,json_file=fallback_models['json'],generate=False,build=False)if mode.startswith('orca_observed')else original_factory(ocp,*x,**kw)
      for field,val in [('max_iter',4),('qp_iter_max',5)]:
       vv=ctypes.c_int(val);native_opts.ocp_nlp_solver_opts_set(solver.nlp_config,solver.nlp_opts,field.encode(),ctypes.byref(vv))
      solver.reset(reset_qp_solver_mem=1)
      for(i,f),v in fresh.items():
       if v.size:solver.set(i,f,v)
      ps={i:v.copy()for i,v in params.items()};mode_initial_x0=fresh[(0,'x')].copy()
      if mode.startswith('orca_observed'):
       # Current observations only; no saved solution is used. Preserve fresh
       # position/velocity/control guesses to isolate model effects.
       for i in range(N+1):
        xx=fresh[(i,'x')].copy()
        for hi in range(NH):
         off=10+6*hi;xx[off+4:off+6]=xx[off:off+2]+.25*obsvel[2*hi:2*hi+2];ps[i][off+2:off+4]=obsvel[2*hi:2*hi+2]
        xx[LW_START:NX]=-np.log(NW);solver.set(i,'x',xx)
        if i==0:mode_initial_x0=xx.copy()
      if mode=='orca_observed_reinit':
       xx=mode_initial_x0.copy()
       for i in range(N):
        solver.set(i,'x',xx);delta=xx[8:10]-xx[:2];th=float(np.arctan2(xx[2],xx[3]));aim=float(np.arctan2(delta[1],delta[0]));angle=float(np.arctan2(np.sin(aim-th),np.cos(aim-th)));target_v=min(policy.mpc_env.pref_speed,float(np.linalg.norm(delta))/.25)*max(0.,np.cos(angle));v_cmd=float(np.clip(target_v,xx[4]+policy.mpc_env.max_l_dcc*.25,xx[4]+policy.mpc_env.max_l_acc*.25));w_cmd=float(np.clip(angle/.25,-policy.mpc_env.max_rot,policy.mpc_env.max_rot));uu=np.zeros(NU);uu[:2]=[v_cmd,w_cmd]
        for hidx,fun in enumerate(policy.mpc_env.casadi_orca.humA_ws_solver_funcs):
         ans=fun(X=xx);uu[2+4*hidx:6+4*hidx]=np.array(ans['fin_prim']).ravel();uu[2+4*NH+NL*hidx:2+4*NH+NL*(hidx+1)]=np.array(ans['fin_dual']).ravel()
        if not np.isfinite(uu).all():raise RuntimeError('nonfinite fresh observed ORCA initial guess')
        solver.set(i,'u',uu);xx=np.array(fmode(xx,uu,ps[i])).ravel()
       solver.set(N,'x',xx)
      for i,v in ps.items():solver.set(i,'p',v)
      for(i,f),v in bounds.items():solver.constraints_set(i,f,mode_initial_x0 if i==0 and f in ['lbx','ubx']else v)
      lo=lo_original.copy();hi=hi_original.copy()
      if mode!='baseline':lo[prox_indices]=-1e6;hi[prox_indices]=1e6
      for i in range(N):solver.constraints_set(i,'lh',lo);solver.constraints_set(i,'uh',hi)
      solver.constraints_set(N,'lh',ocp.constraints.lh_e);solver.constraints_set(N,'uh',ocp.constraints.uh_e)
      solver.options_set('initialize_t_slacks',1);solver.options_set('qp_warm_start',0);solver.options_set('full_step_dual',0);solver.options_set('step_length',1.);solver.options_set('globalization','fixed_step');solver.options_set('line_search_use_sufficient_descent',0)
      initial=evaluate(fmode,mode,ps);initial_cost=mode_cost(ps);errs=[];trace=[];candidate_data={0:{f'{f}_{i}':solver.get(i,f).copy()for i in range(N+1)for f in ['x','sl','su']+(['u']if i<N else[])}}
      audit=os.environ.get('MP_CANDIDATE_AUDIT','0')=='1'
      callback_type=ctypes.CFUNCTYPE(None,ctypes.c_int,ctypes.c_int,ctypes.c_int,*([ctypes.c_double]*5))
      def callback(phase,iteration,qp_code,alpha,*res):
       try:
        if phase!=2 and not audit:return
        row=dict(run_id=current['run_id'],strategy=current['strategy'],seq=seq,case=current['case'],mode=mode,phase=['pre_qp','post_qp','post_update'][phase],iteration=iteration+1,alpha=alpha,metrics=evaluate(fmode,mode,ps),cost=mode_cost(ps),native_residuals=list(res),qp_mapped_status=qp_code)
        if phase==1:
         mem=ctypes.c_void_p();native_lib.ocp_nlp_get(solver.nlp_config,solver.nlp_solver,b'nlp_mem',ctypes.byref(mem));sd=np.zeros(3,dtype=np.int32);stats_reader.hpipm_trace_stats(mem,sd.ctypes.data,None);row['raw_hpipm_status']=int(sd[2])
        if phase in [0,2]:
         candidate_data[0 if phase==0 and iteration==0 else iteration+1]={f'{f}_{i}':solver.get(i,f).copy()for i in range(N+1)for f in ['x','sl','su']+(['u']if i<N else[])} if phase==2 or iteration==0 else candidate_data.get(iteration+1,{})
        append('events.jsonl',row);trace.append(row)
       except Exception as exc:errs.append(repr(exc))
      cb=callback_type(callback)
      if observed:assert trace_library.qp_observer_begin(solver.nlp_solver,ctypes.cast(cb,ctypes.c_void_p))==0
      try:status=solver.solve()
      finally:
       if observed:trace_library.qp_observer_finish()
      if errs:raise RuntimeError(str(errs))
      state={(i,f):solver.get(i,f).copy()for i in range(N+1)for f in ['x','u','sl','su']};last_post=next((r for r in reversed(trace)if r['phase']=='post_update'),None);final=(last_post['metrics']if last_post else initial)if not audit else evaluate(fmode,mode,ps);residuals=(last_post['native_residuals']if last_post else None)if not audit else np.asarray(solver.get_residuals(recompute=True)).tolist();fcost=(last_post['cost']if last_post else initial_cost)if not audit else mode_cost(ps);qpits=np.asarray(solver.get_stats('qp_iter')).tolist();sqpiter=int(solver.get_stats('sqp_iter'))
      if not observed:control=[status,state,residuals]
      else:
       same=None # Observer equivalence was tested separately in the preceding 180-condition experiment.
       post=[r for r in trace if r['phase']=='post_update'];candidates=[dict(iteration=0,metrics=initial,cost=initial_cost)]+[dict(iteration=r['iteration'],metrics=r['metrics'],cost=r['cost'])for r in post];eligible=[r for r in candidates if np.isfinite(r['cost'])and r['metrics']['finite']and r['metrics']['max_violation'] is not None and r['metrics']['max_violation']<=.05 and r['metrics'].get('distance_gate_pass',False)];changed=any(state[(i,f)].size and np.max(abs(state[(i,f)]-fresh[(i,f)]))>1e-8 for i in range(N+1)for f in ['x','u']);best=min(eligible,key=lambda r:r['cost'])if eligible else None
       if current['strategy']=='final_only':best=candidates[-1]if candidates[-1]['metrics']['finite']and candidates[-1]['metrics']['max_violation']<=.05 else None
       selected_data=None;selected_snapshot=None
       if best is not None:
        selected_data=candidate_data[best['iteration']];selected_data.update({f'p_{i}':v for i,v in ps.items()});selected_data['lbx0']=mode_initial_x0;selected_data['ubx0']=mode_initial_x0;selected_snapshot=f"{current['run_id']}_tick{seq:04d}_selected.npz";np.savez_compressed(result_dir/selected_snapshot,**selected_data)
       current['control']=dict(output_available=best is not None,mode=mode,selected_iteration=best['iteration']if best else None,selected_violation=best['metrics']['max_violation']if best else None,selected_cost=best['cost']if best else None,selected_predicted_gap=best['metrics']['predicted_body_gap']if best else None,selected_snapshot=selected_snapshot,action=selected_data['u_0'][:2].tolist()if best else[0.,0.],raw_native_status=int(status),initial_proximity=trigger_metrics['proximity'],initial_mode_violation=initial['max_violation'],final_violation=final['max_violation'])
       summary=dict(run_id=current['run_id'],strategy=current['strategy'],seq=seq,case=current['case'],mode=mode,sqp_budget=4,qp_budget=5,alpha=1.,fresh_input_sha256=fresh_hash,observed_velocity=obsvel.tolist(),observer_control_identical=same,initial=initial,final=final,initial_cost=initial_cost,final_cost=fcost,status=int(status),sqp_iter=sqpiter,qp_iter_history=qpits,changed=changed,ever_feasible=bool(eligible),first_feasible_iteration=eligible[0]['iteration']if eligible else None,best_feasible=best,all_candidates=candidates,robot_u0=state[(0,'u')][:2].tolist())
       summary['selection']=current['control'];append('summary.jsonl',summary)
      finished=solver;solver=base_solver;del finished
    solver_info.update(variants=1,output_available=False,selected_action=None,status=4,residuals=[0.,0.,0.,0.]);return 4
   tracked_p={}
   tracked_bounds={}
   original_constraints_set=solver.constraints_set
   def tracked_constraints_set(stage,field,value,*args,**kwargs):
    if field in ['lbx','ubx','lbu','ubu']:tracked_bounds[(stage,field)]=np.asarray(value).copy()
    return original_constraints_set(stage,field,value,*args,**kwargs)
   solver.constraints_set=tracked_constraints_set
   original_set=solver.set
   def guarded_set(stage,field,value):
    if field in ['p','x','u'] and not np.isfinite(value).all():raise RuntimeError('nonfinite solver parameter '+field)
    if field in ['lbx','ubx','lbu','ubu']:tracked_bounds[(stage,field)]=np.asarray(value).copy()
    if field=='p':tracked_p[stage]=np.asarray(value).copy()
    return original_set(stage,field,value)
   solver.set=guarded_set;solver.solve=solve;return solver
