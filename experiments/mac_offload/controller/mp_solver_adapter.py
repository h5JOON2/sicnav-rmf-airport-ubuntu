"""MP clearance30 adapter: control-required candidate selection only, no audit I/O.
One controller per process. Original MP files are never modified.
"""
import os, sys, json, time, ctypes
from pathlib import Path
from copy import deepcopy
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


def native_residual_eligible(values,tolerance=.05):
    """Use the SAME four cached residuals as the native 0.001 convergence test."""
    r=np.asarray(values,dtype=float)
    return r.shape==(4,) and np.isfinite(r).all() and np.all(r>=0.) and np.max(r)<=tolerance

# Control-only deadline dispatch does not change generated C models.
# Bump this fingerprint whenever factory/fallback_model changes the model or options.
MODEL_BUILD_FINGERPRINT = 'a0be8b66f0f733c92f0fff76f7ea89249b28775a1fb17215c5424a7fd4aa6c10'

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
   ocp.model.name='sicnav_arena10_jmid';ocp.code_export_directory=str(model_dir);ocp.solver_options.ext_fun_compile_flags='-O3 -ffast-math'
   model_json=str(model_dir.parent/'jmid_solver.json');needs_build=not(model_dir/'libacados_ocp_solver_sicnav_arena10_jmid.dylib').exists()
   solver=original_factory(ocp,json_file=model_json,generate=needs_build,build=needs_build);original_solve=solver.solve;solver.print_statistics=lambda:None
   kw=dict(json_file=model_json,generate=False,build=False);x=()
   NX=int(ocp.model.x.numel());NU=int(ocp.model.u.numel());NH=policy.mpc_env.num_hums;NL=policy.mpc_env.nlambda_hum;NW=policy.mpc_env.num_MID_samples;LW_START=10+6*NH

   native_opts=ctypes.CDLL(str(ROOT/'work/acados/lib/libacados.dylib'))
   native_opts.ocp_nlp_solver_opts_set.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_char_p,ctypes.c_void_p]
   lo_original=ocp.constraints.lh.copy();hi_original=ocp.constraints.uh.copy()
   trace_library=ctypes.CDLL(str(LIB_DIR/'qp_observer.dylib'))
   trace_library.qp_observer_begin.argtypes=[ctypes.c_void_p,ctypes.c_void_p];trace_library.qp_observer_begin.restype=ctypes.c_int
   trace_library.qp_observer_finish.argtypes=[];trace_library.qp_observer_finish.restype=None
   trace_library.qp_observer_api_version.restype=ctypes.c_int
   if trace_library.qp_observer_api_version()!=3:raise RuntimeError('Rebuild native observer API v3')
   trace_library.qp_observer_set_proximity_rows.argtypes=[ctypes.c_int,ctypes.c_void_p]
   trace_library.qp_observer_request_abort.argtypes=[]
   trace_library.qp_observer_cached_feasibility.restype=ctypes.c_double
   trace_library.qp_observer_cached_h_rows_max.argtypes=[ctypes.c_int,ctypes.c_void_p]
   trace_library.qp_observer_cached_h_rows_max.restype=ctypes.c_double
   fallback_models={}
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
    fallback_models.update(ocp=mo,ff=fmode,json=str(modelpath),solver=ss)
    return mo,fmode
   fallback_model() # Prebuild before the timed simulation, not during a control tick.
   def solve():
    nonlocal solver
    if 'job'not in current:return original_solve()
    started=time.perf_counter();base_solver=solver
    fresh={(i,f):solver.get(i,f).copy()for i in range(N+1)for f in ['x','u','sl','su','lam','t','z']+(['pi']if i<N else[])}
    params={i:v.copy()for i,v in tracked_p.items()};bounds={k:v.copy()for k,v in tracked_bounds.items()}
    obsvel=np.concatenate([fresh[(0,'x')][12+6*h:14+6*h]for h in range(NH)])
    proximity_rows=np.asarray(prox_indices,dtype=np.int32)
    stationarity_rows=np.asarray([i for i,n in enumerate(policy.mpc_env.all_state_names) if 'eq_grad_Lagr' in n],dtype=np.int32)
    complementarity_rows=np.asarray([i for i,n in enumerate(policy.mpc_env.all_state_names) if 'eq_comp_slack' in n],dtype=np.int32)
    assert trace_library.qp_observer_set_proximity_rows(len(proximity_rows),proximity_rows.ctypes.data)==0
    mode='baseline';proximity=None;best=None;samples=[];errors=[];selection_ms=0.
    status=None;native_stats=None;initial_violation=None;solver_ms=0.;baseline_probe_ms=0.
    try:
     modes=('orca_observed_reinit',) if current.get('force_orca',False) else ('baseline','orca_observed_reinit')
     for mode in modes:
      if mode!='baseline' and proximity is not None and proximity<.05:break
      fmode=fallback_models['ff'] if mode!='baseline' else None
      solver=fallback_models['solver'] if mode!='baseline' else base_solver
      for field,value in [('max_iter',4),('qp_iter_max',5)]:
       v=ctypes.c_int(value);native_opts.ocp_nlp_solver_opts_set(solver.nlp_config,solver.nlp_opts,field.encode(),ctypes.byref(v))
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
      radii=np.asarray(policy.mpc_env.callback_orca.human_radii,dtype=float)
      switch_requested=False
      callback_type=ctypes.CFUNCTYPE(None,ctypes.c_int,ctypes.c_int,ctypes.c_int,*([ctypes.c_double]*7))
      def callback(phase,iteration,qp_status,alpha,stat,eq,ineq,comp,cached_cost,cached_proximity):
       nonlocal best,selection_ms,switch_requested,proximity,initial_violation
       begin=time.perf_counter()
       try:
        if phase not in (0,3):return
        residuals=[stat,eq,ineq,comp]
        feasibility=trace_library.qp_observer_cached_feasibility()
        orca_stat=trace_library.qp_observer_cached_h_rows_max(len(stationarity_rows),stationarity_rows.ctypes.data)
        orca_comp=trace_library.qp_observer_cached_h_rows_max(len(complementarity_rows),complementarity_rows.ctypes.data)
        # Only values from a native residual evaluation paired with THIS iterate.
        samples.append(dict(mode=mode,iteration=iteration,phase=phase,res_stat=stat,res_eq=eq,res_ineq=ineq,res_comp=comp,cost=cached_cost,model_feasibility=feasibility,orca_stationarity=orca_stat,orca_complementarity=orca_comp))
        if mode=='baseline' and iteration==0:
         if not np.isfinite(cached_proximity):raise RuntimeError('Invalid cached proximity residual')
         proximity=cached_proximity
         if current['strategy']=='store_switch' and proximity>=.05:
          switch_requested=True;trace_library.qp_observer_request_abort();return
        if iteration==0:initial_violation=feasibility if np.isfinite(feasibility) else None
        if not np.isfinite(feasibility) or feasibility<0 or feasibility>.05 or not np.isfinite(cached_cost):return
        # Separate geometric swept-segment check of the solver's path only.
        # No hf/he/active_f, residual recomputation, nonlinear rollout or cost evaluation.
        path=np.stack([solver.get(i,'x') for i in range(N+1)])
        action=solver.get(0,'u')[:2]
        if not np.isfinite(path).all() or not np.isfinite(action).all():return
        gap=min_body_clearance(path,policy.mpc_env.radius,radii)
        if gap<.30-1e-9:return
        if best is None or cached_cost<best['cost']:
         best=dict(iteration=iteration,cost=cached_cost,action=action.tolist(),gap=gap,residuals=residuals,feasibility=feasibility,orca_stat=orca_stat,orca_comp=orca_comp)
       except Exception as exc:errors.append(str(exc));trace_library.qp_observer_request_abort()
       finally:selection_ms+=(time.perf_counter()-begin)*1000
      cb=callback_type(callback)
      assert trace_library.qp_observer_begin(solver.nlp_solver,ctypes.cast(cb,ctypes.c_void_p))==0
      solve_start=time.perf_counter()
      try:status=original_solve() if mode=='baseline' else solver.solve()
      finally:trace_library.qp_observer_finish()
      elapsed=(time.perf_counter()-solve_start)*1000;solver_ms+=elapsed
      if errors:raise RuntimeError(str(errors))
      if switch_requested:
       baseline_probe_ms=elapsed;best=None;continue
      native_stats=(int(solver.get_stats('sqp_iter')),np.asarray(solver.get_stats('qp_iter')).tolist())
      break
     current['control']=dict(output_available=best is not None,mode=mode,
        selected_iteration=best['iteration']if best else None,
        selected_violation=best['feasibility']if best else None,
        selected_cost=best['cost']if best else None,
        selected_predicted_gap=best['gap']if best else None,selected_snapshot=None,
        action=best['action']if best else[0.,0.],raw_native_status=int(status),
        initial_proximity=proximity,initial_mode_violation=initial_violation,
        sqp_iter=native_stats[0],qp_iter_history=native_stats[1],
        mode_setup_ms=(time.perf_counter()-started)*1000-solver_ms,
        solver_with_selection_ms=solver_ms,candidate_selection_ms=selection_ms,
        baseline_probe_ms=baseline_probe_ms,diagnostics_enabled=False,snapshot_writes=0,
        residual_source='native_cached',residual_threshold=.05,
        native_convergence_threshold=.001,selected_residuals=best['residuals']if best else None,
        acceptance_definition='cached_model_feasibility_including_orca_kkt',
        selected_orca_stationarity=best['orca_stat']if best else None,
        selected_orca_complementarity=best['orca_comp']if best else None,
        evaluated_iterates=samples,nonlinear_reevaluations=0,
        last_unassessed_update_not_selected=(int(status)==2))
    finally:solver=base_solver
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
