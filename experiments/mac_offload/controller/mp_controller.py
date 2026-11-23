"""TCP-ready controller using MP's clearance30 native selection implementation."""
import os,sys,math,json,time,hashlib,configparser,collections,collections.abc,uuid
from pathlib import Path
from copy import deepcopy, copy
from jmid_deadline import DeadlineForecast
import numpy as np
collections.Sequence=collections.abc.Sequence
ROOT=Path(os.environ.get('SICNAV_MP_ROOT',Path(__file__).resolve().parents[2])).resolve()
REPO=ROOT/'work/repo'
os.environ.setdefault('ACADOS_SOURCE_DIR',str(ROOT/'work/acados'))
sys.path[:0]=[str(REPO),str(REPO/'sicnav_diffusion/JMID/MID')]
import torch
import mp_solver_adapter as adapter
from recovery_planner import RecoveryPlanner, RecoveryLimits

class MPController:
 def __init__(self,work_dir,humans=3,device='cpu',obstacles=None,origin=(0.,0.),forecast_delay_steps=1,recovery_mode="motion",mpc_horizon=4,max_speed=.31,max_omega=1.9,warmup_recovery=False,jmid_deadline_ms=150.):
  if mpc_horizon not in (4,7,8):raise ValueError('mpc_horizon must be 4, 7 or 8')
  self.forecast_runner=DeadlineForecast(jmid_deadline_ms)
  self.jmid_deadline_ms=jmid_deadline_ms
  self.horizon=mpc_horizon
  self.warmup_recovery=warmup_recovery;self.max_speed=max_speed
  if not 1 <= humans <= 8:raise ValueError('humans must be between 1 and 8')
  if device not in ('cpu','mps'):raise ValueError('device must be cpu or mps')
  if forecast_delay_steps not in (0,1):raise ValueError('forecast delay must be 0 or 1')
  if recovery_mode not in ("motion","stop"):raise ValueError("invalid recovery mode")
  self.recovery_mode=recovery_mode
  self.recovery_planner=RecoveryPlanner()
  self.origin=origin;self.humans=humans;self.device=device;self.delay=forecast_delay_steps
  if device=='mps':
   if not torch.backends.mps.is_available():raise RuntimeError('MPS unavailable; no implicit CPU fallback')
   from jmid_mps_support import install
   install('mps')
  global CrowdSimPlus, Robot, FullState, JointState, ObservableState, SICNavAcados, HumanTrajectoryForecasterSim
  from crowd_sim_plus.envs.crowd_sim_plus import CrowdSimPlus
  from crowd_sim_plus.envs.utils.robot_plus import Robot
  from crowd_sim_plus.envs.utils.state_plus import FullState,JointState,ObservableState
  from sicnav_diffusion.policy.sicnav_acados import SICNavAcados
  from lean_runtime import install as remove_print_diagnostics
  remove_print_diagnostics(SICNavAcados)
  from crowd_sim_plus.envs.policy.policy_factory import policy_factory
  policy_factory["sicnav_acados"] = SICNavAcados
  from sicnav_diffusion.JMID.mid_sim_wrapper import HumanTrajectoryForecasterSim
  torch.set_num_threads(1)
  torch.set_num_interop_threads(1)
  if not(REPO/'sicnav_diffusion').is_dir():raise RuntimeError('Set SICNAV_MP_ROOT to the tested environment containing work/repo and work/acados')
  self.out=Path(work_dir).resolve();self.out.mkdir(parents=True,exist_ok=True)
  ec=configparser.RawConfigParser();ec.read(REPO/'sicnav_diffusion/configs/env.config')
  ec.set('sim','human_num',str(humans));ec.set('sim','rect_width','10');ec.set('sim','rect_height','5');ec.set('sim','circle_radius','4');ec.set('robot','policy','sicnav_acados');ec.set('env','time_step','0.25');ec.set('human_trajectory_forecaster','prediction_horizon','8');ec.set('human_trajectory_forecaster','num_samples','15')
  pc=configparser.RawConfigParser();pc.read(REPO/'sicnav_diffusion/configs/policy.config');pc.set('campc','horiz',str(self.horizon));pc.set('mpc_env','orca_kkt_horiz',str(self.horizon));pc.add_section('benchmark');pc.set('benchmark','sqp','50');pc.set('benchmark','qp','15');pc.set('benchmark','version','full_mpc_v1')
  if max_speed is not None:
   if not 0<max_speed<=.31:raise ValueError('max_speed must respect TurtleBot 4 safe-mode limit 0.31')
   for setting in ('max_speed','max_rev_speed','pref_speed'):pc.set('mpc_env',setting,str(max_speed))
  if max_omega is not None:
   if not 0<max_omega<=1.9:raise ValueError('max_omega must respect TurtleBot 4 angular limit 1.9')
   pc.set('mpc_env','max_rot_degrees',str(math.degrees(max_omega)))
  self.recovery_planner=RecoveryPlanner(RecoveryLimits(
      forward_speed=pc.getfloat('mpc_env','max_speed'),reverse_speed=pc.getfloat('mpc_env','max_rev_speed'),
      acceleration=pc.getfloat('mpc_env','max_l_acc'),deceleration=-pc.getfloat('mpc_env','max_l_dcc'),
      angular_speed=math.radians(pc.getfloat('mpc_env','max_rot_degrees'))))
  self.env=CrowdSimPlus();self.env.configure(ec)
  self.obstacles=obstacles if obstacles is not None else [[(-5.,-5.),(5.,-5.)],[(5.,-5.),(5.,5.)],[(5.,5.),(-5.,5.)],[(-5.,5.),(-5.,-5.)]]
  def static_obstacles(rule,static_obstacles=None):
   self.env.static_obstacles=deepcopy(self.obstacles);return self.env.static_obstacles
  self.env.generate_static_obstacles=static_obstacles
  self.policy=SICNavAcados();self.policy.configure(pc)
  self.robot=Robot(ec,'robot');self.robot.set_policy(self.policy);self.env.set_robot(self.robot);self.policy.set_phase('test');self.policy.set_device(torch.device('cpu'))
  self.current=adapter.current;self.current.clear();self.current.update(case=0,run_id='tcp',strategy='store_switch')
  controller=self
  class CachedForecaster:
   num_hist_frames=6
   def update_state_hists(self,*args):pass
   def predict_ret_best(self):
    f=np.asarray(controller.current['forecast']['forecasts'])
    if f.shape[2]!=9:raise ValueError('Expected t0 plus eight JMID future frames')
    # The upstream policy drops t0 and needs N+1 forecast frames.
    adapted=f if controller.horizon<=7 else np.concatenate([f,2*f[:,:,-1:,:]-f[:,:,-2:-1,:]],axis=2)
    return adapted,np.asarray(controller.current['forecast']['weights'])
  self.policy.init_human_prediction=lambda:setattr(self.policy,'hum_traj_forecaster',CachedForecaster())
  reference=self.policy.get_ref_traj
  def fresh_reference(state):
   self.policy.gen_ref_traj(state);return reference(state)
  self.policy.get_ref_traj=fresh_reference
  signature={'humans':humans,'N':self.horizon,'obstacles':self.obstacles,'revision':'mp-clearance30-native-v1',
             'env_config':{section:dict(ec.items(section))for section in ec.sections()},
             'policy_config':{section:dict(pc.items(section))for section in pc.sections()},
             'factory_sha256':adapter.MODEL_BUILD_FINGERPRINT}
  key=hashlib.sha256(json.dumps(signature,sort_keys=True).encode()).hexdigest()[:16]
  adapter.install(self.policy,self.out/'models'/key,self.out/'trace')
  os.chdir(REPO)
  self.policy.set_env(self.env)
  self.actual=HumanTrajectoryForecasterSim(ec,'sicnav_diffusion/JMID/test_time_configs/mid_jp.yaml')
  if self.actual.mid_model.device.type != device:
   raise RuntimeError('JMID device mismatch; refusing hidden CPU fallback')
  assert all(p.device.type==device for p in self.actual.mid_model.model.parameters())
  assert all(p.device.type==device for p in self.actual.mid_model.registrar.parameters())
  self.neural_calls=0;self.jmid_invocations=0
  def verify_neural_device(module,inputs,output):
   if output.device.type!=device:raise RuntimeError('JMID neural output device mismatch')
   self.neural_calls+=1
  self.actual.mid_model.model.vel_predictor.net.register_forward_hook(verify_neural_device)
  rank=self.actual.get_most_likely_samples;self.actual.get_most_likely_samples=lambda x:rank(x.detach().cpu())
  self.start_episode()
 def start_episode(self):
  self.current['run_id']='tcp-'+uuid.uuid4().hex[:10]
  self.seq=0;self.last_stamp=None;self.previous_forecast=None;self.last_ids=None;self.episode_jmid_invocations=0
  self.actual.prev_states=[[]for _ in range(self.humans)];self.actual.prev_robot_states=[]
  self.policy.reset_scenario_values()
 def parse(self,request):
  r=request['robot'];ox,oy=self.origin;theta=float(r['theta']);vx=float(r['vx']);vy=float(r['vy'])
  robot=FullState(px=float(r['px'])-ox,py=float(r['py'])-oy,vx=vx,vy=vy,radius=.25,gx=float(r['gx'])-ox,gy=float(r['gy'])-oy,v_pref=self.max_speed or .9,theta=theta)
  if not np.isfinite([robot.px,robot.py,robot.vx,robot.vy,robot.gx,robot.gy,robot.theta]).all():raise ValueError('Nonfinite robot observation')
  robot.lvel=float(r.get('v',vx*math.cos(theta)+vy*math.sin(theta)));robot.omega=float(r.get('omega',0.));robot.v_dot=float(r.get('v_dot',0.));robot.omega_dot=float(r.get('omega_dot',0.))
  hs=sorted(request['humans'],key=lambda h:int(h['id']))
  if len(hs)!=self.humans:raise ValueError(f'Expected exactly {self.humans} stable humans, got {len(hs)}')
  humans=[]
  for h in hs:
   humans.append(ObservableState(px=float(h['px'])-ox,py=float(h['py'])-oy,vx=float(h['vx']),vy=float(h['vy']),radius=.20))
  if not all(np.isfinite([h.px,h.py,h.vx,h.vy]).all()for h in humans):raise ValueError('Nonfinite human observation')
  return JointState(robot,humans,self.obstacles),[int(h['id'])for h in hs]
 def step(self,request):
  start=time.perf_counter();ts=float(request['timestamp'])
  if not math.isfinite(ts):raise ValueError('finite simulation timestamp required')
  state,ids=self.parse(request)
  if (self.last_stamp is not None and ts<self.last_stamp)or(self.last_ids is not None and ids!=self.last_ids):self.start_episode()
  if self.last_stamp is not None and ts-self.last_stamp<.25-1e-9:raise ValueError('minimum 0.25 simulation seconds between requests')
  self.last_stamp=ts;self.last_ids=ids;self.env.global_time=ts;self.env.global_time_step=self.seq
  self.actual.update_state_hists(state.self_state,state.human_states,ts)
  if len(self.actual.prev_states[0]) < 6:
   if self.warmup_recovery:
    r=state.self_state
    recovery=self.recovery_planner.plan([r.px,r.py,r.theta,r.lvel,r.omega,r.gx,r.gy],
        [[h.px,h.py,h.vx,h.vy,h.radius]for h in state.human_states],self.obstacles)
    return {'status':'success','action':{'v':recovery['action'][0],'r':recovery['action'][1]},'timestamp':ts,
            'mpc_feasible':False,'output_source':'observed_people_warmup_local_plan'if recovery['available']else'emergency_stop',
            'prediction_mode':'observed_people_warmup','forecast_age_sim_s':None,'jmid_ms':None,
            'compute_time_ms':(time.perf_counter()-start)*1000,'recovery':recovery}
   return {'status':'success','action':{'v':0.,'r':0.},'timestamp':ts,
           'mpc_feasible':False,'output_source':'history_warmup',
           'prediction_mode':'history_warmup','forecast_age_sim_s':None,
           'jmid_ms':None,'compute_time_ms':(time.perf_counter()-start)*1000}
  def prepare_forecast():
   # Snapshot histories, not model weights. The background job alone owns MID inference.
   predictor=copy(self.actual)
   predictor.prev_states=deepcopy(self.actual.prev_states)
   predictor.prev_robot_states=deepcopy(self.actual.prev_robot_states)
   self.jmid_invocations+=1;self.episode_jmid_invocations+=1
   metadata=dict(time=ts,run_id=self.current['run_id'],call=self.jmid_invocations,
                 episode_call=self.episode_jmid_invocations,history=predictor.prev_states)
   def infer():
    tf=time.perf_counter();before=self.neural_calls
    with torch.inference_mode():forecasts,weights=predictor.predict_ret_best()
    if self.device=='mps':torch.mps.synchronize()
    elapsed=(time.perf_counter()-tf)*1000
    if not np.isfinite(forecasts).all()or not np.isfinite(weights).all():raise RuntimeError('nonfinite forecast')
    return dict(forecast=dict(forecasts=np.asarray(forecasts).tolist(),weights=np.asarray(weights).tolist(),time=ts),
                jmid_ms=elapsed,neural_calls=self.neural_calls-before)
   return infer,metadata
  forecast_event=self.forecast_runner.step(prepare_forecast)
  ready=forecast_event['status']=='ready'
  meta=forecast_event['metadata'];result=forecast_event['result']
  self.current['force_orca']=not ready
  if ready:
   fresh=result['forecast']
   self.current['forecast']=self.previous_forecast if self.delay and self.previous_forecast is not None else fresh
   self.previous_forecast=fresh
  else:
   # Only shape-compatible policy input; forced ORCA dynamics ignore sample parameters.
   # Use current observations, never the late JMID result or a previous prediction.
   self.previous_forecast=None
   pos=np.asarray([[h.px,h.py]for h in state.human_states])
   vel=np.asarray([[h.vx,h.vy]for h in state.human_states])
   cv=pos[:,None,:]+vel[:,None,:]*(np.arange(9)[None,:,None]*.25)
   self.current['forecast']=dict(forecasts=np.repeat(cv[:,None,:,:],15,axis=1).tolist(),
                                weights=np.full((self.humans,15),1./15).tolist(),time=ts)
  forecast_ms=result['jmid_ms'] if ready else None
  history=meta['history']
  discarded=forecast_event['discarded']
  late_ms=discarded['result']['value']['jmid_ms'] if discarded is not None else None
  self.current['job']={'seq':self.seq};self.current.pop('control',None)
  self.policy.acados_solver.reset(reset_qp_solver_mem=1);self.policy.x_prev=None;self.policy.u_prev=None;self.policy.num_prev_used=0;self.policy.mpc_sol_succ=False;self.policy.prev_rev=state.self_state.lvel<0;self.policy.prev_action=None
  with torch.inference_mode():self.policy.predict(state) # legacy fallback is deliberately ignored
  control=dict(self.current['control']);self.seq+=1
  action,source,recovery=self.select_output(control,state)
  return {'status':'success','action':{'v':float(action[0]),'r':float(action[1])},'timestamp':ts,'mpc_feasible':control['output_available'],'output_source':source,'prediction_mode':control['mode'],'forecast_age_sim_s':ts-self.current['forecast']['time'],'jmid_ms':forecast_ms,'jmid_neural_calls':result['neural_calls'] if ready else None,'jmid_device':self.device,'jmid_call_index':meta['call'],'episode_jmid_call_index':meta['episode_call'],'jmid_input_history':history,'jmid_request_stamp':meta['time'],'jmid_deadline_ms':self.jmid_deadline_ms,'jmid_deadline_status':forecast_event['status'],'jmid_wait_ms':forecast_event['wait_ms'],'jmid_completed_late_ms':late_ms,'jmid_completed_late_stamp':discarded['metadata']['time'] if discarded is not None else None,'controller_prediction_source':'jmid' if ready else 'orca_'+forecast_event['status'],'compute_time_ms':(time.perf_counter()-start)*1000,'selection':control,'recovery':recovery}
 def close(self):
  self.forecast_runner.close()
 def select_output(self,control,state):
  if control['output_available']:
   return control['action'],('initial_feasible'if control['selected_iteration']==0 else 'updated_feasible'),None
  if self.recovery_mode=='stop':return [0.,0.],'no_feasible_stop',None
  r=state.self_state
  recovery=self.recovery_planner.plan([r.px,r.py,r.theta,r.lvel,r.omega,r.gx,r.gy],
      [[h.px,h.py,h.vx,h.vy,h.radius]for h in state.human_states],self.obstacles)
  return recovery['action'],('recovery_motion'if recovery['available']else'emergency_stop'),recovery

