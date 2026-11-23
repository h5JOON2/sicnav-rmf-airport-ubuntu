#!/usr/bin/env python3
"""Run a bounded live shadow capture and automatically produce CSV/PNG/HTML."""
import argparse,datetime,os,shlex,signal,socket,subprocess,sys,tarfile,time
from pathlib import Path

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--runtime',type=Path,default=Path(__file__).resolve().parents[1])
 p.add_argument('--mp-root',type=Path,default=Path(os.environ.get('SICNAV_MP_ROOT', '.')))
 p.add_argument('--remote',required=True,help='Ubuntu SSH target user@host');p.add_argument('--seconds',type=float,default=60)
 p.add_argument('--humans',type=int,default=3,choices=range(1,9))
 p.add_argument('--ros-workspace',default='$HOME/sicnav_airport_ws/install/setup.bash',help='Remote ROS workspace setup path; $HOME expansion allowed')
 p.add_argument('--perception-source',choices=['lidar','gt'],default='lidar')
 p.add_argument('--drive',action='store_true',help='Publish /cmd_vel in Gazebo')
 p.add_argument('--goal-x',type=float,default=138.);p.add_argument('--goal-y',type=float,default=-35.)
 p.add_argument('--start-simulation',action='store_true',help='Use only if no Gazebo instance is already running')
 p.add_argument('--out',type=Path,default=Path(__file__).resolve().parent/('capture-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S')))
 a=p.parse_args();a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=False)
 env={**os.environ,'SICNAV_MP_ROOT':str(a.mp_root)}
 ssh=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8',a.remote]
 remote='codex-offload-validation/'+a.out.name
 def remote_path(path):
  if path.startswith('$HOME/'):
   return '"$HOME"/'+shlex.quote(path[6:])
  if path.startswith('~/'):
   return '"$HOME"/'+shlex.quote(path[2:])
  return shlex.quote(path)
 def shell(s):return subprocess.run(ssh+[s],check=True,text=True,capture_output=True)
 jobs=[];files=[];gateway_started=False;simulation_started=False
 def launch(cmd,name,cwd=None):
  f=(a.out/name).open('w');files.append(f)
  proc=subprocess.Popen(cmd,cwd=cwd,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);jobs.append(proc);return proc
 def wait_port(port,proc):
  deadline=time.monotonic()+3600 # Setup only; excluded from measured capture time.
  while time.monotonic()<deadline:
   if proc.poll() is not None:raise RuntimeError(f'Process exited {proc.returncode}; inspect console logs')
   try:
    with socket.create_connection(('127.0.0.1',port),timeout=.2):return
   except OSError:time.sleep(.25)
  raise TimeoutError(f'Port {port} not ready')
 for port in range(9150,9154):
  try:
   with socket.create_connection(('127.0.0.1',port),timeout=.2):raise RuntimeError(f'Port {port} is already in use. Stop previous offload services before capturing.')
  except OSError:pass
 metadata={'started_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'runtime':str(a.runtime),'mp_root':str(a.mp_root),'remote':a.remote,'remote_dir':remote,'maximum_humans':a.humans,'variable_humans':True,'requested_wall_seconds':a.seconds,'mode':'drive' if a.drive else 'shadow','goal_xy':[a.goal_x,a.goal_y],'scan_odom_slop_s':.035,'control_target_sim_s':.25,'watchdog_target_sim_s':.05}
 (a.out/'metadata.json').write_text(__import__('json').dumps(metadata,indent=2))
 metadata['perception_source']=a.perception_source
 (a.out/'metadata.json').write_text(__import__('json').dumps(metadata,indent=2))
 try:
  package=a.out/'instrumented-runtime.tar.gz'
  with tarfile.open(package,'w:gz') as t:
   for d in ('scripts','perception','remote_runtime','worlds','tests','controller','start_controller.command','start_mac.command','NOTICE','DR_SPAAM_LICENSE','README.md'):
    t.add(a.runtime/d,arcname=d,filter=lambda ti:None if '__pycache__' in ti.name or '/._' in ti.name else ti)
  shell('mkdir -p '+shlex.quote(remote))
  with package.open('rb') as f:subprocess.run(ssh+['tar -xzf - -C '+shlex.quote(remote)],stdin=f,check=True)
  controller=launch(['zsh','start_controller.command','--humans',str(a.humans)],'controller-console.log',a.runtime)
  wait_port(9150,controller)
  server=launch(['zsh','start_mac.command','--humans',str(a.humans),'--perception-source',a.perception_source,'--log',str(a.out/'mac.jsonl')],'mac-console.log',a.runtime)
  wait_port(9151,server)
  tunnel=launch(['zsh','start_tunnel.command',a.remote],'tunnel-console.log',a.runtime)
  time.sleep(1)
  if tunnel.poll() is not None:raise RuntimeError('SSH tunnel failed; inspect tunnel-console.log')
  if a.start_simulation:
   shell('cd '+shlex.quote(remote)+'; nohup bash scripts/run_lidar_simulation.sh > simulation-console.log 2>&1 < /dev/null & echo $! > simulation.pid')
   simulation_started=True;time.sleep(3)
  gateway_options=' --perception-source '+a.perception_source+' --goal-x '+str(a.goal_x)+' --goal-y '+str(a.goal_y)
  if a.drive:gateway_options+=' --drive'
  if a.perception_source=='gt':gateway_options+=' --expected-gt-people '+str(a.humans)
  shell('cd '+shlex.quote(remote)+'; source /opt/ros/jazzy/setup.bash; source '+remote_path(a.ros_workspace)+'; nohup python3 -u scripts/robot_offload_gateway.py'+gateway_options+' --timing-log gateway.jsonl > gateway-console.log 2>&1 < /dev/null & echo $! > gateway.pid')
  gateway_started=True
  print('Capturing live '+('drive' if a.drive else 'shadow')+' data:',a.out,flush=True)
  deadline=time.monotonic()+a.seconds
  while time.monotonic()<deadline:
   if any(proc.poll() is not None for proc in jobs):raise RuntimeError('A local service exited during capture')
   time.sleep(min(1,max(0,deadline-time.monotonic())))
 finally:
  if gateway_started:
   shell('cd '+shlex.quote(remote)+'; kill -INT "$(cat gateway.pid)"; sleep 1')
   for file in ('gateway.jsonl','gateway-console.log'):
    subprocess.run(['scp',a.remote+':'+remote+'/'+file,str(a.out/file)],check=True)
  if simulation_started:
   shell('cd '+shlex.quote(remote)+'; kill -TERM "$(cat simulation.pid)"; sleep 1')
   subprocess.run(['scp',a.remote+':'+remote+'/simulation-console.log',str(a.out/'simulation-console.log')],check=True)
  for proc in reversed(jobs):
   if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
  for proc in jobs:
   try:proc.wait(timeout=8)
   except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM)
  for f in files:f.close()
 from analyze_timing import analyze
 summary=analyze(a.out/'mac.jsonl',a.out/'gateway.jsonl',a.out)
 print('Report:',a.out/'report.html',flush=True)
 print('Computed action RPC deadline met:',summary['computed_250ms_rpc_budget_met'])

if __name__=='__main__':main()
