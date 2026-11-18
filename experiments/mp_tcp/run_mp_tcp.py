"""Persistent Mac TCP endpoint for the MP-native controller (one client/solver)."""
import argparse,json,socket,time,traceback
from pathlib import Path
from mp_controller import MPController

def main():
 p=argparse.ArgumentParser();p.add_argument('--host',default='127.0.0.1');p.add_argument('--port',type=int,default=9090)
 p.add_argument('--humans',type=int,default=3);p.add_argument('--device',choices=['cpu','mps'],default='cpu');p.add_argument('--work-dir',default=str(Path(__file__).resolve().parent/'runtime'))
 p.add_argument('--origin-x',type=float,default=0.);p.add_argument('--origin-y',type=float,default=0.);p.add_argument('--obstacles-json');p.add_argument('--forecast-delay-steps',type=int,choices=[0,1],default=1)
 p.add_argument('--mpc-horizon',type=int,choices=[7,8],default=8,help='MPC stages at 0.25 s; JMID always predicts eight stages')
 p.add_argument("--recovery-mode",choices=["motion","stop"],default="motion")
 a=p.parse_args();obstacles=json.loads(Path(a.obstacles_json).read_text())if a.obstacles_json else None
 c=MPController(a.work_dir,humans=a.humans,device=a.device,obstacles=obstacles,origin=(a.origin_x,a.origin_y),forecast_delay_steps=a.forecast_delay_steps,recovery_mode=a.recovery_mode,mpc_horizon=a.mpc_horizon)
 with socket.socket()as server:
  server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);server.bind((a.host,a.port));server.listen(1)
  print('MP_TCP_READY',a.host,a.port,flush=True)
  while True:
   conn,address=server.accept();c.start_episode()
   with conn:
    conn.settimeout(15);reader=conn.makefile('rb')
    try:
     while True:
      raw=reader.readline(1024*1024+1)
      if not raw:break
      if len(raw)>1024*1024 or not raw.endswith(b'\n'):raise ValueError('packet too large or missing newline')
      try:
       request=json.loads(raw)
       if request.get('cmd')=='ping':result={'status':'ready','algorithm':'mp-clearance30','sqp':4,'qp':5,'horizon':c.horizon}
       else:result=c.step(request)
      except Exception as exc:
       traceback.print_exc();c.start_episode();result={'status':'error','error_msg':str(exc),'action':{'v':0.,'r':0.},'mpc_feasible':False,'output_source':'error_stop'}
      conn.sendall((json.dumps(result,allow_nan=False)+'\n').encode())
    except (OSError,ValueError)as exc:print('CLIENT_END',repr(exc),flush=True)
    finally:reader.close()
if __name__=='__main__':main()
