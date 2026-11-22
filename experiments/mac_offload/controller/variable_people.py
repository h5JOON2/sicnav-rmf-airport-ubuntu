"""Route real observations to the corresponding native solver without padding."""
import json,math,os,signal,socket,subprocess,sys,time
from pathlib import Path
from recovery_planner import RecoveryPlanner,RecoveryLimits

class VariablePeopleController:
    def __init__(self,maximum,worker_factory,origin=(0.,0.),obstacles=None,max_speed=.31,max_omega=1.9,horizon=4):
        if not 1<=maximum<=8:raise ValueError('maximum humans must be between 1 and 8')
        self.maximum=maximum;self.origin=origin;self.horizon=horizon
        self.workers={}
        try:
            for n in range(1,maximum+1):self.workers[n]=worker_factory(n)
        except BaseException:
            self.close();raise
        self.obstacles=obstacles if obstacles is not None else [[(-5.,-5.),(5.,-5.)],[(5.,-5.),(5.,5.)],[(5.,5.),(-5.,5.)],[(-5.,5.),(-5.,-5.)]]
        self.empty_planner=RecoveryPlanner(RecoveryLimits(forward_speed=max_speed,reverse_speed=max_speed,angular_speed=max_omega))
        self.start_episode()

    def start_episode(self):
        self.last_stamp=None;self.last_count=None
        for worker in self.workers.values():worker.start_episode()

    def step(self,request):
        start=time.perf_counter();ts=float(request['timestamp']);humans=request['humans'];count=len(humans)
        if not math.isfinite(ts):raise ValueError('finite simulation timestamp required')
        if count>self.maximum:raise ValueError('observed human count exceeds configured maximum')
        if self.last_stamp is not None and ts<self.last_stamp:self.start_episode()
        if self.last_stamp is not None and ts-self.last_stamp<.25-1e-9:raise ValueError('minimum 0.25 simulation seconds between requests')
        # A returning count must not continue an old episode across missing people.
        if count!=self.last_count and count:self.workers[count].start_episode()
        self.last_stamp=ts;self.last_count=count
        if count:
            response=self.workers[count].step(request)
        else:
            r=request['robot'];ox,oy=self.origin;theta=float(r['theta'])
            v=float(r.get('v',float(r['vx'])*math.cos(theta)+float(r['vy'])*math.sin(theta)))
            robot=[float(r['px'])-ox,float(r['py'])-oy,theta,v,float(r.get('omega',0.)),float(r['gx'])-ox,float(r['gy'])-oy]
            if not all(math.isfinite(value) for value in robot):raise ValueError('finite robot observations required')
            reached=math.hypot(robot[0]-robot[5],robot[1]-robot[6])<=.15
            plan=({'available':True,'action':[0.,0.],'kind':'goal_reached','mpc_constraints_validated':False} if reached else self.empty_planner.plan(robot,[],self.obstacles))
            response={'status':'success','action':{'v':plan['action'][0],'r':plan['action'][1]},'timestamp':ts,
                      'mpc_feasible':False,'output_source':'goal_reached' if reached else ('no_observed_people_local_plan' if plan['available'] else 'emergency_stop'),
                      'prediction_mode':'no_observed_people','jmid_ms':None,'recovery':plan,
                      'compute_time_ms':(time.perf_counter()-start)*1000}
        return {**response,'observed_humans':count,'maximum_humans':self.maximum}

    def close(self):
        for worker in self.workers.values():worker.close()

class NativeWorker:
    """Each native solver owns adapter globals and a separate MPS process."""
    def __init__(self,args,count):
        self.connection=None;self.reader=None;self.process=None
        out=Path(args.work_dir).resolve()/f'people-{count}';out.mkdir(parents=True,exist_ok=True)
        with socket.socket() as reservation:
            reservation.bind(('127.0.0.1',0));port=reservation.getsockname()[1]
        command=[sys.executable,'-u',str(Path(__file__).with_name('run_mp_tcp.py')),
                 '--host','127.0.0.1','--port',str(port),'--humans',str(count),'--device',args.device,
                 '--work-dir',str(out),'--origin-x',str(args.origin_x),'--origin-y',str(args.origin_y),
                 '--mpc-horizon',str(args.mpc_horizon),'--forecast-delay-steps',str(args.forecast_delay_steps),
                 '--recovery-mode',args.recovery_mode,'--max-speed',str(args.max_speed or .31),
                 '--max-omega',str(args.max_omega or 1.9),'--warmup-recovery','--jmid-deadline-ms',str(args.jmid_deadline_ms),'--client-timeout','0']
        if args.obstacles_json:command+=['--obstacles-json',args.obstacles_json]
        self.log=(out/'worker-console.log').open('w')
        self.process=subprocess.Popen(command,stdout=self.log,stderr=subprocess.STDOUT)
        deadline=time.monotonic()+1200 # First optimized C build can exceed three minutes.
        try:
            while time.monotonic()<deadline:
                if self.process.poll() is not None:raise RuntimeError(f'{count}-person native worker failed; see {out}/worker-console.log')
                try:self.connection=socket.create_connection(('127.0.0.1',port),timeout=.2);break
                except OSError:time.sleep(.1)
            if self.connection is None:raise TimeoutError(f'{count}-person worker startup timed out')
            self.connection.settimeout(20);self.reader=self.connection.makefile('rb')
            ready=self.rpc({'cmd':'ping'})
            if ready.get('humans')!=count:raise RuntimeError('native worker count mismatch')
            print('NATIVE_PEOPLE_READY',count,flush=True)
        except BaseException:self.close();raise

    def rpc(self,request):
        self.connection.sendall((json.dumps(request,allow_nan=False)+'\n').encode())
        raw=self.reader.readline(1024*1024)
        if not raw or not raw.endswith(b'\n'):raise EOFError('Native worker disconnected')
        response=json.loads(raw)
        if response.get('status')=='error':raise RuntimeError(response.get('error_msg'))
        return response

    def step(self,request):return self.rpc(request)
    def start_episode(self):self.rpc({'cmd':'reset'})
    def close(self):
        if self.reader:self.reader.close();self.reader=None
        if self.connection:self.connection.close();self.connection=None
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:self.process.wait(timeout=8)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
        if hasattr(self,'log'):self.log.close()
