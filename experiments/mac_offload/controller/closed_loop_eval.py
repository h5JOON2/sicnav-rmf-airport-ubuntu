"""Actual Ubuntu ROS/Gazebo to Mac TCP evaluation, with authoritative crowd states."""
import argparse,json,math,socket,time,threading
from concurrent.futures import ThreadPoolExecutor
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sicnav_crowd_msgs.msg import CrowdState

class Evaluation(Node):
 def __init__(self,a):
  super().__init__('sicnav_closed_loop_evaluation')
  self.set_parameters([Parameter('use_sim_time',value=True)])
  self.a=a;self.lock=threading.RLock();self.loglock=threading.Lock();self.log=open(a.log,'w',buffering=1)
  self.robot=None;self.humans=[];self.crowd_stamp=None;self.last_request=None;self.start_sim=None
  self.wall_start=time.monotonic();self.done=False;self.goal_since=None;self.holding_goal=False;self.busy=False
  self.pool=ThreadPoolExecutor(max_workers=1);self.sock=None;self.buffer=b'';self.index=0
  self.pub=self.create_publisher(Twist,'/cmd_vel',10)
  self.create_subscription(Odometry,'/odom',self.odom,qos_profile_sensor_data)
  self.create_subscription(CrowdState,'/crowd_sim/states',self.crowd,10)
  self.create_timer(.02,self.tick)
  self.record('start',goal=[a.gx,a.gy],tolerance=a.tolerance,settle_s=1.,timeout_sim=a.timeout)
 def now(self):return self.get_clock().now().nanoseconds*1e-9
 def record(self,kind,**kw):
  with self.loglock:self.log.write(json.dumps({'event':kind,'wall':time.monotonic(),'sim':self.now(),**kw})+'\n')
 def stop(self,reason):
  if not self.done:
   self.done=True;self.pub.publish(Twist());self.record('stop',reason=reason)
 def odom(self,m):
  p=m.pose.pose.position;q=m.pose.pose.orientation
  yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
  v=m.twist.twist.linear.x;w=m.twist.twist.angular.z;t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  with self.lock:
   previous=self.robot
   dt=t-previous['stamp'] if previous is not None else 0.
   acceleration=(v-previous['v'])/dt if dt>1e-6 else 0.
   angular_acceleration=(w-previous['omega'])/dt if dt>1e-6 else 0.
   self.robot={'px':p.x,'py':p.y,'vx':v*math.cos(yaw),'vy':v*math.sin(yaw),'theta':yaw,'omega':w,'stamp':t,'v':v,'v_dot':acceleration,'omega_dot':angular_acceleration}
  d=math.hypot(p.x-self.a.gx,p.y-self.a.gy)
  self.record('odom',x=p.x,y=p.y,yaw=yaw,v=v,omega=w,stamp=t,goal_distance=d)
  if self.start_sim is not None and not self.done:
   if d<=self.a.tolerance and not self.holding_goal:
    self.holding_goal=True;self.pub.publish(Twist());self.record('goal_region_entered',distance=d)
   if self.holding_goal:
    self.pub.publish(Twist())
    if d>self.a.tolerance:
     self.holding_goal=False;self.goal_since=None;self.record('goal_region_exited',distance=d)
   if d<=self.a.tolerance and abs(v)<.1:
    if self.goal_since is None:self.goal_since=t
    if t-self.goal_since>=1.:
     self.record('goal_reached',distance=d);self.stop('goal_reached')
   else:self.goal_since=None
 def crowd(self,m):
  t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  humans=[{'id':int(h.agent_id),'px':h.pose.position.x,'py':h.pose.position.y,'vx':h.twist.linear.x,'vy':h.twist.linear.y,'radius':.2,'valid':bool(h.velocity_valid)} for h in m.agents if not h.is_external]
  with self.lock:self.humans=sorted(humans,key=lambda h:h['id']);self.crowd_stamp=t
  self.record('crowd',stamp=t,humans=humans)
 def tick(self):
  if self.done:self.pub.publish(Twist());return
  if time.monotonic()-self.wall_start>self.a.wall_timeout:self.stop('wall_timeout');return
  observation_time=self.now();t=observation_time
  if self.start_sim is not None and t-self.start_sim>self.a.timeout:self.stop('simulation_timeout');return
  if self.holding_goal:
   self.pub.publish(Twist());return
  if self.busy:return
  t=math.floor((t+1e-9)/.25)*.25
  with self.lock:
   if self.robot is None or self.crowd_stamp is None or len(self.humans)!=3:return
   r=dict(self.robot);humans=[dict(h) for h in self.humans];ht=self.crowd_stamp
  ra=t-r['stamp'];ha=t-ht
  check_ra=ra if self.a.legacy_grid_freshness else observation_time-r['stamp']
  check_ha=ha if self.a.legacy_grid_freshness else observation_time-ht
  if not -.03<=check_ra<.25 or not -.03<=check_ha<.25:
   self.pub.publish(Twist());self.record('guard_stop',grid_time=t,robot_grid_age=ra,human_grid_age=ha,robot_actual_age=observation_time-r['stamp'],human_actual_age=observation_time-ht);return
  if self.last_request is not None and t-self.last_request<.25-1e-9:return
  if self.start_sim is None:self.start_sim=t
  for key,vel in [('px','vx'),('py','vy')]:r[key]+=r[vel]*ra
  r['theta']+=r['omega']*ra
  r.update(gx=self.a.gx,gy=self.a.gy,radius=.25,v_pref=.9)
  for h in humans:
   h['px']+=h['vx']*ha;h['py']+=h['vy']*ha;h.pop('valid')
  request={'timestamp':t,'robot':r,'humans':humans}
  self.index+=1;self.last_request=t;self.busy=True
  self.record('request',index=self.index,payload=request,robot_age=ra,human_age=ha)
  self.pool.submit(self.exchange,self.index,request)
 def exchange(self,index,request):
  start=time.monotonic()
  try:
   if self.sock is None:
    self.sock=socket.create_connection((self.a.host,self.a.port),timeout=5);self.sock.settimeout(5)
   self.sock.sendall((json.dumps(request)+'\n').encode())
   while b'\n' not in self.buffer:
    block=self.sock.recv(65536)
    if not block:raise ConnectionError('server closed')
    self.buffer+=block
   line,self.buffer=self.buffer.split(b'\n',1);resp=json.loads(line)
   age=self.now()-request['timestamp'];apply=not self.done and not self.holding_goal and 0<=age<=self.a.max_age and resp.get('status')=='success'
   a=resp.get('action',{});v=float(a.get('v',0));w=float(a.get('r',0))
   apply=apply and math.isfinite(v) and math.isfinite(w)
   command=Twist()
   if apply:command.linear.x=v;command.angular.z=w
   self.pub.publish(command)
   self.record('command',index=index,input_stamp=request['timestamp'],age=age,roundtrip_ms=1000*(time.monotonic()-start),applied=apply,v=command.linear.x,omega=command.angular.z,response=resp)
  except Exception as e:
   self.pub.publish(Twist());self.record('error',index=index,message=str(e));self.stop('transport_error')
  finally:self.busy=False

def main():
 p=argparse.ArgumentParser();p.add_argument('--host',default='127.0.0.1');p.add_argument('--port',type=int,default=19090)
 p.add_argument('--gx',type=float,required=True);p.add_argument('--gy',type=float,required=True);p.add_argument('--log',required=True)
 p.add_argument('--timeout',type=float,default=60);p.add_argument('--wall-timeout',type=float,default=240);p.add_argument('--tolerance',type=float,default=.5);p.add_argument('--max-age',type=float,default=.5)
 p.add_argument('--legacy-grid-freshness',action='store_true')
 a=p.parse_args();rclpy.init();node=Evaluation(a)
 try:
  while rclpy.ok() and not node.done:rclpy.spin_once(node,timeout_sec=.1)
 finally:
  node.stop('interrupted');node.pub.publish(Twist());node.pool.shutdown(wait=True)
  if node.sock:node.sock.close()
  node.log.close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
