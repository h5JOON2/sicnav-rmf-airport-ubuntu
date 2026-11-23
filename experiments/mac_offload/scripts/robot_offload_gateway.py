#!/usr/bin/env python3
"""Robot-side sensor transport and independent command watchdog. Shadow by default."""
import argparse,json,math,socket,sys,threading,time,uuid
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,ReliabilityPolicy,qos_profile_sensor_data
from message_filters import Subscriber,ApproximateTimeSynchronizer
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from perception.latest_frame import LatestFrame
from remote_runtime.wire import send_packet,receive_packet
from remote_runtime.command_guard import CommandGuard


def seconds(stamp):return stamp.sec+stamp.nanosec*1e-9


class Gateway(Node):
    def __init__(self,args):
        super().__init__('robot_offload_gateway')
        self.set_parameters([Parameter('use_sim_time',value=True)])
        self.args=args;self.stop=threading.Event();self.lock=threading.RLock()
        self.session=str(uuid.uuid4());self.epoch=0;self.state_seq=0;self.scan_seq=0
        self.state=None;self.odom_wall=None;self.frames=LatestFrame();self.last_scan_stamp=None
        self.guard=CommandGuard(self.session,max_age=args.command_max_age)
        self.sockets=set()
        self.log_lock=threading.Lock();self.timing_log=None;self.last_packet=None
        if args.timing_log:
            args.timing_log.parent.mkdir(parents=True,exist_ok=True)
            self.timing_log=args.timing_log.open('a',buffering=1)
        scan_qos=QoSProfile(depth=5,reliability=(ReliabilityPolicy.RELIABLE if args.scan_qos=='reliable' else ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        if args.perception_source=='gt':
            from sicnav_crowd_msgs.msg import CrowdState
            self.filters=[Subscriber(self,CrowdState,args.gt_topic,qos_profile=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)),
                          Subscriber(self,Odometry,'/odom',qos_profile=qos_profile_sensor_data)]
        else:
            self.filters=[Subscriber(self,LaserScan,args.scan_topic,qos_profile=scan_qos),
                          Subscriber(self,Odometry,'/odom',qos_profile=qos_profile_sensor_data)]
        self.sync=ApproximateTimeSynchronizer(self.filters,queue_size=8,slop=.035)
        self.sync.registerCallback(self.on_gt if args.perception_source=='gt' else self.on_scan)
        self.cmd=self.create_publisher(Twist,'/cmd_vel' if args.drive else '/remote/cmd_vel_candidate',10)
        self.status=self.create_publisher(String,'/remote/status',10)
        self.create_timer(.05,self.watchdog)
        self.threads=[threading.Thread(target=self.network,args=(i,f),daemon=True) for i,f in enumerate([self.sensors,self.states,self.commands])]
        for t in self.threads:t.start()
        self.get_logger().info('Offload gateway ready: '+('DRIVE' if args.drive else 'SHADOW (no /cmd_vel output)'))

    def record(self,event,**values):
        if self.timing_log:
            row={'event':event,'session':self.session,'epoch':self.epoch,'wall':time.monotonic(),'unix_s':time.time(),'clock_domain':'ubuntu_monotonic','sim_s':self.get_clock().now().nanoseconds*1e-9,**values}
            with self.log_lock:self.timing_log.write(json.dumps(row,allow_nan=False)+'\n')

    def on_odom(self,msg):
        stamp=seconds(msg.header.stamp);p=msg.pose.pose.position;q=msg.pose.pose.orientation
        theta=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        with self.lock:
            if self.state and stamp<self.state['stamp']:
                self.epoch+=1;self.guard.reset(self.epoch);self.last_scan_stamp=None
            self.state={'session':self.session,'epoch':self.epoch,'stamp':stamp,
                        'robot':{'px':p.x,'py':p.y,'theta':theta,'vx':msg.twist.twist.linear.x*math.cos(theta),
                                 'vy':msg.twist.twist.linear.x*math.sin(theta),'omega':msg.twist.twist.angular.z,
                                 'gx':self.args.goal_x,'gy':self.args.goal_y,'radius':.25,'v_pref':.31}}
            self.odom_wall=time.monotonic()

    def on_scan(self,scan,odom):
        if scan.header.frame_id!=self.args.scan_frame:
            self.get_logger().error('Unexpected LiDAR frame; set --scan-frame and verify mount calibration',throttle_duration_sec=5.)
            return
        if odom.child_frame_id!='base_link' or not odom.header.frame_id:return
        if not 32<=len(scan.ranges)<=4096:return
        r,p,y=self.args.lidar_rpy
        cr,sr=math.cos(r/2),math.sin(r/2);cp,sp=math.cos(p/2),math.sin(p/2);cy,sy=math.cos(y/2),math.sin(y/2)
        mount_q=[sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy]
        with self.lock:
            scan_stamp=seconds(scan.header.stamp)
            if self.last_scan_stamp is not None and scan_stamp<=self.last_scan_stamp:
                self.record('scan_duplicate_or_stale',stamp=scan_stamp,previous_stamp=self.last_scan_stamp)
                return
            self.last_scan_stamp=scan_stamp
            self.scan_seq+=1
            meta={'schema':'laser_scan/v1','session':self.session,'epoch':self.epoch,'scan_seq':self.scan_seq,
                  'stamp':seconds(scan.header.stamp),'odom_stamp':seconds(odom.header.stamp),
                  'frame_id':odom.header.frame_id,'scan_frame_id':scan.header.frame_id,'count':len(scan.ranges),
                  'angle_min':float(scan.angle_min),'angle_increment':float(scan.angle_increment),
                  'range_min':float(scan.range_min),'range_max':float(scan.range_max),
                  'scan_time':float(scan.scan_time),'time_increment':float(scan.time_increment),
                  'lidar_position_base':list(self.args.lidar_position),'lidar_quaternion_base':mount_q,
                  'odom_position':[odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z],
                  'odom_quaternion':[odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w]}
        self.record('scan_callback',scan_seq=meta['scan_seq'],stamp=meta['stamp'],odom_stamp=meta['odom_stamp'],sync_delta_s=meta['odom_stamp']-meta['stamp'])
        self.frames.put(meta['stamp'],(meta,scan))

    def on_gt(self,crowd,odom):
        stamp=seconds(crowd.header.stamp)
        humans=[{'id':int(h.agent_id),'px':h.pose.position.x,'py':h.pose.position.y,
                 'vx':h.twist.linear.x,'vy':h.twist.linear.y,'velocity_valid':bool(h.velocity_valid)}
                for h in crowd.agents if not h.is_external]
        if self.args.expected_gt_people is not None and len(humans)!=self.args.expected_gt_people:
            self.record('gt_invalid',error=f'Expected {self.args.expected_gt_people} authoritative people, got {len(humans)}');return
        meta={'schema':'crowd_gt/v1','session':self.session,'epoch':self.epoch,'stamp':stamp,
              'odom_stamp':seconds(odom.header.stamp),'frame_id':crowd.header.frame_id,
              'source_topic':self.args.gt_topic,'humans':humans}
        from perception.ground_truth import gt_observation
        try:gt_observation({**meta,'scan_seq':0},8)
        except (ValueError,KeyError,TypeError) as exc:
            self.record('gt_invalid',error=str(exc));return
        with self.lock:
            if self.last_scan_stamp is not None and stamp<=self.last_scan_stamp:return
            self.last_scan_stamp=stamp;self.scan_seq+=1;meta['scan_seq']=self.scan_seq;meta['epoch']=self.epoch
        self.record('gt_callback',scan_seq=meta['scan_seq'],stamp=stamp,odom_stamp=meta['odom_stamp'],
                    sync_delta_s=meta['odom_stamp']-stamp,source_topic=self.args.gt_topic,frame_id=meta['frame_id'],humans=humans)
        self.frames.put(stamp,(meta,None))

    def network(self,offset,handler):
        while not self.stop.is_set():
            try:
                with socket.create_connection((self.args.host,self.args.base_port+offset),timeout=2.) as sock:
                    sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1);sock.settimeout(1.)
                    with self.lock:self.sockets.add(sock)
                    try:handler(sock)
                    finally:
                        with self.lock:self.sockets.discard(sock)
            except (OSError,EOFError,ValueError) as exc:
                if rclpy.ok() and not self.stop.is_set():
                    self.get_logger().warning(f'Channel {offset}: {exc}',throttle_duration_sec=5.)
            if offset in (1,2):self.guard.invalidate()
            self.stop.wait(.5)

    def sensors(self,sock):
        while not self.stop.is_set():
            item=self.frames.take()
            if item is None:return
            _,(meta,scan),_=item
            send_packet(sock,meta,b'' if scan is None else np.asarray(scan.ranges,dtype='<f4').tobytes())
            self.record('scan_sent',scan_seq=meta['scan_seq'],stamp=meta['stamp'],frames_received=self.frames.received,frames_replaced=self.frames.replaced)

    def states(self,sock):
        while not self.stop.wait(.05):
            with self.lock:
                if self.state is None or time.monotonic()-self.odom_wall>.5:continue
                self.state_seq+=1
                packet={**self.state,'state_seq':self.state_seq}
            self.guard.note_state(packet['state_seq'],packet['epoch'],packet['stamp'],time.monotonic())
            send_packet(sock,packet)
            self.record('state_sent',state_seq=packet['state_seq'],stamp=packet['stamp'])

    def commands(self,sock):
        while not self.stop.is_set():
            try:packet,_=receive_packet(sock)
            except socket.timeout:
                # A timeout may occur mid-packet: reconnect rather than lose framing.
                raise
            if packet.get('kind')=='heartbeat':continue
            if packet.get('kind')=='halt':
                self.guard.invalidate();self.last_packet=None;self.record('halt_received',reason=packet.get('reason'));continue
            accepted=self.guard.accept(packet,time.monotonic())
            if accepted:self.last_packet=packet
            self.record('command_received',accepted=accepted,rejection_reason=self.guard.last_rejection,state_seq=packet.get('state_seq'),scan_seq=packet.get('scan_seq'),action_stamp=packet.get('action_timestamp'),packet=packet)
            status=String();status.data=json.dumps({'event':'command','accepted':accepted,'packet':packet},allow_nan=False)
            if rclpy.ok() and not self.stop.is_set():self.status.publish(status)

    def watchdog(self):
        now=time.monotonic();sim=self.get_clock().now().nanoseconds*1e-9
        with self.lock:fresh=self.odom_wall is not None and now-self.odom_wall<.5
        v,w=self.guard.command(now,sim) if fresh else (0.,0.)
        msg=Twist();msg.linear.x=v;msg.angular.z=w;self.cmd.publish(msg)
        packet=self.last_packet or {}
        with self.guard.lock:current=self.guard.current
        valid=current is not None and now<=current[2] and 0<=sim-current[3]<=self.args.command_max_age and fresh
        self.record('candidate_published',v=v,omega=w,command_valid=valid,state_seq=packet.get('state_seq'),scan_seq=packet.get('scan_seq'),action_stamp=packet.get('action_timestamp'),drive=self.args.drive)

    def destroy_node(self):
        self.stop.set();self.frames.close();self.guard.invalidate()
        if rclpy.ok():self.cmd.publish(Twist())
        with self.lock:
            for sock in self.sockets:
                try:sock.shutdown(socket.SHUT_RDWR)
                except OSError:pass
        for t in self.threads:t.join(timeout=3.)
        if self.timing_log:self.timing_log.close()
        return super().destroy_node()


def main():
    p=argparse.ArgumentParser();p.add_argument('--host',default='127.0.0.1');p.add_argument('--base-port',type=int,default=9151)
    p.add_argument('--scan-topic',default='/scan');p.add_argument('--scan-frame',default='tb4_lidar')
    p.add_argument('--scan-qos',choices=['best_effort','reliable'],default='best_effort')
    p.add_argument('--lidar-position',nargs=3,type=float,default=[0.,0.,.265])
    p.add_argument('--lidar-rpy',nargs=3,type=float,default=[0.,0.,0.],help='Fixed mount roll/pitch/yaw in radians, relative to base_link')
    p.add_argument('--goal-x',type=float,default=138.);p.add_argument('--goal-y',type=float,default=-35.)
    p.add_argument('--command-max-age',type=float,default=1.5);p.add_argument('--drive',action='store_true')
    p.add_argument('--perception-source',choices=['lidar','gt'],default='lidar')
    p.add_argument('--gt-topic',default='/crowd_sim/states',help='Explicit simulator-only authoritative crowd source')
    p.add_argument('--expected-gt-people',type=int,help='Reject GT frames with an unexpected simulator population')
    p.add_argument('--timing-log',type=Path,help='JSONL stage timestamps in Ubuntu monotonic and ROS simulation clocks')
    args=p.parse_args();rclpy.init();node=Gateway(args)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
