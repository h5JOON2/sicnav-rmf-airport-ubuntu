#!/usr/bin/env python3
"""Mac LiDAR/DR-SPAAM inference, separated state/command channels and SICNav RPC."""
import argparse,json,math,socket,sys,threading,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import torch
from perception.dr_spaam import DrSpaamDetector
from perception.ground_truth import gt_observation
from perception.lidar import decode_scan,lidar_points_to_odom
from perception.latest_frame import LatestFrame
from remote_runtime.wire import receive_packet,send_packet
from remote_runtime.tracker import PersonSlots,observation_compatible


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--perception-source',choices=['lidar','gt'],default='lidar')
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--base-port',type=int,default=9151)
    parser.add_argument('--controller-port',type=int,default=9150)
    parser.add_argument('--humans',type=int,choices=range(1,9),default=3,help='Maximum observed people; fewer people, including zero, are supported')
    parser.add_argument('--control-interval',type=float,default=.25)
    parser.add_argument('--model',required=True,help='Official DR-SPAAM checkpoint')
    parser.add_argument('--detector-source',default=str(Path(__file__).resolve().parents[1]/'vendor/2D_lidar_person_detection/dr_spaam'))
    parser.add_argument('--confidence',type=float,default=.5)
    parser.add_argument('--stride',type=int,default=1)
    parser.add_argument('--device',default='mps')
    parser.add_argument('--log',type=Path,required=True)
    args=parser.parse_args()
    if args.perception_source=='lidar' and args.device=='mps' and not torch.backends.mps.is_available():
        raise RuntimeError('MPS unavailable; choose --device cpu explicitly')
    model=DrSpaamDetector(args.detector_source,args.model,args.device,args.confidence,args.stride) if args.perception_source=='lidar' else None
    lock=threading.RLock();loglock=threading.Lock();stop=threading.Event();frames=LatestFrame()
    shared={'state':None,'observation':None,'command':None,'epoch_key':None,'sensor_generation':0,'input_version':0,'command_version':0}
    inputs_changed=threading.Condition(lock);commands_changed=threading.Condition(lock)
    def set_command(command):
        with commands_changed:
            shared['command']=command;shared['command_version']+=1
            commands_changed.notify_all()
    def input_changed():
        # Called while holding the shared lock; no wake-up can be lost.
        shared['input_version']+=1;inputs_changed.notify_all()
    args.log.parent.mkdir(parents=True,exist_ok=True)
    log=args.log.open('a',buffering=1)
    def record(kind,**values):
        with loglock:log.write(json.dumps({'kind':kind,'wall':time.monotonic(),'unix_s':time.time(),'clock_domain':'mac_monotonic',**values},allow_nan=False)+'\n')
    def listener(offset,handler):
        server=socket.socket();server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        server.bind((args.host,args.base_port+offset));server.listen(1);server.settimeout(.5)
        while not stop.is_set():
            try:c,_=server.accept()
            except socket.timeout:continue
            with c:
                c.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1);c.settimeout(2.)
                if offset==0:
                    with lock:shared['sensor_generation']+=1
                try:handler(c)
                except (OSError,EOFError,ValueError,KeyError,TypeError) as exc:record('channel_error',channel=offset,error=str(exc))
            if offset==0:
                with lock:
                    shared['sensor_generation']+=1
                    shared['observation']=None;set_command({'kind':'halt','reason':'lidar_disconnected'});input_changed()
            if offset==1:
                with lock:shared['state']=None;set_command({'kind':'halt','reason':'state_disconnected'});input_changed()
        server.close()
    def sensor_channel(c):
        while not stop.is_set():
            header,payload=receive_packet(c)
            if header.get('schema')!=('crowd_gt/v1' if args.perception_source=='gt' else 'laser_scan/v1') or not math.isfinite(float(header['stamp'])):
                raise ValueError('Expected stamped LiDAR packet')
            with lock:generation=shared['sensor_generation']
            record('scan_received',session=header['session'],epoch=header['epoch'],scan_seq=header['scan_seq'],stamp=header['stamp'])
            frames.put(float(header['stamp']),(header,payload,generation))
    def state_channel(c):
        while not stop.is_set():
            header,_=receive_packet(c)
            key=(header['session'],header['epoch'])
            with lock:
                if key!=shared['epoch_key']:
                    shared['observation']=None;set_command(None);shared['epoch_key']=key
                shared['state']=(header,time.monotonic());input_changed()
    def command_channel(c):
        version=-1;last_sent=time.monotonic()
        while not stop.is_set():
            with commands_changed:
                commands_changed.wait_for(
                    lambda: stop.is_set() or shared['command_version']!=version,
                    timeout=max(0.,.2-(time.monotonic()-last_sent)))
                if stop.is_set():break
                command=shared['command'];version=shared['command_version']
            send_packet(c,command if command is not None else {'kind':'heartbeat'})
            last_sent=time.monotonic()
            if command is not None:
                record('command_sent',session=command.get('session'),epoch=command.get('epoch'),state_seq=command.get('state_seq'),scan_seq=command.get('scan_seq'),command_kind=command.get('kind','action'))
    def perception():
        tracker=PersonSlots(args.humans,retain_missing=False);key=None
        while not stop.is_set():
            item=frames.take()
            if item is None:return
            generation,(h,payload,stream_generation),received=item
            try:
                source_key=(h['session'],h['epoch'])
                with lock:
                    if source_key!=shared['epoch_key'] or stream_generation!=shared['sensor_generation']:continue
                if (source_key,stream_generation)!=key:
                    tracker=PersonSlots(args.humans,retain_missing=False)
                    if model is not None:model.reset()
                    key=(source_key,stream_generation)
                t=time.monotonic()
                if args.perception_source=='gt':
                    observation=gt_observation(h,args.humans)
                    observation.update(process_wall_s=time.monotonic()-t,queue_wall_s=t-received,process_start_wall=t,received_wall=received,frames_received=frames.received,frames_replaced=frames.replaced)
                else:
                    ranges=decode_scan(h,payload)
                    sensor_xy,scores=model.detect(ranges,h)
                    world_xyz=lidar_points_to_odom(sensor_xy,h)
                    points=world_xyz.tolist()
                    detections=[{'sensor_xy_m':xy.tolist(),'position_m':xyz.tolist(),
                                 'distance_m':float(np.linalg.norm(xy)),
                                 'angle_deg':float(np.degrees(np.arctan2(xy[1],xy[0]))),
                                 'confidence':float(score)}for xy,xyz,score in zip(sensor_xy,world_xyz,scores)]
                    # Prioritize nearby observations while keeping existing slots stable.
                    points.sort(key=lambda p:math.dist(p[:2],h['odom_position'][:2]))
                    slots=tracker.update(h['stamp'],points)
                    observation={'session':h['session'],'epoch':h['epoch'],'stamp':h['stamp'],
                                 'scan_seq':h['scan_seq'],'slots':slots,'objects_xyz':points,'variable_humans':True,
                                 'sensor':'lidar','detector':'DR-SPAAM','device':args.device,
                                 'position_kind':'lidar_person_center_xy_at_scan_plane','detections':detections,
                                 'scan_count':h['count'],'odom_time_offset_s':h['odom_stamp']-h['stamp'],
                                 'capacity_exceeded':len(points)>args.humans,
                                 'process_wall_s':time.monotonic()-t,'queue_wall_s':t-received,
                                 'process_start_wall':t,'received_wall':received,'frames_received':frames.received,'frames_replaced':frames.replaced}
                with lock:
                    if source_key!=shared['epoch_key'] or stream_generation!=shared['sensor_generation']:continue
                    shared['observation']=(observation,time.monotonic());input_changed()
                record('perception',**observation)
            except Exception as exc:record('perception_error',error=str(exc))
    def planner():
        connection=None;reader=None;slot_key=None;last_seq=-1;last_request_stamp=None;version=-1
        while not stop.is_set():
            with inputs_changed:
                inputs_changed.wait_for(lambda: stop.is_set() or shared['input_version']!=version,timeout=.1)
                if stop.is_set():break
                version=shared['input_version'];state=shared['state'];observation=shared['observation']
            if not state or not observation:continue
            state,sr=state;observation,orr=observation
            now=time.monotonic();age=state['stamp']-observation['stamp']
            if now-sr>.5 or now-orr>1.5 or age>1.:
                set_command(None)
                continue
            # Synchronized observations can arrive just before the newer robot
            # state on its separate channel. Wait for that state without erasing
            # a fresh computed command that the sender has not delivered yet.
            if age<0:continue
            if observation['capacity_exceeded']:
                set_command({'kind':'halt','reason':'over_capacity_perception'})
                continue
            if state['state_seq']==last_seq:continue
            if last_request_stamp is not None and 0<=state['stamp']-last_request_stamp<args.control_interval:continue
            observed=[v for v in observation['slots'] if v is not None and v['stamp']==observation['stamp']]
            ids=tuple(v['id'] for v in observed)
            identity=(state['session'],state['epoch'])
            try:
                if identity!=slot_key and connection:
                    connection.close();connection=None;reader=None
                if connection is None:
                    connection=socket.create_connection(('127.0.0.1',args.controller_port),timeout=2.)
                    connection.settimeout(20.);reader=connection.makefile('rb');slot_key=identity;last_request_stamp=None
                humans=[]
                for value in observed:
                    dt=state['stamp']-value['stamp']
                    humans.append({**value,'px':value['px']+value['vx']*dt,'py':value['py']+value['vy']*dt})
                request={'timestamp':state['stamp'],'robot':state['robot'],'humans':humans}
                last_request_stamp=state['stamp']
                t=time.monotonic()
                record('controller_request',session=state['session'],epoch=state['epoch'],state_seq=state['state_seq'],scan_seq=observation['scan_seq'],stamp=state['stamp'],scan_stamp=observation['stamp'],observation_age_sim_s=age,observed_humans=len(humans),payload=request,perception_source=args.perception_source)
                connection.sendall((json.dumps(request,allow_nan=False)+'\n').encode())
                raw=reader.readline(1024*1024)
                if not raw or not raw.endswith(b'\n'):raise EOFError('Invalid controller response')
                response=json.loads(raw);last_seq=state['state_seq']
                record('controller',session=state['session'],epoch=state['epoch'],state_seq=last_seq,scan_seq=observation['scan_seq'],stamp=state['stamp'],scan_stamp=observation['stamp'],slot_ids=ids,rpc_wall_s=time.monotonic()-t,response=response)
                if response.get('status')!='success':
                    set_command({'kind':'halt','reason':'controller_error'})
                    continue
                if response.get('control_updated',True):
                    packet={**response,'action_timestamp':response.get('action_timestamp',response['timestamp']),
                            'session':state['session'],'epoch':state['epoch'],
                            'state_seq':last_seq,'scan_seq':observation['scan_seq']}
                    with lock:
                        latest=shared['observation']
                        latest_obs=None if latest is None else latest[0]
                        compatible=observation_compatible(latest_obs,ids)
                        if (state['session'],state['epoch'])==shared['epoch_key'] and compatible:
                            set_command(packet)
            except Exception as exc:
                record('controller_error',error=str(exc))
                if connection:connection.close()
                connection=None;reader=None
                set_command(None)
                stop.wait(.5)
        if connection:connection.close()
    threads=[threading.Thread(target=listener,args=(i,f),daemon=True) for i,f in enumerate([sensor_channel,state_channel,command_channel])]
    threads += [threading.Thread(target=perception,daemon=True),threading.Thread(target=planner,daemon=True)]
    for t in threads:t.start()
    print('Mac offload server ready on',args.host,args.base_port,'device',args.device,'source',args.perception_source,flush=True)
    try:
        while not stop.wait(1.):pass
    except KeyboardInterrupt:pass
    finally:
        stop.set();frames.close()
        with inputs_changed:inputs_changed.notify_all()
        with commands_changed:commands_changed.notify_all()
        for t in threads:t.join(timeout=3.)
        # Threads may still be finishing an external RPC; log remains open until process exit.

if __name__=='__main__':main()
