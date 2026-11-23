"""Explicit simulator-only GT transport; never used as LiDAR fallback."""
import math

def gt_observation(header,maximum):
    if header.get('schema')!='crowd_gt/v1':raise ValueError('Expected simulator GT packet')
    stamp=float(header['stamp']);odom_stamp=float(header['odom_stamp'])
    if not math.isfinite(stamp) or not math.isfinite(odom_stamp):raise ValueError('finite GT timestamps required')
    people=sorted(header['humans'],key=lambda h:h['id']);ids=[];slots=[]
    for h in people:
        if type(h['id']) is not int or h['id'] in ids:raise ValueError('unique integer GT IDs required')
        values=[float(h[k]) for k in ('px','py','vx','vy')]
        if not all(math.isfinite(v) for v in values):raise ValueError('finite GT states required')
        if not h.get('velocity_valid'):raise ValueError('GT velocity unavailable')
        ids.append(h['id']);slots.append({'id':h['id'],'px':values[0],'py':values[1],'vx':values[2],'vy':values[3],'radius':.25,'stamp':stamp})
    return {'session':header['session'],'epoch':header['epoch'],'stamp':stamp,'scan_seq':header['scan_seq'],
            'slots':slots,'variable_humans':True,'objects_xyz':[[h['px'],h['py'],0.] for h in slots],
            'sensor':'simulator_gt','detector':'Menge authoritative state','device':'not_applicable',
            'position_kind':'authoritative_simulator_xy','source_topic':header.get('source_topic'),
            'frame_id':header['frame_id'],'detections':[{'agent_id':h['id'],'position_m':[h['px'],h['py'],0.]} for h in slots],
            'scan_count':len(slots),'odom_time_offset_s':odom_stamp-stamp,'capacity_exceeded':len(slots)>maximum}
