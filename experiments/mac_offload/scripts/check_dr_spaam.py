"""Verify CPU reference parity and benchmark actual recorded LiDAR on CPU/MPS."""
import sys,json,time,math,hashlib,statistics
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import torch
from perception.dr_spaam import DrSpaamDetector
from perception.lidar import prepare_drow_scan
root=Path(__file__).resolve().parents[1]
source=root/'vendor/2D_lidar_person_detection/dr_spaam';checkpoint=root/'models/ckpt_jrdb_ann_dr_spaam_e20.pth'
sys.path.insert(0,str(source))
from dr_spaam.detector import Detector
sample=np.load(root/'samples/actual_lidar_720.npz');ranges=sample['ranges']
torch.set_num_threads(2)
ours=DrSpaamDetector(source,checkpoint,'cpu')
reference=Detector(str(checkpoint),model='DR-SPAAM',gpu=False,panoramic_scan=True)
reference.set_laser_fov(360)
for scan in ranges[:5]:
    normalized,_,_=prepare_drow_scan(scan,-math.pi,2*math.pi/719,.15,12.)
    actual=ours.raw(normalized,360);expected=reference(normalized)
    np.testing.assert_allclose(actual[0],expected[0],atol=1e-6,rtol=1e-6)
    np.testing.assert_allclose(actual[1],expected[1],atol=1e-6,rtol=1e-6)
print('CPU reference parity: PASS (5 real scans)',flush=True)
report={'cpu_reference_parity_scans':5,'checkpoint_sha256':hashlib.sha256(checkpoint.read_bytes()).hexdigest(),'devices':{}}
for device in ['cpu','mps']:
    if device=='mps' and not torch.backends.mps.is_available():raise RuntimeError('MPS unavailable')
    detector=DrSpaamDetector(source,checkpoint,device)
    timings=[];hits=[];example=[]
    for i,scan in enumerate(ranges[:35]):
        header={'stamp':i*.1,'angle_min':-math.pi,'angle_increment':2*math.pi/719,'range_min':.15,'range_max':12.}
        start=time.perf_counter();xy,scores=detector.detect(scan,header);elapsed=time.perf_counter()-start
        if i>=5:
            timings.append(elapsed);hits.append(len(xy))
            if len(xy):example=[{'xy_ros':p.tolist(),'distance_m':float(np.linalg.norm(p)),'angle_deg':float(np.degrees(np.arctan2(p[1],p[0]))),'score':float(s)}for p,s in zip(xy,scores)]
    order=sorted(timings)
    report['devices'][device]={'n':len(timings),'mean_s':statistics.mean(timings),'median_s':statistics.median(timings),'p95_s':order[math.ceil(.95*len(order))-1],'max_s':max(timings),'frames_with_person':sum(n>0 for n in hits),'neural_calls':detector.neural_calls,'example':example}
    print(device,json.dumps(report['devices'][device]),flush=True)
(root/'logs/lidar/model_verification.json').write_text(json.dumps(report,indent=2))
