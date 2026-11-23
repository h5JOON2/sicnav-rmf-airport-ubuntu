import sys,json,time,math,statistics
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import torch,numpy as np
from perception.dr_spaam import DrSpaamDetector
root=Path(__file__).resolve().parents[1];torch.set_num_threads(1)
d=DrSpaamDetector(root/'vendor/2D_lidar_person_detection/dr_spaam',root/'models/ckpt_jrdb_ann_dr_spaam_e20.pth','mps')
samples=np.load(root/'samples/actual_lidar_720.npz')['ranges']
rows=[]
for i,ranges in enumerate(samples[:25]):
 h={'stamp':i*.1,'angle_min':-math.pi,'angle_increment':2*math.pi/719,'range_min':.15,'range_max':12.}
 t=time.monotonic();xy,scores=d.detect(ranges,h);elapsed=time.monotonic()-t
 rows.append({'seconds':elapsed,'count':len(xy),'xy_ros':xy.tolist(),'scores':scores.tolist()})
 if i in (0,1,4,24):print(i,elapsed,'people',len(xy),flush=True)
v=[r['seconds']for r in rows[5:]]
report={'device':'mps','model_parameter_device':str(next(d.model.parameters()).device),'warmup':5,'n':len(v),
        'mean_s':statistics.mean(v),'max_s':max(v),'neural_calls':d.neural_calls,'rows':rows}
(root/'logs/lidar/mps_verification.json').write_text(json.dumps(report,indent=2))
print('MPS verified',report['mean_s'],flush=True)
