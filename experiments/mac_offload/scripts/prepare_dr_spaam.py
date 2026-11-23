"""Fetch the pinned official network and verify the exact checkpoint used here."""
import hashlib,subprocess,urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
COMMIT='d12e90225d52ba6c85d20061f1e7286d6a64aade'
SHA='1b62e50d9bcb954975bda28610e09921e8bde94216b0a291a704a579690b6324'
source=ROOT/'vendor/2D_lidar_person_detection'
source.parent.mkdir(exist_ok=True)
if not source.exists():subprocess.run(['git','clone','--depth','1','https://github.com/RWTHVision/2D_lidar_person_detection.git',str(source)],check=True)
actual=subprocess.check_output(['git','-C',str(source),'rev-parse','HEAD'],text=True).strip()
if actual!=COMMIT:raise RuntimeError('Unexpected upstream version; do not silently replace reviewed code: '+actual)
patches={
 'dr_spaam/dr_spaam/utils/utils.py':('.astype(np.int)','.astype(np.int64)'),
 'dr_spaam/dr_spaam/model/dr_spaam.py':('return masks.cuda(x.get_device()) if x.is_cuda else masks, inds_full','return masks.to(device=x.device), inds_full.to(device=x.device)'),
 'dr_spaam/dr_spaam/detector.py':('torch.load(ckpt_file)','torch.load(ckpt_file, map_location="cpu", weights_only=False)')}
for name,(before,after) in patches.items():
 p=source/name;content=p.read_text()
 if before not in content and after not in content:raise RuntimeError('Cannot apply reviewed compatibility patch: '+name)
 p.write_text(content.replace(before,after))
checkpoint=ROOT/'models/ckpt_jrdb_ann_dr_spaam_e20.pth';checkpoint.parent.mkdir(exist_ok=True)
if not checkpoint.exists():
 temporary=checkpoint.with_suffix('.download')
 urllib.request.urlretrieve('https://drive.usercontent.google.com/download?id=1gH_pdxUC8diIFUe3_Ky8QiVVCyiR3500&export=download&confirm=t',temporary)
 if hashlib.sha256(temporary.read_bytes()).hexdigest()!=SHA:raise RuntimeError('Checkpoint checksum mismatch')
 temporary.replace(checkpoint)
if hashlib.sha256(checkpoint.read_bytes()).hexdigest()!=SHA:raise RuntimeError('Checkpoint checksum mismatch')
print('Pinned DR-SPAAM prepared:',COMMIT,'checkpoint SHA256:',SHA)
