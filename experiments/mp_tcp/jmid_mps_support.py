"""Local, opt-in JMID source overlay; leaves the original checkout untouched."""
import importlib.abc
import importlib.machinery
import os
from pathlib import Path
import sys

ROOT = Path(os.environ.get('SICNAV_MP_ROOT', Path(__file__).resolve().parents[2]))
REPO = ROOT / 'work/repo'
MID = REPO / 'sicnav_diffusion/JMID/MID'


def install(device):
    import torch
    if device not in ('cpu', 'mps'):
        raise ValueError(device)
    if device == 'mps' and not torch.backends.mps.is_available():
        raise RuntimeError('MPS unavailable in this process; no CPU substitution performed')
    os.environ['JMID_DEVICE'] = device
    overlay = Path(__file__).resolve().parent / 'jmid_mps_overlay'
    overlay.mkdir(exist_ok=True)
    source = (MID / 'mid.py').read_text()
    needle = "self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')"
    assert source.count(needle) == 1
    source = source.replace(needle, "self.device = torch.device(__import__('os').environ['JMID_DEVICE'])")
    source = source.replace('"cuda" if torch.cuda.is_available() else "cpu"', 'self.device')
    source = source.replace('map_location=self.device', 'map_location="cpu"')
    source = source.replace('self.registrar.load_models(self.checkpoint["encoder"])',
                            'self.registrar.load_models(self.checkpoint["encoder"])\n            self.registrar.to(self.device)', 1)
    # This registrar contains modules outside the diffusion model's state_dict.
    old = '        if torch.cuda.is_available():\n            self.model = model.cuda()\n        else:\n            self.model = model.cpu()'
    assert source.count(old) == 1
    source = source.replace(old, '        self.model = model.to(self.device)')
    old = '        data = data + np.expand_dims(node.get_mean_x_and_y(), axis=0)'
    assert source.count(old) == 1
    source = source.replace(old,
        '        offset = np.expand_dims(node.get_mean_x_and_y(), axis=0)\n'
        '        if isinstance(data, torch.Tensor) and data.device.type == "mps":\n'
        '            offset = torch.as_tensor(offset, dtype=data.dtype, device=data.device)\n'
        '        data = data + offset')
    (overlay / 'mid.py').write_text(source)
    diffusion = (MID / 'models/diffusion.py').read_text()
    # The old code copied every discarded diffusion state to CPU each iteration.
    old = '            traj[t] = traj[t].cpu()  # Move previous output to CPU memory.\n            if not ret_traj:\n                del traj[t]'
    assert diffusion.count(old) == 1
    diffusion = diffusion.replace(old, '            if ret_traj:\n                traj[t] = traj[t].cpu()\n            else:\n                del traj[t]')
    (overlay / 'diffusion.py').write_text(diffusion)
    targets = {str(MID / 'mid.py'): overlay / 'mid.py',
               str(MID / 'models/diffusion.py'): overlay / 'diffusion.py'}

    class OverlayFinder(importlib.abc.MetaPathFinder):
        def find_spec(self, fullname, path=None, target=None):
            spec = importlib.machinery.PathFinder.find_spec(fullname, path)
            if spec is not None and spec.origin in targets:
                spec.loader = importlib.machinery.SourceFileLoader(fullname, str(targets[spec.origin]))
            return spec if spec is not None and spec.origin in targets else None

    sys.meta_path.insert(0, OverlayFinder())
