import ast,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
class ForecastTests(unittest.TestCase):
 def forecaster(self,frames,horizon=7):
  source=ast.parse((Path(__file__).resolve().parents[1]/'mp_controller.py').read_text())
  cls=next(x for x in ast.walk(source)if isinstance(x,ast.ClassDef)and x.name=='CachedForecaster')
  forecasts=np.arange(3*15*frames*2).reshape(3,15,frames,2)
  env={'np':np,'controller':SimpleNamespace(horizon=horizon,current={'forecast':{'forecasts':forecasts,'weights':np.zeros((3,15))}})}
  exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),'forecaster','exec'),env)
  return env['CachedForecaster'](),forecasts
 def test_jmid_current_plus_eight_future_frames(self):
  c,f=self.forecaster(9);out,_=c.predict_ret_best()
  np.testing.assert_array_equal(out,f)
  # The policy removes t0, leaving eight frames for seven intervals.
  self.assertEqual(out[:,:,1:,:].shape,(3,15,8,2))
 def test_h4_uses_existing_future_frames_without_extrapolation(self):
  c,f=self.forecaster(9,4);out,_=c.predict_ret_best()
  np.testing.assert_array_equal(out,f)
  # Mirror upstream t0 removal and horizon+1 parameter selection.
  selected=out[:,:,1:,:][:,:,:5,:]
  self.assertEqual(selected.shape,(3,15,5,2))
  np.testing.assert_array_equal(selected,f[:,:,1:6,:])
 def test_h8_preserves_extrapolated_terminal_frame(self):
  c,f=self.forecaster(9,8);out,_=c.predict_ret_best()
  self.assertEqual(out.shape,(3,15,10,2))
  np.testing.assert_array_equal(out[:,:,:9],f)
  np.testing.assert_array_equal(out[:,:,9],2*f[:,:,8]-f[:,:,7])
 def test_missing_current_frame_rejected(self):
  c,_=self.forecaster(8)
  with self.assertRaises(ValueError):c.predict_ret_best()
if __name__=='__main__':unittest.main()
