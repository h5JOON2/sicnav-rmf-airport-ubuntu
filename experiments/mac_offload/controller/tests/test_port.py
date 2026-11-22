import ast,unittest,sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parents[1];sys.path.insert(0,str(HERE))
from clearance30_gate import min_body_clearance
class PortTests(unittest.TestCase):
 def test_30cm_boundary(self):
  x=np.zeros((2,16));x[:,10]=.75;self.assertAlmostEqual(min_body_clearance(x,.25,[.2]),.3)
 def test_between_step_crossing(self):
  x=np.zeros((2,16));x[:,0]=[-1,1];x[:,11]=[-1,1];self.assertAlmostEqual(min_body_clearance(x,.25,[.2]),-.45)
 def test_stationary_clearance(self):
  x=np.zeros((2,16));x[:,10]=.8;self.assertAlmostEqual(min_body_clearance(x,.25,[.2]),.35)
 def test_nonfinite_rejected(self):
  x=np.zeros((2,16));x[0,10]=np.nan
  with self.assertRaises(ValueError):min_body_clearance(x,.25,[.2])
if __name__=='__main__':unittest.main()
