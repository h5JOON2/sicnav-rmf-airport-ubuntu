import unittest,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from recovery_planner import RecoveryPlanner,velocity_window,swept_wall_gap,swept_human_gap
class RecoveryTests(unittest.TestCase):
 def setUp(self):self.p=RecoveryPlanner()
 def test_open_space_moves(self):
  x=self.p.plan([0,0,0,0,0,3,0],[],[])
  self.assertTrue(x['available']);self.assertGreater(x['action'][0],0);self.assertLessEqual(x['action'][0],.125)
  self.assertFalse(x['mpc_constraints_validated'])
 def test_close_front_restores_by_reverse(self):
  x=self.p.plan([0,0,0,0,0,3,0],[[.65,0,0,0,.2]],[])
  self.assertTrue(x['available']);self.assertLess(x['action'][0],0)
  self.assertEqual(x['kind'],'clearance_restoration');self.assertAlmostEqual(x['predicted_human_gap_m'],.2)
 def test_overlap_stops(self):
  x=self.p.plan([0,0,0,0,0,3,0],[[.4,0,0,0,.2]],[])
  self.assertFalse(x['available']);self.assertEqual(x['action'],[0,0])
 def test_surrounded_stops(self):
  h=[[.65,0,0,0,.2],[-.65,0,0,0,.2],[0,.65,0,0,.2],[0,-.65,0,0,.2]]
  self.assertFalse(self.p.plan([0,0,0,0,0,3,0],h,[])['available'])
 def test_wall_between_samples(self):
  g=swept_wall_gap(np.array([[[-1,0,0],[1,0,0]]]),np.array([[[0,-1],[0,1]]]),.25)
  self.assertAlmostEqual(float(g[0,0,0]),-.25)
 def test_crossing_person(self):
  g=swept_human_gap(np.array([[[-1,0,0],[1,0,0]]]),np.array([[0,-1,0,2,.2]]),np.array([0,1]),.25)
  self.assertAlmostEqual(float(g[0,0,0]),-.45)
 def test_brake_before_reverse(self):
  lo,hi=velocity_window(.2,.25,.5,1.5,.95,.95)
  self.assertAlmostEqual(lo,-.5*(.25-.2/1.5));self.assertAlmostEqual(hi,.325)
 def test_rotation_bounds(self):
  x=self.p.plan([0,0,0,0,0,0,3],[],[])
  self.assertTrue(x['available']);self.assertLessEqual(abs(x['action'][1]),.75)
 def test_invalid(self):
  with self.assertRaises(ValueError):self.p.plan([0,0,0,float('nan'),0,1,0],[],[])
if __name__=='__main__':unittest.main()
