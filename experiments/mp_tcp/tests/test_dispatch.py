import unittest,sys
from pathlib import Path
from types import SimpleNamespace as S
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mp_controller import MPController
from recovery_planner import RecoveryPlanner
class DispatchTests(unittest.TestCase):
 def setUp(self):
  self.c=MPController.__new__(MPController);self.c.recovery_mode='motion';self.c.recovery_planner=RecoveryPlanner();self.c.obstacles=[]
  self.state=S(self_state=S(px=0,py=0,theta=0,lvel=0,omega=0,gx=3,gy=0),human_states=[])
 def test_no_feasible_moves_and_preserves_selection(self):
  control={'output_available':False,'action':[0,0],'selected_iteration':None}
  action,source,recovery=self.c.select_output(control,self.state)
  self.assertGreater(action[0],0);self.assertEqual(source,'recovery_motion')
  self.assertFalse(control['output_available']);self.assertEqual(control['action'],[0,0])
  self.assertFalse(recovery['mpc_constraints_validated'])
 def test_feasible_unchanged(self):
  a,s,r=self.c.select_output({'output_available':True,'action':[.2,.1],'selected_iteration':2},self.state)
  self.assertEqual(a,[.2,.1]);self.assertEqual(s,'updated_feasible');self.assertIsNone(r)
 def test_baseline_option(self):
  self.c.recovery_mode='stop';a,s,r=self.c.select_output({'output_available':False},self.state)
  self.assertEqual(a,[0,0]);self.assertEqual(s,'no_feasible_stop')
if __name__=='__main__':unittest.main()
