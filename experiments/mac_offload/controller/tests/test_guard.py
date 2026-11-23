"""Execute the real tick method without ROS, with fresh sensor observations."""
import ast,unittest,math,threading
from pathlib import Path
from types import SimpleNamespace as S
src=ast.parse((Path(__file__).resolve().parents[1]/'closed_loop_eval.py').read_text());cls=next(x for x in src.body if isinstance(x,ast.ClassDef)and x.name=='Evaluation');tick=next(x for x in cls.body if isinstance(x,ast.FunctionDef)and x.name=='tick')
class FakeTwist:pass
ns={'math':math,'time':S(monotonic=lambda:0),'Twist':FakeTwist};exec(compile(ast.fix_missing_locations(ast.Module(body=[tick],type_ignores=[])),'tick','exec'),ns)
def run(legacy):
 sent=[];events=[];x=S(done=False,wall_start=0,a=S(wall_timeout=100,timeout=60,legacy_grid_freshness=legacy),start_sim=0,holding_goal=False,busy=False,now=lambda:10.18,lock=threading.RLock(),robot={'stamp':10.16},crowd_stamp=10.16,humans=[{}, {}, {}],last_request=10.,pub=S(publish=sent.append),record=lambda *args,**kwargs:events.append((args,kwargs)))
 ns['tick'](x);return sent,events
class GuardTest(unittest.TestCase):
 def test_legacy_stops_with_fresh_20ms_old_sensors(self):
  sent,events=run(True);self.assertEqual(len(sent),1);self.assertEqual(events[0][0],('guard_stop',));self.assertAlmostEqual(events[0][1]['robot_actual_age'],.02)
 def test_fixed_holds_command_until_next_grid(self):
  sent,events=run(False);self.assertEqual(sent,[]);self.assertEqual(events,[])
if __name__=='__main__':unittest.main()
