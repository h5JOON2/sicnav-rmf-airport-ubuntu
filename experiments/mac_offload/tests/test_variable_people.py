import ast,math,sys,time,unittest
from pathlib import Path
from types import SimpleNamespace as S
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'controller'))
from variable_people import VariablePeopleController
from remote_runtime.tracker import PersonSlots,observation_compatible

def request(stamp,count):
    return {'timestamp':stamp,'robot':{'px':0.,'py':0.,'theta':0.,'vx':0.,'vy':0.,'omega':0.,'gx':3.,'gy':0.},
            'humans':[{'id':i+1,'px':2.,'py':i+2.,'vx':0.,'vy':0.} for i in range(count)]}

class Worker:
    def __init__(self,count):self.count=count;self.calls=[];self.resets=0;self.closed=False
    def start_episode(self):self.resets+=1
    def step(self,r):
        assert len(r['humans'])==self.count
        self.calls.append(r)
        return {'status':'success','action':{'v':.1,'r':0.},'timestamp':r['timestamp'],'output_source':'test_worker'}
    def close(self):self.closed=True

class VariablePeopleTests(unittest.TestCase):
    def test_actual_count_routes_3_to_1_to_2_without_padding(self):
        router=VariablePeopleController(3,Worker)
        for stamp,count in [(1.,3),(1.25,1),(1.5,2)]:
            r=request(stamp,count);response=router.step(r)
            self.assertEqual(response['observed_humans'],count)
            self.assertIs(router.workers[count].calls[-1],r)
        self.assertEqual([len(w.calls) for w in router.workers.values()],[1,1,1])

    def test_fresh_zero_detection_runs_robot_only_plan(self):
        router=VariablePeopleController(3,Worker)
        response=router.step(request(1.,0))
        self.assertEqual(response['observed_humans'],0)
        self.assertEqual(response['output_source'],'no_observed_people_local_plan')
        self.assertGreater(response['action']['v'],0.)
        self.assertLessEqual(abs(response['action']['v']),.3)
        self.assertLessEqual(abs(response['action']['r']),.5)
        self.assertFalse(response['mpc_feasible'])
        self.assertFalse(any(w.calls for w in router.workers.values()))

    def test_over_capacity_and_too_fast_requests_are_rejected(self):
        router=VariablePeopleController(3,Worker)
        with self.assertRaises(ValueError):router.step(request(1.,4))
        router.step(request(1.,1))
        with self.assertRaises(ValueError):router.step(request(1.1,2))

    def test_zero_people_goal_reached_stays_stopped(self):
        router=VariablePeopleController(3,Worker)
        r=request(1.,0);r['robot']['gx']=0.
        response=router.step(r)
        self.assertEqual(response['output_source'],'goal_reached')
        self.assertEqual(response['action'],{'v':0.,'r':0.})

    def test_zero_people_nonfinite_robot_is_rejected(self):
        router=VariablePeopleController(3,Worker)
        r=request(1.,0);r['robot']['px']=float('nan')
        with self.assertRaises(ValueError):router.step(r)

    def test_zero_to_people_resets_the_native_history(self):
        router=VariablePeopleController(3,Worker)
        router.step(request(1.,2));n=router.workers[2].resets
        router.step(request(1.25,0));router.step(request(1.5,2))
        self.assertGreater(router.workers[2].resets,n)

    def test_failed_preload_closes_already_started_workers(self):
        workers=[]
        def factory(n):
            if n==2:raise RuntimeError('failed startup')
            w=Worker(n);workers.append(w);return w
        with self.assertRaises(RuntimeError):VariablePeopleController(3,factory)
        self.assertTrue(workers[0].closed)

    def test_variable_observation_accepts_0_1_2_people_but_not_late_wrong_ids(self):
        tracker=PersonSlots(3,retain_missing=False)
        for stamp,points in [(1.,[[0.,0.],[3.,0.]]),(1.1,[[.1,0.]]),(1.2,[])]:
            slots=tracker.update(stamp,points)
            visible=[v for v in slots if v]
            self.assertEqual(len(visible),len(points))
            obs={'variable_humans':True,'stamp':stamp,'slots':slots,'capacity_exceeded':False}
            self.assertTrue(observation_compatible(obs,[v['id'] for v in visible]))
            self.assertFalse(observation_compatible(obs,[999]))

    def test_warmup_uses_actual_observed_people_for_local_plan(self):
        tree=ast.parse((ROOT/'controller/mp_controller.py').read_text())
        cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='MPController')
        step=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='step')
        env={'time':time,'math':math};exec(compile(ast.fix_missing_locations(ast.Module(body=[step],type_ignores=[])),'warmup','exec'),env)
        robot=S(px=0.,py=0.,theta=0.,lvel=0.,omega=0.,gx=3.,gy=0.)
        humans=[S(px=2.,py=2.,vx=0.,vy=0.,radius=.2)]
        calls=[]
        def plan(r,h,w):
            calls.append(h);return {'available':True,'action':[.1,0.]}
        controller=S(parse=lambda r:(S(self_state=robot,human_states=humans),[1]),last_stamp=None,last_ids=None,
            env=S(),seq=0,actual=S(prev_states=[[]],update_state_hists=lambda *a:None),warmup_recovery=True,
            recovery_planner=S(plan=plan),obstacles=[])
        response=env['step'](controller,request(1.,1))
        self.assertEqual(len(calls[0]),1)
        self.assertEqual(response['output_source'],'observed_people_warmup_local_plan')
        self.assertGreater(response['action']['v'],0.)

if __name__=='__main__':unittest.main()
