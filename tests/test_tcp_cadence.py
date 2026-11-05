"""Test the shared controller without loading ROS, neural weights or Acados."""
import ast
import contextlib
import math
from pathlib import Path
import time
import types
import unittest
import importlib.util

limit_spec = importlib.util.spec_from_file_location("limits", Path(__file__).resolve().parents[1] / "sicnav/sicnav_diffusion/policy/command_limits.py")
limit_module = importlib.util.module_from_spec(limit_spec)
import sys
sys.modules[limit_spec.name] = limit_module
limit_spec.loader.exec_module(limit_module)


def controller_class():
    path = Path(__file__).resolve().parents[1] / 'sicnav/run_tcp_server.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SICNavController')
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in ('step', 'start_episode', 'close')]
    namespace = dict(time=time, math=math, torch=types.SimpleNamespace(no_grad=contextlib.nullcontext))
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace['SICNavController']


class CadenceTest(unittest.TestCase):
    def make_controller(self, interval):
        controller = controller_class()()
        observations, controls = [], []
        controller.env = types.SimpleNamespace(global_time=0.)
        worker = types.SimpleNamespace(update_and_trigger=lambda r,h,t: observations.append(t), stop=lambda: None)
        controller.policy = types.SimpleNamespace(async_worker=worker, reset_scenario_values=lambda: None,
            predict=lambda state: (controls.append(controller.env.global_time) or types.SimpleNamespace(v=.2, r=.05)))
        controller.command_limiter = limit_module.CommandLimiter(.95, .95, math.pi/3, .5, 1.5)
        controller.time_step = .25
        controller.control_interval = interval
        controller.parse_input_payload = lambda req: types.SimpleNamespace(self_state=None, human_states=[])
        return controller, observations, controls

    def test_observations_continue_between_controls_and_clock_reset(self):
        controller, observations, controls = self.make_controller(1.)
        stamps = [20000., 20000.1, 20000.5, 20001., 20001.1, 20002.2, 0.]
        responses = [controller.step({'timestamp': t}) for t in stamps]
        self.assertEqual(observations, stamps)
        self.assertEqual(controls, [20000., 20001., 20002.2, 0.])
        self.assertEqual([r['control_updated'] for r in responses], [True,False,False,True,False,True,True])
        self.assertEqual(responses[2]['action_timestamp'], 20000.)
        self.assertAlmostEqual(responses[0]['action']['r'], .2)

    def test_ten_hz_direct_calls_without_socket(self):
        controller, observations, controls = self.make_controller(.1)
        for i in range(11):
            response = controller.step({'timestamp': i * .1})
            self.assertTrue(response['control_updated'])
            repeated = controller.step({'timestamp': i * .1})
            self.assertFalse(repeated['control_updated'])
        self.assertEqual(len(controls), 11)
        self.assertEqual(len(observations), 22)
        self.assertAlmostEqual(controller.action_timestamp, 1.)
        controller.close()

    def test_first_observation_does_not_shift_the_clock_grid(self):
        controller, _, controls = self.make_controller(.1)
        for stamp in [10.036, 10.1, 10.2, 10.3]:
            self.assertTrue(controller.step({'timestamp': stamp})['control_updated'])
        self.assertEqual(len(controls), 4)

    def test_fallback_and_delayed_updates_are_also_acceleration_limited(self):
        controller, _, _ = self.make_controller(.1)
        controller.policy.mpc_sol_succ = False
        first = controller.step({'timestamp': 1000.})
        self.assertAlmostEqual(first['raw_action']['v'], .2)
        self.assertAlmostEqual(first['action']['v'], .05)
        self.assertTrue(first['command_limited'])
        self.assertIsNone(controller.policy.x_prev)
        delayed = controller.step({'timestamp': 1010.})
        self.assertAlmostEqual(delayed['action']['v'], .05)
        repeated = controller.step({'timestamp': 1010.})
        self.assertEqual(repeated['action'], delayed['action'])
        self.assertFalse(repeated['control_updated'])
