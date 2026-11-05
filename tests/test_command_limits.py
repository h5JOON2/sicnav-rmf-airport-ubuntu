import importlib.util
import math
from pathlib import Path
import sys
import unittest

spec = importlib.util.spec_from_file_location('command_limits_tested', Path(__file__).resolve().parents[1] / 'sicnav/sicnav_diffusion/policy/command_limits.py')
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


class CommandLimitTests(unittest.TestCase):
    def setUp(self):
        self.limits = module.CommandLimiter(.95, .95, math.pi/3, .5, 1.5)

    def test_speed_saturates_in_both_directions(self):
        for sign in [-1, 1]:
            v, _, changed = self.limits.limit(sign*2., 0., sign*.95, 0., .1)
            self.assertAlmostEqual(v, sign*.95)
            self.assertTrue(changed)

    def test_acceleration_from_rest_and_while_moving(self):
        for sign in [-1, 1]:
            v, _, _ = self.limits.limit(sign*2., 0., 0., 0., .1)
            self.assertAlmostEqual(v, sign*.05)
            v, _, _ = self.limits.limit(sign*2., 0., sign*.3, 0., .1)
            self.assertAlmostEqual(v, sign*.35)

    def test_deceleration_and_direction_change_brake_first(self):
        for sign in [-1, 1]:
            for desired in [0., -sign*.7]:
                v, _, _ = self.limits.limit(desired, 0., sign*.8, 0., .1)
                self.assertAlmostEqual(v, sign*.65)
        v, _, _ = self.limits.limit(-.5, 0., .05, 0., .1)
        self.assertAlmostEqual(v, 0.)

    def test_angular_speed_and_acceleration(self):
        _, omega, _ = self.limits.limit(0., 3., 0., 0., .1)
        self.assertAlmostEqual(omega, .3)
        _, omega, _ = self.limits.limit(0., 3., 0., 1., .1)
        self.assertAlmostEqual(omega, math.pi/3)

    def test_feasible_command_passes_without_change(self):
        v, omega, changed = self.limits.limit(.32, .15, .3, .1, .1)
        self.assertEqual((v, omega, changed), (.32, .15, False))

    def test_nonfinite_input_is_rejected(self):
        for value in [math.nan, math.inf, -math.inf]:
            with self.assertRaises(ValueError):
                self.limits.limit(value, 0., 0., 0., .1)
        with self.assertRaises(ValueError):
            self.limits.limit(0., 0., 0., 0., -.1)
