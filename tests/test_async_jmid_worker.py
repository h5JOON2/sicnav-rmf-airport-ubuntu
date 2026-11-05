import contextlib
import importlib.util
from pathlib import Path
import sys
import threading
import time
import types
import unittest
import numpy as np

# These scheduling tests do not require the neural-network runtime.
if 'torch' not in sys.modules:
    try:
        import torch
    except ImportError:
        sys.modules['torch'] = types.SimpleNamespace(no_grad=contextlib.nullcontext)
spec = importlib.util.spec_from_file_location('worker', Path(__file__).resolve().parents[1] / 'sicnav/sicnav_diffusion/policy/async_jmid_worker.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Forecaster:
    time_step = .25
    def __init__(self):
        self.prev_states_lock = threading.Lock()
        self.prev_states = [[]]
        self.prev_robot_states = []
        self.started = threading.Event()
        self.release = threading.Event()
        self.calls = []

    def update_state_hists(self, robot, humans, stamp):
        with self.prev_states_lock:
            self.prev_states[0].append([0, 0, stamp])
            self.prev_robot_states.append([0, 0, stamp])

    def predict_ret_best(self, prev_states, prev_robot_states):
        self.calls.append(prev_robot_states[-1][-1])
        self.started.set()
        self.release.wait(2)
        t = np.arange(9) * .25
        return np.stack((t, 2*t), axis=-1)[None, None], np.zeros((1, 1))


class WorkerTests(unittest.TestCase):
    def setUp(self):
        self.forecaster = Forecaster()
        self.worker = module.AsyncJMIDWorker(self.forecaster)

    def tearDown(self):
        self.forecaster.release.set()
        self.worker.stop()

    def wait_runs(self, count):
        deadline = time.monotonic() + 2
        while self.worker.total_jmid_runs < count and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertEqual(self.worker.total_jmid_runs, count)

    def test_nonblocking_latest_only_and_simulation_cadence(self):
        self.worker.update_and_trigger(None, None, 100000.)
        self.assertTrue(self.forecaster.started.wait(1))
        self.assertEqual(self.worker.get_forecasts(), (None, None))
        for stamp in [100000.1, 100000.5, 100001.0]:
            self.worker.update_and_trigger(None, None, stamp)
        self.forecaster.release.set()
        self.wait_runs(2)
        self.assertEqual(self.forecaster.calls, [100000., 100001.])
        # Duplicate timestamps and paused simulation must not schedule another run.
        self.worker.update_and_trigger(None, None, 100001.)
        self.assertIsNone(self.worker.pending_snapshot)

    def test_age_alignment_and_terminal_extrapolation(self):
        self.forecaster.release.set()
        self.worker.update_and_trigger(None, None, 10.)
        self.wait_runs(1)
        forecasts, _ = self.worker.get_forecasts(10.5, .25, 8)
        np.testing.assert_allclose(forecasts[0, 0, :, 0], .5 + np.arange(10)*.25)
        np.testing.assert_allclose(forecasts[0, 0, :, 1], 1. + np.arange(10)*.5)

    def test_reset_discards_inflight_result(self):
        self.worker.update_and_trigger(None, None, 5.)
        self.assertTrue(self.forecaster.started.wait(1))
        self.worker.reset_cache()
        self.forecaster.release.set()
        self.worker.stop()  # Join the in-flight inference before checking the cache.
        self.assertEqual(self.worker.get_forecasts(), (None, None))
        self.assertEqual(self.forecaster.prev_robot_states, [])


if __name__ == '__main__':
    unittest.main()
