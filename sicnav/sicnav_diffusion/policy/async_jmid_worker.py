"""Nonblocking, latest-only JMID inference on the observation time axis."""
import copy
import logging
import threading
import time
import math
import numpy as np
import torch


class AsyncJMIDWorker:
    def __init__(self, forecaster, target_hz=2.0):
        if target_hz <= 0:
            raise ValueError('JMID frequency must be positive')
        self.forecaster = forecaster
        self.target_hz = target_hz
        self.target_interval = 1.0 / target_hz
        self.lock = threading.Condition()
        self.running = True
        self.generation = 0
        self.pending_snapshot = None
        self.result = None
        self.next_due = None
        self.last_timestamp = None
        self.total_jmid_runs = 0
        self.latest_inference_ms = None
        self.last_used_timestamp = None
        self.thread = threading.Thread(target=self._worker_loop, daemon=True,
                                       name='AsyncJMIDWorkerThread')
        self.thread.start()

    def update_and_trigger(self, robot_state, human_states, time_stamp):
        with self.lock:
            if self.last_timestamp is not None and time_stamp < self.last_timestamp:
                self.reset_cache()
            if time_stamp == self.last_timestamp:
                return
            self.last_timestamp = time_stamp
            self.forecaster.update_state_hists(robot_state, human_states, time_stamp)
            if self.next_due is None or time_stamp + 1e-9 >= self.next_due:
                with self.forecaster.prev_states_lock:
                    snapshot = (copy.deepcopy(self.forecaster.prev_states),
                                copy.deepcopy(self.forecaster.prev_robot_states))
                # Replace queued work instead of building a stale inference backlog.
                self.pending_snapshot = (snapshot, time_stamp, self.generation)
                self.next_due = (math.floor((time_stamp + 1e-9) / self.target_interval) + 1) * self.target_interval
                self.lock.notify()

    def _worker_loop(self):
        while True:
            with self.lock:
                self.lock.wait_for(lambda: not self.running or self.pending_snapshot is not None)
                if not self.running:
                    return
                snapshot, stamp, generation = self.pending_snapshot
                self.pending_snapshot = None
            try:
                started = time.monotonic()
                with torch.no_grad():
                    forecasts, weights = self.forecaster.predict_ret_best(
                        prev_states=snapshot[0], prev_robot_states=snapshot[1])
                with self.lock:
                    if generation == self.generation:
                        self.result = (np.array(forecasts, copy=True),
                                       np.array(weights, copy=True), stamp)
                        self.total_jmid_runs += 1
                        self.latest_inference_ms = (time.monotonic() - started) * 1000.0
            except Exception:
                logging.exception('[AsyncJMIDWorker] Background prediction failed')

    def get_forecasts(self, time_stamp=None, time_step=None, horizon=None):
        """Read an atomic result; interpolate its t=0 onto the current MPC grid."""
        with self.lock:
            result = self.result
            self.last_used_timestamp = None if result is None else result[2]
        if result is None:
            return None, None
        forecasts, weights, stamp = result
        if time_stamp is None:
            return forecasts, weights
        dt = self.forecaster.time_step
        query = max(0.0, time_stamp - stamp) + np.arange(horizon + 2) * time_step
        source = np.arange(forecasts.shape[2]) * dt
        aligned = np.empty((*forecasts.shape[:2], len(query), 2))
        for h in range(forecasts.shape[0]):
            for s in range(forecasts.shape[1]):
                for d in range(2):
                    values = forecasts[h, s, :, d]
                    aligned[h, s, :, d] = np.interp(query, source, values)
                    # Beyond the learned horizon, continue the terminal velocity.
                    if len(source) > 1:
                        aligned[h, s, :, d] += np.maximum(query - source[-1], 0) * (values[-1] - values[-2]) / dt
        return aligned, weights

    def get_status(self, time_stamp):
        with self.lock:
            stamp = self.last_used_timestamp
            return {'jmid_runs': self.total_jmid_runs,
                    'jmid_source_timestamp': stamp,
                    'jmid_latest_timestamp': None if self.result is None else self.result[2],
                    'jmid_age_s': None if stamp is None else max(0.0, time_stamp - stamp),
                    'jmid_inference_ms': self.latest_inference_ms}

    def reset_cache(self):
        with self.lock:
            self.generation += 1  # Discard in-flight results from the old episode.
            self.result = None
            self.latest_inference_ms = None
            self.last_used_timestamp = None
            self.pending_snapshot = None
            self.next_due = None
            self.last_timestamp = None
            with self.forecaster.prev_states_lock:
                self.forecaster.prev_states = [[] for _ in self.forecaster.prev_states]
                self.forecaster.prev_robot_states = []

    def stop(self):
        with self.lock:
            self.running = False
            self.lock.notify_all()
        self.thread.join(timeout=2.0)
