import time
import copy
import threading
import logging
import numpy as np
import torch


class AsyncJMIDWorker:
    """
    Asynchronous Background Worker for JMID Diffusion Trajectory Forecasting.
    Decouples the slower JMID neural network (~180ms) from the high-rate 10 Hz Acados MPC loop (~20ms),
    ensuring every control action is output within 0.1s (100ms) without lag.
    Runs JMID forecasting asynchronously in the background at target rate (default 2.0 Hz = every 0.5s).
    """

    def __init__(self, forecaster, target_hz=2.0):
        self.forecaster = forecaster
        self.target_interval = 1.0 / max(target_hz, 0.1)  # e.g. 0.5s for 2.0 Hz
        self.target_hz = target_hz
        self.lock = threading.Lock()

        # Cached predictions
        self.latest_top_k_forecasts = None
        self.latest_top_k_weights = None
        self.last_predict_wall_time = 0.0
        self.total_jmid_runs = 0

        # Worker control
        self.pending_snapshot = None
        self.is_busy = False
        self.running = True

        self.thread = threading.Thread(
            target=self._worker_loop,
            name="AsyncJMIDWorkerThread",
            daemon=True
        )
        self.thread.start()
        logging.info(f"[AsyncJMIDWorker] Started background worker at {self.target_hz:.1f} Hz (interval: {self.target_interval:.2f}s)")

    def update_and_trigger(self, robot_state, human_states, time_stamp):
        """
        Called on every 10 Hz step. Fast update (~0.1ms).
        Triggers background inference if target interval has passed and worker is idle.
        """
        self.forecaster.update_state_hists(robot_state, human_states, time_stamp)

        now = time.time()
        if (now - self.last_predict_wall_time >= self.target_interval) and not self.is_busy:
            with self.forecaster.prev_states_lock:
                snapshot = (
                    copy.deepcopy(self.forecaster.prev_states),
                    copy.deepcopy(self.forecaster.prev_robot_states)
                )
            with self.lock:
                if not self.is_busy:
                    self.pending_snapshot = snapshot
                    self.is_busy = True

    def _worker_loop(self):
        while self.running:
            snapshot = None
            with self.lock:
                if self.pending_snapshot is not None:
                    snapshot = self.pending_snapshot
                    self.pending_snapshot = None

            if snapshot is not None:
                prev_states, prev_robot_states = snapshot
                t0 = time.time()
                try:
                    with torch.no_grad():
                        forecasts, weights = self.forecaster.predict_ret_best(
                            prev_states=prev_states,
                            prev_robot_states=prev_robot_states
                        )
                    dur_ms = (time.time() - t0) * 1000.0
                    with self.lock:
                        self.latest_top_k_forecasts = forecasts
                        self.latest_top_k_weights = weights
                        self.last_predict_wall_time = time.time()
                        self.total_jmid_runs += 1
                    # logging.debug(f"[AsyncJMIDWorker] Background JMID #{self.total_jmid_runs} finished in {dur_ms:.1f} ms")
                except Exception as e:
                    logging.error(f"[AsyncJMIDWorker] Error in background prediction: {e}")
                finally:
                    with self.lock:
                        self.is_busy = False
            else:
                time.sleep(0.005)

    def get_forecasts(self):
        """
        Returns (latest_top_k_forecasts, latest_top_k_weights).
        If no forecast is cached yet (initial step 0), runs synchronous warm-up once.
        """
        with self.lock:
            if self.latest_top_k_forecasts is not None:
                return self.latest_top_k_forecasts, self.latest_top_k_weights

        # Step 0 initial synchronous warm-up
        logging.info("[AsyncJMIDWorker] Running initial synchronous JMID warm-up...")
        t0 = time.time()
        with torch.no_grad():
            forecasts, weights = self.forecaster.predict_ret_best()
        dur_ms = (time.time() - t0) * 1000.0
        with self.lock:
            self.latest_top_k_forecasts = forecasts
            self.latest_top_k_weights = weights
            self.last_predict_wall_time = time.time()
            self.total_jmid_runs += 1
        logging.info(f"[AsyncJMIDWorker] Initial warm-up completed in {dur_ms:.1f} ms")
        return forecasts, weights

    def reset_cache(self):
        with self.lock:
            self.latest_top_k_forecasts = None
            self.latest_top_k_weights = None
            self.last_predict_wall_time = 0.0
            self.pending_snapshot = None
            self.is_busy = False

    def stop(self):
        self.running = False
