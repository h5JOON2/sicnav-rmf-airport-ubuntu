"""A single background inference; late outputs never enter a later control step."""
from concurrent.futures import ThreadPoolExecutor, TimeoutError
import math
import time


class DeadlineForecast:
    def __init__(self, deadline_ms=150.):
        if not math.isfinite(deadline_ms) or deadline_ms <= 0:
            raise ValueError('positive finite JMID deadline required')
        self.deadline_s = deadline_ms / 1000.
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='jmid')
        self.future = None
        self.metadata = None

    def step(self, prepare):
        """prepare runs only when idle; it returns (immutable-input job, metadata)."""
        start = time.perf_counter()
        discarded = None
        if self.future is not None:
            if not self.future.done():
                return dict(status='inflight', result=None, metadata=self.metadata,
                            wait_ms=0., discarded=None)
            try:
                value = self.future.result()
                discarded = dict(metadata=self.metadata, result=value)
            finally:
                self.future = None
                self.metadata = None
        job, metadata = prepare()
        self.metadata = metadata
        deadline = start + self.deadline_s

        def execute():
            value = job()
            return dict(value=value, finished=time.perf_counter())

        self.future = self.executor.submit(execute)
        try:
            result = self.future.result(timeout=max(0., deadline-time.perf_counter()))
        except TimeoutError:
            # A worker-raised TimeoutError is an inference error, not our deadline.
            if self.future.done() and self.future.exception() is not None:
                failed=self.future
                self.future=None
                self.metadata=None
                failed.result()
            return dict(status='deadline', result=None, metadata=metadata,
                        wait_ms=(time.perf_counter()-start)*1000, discarded=discarded)
        except Exception:
            self.future = None
            self.metadata = None
            raise
        self.future = None
        self.metadata = None
        timely = result['finished'] <= deadline
        return dict(status='ready' if timely else 'deadline',
                    result=result['value'] if timely else None, metadata=metadata,
                    wait_ms=(time.perf_counter()-start)*1000,
                    discarded=discarded if timely else dict(metadata=metadata,result=result))

    def close(self):
        self.executor.shutdown(wait=True, cancel_futures=True)
