"""One pending synchronized frame; processing never queues an unbounded history."""
import threading
import time


class LatestFrame:
    def __init__(self):
        self.condition = threading.Condition()
        self.pending = None
        self.closed = False
        self.generation = 0
        self.last_stamp = None
        self.received = 0
        self.replaced = 0

    def put(self, stamp, payload):
        with self.condition:
            if self.closed:
                return False
            reset = self.last_stamp is not None and stamp < self.last_stamp
            if reset:
                self.generation += 1
                self.pending = None
            self.last_stamp = stamp
            self.received += 1
            if self.pending is not None:
                self.replaced += 1
            self.pending = (self.generation, payload, time.monotonic())
            self.condition.notify()
            return reset

    def take(self):
        with self.condition:
            self.condition.wait_for(lambda: self.closed or self.pending is not None)
            if self.closed:
                return None
            item, self.pending = self.pending, None
            return item

    def close(self):
        with self.condition:
            self.closed = True
            self.pending = None
            self.condition.notify_all()
