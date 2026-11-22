"""Command validity uses the robot's own sent-state clock, never Mac wall time."""
import math
import threading
from collections import OrderedDict


class CommandGuard:
    def __init__(self, session, max_age=1.5, max_v=.31, max_omega=1.9):
        self.session = session
        self.max_age, self.max_v, self.max_omega = max_age, max_v, max_omega
        self.lock = threading.RLock()
        self.epoch = 0
        self.sent = OrderedDict()
        self.current = None
        self.last_sequence = -1
        self.last_rejection = None

    def reset(self, epoch):
        with self.lock:
            self.epoch = epoch
            self.sent.clear()
            self.current = None
            self.last_sequence = -1

    def note_state(self, sequence, epoch, stamp, wall):
        with self.lock:
            if epoch != self.epoch:
                self.reset(epoch)
            self.sent[sequence] = (stamp, wall)
            while len(self.sent) > 200:
                self.sent.popitem(last=False)

    def invalidate(self):
        with self.lock:
            self.current = None

    def accept(self, packet, wall):
        with self.lock:
            self.last_rejection = None
            if packet.get('session') != self.session or packet.get('epoch') != self.epoch:
                self.last_rejection = 'session_or_epoch'
                return False
            sequence = packet.get('state_seq')
            if type(sequence) is not int or sequence not in self.sent or sequence < self.last_sequence:
                self.last_rejection = 'unknown_or_old_state_sequence'
                return False
            stamp, sent_wall = self.sent[sequence]
            try:
                action_stamp = float(packet['action_timestamp'])
                v, omega = float(packet['action']['v']), float(packet['action']['r'])
            except (KeyError, TypeError, ValueError):
                self.last_rejection = 'malformed_action'
                return False
            if not all(math.isfinite(vv) for vv in [action_stamp,v,omega]):
                self.last_rejection = 'nonfinite_action'
                return False
            if not 0 <= wall-sent_wall <= self.max_age:
                self.last_rejection = 'wall_age'
                return False
            if not 0 <= stamp-action_stamp <= self.max_age:
                self.last_rejection = 'action_sim_age'
                return False
            if abs(v)>self.max_v or abs(omega)>self.max_omega:
                self.last_rejection = 'velocity_limit'
                return False
            self.last_sequence = sequence
            # Duplicate heartbeats cannot renew a command's deadline.
            self.current = (v, omega, sent_wall+self.max_age, action_stamp)
            return True

    def command(self, wall, sim_stamp):
        with self.lock:
            if self.current is None:
                return (0.,0.)
            v, omega, deadline, stamp = self.current
            if wall > deadline or not 0 <= sim_stamp-stamp <= self.max_age:
                return (0.,0.)
            return v, omega
