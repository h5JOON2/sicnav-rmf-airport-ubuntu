"""Exercise actual gateway methods without ROS and actual Mac command wakeups."""
import ast
import json
import math
import socket
import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as S

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from remote_runtime.command_guard import CommandGuard
from remote_runtime.wire import send_packet, receive_packet


class Twist:
    def __init__(self):
        self.linear = S(x=0.)
        self.angular = S(z=0.)


class DeliveryTests(unittest.TestCase):
    def setUp(self):
        tree = ast.parse((ROOT/'scripts/robot_offload_gateway.py').read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        self.env = {'Node': object, 'time': time, 'math': math, 'json': json,
                    'threading': threading, 'Twist': Twist, 'String': S,
                    'rclpy': S(ok=lambda: True), 'socket': socket}
        exec(compile(ast.Module(body=[cls], type_ignores=[]), 'gateway', 'exec'), self.env)
        self.g = self.env['Gateway'].__new__(self.env['Gateway'])
        self.g.lock = threading.RLock()
        self.g.stop = threading.Event()
        self.g.state_changed = threading.Condition(self.g.lock)
        self.g.state_version = 1
        self.g.state_seq = 0
        self.g.state = {'session':'test', 'epoch':0, 'stamp':10., 'robot':{}}
        self.g.odom_wall = time.monotonic()
        self.g.guard = CommandGuard('test', max_age=1.)
        self.g.guard.note_state(1, 0, 10., self.g.odom_wall)
        self.g.last_packet = None
        self.g.args = S(command_max_age=1., drive=False)
        self.sim = 10.1
        self.g.get_clock = lambda: S(now=lambda: S(nanoseconds=self.sim*1e9))
        self.published = []
        self.g.cmd = S(publish=lambda m: self.published.append((m.linear.x, m.angular.z)))
        self.g.status = S(publish=lambda m: None)
        self.events = []
        self.g.record = lambda event, **kw: self.events.append((event, kw))
        self.packet = {'session':'test', 'epoch':0, 'state_seq':1,
                       'action_timestamp':10., 'action':{'v':.2, 'r':.1}}

    def receive(self, packets):
        packets = iter(packets)
        def get(_):
            try:
                return next(packets), b''
            except StopIteration:
                self.g.stop.set()
                return {'kind':'heartbeat'}, b''
        self.env['receive_packet'] = get
        self.g.commands(None)

    def test_valid_command_published_without_watchdog_tick(self):
        self.receive([self.packet])
        self.assertEqual(self.published, [(.2, .1)])
        self.assertEqual(self.events[-1][1]['trigger'], 'received')

    def test_rejected_command_does_not_publish(self):
        self.receive([{**self.packet, 'action':{'v':.5, 'r':0.}}])
        self.assertEqual(self.published, [])
        self.assertEqual(self.g.guard.last_rejection, 'velocity_limit')

    def test_halt_publishes_zero_immediately(self):
        self.receive([self.packet, {'kind':'halt', 'reason':'state_disconnected'}])
        self.assertEqual(self.published, [(.2, .1), (0., 0.)])

    def test_watchdog_stops_expired_or_stale_commands(self):
        self.g.guard.accept(self.packet, time.monotonic())
        self.g.last_packet = self.packet
        self.g.watchdog()
        self.assertEqual(self.published[-1], (.2, .1))
        self.sim = 11.1
        self.g.watchdog()
        self.assertEqual(self.published[-1], (0., 0.))
        self.sim = 10.1
        self.g.odom_wall = time.monotonic()-1.
        self.g.watchdog()
        self.assertEqual(self.published[-1], (0., 0.))

    def test_watchdog_snapshot_cannot_overwrite_a_new_command(self):
        old = {**self.packet, 'action':{'v':.1, 'r':0.}}
        self.g.guard.accept(old,time.monotonic())
        entered = threading.Event();release = threading.Event()
        def publish(msg):
            if msg.linear.x == .1:
                entered.set()
                if not release.wait(1.):raise RuntimeError('Test publisher not released')
            self.published.append((msg.linear.x,msg.angular.z))
        self.g.cmd = S(publish=publish)
        watchdog = threading.Thread(target=self.g.watchdog)
        receiver = threading.Thread(target=self.receive,args=([self.packet],))
        watchdog.start()
        try:
            self.assertTrue(entered.wait(1.))
            receiver.start()
        finally:
            release.set()
            watchdog.join(1.)
            if receiver.ident is not None:receiver.join(1.)
        self.assertFalse(watchdog.is_alive())
        self.assertFalse(receiver.is_alive())
        self.assertEqual(self.published,[(.1,0.),(.2,.1)])

    def test_pending_state_is_sent_without_a_timer_or_duplicate(self):
        sent = threading.Event()
        packets = []
        def send(_, packet):
            packets.append(packet)
            sent.set()
        self.env['send_packet'] = send
        worker = threading.Thread(target=self.g.states, args=(None,))
        worker.start()
        try:
            self.assertTrue(sent.wait(1.))
            with self.g.state_changed:
                # Notification with no new state must not fabricate another state.
                self.g.state_changed.notify_all()
            self.assertEqual(packets[0]['stamp'], 10.)
        finally:
            self.g.stop.set()
            with self.g.state_changed:self.g.state_changed.notify_all()
            worker.join(1.)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(packets), 1)


class MacWakeupTests(unittest.TestCase):
    def test_observation_ahead_of_robot_waits_without_erasing_ready_command(self):
        tree = ast.parse((ROOT/'scripts/mac_offload_server.py').read_text())
        main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name=='main')
        planner = next(n for n in main.body if isinstance(n, ast.FunctionDef) and n.name=='planner')
        lock = threading.RLock();stop = threading.Event();processed = threading.Event()
        class ObservedCondition(threading.Condition):
            calls = 0
            def wait_for(self, predicate, timeout=None):
                self.calls += 1
                if self.calls >= 2:processed.set()
                return super().wait_for(predicate, timeout)
        condition = ObservedCondition(lock)
        ready = {'action':{'v':.2,'r':0.}}
        shared = {'input_version':0,'command':ready,
                  'state':({'stamp':10.},time.monotonic()),
                  'observation':({'stamp':10.02},time.monotonic())}
        env = {'time':time,'stop':stop,'lock':lock,'shared':shared,
               'inputs_changed':condition,
               'set_command':lambda command:shared.update(command=command)}
        exec(compile(ast.Module(body=[planner],type_ignores=[]),'planner','exec'),env)
        worker = threading.Thread(target=env['planner'])
        worker.start()
        try:
            self.assertTrue(processed.wait(1.))
            self.assertIs(shared['command'],ready)
        finally:
            stop.set()
            with condition:condition.notify_all()
            worker.join(1.)
        self.assertFalse(worker.is_alive())

    def test_ready_command_wakes_sender_and_shutdown_unblocks_it(self):
        tree = ast.parse((ROOT/'scripts/mac_offload_server.py').read_text())
        main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name=='main')
        functions = [n for n in main.body if isinstance(n, ast.FunctionDef)
                     and n.name in ('set_command', 'command_channel')]
        lock = threading.RLock()
        stop = threading.Event()
        condition = threading.Condition(lock)
        env = {'threading':threading, 'time':time, 'stop':stop,
               'commands_changed':condition,
               'shared':{'command':None, 'command_version':0},
               'send_packet':send_packet, 'record':lambda *a, **k: None}
        exec(compile(ast.Module(body=functions, type_ignores=[]), 'sender', 'exec'), env)
        a, b = socket.socketpair()
        b.settimeout(1.)
        worker = threading.Thread(target=env['command_channel'], args=(a,))
        worker.start()
        try:
            self.assertEqual(receive_packet(b)[0], {'kind':'heartbeat'})
            env['set_command']({'kind':'halt', 'reason':'test'})
            self.assertEqual(receive_packet(b)[0], {'kind':'halt', 'reason':'test'})
        finally:
            stop.set()
            with condition:condition.notify_all()
            worker.join(1.)
            a.close();b.close()
        self.assertFalse(worker.is_alive())


if __name__=='__main__':
    unittest.main()
