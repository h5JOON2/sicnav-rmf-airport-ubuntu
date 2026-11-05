"""Exercise native timestamp fields and common-time projection without ROS imports."""
import ast
import math
from pathlib import Path
import socket
import threading
import types
import unittest


def callbacks():
    path = Path(__file__).resolve().parents[1] / 'scripts/sicnav_gazebo_client.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    cls.bases = []
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in
                ('on_gz_pose', 'on_crowd_states', 'control_step')]
    namespace = dict(math=math, socket=socket,
                     Twist=lambda: types.SimpleNamespace(linear=types.SimpleNamespace(x=0.), angular=types.SimpleNamespace(z=0.)))
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace['SicnavGazeboClient']


class ObservationTimeTests(unittest.TestCase):
    def test_gazebo_nsec_and_crowd_source_priority(self):
        node = callbacks()()
        node.state_lock = threading.Lock()
        node.crowd_received = False
        node.pedestrians, node.ped_id_map, node.next_ped_id = {}, {}, 1
        msg = types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=2, nsec=250000000)),
            pose=[types.SimpleNamespace(name='agent1', position=types.SimpleNamespace(x=1., y=2.))])
        node.on_gz_pose(msg)
        self.assertEqual(node.pedestrians['agent1']['last_t'], 2.25)
        human = types.SimpleNamespace(agent_id=1, is_external=False, velocity_valid=True,
            pose=types.SimpleNamespace(position=types.SimpleNamespace(x=3., y=4.)),
            twist=types.SimpleNamespace(linear=types.SimpleNamespace(x=.4, y=.2)))
        external = types.SimpleNamespace(agent_id=0, is_external=True)
        crowd = types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=2, nanosec=500000000)),
                                      agents=[human, external])
        node.on_crowd_states(crowd)
        self.assertEqual(list(node.pedestrians), ['1'])
        self.assertEqual(node.pedestrians['1']['last_t'], 2.5)
        node.on_gz_pose(msg)
        self.assertEqual(list(node.pedestrians), ['1'])

    def test_robot_and_humans_project_to_simulation_clock(self):
        node = callbacks()()
        node.state_lock = threading.Lock()
        node.last_odom_time = 10.
        node.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=10100000000))
        node.robot_px, node.robot_py = 1., 2.
        node.robot_vx, node.robot_vy, node.robot_theta, node.robot_omega = 2., 0., 0., 1.
        node.goal_x, node.goal_y, node.step_idx = 100., 0., 0
        node.pedestrians = {'1': dict(id=1, px=5., py=6., vx=2., vy=0., radius=.25, last_t=10.05)}
        received, published = [], []
        node.controller = types.SimpleNamespace(step=lambda payload: received.append(payload) or
            dict(action=dict(v=.5, r=.1), control_updated=True))
        node.cmd_pub = types.SimpleNamespace(publish=published.append)
        node.get_logger = lambda: types.SimpleNamespace(info=lambda *a: None, error=lambda *a: None)
        node.control_step()
        self.assertAlmostEqual(received[0]['timestamp'], 10.1)
        self.assertAlmostEqual(received[0]['robot']['px'], 1.2)
        self.assertAlmostEqual(received[0]['robot']['theta'], .1)
        self.assertAlmostEqual(received[0]['humans'][0]['px'], 5.1)
        self.assertEqual(len(published), 1)
