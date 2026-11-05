#!/usr/bin/env python3
"""
Airport Crowd Controller (Menge ORCA-based).
Drives dynamic pedestrians (human_0, human_1, human_2) in Gazebo Harmonic
using Optimal Reciprocal Collision Avoidance (ORCA) around airport concourses and gates.
"""

import os
import sys
import math
import time

if '/opt/homebrew/lib/python3.13/site-packages' not in sys.path:
    sys.path.insert(0, '/opt/homebrew/lib/python3.13/site-packages')

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

try:
    import google.protobuf.runtime_version as rv
    rv.ValidateProtobufRuntimeVersion = lambda *a, **kw: None
except (ImportError, AttributeError):
    pass

import gz.transport13 as gz_transport
from gz.msgs10.twist_pb2 import Twist as GzTwist
from gz.msgs10.pose_v_pb2 import Pose_V as GzPose_V
from orca2d import compute_orca_velocity


class AirportCrowdController:
    def __init__(self):
        self.node = gz_transport.Node()

        SAFETY_MARGIN = 0.15
        self.pedestrians = {
            'human_0': {
                'waypoints': [
                    (144.8, -33.5),
                    (138.0, -33.5),
                    (138.0, -44.0),
                    (144.8, -44.0),
                ],
                'wp_idx': 0,
                'speed': 0.80,
                'radius': 0.35 + SAFETY_MARGIN,
                'pos': [144.8, -44.0],
                'vel': [0.0, 0.0],
                'topic': '/model/human_0/cmd_vel',
            },
            'human_1': {
                'waypoints': [
                    (138.0, -44.0),
                    (131.5, -44.0),
                    (131.5, -34.0),
                    (138.0, -34.0),
                ],
                'wp_idx': 0,
                'speed': 0.75,
                'radius': 0.35 + SAFETY_MARGIN,
                'pos': [138.0, -34.0],
                'vel': [0.0, 0.0],
                'topic': '/model/human_1/cmd_vel',
            },
            'human_2': {
                'waypoints': [
                    (131.5, -34.0),
                    (144.8, -34.0),
                    (144.8, -43.0),
                    (131.5, -43.0),
                ],
                'wp_idx': 0,
                'speed': 0.70,
                'radius': 0.35 + SAFETY_MARGIN,
                'pos': [131.5, -43.0],
                'vel': [0.0, 0.0],
                'topic': '/model/human_2/cmd_vel',
            },
        }

        # Static obstacles: airport waiting chairs & static standing passengers
        self.static_obstacles = [
            {'pos': (133.0, -44.0), 'radius': 0.55},
            {'pos': (134.5, -44.0), 'radius': 0.55},
            {'pos': (133.0, -46.0), 'radius': 0.55},
            {'pos': (134.5, -46.0), 'radius': 0.55},
            {'pos': (144.6, -32.0), 'radius': 0.50},  # passenger_standing_2
        ]

        # Robot tracking (turtlebot4)
        self.robot_pos = [136.5, -41.7]
        self.robot_vel = [0.0, 0.0]
        self.robot_radius = 0.35 + SAFETY_MARGIN

        # Publishers
        self.pubs = {}
        for name, data in self.pedestrians.items():
            self.pubs[name] = self.node.advertise(data['topic'], GzTwist)

        # Pose subscriber
        self.node.subscribe(GzPose_V, '/world/airport_cropped_world/pose/info', self.on_pose_info)
        self.last_log_time = 0.0

    def on_pose_info(self, msg):
        for pose in msg.pose:
            name = pose.name
            if name in self.pedestrians:
                self.pedestrians[name]['pos'] = [pose.position.x, pose.position.y]
            elif name == 'turtlebot4':
                prev_x, prev_y = self.robot_pos
                dt = 0.05
                self.robot_vel = [(pose.position.x - prev_x) / dt, (pose.position.y - prev_y) / dt]
                self.robot_pos = [pose.position.x, pose.position.y]

    def step(self):
        # Update goal waypoints
        for name, ped in self.pedestrians.items():
            tx, ty = ped['waypoints'][ped['wp_idx']]
            dist = math.hypot(tx - ped['pos'][0], ty - ped['pos'][1])
            if dist < 1.0:
                ped['wp_idx'] = (ped['wp_idx'] + 1) % len(ped['waypoints'])

        # Compute ORCA velocities
        for name, ped in self.pedestrians.items():
            tx, ty = ped['waypoints'][ped['wp_idx']]
            dx, dy = tx - ped['pos'][0], ty - ped['pos'][1]
            dist = math.hypot(dx, dy)
            if dist > 1e-4:
                pref_vel = (dx / dist * ped['speed'], dy / dist * ped['speed'])
            else:
                pref_vel = (0.0, 0.0)

            # Build neighbors list (filtered by neighborhood distance)
            NEIGHBOR_DIST = 5.0
            neighbors = []
            for other_name, other_ped in self.pedestrians.items():
                if other_name == name:
                    continue
                d = math.hypot(other_ped['pos'][0] - ped['pos'][0], other_ped['pos'][1] - ped['pos'][1])
                if d < NEIGHBOR_DIST:
                    neighbors.append((
                        other_ped['pos'],
                        other_ped['vel'],
                        other_ped['radius'],
                        0.5
                    ))

            # Add robot
            d_rob = math.hypot(self.robot_pos[0] - ped['pos'][0], self.robot_pos[1] - ped['pos'][1])
            if d_rob < NEIGHBOR_DIST:
                neighbors.append((
                    self.robot_pos,
                    self.robot_vel,
                    self.robot_radius,
                    0.0
                ))

            # Add static obstacles
            for obs in self.static_obstacles:
                d_obs = math.hypot(obs['pos'][0] - ped['pos'][0], obs['pos'][1] - ped['pos'][1])
                if d_obs < NEIGHBOR_DIST:
                    neighbors.append((
                        obs['pos'],
                        (0.0, 0.0),
                        obs['radius'],
                        0.0
                    ))

            # Compute collision-free velocity via ORCA
            new_vx, new_vy = compute_orca_velocity(
                agent_pos=ped['pos'],
                agent_vel=ped['vel'],
                agent_pref_vel=pref_vel,
                agent_radius=ped['radius'],
                agent_max_speed=ped['speed'],
                neighbors=neighbors,
                time_horizon=2.5,
                dt=0.05
            )

            ped['vel'] = [new_vx, new_vy]

            # Publish command
            cmd = GzTwist()
            cmd.linear.x = new_vx
            cmd.linear.y = new_vy
            cmd.linear.z = 0.0
            cmd.angular.z = 0.0
            self.pubs[name].publish(cmd)

        now = time.time()
        if now - self.last_log_time > 1.5:
            self.last_log_time = now
            h0 = self.pedestrians['human_0']
            h1 = self.pedestrians['human_1']
            h2 = self.pedestrians['human_2']
            print(
                f"[ORCA Crowd] "
                f"h0: ({h0['pos'][0]:.2f}, {h0['pos'][1]:.2f}) WP {h0['wp_idx']} | "
                f"h1: ({h1['pos'][0]:.2f}, {h1['pos'][1]:.2f}) WP {h1['wp_idx']} | "
                f"h2: ({h2['pos'][0]:.2f}, {h2['pos'][1]:.2f}) WP {h2['wp_idx']}",
                flush=True
            )

    def run(self):
        print("[AirportCrowdController] ORCA Crowd Controller initializing...", flush=True)
        time.sleep(1.0)
        print("[AirportCrowdController] ORCA Crowd Controller running at 20Hz...", flush=True)
        rate = 0.05
        while True:
            self.step()
            time.sleep(rate)


if __name__ == '__main__':
    controller = AirportCrowdController()
    controller.run()
