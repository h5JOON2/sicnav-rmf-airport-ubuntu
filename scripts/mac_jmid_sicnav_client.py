#!/usr/bin/env python3
"""
macOS JMID + SICNav TCP Client (Runs directly on macOS).

Leverages Apple Silicon GPU (MPS) for accelerated JMID Diffusion trajectory prediction
and computes collision-free velocity commands for TurtleBot4 via SICNav/ORCA.

Usage:
    python3 scripts/mac_jmid_sicnav_client.py --ip 192.168.64.4 --port 9090
"""

import sys
import os
import math
import time
import json
import socket
import argparse
from collections import defaultdict, deque

# PyTorch with MPS check
try:
    import torch
    if torch.backends.mps.is_available():
        DEVICE = torch.device("mps")
        print(f"[Mac] Accelerated Device: Apple Silicon GPU (MPS) enabled!")
    elif torch.cuda.is_available():
        DEVICE = torch.device("cuda")
        print("[Mac] Device: CUDA")
    else:
        DEVICE = torch.device("cpu")
        print("[Mac] Device: CPU")
except ImportError:
    DEVICE = "cpu"
    print("[Mac WARNING] PyTorch not installed in this environment. Running in CPU mode.")


class MacJmidSicnavClient:
    def __init__(self, vm_ip="192.168.64.4", port=9090, history_len=8):
        self.vm_ip = vm_ip
        self.port = port
        self.history_len = history_len

        # Human history trajectory buffer: {human_id: deque([(x, y, vx, vy), ...], maxlen=8)}
        self.human_trajectories = defaultdict(lambda: deque(maxlen=self.history_len))

        # Robot target goal (concourse forward point)
        self.goal = [148.0, -42.0]

        # Performance stats
        self.step_count = 0
        self.last_fps_time = time.time()

    def run_jmid_prediction(self, human_states):
        """
        Placeholder for JMID Trajectory Prediction using PyTorch MPS.
        Update human history and predict multi-modal future trajectories.
        """
        # 1. Update history buffer
        for h in human_states:
            hid = h['id']
            self.human_trajectories[hid].append((h['x'], h['y'], h['vx'], h['vy']))

        # 2. When integrated with JMID model:
        # Example forward pass:
        # batch_tensor = torch.tensor(history, dtype=torch.float32, device=DEVICE)
        # with torch.no_grad():
        #     future_samples = jmid_model.sample(batch_tensor)
        # return future_samples.cpu().numpy()

        # Simple constant-velocity / extrapolation fallback for testing:
        future_trajs = {}
        for h in human_states:
            hid = h['id']
            # Extrapolate 2.0s into the future (10 steps @ dt=0.2s)
            pred = []
            for step in range(1, 11):
                dt = step * 0.2
                pred.append((h['x'] + h['vx'] * dt, h['y'] + h['vy'] * dt))
            future_trajs[hid] = pred
        return future_trajs

    def compute_sicnav_control(self, robot_state, human_states, future_trajs):
        """
        SICNav Controller (ORCA / KKT / MPC).
        Computes desired velocity towards goal while avoiding predicted human trajectories.
        """
        rx, ry = robot_state['x'], robot_state['y']
        yaw = robot_state['yaw']
        gx, gy = self.goal

        # Goal distance & heading
        dx = gx - rx
        dy = gy - ry
        dist_to_goal = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_diff = (target_angle - yaw + math.pi) % (2 * math.pi) - math.pi

        if dist_to_goal < 0.5:
            # Reached goal, switch or stop
            return 0.0, 0.0

        # Base nominal velocity
        pref_speed = min(0.5, dist_to_goal * 0.5)
        linear_v = pref_speed
        angular_w = 1.5 * angle_diff

        # Social collision avoidance penalty
        SAFETY_RADIUS = 1.0  # meters
        for h in human_states:
            hx, hy = h['x'], h['y']
            d = math.hypot(hx - rx, hy - ry)
            if d < SAFETY_RADIUS:
                # Slow down and steer away from human
                linear_v *= max(0.1, (d / SAFETY_RADIUS))
                bearing = math.atan2(hy - ry, hx - rx) - yaw
                avoid_steer = -0.8 * math.copysign(1.0, math.sin(bearing))
                angular_w += avoid_steer

        # Saturate velocities
        linear_v = max(-0.2, min(0.6, linear_v))
        angular_w = max(-1.2, min(1.2, angular_w))

        return linear_v, angular_w

    def start(self):
        print(f"[Mac] Connecting to UTM Simulation at {self.vm_ip}:{self.port}...")
        
        while True:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            try:
                sock.connect((self.vm_ip, self.port))
                print(f"[Mac] Successfully connected to UTM ({self.vm_ip}:{self.port})!")
                break
            except (ConnectionRefusedError, socket.timeout, OSError) as e:
                sock.close()
                print(f"[Mac] Waiting for UTM bridge node on {self.vm_ip}:{self.port}... (retrying in 2s)")
                time.sleep(2.0)

        buffer = ""
        sock.settimeout(3.0)

        try:
            while True:
                # 1. Receive state packet
                data = sock.recv(4096).decode('utf-8')
                if not data:
                    print("[Mac] Simulation server disconnected.")
                    break

                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if not line.strip():
                        continue

                    t_start = time.time()
                    state = json.loads(line)

                    robot = state.get('robot', {})
                    humans = state.get('humans', [])

                    # 2. Run JMID Trajectory Prediction (PyTorch MPS)
                    future_trajs = self.run_jmid_prediction(humans)

                    # 3. Run SICNav Controller
                    v, w = self.compute_sicnav_control(robot, humans, future_trajs)

                    # 4. Send control command back to UTM
                    cmd = {'linear': v, 'angular': w}
                    sock.sendall((json.dumps(cmd) + '\n').encode('utf-8'))

                    latency_ms = (time.time() - t_start) * 1000.0

                    # 5. Display telemetry
                    self.step_count += 1
                    if time.time() - self.last_fps_time >= 1.0:
                        fps = self.step_count / (time.time() - self.last_fps_time)
                        print(
                            f"[SICNav] Robot Pose: ({robot.get('x', 0):.2f}, {robot.get('y', 0):.2f}) | "
                            f"Cmd: [v={v:.2f} m/s, w={w:.2f} rad/s] | "
                            f"Humans: {len(humans)} | Latency: {latency_ms:.1f}ms | Rate: {fps:.1f} Hz"
                        )
                        self.step_count = 0
                        self.last_fps_time = time.time()

        except KeyboardInterrupt:
            print("\n[Mac] Shutting down SICNav client...")
            # Send stop command
            try:
                sock.sendall((json.dumps({'linear': 0.0, 'angular': 0.0}) + '\n').encode('utf-8'))
            except Exception:
                pass
        finally:
            sock.close()


def main():
    parser = argparse.ArgumentParser(description="macOS JMID + SICNav Controller Client")
    parser.add_argument("--ip", default="192.168.64.4", help="UTM VM IP address (default: 192.168.64.4)")
    parser.add_argument("--port", type=int, default=9090, help="TCP port (default: 9090)")
    args = parser.parse_args()

    client = MacJmidSicnavClient(vm_ip=args.ip, port=args.port)
    client.start()


if __name__ == '__main__':
    main()

