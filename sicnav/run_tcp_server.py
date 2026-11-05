#!/usr/bin/env python3
"""
SICNav TCP Server:
Receives robot and human states via TCP socket, runs JMID -> ORCA -> MPC (SICNavAcados),
and sends the computed robot action (v, r) back through the socket.
"""

import os
import sys
import time
import json
import socket
import argparse
import logging
import configparser
import numpy as np
import torch
import gym
import resource
import math

def get_mem_mb():
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / (1024.0 * 1024.0)

from crowd_sim_plus.envs.crowd_sim_plus import CrowdSimPlus
from crowd_sim_plus.envs.utils.robot_plus import Robot
from crowd_sim_plus.envs.utils.state_plus import FullState, FullyObservableJointState
from sicnav_diffusion.policy.policy_factory import policy_factory
from sicnav_diffusion.policy.command_limits import CommandLimiter


def parse_args():
    parser = argparse.ArgumentParser(description="SICNav TCP Server")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="TCP server host IP")
    parser.add_argument("--port", type=int, default=9090, help="TCP server port")
    parser.add_argument("--env_config", type=str, default="sicnav_diffusion/configs/env.config")
    parser.add_argument("--policy_config", type=str, default="sicnav_diffusion/configs/policy.config")
    parser.add_argument("--num_humans", type=int, default=-1, help="Number of humans (overrides config)")
    parser.add_argument("--control_hz", type=float, default=10.0, help="ORCA + MPC output rate in simulation time")
    parser.add_argument("--device", type=str, default="cuda:0" if torch.cuda.is_available() else "cpu")
    return parser.parse_args()


class SICNavController:
    def __init__(self, args):
        if args.control_hz <= 0:
            raise ValueError("control_hz must be positive")
        self.control_interval = 1.0 / args.control_hz
        self.args = args
        self.host = args.host
        self.port = args.port
        self.device = torch.device(args.device)

        logging.info(f"Initializing SICNav on device: {self.device}")

        # 1. Load Configurations
        self.env_config = configparser.RawConfigParser()
        self.env_config.read(args.env_config)

        if args.num_humans != -1:
            self.env_config.set("sim", "human_num", str(args.num_humans))
        self.num_humans = self.env_config.getint("sim", "human_num")

        self.policy_config = configparser.RawConfigParser()
        self.policy_config.read(args.policy_config)

        self.command_limiter = CommandLimiter(
            max_forward=self.policy_config.getfloat('mpc_env', 'max_speed'),
            max_reverse=self.policy_config.getfloat('mpc_env', 'max_rev_speed'),
            max_angular=math.radians(self.policy_config.getfloat('mpc_env', 'max_rot_degrees')),
            acceleration=self.policy_config.getfloat('mpc_env', 'max_l_acc'),
            deceleration=-self.policy_config.getfloat('mpc_env', 'max_l_dcc'))

        # 2. Initialize Gym Environment and Policy
        logging.info("Setting up simulation environment and Acados solver...")
        # This environment supplies custom state/action objects rather than Gym spaces.
        self.env = CrowdSimPlus()
        self.env.configure(self.env_config)
        self.env.test_sim = self.env_config.get("sim", "test_sim")

        # Create SICNav Policy (JMID + ORCA + Acados MPC)
        self.policy = policy_factory["sicnav_acados"]()
        self.policy.configure(self.policy_config)

        robot = Robot(self.env_config, "robot")
        robot.set_policy(self.policy)
        self.env.set_robot(robot)

        self.policy.set_phase("test")
        self.policy.set_device(self.device)
        self.policy.external_clock = True
        self.policy.control_hz = args.control_hz
        self.policy.set_env(self.env)

        self.time_step = self.env.time_step
        self.default_robot_radius = self.policy.robot_radius
        self.default_human_radius = self.policy.human_radius
        self.default_v_pref = self.policy.pref_vel_rob

        logging.info("SICNav policy initialized successfully!")
        logging.info(f"[*] Memory after init: {get_mem_mb():.1f} MB")

    def parse_input_payload(self, data):
        """
        Parses incoming JSON payload to FullyObservableJointState.
        Expected JSON format:
        {
            "timestamp": float (optional),
            "robot": {
                "px": float, "py": float,
                "vx": float, "vy": float,
                "theta": float,
                "gx": float, "gy": float,
                "radius": float (optional),
                "v_pref": float (optional)
            },
            "humans": [
                {
                    "id": int,
                    "px": float, "py": float,
                    "vx": float, "vy": float,
                    "radius": float (optional)
                }, ...
            ]
        }
        """
        r = data["robot"]
        robot_state = FullState(
            px=float(r["px"]),
            py=float(r["py"]),
            vx=float(r["vx"]),
            vy=float(r["vy"]),
            radius=float(r.get("radius", self.default_robot_radius)),
            gx=float(r.get("gx", 0.0)),
            gy=float(r.get("gy", 0.0)),
            v_pref=float(r.get("v_pref", self.default_v_pref)),
            theta=float(r.get("theta", 0.0))
        )

        human_states = []
        raw_humans = data.get("humans", [])
        for h in raw_humans:
            vx = float(h.get("vx", 0.0))
            vy = float(h.get("vy", 0.0))
            theta = np.arctan2(vy, vx) if (vx != 0.0 or vy != 0.0) else 0.0
            
            human_states.append(FullState(
                px=float(h["px"]),
                py=float(h["py"]),
                vx=vx,
                vy=vy,
                radius=float(h.get("radius", self.default_human_radius)),
                gx=0.0,
                gy=0.0,
                v_pref=self.policy.human_max_vel,
                theta=theta
            ))

        # Pad with dummy humans if fewer humans are provided than expected
        while len(human_states) < self.num_humans:
            # Place dummy humans far away
            human_states.append(FullState(
                px=999.0 + len(human_states),
                py=999.0 + len(human_states),
                vx=0.0,
                vy=0.0,
                radius=self.default_human_radius,
                gx=999.0,
                gy=999.0,
                v_pref=0.0,
                theta=0.0
            ))

        # Truncate if more humans than solver supports
        if len(human_states) > self.num_humans:
            human_states = human_states[:self.num_humans]

        joint_state = FullyObservableJointState(
            self_state=robot_state,
            human_states=human_states,
            static_obs=self.policy.static_obs
        )
        return joint_state

    def start_episode(self):
        self.policy.external_clock = True
        self.policy.reset_scenario_values()
        self.next_control = None
        self.last_timestamp = None
        self.action_timestamp = None
        self.action = None
        self.output_action = None
        self.raw_action = None
        self.command_limited = False

    def step(self, request):
        """In-process observation input, shared by ROS and optional TCP transport."""
        if not hasattr(self, 'next_control'):
            self.start_episode()
        start = time.monotonic()
        state = self.parse_input_payload(request)
        ts = float(request['timestamp'])
        if not math.isfinite(ts):
            raise ValueError('timestamp must be finite simulation seconds')
        if self.last_timestamp is not None and ts < self.last_timestamp:
            self.start_episode()
        self.env.global_time = ts
        self.last_timestamp = ts
        worker = getattr(self.policy, 'async_worker', None)
        if worker is not None:
            worker.update_and_trigger(state.self_state, state.human_states, ts)
        updated = self.next_control is None or ts + 1e-9 >= self.next_control
        if updated:
            if self.control_interval > self.time_step + 1e-9:
                self.policy.x_prev = None
                self.policy.u_prev = None
            with torch.no_grad():
                self.action = self.policy.predict(state)
            raw_v = float(self.action.v)
            raw_omega = float(self.action.r / self.time_step)
            robot = request.get('robot', {})
            theta = float(robot.get('theta', 0.0))
            measured_v = (float(robot.get('vx', 0.0)) * math.cos(theta) +
                          float(robot.get('vy', 0.0)) * math.sin(theta))
            measured_omega = float(robot.get('omega', 0.0))
            # A delayed solve must not authorize a large jump in the next command.
            dt = self.control_interval if self.action_timestamp is None else min(
                self.control_interval, max(0.0, ts - self.action_timestamp))
            v, omega, self.command_limited = self.command_limiter.limit(
                raw_v, raw_omega, measured_v, measured_omega, dt)
            self.raw_action = {'v': raw_v, 'r': raw_omega}
            self.output_action = {'v': v, 'r': omega}
            self.policy.prev_lvel = v
            self.policy.prev_avel = omega
            if self.command_limited:
                # The previous MPC trajectory assumed a different applied action.
                self.policy.x_prev = None
                self.policy.u_prev = None
            self.action_timestamp = ts
            # Use the same simulation epoch as the 2Hz prediction scheduler.
            self.next_control = (math.floor((ts + 1e-9) / self.control_interval) + 1) * self.control_interval

        return {
            **(worker.get_status(ts) if worker is not None and hasattr(worker, 'get_status') else {}),
            'status': 'success',
            'action': dict(self.output_action),
            'raw_action': dict(self.raw_action),
            'command_limited': self.command_limited,
            'timestamp': ts,
            'action_timestamp': self.action_timestamp,
            'control_updated': updated,
            'compute_time_ms': round((time.monotonic() - start) * 1000., 2),
        }

    def close(self):
        worker = getattr(self.policy, 'async_worker', None)
        if worker is not None:
            worker.stop()


class SICNavServer(SICNavController):
    def run(self):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((self.host, self.port))
        server_sock.listen(1)
        logging.info(f"[*] TCP Server listening on {self.host}:{self.port}...")

        while True:
            client_sock, client_addr = server_sock.accept()
            client_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            logging.info(f"[+] Client connected from {client_addr}")

            buffer = ""
            self.env.global_time = 0.0

            self.start_episode()

            try:
                while True:
                    data = client_sock.recv(4096).decode("utf-8")
                    if not data:
                        logging.warning("[-] Client disconnected.")
                        break

                    buffer += data
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue

                        try:
                            req_json = json.loads(line)
                            resp = self.step(req_json)
                        except Exception as e:
                            logging.error(f"Error during predict: {e}", exc_info=True)
                            resp = {
                                "status": "error",
                                "error_msg": str(e),
                                "action": {"v": 0.0, "r": 0.0}
                            }

                        resp_str = json.dumps(resp) + "\n"
                        client_sock.sendall(resp_str.encode("utf-8"))

            except (ConnectionResetError, BrokenPipeError):
                logging.warning("[-] Client connection lost unexpectedly.")
            finally:
                client_sock.close()
                worker = getattr(self.policy, 'async_worker', None)
                if worker is not None:
                    worker.stop()


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )
    args = parse_args()
    server = SICNavServer(args)
    server.run()


if __name__ == "__main__":
    main()

