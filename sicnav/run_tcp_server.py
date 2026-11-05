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

def get_mem_mb():
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / (1024.0 * 1024.0)

from crowd_sim_plus.envs.crowd_sim_plus import *
from crowd_sim_plus.envs.utils.robot_plus import Robot
from crowd_sim_plus.envs.utils.state_plus import FullState, FullyObservableJointState
from sicnav_diffusion.policy.policy_factory import policy_factory


def parse_args():
    parser = argparse.ArgumentParser(description="SICNav TCP Server")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="TCP server host IP")
    parser.add_argument("--port", type=int, default=9090, help="TCP server port")
    parser.add_argument("--env_config", type=str, default="sicnav_diffusion/configs/env.config")
    parser.add_argument("--policy_config", type=str, default="sicnav_diffusion/configs/policy.config")
    parser.add_argument("--num_humans", type=int, default=-1, help="Number of humans (overrides config)")
    parser.add_argument("--device", type=str, default="cuda:0" if torch.cuda.is_available() else "cpu")
    return parser.parse_args()


class SICNavServer:
    def __init__(self, args):
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

        # 2. Initialize Gym Environment and Policy
        logging.info("Setting up simulation environment and Acados solver...")
        env_order = gym.make("CrowdSimPlus-v0")
        self.env = env_order.unwrapped
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
            global_step = 0
            self.env.global_time = 0.0

            # Reset history on new client connection to prevent timestamp jumps
            if hasattr(self.policy, 'hum_traj_forecaster'):
                self.policy.hum_traj_forecaster.prev_states = [[] for _ in range(self.num_humans)]
                self.policy.hum_traj_forecaster.prev_robot_states = []

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

                        start_t = time.time()
                        try:
                            req_json = json.loads(line)
                            joint_state = self.parse_input_payload(req_json)

                            # Advance simulation clock for history tracking (safe relative time)
                            ts = req_json.get("timestamp", global_step * self.time_step)
                            if ts > 10000.0:
                                ts = global_step * self.time_step
                            self.env.global_time = float(ts)
                            global_step += 1

                            # Step 1 -> 2 -> 3: JMID -> ORCA -> MPC with no_grad to prevent autograd graph leak
                            with torch.no_grad():
                                action = self.policy.predict(joint_state)

                            if global_step % 20 == 0:
                                if torch.cuda.is_available():
                                    torch.cuda.empty_cache()
                                elif hasattr(torch, "mps") and hasattr(torch.mps, "empty_cache"):
                                    torch.mps.empty_cache()

                            elapsed_ms = (time.time() - start_t) * 1000.0
                            logging.info(f"[Step {global_step}] Done in {elapsed_ms:.1f} ms | RAM: {get_mem_mb():.1f} MB")
                            resp = {
                                "status": "success",
                                "action": {
                                    "v": float(action.v),
                                    "r": float(action.r / self.time_step) if self.time_step > 0 else float(action.r)
                                },
                                "timestamp": self.env.global_time,
                                "compute_time_ms": round(elapsed_ms, 2)
                            }
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

