#!/usr/bin/env python3
"""
Test TCP Client for SICNav TCP Server.
Sends dummy robot and human states and receives computed actions.
"""

import sys
import time
import json
import socket
import argparse


def parse_args():
    parser = argparse.ArgumentParser(description="Test TCP Client")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="Server IP")
    parser.add_argument("--port", type=int, default=9090, help="Server port")
    parser.add_argument("--steps", type=int, default=20, help="Number of test steps")
    parser.add_argument("--dt", type=float, default=0.25, help="Time step between packets (s)")
    return parser.parse_args()


def main():
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((args.host, args.port))
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f"Connected to SICNav TCP Server at {args.host}:{args.port}")

    # Initial dummy states
    robot_x = 0.0
    robot_y = 0.0
    robot_theta = 0.0
    goal_x = 5.0
    goal_y = 0.0

    human_x = 3.5
    human_y = 0.1
    human_vx = -0.5
    human_vy = 0.0

    buffer = ""

    for step in range(args.steps):
        t = step * args.dt

        # Prepare packet
        payload = {
            "timestamp": t,
            "robot": {
                "px": round(robot_x, 3),
                "py": round(robot_y, 3),
                "vx": 0.0,
                "vy": 0.0,
                "theta": round(robot_theta, 3),
                "gx": goal_x,
                "gy": goal_y,
                "radius": 0.25,
                "v_pref": 0.9
            },
            "humans": [
                {
                    "id": 1,
                    "px": round(human_x, 3),
                    "py": round(human_y, 3),
                    "vx": human_vx,
                    "vy": human_vy,
                    "radius": 0.2
                }
            ]
        }

        # Send JSON line
        msg = json.dumps(payload) + "\n"
        sock.sendall(msg.encode("utf-8"))

        # Receive JSON response line
        while "\n" not in buffer:
            chunk = sock.recv(4096).decode("utf-8")
            if not chunk:
                print("Server closed connection.")
                return
            buffer += chunk

        line, buffer = buffer.split("\n", 1)
        resp = json.loads(line)

        action = resp.get("action", {})
        v = action.get("v", 0.0)
        r = action.get("r", 0.0)
        compute_ms = resp.get("compute_time_ms", 0.0)

        print(f"[Step {step:02d}] T={t:0.2f}s | Robot pos=({robot_x:0.2f}, {robot_y:0.2f}) "
              f"| Human pos=({human_x:0.2f}, {human_y:0.2f}) "
              f"==> Output Action: v={v:0.3f} m/s, r={r:0.3f} rad/s ({compute_ms} ms)")

        # Simple motion simulation for next step
        robot_x += v * args.dt
        robot_theta += r * args.dt
        human_x += human_vx * args.dt

        time.sleep(args.dt)

    sock.close()
    print("Test finished.")


if __name__ == "__main__":
    main()

