#!/usr/bin/env python3
"""
SICNav Gazebo Client (Runs in UTM Ubuntu VM).

Connects Gazebo Harmonic + Open-RMF Menge simulation to macOS SICNav TCP Server:
1. Reads TurtleBot4 pose & velocity from /odom
2. Reads pedestrian poses from Gazebo /world/airport_cropped_world/pose/info
3. Sends state JSON to macOS SICNav server (192.168.64.1:9090)
4. Receives optimal velocity command (v, r) from Acados MPC + JMID
5. Publishes Twist command to /cmd_vel
"""

import sys
import os
import math
import time
import json
import socket
import threading
import argparse
from collections import defaultdict

import random
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# 15 Candidate Airport Destinations (matching behavior_file.xml)
CANDIDATE_GOALS = [
    (131.47, -32.31),  # Concourse NW
    (131.14, -44.42),  # Concourse SW
    (138.03, -41.33),  # Central Hallway
    (144.96, -44.65),  # Concourse SE
    (144.57, -32.01),  # Concourse NE
    (131.18, -37.88),  # West Walkway
    (135.03, -42.89),  # South-Central Corridor
    (141.05, -42.99),  # Central Crossing
    (144.89, -37.84),  # East Walkway
    (138.02, -32.16),  # North Corridor
    (147.95, -47.62),  # Gate A
    (127.92, -47.44),  # Gate B
    (130.81, -45.10),  # Gate C Lounge
    (150.22, -49.56),  # East Terminal Entry
    (125.43, -49.47),  # West Terminal Entry
]

# Optional Gazebo Transport for high-precision model poses
try:
    import gz.transport13 as gz_transport
    from gz.msgs10.pose_v_pb2 import Pose_V as GzPose_V
    HAS_GZ_TRANSPORT = True
except ImportError:
    try:
        import gz.transport12 as gz_transport
        from gz.msgs9.pose_v_pb2 import Pose_V as GzPose_V
        HAS_GZ_TRANSPORT = True
    except ImportError:
        HAS_GZ_TRANSPORT = False


def parse_args():
    parser = argparse.ArgumentParser(description="SICNav Gazebo Client")
    parser.add_argument(
        "--server_ip",
        type=str,
        default=os.environ.get("SICNAV_SERVER_IP", "127.0.0.1"),
        help="SICNav Server IP (default: 127.0.0.1 or env $SICNAV_SERVER_IP)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("SICNAV_SERVER_PORT", "9090")),
        help="SICNav Server Port (default: 9090 or env $SICNAV_SERVER_PORT)",
    )
    parser.add_argument("--goal_x", type=float, default=148.0, help="Robot Goal X (Airport global frame)")
    parser.add_argument("--goal_y", type=float, default=-41.7, help="Robot Goal Y (Airport global frame)")
    parser.add_argument("--rate", type=float, default=10.0, help="Control loop rate (Hz)")
    return parser.parse_known_args()[0]


class SicnavGazeboClient(Node):
    def __init__(self, args):
        super().__init__('sicnav_gazebo_client')
        self.server_ip = args.server_ip
        self.port = args.port
        self.goal_x = args.goal_x
        self.goal_y = args.goal_y
        self.dt = 1.0 / args.rate
        self.step_idx = 0

        self.get_logger().info(f"[SICNav Client] Target macOS Server: {self.server_ip}:{self.port}")
        self.get_logger().info(f"[SICNav Client] Robot Goal: ({self.goal_x}, {self.goal_y})")

        # State storage
        self.robot_px = 136.5
        self.robot_py = -41.7
        self.robot_theta = 0.0
        self.robot_vx = 0.0
        self.robot_vy = 0.0
        self.last_odom_time = time.time()
        self.state_lock = threading.Lock()

        # Pedestrian tracking: {name: {'id': int, 'px': float, 'py': float, 'vx': float, 'vy': float, 'last_t': float}}
        self.pedestrians = {}
        self.ped_id_map = {}
        self.next_ped_id = 1

        # ROS 2 Subscriptions & Publishers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(Odometry, '/odom', self.on_odom, qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Gazebo direct pose listener
        if HAS_GZ_TRANSPORT:
            self.gz_node = gz_transport.Node()
            self.gz_node.subscribe(GzPose_V, '/world/airport_cropped_world/pose/info', self.on_gz_pose)
            self.gz_node.subscribe(GzPose_V, '/world/airport_cropped_world/dynamic_pose/info', self.on_gz_pose)
            self.get_logger().info("[SICNav Client] Subscribed to Gazebo /pose/info for crowd ground truth")
        else:
            self.get_logger().warn("[SICNav Client] gz.transport not found; running with default human state")

        # Socket setup
        self.sock = None
        self.buffer = ""
        self.connect_server()

        # Control timer
        self.timer = self.create_timer(self.dt, self.control_step)

    def connect_server(self):
        while rclpy.ok():
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(3.0)
                self.sock.connect((self.server_ip, self.port))
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.get_logger().info(f"[SICNav Client] Successfully connected to macOS SICNav Server at {self.server_ip}:{self.port}!")
                return
            except Exception as e:
                self.get_logger().warn(f"[SICNav Client] Waiting for server ({self.server_ip}:{self.port})... Retrying in 2s")
                time.sleep(2.0)

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        lin_v = msg.twist.twist.linear.x

        with self.state_lock:
            self.robot_px = float(p.x)
            self.robot_py = float(p.y)
            self.robot_theta = float(yaw)
            self.robot_vx = float(lin_v * math.cos(yaw))
            self.robot_vy = float(lin_v * math.sin(yaw))
            self.last_odom_time = time.time()

    def on_gz_pose(self, msg):
        now = time.time()
        with self.state_lock:
            for p in msg.pose:
                name = p.name
                # Identify humans (Menge agents or visitor actors)
                is_human = any(k in name.lower() for k in ['human', 'agent', 'visitor', 'person', 'male', 'female', 'actor'])
                if is_human and 'turtlebot' not in name.lower():
                    if name not in self.ped_id_map:
                        self.ped_id_map[name] = self.next_ped_id
                        self.next_ped_id += 1

                    hid = self.ped_id_map[name]
                    x, y = float(p.position.x), float(p.position.y)

                    if name in self.pedestrians:
                        prev = self.pedestrians[name]
                        dt_p = max(0.01, now - prev['last_t'])
                        vx = (x - prev['px']) / dt_p
                        vy = (y - prev['py']) / dt_p
                        # Filter out teleport noise
                        if math.hypot(vx, vy) > 4.0:
                            vx, vy = prev['vx'], prev['vy']
                    else:
                        vx, vy = 0.0, 0.0

                    self.pedestrians[name] = {
                        'id': hid,
                        'px': x,
                        'py': y,
                        'vx': vx,
                        'vy': vy,
                        'radius': 0.25,
                        'last_t': now
                    }

    def control_step(self):
        with self.state_lock:
            rx = self.robot_px
            ry = self.robot_py
            r_vx = self.robot_vx
            r_vy = self.robot_vy
            r_th = self.robot_theta

            # Active humans (seen in last 1.0s)
            now = time.time()
            active_humans = [
                h for h in self.pedestrians.values()
                if (now - h['last_t']) < 1.0
            ]

        # Check goal distance
        # Use 1.3m tolerance for continuous smooth fly-by waypoint transitions (prevents near-goal crawling)
        dist_to_goal = math.hypot(self.goal_x - rx, self.goal_y - ry)
        if dist_to_goal < 1.3:
            # Reached current goal: randomly choose next destination among candidates (> 4m away)
            valid_candidates = [
                (gx, gy) for (gx, gy) in CANDIDATE_GOALS
                if math.hypot(gx - rx, gy - ry) > 4.0
            ]
            if valid_candidates:
                self.goal_x, self.goal_y = random.choice(valid_candidates)
                self.get_logger().info(
                    f"\n=======================================================\n"
                    f"[SICNav Client] >>> GOAL REACHED! Next Random Target: ({self.goal_x:.2f}, {self.goal_y:.2f}) <<<\n"
                    f"=======================================================\n"
                )

        # Prepare packet
        humans_payload = [
            {
                "id": h['id'],
                "px": round(h['px'], 3),
                "py": round(h['py'], 3),
                "vx": round(h['vx'], 3),
                "vy": round(h['vy'], 3),
                "radius": h['radius']
            }
            for h in active_humans[:5]  # limit to 5 closest if many
        ]

        sim_t = self.step_idx * self.dt
        self.step_idx += 1

        payload = {
            "timestamp": round(sim_t, 3),
            "robot": {
                "px": round(rx, 3),
                "py": round(ry, 3),
                "vx": round(r_vx, 3),
                "vy": round(r_vy, 3),
                "theta": round(r_th, 3),
                "gx": self.goal_x,
                "gy": self.goal_y,
                "radius": 0.25,
                "v_pref": 0.9
            },
            "humans": humans_payload
        }

        # Send JSON line
        try:
            req_data = (json.dumps(payload) + "\n").encode("utf-8")
            self.sock.sendall(req_data)

            # Receive response line
            while "\n" not in self.buffer:
                chunk = self.sock.recv(4096).decode("utf-8")
                if not chunk:
                    self.get_logger().warn("[SICNav Client] Connection closed by server. Reconnecting...")
                    self.connect_server()
                    return
                self.buffer += chunk

            line, self.buffer = self.buffer.split("\n", 1)
            resp = json.loads(line)

            action = resp.get("action", {})
            v = float(action.get("v", 0.0))
            r = float(action.get("r", 0.0))
            compute_time = resp.get("compute_time_ms", 0.0)

            # Publish to /cmd_vel
            twist = Twist()
            twist.linear.x = v
            twist.angular.z = r
            self.cmd_pub.publish(twist)

            self.get_logger().info(
                f"[Step] Robot=({rx:.2f}, {ry:.2f}) Hums={len(humans_payload)} "
                f"==> Cmd: v={v:.2f} m/s, w={r:.2f} rad/s ({compute_time:.1f} ms)"
            )

        except (socket.timeout, ConnectionResetError, BrokenPipeError):
            self.get_logger().warn("[SICNav Client] Socket disconnected. Reconnecting...")
            self.cmd_pub.publish(Twist())
            self.connect_server()
        except Exception as e:
            self.get_logger().error(f"[SICNav Client] Error in control loop: {e}")


def main():
    rclpy.init()
    args = parse_args()
    node = SicnavGazeboClient(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

