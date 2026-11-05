# SICNav Social Navigation in RMF Airport

<p align="center">
  <img src="assets/airport_tb4_demo.gif" width="900">
</p>
<p align="center">
  <b>Open-RMF Airport · Crowd Simulation · TurtleBot4 · Gazebo Harmonic</b><br>
  Social Navigation Environment for SICNav-Diffusion Integration
</p>

---

## 📑 Overview

Open-RMF의 **Airport Terminal** 환경에 moving crowd와 **TurtleBot4**를 통합한 social navigation simulation 환경.

ROS 2 Jazzy와 Gazebo Harmonic을 기반으로 공항 내부의 보행자 환경을 구성하고, TurtleBot4를 spawn하여 ROS2 control 기반 주행까지 검증.

최종 목표는 Gazebo에서 제공하는 보행자 상태를 기반으로 **JMID trajectory prediction + SICNav controller**를 연결하여 다중 보행자 환경에서 closed-loop social navigation을 구현하는 것.

```text
RMF Airport Terminal
        ↓
Crowd Simulation
        ↓
Human Agents
        +
TurtleBot4
        ↓
Human / Robot State
        ↓
JMID Trajectory Prediction
        ↓
SICNav Controller
        ↓
[v, ω]
        ↓
TurtleBot4
```

현재 단계에서는 **Airport + Crowd + TurtleBot4 simulation environment 구축 및 robot control validation** 완료.

---

## 🎯 Project Goal

다중 보행자가 이동하는 공항 환경에서 TurtleBot4 기반 social navigation pipeline 구축.

최종 구조:

```text
Moving Pedestrians
        ↓
Human State Estimation
        ↓
Trajectory History
        ↓
JMID
        ↓
Joint Human Future Prediction
        ↓
SICNav
        ↓
ORCA / KKT / MPC
        ↓
Velocity Command
        ↓
TurtleBot4
```

초기 단계에서는 Gazebo Ground Truth를 사용하고, 이후 RGB-D 기반 human tracking으로 확장 예정.

---

## 🧩 System Architecture

```text
┌──────────────────────────────────────┐
│           Open-RMF Airport           │
│                                      │
│   Crowd Agents        TurtleBot4     │
│       │                   │          │
└───────┼───────────────────┼──────────┘
        │                   │
        │                   ↓
        │              Robot State
        │
        ↓
 Human State
        │
        └─────────┬─────────┘
                  ↓
          Trajectory Buffer
                  ↓
               JMID
                  ↓
      Human Future Trajectories
                  ↓
              SICNav
                  ↓
          ORCA + KKT + MPC
                  ↓
              [v, ω]
                  ↓
       /cmd_vel_unstamped
                  ↓
        diffdrive_controller
                  ↓
             TurtleBot4
```

---

## 🛠 Environment

| Component | Configuration |
|---|---|
| OS | Ubuntu 24.04 |
| ROS | ROS 2 Jazzy |
| Simulator | Gazebo Harmonic |
| Environment | Open-RMF Airport Terminal |
| Crowd Simulation | RMF Crowd Simulation |
| Robot | TurtleBot4 Standard |
| Robot Control | ros2_control |
| Drive Controller | diffdrive_controller |
| ROS-Gazebo Interface | ros_gz_bridge |

---

## 🏢 RMF Airport Environment

Open-RMF demo의 Airport Terminal world 사용.

Crowd simulation 활성화:

```bash
source /opt/ros/jazzy/setup.bash
source ~/rmf_ws/install/setup.bash

ros2 launch rmf_demos_gz \
airport_terminal.launch.xml \
use_crowdsim:=1
```

Airport 내부에 다수의 moving pedestrian agent 생성.

```text
agent1
agent2
agent3
...
agent20
```

Crowd simulator가 각 pedestrian agent를 인식한 뒤 simulation 진행.

```text
Crowd Simulator found agent: agent16
Crowd Simulator found agent: agent17
Crowd Simulator found agent: agent18
...
Gazebo Models are all loaded!
Start simulating...
```

---

## 🚶 Crowd Simulation

RMF Crowd Simulation을 이용해 공항 내부에 이동 보행자 생성.

현재 단계에서는 Gazebo가 알고 있는 agent pose를 직접 활용하는 **Ground Truth 기반 human state pipeline**을 우선 사용.

```text
Gazebo Crowd Agent
        ↓
Agent ID
        ↓
World Pose
        ↓
x, y
        ↓
Temporal Difference
        ↓
vx, vy
        ↓
Human State
```

예정된 JMID 입력 형태:

```text
Human 1
[x, y, vx, vy]

Human 2
[x, y, vx, vy]

Human 3
[x, y, vx, vy]
```

Agent ID가 simulation 내부에서 이미 유지되기 때문에 초기 GT 기반 실험에서는 별도 tracking 불필요.

---

## 🤖 TurtleBot4 Integration

Robot:

```text
TurtleBot4 Standard
```

Airport world 내부 spawn position:

```text
x   = 136.5
y   = -41.7
z   = 0.1
yaw = 0.0
```

Spawn:

```bash
ros2 launch turtlebot4_gz_bringup \
turtlebot4_spawn.launch.py \
model:=standard \
x:=136.5 \
y:=-41.7 \
z:=0.1 \
yaw:=0.0 \
rviz:=false \
nav2:=false \
slam:=false \
localization:=false
```

Gazebo model 확인:

```bash
gz model --list | grep -Ei "turtlebot|dock"
```

확인된 entity:

```text
turtlebot4
standard_dock
```

Robot pose 확인:

```bash
gz model -m turtlebot4 --pose
```

확인된 pose:

```text
x = 136.522
y = -41.700
z ≈ 0.0
```

---

## ⚙️ ROS2 Control

TurtleBot4는 `ros2_control` 기반으로 구동.

기본 구조:

```text
/cmd_vel_unstamped
        ↓
motion_control
        ↓
diffdrive_controller
        ↓
ros2_control
        ↓
Wheel Joints
        ↓
TurtleBot4
```

Controller manager 확인:

```bash
ros2 service list | grep controller_manager
```

주요 service:

```text
/controller_manager/list_controllers
/controller_manager/load_controller
/controller_manager/configure_controller
/controller_manager/switch_controller
```

---

## 🔧 Controller Recovery

RMF Airport + TurtleBot4 통합 과정에서 automatic `diffdrive_controller` spawner가 lock 상태로 멈추는 문제 발생.

Lock file:

```text
~/.ros/locks/ros2-control-controller-spawner.lock
```

Stuck spawner 제거 후 controller manager service를 이용해 직접 controller 초기화.

### Load

```bash
ros2 service call \
/controller_manager/load_controller \
controller_manager_msgs/srv/LoadController \
"{name: 'diffdrive_controller'}"
```

### Configure

```bash
ros2 service call \
/controller_manager/configure_controller \
controller_manager_msgs/srv/ConfigureController \
"{name: 'diffdrive_controller'}"
```

### Activate

```bash
ros2 service call \
/controller_manager/switch_controller \
controller_manager_msgs/srv/SwitchController \
"{activate_controllers: ['diffdrive_controller'],
  deactivate_controllers: [],
  strictness: 2,
  activate_asap: true,
  timeout: {sec: 5, nanosec: 0}}"
```

Controller 활성화 이후 TurtleBot4 motion 정상 확인.

---

## ⏱ Gazebo Clock Bridge

RMF simulation의 ROS `/clock` 연결 문제를 해결하기 위해 별도 Gazebo-ROS clock bridge 사용.

```bash
ros2 run ros_gz_bridge parameter_bridge \
'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
--ros-args \
-r __node:=manual_clock_bridge
```

구조:

```text
Gazebo Simulation Clock
        ↓
ros_gz_bridge
        ↓
/clock
        ↓
ROS2 Simulation Time
```

---

## 📦 Gazebo Resource Path

RMF launch 환경에서 Gazebo resource path가 덮어써지면서 TurtleBot4 visual resource가 정상적으로 로드되지 않는 문제 발생.

필요한 ROS resource path:

```text
/opt/ros/jazzy/share
```

Gazebo system plugin 로딩을 위한 library path:

```text
/opt/ros/jazzy/lib
```

최종적으로 다음 두 환경 경로가 중요.

```text
GZ_SIM_RESOURCE_PATH
GZ_SIM_SYSTEM_PLUGIN_PATH
```

역할:

```text
GZ_SIM_RESOURCE_PATH
        ↓
Robot / Mesh / Model Resources

GZ_SIM_SYSTEM_PLUGIN_PATH
        ↓
Gazebo ROS Plugins
        ↓
gz_ros2_control
```

---

## 🎮 Teleoperation

TurtleBot4 manual control 검증:

```bash
ros2 run teleop_twist_keyboard \
teleop_twist_keyboard \
--ros-args \
--remap cmd_vel:=/cmd_vel_unstamped
```

입력된 velocity command가 motion control과 diffdrive controller를 통해 robot wheel command로 전달되는 것 확인.

```text
Keyboard
   ↓
cmd_vel
   ↓
/cmd_vel_unstamped
   ↓
motion_control
   ↓
diffdrive_controller
   ↓
TurtleBot4
```

---

## 🚀 Integrated Bringup

Airport, Crowd, TurtleBot4, clock bridge, controller bootstrap을 하나의 ROS2 package에서 관리.

```text
sicnav_airport_bringup/
│
├── CMakeLists.txt
├── package.xml
│
├── launch/
│   └── airport_tb4.launch.py
│
└── scripts/
    └── controller_bootstrap.sh
```

목표 실행 방식:

```bash
ros2 launch \
sicnav_airport_bringup \
airport_tb4.launch.py
```

`controller_bootstrap.sh` 역할:

```text
Wait for controller_manager
        ↓
Detect diffdrive_controller
        ↓
Remove stuck spawner
        ↓
Load Controller
        ↓
Configure Controller
        ↓
Activate Controller
        ↓
TurtleBot4 Ready
```

---

## 📁 Project Structure

```text
sicnav-rmf-airport/
│
├── README.md
├── CMakeLists.txt
├── package.xml
│
├── launch/
│   ├── airport_cropped_tb4.launch.py   # Cropped Concourse + TurtleBot4
│   └── airport_tb4.launch.py           # Multi-mode Bringup (Cropped / Full)
│
├── worlds/
│   └── airport_cropped.sdf             # Cropped Airport Gazebo Harmonic World
│
├── models/
│   ├── airport_cropped/                # Cropped Terminal Wall SDF
│   ├── PotatoChipChair/                # Airport Concourse Furniture
│   ├── WhiteChipChair/
│   ├── Standing person/                # Static Pedestrian
│   └── MaleVisitorPhone/               # Animated Pedestrian Actor
│
└── scripts/
    └── controller_bootstrap.sh         # diffdrive_controller auto-activation
```

---

## 🚀 Quick Start

### 1. ROS2 & RMF Source

```bash
source /opt/ros/jazzy/setup.bash
source ~/rmf_ws/install/setup.bash
```

### 2. Build

```bash
cd ~/social_nav_ws

colcon build \
--symlink-install \
--packages-select sicnav_airport_bringup
```

### 3. Source Workspace

```bash
source ~/social_nav_ws/install/setup.bash
```

### 4. Launch Simulation

#### A. 잘라낸 공항(Cropped Concourse) 실행 (기본 권장)
```bash
ros2 launch sicnav_airport_bringup airport_cropped_tb4.launch.py
```
또는
```bash
ros2 launch sicnav_airport_bringup airport_tb4.launch.py
```
> 기본 spawn 위치: `x=0.0, y=0.0, z=0.1, yaw=0.0` (중앙 통로)

#### B. 기존 전체 RMF Airport 실행 시
```bash
ros2 launch sicnav_airport_bringup airport_tb4.launch.py use_cropped:=false x:=136.5 y:=-41.7
```

---

## 🔄 Current Pipeline

현재 구현 범위:

```text
RMF Airport
    ↓
Crowd Simulation
    ↓
TurtleBot4 Spawn
    ↓
ROS2 Control
    ↓
diffdrive_controller
    ↓
Teleoperation
```

다음 integration 단계:

```text
Crowd Agent GT Pose
        ↓
Human Trajectory Buffer
        ↓
JMID
        ↓
Joint Future Prediction
        ↓
SICNav
        ↓
TurtleBot4
```

---

## 🧠 JMID Integration

JMID는 pedestrian history를 이용해 다중 보행자의 future trajectory를 joint하게 prediction.

예정 pipeline:

```text
Human History
        ↓
Trajectron++ Context Encoder
        ↓
JMID Diffusion Model
        ↓
Multiple Joint Future Samples
        ↓
Human Intended Motion
```

초기 입력은 Gazebo Ground Truth 사용.

이후 RGB-D perception pipeline으로 교체 예정.

---

## 🧭 SICNav Integration

JMID prediction을 SICNav controller와 연결.

예정 구조:

```text
JMID Prediction
        ↓
Human Preferred Velocity
        ↓
ORCA
        ↓
KKT Reformulation
        ↓
Bilevel MPC
        ↓
Acados
        ↓
[v, ω]
        ↓
TurtleBot4
```

---

## 📷 Human Perception & Tracking

GT 기반 navigation validation 이후 실제 perception pipeline 추가 예정.

```text
RGB-D Camera
        ↓
Person Detection
        ↓
Multi-Object Tracking
        ↓
Depth
        ↓
Camera XYZ
        ↓
TF
        ↓
Map-frame XYZ
        ↓
Velocity Estimation
        ↓
ID, x, y, vx, vy
        ↓
JMID
```

Detection 결과를 frame 간 association하여 persistent human ID와 trajectory history 생성.

---

## 📊 Planned Evaluation

Navigation 성능:

```text
Success Rate
Collision Rate
Minimum Human Distance
Time to Goal
Path Length
```

Prediction 성능:

```text
ADE
FDE
```

System 성능:

```text
JMID Inference Time
MPC Solve Time
Control Frequency
GPU Memory Usage
```

추가 검증:

```text
Gazebo GT Human State
vs
RGB-D Tracking Human State
```

---

## 🎯 Future Work

- Gazebo crowd Ground Truth extraction
- Human trajectory buffer
- JMID ROS2 integration
- SICNav controller integration
- Closed-loop TurtleBot4 navigation
- Nav2 global path integration
- RGB-D human detection
- Multi-object tracking
- Map-frame human state estimation
- GT vs perception comparison
- Crowd density experiments
- Social navigation performance evaluation

---

## 🚀 Execution Guide (Dual-System Architecture)

본 프로젝트는 **UTM Ubuntu 24.04 VM (Gazebo Harmonic 시뮬레이터)**과 **macOS 호스트 (Apple Silicon MPS JMID Diffusion + Acados MPC 추론)**가 TCP 소켓(`port 9090`)으로 실시간 연동되는 분산 아키텍처로 동작합니다.

```text
 [macOS Host: AI / MPC 서버]                  [UTM Ubuntu 24.04: Gazebo 클라이언트]
 ┌───────────────────────────┐                 ┌───────────────────────────────────┐
 │ sicnav/run_tcp_server.py  │                 │ Gazebo Harmonic (Airport Terminal)│
 │  ├── JMID Diffusion 예측  │  <── JSON ────  │  ├── Menge Crowd Simulator (/crowd)│
 │  ├── ORCA CasADi KKT      │  (Robot/Crowd)  │  └── TurtleBot4 (/odom)           │
 │  └── Acados Bilevel MPC   │                 │                                   │
 │       ↓                   │  ─── JSON ───>  │  sicnav_gazebo_client.py          │
 │   [v, ω] 계산 완료        │   (v, omega)    │  └── /cmd_vel (차동 구동 제어)     │
 └───────────────────────────┘                 └───────────────────────────────────┘
```

### 1. macOS 호스트에서 SICNav 추론 서버 실행

Mac 터미널에서 conda/python 환경 활성화 후:
```bash
cd sicnav
python3 run_tcp_server.py --host 0.0.0.0 --port 9090
```
> `[*] SICNav TCP Server listening on 0.0.0.0:9090...` 메시지와 함께 클라이언트 접속을 대기합니다.

### 2. 시뮬레이터 및 SICNav 클라이언트 실행

Ubuntu 터미널(또는 시뮬레이션 머신)에서:
```bash
# 워크스페이스 환경 로드
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 1) 로컬 단일 PC에서 실행 시 (기본값 server_ip:=127.0.0.1 자동 연결)
ros2 launch sicnav_airport_bringup airport_cropped_tb4.launch.py

# 2) 별도 호스트(Mac/외부 GPU 워크스테이션) 서버와 분산 통신 시
ros2 launch sicnav_airport_bringup airport_cropped_tb4.launch.py server_ip:=<호스트_IP>
# (예: UTM 가상머신 환경일 경우)
# ros2 launch sicnav_airport_bringup airport_cropped_tb4.launch.py server_ip:=192.168.64.1
```
> `airport_cropped_tb4.launch.py`에 `--server_ip` 및 `--port` 인수가 기본 내장되어 있어, 환경변수(`export SICNAV_SERVER_IP=...`) 또는 런치 인수(`server_ip:=...`)로 유연하게 다른 컴퓨터나 로컬 환경에 연결할 수 있습니다.

---

## 📌 Status

```text
RMF Airport Environment        ✅
Crowd Simulation               ✅
TurtleBot4 Spawn               ✅
Gazebo / ROS Clock Bridge      ✅
ROS2 Control                   ✅
diffdrive_controller           ✅
TurtleBot4 Teleoperation       ✅
Integrated Bringup             ✅
Crowd GT State Extraction      ✅
JMID Integration               ✅
SICNav Integration             ✅
Closed-loop Social Navigation  ✅

Nav2 Integration               ⏳
RGB-D Human Tracking           ⏳
```

---

## 📚 Related Projects

- Open-RMF
- Gazebo Harmonic
- TurtleBot4
- ROS 2 Jazzy
- SICNav-Diffusion
- JMID
- Trajectron++
- ORCA
- Acados
