# SICNav Airport — Mac–Ubuntu TCP

Ubuntu는 Gazebo 공항 시뮬레이션과 관측 전송·속도 명령 수신을 담당하고, Apple Silicon Mac은 JMID와 ORCA/KKT MPC를 계산합니다. 이 저장소의 현재 실행 경로는 Mac–Ubuntu 통신 버전입니다.

Ubuntu 전용 JMID/MPC 실행 코드와 중복된 구형 TCP 실험 구현은 현재 트리에서 제거했습니다. 기존 커밋 이력은 유지됩니다. 저장소 이름의 `-ubuntu`는 기존 URL을 유지하기 위한 이름이며, Ubuntu 단독 계산 모드를 뜻하지 않습니다.

## 역할과 주기

| 위치 | 역할 |
| --- | --- |
| Ubuntu 24.04 ARM64, ROS 2 Jazzy, Gazebo Harmonic | 공항·TurtleBot4·군중 시뮬레이션, 관측 전송, 명령 수신 |
| Apple Silicon Mac | JMID(MPS), Acados 기반 ORCA/KKT MPC(CPU), 선택적 DR-SPAAM LiDAR 인식 |
| TCP 연결 | Mac controller 9150, 센서·상태·명령 9151–9153, SSH 역방향 터널 |

제어 요청 목표 간격은 시뮬레이션 시간 **0.25초(4Hz)**입니다. 이는 처리율 보장이 아닙니다. JMID는 복사한 관측 이력으로 비동기 계산하며, 150ms 소프트 데드라인을 넘으면 해당 tick은 현재 관측 기반 ORCA MPC를 사용합니다. 따라서 매 tick에서 새 JMID 결과를 사용하는 것은 아닙니다.

기존 Mac 통신 구현의 H4(4 × 0.25초), SQP4/QP5, 속도 제한과 대체 제어 로직을 유지합니다. 이전에 공유한 `tcp-4hz` 파일 묶음으로 되돌린 것이 아니라, 이 저장소에 이미 포함된 최신 `mac_offload/controller`를 유지한 정리입니다.

## 사전 준비

Ubuntu에는 기존 RMF 군중 플러그인, `ros_gz_bridge`, NumPy, ROS 메시지 및 `message_filters`가 필요합니다. GT 모드는 `sicnav_crowd_msgs`와 `/crowd_sim/states` publisher도 필요합니다. ROS/RMF를 새로 설치하는 자동 설치기는 포함하지 않습니다.

Mac에는 이미 검증된 SICNav 환경이 필요합니다. `SICNAV_MP_ROOT`는 아래 구조를 가진 환경 루트입니다.

```text
controller-environment/
  work/repo/     # SICNav-Diffusion source, configs and JMID checkpoints
  work/acados/   # built Acados installation
  work/venv/     # Python environment with Torch, Acados, RVO2, etc.
```

기반 SICNav revision은 `c702fb8ac9ba6439ca61da7dde68b8524bbc6a1f`, Acados는 `285d382b6c6d59c0983644caf6ed2924ed2153cb`입니다. Mac 환경·모델 가중치·생성 solver 바이너리는 저장소에 포함하지 않습니다. 기존 환경에 필요한 프로젝트 패치도 있어 upstream clone만으로 동일한 환경이 된다고 가정하면 안 됩니다. [Mac 실행 설명](experiments/mac_offload/README.md)과 [라이선스/의존성](experiments/mac_offload/controller/NOTICE)을 확인하세요.

## 실행

두 컴퓨터에 이 저장소를 복제합니다. `REPO`는 각 컴퓨터에서 복제한 폴더입니다.

### 1. Mac 초기 설정

```bash
cd "$REPO/experiments/mac_offload"
export SICNAV_MP_ROOT=/path/to/controller-environment
zsh setup_inference.command
"$SICNAV_MP_ROOT/work/venv/bin/python" controller/build_native.py
```

### 2. Mac: 각각 별도 터미널

```bash
# MPC와 JMID 서버
cd "$REPO/experiments/mac_offload"
export SICNAV_MP_ROOT=/path/to/controller-environment
zsh start_controller.command
```

```bash
# Gazebo 군중 상태를 사용하는 재현용 GT 모드
cd "$REPO/experiments/mac_offload"
zsh start_mac.command --humans 3 --perception-source gt
```

```bash
# Ubuntu SSH 계정/주소로 변경
cd "$REPO/experiments/mac_offload"
zsh start_tunnel.command user@ubuntu-host
```

### 3. Ubuntu: 각각 별도 터미널

```bash
# GUI 시뮬레이션만 시작; Ubuntu에서 JMID/MPC는 실행하지 않음
cd "$REPO"
bash scripts/run_ubuntu_simulation.sh
```

```bash
# 기본은 shadow 모드로 계산 결과를 확인
cd "$REPO"
bash scripts/run_ubuntu_gateway.sh --perception-source gt --expected-gt-people 3 --goal-x 138.03 --goal-y -41.33

# Gazebo 로봇을 실제로 움직일 때는 위 명령에 --drive 추가
```

서버와 gateway의 `--perception-source`는 동일하게 맞춰야 합니다. LiDAR 인식 모드는 양쪽 모두 기본 `lidar`를 사용합니다. 인식 성능은 GT 주행 결과와 구분해야 합니다. GUI 없이 실행하려면 `SICNAV_HEADLESS=1 bash scripts/run_ubuntu_simulation.sh`를 사용하세요.

시뮬레이션 터미널의 Ctrl+C로 Gazebo와 브리지를 종료합니다. Gateway도 별도 터미널에서 Ctrl+C로 종료합니다. Mac 통신 서버가 준비되지 않았으면 시뮬레이터만 켜져 있어도 자율주행하지 않습니다.

## 공항 장면

정지한 장식 사람 `passenger_standing_1`, `passenger_standing_2`를 제거했습니다. 이동 보행자는 유지했습니다. 시작 카메라는 로봇 시작 위치 뒤쪽 약 8m, 높이 6m에서 로봇을 향하도록 설정했습니다. Gazebo Harmonic `MinimalScene`의 `camera_pose`를 사용합니다.

## 검증 범위

[기존 H4 GT 주행 기록](experiments/mac_offload/validation/results/2026-11-22-h4-deadline/README.md)은 90초 실행에서 307회 native MPC 요청과 데드라인 시 ORCA 전환을 기록합니다. 이는 당시 코드·환경의 결과이며 이번 정리 후의 새 주행 결과는 아닙니다. 충돌률·성공률이나 LiDAR 인식 정확도를 보장하지 않습니다.

```bash
python3 -m unittest discover -s experiments/mac_offload/tests -v
python3 -m unittest discover -s experiments/mac_offload/controller/tests -v
```

폴더 구조: `experiments/mac_offload/`는 활성 Mac–Ubuntu 런타임, `models/`·`maps/`·`worlds/`는 공유 공항 자산, `scripts/`는 Ubuntu 시뮬레이션과 gateway 실행 진입점입니다.

## 기반 프로젝트

[Open-RMF](https://github.com/open-rmf/rmf_demos), [SICNav](https://github.com/sepsamavi/safe-interactive-crowdnav), [Acados](https://github.com/acados/acados), DR-SPAAM을 사용합니다. 포함된 코드의 라이선스는 `experiments/mac_offload/controller/UPSTREAM_LICENSE`, `experiments/mac_offload/DR_SPAAM_LICENSE` 및 각 의존성의 라이선스를 따릅니다.
