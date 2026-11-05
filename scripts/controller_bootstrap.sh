#!/usr/bin/env bash

echo ""
echo "========================================"
echo "[SICNav] Waiting for controller_manager"
echo "========================================"

source /opt/ros/jazzy/setup.bash

# controller_manager가 올라올 때까지 최대 90초 기다림
READY=0

for i in $(seq 1 90); do
    if ros2 service list 2>/dev/null | \
       grep -q "^/controller_manager/list_controllers$"; then
        READY=1
        echo "[SICNav] controller_manager is ready"
        break
    fi

    sleep 1
done

if [ "$READY" -ne 1 ]; then
    echo "[SICNav] controller_manager not present (using native gz diff-drive system)"
    exit 0
fi


# 이미 diffdrive_controller가 active라면 건드리지 않음
STATUS=$(ros2 service call \
    /controller_manager/list_controllers \
    controller_manager_msgs/srv/ListControllers \
    "{}" 2>/dev/null)

if echo "$STATUS" | grep -q "name='diffdrive_controller'.*state='active'"; then
    echo "[SICNav] diffdrive_controller already active"

    ros2 param set /motion_control safety_override full 2>/dev/null || true

    exit 0
fi


echo "[SICNav] Removing stuck automatic diffdrive spawner"

# TurtleBot4 spawn launch가 자동으로 만든 뒤 멈춰버리는 spawner 제거
pkill -9 -f "/controller_manager/spawner diffdrive_controller" 2>/dev/null || true

sleep 1

# ros2_control spawner stale lock 삭제
rm -f "$HOME/.ros/locks/ros2-control-controller-spawner.lock"


echo "[SICNav] Loading diffdrive_controller"

ros2 service call \
    /controller_manager/load_controller \
    controller_manager_msgs/srv/LoadController \
    "{name: 'diffdrive_controller'}"

sleep 1


echo "[SICNav] Configuring diffdrive_controller"

ros2 service call \
    /controller_manager/configure_controller \
    controller_manager_msgs/srv/ConfigureController \
    "{name: 'diffdrive_controller'}"

sleep 1


echo "[SICNav] Activating diffdrive_controller"

ros2 service call \
    /controller_manager/switch_controller \
    controller_manager_msgs/srv/SwitchController \
    "{activate_controllers: ['diffdrive_controller'],
      deactivate_controllers: [],
      strictness: 2,
      activate_asap: true,
      timeout: {sec: 5, nanosec: 0}}"

sleep 1


echo ""
echo "========================================"
echo "[SICNav] Controller status"
echo "========================================"

ros2 service call \
    /controller_manager/list_controllers \
    controller_manager_msgs/srv/ListControllers \
    "{}"


# 현재 시뮬에서 주행 성공했던 설정.
# 추후 sensor/hazard bridge가 정상화되면 제거 가능.
ros2 param set /motion_control safety_override full 2>/dev/null || true

echo ""
echo "[SICNav] TurtleBot4 ready"
echo ""
