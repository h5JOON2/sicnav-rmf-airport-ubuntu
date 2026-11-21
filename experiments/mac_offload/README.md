# LiDAR pedestrian detection and Mac offload

TurtleBot / Gazebo `/scan` + synchronized odometry → binary TCP → Mac DR-SPAAM (MPS) → tracked pedestrian positions → existing JMID/MPC controller → guarded robot commands.

Active perception uses only 2D LiDAR. The supplied world has one 720-ray, 10 Hz LiDAR and no camera sensors. The existing `../mp_tcp` controller sources are unchanged.

## Setup on Mac

Use the existing Apple Silicon controller environment described in `../mp_tcp`. It must provide `work/venv/bin/python` with PyTorch, NumPy and SciPy, and the controller's existing repo/Acados dependencies. Set its root explicitly:

```sh
export SICNAV_MP_ROOT=/path/to/controller-environment
zsh setup_inference.command
```

Setup creates `.venv-inference`, installs OpenCV (required by upstream utilities), clones the pinned official DR-SPAAM source and downloads its JRDB checkpoint. It checks the source revision and checkpoint SHA-256. Weights, environments and generated logs are not committed. See `NOTICE` and `DR_SPAAM_LICENSE`.

Start in separate terminals, retaining `SICNAV_MP_ROOT` in the controller terminal:

```sh
zsh start_controller.command
zsh start_mac.command --humans 3
zsh start_tunnel.command user@ubuntu-host
```

The controller uses MPS for JMID and its existing native CPU MPC. DR-SPAAM defaults to MPS and fails explicitly when MPS is unavailable; `start_mac.command --device cpu` selects CPU. The controller expects a fixed number of people (`--humans`, 1–8); configure matching controller capacity according to `../mp_tcp`. Missing tracks or excess detections stop planning. No dummy people or simulator ground truth are inserted.

## Ubuntu / Gazebo

Requires the existing airport assets and `rmf_building_sim_gz_plugins`, ROS 2 Jazzy, Gazebo and `ros_gz_bridge`; the gateway also requires NumPy, `rclpy`, `message_filters`, `sensor_msgs`, `nav_msgs`, and `geometry_msgs`. Copy this directory to Ubuntu. Run this derived world instead of another simulator instance:

```sh
export SICNAV_SIM_ASSETS=/path/to/airport-package
bash scripts/run_lidar_simulation.sh
```

In another terminal source the ROS environment, then:

```sh
python3 scripts/robot_offload_gateway.py --goal-x 138 --goal-y -35
```

SSH reverse forwarding connects Ubuntu localhost ports 9151 (scan), 9152 (state), and 9153 (commands) to Mac. The controller listens on Mac localhost 9150. Default output is `/remote/cmd_vel_candidate`. Add `--drive` to the gateway only when ready to publish `/cmd_vel`. The independent command watchdog checks age, sequence, session, epoch, finite values and velocity limits; disconnects and missing observations trigger stops.

For a real robot, set `--scan-topic`, `--scan-frame`, calibrated `--lidar-position X Y Z` and `--lidar-rpy R P Y` (radians). Defaults are for the included simulation: `tb4_lidar`, mount `(0,0,0.265)`. Odometry must describe `base_link`. LiDAR and odometry must synchronize within 35 ms. Scans are not motion-deskewed.

## Output semantics

DR-SPAAM detects person centers in the LiDAR plane. Logs include confidence, sensor-relative distance/angle and positions transformed using the calibrated sensor mount and synchronized robot odometry. ROS coordinates are x forward, y left. Positive angle is left. These are estimated current XY positions at scan height, not ground truth or full-body 3D centers. Occluded people and people outside scan coverage cannot be guaranteed. Track IDs are short-term nearest-neighbor associations, not persistent identities across occlusions.

## Validation and limits

```sh
.venv-inference/bin/python -m unittest discover -s tests -v
```

19 integration tests pass, covering packet validation, transforms, tracking, command guards, latest-frame behavior and the camera-free world. On saved actual 720-ray LiDAR scans, CPU outputs matched the official implementation for five scans (1e-6 tolerance). Thirty timed MPS calls after warmup averaged 50.3 ms (p95 75.7 ms); CPU averaged 4.86 s on the development Mac. These measure model processing, not network-to-command latency or detection accuracy.

The LiDAR version has not yet been validated live in Gazebo or in a closed driving loop. Moving-robot/moving-person accuracy and end-to-end timing remain to be measured. The 0.25 s controller interval is a scheduling target, not a measured guarantee. Simulation assets and controller environment must already be installed.

To regenerate the world from the airport source:

```sh
python3 scripts/build_lidar_world.py --help
```
