# LiDAR / GT Mac offload with deadline-aware JMID MPC

ROS2 observations → Mac pedestrian perception → JMID on MPS / native MPC on CPU → guarded velocity commands. The bundled `controller/` is the active implementation; Ubuntu only runs simulation and the transport gateway.

## Current behavior

- MPC and ORCA KKT: **horizon4 × 0.25s = 1s**. `--mpc-horizon 7` and `8` remain available. JMID always predicts8 future frames.
- SQP maximum4; each QP maximum5. Outer native convergence tolerance0.001. Candidate model feasibility≤0.05 includes internal ORCA stationarity/complementarity, plus a30cm predicted body-clearance check. Cached native values are reused; diagnostic nonlinear reevaluation, per-iteration audit writes and NPZ snapshots are disabled.
- One background JMID inference. If it misses a **150ms soft waiting budget**, that step directly uses current-observation ORCA MPC. While that inference remains running, later steps also use ORCA without queuing more work. Late predictions are discarded. A fresh timely inference restores the JMID pipeline. Existing proximity-based ORCA switching and one-step forecast delay remain enabled.
- Capacity3, actual observed counts0–3; no dummy people. Positive counts have separate preloaded native workers. Zero observations and the initial six-frame JMID-history warmup use the bounded local recovery planner, explicitly reporting `mpc_feasible=false`.
- TurtleBot4 safe-mode linear limit±0.31m/s; angular limit±1.9rad/s. Candidate tolerance can still produce slight overshoot: the gateway rejects commands beyond the limit, retains the previous valid command until its original deadline, then outputs zero. It does not clamp commands.
- The world contains3 moving pedestrians and no static person models. The actual number detected by LiDAR can be smaller.

## Mac setup

An existing external SICNav/Acados environment is required. Set `SICNAV_MP_ROOT` to its directory containing `work/repo`, `work/acados`, and `work/venv/bin/python`. Tested SICNav revision: `c702fb8ac9ba6439ca61da7dde68b8524bbc6a1f`. See `controller/NOTICE` and `controller/UPSTREAM_LICENSE`.

```sh
export SICNAV_MP_ROOT=/path/to/controller-environment
zsh setup_inference.command
"$SICNAV_MP_ROOT/work/venv/bin/python" controller/build_native.py
```

Setup creates `.venv-inference`, downloads pinned official DR-SPAAM source and JRDB weights, and applies compatibility patches. See `NOTICE` and `DR_SPAAM_LICENSE`. Environments, weights, runtime caches and compiled libraries are excluded from Git.

Start in separate terminals:

```sh
zsh start_controller.command --humans 3
zsh start_mac.command --humans 3
zsh start_tunnel.command user@ubuntu-host
```

JMID and DR-SPAAM require MPS by default and do not silently fall back to CPU. The controller uses the native CPU MPC. `--jmid-deadline-ms 150` overrides the default wait budget. The first solver build can take several minutes per worker; subsequent runs reuse the generated models. Horizon/model changes require a compatible build. Control-only deadline dispatch reuses the existing model-build fingerprint; bump it whenever the generated model or build options change.

## Ubuntu / Gazebo

Requires ROS2 Jazzy, Gazebo, `ros_gz_bridge`, the airport assets and crowd plugin, plus NumPy, `rclpy`, `message_filters`, and the ROS message packages. Export `SICNAV_SIM_ASSETS` to the airport asset directory. Run the derived world without another Gazebo instance:

```sh
export SICNAV_SIM_ASSETS=/path/to/airport-package
bash scripts/run_lidar_simulation.sh
```

Source `/opt/ros/jazzy/setup.bash` and the existing workspace setup, then start the gateway:

```sh
python3 scripts/robot_offload_gateway.py --goal-x 138.03 --goal-y -41.33
```

Default output is `/remote/cmd_vel_candidate` (shadow). Add `--drive` to publish `/cmd_vel` for simulated driving. Reverse SSH forwarding connects ports9151/9152/9153 to the Mac; controller port is9150. Watchdog checks session, epoch, sequence, finite values, velocity and age.

For actual LiDAR mounts configure `--scan-topic`, `--scan-frame`, `--lidar-position X Y Z`, and `--lidar-rpy R P Y`; defaults match the simulated `tb4_lidar` at(0,0,0.265). Odometry must describe `base_link`. Scan/odom pairing tolerance is35ms. Scans are not motion-deskewed. Person positions are current XY estimates, not full-body3D or persistent identities.

## Explicit simulator GT mode

For a timing diagnostic, use `--perception-source gt` on **both** `start_mac.command` and `robot_offload_gateway.py`. The gateway also accepts `--expected-gt-people 3` and requires `sicnav_crowd_msgs`. It consumes `/crowd_sim/states`, excludes external agents and preserves IDs/positions/velocities. This bypasses DR-SPAAM; it is not an automatic fallback on LiDAR failure. JMID remains on MPS.

```sh
zsh start_mac.command --humans 3 --perception-source gt
# Ubuntu after sourcing ROS and workspace:
python3 scripts/robot_offload_gateway.py --perception-source gt --expected-gt-people 3 \
  --goal-x 138.03 --goal-y -41.33 --drive --timing-log gateway.jsonl
```

## Tests and measured result

```sh
.venv-inference/bin/python -m unittest discover -s tests
.venv-inference/bin/python -m unittest discover -s controller/tests
```

68 tests passed. [Latest90s GT driving evidence](validation/results/2026-11-22-h4-deadline/README.md): all307 native MPC RPCs, including deadline-fallback steps, were below250ms in that run (median119.8ms, maximum201.7ms). This is an observation, not a hard timing guarantee or guaranteed4Hz command application. Deadline waiting does not cancel an already executing GPU operation.

`validation/run_validation.py` runs bounded captures and stops the services it launches. It needs SSH access and the external controller/simulator environment. Example from this directory:

```sh
.venv-inference/bin/python validation/run_validation.py --remote user@ubuntu-host \
  --mp-root "$SICNAV_MP_ROOT" --runtime "$PWD" --seconds 90 --humans 3 \
  --perception-source gt --start-simulation --drive --goal-x 138.03 --goal-y -41.33
```

The remote workspace defaults to `$HOME/sicnav_airport_ws/install/setup.bash`; override `--ros-workspace` for another location. Reports join session/epoch/sequence and ROS simulation stamps. Latency differences use same-host monotonic clocks; Mac/Ubuntu Unix timestamps are descriptive only.

Responses expose `jmid_deadline_status` (`ready`, `deadline`, `inflight`), `jmid_wait_ms`, request stamps and discarded-late inference durations. `jmid_ms` is null on fallback steps even though native ORCA MPC still runs; inspect `selection` and `controller_prediction_source` rather than treating those steps as local-planner results.
