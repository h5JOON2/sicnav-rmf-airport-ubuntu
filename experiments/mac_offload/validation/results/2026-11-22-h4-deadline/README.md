# H4 JMID deadline driving validation — 2026-11-22

GT 3 pedestrians, Gazebo driving through `/cmd_vel`, 90 wall seconds. MPC/ORCA KKT horizon4 at 0.25s, SQP4/QP5, JMID MPS, native CPU solver, `-O3 -ffast-math`. Limits: 0.31m/s and 1.9rad/s. Goal: (138.03,-41.33), start: (136.5,-41.7).

- All 307 native MPC requests including the first: RPC median119.8ms, p95 160.4ms, maximum201.7ms; zero over250ms.
- Deadline4 steps and inference-still-running3 steps used forced ORCA MPC; 300 steps received timely JMID results. Logs confirm subsequent return to the JMID pipeline. Existing proximity-triggered ORCA switching remains enabled on timely steps.
- First transition: sim3.068s; return to JMID: sim4.108s. A later transition at37.128s returned at37.752s.
- Actual waits on deadline steps were151.7–155.1ms. The150ms wait budget is soft, not immediate GPU cancellation or an overall latency guarantee.
- Zero controller errors and zero command rejections. Four sensor-disconnect halt messages were recorded during connection transitions.
- First entry within15cm of the goal:6.5 simulation seconds; no settled-arrival criterion was imposed.
- 27 controller tests plus41 integration tests passed. All six H4 solver binary hashes matched the prior H4 build; this control-only change reused them.

`deadline_timings.tsv` is copyable per-request data; `deadline_summary.json` contains transitions and before/after statistics. `summary.json` joins ROS2 timestamps and same-host monotonic durations. `build_verification.json` records hashes and flags; binaries are intentionally external. Sequential runs have different states/random samples/system load, so this is not a matched-input causal benchmark. This validates explicit simulator-GT mode; LiDAR detection quality is not measured here.
