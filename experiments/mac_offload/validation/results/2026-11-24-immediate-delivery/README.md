# Immediate command delivery — 2026-11-24

Two sequential 60-wall-second simulator-GT shadow runs, three moving pedestrians. The robot remained stationary; `/cmd_vel` was not driven. Inputs, random samples, startup discovery and load were not matched.

| Metric | Before | After |
| --- | ---: | ---: |
| Observation callback → first valid candidate, mean | 221.7 ms | 164.8 ms |
| Same end-to-end latency, p95 | 290.6 ms | 228.6 ms |
| Same end-to-end latency, maximum | 387.8 ms | 365.4 ms |
| Received command → first publication, mean | 26.46 ms | 0.22 ms |
| Actual fresh-command frequency, simulation time | 3.41 Hz | 3.85 Hz |
| Controller responses / fresh accepted actions | 210 / 203 | 217 / 217 |
| Rejected commands / invalid nonzero candidates | 0 / 0 | 0 / 0 |

States are sent on odometry updates. The Mac planner wakes on state/perception updates, and the command sender wakes on computed results. Ubuntu immediately publishes validated received commands, while retaining its independent 50 ms watchdog and command/session/expiry limits. Publication is serialized with watchdog output and invalidation. Temporarily newer person observations wait for the corresponding robot state without clearing a pending computed command.

JMID/MPC model, H4, SQP4/QP5, and the minimum 0.25 simulation-second request interval are unchanged. In this world odometry arrives approximately every 0.052 simulation seconds, so the earliest eligible request interval is 0.26 seconds; exact 4 Hz is not achieved. End-to-end tail latency can still exceed 250 ms.

49 integration/transport tests and 27 controller tests passed, including immediate delivery, halt, expiry, stale odometry, concurrent watchdog publication, event wakeup/shutdown, and out-of-order state/observation arrival.

The final capture includes a startup sensor timeout/reconnect, repeated halt packets during discovery, and shutdown channel-close records. These are not classified as command rejections. This is a transport timing check, not a driving-success, collision-rate or matched-input MPC benchmark. `summary.json` preserves aggregate measurements and limitations; raw logs are local outputs.
