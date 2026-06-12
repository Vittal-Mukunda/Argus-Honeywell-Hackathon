# argus_health — VIO health monitor & failure recovery (Pillar 3)

A standalone watchdog over the VIO's **public topics only** (no estimator
coupling), implementing a failure-detection state machine with autonomous
recovery. Node: `health_monitor.py`.

## State machine

```
INITIALIZING ──first optimized odom──▶ NOMINAL ⇄ DEGRADED ⇄ LOST
```

- **INITIALIZING** → waiting for the first `/argus/vio/odom_optimized`.
- **NOMINAL** → healthy (feature count ≥ 80).
- **DEGRADED** → low features (< 40).
- **LOST** → features < 5 or optimized-odom timeout. When LOST persists > 0.4 s,
  **recovery** raises `/argus/health/recovery_active` (Bool), commands a
  zero-velocity hover hold, logs a recovery event, and increments the recovery
  count. It clears automatically when status returns to NOMINAL/DEGRADED.

## Derived signals

| Signal | Source | Purpose |
|--------|--------|---------|
| Inlier feature count | `/argus/vio/point_cloud` size (median-smoothed) | primary observability indicator |
| Parallax proxy | focal × speed × dt / depth | degenerate-motion detection |
| Drift-rate EMA | IMU-propagated vs optimized odom divergence | estimator divergence velocity (m/s) |
| IMU excitation | speed > 0.05 m/s OR gyro/accel variance | a stationary drone cannot converge VIO |
| Odom staleness | time since last optimized odom | dropout / blackout detection |
| Confidence | weighted composite ∈ [0, 1] | single health score |

Publishes `argus_msgs/VIOHealth` on `/argus/vio/health` plus the
`/argus/health/recovery_active` flag and `/argus/health/recovery_event`.

## Run & self-test

```bash
ros2 launch argus_health argus_health.launch.py
python3 scripts/_health_selftest.py   # scripted-timeline self-test, no sim needed
```

The self-test drives a synthetic INIT → HEALTHY → DARK → RELIT timeline and
asserts every transition (observed INIT/NOMINAL/LOST, recovery engaged then
cleared, recovery count ≥ 1, final status NOMINAL). Validated in **Scenario D**
(mid-flight Zone-B blackout): status reaches LOST and recovery engages 4 holds,
clearing on re-lighting — see root README §5 and §8.5.
