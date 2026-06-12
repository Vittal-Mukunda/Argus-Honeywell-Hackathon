# argus_vio — stereo-inertial VIO configuration & launch (Pillar 2)

The VIO deliverable: wires the vendored, locally-patched
[VINS-Fusion-ROS2](../../third_party/VINS-Fusion-ROS2) stereo-inertial estimator
onto the frozen ARGUS topic contract. **0.144 % final drift over 204.8 m**
(0.078 % ATE) — 10× inside the 1.5 %/200 m spec (root README §8.1).

## How it works

VINS-Fusion is a **tightly-coupled, optimization-based** stereo-inertial
estimator:

- **Front-end** — KLT/Harris corner detection + optical-flow tracking across the
  stereo pair (default C1). Optional SuperPoint learned keypoints behind
  `use_superpoint: 1` (ablation C2, see `argus_superpoint`).
- **IMU preintegration** — on-manifold preintegration of the 250 Hz IMU between
  keyframes makes scale, roll and pitch observable.
- **Back-end** — a sliding-window non-linear least-squares solve (Ceres Solver
  2.1) over visual reprojection + IMU residuals, max 8 iterations/keyframe.
- **Loop closure** — `loop_fusion` runs a DBoW2 bag-of-words place recogniser +
  PnP relocalisation and a 4-DOF pose-graph optimisation, snapping accumulated
  drift when the lap closes on the spawn.

Key config: keyframe parallax threshold 2.0 px (denser keyframes → richer
loop-closure DB), 150 tracked features, EuRoC-calibrated IMU noise,
`g_norm: 9.8` (the Gazebo gravity value), and **`multiple_thread: 0`** so the
estimator is single-threaded and bit-deterministic for offline evaluation.

## Files

| File | Role |
|------|------|
| `config/argus_stereo_imu_config.yaml` | Production estimator config (KLT/Harris front-end, `multiple_thread: 0`, EuRoC IMU noise, `g_norm: 9.8`) |
| `config/argus_stereo_imu_superpoint_config.yaml` | Ablation C2 config — identical except `use_superpoint: 1` |
| `config/argus_cam{0,1}_pinhole.yaml` | Camera intrinsics (1280×720, fx=fy=640, cx=640, cy=360, 0.12 m baseline) |
| `launch/argus_vio.launch.py` | `vins_node` remapped onto `/argus/vio/*` |
| `launch/argus_vio_loop.launch.py` | `vins_node` + `loop_fusion` (DBoW2 pose graph) → `/argus/vio/odom_loop` |

## Outputs

| Topic | Rate | Use |
|-------|------|-----|
| `/argus/vio/odom` | 250 Hz | IMU-propagated odometry (low latency) |
| `/argus/vio/odom_optimized` | keyframe-rate | sliding-window estimate — **evaluate this** |
| `/argus/vio/odom_loop` | keyframe-rate | loop-closure-corrected pose |
| `/argus/vio/point_cloud` | keyframe-rate | tracked feature cloud (health-monitor input) |
| `/argus/vio/image_track` | 15 Hz | feature-track overlay (onboard demo view) |

## Running offline (deterministic, decoupled from sim RTF)

```bash
bash scripts/run_vio_offline.sh <sensor_bag> <eval_bag>        # VIO only
bash scripts/run_vio_loop_offline.sh <sensor_bag> <eval_bag>   # VIO + loop closure
```

See root README §4 for the pipeline description and §8 for results.
