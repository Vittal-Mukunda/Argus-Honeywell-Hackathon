# argus_nav — dense perception & reactive navigation (Pillar 4)

GPS-free sense-and-avoid. Four nodes (three classic + one tunnel variant), two
launch files and two RViz views. Every node consumes only public contract topics
(plus the additive `/argus/lidar/*` sensor) and the only frozen interface it
writes is `/argus/cmd_vel`; all other outputs are additive.

## Nodes

| Node | Inputs → Outputs | Complete working |
|------|------------------|------------------|
| `stereo_depth.py` | `/argus/cam{0,1}/image_raw` (+ `camera_info`) → `/argus/depth/{image,points}` | Rectify → **2× decimation** (1280×720 → 640×360) → **SGBM** block matching → **WLS** edge-preserving filter (λ=8000, σ=1.5) → reproject to a metric XYZ+RGB cloud → **Statistical Outlier Removal** (scipy→open3d→numpy fallback) → **0.3–12 m range gating**. Emits a 32FC1 depth image + a cloud capped at ~60k points. |
| `occupancy_mapper.py` | `/argus/depth/points` (+ pose) → `/argus/map/points` | **Log-odds** occupancy: `+l_hit` on the endpoint voxel, `−l_miss` ray-carved along the beam (free space). Per-voxel **RGB running average**, a **vectorised 26-neighbour speck filter** (int64-packed `searchsorted`, ~2× faster — keeps the publish rate up on the 100k+ voxel tunnel map), a **5 cm motion gate**, **AABB world bounds** (per-world), and an evidence **decay timer**. Pose can be identity, odom, or full GT pose (`pose_type`/`orientation_mode`). |
| `reactive_avoider.py` | depth + LiDAR + rangefinder + dead-reckoned distance → `/argus/cmd_vel` | **Corridor** controller (yaw-locked). Potential field = goal attraction (dead-reckoned, not absolute pose) + **density-independent** sector-binned obstacle repulsion (nearest point per angular sector) + lateral wall repulsion. Altitude held by a **PID on the downward rangefinder**. Local-minimum escape slides toward the clearer side; acceleration-limited; 0.8 m/s envelope. |
| `circuit_avoider.py` | GT pose + depth + LiDAR → `/argus/cmd_vel` + `/argus/nav/*` | **Tunnel** controller (yaws to follow the loop). See below. |

## circuit_avoider — the tunnel-loop driver

The corridor avoider holds spawn yaw and never turns, so it cannot fly the closed
`tunnel_circuit`. `circuit_avoider` exploits the drone's **holonomic** actuation
to decouple heading from lateral position:

- **Yaw** → follows the stadium **tangent** heading + arc feed-forward `wz = v/R`
  through the two semicircular end-caps (so the stereo pair stays aimed down the
  tunnel for good VINS parallax). Geometry mirrors `generate_tunnel_circuit.py`:
  straights `y=0` / `y=2R`, caps radius `R=10` about `(0,R)` and `(L,R)`, `L=70`,
  perimeter `2L + 2πR ≈ 202.83 m`, CCW.
- **Strafe (body +y)** → a potential field: a spring `−k·e` to the centreline
  **plus** sector-binned obstacle repulsion + local-minimum escape. The drone
  bows off-centreline to dodge a crate, then the spring rejoins it once the lane
  clears — a visible *detect → adjust → rejoin* arc, **no global replanning**.
- **Forward (body +x)** → constant cruise, braking near the lap goal or when an
  obstacle is inside the hard safety radius dead-ahead.
- **Altitude** → P-controller on GT z.

Obstacle sensing is **fully live** (stereo + LiDAR, gated to the altitude band
and inside the lane so the walls themselves don't repel). Steering reads
**ground-truth pose** because centreline-following needs a reliable pose on the
curving loop where the dynamics-blind kinematic IMU makes live VINS drift; the
demo headline is autonomous avoidance + live mapping, **not** GPS-free
localisation (that is the separate offline Scenario E benchmark).

**Additive judge-facing topics:** `/argus/nav/status` (`String` state line),
`/argus/nav/detected_obstacles` (`PointCloud2`, live in-lane points in world
frame), `/argus/nav/reference_path` (latched `Path` centreline), and
`/argus/nav/banner` (`Marker` — a floating plain-language status banner).

## Launch files & views

| File | Stack |
|------|-------|
| `launch/argus_nav.launch.py` | corridor: `stereo_depth` + `occupancy_mapper` (30 m bounds) + `reactive_avoider` |
| `launch/argus_tunnel_nav.launch.py` | tunnel: `stereo_depth` + `occupancy_mapper` (**full-stadium AABB**, 0.20 m voxels, 600k cap, GT-pose-oriented) + `circuit_avoider`. Args: `speed`, `laps`, `alt`, `decimation`, `enable_*`. |
| `rviz/argus_nav.rviz` | corridor sense-and-avoid view (fused map + trajectory) |
| `rviz/argus_tunnel.rviz` | stadium-framed digital-twin view (map + reference path + detected obstacles + banner) |

Sensor fusion across both controllers: stereo depth + 3D LiDAR (+ downward
rangefinder for the corridor altitude hold).

## Demos

```bash
docker/demo.sh --avoid         # corridor: autonomous sense-and-avoid
docker/demo.sh --tunnel-avoid  # tunnel: full 202.8 m autonomous loop + live map + dashboard
```

The live PyQt5 mission dashboard (`scripts/argus_dashboard_live.py`) is the 4th
window of `--tunnel-avoid`; the presenter script is `docs/DEMO_RUNBOOK.md`. See
root README §6 for the full description and §8 for results.
