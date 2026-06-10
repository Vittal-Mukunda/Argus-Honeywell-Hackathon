# Day 7 — Ubuntu 26.04 bring-up, texture regression, and the literal 200 m gate

**Date:** 2026-06-10
**Objective:** verify every DP7 deliverable end-to-end on the new Ubuntu 26.04 +
RTX 4050 host, and close the one spec gap the project had never demonstrated:
**VIO drift < 1.5 % over 200 m** (longest prior run: 92.7 m shuttle).

> Status tokens: ✅ fixed/verified · ⚠️ open/limited · ❌ failed experiment.

---

## 1. Environment bring-up on Ubuntu 26.04 — ✅ acceptance 11/11

ROS 2 Humble has no 26.04 packages; the stack runs in the `argus:humble`
container (`docker/run.sh` / `docker/demo.sh`), GPU passthrough verified
(`nvidia-smi` inside the container sees the RTX 4050). Full acceptance:

```
ros2 run argus_bringup acceptance --full   →  ACCEPTED, 11/11 gated PASS
RTF = 0.902 under full world + sensors (was 0.42 WSL/iGPU, 0.84 WSL day-6)
```

Host-side eval venv (`~/.venvs/argus-eval`, Python 3.14) imports evo/rosbags/
matplotlib cleanly. The record → replay → eval loop was re-validated live
(fresh corridor bag → `run_vio_offline.sh` → `run_eval.py`).

## 2. detail.png texture regression — ✅ found + fixed  ← the big one

The README docs commit (`1dd00af`, June 8) **accidentally deleted
`src/argus_sim/worlds/detail.png`** — the PBR checker+speckle albedo that gives
the walls/floor their trackable detail. `install/` only holds a symlink into
`src/`, so every sim run since June 8 rendered **untextured flat-colour
walls** (ogre2 silently drops a missing albedo map). The Day-1 contract
explicitly freezes this texture; the README "live capture" GIFs and any run
recorded on this host were feature-starved without anyone noticing.

Restored byte-identical from the initial commit (`75f2c6b`). A/B on the same
flight profile (fresh corridor straight, mt0, 0.8 m/s):

| | untextured (regressed) | textured (restored) |
|---|---|---|
| KITTI segment drift (mean) | 5.91 % | **4.03 %** |
| ATE drift % | 7.68 % | 7.57 % |

Relative (KITTI) drift improves immediately; absolute ATE stays dominated by
two other effects, now isolated (below).

## 3. Fresh-run ATE error budget — ✅ root causes isolated

The fresh corridor runs sit at ~7.6 % ATE, far above day-6's 3.4–4.2 %. The
trajectory plot decomposes it:

* **XY tracking is excellent** (±0.05 m over the whole corridor).
* **Pitch-init tilt → Z-ramp**: VIO z ramps ~3.5 m across the run (≈ 7° tilt
  this pass; day-6 measured 1.3°). Same documented gravity-init limitation,
  worse with the step-start + untextured init view.
* **Hover-tail drift**: the recorder kept running ~10 s after the drive ended;
  a stationary, zero-excitation VINS keeps integrating → the synced tail
  poses inflate ATE RMSE and final drift (the "final drift 4.96 m" is almost
  entirely tail). Mission-segment metrics should end at flight end.

Mitigations baked into Scenario E (not config tuning): smooth 6 s vertical-
sinusoid excitation preamble (C¹-continuous — day-6's failed pre-roll had
sharp reversals), recorder SIGINTed the moment the flight script exits, and
near-field texture everywhere at init.

## 4. Scenario E — the literal 200 m drift gate — ✅ designed + flown

**World** `tunnel_circuit.sdf` (generated): 202.83 m stadium tunnel — two 70 m
straights + two r = 10 m semicircular end-caps, 6 × 3.5 m section. The world
IS the experiment design:

* **Segmented 5 m wall panels** re-tile `detail.png` per panel (an SDF box maps
  the albedo once per face — the corridor's single 30 m wall box stretched it
  into invisibility). Checker squares subtend 30–60 px at flight range: ideal
  Harris/KLT scale.
* **Arch ribs every ~10 m** + colour signage every ~12.7 m: near-field
  parallax + locally distinctive DBoW landmarks.
* **End-caps flown, not turned**: wz = v/R = 0.08 rad/s while translating at
  0.8 m/s — yaw with continuous parallax (the U-turn divergence mode is
  geometrically impossible on this course).
* **Closed circuit**: the lap ends 4 m past the spawn → loop_fusion can close
  the loop exactly where drift is measured.

**Flight** `fly_circuit.py`: GT-feedback path follower (GT steers the vehicle
only — the VIO never sees it), kinematic dry-run max lateral error 5 mm/lap.
Live flight matched: e = 0.00 m at lap end, z hold 1.000 m, 206.8 m flown.

**Recording** `record_scenario_E_tunnel.sh` → 28 GB sensor bag (rgb8 1280×720
stereo @ 15 Hz + 250 Hz IMU + GT). Offline replay `run_vio_loop_offline.sh`
at RATE 0.15 (the proven deterministic envelope for mt0).

## 5. Results — (filled in after the eval pass)

*pending*

## 6. Repo / deliverable hygiene — ✅

* `third_party/VINS-Fusion-ROS2` was **gitignored** → a fresh clone could not
  build the VIO. Now vendored in-repo (largest file 58 MB DBoW vocab, under
  GitHub's limit); `src/VINS-Fusion-ROS2` symlink made **relative**.
* Removed a 254 MB `core.442` (rviz2 crash dump) from the repo root; core
  dumps + bag dirs now gitignored.
* Per-package READMEs added to all 7 ROS 2 packages.
* `data/scenarios/scenario_E_tunnel_200m.yaml` added (reproduction commands
  inline, like A–D).
