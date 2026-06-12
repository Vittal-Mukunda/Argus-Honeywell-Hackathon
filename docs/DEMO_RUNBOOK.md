# ARGUS — Live Tunnel Demo Runbook (judge presentation)

The 90-second story: a drone enters a 200 m service tunnel, builds a live 3D map of
its surroundings, detects obstacles in its path, and autonomously steers around them
while it continues mapping — finishing a full lap of the tunnel.

> Honest framing (say it this way): **autonomous obstacle avoidance / reactive local
> trajectory adjustment / real-time decision making.** This is reactive sense-and-avoid,
> NOT global route replanning — don't claim a planner we don't have. Localisation
> accuracy (the 0.144 % / 200 m drift result) is a **separate, offline-validated**
> benchmark — keep it off the live screen unless asked, and clearly labelled if shown.

---

## 1. One-command launch

```bash
docker/demo.sh --tunnel-avoid
```

Wait ~40 s for all windows to open and the drone to start flying. Stop everything with:

```bash
docker rm -f argus
```

## 2. The four things on screen

| Window | What it is | Point the judge at… |
|---|---|---|
| **RViz** (main) | The live **digital twin** | the RGB map building in real time; the **green** flown path **bowing off the grey centreline** at each obstacle; **red spheres** = obstacles detected; **yellow arrow** = drone; the floating **status banner** |
| **Gazebo** | Chase-cam inside the real tunnel | the drone passing the coloured obstacles (crates, hazard block, barrel) |
| **rqt** | Onboard camera + feature tracks | "what the drone sees" — the textured walls it tracks |
| **Dashboard** | Live mission telemetry | the **minimap**: green trail growing around the stadium, red marks where obstacles were found; the state pill flipping to *OBSTACLE DETECTED*; exploration %; sensor health all ● LIVE |

The **banner** in RViz states the decision in plain language:
`EXPLORING TUNNEL — n% mapped` → `OBSTACLE DETECTED — Adjusting course` →
`NAVIGATING TURN` → `MISSION COMPLETE`.

## 3. The arc to narrate (matches the obstacle layout)

1. **Enters & maps** — from spawn the map starts building immediately around the drone.
2. **First dodge (~24 m)** — amber crate on the right; the green path swings **left** and rejoins. Banner: *OBSTACLE DETECTED → Adjusting course*.
3. **Partial blockage (~36 m)** — yellow hazard block across most of the lane; the drone **slows and detours** to the open side.
4. **Narrow passage (~50 m)** — barrel + crate from both walls; the drone **threads the gap**.
5. **Right dodge (~60 m)** — green crate on the left; path swings **right**.
6. **The turn (~70 m)** — banner shows *NAVIGATING TURN* as it flies the curved end-cap, still mapping.
7. **Completes the lap** — map closes around the full 202.8 m stadium; banner: *MISSION COMPLETE*.

## 4. One-line talking points

- "GPS-denied: it navigates and maps from stereo cameras + LiDAR + IMU, no GPS."
- "The map is built **live** — every voxel is fused from what the drone is seeing right now."
- "It **senses obstacles and reacts** — the course you see bowing is the drone deciding, in real time, to go around."
- "Same mapping quality we validated in the 30 m test environment, now across the full 200 m tunnel."

## 5. If something looks off

- **No windows after ~60 s** → check `docker logs argus`, `/tmp/sim.log`, `/tmp/nav.log` in the container; GPU/X path issues show as blank Gazebo (see `docker/demo.sh` design notes).
- **Map looks clipped** → confirm the tunnel nav launch (full-stadium bounds) is the one running, not the 30 m warehouse launch.
- **Only one sim at a time** — always `docker rm -f argus` before relaunching.
