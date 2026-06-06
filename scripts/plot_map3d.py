#!/usr/bin/env python3
"""Render the VIO warehouse MAP as a static 3D figure (headless, no GPU/GUI):
sparse point-cloud features + estimated trajectory + ground truth. Reliable judge visual."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

TS = get_typestore(Stores.ROS2_HUMBLE)
BAG = Path("/home/vittal/argus/data/bags/map_demo")
OUT = "/home/vittal/argus/data/eval/warehouse_map3d.png"

pts = []          # point_cloud features (current-window, dense)
margin = []       # margin_cloud (persistent map)
vio = []          # estimated trajectory
gt = []           # ground truth

with AnyReader([BAG], default_typestore=TS) as r:
    for conn, ts, raw in r.messages():
        m = r.deserialize(raw, conn.msgtype)
        if conn.topic == "/argus/vio/point_cloud":
            for p in m.points:
                pts.append((p.x, p.y, p.z))
        elif conn.topic == "/argus/vio/margin_cloud":
            for p in m.points:
                margin.append((p.x, p.y, p.z))
        elif conn.topic == "/argus/vio/odom":
            t = m.pose.pose.position
            vio.append((t.x, t.y, t.z))
        elif conn.topic == "/argus/ground_truth/pose":
            t = m.pose.position
            gt.append((t.x, t.y, t.z))

def clip(a):
    """Drop VINS init/triangulation outliers; keep the warehouse volume."""
    a = np.array(a) if len(a) else np.empty((0, 3))
    if not len(a):
        return a
    m = (np.abs(a[:, 0]) < 40) & (np.abs(a[:, 1]) < 12) & (a[:, 2] > -4) & (a[:, 2] < 10)
    return a[m]

pts = clip(pts)
margin = clip(margin)
vio = clip(vio)
gt = clip(gt)
print(f"(clipped) point_cloud pts={len(pts)} margin={len(margin)} vio={len(vio)} gt={len(gt)}")

fig = plt.figure(figsize=(15, 7))
fig.suptitle("ARGUS VIO — Warehouse map (extracted 3-D features) + flown trajectory",
             fontsize=15, fontweight="bold")

for i, (elev, azim, ttl) in enumerate([(22, -60, "perspective"), (89, -90, "top-down")]):
    ax = fig.add_subplot(1, 2, i + 1, projection="3d")
    if len(pts):
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=1.5, c="#ffb000",
                   alpha=0.35, label=f"map features ({len(pts)} pts)")
    if len(vio):
        ax.plot(vio[:, 0], vio[:, 1], vio[:, 2], c="#00c853", lw=2.2,
                label="VIO trajectory")
    if len(gt):
        ax.plot(gt[:, 0], gt[:, 1], gt[:, 2], c="#444", ls="--", lw=1.4,
                label="ground truth")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ax.set_title(ttl)
    ax.view_init(elev=elev, azim=azim)
    xhi = max(28, (pts[:, 0].max() + 1) if len(pts) else 28)
    ax.set_xlim(-1, xhi); ax.set_ylim(-3.5, 3.5); ax.set_zlim(0, 3.2)
    ax.set_box_aspect((xhi + 1, 7, 3.2))
    if i == 0:
        ax.legend(loc="upper left", fontsize=9)

plt.tight_layout()
plt.savefig(OUT, dpi=130, bbox_inches="tight")
print("saved", OUT)
