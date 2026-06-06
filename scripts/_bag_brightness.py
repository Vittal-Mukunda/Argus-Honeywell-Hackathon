#!/usr/bin/env python3
"""Quick cam0 brightness/texture probe for a bag (diagnose dark/low-texture imagery)."""
import sys
from pathlib import Path
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

TS = get_typestore(Stores.ROS2_HUMBLE)


def probe(bag, n_at=(50, 150, 300)):
    print(f"\n=== {Path(bag).name} ===")
    with Reader(bag) as r:
        conns = [c for c in r.connections if c.topic == "/argus/cam0/image_raw"]
        i = 0
        for conn, _t, raw in r.messages(connections=conns):
            if i in n_at:
                m = TS.deserialize_cdr(raw, conn.msgtype)
                d = bytes(m.data)
                mean = sum(d) / len(d)
                # crude texture = mean abs diff of adjacent bytes
                step = max(1, len(d) // 20000)
                s = d[::step]
                tex = sum(abs(s[k] - s[k - 1]) for k in range(1, len(s))) / max(1, len(s) - 1)
                print(f"  frame {i:4d} {m.width}x{m.height} {m.encoding}: "
                      f"mean={mean:6.1f}  texture={tex:5.1f}")
            i += 1
            if i > max(n_at):
                break


for b in sys.argv[1:]:
    probe(b)
