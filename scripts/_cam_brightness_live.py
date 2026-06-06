#!/usr/bin/env python3
"""Subscribe /argus/cam0/image_raw, print mean brightness of the first frames, exit.
Run with ROS env sourced (system python). Diagnoses whether the live sim is lit."""
import sys
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

rclpy.init()
node = rclpy.create_node("brightness_probe")
seen = []


def cb(m):
    d = bytes(m.data)
    seen.append(sum(d) / len(d))


qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(Image, "/argus/cam0/image_raw", cb, qos)
import time
t0 = time.monotonic()
while rclpy.ok() and len(seen) < 5 and time.monotonic() - t0 < 20:
    rclpy.spin_once(node, timeout_sec=0.5)

if seen:
    print(f"cam0 mean brightness (first {len(seen)} frames): "
          f"{[round(v,1) for v in seen]}  -> {'LIT' if sum(seen)/len(seen) > 30 else 'DARK'}")
else:
    print("no cam0 frames received")
node.destroy_node()
rclpy.shutdown()
sys.exit(0)
