#!/usr/bin/env python3
"""ARGUS :: fly_shuttle.py

Fly a multi-leg forward/backward SHUTTLE along the corridor x-axis with NO yaw
turns. The drone always faces +x (cameras look down the corridor); forward legs
command +vx, return legs command -vx. Lateral (y), altitude (z) and heading (yaw)
are held on the corridor centreline by proportional ground-truth feedback.

Why this instead of U-turns (fly_uturn_laps.py):
  * VINS-Fusion stereo-inertial DIVERGES on in-place yaw U-turns: a pure rotation
    gives no translational parallax, KLT loses every track mid-turn, the back-end
    runs IMU-only and the state blows up (Ceres "Residual/Jacobian non-finite",
    t -> thousands of metres). Observed on the U-turn bag.
  * A no-yaw forward/back shuttle is ALWAYS translating along the optical axis, so
    parallax/scale stay observable and the estimator stays well-conditioned.
  * The reversal passes smoothly through zero velocity (trapezoidal ramp), so there
    is no step velocity-reversal transient.
  * Each return leg re-observes the SAME places from the SAME viewpoint (camera
    still faces +x), which is the ideal cue for DBoW loop closure -> the long-path
    drift gate AND the loop-closure validation in one flight.

Run with the ROS env sourced (rclpy + system python, NOT the eval venv):
  python3 scripts/fly_shuttle.py --laps 3 --leg-m 16 --speed 0.5
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped


def yaw_of(q) -> float:
    """Yaw about +z (ENU) from a quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ShuttleFlyer(Node):
    def __init__(self, a):
        super().__init__('shuttle_flyer')
        self.a = a
        self.pub = self.create_publisher(Twist, a.topic, 10)
        self.sub = self.create_subscription(PoseStamped, a.gt_topic, self._gt, 10)
        self.pos = None        # latest GT (x, y, z)
        self.yaw = None        # latest GT yaw
        self.phase = 'WAIT_GT'
        self.leg_idx = 0       # legs completed
        self.x_near = None     # near turnaround (spawn x)
        self.x_far = None      # far turnaround (spawn x + leg_m)
        self._dbg = 0
        self.phase_start = self.get_clock().now()
        self.timer = self.create_timer(1.0 / a.rate, self._tick)

    def _gt(self, msg):
        p = msg.pose.position
        self.pos = (p.x, p.y, p.z)
        self.yaw = yaw_of(msg.pose.orientation)

    def _elapsed(self, since) -> float:
        return (self.get_clock().now() - since).nanoseconds / 1e9

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    @staticmethod
    def _wrap(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _holds(self):
        """Proportional centreline/altitude/heading hold (vy, vz, wz)."""
        _, y, z = self.pos
        vy = self._clamp(self.a.lat_kp * (self.a.y0 - y), -0.3, 0.3)
        vz = self._clamp(self.a.alt_kp * (self.a.z0 - z), -0.3, 0.3)
        wz = self._clamp(self.a.yaw_kp * self._wrap(0.0 - self.yaw), -0.4, 0.4)
        return vy, vz, wz

    def _pub(self, vx=0.0, vy=0.0, vz=0.0, wz=0.0):
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.linear.z = float(vz)
        t.angular.z = float(wz)
        self.pub.publish(t)

    def _hold_pub(self, vx=0.0):
        """Publish a commanded vx plus the standing y/z/yaw holds."""
        if self.pos is None or self.yaw is None:
            self._pub(vx=vx)
            return
        vy, vz, wz = self._holds()
        self._pub(vx=vx, vy=vy, vz=vz, wz=wz)

    @staticmethod
    def _ramp(remaining, ramp):
        """Decelerate to a floor as `remaining` distance shrinks below `ramp`."""
        if ramp <= 0.0:
            return 1.0
        return max(0.2, min(1.0, remaining / ramp))

    def _begin(self, ph):
        self.phase = ph
        self.phase_start = self.get_clock().now()
        p = None if self.pos is None else tuple(round(v, 2) for v in self.pos)
        yd = None if self.yaw is None else round(math.degrees(self.yaw), 1)
        self.get_logger().info(
            f'phase -> {ph}  legs={self.leg_idx}/{self.a.laps * 2}  pos={p}  yaw_deg={yd}')

    def _tick(self):
        if self.pos is None or self.yaw is None:
            return  # wait for first ground-truth message
        x = self.pos[0]
        ph = self.phase

        if ph == 'WAIT_GT':
            self.x_near = x
            self.x_far = x + self.a.leg_m
            self._begin('HOVER_START')

        elif ph == 'HOVER_START':
            self._hold_pub()
            if self._elapsed(self.phase_start) >= self.a.hover_s:
                self._begin('LEG')

        elif ph == 'LEG':
            outbound = (self.leg_idx % 2 == 0)
            target = self.x_far if outbound else self.x_near
            remaining = (target - x) if outbound else (x - target)
            if remaining <= self.a.tol:
                self._hold_pub()  # arrived; stand on holds
                self.leg_idx += 1
                if self.leg_idx >= self.a.laps * 2:
                    self._begin('HOVER_END')
                else:
                    self._begin('HOVER_TURN')
            else:
                speed = self.a.speed * self._ramp(remaining, self.a.ramp_m)
                vx = speed if outbound else -speed
                self._dbg += 1
                if self._dbg % 20 == 0:
                    self.get_logger().info(
                        f'  LEG{"+" if outbound else "-"} x={x:.2f} -> {target:.1f}  '
                        f'rem={remaining:.2f}  y={self.pos[1]:.2f} z={self.pos[2]:.2f} '
                        f'yaw={math.degrees(self.yaw):.0f}')
                self._hold_pub(vx=vx)

        elif ph == 'HOVER_TURN':
            # Smooth dwell at the reversal: velocity is already ~0 here, the holds
            # null any residual lateral/heading error before the next leg starts.
            self._hold_pub()
            if self._elapsed(self.phase_start) >= self.a.turn_hover_s:
                self._begin('LEG')

        elif ph == 'HOVER_END':
            self._hold_pub()
            if self._elapsed(self.phase_start) >= self.a.hover_s:
                self.phase = 'DONE'


def main():
    p = argparse.ArgumentParser(description='No-yaw forward/back corridor shuttle.')
    p.add_argument('--laps', type=int, default=3, help='round trips (2 legs each).')
    p.add_argument('--leg-m', type=float, default=16.0, help='leg length, GT metres.')
    p.add_argument('--speed', type=float, default=0.5, help='cruise speed, m/s.')
    p.add_argument('--ramp-m', type=float, default=2.0, help='decel ramp distance, m.')
    p.add_argument('--tol', type=float, default=0.3, help='turnaround arrival tol, m.')
    p.add_argument('--y0', type=float, default=0.0, help='centreline y to hold, m.')
    p.add_argument('--z0', type=float, default=1.0, help='altitude z to hold, m.')
    p.add_argument('--lat-kp', type=float, default=0.8, help='lateral hold gain.')
    p.add_argument('--alt-kp', type=float, default=0.8, help='altitude hold gain.')
    p.add_argument('--yaw-kp', type=float, default=1.5, help='heading hold gain.')
    p.add_argument('--hover-s', type=float, default=5.0, help='start/end hover, s.')
    p.add_argument('--turn-hover-s', type=float, default=3.0, help='reversal dwell, s.')
    p.add_argument('--rate', type=float, default=20.0, help='cmd publish rate, Hz.')
    p.add_argument('--topic', default='/argus/cmd_vel')
    p.add_argument('--gt-topic', default='/argus/ground_truth/pose')
    a = p.parse_args(rclpy.utilities.remove_ros_args(sys.argv[1:]))

    rclpy.init()
    node = ShuttleFlyer(a)
    # Hard wall-time safety bound so the flight can never hang the recorder.
    est = a.laps * 2 * (a.leg_m / max(0.05, a.speed * 0.3)) + a.laps * 2 * 15 + 60
    deadline = time.monotonic() + est
    try:
        while rclpy.ok() and node.phase != 'DONE' and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info(f'flight finished: phase={node.phase}, legs={node.leg_idx}')
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            node._pub()  # ensure stopped
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
