#!/usr/bin/env python3
"""ARGUS circuit_avoider -- autonomous tunnel navigation with reactive avoidance.

This is the Scenario-E counterpart of ``reactive_avoider``: it flies the full
202.8 m ``tunnel_circuit`` stadium loop AND steers live around obstacles in the
flight lane. The straight-corridor avoider could not be used directly because it
holds spawn yaw and never turns -- the tunnel is a closed loop with two
semicircular end-caps, so the vehicle must yaw to follow it.

The drone is HOLONOMIC (gz VelocityControl twist: body forward/left/up + yaw), so
heading and lateral position are DECOUPLED, which is the whole trick here:

  * yaw   -> follows the tunnel TANGENT heading (drives the loop, arc feed-forward
             ``wz = v/R`` through the caps, exactly like fly_circuit.py),
  * strafe (body +y) -> a potential field that springs the drone back to the
             centreline AND pushes it off the centreline to dodge obstacles,
  * forward (body +x) -> constant cruise, only braking near the lap goal or when
             something sits inside the hard safety radius dead ahead.

So the drone keeps facing down the tunnel (good parallax for VINS, stereo pair
aimed at the corridor) while it slides sideways around a crate, then the spring
pulls it smoothly back onto the centreline once the lane is clear -- a visible
"detect -> adjust course -> rejoin" arc, with NO global replanning (this is
reactive local avoidance, not a route planner).

LOCALISATION: centreline-following needs a reliable pose on a curving loop where
live VINS drifts (the kinematic drone's IMU is dynamics-blind). Like fly_circuit
and the ``--avoid`` map, vehicle steering reads ground-truth pose
(``/argus/ground_truth/pose``). The VIO under test never sees it; obstacle
sensing is fully live (stereo + LiDAR). The headline of this demo is autonomous
obstacle avoidance + live mapping, NOT GPS-free localisation.

Geometry must match generate_tunnel_circuit.py: straights y=0 / y=2R for x in
[0, L]; semicircular caps radius R about (0, R) and (L, R); CCW.

Subscribes:
  /argus/ground_truth/pose  geometry_msgs/PoseStamped  (vehicle steering only)
  /argus/depth/points       sensor_msgs/PointCloud2     (stereo obstacle cloud)
  /argus/lidar/points       sensor_msgs/PointCloud2     (3D LiDAR obstacle cloud)

Publishes (additive -- see CONTRACT.md sec 8):
  /argus/cmd_vel      geometry_msgs/Twist   (existing frozen input topic)
  /argus/nav/status   std_msgs/String       human-readable state line
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       qos_profile_sensor_data)

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker

# ---- stadium geometry (must match generate_tunnel_circuit.py) ----------------
L = 70.0
R = 10.0
PERIM = 2 * L + 2 * math.pi * R     # 202.83 m
CY = R                               # arc centres at (0, R) and (L, R)


def centerline(s):
    """Inverse of path_state: world (x, y, heading) at centreline arc-length s.

    Mirrors generate_tunnel_circuit.centerline so the reference route drawn for
    the judges is exactly the tunnel the world was built around."""
    s = s % PERIM
    if s < L:                                   # straight A: (0,0)->(L,0)
        return s, 0.0, 0.0
    s -= L
    if s < math.pi * R:                         # right end-cap, CCW about (L, R)
        phi = -math.pi / 2 + s / R
        return L + R * math.cos(phi), CY + R * math.sin(phi), phi + math.pi / 2
    s -= math.pi * R
    if s < L:                                   # straight B: (L,2R)->(0,2R)
        return L - s, 2 * R, math.pi
    s -= L                                      # left end-cap, CCW about (0, R)
    phi = math.pi / 2 + s / R
    return R * math.cos(phi), CY + R * math.sin(phi), phi + math.pi / 2


def make_xyz_cloud(header, xyz) -> PointCloud2:
    """Minimal XYZ float32 PointCloud2 (colour is set flat in RViz)."""
    n = xyz.shape[0]
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * n
    msg.is_dense = True
    msg.data = xyz.astype(np.float32).tobytes()
    return msg


def parse_xyz(msg: PointCloud2) -> np.ndarray:
    """Extract an (N,3) float32 XYZ array from a PointCloud2 by field offsets."""
    off = {f.name: f.offset for f in msg.fields}
    if not {'x', 'y', 'z'} <= off.keys() or msg.width * msg.height == 0:
        return np.empty((0, 3), np.float32)
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    out = np.empty((raw.shape[0], 3), np.float32)
    for i, ax in enumerate(('x', 'y', 'z')):
        out[:, i] = raw[:, off[ax]:off[ax] + 4].copy().view(np.float32).ravel()
    return out[np.isfinite(out).all(axis=1)]


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def wrap_phase(a: float) -> float:
    """Angle wrapped to [0, 2*pi) -- arc progress is always forward (CCW)."""
    while a < 0.0:
        a += 2 * math.pi
    while a >= 2 * math.pi:
        a -= 2 * math.pi
    return a


def path_state(x: float, y: float):
    """(s, theta_tangent, e_left, on_arc) for the CCW stadium at world (x, y).

    ``e_left`` is the signed lateral position relative to the centreline,
    POSITIVE to the left of the direction of travel (body +y when the drone
    holds the tangent heading).
    """
    if x > L:                                   # right end-cap, centre (L, R)
        phi = math.atan2(y - R, x - L)
        s = L + R * wrap_phase(phi + math.pi / 2)
        return s, phi + math.pi / 2, R - math.hypot(x - L, y - R), True
    if x < 0.0:                                 # left end-cap, centre (0, R)
        phi = math.atan2(y - R, x)
        s = 2 * L + math.pi * R + R * wrap_phase(phi - math.pi / 2)
        return s, phi + math.pi / 2, R - math.hypot(x, y - R), True
    if y < R:                                   # straight A (heading +x)
        return x, 0.0, y, False
    return L + math.pi * R + (L - x), math.pi, 2 * R - y, False   # straight B


class CircuitAvoider(Node):
    def __init__(self):
        super().__init__('circuit_avoider')

        # --- mission / envelope ---------------------------------------------
        self.declare_parameter('speed', 0.8)            # m/s cruise (flight envelope)
        self.declare_parameter('alt', 1.0)              # m, hold altitude
        self.declare_parameter('laps', 1)               # laps before stopping
        self.declare_parameter('overrun', 4.0)          # m past lap end (re-observe start)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('max_accel', 0.6)        # m/s^2 slew on forward+strafe

        # --- centreline following (heading) ---------------------------------
        self.declare_parameter('k_h', 1.2)              # heading P-gain (1/s)
        self.declare_parameter('k_center', 0.7)         # lateral spring to centreline (1/s)
        self.declare_parameter('lat_max', 0.7)          # m/s lateral strafe clamp
        self.declare_parameter('k_alt', 0.8)            # altitude P-gain
        self.declare_parameter('max_climb', 0.3)        # m/s vertical clamp

        # --- potential-field obstacle repulsion (lateral) -------------------
        self.declare_parameter('k_rep', 0.5)
        self.declare_parameter('rep_sectors', 24)       # density-free angular bins
        self.declare_parameter('influence_d0', 2.0)     # m, repulsion range
        self.declare_parameter('safety_radius', 0.6)    # m, hard "too close" dead-ahead
        self.declare_parameter('obstacle_z_band', 0.8)  # m, |dz| around drone counted
        self.declare_parameter('wall_ignore', 2.6)      # m, ignore obstacles past this (walls)
        self.declare_parameter('tangential_gain', 0.7)  # local-minimum escape (m/s)

        # --- io -------------------------------------------------------------
        self.declare_parameter('gt_topic', '/argus/ground_truth/pose')
        self.declare_parameter('cloud_topic', '/argus/depth/points')
        self.declare_parameter('lidar_topic', '/argus/lidar/points')
        self.declare_parameter('use_lidar', True)
        self.declare_parameter('cmd_topic', '/argus/cmd_vel')
        # cam0 optical-frame + lidar mount offsets in body FLU (from model.sdf).
        self.declare_parameter('cam_offset', [0.10, 0.06, 0.0])
        self.declare_parameter('lidar_offset', [0.0, 0.0, 0.12])

        self.pose = None                # (x, y, z)
        self.yaw = None
        self.s_prev = None
        self.dist = 0.0                 # unwrapped centreline progress (m)
        self.goal = (int(self.get_parameter('laps').value) * PERIM
                     + float(self.get_parameter('overrun').value))
        self.done = False
        self.obs_stereo = np.empty((0, 3), np.float32)  # gated, body FLU
        self.obs_lidar = np.empty((0, 3), np.float32)
        self.obs_body = np.empty((0, 3), np.float32)
        self.min_ahead = float('inf')   # nearest obstacle in the forward lane
        self._cmd = np.zeros(2)         # accel-limited [forward, strafe]
        self._last_ctrl = None
        self._log_t = 0.0

        self.pub_cmd = self.create_publisher(
            Twist, self.get_parameter('cmd_topic').value, 10)
        self.pub_status = self.create_publisher(String, '/argus/nav/status', 10)

        # --- judge-facing viz (additive) ------------------------------------
        # detected_obstacles: the LIVE in-lane obstacle points the drone is
        # reacting to, transformed to world + published so RViz highlights exactly
        # what was sensed. reference_path: the tunnel centreline (the "intended
        # route"), latched so RViz shows the actual green course bowing off it.
        self.map_frame = 'world'
        self.pub_obs = self.create_publisher(
            PointCloud2, '/argus/nav/detected_obstacles', qos_profile_sensor_data)
        latched = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_ref = self.create_publisher(Path, '/argus/nav/reference_path', latched)
        self._ref_path = self._build_reference_path()
        self.create_timer(1.0, self._publish_reference)   # re-latch for late RViz
        # banner: a big plain-language status line floating above the stadium so a
        # non-technical judge reads what the drone is doing at a glance.
        self.pub_banner = self.create_publisher(Marker, '/argus/nav/banner', 10)
        self.declare_parameter('banner_pos', [35.0, 10.0, 8.0])   # above stadium centre

        self.create_subscription(PoseStamped, self.get_parameter('gt_topic').value,
                                 self._on_pose, 10)
        self.create_subscription(PointCloud2, self.get_parameter('cloud_topic').value,
                                 self._on_cloud, qos_profile_sensor_data)
        self.use_lidar = bool(self.get_parameter('use_lidar').value)
        if self.use_lidar:
            self.create_subscription(PointCloud2, self.get_parameter('lidar_topic').value,
                                     self._on_lidar, qos_profile_sensor_data)

        rate = float(self.get_parameter('control_rate').value)
        self.timer = self.create_timer(1.0 / rate, self._control)
        self.get_logger().info(
            'circuit_avoider up: %d lap(s) x %.1f m + %.1f m overrun at %.2f m/s, '
            'lidar=%s -> %s'
            % (int(self.get_parameter('laps').value), PERIM,
               float(self.get_parameter('overrun').value),
               float(self.get_parameter('speed').value), self.use_lidar,
               self.get_parameter('cmd_topic').value))

    # -- subscriptions --------------------------------------------------------
    def _on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        if math.isfinite(p.x) and math.isfinite(p.y):
            self.pose = (p.x, p.y, p.z)
            self.yaw = yaw_of(msg.pose.orientation)

    def _on_cloud(self, msg: PointCloud2):
        xyz = parse_xyz(msg)
        if xyz.shape[0] == 0:
            self.obs_stereo = np.empty((0, 3), np.float32)
        else:
            # optical (x right, y down, z fwd) -> body FLU (x fwd, y left, z up)
            ox, oy, oz = self.get_parameter('cam_offset').value
            body = np.stack([xyz[:, 2] + ox, -xyz[:, 0] + oy, -xyz[:, 1] + oz], axis=1)
            self.obs_stereo = self._gate(body)
        self._fuse()

    def _on_lidar(self, msg: PointCloud2):
        xyz = parse_xyz(msg)
        if xyz.shape[0] == 0:
            self.obs_lidar = np.empty((0, 3), np.float32)
        else:
            lx, ly, lz = self.get_parameter('lidar_offset').value
            body = np.stack([xyz[:, 0] + lx, xyz[:, 1] + ly, xyz[:, 2] + lz], axis=1)
            self.obs_lidar = self._gate(body)
        self._fuse()

    def _gate(self, body: np.ndarray) -> np.ndarray:
        """Keep obstacles near flight altitude, ahead, and inside the lane.

        Points past ``wall_ignore`` laterally are the tunnel walls (at +/-3 m) --
        dropping them stops the corridor itself from exerting repulsion, so only
        real in-lane obstacles steer the drone."""
        zband = float(self.get_parameter('obstacle_z_band').value)
        wall = float(self.get_parameter('wall_ignore').value)
        keep = ((np.abs(body[:, 2]) < zband) & (body[:, 0] > -0.3)
                & (np.abs(body[:, 1]) < wall))
        return body[keep].astype(np.float32)

    def _fuse(self):
        if self.obs_stereo.shape[0] or self.obs_lidar.shape[0]:
            self.obs_body = np.vstack([self.obs_stereo, self.obs_lidar])
        else:
            self.obs_body = np.empty((0, 3), np.float32)

    # -- judge-facing viz -----------------------------------------------------
    def _build_reference_path(self) -> Path:
        """The tunnel centreline as a Path (the 'intended route' the drone follows)."""
        path = Path()
        path.header.frame_id = self.map_frame
        alt = float(self.get_parameter('alt').value)
        n = int(PERIM / 0.5) + 1
        for i in range(n):
            cx, cy, _ = centerline(i * 0.5)
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = cx, cy, alt
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        return path

    def _publish_reference(self):
        self._ref_path.header.stamp = self.get_clock().now().to_msg()
        self.pub_ref.publish(self._ref_path)

    def _publish_detected(self, x, y, z, yaw):
        """Transform the near, in-lane obstacle points to world + publish (red in
        RViz) so judges see exactly what the drone is detecting right now."""
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = self.map_frame
        if self.obs_body.shape[0] == 0:
            self.pub_obs.publish(make_xyz_cloud(hdr, np.empty((0, 3), np.float32)))
            return
        b = self.obs_body
        d = np.hypot(b[:, 0], b[:, 1])
        # Highlight obstacles as soon as they enter close sensor range (a little
        # beyond the repulsion influence d0), so the red cloud appears EARLY and
        # reads continuously as the drone approaches -- VIZ ONLY; the control path
        # still uses d0 for repulsion, so flight behaviour is unchanged.
        viz_range = max(float(self.get_parameter('influence_d0').value), 3.0)
        sel = (d < viz_range) & (np.abs(b[:, 1]) < 1.6) & (b[:, 0] > 0.0)
        bs = b[sel]
        if bs.shape[0] == 0:
            self.pub_obs.publish(make_xyz_cloud(hdr, np.empty((0, 3), np.float32)))
            return
        c, s = math.cos(yaw), math.sin(yaw)
        world = np.empty_like(bs)
        world[:, 0] = x + bs[:, 0] * c - bs[:, 1] * s
        world[:, 1] = y + bs[:, 0] * s + bs[:, 1] * c
        world[:, 2] = z + bs[:, 2]
        self.pub_obs.publish(make_xyz_cloud(hdr, world))

    def _publish_banner(self, state, explored):
        """Big plain-language status floating above the stadium (judge-readable)."""
        text, col = self._banner_text(state, explored)
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'banner'
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        bx, by, bz = self.get_parameter('banner_pos').value
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(bx), float(by), float(bz)
        m.pose.orientation.w = 1.0
        m.scale.z = 2.2                       # text height (m), readable at stadium scale
        m.color.r, m.color.g, m.color.b, m.color.a = col
        m.text = text
        self.pub_banner.publish(m)

    @staticmethod
    def _banner_text(state, explored):
        """Map the internal state to a judge-facing line + colour (RGBA)."""
        if state == 'GOAL_REACHED':
            return 'MISSION COMPLETE\nTunnel mapped', (0.2, 1.0, 0.4, 1.0)        # green
        if state == 'AVOIDING':
            return 'OBSTACLE DETECTED\nAdjusting course', (1.0, 0.65, 0.0, 1.0)   # amber
        if state == 'ARC':
            return 'NAVIGATING TURN\nExploring  %d%% mapped' % int(explored), (0.3, 0.8, 1.0, 1.0)  # cyan
        return 'EXPLORING TUNNEL\n%d%% mapped' % int(explored), (0.95, 0.95, 1.0, 1.0)  # white

    # -- control loop ---------------------------------------------------------
    def _control(self):
        if self.pose is None:
            return
        x, y, z = self.pose
        s, th_t, e, on_arc = path_state(x, y)

        # --- unwrapped lap progress (reject GT glitches / lap wrap) ----------
        if self.s_prev is not None:
            ds = s - self.s_prev
            if ds < -PERIM / 2:
                ds += PERIM
            if -2.0 < ds < 5.0:
                self.dist += max(ds, 0.0)
        self.s_prev = s
        self._publish_detected(x, y, z, self.yaw)   # highlight live detections

        speed = float(self.get_parameter('speed').value)
        remain = self.goal - self.dist
        if remain <= 0.0 and not self.done:
            self.done = True
            self.get_logger().info('circuit_avoider: done -- %.1f m flown, hovering.'
                                   % self.dist)
        if self.done:
            self.pub_cmd.publish(Twist())
            self._status('GOAL_REACHED', 0.0, e, 0.0)
            return

        # --- lateral potential field: spring-to-centreline + repulsion -------
        v_strafe, avoiding = self._lateral_field(e)

        # --- forward cruise: brake near goal; creep only if blocked ahead ----
        vx = speed
        if remain < 2.0:
            vx = min(vx, max(0.15, speed * remain / 2.0))
        if self.min_ahead < float(self.get_parameter('safety_radius').value):
            vx = min(vx, 0.1)            # something dead ahead: nearly stop, strafe clears it

        # --- accel-limit forward + strafe for smooth, physical motion --------
        fwd, strafe = self._accel_limit(np.array([vx, v_strafe]))

        # --- heading: hold tunnel tangent (+ arc feed-forward), no lat term --
        k_h = float(self.get_parameter('k_h').value)
        ff = (speed / R) if on_arc else 0.0
        wz = float(np.clip(ff + k_h * wrap(th_t - self.yaw), -0.5, 0.5))

        # --- altitude hold on GT z (independent of x/y) ----------------------
        k_alt = float(self.get_parameter('k_alt').value)
        max_climb = float(self.get_parameter('max_climb').value)
        vz = float(np.clip(k_alt * (float(self.get_parameter('alt').value) - z),
                           -max_climb, max_climb))

        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z = float(fwd), float(strafe), vz
        cmd.angular.z = wz
        self.pub_cmd.publish(cmd)

        state = 'AVOIDING' if avoiding else ('ARC' if on_arc else 'CRUISE')
        self._status(state, remain, e, strafe)

    def _lateral_field(self, e: float):
        """Body-y strafe velocity = obstacle repulsion + spring to the centreline.

        Returns (v_strafe, avoiding). With no obstacle the spring ``-k_center*e``
        pulls the drone to the centreline; a near obstacle adds a sideways push
        toward the clearer side, bowing the drone off-centreline, after which the
        spring rejoins it once the lane clears."""
        k_center = float(self.get_parameter('k_center').value)
        lat_max = float(self.get_parameter('lat_max').value)
        spring = -k_center * e                 # toward centreline (e>0 = left -> push right)

        rep = 0.0
        tang = 0.0
        self.min_ahead = float('inf')
        if self.obs_body.shape[0]:
            oh = self.obs_body[:, :2]
            d = np.hypot(oh[:, 0], oh[:, 1])
            d0 = float(self.get_parameter('influence_d0').value)
            near = (d < d0) & (d > 1e-3)
            if near.any():
                ohn, dn = oh[near], d[near]
                # density-independent repulsion: nearest point per angular sector,
                # so a richly-textured face does not out-vote a sparse one.
                nsec = int(self.get_parameter('rep_sectors').value)
                k_rep = float(self.get_parameter('k_rep').value)
                bear = np.arctan2(ohn[:, 1], ohn[:, 0])
                sec = np.clip(((bear + math.pi) / (2 * math.pi) * nsec).astype(int),
                              0, nsec - 1)
                for sct in np.unique(sec):
                    m = sec == sct
                    i = int(np.argmin(dn[m]))
                    ds = float(dn[m][i])
                    diry = -ohn[m][i][1] / ds          # push away laterally
                    rep += diry * (k_rep * (1.0 / ds - 1.0 / d0) / (ds * ds))
                # nearest obstacle that is roughly dead ahead in the lane
                safe = float(self.get_parameter('safety_radius').value)
                ahead = (ohn[:, 0] > 0) & (np.abs(ohn[:, 1]) < safe)
                if ahead.any():
                    self.min_ahead = float(np.min(ohn[ahead, 0]))
                    # local-minimum escape: slide toward the side with more room
                    left = ohn[ohn[:, 1] > 0]
                    right = ohn[ohn[:, 1] < 0]
                    left_clr = np.min(np.hypot(left[:, 0], left[:, 1])) if left.shape[0] else d0
                    right_clr = np.min(np.hypot(right[:, 0], right[:, 1])) if right.shape[0] else d0
                    tang = (1.0 if left_clr > right_clr else -1.0) \
                        * float(self.get_parameter('tangential_gain').value)

        v = spring + rep + tang
        # Label AVOIDING only for a MEANINGFUL response (real lane obstacle), not the
        # sub-centimetre nudge the |y|=2 m init-garden decor exerts at the influence
        # edge -- otherwise the demo state line flickers "AVOIDING" with nothing there.
        # Display-only: the strafe command itself is unchanged.
        d0 = float(self.get_parameter('influence_d0').value)
        avoiding = (abs(rep) + abs(tang) > 0.04) or (self.min_ahead < d0)
        return float(np.clip(v, -lat_max, lat_max)), avoiding

    def _accel_limit(self, target):
        """Slew [forward, strafe] toward target under a max-acceleration cap."""
        target = np.asarray(target, dtype=float)
        a_max = float(self.get_parameter('max_accel').value)
        now = self.get_clock().now().nanoseconds / 1e9
        dt = (now - self._last_ctrl) if self._last_ctrl is not None else 1.0 / max(
            1e-3, float(self.get_parameter('control_rate').value))
        self._last_ctrl = now
        dt_eff = max(1e-3, min(dt, 0.5))
        if a_max <= 0.0:
            self._cmd = target
        else:
            dv = a_max * dt_eff
            self._cmd = self._cmd + np.clip(target - self._cmd, -dv, dv)
        return self._cmd

    def _status(self, state, remain, e, strafe):
        explored = min(100.0, 100.0 * self.dist / PERIM)
        m = self.min_ahead
        msg = String()
        msg.data = ('state=%s explored=%.0f%% dist=%.1f/%.0fm offset=%+.2fm '
                    'strafe=%+.2f nearest_ahead=%s obs_pts=%d'
                    % (state, explored, self.dist, self.goal, e, strafe,
                       ('%.2fm' % m) if math.isfinite(m) else 'none',
                       self.obs_body.shape[0]))
        self.pub_status.publish(msg)
        self._publish_banner(state, explored)
        self.get_logger().info(msg.data, throttle_duration_sec=1.0)


def main(argv=None):
    rclpy.init(args=argv)
    node = CircuitAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())  # stop on exit
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
