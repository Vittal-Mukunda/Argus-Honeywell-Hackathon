#!/usr/bin/env python3
"""ARGUS :: argus_dashboard_live.py — the live telemetry dashboard (4th demo window).

A clean, judge-facing Qt window that aggregates ALL the real-time data the autonomous
tunnel run produces, in one place, so a non-technical viewer sees what the drone is
doing / where it is / what it sees / what it decided — without reading a terminal.

Lightweight + robust by design: one rclpy node + PyQt5 (the same Qt that powers RViz/
rqt — already in the image), no web server, no browser, no extra deps. A QTimer drains
ROS callbacks and refreshes the labels; missing data degrades gracefully.

Real, live sources:
  /argus/nav/status          -> mission state, exploration %, distance, lateral
                                correction, nearest obstacle, obstacle-point count
  /argus/ground_truth/pose   -> position, heading, altitude, speed, tunnel section,
                                flight time, the minimap trail
  /argus/map/points          -> live map size (points fused)
  /argus/nav/detected_obstacles -> obstacles being detected right now (world frame)
                                -> minimap red marks, accumulated as "discovered"
  /argus/imu /argus/lidar/points /argus/cam0/image_raw /argus/vio/odom -> sensor health
Battery is MODELLED from flight time against the 40-min mission spec (labelled as est.);
no battery/BMS exists in the kinematic sim, so it is never presented as measured.

Layout (top to bottom): brand header + LIVE pill; the mission STATE banner (with a
plain-language subtitle); a HERO strip = exploration ring gauge + the three numbers
that tell the story (distance / speed / flight time); the MINIMAP (a to-scale top-down
schematic of the 202.8 m stadium — walls at centreline ±3 m, same geometry as
generate_tunnel_circuit.py — with the glowing flown trail, persistent red marks where
obstacles were discovered, and a pulsing drone marker); then detail cards and a sensor
health chip strip. Pose comes from ground truth — the same source the RViz digital
twin renders from (flight control & obstacle sensing stay live; see circuit_avoider).

Run inside the container on the demo display:
  docker exec -d argus bash -lc 'source install/setup.bash; python3 scripts/argus_dashboard_live.py'
"""
import math
import re
import signal
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       qos_profile_sensor_data)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, PointCloud2
from std_msgs.msg import String

from PyQt5 import QtCore, QtGui, QtWidgets

NOMINAL_MISSION_S = 2400.0           # 40-min mission spec, for the modelled battery

# ---- stadium geometry (mirrors generate_tunnel_circuit.py / circuit_avoider) ----
L = 70.0
R = 10.0
PERIM = 2 * L + 2 * math.pi * R      # 202.83 m
HALF_W = 3.0                         # tunnel half-width (walls at centreline ±3 m)


def centerline(s):
    """World (x, y, heading) at centreline arc-length s (CCW stadium)."""
    s = s % PERIM
    if s < L:                                   # straight A: (0,0)->(L,0)
        return s, 0.0, 0.0
    s -= L
    if s < math.pi * R:                         # right end-cap, CCW about (L, R)
        phi = -math.pi / 2 + s / R
        return L + R * math.cos(phi), R + R * math.sin(phi), phi + math.pi / 2
    s -= math.pi * R
    if s < L:                                   # straight B: (L,2R)->(0,2R)
        return L - s, 2 * R, math.pi
    s -= L                                      # left end-cap, CCW about (0, R)
    phi = math.pi / 2 + s / R
    return R * math.cos(phi), R + R * math.sin(phi), phi + math.pi / 2


def wall_ring(offset, step=1.0):
    """The tunnel wall as a world-frame polyline at signed lateral `offset`
    (+ = left of travel = inner ring; - = outer ring)."""
    pts = []
    n = int(PERIM / step) + 1
    for i in range(n + 1):
        x, y, th = centerline(i * step)
        pts.append((x - offset * math.sin(th), y + offset * math.cos(th)))
    return pts


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def parse_cloud_xy(msg: PointCloud2) -> np.ndarray:
    """(N,2) world XY from an XYZ float32 PointCloud2 (what circuit_avoider emits)."""
    n = msg.width * msg.height
    off = {f.name: f.offset for f in msg.fields}
    if n == 0 or not {'x', 'y'} <= off.keys():
        return np.empty((0, 2), np.float32)
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    out = np.empty((raw.shape[0], 2), np.float32)
    for i, ax in enumerate(('x', 'y')):
        out[:, i] = raw[:, off[ax]:off[ax] + 4].copy().view(np.float32).ravel()
    return out[np.isfinite(out).all(axis=1)]


class DashNode(Node):
    """Subscribes to everything the dashboard shows; holds the latest values."""

    def __init__(self):
        super().__init__('argus_dashboard')
        self.status = ''
        self.pose = None                 # (x, y, z, yaw)
        self.speed = 0.0                 # m/s, EMA of measured ground speed
        self.map_pts = 0
        self.det_pts = 0
        self.sim_t0 = None
        self.sim_now = None
        self.last_msg_wall = 0.0         # wall-clock of the last pose (LIVE pill)
        self.fresh = {}                  # sensor -> sim-time of last message
        self.trail = []                  # flown course for the minimap (thinned)
        self.obs_live = np.empty((0, 2), np.float32)   # detections right now
        self.obs_seen = set()            # all detections so far (0.4 m cells)
        self._prev = None                # (t, x, y, z) for the speed estimate

        self.create_subscription(String, '/argus/nav/status', self._on_status, 10)
        self.create_subscription(PoseStamped, '/argus/ground_truth/pose', self._on_pose, 10)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PointCloud2, '/argus/map/points', self._on_map, latched)
        self.create_subscription(PointCloud2, '/argus/nav/detected_obstacles',
                                 self._on_det, qos_profile_sensor_data)
        self.create_subscription(Imu, '/argus/imu',
                                 lambda m: self._mark('IMU', m.header), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/argus/lidar/points',
                                 lambda m: self._mark('LiDAR', m.header), qos_profile_sensor_data)
        self.create_subscription(Image, '/argus/cam0/image_raw',
                                 lambda m: self._mark('Camera', m.header), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/argus/vio/odom',
                                 lambda m: self._mark('VIO', m.header), 10)

    def _mark(self, name, header):
        self.fresh[name] = stamp_s(header.stamp)

    def _on_status(self, m):
        self.status = m.data

    def _on_pose(self, m):
        p, q = m.pose.position, m.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, p.z, yaw)
        t = stamp_s(m.header.stamp)
        if self.sim_t0 is None:
            self.sim_t0 = t
        self.sim_now = t
        self.last_msg_wall = time.time()
        self._mark('Pose', m.header)

        # measured ground speed (EMA over sim-time pose deltas)
        if self._prev is not None:
            dt = t - self._prev[0]
            if dt > 1e-3:
                d = math.hypot(p.x - self._prev[1], p.y - self._prev[2])
                self.speed += 0.25 * (d / dt - self.speed)
        self._prev = (t, p.x, p.y, p.z)

        # minimap trail, thinned to >= 0.5 m steps and bounded
        if not self.trail or math.hypot(p.x - self.trail[-1][0],
                                        p.y - self.trail[-1][1]) >= 0.5:
            self.trail.append((p.x, p.y))
            if len(self.trail) > 2000:
                del self.trail[:500]

    def _on_map(self, m):
        self.map_pts = m.width

    def _on_det(self, m):
        self.det_pts = m.width
        self._mark('Detect', m.header)
        xy = parse_cloud_xy(m)
        self.obs_live = xy
        if xy.shape[0] and len(self.obs_seen) < 6000:
            # remember every place an obstacle was detected (coarse 0.4 m cells)
            for cx, cy in np.round(xy / 0.4).astype(int)[:: max(1, xy.shape[0] // 64)]:
                self.obs_seen.add((int(cx), int(cy)))


def parse_status(s):
    def g(pat, cast, default):
        m = re.search(pat, s)
        return cast(m.group(1)) if m else default
    return {
        'state': g(r'state=(\S+)', str, '—'),
        'explored': g(r'explored=(\d+)%', int, 0),
        'dist': g(r'dist=([\d.]+)/', float, 0.0),
        'total': g(r'/(\d+)m', int, 207),
        'offset': g(r'offset=([+\-\d.]+)m', float, 0.0),
        'strafe': g(r'strafe=([+\-\d.]+)', float, 0.0),
        'nearest': g(r'nearest_ahead=(\S+)', str, 'none'),
        'obs_pts': g(r'obs_pts=(\d+)', int, 0),
    }


# state -> (judge-facing label, accent colour, plain-language subtitle)
STATE_UI = {
    'CRUISE':       ('EXPLORING TUNNEL', '#39d98a', 'Mapping ahead · lane clear'),
    'ARC':          ('NAVIGATING TURN', '#36c5f0', 'Following the tunnel curvature'),
    'AVOIDING':     ('OBSTACLE DETECTED', '#ffb020', 'Reactive avoidance · adjusting course'),
    'GOAL_REACHED': ('MISSION COMPLETE', '#39d98a', 'Full 202.8 m lap mapped'),
}


def section_of(x, y):
    if x > 70:
        return 'Right end-cap'
    if x < 0:
        return 'Left end-cap'
    return 'Straight A' if y < 10 else 'Straight B'


class RingGauge(QtWidgets.QWidget):
    """Circular exploration gauge: dark track, bright arc, % in the middle."""

    def __init__(self, caption='MAPPED'):
        super().__init__()
        self.value = 0.0
        self.caption = caption
        self.colour = QtGui.QColor('#39d98a')
        self.setFixedSize(118, 118)

    def paintEvent(self, _ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        rect = QtCore.QRectF(8, 8, self.width() - 16, self.height() - 16)
        p.setPen(QtGui.QPen(QtGui.QColor('#1d2530'), 9, cap=QtCore.Qt.RoundCap))
        p.drawArc(rect, 0, 360 * 16)
        if self.value > 0:
            p.setPen(QtGui.QPen(self.colour, 9, cap=QtCore.Qt.RoundCap))
            p.drawArc(rect, 90 * 16, -int(360 * 16 * min(self.value, 100.0) / 100.0))
        f = QtGui.QFont('DejaVu Sans Mono')
        f.setPixelSize(24)
        f.setBold(True)
        p.setFont(f)
        p.setPen(QtGui.QColor('#e6edf3'))
        p.drawText(self.rect().adjusted(0, -8, 0, -8), QtCore.Qt.AlignCenter,
                   '%d%%' % int(self.value))
        f2 = QtGui.QFont('DejaVu Sans')
        f2.setPixelSize(9)
        f2.setLetterSpacing(QtGui.QFont.AbsoluteSpacing, 1.5)
        p.setFont(f2)
        p.setPen(QtGui.QColor('#7d8590'))
        p.drawText(self.rect().adjusted(0, 26, 0, 26), QtCore.Qt.AlignCenter,
                   self.caption)
        p.end()


class MiniMap(QtWidgets.QWidget):
    """Top-down schematic of the stadium: walls, glowing flown trail, detected
    obstacles, and a pulsing drone marker. Pure QPainter — no extra dependencies."""

    # world bbox incl. walls: x in [-13, 83], y in [-3, 23]
    WX0, WY0, WW, WH = -13.0, -3.0, 96.0, 26.0

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setMinimumHeight(170)
        self._inner = wall_ring(+HALF_W)
        self._outer = wall_ring(-HALF_W)

    def _w2p(self, wx, wy, s, ox, oy, h):
        return QtCore.QPointF(ox + (wx - self.WX0) * s,
                              h - (oy + (wy - self.WY0) * s))

    def paintEvent(self, _ev):
        w, h = self.width(), self.height()
        pad = 8.0
        s = min((w - 2 * pad) / self.WW, (h - 2 * pad) / self.WH)
        ox = (w - self.WW * s) / 2
        oy = (h - self.WH * s) / 2
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)

        def poly(ring):
            return QtGui.QPolygonF([self._w2p(x, y, s, ox, oy, h) for x, y in ring])

        # tunnel floor: outer ring filled, inner ring cut out (odd-even fill)
        ring = QtGui.QPainterPath()
        ring.setFillRule(QtCore.Qt.OddEvenFill)
        ring.addPolygon(poly(self._outer))
        ring.addPolygon(poly(self._inner))
        p.fillPath(ring, QtGui.QColor('#16202e'))
        p.setPen(QtGui.QPen(QtGui.QColor('#3c4a5e'), 1.4))
        p.drawPolygon(poly(self._outer))
        p.drawPolygon(poly(self._inner))

        # centreline, dashed
        pen = QtGui.QPen(QtGui.QColor('#46536a'), 1.0, QtCore.Qt.DashLine)
        p.setPen(pen)
        p.drawPolygon(poly(wall_ring(0.0, step=2.0)))

        # start marker
        start = self._w2p(0.0, 0.0, s, ox, oy, h)
        p.setBrush(QtCore.Qt.NoBrush)
        p.setPen(QtGui.QPen(QtGui.QColor('#8b949e'), 1.2))
        p.drawEllipse(start, 3.2, 3.2)
        f = QtGui.QFont('DejaVu Sans')
        f.setPixelSize(8)
        f.setLetterSpacing(QtGui.QFont.AbsoluteSpacing, 1.0)
        p.setFont(f)
        p.setPen(QtGui.QColor('#6b7280'))
        p.drawText(QtCore.QPointF(start.x() - 16, start.y() + 14), 'START')

        n = self.node
        # flown trail: soft glow underlay + bright core (the actual course,
        # bowing around obstacles)
        if len(n.trail) >= 2:
            path = QtGui.QPolygonF(
                [self._w2p(x, y, s, ox, oy, h) for x, y in n.trail])
            glow = QtGui.QColor('#2bd97c')
            glow.setAlpha(46)
            p.setPen(QtGui.QPen(glow, 6.0, cap=QtCore.Qt.RoundCap,
                                join=QtCore.Qt.RoundJoin))
            p.drawPolyline(path)
            p.setPen(QtGui.QPen(QtGui.QColor('#2bd97c'), 2.0,
                                cap=QtCore.Qt.RoundCap, join=QtCore.Qt.RoundJoin))
            p.drawPolyline(path)

        # obstacles discovered so far (persist), then the live detections (bright
        # core + glow so "what it senses right now" pops)
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(QtGui.QColor('#b23737'))
        for cx, cy in n.obs_seen:
            p.drawEllipse(self._w2p(cx * 0.4, cy * 0.4, s, ox, oy, h), 2.0, 2.0)
        live_glow = QtGui.QColor('#ff4633')
        live_glow.setAlpha(55)
        for x, y in n.obs_live[:200]:
            c = self._w2p(float(x), float(y), s, ox, oy, h)
            p.setBrush(live_glow)
            p.drawEllipse(c, 6.0, 6.0)
            p.setBrush(QtGui.QColor('#ff4633'))
            p.drawEllipse(c, 3.0, 3.0)

        # the drone: glow + pulsing ring + heading tick + dot
        if n.pose:
            x, y, _z, yaw = n.pose
            c = self._w2p(x, y, s, ox, oy, h)
            halo = QtGui.QColor('#ffd23f')
            halo.setAlpha(36)
            p.setPen(QtCore.Qt.NoPen)
            p.setBrush(halo)
            p.drawEllipse(c, 9.0, 9.0)
            phase = (math.sin(time.time() * 2 * math.pi / 1.6) + 1) / 2
            ring_c = QtGui.QColor('#ffd23f')
            ring_c.setAlphaF(0.55 * (1 - phase) + 0.1)
            p.setBrush(QtCore.Qt.NoBrush)
            p.setPen(QtGui.QPen(ring_c, 1.6))
            p.drawEllipse(c, 6 + 5 * phase, 6 + 5 * phase)
            p.setPen(QtGui.QPen(QtGui.QColor('#ffd23f'), 2.0))
            p.drawLine(c, QtCore.QPointF(c.x() + 11 * math.cos(yaw),
                                         c.y() - 11 * math.sin(yaw)))
            p.setPen(QtCore.Qt.NoPen)
            p.setBrush(QtGui.QColor('#ffd23f'))
            p.drawEllipse(c, 3.5, 3.5)
        p.end()


class Card(QtWidgets.QFrame):
    """A titled panel with a few label rows."""

    def __init__(self, title):
        super().__init__()
        self.setObjectName('card')
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(16, 12, 16, 14)
        lay.setSpacing(6)
        t = QtWidgets.QLabel(title.upper())
        t.setObjectName('cardtitle')
        lay.addWidget(t)
        self.rows = {}
        self._lay = lay

    def add(self, key, label):
        row = QtWidgets.QHBoxLayout()
        lab = QtWidgets.QLabel(label)
        lab.setObjectName('rowlabel')
        val = QtWidgets.QLabel('—')
        val.setObjectName('rowval')
        val.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        row.addWidget(lab)
        row.addStretch(1)
        row.addWidget(val)
        self._lay.addLayout(row)
        self.rows[key] = val
        return val

    def add_widget(self, widget, stretch=1):
        self._lay.addWidget(widget, stretch)


class Dashboard(QtWidgets.QWidget):
    SENSORS = ('Camera', 'LiDAR', 'IMU', 'VIO')

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('ARGUS — Mission Dashboard')
        scr = QtWidgets.QApplication.primaryScreen().availableGeometry()
        self.resize(640, min(1060, scr.height() - 40))
        self._build()
        self._style()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(120)

    # -- layout -------------------------------------------------------------
    def _hero_stat(self, caption):
        """A big numeral with a small letter-spaced caption under it."""
        box = QtWidgets.QVBoxLayout()
        box.setSpacing(2)
        val = QtWidgets.QLabel('—')
        val.setObjectName('hero')
        val.setAlignment(QtCore.Qt.AlignCenter)
        cap = QtWidgets.QLabel(caption.upper())
        cap.setObjectName('herocap')
        cap.setAlignment(QtCore.Qt.AlignCenter)
        box.addStretch(1)
        box.addWidget(val)
        box.addWidget(cap)
        box.addStretch(1)
        return val, box

    def _build(self):
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(18, 16, 18, 16)
        root.setSpacing(12)

        # header: brand + LIVE pill
        head = QtWidgets.QHBoxLayout()
        brand_col = QtWidgets.QVBoxLayout()
        brand_col.setSpacing(0)
        h1 = QtWidgets.QLabel('ARGUS')
        h1.setObjectName('brand')
        h2 = QtWidgets.QLabel('Autonomous GPS-Denied Tunnel Inspection')
        h2.setObjectName('subtitle')
        brand_col.addWidget(h1)
        brand_col.addWidget(h2)
        head.addLayout(brand_col)
        head.addStretch(1)
        self.live_pill = QtWidgets.QLabel('● LIVE')
        self.live_pill.setObjectName('livepill')
        self.live_pill.setAlignment(QtCore.Qt.AlignCenter)
        head.addWidget(self.live_pill, 0, QtCore.Qt.AlignTop)
        root.addLayout(head)

        # state banner: big state + plain-language subtitle
        self.state_box = QtWidgets.QFrame()
        self.state_box.setObjectName('state')
        sb = QtWidgets.QVBoxLayout(self.state_box)
        sb.setContentsMargins(14, 10, 14, 10)
        sb.setSpacing(2)
        self.state_lbl = QtWidgets.QLabel('CONNECTING…')
        self.state_lbl.setObjectName('statemain')
        self.state_lbl.setAlignment(QtCore.Qt.AlignCenter)
        self.state_sub = QtWidgets.QLabel('waiting for mission telemetry')
        self.state_sub.setObjectName('statesub')
        self.state_sub.setAlignment(QtCore.Qt.AlignCenter)
        sb.addWidget(self.state_lbl)
        sb.addWidget(self.state_sub)
        root.addWidget(self.state_box)

        # hero strip: ring gauge + the three numbers that tell the story
        hero = QtWidgets.QFrame()
        hero.setObjectName('card')
        hl = QtWidgets.QHBoxLayout(hero)
        hl.setContentsMargins(18, 10, 18, 10)
        self.ring = RingGauge()
        hl.addWidget(self.ring)
        self.hero_dist, b1 = self._hero_stat('distance')
        self.hero_speed, b2 = self._hero_stat('speed')
        self.hero_time, b3 = self._hero_stat('flight time')
        for b in (b1, b2, b3):
            hl.addStretch(1)
            hl.addLayout(b)
        hl.addStretch(1)
        root.addWidget(hero)

        # minimap
        c_map_view = Card('Live tunnel map — top view')
        self.mini = MiniMap(self.node)
        c_map_view.add_widget(self.mini)
        root.addWidget(c_map_view, 1)

        # detail cards
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(12)
        root.addLayout(grid)

        self.c_dec = Card('Perception & Decision')
        self.c_dec.add('nearest', 'Nearest obstacle')
        self.c_dec.add('det', 'Detections (live)')
        self.c_dec.add('corr', 'Course correction')

        self.c_pos = Card('Position')
        self.c_pos.add('xy', 'X / Y')
        self.c_pos.add('alt', 'Altitude')
        self.c_pos.add('hdg', 'Heading')
        self.c_pos.add('section', 'Section')

        self.c_map = Card('Live 3D Mapping')
        self.c_map.add('pts', 'Map points fused')
        self.c_map.add('cover', 'Coverage')
        self.c_map.add('res', 'Resolution')
        self.c_map.rows['res'].setText('0.20 m voxels')

        self.c_power = Card('Telemetry')
        self.c_power.add('batt', 'Battery (est.)')
        self.c_power.add('world', 'Course')
        self.c_power.add('rtf', 'Mode')
        self.c_power.rows['world'].setText('tunnel_circuit · 202.8 m')
        self.c_power.rows['rtf'].setText('Simulation · RTF 0.5')

        grid.addWidget(self.c_dec, 0, 0)
        grid.addWidget(self.c_pos, 0, 1)
        grid.addWidget(self.c_map, 1, 0)
        grid.addWidget(self.c_power, 1, 1)

        # sensor health chip strip
        chips = QtWidgets.QFrame()
        chips.setObjectName('card')
        cl = QtWidgets.QHBoxLayout(chips)
        cl.setContentsMargins(14, 9, 14, 9)
        title = QtWidgets.QLabel('SENSORS')
        title.setObjectName('cardtitle')
        title.setStyleSheet('#cardtitle{border:none;padding:0;}')
        cl.addWidget(title)
        cl.addStretch(1)
        self.chips = {}
        for s in self.SENSORS:
            chip = QtWidgets.QLabel('○ %s' % s.upper())
            chip.setObjectName('chip')
            cl.addWidget(chip)
            self.chips[s] = chip
        root.addWidget(chips)

        self.foot = QtWidgets.QLabel(
            'Stereo + LiDAR + IMU · live log-odds voxel fusion · reactive obstacle avoidance')
        self.foot.setObjectName('foot')
        self.foot.setAlignment(QtCore.Qt.AlignCenter)
        root.addWidget(self.foot)

    # -- refresh ------------------------------------------------------------
    def _tick(self):
        # drain ROS callbacks, then refresh the UI from the node's latest values
        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.0)
        n = self.node
        d = parse_status(n.status) if n.status else None
        now = time.time()

        if d:
            label, colour, sub = STATE_UI.get(
                d['state'], (d['state'], '#9aa4b2', ''))
            self.state_lbl.setText(label)
            self.state_sub.setText(sub)
            # accent border + a soft tint; the tint PULSES while avoiding so the
            # judge's eye is pulled to the state change the moment it happens.
            tint = 0.10
            if d['state'] == 'AVOIDING':
                tint = 0.10 + 0.14 * (math.sin(now * 2 * math.pi / 0.9) + 1) / 2
            qc = QtGui.QColor(colour)
            self.state_box.setStyleSheet(
                '#state{border-color:%s;background:rgba(%d,%d,%d,%.2f);}'
                '#statemain{color:%s;} #statesub{color:rgba(%d,%d,%d,0.72);}'
                % (colour, qc.red(), qc.green(), qc.blue(), tint,
                   colour, qc.red(), qc.green(), qc.blue()))
            self.ring.value = float(d['explored'])
            self.ring.colour = QtGui.QColor(colour)
            self.ring.update()
            self.hero_dist.setText('%.1f m' % d['dist'])
            self.c_dec.rows['nearest'].setText(
                d['nearest'] if d['nearest'] != 'none' else 'clear')
            self.c_dec.rows['det'].setText('%d pts' % n.det_pts)
            corr = abs(d['offset'])
            self.c_dec.rows['corr'].setText(
                ('%+.2f m (%s)' % (d['offset'], 'left' if d['offset'] > 0 else 'right'))
                if corr > 0.05 else 'on route')
            self.c_map.rows['cover'].setText('%d%% of tunnel' % d['explored'])

        if n.pose:
            x, y, z, yaw = n.pose
            self.c_pos.rows['xy'].setText('%.1f / %.1f m' % (x, y))
            self.c_pos.rows['alt'].setText('%.2f m' % z)
            self.c_pos.rows['hdg'].setText('%d°' % int(math.degrees(yaw) % 360))
            self.c_pos.rows['section'].setText(section_of(x, y))
        self.c_map.rows['pts'].setText('{:,}'.format(n.map_pts))
        self.hero_speed.setText('%.2f m/s' % n.speed)

        # flight time + modelled battery (sim time)
        if n.sim_t0 is not None and n.sim_now is not None:
            ft = max(0.0, n.sim_now - n.sim_t0)
            self.hero_time.setText('%d:%02d' % (int(ft) // 60, int(ft) % 60))
            batt = max(0.0, 100.0 - 100.0 * ft / NOMINAL_MISSION_S)
            bc = '#39d98a' if batt > 50 else ('#ffb020' if batt > 20 else '#f85149')
            self.c_power.rows['batt'].setText('%.0f%%' % batt)
            self.c_power.rows['batt'].setStyleSheet('#rowval{color:%s;}' % bc)

        # LIVE pill: green + blinking dot while poses stream, grey when stale
        if now - n.last_msg_wall < 1.0:
            dot = '●' if int(now * 2) % 2 == 0 else '○'
            self.live_pill.setText('%s LIVE' % dot)
            self.live_pill.setStyleSheet('#livepill{color:#39d98a;border-color:#39d98a;}')
        else:
            self.live_pill.setText('○ STANDBY')
            self.live_pill.setStyleSheet('#livepill{color:#6b7280;border-color:#374151;}')

        # sensor freshness vs the latest sim time
        ref = n.sim_now
        for s in self.SENSORS:
            chip = self.chips[s]
            t = n.fresh.get(s)
            if t is not None and ref is not None and (ref - t) < 1.5:
                chip.setText('● %s' % s.upper())
                chip.setStyleSheet('#chip{color:#39d98a;border-color:#1d4434;}')
            else:
                chip.setText('○ %s' % s.upper())
                chip.setStyleSheet('#chip{color:#6b7280;border-color:#2a3340;}')

        self.mini.update()

    def _style(self):
        self.setStyleSheet('''
          QWidget{ background:#070b11; color:#e6edf3;
            font-family:"DejaVu Sans","Segoe UI",sans-serif; }
          #brand{ font-size:32px; font-weight:800; letter-spacing:5px; color:#ffffff; }
          #subtitle{ font-size:12px; color:#8b949e; }
          #livepill{ font-size:12px; font-weight:700; letter-spacing:1px; color:#39d98a;
            border:1px solid #39d98a; border-radius:11px; padding:4px 12px; }
          #state{ border:2px solid #39d98a; border-radius:14px; }
          #statemain{ font-size:26px; font-weight:800; letter-spacing:2px;
            color:#39d98a; background:transparent; }
          #statesub{ font-size:12px; letter-spacing:0.5px;
            color:rgba(57,217,138,0.72); background:transparent; }
          #card{ background:#0e131b; border:1px solid #1d2530; border-radius:14px; }
          #cardtitle{ font-size:11px; font-weight:700; letter-spacing:2px; color:#7d8590;
            padding-bottom:5px; border-bottom:1px solid #1d2530; background:transparent; }
          #hero{ font-size:25px; font-weight:700; color:#e6edf3;
            font-family:"DejaVu Sans Mono",monospace; background:transparent; }
          #herocap{ font-size:9px; font-weight:700; letter-spacing:2px; color:#7d8590;
            background:transparent; }
          #chip{ font-size:11px; font-weight:700; letter-spacing:1px; color:#6b7280;
            border:1px solid #2a3340; border-radius:10px; padding:4px 10px;
            background:transparent; }
          #rowlabel{ font-size:13px; color:#8b949e; background:transparent; }
          #rowval{ font-size:14px; font-weight:700; color:#e6edf3;
            font-family:"DejaVu Sans Mono",monospace; background:transparent; }
          #foot{ font-size:11px; color:#6b7280; margin-top:2px; }
        ''')


def main():
    rclpy.init()
    node = DashNode()
    app = QtWidgets.QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)   # Ctrl-C closes the window
    dash = Dashboard(node)
    dash.show()
    try:
        app.exec_()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
