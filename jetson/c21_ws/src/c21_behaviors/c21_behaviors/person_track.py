"""Vision follow logic (no ROS): person boxes + depth -> positions, appearance, target tracking.

Frames: base = base_footprint (x ahead, y left), odom = world-fixed (the car's own motion is
taken out by tracking in odom), color = camera_color_optical_frame (x right, y down, z ahead).
"""
import math

import cv2
import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lim):
    return max(-lim, min(lim, v))


def deproject(depth_m, k, stride):
    """Depth image (metres) -> Nx3 optical-frame points; zeros dropped."""
    d = depth_m[::stride, ::stride]
    v, u = np.mgrid[0:depth_m.shape[0]:stride, 0:depth_m.shape[1]:stride]
    ok = d > 0.0
    z = d[ok]
    return np.stack([(u[ok] - k[0, 2]) * z / k[0, 0], (v[ok] - k[1, 2]) * z / k[1, 1], z], axis=1)


def project(p_cam, k):
    """Nx3 optical-frame points -> Nx2 pixels (points behind the camera get NaN)."""
    z = p_cam[:, 2]
    uv = np.full((len(p_cam), 2), np.nan)
    f = z > 0.05
    uv[f, 0] = k[0, 0] * p_cam[f, 0] / z[f] + k[0, 2]
    uv[f, 1] = k[1, 1] * p_cam[f, 1] / z[f] + k[1, 2]
    return uv


def person_position(box, uv, pb, shrink=0.25, min_pts=8, depth_band=0.3):
    """Base-frame (x, y) of a person box from depth points projected into the colour image.

    Only the middle of the box is used (arms / background at the edges), points on the floor are
    dropped, and of what is left the nearest layer is taken (background seen between the legs is
    farther). Returns None when too few depth points land on the person."""
    x1, y1, x2, y2 = box[:4]
    m = (x2 - x1) * shrink
    inside = (uv[:, 0] > x1 + m) & (uv[:, 0] < x2 - m) & (uv[:, 1] > y1) & (uv[:, 1] < y2) & (pb[:, 2] > 0.06)
    p = pb[inside]
    if len(p) < min_pts:
        return None
    r = np.hypot(p[:, 0], p[:, 1])
    near = np.percentile(r, 20)
    q = p[r < near + depth_band]
    if len(q) < min_pts // 2:
        return None
    return np.array([float(np.median(q[:, 0])), float(np.median(q[:, 1]))])


def box_bearing(box, k, r_cam_base):
    """Base-frame bearing of the box centre (works without depth)."""
    u, v = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
    ray = r_cam_base @ np.array([(u - k[0, 2]) / k[0, 0], (v - k[1, 2]) / k[1, 1], 1.0])
    return math.atan2(ray[1], ray[0])


def appearance(rgb, box):
    """Hue/saturation histogram of the middle of a person box (clothes colour), L1-normalised."""
    h, w = rgb.shape[:2]
    x1, y1, x2, y2 = box[:4]
    bw, bh = x2 - x1, y2 - y1
    a, b = int(max(0, x1 + 0.2 * bw)), int(min(w, x2 - 0.2 * bw))
    c, d = int(max(0, y1 + 0.1 * bh)), int(min(h, y2 - 0.1 * bh))
    if b - a < 4 or d - c < 4:
        return None
    hsv = cv2.cvtColor(np.ascontiguousarray(rgb[c:d, a:b]), cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, (0, 0, 30), (180, 255, 255))          # ignore near-black shadows
    hist = cv2.calcHist([hsv], [0, 1], mask, [18, 8], [0, 180, 0, 256])
    s = float(hist.sum())
    if s < 20:
        return None
    return (hist / s).astype(np.float32)


def hist_distance(a, b):
    """Bhattacharyya distance 0 (same) .. 1 (different); 1 when either is missing."""
    if a is None or b is None:
        return 1.0
    return float(cv2.compareHist(a, b, cv2.HISTCMP_BHATTACHARYYA))


class PersonTracker:
    """Lock on to the person standing right in front, then keep following that person.

    Each detection is a dict: pos (odom xy or None without depth), bearing (base rad), range
    (base m or None), hist. Matching uses where the target should be (it walks at most `walk` m/s)
    and what it looks like; a lost target is only taken back when the clothes colour matches.

    The camera says who the target is; the lidar (update_lidar) keeps a live track going between
    and beyond camera frames (out of the picture at the side, or too close for the camera). Only
    the camera can start or take back a track. If the target should be in the picture but the
    camera has not confirmed it for doubt_s, the lidar track is dropped (it may have slid onto a
    chair leg) until the camera finds the person again."""

    def __init__(self, p):
        self.p = p
        self.target = None      # odom xy
        self.range = None       # last measured distance (base)
        self.hist = None
        self.seen = -1e9
        self.locked_at = None
        self.pending, self.pending_n = None, 0
        self.pending_det = None     # this frame's lock-on candidate (for display)
        self.vis_seen = -1e9        # last camera match
        self.pos_seen = -1e9        # last measured position (camera depth or lidar)
        self.lidar_ok = True        # lidar may continue the track (cleared by a doubt)

    @property
    def locked(self):
        return self.hist is not None

    def lost_for(self, now):
        return now - self.seen

    def _acquire(self, dets, now):
        p = self.p
        ahead = [d for d in dets if d['pos'] is not None and d['hist'] is not None
                 and d['range'] < p['acq_range'] and abs(d['bearing']) < math.radians(p['acq_fov'])]
        self.pending_det = None
        if not ahead:
            self.pending, self.pending_n = None, 0
            return None
        d = min(ahead, key=lambda e: abs(e['bearing']))
        if self.pending is not None and math.hypot(*(d['pos'] - self.pending)) < 0.4:
            self.pending_n += 1
        else:
            self.pending_n = 1
        self.pending = d['pos']
        if self.pending_n < p['acq_frames']:
            self.pending_det = d
            return None
        self.hist, self.locked_at = d['hist'], now
        self.pending, self.pending_n = None, 0
        return d

    def update(self, dets, now, robot_xy, robot_yaw):
        """Pick this frame's target detection (or None) and update the track."""
        p = self.p
        if self.locked and self.lost_for(now) > p['forget_s']:
            self.target, self.hist, self.range = None, None, None      # give up: lock on afresh
        if not self.locked:
            d = self._acquire(dets, now)
            if d is not None:
                self._take(d, now, robot_xy, robot_yaw, unique=True)
            return d
        lost = self.lost_for(now) > p['lost_s']
        gate = min(p['gate'] + p['walk'] * max(0.0, self.lost_for(now)), p['gate_max'])
        pred_b = None
        if self.target is not None:
            dx, dy = self.target - robot_xy
            pred_b = wrap(math.atan2(dy, dx) - robot_yaw)
        cands = []
        for d in dets:
            hd = hist_distance(d['hist'], self.hist)
            if lost:
                if d['pos'] is not None and hd < p['reacq_hist']:
                    cands.append((hd, d))
                continue
            if hd > p['max_hist']:
                continue
            if d['pos'] is not None and self.target is not None:
                dist = math.hypot(*(d['pos'] - self.target))
                if dist < gate:
                    cands.append((dist / gate + p['w_hist'] * hd, d))
            elif pred_b is not None:
                da = abs(wrap(d['bearing'] - pred_b))
                if da < math.radians(12):
                    cands.append((0.5 + da / math.radians(12) + p['w_hist'] * hd, d))
        if not cands:
            return None
        cands.sort(key=lambda c: c[0])
        d = cands[0][1]
        self._take(d, now, robot_xy, robot_yaw, unique=len(cands) == 1,
                   hd=hist_distance(d['hist'], self.hist))
        return d

    def _take(self, d, now, robot_xy, robot_yaw, unique, hd=0.0):
        self.vis_seen, self.lidar_ok = now, True
        if d['pos'] is not None:
            self.target, self.range = d['pos'].copy(), d['range']
            self.pos_seen = now
        elif self.range is not None:      # bearing only: keep the last distance
            a = robot_yaw + d['bearing']
            self.target = robot_xy + self.range * np.array([math.cos(a), math.sin(a)])
        self.seen = now
        # learn the look slowly, and only from clean matches (nobody else close by)
        if unique and d['hist'] is not None and hd < self.p['learn_hist']:
            a = self.p['hist_rate']
            self.hist = ((1 - a) * self.hist + a * d['hist']).astype(np.float32)


    def update_lidar(self, cents, now, robot_xy):
        """Continue a live track with lidar person-blob centres (odom xy). Returns the centre used."""
        p = self.p
        if not self.locked or self.target is None or not self.lidar_ok:
            return None
        dt = now - self.seen
        if dt > p['lidar_lost_s']:
            return None
        gate = min(p['lidar_gate'] + p['walk'] * max(0.0, dt), p['gate_max'])
        best, bd = None, gate
        for c in cents:
            d = math.hypot(*(c - self.target))
            if d < bd:
                best, bd = c, d
        if best is None:
            return None
        self.target = np.array(best, dtype=float)
        self.range = float(math.hypot(*(self.target - robot_xy)))
        self.seen = self.pos_seen = now
        return best

    def check_view(self, now, in_view):
        """Drop a lidar-only track the camera should be confirming but is not. True on drop."""
        if self.locked and self.lidar_ok and in_view and now - self.vis_seen > self.p['doubt_s']                 and self.seen > self.vis_seen:
            self.lidar_ok = False
            self.seen = self.vis_seen
            return True
        return False


def follow_cmd(tgt_base, range_ok, p):
    """(vx, vy, wz) towards a target at base-frame xy, keeping p['keep'] metres."""
    dist, ang = math.hypot(*tgt_base), math.atan2(tgt_base[1], tgt_base[0])
    wz = clamp(p['kp_ang'] * ang, p['max_w']) if abs(ang) > math.radians(4) else 0.0
    vx = 0.0
    err = dist - p['keep']
    if range_ok and abs(err) > p['deadband'] and abs(ang) < math.radians(40):
        vx = clamp(p['kp_lin'] * err, p['speed'])
        vx = max(vx, -p['back_speed'])
    return vx, 0.0, wz


def corridor(pts, heading, hl, hw):
    """Free length, from the car's edge, of a hw-half-wide corridor pointing at `heading` (base
    rad). The car is square (hl ~ hw), so the same edge distance works sideways too. Needs two
    points to count as blocked (single stray returns are ignored)."""
    c, s = math.cos(heading), math.sin(heading)
    x, y = c * pts[:, 0] + s * pts[:, 1], -s * pts[:, 0] + c * pts[:, 1]
    xs = x[(x > hl) & (np.abs(y) < hw)]
    if len(xs) < 2:
        return math.inf
    return float(np.partition(xs, 1)[1] - hl)


AV_HEADINGS = np.radians(np.arange(-90, 91, 10))


def avoid_cmd(tgt_base, range_ok, pts, side, p):
    """follow_cmd, but when something stands between the car and the target, slide past it.

    The car keeps facing the person (the camera is fixed forward) and moves (mecanum) along the
    free direction closest to the person; `side` (+1 left / -1 right / 0) keeps it on the side it
    started detouring to. pts: obstacle points (base xy, lidar + depth). Returns
    (cmd, side, state) with state None (plain follow), 'detour' or 'blocked'."""
    vx, vy, wz = follow_cmd(tgt_base, range_ok, p)
    if pts is None or not len(pts):
        return (vx, vy, wz), 0, None
    dist, ang = math.hypot(*tgt_base), math.atan2(tgt_base[1], tgt_base[0])
    pts = pts[np.hypot(pts[:, 0] - tgt_base[0], pts[:, 1] - tgt_base[1]) > p['av_person_r']]   # not the person
    need = min(p['av_clear'] + (0.15 if side else 0.0), dist - p['keep'])
    if vx <= 0.0 or need <= 0.05:
        return (vx, vy, wz), 0, None
    w = p['hw'] + p['av_margin']
    if corridor(pts, 0.0, p['hl'], w) >= need:
        return (vx, vy, wz), 0, None
    best, cost = None, math.inf
    for h in AV_HEADINGS:
        if corridor(pts, h, p['hl'], w) < p['av_clear']:
            continue
        c = abs(wrap(h - ang)) + (math.radians(30) if side and h * side < 0 else 0.0)
        if c < cost:
            best, cost = float(h), c
    if best is None:
        return (0.0, 0.0, wz), side, 'blocked'
    v = p['av_speed']
    return (v * math.cos(best), v * math.sin(best), wz), (side if best == 0 else (1 if best > 0 else -1)), 'detour'
