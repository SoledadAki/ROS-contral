"""Lidar behaviours (one per process, picked with the `behavior` parameter):

    avoid    roam forward, turn in place towards the freer side when the way ahead is blocked
    follow   keep facing the nearest thing in front and hold a fixed distance to it
    guard    stay put, remember the surroundings, turn to face (and beep at) anything new

Publishes cmd_vel_behavior, so the arbiter's limits, lidar guard and e-stop still apply and the
APP joystick overrides it. The rear is lidar-blind, so nothing here drives backwards on purpose
(follow may creep back slowly; the arbiter caps blind directions anyway).
Status: c21/behavior_state (String, JSON {behavior, state, text, target}).
c21/behavior_hold (Bool, from the gateway while the APP link is down): stand still but keep the
learned scene / target, so a short WiFi drop does not restart the behaviour. A hold lasts 1 s
unless refreshed, so it can never get stuck on.
"""
import json
import math
import time

import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String, UInt8


def clamp(v, lim):
    return max(-lim, min(lim, v))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def scan_points(ranges, angle_min, angle_inc, scan_x, scan_yaw, self_range, hl, hw, with_idx=False):
    """Valid returns as an (N, 2) base-frame array, minus the robot's own body (+ their ray indices)."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(len(r)) + scan_yaw
    ok = np.isfinite(r) & (r >= self_range)
    x = scan_x + r[ok] * np.cos(a[ok])
    y = r[ok] * np.sin(a[ok])
    body = (np.abs(x) <= hl + 0.01) & (np.abs(y) <= hw + 0.01)
    pts = np.stack([x[~body], y[~body]], axis=1)
    if with_idx:
        return pts, np.flatnonzero(ok)[~body]
    return pts


def segments(pts, idx, n_rays, gap=0.12, max_skip=4):
    """Split points (in ray order) into unbroken runs; list of index arrays into pts.
    The ray order wraps around straight ahead of the car (lidar mounted backwards), so the last
    and first runs are joined when they touch."""
    if not len(pts):
        return []
    jump = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    brk = np.flatnonzero((jump > gap) | (np.diff(idx) > max_skip)) + 1
    segs = np.split(np.arange(len(pts)), brk)
    if len(segs) > 1:
        i, j = segs[-1][-1], segs[0][0]
        if idx[j] + n_rays - idx[i] <= max_skip and math.hypot(*(pts[i] - pts[j])) <= gap:
            segs[0] = np.concatenate([segs.pop(), segs[0]])
    return segs


def blobs(pts, idx, n_rays, max_width=0.6, min_pts=3, merge=0.35):
    """Person-sized groups of points (walls / big furniture dropped; two legs merged into one).
    Returns a list of index arrays into pts."""
    small = [s for s in segments(pts, idx, n_rays)
             if len(s) >= min_pts and math.hypot(*np.ptp(pts[s], axis=0)) <= max_width]
    cents = [pts[s].mean(axis=0) for s in small]
    out, used = [], [False] * len(small)
    for i in range(len(small)):
        if used[i]:
            continue
        grp, used[i] = [small[i]], True
        for j in range(i + 1, len(small)):
            if not used[j] and math.hypot(*(cents[i] - cents[j])) < merge:
                grp.append(small[j])
                used[j] = True
        out.append(np.concatenate(grp))
    return out


def rotate_back(p, dyaw):
    """Where a world-still point p (base frame) appears after the robot turned by dyaw."""
    c, s = math.cos(dyaw), math.sin(dyaw)
    return np.array([c * p[0] + s * p[1], -s * p[0] + c * p[1]])


def pick(pts, groups, near, gate):
    """Centre of the group closest to `near` (within gate), or None."""
    best, bd = None, gate
    for g in groups:
        c = pts[g].mean(axis=0)
        d = math.hypot(*(c - near))
        if d < bd:
            best, bd = c, d
    return best


def bearing_deg(c):
    return math.degrees(math.atan2(c[1], c[0]))


class Avoid:
    """Drive ahead. When the car-wide corridor in front gets short, pick the most open heading
    (the corridor swept over -90..90 deg; the rear is lidar-blind) and rotate towards it.
    Too close to anything to rotate in place: slide sideways (mecanum) or creep back until there is
    room. Things just beside the path push the car sideways a little so a front corner does not
    clip them. Single stray returns are ignored (a corridor needs two points to be blocked)."""

    HEADINGS = np.radians(np.arange(-90, 91, 10))

    def __init__(self, p):
        self.p = p
        self.state, self.dir = 'go', 1.0
        self.turned = 0.0                # seconds spent rotating in the current turn
        self.esc_t = 0.0                 # seconds spent escaping
        self.last = None

    def corridor(self, pts, heading=0.0):
        """Free length, from the front bumper, of a car-wide corridor pointing at `heading`."""
        p = self.p
        if heading:
            c, s = math.cos(heading), math.sin(heading)
            x, y = c * pts[:, 0] + s * pts[:, 1], -s * pts[:, 0] + c * pts[:, 1]
        else:
            x, y = pts[:, 0], pts[:, 1]
        xs = x[(x > p['hl']) & (np.abs(y) < p['hw'] + p['side_margin'])]
        if len(xs) < 2:
            return math.inf
        return float(np.partition(xs, 1)[1] - p['hl'])

    def best_heading(self, pts):
        free = np.array([min(self.corridor(pts, h), 2.0) for h in self.HEADINGS])
        score = free - 0.002 * np.degrees(np.abs(self.HEADINGS))     # prefer small turns on ties
        i = int(np.argmax(score))
        return float(self.HEADINGS[i]), float(free[i])

    def flank_push(self, pts):
        """Sideways speed away from things just outside the corridor, beside the front half."""
        p = self.p
        x, y = pts[:, 0], pts[:, 1]
        cw = p['hw'] + p['side_margin']
        m = (x > p['hl'] - 0.10) & (x < p['hl'] + 0.40) & (np.abs(y) >= cw) & (np.abs(y) < cw + p['flank'])
        left, right = (m & (y > 0)).sum() >= 2, (m & (y < 0)).sum() >= 2
        if left == right:
            return 0.0
        return -p['side_speed'] if left else p['side_speed']

    def escape(self, pts, d):
        """Move away from the nearest return: sideways if it is at the side and the other side is
        clear, otherwise slowly backwards (the arbiter caps blind directions)."""
        p = self.p
        q = pts[int(np.argmin(d))]
        if abs(q[1]) > p['hw'] - 0.02:                # beside the car: slide away from it
            away = -1.0 if q[1] > 0 else 1.0
            x, y = pts[:, 0], pts[:, 1]
            blocked = ((np.abs(x) < p['hl'] + 0.05) & (y * away > 0) & (np.abs(y) < p['hw'] + 0.10)).any()
            if not blocked:
                return (0.0, away * p['esc_speed'], 0.0), f'太靠近{"左" if q[1] > 0 else "右"}侧障碍，横移让出空间'
        if q[0] > 0:
            return (-p['esc_speed'], 0.0, 0.0), '太靠近前方障碍，慢慢后退'
        return (0.0, 0.0, 0.0), '太靠近障碍，没有退路，已停车'

    def step(self, pts, now):
        p = self.p
        dt = 0.0 if self.last is None else min(max(now - self.last, 0.0), 0.5)
        self.last = now
        if not len(pts):
            pts = np.array([[9.0, 0.0], [9.0, 0.0]])
        front = self.corridor(pts)
        d = np.hypot(pts[:, 0], pts[:, 1])
        can_turn = float(d.min()) > p['turn_radius']
        if self.state == 'go' and front < p['threshold']:
            h, _ = self.best_heading(pts)
            self.state, self.turned, self.esc_t = 'turn', 0.0, 0.0
            self.dir = 1.0 if h > 0 else -1.0 if h < 0 else self.dir
        if self.state == 'turn' and front > p['threshold'] + 0.15:
            self.state = 'go'
        if self.state == 'turn':
            if not can_turn:
                self.esc_t += dt
                if self.esc_t > p['esc_s']:
                    return (0.0, 0.0, 0.0), 'stuck', '离障碍太近又退不出来，已停车（停止再启动可重试）'
                cmd, text = self.escape(pts, d)
                return cmd, 'escape', text
            self.esc_t = 0.0
            if self.turned * p['turn_w'] > 2.2 * math.pi:
                return (0.0, 0.0, 0.0), 'stuck', '转了一圈都没路，已停车（停止再启动可重试）'
            self.turned += dt
            return (0.0, 0.0, self.dir * p['turn_w']), 'turn', \
                f'前方 {front:.2f} m 有障碍，向{"左" if self.dir > 0 else "右"}转'
        v = p['speed'] * min(1.0, max(0.4, (front - p['threshold']) / 0.5))
        return (v, self.flank_push(pts), 0.0), 'go', '前进' + (f'（前方 {front:.1f} m）' if front < 3 else '')


class Follow:
    """Track a person-sized blob that started in front of the car and keep `keep` metres from it.
    Walls are never targets (too wide), and the track follows the blob closest to where the target
    should be (corrected for the car's own turn), so it does not slide off along a wall."""

    def __init__(self, p):
        self.p = p
        self.target, self.seen = None, 0.0

    def step(self, pts, idx, n_rays, now, dyaw=0.0):
        p = self.p
        groups = []
        for g in blobs(pts, idx, n_rays):
            c = pts[g].mean(axis=0)
            if math.hypot(*c) < p['range'] and abs(bearing_deg(c)) < p['fov']:
                groups.append(g)
        tgt = None
        if self.target is not None:
            self.target = rotate_back(self.target, dyaw)
            tgt = pick(pts, groups, self.target, p['gate'])
        if tgt is None and (self.target is None or now - self.seen > p['lost_s']):
            ahead = [g for g in groups if math.hypot(*pts[g].mean(axis=0)) < p['acq_range']
                     and abs(bearing_deg(pts[g].mean(axis=0))) < p['acq_fov']]
            tgt = pick(pts, ahead, np.zeros(2), 9.0)    # (re)acquire: the nearest thing right in front
        if tgt is None:
            if self.target is not None and now - self.seen > p['lost_s']:
                self.target = None
            if self.target is None:
                return (0.0, 0.0, 0.0), 'lost', f'等待目标：请站到车头正前方 {p["acq_range"]:.0f} m 以内'
            return (0.0, 0.0, 0.0), 'hold', '目标短暂丢失'
        self.target, self.seen = tgt, now
        dist, ang = math.hypot(*tgt), math.atan2(tgt[1], tgt[0])
        wz = clamp(p['kp_ang'] * ang, p['max_w']) if abs(ang) > math.radians(4) else 0.0
        err = dist - p['keep']
        vx = 0.0
        if abs(err) > 0.05 and abs(ang) < math.radians(45):
            vx = clamp(p['kp_lin'] * err, p['speed'])
            vx = max(vx, -p['back_speed'])
        return (vx, 0.0, wz), 'track', f'跟随中：目标 {dist:.2f} m，{math.degrees(ang):+.0f}°'


class Guard:
    """Remember the scene (world-fixed bearings via odom yaw); turn to face anything new and closer.

    New things are only looked for while the car stands still (a turning lidar smears the scan);
    while turning it just keeps track of the target it already has. The remembered scene heals:
    a bin that reads clearly farther for heal_s (someone stood there while it was learning) takes
    the farther reading. Bins first seen after a turn are learned unless the target is there."""

    BIN = math.radians(2)

    def __init__(self, p):
        self.p = p
        n = int(round(2 * math.pi / self.BIN))
        self.base = np.full(n, np.nan)
        self.far_since = np.full(n, np.nan)
        self.t_still = None              # since when the car has been standing still
        self.t_learn = None              # first still moment (learning starts)
        self.last = None                 # (now, yaw) of the previous scan
        self.target, self.seen, self.beeped = None, 0.0, -1e9
        self.beep = False
        self.n_new = 0

    def _bins(self, pts, yaw):
        world = np.arctan2(pts[:, 1], pts[:, 0]) + yaw
        return (np.floor((world % (2 * math.pi)) / self.BIN)).astype(int) % len(self.base)

    def step(self, pts, idx, n_rays, now, yaw):
        p = self.p
        dyaw, rate = 0.0, 0.0
        if self.last is not None:
            dyaw = wrap(yaw - self.last[1])
            rate = abs(dyaw) / max(now - self.last[0], 1e-3)
        self.last = (now, yaw)
        if rate > p['still_rate']:
            self.t_still = None
        elif self.t_still is None:
            self.t_still = now
        still = self.t_still is not None and now - self.t_still >= p['settle_s']
        if self.target is not None:
            self.target = rotate_back(self.target, dyaw)
        if not len(pts):
            return (0.0, 0.0, 0.0), 'watch', '警戒中'
        d = np.hypot(pts[:, 0], pts[:, 1])
        b = self._bins(pts, yaw)

        if self.t_learn is None or now - self.t_learn < p['learn_s']:
            if still:
                self.t_learn = self.t_learn or now
                for i in np.unique(b):
                    r = float(d[b == i].min())
                    self.base[i] = r if np.isnan(self.base[i]) else min(self.base[i], r)
            return (0.0, 0.0, 0.0), 'learn', '正在记住周围环境，请离车 1.5 m 以外…'

        groups = blobs(pts, idx, n_rays)
        tgt = None
        if self.target is not None:
            tgt = pick(pts, groups, self.target, p['gate'])
        if still:
            ref = np.fmin(np.fmin(self.base, np.roll(self.base, 1)), np.roll(self.base, -1))[b]
            new = (d < ref - p['delta']) & (d < p['range'])
            self.n_new = int(new.sum())
            near_tgt = np.zeros(len(pts), bool) if tgt is None else np.hypot(*(pts - tgt).T) < 0.4
            for i in np.unique(b):
                m = b == i
                r = float(d[m].min())
                if near_tgt[m].any():
                    continue
                if np.isnan(self.base[i]):
                    self.base[i] = r             # first seen (was in a blind sector before a turn)
                elif r > self.base[i] + p['delta']:
                    if np.isnan(self.far_since[i]):
                        self.far_since[i] = now
                    elif now - self.far_since[i] > p['heal_s']:
                        self.base[i], self.far_since[i] = r, np.nan
                else:
                    self.far_since[i] = np.nan
            cand = [g for g in groups if new[g].sum() >= p['min_pts']]
            if tgt is not None and pick(pts, cand, tgt, 0.01) is None:
                tgt = None                       # what we were tracking is part of the scene again
            if tgt is None and cand:
                tgt = pick(pts, cand, np.zeros(2), 9.0)
        if tgt is None:
            if self.target is not None and now - self.seen > 1.0:
                self.target = None
            if self.target is not None:
                return (0.0, 0.0, 0.0), 'alert', '目标短暂丢失'
            return (0.0, 0.0, 0.0), 'watch', '警戒中，没有发现靠近的东西'
        fresh = self.target is None
        self.target, self.seen = tgt, now
        if fresh or now - self.beeped > p['beep_every']:
            self.beeped = now
            self.beep = True
        ang = math.atan2(tgt[1], tgt[0])
        can_turn = float(d.min()) > p['turn_radius']
        wz = clamp(p['kp_ang'] * ang, p['max_w']) if abs(ang) > math.radians(5) and can_turn else 0.0
        return (0.0, 0.0, wz), 'alert', f'发现目标：{math.hypot(*tgt):.2f} m，{math.degrees(ang):+.0f}°'


class LidarBehavior(Node):
    def __init__(self):
        super().__init__('c21_behavior')
        dp = self.declare_parameter
        self.kind = dp('behavior', 'follow').value
        g = {
            'scan_x': dp('scan_x', 0.0853).value, 'scan_yaw': dp('scan_yaw', math.pi).value,
            'self_range': dp('self_range', 0.14).value,
            'hl': dp('robot_half_length', 0.15).value, 'hw': dp('robot_half_width', 0.15).value,
        }
        self.geo = g
        p = {'hl': g['hl'], 'hw': g['hw'], 'turn_radius': dp('turn_radius', 0.24).value}
        if self.kind == 'avoid':
            p.update(speed=dp('avoid_speed', 0.15).value, threshold=dp('avoid_threshold', 0.45).value,
                     turn_w=dp('avoid_turn_w', 0.6).value, side_margin=dp('avoid_side_margin', 0.05).value,
                     flank=dp('avoid_flank', 0.10).value, side_speed=dp('avoid_side_speed', 0.05).value,
                     esc_speed=dp('avoid_esc_speed', 0.05).value, esc_s=dp('avoid_esc_s', 3.0).value)
            self.logic = Avoid(p)
        elif self.kind == 'follow':
            p.update(range=dp('follow_range', 1.2).value, fov=dp('follow_fov_deg', 70.0).value,
                     acq_range=dp('follow_acquire_range', 1.0).value, acq_fov=dp('follow_acquire_fov_deg', 30.0).value,
                     keep=dp('follow_keep', 0.45).value, gate=dp('follow_gate', 0.35).value,
                     lost_s=dp('follow_lost_s', 0.8).value, kp_lin=dp('follow_kp_lin', 0.8).value,
                     kp_ang=dp('follow_kp_ang', 1.5).value, speed=dp('follow_speed', 0.2).value,
                     back_speed=dp('follow_back_speed', 0.08).value, max_w=dp('follow_max_w', 0.8).value)
            self.logic = Follow(p)
        elif self.kind == 'guard':
            p.update(range=dp('guard_range', 1.5).value, delta=dp('guard_delta', 0.25).value,
                     learn_s=dp('guard_learn_s', 2.0).value, min_pts=dp('guard_min_pts', 3).value,
                     beep_every=dp('guard_beep_every', 4.0).value, kp_ang=dp('guard_kp_ang', 1.5).value,
                     max_w=dp('guard_max_w', 0.8).value, gate=dp('guard_gate', 0.4).value,
                     still_rate=dp('guard_still_rate', 0.1).value, settle_s=dp('guard_settle_s', 0.3).value,
                     heal_s=dp('guard_heal_s', 1.5).value)
            self.logic = Guard(p)
        else:
            raise SystemExit(f'unknown behavior {self.kind}')
        self.yaw = None
        self.prev_yaw = None
        self.log_state, self.log_t = None, 0.0
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_behavior', 5)
        self.state_pub = self.create_publisher(String, 'c21/behavior_state', 5)
        self.beep_pub = self.create_publisher(UInt8, 'c21/beep', 5)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(Bool, 'c21/behavior_hold', self._on_hold, 10)
        self.hold_until = 0.0
        self.last_scan = time.monotonic()
        self.status = {'behavior': self.kind, 'state': 'wait', 'text': '等待雷达数据', 'target': None}
        self.create_timer(0.2, self._tick)
        self.get_logger().info(f'lidar behaviour: {self.kind}')

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def _on_hold(self, msg):
        self.hold_until = time.monotonic() + 1.0 if msg.data else 0.0

    def _on_scan(self, msg):
        if time.monotonic() < self.hold_until:
            self.last_scan = time.monotonic()
            self.prev_yaw = self.yaw
            self._send(0.0, 0.0, 0.0)
            self.status = {**self.status, 'state': 'paused', 'text': '与 APP 的连接中断，已暂停（连上后继续）'}
            if self.log_state != 'paused':
                self.log_state = 'paused'
                self.get_logger().info('paused: APP link down')
            return
        g = self.geo
        pts, idx = scan_points(msg.ranges, msg.angle_min, msg.angle_increment, g['scan_x'], g['scan_yaw'],
                               g['self_range'], g['hl'], g['hw'], with_idx=True)
        n_rays = len(msg.ranges)
        now = time.monotonic()
        self.last_scan = now
        yaw = self.yaw
        if self.kind == 'guard':
            if yaw is None:
                return
            self.logic.beep = False
            (vx, vy, wz), st, text = self.logic.step(pts, idx, n_rays, now, yaw)
            if self.logic.beep:
                self.beep_pub.publish(UInt8(data=2))
        elif self.kind == 'follow':
            dyaw = 0.0 if yaw is None or self.prev_yaw is None else wrap(yaw - self.prev_yaw)
            (vx, vy, wz), st, text = self.logic.step(pts, idx, n_rays, now, dyaw)
        else:
            (vx, vy, wz), st, text = self.logic.step(pts, now)
        self.prev_yaw = yaw
        self._send(vx, vy, wz)
        tgt = getattr(self.logic, 'target', None)
        self.status = {'behavior': self.kind, 'state': st, 'text': text,
                       'target': None if tgt is None else [round(float(tgt[0]), 2), round(float(tgt[1]), 2)]}
        moving = max(abs(vx), abs(vy), abs(wz)) > 1e-3
        if st != self.log_state or (moving and now - self.log_t > 0.5):
            self.log_state, self.log_t = st, now
            extra = f' new={self.logic.n_new}' if self.kind == 'guard' else ''
            self.get_logger().info(f'{st}: {text} | cmd {vx:+.2f} {vy:+.2f} {wz:+.2f} tgt {self.status["target"]}'
                                   f' yaw {math.degrees(yaw or 0):+.0f}{extra}')

    def _send(self, vx, vy, wz):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self.cmd_pub.publish(t)

    def _tick(self):
        if time.monotonic() - self.last_scan > 0.5:
            self._send(0.0, 0.0, 0.0)
            self.status = {**self.status, 'state': 'wait', 'text': '没有雷达数据，已停车'}
        self.state_pub.publish(String(data=json.dumps(self.status, ensure_ascii=False)))


def main():
    rclpy.init()
    node = LidarBehavior()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node._send(0.0, 0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
