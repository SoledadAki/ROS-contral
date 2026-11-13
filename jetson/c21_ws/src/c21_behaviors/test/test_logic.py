import math
import sys
import types

import numpy as np

for m in ('rclpy', 'rclpy.executors', 'rclpy.node', 'rclpy.qos', 'geometry_msgs.msg', 'nav_msgs.msg',
          'sensor_msgs.msg', 'std_msgs.msg'):
    mod = sys.modules.setdefault(m, types.ModuleType(m))
    for n in ('Node', 'qos_profile_sensor_data', 'Twist', 'Odometry', 'LaserScan', 'Bool', 'String', 'UInt8'):
        setattr(mod, n, object)
from c21_behaviors.lidar_behavior import Avoid, Follow, Guard, scan_points  # noqa: E402

P = {'hl': 0.15, 'hw': 0.15, 'turn_radius': 0.24}


def room(obj=None, wall=2.0, n=720, yaw=0.0, idx=False, walls=None):
    """Lidar-frame scan (0 = straight back) of a square room; optional round objects (x, y, r) in
    world frame (list or one tuple); robot at the origin facing `yaw`. walls: (front, back, left, right)."""
    objs = [] if obj is None else ([obj] if isinstance(obj, tuple) else obj)
    wf, wb, wl, wr = walls or (wall,) * 4
    a = -math.pi + 2 * math.pi * np.arange(n) / n
    r = np.empty(n)
    lx, ly = 0.0853 * math.cos(yaw), 0.0853 * math.sin(yaw)
    for i, ang in enumerate(a):
        b = ang + math.pi + yaw
        c, s = math.cos(b), math.sin(b)
        best = 99.0
        if c > 1e-6:
            best = min(best, (wf - lx) / c)
        if c < -1e-6:
            best = min(best, (-wb - lx) / c)
        if s > 1e-6:
            best = min(best, (wl - ly) / s)
        if s < -1e-6:
            best = min(best, (-wr - ly) / s)
        for o in objs:
            ox, oy, rad = o[0] - lx, o[1] - ly, o[2]
            t = ox * c + oy * s
            dd = (ox - t * c) ** 2 + (oy - t * s) ** 2
            if t > 0 and dd < rad ** 2:
                best = min(best, t - math.sqrt(rad ** 2 - dd))
        r[i] = best
    out = scan_points(r, -math.pi, 2 * math.pi / n, 0.0853, math.pi, 0.14, 0.15, 0.15, with_idx=idx)
    return (*out, n) if idx else out


def legs(x, y):
    return [(x, y + 0.09, 0.06), (x, y - 0.09, 0.06)]


def test_points_frame():
    pts = room(obj=(0.6, 0.0, 0.05))
    near = pts[np.argmin(np.hypot(pts[:, 0], pts[:, 1]))]
    assert abs(near[0] - 0.55) < 0.03 and abs(near[1]) < 0.03


AP = {**P, 'speed': 0.15, 'threshold': 0.45, 'turn_w': 0.6, 'side_margin': 0.05, 'flank': 0.10,
      'side_speed': 0.05, 'esc_speed': 0.05, 'esc_s': 3.0}


def test_avoid():
    a = Avoid(AP)
    (vx, vy, wz), st, _ = a.step(room(), 0.0)
    assert st == 'go' and vx > 0.1 and wz == 0 and vy == 0
    (vx, vy, wz), st, _ = a.step(room(obj=(0.5, -0.1, 0.1)), 0.1)      # blocked, obstacle a bit right
    assert st == 'turn' and vx == 0 and wz > 0                          # turns left (freer)


def test_avoid_turns_towards_open_side():
    a = Avoid(AP)
    (_, _, wz), st, _ = a.step(room(walls=(0.55, 3, 0.45, 3)), 0.0)     # wall ahead, left closed, right open
    assert st == 'turn' and wz < 0


def test_avoid_boxed_in_gives_up_after_a_full_turn():
    a = Avoid(AP)
    t, st = 0.0, None
    for _ in range(200):                                                # 20 s in a 0.7 m box
        (_, _, wz), st, _ = a.step(room(wall=0.35), t)
        t += 0.1
    assert st == 'stuck' and wz == 0


def test_avoid_flank_pushes_away():
    a = Avoid(AP)
    (vx, vy, wz), st, _ = a.step(room(obj=(0.45, -0.24, 0.03), wall=3.0), 0.0)   # just right of the path
    assert st == 'go' and vx > 0 and vy > 0                             # slides left while passing


def test_avoid_escape_sideways_then_back():
    a = Avoid(AP)
    side = [(0.6, 0.0, 0.1), (0.05, -0.21, 0.03)]                       # blocked ahead + post at the right
    (vx, vy, wz), st, txt = a.step(room(obj=side, wall=3.0), 0.0)
    assert st == 'escape' and vy > 0 and vx == 0 and wz == 0, txt       # slides left, no rotation
    a = Avoid(AP)
    (vx, vy, wz), st, txt = a.step(room(obj=(0.29, 0.0, 0.06), wall=3.0), 0.0)   # post 8 cm off the bumper
    assert st == 'escape' and vx < 0 and wz == 0, txt                   # creeps back
    t = 0.1
    for _ in range(40):
        (vx, vy, wz), st, _ = a.step(room(obj=(0.29, 0.0, 0.06), wall=3.0), t)
        t += 0.1
    assert st == 'stuck' and vx == 0                                    # did not get free in 3 s


FP = {**P, 'range': 1.2, 'fov': 70, 'acq_range': 1.0, 'acq_fov': 30, 'keep': 0.45, 'gate': 0.35, 'lost_s': 0.8,
      'kp_lin': 0.8, 'kp_ang': 1.5, 'speed': 0.2, 'back_speed': 0.08, 'max_w': 0.8}


def test_follow():
    f = Follow(FP)
    (vx, vy, wz), st, _ = f.step(*room(wall=3.0, idx=True), 0.0)
    assert st == 'lost' and vx == 0 and wz == 0
    (vx, vy, wz), st, _ = f.step(*room(obj=legs(0.9, 0.3), wall=3.0, idx=True), 0.1)
    assert st == 'track' and vx > 0 and wz > 0                          # ahead-left, too far: go + turn left
    (_, _, _), st, _ = f.step(*room(obj=legs(0.3 + 0.08, 0.0), wall=3.0, idx=True), 0.2)
    assert st == "hold"                                                 # jump > gate: not the same target
    (vx, vy, wz), st, _ = f.step(*room(obj=legs(0.3 + 0.08, 0.0), wall=3.0, idx=True), 1.2)
    assert st == 'track' and -0.08 <= vx < 0                            # too close: creeps back slowly


def test_follow_ignores_walls():
    f = Follow(FP)
    (vx, vy, wz), st, _ = f.step(*room(walls=(0.7, 3, 3, 3), idx=True), 0.0)
    assert st == 'lost' and wz == 0                                     # a wall right in front is no target
    (vx, vy, wz), st, _ = f.step(*room(walls=(3, 3, 0.35, 3), idx=True), 0.1)
    assert st == 'lost'                                                 # nor a wall close at the side


def test_follow_close_to_wall_does_not_turn_away():
    """Person walks up close to the car next to a wall on the left: target must stay on the person."""
    f = Follow(FP)
    t = 0.0
    for x in (0.9, 0.8, 0.7, 0.6, 0.5, 0.45, 0.42, 0.40, 0.40, 0.40):
        (vx, vy, wz), st, _ = f.step(*room(obj=legs(x, 0.05), walls=(3, 3, 0.45, 3), idx=True), t)
        t += 0.1
        assert st == 'track', st
        assert abs(math.degrees(math.atan2(f.target[1], f.target[0]))) < 15
        assert abs(wz) < 0.4


def test_follow_turn_compensated():
    """Car turns left 5 deg per scan while the person stands still."""
    f = Follow(FP)
    f.step(*room(obj=legs(0.8, 0.25), wall=3.0, idx=True), 0.0)
    assert f.target is not None
    yaw = 0.0
    for k in range(1, 4):
        yaw = math.radians(5 * k)
        (_, _, wz), st, _ = f.step(*room(obj=legs(0.8, 0.25), wall=3.0, yaw=yaw, idx=True), 0.1 * k,
                                   math.radians(5))
        assert st == 'track'
    want = math.atan2(0.25, 0.8) - yaw
    assert abs(math.atan2(f.target[1], f.target[0]) - want) < math.radians(4)


GP = {**P, 'range': 1.5, 'delta': 0.25, 'learn_s': 2.0, 'min_pts': 3, 'beep_every': 4.0, 'kp_ang': 1.5,
      'max_w': 0.8, 'gate': 0.4, 'still_rate': 0.1, 'settle_s': 0.3, 'heal_s': 1.5}


def run(g, t0, t1, yaw=0.0, obj=None, wall=1.2):
    out = None
    t = t0
    while t < t1 - 1e-9:
        g.beep = False
        out = g.step(*room(obj=obj, wall=wall, yaw=yaw, idx=True), t, yaw)
        t += 0.1
    return out


def test_guard():
    g = Guard(GP)
    (_, _, wz), st, _ = run(g, 0.0, 1.0)
    assert st == 'learn'
    run(g, 1.0, 3.0)
    (_, _, wz), st, _ = run(g, 3.0, 3.5)
    assert st == 'watch' and wz == 0 and not g.beep                     # walls do not trigger
    g.beep = False
    (_, _, wz), st, _ = g.step(*room(obj=legs(0.1, -0.6), wall=1.2, idx=True), 3.5, 0.0)
    assert st == 'alert' and wz < 0 and g.beep                          # intruder on the right
    g.beep = False
    (_, _, wz), st, _ = g.step(*room(obj=legs(0.1, -0.6), wall=1.2, idx=True), 3.6, 0.0)
    assert not g.beep                                                   # rate limited


def test_guard_person_during_learning_heals():
    g = Guard(GP)
    run(g, 0.0, 3.0, obj=legs(0.6, 0.0))                               # someone stood in front while learning
    run(g, 3.0, 5.5)                                                    # walks away, scene heals
    (_, _, wz), st, _ = run(g, 5.5, 6.0, obj=legs(0.6, 0.0))           # comes back to the same spot
    assert st == 'alert'


def test_guard_turn_then_person_in_front():
    g = Guard(GP)
    run(g, 0.0, 3.0)
    (_, _, wz), st, _ = run(g, 3.0, 3.3, obj=legs(0.5, 0.5))           # front-left
    assert st == 'alert' and wz > 0
    yaw, t = 0.0, 3.3
    for k in range(9):                                                  # car turns left 45 deg, person stays
        yaw += math.radians(5)
        g.beep = False
        (_, _, wz), st, _ = g.step(*room(obj=legs(0.5, 0.5), wall=1.2, yaw=yaw, idx=True), t, yaw)
        t += 0.1
        assert st == 'alert'
    (_, _, wz), st, txt = run(g, t, t + 0.6, yaw=yaw, obj=legs(0.5, 0.5))
    assert st == 'alert' and abs(wz) < 0.2, txt                         # facing it now, still alerted
    (_, _, wz), st, _ = run(g, t + 0.6, t + 2.0, yaw=yaw)              # person leaves
    assert st == 'watch' and wz == 0
    (_, _, wz), st, _ = run(g, t + 2.0, t + 2.5, yaw=yaw, obj=legs(0.75, 0.75))   # stands in front again
    assert st == 'alert'


def test_guard_rotation_alone_no_false_alarm():
    g = Guard(GP)
    run(g, 0.0, 3.0)
    yaw, t = 0.0, 3.0
    for k in range(60):                                                 # spin a full turn, nobody around
        yaw += math.radians(6)
        g.beep = False
        (_, _, wz), st, _ = g.step(*room(wall=1.2, yaw=yaw, idx=True), t, yaw)
        t += 0.1
        assert st == 'watch', (k, st)
    (_, _, wz), st, _ = run(g, t, t + 1.0, yaw=yaw)
    assert st == 'watch'
