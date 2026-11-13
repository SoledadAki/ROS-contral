import math

import numpy as np

from c21_behaviors.person_track import (PersonTracker, appearance, avoid_cmd, box_bearing, follow_cmd, hist_distance,
                                        person_position, project)

K = np.array([[367.0, 0.0, 320.0], [0.0, 367.0, 240.0], [0.0, 0.0, 1.0]])
# level camera 0.24 m up looking along +x: optical (x right, y down, z fwd) -> base
R = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
T = np.array([0.05, 0.0, 0.24])

P = {'acq_range': 2.5, 'acq_fov': 20.0, 'acq_frames': 3, 'gate': 0.6, 'walk': 1.2, 'gate_max': 1.5,
     'max_hist': 0.6, 'reacq_hist': 0.4, 'learn_hist': 0.35, 'hist_rate': 0.1, 'w_hist': 1.5,
     'lost_s': 1.0, 'forget_s': 20.0, 'keep': 1.0, 'deadband': 0.08, 'kp_lin': 0.8, 'kp_ang': 1.5,
     'speed': 0.3, 'back_speed': 0.08, 'max_w': 0.9, 'lidar_gate': 0.3, 'lidar_lost_s': 0.5, 'doubt_s': 2.5}


def person_scene(px, py, wall=3.0):
    """Base-frame points: a 0.4 m wide person (slab) at (px, py) in front of a wall, plus floor."""
    ys = np.linspace(-1.5, 1.5, 150)
    zs = np.linspace(0.0, 1.6, 80)
    Y, Z = np.meshgrid(ys, zs)
    wallp = np.stack([np.full(Y.size, wall), Y.ravel(), Z.ravel()], axis=1)
    py_, pz = np.meshgrid(np.linspace(py - 0.2, py + 0.2, 20), np.linspace(0.0, 1.7, 60))
    pers = np.stack([np.full(py_.size, px), py_.ravel(), pz.ravel()], axis=1)
    fx, fy = np.meshgrid(np.linspace(0.4, 2.9, 40), np.linspace(-1.5, 1.5, 40))
    floor = np.stack([fx.ravel(), fy.ravel(), np.zeros(fx.size)], axis=1)
    pb = np.concatenate([wallp, pers, floor])
    uv = project((pb - T) @ R, K)
    return pb, uv


def box_of(px, py, h=1.7):
    """Image box of the person slab (clipped to the image)."""
    corners = np.array([[px, py + 0.2, h], [px, py - 0.2, 0.0]])
    uv = project((corners - T) @ R, K)
    x1, y1 = uv[0]
    x2, y2 = uv[1]
    return [max(0, x1), max(0, y1), min(639, x2), min(479, y2), 0.9]


def test_person_position_ignores_wall_and_floor():
    pb, uv = person_scene(1.5, 0.3)
    pos = person_position(box_of(1.5, 0.3), uv, pb)
    assert pos is not None
    assert abs(pos[0] - 1.5) < 0.05 and abs(pos[1] - 0.3) < 0.1


def test_person_position_needs_points():
    pb, uv = person_scene(1.5, 0.0)
    assert person_position([600, 10, 620, 30, 0.9], uv, pb) is None


def test_box_bearing():
    b = box_of(2.0, 0.5)
    assert abs(box_bearing(b, K, R) - math.atan2(0.5, 2.0 - T[0])) < 0.05


def solid(color, h=200, w=100):
    img = np.zeros((h, w, 3), np.uint8)
    img[:] = color
    return img


def test_appearance_tells_colours_apart():
    red = appearance(solid((200, 30, 30)), [0, 0, 100, 200])
    red2 = appearance(solid((190, 40, 35)), [0, 0, 100, 200])
    blue = appearance(solid((30, 40, 200)), [0, 0, 100, 200])
    assert hist_distance(red, red2) < 0.3
    assert hist_distance(red, blue) > 0.8
    assert hist_distance(None, red) == 1.0


def det(x, y, hist, robot=(0.0, 0.0)):
    return {'pos': np.array([x, y]), 'bearing': math.atan2(y - robot[1], x - robot[0]),
            'range': math.hypot(x - robot[0], y - robot[1]), 'hist': hist, 'box': None}


RED = appearance(solid((200, 30, 30)), [0, 0, 100, 200])
BLUE = appearance(solid((30, 40, 200)), [0, 0, 100, 200])
Z = np.zeros(2)


def locked_tracker():
    tr = PersonTracker(P)
    for i in range(3):
        tr.update([det(1.5, 0.0, RED)], i * 0.1, Z, 0.0)
    assert tr.locked
    return tr


def test_locks_only_on_person_in_front_after_three_frames():
    tr = PersonTracker(P)
    assert tr.update([det(1.0, 1.0, RED)], 0.0, Z, 0.0) is None       # 45 deg to the side
    for i in range(2):
        dets = [det(1.5, 0.0, RED), det(1.0, 1.0, BLUE)]
        assert tr.update(dets, i * 0.1, Z, 0.0) is None
        assert tr.pending_det is dets[0] and tr.pending_n == i + 1
    d = tr.update([det(1.5, 0.0, RED), det(1.0, 1.0, BLUE)], 0.2, Z, 0.0)
    assert d is not None and tr.locked and np.allclose(tr.target, [1.5, 0.0])
    assert tr.pending_det is None


def test_follows_target_not_passer_by():
    tr = locked_tracker()
    # target walks left; a differently dressed person appears where the target was
    d = tr.update([det(1.5, 0.0, BLUE), det(1.6, 0.35, RED)], 0.3, Z, 0.0)
    assert d['hist'] is RED and np.allclose(tr.target, [1.6, 0.35])


def test_lost_target_taken_back_only_by_colour():
    tr = locked_tracker()
    for t in (0.5, 1.0, 1.5):                  # hidden
        assert tr.update([], t, Z, 0.0) is None
    assert tr.update([det(2.0, -1.0, BLUE)], 2.0, Z, 0.0) is None
    d = tr.update([det(2.0, -1.0, BLUE), det(1.0, 1.2, RED)], 2.1, Z, 0.0)
    assert d is not None and d['hist'] is RED


def test_bearing_only_keeps_distance():
    tr = locked_tracker()
    d = {'pos': None, 'bearing': math.radians(5), 'range': None, 'hist': RED, 'box': None}
    assert tr.update([d], 0.3, Z, 0.0) is d
    assert abs(math.hypot(*tr.target) - 1.5) < 1e-6


def test_forget_relocks_on_anyone_in_front():
    tr = locked_tracker()
    tr.update([], 25.0, Z, 0.0)
    assert not tr.locked


def test_follow_cmd():
    vx, _, wz = follow_cmd(np.array([2.0, 0.0]), True, P)
    assert vx == P['speed'] and wz == 0.0
    vx, _, _ = follow_cmd(np.array([0.5, 0.0]), True, P)
    assert vx == -P['back_speed']
    vx, _, wz = follow_cmd(np.array([2.0, 0.0]), False, P)       # no distance: turn only
    assert vx == 0.0
    vx, _, wz = follow_cmd(np.array([0.5, 1.0]), True, P)        # far to the side: turn first
    assert vx == 0.0 and wz > 0


AV = {**P, 'hl': 0.15, 'hw': 0.15, 'av_margin': 0.05, 'av_clear': 0.45, 'av_speed': 0.15, 'av_person_r': 0.4}


def box_pts(x0, x1, y0, y1, n=12):
    X, Y = np.meshgrid(np.linspace(x0, x1, n), np.linspace(y0, y1, n))
    return np.stack([X.ravel(), Y.ravel()], axis=1)


def test_avoid_clear_path_is_plain_follow():
    tgt = np.array([2.0, 0.0])
    person = box_pts(1.9, 2.1, -0.15, 0.15)              # the target's own legs
    cmd, side, st = avoid_cmd(tgt, True, person, 0, AV)
    assert st is None and cmd == follow_cmd(tgt, True, AV)


def test_avoid_slides_round_obstacle_facing_target():
    tgt = np.array([2.5, 0.0])
    obst = box_pts(0.45, 0.6, -0.35, 0.1)                # box in the way, sticking out to the right
    cmd, side, st = avoid_cmd(tgt, True, obst, 0, AV)
    assert st == 'detour' and side == 1 and cmd[1] > 0   # goes left (shorter way round)
    assert cmd[2] == 0.0                                 # still facing the person
    # once past it, straight again
    cmd, side, st = avoid_cmd(tgt, True, obst - [0.0, 0.6], side, AV)
    assert st is None


def test_avoid_keeps_side_and_reports_blocked():
    tgt = np.array([2.5, 0.0])
    obst = box_pts(0.45, 0.6, -0.2, 0.2)                 # symmetric: either way
    _, side, st = avoid_cmd(tgt, True, obst, -1, AV)
    assert st == 'detour' and side == -1
    wall = np.concatenate([box_pts(0.3, 0.4, -3, 3, 60), box_pts(-0.3, 0.4, 0.3, 0.4, 30),
                           box_pts(-0.3, 0.4, -0.4, -0.3, 30)])
    cmd, _, st = avoid_cmd(tgt, True, wall, 0, AV)
    assert st == 'blocked' and cmd[0] == 0.0 and cmd[1] == 0.0


def test_lidar_keeps_track_between_camera_frames():
    tr = locked_tracker()                                   # camera saw target at (1.5, 0) at t=0.2
    xs = [1.5 + 0.1 * i for i in range(1, 30)]              # walks away at 1 m/s, camera blind
    for i, x in enumerate(xs):
        t = 0.3 + 0.1 * i
        c = tr.update_lidar([np.array([x, 0.0]), np.array([1.0, -1.0])], t, Z)   # + a chair leg
        assert c is not None and np.allclose(c, [x, 0.0])
        assert tr.update([], t, Z, 0.0) is None
    assert tr.lost_for(t) < 0.05 and abs(tr.range - xs[-1]) < 1e-6


def test_lidar_cannot_start_or_take_back_a_track():
    tr = PersonTracker(P)
    assert tr.update_lidar([np.array([1.0, 0.0])], 0.0, Z) is None and not tr.locked
    tr = locked_tracker()
    assert tr.update_lidar([np.array([1.5, 0.0])], 1.5, Z) is None      # 1.3 s unseen: too late


def test_doubt_drops_lidar_track_in_view():
    tr = locked_tracker()
    t = 0.2
    while t < 3.0:
        t += 0.1
        tr.update_lidar([np.array([1.5, 0.0])], t, Z)       # lidar on something that never looks right
        assert not tr.check_view(t, in_view=False)          # out of the picture: no doubt
    assert tr.check_view(t, in_view=True) and not tr.lidar_ok
    assert tr.lost_for(t) > 2.5
    assert tr.update_lidar([np.array([1.5, 0.0])], t + 0.1, Z) is None
    d = tr.update([det(1.6, 0.4, RED)], t + 0.2, Z, 0.0)    # camera finds the person again
    assert d is not None and tr.lidar_ok
