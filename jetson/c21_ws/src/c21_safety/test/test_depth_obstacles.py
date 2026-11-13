import numpy as np

from c21_safety.depth_obstacles import CellMemory, cells, classify, deproject, quat_matrix, seen_through

K = np.array([[100.0, 0.0, 40.0], [0.0, 100.0, 30.0], [0.0, 0.0, 1.0]])
P = {'range_min': 0.15, 'range_max': 1.6, 'min_height': 0.035, 'max_height': 0.32, 'floor_band': 0.02}

# level camera 0.24 m up, looking along +x: optical (x right, y down, z fwd) -> base
R_CAM = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
T_CAM = np.array([0.05, 0.0, 0.24])


def scene(box_x=None, box_h=0.06, w=80, h=60):
    """Depth image of a flat floor, optionally with a box face at distance box_x (base x)."""
    v, u = np.mgrid[0:h, 0:w].astype(np.float64)
    d = np.zeros((h, w))
    below = v > K[1, 2]
    d[below] = T_CAM[2] * K[1, 1] / (v[below] - K[1, 2])          # floor
    if box_x is not None:
        z = box_x - T_CAM[0]
        top_v = K[1, 2] + (T_CAM[2] - box_h) * K[1, 1] / z            # box top edge row
        bot_v = K[1, 2] + T_CAM[2] * K[1, 1] / z
        face = (v >= top_v) & (v <= bot_v) & (np.abs(u - K[0, 2]) < 10)
        d[face] = z
    d[d > 4.0] = 0.0
    return d.astype(np.float32)


def to_base(d):
    pts, _, _ = deproject(d, K, 1)
    return pts, pts @ R_CAM.T + T_CAM


def test_floor_is_not_an_obstacle():
    pts, pb = to_base(scene())
    obstacle, floor = classify(pb, np.linalg.norm(pts, axis=1), P)
    assert not obstacle.any()
    assert floor.sum() > 100


def test_low_box_is_an_obstacle():
    pts, pb = to_base(scene(box_x=0.8, box_h=0.06))     # 6 cm: under the lidar plane
    obstacle, _ = classify(pb, np.linalg.norm(pts, axis=1), P)
    ob = pb[obstacle]
    assert len(ob) > 20
    assert np.allclose(ob[:, 0], 0.8, atol=0.02)
    grid = cells(ob, 0.03, 4)
    assert grid and all(0.75 < x < 0.85 and z <= 0.065 for x, _, z in grid.values())


def test_cells_drop_sparse_noise():
    pts = np.array([[0.5, 0.0, 0.1]] * 5 + [[1.0, 0.3, 0.1]])
    grid = cells(pts, 0.03, 4)
    assert len(grid) == 1


def test_memory_needs_two_frames_and_forgets():
    m = CellMemory(confirm=2)
    m.update({(1, 2): (0.04, 0.07, 0.05)}, 1.0)
    assert len(m.confirmed()[0]) == 0
    m.update({(1, 2): (0.04, 0.07, 0.05), (5, 5): (0.2, 0.2, 0.1)}, 2.0)
    keys, _ = m.confirmed()
    assert keys == [(1, 2)]
    m.update({}, 3.0)                     # out of view: confirmed cell stays, the one-frame cell goes
    assert list(m.cells) == [(1, 2)]
    m.prune(3.0, 10.0, (5.0, 0.0), 1.2)   # robot drove away
    assert not m.cells


def test_seen_through():
    d = scene()                            # floor only
    # a remembered point 1.5 m ahead, 5 cm up: the floor behind it is visible -> gone
    pc = (np.array([[1.5, 0.0, 0.05]]) - T_CAM) @ R_CAM
    assert seen_through(pc, d, K, 0.08)[0]
    # same point while the box is still there -> kept
    d2 = scene(box_x=1.5, box_h=0.06)
    assert not seen_through(pc, d2, K, 0.08)[0]
    # behind the camera / outside the image -> kept
    assert not seen_through(np.array([[0.0, 0.0, -1.0], [5.0, 0.0, 1.0]]), d, K, 0.08).any()


def test_quat_matrix_yaw():
    class Q:
        x, y, z, w = 0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)
    assert np.allclose(quat_matrix(Q) @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
