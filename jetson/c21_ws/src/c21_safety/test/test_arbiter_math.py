import math

from c21_safety.arbiter import (outside_footprint, ramp, ray_index, sector_rays, sweep_clearance,
                                sweep_distance)


def test_sweep_front_obstacle():
    pts = [(0.5, 0.0), (0.3, 0.5), (-0.4, 0.0)]
    assert math.isclose(sweep_clearance(pts, 1.0, 0.0, 0.15, 0.15), 0.35)
    # moving left: (0.3, 0.5) is beyond the front edge, so the box slides past it
    assert sweep_clearance(pts, 0.0, 1.0, 0.15, 0.15) == math.inf


def test_ramp_fast_decel():
    assert math.isclose(ramp(0.0, 0.3, 0.04, 0.12), 0.04)
    assert math.isclose(ramp(0.3, 0.0, 0.04, 0.12), 0.18)
    assert ramp(0.05, 0.0, 0.04, 0.12) == 0.0


def test_sweep_ignores_single_outlier():
    pts = [(0.3, 0.0), (0.8, 0.01), (0.81, -0.01)]
    assert math.isclose(sweep_clearance(pts, 1.0, 0.0, 0.15, 0.15, min_hits=2), 0.65)
    assert sweep_clearance([(0.3, 0.0)], 1.0, 0.0, 0.15, 0.15, min_hits=2) == math.inf


def test_self_filter():
    assert not outside_footprint(0.03, 0.0, 0.15, 0.15, 0.01)      # gimbal mast
    assert not outside_footprint(-0.02, -0.07, 0.15, 0.15, 0.01)
    assert outside_footprint(0.20, -0.10, 0.15, 0.15, 0.01)


def test_ray_index_yaw_pi():
    # lidar at x=0.085 mounted backwards: a point straight behind the robot is lidar angle 0
    n, a0, inc = 720, -math.pi, 2 * math.pi / 720
    assert ray_index(-1.0, 0.0, 0.085, math.pi, a0, inc, n) == 360
    # straight ahead is lidar angle +-pi
    assert ray_index(1.0, 0.0, 0.085, math.pi, a0, inc, n) == 0


def test_sector_rays():
    n, a0, inc = 720, -math.pi, 2 * math.pi / 720
    idx = sector_rays([-10.0, 10.0], a0, inc, n)
    assert 360 in idx and 340 in idx and 380 in idx and 300 not in idx
    assert sector_rays([0.0, 0.0], a0, inc, n) <= {359, 360, 361}


def test_sweep_distance():
    hl, hw = 0.15, 0.15
    assert math.isclose(sweep_distance(0.5, 0.0, 1.0, 0.0, hl, hw), 0.35)
    assert sweep_distance(0.5, 0.3, 1.0, 0.0, hl, hw) == math.inf       # passes beside
    assert sweep_distance(-0.5, 0.0, 1.0, 0.0, hl, hw) == math.inf      # behind
    # diagonal: point beside the robot's front-left corner is not "negative distance"
    r = math.sqrt(0.5)
    d = sweep_distance(0.25, 0.10, r, r, hl, hw)
    assert math.isclose(d, 0.10 / r, rel_tol=1e-6)
    assert sweep_distance(0.0, 0.2, r, r, hl, hw) > 0.0
