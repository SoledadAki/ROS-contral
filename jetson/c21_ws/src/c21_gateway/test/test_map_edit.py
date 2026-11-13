import numpy as np

from c21_gateway.map_edit import erase, read_pgm, write_pgm


def grid():
    img = np.full((40, 60), 254, np.uint8)       # 3 x 2 m at 5 cm, origin (-1, -1)
    img[:5, :] = 205                             # unknown strip at the top
    img[20, 10:50] = 0                           # false line, cell centres at y = -1 + (39 - 20 + 0.5) * 0.05 = -0.025
    img[30, 10:50] = 0                           # real wall at y = -0.525
    return img


def test_erase_only_along_stroke_and_only_obstacles():
    img = grid()
    n = erase(img, [[[0.0, -0.025], [1.0, -0.025]]], 0.05, (-1.0, -1.0), 0.1)
    assert n == 24                               # centres x -0.1 .. 1.1 (cols 18..41)
    assert (img[20, 18:42] == 254).all() and (img[20, 10:18] == 0).all() and (img[20, 42:50] == 0).all()
    assert (img[30, 10:50] == 0).all()           # wall untouched
    assert (img[:5] == 205).all()


def test_tap_and_unknown_kept():
    img = grid()
    n = erase(img, [[[-1.0, 0.9]]], 0.05, (-1.0, -1.0), 0.3)
    assert n == 0 and (img[:5] == 205).all()


def test_pgm_roundtrip(tmp_path):
    img = grid()
    p = tmp_path / 'm.pgm'
    write_pgm(p, img)
    assert (read_pgm(p) == img).all()
    p.write_bytes(b'P5\n# comment\n60 40\n255\n' + img.tobytes())
    assert (read_pgm(p) == img).all()
