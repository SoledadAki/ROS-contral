"""Saved-map editing (no ROS): erase false obstacles from a map_server pgm."""
import numpy as np

FREE, OCC_MAX = 254, 100        # map_server trinary: free 254, unknown 205, occupied 0


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    parts, i = [], 0
    while len(parts) < 4:                       # magic, width, height, maxval (comments skipped)
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            i = data.index(b'\n', i)
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        parts.append(data[i:j])
        i = j
    if parts[0] != b'P5':
        raise ValueError('not a binary pgm')
    w, h = int(parts[1]), int(parts[2])
    return np.frombuffer(data[i + 1:i + 1 + w * h], np.uint8).reshape(h, w).copy()


def write_pgm(path, img):
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (img.shape[1], img.shape[0]))
        f.write(np.ascontiguousarray(img).tobytes())


def erase(img, strokes, res, origin, radius):
    """Turn occupied cells within `radius` m of the stroke polylines (map xy) into free space.

    img row 0 is the top (max y), as map_server reads it. Unknown cells are left unknown, so
    nothing unexplored becomes drivable. Returns the number of cells changed."""
    h, w = img.shape
    rc = radius / res
    mask = np.zeros_like(img, dtype=bool)
    for pts in strokes:
        pts = np.asarray(pts, float).reshape(-1, 2)
        if len(pts) == 1:
            pts = np.vstack([pts, pts])
        for a, b in zip(pts[:-1], pts[1:]):
            n = max(1, int(np.ceil(np.hypot(*(b - a)) / (res / 2))))
            for t in np.linspace(0.0, 1.0, n + 1):
                x, y = a + t * (b - a)
                cx = (x - origin[0]) / res - 0.5                 # cell centres at integer col / row
                cy = h - 1 - ((y - origin[1]) / res - 0.5)
                x0, x1 = max(0, int(cx - rc - 1)), min(w, int(cx + rc + 2))
                y0, y1 = max(0, int(cy - rc - 1)), min(h, int(cy + rc + 2))
                if x0 >= x1 or y0 >= y1:
                    continue
                yy, xx = np.mgrid[y0:y1, x0:x1]
                mask[y0:y1, x0:x1] |= (xx - cx) ** 2 + (yy - cy) ** 2 <= rc * rc
    hit = mask & (img < OCC_MAX)
    img[hit] = FREE
    return int(hit.sum())
