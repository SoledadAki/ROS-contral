"""Depth camera -> obstacle points for the arbiter and the Nav2 costmaps.

The lidar plane is 9 cm above the floor, so it misses low things (slippers, cables, a box lid) and
anything overhanging (table tops, chair rails). This node turns the Gemini depth image into points
in base_footprint whose height lies between min_height and max_height (the car is 26 cm tall).

    in : camera/depth/image_raw (16UC1 mm), camera/depth/camera_info, TF
    out: c21/depth_obstacles  PointCloud2 base_footprint  obstacles (fresh + remembered)
         c21/depth_floor      PointCloud2 base_footprint  floor hits (costmap clearing source)

The camera cannot see the floor closer than ~0.4 m, so obstacle cells are remembered in the odom
frame and kept while they are out of view: a low box the car approaches stays an obstacle after it
drops below the image. A remembered cell is forgotten when the camera sees through it again (the
depth at its pixel is clearly farther), when the car is far from it, or after memory_s.
"""
import math
import time

import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


def quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tf_matrix(tf):
    """geometry_msgs/TransformStamped -> (R 3x3, t 3)."""
    t = tf.transform.translation
    return quat_matrix(tf.transform.rotation), np.array([t.x, t.y, t.z])


def deproject(depth_m, k, stride):
    """Depth image (metres) -> Nx3 optical-frame points and their (row, col) pixels; zeros dropped."""
    d = depth_m[::stride, ::stride]
    v, u = np.mgrid[0:depth_m.shape[0]:stride, 0:depth_m.shape[1]:stride]
    ok = d > 0.0
    z, u, v = d[ok], u[ok], v[ok]
    pts = np.stack([(u - k[0, 2]) * z / k[0, 0], (v - k[1, 2]) * z / k[1, 1], z], axis=1)
    return pts, v, u


def classify(p_base, cam_range, p):
    """Split base-frame points into obstacle / floor masks (cam_range = distance along the ray)."""
    h = p_base[:, 2]
    near = (cam_range > p['range_min']) & (cam_range < p['range_max'])
    obstacle = near & (h > p['min_height']) & (h < p['max_height'])
    floor = near & (np.abs(h) < p['floor_band'])
    return obstacle, floor


def cells(pts, res, min_count):
    """Grid cells (ix, iy) holding at least min_count points -> {cell: (x, y, top z)}."""
    if len(pts) == 0:
        return {}
    ij = np.floor(pts[:, :2] / res).astype(np.int64)
    uniq, inv, counts = np.unique(ij, axis=0, return_inverse=True, return_counts=True)
    inv = inv.reshape(-1)
    top = np.full(len(uniq), -np.inf)
    np.maximum.at(top, inv, pts[:, 2])
    keep = np.nonzero(counts >= min_count)[0]
    return {(int(uniq[n, 0]), int(uniq[n, 1])): ((uniq[n, 0] + 0.5) * res, (uniq[n, 1] + 0.5) * res, float(top[n]))
            for n in keep}


def seen_through(cam_pts, depth_m, k, margin, border=4):
    """For optical-frame points: True where the image shows something clearly behind them."""
    out = np.zeros(len(cam_pts), dtype=bool)
    if len(cam_pts) == 0:
        return out
    z = cam_pts[:, 2]
    front = z > 0.05
    u = np.full(len(z), -1, dtype=np.int64)
    v = np.full(len(z), -1, dtype=np.int64)
    u[front] = np.round(k[0, 0] * cam_pts[front, 0] / z[front] + k[0, 2]).astype(np.int64)
    v[front] = np.round(k[1, 1] * cam_pts[front, 1] / z[front] + k[1, 2]).astype(np.int64)
    h, w = depth_m.shape
    inside = front & (u >= border) & (u < w - border) & (v >= border) & (v < h - border)
    idx = np.nonzero(inside)[0]
    meas = depth_m[v[idx], u[idx]]
    out[idx] = (meas > 0.0) & (meas > z[idx] + margin)
    return out


class CellMemory:
    """Obstacle cells in the odom frame; a cell needs confirm frames in a row before it counts."""

    def __init__(self, confirm=2):
        self.confirm = confirm
        self.cells = {}          # cell -> [x, y, z, last_seen, streak]

    def update(self, seen, now):
        """seen: {cell: (x, y, z)} from this frame (odom frame)."""
        for c, (x, y, z) in seen.items():
            e = self.cells.get(c)
            if e is None:
                self.cells[c] = [x, y, z, now, 1]
            else:
                e[0], e[1], e[2], e[3] = x, y, z, now
                e[4] = e[4] + 1 if e[4] < self.confirm else self.confirm
        # an unconfirmed cell that missed a frame was probably an edge artefact
        for c in [c for c, e in self.cells.items() if e[4] < self.confirm and e[3] < now]:
            del self.cells[c]

    def forget(self, keys):
        for c in keys:
            self.cells.pop(c, None)

    def prune(self, now, max_age, robot_xy, radius):
        rx, ry = robot_xy
        for c in [c for c, e in self.cells.items()
                  if now - e[3] > max_age or math.hypot(e[0] - rx, e[1] - ry) > radius]:
            del self.cells[c]

    def confirmed(self):
        keys = [c for c, e in self.cells.items() if e[4] >= self.confirm]
        pts = np.array([self.cells[c][:3] for c in keys], dtype=np.float64).reshape(-1, 3)
        return keys, pts


class DepthObstacles(Node):
    def __init__(self):
        super().__init__('c21_depth_obstacles')
        dp = self.declare_parameter
        self.base_frame = dp('base_frame', 'base_footprint').value
        self.odom_frame = dp('odom_frame', 'odom').value
        self.p = {
            'range_min': dp('range_min', 0.15).value,
            'range_max': dp('range_max', 1.6).value,       # depth noise grows with range
            'min_height': dp('min_height', 0.035).value,   # floor fit noise is +-1.5 cm
            'max_height': dp('max_height', 0.32).value,    # car top 0.26 + margin
            'floor_band': dp('floor_band', 0.02).value,
        }
        self.stride = int(dp('stride', 2).value)
        self.cell = dp('cell', 0.03).value
        self.min_pixels = int(dp('min_pixels', 4).value)
        self.max_rate = dp('max_rate', 10.0).value
        self.memory_s = dp('memory_s', 10.0).value
        self.memory_radius = dp('memory_radius', 1.2).value
        self.see_through = dp('see_through_margin', 0.08).value
        self.memory = CellMemory(int(dp('confirm_frames', 2).value))

        self.k = None
        self.last = 0.0
        self.stats_t = time.monotonic()
        self.frames = 0
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.obs_pub = self.create_publisher(point_cloud2.PointCloud2, 'c21/depth_obstacles', 5)
        self.floor_pub = self.create_publisher(point_cloud2.PointCloud2, 'c21/depth_floor', 5)
        self.create_subscription(CameraInfo, 'camera/depth/camera_info', self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, 'camera/depth/image_raw', self._on_depth, qos_profile_sensor_data)
        self.get_logger().info('depth obstacles up: height %.3f..%.2f m, range %.2f..%.2f m' % (
            self.p['min_height'], self.p['max_height'], self.p['range_min'], self.p['range_max']))

    def _on_info(self, msg):
        self.k = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _lookup(self, target, source, stamp):
        try:
            return tf_matrix(self.buf.lookup_transform(target, source, stamp))
        except TransformException:
            try:
                return tf_matrix(self.buf.lookup_transform(target, source, Time()))
            except TransformException:
                return None

    def _on_depth(self, msg):
        now = time.monotonic()
        if self.k is None or now - self.last < 1.0 / self.max_rate:
            return
        if msg.encoding not in ('16UC1', 'mono16'):
            self.get_logger().warn('unexpected depth encoding %s' % msg.encoding, throttle_duration_sec=30.0)
            return
        stamp = Time.from_msg(msg.header.stamp)
        cam_base = self._lookup(self.base_frame, msg.header.frame_id, stamp)
        if cam_base is None:
            self.get_logger().warn('waiting for camera TF', throttle_duration_sec=10.0)
            return
        odom_base = self._lookup(self.odom_frame, self.base_frame, stamp)
        if odom_base is None:
            # no odometry (chassis off): the car cannot drive either, so remember in the base frame
            self.get_logger().warn('no %s TF, memory uses the base frame' % self.odom_frame,
                                   throttle_duration_sec=30.0)
            odom_base = (np.eye(3), np.zeros(3))
        self.last = now
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).astype(np.float32) * 1e-3

        pts, _, _ = deproject(depth, self.k, self.stride)
        rc, tc = cam_base
        pb = pts @ rc.T + tc
        obstacle, floor = classify(pb, np.linalg.norm(pts, axis=1), self.p)

        # this frame's obstacle cells, in odom
        ro, to = odom_base
        ob = pb[obstacle]
        cur = cells(ob @ ro.T + to, self.cell, self.min_pixels)
        self.memory.update(cur, now)

        # forget remembered cells the camera now sees through
        keys, _ = self.memory.confirmed()
        stale = [c for c in keys if c not in cur]
        if stale:
            sp = np.array([self.memory.cells[c][:3] for c in stale])
            pb_m = (sp - to) @ ro                         # odom -> base
            pc_m = (pb_m - tc) @ rc                       # base -> optical
            gone = seen_through(pc_m, depth, self.k, self.see_through)
            self.memory.forget([c for c, g in zip(stale, gone) if g])
        self.memory.prune(now, self.memory_s, (to[0], to[1]), self.memory_radius)

        _, mem = self.memory.confirmed()
        mem_b = (mem - to) @ ro if len(mem) else np.zeros((0, 3))
        hdr = Header(stamp=msg.header.stamp, frame_id=self.base_frame)
        self.obs_pub.publish(point_cloud2.create_cloud_xyz32(hdr, mem_b.astype(np.float32)))
        fl = pb[floor][::8]
        self.floor_pub.publish(point_cloud2.create_cloud_xyz32(hdr, fl.astype(np.float32)))

        self.frames += 1
        if now - self.stats_t > 30.0:
            self.get_logger().info('%.1f Hz, %d obstacle cells' % (self.frames / (now - self.stats_t), len(mem_b)))
            self.stats_t, self.frames = now, 0


def main():
    rclpy.init()
    node = DepthObstacles()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
