"""Follow: camera (YOLO person boxes + clothes colour) says who, lidar keeps track of where.

    in : camera/color/image_raw, camera/color/camera_info, camera/depth/image_raw,
         camera/depth/camera_info, TF (odom <- base_footprint <- camera frames), c21/behavior_hold,
         scan + c21/depth_obstacles (things in the way: the car slides round them, still facing
         the person)
    out: cmd_vel_behavior, c21/behavior_state, c21/beep (on lock), c21/vision/detections (JSON
         person boxes per frame, drawn by the gateway's feature view), c21/vision/debug_image
         (annotated colour image, only while someone watches it)

The detector runs on the Jetson host (host/yolo_server.py, user service c21-yolo) because the ROS
container has no GPU; frames go over TCP on localhost. The person standing right in front when the
mode starts is locked on; it is tracked in the odom frame (the car's own turning is taken out) and
recognised again by its clothes colour after it was hidden. Between camera frames, out of the
picture at the side or too close for the camera, the lidar person blob nearest the track keeps it
going (see PersonTracker). Same node name / topics as the lidar
behaviours, so the gateway hold, APP status and arbiter apply unchanged.
"""
import json
import math
import socket
import struct
import threading
import time

import cv2
import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String, UInt8
from tf2_ros import Buffer, TransformException, TransformListener

from c21_behaviors.lidar_behavior import blobs, scan_points
from c21_behaviors.person_track import (PersonTracker, appearance, avoid_cmd, box_bearing, deproject,
                                        follow_cmd, person_position, project)


def quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


BOX_RGB = {'target': (0, 220, 0), 'pending': (255, 220, 0), 'person': (255, 255, 255)}


class DetectorClient:
    """Blocking client for host/yolo_server.py; reconnects on failure."""

    def __init__(self, host, port, timeout=1.0):
        self.addr, self.timeout = (host, port), timeout
        self.sock = None
        self.retry_at = 0.0

    def detect(self, rgb):
        if self.sock is None:
            if time.monotonic() < self.retry_at:
                return None
            try:
                self.sock = socket.create_connection(self.addr, timeout=self.timeout)
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.sock.settimeout(self.timeout * 10)      # first frame loads the model
            except OSError:
                self.sock, self.retry_at = None, time.monotonic() + 2.0
                return None
        h, w = rgb.shape[:2]
        try:
            self.sock.sendall(b'C21F' + struct.pack('>HH', w, h) + np.ascontiguousarray(rgb).tobytes())
            n = struct.unpack('>I', self._recv(4))[0]
            out = json.loads(self._recv(n))
            self.sock.settimeout(self.timeout)
            return out
        except (OSError, ValueError, ConnectionError):
            self.close()
            self.retry_at = time.monotonic() + 1.0
            return None

    def _recv(self, n):
        buf = bytearray()
        while len(buf) < n:
            c = self.sock.recv(n - len(buf))
            if not c:
                raise ConnectionError('closed')
            buf += c
        return bytes(buf)

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None


class VisionFollow(Node):
    def __init__(self):
        super().__init__('c21_behavior')
        dp = self.declare_parameter
        self.base_frame = dp('base_frame', 'base_footprint').value
        self.odom_frame = dp('odom_frame', 'odom').value
        self.min_conf = dp('vision_min_conf', 0.45).value
        self.p = {
            'acq_range': dp('vision_acquire_range', 2.5).value,     # lock on: within this, straight ahead
            'acq_fov': dp('vision_acquire_fov_deg', 20.0).value,
            'acq_frames': int(dp('vision_acquire_frames', 3).value),
            'gate': dp('vision_gate', 0.6).value,
            'walk': dp('vision_walk', 1.2).value,                    # m/s a person may move while unseen
            'gate_max': dp('vision_gate_max', 1.5).value,
            'max_hist': dp('vision_max_hist', 0.6).value,
            'reacq_hist': dp('vision_reacquire_hist', 0.4).value,
            'learn_hist': dp('vision_learn_hist', 0.35).value,
            'hist_rate': dp('vision_hist_rate', 0.1).value,
            'w_hist': dp('vision_w_hist', 1.5).value,
            'lost_s': dp('vision_lost_s', 1.0).value,
            'forget_s': dp('vision_forget_s', 20.0).value,
            'keep': dp('vision_keep', 1.0).value,                    # closer than ~1 m the camera sees legs only
            'deadband': dp('vision_deadband', 0.08).value,
            'kp_lin': dp('vision_kp_lin', 0.8).value,
            'kp_ang': dp('vision_kp_ang', 1.5).value,
            'speed': dp('vision_speed', 0.3).value,
            'back_speed': dp('vision_back_speed', 0.08).value,
            'max_w': dp('vision_max_w', 0.9).value,
            'hl': dp('robot_half_length', 0.15).value,
            'hw': dp('robot_half_width', 0.15).value,
            'av_margin': dp('vision_avoid_margin', 0.05).value,       # corridor half width = hw + this
            'av_clear': dp('vision_avoid_clear', 0.45).value,         # detour when the way is shorter
            'av_speed': dp('vision_avoid_speed', 0.15).value,
            'av_person_r': dp('vision_avoid_person_r', 0.4).value,    # points this near the target are the person
            'lidar_gate': dp('vision_lidar_gate', 0.3).value,         # lidar blob this near the track = target
            'lidar_lost_s': dp('vision_lidar_lost_s', 0.5).value,     # lidar only continues a track this fresh
            'doubt_s': dp('vision_doubt_s', 2.5).value,               # in the picture but unconfirmed this long
        }
        self.use_lidar = dp('vision_lidar', True).value
        self.lidar_range = dp('vision_lidar_range', 4.0).value
        # where the camera should see a whole person (for the doubt check)
        self.view = (math.radians(dp('vision_view_fov_deg', 35.0).value), dp('vision_view_min', 1.2).value,
                     dp('vision_view_max', 4.0).value)
        self.trk_lock = threading.Lock()
        self.avoid = dp('vision_avoid', True).value
        self.geo = (dp('scan_x', 0.0853).value, dp('scan_yaw', math.pi).value, dp('self_range', 0.14).value)
        self.scan_pts, self.depth_pts = (0.0, None), (0.0, None)
        self.side = 0
        self.search_s = dp('vision_search_s', 6.0).value
        self.search_w = dp('vision_search_w', 0.5).value
        self.det = DetectorClient('127.0.0.1', int(dp('vision_port', 5577).value))
        self.tracker = PersonTracker(self.p)

        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.lock = threading.Lock()
        self.color, self.depth, self.kc, self.kd = None, None, None, None
        self.new_frame = threading.Event()
        self.hold_until = 0.0
        self.last_frame = time.monotonic()
        self.status = {'behavior': 'vision_follow', 'state': 'wait', 'text': '等待相机画面', 'target': None}
        self.log_state, self.log_t = None, 0.0
        self.stats = [time.monotonic(), 0, 0.0, 0.0, 0.0]

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_behavior', 5)
        self.state_pub = self.create_publisher(String, 'c21/behavior_state', 5)
        self.beep_pub = self.create_publisher(UInt8, 'c21/beep', 5)
        self.dbg_pub = self.create_publisher(Image, 'c21/vision/debug_image', 2)
        self.det_pub = self.create_publisher(String, 'c21/vision/detections', 5)
        self.create_subscription(Image, 'camera/color/image_raw', self._on_color, qos_profile_sensor_data)
        self.create_subscription(Image, 'camera/depth/image_raw', self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, 'camera/color/camera_info', self._on_kc, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, 'camera/depth/camera_info', self._on_kd, qos_profile_sensor_data)
        self.create_subscription(Bool, 'c21/behavior_hold', self._on_hold, 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, 'c21/depth_obstacles', self._on_obst, 5)
        self.create_timer(0.2, self._tick)
        self.running = True
        self.worker = threading.Thread(target=self._loop, daemon=True)
        self.worker.start()
        self.get_logger().info('vision follow up: keep %.2f m' % self.p['keep'])

    # ---- callbacks (executor thread) ----
    def _on_color(self, msg):
        with self.lock:
            self.color = msg
        self.new_frame.set()

    def _on_depth(self, msg):
        with self.lock:
            self.depth = msg

    def _on_kc(self, msg):
        self.kc = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _on_kd(self, msg):
        self.kd = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _on_scan(self, msg):
        x, yaw, sr = self.geo
        now = time.monotonic()
        pts, idx = scan_points(msg.ranges, msg.angle_min, msg.angle_increment, x, yaw, sr,
                               self.p['hl'], self.p['hw'], with_idx=True)
        self.scan_pts = (now, pts)
        if not self.use_lidar or not self.tracker.locked:
            return
        odom = self._tf(self.odom_frame, self.base_frame, Time.from_msg(msg.header.stamp))
        if odom is None:
            return
        cents = [c for c in (pts[g].mean(axis=0) for g in blobs(pts, idx, len(msg.ranges)))
                 if math.hypot(*c) < self.lidar_range]
        ro, to = odom
        with self.trk_lock:
            self.tracker.update_lidar([ro[:2, :2] @ c + to[:2] for c in cents], now, to[:2])

    def _on_obst(self, msg):
        xyz = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        self.depth_pts = (time.monotonic(), np.asarray(xyz, dtype=np.float64).reshape(-1, 3)[:, :2])

    def _obstacles(self, now):
        """Fresh lidar + depth obstacle points (base xy), or None when neither is fresh."""
        out = [p for t, p in (self.scan_pts, self.depth_pts) if p is not None and now - t < 0.5]
        return np.concatenate(out) if out else None

    def _on_hold(self, msg):
        self.hold_until = time.monotonic() + 1.0 if msg.data else 0.0

    def _tick(self):
        if time.monotonic() - self.last_frame > 0.6:
            self._send(0.0, 0.0, 0.0)
            self.status = {**self.status, 'state': 'wait', 'text': '没有相机画面，已停车'}
        self.state_pub.publish(String(data=json.dumps(self.status, ensure_ascii=False)))

    # ---- worker thread ----
    def _tf(self, target, source, stamp):
        for t in (stamp, Time()):
            try:
                tf = self.buf.lookup_transform(target, source, t)
                tr = tf.transform.translation
                return quat_matrix(tf.transform.rotation), np.array([tr.x, tr.y, tr.z])
            except TransformException:
                continue
        return None

    def _loop(self):
        while self.running:
            if not self.new_frame.wait(0.5):
                continue
            self.new_frame.clear()
            with self.lock:
                cm, dm = self.color, self.depth
            if cm is None or self.kc is None:
                continue
            try:
                self._process(cm, dm)
            except Exception as e:  # noqa: BLE001 - keep the behaviour alive, but stopped
                if not self.running or not rclpy.ok():
                    return                                  # shutting down
                self._send(0.0, 0.0, 0.0)
                self.get_logger().error(f'vision frame failed: {e}', throttle_duration_sec=5.0)

    def _process(self, cm, dm):
        now = time.monotonic()
        self.last_frame = now
        if now < self.hold_until:
            self._send(0.0, 0.0, 0.0)
            self._set('paused', '与 APP 的连接中断，已暂停（连上后继续）', None, 0, 0, 0)
            return
        rgb = np.frombuffer(cm.data, np.uint8).reshape(cm.height, cm.width, 3)
        if cm.encoding == 'bgr8':
            rgb = rgb[:, :, ::-1]
        stamp = Time.from_msg(cm.header.stamp)
        cam = self._tf(self.base_frame, cm.header.frame_id, stamp)
        odom = self._tf(self.odom_frame, self.base_frame, stamp)
        if cam is None or odom is None:
            self._send(0.0, 0.0, 0.0)
            self._set('wait', '等待坐标变换（底盘控制板开机了吗？）', None, 0, 0, 0)
            return
        t0 = time.perf_counter()
        res = self.det.detect(rgb)
        t_det = time.perf_counter() - t0
        if res is None or 'error' in res:
            self._send(0.0, 0.0, 0.0)
            self._set('nodet', '视觉识别服务没有响应（主机 c21-yolo 服务），已停车', None, 0, 0, 0)
            return
        boxes = [b for b in res['boxes'] if b[4] >= self.min_conf and b[3] - b[1] > 40]

        # depth points (base frame) and where they land in the colour image
        uv = pb = None
        if dm is not None and self.kd is not None and abs((Time.from_msg(dm.header.stamp) - stamp).nanoseconds) < 2e8:
            dcam = self._tf(self.base_frame, dm.header.frame_id, Time.from_msg(dm.header.stamp))
            if dcam is not None:
                d = np.frombuffer(dm.data, np.uint16).reshape(dm.height, dm.width).astype(np.float32) * 1e-3
                d[d > 5.0] = 0.0
                pts = deproject(d, self.kd, 4)
                pb = pts @ dcam[0].T + dcam[1]
                uv = project((pb - cam[1]) @ cam[0], self.kc)

        ro, to = odom
        robot_xy = to[:2]
        yaw = math.atan2(ro[1, 0], ro[0, 0])
        dets = []
        for b in boxes:
            pos = person_position(b, uv, pb) if uv is not None else None
            dets.append({
                'box': b,
                'bearing': math.atan2(pos[1], pos[0]) if pos is not None else box_bearing(b, self.kc, cam[0]),
                'range': None if pos is None else float(math.hypot(*pos)),
                'pos': None if pos is None else (ro[:2, :2] @ pos + robot_xy),
                'hist': appearance(rgb, b),
            })
        # current odom -> base (the frame is ~0.1 s old; drive from the latest pose)
        cur = self._tf(self.odom_frame, self.base_frame, Time()) or odom
        with self.trk_lock:
            was_locked = self.tracker.locked
            chosen = self.tracker.update(dets, now, robot_xy, yaw)
            if self.tracker.locked and not was_locked:
                self.beep_pub.publish(UInt8(data=1))
                self.get_logger().info('locked on target')
            tb = None
            if self.tracker.target is not None:
                tb = cur[0][:2, :2].T @ (self.tracker.target - cur[1][:2])
            if tb is not None and self.tracker.check_view(now, self._in_view(tb)):
                self.get_logger().warn('camera does not confirm the lidar track: dropped')
            cmd, st, text = self._decide(tb, now)
        self._send(*cmd)
        self._set(st, text, tb, *cmd)
        marks = self._marks(dets, chosen)
        self.det_pub.publish(String(data=json.dumps({
            'w': int(cm.width), 'h': int(cm.height), 'state': st,
            'stamp': cm.header.stamp.sec + cm.header.stamp.nanosec * 1e-9,
            'boxes': [{'b': [round(float(v), 1) for v in d['box'][:4]], 'kind': kind, 'label': label}
                      for d, kind, label in marks]})))
        self._debug(rgb, marks)
        s = self.stats
        s[1] += 1
        s[2] += res.get('ms', 0.0)
        s[3] += t_det
        s[4] += time.monotonic() - now
        if now - s[0] > 30.0:
            k = 1000.0 / max(s[1], 1)
            self.get_logger().info('%.1f Hz, yolo %.0f ms (round trip %.0f ms), frame %.0f ms' % (
                s[1] / (now - s[0]), s[2] / max(s[1], 1), s[3] * k, s[4] * k))
            self.stats = [now, 0, 0.0, 0.0, 0.0]

    def _in_view(self, tb):
        fov, lo, hi = self.view
        return abs(math.atan2(tb[1], tb[0])) < fov and lo < math.hypot(*tb) < hi

    def _decide(self, tb, now):
        tr = self.tracker
        if not tr.locked:
            n = tr.pending_n
            if n:
                return (0.0, 0.0, 0.0), 'acquire', '发现前方有人，正在锁定…'
            return (0.0, 0.0, 0.0), 'lost', \
                f'等待目标：请站到车头正前方 {self.p["acq_range"]:.1f} m 以内'
        if tb is not None and tr.lost_for(now) < 0.4:          # seen by camera or lidar just now
            range_ok = now - tr.pos_seen < 0.4
            info = f'目标 {math.hypot(*tb):.2f} m，{math.degrees(math.atan2(tb[1], tb[0])):+.0f}°'
            if now - tr.vis_seen > 0.4:
                info += '（雷达）'
            if not self.avoid:
                return follow_cmd(tb, range_ok, self.p), 'track', '跟随中：' + info
            cmd, self.side, av = avoid_cmd(tb, range_ok, self._obstacles(now), self.side, self.p)
            if av == 'detour':
                way = {1: '从左边绕', -1: '从右边绕'}.get(self.side, '慢行')
                return cmd, 'detour', f'有障碍，{way}：' + info
            if av == 'blocked':
                return cmd, 'blocked', '前面被挡住，没有路绕过去，原地等待'
            return cmd, 'track', '跟随中：' + info
        self.side = 0
        lost = tr.lost_for(now)
        if lost < self.p['lost_s']:
            return (0.0, 0.0, 0.0), 'hold', '目标短暂丢失'
        if tb is not None and lost < self.p['lost_s'] + self.search_s:
            ang = math.atan2(tb[1], tb[0])
            if abs(ang) > math.radians(25):
                w = self.search_w if ang > 0 else -self.search_w
                return (0.0, 0.0, w), 'search', f'目标丢失，向{"左" if w > 0 else "右"}转寻找'
        return (0.0, 0.0, 0.0), 'lost', '目标丢失：请回到车前（会按衣服颜色认出你）'

    def _set(self, st, text, tb, vx, vy, wz):
        self.status = {'behavior': 'vision_follow', 'state': st, 'text': text,
                       'target': None if tb is None else [round(float(tb[0]), 2), round(float(tb[1]), 2)]}
        now = time.monotonic()
        moving = max(abs(vx), abs(vy), abs(wz)) > 1e-3
        if st != self.log_state or (moving and now - self.log_t > 0.5):
            self.log_state, self.log_t = st, now
            self.get_logger().info(f'{st}: {text} | cmd {vx:+.2f} {vy:+.2f} {wz:+.2f} tgt {self.status["target"]}')

    def _marks(self, dets, chosen):
        """(det, kind, label) per box: 'target' = confirmed track (green), 'pending' = being locked
        on (yellow), 'person' = just detected (white)."""
        tr, out = self.tracker, []
        for d in dets:
            rng = f'{d["range"]:.2f}m' if d['range'] is not None else '?m'
            if tr.locked and d is chosen:
                out.append((d, 'target', 'TARGET ' + rng))
            elif not tr.locked and d is tr.pending_det:
                out.append((d, 'pending', f'LOCKING {tr.pending_n}/{self.p["acq_frames"]} ' + rng))
            else:
                out.append((d, 'person', 'person ' + rng))
        return out

    def _debug(self, rgb, marks):
        if self.dbg_pub.get_subscription_count() == 0:
            return
        img = np.ascontiguousarray(rgb).copy()
        for d, kind, label in marks:
            x1, y1, x2, y2 = (int(v) for v in d['box'][:4])
            col = BOX_RGB[kind]
            thick = 1 if kind == 'person' else 3
            cv2.rectangle(img, (x1, y1), (x2, y2), col, thick)
            cv2.putText(img, label, (x1 + 3, max(14, y1 + 16)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col,
                        2 if thick > 1 else 1)
        st = self.status['state']
        cv2.putText(img, st, (8, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 220, 0), 2)
        m = Image()
        m.header.frame_id = 'camera_color_optical_frame'
        m.height, m.width = img.shape[:2]
        m.encoding, m.step = 'rgb8', img.shape[1] * 3
        m.data = img.tobytes()
        self.dbg_pub.publish(m)

    def _send(self, vx, vy, wz):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self.cmd_pub.publish(t)


def main():
    rclpy.init()
    node = VisionFollow()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.running = False
        node.worker.join(2.0)
        try:
            node._send(0.0, 0.0, 0.0)
        except Exception:
            pass
        node.det.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
