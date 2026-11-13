"""C21 APP gateway.

HTTP  :  /            APP web build (static_dir)
         /ws          WebSocket JSON control + state stream
         /video       MJPEG stream   ?src=color|depth|features|vision|<topic>&w=640&q=70&fps=15
         /snapshot    single JPEG    ?src=...
         /api/info    robot info / mode catalogue
         /map.png     latest occupancy grid (row 0 = max y), meta in state.map
         /api/maps    saved maps in maps_dir

WebSocket client -> gateway (all messages: {"t": <type>, ...}):
   hb                              heartbeat (send >= 5 Hz; lease = lease_s)
   cmd   vx vy wz                  teleop in m/s, rad/s (also renews lease)
   estop on                        true engages, false releases
   gimbal angle                    rad
   beep  n
   limits max_linear max_angular [scan_guard]
   mode  name on [args]
   map_save name                   write <maps_dir>/<name>.pgm/.yaml + slam_toolbox pose graph
   nav_plan id x y                 navigation mode: preview a route (ack carries path + length, no motion)
   nav_goal x y [yaw]              navigation mode: drive to a map pose (yaw omitted: face travel)
   nav_route_check id pts loop     navigation mode: plan every leg of a patrol (ack: path + length, no motion)
   nav_route pts loop              navigation mode: drive through pts [[x,y],...] in order (loop: forever)
   nav_home                        navigation mode: drive back to the map's home pose
   home_set [x y yaw]              set the map's home pose (default: where the robot is now)
   nav_cancel                      also ends a patrol
   init_pose x y yaw               navigation mode: tell AMCL where the robot is
   map_erase strokes               navigation mode: clear false obstacles along [[[x,y],...],...] in the saved map
   map_restore                     navigation mode: undo all erasing (back to the map as mapped)
gateway -> client:
   hello  info                     on connect
   state  {...}                    state_hz
   scan   pts [[x,y],...]          scan_hz, base frame, metres
   ack    req ok msg               reply to mode/limits/map_save
"""
import asyncio
import json
import math
import os
import re
import threading
import time

import cv2
import numpy as np
import rclpy
from aiohttp import WSMsgType, web
from cv_bridge import CvBridge
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Image, LaserScan
from std_msgs.msg import Bool, Float32, String, UInt8
from tf2_ros import Buffer, TransformListener

from c21_gateway import map_edit
from c21_gateway.modes import ModeManager

VIDEO_ALIASES = {
    'color': '/camera/color/image_raw',
    'depth': '/camera/depth/image_raw',
    'vision': '/c21/vision/debug_image',
    'features': '/camera/color/image_raw#features',
}
FEATURE_SUFFIX = '#features'
BOX_BGR = {'target': (0, 220, 0), 'pending': (0, 220, 255), 'person': (255, 255, 255)}


class ImageTap:
    """Lazily subscribed image topic -> latest JPEG."""

    def __init__(self, node, topic):
        self.node, self.topic = node, topic
        self.sub = None
        self.users = 0
        self.jpeg = None
        self.seq = 0
        self.stamp = 0.0
        self.width = 640
        self.quality = 70
        self.min_dt = 1.0 / 15
        self._last = 0.0
        self._bridge = CvBridge()
        self._lock = threading.Lock()

    def acquire(self, width, quality, fps):
        with self._lock:
            self.users += 1
            self.width, self.quality = width, quality
            self.min_dt = 1.0 / max(1.0, fps)
            if self.sub is None:
                self.sub = self.node.create_subscription(Image, self.topic, self._on_img, qos_profile_sensor_data)

    def release(self):
        with self._lock:
            self.users -= 1
            if self.users <= 0 and self.sub is not None:
                self.node.destroy_subscription(self.sub)
                self.sub = None
                self.users = 0

    def _on_img(self, msg):
        now = time.monotonic()
        if now - self._last < self.min_dt:
            return
        self._last = now
        try:
            img = self._convert(msg)
            h, w = img.shape[:2]
            if w > self.width:
                img = cv2.resize(img, (self.width, int(h * self.width / w)), interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, int(self.quality)])
            if ok:
                self.jpeg = buf.tobytes()
                self.seq += 1
                self.stamp = now
        except Exception as e:  # noqa: BLE001 - never kill the executor on a bad frame
            self.node.get_logger().warn(f'image convert failed on {self.topic}: {e}',
                                        throttle_duration_sec=5.0)

    def _convert(self, msg):
        """Image msg -> BGR image to send."""
        if msg.encoding in ('16UC1', 'mono16'):
            d = self._bridge.imgmsg_to_cv2(msg, 'passthrough').astype(np.float32)
            d = np.clip(d / 4000.0 * 255.0, 0, 255).astype(np.uint8)   # 0..4 m
            img = cv2.applyColorMap(255 - d, cv2.COLORMAP_JET)
            img[d == 0] = 0
        else:
            img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        return img


class FeatureTap(ImageTap):
    """Colour image with ORB feature points drawn on it, plus the vision-follow person boxes
    (c21/vision/detections) while that mode runs: white = person seen, yellow = being locked on,
    green = the confirmed target."""

    def __init__(self, node, topic):
        super().__init__(node, topic)
        self._orb = cv2.ORB_create(nfeatures=400, fastThreshold=12)

    def _convert(self, msg):
        img = self._bridge.imgmsg_to_cv2(msg, 'bgr8').copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        kps = self._orb.detect(gray, None)
        for k in kps:
            x, y = int(k.pt[0]), int(k.pt[1])
            cv2.circle(img, (x, y), max(2, int(k.size / 12)), (255, 0, 255), 1, cv2.LINE_AA)
            cv2.circle(img, (x, y), 1, (255, 0, 255), -1)
        cv2.putText(img, f'ORB {len(kps)}', (img.shape[1] - 110, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 0, 255), 2)
        det = self.node.vision_dets
        if det is not None and time.monotonic() - det[0] < 0.6:
            d = det[1]
            sx, sy = img.shape[1] / max(1, d.get('w', img.shape[1])), img.shape[0] / max(1, d.get('h', img.shape[0]))
            for b in d.get('boxes', []):
                x1, y1, x2, y2 = b['b']
                p1, p2 = (int(x1 * sx), int(y1 * sy)), (int(x2 * sx), int(y2 * sy))
                kind = b.get('kind', 'person')
                col = BOX_BGR.get(kind, BOX_BGR['person'])
                thick = 1 if kind == 'person' else 3
                cv2.rectangle(img, p1, p2, col, thick)
                cv2.putText(img, b.get('label', ''), (p1[0] + 3, max(14, p1[1] + 16)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2 if thick > 1 else 1)
        return img


class GatewayNode(Node):
    def __init__(self):
        super().__init__('c21_gateway')
        dp = self.declare_parameter
        self.port = dp('port', 8080).value
        self.static_dir = dp('static_dir', '/ws/app_dist').value
        self.modes_file = dp('modes_file', '').value
        self.lease_s = dp('lease_s', 0.5).value
        self.state_hz = dp('state_hz', 5.0).value
        self.scan_hz = dp('scan_hz', 5.0).value
        self.scan_yaw = dp('scan_yaw', math.pi).value
        self.scan_x = dp('scan_x', 0.0).value
        self.token = dp('token', '').value
        self.robot_name = dp('robot_name', 'C21').value
        self.maps_dir = dp('maps_dir', '/root/c21_maps').value

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_app', 10)
        self.estop_pub = self.create_publisher(Bool, 'c21/estop', 10)
        self.gimbal_pub = self.create_publisher(Float32, 'c21/gimbal', 10)
        self.beep_pub = self.create_publisher(UInt8, 'c21/beep', 10)
        # lidar behaviours stand still (but keep their state) while this says True; refreshed at 5 Hz
        self.hold_pub = self.create_publisher(Bool, 'c21/behavior_hold', 10)
        self.hold_since, self._hold_pub_t = None, 0.0

        self.base_state = {}
        self.safety_state = {}
        self.behavior, self._behavior_t = None, 0.0
        self.battery = None
        self.odom = None
        self.scan_pts = []
        self.estop = False
        self.gimbal = 0.0
        self.create_subscription(String, 'c21/base_state', lambda m: self._json(m, 'base_state'), 5)
        self.create_subscription(String, 'c21/safety_state', lambda m: self._json(m, 'safety_state'), 5)
        self.create_subscription(String, 'c21/behavior_state', self._on_behavior, 5)
        self.vision_dets = None          # (monotonic, msg dict) from vision_follow
        self.create_subscription(String, 'c21/vision/detections', self._on_vision_dets, 5)
        self.create_subscription(BatteryState, 'battery', self._on_battery, 5)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        # track e-stop from every source (APP, scripts, future hardware button / voice)
        self.create_subscription(Bool, 'c21/estop', self._on_estop, 10)
        self.param_cli = self.create_client(SetParameters, '/c21_safety/set_parameters')

        # SLAM map + robot pose in the map frame (only while a mapping/navigation mode runs)
        self.grid = None
        self.map_seq = 0
        self._png = (None, None)          # (seq, bytes)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, latched)
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.serialize_cli = None
        self.load_map_cli = None

        # navigation (Nav2 NavigateToPose); goals are cancelled on e-stop, joystick or APP loss
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.plan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.preview = None              # (x, y, path) of the last route preview
        self.route_snap = None           # (pts, loop, reachable leg ends) of the last patrol check
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 5)
        self.create_subscription(Path, 'plan', self._on_plan, 5)
        self.nav = {'status': 'idle'}
        self.path = []
        self._goal_handle = None
        self._init_state = None          # None | ('wait', t) | 'sent'
        self.route = None                # patrol: {'pts', 'i', 'loop', 'lap', 'next_at'}
        self.home = None                 # {'x', 'y', 'yaw', 'set'} of the running map
        self.paused = None               # link-loss pause: {'goal', 'kind', 'route', 'since'}
        self.create_timer(0.5, self._nav_tick)
        self.create_timer(0.5, self._estop_refresh)

        self.taps = {}
        self.modes = ModeManager(self.modes_file, self.get_logger())

    # ---- ROS callbacks ----
    def _json(self, msg, attr):
        try:
            setattr(self, attr, json.loads(msg.data))
        except ValueError:
            pass

    def _on_vision_dets(self, msg):
        try:
            self.vision_dets = (time.monotonic(), json.loads(msg.data))
        except ValueError:
            pass

    def _on_behavior(self, msg):
        try:
            self.behavior, self._behavior_t = json.loads(msg.data), time.monotonic()
        except ValueError:
            pass

    def _on_battery(self, msg):
        self.battery = {'v': round(msg.voltage, 2), 'pct': round(msg.percentage * 100)}

    def _on_odom(self, msg):
        p, q, tw = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = {'x': round(p.x, 3), 'y': round(p.y, 3), 'yaw': round(yaw, 3),
                     'vx': round(tw.linear.x, 3), 'vy': round(tw.linear.y, 3), 'wz': round(tw.angular.z, 3)}

    def _on_scan(self, msg):
        c, s = math.cos(self.scan_yaw), math.sin(self.scan_yaw)
        n = len(msg.ranges)
        step = max(1, n // 360)
        pts = []
        for i in range(0, n, step):
            r = msg.ranges[i]
            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                a = msg.angle_min + i * msg.angle_increment
                lx, ly = r * math.cos(a), r * math.sin(a)
                pts.append([round(self.scan_x + lx * c - ly * s, 3), round(lx * s + ly * c, 3)])
        self.scan_pts = pts

    def _on_map(self, msg):
        self.grid = msg
        self.map_seq += 1

    def map_meta(self):
        if self.grid is not None and not self.modes.status()['running']:
            self.grid = None              # SLAM / localization stopped: drop the stale map
        g = self.grid
        if g is None:
            return None
        i = g.info
        return {'seq': self.map_seq, 'res': round(i.resolution, 4), 'w': i.width, 'h': i.height,
                'ox': round(i.origin.position.x, 3), 'oy': round(i.origin.position.y, 3)}

    def map_image(self):
        """Grid -> 8-bit image, map_server colours (free 254, occupied 0, unknown 205), row 0 = top."""
        g = self.grid
        d = np.asarray(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)
        img = np.full(d.shape, 205, np.uint8)
        img[(d >= 0) & (d <= 25)] = 254
        img[d >= 65] = 0
        return np.ascontiguousarray(img[::-1])

    def map_png(self):
        if self.grid is None:
            return None
        seq = self.map_seq
        if self._png[0] != seq:
            ok, buf = cv2.imencode('.png', self.map_image())
            self._png = (seq, buf.tobytes() if ok else None)
        return self._png[1]

    def map_pose(self):
        if self.grid is None:
            return None
        try:
            t = self.tf_buf.lookup_transform('map', 'base_footprint', Time())
        except Exception:  # noqa: BLE001 - no SLAM running / not yet localized
            return None
        q, p = t.transform.rotation, t.transform.translation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return {'x': round(p.x, 3), 'y': round(p.y, 3), 'yaw': round(yaw, 3)}

    # ---- navigation ----
    def nav_map(self):
        a = self.modes.args_of('navigation')
        return a.get('map') if a else None

    def _pose_file(self, name):
        return os.path.join(self.maps_dir, f'{name}.pose.json')

    def _remember_pose(self, name, pose):
        try:
            with open(self._pose_file(name), 'w', encoding='utf-8') as f:
                json.dump(pose, f)
        except OSError:
            pass

    def _home_file(self, name):
        return os.path.join(self.maps_dir, f'{name}.home.json')

    def _load_home(self, name):
        try:
            with open(self._home_file(name), encoding='utf-8') as f:
                h = json.load(f)
            return {'x': float(h['x']), 'y': float(h['y']), 'yaw': float(h['yaw']), 'set': True}
        except (OSError, ValueError, KeyError, TypeError):
            return {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'set': False}   # where mapping started

    def home_set(self, x=None, y=None, yaw=None):
        name = self.nav_map()
        if name is None:
            return False, '先启动导航模式'
        if x is None:
            p = self.map_pose()
            if p is None:
                return False, '还没有定位'
            x, y, yaw = p['x'], p['y'], p['yaw']
        self.home = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3), 'set': True}
        try:
            with open(self._home_file(name), 'w', encoding='utf-8') as f:
                json.dump(self.home, f)
        except OSError:
            return False, '保存失败'
        return True, '已把当前位置设为起点'

    def nav_home(self):
        if self.nav_map() is None:
            return False, '先启动导航模式'
        h = self.home or self._load_home(self.nav_map())
        ok, m = self.nav_goal(h['x'], h['y'], h['yaw'])
        if ok:
            self.nav['kind'] = 'home'
            m = '正在回起点'
        return ok, m

    def _route_yaw(self, i):
        """Arrive at waypoint i facing the next one (saves a turn); the last one faces along its leg."""
        r = self.route
        pts, n = r['goals'], len(r['goals'])
        if i + 1 < n or (r['loop'] and n > 1):
            a, b = pts[i], pts[(i + 1) % n]
        elif i > 0:
            a, b = pts[i - 1], pts[i]
        else:
            p = self.map_pose() or {'x': 0.0, 'y': 0.0}
            a, b = (p['x'], p['y']), pts[i]
        return math.atan2(b[1] - a[1], b[0] - a[0])

    def nav_route(self, pts, loop):
        if self.nav_map() is None:
            return False, '先启动导航模式'
        loop = bool(loop) and len(pts) > 1
        snap = self.route_snap
        # drive to the reachable leg ends found by the check (a waypoint inside a wall's inflation
        # would otherwise never satisfy the goal checker)
        goals = snap[2] if snap and snap[0] == pts and snap[1] == loop else pts
        self.route = {'pts': pts, 'goals': goals, 'i': 0, 'loop': loop, 'lap': 1, 'next_at': None}
        return self._route_go()

    def _route_go(self):
        r = self.route
        x, y = r['goals'][r['i']]
        ok, m = self.nav_goal(x, y, self._route_yaw(r['i']), keep_route=True)
        if not ok:
            self.route = None
        return ok, (f'开始巡航，共 {len(r["pts"])} 个点' if ok else m)

    def _route_info(self):
        r = self.route
        if r is None:
            return None
        return {'pts': r['pts'], 'i': r['i'], 'n': len(r['pts']), 'loop': r['loop'], 'lap': r['lap'],
                'waiting': r['next_at'] is not None}

    def _nav_tick(self):
        name = self.nav_map()
        if name is None:
            self._init_state = None
            self.route = self.home = self.paused = None
            if self.nav['status'] != 'idle':
                self.nav, self.path, self._goal_handle = {'status': 'idle'}, [], None
            return
        if self.home is None:
            self.home = self._load_home(name)
        if self.paused is not None and time.monotonic() - self.paused['since'] > self.PAUSE_S:
            self.paused = None
            self.nav = {'status': 'canceled', 'goal': self.nav.get('goal'), 'route_end': '网络中断太久，已取消'}
        r = self.route
        if r is not None and r['next_at'] is not None and time.monotonic() >= r['next_at']:
            r['next_at'] = None
            if self.estop:
                self.route = None
            else:
                self._route_go()
        # hand AMCL the remembered pose once it is up (1 s after it subscribes, map loaded)
        if self._init_state is None and self.count_subscribers('initialpose') > 0:
            self._init_state = ('wait', time.monotonic())
        elif isinstance(self._init_state, tuple) and time.monotonic() - self._init_state[1] > 1.0:
            pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            try:
                with open(self._pose_file(name), encoding='utf-8') as f:
                    pose = json.load(f)
            except (OSError, ValueError):
                pass
            self.set_init_pose(pose['x'], pose['y'], pose['yaw'], remember=False)
            self.get_logger().info(f'initial pose for {name}: {pose}')
            self._init_state = 'sent'
        elif self._init_state == 'sent':
            pose = self.map_pose()
            if pose is not None:
                self._remember_pose(name, pose)

    def set_init_pose(self, x, y, yaw, remember=True):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        # stamp left at 0: AMCL then uses the latest odom TF instead of extrapolating
        m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.15 ** 2
        cov[35] = 0.2 ** 2
        m.pose.covariance = cov
        self.init_pub.publish(m)
        name = self.nav_map()
        if remember and name:
            self._remember_pose(name, {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)})

    def nav_goal(self, x, y, yaw=None, keep_route=False):
        if self.nav_map() is None:
            return False, '先启动导航模式'
        if self.modes.running_motion():
            return False, '先停止雷达功能再导航'
        if not keep_route:
            self.route = self.paused = None
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            return False, '导航还在启动，请稍等'
        if yaw is None:
            yaw = self._arrival_yaw(x, y)
        pv = self.preview
        if not keep_route and pv and math.hypot(pv[0] - x, pv[1] - y) < 0.05 and pv[2]:
            x, y = pv[2][-1]             # previewed path end = nearest reachable spot
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        target = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)}
        self._goal_handle = None         # Nav2 preempts the old goal; ignore its result
        self.nav = {'status': 'pending', 'goal': target}
        fut = self.nav_client.send_goal_async(goal, feedback_callback=self._on_nav_feedback)
        fut.add_done_callback(lambda f, t=target: self._on_goal_response(f, t))
        return True, '已发送目标'

    def _arrival_yaw(self, x, y):
        """Face the way the previewed route arrives; else the straight line from the robot."""
        pv = self.preview
        if pv and math.hypot(pv[0] - x, pv[1] - y) < 0.05 and len(pv[2]) >= 2:
            ex, ey = pv[2][-1]
            for bx, by in reversed(pv[2][:-1]):
                if math.hypot(ex - bx, ey - by) >= 0.3:
                    break
            return math.atan2(ey - by, ex - bx)
        p = self.map_pose() or {'x': 0.0, 'y': 0.0}
        return math.atan2(y - p['y'], x - p['x'])

    def _plan_check(self):
        if self.nav_map() is None:
            return '先启动导航模式'
        if self._goal_handle is not None or self.route is not None:
            return '正在导航中，先取消或等它到达'
        if not self.plan_client.wait_for_server(timeout_sec=1.0):
            return '导航还在启动，请稍等'
        return None

    def nav_plan(self, x, y):
        """Blocking (call from a worker thread): plan robot -> (x, y) with the planner server only."""
        err = self._plan_check()
        if err:
            return False, err, None
        poses = self._compute_path(x, y)
        if poses is None:
            return False, '规划超时，请重试', None
        if not poses:
            return False, '到不了这个点（在墙里、未知区域，或被挡住）', None
        pts = self._thin(poses)
        length = self._length(pts)
        self.preview = (x, y, pts)
        return True, f'路线 {length:.1f} m', {'path': pts, 'len': round(length, 2)}

    def nav_route_check(self, pts, loop):
        """Blocking: plan robot -> p0 -> p1 ... (-> p0 when looping); no motion."""
        err = self._plan_check()
        if err:
            return False, err, None
        legs = [(None, pts[0])] + list(zip(pts, pts[1:]))
        if loop and len(pts) > 1:
            legs.append((pts[-1], pts[0]))
        path, bad, end, ends = [], [], None, []
        for k, (a, b) in enumerate(legs):
            # start each leg where the previous one really ends: the planner stops at the nearest free
            # cell (tolerance 0.3 m) when a waypoint sits in a wall's inflation, so would the car
            poses = self._compute_path(b[0], b[1], end if end is not None else a)
            if not poses:
                bad.append(k)
                end = b
                continue
            e = poses[-1].pose.position
            end = (e.x, e.y)
            ends.append([round(e.x, 3), round(e.y, 3)])
            path += self._thin(poses, step=0.15, cap=200)
        if bad:
            names = ['当前位置→1' if k == 0 else (f'{k}→{k + 1}' if k < len(pts) else f'{len(pts)}→1') for k in bad]
            return False, '这几段走不通：' + '、'.join(names), {'bad': bad}
        self.route_snap = (pts, bool(loop), ends[:len(pts)])
        length = self._length(path)
        return True, f'整圈 {length:.1f} m' if loop else f'全程 {length:.1f} m', {'path': path, 'len': round(length, 2)}

    @staticmethod
    def _length(pts):
        return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))

    def _compute_path(self, x, y, start=None):
        """Blocking planner call; [] = no path, None = timeout."""
        g = ComputePathToPose.Goal()
        g.goal.header.frame_id = 'map'
        g.goal.pose.position.x, g.goal.pose.position.y = float(x), float(y)
        g.goal.pose.orientation.w = 1.0
        g.planner_id = 'GridBased'
        g.use_start = start is not None
        if start is not None:
            g.start.header.frame_id = 'map'
            g.start.pose.position.x, g.start.pose.position.y = float(start[0]), float(start[1])
            g.start.pose.orientation.w = 1.0
        done, box = threading.Event(), {}

        def on_goal(f):
            gh = f.result()
            if gh is None or not gh.accepted:
                done.set()
                return
            gh.get_result_async().add_done_callback(lambda r: (box.setdefault('r', r.result()), done.set()))
        self.plan_client.send_goal_async(g).add_done_callback(on_goal)
        if not done.wait(5.0):
            return None
        r = box.get('r')
        if r is None or r.status != GoalStatus.STATUS_SUCCEEDED:
            return []
        return list(r.result.path.poses)

    def _on_goal_response(self, fut, target):
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.nav = {'status': 'rejected', 'goal': target}
            self.route = None
            return
        if self.nav.get('status') != 'pending' or self.nav.get('goal') is not target:
            gh.cancel_goal_async()       # cancelled (or replaced) before Nav2 accepted it
            return
        self._goal_handle = gh
        self.nav = {'status': 'active', 'goal': target, 'kind': self.nav.get('kind')}
        gh.get_result_async().add_done_callback(lambda f, g=gh: self._on_nav_result(f, g))

    def _on_nav_feedback(self, msg):
        if self.nav.get('status') == 'active':
            self.nav['dist'] = round(msg.feedback.distance_remaining, 2)
            self.nav['retries'] = int(msg.feedback.number_of_recoveries)

    def _on_nav_result(self, fut, gh):
        if gh is not self._goal_handle:
            return                       # superseded by a newer goal
        code = fut.result().status
        st = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled',
              GoalStatus.STATUS_ABORTED: 'failed'}.get(code, 'failed')
        self.nav = {'status': st, 'goal': self.nav.get('goal'), 'kind': self.nav.get('kind')}
        self._goal_handle = None
        self.path = []
        r = self.route
        if r is None:
            return
        if st != 'succeeded':
            self.nav['route_end'] = f'巡航在第 {r["i"] + 1} 个点{"取消" if st == "canceled" else "失败"}'
            self.route = None
            return
        if r['i'] + 1 < len(r['pts']) or r['loop']:
            r['i'] = (r['i'] + 1) % len(r['pts'])
            r['lap'] += r['i'] == 0
            r['next_at'] = time.monotonic() + 1.0      # short stop at each waypoint
            self.nav['status'] = 'waypoint'
        else:
            self.nav['route_end'] = f'巡航完成，共 {len(r["pts"])} 个点'
            self.route = None

    def nav_busy(self):
        return self._goal_handle is not None or self.route is not None or self.nav.get('status') == 'pending'

    PAUSE_S = 15.0

    def nav_pause(self, why=''):
        """APP link lost: stop now but keep the goal / patrol, resume if the link is back within PAUSE_S
        (campus WiFi roaming drops the link for ~5 s). Nothing moves while no APP is connected."""
        if not self.nav_busy() or self.paused is not None:
            return
        r = self.route
        if r is not None:
            r = dict(r, next_at=None)
        self.paused = {'goal': self.nav.get('goal'), 'kind': self.nav.get('kind'), 'route': r,
                       'since': time.monotonic()}
        gh, self._goal_handle, self.route = self._goal_handle, None, None
        if gh is not None:
            gh.cancel_goal_async()
        self.nav = {'status': 'paused', 'goal': self.paused['goal'], 'kind': self.paused['kind']}
        self.get_logger().warn(f'nav paused {why}')

    def behavior_hold(self, why=''):
        """APP link lost while a lidar behaviour runs: hold it (stopped, state kept) and keep it held.
        Returns True once the hold is older than PAUSE_S, when the caller should stop the mode."""
        if not self.modes.running_motion():
            self.hold_since = None
            return False
        now = time.monotonic()
        if self.hold_since is None:
            self.hold_since = now
            self.get_logger().warn(f'lidar behaviour held {why}')
        if now - self._hold_pub_t >= 0.2:
            self._hold_pub_t = now
            self.hold_pub.publish(Bool(data=True))
        if now - self.hold_since > self.PAUSE_S:
            self.hold_since = None
            return True
        return False

    def behavior_release(self):
        if self.hold_since is None:
            return
        self.hold_since = None
        self.hold_pub.publish(Bool(data=False))
        self.get_logger().info('APP link back: lidar behaviour resumed')

    def nav_resume(self):
        pz, self.paused = self.paused, None
        if pz is None:
            return
        if self.estop or time.monotonic() - pz['since'] > self.PAUSE_S:
            self.nav = {'status': 'canceled', 'goal': pz['goal'], 'route_end': '网络中断太久，已取消'}
            return
        self.get_logger().info('APP link back: resuming nav')
        if pz['route'] is not None:
            self.route = pz['route']
            self._route_go()
        elif pz['goal'] is not None:
            g = pz['goal']
            self.nav_goal(g['x'], g['y'], g['yaw'], keep_route=True)
            self.nav['kind'] = pz['kind']

    def nav_cancel(self, why=''):
        if self.paused is not None:
            self.paused = None
            self.nav = {'status': 'canceled', 'goal': self.nav.get('goal')}
        busy = self.nav_busy()
        if self.route is not None and self._goal_handle is None:
            self.nav = {'status': 'canceled', 'goal': self.nav.get('goal'),
                        'route_end': f'巡航在第 {self.route["i"] + 1} 个点前取消'}
        self.route = None
        if self.nav.get('status') == 'pending':
            self.nav = {'status': 'canceled', 'goal': self.nav.get('goal')}
        gh = self._goal_handle
        if gh is not None:
            gh.cancel_goal_async()
        if busy:
            self.get_logger().info(f'nav goal cancelled {why}')
        return busy

    @staticmethod
    def _thin(poses, step=0.1, cap=300):
        pts, last = [], None
        for ps in poses:
            x, y = ps.pose.position.x, ps.pose.position.y
            if last is None or math.hypot(x - last[0], y - last[1]) >= step:
                pts.append([round(x, 2), round(y, 2)])
                last = (x, y)
        if poses:
            e = poses[-1].pose.position
            if pts[-1] != [round(e.x, 2), round(e.y, 2)]:
                pts.append([round(e.x, 2), round(e.y, 2)])
        return pts[:cap]

    def _on_plan(self, msg):
        self.path = self._thin(msg.poses)

    def nav_state(self):
        if self.nav_map() is None:
            return None
        d = dict(self.nav)
        if d['status'] in ('active', 'pending'):
            d['path'] = self.path
        d['route'] = self._route_info()
        d['home'] = self.home
        return d

    def list_maps(self):
        if not os.path.isdir(self.maps_dir):
            return []
        out = []
        for f in sorted(os.listdir(self.maps_dir)):
            if f.endswith('.yaml'):
                full = os.path.join(self.maps_dir, f)
                out.append({'name': f[:-5], 'mtime': int(os.path.getmtime(full)),
                            'posegraph': os.path.exists(full[:-5] + '.posegraph')})
        return out

    def save_map(self, name):
        """Write map_server pgm/yaml from the live grid; ask slam_toolbox to serialize its pose graph."""
        if self.grid is None:
            return False, '没有地图（先启动建图）'
        name = name or time.strftime('map_%Y%m%d_%H%M')
        if not re.fullmatch(r'[\w\-]{1,40}', name):
            return False, '名称只能用字母、数字、汉字、_ 和 -'
        os.makedirs(self.maps_dir, exist_ok=True)
        base = os.path.join(self.maps_dir, name)
        i = self.grid.info
        img = self.map_image()
        with open(base + '.pgm', 'wb') as f:
            f.write(b'P5\n%d %d\n255\n' % (img.shape[1], img.shape[0]))
            f.write(img.tobytes())
        with open(base + '.yaml', 'w', encoding='utf-8') as f:
            f.write(f'image: {name}.pgm\nmode: trinary\nresolution: {i.resolution:.4f}\n'
                    f'origin: [{i.origin.position.x:.4f}, {i.origin.position.y:.4f}, 0.0]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
        graph = self._serialize(base)
        pose = self.map_pose()
        if pose is not None:
            self._remember_pose(name, pose)
        msg = f'已保存 {name}（{i.width * i.resolution:.1f} × {i.height * i.resolution:.1f} m）'
        return True, msg + ('' if graph else '；位姿图未保存')

    ERASE_R = 0.15      # eraser radius, m

    def map_erase(self, strokes):
        """Erase false obstacles from the running navigation map (pgm on disk) and reload it.

        The first edit keeps the map as mapped in <name>.orig.pgm (map_restore puts it back)."""
        name = self.nav_map()
        if name is None:
            return False, '先启动导航'
        if self.nav_busy():
            return False, '导航进行中，先取消'
        base = os.path.join(self.maps_dir, name)
        try:
            meta = self._map_yaml(base + '.yaml')
            img = map_edit.read_pgm(base + '.pgm')
            if not os.path.exists(base + '.orig.pgm'):
                map_edit.write_pgm(base + '.orig.pgm', img)
            n = map_edit.erase(img, strokes, meta['resolution'], meta['origin'], self.ERASE_R)
            if n:
                map_edit.write_pgm(base + '.pgm', img)
        except (OSError, ValueError, KeyError) as e:
            return False, f'擦除失败：{e}'
        if not n:
            return True, '这里没有障碍可擦'
        return self._reload_map(base, f'已擦除 {n} 格')

    def map_restore(self):
        name = self.nav_map()
        if name is None:
            return False, '先启动导航'
        if self.nav_busy():
            return False, '导航进行中，先取消'
        base = os.path.join(self.maps_dir, name)
        if not os.path.exists(base + '.orig.pgm'):
            return True, '地图没有被擦过'
        try:
            os.replace(base + '.orig.pgm', base + '.pgm')
        except OSError as e:
            return False, f'恢复失败：{e}'
        return self._reload_map(base, '已恢复原图')

    @staticmethod
    def _map_yaml(path):
        meta = {}
        with open(path, encoding='utf-8') as f:
            for line in f:
                k, _, v = line.partition(':')
                if k.strip() == 'resolution':
                    meta['resolution'] = float(v)
                elif k.strip() == 'origin':
                    meta['origin'] = [float(x) for x in v.strip().strip('[]').split(',')[:2]]
        return meta

    def _reload_map(self, base, msg):
        """map_server re-reads the yaml/pgm and republishes; AMCL and the global costmap follow.
        AMCL is then re-seeded at the current pose (it keeps no particles across a new map)."""
        from nav2_msgs.srv import LoadMap
        if self.load_map_cli is None:
            self.load_map_cli = self.create_client(LoadMap, '/map_server/load_map')
        if not self.load_map_cli.wait_for_service(timeout_sec=1.0):
            return False, msg + '，但地图服务没响应（下次启动导航时生效）'
        pose = self.map_pose()
        req = LoadMap.Request()
        req.map_url = base + '.yaml'
        fut = self.load_map_cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 5.0:
            time.sleep(0.05)
        if not fut.done() or fut.result() is None or fut.result().result != 0:
            return False, msg + '，但重新加载失败（下次启动导航时生效）'
        if pose is not None:
            time.sleep(0.5)
            self.set_init_pose(pose['x'], pose['y'], pose['yaw'])
        return True, msg

    def _serialize(self, base):
        from slam_toolbox.srv import SerializePoseGraph
        if self.serialize_cli is None:
            self.serialize_cli = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        if not self.serialize_cli.wait_for_service(timeout_sec=1.0):
            return False
        req = SerializePoseGraph.Request()
        req.filename = base
        fut = self.serialize_cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 10.0:
            time.sleep(0.05)
        return fut.done() and fut.result() is not None and fut.result().result == 0

    def _on_estop(self, msg):
        self.estop = bool(msg.data)

    def _estop_refresh(self):
        # Re-assert an engaged e-stop so a restarted arbiter picks it up. Never re-assert
        # "released": that would silently cancel an e-stop raised by another source.
        if self.estop:
            self.estop_pub.publish(Bool(data=True))

    # ---- actions ----
    def send_cmd(self, vx, vy, wz):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self.cmd_pub.publish(t)

    def set_estop(self, on):
        self.estop = bool(on)
        self.estop_pub.publish(Bool(data=self.estop))
        if self.estop:
            self.send_cmd(0, 0, 0)

    def set_gimbal(self, angle):
        self.gimbal = float(angle)
        self.gimbal_pub.publish(Float32(data=self.gimbal))

    def set_limits(self, **kw):
        if not self.param_cli.service_is_ready():
            return None
        req = SetParameters.Request()
        for k, v in kw.items():
            if isinstance(v, bool):
                pv = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=v)
            else:
                pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v))
            req.parameters.append(Parameter(name=k, value=pv))
        return self.param_cli.call_async(req)

    def tap(self, topic):
        if topic not in self.taps:
            if topic.endswith(FEATURE_SUFFIX):
                self.taps[topic] = FeatureTap(self, topic[:-len(FEATURE_SUFFIX)])
            else:
                self.taps[topic] = ImageTap(self, topic)
        return self.taps[topic]

    def info(self):
        return {'name': self.robot_name, 'modes': self.modes.catalogue(),
                'video': list(VIDEO_ALIASES), 'lease_s': self.lease_s}

    def state(self):
        return {'t': 'state', 'ts': time.time(), 'base': self.base_state, 'safety': self.safety_state,
                'battery': self.battery, 'odom': self.odom, 'estop': self.estop, 'gimbal': self.gimbal,
                'modes': self.modes.status(), 'map': self.map_meta(), 'pose': self.map_pose(),
                'nav': self.nav_state(),
                'behavior': self.behavior if time.monotonic() - self._behavior_t < 1.5 else None}


class Server:
    def __init__(self, node: GatewayNode):
        self.node = node
        self.clients = {}            # ws -> {'seen': t}
        self.driver = None           # ws currently commanding motion
        self.app = web.Application()
        self.app.router.add_get('/ws', self.ws_handler)
        self.app.router.add_get('/video', self.video_handler)
        self.app.router.add_get('/snapshot', self.snapshot_handler)
        cors = {'Access-Control-Allow-Origin': '*', 'Cache-Control': 'no-cache'}   # APK pages are cross-origin
        self.app.router.add_get('/api/info', lambda r: web.json_response(self.node.info(), headers=cors))
        self.app.router.add_get('/api/maps', lambda r: web.json_response(self.node.list_maps(), headers=cors))
        self.app.router.add_get('/map.png', self.map_handler)
        if os.path.isdir(node.static_dir):
            self.app.router.add_get('/', self.index)
            self.app.router.add_static('/', node.static_dir, show_index=False)

    async def index(self, _req):
        return web.FileResponse(os.path.join(self.node.static_dir, 'index.html'))

    def _auth(self, req):
        return not self.node.token or req.query.get('token') == self.node.token

    # ---- websocket ----
    async def ws_handler(self, req):
        if not self._auth(req):
            raise web.HTTPUnauthorized()
        ws = web.WebSocketResponse(heartbeat=5.0, max_msg_size=1 << 20)
        await ws.prepare(req)
        self.clients[ws] = {'seen': time.monotonic()}
        await ws.send_json({'t': 'hello', 'info': self.node.info()})
        self.node.get_logger().info(f'client connected {req.remote} ({len(self.clients)})')
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    data = json.loads(msg.data)
                except ValueError:
                    continue
                self.clients[ws]['seen'] = time.monotonic()
                await self.handle(ws, data)
        finally:
            self.clients.pop(ws, None)
            if self.driver is ws:
                self.driver = None
                self.node.send_cmd(0, 0, 0)
            self.node.get_logger().info(f'client gone {req.remote} ({len(self.clients)})')
            self._check_unattended()
        return ws

    async def handle(self, ws, d):
        t = d.get('t')
        n = self.node
        if t == 'hb':
            return
        if t == 'cmd':
            try:
                vx, vy, wz = (float(d.get(k, 0.0)) for k in ('vx', 'vy', 'wz'))
            except (TypeError, ValueError):
                return
            if not all(math.isfinite(v) for v in (vx, vy, wz)):
                return
            self.driver = ws
            if n.nav_busy() and max(abs(vx), abs(vy), abs(wz)) > 1e-3:
                n.nav_cancel('(joystick)')
            n.send_cmd(vx, vy, wz)
        elif t == 'estop':
            n.set_estop(bool(d.get('on', True)))
            if n.estop:
                n.nav_cancel('(e-stop)')
                n.modes.stop_motion()
        elif t == 'gimbal':
            try:
                n.set_gimbal(float(d.get('angle', 0.0)))
            except (TypeError, ValueError):
                pass
        elif t == 'beep':
            n.beep_pub.publish(UInt8(data=max(1, min(5, int(d.get('n', 1))))))
        elif t == 'limits':
            kw = {k: d[k] for k in ('max_linear', 'max_angular') if k in d}
            if 'scan_guard' in d:
                kw['scan_guard'] = bool(d['scan_guard'])
            fut = n.set_limits(**kw)
            ok = fut is not None
            if ok:
                ok = await self._await_future(fut)
            await ws.send_json({'t': 'ack', 'req': 'limits', 'ok': ok})
        elif t == 'mode':
            name, on = d.get('name'), bool(d.get('on', True))
            loop = asyncio.get_running_loop()
            if on:
                if n.modes.modes.get(name, {}).get('motion'):
                    n.nav_cancel('(behaviour mode started)')     # one driver at a time
                ok, m = await loop.run_in_executor(None, n.modes.start, name, d.get('args'))
            else:
                ok, m = await loop.run_in_executor(None, n.modes.stop, name)
            await ws.send_json({'t': 'ack', 'req': 'mode', 'name': name, 'ok': ok, 'msg': m})
        elif t == 'map_save':
            ok, m = await asyncio.get_running_loop().run_in_executor(
                None, n.save_map, str(d.get('name') or '').strip())
            await ws.send_json({'t': 'ack', 'req': 'map_save', 'ok': ok, 'msg': m})
        elif t == 'nav_goal':
            try:
                x, y = float(d['x']), float(d['y'])
                yaw = None if d.get('yaw') is None else float(d['yaw'])
            except (KeyError, TypeError, ValueError):
                return
            if n.estop:
                ok, m = False, '急停中'
            else:
                ok, m = await asyncio.get_running_loop().run_in_executor(None, n.nav_goal, x, y, yaw)
            await ws.send_json({'t': 'ack', 'req': 'nav_goal', 'ok': ok, 'msg': m})
        elif t == 'nav_plan':
            try:
                x, y = float(d['x']), float(d['y'])
            except (KeyError, TypeError, ValueError):
                return
            ok, m, extra = await asyncio.get_running_loop().run_in_executor(None, n.nav_plan, x, y)
            await ws.send_json({'t': 'ack', 'req': 'nav_plan', 'id': d.get('id'), 'ok': ok, 'msg': m, **(extra or {})})
        elif t in ('nav_route', 'nav_route_check'):
            try:
                pts = [[float(p[0]), float(p[1])] for p in d['pts']]
            except (KeyError, TypeError, ValueError, IndexError):
                return
            if not 1 <= len(pts) <= 30 or not all(math.isfinite(v) for p in pts for v in p):
                await ws.send_json({'t': 'ack', 'req': t, 'id': d.get('id'), 'ok': False, 'msg': '路线要 1~30 个点'})
                return
            loop = bool(d.get('loop'))
            ex = asyncio.get_running_loop().run_in_executor
            if t == 'nav_route_check':
                ok, m, extra = await ex(None, n.nav_route_check, pts, loop)
                await ws.send_json({'t': 'ack', 'req': t, 'id': d.get('id'), 'ok': ok, 'msg': m, **(extra or {})})
                return
            ok, m = (False, '急停中') if n.estop else await ex(None, n.nav_route, pts, loop)
            await ws.send_json({'t': 'ack', 'req': 'nav_goal', 'ok': ok, 'msg': m})
        elif t == 'nav_home':
            ok, m = (False, '急停中') if n.estop else await asyncio.get_running_loop().run_in_executor(None, n.nav_home)
            await ws.send_json({'t': 'ack', 'req': 'nav_goal', 'ok': ok, 'msg': m})
        elif t == 'home_set':
            try:
                xyz = (float(d['x']), float(d['y']), float(d['yaw'])) if 'x' in d else ()
            except (TypeError, ValueError, KeyError):
                return
            ok, m = n.home_set(*xyz)
            await ws.send_json({'t': 'ack', 'req': 'home_set', 'ok': ok, 'msg': m})
        elif t == 'nav_cancel':
            n.nav_cancel('(APP)')
        elif t in ('map_erase', 'map_restore'):
            if t == 'map_erase':
                try:
                    strokes = [[[float(p[0]), float(p[1])] for p in st] for st in d['strokes']]
                except (KeyError, TypeError, ValueError, IndexError):
                    return
                if not 1 <= sum(len(st) for st in strokes) <= 2000 or                         not all(math.isfinite(v) for st in strokes for p in st for v in p):
                    return
                job = lambda: n.map_erase(strokes)     # noqa: E731
            else:
                job = n.map_restore
            ok, m = await asyncio.get_running_loop().run_in_executor(None, job)
            await ws.send_json({'t': 'ack', 'req': 'map_erase', 'ok': ok, 'msg': m})
        elif t == 'init_pose':
            try:
                n.set_init_pose(float(d['x']), float(d['y']), float(d['yaw']))
            except (KeyError, TypeError, ValueError):
                return
            await ws.send_json({'t': 'ack', 'req': 'init_pose', 'ok': True, 'msg': '已设定位置'})

    async def _await_future(self, fut, timeout=2.0):
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout:
                return False
            await asyncio.sleep(0.02)
        try:
            return all(r.successful for r in fut.result().results)
        except Exception:  # noqa: BLE001
            return False

    def _check_unattended(self):
        if not self.clients:
            self.node.nav_pause('(no APP connected)')
        if not self.clients:
            self.node.behavior_hold('(no APP connected)')    # lease_watchdog stops it after PAUSE_S

    async def lease_watchdog(self):
        stale_since = None
        while True:
            await asyncio.sleep(0.05)
            now = time.monotonic()
            d = self.driver
            if d is not None and now - self.clients.get(d, {}).get('seen', 0.0) > self.node.lease_s:
                self.driver = None
                self.node.send_cmd(0, 0, 0)
                self.node.get_logger().warn('driver lease expired -> stop')
            fresh = any(now - c['seen'] <= 2.0 for c in self.clients.values())
            if fresh:
                stale_since = None
                if self.node.paused is not None:
                    await asyncio.get_running_loop().run_in_executor(None, self.node.nav_resume)
                self.node.behavior_release()
            else:
                stale_since = stale_since or now
                if now - stale_since > 2.0 and self.node.nav_busy():
                    self.node.nav_pause('(APP heartbeat lost)')
                if (now - stale_since > 2.0 or self.node.hold_since is not None) and \
                        self.node.behavior_hold('(APP heartbeat lost)'):
                    self.node.get_logger().warn('APP link lost too long: stopping motion modes')
                    await asyncio.get_running_loop().run_in_executor(None, self.node.modes.stop_motion)

    async def broadcaster(self):
        k = 0
        div = max(1, int(round(self.node.state_hz / max(0.1, self.node.scan_hz))))
        while True:
            await asyncio.sleep(1.0 / self.node.state_hz)
            if not self.clients:
                continue
            msgs = [json.dumps(self.node.state())]
            k += 1
            if k % div == 0:
                msgs.append(json.dumps({'t': 'scan', 'pts': self.node.scan_pts}))
            for ws in list(self.clients):
                for m in msgs:
                    try:
                        await ws.send_str(m)
                    except (ConnectionResetError, RuntimeError):
                        break

    async def map_handler(self, req):
        if not self._auth(req):
            raise web.HTTPUnauthorized()
        png = self.node.map_png()
        if png is None:
            raise web.HTTPNotFound(text='no map')
        return web.Response(body=png, content_type='image/png',
                            headers={'Cache-Control': 'no-cache', 'Access-Control-Allow-Origin': '*'})

    # ---- video ----
    def _video_params(self, req):
        src = req.query.get('src', 'color')
        topic = VIDEO_ALIASES.get(src, src if src.startswith('/') else None)
        if topic is None:
            raise web.HTTPBadRequest(text='bad src')
        w = max(160, min(1280, int(req.query.get('w', 640))))
        q = max(20, min(95, int(req.query.get('q', 70))))
        fps = max(1.0, min(30.0, float(req.query.get('fps', 15))))
        return topic, w, q, fps

    async def video_handler(self, req):
        if not self._auth(req):
            raise web.HTTPUnauthorized()
        topic, w, q, fps = self._video_params(req)
        tap = self.node.tap(topic)
        tap.acquire(w, q, fps)
        resp = web.StreamResponse(headers={
            'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
            'Cache-Control': 'no-cache, no-store', 'Access-Control-Allow-Origin': '*'})
        await resp.prepare(req)
        last = -1
        try:
            while True:
                if tap.seq != last and tap.jpeg is not None:
                    last, jpg = tap.seq, tap.jpeg
                    await resp.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(jpg)).encode() + b'\r\n\r\n' + jpg + b'\r\n')
                await asyncio.sleep(0.01)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        finally:
            tap.release()
        return resp

    async def snapshot_handler(self, req):
        if not self._auth(req):
            raise web.HTTPUnauthorized()
        topic, w, q, fps = self._video_params(req)
        tap = self.node.tap(topic)
        tap.acquire(w, q, fps)
        try:
            t0 = time.monotonic()
            seq0 = tap.seq
            while tap.seq == seq0 and time.monotonic() - t0 < 2.0:
                await asyncio.sleep(0.02)
            if tap.jpeg is None:
                raise web.HTTPServiceUnavailable(text='no image')
            return web.Response(body=tap.jpeg, content_type='image/jpeg',
                                headers={'Access-Control-Allow-Origin': '*'})
        finally:
            tap.release()


def main():
    os.environ.setdefault('TZ', 'Asia/Shanghai')   # container runs UTC; map names use local time
    time.tzset()
    rclpy.init()
    node = GatewayNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    server = Server(node)

    async def run():
        runner = web.AppRunner(server.app)
        await runner.setup()
        await web.TCPSite(runner, '0.0.0.0', node.port).start()
        node.get_logger().info(f'gateway listening on :{node.port}')
        await asyncio.gather(server.lease_watchdog(), server.broadcaster())

    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass
    finally:
        node.send_cmd(0, 0, 0)
        node.modes.stop_all()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
