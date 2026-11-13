"""Velocity arbiter - the single path to cmd_vel_safe.

Sources (highest priority first; a source counts only while fresh):
    cmd_vel_app       APP teleop (gateway)
    cmd_vel_behavior  lidar / vision behaviours
    cmd_vel_nav       Nav2
E-stop (c21/estop, Bool) overrides everything until explicitly released.
Applies speed limits, acceleration limits and an optional lidar corridor guard. Obstacle points from
the depth camera (c21/depth_obstacles, base frame; low / overhanging things the lidar misses) join
the lidar points while fresh; without them the guard keeps working on the lidar alone.
Limits can be changed at runtime with ros2 param set (the gateway does this).
"""
import json
import math
import time

import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String

SOURCES = [('app', 'cmd_vel_app', 0.3), ('behavior', 'cmd_vel_behavior', 0.5), ('nav', 'cmd_vel_nav', 0.5)]


def sweep_distance(px, py, ux, uy, hl, hw):
    """How far the box [-hl,hl]x[-hw,hw] can translate along unit (ux, uy) before touching (px, py).

    Equivalent to casting a ray from the point in direction -u against the box (slab method).
    Returns inf if the swept box never touches the point, 0 if it already overlaps.
    """
    t_in, t_out = -math.inf, math.inf
    for p, d, h in ((px, -ux, hl), (py, -uy, hw)):
        if abs(d) < 1e-9:
            if abs(p) > h:
                return math.inf
            continue
        t1, t2 = (-h - p) / d, (h - p) / d
        if t1 > t2:
            t1, t2 = t2, t1
        t_in, t_out = max(t_in, t1), min(t_out, t2)
    if t_in > t_out or t_out < 0.0:
        return math.inf
    return max(t_in, 0.0)


def sweep_clearance(points, ux, uy, hl, hw, min_hits=1):
    """Travel distance before the min_hits-th nearest point is touched (ignores lone noise)."""
    hits = []
    for px, py in points:
        d = sweep_distance(px, py, ux, uy, hl, hw)
        if d != math.inf:
            hits.append(d)
    if len(hits) < min_hits:
        return math.inf
    hits.sort()
    return hits[min_hits - 1]


def outside_footprint(x, y, half_length, half_width, margin):
    """False for returns from the robot itself (gimbal mast, cables) inside its footprint."""
    return abs(x) > half_length + margin or abs(y) > half_width + margin


def blind_rays(ranges, min_valid, self_range):
    """Indices of rays blocked by the robot itself (returns closer than self_range)."""
    return {i for i, r in enumerate(ranges) if math.isfinite(r) and min_valid < r < self_range}


def sector_rays(sectors_deg, angle_min, angle_inc, n):
    """Ray indices inside [start, end] lidar-frame degree pairs (flat list)."""
    idx = set()
    for a, b in zip(sectors_deg[0::2], sectors_deg[1::2]):
        k0 = int(math.floor((math.radians(a) - angle_min) / angle_inc))
        k1 = int(math.ceil((math.radians(b) - angle_min) / angle_inc))
        idx.update(k % n for k in range(k0, k1 + 1))
    return idx


def ray_index(px, py, scan_x, scan_yaw, angle_min, angle_inc, n):
    """Index of the lidar ray pointing at base-frame point (px, py)."""
    dx, dy = px - scan_x, py
    c, s = math.cos(scan_yaw), math.sin(scan_yaw)
    a = math.atan2(-s * dx + c * dy, c * dx + s * dy)
    k = (a - angle_min) / angle_inc
    return int(round(k)) % n


def ramp(current, target, max_step_up, max_step_down):
    """Accelerate gently, decelerate quickly (moving toward zero uses max_step_down)."""
    d = target - current
    toward_zero = abs(target) < abs(current) or (current != 0.0 and target * current < 0.0)
    step = max_step_down if toward_zero else max_step_up
    if d > step:
        return current + step
    if d < -step:
        return current - step
    return target


class Arbiter(Node):
    def __init__(self):
        super().__init__('c21_safety')
        dp = self.declare_parameter
        self.max_linear = dp('max_linear', 0.3).value
        self.max_angular = dp('max_angular', 1.0).value
        self.accel_linear = dp('accel_linear', 0.8).value       # m/s^2
        self.accel_angular = dp('accel_angular', 3.0).value     # rad/s^2
        self.decel_factor = dp('decel_factor', 3.0).value
        self.rate = dp('rate', 20.0).value
        self.scan_guard = dp('scan_guard', False).value
        self.scan_yaw = dp('scan_yaw', math.pi).value           # lidar yaw in base frame
        self.scan_x = dp('scan_x', 0.0).value
        self.half_length = dp('robot_half_length', 0.15).value
        self.half_width = dp('robot_half_width', 0.14).value
        self.stop_margin = dp('stop_margin', 0.10).value
        self.slow_zone = dp('slow_zone', 0.35).value
        self.slow_time = dp('slow_time', 1.0).value           # slow zone grows to speed * slow_time
        self.self_margin = dp('self_filter_margin', 0.01).value
        self.min_hits = dp('min_hits', 2).value
        # rays shadowed by the robot itself (gimbal mast) cannot see obstacles: creep only
        self.self_range = dp('self_range', 0.14).value
        self.blind_speed = dp('blind_speed', 0.08).value
        self.partial_blind_speed = dp('partial_blind_speed', 0.15).value
        self.side_margin = dp('side_margin', 0.03).value     # extra width when sweeping the box
        # lidar-frame sectors the shell blocks without producing any return (measured on the car)
        self.blind_sectors = list(dp('blind_sectors_deg', [0.0, 0.0]).value)
        self._static_blind = None
        self.scan_timeout = dp('scan_timeout', 0.5).value
        self.depth_guard = dp('depth_guard', True).value
        self.depth_timeout = dp('depth_timeout', 0.5).value
        self.add_on_set_parameters_callback(self._on_params)

        self.estop = False
        self.latest = {name: (None, 0.0) for name, _, _ in SOURCES}
        self.out = [0.0, 0.0, 0.0]
        self.active = None
        self.idle_zero_sent = 0
        self.points = []
        self.scan_stamp = 0.0
        self.clearance = math.inf
        self.guard_factor = 1.0
        self.blind = False
        self.blind_idx = set()
        self.scan_geom = None
        self.depth_points = []
        self.depth_stamp = 0.0

        for name, topic, _ in SOURCES:
            self.create_subscription(Twist, topic, lambda m, n=name: self._on_src(n, m), 10)
        self.create_subscription(Bool, 'c21/estop', self._on_estop, 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, 'c21/depth_obstacles', self._on_depth, 5)
        self.pub = self.create_publisher(Twist, 'cmd_vel_safe', 10)
        self.state_pub = self.create_publisher(String, 'c21/safety_state', 5)
        self.create_timer(1.0 / self.rate, self._tick)
        self.create_timer(0.5, self._publish_state)
        self.get_logger().info('arbiter up: limits %.2f m/s %.2f rad/s' % (self.max_linear, self.max_angular))

    # parameter names whose attribute is named differently (hasattr() alone missed these)
    PARAM_ATTR = {'robot_half_length': 'half_length', 'robot_half_width': 'half_width',
                  'self_filter_margin': 'self_margin'}

    def _on_params(self, params):
        for p in params:
            if p.name in ('max_linear', 'max_angular', 'accel_linear', 'accel_angular', 'stop_margin',
                          'slow_zone', 'scan_yaw', 'scan_x', 'robot_half_length', 'robot_half_width',
                          'decel_factor', 'slow_time', 'self_filter_margin', 'self_range', 'blind_speed',
                          'partial_blind_speed', 'side_margin'):
                if float(p.value) < 0.0:
                    return SetParametersResult(successful=False, reason='must be >= 0')
        for p in params:
            if p.name == 'max_linear':
                p_val = min(float(p.value), 0.8)
                self.max_linear = p_val
            elif p.name == 'max_angular':
                self.max_angular = min(float(p.value), 3.0)
            elif p.name == 'depth_guard':
                self.depth_guard = bool(p.value)
                self.get_logger().info('depth guard %s' % ('ON' if self.depth_guard else 'OFF'))
            elif p.name == 'scan_guard':
                self.scan_guard = bool(p.value)
                self.get_logger().info('scan guard %s' % ('ON' if self.scan_guard else 'OFF'))
            elif p.name in self.PARAM_ATTR:
                setattr(self, self.PARAM_ATTR[p.name], p.value)
            elif hasattr(self, p.name):
                setattr(self, p.name, p.value)
        return SetParametersResult(successful=True)

    def _on_src(self, name, msg):
        vals = (msg.linear.x, msg.linear.y, msg.angular.z)
        if all(math.isfinite(v) for v in vals):
            self.latest[name] = (vals, time.monotonic())

    def _on_estop(self, msg):
        if msg.data and not self.estop:
            self.get_logger().warn('E-STOP engaged')
        if not msg.data and self.estop:
            self.get_logger().info('E-STOP released')
        self.estop = bool(msg.data)
        if self.estop:
            self.out = [0.0, 0.0, 0.0]
            self.pub.publish(Twist())

    def _on_scan(self, msg):
        pts = []
        c, s = math.cos(self.scan_yaw), math.sin(self.scan_yaw)
        a = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min < r < min(msg.range_max, 3.0):
                lx, ly = r * math.cos(a), r * math.sin(a)
                x, y = self.scan_x + lx * c - ly * s, lx * s + ly * c
                if outside_footprint(x, y, self.half_length, self.half_width, self.self_margin):
                    pts.append((x, y))
            a += msg.angle_increment
        self.points = pts
        geom = (msg.angle_min, msg.angle_increment, len(msg.ranges))
        if self._static_blind is None or self.scan_geom != geom:
            self._static_blind = sector_rays(self.blind_sectors, *geom)
        self.scan_geom = geom
        self.blind_idx = blind_rays(msg.ranges, msg.range_min, self.self_range) | self._static_blind
        self.scan_stamp = time.monotonic()

    def _on_depth(self, msg):
        pts = []
        for x, y, _ in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y = float(x), float(y)
            if outside_footprint(x, y, self.half_length, self.half_width, self.self_margin):
                pts.append((x, y))
        self.depth_points = pts
        self.depth_stamp = time.monotonic()

    def _depth_fresh(self):
        return self.depth_guard and time.monotonic() - self.depth_stamp <= self.depth_timeout

    def _select(self, now):
        for name, _, timeout in SOURCES:
            vals, stamp = self.latest[name]
            if vals is not None and now - stamp <= timeout:
                return name, vals
        return None, (0.0, 0.0, 0.0)

    def _blind_samples(self, ux, uy, hl, hw):
        """How many of 5 probe points spread across the swept path, 0.3 m beyond the robot
        outline, lie in the robot's own lidar shadow."""
        if not self.scan_geom or not self.blind_idx:
            return 0
        a0, inc, n = self.scan_geom
        front = abs(hl * ux) + abs(hw * uy)
        half = abs(hl * uy) + abs(hw * ux)              # half extent across the motion
        along = front + 0.3
        blocked = 0
        for lat in (-half, -half / 2, 0.0, half / 2, half):
            px, py = along * ux - lat * uy, along * uy + lat * ux
            if ray_index(px, py, self.scan_x, self.scan_yaw, a0, inc, n) in self.blind_idx:
                blocked += 1
        return blocked

    def _guard(self, vx, vy):
        self.guard_factor = 1.0
        self.clearance = math.inf
        self.blind = False
        if not self.scan_guard:
            return vx, vy
        speed = math.hypot(vx, vy)
        if speed < 1e-3:
            return vx, vy
        if time.monotonic() - self.scan_stamp > self.scan_timeout:
            self.guard_factor = 0.0          # guard enabled but blind -> no translation
            return 0.0, 0.0
        ux, uy = vx / speed, vy / speed
        hl = self.half_length + self.side_margin * abs(uy)
        hw = self.half_width + self.side_margin * abs(ux)
        d = sweep_clearance(self.points, ux, uy, hl, hw, self.min_hits)
        if self._depth_fresh():      # depth cells are already denoised (pixel count + 2 frames): 1 hit counts
            d = min(d, sweep_clearance(self.depth_points, ux, uy, hl, hw, 1))
        self.clearance = d
        slow = max(self.slow_zone, speed * self.slow_time)
        f = max(0.0, min(1.0, (d - self.stop_margin) / max(slow, 1e-3)))
        blocked = self._blind_samples(ux, uy, self.half_length, self.half_width)
        if blocked >= 2:
            self.blind = True
            cap = self.blind_speed if blocked >= 4 else self.partial_blind_speed
            f = min(f, cap / speed)
        self.guard_factor = f
        return vx * f, vy * f

    def _tick(self):
        now = time.monotonic()
        name, (vx, vy, wz) = self._select(now)
        self.active = name
        if self.estop:
            name, vx, vy, wz = None, 0.0, 0.0, 0.0

        # clamp: scale translation as a vector so direction is preserved
        sp = math.hypot(vx, vy)
        if sp > self.max_linear > 0.0:
            vx, vy = vx * self.max_linear / sp, vy * self.max_linear / sp
        elif self.max_linear <= 0.0:
            vx = vy = 0.0
        wz = max(-self.max_angular, min(self.max_angular, wz))
        vx, vy = self._guard(vx, vy)

        dt = 1.0 / self.rate
        if self.estop:
            self.out = [0.0, 0.0, 0.0]
        else:
            lu, au = self.accel_linear * dt, self.accel_angular * dt
            self.out = [ramp(self.out[0], vx, lu, lu * self.decel_factor),
                        ramp(self.out[1], vy, lu, lu * self.decel_factor),
                        ramp(self.out[2], wz, au, au * self.decel_factor)]

        moving = any(abs(v) > 1e-4 for v in self.out)
        if name is not None or moving:
            self.idle_zero_sent = 0
        elif self.idle_zero_sent >= 5:
            return                      # idle: stop streaming, base driver handles timeout
        else:
            self.idle_zero_sent += 1
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = self.out
        self.pub.publish(t)

    def _publish_state(self):
        st = {
            'estop': self.estop,
            'source': self.active,
            'out': [round(v, 3) for v in self.out],
            'max_linear': self.max_linear,
            'max_angular': self.max_angular,
            'scan_guard': self.scan_guard,
            'depth': self._depth_fresh(),
            'depth_points': len(self.depth_points) if self._depth_fresh() else 0,
            'guard_factor': round(self.guard_factor, 2),
            'blind': self.blind,
            'clearance': None if math.isinf(self.clearance) else round(self.clearance, 3),
        }
        self.state_pub.publish(String(data=json.dumps(st)))


def main():
    rclpy.init()
    node = Arbiter()
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
