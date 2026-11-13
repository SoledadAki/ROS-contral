"""Ground motion step: python3 ground_test.py vx vy wz seconds
Compares odom delta with lidar-measured displacement (median range to front/rear/left/right walls)
and gyro yaw. Raw record -> ~/.ros/c21_tests/ground_*.csv"""
import math, os, statistics, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

SCAN_YAW = math.pi

class G(Node):
    def __init__(self):
        super().__init__('c21_ground_test')
        self.pub = self.create_publisher(Twist, 'cmd_vel_app', 10)
        self.odom = None; self.scan = None
        self.create_subscription(Odometry, 'odom', lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        d = os.path.expanduser('~/.ros/c21_tests'); os.makedirs(d, exist_ok=True)
        self.path = os.path.join(d, time.strftime('ground_%Y%m%d_%H%M%S.csv'))
        self.f = open(self.path, 'w'); self.f.write('t,x,y,yaw,vx,vy,wz\n'); self.t0 = time.monotonic()
    def on_scan(self, m):
        c, s = math.cos(SCAN_YAW), math.sin(SCAN_YAW); pts = []
        for i, r in enumerate(m.ranges):
            if math.isfinite(r) and m.range_min < r < m.range_max:
                a = m.angle_min + i * m.angle_increment
                lx, ly = r * math.cos(a), r * math.sin(a)
                pts.append((lx * c - ly * s, lx * s + ly * c))
        self.scan = pts
    def walls(self):
        p = self.scan or []
        def med(v): return round(statistics.median(v), 3) if len(v) >= 3 else None
        return {'front': med([x for x, y in p if x > 0.1 and abs(y) < 0.08]),
                'rear': med([-x for x, y in p if x < -0.1 and abs(y) < 0.08]),
                'left': med([y for x, y in p if y > 0.1 and abs(x) < 0.08]),
                'right': med([-y for x, y in p if y < -0.1 and abs(x) < 0.08])}
    def pose(self):
        o = self.odom.pose.pose; q = o.orientation
        return o.position.x, o.position.y, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)
    def run(self, sec, tw):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            if tw:
                t = Twist(); t.linear.x, t.linear.y, t.angular.z = tw; self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom:
                x, y, yaw = self.pose(); v = self.odom.twist.twist
                self.f.write(f'{time.monotonic()-self.t0:.3f},{x:.4f},{y:.4f},{yaw:.4f},{v.linear.x:.3f},{v.linear.y:.3f},{v.angular.z:.3f}\n')

def main():
    vx, vy, wz, sec = map(float, sys.argv[1:5])
    rclpy.init(); n = G()
    n.run(1.5, None)
    w0, p0 = n.walls(), n.pose()
    n.run(sec, (vx, vy, wz)); n.run(1.5, (0.0, 0.0, 0.0))
    w1, p1 = n.walls(), n.pose()
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    dyaw = math.degrees(math.atan2(math.sin(p1[2] - p0[2]), math.cos(p1[2] - p0[2])))
    print(f'cmd vx={vx} vy={vy} wz={wz} for {sec}s')
    print(f'odom: dx={dx:+.3f} m  dy={dy:+.3f} m  dyaw={dyaw:+.1f} deg')
    print(f'lidar walls before {w0}')
    print(f'lidar walls after  {w1}')
    for k, sign in (('front', -1), ('rear', 1), ('left', -1), ('right', 1)):
        if w0[k] is not None and w1[k] is not None:
            axis = 'x' if k in ('front', 'rear') else 'y'
            print(f'  lidar {k:5s}: robot moved {axis} {sign * (w1[k] - w0[k]):+.3f} m')
    print('record:', n.path); n.f.close(); rclpy.shutdown()

main()
