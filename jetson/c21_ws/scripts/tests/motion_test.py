"""Wheels-up motion test: gimbal sweep, then each motion axis for 1 s at low speed.
Raw records -> ~/.ros/c21_tests/<stamp>.csv"""
import json, os, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String, Bool

V, W = 0.10, 0.5
STEPS = [('gimbal', 0.52), ('gimbal', -0.52), ('gimbal', 0.0),
         ('前进', (V, 0, 0)), ('后退', (-V, 0, 0)), ('左移', (0, V, 0)), ('右移', (0, -V, 0)),
         ('左转', (0, 0, W)), ('右转', (0, 0, -W)), ('estop', (V, 0, 0))]

class T(Node):
    def __init__(self):
        super().__init__('c21_motion_test')
        self.cmd = self.create_publisher(Twist, 'cmd_vel_app', 10)
        self.gim = self.create_publisher(Float32, 'c21/gimbal', 10)
        self.es = self.create_publisher(Bool, 'c21/estop', 10)
        d = os.path.expanduser('~/.ros/c21_tests'); os.makedirs(d, exist_ok=True)
        self.path = os.path.join(d, time.strftime('motion_%Y%m%d_%H%M%S.csv'))
        self.f = open(self.path, 'w'); self.f.write('t,step,topic,data\n')
        self.step = 'idle'; self.t0 = time.monotonic()
        self.odom = None; self.safe = None
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(Twist, 'cmd_vel_safe', self.on_safe, 10)
        self.create_subscription(String, 'c21/base_state', lambda m: self.rec('base_state', m.data), 10)
    def rec(self, topic, data):
        self.f.write(f'{time.monotonic()-self.t0:.3f},{self.step},{topic},"{data.replace(chr(34), chr(39))}"\n')
    def on_odom(self, m):
        p, t = m.pose.pose.position, m.twist.twist
        self.odom = m
        self.rec('odom', f'{p.x:.4f} {p.y:.4f} {m.pose.pose.orientation.z:.4f} | {t.linear.x:.3f} {t.linear.y:.3f} {t.angular.z:.3f}')
    def on_safe(self, m):
        self.safe = m
        self.rec('cmd_vel_safe', f'{m.linear.x:.3f} {m.linear.y:.3f} {m.angular.z:.3f}')
    def spin_for(self, sec, twist=None):
        end = time.monotonic() + sec; peak = [0, 0, 0]
        while time.monotonic() < end:
            if twist is not None:
                t = Twist(); t.linear.x, t.linear.y, t.angular.z = map(float, twist); self.cmd.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom:
                tw = self.odom.twist.twist
                for i, v in enumerate((tw.linear.x, tw.linear.y, tw.angular.z)):
                    if abs(v) > abs(peak[i]): peak[i] = v
        return peak

def main():
    global STEPS
    if len(sys.argv) > 1:
        STEPS = [x for x in STEPS if x[0] in sys.argv[1:]]
    rclpy.init(); n = T()
    n.es.publish(Bool(data=False)); n.spin_for(1.0)
    for name, arg in STEPS:
        n.step = name
        if name == 'gimbal':
            n.gim.publish(Float32(data=arg)); n.spin_for(1.5)
            print(f'gimbal -> {arg:+.2f} rad', flush=True); continue
        if name == 'estop':
            # drive forward, engage e-stop mid-run, verify output drops to 0
            # keep commanding forward for 1.5 s after e-stop: output must stay 0 the whole time
            n.spin_for(0.6, arg); n.es.publish(Bool(data=True)); n.spin_for(0.1, arg)
            leaks = []
            orig = n.on_safe
            n.on_safe = lambda m: (orig(m), leaks.append(m.linear.x) if abs(m.linear.x) > 1e-4 else None)
            n.sub_check = n.create_subscription(Twist, 'cmd_vel_safe', lambda m: leaks.append(m.linear.x) if abs(m.linear.x) > 1e-4 else None, 10)
            n.spin_for(1.5, arg)
            print(f'estop: non-zero outputs during 1.5 s of e-stop = {len(leaks)}', flush=True)
            n.es.publish(Bool(data=False)); n.spin_for(1.0, (0, 0, 0)); continue
        pk = n.spin_for(1.0, arg)
        n.spin_for(1.0, (0, 0, 0))
        print(f'{name}: cmd {arg}  odom peak vx {pk[0]:+.3f} vy {pk[1]:+.3f} wz {pk[2]:+.3f}', flush=True)
    n.step = 'end'; n.spin_for(0.5, (0, 0, 0))
    print('record:', n.path); n.f.close(); rclpy.shutdown()

main()
