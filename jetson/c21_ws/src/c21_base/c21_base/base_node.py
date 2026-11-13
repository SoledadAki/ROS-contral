"""C21 chassis driver node.

Subscribes : cmd_vel_safe (Twist)  - only the safety arbiter should publish here
             c21/gimbal  (Float32, rad)  c21/beep (UInt8, times)  c21/imu_calibrate (Empty)
Publishes  : odom, imu/data_raw, battery (BatteryState), c21/base_state (String JSON)
             TF odom->base_footprint (optional; disable when the EKF owns it)

Safety: the MCU has no command watchdog, so this node streams the latest command at
send_rate and replaces it with zero once it is older than cmd_timeout. Zeros are held
for zero_hold seconds and then streaming stops so the PS2 pad can drive the MCU.
"""
import json
import math
import threading
import time

import rclpy
import rclpy.executors
import serial
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_msgs.msg import Empty, Float32, String, UInt8
from tf2_ros import TransformBroadcaster

from c21_base import protocol as P


def _clamp(v, lim):
    return max(-lim, min(lim, v))


def _yaw_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class BaseNode(Node):
    def __init__(self):
        super().__init__('c21_base')
        dp = self.declare_parameter
        self.port = dp('port', '/dev/ttyTHS0').value
        self.baud = dp('baud', 115200).value
        self.cmd_timeout = dp('cmd_timeout', 0.3).value
        self.send_rate = dp('send_rate', 20.0).value
        self.zero_hold = dp('zero_hold', 1.0).value
        self.max_v = dp('max_linear', 0.5).value
        self.max_w = dp('max_angular', 1.5).value
        self.publish_tf = dp('publish_tf', True).value
        self.odom_frame = dp('odom_frame', 'odom').value
        self.base_frame = dp('base_frame', 'base_footprint').value
        self.imu_frame = dp('imu_frame', 'imu_link').value
        self.yaw_source = dp('yaw_source', 'gyro').value          # gyro | wheel
        self.scale_x = dp('odom_scale_x', 1.0).value
        self.scale_y = dp('odom_scale_y', 1.0).value
        # firmware (RF4 freertos.c) sends Bat_Vol = percent of the 8.4..12.6 V range (3S pack)
        self.battery_empty_v = dp('battery_empty_v', 8.4).value
        self.battery_full_v = dp('battery_full_v', 12.6).value
        self.gimbal_limit = dp('gimbal_limit', 0.78).value        # URDF arm_0_joint +-0.785
        self.gimbal_move_ms = dp('gimbal_move_ms', 120).value
        self.gyro_calib_s = dp('gyro_calib_seconds', 2.0).value
        # heading hold: while driving straight (wz == 0) steer on the gyro to cancel drift
        self.heading_hold = dp('heading_hold', True).value
        self.heading_kp = dp('heading_kp', 2.0).value
        self.heading_max = dp('heading_max_correction', 0.3).value

        self._lock = threading.Lock()
        self._cmd = (0.0, 0.0, 0.0)
        self._cmd_stamp = 0.0
        self._zero_until = time.monotonic() + self.zero_hold
        self._tx_queue = []
        self._ser = None
        self._parser = P.FrameParser()
        self._rx_count = 0
        self._last_rx = 0.0
        self._last_fb = None
        self._gimbal = 0.0
        self._streaming = False
        self._sent_cmd = None

        self.x = self.y = self.yaw = 0.0
        self._last_odom_t = None
        self._gyro_bias = [0.0, 0.0, 0.0]
        self._bias_acc = [0.0, 0.0, 0.0]
        self._bias_n = 0
        self._bias_done = self.gyro_calib_s <= 0
        self._bias_t0 = None
        self._last_wheel_motion = 0.0
        self._still = False
        self._hold_yaw = None

        self.odom_pub = self.create_publisher(Odometry, 'odom', 20)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', qos_profile_sensor_data)
        self.bat_pub = self.create_publisher(BatteryState, 'battery', 5)
        self.state_pub = self.create_publisher(String, 'c21/base_state', 5)
        self.tf_pub = TransformBroadcaster(self) if self.publish_tf else None
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.create_subscription(Twist, 'cmd_vel_safe', self._on_cmd, 10)
        self.create_subscription(Float32, 'c21/gimbal', self._on_gimbal, 10)
        self.create_subscription(UInt8, 'c21/beep', self._on_beep, 10)
        self.create_subscription(Empty, 'c21/imu_calibrate', self._on_imu_calib, 1)

        self.create_timer(1.0 / self.send_rate, self._tx_tick)
        self.create_timer(1.0, self._status_tick)
        self.create_timer(0.1, self._joint_tick)

        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        self.get_logger().info(f'c21_base on {self.port}@{self.baud}, timeout {self.cmd_timeout}s')

    # ---------------- serial ----------------
    def _open(self):
        try:
            s = serial.Serial(self.port, self.baud, timeout=0.05, write_timeout=0.05)
            s.reset_input_buffer()
            self._ser = s
            self.get_logger().info('serial opened')
        except serial.SerialException as e:
            self._ser = None
            self.get_logger().warn(f'serial open failed: {e}', throttle_duration_sec=5.0)

    def _write(self, data: bytes):
        s = self._ser
        if s is None:
            return False
        try:
            s.write(data)
            return True
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'serial write failed: {e}')
            self._close()
            return False

    def _close(self):
        s, self._ser = self._ser, None
        if s is not None:
            try:
                s.close()
            except Exception:
                pass

    def _rx_loop(self):
        while self._running and rclpy.ok():
            if self._ser is None:
                self._open()
                if self._ser is None:
                    time.sleep(1.0)
                    continue
            try:
                data = self._ser.read(256)
            except (serial.SerialException, OSError, TypeError, AttributeError) as e:
                self.get_logger().error(f'serial read failed: {e}')
                self._close()
                continue
            if data:
                for fb in self._parser.feed(data):
                    self._handle_feedback(fb)

    # ---------------- commands ----------------
    def _on_cmd(self, msg: Twist):
        vals = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(math.isfinite(v) for v in vals):
            return
        with self._lock:
            self._cmd = (_clamp(vals[0], self.max_v), _clamp(vals[1], self.max_v), _clamp(vals[2], self.max_w))
            self._cmd_stamp = time.monotonic()

    def _on_gimbal(self, msg: Float32):
        if not math.isfinite(msg.data):
            return
        self._gimbal = _clamp(msg.data, self.gimbal_limit)
        with self._lock:
            self._tx_queue.append(P.encode_servos([self._gimbal], self.gimbal_move_ms))

    def _on_beep(self, msg: UInt8):
        with self._lock:
            self._tx_queue.append(P.encode_beep(max(1, min(10, msg.data)), 100, 100))

    def _on_imu_calib(self, _msg):
        with self._lock:
            self._tx_queue.append(P.encode_imu_calibration())
        self._bias_done = False
        self._bias_acc = [0.0, 0.0, 0.0]
        self._bias_n = 0
        self._bias_t0 = None

    def _tx_tick(self):
        now = time.monotonic()
        with self._lock:
            fresh = now - self._cmd_stamp <= self.cmd_timeout
            cmd = self._cmd if fresh else (0.0, 0.0, 0.0)
            if fresh:
                self._zero_until = now + self.zero_hold
            queue, self._tx_queue = self._tx_queue, []
        self._streaming = fresh or now < self._zero_until
        cmd = self._apply_heading_hold(cmd)
        if self._streaming:
            self._write(P.encode_velocity(*cmd))
            self._sent_cmd = cmd
        else:
            self._sent_cmd = None
        for frame in queue[-4:]:
            self._write(frame)

    def _apply_heading_hold(self, cmd):
        vx, vy, wz = cmd
        usable = (self.heading_hold and self.yaw_source == 'gyro' and self._bias_done
                  and time.monotonic() - self._last_rx < 0.2)
        if not usable or abs(wz) > 1e-3 or math.hypot(vx, vy) < 0.01:
            self._hold_yaw = None
            return cmd
        if self._hold_yaw is None:
            self._hold_yaw = self.yaw
            return cmd
        err = math.atan2(math.sin(self._hold_yaw - self.yaw), math.cos(self._hold_yaw - self.yaw))
        if abs(err) > 0.5:          # something is badly off (pushed, stuck): don't fight it
            self._hold_yaw = self.yaw
            return cmd
        return (vx, vy, _clamp(self.heading_kp * err, self.heading_max))

    # ---------------- feedback ----------------
    def _handle_feedback(self, fb: P.Feedback):
        now = time.monotonic()
        self._rx_count += 1
        self._last_rx = now
        self._last_fb = fb
        stamp = self.get_clock().now()

        gyro = list(fb.gyro)
        if not self._bias_done:
            moving = any(abs(v) > 1e-3 for v in fb.vel)
            if moving:
                self._bias_acc, self._bias_n, self._bias_t0 = [0.0] * 3, 0, None
            else:
                if self._bias_t0 is None:
                    self._bias_t0 = now
                self._bias_acc = [a + g for a, g in zip(self._bias_acc, gyro)]
                self._bias_n += 1
                if now - self._bias_t0 >= self.gyro_calib_s and self._bias_n > 10:
                    self._gyro_bias = [a / self._bias_n for a in self._bias_acc]
                    self._bias_done = True
                    self.get_logger().info('gyro bias: %.5f %.5f %.5f' % tuple(self._gyro_bias))
        else:
            # Zero-velocity update: when the wheels have been still for a while, keep tracking
            # the (temperature-dependent) gyro bias and freeze yaw so it cannot drift.
            if any(abs(v) > 1e-3 for v in fb.vel):
                self._last_wheel_motion = now
            if now - self._last_wheel_motion > 0.5 and abs(gyro[2] - self._gyro_bias[2]) < 0.05:
                a = 0.01            # ~2 s time constant at 50 Hz
                self._gyro_bias = [b + a * (g - b) for b, g in zip(self._gyro_bias, gyro)]
                self._still = True
            else:
                self._still = False
        gyro = [g - b for g, b in zip(gyro, self._gyro_bias)]

        # odometry integration
        vx, vy, wz_wheel = fb.vel[0] * self.scale_x, fb.vel[1] * self.scale_y, fb.vel[2]
        wz = gyro[2] if self.yaw_source == 'gyro' else wz_wheel
        if self._last_odom_t is not None:
            dt = now - self._last_odom_t
            if 0.0 < dt < 0.5:
                c, s = math.cos(self.yaw), math.sin(self.yaw)
                self.x += (vx * c - vy * s) * dt
                self.y += (vx * s + vy * c) * dt
                if (self._bias_done or self.yaw_source != 'gyro') and not self._still:
                    self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))
        self._last_odom_t = now

        q = _yaw_quat(self.yaw)
        od = Odometry()
        od.header.stamp = stamp.to_msg()
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = q[2], q[3]
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.angular.z = vx, vy, wz
        pc = [0.0] * 36
        pc[0] = pc[7] = 0.01
        pc[14] = pc[21] = pc[28] = 1e6
        pc[35] = 0.05
        od.pose.covariance = pc
        tc = [0.0] * 36
        tc[0] = tc[7] = 0.005
        tc[14] = tc[21] = tc[28] = 1e6
        tc[35] = 0.02
        od.twist.covariance = tc
        self.odom_pub.publish(od)

        if self.tf_pub is not None:
            t = TransformStamped()
            t.header = od.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z, t.transform.rotation.w = q[2], q[3]
            self.tf_pub.sendTransform(t)

        imu = Imu()
        imu.header.stamp = od.header.stamp
        imu.header.frame_id = self.imu_frame
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = gyro
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = fb.accel
        imu.angular_velocity_covariance = [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.0004]
        imu.linear_acceleration_covariance = [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]
        self.imu_pub.publish(imu)

    def _joint_tick(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['arm_0_joint']
        js.position = [float(self._gimbal)]
        self.joint_pub.publish(js)

    def _battery(self):
        """(volts, fraction) from the firmware's percent byte; 0 means below empty (or ADC not ready)."""
        fb = self._last_fb
        if fb is None:
            return None, None
        frac = max(0.0, min(1.0, fb.battery_raw / 100.0))
        return self.battery_empty_v + frac * (self.battery_full_v - self.battery_empty_v), frac

    def _status_tick(self):
        now = time.monotonic()
        link = self._ser is not None and (now - self._last_rx) < 0.5
        volts, frac = self._battery()
        if volts is not None:
            b = BatteryState()
            b.header.stamp = self.get_clock().now().to_msg()
            b.voltage = float(volts)
            b.percentage = float(frac)
            b.present = True
            b.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
            self.bat_pub.publish(b)
        fb = self._last_fb
        state = {
            'link': link,
            'port_open': self._ser is not None,
            'rx_hz': self._rx_count,
            'bad_frames': self._parser.bad,
            'streaming': self._streaming,
            'cmd': list(self._sent_cmd or (0, 0, 0)),
            'battery_v': None if volts is None else round(volts, 2),
            'battery_raw': None if fb is None else fb.battery_raw,
            'ultrasonic_raw': None if fb is None else fb.ultrasonic_raw,
            'gimbal': self._gimbal,
            'gyro_bias_ready': self._bias_done,
            'gyro_bias_z': round(self._gyro_bias[2], 5),
            'still': self._still,
            'heading_hold': None if self._hold_yaw is None else round(self._hold_yaw, 3),
            'odom': [round(self.x, 3), round(self.y, 3), round(self.yaw, 3)],
        }
        self._rx_count = 0
        self.state_pub.publish(String(data=json.dumps(state)))
        if not link:
            self.get_logger().warn('no feedback from MCU', throttle_duration_sec=5.0)

    def shutdown(self):
        # explicit stop burst before releasing the port
        for _ in range(3):
            self._write(P.encode_velocity(0.0, 0.0, 0.0))
            time.sleep(0.02)
        self._running = False
        self._close()


def main():
    rclpy.init()
    node = BaseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
