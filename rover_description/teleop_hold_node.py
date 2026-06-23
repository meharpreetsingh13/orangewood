#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════
  teleop_hold_node.py  (PID + SLAM edition)
  ───────────────────────────────────────────────────
  Arrow key hold-to-move teleop for rover with:
    • Dual PID loops (velocity + hold-position)
      driven by /odom feedback
    • slam_toolbox integration — subscribes to /map,
      logs mapping status, and exposes a save-map
      keybind ('s') via slam_toolbox/save_map service

  Topic wiring (Gazebo Fortress via ros_gz_bridge):
    /cmd_vel  ← published here         → Gazebo diff_drive
    /odom     ← Gazebo diff_drive      → PID feedback + SLAM
    /scan     ← Gazebo gpu_lidar       → slam_toolbox (not used here)
    /tf       ← Gazebo diff_drive/RSP  → slam_toolbox (not used here)
    /map      ← slam_toolbox           → status display here

  Modes
  ─────
  DRIVE  – keys held → velocity PID tracks commanded speed
  HOLD   – SPACE pressed → position PID locks current odom pose

  Controls
  ────────
  ↑ / ↓       forward / backward
  ← / →       rotate left / right
  SPACE       toggle hold-position mode
  s           save map to disk (rover_map.pgm / .yaml)
  q / z       increase / decrease speed (both)
  w / x       increase / decrease linear only
  e / c       increase / decrease angular only
  CTRL+C      quit
═══════════════════════════════════════════════════════
"""

import math
import threading
import sys
import tty
import termios

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from slam_toolbox.srv import SaveMap


# ── Default speed limits ───────────────────────────
LINEAR_SPEED  = 0.3    # m/s   — well within diff_drive max_linear_velocity=2.0
ANGULAR_SPEED = 0.8    # rad/s — within max_angular_velocity=2.0
SPEED_STEP    = 0.05

# ── Velocity PID gains ─────────────────────────────
# Tune KP first; add KI to kill steady-state drift;
# KD damps oscillation. Start with KI=0 if overshoot.
VEL_LIN_KP, VEL_LIN_KI, VEL_LIN_KD = 1.2, 0.4, 0.05
VEL_ANG_KP, VEL_ANG_KI, VEL_ANG_KD = 1.5, 0.3, 0.04

# ── Position PID gains ─────────────────────────────
POS_LIN_KP, POS_LIN_KI, POS_LIN_KD = 1.8, 0.1, 0.3
POS_ANG_KP, POS_ANG_KI, POS_ANG_KD = 2.0, 0.1, 0.25

# ── Integral wind-up clamp (output units) ──────────
ICLAMP = 1.0

# ── Arrow key escape sequences ─────────────────────
ARROW_UP    = '\x1b[A'
ARROW_DOWN  = '\x1b[B'
ARROW_RIGHT = '\x1b[C'
ARROW_LEFT  = '\x1b[D'

KEY_BINDINGS = {
    ARROW_UP    : ( 1.0,  0.0),   # forward
    ARROW_DOWN  : (-1.0,  0.0),   # backward
    ARROW_LEFT  : ( 0.0,  1.0),   # rotate left
    ARROW_RIGHT : ( 0.0, -1.0),   # rotate right
}

SPEED_BINDINGS = {
    'q': ( 1,  1), 'z': (-1, -1),   # both ±
    'w': ( 1,  0), 'x': (-1,  0),   # linear only
    'e': ( 0,  1), 'c': ( 0, -1),   # angular only
}

BANNER = """
╔══════════════════════════════════════════════════════╗
║   ROVER TELEOP  ·  PID + SLAM EDITION               ║
║                                                      ║
║   ↑ forward   ↓ backward                            ║
║   ← rotate left   → rotate right                    ║
║                                                      ║
║   SPACE  = toggle hold-position mode                 ║
║   s      = save map to disk                          ║
║   q/z    = faster / slower (both)                    ║
║   w/x    = linear ±    e/c = angular ±               ║
║   CTRL+C = quit                                      ║
╚══════════════════════════════════════════════════════╝
"""


# ═══════════════════════════════════════════════════
#  Generic PID
# ═══════════════════════════════════════════════════
class PID:
    """Discrete PID with integral anti-windup and first-tick guard."""

    def __init__(self, kp, ki, kd,
                 i_clamp=ICLAMP, out_min=-10.0, out_max=10.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_clamp = i_clamp
        self.out_min = out_min
        self.out_max = out_max
        self._integral   = 0.0
        self._prev_error = 0.0
        self._first_tick = True

    def reset(self):
        self._integral   = 0.0
        self._prev_error = 0.0
        self._first_tick = True

    def compute(self, setpoint: float, measured: float, dt: float) -> float:
        if dt <= 0:
            return 0.0
        error = setpoint - measured
        p = self.kp * error
        self._integral = max(-self.i_clamp,
                         min( self.i_clamp,
                              self._integral + error * dt))
        i = self.ki * self._integral
        d = 0.0 if self._first_tick else \
            self.kd * (error - self._prev_error) / dt
        self._first_tick = False
        self._prev_error = error
        return max(self.out_min, min(self.out_max, p + i + d))


# ═══════════════════════════════════════════════════
#  Helpers
# ═══════════════════════════════════════════════════
def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a: float, b: float) -> float:
    """Shortest signed angular difference."""
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


def get_key(settings) -> str:
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    if key == '\x1b':
        key += sys.stdin.read(1) + sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


# ═══════════════════════════════════════════════════
#  Node
# ═══════════════════════════════════════════════════
class TeleopHoldNode(Node):

    MODE_DRIVE = 'DRIVE'
    MODE_HOLD  = 'HOLD'

    def __init__(self):
        super().__init__('teleop_hold_node')

        # ── /cmd_vel publisher ─────────────────────
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── /odom subscriber ──────────────────────
        # Odometry is published by Gazebo diff_drive (front axle)
        # at 50 Hz and bridged to ROS via ros_gz_bridge
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

        # ── /map subscriber ───────────────────────
        # OccupancyGrid published by slam_toolbox at ~1 Hz.
        # QoS depth=1: we only need the latest map.
        self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, 1)

        # ── slam_toolbox save-map service ─────────
        self._save_map_cli = self.create_client(
            SaveMap, '/slam_toolbox/save_map')

        # ── Speed setpoints ───────────────────────
        self.linear_speed  = LINEAR_SPEED
        self.angular_speed = ANGULAR_SPEED
        self.current_key   = None

        # ── Odom state (thread-safe) ──────────────
        self._odom_lock     = threading.Lock()
        self._meas_lin      = 0.0
        self._meas_ang      = 0.0
        self._pos_x         = 0.0
        self._pos_y         = 0.0
        self._pos_yaw       = 0.0
        self._odom_received = False

        # ── SLAM / map state ──────────────────────
        self._map_lock        = threading.Lock()
        self._map_w           = 0
        self._map_h           = 0
        self._map_res         = 0.0
        self._map_known       = 0     # cells with value != -1
        self._map_received    = False

        # ── Hold-position target ──────────────────
        self._hold_x   = 0.0
        self._hold_y   = 0.0
        self._hold_yaw = 0.0

        # ── Mode ──────────────────────────────────
        self._mode    = self.MODE_DRIVE
        self._running = True

        # ── PIDs ──────────────────────────────────
        lv, av = self.linear_speed, self.angular_speed
        self._vel_lin_pid = PID(VEL_LIN_KP, VEL_LIN_KI, VEL_LIN_KD,
                                out_min=-lv * 2, out_max=lv * 2)
        self._vel_ang_pid = PID(VEL_ANG_KP, VEL_ANG_KI, VEL_ANG_KD,
                                out_min=-av * 2, out_max=av * 2)
        self._pos_lin_pid = PID(POS_LIN_KP, POS_LIN_KI, POS_LIN_KD,
                                out_min=-lv, out_max=lv)
        self._pos_ang_pid = PID(POS_ANG_KP, POS_ANG_KI, POS_ANG_KD,
                                out_min=-av, out_max=av)

        # ── Timers ────────────────────────────────
        self._last_time = self.get_clock().now()
        self.create_timer(0.05,  self._control_loop)    # 20 Hz control
        self.create_timer(0.5,   self._status_line)     #  2 Hz display

        print(BANNER)
        self._print_speeds()

        # ── Keyboard thread ───────────────────────
        threading.Thread(target=self._key_loop, daemon=True).start()

    # ─────────────────────────────────────────────
    #  Callbacks
    # ─────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        with self._odom_lock:
            self._meas_lin = msg.twist.twist.linear.x
            self._meas_ang = msg.twist.twist.angular.z
            self._pos_x    = msg.pose.pose.position.x
            self._pos_y    = msg.pose.pose.position.y
            self._pos_yaw  = yaw_from_quaternion(msg.pose.pose.orientation)
            self._odom_received = True

    def _map_cb(self, msg: OccupancyGrid):
        """Track explored area from slam_toolbox OccupancyGrid."""
        known = sum(1 for c in msg.data if c != -1)
        with self._map_lock:
            self._map_w        = msg.info.width
            self._map_h        = msg.info.height
            self._map_res      = msg.info.resolution
            self._map_known    = known
            self._map_received = True

    # ─────────────────────────────────────────────
    #  Status display  (2 Hz, single overwriting line)
    # ─────────────────────────────────────────────
    def _status_line(self):
        with self._map_lock:
            if not self._map_received:
                slam_str = 'SLAM: waiting for /map ...'
            else:
                area = self._map_known * self._map_res ** 2
                slam_str = (
                    f'MAP {self._map_w}×{self._map_h}'
                    f'@{self._map_res*100:.0f}cm  '
                    f'explored≈{area:.1f}m²'
                )

        with self._odom_lock:
            pose_str = (
                f'x={self._pos_x:.2f} '
                f'y={self._pos_y:.2f} '
                f'yaw={math.degrees(self._pos_yaw):.1f}°'
            )

        mode_str = f'[{self._mode}]'
        sys.stdout.write(
            f'\r\033[K  {mode_str}  {slam_str}  |  {pose_str}')
        sys.stdout.flush()

    # ─────────────────────────────────────────────
    #  20 Hz control loop
    # ─────────────────────────────────────────────
    def _control_loop(self):
        now = self.get_clock().now()
        dt  = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        if not self._odom_received:
            self.pub.publish(Twist())
            return

        with self._odom_lock:
            meas_lin = self._meas_lin
            meas_ang = self._meas_ang
            cur_x    = self._pos_x
            cur_y    = self._pos_y
            cur_yaw  = self._pos_yaw

        msg = Twist()

        # ══ DRIVE ══ velocity PID ══════════════════
        if self._mode == self.MODE_DRIVE:
            if self.current_key in KEY_BINDINGS:
                lx, az = KEY_BINDINGS[self.current_key]
                cmd_lin = lx * self.linear_speed
                cmd_ang = az * self.angular_speed
            else:
                cmd_lin = cmd_ang = 0.0

            msg.linear.x  = self._vel_lin_pid.compute(cmd_lin, meas_lin, dt)
            msg.angular.z = self._vel_ang_pid.compute(cmd_ang, meas_ang, dt)

        # ══ HOLD ══ position PID ═══════════════════
        else:
            dx  = self._hold_x   - cur_x
            dy  = self._hold_y   - cur_y
            dist = math.hypot(dx, dy)
            heading_err = math.atan2(dy, dx) - cur_yaw
            lin_err = dist * math.cos(heading_err)   # signed
            ang_err = angle_diff(self._hold_yaw, cur_yaw)

            if abs(lin_err) < 0.01:
                lin_err = 0.0
                self._pos_lin_pid.reset()
            if abs(ang_err) < 0.005:
                ang_err = 0.0
                self._pos_ang_pid.reset()

            msg.linear.x  = self._pos_lin_pid.compute(lin_err, 0.0, dt)
            msg.angular.z = self._pos_ang_pid.compute(ang_err, 0.0, dt)

        self.pub.publish(msg)

    # ─────────────────────────────────────────────
    #  Mode transitions
    # ─────────────────────────────────────────────
    def _enter_hold(self):
        with self._odom_lock:
            self._hold_x   = self._pos_x
            self._hold_y   = self._pos_y
            self._hold_yaw = self._pos_yaw
        self._pos_lin_pid.reset()
        self._pos_ang_pid.reset()
        self._mode = self.MODE_HOLD
        self.get_logger().warn(
            f'HOLD — x={self._hold_x:.3f} y={self._hold_y:.3f} '
            f'yaw={math.degrees(self._hold_yaw):.1f}°')

    def _enter_drive(self):
        self._vel_lin_pid.reset()
        self._vel_ang_pid.reset()
        self._mode = self.MODE_DRIVE
        self.get_logger().info('DRIVE MODE')

    # ─────────────────────────────────────────────
    #  Save map via slam_toolbox service
    # ─────────────────────────────────────────────
    def _save_map(self):
        if not self._save_map_cli.service_is_ready():
            self.get_logger().warn(
                'SaveMap service not ready — is slam_toolbox running?')
            return
        req = SaveMap.Request()
        req.name.data = 'rover_map'
        future = self._save_map_cli.call_async(req)
        future.add_done_callback(self._save_done_cb)
        self.get_logger().info("Saving map as 'rover_map' ...")

    def _save_done_cb(self, future):
        try:
            result = future.result()
            status = 'OK' if result.result == 0 else f'code {result.result}'
            self.get_logger().info(f'Map save: {status}')
        except Exception as e:
            self.get_logger().error(f'Map save failed: {e}')

    # ─────────────────────────────────────────────
    #  Speed limit helpers
    # ─────────────────────────────────────────────
    def _update_pid_limits(self):
        lv, av = self.linear_speed, self.angular_speed
        self._vel_lin_pid.out_min, self._vel_lin_pid.out_max = -lv*2,  lv*2
        self._vel_ang_pid.out_min, self._vel_ang_pid.out_max = -av*2,  av*2
        self._pos_lin_pid.out_min, self._pos_lin_pid.out_max = -lv,    lv
        self._pos_ang_pid.out_min, self._pos_ang_pid.out_max = -av,    av

    def _print_speeds(self):
        print(f'\n  Lin={self.linear_speed:.2f}m/s  '
              f'Ang={self.angular_speed:.2f}rad/s  '
              f'Mode=[{self._mode}]')

    # ─────────────────────────────────────────────
    #  Keyboard loop
    # ─────────────────────────────────────────────
    def _key_loop(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            while self._running:
                key = get_key(settings)

                if key == '\x03':              # CTRL+C — quit
                    self._running = False
                    rclpy.shutdown()
                    break

                if key == ' ':                 # toggle HOLD / DRIVE
                    if self._mode == self.MODE_DRIVE:
                        self.current_key = None
                        self._enter_hold()
                    else:
                        self._enter_drive()
                    self._print_speeds()
                    continue

                if key == 's':                 # save map
                    self._save_map()
                    continue

                if key in SPEED_BINDINGS:
                    dl, da = SPEED_BINDINGS[key]
                    self.linear_speed = max(
                        0.05, self.linear_speed  + dl * SPEED_STEP)
                    self.angular_speed = max(
                        0.05, self.angular_speed + da * SPEED_STEP)
                    self._update_pid_limits()
                    self._print_speeds()
                    continue

                if self._mode == self.MODE_DRIVE:
                    self.current_key = (
                        key if key in KEY_BINDINGS else None)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            self.pub.publish(Twist())   # safe stop


# ═══════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = TeleopHoldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()