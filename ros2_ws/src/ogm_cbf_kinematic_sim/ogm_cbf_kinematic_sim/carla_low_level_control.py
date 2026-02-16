#!/usr/bin/env python3
import math
from time import monotonic

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def rate_limit(prev, target, max_rate_per_s, dt):
    # limit |target-prev| <= max_rate_per_s * dt
    step = max_rate_per_s * dt
    if target > prev + step:
        return prev + step
    if target < prev - step:
        return prev - step
    return target


class CmdVelToCarlaBetter(Node):
    def __init__(self):
        super().__init__("cmdvel_to_carla_better")

        # topics
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/carla/ego_vehicle/odometry")
        self.declare_parameter("out_topic", "/carla/ego_vehicle/vehicle_control_cmd")

        # timing/safety
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("cmd_timeout_s", 0.5)

        # steering model
        self.declare_parameter("wheelbase_L", 2.7)          # m
        self.declare_parameter("max_steer_rad", 0.7)        # rad (~40deg) -> steer=1
        self.declare_parameter("steer_rate", 2.5)           # per second (normalized steer/s)

        # longitudinal model (accel-domain PID)
        self.declare_parameter("max_accel", 3.0)            # m/s^2
        self.declare_parameter("max_decel", 6.0)            # m/s^2 (brake)
        self.declare_parameter("Kp", 1.2)
        self.declare_parameter("Ki", 0.2)
        self.declare_parameter("Kd", 0.0)
        self.declare_parameter("i_limit", 5.0)              # integral clamp
        self.declare_parameter("throttle_rate", 3.0)        # per second
        self.declare_parameter("brake_rate", 6.0)           # per second
        self.declare_parameter("speed_lpf_alpha", 0.2)      # 0..1 (meas smoothing)
        self.declare_parameter("speed_deadband", 0.05)      # m/s (ignore tiny errors)

        # read params
        p = self.get_parameter
        self.cmd_topic = p("cmd_vel_topic").value
        self.odom_topic = p("odom_topic").value
        self.out_topic = p("out_topic").value

        self.hz = float(p("publish_hz").value)
        self.timeout = float(p("cmd_timeout_s").value)

        self.L = float(p("wheelbase_L").value)
        self.max_steer_rad = float(p("max_steer_rad").value)
        self.steer_rate = float(p("steer_rate").value)

        self.max_accel = float(p("max_accel").value)
        self.max_decel = float(p("max_decel").value)
        self.Kp = float(p("Kp").value)
        self.Ki = float(p("Ki").value)
        self.Kd = float(p("Kd").value)
        self.i_limit = float(p("i_limit").value)
        self.thr_rate = float(p("throttle_rate").value)
        self.brk_rate = float(p("brake_rate").value)
        self.alpha = float(p("speed_lpf_alpha").value)
        self.deadband = float(p("speed_deadband").value)

        # state
        self.v_ref = 0.0
        self.w_ref = 0.0
        self.last_cmd_t = None

        self.v_meas = 0.0
        self.v_meas_f = 0.0

        self.e_prev = 0.0
        self.i = 0.0
        self.t_prev = monotonic()

        self.steer_prev = 0.0
        self.thr_prev = 0.0
        self.brk_prev = 0.0

        # ROS I/O
        self.sub_cmd = self.create_subscription(Twist, self.cmd_topic, self.on_cmd, 10)
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.pub = self.create_publisher(CarlaEgoVehicleControl, self.out_topic, 1)

        self.timer = self.create_timer(1.0 / max(self.hz, 1e-6), self.tick)

    def on_cmd(self, msg: Twist):
        self.v_ref = float(msg.linear.x)
        self.w_ref = -float(msg.angular.z)
        self.last_cmd_t = monotonic()

    def on_odom(self, msg: Odometry):
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        vz = float(msg.twist.twist.linear.z)
        v = math.sqrt(vx*vx + vy*vy + vz*vz)
        self.v_meas = v
        # low-pass
        self.v_meas_f = (1.0 - self.alpha) * self.v_meas_f + self.alpha * self.v_meas

    def compute_steer(self, v_ref, w_ref):
        # bicycle: w = v/L * tan(delta) -> delta = atan(L*w/v)
        v_eps = 0.2
        if abs(v_ref) < v_eps or abs(w_ref) < 1e-4:
            delta = 0.0
        else:
            delta = math.atan(self.L * w_ref / v_ref)  # v_ref signed => reverse handled
        delta = clamp(delta, -self.max_steer_rad, +self.max_steer_rad)
        steer = delta / self.max_steer_rad
        return float(clamp(steer, -1.0, 1.0))

    def compute_longitudinal(self, v_ref, v_meas, dt):
        # control speed magnitude; gear handled separately
        v_ref_abs = abs(v_ref)
        e = v_ref_abs - v_meas

        if abs(e) < self.deadband:
            e = 0.0

        de = (e - self.e_prev) / max(dt, 1e-6)
        self.e_prev = e

        self.i += e * dt
        self.i = clamp(self.i, -self.i_limit, +self.i_limit)

        a_cmd = self.Kp * e + self.Ki * self.i + self.Kd * de
        a_cmd = clamp(a_cmd, -self.max_decel, +self.max_accel)

        if a_cmd >= 0.0:
            throttle = a_cmd / self.max_accel
            brake = 0.0
        else:
            throttle = 0.0
            brake = (-a_cmd) / self.max_decel

        return float(clamp(throttle, 0.0, 1.0)), float(clamp(brake, 0.0, 1.0))

    def tick(self):
        now = monotonic()
        dt = max(now - self.t_prev, 1e-6)
        self.t_prev = now

        timed_out = (self.last_cmd_t is None) or ((now - self.last_cmd_t) > self.timeout)

        if timed_out:
            v_ref = 0.0
            w_ref = 0.0
            # hard stop + reset integrator
            self.i = 0.0
            self.e_prev = 0.0
        else:
            v_ref = self.v_ref
            w_ref = self.w_ref

        steer_tgt = self.compute_steer(v_ref, w_ref)
        thr_tgt, brk_tgt = self.compute_longitudinal(v_ref, self.v_meas_f, dt)

        # exclusivity (never both)
        if thr_tgt > 0.0:
            brk_tgt = 0.0
        if brk_tgt > 0.0:
            thr_tgt = 0.0

        # rate limit outputs (smooth)
        steer = rate_limit(self.steer_prev, steer_tgt, self.steer_rate, dt)
        thr = rate_limit(self.thr_prev, thr_tgt, self.thr_rate, dt)
        brk = rate_limit(self.brk_prev, brk_tgt, self.brk_rate, dt)

        self.steer_prev, self.thr_prev, self.brk_prev = steer, thr, brk

        msg = CarlaEgoVehicleControl()
        msg.steer = float(clamp(steer, -1.0, 1.0))
        msg.throttle = float(clamp(thr, 0.0, 1.0))
        msg.brake = float(clamp(brk, 0.0, 1.0))
        msg.reverse = bool(v_ref < 0.0)

        # on timeout: prefer full brake
        if timed_out:
            msg.throttle = 0.0
            msg.brake = 1.0

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToCarlaBetter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
