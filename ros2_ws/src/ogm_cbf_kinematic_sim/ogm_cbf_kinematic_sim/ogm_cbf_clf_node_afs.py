#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float32, Float64MultiArray

from tf_transformations import euler_from_quaternion
from cv_bridge import CvBridge
import cv2

from qpsolvers import solve_qp

from ogm_cbf_kinematic_sim.utils import world_to_pixel
from ogm_cbf_kinematic_sim.conf import controller_frequency


# def bilinear(arr: np.ndarray, x: float, y: float) -> float:
#     H, W = arr.shape
#     if x < 0 or x > W - 1 or y < 0 or y > H - 1:
#         return 0.0
#     x0 = int(np.floor(x)); y0 = int(np.floor(y))
#     x1 = min(x0 + 1, W - 1); y1 = min(y0 + 1, H - 1)
#     dx = x - x0; dy = y - y0
#     Q11 = arr[y0, x0]; Q21 = arr[y0, x1]
#     Q12 = arr[y1, x0]; Q22 = arr[y1, x1]
#     return float(
#         Q11 * (1 - dx) * (1 - dy) +
#         Q21 * dx       * (1 - dy) +
#         Q12 * (1 - dx) * dy       +
#         Q22 * dx       * dy
#     )


def bilinear(arr: np.ndarray, x: float, y: float, eps: float = 1e-6) -> float:
    H, W = arr.shape
    # keep inside, avoid x==W-1 / y==H-1 which degenerates indices
    x = float(np.clip(x, 0.0, (W - 1) - eps))
    y = float(np.clip(y, 0.0, (H - 1) - eps))

    x0 = int(np.floor(x)); y0 = int(np.floor(y))
    x1 = x0 + 1;         y1 = y0 + 1

    dx = x - x0; dy = y - y0

    # edge-safe fetch (equiv to edge padding)
    x0c = np.clip(x0, 0, W-1); x1c = np.clip(x1, 0, W-1)
    y0c = np.clip(y0, 0, H-1); y1c = np.clip(y1, 0, H-1)

    Q11 = arr[y0c, x0c]; Q21 = arr[y0c, x1c]
    Q12 = arr[y1c, x0c]; Q22 = arr[y1c, x1c]

    return float(
        Q11 * (1 - dx) * (1 - dy) +
        Q21 * dx       * (1 - dy) +
        Q12 * (1 - dx) * dy       +
        Q22 * dx       * dy
    )



def wrap_pi(a: float) -> float:
    a = (a + math.pi) % (2.0 * math.pi) - math.pi
    return float(a)


class OGMCBFCLF_AFS(Node):
    """
    AFS version of your OGM-CBF-CLF node.

    State used: (x,y,yaw) from odom_2 + beta from /beta.
    Control: u = [v, beta_dot, Delta]
      - publish Twist: linear.x=v , angular.z=beta_dot  (AFS sim interprets angular.z as beta_dot)
    """

    def __init__(self):
        super().__init__("ogm_cbf_clf_afs")

        # --- subs/pubs (match your sim naming) ---
        self.sub_odom = self.create_subscription(Odometry, "odom_2", self.odom_cb, 1)
        self.sub_beta = self.create_subscription(Float32, "beta", self.beta_cb, 1)
        self.sub_map = self.create_subscription(Image, "map_image", self.map_cb, 1)

        self.pub_cmd = self.create_publisher(Twist, "cmd_vel_2", 1)
        self.pub_plot = self.create_publisher(TwistStamped, "/plot_vel", 1)
        self.pub_cbf = self.create_publisher(Float64MultiArray, "/cbf_array", 1)

        self.bridge = CvBridge()

        self.timer = self.create_timer(1.0 / controller_frequency, self.control_step)

        # --- AFS geometry ---
        self.declare_parameter("l_f", 1.0)
        self.declare_parameter("l_r", 1.0)
        self.l_f = float(self.get_parameter("l_f").value)
        self.l_r = float(self.get_parameter("l_r").value)

        # articulation limits (soft via CBF constraints)
        self.declare_parameter("beta_max", float(np.deg2rad(40.0)))
        self.beta_max = float(self.get_parameter("beta_max").value)

        # --- CBF/CLF params ---
        self.declare_parameter("C_alpha", 0.9)   # cbf rate
        self.declare_parameter("C_gamma", 1.0)          # clf rate

        # bounds
        self.declare_parameter("Vmin", -2.0)
        self.declare_parameter("Vmax",  2.0)
        self.declare_parameter("Bdot_min", -2.0)
        self.declare_parameter("Bdot_max",  2.0)
        self.declare_parameter("Delta_lb", -0.5)
        self.declare_parameter("Delta_ub",  0.5)

        # objective weights (QP)
        self.declare_parameter("w_v", 1.0)
        self.declare_parameter("w_bdot", 0.01)
        self.declare_parameter("w_delta", 1.0)

        # your CBF geometry offsets
        self.declare_parameter("l_a", 0.25)
        self.declare_parameter("l_b", 0.25)
        # keep your choice (negative) for now
        self.declare_parameter("l_s", float(-math.sqrt(0.25**2 + 0.25**2)))
        self.declare_parameter("eps_abs", 1e-4)  # smooth |.| for symmetry

        # simple heading "planner"
        self.declare_parameter("heading_ref", float(math.pi / 4.0))
        self.declare_parameter("v_ref", 0.5)
        self.declare_parameter("k_heading", 0.5)  # omega_ref = k*(heading_ref - yaw)

        # beta-limit CBF gain
        #self.declare_parameter("beta_alpha", 2.0)

        # map meta
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.recieved_map = False

        # arrays
        self.sdf = None
        self.dsdf_x = None
        self.dsdf_y = None
        self.ddsdf_xx = None
        self.ddsdf_xy = None
        self.ddsdf_yx = None
        self.ddsdf_yy = None

        # state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.beta = 0.0

        self.vx_w = 0.0
        self.vy_w = 0.0
        self.yaw_rate = 0.0

        self.prev_odom_t_ns = None
        self.dt = 1.0 / controller_frequency

        # for debug
        self.cbf_prev = 0.0

    # ---------- map + SDF ----------
    def map_cb(self, msg: Image):
        if self.recieved_map:
            return

        map_img = self.bridge.imgmsg_to_cv2(msg)
        _, map_img = cv2.threshold(map_img, 127, 255, cv2.THRESH_BINARY)
        map_img = np.asarray(map_img)
        map_img = cv2.bitwise_not(map_img)  # obstacles=255, free=0

        # small erosion (keep your style)
        k = 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1))
        map_img = cv2.erode(map_img, kernel, iterations=1)

        self.map_height, self.map_width = map_img.shape
        self.recieved_map = True

        map_not = (255 - map_img).astype(np.uint8)
        map_img_u8 = map_img.astype(np.uint8)

        # IMPORTANT: use CV_32F (no saturation)
        phi_safe = cv2.distanceTransform(map_img_u8, cv2.DIST_L2, 3, dstType=cv2.CV_32F) - 1.0
        phi_unsafe = -cv2.distanceTransform(map_not, cv2.DIST_L2, 3, dstType=cv2.CV_32F)

        self.sdf = (phi_safe + phi_unsafe).astype(np.float32)

        edges_y, edges_x = np.gradient(self.sdf)
        self.dsdf_x = edges_x
        self.dsdf_y = -edges_y  # Cartesian adjust

        gyy, gyx = np.gradient(edges_y)
        gxy, gxx = np.gradient(edges_x)

        self.ddsdf_xx = gxx
        self.ddsdf_yy = gyy
        self.ddsdf_xy = -gxy
        self.ddsdf_yx = -gyx

        self.get_logger().info(f"SDF ready: {self.map_width}x{self.map_height}")

    def sdf_at_world(self, x_world: float, y_world: float) -> float:
        img_height = self.sdf.shape[0]
        px, py = world_to_pixel(
            x_world, y_world,
            resolution=self.map_resolution,
            origin_x=self.map_origin_x,
            origin_y=self.map_origin_y,
            img_height=img_height,
            continuous=True,
        )
        # convert px->m
        return bilinear(self.sdf, px, py) * self.map_resolution

    def sdf_grad_at_world(self, x_world: float, y_world: float):
        img_height = self.sdf.shape[0]
        px, py = world_to_pixel(
            x_world, y_world,
            resolution=self.map_resolution,
            origin_x=self.map_origin_x,
            origin_y=self.map_origin_y,
            img_height=img_height,
            continuous=True,
        )
        # numeric derivative invariant under uniform scaling, so return as-is
        gx = bilinear(self.dsdf_x, px, py)
        gy = bilinear(self.dsdf_y, px, py)
        return gx, gy

    def hessian_at_world(self, x_world: float, y_world: float):
        img_height = self.sdf.shape[0]
        px, py = world_to_pixel(
            x_world, y_world,
            resolution=self.map_resolution,
            origin_x=self.map_origin_x,
            origin_y=self.map_origin_y,
            img_height=img_height,
            continuous=True,
        )
        dxx = bilinear(self.ddsdf_xx, px, py)
        dxy = bilinear(self.ddsdf_xy, px, py)
        dyx = bilinear(self.ddsdf_yx, px, py)
        dyy = bilinear(self.ddsdf_yy, px, py)
        # px-units -> 1/m^2 not needed here; kept consistent with your current usage
        return dxx/self.map_resolution, dxy/self.map_resolution, dyx/self.map_resolution, dyy/self.map_resolution

    # ---------- state ----------
    def beta_cb(self, msg: Float32):
        self.beta = float(msg.data)

    def odom_cb(self, msg: Odometry):
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)

        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.yaw = float(yaw)

        self.vx_w = float(msg.twist.twist.linear.x)
        self.vy_w = float(msg.twist.twist.linear.y)
        self.yaw_rate = float(msg.twist.twist.angular.z)

        t_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if self.prev_odom_t_ns is not None:
            dt = (t_ns - self.prev_odom_t_ns) * 1e-9
            if dt > 1e-6:
                self.dt = float(dt)
        self.prev_odom_t_ns = t_ns

    # ---------- CBF formula (your symmetric version) ----------
    def cbf_and_derivs(self):
        sdf = self.sdf_at_world(self.x, self.y)
        dsx, dsy = self.sdf_grad_at_world(self.x, self.y)
        dxx, dxy, dyx, dyy = self.hessian_at_world(self.x, self.y)

        yaw = self.yaw
        l_a = float(self.get_parameter("l_a").value)
        l_b = float(self.get_parameter("l_b").value)
        l_s = float(self.get_parameter("l_s").value)
        eps_abs = float(self.get_parameter("eps_abs").value)

        g = np.array([dsx, dsy], dtype=float)
        x = np.array([math.cos(yaw), math.sin(yaw)], dtype=float)
        x_perp = np.array([-math.sin(yaw), math.cos(yaw)], dtype=float)

        # smooth |g·x_perp|
        z = float(g @ x_perp)
        absz = math.sqrt(z * z + eps_abs)
        chain = z / absz

        cbf = float(sdf + l_s + l_a * float(g @ x) + l_b * absz)

        # Hessian columns: ∂g/∂x, ∂g/∂y
        g_x = np.array([dxx, dyx], dtype=float)
        g_y = np.array([dxy, dyy], dtype=float)

        z_x = float(g_x @ x_perp)
        z_y = float(g_y @ x_perp)

        dcbf_x = float(dsx + l_a * float(g_x @ x) + l_b * chain * z_x)
        dcbf_y = float(dsy + l_a * float(g_y @ x) + l_b * chain * z_y)
        dcbf_yaw = float(l_a * float(g @ x_perp) - l_b * chain * float(g @ x))

        return cbf, dcbf_x, dcbf_y, dcbf_yaw

    # ---------- controller ----------
    def control_step(self):
        if not self.recieved_map or self.sdf is None:
            self.pub_cmd.publish(Twist())
            return

        C_alpha = float(self.get_parameter("C_alpha").value)
        C_gamma = float(self.get_parameter("C_gamma").value)

        Vmin = float(self.get_parameter("Vmin").value)
        Vmax = float(self.get_parameter("Vmax").value)
        Bmin = float(self.get_parameter("Bdot_min").value)
        Bmax = float(self.get_parameter("Bdot_max").value)
        Dlb = float(self.get_parameter("Delta_lb").value)
        Dub = float(self.get_parameter("Delta_ub").value)

        w_v = float(self.get_parameter("w_v").value)
        w_b = float(self.get_parameter("w_bdot").value)
        w_d = float(self.get_parameter("w_delta").value)

        # --- desired motion (same “simple heading follower” pattern) ---
        heading_ref = float(self.get_parameter("heading_ref").value)
        v_ref = float(self.get_parameter("v_ref").value)
        k_head = float(self.get_parameter("k_heading").value)

        e = wrap_pi(heading_ref - self.yaw)
        omega_ref = k_head * e

        # measured front-body speed (from world vel projected onto body axis)
        v_meas = self.vx_w * math.cos(self.yaw) + self.vy_w * math.sin(self.yaw)

        # ω -> beta_dot conversion (front body)
        denom = (self.l_f * math.cos(self.beta) + self.l_r)
        denom = max(1e-6, denom)
        beta_dot_ref = (-v_meas / self.l_r) * math.sin(self.beta) + (denom / self.l_r) * omega_ref

        # ---------- build QP: min 0.5 u^T P u + q^T u ----------
        # u = [v, beta_dot, Delta]
        P = np.diag([w_v, w_b, w_d]).astype(float)
        q = np.array([-w_v * v_ref, -w_b * beta_dot_ref, 0.0], dtype=float)

        # ---------- constraints G u <= h ----------
        G_list = []
        h_list = []

        # --- main OGM-CBF constraint ---
        cbf, dcbf_x, dcbf_y, dcbf_yaw = self.cbf_and_derivs()

        # AFS kinematics:
        # xdot = v*cos(yaw), ydot = v*sin(yaw)
        # yawdot = (v*sin(beta) + l_r*beta_dot)/denom
        A_v = dcbf_x * math.cos(self.yaw) + dcbf_y * math.sin(self.yaw) + dcbf_yaw * (math.sin(self.beta) / denom)
        A_b = dcbf_yaw * (self.l_r / denom)

        # enforce: dcbf/dt >= -alpha * cbf  ->  -A_v v - A_b beta_dot <= alpha*cbf
        G_list.append([-A_v, -A_b, 0.0])
        h_list.append(C_alpha * cbf)

        # --- CLF constraint on heading: V=0.5 e^2, dotV = e*yawdot
        V = 0.5 * e * e
        # e*yawdot - Delta <= -gamma*V
        # yawdot = (v*sin(beta) + l_r*beta_dot)/denom
        G_list.append([ (e * math.sin(self.beta) / denom),
                        (e * self.l_r / denom),
                       -1.0 ])
        h_list.append(-C_gamma * V)

        # # --- beta limits as 1st-order CBFs ---
        # beta_alpha = float(self.get_parameter("beta_alpha").value)
        # beta_min = -self.beta_max
        # beta_max = +self.beta_max

        # # h2 = beta - beta_min >=0 , dot = beta_dot
        # # beta_dot >= -a*(beta - beta_min)  ->  -beta_dot <= a*(beta - beta_min)
        # G_list.append([0.0, -1.0, 0.0])
        # h_list.append(beta_alpha * (self.beta - beta_min))

        # # h3 = beta_max - beta >=0 , dot = -beta_dot
        # # -beta_dot >= -a*(beta_max - beta) -> beta_dot <= a*(beta_max - beta)
        # G_list.append([0.0, 1.0, 0.0])
        # h_list.append(beta_alpha * (beta_max - self.beta))

        G = np.array(G_list, dtype=float)
        h = np.array(h_list, dtype=float)

        lb = np.array([Vmin, Bmin, Dlb], dtype=float)
        ub = np.array([Vmax, Bmax, Dub], dtype=float)

        # solve
        u = solve_qp(P, q, G, h, lb=lb, ub=ub, solver="quadprog")
        if u is None:
            # fallback: just track reference (clipped)
            v_cmd = float(np.clip(v_ref, Vmin, Vmax))
            b_cmd = float(np.clip(beta_dot_ref, Bmin, Bmax))
            d_cmd = 0.0
        else:
            #self.get_logger().error("Optimization failed, using previous solution.")
            v_cmd = float(0)
            b_cmd = float(0)
            d_cmd = float(0)

                # ---------- DEBUG (AFS equivalent of Av/Bw prints) ----------
        # pieces (same as diff-drive):
        #   A_trans = dcbf_x*cos(yaw) + dcbf_y*sin(yaw)
        #   cbf_dot = A_trans*v + dcbf_yaw*yawdot
        # but in AFS: yawdot depends on (v, beta_dot), so you also have:
        #   cbf_dot = A_v*v + A_b*beta_dot   (A_v,A_b already computed above)

        sdf = self.sdf_at_world(self.x, self.y)  # just for printing
        dsx, dsy = self.sdf_grad_at_world(self.x, self.y)
        g = np.array([dsx, dsy], dtype=float)
        xdir = np.array([math.cos(self.yaw), math.sin(self.yaw)], dtype=float)

        # signed angle from heading to gradient (optional debug)
        eta = math.atan2(g[0]*xdir[1] - g[1]*xdir[0], g[0]*xdir[0] + g[1]*xdir[1])

        # denom already computed above
        yawdot_cmd = (v_cmd * math.sin(self.beta) + self.l_r * b_cmd) / denom

        A_trans = dcbf_x * math.cos(self.yaw) + dcbf_y * math.sin(self.yaw)  # translation-only coeff
        rot_term = dcbf_yaw * yawdot_cmd

        cbf_dot = A_v * v_cmd + A_b * b_cmd
        cbf_dot_alpha_cbf = cbf_dot + C_alpha * cbf

        dt = max(self.dt, 1e-6)
        true_cbf_dot = (cbf - self.cbf_prev) / dt

        print(f"A_trans (like Av) = {A_trans:.2f}")
        print(f"dcbf_yaw (like B) = {dcbf_yaw:.2f}")
        print(f"QP: v={v_cmd:.2f}, beta_dot={b_cmd:.2f}, yawdot={yawdot_cmd:.2f}")
        print(f"A_v={A_v:.2f}, A_b={A_b:.2f}")
        print(f"A_v*v = {(A_v*v_cmd):.2f}")
        print(f"A_b*beta_dot = {(A_b*b_cmd):.2f}")
        print(f"trans = {(A_trans*v_cmd):.2f}, rot = {rot_term:.2f}, sum = {(A_trans*v_cmd + rot_term):.2f}")
        print(f"cbf={cbf:.2f}, cbf_dot={cbf_dot:.2f}, cbf_dot+alpha*cbf={cbf_dot_alpha_cbf:.2f}")
        print(f"cbf_dot(true)={true_cbf_dot:.2f}, diff={cbf_dot-true_cbf_dot:.2f}")
        print(f"sdf={sdf:.2f}, eta(deg)={math.degrees(eta):.2f}, cbf-sdf={(cbf-sdf):.2f}")
        print("--------------------------------------------------")

        self.cbf_prev = cbf  # keep for next step true derivative



        # publish cmd to sim
        tw = Twist()
        tw.linear.x = v_cmd
        tw.angular.z = b_cmd
        self.pub_cmd.publish(tw)

        # debug pubs
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.twist.linear.x = v_cmd
        ts.twist.angular.z = b_cmd
        self.pub_plot.publish(ts)

        arr = Float64MultiArray()
        arr.data = [float(cbf), float(A_v * v_cmd + A_b * b_cmd), float(d_cmd)]
        self.pub_cbf.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = OGMCBFCLF_AFS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
