import numpy as np
import math
from qpsolvers import solve_qp
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import matplotlib
import matplotlib.pyplot as plt
from numpy.linalg import norm
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped, Twist
from time import monotonic
import os
import csv
from ogm_cbf_kinematic_sim.utils import world_to_pixel
from time import time
import sys
from ogm_cbf_kinematic_sim.conf import controller_frequency, simulator_frequency
from rcl_interfaces.msg import SetParametersResult


def bilinear(arr, x, y , debug=False):
    H, W = arr.shape
    if x < 0 or x > W - 1 or y < 0 or y > H - 1:
        # outside map, choose convention
        return 0.0

    x0 = int(np.floor(x))
    y0 = int(np.floor(y))
    x1 = min(x0 + 1, W - 1)
    y1 = min(y0 + 1, H - 1)

    dx = x - x0
    dy = y - y0

    Q11 = arr[y0, x0]
    Q21 = arr[y0, x1]
    Q12 = arr[y1, x0]
    Q22 = arr[y1, x1]

    val = float(
        Q11 * (1 - dx) * (1 - dy) +
        Q21 * dx       * (1 - dy) +
        Q12 * (1 - dx) * dy       +
        Q22 * dx       * dy
    )
    if debug:
        print(
            "bilinear:",
            f"x={x:.6f}, y={y:.6f}, x0={x0}, y0={y0}, x1={x1}, y1={y1}, "
            f"dx={dx:.6f}, dy={dy:.6f}, "
            f"Q11={float(Q11):.6f}, Q21={float(Q21):.6f}, "
            f"Q12={float(Q12):.6f}, Q22={float(Q22):.6f}, val={val:.6f}"
        )
    return val


matplotlib.use('Agg')
vel_prev = 0
dPsi_prev = 0

def normalize_angle(angle):
    """Normalize an angle to the range [0, 2*pi]."""
    angle = angle % (2.0 * np.pi)
    if angle < 0.0:
        angle += 2.0 * np.pi
    return angle

def normalize_difference(angle):
    """Normalize an angle to the range [-pi, pi]."""
    angle = angle % (2.0 * np.pi)
    if angle < 0.0:
        angle += 2.0 * np.pi
    if angle > np.pi:
        angle -= 2.0 * np.pi
    return angle

def min_pool_2x2_free(free_u8):  # conservative: obstacles grow
    H, W = free_u8.shape
    Hp = H + (H % 2)
    Wp = W + (W % 2)
    pad = np.zeros((Hp, Wp), dtype=free_u8.dtype)  # pad as obstacle (0) => conservative
    pad[:H, :W] = free_u8
    blk = pad.reshape(Hp//2, 2, Wp//2, 2)
    return blk.min(axis=(1,3)).astype(np.uint8)  # 255 only if all 4 free


def gaussian_pyr_down(img_u8, K=4):
    pyr = [img_u8]
    x = img_u8
    for _ in range(K-1):
        x = cv2.pyrDown(x)     # Gaussian low-pass + /2
        pyr.append(x)
    return pyr

def signed_sdf_and_grad_from_free(free_u8):
    free = free_u8
    obs  = 255 - free
    phi_safe   = cv2.distanceTransform(free, cv2.DIST_L2, 3, dstType=cv2.CV_32F) - 1.0
    phi_unsafe = cv2.distanceTransform(obs,  cv2.DIST_L2, 3, dstType=cv2.CV_32F)
    sdf_px = (-phi_unsafe + phi_safe).astype(np.float32)  # signed distance in pixels

    dy, dx = np.gradient(sdf_px)
    dsdfx = dx.astype(np.float32)
    dsdfy = (-dy).astype(np.float32)  # pixel->Cartesian
    return sdf_px, dsdfx, dsdfy

class MobileRobot(Node):
    def __init__(self, state=[0,0,0], timestep=0.1):
        super().__init__('CBF_Controller_Node')
        self.vehicle_info = self.create_subscription(Odometry, "odom_2", self.vehicle_odom_callback, 1)
        self.subscription_2 = self.create_subscription(Image, 'map_image', self.listener_callback_map, 1)
        self.publisher_image_ = self.create_publisher(Image, '/cbf_image', 1)
        self.contour_timer_ = self.create_timer(1.0, self.publish_image)
        self.bridge = CvBridge()
        self.publisher_cbf_ = self.create_publisher(Float64MultiArray, '/cbf_array', 1)
        self.publisher_plot_twist_ = self.create_publisher(TwistStamped, '/plot_vel', 1)
        self.twist_publisher_ = self.create_publisher(Twist, 'cmd_vel_2', 1)

        vel_pub_time = 1.0 / controller_frequency
        self.twist_timer = self.create_timer(vel_pub_time, self.publish_velocity)

        self.publisher_erode_image_ = self.create_publisher(Image, '/erode_map_image', 1)
        self.contour_timer_2_ = self.create_timer(1.0, self.publish_erode_image)

        self.start_time = self.get_clock().now().nanoseconds

        # Initialize state variables
        self.x = self.y = self.yaw = 0.0
        self.lin_x = self.ang_w = 0.0
        self.linear_velocity = self.angular_velocity = 0.0
        self.x_init = self.y_init = self.yaw_init = 0.0
        self.counter = 0.0

        self.prev_odom_t_ns = None
        self.dt = 1.0 / controller_frequency  # fallback
        

        # Global map and dimensions (will be updated on map reception)
        self.global_map = None
        self.map = None
        self.map_height = 146#None
        self.map_width = 192#None
        self.recieved_map = False


        # Do not preallocate arrays with fixed dimensions;
        # they will be allocated when the map is received.
        self.sdf = None
        self.dsdf_x = None
        self.dsdf_y = None
        self.dsdf_x_normalized = None
        self.dsdf_y_normalized = None
        self.grad_sdf = None
        self.grad_sdf_normalized = None
        self.ddsdf_xx = self.ddsdf_yy = self.ddsdf_xy = self.ddsdf_yx = None
        self.fig, self.ax = plt.subplots(1, 1)
        self.Vx = self.Vy = self.Vz = 0.0

        # Initialize image holder; will be set after processing
        self.h_img = None
        self.time_map = 0.0
        self.cbf_array = [0.0]
        self.linear_velocity = self.angular_velocity = 0.0

        # Controller hyperparameters
        self.declare_parameter('C_alpha',     0.9)
        self.declare_parameter('Vmin',       -0.5)
        self.declare_parameter('Vmax',        0.5)
        self.declare_parameter('Wmin', -0.25*np.pi)
        self.declare_parameter('Wmax',  0.25*np.pi)

        self.declare_parameter('Delta_lb',   -1.0)
        self.declare_parameter('Delta_ub',    1.0)

        # CBF-specific offsets
        self.declare_parameter('l_a',      0.25)
        self.declare_parameter('l_s',     -0.25)
        self.declare_parameter('target_heading', 0.0) #radians

        




        # self.file_path = "cbf_data.csv"
        # with open(self.file_path, mode="w", newline="") as file:
        #     writer = csv.writer(file)
        #     writer.writerow(["Time (s)", "CBF Array[0]", "CBF Array[1]"])

        self.cbf_prev = 0.0
        self.time_cbf_prev = 0.0
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_erode = np.zeros((self.map_height, self.map_width), dtype=np.uint8)

        self.declare_parameter('pyr_levels', 5)

        self.pyr_levels = self.get_parameter('pyr_levels').value
        self.sdf_levels = []         # list[np.ndarray float32]  (pixels)
        self.dsdfx_levels = []       # list[np.ndarray float32]
        self.dsdfy_levels = []
        self.res_levels = []         # list[float] meters/pixel
        self.ddsdf_xx_levels = []
        self.ddsdf_xy_levels = []
        self.ddsdf_yx_levels = []
        self.ddsdf_yy_levels = []

        # ---- SDF publish params ----
        self.declare_parameter('sdf_pub', True)              # enable publishing
        self.declare_parameter('sdf_draw_grad', False)       # FLAG: arrows on/off
        self.declare_parameter('sdf_pub_hz', 1.0)
        self.declare_parameter('sdf_arrow_step', 25)         # px grid step (per level)
        self.declare_parameter('sdf_arrow_len', 12)          # px arrow length

        self.sdf_pub       = bool(self.get_parameter('sdf_pub').value)
        self.sdf_draw_grad = bool(self.get_parameter('sdf_draw_grad').value)
        self.sdf_pub_hz    = float(self.get_parameter('sdf_pub_hz').value)
        self.sdf_arrow_step= int(self.get_parameter('sdf_arrow_step').value)
        self.sdf_arrow_len = float(self.get_parameter('sdf_arrow_len').value)

        self.publisher_sdf_levels_ = [
            self.create_publisher(Image, f'/sdf_level_{k}', 1)
            for k in range(self.pyr_levels)
        ]
        self.sdf_timer_ = self.create_timer(1.0 / max(1e-3, self.sdf_pub_hz), self.publish_sdf_levels)

        self.add_on_set_parameters_callback(self._on_params)



    def _on_params(self, params):
        for p in params:
            if p.name == 'sdf_pub':        self.sdf_pub = bool(p.value)
            if p.name == 'sdf_draw_grad':  self.sdf_draw_grad = bool(p.value)
            if p.name == 'sdf_pub_hz':     self.sdf_pub_hz = float(p.value)
            if p.name == 'sdf_arrow_step': self.sdf_arrow_step = int(p.value)
            if p.name == 'sdf_arrow_len':  self.sdf_arrow_len = float(p.value)
        return SetParametersResult(successful=True)

    def sdf_to_rgb(self, sdf_px: np.ndarray) -> np.ndarray:
        s = np.nan_to_num(sdf_px, nan=0.0, posinf=0.0, neginf=0.0).astype(np.float32)
        s_u8 = cv2.normalize(s, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        bgr  = cv2.applyColorMap(s_u8, cv2.COLORMAP_VIRIDIS)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def overlay_grad_arrows(self, rgb, dsdfx, dsdfy, step, arrow_len):
        H, W = dsdfx.shape
        step = max(1, int(step))
        L    = float(arrow_len)
        for y in range(0, H, step):
            for x in range(0, W, step):
                gx = float(dsdfx[y, x])
                gy = float(-dsdfy[y, x])          # dsdfy is Cartesian-up; image y is down
                mag = math.hypot(gx, gy)
                if mag < 1e-6: 
                    continue
                ux, uy = gx / mag, gy / mag
                x2 = int(round(x + ux * L))
                y2 = int(round(y + uy * L))
                cv2.arrowedLine(rgb, (x, y), (x2, y2), (255, 0, 0), 1, tipLength=0.3)
        return rgb

    def publish_sdf_levels(self):
        if (not self.sdf_pub) or (len(self.sdf_levels) == 0):
            return

        stamp = self.time_map.to_msg() if isinstance(self.time_map, Time) else self.get_clock().now().to_msg()

        for k in range(min(self.pyr_levels, len(self.sdf_levels))):
            rgb = self.sdf_to_rgb(self.sdf_levels[k])

            if self.sdf_draw_grad:
                rgb = self.overlay_grad_arrows(rgb, self.dsdfx_levels[k], self.dsdfy_levels[k],
                                            self.sdf_arrow_step, self.sdf_arrow_len)

            msg = self.bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
            msg.header.stamp = stamp
            self.publisher_sdf_levels_[k].publish(msg)

   

    def publish_velocity(self):
        """Publish the velocity as a Twist message."""
       #twist in ego frame
        msg = Twist()
        msg.linear.x = self.linear_velocity # or use self.linear_velocity if available
        msg.linear.y = 0.0
        msg.angular.z = self.angular_velocity
        self.twist_publisher_.publish(msg)

    def process_image(self):
        """Visualize the phi_s and its gradients on an image."""
        if self.sdf is None:
            return
        final = self.sdf
        edges_x = self.dsdf_x
        edges_y = self.dsdf_y
        im_height, im_width = self.sdf.shape  # use dynamic dimensions
        self.ax.clear()
        dstep = max(1, im_width // 20)  # adjust the step size based on map width
        Y, X = np.mgrid[0:im_height:dstep, 0:im_width:dstep]
        self.ax.quiver(
            X, Y,
            edges_x[0:im_height:dstep, 0:im_width:dstep],
            edges_y[0:im_height:dstep, 0:im_width:dstep],
            color='r',
            # angles="xy",
            # scale_units="xy",
            # scale=1
        )
        t = self.ax.imshow(final, cmap='viridis', vmin=np.min(final), vmax=np.max(final))
        circle = plt.Circle((self.x, self.y), 5, fill=False, color='blue')
        self.ax.add_patch(circle)
        cax = self.fig.colorbar(t, ax=self.ax)
        canvas = plt.gca().figure.canvas
        canvas.draw()
        data = np.frombuffer(canvas.tostring_rgb(), dtype=np.uint8)
        image = data.reshape(canvas.get_width_height()[::-1] + (3,))
        canvas.flush_events()
        plt.pause(0.1)
        cax.remove()
        self.h_img = image

    def publish_image(self):
        """Publish the constructed image on ROS."""
        try:
            self.process_image()
            if self.h_img is not None:
                img_msg = self.bridge.cv2_to_imgmsg(self.h_img)
                # Use the map's timestamp if available
                img_msg.header.stamp = self.time_map.to_msg() if isinstance(self.time_map, Time) else self.get_clock().now().to_msg()
                self.publisher_image_.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Error in publish_image: {e}")

    def publish_erode_image(self):
        """Publish the constructed image on ROS."""
        img_msg = self.bridge.cv2_to_imgmsg(self.map_erode)
        # Use the map's timestamp if available
        img_msg.header.stamp = self.time_map.to_msg() if isinstance(self.time_map, Time) else self.get_clock().now().to_msg()
        self.publisher_erode_image_.publish(img_msg)
       


    def listener_callback_map(self, msg: Image):
        """
        When a map is received, use its dimensions to create the sdf and gradient arrays.
        """
        #if not self.recieved_map:
        self.time_map = Time.from_msg(msg.header.stamp)
        map_img = self.bridge.imgmsg_to_cv2(msg)
        # Binarize the map image
        _, map_img = cv2.threshold(map_img, 127, 255, cv2.THRESH_BINARY)
        map_img = np.asarray(map_img)

        map_img = cv2.bitwise_not(map_img)  # Invert colors: obstacles=255, free=0

        K = self.pyr_levels
        #free_k = map_img.copy()

        free_pyr = gaussian_pyr_down(map_img, K=self.pyr_levels)

        self.sdf_levels.clear()
        self.dsdfx_levels.clear()
        self.dsdfy_levels.clear()
        self.res_levels.clear()

        self.ddsdf_xx_levels.clear()
        self.ddsdf_xy_levels.clear()
        self.ddsdf_yx_levels.clear()
        self.ddsdf_yy_levels.clear()


        for k, img in enumerate(free_pyr):
            # optional: extra conservative inflation per level (dilate obstacles == erode free)
            # free_k = cv2.erode(free_k, kernel, iterations=infl_iters_k)

            free_k = ((img > 127).astype(np.uint8) * 255)


            sdf_px, dsdfx, dsdfy = signed_sdf_and_grad_from_free(free_k)

            # Hessian in pixel space 
            edges_y, edges_x = np.gradient(sdf_px)
            norm = np.sqrt(edges_x**2 + edges_y**2)
            edges_x /= norm + 1e-6
            edges_y /= norm + 1e-6

            # we only get derivative of normalized dsdf because that is what we use in CBF

            gyy, gyx = np.gradient(edges_y)
            gxy, gxx = np.gradient(edges_x)


            ddsdf_xx = gxx
            ddsdf_yy = gyy
            ddsdf_xy = -gxy
            ddsdf_yx = -gyx

            self.ddsdf_xx_levels.append(ddsdf_xx.astype(np.float32))
            self.ddsdf_xy_levels.append(ddsdf_xy.astype(np.float32))
            self.ddsdf_yx_levels.append(ddsdf_yx.astype(np.float32))
            self.ddsdf_yy_levels.append(ddsdf_yy.astype(np.float32))

            self.sdf_levels.append(sdf_px)
            self.dsdfx_levels.append(dsdfx)
            self.dsdfy_levels.append(dsdfy)
            self.res_levels.append(self.map_resolution * (2**k))
            #free_k = min_pool_2x2_free(free_k)   # next level
            


    def sdf_at_world_level(self, k, xw, yw):
        sdf = self.sdf_levels[k]
        res = self.res_levels[k]
        H = sdf.shape[0]
        px, py = world_to_pixel(xw, yw, resolution=res,
                                origin_x=self.map_origin_x, origin_y=self.map_origin_y,
                                img_height=H, continuous=True)
        return bilinear(sdf, px, py) * res  # meters

    def grad_at_world_level(self, k, xw, yw, normalize=False):
        dsx = self.dsdfx_levels[k]
        dsy = self.dsdfy_levels[k]
        res = self.res_levels[k]
        H = dsx.shape[0]
        px, py = world_to_pixel(xw, yw, resolution=res,
                                origin_x=self.map_origin_x, origin_y=self.map_origin_y,
                                img_height=H, continuous=True)
        if normalize:
            gx = bilinear(dsx, px, py)
            gy = bilinear(dsy, px, py)
            norm = math.sqrt(gx**2 + gy**2)
            # print(f"grad_at_world_level normalize: gx={gx:.6f}, gy={gy:.6f}, norm={norm:.6f}")
            gx /= norm + 1e-6
            gy /= norm + 1e-6
            return gx, gy  # ~ ∂φ/∂x, ∂φ/∂y (unitless)
        
        gx = bilinear(dsx, px, py)
        gy = bilinear(dsy, px, py)
        
        return gx, gy  # ~ ∂φ/∂x, ∂φ/∂y (m/m)
    

    def hessian_at_world_level(self, k, xw, yw):
        res = self.res_levels[k]
        H = self.sdf_levels[k].shape[0]
        px, py = world_to_pixel(xw, yw, resolution=res,
                                origin_x=self.map_origin_x, origin_y=self.map_origin_y,
                                img_height=H, continuous=True)

        dxx = bilinear(self.ddsdf_xx_levels[k], px, py)
        dxy = bilinear(self.ddsdf_xy_levels[k], px, py)
        dyx = bilinear(self.ddsdf_yx_levels[k], px, py)
        dyy = bilinear(self.ddsdf_yy_levels[k], px, py)
        return dxx/res, dxy/res, dyx/res, dyy/res



            

    def vehicle_odom_callback(self, msg):
        """
        Receive the vehicle's odometry and update the state.
        """
        t_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if self.prev_odom_t_ns is None:
            self.dt = 1.0 / controller_frequency
        else:
            self.dt = max((t_ns - self.prev_odom_t_ns) * 1e-9, 1e-6)  # seconds, clamp
        self.prev_odom_t_ns = t_ns


        global vel_prev, dPsi_prev
        # Initialize the pixel pose based on the first odometry message
        if self.counter == 0:
            self.x_init_real = msg.pose.pose.position.x
            self.y_init_real = msg.pose.pose.position.y
            _, _, self.yaw_init = euler_from_quaternion((
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w))
            
        self.counter += 1

        self.x_real = msg.pose.pose.position.x
        self.y_real = msg.pose.pose.position.y
        _, _, self.yaw = euler_from_quaternion((
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w))
        self.Vx = msg.twist.twist.linear.x
        self.Vy = msg.twist.twist.linear.y
        self.Vz = msg.twist.twist.linear.z

        if len(self.sdf_levels) > 0:
            #t_1 = time()
            self.controller()
            #t_2 = time()
            #print(f"---------------------------------Controller computation time: {t_2 - t_1:.6f} seconds")
            #self.publish_cbf()
            #self.publish_plot_twist()

    # def publish_cbf(self):
    #     """
    #     Publish the CBF array (for debugging) and log the data.
    #     """
    #     try:
    #         msg = Float64MultiArray()
    #         now = self.get_clock().now().nanoseconds
    #         elapsed_time = (now - self.start_time) / 1e9
    #         with open(self.file_path, mode="a", newline="") as file:
    #             writer = csv.writer(file)
    #             writer.writerow([elapsed_time, self.cbf_array[0], self.cbf_array[1], self.cbf_array[2]])
    #         data = self.cbf_array + [0.0]
    #         msg.data = data
    #         self.publisher_cbf_.publish(msg)
    #     except Exception as e:
    #         self.get_logger().error(f"Error in publish_cbf: {e}")

    def cbf_terms_level(self, k, xw, yw, yaw):
        sdf = self.sdf_at_world_level(k, xw, yw)
        dsdf_x, dsdf_y = self.grad_at_world_level(k, xw, yw, normalize=False) 
        dsdf_x_norm, dsdf_y_norm = self.grad_at_world_level(k, xw, yw, normalize=True)      
        dxx, dxy, dyx, dyy = self.hessian_at_world_level(k, xw, yw)

        

        l_a = 0.25
        l_s = -l_a

        g = np.array([dsdf_x_norm, dsdf_y_norm])
        x = np.array([np.cos(yaw), np.sin(yaw)])           
        x_perp = np.array([-np.sin(yaw), np.cos(yaw)]) 


        x = np.around(x, decimals=6)
        x_perp = np.around(x_perp, decimals=6)
        g = np.around(g, decimals=6)
        sdf = np.around(sdf, decimals=6)
        dsdf_x = np.around(dsdf_x, decimals=6)
        dsdf_y = np.around(dsdf_y, decimals=6)


        cbf = sdf + l_s + l_a*(g @ x)                      

        g_x = np.array([dxx, dyx])                         
        g_y = np.array([dxy, dyy])

        dcbf_x   = dsdf_x + l_a*(g_x @ x)
        dcbf_y   = dsdf_y + l_a*(g_y @ x)
        dcbf_yaw = l_a*(g @ x_perp)

        cbf = np.around(cbf, decimals=6)
        dcbf_x = np.around(dcbf_x, decimals=6)
        dcbf_y = np.around(dcbf_y, decimals=6)
        dcbf_yaw = np.around(dcbf_yaw, decimals=6)

        return cbf, dcbf_x, dcbf_y, dcbf_yaw


    def controller(self):
        """
        The CBF-CLF-QP controller implementation.
        (The equations remain mostly unchanged; make sure that all indexing now uses the dynamic map dimensions.)
        """
        global vel_prev, dPsi_prev
        # Hyperparameters and reference values
        C_alpha = self.get_parameter('C_alpha').value 

        Kv = 1.0
        Kw = 0.01
        Kd = 1.0
        
        Vmax = self.get_parameter('Vmax').value 
        Vmin = self.get_parameter('Vmin').value 
        Wmax = self.get_parameter('Wmax').value 
        Wmin = self.get_parameter('Wmin').value 

        
        
        Delta_ub = self.get_parameter('Delta_ub').value
        Delta_lb = self.get_parameter('Delta_lb').value

        target_heading = normalize_angle(self.get_parameter('target_heading').value)

        xw = self.x_real
        yw = self.y_real
        yaw = self.yaw

        
        G_rows = [
        [ 1,0,0], [-1,0,0],
        [ 0,1,0], [ 0,-1,0],
        [ 0,0,1], [ 0,0,-1],
        ]
        h_rows = [Vmax, -Vmin, Wmax, -Wmin, Delta_ub, -Delta_lb]

        for k in range(self.pyr_levels):
            cbf, dcbf_x, dcbf_y, dcbf_yaw = self.cbf_terms_level(k, xw, yw, yaw)

            
            self.get_logger().info(f"Level {k}: CBF={cbf:.6f}")
            if cbf < 0.0:
                self.get_logger().warn(f"*** WARNING: CBF level {k} is negative: {cbf:.6f} ***")
                sys.exit(1)

            Av = dcbf_x*np.cos(yaw) + dcbf_y*np.sin(yaw)   
            Bw = dcbf_yaw                                  

            G_rows.append([-Av, -Bw, 0.0])
            h_rows.append(C_alpha * cbf)

        

        G = np.array(G_rows, dtype=float)
        h_vec = np.array(h_rows, dtype=float)


        Vref = Vmax
        K_Wref = 0.5
        Wref = K_Wref * normalize_difference(target_heading - yaw)
        P_mat = np.diag([Kv, Kw, Kd])
        q = np.array([-Kv * Vref, -Kw * Wref, 0.0])

        try:
            [vel, dPsi, Delta] = solve_qp(P_mat, q, G, h_vec, solver='quadprog')

            # sol = np.array([vel, dPsi, Delta], dtype=float)
            # viol = np.max(G @ sol - h_vec)   # <= 0 is satisfied
            # self.get_logger().info(f"max_violation: {viol}")


            self.linear_velocity = float(vel)
            self.angular_velocity = float(dPsi)
        except Exception as e:
            self.get_logger().error(f"QP solver error: {e}")
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            sys.exit(1)


       

        


def main(args=None):
    rclpy.init(args=args)
    node = MobileRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
