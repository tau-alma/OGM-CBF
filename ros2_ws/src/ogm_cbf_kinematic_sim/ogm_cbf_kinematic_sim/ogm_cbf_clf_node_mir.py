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

class MobileRobot(Node):
    def __init__(self, state=[0,0,0], timestep=0.1):
        super().__init__('minimal_publisher')
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
        self.declare_parameter('C_alpha',     0.01 * 0.5)
        self.declare_parameter('Vmin',       -1.0)
        self.declare_parameter('Vmax',        1.0)
        self.declare_parameter('Wmin', -4*np.pi)
        self.declare_parameter('Wmax',  4*np.pi)

        self.declare_parameter('Delta_lb',   -0.5)
        self.declare_parameter('Delta_ub',    0.5)

        # CBF-specific offsets
        self.declare_parameter('l_a',      0.25)
        self.declare_parameter('l_s',     -0.25)
        self.declare_parameter('sdf_a',   3.0)

        self.file_path = "cbf_data.csv"
        with open(self.file_path, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Time (s)", "CBF Array[0]", "CBF Array[1]"])

        self.cbf_prev = 0.0
        self.time_cbf_prev = 0.0
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_erode = np.zeros((self.map_height, self.map_width), dtype=np.uint8)



    def sdf_at_world(self, x_world, y_world):

        """
        SDF value at continuous world pose (x_world,y_world) in meters.

        Steps:
            1) world (m) -> continuous pixel coords (px,py) via world_to_pixel(res, origin, img_height)
            2) bilinear interpolate self.sdf in pixel space (self.sdf is in *pixels*)
            3) convert to meters: multiply by map_resolution [m/pixel]

        Returns:
            sdf_m : float, signed distance in [m] (positive=free, negative=inside obstacle depending on construction).
        """

        img_height = self.sdf.shape[0]
        px, py = world_to_pixel(
            x_world, y_world,
            resolution=self.map_resolution,
            origin_x=self.map_origin_x,
            origin_y=self.map_origin_y,
            img_height=img_height,
            continuous=True,
        )
        return bilinear(self.sdf, px, py) * self.map_resolution

    def sdf_grad_at_world(self, x_world, y_world):

        """
        SDF gradient at continuous world pose (x_world,y_world).

        Notes:
        - self.dsdf_x, self.dsdf_y are computed by np.gradient(self.sdf) => derivatives in pixel units:
            dsdf_x ~ ∂(sdf_pixels)/∂px   , dsdf_y ~ ∂(sdf_pixels)/∂py
        - (coords+values both scaled), the numeric first-derivative is unchanged:
            ∂(sdf_m)/∂(x_m) == ∂(sdf_px)/∂(px)   (same numbers)
        - returning bilinear(dsdf_*) directly gives gradient in [m/m] (dimensionless slope) w.r.t world axes.

        Returns:
        (gx, gy): floats ~ (∂sdf/∂x_world, ∂sdf/∂y_world), units [m/m].
        """

        

        img_height = self.sdf.shape[0]
        px, py = world_to_pixel(
            x_world, y_world,
            resolution=self.map_resolution,
            origin_x=self.map_origin_x,
            origin_y=self.map_origin_y,
            img_height=img_height,
            continuous=True,
        )

        ix = int(np.floor(px))
        iy = int(np.floor(py))
        print(f"+++++++++++++++dsdfx is {self.dsdf_x[iy, ix]} dsdfy is {self.dsdf_y[iy, ix]}")


        
        gx = bilinear(self.dsdf_x, px, py, debug=True)
        gy = bilinear(self.dsdf_y, px, py, debug=True)
        vector_norm = math.sqrt(gx**2 + gy**2)
        gx /= vector_norm + 1e-8
        gy /= vector_norm + 1e-8
        return gx, gy
    
    def hessian_at_world(self, x_world, y_world, normalized=False):

        """
        Hessian of SDF at continuous world pose (x_world,y_world).

        What it returns:
        (dxx, dxy, dyx, dyy) where
            dxx = ∂²sdf/∂x², dxy = ∂²sdf/∂x∂y, dyx = ∂²sdf/∂y∂x, dyy = ∂²sdf/∂y²

        Scaling:
        - Second derivatives scale with 1/length².
        - stored second-derivative arrays are in pixel space (from np.gradient on pixel gradients),
        it is converted here to world by dividing by map_resolution [m/pixel] once per “extra” derivative order.
            (`/ self.map_resolution`.)

        normalized=True uses Hessian of normalized gradient field (curvature-ish direction field), not raw SDF.
        """

        # Chain rule: since (px,py)=world_to_pixel(x_world,y_world,res) and sdf_world = sdf_px*map_resolution,
        # ∂/∂x_world = (1/map_resolution)∂/∂px so ∂sdf_world/∂x_world = ∂sdf_px/∂px, and ∂²sdf_world/∂x_world² = (1/map_resolution)∂²sdf_px/∂px².

        H = self.sdf.shape[0]

        px, py = world_to_pixel(
            x_world, y_world,
            resolution=self.map_resolution,
            origin_x=self.map_origin_x,
            origin_y=self.map_origin_y,
            img_height=H,
            continuous=True,        # <-- IMPORTANT: keep fractional pixels
        )

        dxx = bilinear(self.ddsdf_xx, px, py)
        dxy = bilinear(self.ddsdf_xy, px, py)
        dyx = bilinear(self.ddsdf_yx, px, py)
        dyy = bilinear(self.ddsdf_yy, px, py)
        if normalized:
            dxx = bilinear(self.ddsdf_xx_normalized, px, py)
            dxy = bilinear(self.ddsdf_xy_normalized, px, py)
            dyx = bilinear(self.ddsdf_yx_normalized, px, py)
            dyy = bilinear(self.ddsdf_yy_normalized, px, py)


        return dxx/self.map_resolution, dxy/self.map_resolution, dyx/self.map_resolution, dyy/self.map_resolution

        

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
        if not self.recieved_map:
            self.time_map = Time.from_msg(msg.header.stamp)
            map_img = self.bridge.imgmsg_to_cv2(msg)
            # Binarize the map image
            _, map_img = cv2.threshold(map_img, 127, 255, cv2.THRESH_BINARY)
            map_img = np.asarray(map_img)
            
            map_img = cv2.bitwise_not(map_img)  # Invert colors: obstacles=255, free=0

            k = 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*k + 1, 2*k + 1))
            map_img = cv2.erode(map_img, kernel, iterations=1)

            self.map_erode = map_img
            

            self.map_height, self.map_width = map_img.shape
            self.recieved_map = True

            # (Optionally) store a copy of the global map
            self.global_map = map_img.copy()


            # Create the signed distance function (phi_s)
            map_not = 255 - map_img
            map_img = np.uint8(map_img)
            map_not = np.uint8(map_not)
            phi_safe = cv2.distanceTransform(map_img, distanceType=cv2.DIST_L2, maskSize=3, dstType=cv2.CV_32F)
            phi_safe = phi_safe - 1.0
            phi_s_safe = phi_safe
            
            phi_unsafe = cv2.distanceTransform(map_not, distanceType=cv2.DIST_L2, maskSize=3, dstType=cv2.CV_32F)
            phi_s_unsafe = -phi_unsafe

            phi_s = phi_s_unsafe + phi_s_safe
            self.sdf = phi_s.astype(np.float32)

            # Compute the gradients
            edges_y, edges_x = np.gradient(self.sdf)
            self.dsdf_x = edges_x
            self.dsdf_y = -edges_y  # adjust for Cartesian coordinates

            # Second derivatives in pixel space
            gyy, gyx = np.gradient(edges_y)  
            gxy, gxx = np.gradient(edges_x) 


            res = self.map_resolution
            self.ddsdf_xx = gxx
            self.ddsdf_yy = gyy 
            self.ddsdf_xy = -gxy 
            self.ddsdf_yx = -gyx

           # --------------------------------------------------

            # Normalize the gradient vectors
            norm_grad = np.sqrt(self.dsdf_x**2 + self.dsdf_y**2)
            norm_grad[norm_grad == 0] = 1.0  # avoid division by zero
            self.dsdf_x_normalized = self.dsdf_x / norm_grad
            self.dsdf_y_normalized = self.dsdf_y / norm_grad
            self.grad_sdf = np.array([self.dsdf_x, self.dsdf_y])
            self.grad_sdf_normalized = np.array([self.dsdf_x_normalized, self.dsdf_y_normalized])
            # Second derivatives will be computed at the robot's location later.

            gyy_norm, gyx_norm = np.gradient(-self.dsdf_y_normalized)
            gxy_norm, gxx_norm = np.gradient(self.dsdf_x_normalized)

            self.ddsdf_xx_normalized = gxx_norm 
            self.ddsdf_yy_normalized = gyy_norm 
            self.ddsdf_xy_normalized = -gxy_norm 
            self.ddsdf_yx_normalized = -gyx_norm 



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

        if self.sdf is not None:
            #t_1 = time()
            self.controller()
            #t_2 = time()
            #print(f"---------------------------------Controller computation time: {t_2 - t_1:.6f} seconds")
            self.publish_cbf()
            #self.publish_plot_twist()

    def publish_cbf(self):
        """
        Publish the CBF array (for debugging) and log the data.
        """
        try:
            msg = Float64MultiArray()
            now = self.get_clock().now().nanoseconds
            elapsed_time = (now - self.start_time) / 1e9
            with open(self.file_path, mode="a", newline="") as file:
                writer = csv.writer(file)
                writer.writerow([elapsed_time, self.cbf_array[0], self.cbf_array[1], self.cbf_array[2]])
            data = self.cbf_array + [0.0]
            msg.data = data
            self.publisher_cbf_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error in publish_cbf: {e}")

    def controller(self):
        """
        The CBF-CLF-QP controller implementation.
        (The equations remain mostly unchanged; make sure that all indexing now uses the dynamic map dimensions.)
        """
        global vel_prev, dPsi_prev
        # Hyperparameters and reference values
        C_alpha = 0.9#5#0.005#self.get_parameter('C_alpha').value #0.05#0.01 * 0.5
        P_alpha = 1.0
        Kv = 1.0
        Kw = 0.01
        Kd = 1.0
        C_gamma = 1.0
        P_gamma = 1.0
        Vmax =0.5 #self.get_parameter('Vmax').value #1.0
        Vmin = -0.5#self.get_parameter('Vmin').value #-1.0
        Wmax = 0.25 * np.pi#self.get_parameter('Wmax').value #4 * np.pi
        Wmin = -0.25* np.pi#self.get_parameter('Wmin').value #-4 * np.pi

        
        
        Delta_ub = 1.0#0.5#self.get_parameter('Delta_ub').value #0.5
        Delta_lb = -1.0#-0.5#self.get_parameter('Delta_lb').value #-0.5
        #heading = normalize_angle(np.pi - np.pi/6)
        heading = normalize_angle(np.pi)

        xw = self.x_real
        yw = self.y_real
        yaw = self.yaw

        # Continuous SDF and gradient in WORLD coordinates
        sdf = self.sdf_at_world(xw, yw)
        dsdf_x_true, dsdf_y_true = self.sdf_grad_at_world(xw, yw)
        ddsdf_xx, ddsdf_xy, ddsdf_yx, ddsdf_yy = self.hessian_at_world(xw, yw)

       


        yaw = self.yaw
        l_a = 0.25#0.025#0.1 (lower, omega has to turn more and be more agressive)
        l_b = 0.0#2.25
        l_s = -np.sqrt(l_a**2 + l_b**2) #-l_a -(l_a*beta)# * (2*np.pi*beta + 1)
        
        eta = 0.0
        
        if math.isnan(eta):
            self.get_logger().warn("eta is NaN!")
            eta = 0.0

        
        g = np.array([dsdf_x_true, dsdf_y_true])   # raw, world
        x = np.array([np.cos(yaw), np.sin(yaw)])
        x = np.around(x, decimals=3)
        x_perp = np.array([-np.sin(yaw), np.cos(yaw)])
        x_perp = np.around(x_perp, decimals=3)

        print(
            f"x vector: {np.array2string(x, precision=16, separator=', ')} "
            f"and sdf g: {np.array2string(g, precision=16, separator=', ')}"
        )
        print(f"g @ x: {float(g @ x):.16f}")

        # --- with perpendicular term---

        cbf = sdf + l_s + l_a*(g @ x)# + l_b*(g @ x_perp)

        # Hessian columns (world)
        g_x = np.array([ddsdf_xx, ddsdf_yx])   # ∂g/∂x
        g_y = np.array([ddsdf_xy, ddsdf_yy])   # ∂g/∂y

        dcbf_x = dsdf_x_true + l_a*(g_x @ x)# + l_b*(g_x @ x_perp)
        dcbf_y = dsdf_y_true + l_a*(g_y @ x)# + l_b*(g_y @ x_perp)
        dcbf_yaw = l_a*(g @ x_perp)# - l_b*(g @ x)
        # ----------------------------

        # --- symmetric (no left/right bias), smooth -------------------
        # eps_abs = 1e-4  # >0 keeps differentiable at z=0

        # z = float(g @ x_perp)                    # lateral alignment (signed)
        # absz = math.sqrt(z*z + eps_abs)          # smooth |z|
        # chain = z / absz                         # d(absz)/dz in (-1,1), well-defined

        # cbf = sdf + l_s + l_a*float(g @ x) + l_b*absz

        # # Hessian columns (world): g_x = ∂g/∂x, g_y = ∂g/∂y
        # g_x = np.array([ddsdf_xx, ddsdf_yx])
        # g_y = np.array([ddsdf_xy, ddsdf_yy])

        # # z_x = ∂(g·x_perp)/∂x = (∂g/∂x)·x_perp   (x_perp independent of x,y)
        # z_x = float(g_x @ x_perp)
        # z_y = float(g_y @ x_perp)

        # # yaw: ∂(g·x)/∂yaw = g·x_perp,  ∂(g·x_perp)/∂yaw = -g·x
        # dcbf_x   = dsdf_x_true + l_a*float(g_x @ x) + l_b*chain*z_x
        # dcbf_y   = dsdf_y_true + l_a*float(g_y @ x) + l_b*chain*z_y
        # dcbf_yaw = l_a*float(g @ x_perp) - l_b*chain*float(g @ x)

        # print("---------------------------------------------------------------")
        # print(f"dcbf_yaw: {dcbf_yaw:.2f}")
        # print(f"(g @ x_perp): {float(g @ x_perp):.2f}")
        # print(f"-chain*float(g @ x): {-chain*float(g @ x):.2f}")
        # ---------------------------------------------------------------


        #delta_time = 1/controller_frequency
        delta_time = self.dt
        true_cbf_dot = (cbf - self.cbf_prev)/ delta_time



        #print(f"eta: {np.rad2deg(eta)}")

        print(f"dcbf_x: {dcbf_x:.2f}, dcbf_y: {dcbf_y:.2f}, dcbf_yaw: {dcbf_yaw:.16f}")
       

        Vref = Vmax
        K_Wref = 0.5
        Wref = K_Wref * normalize_difference(heading - yaw)
        P_mat = np.diag([Kv, Kw, Kd])
        q = np.array([-Kv * Vref, -Kw * Wref, 0.0])

        # Set up inequality constraints for the QP
        G = np.array([
            [1.0, 0, 0],
            [-1.0, 0, 0],
            [0, 1.0, 0],
            [0, -1.0, 0],
            [0, 0, 1.0],
            [0, 0, -1.0],
            [0, 0, 0],
            [0, 0, 0]
        ])
        h_vec = np.array([Vmax, -Vmin, Wmax, -Wmin, Delta_ub, -Delta_lb, 0, 0])

        G[6][0] = -((dcbf_x * np.cos(yaw)) + (dcbf_y * np.sin(yaw)))
        G[6][1] = -dcbf_yaw
        G[6][2] = 0
        h_vec[6] = C_alpha * cbf

        G[7][0] = 0
        G[7][1] = normalize_difference(yaw - heading)
        G[7][2] = -1
        V_val = 0.5 * (normalize_difference(yaw - heading))**2
        h_vec[7] = -C_gamma * V_val

        try:
            [vel, dPsi, Delta] = solve_qp(P_mat, q, G, h_vec, solver='quadprog')
            vel = vel#/0.05
            dPsi = dPsi

            vel_prev = vel
            dPsi_prev = dPsi

            cbf_dot_alpha_cbf = (dcbf_x * np.cos(yaw) + dcbf_y * np.sin(yaw)) * vel + dcbf_yaw * dPsi + C_alpha * cbf
            cbf_dot = (dcbf_x * np.cos(yaw) + dcbf_y * np.sin(yaw)) * vel + dcbf_yaw * dPsi
            self.cbf_array = [float(cbf), float(cbf_dot), float(cbf_dot_alpha_cbf)]
            self.linear_velocity = float(vel)
            self.angular_velocity = float(dPsi)

            self.cbf_prev = cbf

            
            print(f"A (Av) is: {((dcbf_x * np.cos(yaw)) + (dcbf_y * np.sin(yaw))):.2f}")
            print(f"B (Bw) is: {dcbf_yaw:.16f}")
            print(f"QP Solution: vel={vel:.2f}, omega={dPsi:.16f}")
            print(f"Av is{((dcbf_x * np.cos(yaw)) + (dcbf_y * np.sin(yaw)))* vel:.2f}")
            print(f"Bw is {dcbf_yaw * dPsi:.16f}")
            print(f"Av + Bw is {((dcbf_x * np.cos(yaw)) + (dcbf_y * np.sin(yaw)))* vel + dcbf_yaw * dPsi:.2f}")
            print(f"cbf: {cbf:.2f}, cbf_dot: {cbf_dot:.2f}, cbf_dot + alpha*cbf: {cbf_dot_alpha_cbf:.2f}")
            print(f"cbf_dot (true): {true_cbf_dot:.2f}, difference: {cbf_dot - true_cbf_dot:.2f}")
            print(f"sdf: {sdf:.2f}, eta (deg): {np.rad2deg(eta):.2f}, cbf-sdf: {cbf - sdf:.2f}")
            print("--------------------------------------------------")
            
            
        except Exception as e:
            self.get_logger().error("Optimization failed, using previous solution.")
            vel, dPsi = 0.0, 0.0
            Delta = 0

            vel_prev = vel
            dPsi_prev = dPsi

            cbf_dot_alpha_cbf = (dcbf_x * np.cos(yaw) + dcbf_y * np.sin(yaw)) * vel + dcbf_yaw * dPsi + C_alpha * cbf
            cbf_dot = (dcbf_x * np.cos(yaw) + dcbf_y * np.sin(yaw)) * vel + dcbf_yaw * dPsi
            self.cbf_array = [float(cbf), float(cbf_dot), float(cbf_dot_alpha_cbf)]
            self.linear_velocity = float(vel)
            self.angular_velocity = float(dPsi)

            self.cbf_prev = cbf

            
            print(f"A (Av) is: {((dcbf_x * np.cos(yaw)) + (dcbf_y * np.sin(yaw)))}")
            print(f"B (Bw) is: {dcbf_yaw}")
            print(f"QP Solution: vel={vel}, omega={dPsi}")
            print(f"Av is{((dcbf_x * np.cos(yaw)) + (dcbf_y * np.sin(yaw)))* vel}")
            print(f"Bw is {dcbf_yaw * dPsi}")
            print(f"Av + Bw is {((dcbf_x * np.cos(yaw)) + (dcbf_y * np.sin(yaw)))* vel + dcbf_yaw * dPsi}")
            print(f"cbf: {cbf}, cbf_dot: {cbf_dot}, cbf_dot + alpha*cbf: {cbf_dot_alpha_cbf}")
            print(f"cbf_dot (true): {true_cbf_dot}, difference: {cbf_dot - true_cbf_dot}")
            print(f"sdf: {sdf}, eta (deg): {np.rad2deg(eta)}, cbf-sdf: {cbf - sdf}")
            print("--------------------------------------------------")

            sys.exit(1)

        


def main(args=None):
    rclpy.init(args=args)
    node = MobileRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
