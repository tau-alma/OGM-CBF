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

def interpolate(sdf, x, y):
    """
    Interpolates the value from the SDF matrix at coordinates (x, y) using bilinear interpolation.

    Parameters:
    - sdf: 2D NumPy array representing the signed distance function.
    - x: X-coordinate (float).
    - y: Y-coordinate (float).

    Returns:
    - Interpolated value at (x, y).
    """
    height, width = sdf.shape

    # Clamp coordinates to be within the valid range
    if x < 0 or x > width - 1 or y < 0 or y > height - 1:
        raise ValueError("Coordinates are outside the bounds of the SDF matrix.")

    # Coordinates of the top-left corner
    x0 = int(np.floor(x))
    y0 = int(np.floor(y))
    x1 = min(x0 + 1, width - 1)
    y1 = min(y0 + 1, height - 1)

    # Distances between the coordinates and the top-left corner
    dx = x - x0
    dy = y - y0

    # Retrieve the values at the four surrounding pixels
    Q11 = sdf[y0, x0]
    Q21 = sdf[y0, x1]
    Q12 = sdf[y1, x0]
    Q22 = sdf[y1, x1]

    # Perform bilinear interpolation
    interpolated_value = (Q11 * (1 - dx) * (1 - dy) +
                          Q21 * dx * (1 - dy) +
                          Q12 * (1 - dx) * dy +
                          Q22 * dx * dy)

    return interpolated_value

def sdf_world(sdf_array, x_world, y_world,
              resolution=0.05, origin_x=0.0, origin_y=0.0):
    img_height = sdf_array.shape[0]
    px, py = world_to_pixel(
        x_world, y_world,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        img_height=img_height,
        continuous=True,          # <-- IMPORTANT
    )
    return interpolate(sdf_array, px, py)

def bilinear(arr, x, y):
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

    return float(
        Q11 * (1 - dx) * (1 - dy) +
        Q21 * dx       * (1 - dy) +
        Q12 * (1 - dx) * dy       +
        Q22 * dx       * dy
    )


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
        self.vehicle_info = self.create_subscription(Odometry, "odom", self.vehicle_odom_callback, 1)
        self.subscription_2 = self.create_subscription(Image, 'map_image', self.listener_callback_map, 1)
        self.publisher_image_ = self.create_publisher(Image, '/cbf_image', 1)
        self.contour_timer_ = self.create_timer(1.0, self.publish_image)
        self.bridge = CvBridge()
        self.publisher_cbf_ = self.create_publisher(Float64MultiArray, '/cbf_array', 1)
        self.publisher_plot_twist_ = self.create_publisher(TwistStamped, '/plot_vel', 1)
        self.twist_publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

        vel_pub_time = 1.0 / controller_frequency
        self.twist_timer = self.create_timer(vel_pub_time, self.publish_velocity)

        self.start_time = self.get_clock().now().nanoseconds

        # Initialize state variables
        self.x = self.y = self.yaw = 0.0
        self.lin_x = self.ang_w = 0.0
        self.linear_velocity = self.angular_velocity = 0.0
        self.x_init = self.y_init = self.yaw_init = 0.0
        self.counter = 0.0
        self.Uref = [0.0, 0.0]

        # Do not preallocate arrays with fixed dimensions;
        # they will be allocated when the map is received.
        self.sdf = None
        self.dsdf_x = None
        self.dsdf_y = None
        self.dsdf_x_normalized = None
        self.dsdf_y_normalized = None
        self.grad_sdf = None
        self.grad_sdf_normalized = None
        self.ddsdf_xx = self.ddsdf_yy = self.ddsdf_xy = self.ddsdf_yx = 0.0

        self.fig, self.ax = plt.subplots(1, 1)
        self.Vx = self.Vy = self.Vz = 0.0

        # Initialize image holder; will be set after processing
        self.h_img = None
        self.time_map = 0.0
        self.cbf_array = [0.0]
        self.linear_velocity = self.angular_velocity = 0.0

        self.prev_time = monotonic()
        self.time_integrator = monotonic()
        self.throttle = self.steer = self.carBrake = 0.0

        # Global map and dimensions (will be updated on map reception)
        self.global_map = None
        self.map = None
        self.map_height = 146#None
        self.map_width = 192#None
        self.recieved_map = False

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

        self.sdf_calculated = False
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

    def sdf_at_world(self, x_world, y_world):
        img_height = self.sdf.shape[0]
        px, py = world_to_pixel(
            x_world, y_world,
            resolution=self.map_resolution,
            origin_x=self.map_origin_x,
            origin_y=self.map_origin_y,
            img_height=img_height,
            continuous=True,
        )
        return bilinear(self.sdf, px, py)

    def sdf_grad_at_world(self, x_world, y_world):
        img_height = self.sdf.shape[0]
        px, py = world_to_pixel(
            x_world, y_world,
            resolution=self.map_resolution,
            origin_x=self.map_origin_x,
            origin_y=self.map_origin_y,
            img_height=img_height,
            continuous=True,
        )
        gx = bilinear(self.dsdf_x_world, px, py)
        gy = bilinear(self.dsdf_y_world, px, py)
        return gx, gy
    
    def hessian_at_world(self, x_world, y_world):
        """
        Returns continuous Hessian entries (dxx, dxy, dyx, dyy)
        at world coordinates (x_world, y_world).
        All in [1/m²].
        """
        H = self.sdf.shape[0]

        px, py = world_to_pixel(
            x_world, y_world,
            resolution=self.map_resolution,
            origin_x=self.map_origin_x,
            origin_y=self.map_origin_y,
            img_height=H,
            continuous=True,        # <-- IMPORTANT: keep fractional pixels
        )

        dxx = interpolate(self.ddsdf_xx, px, py)
        dxy = interpolate(self.ddsdf_xy, px, py)
        dyx = interpolate(self.ddsdf_yx, px, py)
        dyy = interpolate(self.ddsdf_yy, px, py)

        return dxx, dxy, dyx, dyy

        

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
            color='r'
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
            self.map_height, self.map_width = map_img.shape
            self.recieved_map = True

            # (Optionally) store a copy of the global map
            self.global_map = map_img.copy()

            sdf_a = 3#0.8#self.get_parameter('sdf_a').value

            # Create the signed distance function (phi_s)
            map_not = 255 - map_img
            map_img = np.uint8(map_img)
            map_not = np.uint8(map_not)
            phi_safe = cv2.distanceTransform(map_img, distanceType=cv2.DIST_L2, maskSize=3, dstType=cv2.CV_8UC1)
            phi_safe = phi_safe - 1.0
            phi_s_safe = sdf_a * np.tanh( 0.005*phi_safe )
            
            phi_unsafe = cv2.distanceTransform(map_not, distanceType=cv2.DIST_L2, maskSize=3, dstType=cv2.CV_8UC1)
            #phi_unsafe = np.where(phi_unsafe != 1.0, phi_unsafe, 0.0)
            phi_s_unsafe = -sdf_a * np.tanh( 0.005*phi_unsafe)
            phi_s = phi_s_unsafe + phi_s_safe
            self.sdf = phi_s.astype(np.float32)

            #self.sdf = np.where(self.sdf != -1.0, self.sdf, 0.0)

            # Compute the gradients
            edges_y, edges_x = np.gradient(self.sdf)
            self.dsdf_x = edges_x
            self.dsdf_y = -edges_y  # adjust for Cartesian coordinates

            # Second derivatives in pixel space
            gyy, gyx = np.gradient(edges_y)  # gyy = d²φ/dpy², gyx = d²φ/(dpx dpy)
            gxy, gxx = np.gradient(edges_x)  # gxy = d²φ/(dpy dpx), gxx = d²φ/dpx²


            res = self.map_resolution
            self.ddsdf_xx = gxx / (res ** 2)         # ∂²φ / ∂x²
            self.ddsdf_yy = gyy / (res ** 2)         # ∂²φ / ∂y² (with sign from y mapping)
            self.ddsdf_xy = -gxy / (res ** 2)        # ∂²φ / ∂x∂y
            self.ddsdf_yx = -gyx / (res ** 2)        # ∂²φ / ∂y∂x

            # ---- NEW: gradient in WORLD coordinates [1/m] ----
            res = self.map_resolution
            self.dsdf_x_world = self.dsdf_x / res          # ∂φ / ∂x_world
            self.dsdf_y_world = self.dsdf_y / res          # ∂φ / ∂y_world
            # --------------------------------------------------

            # Normalize the gradient vectors
            norm_grad = np.sqrt(self.dsdf_x**2 + self.dsdf_y**2)
            norm_grad[norm_grad == 0] = 1.0  # avoid division by zero
            self.dsdf_x_normalized = self.dsdf_x / norm_grad
            self.dsdf_y_normalized = self.dsdf_y / norm_grad
            self.grad_sdf = np.array([self.dsdf_x, self.dsdf_y])
            self.grad_sdf_normalized = np.array([self.dsdf_x_normalized, self.dsdf_y_normalized])
            # Second derivatives will be computed at the robot's location later.

    def vehicle_odom_callback(self, msg):
        """
        Receive the vehicle's odometry and update the state.
        """
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
            self.x_init, self.y_init = world_to_pixel(self.x_init_real, self.y_init_real, img_height=self.map_height)
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

        # Convert real-world pose to pixel coordinates using the map dimensions
        if self.sdf is not None:
            self.x, self.y = world_to_pixel(self.x_real, self.y_real, img_height=self.map_height)
            #t_1 = time()
            self.controller()
            #t_2 = time()

            #print(f"---------------------------------Controller computation time: {t_2 - t_1:.6f} seconds")
            self.publish_cbf()
            #self.publish_plot_twist()

    def pose_to_pixel(self, x_real, y_real, resolution=0.05, map_origin=[0.0, 0.0]):
        """
        Convert real-world (meters) coordinates to pixel indices.
        If the map dimensions are not available yet, default to 400.
        """
        img_height = self.map_height if self.map_height is not None else 400
        pixel_x = (x_real - map_origin[0]) / resolution
        pixel_y = img_height - ((y_real - map_origin[1]) / resolution)
        return pixel_x, pixel_y

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
        C_alpha = 0.1#5#0.005#self.get_parameter('C_alpha').value #0.05#0.01 * 0.5
        P_alpha = 1.0
        Kv = 1.0
        Kw = 0.01
        Kd = 1.0
        C_gamma = 1.0
        P_gamma = 1.0
        Vmax =1.0 #self.get_parameter('Vmax').value #1.0
        Vmin = -1.0#self.get_parameter('Vmin').value #-1.0
        Wmax = 4 * np.pi#self.get_parameter('Wmax').value #4 * np.pi
        Wmin = -4 * np.pi#self.get_parameter('Wmin').value #-4 * np.pi
        Delta_ub = 1.0#0.5#self.get_parameter('Delta_ub').value #0.5
        Delta_lb = -1.0#-0.5#self.get_parameter('Delta_lb').value #-0.5
        #heading = normalize_angle(np.pi - np.pi/6)
        heading = normalize_angle(np.pi)

        # # Use the dynamic map indices (make sure x and y are integers)
        # sdf = self.sdf[int(self.y), int(self.x)]
        # #sdf = interpolate(self.sdf, self.x, self.y)
        # dsdf_x_true = self.dsdf_x[int(self.y), int(self.x)]
        # dsdf_y_true = self.dsdf_y[int(self.y), int(self.x)]
        # dsdf_x_normalized = self.dsdf_x_normalized[int(self.y), int(self.x)]
        # dsdf_y_normalized = self.dsdf_y_normalized[int(self.y), int(self.x)]

        xw = self.x_real
        yw = self.y_real
        yaw = self.yaw

        # Continuous SDF and gradient in WORLD coordinates
        sdf = self.sdf_at_world(xw, yw)
        dsdf_x_true, dsdf_y_true = self.sdf_grad_at_world(xw, yw)
        ddsdf_xx, ddsdf_xy, ddsdf_yx, ddsdf_yy = self.hessian_at_world(xw, yw)

       

        # Normalized gradient (world)
        grad_norm = math.hypot(dsdf_x_true, dsdf_y_true)
        if grad_norm < 1e-6:
            dsdf_x_normalized = 0.0
            dsdf_y_normalized = 0.0
        else:
            dsdf_x_normalized = dsdf_x_true / grad_norm
            dsdf_y_normalized = dsdf_y_true / grad_norm



        yaw = self.yaw
        l_a = 0.025
        l_s = -l_a
        

        if math.isnan(dsdf_x_normalized) or math.isnan(dsdf_y_normalized):
            self.get_logger().error("NaN in normalized gradient!")
            dsdf_x_normalized = dsdf_y_normalized = 0.0
        if any(math.isnan(val) for val in [ddsdf_xx, ddsdf_xy, ddsdf_yx, ddsdf_yy]):
            self.get_logger().error("NaN in second derivative!")
            ddsdf_xx = ddsdf_xy =ddsdf_yx = ddsdf_yy = 0.0

        x_vector = np.array([np.cos(yaw), np.sin(yaw)])
        sdf_normalized_grad_vector = np.array([dsdf_x_normalized, dsdf_y_normalized])
        cosine_eta = np.dot(sdf_normalized_grad_vector, x_vector)
        sine_eta = np.cross(sdf_normalized_grad_vector, x_vector)
        eta = np.arctan2(sine_eta, cosine_eta)
        eta = normalize_angle(eta)
        
        if math.isnan(eta):
            self.get_logger().warn("eta is NaN!")
            eta = 0.0

        
        cbf = sdf + l_s + l_a * (np.cos(eta))

        #time_now = time()
        #delta_time = time_now - self.time_cbf_prev if self.time_cbf_prev != 0.0 else 1e-5
        #self.time_cbf_prev = time_now
        delta_time = 1/controller_frequency#1/100
        true_cbf_dot = (cbf - self.cbf_prev)/ delta_time

        try:
            dyaw_x = self.angular_velocity / self.linear_velocity * np.cos(yaw)
            dyaw_y = self.angular_velocity / self.linear_velocity * np.sin(yaw)
        except Exception as e:
            self.get_logger().warn("Division by zero in dyaw computation")
            dyaw_x = dyaw_y = 0.0

        dx_vector_x = np.array([-np.sin(yaw), np.cos(yaw)]) * dyaw_x
        dx_vector_y = np.array([-np.sin(yaw), np.cos(yaw)]) * dyaw_y

        # dcbf_x = dsdf_x_true + P_alpha * l_a * (
        #     np.dot(np.array([self.ddsdf_xx, self.ddsdf_yx]), x_vector) +
        #     np.dot(sdf_normalized_grad_vector, dx_vector_x)
        # ) * np.cos(eta) ** (P_alpha - 1)
        # dcbf_y = dsdf_y_true + P_alpha * l_a * (
        #     np.dot(np.array([self.ddsdf_xy, self.ddsdf_yy]), x_vector) +
        #     np.dot(sdf_normalized_grad_vector, dx_vector_y)
        # ) * np.cos(eta) ** (P_alpha - 1)
        #dcbf_yaw = P_alpha * l_a * (-np.sin(eta)) * np.cos(eta) ** (P_alpha - 1)
        dcbf_x = dsdf_x_true 
        + l_a * (
            np.dot(np.array([ddsdf_xx, ddsdf_yx]), x_vector) +
            np.dot(sdf_normalized_grad_vector, dx_vector_x)
        ) 
        dcbf_y = dsdf_y_true 
        
        + l_a * (
            np.dot(np.array([ddsdf_xy, ddsdf_yy]), x_vector) +
            np.dot(sdf_normalized_grad_vector, dx_vector_y) 
        )
        dcbf_yaw = l_a * (-np.sin(eta) )

        

        print(f"eta: {np.rad2deg(eta)}")

        print(f"dcbf_x: {dcbf_x}, dcbf_y: {dcbf_y}, dcbf_yaw: {dcbf_yaw}")
        # print(f"""dcbf_x_cos{l_a * (
        #     np.dot(np.array([self.ddsdf_xx, self.ddsdf_yx]), x_vector) +
        #     np.dot(sdf_normalized_grad_vector, dx_vector_x) +

        #     (np.dot(np.array([self.ddsdf_xx, self.ddsdf_yx]), x_vector) +
        #     np.dot(sdf_normalized_grad_vector, dx_vector_x))* (1/np.sqrt(1 - np.cos(eta)**2 + epsilon))

        # ) 
        # }""")

        # print(f"""dcbf_x_cos_arccos: {
        #       l_a* (np.dot(np.array([self.ddsdf_xx, self.ddsdf_yx]), x_vector) +
        #     np.dot(sdf_normalized_grad_vector, dx_vector_x))* (1/np.sqrt(1 - np.cos(eta)**2 + epsilon))
        # }""")


        # print(f"""dcbf_y_cos{
        #     l_a * (
        #     np.dot(np.array([self.ddsdf_xy, self.ddsdf_yy]), x_vector) +
        #     np.dot(sdf_normalized_grad_vector, dx_vector_y) +

        #     ( np.dot(np.array([self.ddsdf_xy, self.ddsdf_yy]), x_vector) +
        #     np.dot(sdf_normalized_grad_vector, dx_vector_y))* (1/np.sqrt(1 - np.cos(eta)**2 + epsilon))

        # )
        #     }""")
        
        # print(f"""dcbf_y_cos_arccos: {
        #       l_a*  ( np.dot(np.array([self.ddsdf_xy, self.ddsdf_yy]), x_vector) +
        #     np.dot(sdf_normalized_grad_vector, dx_vector_y))* (1/np.sqrt(1 - np.cos(eta)**2 + epsilon))
        # }""")

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
