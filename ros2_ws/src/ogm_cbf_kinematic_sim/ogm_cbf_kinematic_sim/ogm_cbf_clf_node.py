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
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped, Twist
from time import monotonic
import os

####################
# This code is the implementation of OGM-CBF in CARLA simulator (link to paper: https://arxiv.org/abs/2405.10703)
# The code assumes that Occupancy Grid Map (OGM) is published on "map" topic as an binary image.
# The map is ego centric, local OGM. Therefore robot's heading vector in map frame will always be [0, 1]
####################

matplotlib.use('Agg')
vel_prev = 0
dPsi_prev = 0

#TODO normalize_angle and normalize_difference and pose_to_pixel functions should go to utils.py

def normalize_angle(angle):
        """Normalize an angle to the range [0, 2*pi]."""
        angle = angle % (2.0 * np.pi) # Wrap to [0, 2*pi]
        if angle < 0.0:
            angle += 2.0 * np.pi
        
        return angle

def normalize_difference(angle):
        """Normalize an angle to the range [-pi, pi]."""
        angle = angle % (2.0 * np.pi) # Wrap to [0, 2*pi]
        if angle < 0.0:
            angle += 2.0 * np.pi
        if angle > np.pi:
            angle -= 2.0 * np.pi
        
        return angle

class MobileRobot(Node):
    def __init__(self, state=[0,0,0], timestep=0.1):

        super().__init__('minimal_publisher')
        #self.callback_group_async = ReentrantCallbackGroup()
        self.vehicle_info = self.create_subscription(Odometry, "odom", self.vehicle_odom_callback, 1)#, callback_group = self.callback_group_async)
        self.subscription_2 = self.create_subscription(Image,'map_image',self.listener_callback_map, 1)#, callback_group = self.callback_group_async)
        
        self.publisher_image_ = self.create_publisher(Image, '/cbf_image', 1)#, callback_group = self.callback_group_async)
        self.contour_timer_ = self.create_timer(1.0, self.publish_image)#, callback_group = self.callback_group_async)
        self.bridge = CvBridge()
        self.publisher_cbf_ = self.create_publisher(Float64MultiArray, '/cbf_array', 1) #, callback_group = self.callback_group_async)
        self.publisher_plot_twist_ = self.create_publisher(TwistStamped, '/plot_vel', 1)
        # Create a publisher for the cmd_vel topic
        self.twist_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # Timer to publish messages at a regular interval
        self.twist_timer = self.create_timer(1.0, self.publish_velocity)  # Publish every 1 second
        #self.global_map_subscription = self.create_subscription(Image,'/map/full',self.global_map_callback,1, callback_group = self.callback_group_async)
        

        # initializing class variables

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.lin_x = 0.0
        self.ang_w = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.x_init = 0.0
        self.y_init = 0.0
        self.yaw_init = 0.0
        self.counter = 0.0
        self.Uref = [0.0, 0.0]

        self.sdf = np.zeros((400, 400))
        self.dsdf_x = self.dsdf_x_normalized = np.zeros((400, 400))
        self.dsdf_y = self.dsdf_y_normalized = np.zeros((400, 400))
        self.grad_sdf_normalized = self.grad_sdf = np.array([self.dsdf_x, self.dsdf_y])
        self.ddsdf_xx = self.ddsdf_yy = self.ddsdf_xy = self.ddsdf_yx= 0.0

        self.fig, (self.ax) = plt.subplots(1,1)
        self.Vx = self.Vy = self.Vz = 0.0
        
        self.h_img = np.uint8(np.zeros((400, 400,3))) # the contructed h(x) for the whole map
        self.time_map = 0.0
        self.cbf_array = [0.0]
        self.linear_velocity = self.angular_velocity = 0.0

        # time integrator is for low level control of vehicle in CARLA 
        self.prev_time = monotonic()
        self.time_integrator = monotonic()
        self.throttle = self.steer = self.carBrake = 0.0
        
        self.global_map = np.zeros((400,400))
        self.counter = 0.0
        self.recieved_map = False
        self.map = None


        

    def publish_velocity(self):
        """Publish the velocity as a Twist message."""
        msg = Twist()
        msg.linear.x = 0.0#self.linear_velocity*np.cos(self.yaw)
        msg.linear.y = -0.1#self.linear_velocity*np.sin(self.yaw)
        msg.angular.z =0.0#self.angular_velocity
        self.twist_publisher_.publish(msg)
        #self.get_logger().info(f'Published cmd_vel: linear x={msg.linear.x}, linear y = {msg.linear.y}, angular={msg.angular.z}')

        

    def global_map_callback(self, msg):
        
        # This function recieves global map for visualizations
        
        
        
        # Convert ROS Image message to OpenCV image
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        img = np.asarray(img)
        
        # Saving global maps as images
        #image_filename = os.path.join("plot/global_map", f"global_map_carla_{self.counter}.jpg")
        #cv2.imwrite(image_filename, img)
        self.counter = self.counter + 1


      

    def publish_plot_twist(self):
        
        """
        publishing velocity commands (u_cmd) on ros
        """
        
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()#self.time_map.to_msg()

        msg.twist.linear.x = self.linear_velocity 
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = self.angular_velocity

        self.publisher_plot_twist_.publish(msg)


    def process_image(self):
        ###########
        # This function visualizes the phi_s (refer to paper) and the gradients of it as a image called h_img
        ###########
        final = self.sdf
        edges_x = self.dsdf_x
        edges_y = self.dsdf_y
        im_width, im_height = self.sdf.shape

        self.ax.clear()
        dstep = 20
        Y,X = np.mgrid[0:im_width:dstep, 0:im_height:dstep]
        self.ax.quiver(X,Y, edges_x[0:im_width:dstep, 0:im_height:dstep],edges_y[0:im_width:dstep, 0:im_height:dstep],  color='r')
        t = self.ax.imshow(final, cmap='viridis', vmin = np.min(final), vmax = np.max(final))

        # Draw a circle around the specific point
        circle = plt.Circle((self.x,self.y), 5, fill=False, color='blue')
        #print(f"------------------sdf is {self.sdf[(self.x), (self.y)]}-----------------")
        self.ax.add_patch(circle)
        
        cax = self.fig.colorbar(t, ax = self.ax)
        canvas = plt.gca().figure.canvas
        canvas.draw()
        data = np.frombuffer(canvas.tostring_rgb(), dtype=np.uint8)
        image = data.reshape(canvas.get_width_height()[::-1] + (3,))
        canvas.flush_events()
        plt.pause(0.1)
        cax.remove()
        self.h_img = image


    def publish_image(self):
        """
        publishing h_img on ros
        """
        try:
            self.process_image()
            frame = self.h_img
            img_msg = self.bridge.cv2_to_imgmsg(frame)#, '8UC3')
            img_msg.header.stamp = self.time_map.to_msg()
            self.publisher_image_.publish(img_msg)
        except:
            pass
              

    def listener_callback_map(self, msg: Image):
        """
        receiving map, and contructing phi_s
        """
        #print(f"-------------------recieved map flag is  {self.recieved_map}-----------------")
        if self.recieved_map == False:

            self.time_map = Time.from_msg(msg.header.stamp)

            map = self.bridge.imgmsg_to_cv2(msg)

            _,map = cv2.threshold(map, 127, 255, cv2.THRESH_BINARY) # binarzing the map
            map = np.asarray(map)

            
            """
            now we construct phi_s:
            first distance function of safe and unsafe sets are created seperately using openCV distanceTransform
            then they are added with the correct sign to create the final signed distance function (phi in the paper)
            """

            map_not = 255 - map
            map = np.uint8(map)
            map_not = np.uint8(map_not)

            phi_safe = cv2.distanceTransform(map, distanceType=cv2.DIST_L2, maskSize=3, dstType=cv2.CV_8UC1) # black is obstacle, closest distance to black is being calculated
            phi_s_safe = 3.0*np.tanh(0.01*phi_safe) # phi_s = a*tanh(b*phi) (adding non-linearity)
            
            phi_unsafe = cv2.distanceTransform(map_not, distanceType=cv2.DIST_L2, maskSize=3, dstType=cv2.CV_8UC1) # in map_not safe_set is black, closest distance to that is calculated
            phi_s_unsafe = 3.0*np.tanh(0.01*phi_unsafe) # phi_s = a*tanh(b*phi) (adding non-linearity) 

            phi_s_unsafe = -phi_s_unsafe # inside obstacles are negative
            phi_s = phi_s_unsafe + phi_s_safe # phi_s is constructed
        
            #--------now we calculate gradient of sdf--------------------------------

            edges_y, edges_x = np.gradient(phi_s) # getting gradient of phi_s numerically  
            # numpy coordinate by defualt is x down and y right 
            # my notations are in cartesian space and I will use cartesian coordinate for CBF. Therefore I need to flip the sign of y gradient.
            # edges_y: first axis is row in np.array terms, gradient with respect to rows is -y of cartesian space (increases downwards)
            # edges_x: second axis is column in np.array terms, gradient with respect to columns is x of cartesian space (increases rightwards)

            self.sdf = phi_s
            self.dsdf_x = edges_x           # No adjustment needed for x
            self.dsdf_y = -edges_y          # Flip the sign to align with Cartesian y-up

            # normalizing gradient vector of SDF. we use normalized gradient version in dot product
            self.grad_sdf = np.array([self.dsdf_x, self.dsdf_y])

            self.dsdf_x_normalized = self.dsdf_x / (np.sqrt(self.dsdf_x**2 + self.dsdf_y**2)) 
            self.dsdf_y_normalized = self.dsdf_y / (np.sqrt(self.dsdf_x**2 + self.dsdf_y**2))
            
            self.grad_sdf_normalized = np.array([self.dsdf_x_normalized, self.dsdf_y_normalized])

            # note that np.linalg.norm will give incorrect result since it normalizes the whole matrix
            # but we want each gradient vector to be normalized by its own norm (not the norm of the whole matrix)

            # getting the ddSDF as is used in h_dot
            edges_y, edges_x = np.gradient(self.dsdf_x_normalized)
            self.ddsdf_xx = edges_x[int(self.y), int(self.x)] # I only save ddsdf for where there robot is
            self.ddsdf_xy = -edges_y[int(self.y),int(self.x)]

            edges_y, edges_x = np.gradient(self.dsdf_y_normalized)
            self.ddsdf_yx = edges_x[int(self.y), int(self.x)]
            self.ddsdf_yy = -edges_y[int(self.y), int(self.x)]
            
            # now for gradients x and y are swapped and y got negative. (we moved from numpy convention(y right, x down) to pixel coordinate (x right, y down), and from there
            # to cartesian coordinate (x right, y up). 
            
        

        
        # coordinate of the gradients: this will give gradient in local frame which y is forward and x is right (same as in right hand coordinate in ros-carla)

            
    def vehicle_odom_callback(self, msg):
        """
        Receive information of ego vehicle from carla
        """
        global x_init, y_init, yaw_init

        if self.counter == 0:
            self.x_init_real = msg.pose.pose.position.x
            self.y_init_real = msg.pose.pose.position.y
            _, _, self.yaw_init = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
            x_init = self.x_init
            y_init = self.y_init
            yaw_init = self.yaw_init

            self.y_init, self.x_init = self.pose_to_pixel(self.x_init_real, self.y_init_real, img_height_pixel=400, img_width_pixel=400, resolution=0.05, map_origin=[0.0, 0.0])
        
        
        self.counter += 1
        self.x_real = msg.pose.pose.position.x
        self.y_real = msg.pose.pose.position.y
        _, _, self.yaw = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        
        self.Vx = msg.twist.twist.linear.x
        self.Vy = msg.twist.twist.linear.y
        self.Vz = msg.twist.twist.linear.z

        self.x, self.y = self.pose_to_pixel(self.x_real, self.y_real, img_height_pixel=400, img_width_pixel=400, resolution=0.05, map_origin=[0.0, 0.0])
        #print(f"pose is {self.x}, {self.y}")

        ### CBF-CLF-QP is called in this callback each time we recieve new state from odom callback     
        self.controller()
        self.publish_cbf() # publish CBF values for debugging
        self.publish_plot_twist() # publish u_cmd for debugging
        #self.publish_cmd()
       

    def pose_to_pixel(self, 
                        x_real,                   # x coordinate of pose [m]
                        y_real,                   # y coodinate of pose [m]
                        img_height_pixel=400,    # image width in pixels
                        img_width_pixel=400,     # image height in pixels
                        resolution=0.05,          # image resolution [m/pixel]
                        map_origin=[0.0,  # map bottom left x coordinate [m]
                                    0.0], # map bottom left y coordinate [m]
                        ):
            

            # convert to integers with just cutting off the decimals 
            return (x_real - map_origin[0]) / resolution, img_height_pixel - ((y_real - map_origin[1]) / resolution)

        
    def publish_cbf(self):
        """
        publishing cbf_array for debugging purposes
        """
        try:
            msg = Float64MultiArray()
            timestamp = self.time_map.to_msg()
            
            #data = [float(timestamp.sec)] + [float(timestamp.nanosec)] + self.cbf_array + [float(0.0)] 
            data = self.cbf_array + [float(0.0)] + [float(self.dsdf_x_normalized[int(self.y), int(self.x)])] + [float(self.dsdf_y_normalized[int(self.y), int(self.x)])]
            #print(f"------cbf array is {self.cbf_array}-----------")
            msg.data = data
            self.publisher_cbf_.publish(msg)   
        except:
            pass

    def controller(self):
        
        global vel_prev, dPsi_prev

        ##------------------------- hyperparameters of the controller---------------------------------##
        C_alpha = 0.6            # more conservative is value of it is less. This is alpha(.) in the paper
        P_alpha = 1.0           # power of cosine in cbf. do not touch this, it is always 1 for now
        l_a = 0.25         
        l_s = -1*l_a                  
        
        Kv = 1.0
        Kw = 1.0
        Kd = 1.0
        C_gamma = 1.0
        P_gamma = 1.0
        
        # Upper bound and lower bound of control inputs
        Vmax = 1.0
        Vmin = -1.0
        Wmax = 4*np.pi
        Wmin = -4*np.pi

        # Upper bound and lower bound of relaxation parameter
        Delta_ub = +0.5
        Delta_lb = -0.5

        # target heading
        heading = -np.pi/2
        heading = normalize_angle(heading)
        ##--------------------------------------------------------------------------------------------##
        """
        system is assumed to be a 2d differential drive:
        X = [x, y, yaw]
        U = [v, omega]
        x_dot = v.cos(yaw)
        y_dot = v.sin(yaw)
        yaw_dot = omega
        """
        ##--------------------simplifying equations in 2D------------------------------
        """        
        In 2d version we can write h(x)=phi_s + l_s + la*cosine(etha)
        where cosine(etha) = the dot product of gradient of phi_s and x_hat(heading vector) (both are unit vectors)
        on the other hand etha can be seen as (etha = yaw - some angle).
        This means cosine eta has derivative with respect to yaw.
        As a result omega (yaw_dot) will appear in h_dot
        """
        ##--------------------------------------------------------------------------------------------##
        sdf = self.sdf[int(self.y), int(self.x)]
        dsdf_x = self.dsdf_x[int(self.y), int(self.x)]
        dsdf_y = self.dsdf_y[int(self.y), int(self.x)]
        dsdf_x_normalized = self.dsdf_x_normalized[int(self.y), int(self.x)]
        dsdf_y_normalized = self.dsdf_y_normalized[int(self.y), int(self.x)]
        yaw = self.yaw # this should be used for CLF calculation which is indeed in cartesian coordinate

        # handle when derivative of sdf is nan
        if math.isnan(dsdf_x_normalized) or math.isnan(dsdf_y_normalized):
            print("nan in gradient!")
            dsdf_x_normalized = 0.0
            dsdf_y_normalized = 0.0
        if math.isnan(self.ddsdf_xx) or math.isnan(self.ddsdf_xy) or math.isnan(self.ddsdf_yx) or math.isnan(self.ddsdf_yy):
            print("nan in gradient of gradient!!")
            self.ddsdf_xx = 0.0
            self.ddsdf_xy = 0.0
            self.ddsdf_yx = 0.0
            self.ddsdf_yy = 0.0
        
        x_vector = np.array([np.cos(yaw), np.sin(yaw)]) # heading vector of robot in pixel coordinate
        
        sdf_normalized_grad_vector = np.array([dsdf_x_normalized, dsdf_y_normalized]) #gradient of phi_s in the robots pose
        cosine_eta = np.dot(sdf_normalized_grad_vector, x_vector) # as both vectors are normalized cosine_eta is equal to dot_product
        sine_eta = np.cross(sdf_normalized_grad_vector, x_vector)
        eta = np.arctan2(sine_eta, cosine_eta)
        #eta = np.arccos(cosine_eta)* sine_eta / np.abs(sine_eta)

        eta = normalize_angle(eta)

        print(f"------------------cbf yaw is {np.rad2deg(yaw)}-----------------")

        print(f"------------------eta is {np.rad2deg(eta)}-----------------")
        if math.isnan(eta):
            print("eta for nan!")
            eta = 0.0
        
        
        cbf = sdf + l_s  + (np.cos(eta))*l_a

        # The derivative of the dot product of two vector-valued functions a(t) and b(t) is:
        # d/dt [a(t) · b(t)] = a'(t) · b(t) + a(t) · b'(t)

        # derivative of heading vector with respect to x
        #dyaw_x = dyaw_t * dt_x which dyaw/dt is omega and dx/dt is v*cos(yaw)
        dyaw_x = self.angular_velocity/ self.linear_velocity*np.cos(yaw)
        dyaw_y = self.angular_velocity/ self.linear_velocity*np.sin(yaw) 
        #TODO what if linear velocity is zero???

        # d(cos(yaw))/dx = -sin(yaw) * dyaw_x and d(sin(yaw))/dx = cos(yaw) * dyaw_x
        dx_vector_x = np.array([-np.sin(yaw), np.cos(yaw)])* dyaw_x
        dx_vector_y = np.array([-np.sin(yaw), np.cos(yaw)])* dyaw_y


        dcbf_x = dsdf_x + l_a * ( np.dot(np.array([self.ddsdf_xx, self.ddsdf_yx]), x_vector) + np.dot(sdf_normalized_grad_vector, dx_vector_x))
        dcbf_y = dsdf_y + l_a * ( np.dot(np.array([self.ddsdf_xy, self.ddsdf_yy]), x_vector) + np.dot(sdf_normalized_grad_vector, dx_vector_y))


        dcbf_yaw = l_a* (-np.sin(eta))
        
        
        # debugging prints
        #print([np.round(cbf, 3), np.round(dsdf_x,3), np.round(dsdf_y,3), np.round(math.cos(eta),3)])
        ##--------------------------------------------------------------------------------------------##
        
        ##------------------------------------- Constructing the Optimization ------------------------------##
        """
        optimization variables u = [v, omega, delta]^T 
        minimize cost function: J = 1/2*u^T*P*u + q^T*u  s.t:  G*u <= h
        note that this h is an array for inequality constraints, don't mix it up with CBF function h(x)

        we write J = 1/2 (v-v_ref)^2 + 1/2 (omega-omega_ref)^2 + 1/2 delta^2
        """

        ## reference of control inputs
        Vref = Vmax     # linear velocity reference
        K_Wref = 0.5    # gain for calculating the angular velocity reference using a proportional controller
        Wref = K_Wref*  normalize_difference(heading - yaw)      # calculating the angular velocity reference using a proportional controller

        P = np.diag([Kv, Kw, Kd])
        q = np.array([-Kv*Vref, -Kw*Wref, 0.0])

        ## six first rows are for setting upper bounds and lower bounds
        G = np.array([[1.0, 0, 0],[-1.0, 0, 0],[0, 1.0, 0],[0, -1.0, 0],[0, 0, 1.0],[0, 0, -1.0],\
                    [0, 0, 0],\
                    [0, 0, 0]])

        h = np.array([Vmax,-Vmin,Wmax,-Wmin,Delta_ub,-Delta_lb, \
                        0,\
                        0])

        ## Lyapunov function
        # V(x) = 0.5*(x-xd)^2 + 0.5*(y-yd)^2 + 0.5*(psi-psid)^2
        V = 0.5*(0)**2 + 0.5*(0)**2 + 0.5*normalize_difference(yaw - heading)**2     

        ## making 7th row of G for the CBF constraint
        ## derivative of CBF: dh(x)/dt = (dh/dX).(dX/dt)
        ## h_dot should be wrriten as affine function in control.
        ## G[6][0] is affine part of h_dot with respect to v
        ## G[6][1] is affine part of h_dot with respect to omega


        #G[6][0] = -(((dsdf_x + l_a * np.dot(x_vector, np.array([self.ddsdf_xx, self.ddsdf_xy]))) * math.cos(yaw)) + ((dsdf_y + l_a * np.dot(x_vector, np.array([self.ddsdf_yx, self.ddsdf_yy]))) * math.sin(yaw)))
        #G[6][0] = -(((dsdf_x + l_a * np.dot(x_vector, np.array([self.ddsdf_xx, self.ddsdf_xy]))) * math.cos(yaw)) + ((dsdf_y + l_a * np.dot(x_vector, np.array([self.ddsdf_yx, self.ddsdf_yy]))) * math.sin(yaw)))
        G[6][0] = -1*( (dcbf_x * math.cos(yaw)) + (dcbf_y * math.sin(yaw)) )
        
        G[6][1] = -1*dcbf_yaw
        G[6][2] = 0     # No relaxation
        h[6] = C_alpha * cbf 

        #print(f"-------------------{G[6][0]}------------")
            
        
        ## making matrices of G and h to satisfy the CLF condition

        G[7][0] = 0
        G[7][1] = normalize_difference(yaw - heading)
        G[7][2] = -1     # relaxation
        h[7] = -C_gamma*V**P_gamma


        
        # Optimization Solver
        try:
            [vel, dPsi, Delta] = solve_qp(P, q, G, h, solver='quadprog')
            #print(f"dpsi is {np.round(dPsi, 3)} and vel is {np.round(vel,3)} and delta is {np.round(Delta,3)}")

        ## Error handling in case of infeasibility
        except Exception as e:
            #print("exception on optimization occured")
            #print(e)
            vel, dPsi = vel_prev, dPsi_prev    # in case of infeasibility, return the previous solution
            Delta = 0

        ## save current solution as the previous values
        vel_prev = vel
        dPsi_prev = dPsi

        # saving  h and hdot+alpha(h) for plotting
        #cbf_dot_alpha_cbf = (((dsdf_x + l_a* np.dot(x_vector, np.array([self.ddsdf_xx, self.ddsdf_xy]))) * math.cos(yaw)) + ((dsdf_y + l_a* np.dot(x_vector, np.array([self.ddsdf_yx, self.ddsdf_yy]))) * math.sin(yaw)))*vel + (P_alpha*(-math.sin(eta))*l_a*math.cos(eta)**(P_alpha-1))*dPsi + (C_alpha * cbf)
        cbf_dot_alpha_cbf = (dcbf_x * np.cos(yaw) + dcbf_y * np.sin(yaw))*vel + dcbf_yaw*dPsi + C_alpha * cbf
        self.cbf_array = [float(cbf), float(cbf_dot_alpha_cbf)]
        self.linear_velocity = float(vel)
        self.angular_velocity = float(dPsi)
       



def main(args=None):

    rclpy.init(args=args)
    #executor = MultiThreadedExecutor()
    
    robot = MobileRobot()
    #rclpy.spin(robot)
    
    #try:
        #executor.add_node(robot)
    rclpy.spin(robot)

   
    robot.destroy_node()
    rclpy.shutdown()
    # robot.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()

