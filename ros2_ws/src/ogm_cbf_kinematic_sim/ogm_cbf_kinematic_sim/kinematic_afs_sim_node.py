import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import time
import math

from ogm_cbf_kinematic_sim.conf import simulator_frequency


class KinematicAFSSimNode(Node):
    """
    Articulated Frame Steering (AFS) kinematic simulator.
    State (front body): x_f, y_f, theta_f, beta
    Inputs: u_v (forward speed), u_beta_dot (articulation rate)
    """

    def __init__(self):
        super().__init__('kinematic_afs_sim_node')

        # --- timing ---
        self.declare_parameter('update_rate', simulator_frequency)
        self.update_rate = float(self.get_parameter('update_rate').value)

        # --- AFS geometry (meters) ---
        self.declare_parameter('l_f', 1.0)  # joint -> front ref point
        self.declare_parameter('l_r', 1.0)  # joint -> rear ref point
        self.l_f = float(self.get_parameter('l_f').value)
        self.l_r = float(self.get_parameter('l_r').value)

        # --- articulation limits ---
        self.declare_parameter('beta_max', float(np.deg2rad(40.0)))  # clamp
        self.beta_max = float(self.get_parameter('beta_max').value)

        # --- initial pose ---
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', float(np.pi))
        self.declare_parameter('initial_beta', 0.0)

        self.state = {
            'x': float(self.get_parameter('initial_x').value),     # x_f
            'y': float(self.get_parameter('initial_y').value),     # y_f
            'yaw': float(self.get_parameter('initial_yaw').value), # theta_f
            'beta': float(self.get_parameter('initial_beta').value),
        }

        # wait for initial pose (x,y,yaw)
        self.declare_parameter('wait_for_initial_pose', True)
        self.wait_for_initial_pose = bool(self.get_parameter('wait_for_initial_pose').value)
        self.initial_pose_received = False
        self.create_subscription(Pose, 'initial_pose', self.initial_pose_callback, 1)

        # --- cmd timeout ---
        self.cmd_timeout = 0.5
        self.last_cmd_time = time.time()

        # --- inputs (body-level) ---
        self.u_v = 0.0
        self.u_beta_dot = 0.0

        # topics (keep your naming style)
        self.create_subscription(Twist, 'cmd_vel_2', self.cmd_vel_callback, 1)
        self.odom_pub = self.create_publisher(Odometry, 'odom_2', 1)

        # optional: publish articulation angle for debugging
        self.beta_pub = self.create_publisher(Float32, 'beta', 1)

        self.last_time = time.time()
        self.create_timer(1.0 / self.update_rate, self.update_simulation)

    def initial_pose_callback(self, msg: Pose):
        if self.initial_pose_received:
            return

        self.state['x'] = float(msg.position.x)
        self.state['y'] = float(msg.position.y)

        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        _, _, yaw = euler_from_quaternion(q)
        self.state['yaw'] = float(yaw)

        self.initial_pose_received = True
        self.get_logger().info(
            f"Initial pose set: x={self.state['x']:.2f}, y={self.state['y']:.2f}, yaw={self.state['yaw']:.2f} rad"
        )

    def cmd_vel_callback(self, msg: Twist):
        # Map Twist to AFS inputs
        self.u_v = float(msg.linear.x)         # forward speed along front body axis
        self.u_beta_dot = float(msg.angular.z) # articulation rate

        self.last_cmd_time = time.time()

    def update_simulation(self):
        t = time.time()
        dt = t - self.last_time
        self.last_time = t

        if self.wait_for_initial_pose and not self.initial_pose_received:
            self.publish()
            return

        if t - self.last_cmd_time > self.cmd_timeout:
            self.u_v = 0.0
            self.u_beta_dot = 0.0

        # clamp beta
        self.state['beta'] = float(np.clip(self.state['beta'], -self.beta_max, self.beta_max))

        # --- AFS kinematics (front body reference point) ---
        x = self.state['x']
        y = self.state['y']
        theta = self.state['yaw']
        beta = self.state['beta']

        denom = (self.l_f * math.cos(beta) + self.l_r)
        # avoid blow-up if someone sets crazy geometry; denom should be >0 in practice
        denom = max(1e-6, denom)

        x_dot = self.u_v * math.cos(theta)
        y_dot = self.u_v * math.sin(theta)
        theta_dot = (self.u_v * math.sin(beta) + self.l_r * self.u_beta_dot) / denom
        beta_dot = self.u_beta_dot

        self.state['x'] = x + x_dot * dt
        self.state['y'] = y + y_dot * dt
        self.state['yaw'] = self.normalize_angle(theta + theta_dot * dt)
        self.state['beta'] = float(np.clip(beta + beta_dot * dt, -self.beta_max, self.beta_max))

        self.publish(x_dot, y_dot, theta_dot, beta_dot)

    def publish(self, x_dot=0.0, y_dot=0.0, theta_dot=0.0, beta_dot=0.0):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"

        odom.pose.pose.position.x = float(self.state['x'])
        odom.pose.pose.position.y = float(self.state['y'])
        odom.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, float(self.state['yaw']))
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # store world-frame velocity + yaw rate of front body
        odom.twist.twist.linear.x = float(x_dot)
        odom.twist.twist.linear.y = float(y_dot)
        odom.twist.twist.angular.z = float(theta_dot)

        self.odom_pub.publish(odom)

        b = Float32()
        b.data = float(self.state['beta'])
        self.beta_pub.publish(b)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        angle = angle % (2.0 * np.pi)
        if angle < 0.0:
            angle += 2.0 * np.pi
        return float(angle)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicAFSSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
