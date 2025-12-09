import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Quaternion
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
import math
import time
import numpy as np
from tf_transformations import euler_from_quaternion
from ogm_cbf_kinematic_sim.conf import controller_frequency, simulator_frequency



class KinematicDiffDriveSimNode(Node):
    def __init__(self):
        super().__init__('kinematic_diff_drive_sim_node')
        
        self.declare_parameter('update_rate', simulator_frequency)  # Hz
        self.update_rate = self.get_parameter('update_rate').value

        # Robot state: x, y, and yaw (orientation)
        #self.state = {'x': 12.5, 'y': 10.0, 'yaw': -np.pi/2 }
        #self.state = {'x': 10, 'y': 19.5, 'yaw': -np.pi/2 }
        #self.state = {'x': 7.5, 'y': 2.5, 'yaw': np.pi }
                # Parameters for initial pose (used if no /initial_pose is received)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', float(np.pi))  # radians

        self.state = {
            'x': float(self.get_parameter('initial_x').value),
            'y': float(self.get_parameter('initial_y').value),
            'yaw': float(self.get_parameter('initial_yaw').value),
        }

        # Whether to wait for an external initial pose before moving
        self.declare_parameter('wait_for_initial_pose', True)
        self.wait_for_initial_pose = bool(self.get_parameter('wait_for_initial_pose').value)
        self.initial_pose_received = False

        # Subscribe to an externally chosen initial pose (from GUI node)
        self.create_subscription(Pose, 'initial_pose', self.initial_pose_callback, 1)


        # Linear and angular velocities
        self.linear_velocity_x = self.linear_velocity_y = 0.0
        self.angular_velocity = 0.0

        # Deadman timeout (seconds)
        self.cmd_timeout = 0.5
        self.last_cmd_time = time.time()

        # Subscribes to cmd_vel (linear and angular velocities)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)

        # Publishes odometry
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 1)

        # Timer for simulation update
        self.last_time = time.time()
        self.create_timer(1.0 / self.update_rate, self.update_simulation)

        #self.get_logger().info("Kinematic Differential Drive Simulator Node started")

    def initial_pose_callback(self, msg: Pose):
        """Set the initial pose from an external selector (only once)."""
        if self.initial_pose_received:
            return  # ignore further updates

        self.state['x'] = float(msg.position.x)
        self.state['y'] = float(msg.position.y)

        # Extract yaw (heading) from quaternion
        q = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        _, _, yaw = euler_from_quaternion(q)
        self.state['yaw'] = float(yaw)

        self.initial_pose_received = True
        self.get_logger().info(
            f"Initial pose set from /initial_pose: "
            f"x={self.state['x']:.2f}, y={self.state['y']:.2f}, yaw={self.state['yaw']:.2f} rad"
        )


    def cmd_vel_callback(self, msg: Twist):
        """Callback to update linear and angular velocity from cmd_vel topic."""
        # recieve the cmd_vel in ego frame and move it to the world frame by rotating it
        # by the current yaw angle of the robot
        
        vel = (msg.linear.x)
        self.linear_velocity_x = np.cos(self.state['yaw']) * vel
        self.linear_velocity_y = np.sin(self.state['yaw']) * vel
        self.angular_velocity = msg.angular.z

        # Record when we last got a command
        self.last_cmd_time = time.time()
        # self.get_logger().info(
        #     f"Received cmd_vel: linear x={self.linear_velocity_x:.2f}, linear y = {self.linear_velocity_y:.2f}, angular={self.angular_velocity:.2f}"
        # )

    def update_simulation(self):
        """Update the robot's state based on the current velocities."""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        if self.wait_for_initial_pose and not self.initial_pose_received:
            # Just keep publishing the current (static) pose so others can see the robot
            self.publish_odometry()
            return
        
        if current_time - self.last_cmd_time > self.cmd_timeout:
            self.linear_velocity_x = 0.0
            self.linear_velocity_y = 0.0
            self.angular_velocity = 0.0

        # Update the robot's state using the kinematic model
        dx = self.linear_velocity_x * dt
        dy = self.linear_velocity_y * dt
        dyaw = self.angular_velocity * dt

        self.state['x'] += dx
        self.state['y'] += dy
        self.state['yaw'] += dyaw
        self.state['yaw'] = self.normalize_angle(self.state['yaw'])

        # Publish the updated odometry
        self.publish_odometry()

    def publish_odometry(self):
        """Publish the robot's odometry."""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"

        # Set the position
        odom_msg.pose.pose.position.x = float(self.state['x'])
        odom_msg.pose.pose.position.y = float(self.state['y'])
        odom_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        q = quaternion_from_euler(0.0, 0.0, self.state['yaw'])
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Set the velocity
        odom_msg.twist.twist.linear.x = self.linear_velocity_x
        odom_msg.twist.twist.linear.y = self.linear_velocity_y
        odom_msg.twist.twist.angular.z = self.angular_velocity

        self.odom_publisher.publish(odom_msg)
        # self.get_logger().info(
        #     f"Published odometry: x={self.state['x']:.2f}, y={self.state['y']:.2f}, yaw={math.degrees(self.state['yaw']):.2f}°"
        # )

    @staticmethod
    def normalize_angle(angle):
        """Normalize an angle to the range [0, 2*pi]."""
        angle = angle % (2.0 * np.pi) # Wrap to [0, 2*pi]
        if angle < 0.0:
            angle += 2.0 * np.pi
        
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = KinematicDiffDriveSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
