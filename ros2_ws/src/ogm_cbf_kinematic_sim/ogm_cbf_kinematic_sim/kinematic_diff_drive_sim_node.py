import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Quaternion
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
import math
import time
import numpy as np


class KinematicDiffDriveSimNode(Node):
    def __init__(self):
        super().__init__('kinematic_diff_drive_sim_node')
        self.declare_parameter('update_rate', 100.0)  # Hz
        self.update_rate = self.get_parameter('update_rate').value

        # Robot state: x, y, and yaw (orientation)
        #self.state = {'x': 12.5, 'y': 10.0, 'yaw': -np.pi/2 }
        self.state = {'x': 10, 'y': 19.5, 'yaw': -np.pi/2 }

        # Linear and angular velocities
        self.linear_velocity_x = self.linear_velocity_y = 0.0
        self.angular_velocity = 0.0

        # Subscribes to cmd_vel (linear and angular velocities)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)

        # Publishes odometry
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 1)

        # Timer for simulation update
        self.last_time = time.time()
        self.create_timer(1.0 / self.update_rate, self.update_simulation)

        self.get_logger().info("Kinematic Differential Drive Simulator Node started")

    def cmd_vel_callback(self, msg: Twist):
        """Callback to update linear and angular velocity from cmd_vel topic."""
        self.linear_velocity_x = msg.linear.x
        self.linear_velocity_y = msg.linear.y
        self.angular_velocity = msg.angular.z
        self.get_logger().info(
            f"Received cmd_vel: linear x={self.linear_velocity_x:.2f}, linear y = {self.linear_velocity_y:.2f}, angular={self.angular_velocity:.2f}"
        )

    def update_simulation(self):
        """Update the robot's state based on the current velocities."""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

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
        self.get_logger().info(
            f"Published odometry: x={self.state['x']:.2f}, y={self.state['y']:.2f}, yaw={math.degrees(self.state['yaw']):.2f}°"
        )

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
