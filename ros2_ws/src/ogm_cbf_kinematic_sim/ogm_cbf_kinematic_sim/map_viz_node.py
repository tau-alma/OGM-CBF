import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import matplotlib
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np

matplotlib.use('Agg')  # Non-interactive backend for ROS

class MapVizNode(Node):
    def __init__(self):
        super().__init__('map_viz_node')
        self.callback_group_async = ReentrantCallbackGroup()

        # Subscribers
        self.map_subscriber = self.create_subscription(Image, 'map_image', self.map_callback, 1, callback_group = self.callback_group_async)
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 1,  callback_group = self.callback_group_async)
        self.publisher_image_ = self.create_publisher(Image, '/map_traj_image', 1,  callback_group = self.callback_group_async)

        # Timer for publishing trajectory map
        self.timer = self.create_timer(0.1, self.timer_callback)

        # CVBridge for ROS <-> OpenCV conversions
        self.bridge = CvBridge()

        # Store map and trajectory
        self.map_image = None
        self.trajectory = []

        # Initialize Matplotlib figure
        self.fig, self.ax = plt.subplots(figsize=(6, 6))

        self.robot_x_pixel = 0.0

        self.get_logger().info("Map Visualization Node started")

    def timer_callback(self):
        """Publish the map with the trajectory overlaid."""
        if self.map_image is None:
            self.get_logger().warning("Map image not available. Skipping publishing.")
            return

        self.process_image()
        canvas = self.fig.canvas
        canvas.draw()
        data = np.frombuffer(canvas.tostring_rgb(), dtype=np.uint8)
        image = data.reshape(canvas.get_width_height()[::-1] + (3,))
        self.map_traj_image = image

        # Publish the processed image
        image_message = self.bridge.cv2_to_imgmsg(self.map_traj_image.astype(np.uint8), encoding='bgr8')
        self.publisher_image_.publish(image_message)

    def map_callback(self, msg: Image):
        """Callback to handle map updates."""
        if self.map_image is None:
            try:
                # Convert ROS image to NumPy array and save the map
                gray_map = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                self.map_image = gray_map  # Keep the grayscale map for Matplotlib
                self.get_logger().info("Map received and stored. Unsubscribing from map_image topic.")

                # Unsubscribe from the map_image topic
                self.map_subscriber
                self.destroy_subscription(self.map_subscriber)
            except Exception as e:
                self.get_logger().error(f"Failed to process map image: {e}")

    def process_image(self):
        """Draw the trajectory on the map using Matplotlib."""
        self.ax.clear()
        self.ax.imshow(self.map_image, cmap='gray')  # Display the map image

        # Plot the trajectory
        if self.trajectory:
            x_coords, y_coords = zip(*self.trajectory)
            self.ax.plot(x_coords, y_coords, 'bo-', markersize=3, label='Trajectory')  # Blue line with markers

        # Add grid, labels, and legend
        self.ax.set_xticks(range(0, self.map_image.shape[1], 50))
        self.ax.set_yticks(range(0, self.map_image.shape[0], 50))
        self.ax.grid(color='gray', linestyle='--', linewidth=0.5)
        self.ax.set_xlabel('X-axis (pixels)')
        self.ax.set_ylabel('Y-axis (pixels)')
        self.ax.legend(loc='upper right')
        self.fig.tight_layout()

    def odom_callback(self, msg: Odometry):
        
        """Callback to update robot pose and trajectory."""
        if self.map_image is None:
            self.get_logger().warning("Map not received yet. Cannot draw trajectory.")
            return

        # Get robot position from odometry
        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y

        # Convert to pixel coordinates
        robot_x_pixel, robot_y_pixel = self.pose_to_pixel(
            robot_x, robot_y, self.map_image.shape[1], self.map_image.shape[0], 0.05, [0, 0]
        )
    

        # Append to trajectory
    
        self.trajectory.append((robot_x_pixel, robot_y_pixel))
        self.get_logger().info(f"Added trajectory point: ({robot_x_pixel}, {robot_y_pixel})")

    def pose_to_pixel(self, x, y, im_width_pixel, im_height_pixel, resolution, map_origin):
        """Converts a world coordinate to a pixel coordinate on the map image."""
        return (x - map_origin[0]) / resolution, im_height_pixel - ((y - map_origin[1]) / resolution)


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = MapVizNode()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
