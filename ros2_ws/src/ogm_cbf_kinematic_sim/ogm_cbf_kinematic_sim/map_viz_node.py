import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib
matplotlib.use("Agg")          # before pyplot
import matplotlib.pyplot as plt
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
from ogm_cbf_kinematic_sim.utils import world_to_pixel
import math
from tf_transformations import euler_from_quaternion


class MapVizNode(Node):
    def __init__(self):
        super().__init__('map_viz_node')
        self.callback_group_async = ReentrantCallbackGroup()

        # Subscribers
        self.map_subscriber = self.create_subscription(Image, 'map_image', self.map_callback, 1, callback_group = self.callback_group_async)
        self.odom_subscriber = self.create_subscription(Odometry, 'odom_2', self.odom_callback, 1,  callback_group = self.callback_group_async)
        self.publisher_image_ = self.create_publisher(Image, '/map_traj_image', 1,  callback_group = self.callback_group_async)

        # Timer for publishing trajectory map
        self.timer = self.create_timer(0.1, self.timer_callback)

        # CVBridge for ROS <-> OpenCV conversions
        self.bridge = CvBridge()

        # Store map and trajectory
        self.map_image = None
        self.trajectory = []
        
        self.sample_every = 20         # keep 1 of every N odom msgs (avoid clutter)
        self._odom_k = 0

        self.arrow_len_px = 16     # length
        self.arrow_width  = 0.003  # shaft thickness (axes units)
        self.head_width   = 2.5
        self.head_length  = 2.5
        self.headaxislength = 3.0  


        # Initialize Matplotlib figure
        self.fig, self.ax = plt.subplots(figsize=(6, 6))

        self.robot_x_pixel = 0.0
        
        # self.get_logger().info("Map Visualization Node started")

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
        image_message = self.bridge.cv2_to_imgmsg(self.map_traj_image.astype(np.uint8), encoding='rgb8')
        self.publisher_image_.publish(image_message)

    def map_callback(self, msg: Image):
        """Callback to handle map updates."""
        if self.map_image is None:
            try:
                # Convert ROS image to NumPy array and save the map
                gray_map = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                self.map_image = gray_map  # Keep the grayscale map for Matplotlib
                #self.get_logger().info("Map received and stored. Unsubscribing from map_image topic.")

                # Unsubscribe from the map_image topic
                self.map_subscriber
                self.destroy_subscription(self.map_subscriber)
            except Exception as e:
                self.get_logger().error(f"Failed to process map image: {e}")

    def process_image(self):
        """Draw the trajectory on the map using Matplotlib."""
        self.ax.clear()
        self.ax.imshow(self.map_image, cmap='gray')  # Display the map image

        # # Plot the trajectory
        # if self.trajectory:
        #     x_coords, y_coords = zip(*self.trajectory)
        #     self.ax.plot(x_coords, y_coords, 'bo-', markersize=3, label='Trajectory')  # Blue line with markers
        if self.trajectory:
            #x_coords, y_coords, _ = zip(*self.trajectory)
            #self.ax.plot(x_coords, y_coords, 'bo-', markersize=3, label='Trajectory')  # Blue line with markers
    
            xs, ys, yaws = zip(*self.trajectory)
            # us = [self.arrow_len_px * math.cos(a) for a in yaws]
            # vs = [-self.arrow_len_px * math.sin(a) for a in yaws]  # minus: image y down

            # self.ax.quiver(xs, ys, us, vs,
            #             angles="xy", scale_units="xy", scale=1,
            #             width=0.003, color='blue')
            us = [self.arrow_len_px * math.cos(a) for a in yaws]
            vs = [-self.arrow_len_px * math.sin(a) for a in yaws]

            self.ax.quiver(xs, ys, us, vs,
                        angles="xy", scale_units="xy", scale=1,
                        width=self.arrow_width,
                        headwidth=self.head_width,
                        headlength=self.head_length,
                        headaxislength=self.headaxislength,

                        zorder=10, color='red')


        # Add grid, labels, and legend
        self.ax.set_xticks(range(0, self.map_image.shape[1], 50))
        self.ax.set_yticks(range(0, self.map_image.shape[0], 50))
        self.ax.grid(color='gray', linestyle='--', linewidth=0.5)
        #self.ax.set_xlabel('X-axis (pixels)')
        #self.ax.set_ylabel('Y-axis (pixels)')
        #self.ax.legend(loc='upper right')
        self.fig.tight_layout()
        t = self.ax.imshow(self.map_image, cmap='gray')
        cax = self.fig.colorbar(t, ax=self.ax)
        canvas = plt.gca().figure.canvas
        canvas.draw()
        data = np.frombuffer(canvas.tostring_rgb(), dtype=np.uint8)
        image = data.reshape(canvas.get_width_height()[::-1] + (3,))
        canvas.flush_events()
        plt.pause(0.1)
        cax.remove()
        self.h_img = image

    def odom_callback(self, msg: Odometry):
        
        """Callback to update robot pose and trajectory."""
        if self.map_image is None:
            self.get_logger().warning("Map not received yet. Cannot draw trajectory.")
            return

        # Get robot position from odometry
        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        robot_x_pixel, robot_y_pixel = world_to_pixel(robot_x, robot_y, img_height=self.map_image.shape[0])

        self._odom_k += 1
        if self._odom_k % self.sample_every == 0:
            self.trajectory.append((robot_x_pixel, robot_y_pixel, yaw))
    # def pose_to_pixel(self, x, y, im_width_pixel, im_height_pixel, resolution, map_origin):
    #     """Converts a world coordinate to a pixel coordinate on the map image."""
    #     return (x - map_origin[0]) / resolution, im_height_pixel - ((y - map_origin[1]) / resolution)


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
