import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge

from tf_transformations import quaternion_from_euler

import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox, Button

from ogm_cbf_kinematic_sim.utils import pixel_to_world
import numpy as np



class StartPoseSelector(Node):
    def __init__(self):
        super().__init__('start_pose_selector')

        self.bridge = CvBridge()
        self.map_image = None

        # Parameters for map geometry (same as in your other nodes)
        self.declare_parameter('map_resolution', 0.05)  # meters per pixel
        self.declare_parameter('map_origin_x', 0.0)
        self.declare_parameter('map_origin_y', 0.0)

        # Subscribe to map_image once
        self.map_sub = self.create_subscription(
            Image, 'map_image', self.map_callback, 1
        )

        # Publisher for the selected initial pose
        self.initial_pose_pub = self.create_publisher(Pose, 'initial_pose', 1)

        # Will be filled after user chooses
        self.selected_pose = None
        self.timer = None

    # ---------- ROS callbacks ----------

    def map_callback(self, msg: Image):
        if self.map_image is not None:
            return  # already stored
        try:
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            self.map_image = gray
            self.get_logger().info(
                f"Received map image of size {gray.shape[1]}x{gray.shape[0]}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to convert map image: {e}")

    def wait_for_map(self):
        """Spin until we have received the map."""
        self.get_logger().info("Waiting for map_image ...")
        while rclpy.ok() and self.map_image is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.map_image is not None:
            self.get_logger().info("Map image received, opening selection GUI.")

    # ---------- Coordinate conversion ----------

    # def pixel_to_world(self, px: float, py: float):
    #     """
    #     Convert pixel coordinates (px, py) to world coordinates (x, y) [meters].

    #     Uses the same convention as your other nodes:
    #         pixel_x = (x_real - origin_x) / res
    #         pixel_y = img_height - ((y_real - origin_y) / res)
    #     so we invert that here.
    #     """
    #     resolution = float(self.get_parameter('map_resolution').value)
    #     origin_x = float(self.get_parameter('map_origin_x').value)
    #     origin_y = float(self.get_parameter('map_origin_y').value)

    #     img_height = self.map_image.shape[0]

    #     x_world = px * resolution + origin_x
    #     y_world = (img_height - py) * resolution + origin_y

    #     return x_world, y_world

    # ---------- GUI logic ----------

    def ask_user_for_start_pose(self):
        """
        Opens a matplotlib window where you can:
          - click on the map to set start position
          - or type x [m], y [m], yaw [deg] in text boxes
        """
        img = self.map_image
        if img is None:
            raise RuntimeError("No map image available in ask_user_for_start_pose")

        # Default guess: center of map, yaw = 0 deg
        resolution = float(self.get_parameter('map_resolution').value)
        h, w = img.shape
        default_x = 10.30#(w * resolution) / 2.0
        default_y = 9.0#(h * resolution) / 2.0
        default_yaw_deg = 0.0#180.0

        # Internal state that the GUI modifies
        state = {
            'x': default_x,
            'y': default_y,
            'yaw_deg': default_yaw_deg,
        }

        fig, ax = plt.subplots()
        plt.subplots_adjust(bottom=0.3)  # space for text boxes & button

        ax.imshow(img, cmap='gray', origin='upper')
        ax.set_title("Click to choose start position, or edit the boxes below")

        # --- Text boxes ---

        # x [m]
        axbox_x = plt.axes([0.15, 0.18, 0.2, 0.05])
        tb_x = TextBox(axbox_x, "x [m]", initial=f"{default_x:.2f}")

        # y [m]
        axbox_y = plt.axes([0.45, 0.18, 0.2, 0.05])
        tb_y = TextBox(axbox_y, "y [m]", initial=f"{default_y:.2f}")

        # yaw [deg]
        axbox_yaw = plt.axes([0.15, 0.1, 0.2, 0.05])
        tb_yaw = TextBox(axbox_yaw, "yaw [deg]", initial=f"{default_yaw_deg:.1f}")

        # Button to confirm
        ax_button = plt.axes([0.45, 0.1, 0.3, 0.07])
        btn = Button(ax_button, "Use this start pose")

        def on_click(event):
            """Mouse click on the map: update x,y based on pixel position."""
            if event.inaxes != ax:
                return
            if event.xdata is None or event.ydata is None:
                return

            px = float(event.xdata)
            py = float(event.ydata)
            x_world, y_world = pixel_to_world(px, py, img_height=self.map_image.shape[0])

            state['x'] = x_world
            state['y'] = y_world

            # Update text boxes
            tb_x.set_val(f"{x_world:.2f}")
            tb_y.set_val(f"{y_world:.2f}")

            print(f"Clicked at pixel ({px:.1f}, {py:.1f}) -> world ({x_world:.3f}, {y_world:.3f})")

        def submit_x(text):
            try:
                state['x'] = float(text)
            except ValueError:
                print("Invalid x value, ignoring")

        def submit_y(text):
            try:
                state['y'] = float(text)
            except ValueError:
                print("Invalid y value, ignoring")

        def submit_yaw(text):
            try:
                state['yaw_deg'] = float(text)
            except ValueError:
                print("Invalid yaw value, ignoring")

        def on_button_press(_):
            # Close the GUI when user confirms
            plt.close(fig)

        fig.canvas.mpl_connect('button_press_event', on_click)
        tb_x.on_submit(submit_x)
        tb_y.on_submit(submit_y)
        tb_yaw.on_submit(submit_yaw)
        btn.on_clicked(on_button_press)

        # This blocks until the window is closed (button or close window)
        plt.show()

        yaw_rad = math.radians(state['yaw_deg'])
        self.get_logger().info(
            f"Selected start pose: x={state['x']:.3f} m, y={state['y']:.3f} m, yaw={yaw_rad:.3f} rad"
        )
        return state['x'], state['y'], yaw_rad

    # ---------- Publishing initial pose repeatedly ----------

    def start_publishing_initial_pose(self, x, y, yaw):
        """Store the chosen pose and start a timer that republishes it."""
        self.selected_pose = (x, y, yaw)

        # Publish at 2 Hz so late subscribers still receive it
        self.timer = self.create_timer(0.5, self.timer_publish_initial_pose)

    def timer_publish_initial_pose(self):
        if self.selected_pose is None:
            return
        x, y, yaw = self.selected_pose

        pose_msg = Pose()
        pose_msg.position.x = float(x)
        pose_msg.position.y = float(y)
        pose_msg.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, float(yaw))
        pose_msg.orientation.x = q[0]
        pose_msg.orientation.y = q[1]
        pose_msg.orientation.z = q[2]
        pose_msg.orientation.w = q[3]

        self.initial_pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StartPoseSelector()

    try:
        # 1) Wait until map_image arrives
        node.wait_for_map()

        # 2) Let the user choose a start pose
        x, y, yaw = node.ask_user_for_start_pose()

        # 3) After GUI closes, start publishing that pose periodically
        node.start_publishing_initial_pose(x, y, yaw)

        # 4) Keep node alive so it can keep publishing /initial_pose
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("StartPoseSelector interrupted, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
