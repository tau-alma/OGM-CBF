#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import wandb
import numpy as np
from datetime import datetime

class WandbLogger(Node):
    def __init__(self):
        super().__init__('wandb_logger')
        # 1) Init W&B run
        wandb.init(
            project="ogm-cbf",                # choose your project
            #name=str(self.get_clock().now().to_msg().sec),  # or any run name
            name=datetime.now().strftime("run_%Y%m%d_%H%M%S"),  # e.g. "run_20250509_142530"
            config={"node": "wandb_logger"}
        )
        # 2) Subscriptions
        self.cbfs_sub = self.create_subscription(
            Float64MultiArray,
            '/cbf_array',
            self.on_cbf,
            1)


        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.on_odom,
            1)

        self.mapimg_sub = self.create_subscription(
            Image,
            '/map_traj_image',
            self.on_map_image,
            1)

        self.bridge = CvBridge()
        # 3) state buffers
        self.latest = {}

    def on_cbf(self, msg: Float64MultiArray):
        # h and ḣ+α(h) are in msg.data[0], msg.data[1]
        self.latest['h']     = msg.data[0]
        self.latest['h_dot'] = msg.data[1]
        self.latest['h_dot_alpha_h'] = msg.data[2]

        # log as soon as we have them
        wandb.log({
            "h":     self.latest['h'],
            "h_dot": self.latest['h_dot'],
            "h_dot_alpha_h": self.latest['h_dot_alpha_h'],
        }, step=wandb.run.step + 1)

    
    def on_odom(self, msg: Odometry):
        act_v = msg.twist.twist.linear.x
        act_w = msg.twist.twist.angular.z
        # assume cmd already received; if not, default to None
        wandb.log({
            "act_v": act_v,
            "act_w": act_w,
        }, step=wandb.run.step + 1)

    def on_map_image(self, msg: Image):
        # convert to OpenCV BGR numpy array
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        wandb.log({
            "map_viz": wandb.Image(img)
        }, step=wandb.run.step + 1)

def main(args=None):
    rclpy.init(args=args)
    node = WandbLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
