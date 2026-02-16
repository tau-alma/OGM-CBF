#!/usr/bin/env python3
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from tf_transformations import euler_from_quaternion


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def depth_to_meters(cv_img, encoding, depth_max_m):
    # common CARLA ROS bridge cases:
    # - 32FC1: meters
    # - 16UC1: millimeters (often) -> meters
    # - mono8: normalized -> map to [0, depth_max_m]
    if encoding in ("32FC1", "32FC"):
        d = cv_img.astype(np.float32)
    elif encoding in ("16UC1", "mono16"):
        d = cv_img.astype(np.float32) / 1000.0
    elif encoding in ("mono8", "8UC1"):
        d = (cv_img.astype(np.float32) / 255.0) * float(depth_max_m)
    else:
        d = cv_img.astype(np.float32)
    d[(~np.isfinite(d)) | (d <= 1e-6)] = np.inf
    d[d > depth_max_m] = np.inf
    return d


def seg_to_obstacle_mask(seg_bgr_or_bgra, mode, obstacle_labels, obstacle_bgr_colors):
    """
    returns mask_u8 where obstacles=255, else 0
    mode:
      - 'labels': seg has raw class id in channel 2 (CARLA Raw semantic) like vcbf file :contentReference[oaicite:2]{index=2}
      - 'cityscapes': seg is colored palette; match colors
    """
    if seg_bgr_or_bgra.ndim == 3 and seg_bgr_or_bgra.shape[2] == 4:
        seg = seg_bgr_or_bgra[:, :, :3]
    else:
        seg = seg_bgr_or_bgra

    if mode == "labels":
        # CARLA Raw semantic often encodes label id in R channel; in BGRA that is channel 2 (same as your code) :contentReference[oaicite:3]{index=3}
        labels = seg[:, :, 2].astype(np.uint8)
        m = np.isin(labels, np.array(obstacle_labels, dtype=np.uint8))
        return (m.astype(np.uint8) * 255)

    # cityscapes palette mode: match exact BGR colors
    mask = np.zeros(seg.shape[:2], dtype=np.uint8)
    for (b, g, r) in obstacle_bgr_colors:
        hit = (seg[:, :, 0] == b) & (seg[:, :, 1] == g) & (seg[:, :, 2] == r)
        mask[hit] = 255
    return mask


def dist_to_obs_and_grad(mask_obs_u8):
    # build "free=255, obs=0" for OpenCV distanceTransform
    free = (255 - mask_obs_u8).astype(np.uint8)
    dist = cv2.distanceTransform(free, cv2.DIST_L2, 3).astype(np.float32)  # pixels
    gy, gx = np.gradient(dist)
    return dist, gx.astype(np.float32), gy.astype(np.float32)


class VCBFNode(Node):
    def __init__(self):
        super().__init__("vcbf_node")

        # ---- params (edit defaults to your topics) ----
        self.declare_parameter("odom_topic", "/carla/ego_vehicle/odometry")
        self.declare_parameter("cmd_topic", "/cmd_vel")

        self.declare_parameter("depth0_topic", "/carla/ego_vehicle/rgb_depth/image")
        self.declare_parameter("seg0_topic",   "/carla/ego_vehicle/rgb_seg/image")
        self.declare_parameter("depth1_topic", "/carla/ego_vehicle/rgb_depth/image")
        self.declare_parameter("seg1_topic",   "/carla/ego_vehicle/rgb_seg/image")
        self.declare_parameter("depth2_topic", "/carla/ego_vehicle/rgb_depth/image")
        self.declare_parameter("seg2_topic",   "/carla/ego_vehicle/rgb_seg/image")

        self.declare_parameter("controller_hz", 20.0)


        self.declare_parameter("seg_mode", "labels")  # 'labels' or 'cityscapes'
        self.declare_parameter("obstacle_labels", [10, 4])  # vehicles=10, pedestrians=4 (if labels mode)
        # cityscapes colors in BGR (OpenCV): vehicles RGB(0,0,142)->BGR(142,0,0); pedestrians RGB(220,20,60)->BGR(60,20,220)
        # was: [[142,0,0],[60,20,220]]
        self.declare_parameter("obstacle_bgr_colors", [142, 0, 0, 60, 20, 220])


        self.declare_parameter("depth_max_m", 30.0)
        self.declare_parameter("roi_px", 64)

        # simple safe-control params (replace with your QP later)
        self.declare_parameter("target_yaw_deg", 90.0)
        self.declare_parameter("v_max", 1.0)
        self.declare_parameter("w_max", 1.0)
        self.declare_parameter("k_heading", 1.5)
        self.declare_parameter("k_avoid", 1.0)
        self.declare_parameter("d_stop", 1.0)  # m
        self.declare_parameter("d_slow", 3.0)  # m
        self.declare_parameter("d_gate", 4.0)  # m

        self.bridge = CvBridge()

        # ---- state buffers ----
        self.have_odom = False
        self.x = self.y = self.yaw = 0.0

        self.depth = [None, None, None]
        self.depth_enc = [None, None, None]
        self.seg = [None, None, None]
        self.have_cam = [False, False, False]

        # ---- ROS I/O ----
        self.sub_odom = self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.cb_odom, 10
        )

        self.sub_depth = []
        self.sub_seg = []
        for i in range(3):
            dt = self.get_parameter(f"depth{i}_topic").value
            st = self.get_parameter(f"seg{i}_topic").value
            self.sub_depth.append(self.create_subscription(Image, dt, lambda m, k=i: self.cb_depth(m, k), qos_profile_sensor_data))
            self.sub_seg.append(self.create_subscription(Image, st, lambda m, k=i: self.cb_seg(m, k), qos_profile_sensor_data))

        self.pub_cmd = self.create_publisher(Twist, self.get_parameter("cmd_topic").value, 10)

        hz = float(self.get_parameter("controller_hz").value)
        self.timer = self.create_timer(1.0 / max(1e-3, hz), self.step)

    def cb_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        self.yaw = float(yaw)
        self.have_odom = True

    def cb_depth(self, msg: Image, k: int):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.depth[k] = cv_img
        self.depth_enc[k] = msg.encoding
        self.have_cam[k] = self.have_cam[k] and (self.seg[k] is not None) or (self.seg[k] is not None)

    def cb_seg(self, msg: Image, k: int):
        # keep as BGR(A) or raw; passthrough preserves channels
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.seg[k] = cv_img
        self.have_cam[k] = self.have_cam[k] and (self.depth[k] is not None) or (self.depth[k] is not None)

    def step(self):
        if not self.have_odom:
            return
        if any((self.depth[i] is None or self.seg[i] is None) for i in range(3)):
            return

        seg_mode = str(self.get_parameter("seg_mode").value)
        obstacle_labels = list(self.get_parameter("obstacle_labels").value)
        flat = list(self.get_parameter("obstacle_bgr_colors").value)  # [b,g,r,b,g,r,...]
        if len(flat) % 3 != 0:
            flat = flat[: (len(flat)//3)*3]
        obstacle_bgr_colors = [tuple(flat[i:i+3]) for i in range(0, len(flat), 3)]


        depth_max_m = float(self.get_parameter("depth_max_m").value)
        roi_px = int(self.get_parameter("roi_px").value)

        # compute per-camera min depth + grad-x (for steering sign)
        cam_min_d = []
        cam_grad_x = []

        for i in range(3):
            d = depth_to_meters(self.depth[i], self.depth_enc[i], depth_max_m)

            # seg image may be mono/label or BGR(A); we pass through
            seg_img = self.seg[i]
            if seg_img.ndim == 2:
                seg_img = np.repeat(seg_img[:, :, None], 3, axis=2)

            # try convert to BGR if it's RGB (best-effort)
            if seg_img.ndim == 3 and seg_img.shape[2] == 3:
                seg_bgr = seg_img
            else:
                seg_bgr = seg_img

            mask_obs = seg_to_obstacle_mask(seg_bgr, seg_mode, obstacle_labels, obstacle_bgr_colors)

            dist_px, gx, gy = dist_to_obs_and_grad(mask_obs)

            H, W = d.shape[:2]
            cx, cy = W // 2, H // 2
            r = min(roi_px, cx - 1, cy - 1)

            # only obstacle pixels contribute to min-depth
            proc = np.full((H, W), np.inf, dtype=np.float32)
            proc[mask_obs == 255] = d[mask_obs == 255]

            sub = proc[cy - r:cy + r, cx - r:cx + r]
            if not np.isfinite(sub).any():
                cam_min_d.append(np.inf)
                cam_grad_x.append(0.0)
                continue

            iy, ix = np.unravel_index(int(np.nanargmin(sub)), sub.shape)
            py = (cy - r) + iy
            px = (cx - r) + ix

            cam_min_d.append(float(sub[iy, ix]))
            cam_grad_x.append(float(gx[py, px]))

        # pick worst camera (smallest depth)
        worst = int(np.nanargmin(np.array(cam_min_d, dtype=np.float32)))
        dmin = cam_min_d[worst]
        gx_w = cam_grad_x[worst]

        # ---- control (replace with your QP controller) ----
        target_yaw = math.radians(float(self.get_parameter("target_yaw_deg").value))
        v_max = float(self.get_parameter("v_max").value)
        w_max = float(self.get_parameter("w_max").value)
        k_heading = float(self.get_parameter("k_heading").value)
        k_avoid = float(self.get_parameter("k_avoid").value)
        d_stop = float(self.get_parameter("d_stop").value)
        d_slow = float(self.get_parameter("d_slow").value)
        d_gate = float(self.get_parameter("d_gate").value)

        # speed gate by obstacle distance
        if not np.isfinite(dmin):
            v = v_max
            gate = 0.0
        else:
            if dmin <= d_stop:
                v = 0.0
            elif dmin >= d_slow:
                v = v_max
            else:
                v = v_max * (dmin - d_stop) / max(1e-6, (d_slow - d_stop))
            gate = float(np.clip((d_gate - dmin) / max(1e-6, d_gate), 0.0, 1.0))

        w_heading = k_heading * wrap_pi(target_yaw - self.yaw)

        # if gx>0 => safer to the right => turn right => angular.z negative
        w_avoid = (-k_avoid * np.sign(gx_w)) * gate

        w = float(np.clip(w_heading + w_avoid, -w_max, w_max))

        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub_cmd.publish(msg)


def main():
    rclpy.init()
    n = VCBFNode()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
