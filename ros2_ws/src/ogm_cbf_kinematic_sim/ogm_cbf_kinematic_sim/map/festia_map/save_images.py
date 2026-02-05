#!/usr/bin/env python3
import os
import time
import argparse
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CompressedImage

import cv2
import numpy as np
from cv_bridge import CvBridge


def stamp_to_str(stamp) -> str:
    return f"{int(stamp.sec):010d}_{int(stamp.nanosec):09d}"


class ImageSaver(Node):
    def __init__(self, topic: str, out_dir: str, stride: int, max_frames: int | None,
                 normalize_mono16: bool):
        super().__init__("image_saver")
        self.topic = topic
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)

        self.stride = max(1, stride)
        self.max_frames = max_frames
        self.normalize_mono16 = normalize_mono16

        self.bridge = CvBridge()
        self.rx_count = 0
        self.saved_count = 0

        # Detect actual topic type (wait briefly for discovery)
        msg_type = self._detect_topic_type(topic, timeout_sec=3.0)
        if msg_type is None:
            self.get_logger().error(
                f"Could not detect type for topic '{topic}'. Is the bag playing?"
            )
            raise RuntimeError("Topic type not found")

        if msg_type == "sensor_msgs/msg/Image":
            self.create_subscription(Image, topic, self.cb_image, qos_profile_sensor_data)
            self.get_logger().info(f"Subscribed to {topic} as sensor_msgs/msg/Image")
        elif msg_type == "sensor_msgs/msg/CompressedImage":
            self.create_subscription(CompressedImage, topic, self.cb_compressed, qos_profile_sensor_data)
            self.get_logger().info(f"Subscribed to {topic} as sensor_msgs/msg/CompressedImage")
        else:
            self.get_logger().error(f"Unsupported message type on {topic}: {msg_type}")
            raise RuntimeError(f"Unsupported type: {msg_type}")

        self.get_logger().info(f"Saving PNGs to: {self.out_dir}")
        self.get_logger().info(f"Stride: {self.stride} | Max frames: {self.max_frames if self.max_frames else 'no limit'}")

    def _detect_topic_type(self, topic: str, timeout_sec: float):
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            for name, types in self.get_topic_names_and_types():
                if name == topic and len(types) > 0:
                    # pick the first (usually only) type
                    return types[0]
            time.sleep(0.1)
        return None

    def _maybe_save(self, cv_img, stamp):
        self.rx_count += 1
        if (self.rx_count - 1) % self.stride != 0:
            return

        ts = stamp_to_str(stamp)
        base = f"frame_{self.saved_count:06d}_{ts}"
        out_path = self.out_dir / f"{base}.png"

        ok = cv2.imwrite(str(out_path), cv_img)
        if not ok:
            self.get_logger().error(f"Failed to write: {out_path}")
            rclpy.shutdown()
            return

        # Optional: extra viewable preview for mono16
        if self.normalize_mono16 and cv_img.dtype == np.uint16:
            norm = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            cv2.imwrite(str(self.out_dir / f"{base}_norm8.png"), norm)

        self.saved_count += 1

        if self.max_frames is not None and self.saved_count >= self.max_frames:
            self.get_logger().info(f"Reached max_frames={self.max_frames}. Stopping.")
            rclpy.shutdown()

    def cb_image(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            rclpy.shutdown()
            return
        self._maybe_save(cv_img, msg.header.stamp)

    def cb_compressed(self, msg: CompressedImage):
        try:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            cv_img = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)  # preserves grayscale if present
        except Exception as e:
            self.get_logger().error(f"Compressed decode failed: {e}")
            rclpy.shutdown()
            return

        if cv_img is None:
            self.get_logger().error("cv2.imdecode returned None.")
            rclpy.shutdown()
            return

        # ALWAYS save as PNG
        self._maybe_save(cv_img, msg.header.stamp)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", required=True)
    ap.add_argument("--out_dir", default="./saved_png")
    ap.add_argument("--stride", type=int, default=1)
    ap.add_argument("--max_frames", type=int, default=0, help="0 = no limit")
    ap.add_argument("--normalize_mono16", action="store_true")
    args = ap.parse_args()

    max_frames = None if args.max_frames == 0 else args.max_frames

    rclpy.init()
    node = ImageSaver(
        topic=args.topic,
        out_dir=args.out_dir,
        stride=args.stride,
        max_frames=max_frames,
        normalize_mono16=args.normalize_mono16,
    )
    rclpy.spin(node)


if __name__ == "__main__":
    main()
