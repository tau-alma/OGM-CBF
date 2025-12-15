import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
#from tf_transformations import euler_from_quaternion
from cv_bridge import CvBridge
import numpy as np
import cv2


def euler_from_quaternion(q: np.array):
    """Calculates the euler form from the quaternion

    Args:
        q (numpy array [4x1]): Quaternion state.
    Returns: 
        euler (numpy array [3x1]): Euler angles corresponding to the quaternion.
    """
    qw, qx, qy, qz = q
    t0 = +2.0 * (qw * qx + qy * qz)
    t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(t0, t1)
    
    t2 = +2.0 * (qw * qy - qz * qx)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = np.arcsin(t2)
    
    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(t3, t4)
    euler = np.array([yaw, pitch, roll])  # same order as in matlab simulink reference model!
    return euler


class MAP:
    def __init__(self):
        self.width = 200
        self.height = 200


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('map_to_local')
        
        self.odom_sub = self.create_subscription(
            Odometry,
            "odometry/global",
            self.odom_callback,
            1
        )

        self.pose_sub = self.create_subscription(
            Pose,
            "robot_pose",
            self.pose_callback,
            1
        )

        self.img_sub = self.create_subscription(
            Image,
            "map",
            self.map_im_callback,
            1
        )

        # We'll use the same publisher to publish the updated global map.
        self.img_pub = self.create_publisher(
            Image, 
            "map_im",
            1
        )

        # self.img_pub_comb = self.create_publisher(
        #     Image, 
        #     "map_combined",
        #     1
        # )
        
        self.cv_bridge = CvBridge()
        self.origin_x = 0.0  # from metadata if available
        self.origin_y = 0.0
        self.resolution = 0.05

        # Load the map and also cache a static copy to reset after integration.
        self.map  = cv2.imread("/home/nvidia/K2--cleared.png", cv2.IMREAD_GRAYSCALE)
        self.map = np.fliplr(np.flip(self.map))
        self.map_static = self.map.copy()
        
        self.scan = None
        self.map_received = False

    def map_im_callback(self, msg: Image):
        # Scan received by lidar on MIR.
        self.map_received = True
        self.scan = self.cv_bridge.imgmsg_to_cv2(msg)
        
    def show_map(self):
        cv2.imshow("map cropped and rotated from disk", self.map)
        cv2.waitKey(1)
        
    def odom_callback(self, msg: Odometry):
        self.pose_callback(msg.pose.pose)
        
    def pose_callback(self, msg: Pose):
        if self.map_received:
            pose_x = msg.position.x
            pose_y = msg.position.y
            quat = msg.orientation
            _, _, heading = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))

            # Convert pose to map coordinates.
            map_x = int((pose_x - self.origin_x) / self.resolution)
            map_y = int((pose_y - self.origin_y) / self.resolution)
            current_cell = (map_x, map_y)
            
            # Rotate a patch from the map.
            img = rotate_image(self.map, heading, current_cell)
            # Combine the rotated patch with the online scan.
            target = cv2.addWeighted(255 - img, 0.5, 255 - self.scan, 0.5, 0)
            ret, target = cv2.threshold(target, 127, 255, cv2.THRESH_BINARY)
            mask_radius = 12
            mask = np.zeros((200, 200), dtype=np.uint8)
            cv2.circle(mask, (100, 100), mask_radius, 255, -1)
            target[mask == 255] = 0

            # (Optional) Publish the intermediate images.
            #msg_img_prev = self.cv_bridge.cv2_to_imgmsg(255 - img, encoding='8UC1')
            #msg_img_prev.header.stamp = self.get_clock().now().to_msg()
            #self.img_pub_comb.publish(msg_img_prev)

            # Integrate the combined patch back into the global map.
            self.integrate_combined_image(target, heading, current_cell)
            #self.show_map()

            # Now publish the updated global map.
            msg_map = self.cv_bridge.cv2_to_imgmsg(self.map, encoding='8UC1')
            msg_map.header.stamp = self.get_clock().now().to_msg()
            self.img_pub.publish(msg_map)

            # Reset the working map using the cached static copy.
            self.map = self.map_static.copy()

    def integrate_combined_image(self, combined_patch, heading, current_cell):
        """
        Inverts the rotation and flip applied in rotate_image so that the 200x200 combined_patch
        is transformed back into the global map coordinate frame.
        Then, for pixels where the combined patch has information, update self.map.
        """
        cropped_map = MAP()  # Dimensions: 200x200.
        ROI_size = cropped_map.width * 2  # 400 pixels.

        # Create a blank 400x400 image and center the combined_patch.
        combined_full = np.zeros((ROI_size, ROI_size), dtype=combined_patch.dtype)
        offset = ROI_size // 2 - combined_patch.shape[0] // 2  # should be 100.
        combined_full[offset:offset+combined_patch.shape[0],
                      offset:offset+combined_patch.shape[1]] = combined_patch

        # Recreate the rotation matrix.
        center = (ROI_size // 2, ROI_size // 2)
        angle = -heading * (180.0 / np.pi) - 90
        T = cv2.getRotationMatrix2D(center, angle, 1.0)
        T_inv = cv2.invertAffineTransform(T)
        
        patch_before_crop = cv2.warpAffine(combined_full, T_inv, (ROI_size, ROI_size))
        patch_original_ROI = np.fliplr(patch_before_crop)
        
        # Define the ROI in the global map.
        global_top_left_x = int(current_cell[0] - ROI_size // 2)
        global_top_left_y = int(current_cell[1] - ROI_size // 2)
        
        map_height, map_width = self.map.shape[:2]
        roi_x_start = max(global_top_left_x, 0)
        roi_y_start = max(global_top_left_y, 0)
        roi_x_end = min(global_top_left_x + ROI_size, map_width)
        roi_y_end = min(global_top_left_y + ROI_size, map_height)
        
        patch_x_start = roi_x_start - global_top_left_x
        patch_y_start = roi_y_start - global_top_left_y
        patch_x_end = patch_x_start + (roi_x_end - roi_x_start)
        patch_y_end = patch_y_start + (roi_y_end - roi_y_start)
        
        recovered_patch = patch_original_ROI[patch_y_start:patch_y_end, patch_x_start:patch_x_end]
        mask = recovered_patch > 0
        
        map_region = self.map[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
        map_region[mask] = 255 - recovered_patch[mask]
        self.map[roi_y_start:roi_y_end, roi_x_start:roi_x_end] = map_region
        
        self.get_logger().info("Integrated combined patch into global map.")


def rotate_image(img, heading, current_cell):
    """
    Extracts a region of interest from img around current_cell, applies a horizontal flip,
    rotates the ROI based on heading, and crops the central 200x200 patch.
    """
    cropped_map = MAP()
    center = (cropped_map.width * 2 // 2, cropped_map.height * 2 // 2)
    rotation_matrix = cv2.getRotationMatrix2D(center, -heading * (180.0 / np.pi) - 90, 1.0)

    rotated_p1 = (-cropped_map.width * 2 // 2, -cropped_map.height * 2 // 2)
    img_roi = img[
        int(rotated_p1[1] + current_cell[1]): int(rotated_p1[1] + current_cell[1] + cropped_map.height * 2),
        int(rotated_p1[0] + current_cell[0]): int(rotated_p1[0] + current_cell[0] + cropped_map.width * 2)
    ]

    img_roi = np.fliplr(img_roi)
    warped_image = cv2.warpAffine(img_roi, rotation_matrix, (img_roi.shape[0], img_roi.shape[1]))
    warped_image = warped_image[100:300, 100:300]
    return warped_image


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
