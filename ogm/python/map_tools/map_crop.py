import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from tf_transformations import euler_from_quaternion
from cv_bridge import CvBridge
import numpy as np
import cv2

class MAP:
    def __init__(self):
        self.width = 200
        self.height = 200

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('map_to_local')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            1)
        
        self.resolution = 0.5

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


        self.img_pub = self.create_publisher(
            Image, 
            "map_im",
            1
        )
        self.cv_bridge = CvBridge()
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.resolution = 0.05
        self.map_received = False
        self.map = None


    def map_callback(self, msg: OccupancyGrid):
        # Extracting information from the occupancy grid message
        self.map_received = True
        width = msg.info.width
        height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        data = msg.data

        # Convert data to a numpy array
        data = np.array(data).reshape((height, width))

        # Convert to CV2 image
        img = np.zeros((height, width, 3), dtype=np.uint8)
        img[data >= 66] = [0, 0, 0]  # Occupied cells in black
        img[data <= 66] = [255, 255, 255]  # Free cells in white
        #img = np.fliplr(img)
        #img = np.rot90(img, 3)
        img[2420:2460, 2620:2660]= (0, 255, 0)
        show = cv2.resize(img, None, fx=0.2, fy=0.2, interpolation=cv2.INTER_NEAREST)
        # Rescale image to match resolution
        cv2.imshow("map cropped and rotated", show)
        cv2.waitKey(0)
        self.map = img#cv2.resize(img, None, fx=self.resolution, fy=self.resolution, interpolation=cv2.INTER_NEAREST)

    def odom_callback(self, msg: Odometry):
        self.pose_callback(msg.pose.pose)
        
    def pose_callback(self, msg: Pose):
            # Extracting odometry information
        if self.map_received:
            pose_x = msg.position.x
            pose_y = msg.position.y
            quat = msg.orientation
            _, _, heading = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))

            # Convert pose to map coordinates
            map_x = int((pose_x - self.origin_x) / self.resolution)
            map_y = int((pose_y - self.origin_y) / self.resolution)
            map = (map_x, map_y)
            img = rotate_image(self.map, heading, map)
            msg_img: Image = self.cv_bridge.cv2_to_imgmsg(img, encoding='bgr8')
            msg_img.header.stamp = self.get_clock().now().to_msg()
            cv2.imshow("map cropped and rotated", img)
            cv2.waitKey(1)
            self.img_pub.publish(msg_img)

        

def rotate_image(img, heading, current_cell):
    # Calculate center point
    cropped_map = MAP()
    center = (cropped_map.width * 2 // 2, cropped_map.height * 2 // 2)

    # Create rotation matrix
    rotation_matrix = cv2.getRotationMatrix2D(center, -heading * (180.0 / np.pi) - 90, 1.0)

    # Extract region of interest from the image
    rotated_p1 = (- cropped_map.width * 2 // 2, - cropped_map.height * 2 // 2)
    img_roi = img[int(rotated_p1[1] + current_cell[1]):int(rotated_p1[1] + current_cell[1] + cropped_map.height * 2),
                  int(rotated_p1[0] + current_cell[0]):int(rotated_p1[0] + current_cell[0] + cropped_map.width * 2)]

    img_roi = np.fliplr(img_roi)
    #cv2.imshow("map", img_roi)
    #cv2.waitKey(1)
    # Perform the rotation
    warped_image = cv2.warpAffine(img_roi, rotation_matrix, (img_roi.shape[0], img_roi.shape[1]))
    warped_image = warped_image[100:300, 100:300]

    return warped_image


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()