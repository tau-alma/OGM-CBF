import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os
from ament_index_python.packages import get_package_share_directory

class MapPublisherNode(Node):
    def __init__(self):
        super().__init__('map_publisher_node')
        self.publisher_ = self.create_publisher(Image, 'map_image', 1)
        self.timer = self.create_timer(0.01, self.publish_map)
        self.bridge = CvBridge()
       
        default_map = os.path.join(
            get_package_share_directory('ogm_cbf_kinematic_sim'),
            'map', 'festia_map', 'festia_map.png'
        )
        self.declare_parameter('map_path', default_map)
        self.map_path = self.get_parameter('map_path').value
        self.get_logger().info(f'Map path set to: {self.map_path}')

    def publish_map(self):
        #self.get_logger().info('Attempting to read map...')
        img = cv2.imread(self.map_path, cv2.IMREAD_GRAYSCALE)
        #print(img)
        if img is not None:
            #self.get_logger().info('Map loaded successfully')
            msg = self.bridge.cv2_to_imgmsg(img, encoding='mono8')
            self.publisher_.publish(msg)
            #self.get_logger().info('Published map image')
        else:
            self.get_logger().error('Failed to load map. Check the file path.')

def main(args=None):
    try:
        rclpy.init(args=args)
        node = MapPublisherNode()
        rclpy.spin(node)
    
    except Exception as e:
        print(f"Error occurred: {e}")

    finally:
        print('Shutting down...')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()