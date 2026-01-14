import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class CameraBroadcaster(Node):

    def __init__(self):
        super().__init__('camera_broadcaster')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('tag_link_frame', 'april_link')
        self.declare_parameter('tag_frame', 'tagStandard41h12:0')
        self.declare_parameter('camera_frame', 'l500_color_optical_frame')
        self.declare_parameter('tag_init_frame', 'april_init_link')

        self.map_frame = self.get_parameter('map_frame').value
        self.tag_link_frame = self.get_parameter('tag_link_frame').value
        self.tag_frame = self.get_parameter('tag_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.tag_init_frame = self.get_parameter('tag_init_frame').value
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.timer = self.create_timer(1.0, self.tick)

        self.t_map_2_tag_link = None
        self.t_tag_2_camera = None

    def tick(self):
        
            
        try:
            self.t_map_2_tag_link = self.tf_buffer.lookup_transform(
                    self.tag_link_frame,
                    self.map_frame,
                    rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info("waiting for %s --> %s" % (self.map_frame, self.tag_link_frame))
            self.t_map_2_tag_link = None

        try:
            self.t_tag_2_camera = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    self.tag_frame,
                    rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info("waiting for %s --> %s" % (self.tag_frame, self.camera_frame))
            self.t_tag_2_camera = None
        
        if self.t_map_2_tag_link is not None and self.t_tag_2_camera is not None:
            t_map_2_connect = TransformStamped()
            t_map_2_connect.header.stamp = self.get_clock().now().to_msg()
            t_map_2_connect.header.frame_id = self.map_frame
            t_map_2_connect.child_frame_id = self.tag_init_frame
            t_map_2_connect.transform = self.t_map_2_tag_link.transform
            print (t_map_2_connect)
            self.tf_static_broadcaster.sendTransform(t_map_2_connect)

            #t_connect_2_camera = self.t_tag_2_camera
            #t_connect_2_camera.header.frame_id = self.tag_init_frame
            #self.tf_static_broadcaster.sendTransform(t_connect_2_camera)

            self.get_logger().info("%s --> %s static init done" % (self.map_frame, self.camera_frame))
            

def main(args=None):
    rclpy.init(args=args)

    cambr = CameraBroadcaster()

    rclpy.spin(cambr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
