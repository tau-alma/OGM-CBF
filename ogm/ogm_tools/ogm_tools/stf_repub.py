import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import time

class StaticTFRepub(Node):

    def __init__(self):
        super().__init__('camera_broadcaster')

        self.declare_parameter('parent_source_frame', 'parent_source_frame')
        self.declare_parameter('child_source_frame', 'child_source_frame')
        self.declare_parameter('parent_report_frame', 'parent_report_frame')
        self.declare_parameter('child_report_frame', 'child_report_frame')

        self.parent_source_frame = self.get_parameter('parent_source_frame').value
        self.child_source_frame = self.get_parameter('child_source_frame').value
        self.parent_report_frame = self.get_parameter('parent_report_frame').value
        self.child_report_frame = self.get_parameter('child_report_frame').value
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.server_reset = self.create_service(
            Trigger, "reset", self.callback_reset)

        self.timer = self.create_timer(1., self.tick)

        self.t_source = None

        self.timer = self.create_timer(1., self.tick)

    def tick(self):
        
        if self.t_source is None:
            
            try:
                self.t_source = self.tf_buffer.lookup_transform(
                        self.parent_source_frame,
                        self.child_source_frame,
                        rclpy.time.Time(seconds=0, nanoseconds=0))
            except TransformException as ex:
                self.get_logger().info(str(ex))
                self.get_logger().info("waiting for %s --> %s" % (self.parent_source_frame, self.child_source_frame))
                self.t_source = None
                

            if self.t_source is not None:
                t_report = TransformStamped()
                t_report.header.stamp = self.t_source.header.stamp
                t_report.header.frame_id = self.parent_report_frame
                t_report.child_frame_id = self.child_report_frame
                t_report.transform = self.t_source.transform
                self.tf_static_broadcaster.sendTransform(t_report)

                self.get_logger().info("%s --> %s static init done" % (self.parent_report_frame, self.child_report_frame))
            
    def callback_reset(self, request, response):
        self.t_source = None
        response.message = "static tf reset"
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)

    repubbr = StaticTFRepub()

    rclpy.spin(repubbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
