#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from nav_msgs.msg import Odometry

class TransformAligner(Node):

    def __init__(self):
        super().__init__('tf2odom')
        self.declare_parameter('map_frame', rclpy.Parameter.Type.STRING)
        self.declare_parameter('target_frame', rclpy.Parameter.Type.STRING)
        self.map_frame = self.get_parameter('map_frame').value 
        self.target_frame = self.get_parameter('target_frame').value 
 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.pub = self.create_publisher(Odometry, 'odom', 1)

        self.timer = self.create_timer(.01, self.tick)

    def tick(self):

        fsf, tf_msg_to_target = self.get_tf()   
        # close or 0-time static
        if tf_msg_to_target is not None:
            self.get_logger().info ("transfrom at %f" % (fsf))

            msg = Odometry()  
            msg.header = tf_msg_to_target.header
            msg.child_frame_id = tf_msg_to_target.child_frame_id
            msg.pose.pose.position = tf_msg_to_target.transform.translation 
            msg.pose.pose.orientation = tf_msg_to_target.transform.rotation

            self.pub.publish(msg)
        else:
            self.get_logger().info ("no transfrom at %f" % (fsf))

    def get_tf(self):
        try:
            tf_msg_to_target = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    self.map_frame,
                    rclpy.time.Time()
                    )
            fsf = tf_msg_to_target.header.stamp.sec + tf_msg_to_target.header.stamp.nanosec/1e9 
        except TransformException as ex:
            print (ex)
            fsf = -1
            tf_msg_to_target = None
        return fsf, tf_msg_to_target    
    

def main(args=None):
    rclpy.init(args=args)

    tf2odom = TF2Odom()
    rclpy.spin(tf2odom)
    tf2odom.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

