#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as ROT

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from tf2_ros import TransformException, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from nav_msgs.msg import Odometry

class TF2Odom(Node):

    def __init__(self):
        super().__init__('tf2odom')
        self.declare_parameter('parent_frame', rclpy.Parameter.Type.STRING)
        self.declare_parameter('target_frame', rclpy.Parameter.Type.STRING)
        self.parent_frame = self.get_parameter('parent_frame').value 
        self.target_frame = self.get_parameter('target_frame').value 
 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.pub = self.create_publisher(
                Odometry,
                'odom_out',
                1)

        self.sub = self.create_subscription(
                Odometry,
                'odom_in',
                self.callback_odom,
                qos_profile_sensor_data)

    def callback_odom(self, msg):

        fsf, tf_msg_to_target = self.get_tf()   
        # close or 0-time static
        if tf_msg_to_target is not None:
            self.get_logger().info ("transfrom at %f" % (fsf))

            T_odom = self.T_from_odom(msg)
            T_tf = self.T_from_tf(tf_msg_to_target)

            T = np.matmul(T_odom, T_tf)
            R = T[:3,:3]  
            q = ROT.from_matrix(R).as_quat()
            t = T[:3,3]

            out = Odometry()  
            out.header = msg.header
            out.child_frame_id = self.target_frame
            out.pose.pose.position.x = t[0]
            out.pose.pose.position.y = t[1]
            out.pose.pose.position.z = t[2]
            out.pose.pose.orientation.x = q[0]
            out.pose.pose.orientation.y = q[1]
            out.pose.pose.orientation.z = q[2]
            out.pose.pose.orientation.w = q[3]

            self.pub.publish(out)
        else:
            self.get_logger().info ("no transfrom at %f" % (fsf))

    def get_tf(self):
        try:
            tf_msg_to_target = self.tf_buffer.lookup_transform(
                    self.parent_frame,
                    self.target_frame,
                    rclpy.time.Time()
                    )
            fsf = tf_msg_to_target.header.stamp.sec + tf_msg_to_target.header.stamp.nanosec/1e9 
        except TransformException as ex:
            self.get_logger().info (str(ex))
            #self.get_logger().info ("failed to get %s %s" % (self.target_frame, self.parent_frame))
            fsf = -1
            tf_msg_to_target = None
        return fsf, tf_msg_to_target 

    def T_from_odom(self, odom_msg):

        t = np.array([
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z,
            ])

        q = np.array([
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w,
            ])
        R = ROT.from_quat(q).as_matrix()

        T = np.eye(4)
        T[:3,3] = t
        T[:3,:3] = R

        return T

    def T_from_tf(self, tf_msg_to_target):

        t = np.array([
            tf_msg_to_target.transform.translation.x,
            tf_msg_to_target.transform.translation.y,
            tf_msg_to_target.transform.translation.z,
            ])

        q = np.array([
            tf_msg_to_target.transform.rotation.x,
            tf_msg_to_target.transform.rotation.y,
            tf_msg_to_target.transform.rotation.z,
            tf_msg_to_target.transform.rotation.w,
            ])
        R = ROT.from_quat(q).as_matrix()

        T = np.eye(4)
        T[:3,3] = t
        T[:3,:3] = R

        return T


    

def main(args=None):
    rclpy.init(args=args)

    tf2odom = TF2Odom()

    #tf_future = tf2odom.tf_buffer.wait_for_transform_async(
    #        target_frame=tf2odom.target_frame,
    #        source_frame=tf2odom.parent_frame,
    #        time=rclpy.time.Time())
    #rclpy.spin_until_future_complete(tf2odom, tf_future)
    
    rclpy.spin(tf2odom)
    tf2odom.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

