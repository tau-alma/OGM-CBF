#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as ROT

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from rover_custom_msgs.msg import ControllerCommands

class OdomCmd(Node):

    def __init__(self):
        super().__init__('odom_cmd')
        
        self.pub = self.create_publisher(
                Odometry,
                'odom_out',
                1)

        self.sub = self.create_subscription(
                Odometry,
                'odom_in',
                self.callback_odom,
                qos_profile_sensor_data)

        self.sub_cmd = self.create_subscription(
                ControllerCommands,
                'cmd',
                self.callback_cmd,
                qos_profile_sensor_data)

        self.cmd_dir = 0

    def callback_cmd(self, msg):

        left = np.sign(msg.command_left_motor)
        right = np.sign(-1*msg.command_right_motor)

        is_fwd = (left > 0) or (right > 0) 
        is_bck = (left < 0) or (right < 0) 

        if is_fwd and not is_bck:
            self.cmd_dir = 1
        elif not is_fwd and is_bck:
            self.cmd_dir = -1



    def callback_odom(self, msg):

        q = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
            ])

        if self.cmd_dir < 0:
            R = ROT.from_quat(q).as_matrix()
            R_flip = np.eye(3)
            R_flip[0,0] = -1
            R_flip[1,1] = -1
            R_cmd = np.matmul(R_flip, R)
            q_cmd = ROT.from_matrix(R_cmd).as_quat()
        else:
            q_cmd = q

        out = Odometry()  
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id + "_cmd"
        out.pose.pose.position = msg.pose.pose.position
        out.pose.pose.orientation.x = q_cmd[0]
        out.pose.pose.orientation.y = q_cmd[1]
        out.pose.pose.orientation.z = q_cmd[2]
        out.pose.pose.orientation.w = q_cmd[3]


        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)

    odom_cmd = OdomCmd()

    rclpy.spin(odom_cmd)
    odom_cmd.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

