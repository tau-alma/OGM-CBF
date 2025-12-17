"""
Teemu Mökkönen

This code is simple implementation of turning ps4 controller commands to 
ros2 messages that are compatible with turtlebot3 model. This code is mostly designed to 
work with certain hardware/simulation of ros2 avant model
"""

import rclpy
import numpy as np
from rclpy.node import Node
import threading
from math import copysign
from pyPS4Controller.controller import Controller
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import sys

class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.cont = ControllerInterface()
        self.bumber_scale = 32431

    def on_L3_up(self, value):
        self.cont.telescope_vel(-value)

    def on_L3_down(self, value):
        self.cont.telescope_vel(-value)
    
    def on_L3_right(self, value):
        if abs(value) < 2000:
            self.cont.angular_vel_callback(0.0)
        else:
            self.cont.angular_vel_callback(value-2000)
         
    def on_L3_left(self, value):
        if abs(value) < 2000:
            self.cont.angular_vel_callback(0.0)
        else:
            self.cont.angular_vel_callback(value+2000)
    
    # def on_L3_y_at_rest(self):
    #     self.cont.linear_vel_callback(0.0)

    def on_L3_x_at_rest(self):
        self.cont.angular_vel_callback(0.0)
    
    def on_R3_up(self, value):
        if abs(value) < 3500:
            self.cont.boom_vel(0)
        else:
            self.cont.boom_vel(-value)
         
    def on_R3_down(self, value):
        if abs(value) < 3500:
            self.cont.boom_vel(0)
        else:
            self.cont.boom_vel(-value)
    
    def on_R3_right(self, value):
        if abs(value) < 3500:
            self.cont.bucket_vel(0)
        else:
            self.cont.bucket_vel(value)
         
    def on_R3_left(self, value):
        if abs(value) < 3500:
            self.cont.bucket_vel(0)
        else:
            self.cont.bucket_vel(value)
    
    def on_R3_y_at_rest(self):
        self.cont.boom_vel(0.0)

    def on_R3_x_at_rest(self):
        self.cont.bucket_vel(0.0)

    def on_R2_press(self, value):
        value = value
        if value < 0:
            value = value + self.cont.scaling_factor
    
        elif value >= 0:
            value = (value) + self.cont.scaling_factor
        self.cont.linear_vel_callback(value)

    def on_L2_release(self):
        self.cont.linear_vel_callback(0.0)

    def on_L2_press(self, value):
        value = value
        if value < 0:
            value = value + self.cont.scaling_factor
    
        elif value >= 0:
            value = (value) + self.cont.scaling_factor

        value = -value
        self.cont.linear_vel_callback(value)

    def on_R2_release(self):
        self.cont.linear_vel_callback(0.0)
    

class ControllerInterface(Node):
    def __init__(self):
        # init node and publishers
        super().__init__('controller_interface')
        #self.publisher_ = self.create_publisher(JointState, 'wanted_speeds', 10)
        self.cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.manipulator_commands_ = self.create_publisher(JointState, 'manipulator_commands', 10)
        timer_period = 0.01  # seconds
        # params for sending
        self.v_x = 0.0
        self.omega_z = 0.0
        self.boom = 0.0
        self.bucket = 0.0
        self.telescope = 0.0
        self.scaling_factor = 32767
        self.send_manipulator_command()
        self.send_vel_command()

    def linear_vel_callback(self, vel):
        self.v_x = vel / (self.scaling_factor * 2)
        #self.send_vel_command()

    def angular_vel_callback(self, vel):
        self.omega_z = - vel / self.scaling_factor
        #self.send_vel_command()

    def send_vel_command(self):
        threading.Timer(0.005, self.send_vel_command).start()
        msg = JointState()
        msg_twist = Twist()
        

        msg_twist.linear.x = np.clip(self.v_x, -1.0, 1.0)

        if self.v_x < 0:
            msg_twist.angular.z = -self.omega_z
        else:
            msg_twist.angular.z = self.omega_z
        #msg_twist.angular.z = self.omega_z

        msg.position.append(self.v_x)
        msg.position.append(self.omega_z)
        self.cmd.publish(msg_twist)
        #self.publisher_.publish(msg)
        
    def boom_vel(self, vel):
        self.boom = (vel / self.scaling_factor) * 0.6

    def bucket_vel(self, vel):
        self.bucket = vel / self.scaling_factor * 0.6

    def telescope_vel(self, vel):
        self.telescope = vel / (self.scaling_factor * 2)
        

    def send_manipulator_command(self):
        """
        Turns the saved velocity commands to the ROS2 jointState
        message to be sent to the avant hardware interface.
        """
        threading.Timer(0.005, self.send_manipulator_command).start()
        if abs(self.v_x) == 0:
            msg = JointState()
            msg.velocity = [0.0, 0.0, 0.0]
            msg.velocity[0] = self.boom
            msg.velocity[1] = self.bucket
            if abs(self.telescope) < 0.05:
                self.telescope = 0.0
            msg.velocity[2] = self.telescope
            self.manipulator_commands_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    intface = "/dev/input/"
    intface += sys.argv[1]
    controller = MyController(interface=intface, connecting_using_ds4drv=False)
    controller.listen()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
