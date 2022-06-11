#!/usr/bin/env python
# license removed for brevity

import sys
import rospy
import rosparam
from pid import PID
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Float64

class Node():
    def __init__(self):
        self.spd_x_state_pub = None
        self.spd_x_sp_pub = None
        self.spd_z_state_pub = None
        self.spd_z_sp_pub = None
        self.spd_y_state_pub = None
        self.spd_y_sp_pub = None
        self.left_pub = None
        self.right_pub = None
        self.lateral_pub = None
        self.cmd_x = Float32() 
        self.cmd_z = Float32()
        self.cmd_y = Float32()

    def callback_state(self,data):
        self.spd_x_state_pub.publish(data.twist.twist.linear.x)
        self.spd_z_state_pub.publish(data.twist.twist.angular.z)
        self.spd_y_state_pub.publish(data.twist.twist.linear.y)

    def callback_sp(self,data):
        self.spd_x_sp_pub.publish(data.linear.x)
        self.spd_z_sp_pub.publish(data.angular.z)
        self.spd_y_sp_pub.publish(data.linear.y)

    def callback_cmd_x(self,data):
        self.cmd_x.data = data.data
        self.send_thrusters_cmd()

    def callback_cmd_z(self,data):
        self.cmd_z.data = data.data

    def callback_cmd_y(self,data):
        self.cmd_y.data = data.data
        self.lateral_pub.publish(self.cmd_y.data)

    def send_thrusters_cmd(self):
        self.left_pub.publish(self.cmd_x.data - self.cmd_z.data)
        self.right_pub.publish(self.cmd_x.data + self.cmd_z.data)

if __name__ == '__main__':

    rospy.init_node('velocity_control', anonymous=True)

    node=Node()

    # Publisher
    node.left_pub = rospy.Publisher("thrusters/left_thrust_cmd",Float32,queue_size=10)
    node.right_pub = rospy.Publisher("thrusters/right_thrust_cmd",Float32,queue_size=10)
    node.lateral_pub = rospy.Publisher("thrusters/lateral_thrust_cmd",Float32,queue_size=10)

    node.spd_x_state_pub = rospy.Publisher("control/speed_x/state",Float64,queue_size=10)
    node.spd_x_sp_pub = rospy.Publisher("control/speed_x/sp",Float64,queue_size=10)

    node.spd_z_state_pub = rospy.Publisher("control/speed_z/state",Float64,queue_size=10)
    node.spd_z_sp_pub = rospy.Publisher("control/speed_z/sp",Float64,queue_size=10)

    node.spd_y_state_pub = rospy.Publisher("control/speed_y/state",Float64,queue_size=10)
    node.spd_y_sp_pub = rospy.Publisher("control/speed_y/sp",Float64,queue_size=10)

    # Subscriber
    rospy.Subscriber("robot_localization/odometry/filtered",Odometry,node.callback_state)
    rospy.Subscriber("cmd_vel/sp",Twist,node.callback_sp)

    rospy.Subscriber("control/speed_x/cmd",Float64,node.callback_cmd_x)
    rospy.Subscriber("control/speed_z/cmd",Float64,node.callback_cmd_z)
    rospy.Subscriber("control/speed_y/cmd",Float64,node.callback_cmd_y)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass