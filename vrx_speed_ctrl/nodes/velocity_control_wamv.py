#!/usr/bin/env python
# license removed for brevity

import sys
import rospy
import rosparam
import rospkg
from pid import PID
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class Node():
    def __init__(self,PID_h, control):
        self.PID_h = PID_h
        self.control = control
        self.left_pub = None
        self.right_pub = None
        self.lateral_pub = None
        self.left_msg =None
        self.right_msg =None
        self.lateral_msg =None
        self.vel =None
        self.vel_sp =None
        self.Iacc = 0.0

    def callback_vel(self,data):
        self.vel = data.twist.twist

    def callback_control(self,data):

        self.update_pid_param()

        self.vel_sp = data

        errorx = self.vel.linear.x - self.vel_sp.linear.x
        errory = self.vel.linear.y - self.vel_sp.linear.y
        errorz = self.vel.angular.z - self.vel_sp.angular.z

        cmdx = self.PID_h[0].update_PID(errorx)
        cmdy = self.PID_h[1].update_PID(errory)
        cmdz = self.PID_h[2].update_PID(errorz)

        # x and z control
        self.left_msg.data = cmdx - cmdz
        self.right_msg.data = cmdx + cmdz

        # y control
        self.lateral_msg.data = cmdy

        self.left_pub.publish(self.left_msg)
        self.right_pub.publish(self.right_msg)
        self.lateral_pub.publish(self.lateral_msg)

        for x, y in zip(self.PID_h, self.control):
            rospy.loginfo('Control: '+y)
            rospy.loginfo(x)
        #rospy.loginfo(errorz)

    def update_pid_param(self):
        # get ROS Parameters
        for x, i in zip(self.control, range(len(self.PID_h))):
            # get PID param for x speed control
            PID_dict = rospy.get_param('control/speed_'+x)
            P = PID_dict.get('P')
            I = PID_dict.get('I')
            D = PID_dict.get('D')
            Imax = PID_dict.get('Imax')
            Imin = PID_dict.get('Imin')
            self.PID_h[i].set_gains(P, I, D, Imax, Imin)

if __name__ == '__main__':

    rospy.init_node('velocity_control', anonymous=True)

    rospack = rospkg.RosPack()

    control = ['x', 'y', 'z']
    PID_h = []

    for x in control:
        # get PID param for x speed control
        PID_dict = rospy.get_param('control/speed_'+x)
        P = PID_dict.get('P')
        I = PID_dict.get('I')
        D = PID_dict.get('D')
        Imax = PID_dict.get('Imax')
        Imin = PID_dict.get('Imin')
        PID_h.append(PID(P, I, D, Imax, Imin))

    node=Node(PID_h, control)

    # Publisher
    node.left_pub = rospy.Publisher("thrusters/left_thrust_cmd",Float32,queue_size=10)
    node.right_pub = rospy.Publisher("thrusters/right_thrust_cmd",Float32,queue_size=10)
    node.lateral_pub = rospy.Publisher("thrusters/lateral_thrust_cmd",Float32,queue_size=10)
    node.left_msg = Float32()
    node.right_msg = Float32()
    node.lateral_msg = Float32()

    # Subscriber
    rospy.Subscriber("robot_localization/odometry/filtered",Odometry,node.callback_vel)
    rospy.Subscriber("cmd_vel/sp",Twist,node.callback_control)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    if rospy.is_shutdown():
        try:
            rosparam.dump_params(rospack.get_path('vrx_speed_ctrl')+'/config/speed_pid.yaml', 'control')
            rospy.loginfo("PID params dumped to .yaml file") 
        except rosparam.RosParamException:
            rospy.loginfo("PARAMS NOT SAVED!. CHECK FILEPATH") 
            
