#!/usr/bin/env python
# license removed for brevity

import sys
import rospy
import rosparam
import rospkg
import math
from pid import PID
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geographic_msgs.msg import GeoPose
from geographiclib.geodesic import Geodesic
from sensor_msgs.msg import NavSatFix
import geopy.distance
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Node():
    def __init__(self, PID_h, control):
        self.PID_h = PID_h
        self.control = control
        self.current_pose = Point()
        self.geopose_sp = Point()
        self.sp_distance = None
        self.sp_heading = None
        self.current_heading = None
        self.vel_pub = None
        self.sp_yaw = None

    def callback_geopose(self,data):
        self.geopose_sp = data

        orientation_q = data.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        angle = math.degrees(yaw)
        self.sp_yaw = self.norm_angle(angle)
        #print "sp_yaw: ", self.sp_yaw

        sp_x = self.geopose_sp.position.latitude
        sp_y = self.geopose_sp.position.longitude

        x1 = sp_x
        y1 = sp_y
        x0 = self.current_pose.x
        y0 = self.current_pose.y

        myradians = math.atan2(y1-y0, x1-x0)
        self.sp_heading = self.norm_angle(math.degrees(myradians))

        #print "sp_heading: ", self.sp_heading
        #print "current_heading: ", self.current_heading

        self.sp_distance = math.sqrt(((x0-x1)**2)+((y0-y1)**2))
        #self.sp_distance = x0-x1 

        #print "distance: ", self.sp_distance

        # controle
        self.pose_control()
        #self.angular_vel()
        #self.linear_vel()

    def callback_current_odometry(self,data):
        # pega odometria
        orientation_q = data.pose.pose.orientation
        self.current_pose = data.pose.pose.position

        # passa de quaternion para euler
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        angle = math.degrees(yaw)
        # guarda
        self.current_heading = self.norm_angle(angle)

    def pose_control(self):
        if self.sp_distance > 0 and self.sp_distance < 2:
            errorz = self.current_heading - self.sp_yaw
            errorz = self.norm_angle(errorz)
            #print "cond: ", True
        else:
            errorz = self.current_heading - self.sp_heading
            #errorz = self.norm_angle(errorz)

        #print "errorz: ", errorz

        errorx = -self.sp_distance

        cmdz = self.norm_cmd(self.PID_h[0].update_PID(errorz))
        cmdx = self.norm_cmdx(self.PID_h[1].update_PID(errorx))

        #print "cmdx: ", cmdx
        #print "cmdz: ", cmdz

        # monta mensagem para cmd_vel
        msg = Twist()
        msg.linear.x = cmdx
        msg.angular.z = cmdz

        # publica
        self.vel_pub.publish(msg)

    def norm_angle(self, angle):
        if angle > 180:
            angle -= 360
        if angle < -180:
            angle += 180
        return angle

    def norm_cmd(self, cmd):
        if cmd > 1:
            cmd = 1
        if cmd < -1:
            cmd = -1
        return cmd

    def norm_cmdx(self, cmd):
        speed = 3.0
        if cmd > speed:
            cmd = speed
        if cmd < -speed:
            cmd = -speed
        return cmd

if __name__ == '__main__':

    rospy.init_node('geopose_control', anonymous=True)

    control = ['heading', 'distance']
    PID_h = []

    for x in control:
        # get PID param for x speed control
        PID_dict = rospy.get_param('control/pose_'+x)
        P = PID_dict.get('P')
        I = PID_dict.get('I')
        D = PID_dict.get('D')
        Imax = PID_dict.get('Imax')
        Imin = PID_dict.get('Imin')
        PID_h.append(PID(P, I, D, Imax, Imin))

    node=Node(PID_h, control)

    node.vel_pub = rospy.Publisher("cmd_vel/sp",Twist,queue_size=10)

    # Subscriber
    rospy.Subscriber("robot_localization/odometry/filtered",Odometry,node.callback_current_odometry)
    rospy.Subscriber("cmd_pose/sp",GeoPose,node.callback_geopose)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass         