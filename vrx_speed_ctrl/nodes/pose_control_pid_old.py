#!/usr/bin/env python
# license removed for brevity

import sys
import rospy
import rosparam
import rospkg
import math
from pid import PID
from geometry_msgs.msg import Twist
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
        self.current_pose = GeoPose()
        self.geopose_sp = GeoPose()
        self.sp_distance = None
        self.sp_heading = None
        self.current_heading = None
        self.vel_pub = None

    def callback_geopose(self,data):
        self.geopose_sp = data

        lat1 = self.geopose_sp.position.latitude
        lon1 = self.geopose_sp.position.longitude
        lat2 = self.current_pose.position.latitude
        lon2 = self.current_pose.position.longitude

        #print(lat1, lon1, lat2, lon2)

        # calc heading, current point desired point
        self.sp_heading = Geodesic.WGS84.Inverse(lat1, lon1, lat2, lon2)['azi1']
        print "sp_heading: ", self.sp_heading
        print "current_heading: ", self.current_heading

        # calc distance, current point desired point
        coords_1 = (lat1, lon1)
        coords_2 = (lat2, lon2)
        self.sp_distance = geopy.distance.vincenty(coords_1, coords_2).km*1000

        print "distance: ", self.sp_distance

        # controle
        self.pose_control()
        #self.angular_vel()
        #self.linear_vel()

    def callback_current_pose(self,data):
        self.current_pose.position.latitude = data.latitude
        self.current_pose.position.longitude = data.longitude

    def callback_current_heading(self,data):
        # pega odometria
        orientation_q = data.pose.pose.orientation
        # passa de quaternion para euler
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        angle = math.degrees(yaw)+180
        # guarda
        self.current_heading = self.norm_angle(angle)

    def pose_control(self):
        errorz = self.current_heading - self.sp_heading
        errorx = -self.sp_distance

        cmdz = self.norm_cmd(self.PID_h[0].update_PID(errorz))
        cmdx = self.norm_cmdx(self.PID_h[1].update_PID(errorx))

        print "cmdx: ", cmdx
        print "cmdz: ", cmdz
        
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
        if cmd > 2.5:
            cmd = 2.5
        if cmd < -2.5:
            cmd = -2.5
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
    rospy.Subscriber("robot_localization/gps/filtered",NavSatFix,node.callback_current_pose)
    rospy.Subscriber("robot_localization/odometry/filtered",Odometry,node.callback_current_heading)
    rospy.Subscriber("cmd_pose/sp",GeoPose,node.callback_geopose)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass         