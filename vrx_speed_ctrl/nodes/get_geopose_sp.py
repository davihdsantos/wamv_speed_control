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
from geographic_msgs.msg import GeoPath
from geographic_msgs.msg import GeoPose
from geographic_msgs.msg import GeoPoint

class Node():
    def __init__(self):
        self.geopose_sp = None

    def callback_waypoints(self,data):
        for x in data.poses:
            print(x)

if __name__ == '__main__':

    rospy.init_node('position_control', anonymous=True)    

    node=Node()

    # Publisher
    #node.left_pub = rospy.Publisher("thrusters/left_thrust_cmd",Float32,queue_size=10)
    #node.right_pub = rospy.Publisher("thrusters/right_thrust_cmd",Float32,queue_size=10)
    #node.lateral_pub = rospy.Publisher("thrusters/lateral_thrust_cmd",Float32,queue_size=10)
    #node.left_msg = Float32()
    #node.right_msg = Float32()
    #node.lateral_msg = Float32()

    # Subscriber
    #rospy.Subscriber("robot_localization/odometry/filtered",Odometry,node.callback_vel)
    rospy.Subscriber("vrx/wayfinding/waypoints",GeoPath,node.callback_waypoints)
    #rospy.Subscriber("cmd_vel/sp",Twist,node.callback_control)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass