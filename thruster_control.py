#!/usr/bin/env python

import rospy
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.msg import AttitudeTarget, 
import time

self.pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)#2
rospy.init_node('mavros_takeoff_python')
rate = rospy.Rate(10)

# Set Mode
print "\nSetting Mode"
rospy.wait_for_service('/mavros/set_mode')
try:
    change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    response = change_mode(custom_mode="STABILIZED")
    rospy.loginfo(response)
except rospy.ServiceException as e:
    print("Set mode failed: %s" %e)

# Arm
print "\nArming"
rospy.wait_for_service('/mavros/cmd/arming')
try:
    arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    response = arming_cl(value = True)
    rospy.loginfo(response)
except rospy.ServiceException as e:
    print("Arming failed: %s" %e)


while not rospy.is_shutdown():
    qw,qx,qy,qz = euler_to_quaternion_angle(self.roll, self.pitch, self.yaw)
   
    AT = AttitudeTarget()
    AT.header = self.h
    AT.type_mask = 7
    AT.orientation.x = qx
    AT.orientation.y = qy
    AT.orientation.z = qz
    AT.orientation.w = qw
    AT.body_rate.x = 0
    AT.body_rate.y = 0
    AT.body_rate.z = 0
    AT.thrust = self.thrust
    
    self.pub.publish(AT)
    self.r.sleep() #For frequency
