#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
A node that subscribes to golfcar_brake and forwards it to brake_angle (Arduino).
This will turn on/off the stop lights.
'''

import roslib; roslib.load_manifest('lowlevel')
import rospy
from std_msgs.msg import Float64
from golfcar_halstreamer.msg import brakepedal as BrakePedalMsg


rospy.init_node('stop_light_node')
arduino_pub = rospy.Publisher('arduino/brake_angle', Float64)

def callback(msg):
    # in golfcar_lowlevel, braking is given as a negative angle.
    # in arduino, it is given as a positive angle.
    arduino_pub.publish(Float64(-msg.angle));

sub = rospy.Subscriber('golfcar_brake', BrakePedalMsg, callback)
rospy.spin()
