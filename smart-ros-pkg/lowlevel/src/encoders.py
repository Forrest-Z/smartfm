#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
ROS node in charge of interfacing with the encoders phidget, and publishing
odometry messages (pose and vel) on the "encoders" topic.
'''

import roslib; roslib.load_manifest('lowlevel')
import rospy
from lowlevel.msg import Odo as OdometryMessage

from Phidgets.PhidgetException import *
from Phidgets.Events.Events import *
from Phidgets.Devices.Encoder import Encoder


#ROS stuffs
rospy.init_node('encoders_node')
pub = rospy.Publisher('encoders', OdometryMessage)
odom = OdometryMessage()

#Encoders / odometry parameters:
#4000 pulses for 1 revolution, corresponding to 1.3 meters
SCALE = 1.3 / 4000.0
DIST_BETWEEN_WHEELS = 1.0

# Odo messages are sent at a periodic rate 
PERIOD = 0.2 # period in seconds




def err(e):
    rospy.logerr("Phidget error %i: %s. Exiting..." % (e.code, e.details))
    exit(1)



#Create an encoder object
try:
    encoder = Encoder()
except RuntimeError as e:
    rospy.logerr("Runtime exception: %s. Exiting..." % e.details)
    exit(1)


#A callback function called whenever the position changed
def encoderPositionChange(e):
    source = e.device
    #rospy.loginfo("Encoder %i: Encoder %i -- Change: %i -- Time: %i -- Position: %i" % (source.getSerialNum(), e.index, e.positionChange, e.time, encoder.getPosition(e.index)))

    if e.index == 0:
        disp = e.positionChange * SCALE
        odom.pose += disp
        odom.vel = disp / (e.time / 1000.0)

    #da = (dispR-dispL) / WIDTH_BTW_GLIDE_WHEELS;
    #dl = (dispR+dispL) / 2;


#Set the callback
try:
    encoder.setOnPositionChangeHandler(encoderPositionChange)
except PhidgetException as e:
    err(e)


print("Opening phidget object....")
try:
    encoder.openPhidget()
except PhidgetException as e:
    err(e)

rospy.loginfo("Waiting for attach....")
try:
    encoder.waitForAttach(10000)
except PhidgetException as e:
    rospy.logerr("Phidget error %i: %s" % (e.code, e.details))
    try:
        encoder.closePhidget()
    except PhidgetException as e:
        err(e)
    exit(1)


encoder.setEnabled(0, True)



rospy.loginfo('Spinning....')
while not rospy.is_shutdown():
    rospy.loginfo('pose: %fm -- vel: %fm/s' % (odom.pose, odom.vel))
    pub.publish(odom)
    rospy.sleep(PERIOD)



print("Closing...")
try:
    encoder.closePhidget()
except PhidgetException as e:
    err(e)


rospy.loginfo("Done.")
exit(0)
