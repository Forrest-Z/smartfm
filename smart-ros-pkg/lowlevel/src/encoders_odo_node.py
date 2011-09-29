#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
ROS node in charge of interfacing with the encoders phidget, and publishing
odometry messages on the "encoders" topic.

We are reading from 2 encoders (left and right) so we can get the 2D pose of
the vehicle (x,y,theta) and velocities (v,w). This will be published as a
nav_msgs/Odometry and tf broadcast (as recommended by the navigation stacks
tutorials).
'''

import roslib; roslib.load_manifest('lowlevel')
import rospy
from math import sin,cos,pi
from datetime import datetime

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

from Phidgets.PhidgetException import *
from Phidgets.Events.Events import *
from Phidgets.Devices.Encoder import Encoder



def err(e):
    '''A helper function to report Phidget errors'''
    rospy.logerr("Phidget error %i: %s. Exiting..." % (e.code, e.details))
    exit(1)


class CountBuffer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.dp = 0 #dist
        self.dt = 0 #time

    def add(self,e):
        self.dp += e.positionChange
        self.dt += e.time


class PhidgetEncoder:
    '''Monitor the pose of each encoders'''

    def __init__(self, scale, distBtwWheels):
        self.scale = scale
        self.distBtwWheels = distBtwWheels

        self.left = 0
        self.right = 1
        self.countBufs = {self.left: CountBuffer(), self.right: CountBuffer()}

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.v = 0.0
        self.w = 0.0

        self.initialize()
        self.encoder.setEnabled(self.left, True)
        self.encoder.setEnabled(self.right, True)

        self.odomPub = rospy.Publisher('odom',Odometry)
        self.odomBroadcaster = TransformBroadcaster()
        self.then = None


    def initialize(self):
        '''Connect to the phidget and init communication.'''

        #Create an encoder object (from the Phidget library)
        try:
            self.encoder = Encoder()
        except RuntimeError as e:
            rospy.logerr("Runtime exception: %s. Exiting..." % e.details)
            exit(1)

        #Set the callback
        try:
            self.encoder.setOnPositionChangeHandler(self.encoderPositionChange)
        except PhidgetException as e:
            err(e)


        rospy.loginfo("Opening phidget object....")
        try:
            self.encoder.openPhidget()
        except PhidgetException as e:
            err(e)

        rospy.loginfo("Waiting for attach....")
        try:
            self.encoder.waitForAttach(10000)
        except PhidgetException as e:
            rospy.logerr("Phidget error %i: %s" % (e.code, e.details))
            try:
                self.encoder.closePhidget()
            except PhidgetException as e:
                err(e)
            exit(1)


    def close(self):
        rospy.loginfo("Closing...")
        try:
            self.encoder.closePhidget()
        except PhidgetException as e:
            err(e)


    def encoderPositionChange(self, e):
        '''A callback function called whenever the position changed'''

        rospy.loginfo("Encoder %i: Encoder %i -- Change: %i -- Time: %i -- Position: %i" % (e.device.getSerialNum(), e.index, e.positionChange, e.time, self.encoder.getPosition(e.index)))

        if e.index in self.countBufs.keys():
            self.countBufs[e.index].add(e)
            dt = self.countBufs[e.index].dt
            if dt > 50000: #in milliseconds
                dl = self.countBufs[self.left].dp * self.scale
                dr = self.countBufs[self.right].dp * self.scale
                d_dist = (dl+dr)/2
                d_th = (dr-dl)/self.distBtwWheels

                d_x = cos(d_th) * d_dist
                d_y = -sin(d_th) * d_dist
                self.x += cos(self.th)*d_x - sin(self.th)*d_y
                self.y += sin(self.th)*d_x + cos(self.th)*d_y
                self.th += d_th
                self.v = d_dist/dt
                self.w = d_th/dt

                self.pubOdo()

                self.countBufs[self.left].reset()
                self.countBufs[self.right].reset()


    def pubOdo(self):
        now = datetime.now()
        if self.then is None:
            self.then = now
            return

        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.th/2)
        quaternion.w = cos(self.th/2)

        self.odomBroadcaster.sendTransform(
            (self.x, self.y, 0),
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            rospy.Time.now(),
            "base_link",
            "odom"
            )

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion

        odom.twist.twist.linear.x = self.v
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.w

        self.odomPub.publish(odom)



if __name__=='__main__':
    rospy.init_node('encoders_node')
    enc = PhidgetEncoder(1.3/4000, 1.0)
    rospy.loginfo('Spinning...')

    while not rospy.is_shutdown():
        rospy.sleep(0.2)

    enc.close()
    rospy.loginfo("Done.")
    exit(0)
