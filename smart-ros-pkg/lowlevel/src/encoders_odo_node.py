#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
ROS node in charge of interfacing with the encoders phidget, and publishing
odometry messages on the "encoders" topic.

We are reading from 2 encoders (left and right) so we can get the 2D pose of
the vehicle (x,y,theta) and velocities (v,w). This will be published as a
nav_msgs/Odometry and tf broadcast (as recommended by the navigation stacks
tutorials).

Since this information is not very accurate, we are also publishing the distance
traveled and the linear velocity (averaged over the 2 encoders). This is
published as a custom lowlevel/Odometry msg on the odom_linear channel.
'''

import roslib; roslib.load_manifest('lowlevel')
import rospy
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

from Phidgets.PhidgetException import *
from Phidgets.Events.Events import *
from Phidgets.Devices.Encoder import Encoder

import lowlevel.msg.Odometry as SimpleOdo


# Configuration of the output topic / tf
frameID = 'odom'      #TF frame ID
odoTopicID  = 'odom'  #Output topic frame ID


def err(e):
    '''A helper function to report Phidget errors'''
    rospy.logerr("Phidget error %i: %s. Exiting..." % (e.code, e.details))
    exit(1)


class CountBuffer:
    def __init__(self, wheelSize, counts=6000):
        self.wheelSize = wheelSize
        self.counts = counts
        self.reset()

    def reset(self):
        self.dp = 0.0 #dist (m)
        self.dt = 0.0 #time (sec)

    def add(self, e):
        self.dp += e.positionChange * self.wheelSize / self.counts
        self.dt += e.time * 1e-6 #usec to sec
        return self.dt



class PhidgetEncoder:
    '''Monitor the pose of each encoders'''

    def __init__(self):
        self.distBtwWheels = 1.0
        wheelSize = 1.38

        self.left = 0
        self.right = 1
        self.countBufs = {self.left: CountBuffer(wheelSize*1.011), self.right: CountBuffer(-wheelSize)}

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.v = 0.0
        self.w = 0.0
        self.totalDist = 0.0

        self.initPhidget()
        self.encoder.setEnabled(self.left, True)
        self.encoder.setEnabled(self.right, True)

        self.odomPub = rospy.Publisher(odoTopicID, Odometry)
        self.odomBroadcaster = TransformBroadcaster()
        self.simpleOdoPub = rospy.Publisher('odom_linear', SimpleOdo)


    def initPhidget(self):
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


    def closePhidget(self):
        rospy.loginfo("Closing...")
        try:
            self.encoder.closePhidget()
        except PhidgetException as e:
            err(e)


    def encoderPositionChange(self, e):
        '''A callback function called whenever the position changed'''

        #rospy.loginfo("Encoder %i: Encoder %i -- Change: %i -- Time: %i -- Position: %i" % (e.device.getSerialNum(), e.index, e.positionChange, e.time, self.encoder.getPosition(e.index)))

        if e.index in self.countBufs.keys():
            dt = self.countBufs[e.index].add(e)
            if dt > .05:
                dl = self.countBufs[self.left].dp
                dr = self.countBufs[self.right].dp
                d_dist = (dl+dr)/2
                d_th = (dr-dl)/self.distBtwWheels

                d_x = cos(d_th) * d_dist
                d_y = -sin(d_th) * d_dist
                self.totalDist += d_dist
                self.x += cos(self.th)*d_x - sin(self.th)*d_y
                self.y += sin(self.th)*d_x + cos(self.th)*d_y
                self.th += d_th
                self.v = d_dist/dt
                self.w = d_th/dt

                rospy.loginfo('dt=%f, counts: l=%i, -r=%i' % (dt, self.encoder.getPosition(self.left), -self.encoder.getPosition(self.right)))
                rospy.loginfo('pose (x,y,th_deg)=(%.2f, %.2f, %+d), (v,w)=(%.2f, %.2f)' % (self.x, self.y, int(self.th*180.0/pi), self.v, self.w))

                simpleOdoMsg = SimpleOdo()
                simpleOdoMsg.pose = self.totalDist
                simpleOdoMsg.vel = self.v
                self.simpleOdoPub.publish(simpleOdoMsg)

                self.pubOdo()

                self.countBufs[self.left].reset()
                self.countBufs[self.right].reset()


    def pubOdo(self):
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
            frameID
            )

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = frameID
        odom.child_frame_id = 'base_link'

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
    enc = PhidgetEncoder()
    rospy.loginfo('Spinning...')

    while not rospy.is_shutdown():
        rospy.sleep(0.2)

    enc.closePhidget()
    rospy.loginfo("Done.")
    exit(0)
