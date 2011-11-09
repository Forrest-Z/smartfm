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

from lowlevel.msg import Encoders as EncodersMsg


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
        self.distBtwWheels = rospy.get_param('dist_btw_wheels',0.995)
        wheelSize = rospy.get_param('wheel_size', 1.335)
        leftCorrectionFactor = rospy.get_param('left_correction_factor', 1.011)

        # Configuration of the output topic / tf
        self.frameID = rospy.get_param('frame_id', 'odom') #TF frame ID
        self.odoTopicID  = rospy.get_param('odo_topic_name', 'odom') #Output topic frame ID

        self.minPubPeriod = rospy.get_param('min_pub_period', 0.2)

        self.left = 0
        self.right = 1
        self.countBufs = {self.left: CountBuffer(wheelSize*leftCorrectionFactor), self.right: CountBuffer(-wheelSize)}
        self.lastPub = None

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.v = 0.0
        self.w = 0.0
        self.dt = 0.0
        self.d_th = 0.0
        self.d_dist = 0.0

        self.initPhidget()
        self.encoder.setEnabled(self.left, True)
        self.encoder.setEnabled(self.right, True)

        self.odomPub = rospy.Publisher(self.odoTopicID, Odometry)
        self.odomBroadcaster = TransformBroadcaster()
        self.simpleOdoPub = rospy.Publisher('odom_linear', SimpleOdo)
        self.encodersPub = rospy.Publisher('encoders', EncodersMsg)


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

        #rospy.logdebug("Encoder %i: Encoder %i -- Change: %i -- Time: %i -- Position: %i" % (e.device.getSerialNum(), e.index, e.positionChange, e.time, self.encoder.getPosition(e.index)))

        if e.index in self.countBufs.keys():
            self.dt = self.countBufs[e.index].add(e)
            if self.dt > .05:
                self.process()

    def process(self):
        self.now = rospy.Time.now()
        self.pubEncodersMsg()
        self.integratePos()
        self.pubOdo()
        self.lastPub = self.now

    def pubEncodersMsg(self):
        dl = self.countBufs[self.left].dp
        dr = self.countBufs[self.right].dp
        self.d_dist = (dl+dr)/2
        self.d_th = (dr-dl)/self.distBtwWheels

        EncodersMsg encodersMsg
        encodersMsg.stamp = self.now
        encodersMsg.dt = self.dt
        encodersMsg.d_dist = self.d_dist
        encodersMsg.d_th = self.d_th
        if self.dt>0:
            encodersMsg.v = self.d_dist/self.dt
            encodersMsg.w = self.d_th/self.dt
        else:
            encodersMsg.v = 0
            encodersMsg.w = 0

        self.encodersPub.publish(encodersMsg)

        self.countBufs[self.left].reset()
        self.countBufs[self.right].reset()

    def integratePos(self):
        d_x = cos(self.d_th) * self.d_dist
        d_y = -sin(self.d_th) * self.d_dist
        self.x += cos(self.th)*d_x - sin(self.th)*d_y
        self.y += sin(self.th)*d_x + cos(self.th)*d_y
        self.th += self.d_th
        self.v = self.d_dist/self.dt
        self.w = self.d_th/self.dt

        rospy.logdebug('pose (x,y,th_deg)=(%.2f, %.2f, %+d), (v,w)=(%.2f, %.2f)' % (self.x, self.y, int(self.th*180.0/pi), self.v, self.w))

    def pubOdo(self):
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.th/2)
        quaternion.w = cos(self.th/2)

        self.odomBroadcaster.sendTransform(
            (self.x, self.y, 0),
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            now, "base_link", self.frameID
        )

        odom = Odometry()
        odom.header.stamp = self.now
        odom.header.frame_id = self.frameID
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0(rospy.Time.now()-self.lastPub).to_sec() > self.minPubPeriod
        odom.pose.pose.orientation = quaternion

        odom.twist.twist.linear.x = self.v
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.w

        self.odomPub.publish(odom)


    def loop(self):
        dt = (rospy.Time.now()-self.lastPub).to_sec()
        if self.lastPub is None or dt > self.minPubPeriod:
            if self.lastPub is None:
                self.dt = 0
            else:
                self.dt = dt
            self.process()
        rospy.sleep(self.minPubPeriod)



if __name__=='__main__':
    rospy.init_node('encoders_node')
    enc = PhidgetEncoder()
    rospy.loginfo('Spinning...')

    while not rospy.is_shutdown():
        enc.loop()

    enc.closePhidget()
    rospy.loginfo("Done.")
    exit(0)
