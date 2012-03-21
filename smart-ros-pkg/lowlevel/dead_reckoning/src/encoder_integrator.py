#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('dead_reckoning')
import rospy

from math import sin, cos, pi

from phidget_encoders.msg import Encoders
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
import tf.transformations

'''
Subscribes to the 'encoders' topic (lowlevel/Encoders messages), integrates the
position from the incoming messages, and publishes the corresponding odometry
info: nav_msgs/Odometry on 'odom' and TF broadcast from frame 'odom' to 'base_link'.

Parameters:
- frame_id: the frame ID of the TF transform broadcasted (defaults to 'odom')
- x: the initial value of x (defaults to 0)
- y: the initial value of y (defaults to 0)
- th: the initial value of th (defaults to 0)
'''


class EncoderIntegrator:

    def __init__(self):
        self.frameID = rospy.get_param('~frame_id', 'odom') #TF frame ID

        self.x = rospy.get_param('~x', 0.0)
        self.y = rospy.get_param('~y', 0.0)
        self.th = rospy.get_param('~th', 0.0)

        self.odomPub = rospy.Publisher('odom', Odometry)
        self.odomBroadcaster = TransformBroadcaster()
        self.encsub = rospy.Subscriber('encoders', Encoders, self.callback)


    def callback(self, encmsg):
        d_x = cos(encmsg.d_th) * encmsg.d_dist
        d_y = -sin(encmsg.d_th) * encmsg.d_dist
        self.x += cos(self.th)*d_x - sin(self.th)*d_y
        self.y += sin(self.th)*d_x + cos(self.th)*d_y
        self.th += encmsg.d_th

        rospy.logdebug('pose (x,y,th_deg)=(%.2f, %.2f, %+d), (v,w)=(%.2f, %.2f)' % (self.x, self.y, int(self.th*180.0/pi), encmsg.v, encmsg.w))

        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.th)

        self.odomBroadcaster.sendTransform(
            (self.x, self.y, 0),
            (quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
            encmsg.stamp, "base_link", self.frameID
        )

        odom = Odometry()
        odom.header.stamp = encmsg.stamp
        odom.header.frame_id = self.frameID
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        odom.twist.twist.linear.x = encmsg.v
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = encmsg.w

        # What about the covariance ?

        self.odomPub.publish(odom)




if __name__=='__main__':
    rospy.init_node('encoderIntegrator')
    enc = EncoderIntegrator()
    rospy.spin()
