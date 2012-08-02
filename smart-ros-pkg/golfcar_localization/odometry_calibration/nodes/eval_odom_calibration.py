#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('odometry_calibration')
import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion
import numpy as np
import copy


class OdomEvaluationNode:

    def __init__(self):
        self.pub = rospy.Publisher('odom_raw', Odometry)
        self.br = tf.TransformBroadcaster()
        self.transform_t = self.transform_r = None
        self.listener = tf.TransformListener()
        self.odo_sub = rospy.Subscriber('odom', Odometry, self.odo_callback)
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)

        self.error = 0
        self.error_n = 0

    def timer_callback(self, dummy):
        # measure the difference between the two frames
        p = PoseStamped()
        p.header.frame_id = '/odom_raw'
        try:
            p = self.listener.transformPose('/odom',p)
        except tf.Exception as e:
            rospy.logwarn("Could not transform from odom to odom_raw: " + str(e))
            return
        dx = p.pose.position.x
        dy = p.pose.position.y
        self.error += np.sqrt(dx*dx + dy*dy)
        self.error_n += 1
        rospy.loginfo('Odometry error: %f' % (self.error/self.error_n))

    def odo_callback(self, msg):

        # Initialize the transform from map to odom_raw, from the first available
        # transform between map and odom
        if self.transform_r is None or self.transform_t is None:
            try:
                (self.transform_t, self.transform_r) = \
                    self.listener.lookupTransform('/map', '/odom', rospy.Time(0))
            except tf.Exception as e:
                rospy.loginfo("Could not transform from map to odom: " + str(e))
                return
            angles = euler_from_quaternion(self.transform_r)
            rospy.loginfo('Initializing transform to x=%f, y=%f, yaw=%fdeg' %
                (self.transform_t[0], self.transform_t[1], np.degrees(angles[2])))

        # publish the static transform
        self.br.sendTransform(self.transform_t, self.transform_r,
                                            msg.header.stamp, '/odom_raw', '/map')

        # republish the odom message in frame odom_raw
        msg.header.frame_id = '/odom_raw'
        msg.pose.pose.position.z = 0
        self.pub.publish(msg)


rospy.init_node('pub_odom_raw')
node = OdomEvaluationNode()
rospy.spin()