#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('phidget_encoders')
import rospy

from phidget_encoders.msg import Encoders
from geometry_msgs.msg import PoseWithCovarianceStamped

import math



class OdometryCalibrationNode:

    def __init__(self):
        self.encoders_dist = 0
        self.encoders_sub = rospy.Subscriber('encoders', Encoders, self.encoders_callback)

        self.amcl_dist = 0
        self.amcl_last_point = None
        self.amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)

    def encoders_callback(self, msg):
        self.encoders_dist += msg.d_dist

    def amcl_callback(self, msg):
        if self.amcl_last_point is None:
            self.amcl_last_point = msg.pose.pose.position
        else:
            p = msg.pose.pose.position
            dx = p.x - self.amcl_last_point.x
            dy = p.y - self.amcl_last_point.y
            self.amcl_dist += math.sqrt( dx*dx + dy*dy )
            self.amcl_last_point = p
            print 'e=%f, a=%f' % (self.encoders_dist, self.amcl_dist)



if __name__=='__main__':
    rospy.init_node('odo_cal_node')
    node = OdometryCalibrationNode()
    rospy.spin()