#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('odometry_calibration')
import rospy

from phidget_encoders.msg import EncoderCounts
from geometry_msgs.msg import PoseWithCovarianceStamped, Point32
from sensor_msgs.msg import PointCloud

from odometry_calibration import *



class OdometryCalibrationNode:
    '''From encoder counts and AMCL trajectory, calibrate the odometry.

    To derive odometry from encoder we need 2 parameters: the size of the wheels and
    the distance between the wheels. Additionaly, a correction factor is introduced
    to take care of the fact that the 2 wheels do not have exactly the same size.
    Those 3 parameters can be tuned if we know the true trajectory of the vehicle.
    This can be obtained from AMCL when the laser sees enough features to localize
    the car better than the odometry. This node extracts portions of the trajectory
    that are straight segments and those that are segments of constant curvatures,
    and uses them to calibrate the 3 parameters above.
    '''

    def __init__(self):
        self.calibration_tool = CalibrationTool()

        self.encoders_sub = rospy.Subscriber('encoder_counts', EncoderCounts, self.encoders_callback)
        self.amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)

        # publish the extracted segments as a point cloud (for debugging)
        self.segments_pub = rospy.Publisher('calibration_segments', PointCloud)

        # every 5 seconds, analyze the trajectory and attempt to calibrate
        self.timer = rospy.Timer(rospy.Duration(5), self.timer_callback)

        # do it straight away for debugging purpose
        self.timer_callback(0)

    def encoders_callback(self, msg):
        self.calibration_tool.append(msg)

    def amcl_callback(self, msg):
        self.calibration_tool.segment_classifier.append( PoseFromAMCLMsg(msg) )

    def timer_callback(self, dummy):
        self.calibration_tool.calibrate()

        # publish the segments (for visualization purpose)
        msg = PointCloud()
        msg.header.frame_id = 'map'
        d = {1: self.calibration_tool.straight_segments,
             -1: self.calibration_tool.curved_segments}
        for (z, segments) in d.iteritems():
            for i in range(len(segments)):
                for p in segments[i].poses:
                    msg.points.append( Point32(p.x, p.y, z*(i+1)) )
        self.segments_pub.publish(msg)


if __name__=='__main__':
    rospy.init_node('odo_cal_node')
    node = OdometryCalibrationNode()
    rospy.spin()